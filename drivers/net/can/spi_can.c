/*
 *  spi_can.c - SPI interface driver using emulation on McBSP interface
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated
 *  Author: Aurelien Jacquiot <a-jacquiot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>

#include <linux/spi/spi.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/platform/spi_can.h>

#include <asm/setup.h>
#include <asm/machdep.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <asm/gpio.h>

#include <mach/board.h>

#ifdef DEBUG
#define DPRINTK(fmt, args...) printk(KERN_DEBUG fmt, ## args)
#else
#define DPRINTK(fmt, args...)                        
#endif

#define DRV_NAME                     "spi_can"
#define SPI_CAN_MODULE_VERSION       "0.1"
#define DRV_DESC                     "CAN over SPI driver " SPI_CAN_MODULE_VERSION
#define TX_ECHO_SKB_MAX              1

#define SANITY_SPI_DELAY             100
#define TIMEOUT_SPI_LOOP             500

#define SPI_CAN_SPI_BITRATE          4000000 /* SPI bitrate: 4MHz max (50MHz / 12) */

/*
 * CAN timing parameters
 * 
 * bitrate = CAN_CLK / ((sync + prop + phase1 + phase2 + 1) * QP)
 * 
 * Here we have: sync + prop + phase1 seg = 4
 *               phase2 seg = 3
 *               SJW (synchronization jump width) = 2
 *               BRP or QP (quantum prescaler) = 4 for a 8MHz clock (TQ of 500 nanosec)
 *
 *               bitrate = 8MHz / ((4 + 3 + 1) * 4) = 250 Kbit/sec
 *
 */
#define SPI_CAN_CLK_RATE             8000000 /* CAN controller clock rate: 8MHz */
#define SPI_CAN_DEFAULT_BITRATE      500000  /* CAN default bitrate: 250Kbps */

static struct can_bittiming spi_can_default_bt = {
	.bitrate    = SPI_CAN_DEFAULT_BITRATE, 
	.tq         = 250,
	.prop_seg   = 2,
	.phase_seg1 = 2,
	.phase_seg2 = 3,
	.sjw        = 2,
	.brp        = 2,
};

static struct can_bittiming_const spi_can_bittiming_const = {
	.name      = DRV_NAME,
	.tseg1_min = 1,
	.tseg1_max = 16,
	.tseg2_min = 1,
	.tseg2_max = 8,
	.sjw_max   = 4,
	.brp_min   = 1,
	.brp_max   = 1024,
	.brp_inc   = 1,
};

static int spi_can_open(struct net_device *ndev);
static int spi_can_xmit(struct sk_buff *skb, struct net_device *ndev);
static int spi_can_stop(struct net_device *ndev);
static int spi_can_set_mode(struct net_device *dev, enum can_mode mode);
static int spi_can_set_bittiming(struct net_device *ndev);
static void spi_can_rx(struct spi_can_priv *can);
static inline int spi_can_init_queue(struct spi_can_priv *can);
static inline int spi_can_start_queue(struct spi_can_priv *can);
static inline int spi_can_stop_queue(struct spi_can_priv *can);
static inline int spi_can_destroy_queue(struct spi_can_priv *can);

/*
 * Low-level I/O access over SPI
 */
static inline int
spi_rw(struct spi_device *spi, u8 *tx_buf, u8 *rx_buf, size_t len)
{
	struct spi_transfer t = {
		.tx_buf		= tx_buf,
		.rx_buf		= rx_buf,
		.len		= len,
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

static int spi_can_mem_write(struct spi_can_priv *can, u32 addr, size_t len, u32 *buffer)
{
	int res;

	if ((len) && (len < 4))
		len = 4;

	for(;len; len -= 4, buffer++) {
		int timeout = TIMEOUT_SPI_LOOP;

		mutex_lock(&can->lock);

		/* Send Address and command */
		can->tx_buf->cmd       = SPI_CAN_CMD_MEM_WRITE;
		can->tx_buf->u.mf.addr = addr;
		can->tx_buf->u.mf.data = *buffer;

		res = spi_write(can->spi, (u8*) can->tx_buf,
				SPI_CAN_FRAME_HEADER_LEN + sizeof(struct spi_can_mem_frame));
		if (res) {
			dev_err(CAN_TO_DEV(can), "SPI write error in %s (res = %d)\n",
				__FUNCTION__, res);
			mutex_unlock(&can->lock);
			return -EIO;
		}

		*can->tx_ack = (u16) 0;

	      	while(*can->tx_ack != (u16) SPI_CAN_CMD_READY) {

			res = spi_rw(can->spi,
				     (u8*) can->tx_dummy,
				     (u8*) can->tx_ack,
				     sizeof(u16));
			
			if ((res) || (timeout-- < 0)) {
				dev_err(CAN_TO_DEV(can), "SPI ACK error (res = %d, timeout = %d) in %s\n",
					res, timeout, __FUNCTION__);
				mutex_unlock(&can->lock);
				return -EIO;
			}

			if (*can->tx_ack == (u16) SPI_CAN_CMD_READY)
				break;

			udelay(SANITY_SPI_DELAY);
		}

		mutex_unlock(&can->lock);

		if (*can->tx_ack != SPI_CAN_CMD_READY) {
			dev_err(CAN_TO_DEV(can), "bad SPI ACK (0x%x) in %s\n", *can->tx_ack, __FUNCTION__);
			return -EIO;
		}
	}

	return res;
}

static int spi_can_mem_read(struct spi_can_priv *can, u32 addr, size_t len, void *buffer)
{
	int res;

	for(;len; len -= 4, buffer++) {
		mutex_lock(&can->lock);

		/* Send Address and command */
		can->rx_buf->cmd       = SPI_CAN_CMD_MEM_READ;
		can->rx_buf->u.mf.addr = addr;
		can->rx_buf->u.mf.data = 0;

		res = spi_write(can->spi, (u8*) can->rx_buf,
 				SPI_CAN_FRAME_HEADER_LEN + sizeof(struct spi_can_mem_frame));
		if (res) {
			dev_err(CAN_TO_DEV(can), "SPI write error in %s (res = %d)\n",
				__FUNCTION__, res);
			mutex_unlock(&can->lock);
			return -EIO;
		}

		res = spi_rw(can->spi,
			     (u8*) can->rx_dummy,
			     (u8*) can->rx_buf,
			     SPI_CAN_FRAME_HEADER_LEN + sizeof(struct spi_can_mem_frame));

		mutex_unlock(&can->lock);

		if (res) {
			dev_err(CAN_TO_DEV(can), "tx/rx error (res = %d) in %s\n",
				res, __FUNCTION__);
			return -EIO;
		}
		
		DPRINTK("%s: rx =  0x%x 0x%x 0x%x 0x%x 0x%x\n", __FUNCTION__,
			can->rx_buf[0], can->rx_buf[1], can->rx_buf[2], can->rx_buf[3], can->rx_buf[4]);
		
		*((u32 *) buffer) = can->rx_buf->u.mf.data;
	}
	return res;
}

static const struct net_device_ops spi_can_netdev_ops = {
	.ndo_open       = spi_can_open,
	.ndo_stop       = spi_can_stop,
	.ndo_start_xmit = spi_can_xmit,
};

static int spi_can_probe(struct spi_device *spi)
{
	int                   err;
	int                   i;
	struct net_device    *ndev;
	struct spi_can_priv  *priv;
	struct device        *dev = &spi->dev;
 
	DPRINTK("%s: SPI %d probe\n", __FUNCTION__, spi->chip_select);
	
	ndev = alloc_candev(sizeof(*priv), TX_ECHO_SKB_MAX);
	if (!ndev) {
		err = -ENOMEM;
		goto exit;
	}

	priv      = netdev_priv(ndev);
	priv->spi = spi;
	
        /* Initialize CAN properties */
	priv->can.bittiming_const    = &spi_can_bittiming_const;
	priv->can.do_set_bittiming   = spi_can_set_bittiming;
	priv->can.do_set_mode        = spi_can_set_mode;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES;
	priv->can.bittiming.bitrate  = SPI_CAN_DEFAULT_BITRATE;
	priv->can.clock.freq         = SPI_CAN_CLK_RATE;

	dev_set_drvdata(dev, ndev);
	SET_NETDEV_DEV(ndev, dev);

	ndev->netdev_ops = &spi_can_netdev_ops;

  	ndev->flags     |= IFF_ECHO;	/* we support local echo */
	priv->ndev       = ndev;

	/* Register CAN device */
	err = register_candev(ndev);
	if (err) {
		dev_err(dev, "registering failed (err=%d)\n", err);
		goto exit_candev_free;
	}

	dev_info(dev, "%s device registered\n", DRV_NAME);

	/* Initiate and start queue */
	err = spi_can_init_queue(priv);
	if (err != 0) {
		dev_err(dev, "problem initializing queue\n");
		goto exit_candev_free;
	}

	err = spi_can_start_queue(priv);
	if (err != 0) {
		dev_err(dev, "problem starting queue\n");
		goto exit_candev_free;
	}
	
	/* Allocate SPI tx/rx buffers */
	priv->tx_buf   = (struct spi_can_frame*) kzalloc(max((int) sizeof(struct spi_can_frame), L1_CACHE_BYTES), GFP_KERNEL);
	priv->rx_buf   = (struct spi_can_frame*) kzalloc(max((int) sizeof(struct spi_can_frame), L1_CACHE_BYTES), GFP_KERNEL);
	priv->tx_dummy = (struct spi_can_frame*) kzalloc(max((int) sizeof(struct spi_can_frame), L1_CACHE_BYTES), GFP_KERNEL);
	priv->rx_dummy = (struct spi_can_frame*) kzalloc(max((int)sizeof(struct spi_can_frame), L1_CACHE_BYTES), GFP_KERNEL);
	priv->tx_ack   = (u16*) kzalloc(max((int) sizeof(u16), L1_CACHE_BYTES), GFP_KERNEL);
	priv->rx_ack   = (u16*) kzalloc(max((int) sizeof(u16), L1_CACHE_BYTES), GFP_KERNEL);

	for (i = 0; i < 5; i++) {
		priv->tx_dummy->cmd = SPI_CAN_CMD_DUMMY;
		priv->rx_dummy->cmd = SPI_CAN_CMD_DUMMY;
	}
		
	return 0;

exit_candev_free:
	free_candev(ndev);
exit:
	return err;
}

static int spi_can_remove(struct spi_device *spi)
{
	struct device         *dev  = &spi->dev;
	struct net_device     *ndev = dev_get_drvdata(dev);
	struct spi_can_priv   *priv = netdev_priv(ndev);
	int                    res;

	DPRINTK("%s: SPI %d remove\n", __FUNCTION__, spi->chip_select);

	/* Remove the queue */
	res = spi_can_destroy_queue(priv);
	if (res != 0)
		return res;

	/* Release SPI tx/rx buffers */
	kfree(priv->tx_buf);
	kfree(priv->rx_buf);
	kfree(priv->tx_ack);
	kfree(priv->rx_ack);
	kfree(priv->tx_dummy);
	kfree(priv->rx_dummy);

	/* Unregister CAN device */
	unregister_candev(ndev);

	dev_set_drvdata(dev, NULL);

	free_candev(ndev);

	return 0;
}

static struct platform_device *spi_can_platform_device;

static struct spi_driver spi_can = {
	.driver = {
		/* Name must match modalias in the spi_board_info definition */
		.name  =        "spi3",
		.owner =	THIS_MODULE,
		.bus   =        &spi_bus_type,
	},
	.probe  =       spi_can_probe,
	.remove =	__devexit_p(spi_can_remove),
};

static irqreturn_t spi_can_interrupt_handler(int irq, void *data)
{
	struct spi_can_priv *can  = (struct spi_can_priv *) data;

	if (can->ndev->flags & IFF_UP)
		spi_can_rx(can);

	return IRQ_HANDLED;
}

static void spi_can_interrupt_setup(struct spi_can_priv *can)
{
	int status;

	/* Request an GPIO interrupt */
	status = gpio_request(SPI_CAN_GPIO, "SPI-CAN IRQ");
	if (status < 0)
		printk(KERN_ERR "Cannot get GPIO for SPI CAN: %d\n", status);
	else {
		gpio_direction_input(SPI_CAN_GPIO);
		can->irq = gpio_to_irq(SPI_CAN_GPIO);
		if (can->irq < 0)
			printk(KERN_ERR "GPIO for SPI CAN has no IRQ: %d\n", can->irq);
		else
			set_irq_type(can->irq, IRQ_TYPE_EDGE_RISING);
	}

	/* Set handler for CAN interrupt */
	request_irq(can->irq, 
		    spi_can_interrupt_handler,
		    IRQF_DISABLED,
		    "SPI-CAN",
		    (void *) can);
}

static void spi_can_interrupt_clear(struct spi_can_priv *can)
{
	gpio_free(SPI_CAN_GPIO);
	free_irq(can->irq, (void *) can);
}

static int spi_can_setup(struct net_device *ndev, struct spi_can_priv *priv)
{
	struct can_bittiming bt;
	int                  err;

	/* Do not allow changing bittiming while running */
	if (ndev->flags & IFF_UP)
		return -EBUSY;

	memcpy(&bt, &spi_can_default_bt, sizeof(bt));
	err = can_get_bittiming(ndev, &bt);
	if (err)
		return err;
	memcpy(&priv->can.bittiming, &bt, sizeof(bt));

	if (priv->can.do_set_bittiming) {
		/* Finally, set the bit-timing registers */
		err = priv->can.do_set_bittiming(ndev);
		if (err)
			return err;
	}

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}

static int spi_can_send_frame(struct spi_can_priv *can, struct can_frame *cf)
{
	int                      res;
	int                      timeout = TIMEOUT_SPI_LOOP;

	/* Send CAN transmit command (2 half words + can frame) */
	can->tx_buf->cmd  = SPI_CAN_CMD_CAN_TX;
	can->tx_buf->size = sizeof(struct spi_can_c_frame) - (8 - cf->can_dlc);

	/* Copy the CAN packet */
	can->tx_buf->u.cf.can_id  = cf->can_id;
	can->tx_buf->u.cf.can_dlc = cf->can_dlc;
	memcpy(&can->tx_buf->u.cf.data, cf->data, cf->can_dlc);

	res = spi_write(can->spi, (u8*) can->tx_buf, 
			SPI_CAN_FRAME_HEADER_LEN + sizeof(struct spi_can_c_frame));
	if (res) {
		dev_err(CAN_TO_DEV(can), "SPI write error in %s (res = %d)\n",
			__FUNCTION__, res);
		return -EIO;
	}

	*can->tx_ack = (u16) 0;
	while(*can->tx_ack != (u16) SPI_CAN_CMD_READY) {
		res = spi_rw(can->spi,
			     (u8*) can->tx_dummy,
			     (u8*) can->tx_ack,
			     sizeof(u16));

		if ((res) || (timeout-- < 0)) {
			dev_err(CAN_TO_DEV(can), "SPI ACK error (res = %d, timeout = %d) in %s\n",
				res, timeout, __FUNCTION__);
			return -EIO;
		}

		if (*can->tx_ack == (u16) SPI_CAN_CMD_READY)
			break;

		udelay(SANITY_SPI_DELAY);
	}

	DPRINTK("%s: ACK tx = 0x%x, timeout = %d\n", __FUNCTION__, *can->tx_ack, timeout);

	if (*can->tx_ack != SPI_CAN_CMD_READY) {
		dev_err(CAN_TO_DEV(can), "bad SPI ACK (0x%x) in %s\n", *can->tx_ack, __FUNCTION__);
		return -EIO;
	}

	DPRINTK("%s: sending to CAN ID 0x%x, len = %d\n",
		__FUNCTION__, cf->can_id & CAN_EFF_MASK, cf->can_dlc);

	return 0;
}

/* 
 * Retrieve and process next CAN message
 * Messages can be a CAN frame, a CAN interrupt ACK or a CAN error
 */
static inline int spi_can_get_message(struct spi_can_priv *can)
{
	int                      res;
	struct sk_buff          *skb;
	struct can_frame        *cf;
	struct net_device_stats *stats = &can->stats;
	int                      timeout = TIMEOUT_SPI_LOOP;

retry:
	/* Send CAN receive command (2 half words) */
	can->rx_buf->cmd  = SPI_CAN_CMD_CAN_RX;
	can->rx_buf->size = 0;

	res = spi_write(can->spi, (u8*) can->rx_buf, SPI_CAN_FRAME_HEADER_LEN);
	if (res) {
		dev_err(CAN_TO_DEV(can), "SPI write error in %s (res = %d)\n",
			__FUNCTION__, res);
		
		return -EIO;
	}

	res = spi_rw(can->spi,
		     (u8*) can->rx_dummy,
		     (u8*) can->rx_buf,
		     SPI_CAN_FRAME_HEADER_LEN + sizeof(struct spi_can_c_frame));

	if (res) {
		dev_err(CAN_TO_DEV(can), "tx/rx error (res = %d) in %s\n",
			res, __FUNCTION__);
		return -EIO;
	}

	switch (can->rx_buf->cmd) {

	case SPI_CAN_CMD_NOP:
		/* No more message */
		return 0;

	case SPI_CAN_CMD_CAN_RX:
		/* CAN incoming frame */
		skb = alloc_can_skb(can->ndev, &cf);
		if (skb == NULL)
			return -ENOMEM;

		/* Copy SPI buffer to CAN buffer */
		cf->can_id  = can->rx_buf->u.cf.can_id;
		cf->can_dlc = can->rx_buf->u.cf.can_dlc;
		memcpy(&cf->data, &can->rx_buf->u.cf.data, cf->can_dlc);

		netif_rx(skb);
		
		stats->rx_packets++;
		stats->rx_bytes += cf->can_dlc;

		DPRINTK("%s: received frame from id=0x%x, dlc=%d, data=0x%x 0x%x 0x%x 0x%x\n",
			__FUNCTION__, cf->can_id, cf->can_dlc,
			cf->data[0], cf->data[1], cf->data[2], cf->data[3]);

		break;

	case SPI_CAN_CMD_CAN_ERROR: 
	{
		/* We issued an CAN controller error */
		u32            error;
		enum can_state new_state;

		skb = alloc_can_err_skb(can->ndev, &cf);
		if (skb == NULL)
			return -ENOMEM; 

		error = can->rx_buf->u.cf.can_id;

		dev_err(CAN_TO_DEV(can),"CAN controller error (error = 0x%x)\n", error);

		if (error & SPI_CAN_STATUS_BUS_OFF) {
			new_state    = CAN_STATE_BUS_OFF;
			cf->can_id  |= CAN_ERR_BUSOFF;
		} else if (error & SPI_CAN_STATUS_EPASS) {
			new_state    = CAN_STATE_ERROR_PASSIVE;
			cf->can_id  |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE | CAN_ERR_CRTL_RX_PASSIVE;
		} else if (error & SPI_CAN_STATUS_EWARN) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_TX_WARNING | CAN_ERR_CRTL_RX_WARNING; 
		} else if (error & SPI_CAN_STATUS_LEC_STUFF) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_PROT;
			cf->data[2] |= CAN_ERR_PROT_STUFF;
		} else if (error & SPI_CAN_STATUS_LEC_FORM) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_PROT;
			cf->data[2] |= CAN_ERR_PROT_FORM;
		} else if (error & SPI_CAN_STATUS_LEC_ACK) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_ACK;
		} else if (error & SPI_CAN_STATUS_LEC_BIT1) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_PROT;
			cf->data[2] |= CAN_ERR_PROT_BIT1;
		} else if (error & SPI_CAN_STATUS_LEC_BIT0) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_PROT;
			cf->data[2] |= CAN_ERR_PROT_BIT0;
		} else if (error & SPI_CAN_STATUS_LEC_CRC) {
			new_state    = CAN_STATE_ERROR_WARNING;
			cf->can_id  |= CAN_ERR_PROT;
			cf->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ | CAN_ERR_PROT_LOC_CRC_DEL;
		}  else {
			new_state = CAN_STATE_ERROR_ACTIVE;
		}

		/* Update CAN state statistics */
		switch (can->can.state) {
		case CAN_STATE_ERROR_ACTIVE:
			if (new_state >= CAN_STATE_ERROR_WARNING &&
			    new_state <= CAN_STATE_BUS_OFF)
				can->can.can_stats.error_warning++;
		case CAN_STATE_ERROR_WARNING:	/* fallthrough */
			if (new_state >= CAN_STATE_ERROR_PASSIVE &&
			    new_state <= CAN_STATE_BUS_OFF)
				can->can.can_stats.error_passive++;
			break;
		default:
			break;
		}

		can->can.state = new_state;

		if (new_state == CAN_STATE_BUS_OFF) {
			can_bus_off(can->ndev);
			break;
		}

		netif_rx(skb);

		return -1;
	}
	case SPI_CAN_CMD_CAN_TX_INT:
		/* CAN TX acknowledge (transmission complete) */

		can->stats.tx_packets++;
		can->stats.tx_bytes += can->tx_q.len - 1;
		if (can->tx_q.len) {
			can_get_echo_skb(can->ndev, 0);
			can->tx_q.len = 0;
		}
		netif_wake_queue(can->ndev);

		break;

	default:
		if (timeout-- < 0) {
			dev_warn(CAN_TO_DEV(can), "bad rx message (cmd = 0x%x)\n",
				 can->rx_buf->cmd);
			return -1;
		}
		udelay(SANITY_SPI_DELAY);
		goto retry;
	}

	return 1;
}

static void spi_can_receive_messages(struct work_struct *work)
{
	struct spi_can_priv *can = container_of(work, struct spi_can_priv, rx_q.work);
	int                  res;

	/* Retrieve and process all CAN messages */
	mutex_lock(&can->lock);
	for (res = 1; res == 1;)
		res = spi_can_get_message(can);
	mutex_unlock(&can->lock);
}

static void spi_can_transmit_messages(struct work_struct *work)
{
	struct spi_can_priv *can = container_of(work, struct spi_can_priv, tx_q.work);

	if (can->tx_q.run == QUEUE_STOPPED)
		return;

	mutex_lock(&can->lock);
	if (can->tx_q.skb) {
		struct can_frame *frame = (struct can_frame *)can->tx_q.skb->data;

		/* Transmit CAN frame to the physical layer */
		spi_can_send_frame(can, frame);
		can->tx_q.len = 1 + frame->can_dlc;
		can_put_echo_skb(can->tx_q.skb, can->ndev, 0);
		can->tx_q.skb = NULL;
	}
	mutex_unlock(&can->lock);
}

static inline int spi_can_init_queue(struct spi_can_priv *can)
{
	mutex_init(&can->lock);

	can->tx_q.run  = QUEUE_STOPPED;
	can->rx_q.run  = QUEUE_STOPPED;

	/* Init message transmit workqueue */
	INIT_WORK(&can->tx_q.work, spi_can_transmit_messages);
	can->tx_q.workqueue =
		create_singlethread_workqueue(dev_name(CAN_TO_DEV(can)->parent));
	if (can->tx_q.workqueue == NULL)
		return -EBUSY;

	/* Init message receive workqueue */
	INIT_WORK(&can->rx_q.work, spi_can_receive_messages);
	can->rx_q.workqueue =
	        create_singlethread_workqueue(dev_name(CAN_TO_DEV(can)->parent));
	if (can->rx_q.workqueue == NULL)
		return -EBUSY;

	return 0;
}

static inline int spi_can_start_queue(struct spi_can_priv *can)
{
	/* Transmit queue */
	mutex_lock(&can->lock);
	can->tx_q.run = QUEUE_RUNNING;
	can->rx_q.run = QUEUE_RUNNING;
	mutex_unlock(&can->lock);

	return 0;
}

static inline int spi_can_stop_queue(struct spi_can_priv *can)
{
	unsigned      limit = 500;
	int           status = 0;

	mutex_lock(&can->lock);

	/* Transmit queue */
	can->tx_q.run = QUEUE_STOPPED;
	while (can->tx_q.skb && limit--) {
		mutex_unlock(&can->lock);
		msleep(10);
		mutex_lock(&can->lock);
	}

	if (can->tx_q.skb )
		status = -EBUSY;

	/* Receive queue */
	can->rx_q.run = QUEUE_STOPPED;
	while (can->rx_q.skb && limit--) {
		mutex_unlock(&can->lock);
		msleep(10);
		mutex_lock(&can->lock);
	}

	if (can->rx_q.skb )
		status = -EBUSY;

	mutex_unlock(&can->lock);

	return status;
}

static inline int spi_can_destroy_queue(struct spi_can_priv *can)
{
	int status;

	status = spi_can_stop_queue(can);
	if (status != 0)
		return status;

	destroy_workqueue(can->tx_q.workqueue);
	destroy_workqueue(can->rx_q.workqueue);

	return 0;
}

static int spi_can_set_mode(struct net_device *ndev, enum can_mode mode)
{
	switch (mode) {
	case CAN_MODE_START:
		if (netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int spi_can_set_bittiming(struct net_device *ndev)
{
	struct spi_can_priv     *can = netdev_priv(ndev);
	struct can_bittiming    *bt  = &can->can.bittiming;
	struct spi_can_bt_frame *bf  = &can->tx_buf->u.bf;
	int                      res;
	int                      timeout = TIMEOUT_SPI_LOOP;

	mutex_lock(&can->lock);

	/* Send CAN bittiming set command (2 half words + bt frame) */
	can->tx_buf->cmd         = SPI_CAN_CMD_CAN_BTSET;
	can->tx_buf->size        = 0;

	bf->bitrate              = bt->bitrate;
	bf->sync_prop_phase1_seg = (u16) (bt->phase_seg1 + bt->prop_seg);
	bf->phase2_seg           = (u16) (bt->phase_seg2);
	bf->sjw                  = (u16) (bt->sjw);
	bf->quantum_prescaler    = (u16) (((SPI_CAN_CLK_RATE / 1000) * bt->tq) / 1000000);

	res = spi_write(can->spi, (u8*) can->tx_buf, 
			SPI_CAN_FRAME_HEADER_LEN + sizeof(struct spi_can_bt_frame));
	if (res) {
		dev_err(CAN_TO_DEV(can), "SPI write error in %s (res = %d)\n",
			__FUNCTION__, res);
		mutex_unlock(&can->lock);
		return -EIO;
	}

	*can->tx_ack = (u16) 0;

	while(*can->tx_ack != (u16) SPI_CAN_CMD_READY) {
		
		res = spi_rw(can->spi,
			     (u8*) can->tx_dummy,
			     (u8*) can->tx_ack,
			     sizeof(u16));
		
		if ((res) || (timeout-- < 0)) {
			dev_err(CAN_TO_DEV(can), "SPI ACK error (res = %d, timeout = %d) in %s\n",
				res, timeout, __FUNCTION__);
			mutex_unlock(&can->lock);
			return -EIO;
		}

		if (*can->tx_ack == (u16) SPI_CAN_CMD_READY)
			break;
		
		udelay(SANITY_SPI_DELAY);
	}

	mutex_unlock(&can->lock);

	if (*can->tx_ack != SPI_CAN_CMD_READY) {
		dev_err(CAN_TO_DEV(can), "bad SPI ACK (0x%x) in %s\n",
			*can->tx_ack, __FUNCTION__);
		return -EIO;
	}

	dev_info(CAN_TO_DEV(can), "setting bitrate to %d\n", bt->bitrate);
	
	return 0;
}

static void spi_can_rx(struct spi_can_priv *can)
{
	if (can->rx_q.run == QUEUE_RUNNING)
		queue_work(can->rx_q.workqueue, &can->rx_q.work);
}

static int spi_can_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct spi_can_priv    *can = netdev_priv(ndev);

	if (can->tx_q.skb) {
		dev_warn(CAN_TO_DEV(can), "%s called while tx busy\n", __FUNCTION__);
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(ndev);
	can->tx_q.skb = skb;
	ndev->trans_start = jiffies;
	queue_work(can->tx_q.workqueue, &can->tx_q.work);

	DPRINTK("%s: adding skb 0x%x to transfer\n", __FUNCTION__, skb);

	return NETDEV_TX_OK;
}

static int spi_can_open(struct net_device *ndev)
{
	struct spi_can_priv *can = netdev_priv(ndev);
	int err;

	/* Common CAN open */
	err = open_candev(ndev);
	if (err)
		return err;

	/* Initialize transmit queue */
	can->tx_q.skb = NULL;
	can->tx_q.len = 0;

	/* Configure bittiming */
	err = spi_can_setup(ndev, can);
	if (err)
		return err;

	spi_can_interrupt_setup(can);

	netif_start_queue(ndev);

	return 0;
}

static int spi_can_stop(struct net_device *ndev)
{
	struct spi_can_priv *can = netdev_priv(ndev);

	close_candev(ndev);

	spi_can_interrupt_clear(can);

	return 0;
}

/*
 * Linux driver
 */
static int __init spi_can_init(void)
{
	int res;

	res = spi_register_driver(&spi_can);
	if (res)
		return res;

	printk(KERN_INFO DRV_DESC "\n");

	return 0;
}
module_init(spi_can_init);

static void __exit spi_can_exit(void)
{
	printk(KERN_INFO DRV_DESC " unloaded\n");

	spi_unregister_driver(&spi_can);
}
module_exit(spi_can_exit);

MODULE_AUTHOR("Aurelien Jacquiot <a-jacquiot@ti.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CAN over SPI Driver");
MODULE_VERSION(SPI_CAN_MODULE_VERSION);
