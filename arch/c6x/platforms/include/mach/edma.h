/*
 *  TI EDMA definitions for TMS320C64xx
 *
 *  Copyright (C) 2006-2009, 2011 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
 * This EDMA3 programming framework exposes two basic kinds of resource:
 *
 *  Channel	Triggers transfers, usually from a hardware event but
 *		also manually or by "chaining" from DMA completions.
 *		Each channel is coupled to a Parameter RAM (PaRAM) slot.
 *
 *  Slot	Each PaRAM slot holds a DMA transfer descriptor (PaRAM
 *		"set"), source and destination addresses, a link to a
 *		next PaRAM slot (if any), options for the transfer, and
 *		instructions for updating those addresses.  There are
 *		more than twice as many slots as event channels.
 *
 * Each PaRAM set describes a sequence of transfers, either for one large
 * buffer or for several discontiguous smaller buffers.  An EDMA transfer
 * is driven only from a channel, which performs the transfers specified
 * in its PaRAM slot until there are no more transfers.  When that last
 * transfer completes, the "link" field may be used to reload the channel's
 * PaRAM slot with a new transfer descriptor.
 *
 * The EDMA Channel Controller (CC) maps requests from channels into physical
 * Transfer Controller (TC) requests when the channel triggers (by hardware
 * or software events, or by chaining).  The physical DMA channels provided
 * by the TCs are thus shared by many logical channels.
 *
 * DaVinci hardware also has a "QDMA" mechanism which is not currently
 * supported through this interface.  (DSP firmware uses it though.)
 */

#ifndef EDMA3_H_
#define EDMA3_H_

#if defined(CONFIG_SOC_TMS320C6455)
#include <mach/edma-c6455.h>
#elif defined(CONFIG_SOC_TMS320C6457)
#include <mach/edma-c6457.h>
#elif defined(CONFIG_SOC_TMS320C6472)
#include <mach/edma-c6472.h>
#elif defined(CONFIG_SOC_TMS320C6474)
#include <mach/edma-c6474.h>
#else
#error "No machine IRQ definitions"
#endif

/* PaRAM slots are laid out like this */
struct edmacc_param {
	unsigned int opt;
	unsigned int src;
	unsigned int a_b_cnt;
	unsigned int dst;
	unsigned int src_dst_bidx;
	unsigned int link_bcntrld;
	unsigned int src_dst_cidx;
	unsigned int ccnt;
};

/* fields in edmacc_param.opt */
#define SAM		BIT(0)
#define DAM		BIT(1)
#define SYNCDIM		BIT(2)
#define STATIC		BIT(3)
#define EDMA_FWID	(0x07 << 8)
#define TCCMODE		BIT(11)
#define EDMA_TCC(t)	((t) << 12)
#define TCINTEN		BIT(20)
#define ITCINTEN	BIT(21)
#define TCCHEN		BIT(22)
#define ITCCHEN		BIT(23)

#define TRWORD (0x7<<2)
#define PAENTRY (0x1ff<<5)

/*if changing the QDMA_TRWORD do appropriate change in start_edma */
#define QDMA_TRWORD (7 & 0x7)


/*Used by driver*/

/*
 * EDMA3 base register addresses
 */
#define EDMA_REGISTER_BASE	0x02a00000
#define EDMA_TC0_BASE		0x02a20000
#define EDMA_TC1_BASE		0x02a28000
#define EDMA_TC2_BASE		0x02a30000
#define EDMA_TC3_BASE		0x02a38000
#define EDMA_TC4_BASE		0x02a40000
#define EDMA_TC5_BASE		0x02a48000

#ifdef MACH_EDMA_REGION
#define EDMA_REGION               MACH_EDMA_REGION
#else
#define EDMA_REGION               0
#endif

#define EDMA_NUM_DMACH           64

#ifdef MACH_EDMA_NUM_QDMACH
#define EDMA_NUM_QDMACH           MACH_EDMA_NUM_QDMACH
#else
#define EDMA_NUM_QDMACH           8
#endif

#define EDMA_NUM_PARAMENTRY     512

#ifdef MACH_EDMA_NUM_EVQUE
#define EDMA_NUM_EVQUE            MACH_EDMA_NUM_EVQUE
#else
#define EDMA_NUM_EVQUE            6
#endif

#define EDMA_CHMAPEXIST           0

#ifdef MACH_EDMA_NUM_REGIONS
#define EDMA_NUM_REGIONS          MACH_EDMA_NUM_REGIONS
#else
#define EDMA_NUM_REGIONS          4
#endif

#define EDMA_MEMPROTECT           0

#ifdef MACH_EDMA_IRQ_CCINT
#define EDMA_IRQ_CCINT		MACH_EDMA_IRQ_CCINT
#endif

#ifdef MACH_EDMA_IRQ_CCERRINT
#define EDMA_IRQ_CCERRINT	MACH_EDMA_IRQ_CCERRINT
#endif

#define TCC_ANY                  -1

#define EDMA_PARAM_ANY           -2
#define EDMA_CHANNEL_ANY         -1
#define EDMA_QDMA0               64
#define EDMA_QDMA1               65
#define EDMA_QDMA2               66
#define EDMA_QDMA3               67
#define EDMA_QDMA4               68
#define EDMA_QDMA5               69
#define EDMA_QDMA6               71
#define EDMA_QDMA7               72

/*ch_status paramater of callback function possible values*/
#define DMA_COMPLETE 1
#define DMA_CC_ERROR 2
#define DMA_TC1_ERROR 3
#define DMA_TC2_ERROR 4

enum address_mode {
	INCR = 0,
	FIFO = 1
};

enum fifo_width {
	W8BIT = 0,
	W16BIT = 1,
	W32BIT = 2,
	W64BIT = 3,
	W128BIT = 4,
	W256BIT = 5
};

enum dma_event_q {
	EVENTQ_0 = 0,
	EVENTQ_1 = 1,
	EVENTQ_2 = 2,
	EVENTQ_3 = 3,
	EVENTQ_4 = 4,
	EVENTQ_5 = 5,
	EVENTQ_DEFAULT = -1
};

enum sync_dimension {
	ASYNC = 0,
	ABSYNC = 1
};

#define EDMA_CTLR_CHAN(ctlr, chan)	(((ctlr) << 16) | (chan))
#define EDMA_CTLR(i)			((i) >> 16)
#define EDMA_CHAN_SLOT(i)		((i) & 0xffff)

#define EDMA_CHANNEL_ANY		-1	/* for edma_alloc_channel() */
#define EDMA_SLOT_ANY			-1	/* for edma_alloc_slot() */
#define EDMA_CONT_PARAMS_ANY		 1001
#define EDMA_CONT_PARAMS_FIXED_EXACT	 1002
#define EDMA_CONT_PARAMS_FIXED_NOT_EXACT 1003

#define EDMA_MAX_CC               2

/* alloc/free DMA channels and their dedicated parameter RAM slots */
int edma_alloc_channel(int channel,
	void (*callback)(unsigned channel, u16 ch_status, void *data),
	void *data, enum dma_event_q);
void edma_free_channel(unsigned channel);

/* alloc/free parameter RAM slots */
int edma_alloc_slot(unsigned ctlr, int slot);
void edma_free_slot(unsigned slot);

/* alloc/free a set of contiguous parameter RAM slots */
int edma_alloc_cont_slots(unsigned ctlr, unsigned int id, int slot, int count);
int edma_free_cont_slots(unsigned slot, int count);

/* calls that operate on part of a parameter RAM slot */
void edma_set_src(unsigned slot, dma_addr_t src_port,
				enum address_mode mode, enum fifo_width);
void edma_set_dest(unsigned slot, dma_addr_t dest_port,
				 enum address_mode mode, enum fifo_width);
void edma_get_position(unsigned slot, dma_addr_t *src, dma_addr_t *dst);
void edma_set_src_index(unsigned slot, s16 src_bidx, s16 src_cidx);
void edma_set_dest_index(unsigned slot, s16 dest_bidx, s16 dest_cidx);
void edma_set_transfer_params(unsigned slot, u16 acnt, u16 bcnt, u16 ccnt,
		u16 bcnt_rld, enum sync_dimension sync_mode);
void edma_link(unsigned from, unsigned to);
void edma_unlink(unsigned from);

void edma_chain(unsigned from, unsigned to);
void edma_unchain(unsigned from);

/* calls that operate on an entire parameter RAM slot */
void edma_write_slot(unsigned slot, const struct edmacc_param *params);
void edma_read_slot(unsigned slot, struct edmacc_param *params);

/* channel control operations */
int edma_start(unsigned channel);
void edma_stop(unsigned channel);
void edma_clean_channel(unsigned channel);
void edma_clear_event(unsigned channel);
void edma_pause(unsigned channel);
void edma_resume(unsigned channel);

struct edma_rsv_info {

	const s16	(*rsv_chans)[2];
	const s16	(*rsv_slots)[2];
};

/* platform_data for EDMA driver */
struct edma_soc_info {

	/* how many dma resources of each type */
	unsigned	n_channel;
	unsigned	n_region;
	unsigned	n_slot;
	unsigned	n_tc;
	unsigned	n_cc;
	enum dma_event_q	default_queue;

	/* Resource reservation for other cores */
	struct edma_rsv_info	*rsv;

	const s8	(*queue_tc_mapping)[2];
	const s8	(*queue_priority_mapping)[2];
};

#endif
