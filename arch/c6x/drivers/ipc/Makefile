#
# Makefile for arch/c6x/drivers/ipc
#
obj-y := core.o

# Low-level IPC objects
obj-$(CONFIG_SOC_TMS320C6472) += ipc-tci648x.o
obj-$(CONFIG_SOC_TMS320C6474) += ipc-tci648x.o

# virtio support objects
obj-$(CONFIG_VIRTIO_IPC)      += virtio_ipc.o virtio_bridge.o
