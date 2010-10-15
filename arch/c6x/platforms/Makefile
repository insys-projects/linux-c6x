#
# Makefile for arch/c6x/platforms
#
# Copyright 2010 Texas Instruments Incorporated
#

include $(srctree)/arch/c6x/Makefile.lib

obj-y  += platform-c64xx.o

# SoC objects
obj-$(CONFIG_SOC_TMS320C6472)	+= mcore.o ipc.o
obj-$(CONFIG_SOC_TMS320C6474)	+= mcore.o ipc.o

# Board objects
obj-$(CONFIG_ARCH_BOARD_DSK6455) += board-dsk6455.o
obj-$(CONFIG_ARCH_BOARD_EVM6472) += board-evm6472.o
obj-$(CONFIG_ARCH_BOARD_EVM6474) += board-evm6474.o phy-evm6474.o
obj-$(CONFIG_ARCH_BOARD_EVM6474L) += board-evm6474lite.o phy-evm6474l.o
