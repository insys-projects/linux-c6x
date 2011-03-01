#
# Makefile for arch/c6x/platforms
#
# Copyright 2010, 2011 Texas Instruments Incorporated
#

include $(srctree)/arch/c6x/Makefile.lib

obj-$(CONFIG_TMS320C64XPLUS) += platform-c64xx.o
obj-$(CONFIG_TMS320C66X)     += platform-c66xx.o

# SoC objects
obj-$(CONFIG_SOC_TMS320C6472)	+= mcore.o
obj-$(CONFIG_SOC_TMS320C6474)	+= mcore.o cic.o

# Board objects
obj-$(CONFIG_ARCH_BOARD_DSK6455)    += board-dsk6455.o
obj-$(CONFIG_ARCH_BOARD_EVM6457)    += board-evm6457.o phy.o
obj-$(CONFIG_ARCH_BOARD_EVM6472)    += board-evm6472.o phy.o
obj-$(CONFIG_ARCH_BOARD_EVM6474)    += board-evm6474.o phy.o
obj-$(CONFIG_ARCH_BOARD_EVM6474L)   += board-evm6474lite.o phy.o
obj-$(CONFIG_ARCH_BOARD_EVMTCI6616) += board-evmtci6616.o
obj-$(CONFIG_ARCH_BOARD_EVM6678)    += board-evm6678.o
obj-$(CONFIG_ARCH_BOARD_EVM6670)    += board-evm6670.o
