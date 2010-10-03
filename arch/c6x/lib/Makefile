#
# Makefile for arch/c6x/lib/
#

include $(srctree)/arch/c6x/Makefile.lib

lib-y	 := memcmp.o memmove.o memset.o misc.o delay.o io-readwrite.o

lib-$(CONFIG_TMS320C64X)     += csum.o memcpy.o
lib-$(CONFIG_TMS320C64XPLUS) += csum_64plus.o memcpy_64plus.o
