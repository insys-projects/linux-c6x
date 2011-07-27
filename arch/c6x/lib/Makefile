#
# Makefile for arch/c6x/lib/
#

ifdef CONFIG_TI_C6X_COMPILER
include $(srctree)/arch/c6x/Makefile.lib
endif

lib-y  := memcmp.o memmove.o memset.o misc.o delay.o io-readwrite.o \
          unaligned.o divu.o divi.o pop_rts.o push_rts.o remi.o remu.o \
          strasgi.o llshru.o llshr.o llshl.o negll.o mpyll.o divull.o \
	  divremi.o divremu.o

lib-$(CONFIG_TMS320C64X)     += csum.o memcpy.o
lib-$(CONFIG_TMS320C64XPLUS) += csum_64plus.o memcpy_64plus.o strasgi_64plus.o
lib-$(CONFIG_TMS320C66X)     += csum_64plus.o memcpy_64plus.o strasgi_64plus.o
