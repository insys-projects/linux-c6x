#
# Makefile for arch/c6x/lib/
#

include $(srctree)/arch/c6x/Makefile.lib

lib-y	 := memcmp.o memmove.o memset.o misc.o delay.o io-readwrite.o

lib-$(CONFIG_TMS320C64X)     += csum.o memcpy.o
lib-$(CONFIG_TMS320C64XPLUS) += csum_64plus.o memcpy_64plus.o
lib-$(CONFIG_BINFMT_COFF) += coff.o

ifeq ($(CONFIG_TMS320C64XPLUS),y)
csum_64plus.o: $(src)/../include/asm/cnames.inc
memcpy_64plus.o: $(src)/../include/asm/cnames.inc
endif

ifeq ($(CONFIG_TMS320C64X),y)
csum.o: $(src)/../include/asm/cnames.inc
memcpy.o: $(src)/../include/asm/cnames.inc
endif

delay.o: $(src)/../include/asm/cnames.inc
