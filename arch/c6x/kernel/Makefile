#
# Makefile for arch/c6x/kernel/
#

extra-y	 := head.o vmlinux.lds

obj-y 	 := process.o traps.o irq.o signal.o ptrace.o \
	    setup.o sys_c6x.o time.o \
            entry.o vectors.o c6x_ksyms.o tags.o

obj-$(CONFIG_PCI)              += bios32.o
obj-$(CONFIG_MODULES)          += module.o
obj-$(CONFIG_COMMON_CLKDEV)    += clkdev.o

ifdef CONFIG_TI_C6X_COMPILER

$(obj)/entry.o:    $(obj)/c6x_defs.inc $(obj)/c6x_config.inc
$(obj)/head.o:     $(obj)/c6x_config.inc

config-preempt-y += "CONFIG_PREEMPT    		.set 1"
config-preempt-  += "CONFIG_PREEMPT    		.set 0"
config-c64-y     += "CONFIG_TMS320C64X 		.set 1"
config-c64-      += "CONFIG_TMS320C64X 		.set 0"
config-c64plus-y += "CONFIG_TMS320C64XPLUS	.set 1"
config-c64plus-  += "CONFIG_TMS320C64XPLUS	.set 0"
config-c66-y     += "CONFIG_TMS320C66X		.set 1"
config-c66-      += "CONFIG_TMS320C66X		.set 0"
config-mtduclinux-y += "CONFIG_MTD_UCLINUX	.set 1"
config-mtduclinux-  += "CONFIG_MTD_UCLINUX	.set 0"

$(obj)/c6x_defs.inc: include/generated/asm-offsets.h $(srctree)/arch/c6x/kernel/tags.h
	@sed -ne 's/#define \(.*\) \(.*\)/\1 .set \2/p' $< > $@
	@sed -ne 's/#define[ \t]*\(.*\)[ \t]\(.*\)/\1 .set \2/p' $(srctree)/arch/c6x/kernel/tags.h >> $@

$(obj)/c6x_config.inc: include/generated/autoconf.h
	@echo $(config-preempt-$(CONFIG_PREEMPT)) > $(obj)/c6x_config.inc
	@echo $(config-c64-$(CONFIG_TMS320C64X)) >> $(obj)/c6x_config.inc
	@echo $(config-c64plus-$(CONFIG_TMS320C64XPLUS)) >> $(obj)/c6x_config.inc
	@echo $(config-c66-$(CONFIG_TMS320C66X)) >> $(obj)/c6x_config.inc
	@echo $(config-mtduclinux-$(CONFIG_MTD_UCLINUX)) >> $(obj)/c6x_config.inc

endif

