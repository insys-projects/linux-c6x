#
# arch/c6x/boot/Makefile
#
suffix-y := bin
suffix-$(CONFIG_KERNEL_GZIP)	:= gz
suffix-$(CONFIG_KERNEL_BZIP2)	:= bz2
suffix-$(CONFIG_KERNEL_LZMA)	:= lzma
suffix-$(CONFIG_KERNEL_LZO)	:= lzo

targets := zImage zImage.bin
extra-y += vmlinux.bin vmlinux.bin.gz vmlinux.bin.bz2 vmlinux.bin.lzma \
	   vmlinux.bin.lzo
subdir- := compressed

OBJCOPYFLAGS:= -R .comment -S

$(obj)/zImage: $(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)
	@echo 'Kernel: $@ is ready'

$(obj)/zImage.bin: $(obj)/compressed/vmlinux FORCE
	$(Q)$(OBJCOPY) $(OBJCOPYFLAGS) -O binary $< $@
	@echo 'Kernel: $@ is ready'

$(obj)/compressed/vmlinux: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed $@

$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/vmlinux.bin.gz: $(obj)/vmlinux.bin FORCE
	$(call if_changed,gzip)

$(obj)/vmlinux.bin.bz2: $(obj)/vmlinux.bin FORCE
	$(call if_changed,bzip2)

$(obj)/vmlinux.bin.lzma: $(obj)/vmlinux.bin FORCE
	$(call if_changed,lzma)

$(obj)/vmlinux.bin.lzo: $(obj)/vmlinux.bin FORCE
	$(call if_changed,lzo)

export suffix-y
