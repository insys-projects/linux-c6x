#
# linux/arch/c6x/boot/compressed/Makefile
#
# create a compressed vmlinux image from the original vmlinux
#

targets		:= vmlinux vmlinux.bin vmlinux.bin.gz \
		   vmlinux.bin.bz2 vmlinux.bin.lzma \
		   vmlinux.bin.lzo \
		   head.o misc.o piggy.o vmlinux.lds

be-flags-       := le
be-flags-y      := be
ld-bfd		:= elf32-tic6x-linux-$(be-flags-$(CONFIG_CPU_BIG_ENDIAN))

OBJECTS = $(obj)/head.o $(obj)/misc.o $(obj)/cache.o

#
# IMAGE_OFFSET is the load offset of the compression loader
#
# default CONFIG_BOOT_LINK_OFFSET set to 8MB (can be overidden by Kconfig)
#
CONFIG_BOOT_LINK_OFFSET	?= 0x00800000

IMAGE_OFFSET	:= $(shell /bin/bash -c 'printf "0x%08x" \
		     $$[$(CONFIG_PAGE_OFFSET)  + \
			$(CONFIG_BOOT_LINK_OFFSET)]')

LIBGCC	:= $(shell $(CC) $(KBUILD_CFLAGS) -print-libgcc-file-name)

LDFLAGS_vmlinux := -Ttext $(IMAGE_OFFSET) -e startup -T

OBJCOPYFLAGS:= -O binary -R .note -R .comment -R .GCC-command-line -R .note.gnu.build-id -S

$(obj)/vmlinux: $(obj)/vmlinux.lds $(OBJECTS) $(obj)/piggy.o $(LIBGCC) FORCE
	$(call if_changed,ld)
	@:

$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

vmlinux.bin.all-y := $(obj)/vmlinux.bin

$(obj)/vmlinux.bin.gz: $(vmlinux.bin.all-y) FORCE
	$(call if_changed,gzip)
$(obj)/vmlinux.bin.bz2: $(vmlinux.bin.all-y) FORCE
	$(call if_changed,bzip2)
$(obj)/vmlinux.bin.lzma: $(vmlinux.bin.all-y) FORCE
	$(call if_changed,lzma)
$(obj)/vmlinux.bin.lzo: $(vmlinux.bin.all-y) FORCE
	$(call if_changed,lzo)

LDFLAGS_piggy.o := -r --format binary --oformat $(ld-bfd) -T

$(obj)/piggy.o: $(obj)/vmlinux.scr $(obj)/vmlinux.bin.$(suffix-y) FORCE
	$(call if_changed,ld)
