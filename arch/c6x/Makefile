#
# linux/arch/c6x/Makefile
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#

PROCESSOR	 = c6x
ABI		:= elf

DEFINES	        := -D__linux__ -DNO_MM -D__TMS320C6X__
ifdef CONFIG_TMS320C64XPLUS
DEFINES         += -D__TMS320C6XPLUS__
endif

LDSCRIPT = $(obj)/arch/c6x/kernel/vmlinux.lds

ifdef CONFIG_TI_C6X_COMPILER
include $(srctree)/arch/c6x/Makefile.lib

CFLAGS_MODULE   += -mlarge-memory-model=3 --visibility=default
AFLAGS_MODULE   += -mlarge-memory-model=3
KBUILD_AFLAGS   = -Wa,-d__ASSEMBLY__ -Wa,-I$(srctree)/arch/c6x/include/asm -Wa,-I$(obj)/arch/c6x/kernel

AS		 = $(CROSS_COMPILE)gcc -c

HOSTCFLAGS_kallsyms.o += -DTI_ASM
export HOSTCFLAGS_kallsyms.o

# native works well enough but cross toolchain should have one
NM		= /usr/bin/nm
endif

ifdef CONFIG_TI_C6X_LINKER
LINKFLAGS	= -Wl,--unused_section_elimination=off

MODULESCRIPT := $(srctree)/arch/c6x/modules.cmd
export MODULESCRIPT
LDFLAGS_MODULE = $(MODULESCRIPT)

# Flags used for the final link
LDFLAGS_vmlinux	:= --final-script=arch/c6x/vmlinux.cmd \
		   -Wl,-m,arch/c6x/vmlinux.map -Wl,--reread_libs

# FIXME. Why isn't the elf linker getting this from --final-script ?
LDFLAGS_vmlinux	+= -Wl,arch/c6x/vmlinux.cmd

LD		= $(CROSS_COMPILE)gcc -nostdlib $(LINKFLAGS)
endif

KBUILD_CPPFLAGS += -I$(srctree)/arch/c6x/platforms/include -I$(obj)/arch/c6x/kernel

ifdef CONFIG_DEBUG_INFO
# KBUILD_CFLAGS	:= $(CPPFLAGS) $(DEFINES) -g
KBUILD_CFLAGS   += $(DEFINES) -g
KBUILD_AFLAGS   += -g
else
# KBUILD_CFLAGS := $(CPPFLAGS) $(DEFINES) -Os
KBUILD_CFLAGS   += $(DEFINES) -O2
endif

ifdef CONFIG_NO_FRAME_POINTER
KBUILD_CFLAGS	:= $(KBUILD_CFLAGS:-fomit-frame-pointer=)
endif

# Select CPU dependent flags.  Note that order of declaration is important;
# the options further down the list override previous items.

arch-$(CONFIG_TMS320C64X)     := -march=6400
arch-$(CONFIG_TMS320C64XPLUS) := -march=6400+
nk-flags-                 := 0
nk-flags-y                := 1
be-flags-                 := 0
be-flags-y                := 1

ifdef CONFIG_CPU_BIG_ENDIAN
KBUILD_CFLAGS   += -mbig-endian
KBUILD_AFLAGS	+= -mbig-endian
LINKFLAGS	+= -mbig-endian
KBUILD_LDFLAGS	+= -mbig-endian
endif

KBUILD_CFLAGS	+= $(arch-y)
KBUILD_AFLAGS	+= $(arch-y) -Wa,-d__NK__=$(nk-flags-$(CONFIG_NK)) \
                   -Wa,-d__BIG_ENDIAN__=$(be-flags-$(CONFIG_CPU_BIG_ENDIAN))

head-y		:= arch/c6x/kernel/head.o
core-y		+= arch/c6x/kernel/ arch/c6x/mm/ arch/c6x/drivers/ arch/c6x/platforms/
libs-y		+= arch/c6x/lib/

ifdef CONFIG_NK
core-y         	+= arch/c6x/nkernel/
endif

ifdef CONFIG_TI_C6X_COMPILER
LIBGCC := $(shell $(CC) $(KBUILD_CFLAGS) -print-file-name=libcl6x.a)
else
LIBGCC := $(shell $(CC) $(KBUILD_CFLAGS) -print-libgcc-file-name)
endif
libs-y		+= $(LIBGCC)

ifdef CONFIG_TI_C6X_LINKER
TI_LDSCRIPT := $(obj)/arch/c6x/vmlinux.cmd
BUILTINSCRIPT := $(srctree)/arch/c6x/builtin.cmd
export BUILTINSCRIPT

.tmp_vmlinux1: $(TI_LDSCRIPT)

$(TI_LDSCRIPT): $(LDSCRIPT)
	cp $< $@

cmd_vmlinux__ = $(LD) $(LDFLAGS) $(LDFLAGS_vmlinux) -o $@ \
	$(vmlinux-init) \
	--start-group $(vmlinux-main) --end-group \
	$(LIBGCC) \
	$(obj)/arch/c6x/lib/lib.a $(obj)/lib/lib.a \
	$(filter-out $(vmlinux-lds) $(TI_LDSCRIPT) $(vmlinux-init) $(vmlinux-main) vmlinux.o FORCE ,$^)

cmd_vmlinux-modpost = $(LD) $(LDFLAGS) $(BUILTINSCRIPT) -r -o $@         \
	 $(vmlinux-init) --start-group $(vmlinux-main) --end-group       \
	 $(filter-out $(vmlinux-init) $(vmlinux-main) FORCE ,$^)
endif

#CLEAN_FILES =
MRPROPER_FILES = arch/c6x/kernel/c6x_config.inc arch/c6x/kernel/c6x_defs.inc
ifdef CONFIG_TI_C6X_LINKER
MRPROPER_FILES += arch/c6x/vmlinux.cmd arch/c6x/vmlinux.map
endif
#archclean:
