#
# Makefile for the linux c6x-specific parts of the memory manager.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#
# Note 2! The CFLAGS definition is now in the main makefile...

obj-y	 := init.o fault.o memory.o kmap.o dma-coherent.o

obj-$(CONFIG_TMS320C66X) += msmc.o
