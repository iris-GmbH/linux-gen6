#
# Makefile for the linux kernel.
#

obj-y					:= core.o gptimers.o dma.o clock.o sec.o
obj-$(CONFIG_ARCH_SC58X)		+= sc58x-ezkit.o core_control.o
obj-$(CONFIG_ARCH_SRAM_ALLOC)		+= sram.o sram_mmap.o
obj-$(CONFIG_SC58X_PCI)                 += pcie.o
obj-$(CONFIG_ICC:m=y) += icc.o
