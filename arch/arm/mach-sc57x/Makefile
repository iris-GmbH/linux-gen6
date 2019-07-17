#
# Makefile for the linux kernel.
#

obj-y					:= core.o gptimers.o dma.o clock.o sec.o
obj-$(CONFIG_ARCH_SC57X)		+= sc57x-ezkit.o core_control.o early_i2c.o
obj-$(CONFIG_ARCH_SRAM_ALLOC)		+= sram.o sram_mmap.o
obj-$(CONFIG_SC57X_PCI)                 += pcie.o
obj-$(CONFIG_ICC:m=y) += icc.o

obj-$(CONFIG_IRIS_CUSTOM_DRIVERS) += iris/
