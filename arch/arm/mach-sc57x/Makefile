#
# Makefile for the linux kernel.
#

obj-y					:= core.o gptimers.o dma.o clock.o sec.o
obj-$(CONFIG_MACH_SC573_EZKIT)		+= sc57x-ezkit.o
obj-$(CONFIG_ICC:m=y) += icc.o

obj-$(CONFIG_IRIS_CUSTOM_DRIVERS) += iris/
