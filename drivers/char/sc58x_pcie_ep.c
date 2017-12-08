/*
 * Analog Devices PCIe endpoint driver
 *
 * Copyright (c) 2014 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/sizes.h>

#include <mach/cpu.h>

#define PCIE_REG_BASE          0x310B8000
#define MSI_CAP_PTR            0x50
#define DEV_CAP_REG            0x74
#define DEV_CTRL_STATUS_REG    0x78
#define LINK_CAP_REG           0X7C
#define LINK_CTRL_STATUS_REG   0x80
#define DEV_CAP_REG2           0x94
#define DEV_CTRL_STATUS_REG2   0x98
#define LINK_CAP_REG2          0X9C
#define LINK_CTRL_STATUS_REG2  0xA0
#define ADV_ERR_CAP_CTRL       0x118

#define PORT_LINK_CTRL         0x710
#define PIPE_LOOPBACK_CTRL     0x8B8
#define MISC_CTRL1             0x8BC
#define IATU_VIEWPORT          0x900
#define IATU_REGION_CTRL1      0x904
#define IATU_REGION_CTRL2      0x908
#define IATU_LWR_BASE_ADDR     0x90C
#define IATU_UPPER_BASE_ADDR   0x910
#define IATU_LIMIT_ADDR        0x914
#define IATU_LWR_TAR_ADDR      0x918
#define IATU_UPPER_TAR_ADDR    0x91C
#define IATU_REGION_CTRL3      0x920

#define DMA_WRITE_EN           0x97C
#define DMA_WRITE_START        0x980
#define DMA_READ_EN            0x99C
#define DMA_READ_START         0x9A0
#define DMA_WRITE_INT_STAT     0x9BC
#define DMA_WRITE_INT_MASK     0x9C4
#define DMA_WRITE_INT_CLEAR    0x9C8
#define DMA_READ_INT_STAT      0xA10
#define DMA_READ_INT_MASK      0xA18
#define DMA_READ_INT_CLEAR     0xA1C
#define DMA_VIEWPORT_SEL       0xA6C
#define DMA_CTRL1              0xA70
#define DMA_XFER_SIZE          0xA78
#define DMA_SAR_LOW            0xA7C
#define DMA_SAR_HIGH           0xA80
#define DMA_DAR_LOW            0xA84
#define DMA_DAR_HIGH           0xA88

#define BAR0_CS                0x1010
#define BAR1_CS                0x1014
#define BAR2_CS                0x1018
#define BAR3_CS                0x101C
#define BAR4_CS                0x1020

/* ADI APP Registers */
#define APP_CTRL               0x2C00
#define APP_STAT               0x2C04


/* RSCKPHY Registers */
#define RSCKPHY_CTL            0x3000
#define RSCKPHY_STAT           0x3004
#define RSCKPHY_ENG            0x3008
#define RSCKPHY_REVID          0x3FFC

#define RX_BUFF_SIZE           0x200
#define REG(x) offsetof(struct dma_buff, x)
static struct pcie_dev {
	int major;
	struct cdev cdev;
	unsigned long base;
	void *virt;
	dma_addr_t phy;
	dma_addr_t dma_base;
} device;

struct dma_buff {
	u32 command; /* bit 0: reset, bit 1: write, bit 2: read */
	u32 status; /* bit 0: write finish, bit 1: read finish */
	u32 dma_addr;
	u32 dma_size;
	u32 data[RX_BUFF_SIZE];
};

static inline u32 reg_read(unsigned long addr)
{
	return ioread32((u32 *)addr);
}

static inline void reg_write(u32 val, unsigned long addr)
{
	iowrite32(val, (u32 *)addr);
}

static int sc58x_link_train(struct pcie_dev *dev)
{
	pr_info("start link training...\n");
	reg_write(0x1, dev->base + APP_CTRL);
	while (1) {
		if (reg_read(dev->base + APP_STAT) & 0x1) {
			pr_info("PCIE link is up\n");
			return 0;
		} else if (reg_read(dev->base + RSCKPHY_STAT) & 0x20) {
			reg_write(0x20, dev->base + RSCKPHY_STAT);
			reg_write(0x102, dev->base + RSCKPHY_CTL);
			pr_info("PCIE hot reset\n");
			return -1;
		}
		udelay(100);
	}
	return 0;
}

static void sc58x_pcie_init(struct pcie_dev *dev)
{
	u32 val;

	/* enable write to read-only register through DBI */
	reg_write(0x1, dev->base + MISC_CTRL1);

	reg_write(0x8, dev->base + PCI_BASE_ADDRESS_0);
	reg_write(0xfffffff8, dev->base + PCI_BASE_ADDRESS_1);
	reg_write(0xfffffff8, dev->base + PCI_BASE_ADDRESS_2);
	reg_write(0xfffffff8, dev->base + PCI_BASE_ADDRESS_3);
	reg_write(0xfffffff8, dev->base + PCI_BASE_ADDRESS_4);
	reg_write(0xfffffff8, dev->base + PCI_BASE_ADDRESS_5);

	reg_write(0x0, dev->base + BAR1_CS);
	reg_write(0x0, dev->base + BAR2_CS);
	reg_write(0x0, dev->base + BAR3_CS);
	reg_write(0x0, dev->base + BAR4_CS);

	reg_write((0xec30 << 16) | 0x1204, dev->base + PCI_VENDOR_ID);
	reg_write((0x3010 << 16) | 0x1204,
			dev->base + PCI_SUBSYSTEM_VENDOR_ID);
	reg_write(PCI_CLASS_STORAGE_OTHER << 16,
			dev->base + PCI_CLASS_REVISION);

	/* enable MSI and request one interrupt */
	val = reg_read(dev->base + MSI_CAP_PTR);
	//reg_write((val & 0x0000ffff) | 0x00030000, dev->base + MSI_CAP_PTR);
	reg_write((val & 0x0000ffff) | 0x00010000, dev->base + MSI_CAP_PTR);

	reg_write(0x0040ac11, dev->base + LINK_CAP_REG);
	reg_write(PCI_COMMAND_IO | PCI_COMMAND_MEMORY, dev->base + PCI_COMMAND);
	reg_write(0x0, dev->base + MISC_CTRL1);
}

static void sc58x_send_msi(struct pcie_dev *dev, int vec)
{
	u32 val;

	while (reg_read(dev->base + APP_STAT) & 0x8)
		udelay(100);
	val = reg_read(dev->base + APP_CTRL);
	val &= 0xfffffc1f;
	val |= (vec << 5) | 0x8;
	reg_write(val, dev->base + APP_CTRL);
	while (!(reg_read(dev->base + APP_STAT) & 0x4))
		udelay(100);
	reg_write(0x4, dev->base + APP_STAT);
	pr_info("send msi %d\n", vec);
}

static void sc58x_dma_read(struct pcie_dev *dev, int size,
		unsigned long src, unsigned long dst)
{
	u32 val;

	reg_write(0x80000000, dev->base + DMA_VIEWPORT_SEL);
	reg_write(0x1, dev->base + DMA_READ_EN);
	reg_write(0x0, dev->base + DMA_READ_INT_MASK);
	reg_write(0x04000008, dev->base + DMA_CTRL1);

	reg_write(size, dev->base + DMA_XFER_SIZE);
	reg_write(src, dev->base + DMA_SAR_LOW);
	reg_write(0x0, dev->base + DMA_SAR_HIGH);
	reg_write(dst, dev->base + DMA_DAR_LOW);
	reg_write(0x0, dev->base + DMA_DAR_HIGH);
	reg_write(0x0, dev->base + DMA_READ_START);

	while (1) {
		val = reg_read(dev->base + DMA_READ_INT_STAT);
		if (val & 0x1)
			break;
	}
	reg_write(0x1, dev->base + DMA_READ_INT_CLEAR);
}

static void sc58x_dma_write(struct pcie_dev *dev, int size,
		unsigned long src, unsigned long dst)
{
	u32 val;

	reg_write(0x0, dev->base + DMA_VIEWPORT_SEL);
	reg_write(0x1, dev->base + DMA_WRITE_EN);
	reg_write(0x0, dev->base + DMA_WRITE_INT_MASK);
	reg_write(0x04000008, dev->base + DMA_CTRL1);

	reg_write(size, dev->base + DMA_XFER_SIZE);
	reg_write(src, dev->base + DMA_SAR_LOW);
	reg_write(0x0, dev->base + DMA_SAR_HIGH);
	reg_write(dst, dev->base + DMA_DAR_LOW);
	reg_write(0x0, dev->base + DMA_DAR_HIGH);
	reg_write(0x0, dev->base + DMA_WRITE_START);

	while (1) {
		val = reg_read(dev->base + DMA_WRITE_INT_STAT);
		if (val & 0x1)
			break;
	}
	reg_write(0x1, dev->base + DMA_WRITE_INT_CLEAR);
}

static int sc58x_open(struct inode *inode, struct file *filp)
{
	unsigned long base;
	u32 val;

	set_spu_securep_msec(151, true);
	base = (unsigned long)ioremap_nocache(PCIE_REG_BASE, SZ_16K);
	if (!base)
		return -1;
	device.base = base;
	device.phy = 0x20080000;
	device.virt = ioremap_nocache(device.phy, SZ_2K);
	if (!device.virt)
		return -1;
	reg_write(0x3, base + RSCKPHY_CTL);
reset:	reg_write(0x0, base + RSCKPHY_CTL);
	while (reg_read(base + RSCKPHY_STAT) & 0x1f)
		mdelay(100);

	val = reg_read(base + RSCKPHY_STAT);
	/* clear hot reset request */
	if (val & 0x20)
		reg_write(0x20, base + RSCKPHY_STAT);
	/* clear lock write error */
	if (val & 0x20000)
		reg_write(0x20000, base + RSCKPHY_STAT);

	sc58x_pcie_init(&device);

	/* outbound address translation */
	reg_write(0x0, base + IATU_VIEWPORT);
	reg_write(0x50000000, base + IATU_LWR_BASE_ADDR);
	reg_write(0x0, base + IATU_UPPER_BASE_ADDR);
	reg_write(0x5fffffff, base + IATU_LIMIT_ADDR);
	reg_write(0x0, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x0, base + IATU_REGION_CTRL1);
	reg_write(0x80000000, base + IATU_REGION_CTRL2);
	/* inbound address translation */
	reg_write(0x80000000, base + IATU_VIEWPORT);
	reg_write(0x20000000, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x0, base + IATU_REGION_CTRL1);
	reg_write(0xc0000000, base + IATU_REGION_CTRL2);

	reg_write(0x80000001, base + IATU_VIEWPORT);
	reg_write(0x30000000, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x0, base + IATU_REGION_CTRL1);
	reg_write(0xc0000100, base + IATU_REGION_CTRL2);

	if (sc58x_link_train(&device))
		goto reset;
	pr_info("waiting rc enable device...\n");
	while (1) {
		if (reg_read(base + PCI_COMMAND) & PCI_COMMAND_MASTER) {
			pr_info("BAR0 = %x\n",
				reg_read(base + PCI_BASE_ADDRESS_0));

			break;
		}
		udelay(100);
	}
	mdelay(10000);
	{
		struct dma_buff *buf = device.virt;
		unsigned long offset;

		/* in RESET state when startup */
		buf->command = 0x1;
		buf->status = 0x0;
		sc58x_send_msi(&device, 0);
		while (1) {
			if (!(buf->command & 0x1)) {
				device.dma_base = buf->dma_addr;
				pr_info("out of reset: %x\n", device.dma_base);
				sc58x_send_msi(&device, 0);
				break;
			}
			mdelay(100);
		}
		while (1) {
			if (buf->command & 0x2) {
				pr_info("inbound write %d bytes to %x\n",
						buf->dma_size, buf->dma_addr);
				buf->status |= 0x1;
				offset = buf->dma_addr - device.dma_base;
				buf->data[0] = 0x1234abcd;
				buf->data[1] = 0xffffffff;
				sc58x_dma_write(&device, buf->dma_size,
						device.phy + offset,
						buf->dma_addr);
				sc58x_send_msi(&device, 0);
			} else if (buf->command & 0x4) {
				offset = buf->dma_addr - device.dma_base;
				sc58x_dma_read(&device, buf->dma_size,
						buf->dma_addr,
						device.phy + offset);
				pr_info("inbound read %x[%x]\n", buf->data[0],
						buf->data[1]);
				buf->status |= 0x2;
				sc58x_send_msi(&device, 0);
			}
			mdelay(100);
		}
	}
	return 0;
}

static int sc58x_release(struct inode *inode, struct file *filp)
{
	void *base = (void *)device.base;

	iounmap(base);
	return 0;
}

static const struct file_operations sc58x_fops = {
	.owner = THIS_MODULE,
	.open = sc58x_open,
	.release = sc58x_release,
};

static int __init sc58x_pcie_ep_init(void)
{
	dev_t dev;
	int ret;

	ret = alloc_chrdev_region(&dev, 0, 1, "pcie ep");
	if (ret < 0)
		return ret;
	cdev_init(&device.cdev, &sc58x_fops);
	device.cdev.owner = THIS_MODULE;
	ret = cdev_add(&device.cdev, dev, 1);
	if (ret < 0)
		return ret;
	device.major = MAJOR(dev);
	sc58x_open(NULL, NULL);

	return 0;
}

static void __exit sc58x_pcie_ep_exit(void)
{
	dev_t dev = MKDEV(device.major, 0);

	cdev_del(&device.cdev);
	unregister_chrdev_region(dev, 1);
}

module_init(sc58x_pcie_ep_init);
module_exit(sc58x_pcie_ep_exit);

MODULE_DESCRIPTION("sc58x pcie endpoint driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
