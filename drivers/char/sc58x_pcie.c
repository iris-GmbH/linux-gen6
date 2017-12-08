/*
 * Analog Devices SC58X ezkit pcie device driver
 *
 * Copyright (c) 2015 Analog Devices Inc.
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
#include <linux/io.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#define PCI_VENDOR_ID_SC58X   0x1204
#define PCI_DEVICE_ID_SC58X   0xec30

#define PCIE_REG_BASE          0x310B8000

#define APP_INTSTAT            0x2C08
#define MSI_CTL_LADDR          0x820
#define MSI_CTL_UADDR          0x824
#define MSI_IEN0               0x828
#define MSI_IMSK0              0x82C
#define MSI_ISTAT0             0x830

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

#define DRIVER_NAME "sc58x ezkit pcie"
#define RX_BUFF_SIZE           0x200

#define REG(x) offsetof(struct dma_buff, x)
struct sc58x_pcie {
	struct pci_dev *dev;
	void __iomem *bar0;
	unsigned long base;
	void *data;
	dma_addr_t dma_addr;
};

struct dma_buff {
	u32 command; /* bit 0: reset, bit 1: write, bit 2: read */
	u32 status; /* bit 0: write finish, bit 1: read finish */
	u32 dma_addr;
	u32 dma_size;
	u32 data[RX_BUFF_SIZE];
};

static const struct pci_device_id sc58x_ezkit_ids[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_SC58X, PCI_DEVICE_ID_SC58X)},
	{0},
};
MODULE_DEVICE_TABLE(pci, sc58x_ezkit_ids);

static inline u32 reg_read(unsigned long addr)
{
	return ioread32((u32 *)addr);
}

static inline void reg_write(u32 val, unsigned long addr)
{
	iowrite32(val, (u32 *)addr);
}

static void sc58x_dma_read(struct sc58x_pcie *adapter, int size,
		unsigned long src, unsigned long dst)
{
	u32 val;

	reg_write(0x80000000, adapter->base + DMA_VIEWPORT_SEL);
	reg_write(0x1, adapter->base + DMA_READ_EN);
	reg_write(0x0, adapter->base + DMA_READ_INT_MASK);
	reg_write(0x04000008, adapter->base + DMA_CTRL1);
	
	reg_write(size, adapter->base + DMA_XFER_SIZE);
	reg_write(src, adapter->base + DMA_SAR_LOW);
	reg_write(0x0, adapter->base + DMA_SAR_HIGH);
	reg_write(dst, adapter->base + DMA_DAR_LOW);
	reg_write(0x0, adapter->base + DMA_DAR_HIGH);
	reg_write(0x0, adapter->base + DMA_READ_START);

	while (1) {
		val = reg_read(adapter->base + DMA_READ_INT_STAT);
		if (val & 0x1)
			break;
	}
	reg_write(0x1, adapter->base + DMA_READ_INT_CLEAR);
}

static void sc58x_dma_write(struct sc58x_pcie *adapter, int size,
		unsigned long src, unsigned long dst)
{
	u32 val;

	reg_write(0x0, adapter->base + DMA_VIEWPORT_SEL);
	reg_write(0x1, adapter->base + DMA_WRITE_EN);
	reg_write(0x0, adapter->base + DMA_WRITE_INT_MASK);
	reg_write(0x04000008, adapter->base + DMA_CTRL1);
	
	reg_write(size, adapter->base + DMA_XFER_SIZE);
	reg_write(src, adapter->base + DMA_SAR_LOW);
	reg_write(0x0, adapter->base + DMA_SAR_HIGH);
	reg_write(dst, adapter->base + DMA_DAR_LOW);
	reg_write(0x0, adapter->base + DMA_DAR_HIGH);
	reg_write(0x0, adapter->base + DMA_WRITE_START);

	while (1) {
		val = reg_read(adapter->base + DMA_WRITE_INT_STAT);
		if (val & 0x1)
			break;
	}
	reg_write(0x1, adapter->base + DMA_WRITE_INT_CLEAR);
}

static irqreturn_t sc58x_irq(int irq, void *dev)
{
	struct sc58x_pcie *adapter = (struct sc58x_pcie *)dev;
	struct dma_buff *buf = (struct dma_buff *)adapter->data;
	unsigned long ep_addr = (unsigned long)adapter->bar0 + 0x80000;

	buf->command  = *(u32 *)(ep_addr + REG(command));
	buf->status = *(u32 *)(ep_addr + REG(status));

	if (buf->command & 0x1) {
		/* remember to tell ep your pci address
		 * before take it out of reset
		 */
		buf->command = 0x0;
		buf->dma_addr = adapter->dma_addr;
		*(u32 *)(ep_addr + REG(dma_addr)) = buf->dma_addr;
		*(u32 *)(ep_addr + REG(command)) = buf->command;
	} else if (buf->command == 0x0) {
		buf->command = 0x2;
		buf->dma_addr = adapter->dma_addr + REG(data[0]);
		buf->dma_size = 4;
		*(u32 *)(ep_addr + REG(dma_addr)) = buf->dma_addr;
		*(u32 *)(ep_addr + REG(dma_size)) = buf->dma_size;
		*(u32 *)(ep_addr + REG(command)) = buf->command;
	} else if (buf->status & 0x1) {
		pr_info("inbound write finish %x\n", buf->data[0]);
		buf->command = 0x4;
		buf->status &= ~0x1;
		buf->dma_addr = adapter->dma_addr + REG(data[1]);
		buf->dma_size = 4;
		buf->data[0] = 0xffffffff;
		buf->data[1] = 0x55aaaa55;
		*(u32 *)(ep_addr + REG(dma_addr)) = buf->dma_addr;
		*(u32 *)(ep_addr + REG(dma_size)) = buf->dma_size;
		*(u32 *)(ep_addr + REG(status)) = buf->status;
		*(u32 *)(ep_addr + REG(command)) = buf->command;
	} else if (buf->status & 0x2) {
		pr_info("inbound read finish\n");
		buf->status &= ~0x2;
		buf->command = 0x1;
		*(u32 *)(ep_addr + REG(status)) = buf->status;
		*(u32 *)(ep_addr + REG(command)) = buf->command;
	}

	return IRQ_HANDLED;
}

static int sc58x_ezkit_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int i, ret;
	unsigned char *buf;
	struct sc58x_pcie *adapter;
	u32 base;
	
	adapter = kzalloc(sizeof(struct sc58x_pcie), GFP_KERNEL);
	if (!adapter)
		return -ENOMEM;
	adapter->data = dma_alloc_coherent(&dev->dev, sizeof(struct dma_buff),
				&adapter->dma_addr, GFP_KERNEL);
	if (!adapter->data)
		goto free_adapter;
	adapter->base = (unsigned long)ioremap_nocache(PCIE_REG_BASE, SZ_16K);
	if (!adapter->base)
		goto free_data;
	adapter->dev = dev;
	pci_set_drvdata(dev, adapter);

	ret = pci_enable_device(dev);
	if (ret)
		goto reg_unmap;
	pci_set_master(dev);

	ret = pci_set_dma_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&dev->dev, "can not set dma mask\n");
		goto disable_dev;
	}

	ret = pci_request_regions(dev, DRIVER_NAME);
	if (ret)
		goto disable_dev;
	for(i=0; i<6; i++) {
		pr_info("bar[%d]=%x@%x\n", i, pci_resource_start(dev, i),
				pci_resource_len(dev, i));
	}

	base = pci_resource_start(dev, 0);
	if (!base) {
		ret = -EIO;
		dev_err(&dev->dev, "no MMIO\n");
		goto release_region;
	}
	adapter->bar0 = ioremap(base, pci_resource_len(dev, 0));
	if (!adapter->bar0) {
		ret = -EIO;
		dev_err(&dev->dev, "can not map MMIO\n");
		goto release_region;
	}

	ret = pci_enable_msi(dev);
	if (ret) {
		dev_err(&dev->dev, "can not enable msi\n");
		goto unmap;
	}

	ret = request_irq(dev->irq, sc58x_irq, 0, DRIVER_NAME, adapter);
	if (ret) {
		dev_err(&dev->dev, "can not request irq\n");
		goto disable_msi;
	}

	return ret;

disable_msi:
	pci_disable_msi(dev);
unmap:
	iounmap(adapter->bar0);
release_region:
	pci_release_regions(dev);
disable_dev:
	pci_disable_device(dev);
reg_unmap:
	iounmap((void *)adapter->base);
free_data:
	dma_free_coherent(&dev->dev, sizeof(struct dma_buff),
			adapter->data, adapter->dma_addr);
free_adapter:
	kfree(adapter);
	return 0;
}

static void sc58x_ezkit_remove(struct pci_dev *dev)
{
	struct sc58x_pcie *adapter = pci_get_drvdata(dev);

	free_irq(dev->irq, NULL);
	pci_disable_msi(dev);
	iounmap(adapter->bar0);
	pci_release_regions(dev);
	pci_disable_device(dev);
	iounmap((void *)adapter->base);
	dma_free_coherent(&dev->dev, sizeof(struct dma_buff),
			adapter->data, adapter->dma_addr);
	kfree(adapter);
}

static struct pci_driver sc58x_ezkit_driver = {
	.name = DRIVER_NAME, 
	.id_table = sc58x_ezkit_ids,
	.probe = sc58x_ezkit_probe,
	.remove = sc58x_ezkit_remove,
};

module_pci_driver(sc58x_ezkit_driver);

MODULE_DESCRIPTION("SC58X ezkit board PCIE  device driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
