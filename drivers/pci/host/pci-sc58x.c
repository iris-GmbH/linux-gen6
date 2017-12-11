/*
 * Analog Devices PCIe host controller driver
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/sizes.h>

#include <mach/cpu.h>

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
#define PORT_LINK_DEBUG0       0x728
#define PORT_LINK_DEBUG1       0x72C
#define QUEUE_STAT             0x73C

#define MSI_CTL_LADDR          0x820
#define MSI_CTL_UADDR          0x824
#define MSI_IEN0               0x828
#define MSI_IMSK0              0x82C
#define MSI_ISTAT0             0x830
#define MSI_IEN1               0x834
#define MSI_IMSK1              0x838
#define MSI_ISTAT1             0x83C
#define MSI_IEN2               0x840
#define MSI_IMSK2              0x844
#define MSI_ISTAT2             0x848
#define MSI_IEN3               0x84C
#define MSI_IMSK3              0x850
#define MSI_ISTAT3             0x854
#define MSI_IEN4               0x858
#define MSI_IMSK4              0x85C
#define MSI_ISTAT4             0x860
#define MSI_IEN5               0x864
#define MSI_IMSK5              0x868
#define MSI_ISTAT5             0x86C
#define MSI_IEN6               0x870
#define MSI_IMSK6              0x874
#define MSI_ISTAT6             0x878
#define MSI_IEN7               0x87C
#define MSI_IMSK7              0x880
#define MSI_ISTAT7             0x884
#define MSI_GPIO_IO            0x888

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

/* Link Control and Status register bits */
#define LNK_CTLSTAT_SP         0xf0000
#define LNK_CTLSTAT_NEG_LW     0x1f00000

/* ADI APP Registers */
#define APP_CTRL               0x2C00
#define APP_STAT               0x2C04
#define APP_INTSTAT            0x2C08

/* APP CTRL register bits */
#define APP_CTRL_LTSSMEN       0x1
#define APP_CTRL_LEGACY_EN     0xF0000
#define APP_CTRL_VENINTEN      0x100000
#define APP_CTRL_UNLCKINTEN    0x200000

/* APP STAT register bits */
#define APP_STAT_LNKUP         0x1

/* ADI APP INTSTAT legacy int bits mask */
#define APP_INT_LEGACY	       0x1E
#define APP_INT_MSI            0x1

/* RSCKPHY Registers */
#define RSCKPHY_CTL            0x3000
#define RSCKPHY_STAT           0x3004
#define RSCKPHY_ENG            0x3008
#define RSCKPHY_REVID          0x3FFC

#define INT_PCI_MSI_NR (8 * 32)

#define SC58X_EZKIT_EP         1

struct sc58x_msi {
	struct msi_controller chip;
	DECLARE_BITMAP(used, INT_PCI_MSI_NR);
	struct irq_domain *domain;
	unsigned long pages;
	struct mutex lock;
};

struct sc58x_pcie {
	struct device *dev;
	void __iomem *base;
	void __iomem *conf;

	struct resource io;
	struct resource mem;
	struct resource prefetch;
	struct resource cfg;
	struct resource busn;
	int irq;

	struct sc58x_msi msi;
};

static inline struct sc58x_pcie *to_host(struct pci_sys_data *sys)
{
	return sys->private_data;
}

static inline struct sc58x_msi *to_sc58x_msi(struct msi_controller *chip)
{
	return container_of(chip, struct sc58x_msi, chip);
}

static inline u32 reg_read(unsigned long addr)
{
	return ioread32((u32 *)addr);
}

static inline void reg_write(u32 val, unsigned long addr)
{
	iowrite32(val, (u32 *)addr);
}

static int sc58x_pcie_read_conf(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	struct sc58x_pcie *host = to_host(bus->sysdata);
	unsigned long base = (unsigned long)host->base;
	unsigned long conf = (unsigned long)host->conf;
	unsigned long addr;
	u32 bdf;
#ifdef SC58X_EZKIT_EP
	static int quirk;
#endif

	bdf = (bus->number << 24) + (PCI_SLOT(devfn) << 19)
		+ (PCI_FUNC(devfn) << 16);
	addr = conf + (where & ~0x3);

	if (!bus->number && PCI_SLOT(devfn)) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	reg_write(0x1, base + IATU_VIEWPORT);
	reg_write(bdf, base + IATU_LWR_TAR_ADDR);
	*val = reg_read(addr);
#ifdef SC58X_EZKIT_EP
	if (!quirk) {
		if (((where & ~0x3) == PCI_VENDOR_ID) && (*val == 0xec301204))
			quirk = 1;
	} else {
		if (((where & ~0x3) == PCI_BASE_ADDRESS_0)
				&& ((*val & ~0xf)) == 0xf0000000)
			*val |= 0xff00000;
	}
#endif
	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xff;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xffff;
	else if (size != 4)
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static int sc58x_pcie_write_conf(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	struct sc58x_pcie *host = to_host(bus->sysdata);
	unsigned long base = (unsigned long)host->base;
	unsigned long conf = (unsigned long)host->conf;
	u32 bdf, mask, tmp;
	unsigned long addr;

	bdf = (bus->number << 24) + (PCI_SLOT(devfn) << 19)
		+ (PCI_FUNC(devfn) << 16);
	addr = conf + (where & ~0x3);

	if (!bus->number && PCI_SLOT(devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;
	reg_write(0x1, base + IATU_VIEWPORT);
	reg_write(bdf, base + IATU_LWR_TAR_ADDR);
	if (size == 4) {
		reg_write(val, addr);
		return PCIBIOS_SUCCESSFUL;
	}

	if (size == 1)
		mask = ~(0xff << ((where & 0x3) * 8));
	else if (size == 2)
		mask = ~(0xffff << ((where & 0x3) * 8));
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	tmp = reg_read(addr) & mask;
	tmp |= val << ((where & 0x3) * 8);
	reg_write(tmp, addr);

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops sc58x_pcie_ops = {
	.read	= sc58x_pcie_read_conf,
	.write	= sc58x_pcie_write_conf,
};

static int sc58x_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct sc58x_pcie *host = to_host(sys);

	pci_add_resource_offset(&sys->resources, &host->mem, sys->mem_offset);
	pci_add_resource_offset(&sys->resources, &host->prefetch,
				sys->mem_offset);
	pci_add_resource(&sys->resources, &host->busn);

	pci_ioremap_io(nr * SZ_64K, host->io.start);

	return 1;
}

static int sc58x_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct sc58x_pcie *host = to_host(dev->bus->sysdata);
	int irq;

	irq = of_irq_parse_and_map_pci(dev, slot, pin);
	if (!irq)
		irq = host->irq;

	return irq;
}

static irqreturn_t sc58x_pcie_isr(int irq, void *arg)
{
	u32 state;
	struct sc58x_pcie *host = (struct sc58x_pcie *)arg;
	unsigned long base = (unsigned long)host->base;
	struct sc58x_msi *msi = &host->msi;
	int i, processed = 0;

	state = reg_read(base + APP_INTSTAT);
	if (state & APP_INT_LEGACY)
		return IRQ_NONE;

	if (state & APP_INT_MSI) {
		for (i = 0; i < 8; i++) {
			unsigned long stat = reg_read(base + MSI_ISTAT0
					+ i * 12);
			while (stat) {
				unsigned int offset = find_first_bit(&stat, 32);
				unsigned int index = i * 32 + offset;
				unsigned int irq;

				pr_debug("receive msi %d\n", index);

				reg_write(1 << offset, base + MSI_ISTAT0
						+ i * 12);
				irq = irq_find_mapping(msi->domain, index);
				if (irq) {
					if (test_bit(index, msi->used))
						generic_handle_irq(irq);
					else
						dev_info(host->dev,
							"unhandled MSI\n");
				} else {
					dev_info(host->dev, "unexpected MSI\n");
				}
				stat = reg_read(base + MSI_ISTAT0 + i * 12);
				processed++;
			}
		}
		return processed > 0 ? IRQ_HANDLED : IRQ_NONE;
	}
	reg_write(state, base + APP_INTSTAT);
	return IRQ_HANDLED;
}

static inline void print_link_status(struct sc58x_pcie *host)
{
	const char *speed[] = {
		"2.5 GT/s",
		"5.0 GT/s",
		"8.0 GT/s",
		"Unknown Speed",
	};
	const char *width[] = {
		"Width x1",
		"Width x2",
		"Width x4",
		"Width x8",
		"Width x12",
		"Width x16",
		"Unknown Width",
	};

	int spd, lw;
	unsigned long base = (unsigned long)host->base;
	u32 val;

	val = reg_read(base + LINK_CTRL_STATUS_REG);
	spd = (val & LNK_CTLSTAT_SP) >> 16;
	switch (spd) {
	case 1:
		spd = 0;
		break;
	case 2:
		spd = 1;
		break;
	case 3:
		spd = 2;
		break;
	default:
		spd = 3;
	}
	lw = (val & LNK_CTLSTAT_NEG_LW) >> 20;
	switch (lw) {
	case 1:
		lw = 0;
		break;
	case 2:
		lw = 1;
		break;
	case 4:
		lw = 2;
		break;
	case 8:
		lw = 3;
		break;
	case 12:
		lw = 4;
		break;
	case 16:
		lw = 5;
		break;
	default:
		lw = 6;
	}

	dev_info(host->dev, "%s : %s\n", speed[spd], width[lw]);
}

static void sc58x_pcie_init(struct sc58x_pcie *host)
{
	unsigned long base = (unsigned long)host->base;
	u32 val;

	reg_write(0x1, base + MISC_CTRL1);
	reg_write(0x7, base + PCI_COMMAND);
	reg_write(0x10020, base + PORT_LINK_CTRL);
	val = reg_read(base + DEV_CAP_REG);
	reg_write((val & 0xfffffff8) | 0x1, base + DEV_CAP_REG);
	val = reg_read(base + DEV_CTRL_STATUS_REG);
	reg_write((val & 0xffffff1f) | 0x20, base + DEV_CTRL_STATUS_REG);
	val = reg_read(base + ADV_ERR_CAP_CTRL);
	reg_write(val | 0x40, base + ADV_ERR_CAP_CTRL);
	val= reg_read(base + LINK_CTRL_STATUS_REG2);
	reg_write((val & 0xfffffff0) | 0x1, base + LINK_CTRL_STATUS_REG2);
	reg_write(0x0, base + MISC_CTRL1);
}

static int sc58x_pcie_enable_controller(struct sc58x_pcie *host)
{
	unsigned long base = (unsigned long)host->base;
	u32 value;
	int i;

	set_spu_securep_msec(151, true);

	reg_write(0x2, base + RSCKPHY_ENG);
	reg_write(0x13, base + RSCKPHY_CTL);
	reg_write(0x10, base + RSCKPHY_CTL);
	while (reg_read(base + RSCKPHY_STAT) & 0x3)
		mdelay(100);

	sc58x_pcie_init(host);
	/* outbound address translation */
	reg_write(0x0, base + IATU_VIEWPORT);
	reg_write(host->mem.start, base + IATU_LWR_BASE_ADDR);
	reg_write(0x0, base + IATU_UPPER_BASE_ADDR);
	reg_write(host->mem.end, base + IATU_LIMIT_ADDR);
	reg_write(host->mem.start, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x0, base + IATU_REGION_CTRL1);
	reg_write(0x80000000, base + IATU_REGION_CTRL2);

	reg_write(0x1, base + IATU_VIEWPORT);
	reg_write(host->cfg.start, base + IATU_LWR_BASE_ADDR);
	reg_write(0x0, base + IATU_UPPER_BASE_ADDR);
	reg_write(host->cfg.end, base + IATU_LIMIT_ADDR);
	reg_write(host->cfg.start, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x4, base + IATU_REGION_CTRL1);
	reg_write(0x80000000, base + IATU_REGION_CTRL2);

	reg_write(0x2, base + IATU_VIEWPORT);
	reg_write(host->prefetch.start, base + IATU_LWR_BASE_ADDR);
	reg_write(0x0, base + IATU_UPPER_BASE_ADDR);
	reg_write(host->prefetch.end, base + IATU_LIMIT_ADDR);
	reg_write(host->prefetch.start, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x0, base + IATU_REGION_CTRL1);
	reg_write(0x80000000, base + IATU_REGION_CTRL2);

	/* inbound address translation */
	reg_write(0x80000000, base + IATU_VIEWPORT);
	reg_write(0x80000000, base + IATU_LWR_BASE_ADDR);
	reg_write(0x0, base + IATU_UPPER_BASE_ADDR);
	reg_write(0xbfffffff, base + IATU_LIMIT_ADDR);
	reg_write(0x80000000, base + IATU_LWR_TAR_ADDR);
	reg_write(0x0, base + IATU_UPPER_TAR_ADDR);
	reg_write(0x0, base + IATU_REGION_CTRL1);
	reg_write(0x80000000, base + IATU_REGION_CTRL2);

	/* Enable link training */
	reg_write(APP_CTRL_LTSSMEN, base + APP_CTRL);
	for (i = 0; i < 5000; i++) {
		if (reg_read(base + APP_STAT) & APP_STAT_LNKUP)
			break;
		mdelay(1);
	}

	if (i == 5000) {
		pr_info("APP_STAT=%x,RSCKPHY_STAT=%x,LINK_CTRL_STATUS_REG2=%x\n",
				reg_read(base + APP_STAT),
				reg_read(base + RSCKPHY_STAT),
				reg_read(base + LINK_CTRL_STATUS_REG2));
		dev_err(host->dev, "Link training failed\n");
		dev_err(host->dev, "PORT_LINK_DEBUG0 = %x, PORT_LINK_DEBUG1 = %x\n",
			reg_read(base + PORT_LINK_DEBUG0),
			reg_read(base + PORT_LINK_DEBUG1));

		return -ETIMEDOUT;
	}

	print_link_status(host);
	/* Enable legacy interrupt */
	reg_write(APP_CTRL_UNLCKINTEN | APP_CTRL_VENINTEN
			| APP_CTRL_LTSSMEN, base + APP_CTRL);

	return 0;
}

static const struct of_device_id sc58x_pcie_of_match[] = {
	{
		.compatible = "sc58x,pcie",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sc58x_pcie_of_match);

static int sc58x_pcie_parse_dt(struct sc58x_pcie *host)
{
	struct device_node *np = host->dev->of_node;
	struct platform_device *pdev = to_platform_device(host->dev);
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	struct resource res, *mem;
	int ret;

	ret = of_pci_range_parser_init(&parser, np);
	if (ret)
		return ret;

	for_each_of_pci_range(&parser, &range) {
		of_pci_range_to_resource(&range, np, &res);
		switch (res.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			memcpy(&host->io, &res, sizeof(res));
			host->io.name = "I/O";
			break;
		case IORESOURCE_MEM:
			if (res.flags & IORESOURCE_PREFETCH) {
				memcpy(&host->prefetch, &res, sizeof(res));
				host->prefetch.name = "PREFETCH";
			} else {
				memcpy(&host->mem, &res, sizeof(res));
				host->mem.name = "MEM";
			}
			break;
		default:
			memcpy(&host->cfg, &res, sizeof(res));
			host->cfg.name = "CONFIG";
			break;
		}
	}
	ret = of_pci_parse_bus_range(np, &host->busn);
	if (ret)
		return ret;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	host->base = devm_ioremap_resource(host->dev, mem);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		return ret;
	}
	host->conf = devm_ioremap_nocache(host->dev, host->cfg.start,
			resource_size(&host->cfg));
	if (!host->conf)
		return -ENOMEM;

	/* request interrupt */
	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(&pdev->dev, "failed to get IRQ: %d\n", ret);
		return ret;
	}

	host->irq = ret;

	ret = devm_request_irq(&pdev->dev, host->irq, sc58x_pcie_isr,
				IRQF_SHARED, "PCIE", host);
	if (ret) {
		dev_err(&pdev->dev, "failed to register IRQ: %d\n", ret);
		return ret;
	}

	return 0;
}

static int sc58x_msi_alloc(struct sc58x_msi *chip)
{
	int msi;

	mutex_lock(&chip->lock);

	msi = find_first_zero_bit(chip->used, INT_PCI_MSI_NR);
	if (msi < INT_PCI_MSI_NR)
		set_bit(msi, chip->used);
	else
		msi = -ENOSPC;

	mutex_unlock(&chip->lock);

	return msi;
}

static void sc58x_msi_free(struct sc58x_msi *chip, unsigned long irq)
{
	struct device *dev = chip->chip.dev;

	mutex_lock(&chip->lock);

	if (!test_bit(irq, chip->used))
		dev_err(dev, "trying to free unused MSI#%lu\n", irq);
	else
		clear_bit(irq, chip->used);

	mutex_unlock(&chip->lock);
}

static int sc58x_msi_setup_irq(struct msi_controller *chip,
			       struct pci_dev *pdev, struct msi_desc *desc)
{
	struct sc58x_msi *msi = to_sc58x_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;

	hwirq = sc58x_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq) {
		sc58x_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	msg.address_lo = virt_to_phys((void *)msi->pages);
	/* 32 bit address only */
	msg.address_hi = 0;
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static void sc58x_msi_teardown_irq(struct msi_controller *chip,
				   unsigned int irq)
{
	struct sc58x_msi *msi = to_sc58x_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	irq_dispose_mapping(irq);
	sc58x_msi_free(msi, hwirq);
}

static struct irq_chip sc58x_msi_irq_chip = {
	.name = "SC58x PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static int sc58x_msi_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &sc58x_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);
	return 0;
}

static const struct irq_domain_ops sc58x_domain_ops = {
	.map = sc58x_msi_map,
};

static int sc58x_pcie_enable_msi(struct sc58x_pcie *host)
{
	struct platform_device *pdev = to_platform_device(host->dev);
	struct sc58x_msi *msi = &host->msi;
	unsigned long msi_base, reg_base = (unsigned long)host->base;

	mutex_init(&msi->lock);

	msi->chip.dev = host->dev;
	msi->chip.setup_irq = sc58x_msi_setup_irq;
	msi->chip.teardown_irq = sc58x_msi_teardown_irq;

	msi->domain = irq_domain_add_linear(host->dev->of_node, INT_PCI_MSI_NR,
					    &sc58x_domain_ops, &msi->chip);
	if (!msi->domain) {
		dev_err(&pdev->dev, "failed to create IRQ domain\n");
		return -ENOMEM;
	}

	msi->pages = __get_free_pages(GFP_KERNEL, 0);
	msi_base = virt_to_phys((void *)msi->pages);

	reg_write(msi_base, reg_base + MSI_CTL_LADDR);
	reg_write(0, reg_base + MSI_CTL_UADDR);

	/* enable all MSI vectors */
	reg_write(0xffffffff, reg_base + MSI_IEN0);
	reg_write(0xffffffff, reg_base + MSI_IEN1);
	reg_write(0xffffffff, reg_base + MSI_IEN2);
	reg_write(0xffffffff, reg_base + MSI_IEN3);
	reg_write(0xffffffff, reg_base + MSI_IEN4);
	reg_write(0xffffffff, reg_base + MSI_IEN5);
	reg_write(0xffffffff, reg_base + MSI_IEN6);
	reg_write(0xffffffff, reg_base + MSI_IEN7);

	/* and unmask the MSI interrupt */
	reg_write(0x0, reg_base + MSI_IMSK0);
	reg_write(0x0, reg_base + MSI_IMSK1);
	reg_write(0x0, reg_base + MSI_IMSK2);
	reg_write(0x0, reg_base + MSI_IMSK3);
	reg_write(0x0, reg_base + MSI_IMSK4);
	reg_write(0x0, reg_base + MSI_IMSK5);
	reg_write(0x0, reg_base + MSI_IMSK6);
	reg_write(0x0, reg_base + MSI_IMSK7);

	return 0;
}

static int sc58x_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sc58x_pcie *host;
	struct hw_pci hw;
	int ret;

	host = devm_kzalloc(dev, sizeof(*host), GFP_KERNEL);
	if (!host)
		return -ENOMEM;
	host->dev = dev;

	ret = sc58x_pcie_parse_dt(host);
	if (ret < 0)
		return ret;
	ret = sc58x_pcie_enable_controller(host);
	if (ret < 0)
		return ret;

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = sc58x_pcie_enable_msi(host);
		if (ret < 0) {
			dev_err(&pdev->dev,
					"failed to enable MSI support\n");
			return ret;
		}
	}

	memset(&hw, 0, sizeof(hw));
#ifdef CONFIG_PCI_MSI
	hw.msi_ctrl = &host->msi.chip;
#endif
	hw.nr_controllers = 1;
	hw.private_data = (void **)&host;
	hw.setup = sc58x_pcie_setup;
	hw.map_irq = sc58x_pcie_map_irq;
	hw.ops = &sc58x_pcie_ops;
	pci_common_init_dev(dev, &hw);

	return 0;
}

static struct platform_driver sc58x_pcie_driver = {
	.driver	= {
		.name	= "sc58x-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = sc58x_pcie_of_match,
	},
};

module_platform_driver_probe(sc58x_pcie_driver, sc58x_pcie_probe);

MODULE_DESCRIPTION("Analog Devices PCIE controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
