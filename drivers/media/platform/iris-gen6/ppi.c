/*
 * ppi.c Analog Devices Parallel Peripheral Interface driver
 *
 * Copyright (c) 2011 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/portmux.h>
#include <mach/dma.h>
#else
#include <asm/bfin_ppi.h>
#include <asm/blackfin.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#endif

#include <asm/cacheflush.h>
#include <mach/cpu.h>
#include <media/adi/ppi.h>

static int ppi_start(struct ppi_if *ppi);
static int ppi_stop(struct ppi_if *ppi);
static int ppi_set_params(struct ppi_if *ppi, struct ppi_params *params);

static const struct ppi_ops ppi_ops = {
	.start = ppi_start,
	.stop = ppi_stop,
	.set_params = ppi_set_params,
};

static irqreturn_t ppi_irq_err(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	const struct ppi_info *info = ppi->info;

	switch (info->type) {
	case PPI_TYPE_PPI:
	{
		struct adi_ppi_regs *reg = info->base;
		unsigned short status;

		/* register on bf561 is cleared when read 
		 * others are W1C
		 */
		status = readw(&reg->status);
		if (status & 0x3000) {
			ppi->err = true;
			trace_printk("PPI error 0x%04x\n", status);
		}
		writew(0xff00, &reg->status);
		break;
	}
	case PPI_TYPE_EPPI:
	{
		struct adi_eppi_regs *reg = info->base;
		unsigned short status;

		status = readw(&reg->status);
		if (status & 0x2) {
			ppi->err = true;
			trace_printk("EPPI error 0x%04x\n", status);
		}
		writew(0xffff, &reg->status);
		break;
	}
	case PPI_TYPE_EPPI3:
	{
		struct adi_eppi3_regs *reg = info->base;
		unsigned long stat;

		stat = readl(&reg->stat);
		if (stat & 0x02) {
			ppi->err = true;
			trace_printk("EPPI3 error 0x%04lx\n", stat);
		}
		writel(0x40ff, &reg->stat);
		break;
	}
	default:
		break;
	}

	return IRQ_HANDLED;
}

static int ppi_start(struct ppi_if *ppi)
{
	const struct ppi_info *info = ppi->info;

	int ret;
	if (ppi->err_int) {
		ret = request_irq(info->irq_err, ppi_irq_err, 0, "PPI ERROR", ppi);
		if (ret) {
			pr_err("Unable to allocate IRQ for PPI\n");
			return ret;
		}
	}

	/* enable PPI */
	ppi->ppi_control |= PORT_EN;
	switch (info->type) {
	case PPI_TYPE_PPI:
	{
		struct adi_ppi_regs *reg = info->base;
		writew(ppi->ppi_control, &reg->control);
		break;
	}
	case PPI_TYPE_EPPI:
	{
		struct adi_eppi_regs *reg = info->base;
		writel(ppi->ppi_control, &reg->control);
		break;
	}
	case PPI_TYPE_EPPI3:
	{
		struct adi_eppi3_regs *reg = info->base;
		writel(ppi->ppi_control, &reg->ctl);
		break;
	}

	default:
		return -EINVAL;
	}

	return 0;
}

static int ppi_stop(struct ppi_if *ppi)
{
	const struct ppi_info *info = ppi->info;

	/* disable PPI */
	ppi->ppi_control &= ~PORT_EN;
	switch (info->type) {
	case PPI_TYPE_PPI:
	{
		struct adi_ppi_regs *reg = info->base;
		writew(ppi->ppi_control, &reg->control);
		break;
	}
	case PPI_TYPE_EPPI:
	{
		struct adi_eppi_regs *reg = info->base;
		writel(ppi->ppi_control, &reg->control);
		break;
	}
	case PPI_TYPE_EPPI3:
	{
		struct adi_eppi3_regs *reg = info->base;
		writel(ppi->ppi_control, &reg->ctl);
		break;
	}

	default:
		return -EINVAL;
	}

	if (ppi->err_int)
		free_irq(info->irq_err, ppi);

	return 0;
}

static int ppi_set_params(struct ppi_if *ppi, struct ppi_params *params)
{
	const struct ppi_info *info = ppi->info;
	int hcount, hdelay, samples_per_line;
	int chosenPinState;

#ifdef CONFIG_PINCTRL
	struct container {
		int bitCnt;
		const char* name;
	};
	static const struct container pin_state[] = {
			{8, "8bit"},
			{10, "10bit"},
			{12, "12bit"},
			{16, "16bit"},
			{24, "24bit"}
	};
	struct pinctrl *pctrl;
	struct pinctrl_state *pstate;

	for (chosenPinState = 0; chosenPinState < ARRAY_SIZE(pin_state); ++chosenPinState) {
		if (pin_state[chosenPinState].bitCnt == params->dlen) {
			break;
		}
	}

	if (chosenPinState == ARRAY_SIZE(pin_state)) {
		printk("not supported bus config requested\n");
		return -EINVAL;
	}
	if (params->dlen > 24 || params->dlen <= 0)
		return -EINVAL;
	pctrl = devm_pinctrl_get(ppi->dev);


	pstate = pinctrl_lookup_state(pctrl, pin_state[chosenPinState].name);

	if (pinctrl_select_state(pctrl, pstate))
		return -EINVAL;
#endif
	/* convert parameters unit from pixels to samples */
	hcount = params->width * (params->bpp / params->dlen);
	hdelay = params->hdelay * (params->bpp / params->dlen);
	samples_per_line = params->line * (params->bpp / params->dlen);
	if (params->int_mask == 0xFFFFFFFF)
		ppi->err_int = false;
	else
		ppi->err_int = true;

	ppi->ppi_control = params->ppi_control & ~PORT_EN;
	switch (info->type) {
	case PPI_TYPE_PPI:
	{
		struct adi_ppi_regs *reg = info->base;

		writew(ppi->ppi_control, &reg->control);
		writew(samples_per_line - 1, &reg->count);
		writew(params->frame, &reg->frame);
		break;
	}
	case PPI_TYPE_EPPI:
	{
		struct adi_eppi_regs *reg = info->base;

		writel(ppi->ppi_control, &reg->control);
		writew(samples_per_line, &reg->line);
		writew(params->frame, &reg->frame);
		writew(hdelay, &reg->hdelay);
		writew(params->vdelay, &reg->vdelay);
		writew(hcount, &reg->hcount);
		writew(params->height, &reg->vcount);
		break;
	}
	case PPI_TYPE_EPPI3:
	{
		struct adi_eppi3_regs *reg = info->base;

		writel(ppi->ppi_control, &reg->ctl);
		writel(samples_per_line, &reg->line);
		writel(params->frame, &reg->frame);
		writel(hdelay, &reg->hdly);
		writel(params->vdelay, &reg->vdly);
		writel(hcount, &reg->hcnt);
		writel(params->height, &reg->vcnt);
		if (params->int_mask)
			writel(params->int_mask & 0xFF, &reg->imsk);
		if (ppi->ppi_control & PORT_DIR) {
			if (ppi->ppi_control & BLANKGEN) {
				u32 blank_sample;

				blank_sample = samples_per_line - hcount - 8;
				writel(blank_sample, &reg->fs1_wlhb);
				writel(hcount, &reg->fs1_paspl);
				writel(params->blank_lines, &reg->fs2_wlvb);
				writel(params->active_lines, &reg->fs2_palpf);
			} else {
				u32 hsync_width, vsync_width, vsync_period;

				hsync_width = params->hsync
						* params->bpp / params->dlen;
				vsync_width = params->vsync * samples_per_line;
				vsync_period = samples_per_line * params->frame;
				writel(hsync_width, &reg->fs1_wlhb);
				writel(samples_per_line, &reg->fs1_paspl);
				writel(vsync_width, &reg->fs2_wlvb);
				writel(vsync_period, &reg->fs2_palpf);
			}
		}
/*
		printk("EPPI3\n"
				"\tstat: %08X\n"
				"\thcnt: %08X\n"
				"\tu32 hdly: %08X\n"
				"\tvcnt: %08X\n"
				"\tvdly: %08X\n"
				"\tframe: %08X\n"
				"\tline: %08X\n"
				"\tclkdiv: %08X\n"
				"\tctl: %08X\n"
				"\tfs1_wlhb: %08X\n"
				"\tfs1_paspl: %08X\n"
				"\tfs2_wlvb: %08X\n"
				"\tfs2_palpf: %08X\n"
				"\timsk: %08X\n"
				"\toddclip: %08X\n"
				"\tevenclip: %08X\n"
				"\tfs1_dly: %08X\n"
				"\tfs2_dly: %08X\n"
				"\tctl2: %08X\n"
				, reg->stat
				, reg->hcnt
				, reg->hdly
				, reg->vcnt
				, reg->vdly
				, reg->frame
				, reg->line
				, reg->clkdiv
				, reg->ctl
				, reg->fs1_wlhb
				, reg->fs1_paspl
				, reg->fs2_wlvb
				, reg->fs2_palpf
				, reg->imsk
				, reg->oddclip
				, reg->evenclip
				, reg->fs1_dly
				, reg->fs2_dly
				, reg->ctl2);
*/
		break;
	}
	default:
		return -EINVAL;
	}

	return 0;
}

struct ppi_if *ppi_create_instance(struct platform_device *pdev,
			const struct ppi_info *info)
{
	struct ppi_if *ppi;
	if (!info)
		return NULL;
#ifndef CONFIG_PINCTRL
	if (!info->pin_req || peripheral_request_list(info->pin_req, KBUILD_MODNAME)) {
		dev_err(&pdev->dev, "request peripheral failed\n");
		return NULL;
	}
#endif

	ppi = kzalloc(sizeof(*ppi), GFP_KERNEL);
	if (!ppi) {
		peripheral_free_list(info->pin_req);
		dev_err(&pdev->dev, "unable to allocate memory for ppi handle\n");
		return NULL;
	}
#if defined(CONFIG_ARCH_SC58X) || defined(CONFIG_ARCH_SC57X)
	set_spu_securep_msec(info->spu, true);
#endif
	ppi->ops = &ppi_ops;
	ppi->info = info;
	ppi->dev = &pdev->dev;

	pr_info("ppi probe success\n");
	return ppi;
}
EXPORT_SYMBOL(ppi_create_instance);

void ppi_delete_instance(struct ppi_if *ppi)
{
	peripheral_free_list(ppi->info->pin_req);
	kfree(ppi);
}
EXPORT_SYMBOL(ppi_delete_instance);

MODULE_DESCRIPTION("Analog Devices PPI driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
