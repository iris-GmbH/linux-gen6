/*
 * Rotary counter driver for Analog Devices Blackfin Processors
 *
 * Copyright 2008-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/platform_data/bfin_rotary.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/portmux.h>
#else
#include <asm/portmux.h>
#endif

#define CNT_CONFIG_OFF		0	/* CNT Config Offset */
#define CNT_IMASK_OFF		4	/* CNT Interrupt Mask Offset */
#define CNT_STATUS_OFF		8	/* CNT Status Offset */
#define CNT_COMMAND_OFF		12	/* CNT Command Offset */
#define CNT_DEBOUNCE_OFF	16	/* CNT Debounce Offset */
#define CNT_COUNTER_OFF		20	/* CNT Counter Offset */
#define CNT_MAX_OFF		24	/* CNT Maximum Count Offset */
#define CNT_MIN_OFF		28	/* CNT Minimum Count Offset */

struct bfin_rot {
	struct input_dev *input;
	void __iomem *base;
	int irq;
	unsigned int up_key;
	unsigned int down_key;
	unsigned int button_key;
	unsigned int rel_code;

	unsigned short mode;
	unsigned short debounce;

	unsigned short cnt_config;
	unsigned short cnt_imask;
	unsigned short cnt_debounce;
};

static void report_key_event(struct input_dev *input, int keycode)
{
	/* simulate a press-n-release */
	input_report_key(input, keycode, 1);
	input_sync(input);
	input_report_key(input, keycode, 0);
	input_sync(input);
}

static void report_rotary_event(struct bfin_rot *rotary, int delta)
{
	struct input_dev *input = rotary->input;

	if (rotary->up_key) {
		report_key_event(input,
				 delta > 0 ? rotary->up_key : rotary->down_key);
	} else {
		input_report_rel(input, rotary->rel_code, delta);
		input_sync(input);
	}
}

static irqreturn_t bfin_rotary_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct bfin_rot *rotary = platform_get_drvdata(pdev);
	int delta;

	switch (readw(rotary->base + CNT_STATUS_OFF)) {

	case ICII:
		break;

	case UCII:
	case DCII:
		delta = readl(rotary->base + CNT_COUNTER_OFF);
		if (delta)
			report_rotary_event(rotary, delta);
		break;

	case CZMII:
		report_key_event(rotary->input, rotary->button_key);
		break;

	default:
		break;
	}

	writew(W1LCNT_ZERO, rotary->base + CNT_COMMAND_OFF); /* Clear COUNTER */
	writew(-1, rotary->base + CNT_STATUS_OFF); /* Clear STATUS */

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static const struct of_device_id bfin_rotary_of_match[] = {
	{ .compatible = "adi,rotary", },
	{},
};
MODULE_DEVICE_TABLE(of, bfin_rotary_of_match);
#endif

static int bfin_rotary_open(struct input_dev *input)
{
	struct bfin_rot *rotary = input_get_drvdata(input);
	unsigned short val;

	if (rotary->mode & ROT_DEBE)
		writew(rotary->debounce & DPRESCALE,
			rotary->base + CNT_DEBOUNCE_OFF);

	writew(rotary->mode & ~CNTE, rotary->base + CNT_CONFIG_OFF);

	val = UCIE | DCIE;
	if (rotary->button_key)
		val |= CZMIE;
	writew(val, rotary->base + CNT_IMASK_OFF);

	writew(rotary->mode | CNTE, rotary->base + CNT_CONFIG_OFF);

	return 0;
}

static void bfin_rotary_close(struct input_dev *input)
{
	struct bfin_rot *rotary = input_get_drvdata(input);

	writew(0, rotary->base + CNT_CONFIG_OFF);
	writew(0, rotary->base + CNT_IMASK_OFF);
}

static void bfin_rotary_free_action(void *data)
{
	unsigned short *pin_list = (unsigned short *)data;
	peripheral_free_list(pin_list);
}

static int bfin_rotary_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bfin_rotary_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct bfin_rot *rotary;
	struct resource *res;
	struct device_node *node = pdev->dev.of_node;
	struct input_dev *input;
	int error;

	rotary = devm_kzalloc(dev, sizeof(struct bfin_rot), GFP_KERNEL);
	if (!rotary) {
		dev_err(dev, "fail to malloc bfin_rot\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rotary->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(rotary->base))
		return PTR_ERR(rotary->base);

	if (node) {
		u16 val;

		pdata = devm_kzalloc(dev,
				sizeof(struct bfin_rotary_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(dev, "fail to malloc platform data\n");
			return -ENOMEM;
		}

		of_property_read_u32(node, "rotary_up_key",
			&pdata->rotary_up_key);
		of_property_read_u32(node, "rotary_down_key",
			&pdata->rotary_down_key);
		of_property_read_u32(node, "rotary_rel_code",
			&pdata->rotary_rel_code);
		if (of_property_read_u32(node, "rotary_button_key",
			&pdata->rotary_button_key))
			return -ENOENT;
		of_property_read_u16(node, "debounce", &pdata->debounce);
		of_property_read_u16(node, "debounce_en", &val);
		pdata->mode |= (val & 1) << 1;
		of_property_read_u16(node, "cnt_mode", &val);
		pdata->mode |= (val & 7) << CNTMODE_SHIFT;
		of_property_read_u16(node, "boundary_mode", &val);
		pdata->mode |= (val & 3) << BNDMODE_SHIFT;
		of_property_read_u16(node, "invert_czm", &val);
		pdata->mode |= (val & 1) << 6;
		of_property_read_u16(node, "invert_cud", &val);
		pdata->mode |= (val & 1) << 5;
		of_property_read_u16(node, "invert_cdg", &val);
		pdata->mode |= (val & 1) << 4;
	}

	/* Basic validation */
	if ((pdata->rotary_up_key && !pdata->rotary_down_key) ||
	    (!pdata->rotary_up_key && pdata->rotary_down_key)) {
		return -EINVAL;
	}

	error = peripheral_request_list(pdata->pin_list, dev_name(&pdev->dev));
	if (error) {
		dev_err(dev, "requesting peripherals failed\n");
		return error;
	}

	devm_add_action(dev, bfin_rotary_free_action, pdata->pin_list);

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	rotary->input = input;

	rotary->up_key = pdata->rotary_up_key;
	rotary->down_key = pdata->rotary_down_key;
	rotary->button_key = pdata->rotary_button_key;
	rotary->rel_code = pdata->rotary_rel_code;

	rotary->mode = pdata->mode;
	rotary->debounce = pdata->debounce;

	input->name = pdev->name;
	input->phys = "bfin-rotary/input0";
	input->dev.parent = &pdev->dev;

	input_set_drvdata(input, rotary);

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input->open = bfin_rotary_open;
	input->close = bfin_rotary_close;

	if (rotary->up_key) {
		__set_bit(EV_KEY, input->evbit);
		__set_bit(rotary->up_key, input->keybit);
		__set_bit(rotary->down_key, input->keybit);
	} else {
		__set_bit(EV_REL, input->evbit);
		__set_bit(rotary->rel_code, input->relbit);
	}

	if (rotary->button_key) {
		__set_bit(EV_KEY, input->evbit);
		__set_bit(rotary->button_key, input->keybit);
	}

	/* Quiesce the device before requesting irq */
	bfin_rotary_close(input);

	rotary->irq = platform_get_irq(pdev, 0);
	if (rotary->irq < 0) {
		dev_err(dev, "No rotary IRQ specified\n");
		return -ENOENT;
	}

	error = devm_request_irq(dev, rotary->irq, bfin_rotary_isr,
				 0, dev_name(dev), pdev);
	if (error) {
		dev_err(dev, "unable to claim irq %d; error %d\n",
			rotary->irq, error);
		return error;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "unable to register input device (%d)\n", error);
		return error;
	}

	platform_set_drvdata(pdev, rotary);
	device_init_wakeup(&pdev->dev, pdata->pm_wakeup);

	return 0;
}

static int bfin_rotary_remove(struct platform_device *pdev)
{
	struct bfin_rot *rotary = platform_get_drvdata(pdev);

	writew(0, rotary->base + CNT_CONFIG_OFF);
	writew(0, rotary->base + CNT_IMASK_OFF);

	return 0;
}

#ifdef CONFIG_PM
static int __maybe_unused bfin_rotary_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bfin_rot *rotary = platform_get_drvdata(pdev);

	rotary->cnt_config = readw(rotary->base + CNT_CONFIG_OFF);
	rotary->cnt_imask = readw(rotary->base + CNT_IMASK_OFF);
	rotary->cnt_debounce = readw(rotary->base + CNT_DEBOUNCE_OFF);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(rotary->irq);

	return 0;
}

static int bfin_rotary_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bfin_rot *rotary = platform_get_drvdata(pdev);

	writew(rotary->cnt_debounce, rotary->base + CNT_DEBOUNCE_OFF);
	writew(rotary->cnt_imask, rotary->base + CNT_IMASK_OFF);
	writew(rotary->cnt_config & ~CNTE, rotary->base + CNT_CONFIG_OFF);

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(rotary->irq);

	if (rotary->cnt_config & CNTE)
		writew(rotary->cnt_config, rotary->base + CNT_CONFIG_OFF);

	return 0;
}

static const struct dev_pm_ops bfin_rotary_pm_ops = {
	.suspend	= bfin_rotary_suspend,
	.resume		= bfin_rotary_resume,
};
#endif

static struct platform_driver bfin_rotary_device_driver = {
	.probe		= bfin_rotary_probe,
	.driver		= {
		.name	= "bfin-rotary",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(bfin_rotary_of_match),
#ifdef CONFIG_PM
		.pm	= &bfin_rotary_pm_ops,
#endif
	},
};
module_platform_driver(bfin_rotary_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Rotary Counter driver for Blackfin Processors");
MODULE_ALIAS("platform:bfin-rotary");
