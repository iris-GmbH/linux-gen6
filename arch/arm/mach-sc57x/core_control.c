/*
 * ADSP-SC57x Core Control Driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/sc57x.h>
#include <asm/io.h>

#define CMD_CORE_START         _IO('b', 0)
#define CMD_CORE_STOP          _IO('b', 1)
#define CMD_SET_SVECT1         _IO('m', 17)
#define CMD_SET_SVECT2         _IO('m', 18)

#define VALID_CORE_MIN         1
#define VALID_CORE_MAX         2


static long adi_core_stop(unsigned int coreid)
{
	long ret = 0;
	if (coreid < VALID_CORE_MIN || coreid > VALID_CORE_MAX) {
		ret = -EINVAL;
	} else {
		if (readl(__io_address(REG_RCU0_CRCTL)) & 1 << coreid) {
			pr_info("corectl core %d is already unter reset\n", coreid);
			return 0;
		}
		/* clear CRSTAT bit for given coreid */
		writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
		/* disable the system interface */
		writel(readl(__io_address(REG_RCU0_SIDIS)) | 1 << (coreid - 1),
				__io_address(REG_RCU0_SIDIS));
		/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
		/* while (readl(REG_RCU0_SISTAT) & 1 << (coreid - 1) != 1); */
		udelay(50);
		/* put core in reset */
		writel(readl(__io_address(REG_RCU0_CRCTL)) | 1 << coreid,
				__io_address(REG_RCU0_CRCTL));
		/* reenable the system interface */
		writel(readl(__io_address(REG_RCU0_SIDIS)) & ~(1 << (coreid - 1)),
				__io_address(REG_RCU0_SIDIS));
		/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
		/* while (readl(REG_RCU0_SISTAT) & 1 << (coreid - 1) != 0); */
		udelay(50);
		/* clear CRSTAT bit for given coreid */
		writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
	}
	return ret;
}

static long adi_core_start(unsigned int coreid)
{
	long ret = 0;
	if (coreid < VALID_CORE_MIN || coreid > VALID_CORE_MAX) {
		ret = -EINVAL;
	} else {
		ret = adi_core_stop(coreid);
		if (ret != 0) {
			return ret;
		}
		/* move core out of reset */
		writel(readl(__io_address(REG_RCU0_CRCTL)) & ~(1 << coreid),
				__io_address(REG_RCU0_CRCTL));
		/* clear CRSTAT bit for given coreid */
		writel(1 << coreid, __io_address(REG_RCU0_CRSTAT));
		/* notify CCES */
		writel(1 << (18 + coreid), __io_address(REG_RCU0_MSG_SET));
	}
	return ret;
}


/*
 * set the address of the first instruction to execute for coreid
 */
static long core_set_svector(unsigned int coreid, unsigned long addr)
{
	if (addr && (1 == coreid)) {
		writel(addr, __io_address(REG_RCU0_SVECT1));
	} else if (addr && (2 == coreid)) {
		writel(addr, __io_address(REG_RCU0_SVECT2));
	} else {
		return -EINVAL;
	}
	return 0;
}

static long core_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case CMD_CORE_START:
		ret = adi_core_start(arg);
		break;
	case CMD_CORE_STOP:
		ret = adi_core_stop(arg);
		break;
	case CMD_SET_SVECT1:
		ret = core_set_svector(1, arg);
		break;
	case CMD_SET_SVECT2:
		ret = core_set_svector(2, arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct file_operations core_fops = {
		.owner          = THIS_MODULE,
		.unlocked_ioctl = core_ioctl,
		.llseek         = noop_llseek,
};

static struct miscdevice core_dev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = "corectrl",
		.fops  = &core_fops,
};

static int __init adi_core_init(void)
{
		return misc_register(&core_dev);
}

module_init(adi_core_init);

MODULE_DESCRIPTION("SC57x Core Control Support");
MODULE_LICENSE("GPL v2");
