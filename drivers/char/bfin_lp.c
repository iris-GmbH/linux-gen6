/*
 * Blackfin Linkport driver
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/scatterlist.h>
#include <asm/irq.h>
#include <mach/dma.h>
#include <mach/cpu.h>
#include <mach/irqs.h>
#include <mach/hardware.h>
#ifdef ARCH_SC58X
#include <mach/sc58x.h>
#elif defined(ARCH_SC57X)
#include <mach/sc57x.h>
#endif

/* fifo size in elements (ints) */
#define FIFO_SIZE      1024

#define LINKPORT_DRVNAME "bfin-linkport"

#define BFIN_LP_DMA_MODE

#define LP_CTL_EN 0x1
#define LP_CTL_TRAN 0x8
#define LP_CTL_TRQMSK  0x100
#define LP_CTL_RRQMSK  0x200
#define LP_CTL_ITMSK  0x800

#define LP_STAT_DONE 0x1000
#define LP_STAT_LTRQ 0x1
#define LP_STAT_LRRQ 0x2
#define LP_STAT_LPIT 0x8
#define LP_STAT_FFST 0x70
#define LP_STAT_LERR 0x80
#define LP_STAT_LPBS 0x100

#define LP_CTL_OFF 0x0
#define LP_STAT_OFF 0x4
#define LP_DIV_OFF 0x8
#define LP_CNT_OFF 0xC
#define LP_TX_OFF 0x10
#define LP_RX_OFF 0x14
#define LP_TX_SHADOW_OFF 0x18
#define LP_RX_SHADOW_OFF 0x1C

#define LP_DIV_DEFAULT 2

struct bfin_linkport {
	struct list_head lp_dev;
	struct class *class;
	int major;
	spinlock_t lp_dev_lock;
};

struct bfin_lp_dev {
	struct list_head list;
	struct device *device;
	phys_addr_t preg_base;
	void __iomem *reg_base;
	wait_queue_head_t rx_waitq;
	spinlock_t lock;
	struct workqueue_struct *workqueue;
	struct work_struct transfer_work;
	int linkport_num;
	int dma_chan;
	int status;
	int lp_div;
	int irq;
	int irq_disabled;
	int status_irq;
	int status_irq_disabled;
	int count;
	DECLARE_KFIFO_PTR(lpfifo, unsigned int);
	struct completion complete;
};

struct bfin_linkport *linkport_dev;

struct bfin_lp_dev lp_dev_info[2] = {
	{
		.preg_base = LP0_CTL,
		.irq = IRQ_LP0,
		.status_irq = IRQ_LP0_STAT,
		.dma_chan = CH_LP0,
	},
	{
		.preg_base = LP1_CTL,
		.irq = IRQ_LP1,
		.status_irq = IRQ_LP1_STAT,
		.dma_chan = CH_LP1,
	},
};


int bfin_lp_config_channel(struct bfin_lp_dev *lpdev, int direction)
{
	uint32_t reg;
	if (direction)
		reg = LP_CTL_TRAN | LP_CTL_TRQMSK;
	else
		reg =  LP_CTL_RRQMSK;

	writel(reg, lpdev->reg_base + LP_CTL_OFF);

	return 0;
}

void bfin_lp_enable(struct bfin_lp_dev *lpdev)
{
	uint32_t ctl;

	ctl = readl(lpdev->reg_base + LP_CTL_OFF);
	writel(ctl | LP_CTL_EN, lpdev->reg_base + LP_CTL_OFF);
}

int bfin_lp_get_rx_fifo(struct bfin_lp_dev *lpdev)
{
	uint32_t state = readl(lpdev->reg_base + LP_STAT_OFF);

	state = (state & LP_STAT_FFST) >> 4;

	if (state <= 4)
		return state;
	else
		return 0;
}

static void lp_rx_fifo(struct bfin_lp_dev *dev)
{
	int cnt;

	cnt = bfin_lp_get_rx_fifo(dev);
	while (cnt) {
		unsigned int data;
		data = readl(dev->reg_base + LP_RX_OFF);
		if (!kfifo_put(&dev->lpfifo, data))
			goto out;
		cnt--;
	}
out:
	enable_irq(dev->irq);
	dev->irq_disabled = 0;

	/* wake up read/write block. */
	wake_up_interruptible(&dev->rx_waitq);
}

static void transfer_fn(struct work_struct *work)
{
	struct bfin_lp_dev *dev = container_of(work,
			struct bfin_lp_dev, transfer_work);

	if (dev->status == LP_STAT_LTRQ) {
		unsigned int data = 0;
		while (kfifo_get(&dev->lpfifo, &data)) {
			while (readl(dev->reg_base + LP_STAT_OFF) & LP_STAT_LPBS);
			writel(data, dev->reg_base + LP_TX_OFF);
		}

		if (kfifo_len(&dev->lpfifo) == 0) {
			dev->status = LP_STAT_DONE;
			kfifo_reset(&dev->lpfifo);
			complete(&dev->complete);
		}
	} else if (dev->status == LP_STAT_DONE) {
		complete(&dev->complete);
	} else {
		while (readl(dev->reg_base + LP_STAT_OFF) & LP_STAT_LPBS);
		lp_rx_fifo(dev);
	}
}

static irqreturn_t bfin_lp_irq(int irq, void *dev_id)
{
	struct bfin_lp_dev *dev = (struct bfin_lp_dev *)dev_id;
	uint32_t stat = readl(dev->reg_base + LP_STAT_OFF);
	uint32_t ctl;

	pr_debug("bfin lp irq %d stat %x dev %p status %d\n", irq, stat, dev, dev->status);

	if (stat & LP_STAT_LTRQ) {
		bfin_lp_enable(dev);
		if (kfifo_len(&dev->lpfifo)) {
			dev->status = LP_STAT_LTRQ;
		} else
			goto out;
	}

	if (stat & LP_STAT_LRRQ) {
		dev->status = LP_STAT_LRRQ;
		goto out;
	}

	disable_irq_nosync(irq);
	if (irq == dev->irq)
		dev->irq_disabled = 1;
	else
		dev->status_irq_disabled = 1;
	queue_work(dev->workqueue, &dev->transfer_work);
out:
	writel(stat, dev->reg_base + LP_STAT_OFF);
	dev->count++;
	return IRQ_HANDLED;
}


static int bfin_lp_open(struct inode *inode, struct file *filp)
{
	unsigned long flags;
	struct bfin_lp_dev *dev;
	unsigned int index = iminor(inode);
	int ret = -EBUSY;

	dev = &lp_dev_info[index];

	spin_lock_irqsave(&dev->lock, flags);

	filp->private_data = dev;

	spin_unlock_irqrestore(&dev->lock, flags);


	writel(dev->lp_div, dev->reg_base + LP_DIV_OFF);
	writel(0xFF, dev->reg_base + LP_STAT_OFF);
	writel(0, dev->reg_base + LP_CTL_OFF);

	pr_debug("bfin lp open %d\n", index);
	return 0;

	return ret;
}

static int bfin_lp_release(struct inode *inode, struct file *filp)
{
	struct bfin_lp_dev *dev = filp->private_data;
	unsigned int index = iminor(inode);

	pr_debug("bfin lp relese %d\n", index);
	if (readl(dev->reg_base + LP_CTL_OFF) & LP_CTL_TRAN)
		wait_for_completion_interruptible(&dev->complete);

	writel(0xFF, dev->reg_base + LP_STAT_OFF);
	writel(0, dev->reg_base + LP_CTL_OFF);
	if (dev->irq_disabled) {
		enable_irq(dev->irq);
		dev->irq_disabled = 0;
	}
	if (dev->status_irq_disabled) {
		enable_irq(dev->status_irq);
		dev->status_irq_disabled = 0;
	}
	dev->status = 0;

	return 0;
}

static ssize_t bfin_lp_read(struct file *filp, char *buf, size_t count, loff_t *pos)
{
	struct bfin_lp_dev *dev = filp->private_data;
	int fifo_cnt = 0;
	unsigned int copied = 0;
	unsigned int n;
	uint32_t ctl;
	int ret;

	n = count / 4;
	count = 4 * n;
	bfin_lp_config_channel(dev, 0);
	bfin_lp_enable(dev);

	while (n) {
		fifo_cnt = kfifo_len(&dev->lpfifo);
		if (!fifo_cnt) {
			pr_debug("wait event\n");
			ret = wait_event_interruptible(dev->rx_waitq, kfifo_len(&dev->lpfifo) != 0);
			if (ret) {
				pr_debug("wake from signal\n");
				return 0;
			} else
				continue;
		}

		ret = kfifo_to_user(&dev->lpfifo, buf + copied, fifo_cnt * 4, &copied);
		n -= fifo_cnt;
	}

	return count;
}

static ssize_t bfin_lp_write(struct file *filp, const char *buf, size_t count, loff_t *pos)
{
	struct bfin_lp_dev *dev = filp->private_data;
	unsigned int copied;
	int ret;

	ret = kfifo_from_user(&dev->lpfifo, buf, count, &copied);

	writel(dev->lp_div, dev->reg_base + LP_DIV_OFF);

	bfin_lp_config_channel(dev, 1);

	return ret ? ret : copied;
}

static long bfin_lp_ioctl(struct file *filp, uint cmd, unsigned long arg)
{
	return 0;
}


static const struct file_operations linkport_fops = {
	.owner = THIS_MODULE,
	.read = bfin_lp_read,
	.write = bfin_lp_write,
	.unlocked_ioctl = bfin_lp_ioctl,
	.open = bfin_lp_open,
	.release = bfin_lp_release,
};

static ssize_t
linkport_status_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct bfin_lp_dev *dev;

	p += sprintf(p, "linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
	}
	return p - buf;
}

static ssize_t
linkport_reg_show(struct class *class, struct class_attribute *attr, char *buf)
{
	char *p = buf;
	struct bfin_lp_dev *dev;

	p += sprintf(p, "linkport status\n");
	list_for_each_entry(dev, &linkport_dev->lp_dev, list) {
		p += sprintf(p, "linkport num %d\n", dev->linkport_num);
		p += sprintf(p, "\t clt %d\n", readl(dev->reg_base + LP_CTL_OFF));
		p += sprintf(p, "\t stat %d\n", readl(dev->reg_base + LP_STAT_OFF));
	}
	return (p - buf);
}

static ssize_t
linkport_reg_store(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
	char *p;
	int rw = 0;
	uint32_t value = 0;
	char buffer[64];
	uint32_t res;
	unsigned long temp;
	char *endp;
	void __iomem *addr;

	if (copy_from_user(buffer, buf, count))
		return -EFAULT;

	buffer[count] = '\0';

	p = buffer;

	while (*p == ' ')
		p++;

	if (p[0] == 'r')
		rw = 0;
	else if (p[0] == 'w')
		rw = 1;
	else
		printk(KERN_DEBUG "-EINVAL\n");

	if (p[1] < '0' && p[1] > '9')
		printk(KERN_DEBUG "-EINVAL2\n");

	res = simple_strtoul(&p[1], &endp, 10);

	if (res == 8)
		p += 2;
	else if (res == 16)
		p += 3;
	else if (res == 32)
		p += 3;
	else
		printk(KERN_DEBUG "-EINVAL3\n");


	while (*p == ' ')
		p++;

	temp = simple_strtoul(p, &endp, 16);
	addr = (void __iomem *)temp;

	if (rw) {
		p = endp;
		while (*p == ' ')
			p++;

		value = simple_strtoul(p, NULL, 16);
		switch (res) {
		case 8:
			writeb((uint8_t)value, addr);
			value = readb(addr);
			break;
		case 16:
			writew((uint16_t)value, addr);
			value = readw(addr);
			break;
		case 32:
			writel((uint32_t)value, addr);
			value = readl(addr);
			break;
		}
		printk(KERN_DEBUG "write addr %p reg %08x\n", addr, value);
	} else {
		switch (res) {
		case 8:
			value = readb(addr);
			break;
		case 16:
			value = readw(addr);
			break;
		case 32:
			value = readl(addr);
			break;
		}
		printk(KERN_DEBUG "read addr %p reg %08x\n", addr, value);
	}
	return count;
}

static CLASS_ATTR(status, S_IRWXU, &linkport_status_show, NULL);
static CLASS_ATTR(reg, S_IRWXU, &linkport_reg_show, &linkport_reg_store);

/*
 * bfin_linkport_init - Initialize module
 *
 *
 */

static int __init bfin_linkport_init(void)
{
	struct device_node *np;
	int err;
	dev_t lp_dev;
	int i;
	int ret;
	struct bfin_lp_dev *lpdev;


	linkport_dev = kzalloc(sizeof(*linkport_dev), GFP_KERNEL);
	if (!linkport_dev) {
		return -ENOMEM;
	}

	linkport_dev->major = register_chrdev(0, "bfin-linkport", &linkport_fops);
	if (linkport_dev->major < 0) {
		err = linkport_dev->major;
		printk("Error %d registering chrdev for device\n", err);
		goto free;
	}

	lp_dev = MKDEV(linkport_dev->major, 0);

	linkport_dev->class = class_create(THIS_MODULE, "linkport");
	err = class_create_file(linkport_dev->class, &class_attr_status);
	if (err) {
		printk("Error %d registering class device\n", err);
		goto free_chrdev;
	}

	err = class_create_file(linkport_dev->class, &class_attr_reg);
	if (err) {
		printk("Error %d registering class device\n", err);
		class_remove_file(linkport_dev->class, &class_attr_status);
		goto free_chrdev;
	}

	INIT_LIST_HEAD(&linkport_dev->lp_dev);
	spin_lock_init(&linkport_dev->lp_dev_lock);

	for (i = 0; i < 2; i++) {
		struct device *dev;

		dev = device_create(linkport_dev->class, NULL, lp_dev + i, &lp_dev_info[i], "linkport%d", i);
		if (!dev)
			goto free_chrdev;

		np = of_find_compatible_node(NULL, NULL, dev_name(dev));
		if (np) {
			printk("find dt node %s\n", np->name);
			dev->of_node = of_node_get(np);
			ret = of_property_read_u32(dev->of_node, "clock-div",
						&lp_dev_info[i].lp_div);
			if (ret)
				lp_dev_info[i].lp_div = LP_DIV_DEFAULT;
			printk("linkport clock div: %d\n", lp_dev_info[i].lp_div);
			lp_dev_info[i].irq = irq_of_parse_and_map(dev->of_node, 0);
			if (lp_dev_info[i].irq <= 0)
				panic("Can't parse IRQ");
			lp_dev_info[i].status_irq = irq_of_parse_and_map(dev->of_node, 1);
			if (lp_dev_info[i].status_irq <= 0)
				panic("Can't parse IRQ");
			printk("of parse irq %d status irq %d\n", lp_dev_info[i].irq, lp_dev_info[i].status_irq);
			set_spu_securep_msec(5, true);
			set_spu_securep_msec(6, true);

		} else
			printk("not found dt node %s %s\n", __func__, dev_name(dev));

		lp_dev_info[i].reg_base = ioremap(lp_dev_info[i].preg_base, 0x20);
		lp_dev_info[i].device = dev;
		lp_dev_info[i].linkport_num = i;
		spin_lock_init(&lp_dev_info[i].lock);
		init_waitqueue_head(&lp_dev_info[i].rx_waitq);
		INIT_WORK(&lp_dev_info[i].transfer_work, transfer_fn);
		INIT_LIST_HEAD(&lp_dev_info[i].list);
		init_completion(&lp_dev_info[i].complete);
		list_add(&lp_dev_info[i].list, &linkport_dev->lp_dev);

		lpdev = &lp_dev_info[i];

		ret = kfifo_alloc(&lpdev->lpfifo, FIFO_SIZE, GFP_KERNEL);
		if (ret) {
			printk(KERN_ERR "error kfifo_alloc\n");
			err = ret;
		}

		if (request_irq(lpdev->irq, bfin_lp_irq, 0, LINKPORT_DRVNAME, lpdev)) {
			printk(KERN_ERR "Requesting irq %d failed\n", lpdev->irq);
			err = -ENODEV;
		}

		if (request_irq(lpdev->status_irq, bfin_lp_irq, 0, LINKPORT_DRVNAME, lpdev)) {
			printk(KERN_ERR "Requesting status irq  %d failed\n", lpdev->status_irq);
			ret = -ENODEV;
		}

		lpdev->workqueue = create_singlethread_workqueue("linkport_work");
		if (!lpdev->workqueue) {
			printk(KERN_ERR "create workqueue failed\n");
			err = -ENOMEM;
		}


		if (IS_ERR(devm_pinctrl_get_select_default(dev))) {
			printk("Requesting Peripheral for %s failed.\n", dev_name(dev));
			device_destroy(linkport_dev->class, lp_dev + i);
			err = -EINVAL;
		}

		if (err) {
			destroy_workqueue(lpdev->workqueue);
			kfifo_free(&lpdev->lpfifo);
			free_irq(lpdev->irq, dev);
			free_irq(lpdev->status_irq, dev);
		}
	}

	return 0;

free_chrdev:
	unregister_chrdev(linkport_dev->major, "bfin-linkport");
free:
	kfree(linkport_dev);

	return err;
}
module_init(bfin_linkport_init);
