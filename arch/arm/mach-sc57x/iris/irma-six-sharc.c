#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/irq.h>
#include <linux/irqflags.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <mach/icc.h>

#define DRV_NAME "sharc"

struct driverInfo {
	wait_queue_head_t read_waitqueue;
	wait_queue_head_t write_waitqueue;
	wait_queue_head_t poll_waitqueue;
	struct miscdevice misc_device;
	int irq;
	int irq_type;

	// Core # - ARM = 0, SharcCore1 = 1, SharcCore2 = 2
	u8 core_id;

	// a lock to synchronize the irqPending counter
	spinlock_t lock;

	// counter for the amount of IRQs that happened since the last call to read()
	int readPending;
	// a flag indicating that a write can happen
	int writePending;

	// counter for how often the device was opened
	atomic_t opened;

	// pointer to the region in memory where the exchange data resides
	unsigned char __iomem *membase;

	// how big is the size of the memory region with the exchange data
	resource_size_t memSize;
};

static int open(struct inode *inode, struct file *filp)
{
	/* filp->private_data is populated with misc_device, extract the parent struct aka our driverInfo struct */
	struct driverInfo *sharcDevice = container_of(
		filp->private_data, struct driverInfo, misc_device);
	/* make sure device is only opened once */
	if (0 == atomic_dec_and_test(&sharcDevice->opened)) {
		atomic_inc(&sharcDevice->opened);
		return -EBUSY;
	}
	/* put our struct back */
	filp->private_data = sharcDevice;
	sharcDevice->readPending = 0;
	sharcDevice->writePending = 1;
	return 0;
}

static int release(struct inode *inode, struct file *filp)
{
	struct driverInfo *info = filp->private_data;
	atomic_inc(&info->opened);
	return 0;
}

static ssize_t read(struct file *filp, char __user *buf, size_t len,
		    loff_t *off)
{
	unsigned long flags;
	struct driverInfo *info = filp->private_data;

	if (len > info->memSize) {
		return -EINVAL;
	}

	spin_lock_irqsave(&info->lock, flags);
	while (0 == info->readPending) {
		spin_unlock_irqrestore(&info->lock, flags);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		if (wait_event_interruptible(info->read_waitqueue,
					     (0 != info->readPending))) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
		spin_lock_irqsave(&info->lock, flags);
	}

	info->readPending = 0;
	info->writePending = 1;
	if (copy_to_user(buf, info->membase, len)) {
		spin_unlock_irqrestore(&info->lock, flags);
		return -EFAULT;
	}
	spin_unlock_irqrestore(&info->lock, flags);
	wake_up_interruptible(&info->write_waitqueue);
	wake_up_interruptible(&info->poll_waitqueue);
	return len;
}

static void kickOffSharc(u8 core_id)
{
	platform_send_ipi_cpu(core_id, 0);
}

static ssize_t write(struct file *filp, const char __user *buf, size_t len,
		     loff_t *offset)
{
	unsigned long flags;
	struct driverInfo *info = filp->private_data;

	if (len > info->memSize) {
		return -EINVAL;
	}

	spin_lock_irqsave(&info->lock, flags);
	while (0 == info->writePending) {
		spin_unlock_irqrestore(&info->lock, flags);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		if (wait_event_interruptible(info->write_waitqueue,
					     (0 != info->writePending))) {
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
		spin_lock_irqsave(&info->lock, flags);
	}
	info->writePending = 0;
	if (copy_from_user(info->membase, buf, len)) {
		spin_unlock_irqrestore(&info->lock, flags);
		return -EFAULT;
	}
	kickOffSharc(info->core_id);
	spin_unlock_irqrestore(&info->lock, flags);
	return len;
}

static unsigned int poll(struct file *filp, struct poll_table_struct *wait)
{
	unsigned int ret = 0;
	unsigned long flags;

	struct driverInfo *info = filp->private_data;

	poll_wait(filp, &info->poll_waitqueue, wait);

	spin_lock_irqsave(&info->lock, flags);
	if (info->readPending) {
		ret |= POLLIN;
	}
	if (info->writePending) {
		ret |= POLLOUT;
	}
	spin_unlock_irqrestore(&info->lock, flags);

	return ret;
}

static irqreturn_t sharc_core_irq(int irq, void *dev_id)
{
	unsigned long flags;
	struct driverInfo *sharcDevice = dev_get_drvdata(dev_id);
	spin_lock_irqsave(&sharcDevice->lock, flags);
	if (sharcDevice->readPending + 1 != 0) {
		++sharcDevice->readPending;
	}
	spin_unlock_irqrestore(&sharcDevice->lock, flags);
	wake_up(&sharcDevice->read_waitqueue);
	wake_up(&sharcDevice->poll_waitqueue);
	return IRQ_HANDLED;
}

static const struct file_operations sharc_fops = { .owner = THIS_MODULE,
						   .open = open,
						   .release = release,
						   .read = read,
						   .write = write,
						   .poll = poll };

struct sharcDescriptor {
	char *name;
	u8 core_id;
};

static int sharc_remove(struct platform_device *pdev);
static int sharc_probe(struct platform_device *pdev);

static const struct of_device_id cap_match[] = {
	{ .compatible = "iris,sharc" },
	{},
};

MODULE_DEVICE_TABLE(of, cap_match);
static struct platform_driver sharc_driver = {
	.driver =
		{
			.name = DRV_NAME,
			.of_match_table = cap_match,
		},
	.probe = sharc_probe,
	.remove = sharc_remove,
};
module_platform_driver(sharc_driver);

static int sharc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct driverInfo *sharcDevice;
	const void *of_property_coreid;
	const struct of_device_id *of_id;
	struct resource *res;
	int ret = 0;

	if (!of_match_device(cap_match, &pdev->dev)) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}

	/* Prepare our private structure */
	sharcDevice = devm_kzalloc(dev, sizeof(struct driverInfo), GFP_ATOMIC);
	if (!sharcDevice) {
		dev_err(dev, "Can't allocate private structure\n");
		return -ENODEV;
	}

	init_waitqueue_head(&sharcDevice->read_waitqueue);
	init_waitqueue_head(&sharcDevice->write_waitqueue);
	init_waitqueue_head(&sharcDevice->poll_waitqueue);

	of_id = of_match_node(cap_match, dev->of_node);
	if (!of_id)
		return -EINVAL;

	of_property_coreid = of_get_property(dev->of_node, "coreid", NULL);
	if (!of_property_coreid) {
		dev_err(dev, "Missing `coreid` property in device tree\n");
		return -ENOENT;
	}

	sharcDevice->core_id = be32_to_cpup(of_property_coreid);

	sharcDevice->lock = __SPIN_LOCK_UNLOCKED(sharcDevice->lock);

	sharcDevice->readPending = 0;
	sharcDevice->writePending = 1;
	sharcDevice->irq = platform_get_irq(pdev, 0);

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	sharcDevice->irq_type = (res->flags & IORESOURCE_BITS) | IRQF_PERCPU;

	if (devm_request_irq(dev, sharcDevice->irq, sharc_core_irq,
			     sharcDevice->irq_type, dev->of_node->name, dev)) {
		dev_err(dev, "unable to attach sharc IRQ\n");
		return -EBUSY;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "Cannot get IORESOURCE_MEM\n");
		return -ENOENT;
	}
	sharcDevice->memSize = resource_size(res);
	sharcDevice->membase = devm_ioremap_nocache(&pdev->dev, res->start,
						    sharcDevice->memSize);
	if (!sharcDevice->membase) {
		dev_err(dev, "Cannot map shared memory\n");
		return -ENOENT;
	}

	sharcDevice->misc_device.name = dev->of_node->name;
	sharcDevice->misc_device.minor = MISC_DYNAMIC_MINOR;
	sharcDevice->misc_device.fops = &sharc_fops;
	ret = misc_register(&sharcDevice->misc_device);

	if (0 != ret) {
		dev_err(dev, "cannot register sharc_misc device\n");
		return ret;
	}

	atomic_set(&sharcDevice->opened, 1);
	dev_info(dev, "sharc core enabled\n");

	dev_set_drvdata(dev, sharcDevice);

	return ret;
}

static int sharc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct driverInfo *sharcDevice = dev_get_drvdata(dev);
	free_irq(sharcDevice->irq, dev);
	return 0;
}

MODULE_DESCRIPTION("A driver that synchronizes against the SHARC cores");
MODULE_AUTHOR("Erik Schumacher <Erik.Schumacher@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
