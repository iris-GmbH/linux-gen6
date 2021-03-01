#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>

#define DRV_NAME "iris_cache_control"

enum CACHE_OPERATION {
	OP_INVALIDATE = 0,
	OP_FLUSH = 1,
};

struct cache_control_data {
	struct miscdevice misc_device;
	enum CACHE_OPERATION op;
};

struct memory_info {
	unsigned long virtPtr;
	unsigned long phyPtr;
	unsigned long size;
};

static int cache_open(struct inode *inode, struct file *filp)
{
	/* filp->private_data is populated with misc_device,
	 * extract the parent struct aka our driverInfo struct
	 */
	struct cache_control_data *cacheDevice = container_of(
		filp->private_data, struct cache_control_data, misc_device);
	/* put our struct back */
	filp->private_data = cacheDevice;
	return 0;
}

static long cache_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct cache_control_data *info = filp->private_data;
	enum CACHE_OPERATION op = info->op;

	if (cmd != 0) {
		return -EINVAL;
	}

	switch (op) {
	case OP_INVALIDATE: /* invalidate cache for given address and size */
	{
		/* Might be a useful operation later. Could not be tested.
		 * Probing for OP_INVALIDATE will return -EINVAL
		 */
	} break;
	case OP_FLUSH: /* flush cache for given address and size */
	{
		struct memory_info info;

		ret = copy_from_user(&(info), (void *)arg, sizeof(info));
		if (!ret) {
			v7_flush_kern_dcache_area((void *)info.virtPtr,
						  info.size);
			outer_flush_range(info.phyPtr,
					  (info.phyPtr + info.size));
		}
	} break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct file_operations cache_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cache_ioctl,
	.open = cache_open,
};

static int cache_remove(struct platform_device *pdev);
static int cache_probe(struct platform_device *pdev);
static const struct of_device_id cap_match[] = {
	{
		.compatible = "iris,cachecontrol-invalidate",
		.data = (void *)OP_INVALIDATE,
	},
	{
		.compatible = "iris,cachecontrol-flush",
		.data = (void *)OP_FLUSH,
	},
	{},
};
MODULE_DEVICE_TABLE(of, cap_match);
static struct platform_driver cache_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = cap_match,
	},
	.probe = cache_probe,
	.remove = cache_remove,
};
module_platform_driver(cache_driver);

static int cache_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct device *dev = &pdev->dev;
	struct cache_control_data *driverData;
	enum CACHE_OPERATION op;
	int ret = 0;

	match = of_match_device(cap_match, &pdev->dev);
	if (!match) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}

	op = (enum CACHE_OPERATION)match->data;

	/* remove this check if you implement OP_INVALIDATE */
	if (op == OP_INVALIDATE) {
		return -EINVAL;
	}

	driverData = devm_kzalloc(&pdev->dev, sizeof(*driverData), GFP_KERNEL);
	if (!driverData) {
		return -ENOMEM;
	}

	driverData->op = op;
	driverData->misc_device.name = dev->of_node->name;
	driverData->misc_device.minor = MISC_DYNAMIC_MINOR;
	driverData->misc_device.fops = &cache_fops;
	ret = misc_register(&driverData->misc_device);
	if (ret) {
		devm_kfree(&pdev->dev, driverData);
		dev_err(dev, "cannot register misc device for region: %s\n",
			dev->of_node->name);
	}

	dev_set_drvdata(dev, driverData);

	return ret;
}

static int cache_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cache_control_data *driverData = dev_get_drvdata(dev);

	misc_deregister(&driverData->misc_device);
	devm_kfree(dev, driverData);
	return 0;
}

MODULE_DESCRIPTION(
	"Driver to flush, clean and invalidate cached data of a given memory region");
MODULE_AUTHOR("Erik Schumacher <Erik.Schumacher@irisgmbh.de>");
MODULE_LICENSE("GPL v2");

