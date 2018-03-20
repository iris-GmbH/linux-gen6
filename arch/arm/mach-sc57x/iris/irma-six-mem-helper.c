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

#define DRV_NAME        "gen6_cont_memory"

struct cont_mem_dev_data {
	struct list_head list;
	struct resource r;
	struct miscdevice misc_device;
};

static struct list_head dev_datas = LIST_HEAD_INIT(dev_datas);

struct ioctl_info {
	unsigned long baseAddr;
	unsigned long size;
};


static int cont_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct list_head* position;
	struct cont_mem_dev_data* dd = NULL;
	size_t size = vma->vm_end - vma->vm_start;

	list_for_each (position , &dev_datas) {
		struct cont_mem_dev_data* dev_d = list_entry(position, struct cont_mem_dev_data, list);
		if (MINOR(file->f_inode->i_rdev) == dev_d->misc_device.minor) {
			dd = dev_d;
			break;
		}
	}
	if (!dd) {
		printk("cannot mmap a file which is not handled\n");
		return -EINVAL;
	}

	if (size > resource_size(&dd->r)) {
		printk("cannot mmap bigger region of cont memory than available");
		return -ENOMEM;
	}

	if (remap_pfn_range(vma, vma->vm_start,
			dd->r.start >> PAGE_SHIFT,
			size, PAGE_SHARED))
	{
	     printk("remap page range failed\n");
	     return -ENXIO;
	}
	return 0;
}

static long cont_mem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	switch (cmd) {
	case 0:
	{
		struct list_head* position;
		list_for_each (position , &dev_datas) {
			struct cont_mem_dev_data* dd = NULL;
			dd = list_entry(position, struct cont_mem_dev_data, list);
			if (dd->misc_device.minor == MINOR(filp->f_inode->i_rdev)) {
				struct ioctl_info info;
				info.baseAddr = dd->r.start;
				info.size = resource_size(&dd->r);
				ret = copy_to_user((void*)arg, &(info), sizeof(info));
				break;
			}
		}
	}
	break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static const struct file_operations cont_memory_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cont_mem_ioctl,
	.mmap = cont_mem_mmap,
};

static int cont_mem_remove(struct platform_device *pdev);
static int cont_mem_probe(struct platform_device *pdev);
static const struct of_device_id cap_match[] = {
	{ .compatible = "iris,gen6-cont-memory", }, {},
};
MODULE_DEVICE_TABLE(of, cap_match);
static struct platform_driver cont_mem_driver = {
	.driver = {
		.name  = DRV_NAME,
		.of_match_table = cap_match,
	},
	.probe = cont_mem_probe,
	.remove = cont_mem_remove,
};
module_platform_driver(cont_mem_driver);

static int cont_mem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node* np;
	int ret;
	int num_regions = 0;

	if (!of_match_device(cap_match, &pdev->dev)) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}
	while (1) {
		int subRet = 0;
		struct cont_mem_dev_data* dd;
		np = of_parse_phandle(dev->of_node, "memory-region", num_regions);
		++num_regions;
		if (!np) {
			break;
		}
		dd = devm_kzalloc(&pdev->dev, sizeof(*dd), GFP_KERNEL);
		if (!dd) {
			dev_err(dev, "cannot allocate device data\n");
			return -ENOMEM;
		}
		subRet = of_address_to_resource(np, 0, &dd->r);
		if (subRet) {
			devm_kfree(&pdev->dev, dd);
			break;
		}
		INIT_LIST_HEAD(&dd->list);
		dd->misc_device.name = dd->r.name;
		dd->misc_device.minor = MISC_DYNAMIC_MINOR;
		dd->misc_device.fops  = &cont_memory_fops;
		subRet = misc_register(&dd->misc_device);
		if (subRet) {
			devm_kfree(&pdev->dev, dd);
			dev_err(dev, "cannot register misc device for region: %s %d\n", dd->r.name, subRet);
			continue;
		}
		list_add_tail(&dd->list, &dev_datas);
		dev_info(dev, "reserved memory %s@0x%X-0x%X", dd->r.name, dd->r.start, dd->r.end);
	}
	return ret;
}

static int cont_mem_remove(struct platform_device *pdev)
{
	struct list_head *position, *q;
	list_for_each_safe(position, q, &dev_datas) {
		struct cont_mem_dev_data* dd = NULL;
		dd = list_entry(position, struct cont_mem_dev_data, list);
		list_del(position);
		devm_kfree(&pdev->dev, dd);
	}
	return 0;
}

MODULE_DESCRIPTION("Driver that provides mmapped contiguous memory regions into userspace");
MODULE_AUTHOR("Lutz Freitag <Lutz.Freitag@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
