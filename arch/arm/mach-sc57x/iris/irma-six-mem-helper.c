#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/types.h>

#define DRV_NAME        "gen6_cont_memory"

struct cont_mem_dev_data {
	/* used to access capture device */
	struct mutex mutex;
	struct resource r;
};

static struct cont_mem_dev_data* dev_data;

struct cont_mem_dev_data* getExtraInfo(struct file* file) {
	return (struct cont_mem_dev_data*)(file->private_data);
}

static int cont_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	if (size > resource_size(&dev_data->r)) {
		printk("cannot mmap bigger region of cont memory than available");
		return -ENOMEM;
	}

	if (remap_pfn_range(vma, vma->vm_start,
			dev_data->r.start >> PAGE_SHIFT,
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
	ret = copy_to_user(&arg, &(dev_data->r.start), sizeof(dev_data->r.start));
	return ret;
}

static const struct file_operations cont_memory_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = cont_mem_ioctl,
	.mmap = cont_mem_mmap,
};

static struct miscdevice cont_memory_dev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = "cont-memory",
		.fops  = &cont_memory_fops,
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

	if (!of_match_device(cap_match, &pdev->dev)) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}
	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		dev_err(dev, "No %s specified\n", "memory-region");
		return -EINVAL;
	}
	dev_data = devm_kzalloc(&pdev->dev, sizeof(*dev_data), GFP_KERNEL);
	if (!dev_data) {
		dev_err(dev, "cannot allocate dev data\n");
		return -ENOMEM;
	}
	ret = of_address_to_resource(np, 0, &dev_data->r);
	if (ret) {
		dev_err(dev, "No memory address assigned to the region\n");
		devm_kfree(&pdev->dev, dev_data);
		return -EINVAL;
	}

	ret = misc_register(&cont_memory_dev);
	if (ret) {
		dev_err(dev, "No memory register misc device\n");
		devm_kfree(&pdev->dev, dev_data);
		return -EINVAL;
	}

	dev_info(dev, "cont_mem reserved memory 0x%X-0x%X", dev_data->r.start, dev_data->r.end);
	return ret;
}

static int cont_mem_remove(struct platform_device *pdev)
{
	devm_kfree(&pdev->dev, dev_data);
	return 0;
}

MODULE_DESCRIPTION("Driver that provides mmapped contiguous memory regions into userspace");
MODULE_AUTHOR("Lutz Freitag <Lutz.Freitag@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
