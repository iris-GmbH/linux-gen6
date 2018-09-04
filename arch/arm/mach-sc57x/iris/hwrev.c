#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h> /* ioremap */
#include <linux/uaccess.h>
#include <sound/sc5xx-sru.h>

#define PADS0_DAI0_PUE		0x31004498  //= 1 (default: 0x00000000)
#define PADS0_DAI0_PUD		0x310044D8	//= 0 (default: 0x0000FFFF)
#define DAI0_PIN_STAT 		0x310C92E4
#define HWMASK				0x000F 		// DAI0_PIN1 to DAI0_PIN4
#define MODULNAME 			"hwrev"

static dev_t		 	hwrev_dev_number;
static struct cdev		*driver_object;
static struct class		*hwrev_class;
static struct device	*hwrev_dev;
static void __iomem 	*regBaseAddressPinStat;

int initDaiAsGpios(void) {
	uint32_t t = 0;
	void __iomem *regBaseAddressPUE;
	void __iomem *regBaseAddressPUD;

	if((regBaseAddressPUE = ioremap(PADS0_DAI0_PUE, SZ_4))==NULL ||
		(regBaseAddressPUD = ioremap(PADS0_DAI0_PUD, SZ_4))==NULL) {
		pr_err("Error initPullUps ioremap\n");
		return -1;
	}
	writel(HWMASK, regBaseAddressPUE); //enable pull ups
	t = readl(regBaseAddressPUD);
	writel(t & ~HWMASK , regBaseAddressPUD); //clear pud disable
	
	/* Configure DAI0_PIN1 to DAI0_PIN4 as Input pin's */
	SRU(0, LOW, DAI0_PBEN01_I);
	SRU(0, LOW, DAI0_PBEN02_I);
	SRU(0, LOW, DAI0_PBEN03_I); 
	SRU(0, LOW, DAI0_PBEN04_I);
	
	iounmap(regBaseAddressPUE);
	iounmap(regBaseAddressPUD);
	return 0;
}
static int hwrev_open(struct inode *device_file, struct file *entity){
	//dev_info(hwrev_dev, "hwrev_open called\n");
	if((regBaseAddressPinStat = ioremap(DAI0_PIN_STAT, SZ_4))==NULL){
		pr_err("Error initPullUps ioremap\n");
		return -1;
	}
	return 0;
}
static int hwrev_close(struct inode *device_file, struct file *entity){
	iounmap(regBaseAddressPinStat);
	//dev_info(hwrev_dev, "hwrev_close called\n");
	return 0;
}
static ssize_t hwrev_read(struct file *entity, char __user *user,
	size_t count, loff_t *offset){
	uint32_t pinState = readl(regBaseAddressPinStat);
	
	if(count!=sizeof(uint32_t))
		return -EINVAL;
	
	if(put_user(pinState &= HWMASK, (uint32_t*)user))
		return -EFAULT;
	return pinState;
}
static struct file_operations fops={
	.owner   = THIS_MODULE,
	.read    = hwrev_read,
	.open    = hwrev_open,
	.release = hwrev_close,
};
static int __init hwrev_init(void){
	if(alloc_chrdev_region(&hwrev_dev_number,0,1,MODULNAME)<0)
		return -EIO;
	driver_object = cdev_alloc();
	if(driver_object==NULL)
		goto free_device_number;
	driver_object->owner = THIS_MODULE;
	driver_object->ops = &fops;
	if(cdev_add(driver_object,hwrev_dev_number,1))
		goto free_cdev;
	hwrev_class = class_create(THIS_MODULE,MODULNAME);
	if(IS_ERR(hwrev_class)){
		pr_err("hwrev_class: no udev support\n");	
		goto free_cdev;
	}
	hwrev_dev = device_create(hwrev_class, NULL, hwrev_dev_number,
		NULL, "%s",MODULNAME);
	if(IS_ERR(hwrev_dev)){
		pr_err("hwrev_dev: device_create() failed\n");
		goto free_class;
	}
	if(initDaiAsGpios()!=0) {
		goto free_class;
	}
	return 0;
free_class:
	class_destroy(hwrev_class);
free_cdev:
	kobject_put(&driver_object->kobj);
free_device_number:
	unregister_chrdev_region(hwrev_dev_number,1);
	return -EIO;
}
static void __exit hwrev_exit(void){
	device_destroy(hwrev_class, hwrev_dev_number);
	class_destroy(hwrev_class);
	cdev_del(driver_object);
	unregister_chrdev_region(hwrev_dev_number,1);
	return;
}

module_init(hwrev_init);
module_exit(hwrev_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael Glembotzki <Michael.Glembotzki@irisgmbh.de>");
MODULE_DESCRIPTION("Driver that provides dai-gpio's hw-info.");
MODULE_VERSION("V0.0");
