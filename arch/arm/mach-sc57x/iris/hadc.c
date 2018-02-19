/* housekeeping adc (hadc) */

#include <linux/module.h>	/* Needed by all modules */
#include <linux/kernel.h>	/* Needed for KERN_INFO */
#include <linux/init.h>		/* Needed for the macros */
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/io.h> /* ioremap */
#include <linux/types.h>
#include <mach/sc57x.h>
#include "hadc.h"


struct hadc0_data{
	uint32_t data[8];
};

#define DEV_NAME				"hadc"
#define MAX_HADC_CHANNEL		8

static dev_t hadc_dev_number;
static struct cdev *hadc_object;
static struct class *hadc_class;
static struct device *hadc_dev;

void initHADC(void){
	void __iomem *hadc_ctrl_reg = ioremap(HADC0_CTL, 4);
	void __iomem *status_reg = ioremap(HADC0_STAT, 4);
	uint32_t tmp_ctrl = readl(hadc_ctrl_reg);
	uint32_t tmp_status;

	CLEAR(tmp_ctrl, HADC_CTRL_PD);		//- Deassert the HADC_CTL.PD bit (HADC power down)
	SET(tmp_ctrl,HADC_CTRL_NRST);		//- Set the HADC_CTL.NRST bit (Reset)
	SET(tmp_ctrl,HADC_CTRL_ENLS);		//- Set the HADC_CTL.ENLS bit (Enable level shifters)
	SET(tmp_ctrl,HADC_CTRL_STARTCNV);	//- Enabling the HADC
	writel(tmp_ctrl, hadc_ctrl_reg);

	//wait until adc is ready
	while( ((tmp_status=readl(status_reg))&0x01)==0)
	{
		pr_info("not ready. status_reg:%#04x\n", tmp_status);
	}
	iounmap(hadc_ctrl_reg);
	iounmap(status_reg);
}

void stopHADC(void){
	void __iomem *hadc_ctrl_reg = ioremap(HADC0_CTL, 4);
	uint32_t tmp_ctrl = readl(hadc_ctrl_reg);
	CLEAR(tmp_ctrl, HADC_CTRL_PD);		//- Deassert the HADC_CTL.PD bit (HADC power down)
	SET(tmp_ctrl,HADC_CTRL_NRST);		//- Set the HADC_CTL.NRST bit (Reset)
	writel(tmp_ctrl, hadc_ctrl_reg);
	iounmap(hadc_ctrl_reg);
}

struct hadc0_data getValue(void){
	void __iomem *data = ioremap(HADC0_DATA0, MAX_HADC_CHANNEL * sizeof(uint32_t));
	struct hadc0_data newdata;
	int i=0;
	for(i=0;i<MAX_HADC_CHANNEL;i++){
		newdata.data[i] = readl(data+(i* sizeof(uint32_t)));
		//pr_info("HADC0_DATA%d: %#04x\n", i, newdata.data[i]);
	}
	iounmap(data);
	return newdata;
}

static long driver_ioctl(struct file *instance, unsigned int cmd, unsigned long arg){
	int not_copied;
	struct hadc0_data mydata;
	//dev_info(hadc_dev, "ioctl called 0x%4.4x %p\n",cmd, (void *)arg);

	switch(cmd){
	case START_HADC:
		stopHADC();
		dev_info(hadc_dev, "start continous hadc\n");
		break;
	case STOP_HADC:
		initHADC();
		dev_info(hadc_dev, "stop hadc\n");
		break;
	case READ_HADC_IOTCL:
		/* do not forget to start & stop in userspace application */
		mydata = getValue();
		//dev_info(hadc_dev, "val0:%u\n",mydata.data[0]);
		not_copied=copy_to_user((void *)arg, &mydata,sizeof(struct hadc0_data));
		break;
	case READ_HADC_IOTCL_SINGLE:
		initHADC();
		mydata = getValue();
		stopHADC();
		not_copied=copy_to_user((void *)arg, &mydata,sizeof(struct hadc0_data));
		break;
	case IOCTL_TESTINPUT:
		 printk(KERN_INFO "HADC welcomes you to the world!\n");
		break;
	default:
		printk("unknown IOCTL 0x%x\n",cmd);
		return -EINVAL;
	}
	return 0;
}

static struct file_operations hadc_fops = {
	/*.open = driver_open,
	.release = driver_close,*/
	.owner= THIS_MODULE,
	/*.compat_ioctl=driver_ioctl*/
	.unlocked_ioctl=driver_ioctl
};


static int __init hadc_init(void)
{
	if(alloc_chrdev_region(&hadc_dev_number,0,1,DEV_NAME)<0)
		return -EIO;
	hadc_object = cdev_alloc();
	if(hadc_object==NULL)
		goto free_device_number;
	hadc_object->owner = THIS_MODULE;
	hadc_object->ops = &hadc_fops;
	if(cdev_add(hadc_object,hadc_dev_number,1))
		goto free_cdev;
	hadc_class = class_create(THIS_MODULE, DEV_NAME);
	if(IS_ERR(hadc_class)){
		pr_err("hadc:no udev support\n");
		goto free_cdev;
	}
	hadc_dev = device_create(hadc_class, NULL, hadc_dev_number,
			NULL, "%s", DEV_NAME);
	if(IS_ERR(hadc_dev)){
		pr_err("hadc: device create faileed\n");
		goto free_class;
	}
	dev_info(hadc_dev, "hadc __init called\n");
	return 0;
free_class:
	class_destroy(hadc_class);
free_cdev:
	kobject_put(&hadc_object->kobj);
free_device_number:
	unregister_chrdev_region(hadc_dev_number, 1);
	return -EIO;
}

static void __exit hadc_exit( void )
{
	device_destroy(hadc_class, hadc_dev_number);
	class_destroy(hadc_class);
	cdev_del(hadc_object);
	unregister_chrdev_region(hadc_dev_number, 1);
	return;
}

module_init(hadc_init);
module_exit(hadc_exit);
MODULE_DESCRIPTION("Driver that provides HADC values over ioctl");
MODULE_AUTHOR("Michael Glembotzki <Michael.Glembotzki@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
