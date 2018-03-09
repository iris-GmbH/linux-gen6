/*  Thermal monitoring unit (tmu) */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <asm/io.h> /* ioremap */
#include <linux/types.h>
#include <mach/sc57x.h>
#include <mach/irqs.h>
#include <linux/interrupt.h>
#include "tmu.h"

#define DEV_NAME	"tmu"

static dev_t tmu_dev_number;
static struct cdev *tmu_object;
static struct class *tmu_class;
static struct device *tmu_dev;

static void __iomem *regTmuBaseAddress; //REG_TMU0_CTL
static uint16_t temperatureQ7_8;
static irqreturn_t alert_high_isr(int p_irq, void *p_data){
#if 0
	uint32_t status = readl(regTmuBaseAddress + SZ_4*REGP_TMU0_STAT);
/*	uint32_t temp   = readl(regTmuBaseAddress + SZ_4*REGP_TMU0_TEMP);*/
	if(status& (1>>5))
		printk("tmu alert high cpu-temperature: %uC \n", (temp>>8)&0xFF);
	if(status& (1>>4))
#endif
	printk("tmu alert high cpu-temperature: \n");


	return IRQ_HANDLED;
}

void setOffset(uint32_t offset){
	writel((offset & TMU0_OFFSET_MASK), regTmuBaseAddress + SZ_4 * REGP_TMU0_OFFSET); //set REG_TMU0_OFFSET
}

void setAVG(uint32_t avg){
	writel((avg & TMU0_AVG_MASK), regTmuBaseAddress + SZ_4 * REGP_TMU0_AVG); //set REG_TMU0_AVG
}

void setGain(uint32_t gain){
	writel(gain & TMU0_GAIN_MASK, regTmuBaseAddress + SZ_4 * REGP_TMU0_GAIN); //set REG_TMU0_GAIN
}

void setConfigure(uint32_t config){
	//power down up and start periodic read
	writel((config & TMU0_CTL_MASK), regTmuBaseAddress); //set REG_TMU0_CTL
}

void setAlertHigh(uint8_t alertTemp){
	//should only be set, for values higher then 60°C
	writel((alertTemp & TMU0_ALRT_LIM_HI_MASK), regTmuBaseAddress + SZ_4 * REGP_TMU0_ALRT_LIM_HI); //set REG_TMU0_CTL
}

static int tmu_open( struct inode *device_file, struct file *entity){
	setGain(0);
	setOffset(0);
	setAVG(1);
	setConfigure( (TMU0_CTL_TMPU | TMU0_CTL_TMEN) );
	return 0;
}

static int tmu_close( struct inode *device_file, struct file *entity){
	//power down tmu driver
	writel( (0x0 & TMU0_CTL_MASK), regTmuBaseAddress); //set REG_TMU0_CTL
	return 0;
}

void printTmuRegisters(void){
	uint32_t regs[13];
	int i = 0;

	for(i=0;i<13;i++){
		regs[i] = readl(regTmuBaseAddress + SZ_4*i);
		printk(KERN_INFO "tmu_read reg[%d]: 0x%08x\n", i, regs[i]);
	}
}

static ssize_t tmu_read(struct file *entity, char __user *user,
	size_t count, loff_t *offset){

	//printTmuRegisters();
	if(count != sizeof(temperatureQ7_8))
		return -EINVAL; //other sizes are not valid

	temperatureQ7_8 = (uint16_t) readl(regTmuBaseAddress + SZ_4*REGP_TMU0_TEMP) & 0xFFFF; // return value in Q7.8 format
	//printk(KERN_INFO "tmu_read: tempvalue: %u hex:0x%08x\n\n", (rawQ7_8>>8)&0xFFFF, rawQ7_8);

	if(put_user(temperatureQ7_8, (uint16_t*)user)){
		printk( "tmu_read: put_user failed\n");
		return -EFAULT;
	}
	return temperatureQ7_8;
}


static struct file_operations tmu_fops = {
	.owner		= THIS_MODULE,
	.read   	= tmu_read,
	.open 		= tmu_open,
	.release 	= tmu_close,
};


static int __init tmu_init(void){
	int ret;

	if( (regTmuBaseAddress = ioremap(REG_TMU0_BASE_ADDRESS, SZ_4 *13))==NULL) //base address
		goto unioremap;

	if(alloc_chrdev_region(&tmu_dev_number,0,1,DEV_NAME)<0)
		return -EIO;
	tmu_object = cdev_alloc();
	if(tmu_object==NULL)
		goto free_device_number;
	tmu_object->owner = THIS_MODULE;
	tmu_object->ops = &tmu_fops;
	if(cdev_add(tmu_object,tmu_dev_number,1))
		goto free_cdev;
	tmu_class = class_create(THIS_MODULE, DEV_NAME);
	if(IS_ERR(tmu_class)){
		pr_err("tmu:no udev support\n");
		goto free_cdev;
	}
	tmu_dev = device_create(tmu_class, NULL, tmu_dev_number,
			NULL, "%s", DEV_NAME);
	if(IS_ERR(tmu_dev)){
		pr_err("tmu: device create faileed\n");
		goto free_class;
	}

	setAlertHigh(36);
#if 0
	ret = request_irq(IRQ_TMU0_ALERT, alert_high_isr,
				0,
				"tmu alert irq", tmu_object);
#endif
#if 0
	ret = devm_request_irq(tmu_dev, IRQ_TMU0_ALERT, alert_high_isr,
			       0, "tmu alert irq", tmu_dev_number);

	if(ret<0){
		pr_err("tmu: configure_irq error\n");
		goto free_class;
	}
#endif
	dev_info(tmu_dev, "tmu __init called\n");
	return 0;

free_class:
	class_destroy(tmu_class);
free_cdev:
	kobject_put(&tmu_object->kobj);
free_device_number:
	unregister_chrdev_region(tmu_dev_number, 1);
unioremap:
	return -EIO;
}

static void __exit tmu_exit(void){
	iounmap(regTmuBaseAddress);
	device_destroy(tmu_class, tmu_dev_number);
	class_destroy(tmu_class);
	cdev_del(tmu_object);
	unregister_chrdev_region(tmu_dev_number, 1);
	return;
}

module_init(tmu_init);
module_exit(tmu_exit);
MODULE_DESCRIPTION("Driver that provides the processor temperature (via tmu).");
MODULE_AUTHOR("Michael Glembotzki <Michael.Glembotzki@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
