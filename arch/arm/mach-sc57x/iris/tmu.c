#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <asm/io.h> /* ioremap */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <mach/sc57x.h>
#include "tmu.h"

#define DEV_NAME	"tmu"

//#define EnableIR

static dev_t tmu_dev_number;
static struct cdev *tmu_object;
static struct class *tmu_class;
static struct platform_device_id tmu_pdi;
static void __iomem *regTmuBaseAddress; //REG_TMU0_CTL
static uint16_t temperatureQ7_8;

//static struct file_operations tmu_fops;
static DECLARE_COMPLETION( dev_obj_is_free );

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
	writel((alertTemp & TMU0_ALRT_LIM_HI_MASK), regTmuBaseAddress + SZ_4 * REGP_TMU0_ALRT_LIM_HI);
}

void setFaultHigh(uint8_t alertTemp){
	//should only be set, for values higher then 60°C
	writel((alertTemp & TMU0_FLT_LIM_HI_MASK), regTmuBaseAddress + SZ_4 * REGP_TMU0_FLT_LIM_HI);
}

void setHighIrMask(void){
	//only enable high fault/alert interrupts
	writel(((TMU0_IMSK_FLTHI|TMU0_IMSK_ALRTHI) & TMU0_IMSK_MASK), regTmuBaseAddress + SZ_4 * REGP_TMU0_IMSK);
}

void printTmuRegisters(void){
	uint32_t regs[13];
	int i = 0;

	for(i=0;i<13;i++){
		regs[i] = readl(regTmuBaseAddress + SZ_4*i);
		printk(KERN_INFO "tmu_read reg[%d]: 0x%08x\n", i, regs[i]);
	}
}

static int tmu_open( struct inode *device_file, struct file *entity){
	pr_info("TMU OPEN\n");
	setGain(0);
	setOffset(0);
	setAVG(1);
	setConfigure( (TMU0_CTL_TMPU | TMU0_CTL_TMEN) );
	return 0;
}

static int tmu_close( struct inode *device_file, struct file *entity){
	pr_info("TMU CLOSE\n");
	writel( (0x0 & TMU0_CTL_MASK), regTmuBaseAddress); //power down tmu driver
	return 0;
}

static ssize_t tmu_read(struct file *entity, char __user *user,
	size_t count, loff_t *offset){

	//printTmuRegisters();
	if(count != sizeof(temperatureQ7_8))
		return -EINVAL; //other sizes are not valid

	temperatureQ7_8 = (uint16_t) readl(regTmuBaseAddress + SZ_4*REGP_TMU0_TEMP) & 0xFFFF; // return value in Q7.8 format
	if(put_user(temperatureQ7_8, (uint16_t*)user)){
		printk( "tmu_read: put_user failed\n");
		return -EFAULT;
	}
	return temperatureQ7_8;
}

#ifdef EnableIR
static irqreturn_t tmu_isr(int p_irq, void *p_data){
	uint32_t status = readl(regTmuBaseAddress + SZ_4*REGP_TMU0_STAT);
	if(status&TMU0_STAT_FLTHI){
		printk("tmu fault high cpu-temperature >%uC\n", HIGH_FAULT_LIM);
		writel(TMU0_STAT_FLTHI &TMU0_STAT_MASK, regTmuBaseAddress + SZ_4*REGP_TMU0_STAT); //clear irq
		return IRQ_HANDLED;
	}else if(status&TMU0_STAT_ALRTHI){
		printk("tmu alert high cpu-temperature >%uC\n", HIGH_ALERT_LIM);
		writel(TMU0_STAT_ALRTHI &TMU0_STAT_MASK, regTmuBaseAddress + SZ_4*REGP_TMU0_STAT); //clear irq
		return IRQ_HANDLED;
	}
	return IRQ_NONE; //e.g. low temp not implemented yet
}
#endif

static int tmu_probe_device(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
#ifdef EnableIR
	int ret, irq;
#endif
	pr_info("tmu_probe_device( %p )\n", dev);
//	pr_info("pdev->id: %d\n", pdev->id );

	if( (regTmuBaseAddress = ioremap(REG_TMU0_BASE_ADDRESS, SZ_4 *13))==NULL) //base address
		return -1;
#ifdef EnableIR
	setHighIrMask();

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "%s: no TMU_FAULT irq.\n", __func__);
		return -ENODEV;
	}
	setFaultHigh(HIGH_FAULT_LIM);
	ret = request_irq(irq, tmu_isr, 0, "TMU_FAULT", NULL);
	if(ret<0){
		dev_err(dev, "TMU_FAULT: configure_irq error, ret:%d\n",ret);
		return ret;
	}
	setAlertHigh(HIGH_ALERT_LIM);
	irq = platform_get_irq(pdev, 1);
	if (irq < 0) {
		dev_err(dev, "%s: no TMU_ALERT irq.\n", __func__);
		return -ENODEV;
	}
	ret = request_irq(irq, tmu_isr, 0, "TMU_ALERT", NULL);
	if(ret<0){
		dev_err(dev, "TMU_ALERT: configure_irq error, ret:%d\n",ret);
		return ret;
	}
#endif
	return 0;
}

static int tmu_remove_device(struct platform_device *pdev){
	struct device *dev = &pdev->dev;
	int irq;
	pr_info("tmu_remove_device( %p )\n", dev);
//	pr_info("pdev->id: %d\n", pdev->id );
	iounmap(regTmuBaseAddress);
#ifdef EnableIR
	irq = platform_get_irq(pdev, 0);
	free_irq(irq,dev);
	irq = platform_get_irq(pdev, 1);
	free_irq(irq,dev);
#endif
	return 0;
}

static void tmu_release( struct device *dev )
{
	complete(&dev_obj_is_free);
}

static struct file_operations tmu_fops = {
	.owner		= THIS_MODULE,
	.read   	= tmu_read,
	.open 		= tmu_open,
	.release 	= tmu_close,
};

struct platform_device tmu_device = {
	.name  = "tmu",
	.id   = -1, /* remove ".0" from /dev/tmu.0 to /dev/tmu */
	.dev = {
		.release = tmu_release,
	}
};

static const struct of_device_id cap_match[] = {
	{ .compatible = "adi,sc57x-tmu", }, {},
};
MODULE_DEVICE_TABLE(of, cap_match);

static struct platform_driver tmu_driver = {
	.probe = tmu_probe_device,
	.remove = tmu_remove_device,
	.driver = {
		.name = "tmu_dev_drv",
		.of_match_table = cap_match,
	},
};

static int __init tmu_init(void){
	pr_info("tmu_init()\n");
	strcpy( tmu_pdi.name, "tmudev" );
	tmu_driver.id_table = &tmu_pdi;
	if (platform_driver_register(&tmu_driver)!=0) {
		pr_err("driver_register failed\n");
		return -EIO;
	}
	if (alloc_chrdev_region(&tmu_dev_number,0,1,DEV_NAME)<0)
		return -EIO;
	tmu_object = cdev_alloc();
	if (tmu_object==NULL)
		goto free_device_number;
	tmu_object->owner = THIS_MODULE;
	tmu_object->ops = &tmu_fops;
	if (cdev_add(tmu_object,tmu_dev_number,1))
		goto free_cdev;
	tmu_class = class_create( THIS_MODULE, DEV_NAME );
	if (IS_ERR(tmu_class)) {
		printk("tmu: no udev support.\n");
		goto free_cdev;
	}
	tmu_device.dev.devt = tmu_dev_number;
	platform_device_register( &tmu_device );
	return 0;

free_cdev:
	kobject_put( &tmu_object->kobj );
free_device_number:
	unregister_chrdev_region( tmu_dev_number, 1 );
	return -EIO;
}

static void __exit tmu_exit(void){
	device_release_driver( &tmu_device.dev );
	platform_device_unregister( &tmu_device );
	class_destroy( tmu_class );
	cdev_del( tmu_object );
	unregister_chrdev_region( tmu_dev_number, 1 );
	platform_driver_unregister(&tmu_driver);
	wait_for_completion( &dev_obj_is_free );
}

module_init( tmu_init );
module_exit( tmu_exit );
MODULE_DESCRIPTION("Driver that provides the processor temperature (via tmu).");
MODULE_AUTHOR("Michael Glembotzki <Michael.Glembotzki@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
