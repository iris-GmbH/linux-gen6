#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>  /*copy_to_user()*/
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <mach/hardware.h>

#include "sx127x.h"

#define MEMBUF		20

#define MODULNAME	"sx127x0"
#define VERSION_NR	"V0.0"
#define RELEASE_DATE	"2017-03-14"
#define AUTHOR		"linkjumper"
#define FILENAME	"sx127x.c"
#define VERSION 	"Id:"FILENAME" "VERSION_NR" "RELEASE_DATE" "AUTHOR 

#if IS_ENABLED(CONFIG_ARCH_SC57X)
#if IS_ENABLED(CONFIG_LORAWAN_SEMTECH)

#define REG_PINT4_MSK_SET  0x31005400
#define REG_PINT4_MSK_CLR  0x31005404
#define REG_PINT4_REQ      0x31005408
#define REG_PINT4_ASSIGN   0x3100540C
#define REG_PINT4_EDGE_SET 0x31005410
#define REG_PINT4_EDGE_CLR 0x31005414
#define REG_PINT4_INV_SET  0x31005418
#define REG_PINT4_INV_CLR  0x3100541C
#define REG_PINT4_PINSTATE 0x31005420
#define REG_PINT4_LATCH    0x31005424

#define REG_PINT0_MSK_SET  0x31005000
#define REG_PINT0_MSK_CLR  0x31005004
#define REG_PINT0_REQ      0x31005008
#define REG_PINT0_ASSIGN   0x3100500C
#define REG_PINT0_EDGE_SET 0x31005010
#define REG_PINT0_EDGE_CLR 0x31005014
#define REG_PINT0_INV_SET  0x31005018
#define REG_PINT0_INV_CLR  0x3100501C
#define REG_PINT0_PINSTATE 0x31005020
#define REG_PINT0_LATCH    0x31005024

#define REG_PORTF_FER      0x31004280
#define REG_PORTF_FER_SET  0x31004284
#define REG_PORTF_FER_CLR  0x31004288
#define REG_PORTF_DATA      0x3100428C
#define REG_PORTF_DATA_SET  0x31004290
#define REG_PORTF_DATA_CLR  0x31004294
#define REG_PORTF_DIR      0x31004298
#define REG_PORTF_DIR_SET  0x3100429C
#define REG_PORTF_DIR_CLR  0x310042A0
#define REG_PORTF_INEN     0x310042A4
#define REG_PORTF_INEN_SET 0x310042A8
#define REG_PORTF_INEN_CLR 0x310042A8
#define REG_PORTF_MUX      0x310042B0

#define GPO1_PORT           0x00000020 //SC57x Pin PF_05 (A19) = GPO1 ("LP-GEN6-FE-01.pdf" Frontend schematic)
#define GPO3_PORT           0x00000008 //SC57x Pin PB_03 (C6)  = GPO3 ("LP-GEN6-FE-01.pdf" Frontend schematic)

#define REG_PORTB_FER      0x31004080
#define REG_PORTB_FER_SET  0x31004084
#define REG_PORTB_FER_CLR  0x31004088
#define REG_PORTB_DATA      0x3100408C
#define REG_PORTB_DATA_SET  0x31004090
#define REG_PORTB_DATA_CLR  0x31004094
#define REG_PORTB_DIR      0x31004098
#define REG_PORTB_DIR_SET  0x3100409C
#define REG_PORTB_DIR_CLR  0x310040A0
#define REG_PORTB_INEN     0x310040A4
#define REG_PORTB_INEN_SET 0x310040A8
#define REG_PORTB_INEN_CLR 0x310040A8
#define REG_PORTB_MUX      0x310040B0

//#define INTERRUPT_NUM     16
#define INTERRUPT_NUM     20
/*********************************************/
#if (INTERRUPT_NUM == 16)
/*********************************************/
#define INTERRUPT_PORT 0x00000020

//config_gpio before request_threaded_irq()
#define REG_PORT_FER_CLR  REG_PORTB_FER_CLR
#define REG_PINT_INV_CLR  REG_PINT0_INV_CLR
#define REG_PORT_DIR_CLR  REG_PORTB_DIR_CLR

//config_gpio after request_threaded_irq()
#define REG_PINT_MSK_CLR  REG_PINT0_MSK_CLR //permits masking (disabling) of interrupt requests
#define REG_PINT_ASSIGN   REG_PINT0_ASSIGN
#define REG_PINT_EDGE_SET REG_PINT0_EDGE_SET
#define REG_PINT_LATCH    REG_PINT0_LATCH //writing a 1 to the register clears the respective bits in the register
//#define REG_PINT_MSK_CLR  REG_PINT0_MSK_CLR //permits masking (disabling) of interrupt requests
#define REG_PORT_INEN_SET REG_PORTB_INEN_SET

//mod_exit
#define REG_PINT_MSK_SET  REG_PINT0_MSK_SET //permits unmasking (enabling) of interrupt requests
#define REG_PORT_INEN_CLR REG_PORTB_INEN_CLR

#define REG_PINT_REQ      REG_PINT0_REQ
/*********************************************/
#else
/*********************************************/
#define INTERRUPT_PORT 0x00000800

//config_gpio before request_threaded_irq()
#define REG_PORT_FER_CLR  REG_PORTF_FER_CLR
#define REG_PINT_INV_CLR  REG_PINT4_INV_CLR
#define REG_PORT_DIR_CLR  REG_PORTF_DIR_CLR

//config_gpio after request_threaded_irq()
#define REG_PINT_MSK_CLR  REG_PINT4_MSK_CLR //permits masking (disabling) of interrupt requests
#define REG_PINT_ASSIGN   REG_PINT4_ASSIGN
#define REG_PINT_EDGE_SET REG_PINT4_EDGE_SET
#define REG_PINT_LATCH    REG_PINT4_LATCH //writing a 1 to the register clears the respective bits in the register
//#define REG_PINT_MSK_CLR  REG_PINT4_MSK_CLR //permits masking (disabling) of interrupt requests
#define REG_PORT_INEN_SET REG_PORTF_INEN_SET

//mod_exit
#define REG_PINT_MSK_SET  REG_PINT4_MSK_SET //permits unmasking (enabling) of interrupt requests
#define REG_PORT_INEN_CLR REG_PORTF_INEN_CLR

#define REG_PINT_REQ      REG_PINT4_REQ
/*********************************************/
#endif
/*********************************************/

#endif
#endif

static dev_t		 gpio_dev_number;
static struct cdev	*driver_object;
static struct class	*gpio_class;
static struct device	*gpio_dev;
static int 		 rpi_irq_16;
static char		*devname = "int_pin_to_irq16";
static wait_queue_head_t sleeping_for_ir;
static int 		 interrupt_arrived;

static irqreturn_t rpi_gpio_isr(int p_irq, void *p_data){
	printk("rpi_gpio_isr( %d, %p )\n", p_irq, p_data);
	interrupt_arrived += 1;
	wake_up(&sleeping_for_ir);
	return IRQ_HANDLED;
}

static irqreturn_t hard_isr(int irq, void *dev_id){
	writel(0xffffffff, __io_address(REG_PINT_REQ)); //clear all interrupt requests:  //writing a 1 to the register clears the respective bits in the register
	writel(0xffffffff, __io_address(REG_PINT_LATCH)); //clear potential latches due to history:  //writing a 1 to the register clears the respective bits in the register
	printk("hard_isr( %d, %p )\n", irq, dev_id);
	return IRQ_WAKE_THREAD;
}

static int config_gpio(int p_gpionr){
	int _err, _rpi_irq;

//before request_threaded_irq()
	writel(INTERRUPT_PORT, __io_address(REG_PORT_FER_CLR));
	writel(INTERRUPT_PORT, __io_address(REG_PINT_INV_CLR));
	writel(INTERRUPT_PORT, __io_address(REG_PORT_DIR_CLR));

	_rpi_irq = p_gpionr;
	printk("gpio_to_irq returned %d\n",_rpi_irq);
	_err = request_threaded_irq(_rpi_irq, hard_isr, rpi_gpio_isr,
		IRQF_TRIGGER_RISING, //gpio 16 successfull configured ... | IRQF_TRIGGER_FALLING, genirq: Setting trigger mode 3 for irq 16 failed (0xc017744c)
//    0,
//    IRQF_ONESHOT,
//    IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
//    IRQF_ONESHOT | IRQF_TRIGGER_RISING,
//    IRQF_EARLY_RESUME | IRQF_TRIGGER_RISING,
//    IRQF_SHARED | IRQF_TRIGGER_RISING,
//    IRQF_SHARED | IRQF_TRIGGER_HIGH,
/*
 * These correspond to the IORESOURCE_IRQ_* defines in
 * linux/ioport.h to select the interrupt line behaviour.  When
 * requesting an interrupt without specifying a IRQF_TRIGGER, the
 * setting should be assumed to be "as already configured", which
 * may be as per machine or firmware initialisation.
#define IRQF_TRIGGER_NONE	0x00000000
#define IRQF_TRIGGER_RISING	0x00000001
#define IRQF_TRIGGER_FALLING	0x00000002
#define IRQF_TRIGGER_HIGH	0x00000004
#define IRQF_TRIGGER_LOW	0x00000008
#define IRQF_TRIGGER_MASK	(IRQF_TRIGGER_HIGH | IRQF_TRIGGER_LOW | \
				 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING)
#define IRQF_TRIGGER_PROBE	0x00000010
 */
/*
 * These flags used only by the kernel as part of the
 * irq handling routines.
 *
 * IRQF_SHARED - allow sharing the irq among several devices
 * IRQF_PROBE_SHARED - set by callers when they expect sharing mismatches to occur
 * IRQF_TIMER - Flag to mark this interrupt as timer interrupt
 * IRQF_PERCPU - Interrupt is per cpu
 * IRQF_NOBALANCING - Flag to exclude this interrupt from irq balancing
 * IRQF_IRQPOLL - Interrupt is used for polling (only the interrupt that is
 *                registered first in an shared interrupt is considered for
 *                performance reasons)
 * IRQF_ONESHOT - Interrupt is not reenabled after the hardirq handler finished.
 *                Used by threaded interrupts which need to keep the
 *                irq line disabled until the threaded handler has been run.
 * IRQF_NO_SUSPEND - Do not disable this IRQ during suspend.  Does not guarantee
 *                   that this interrupt will wake the system from a suspended
 *                   state.  See Documentation/power/suspend-and-interrupts.txt
 * IRQF_FORCE_RESUME - Force enable it on resume even if IRQF_NO_SUSPEND is set
 * IRQF_NO_THREAD - Interrupt cannot be threaded
 * IRQF_EARLY_RESUME - Resume IRQ early during syscore instead of at device
 *                resume time.
 * IRQF_COND_SUSPEND - If the IRQ is shared with a NO_SUSPEND user, execute this
 *                interrupt handler after suspending interrupts. For system
 *                wakeup devices users need to implement wakeup detection in
 *                their interrupt handlers.
#define IRQF_SHARED		0x00000080
#define IRQF_PROBE_SHARED	0x00000100
#define __IRQF_TIMER		0x00000200
#define IRQF_PERCPU		0x00000400
#define IRQF_NOBALANCING	0x00000800
#define IRQF_IRQPOLL		0x00001000
#define IRQF_ONESHOT		0x00002000
#define IRQF_NO_SUSPEND		0x00004000
#define IRQF_FORCE_RESUME	0x00008000
#define IRQF_NO_THREAD		0x00010000
#define IRQF_EARLY_RESUME	0x00020000
#define IRQF_COND_SUSPEND	0x00040000
*/
  devname, driver_object);
	printk("driver_object: %p\n", driver_object);
	if(_err){
		printk("request_irq failed with %d\n", _err);
//		gpio_free(p_gpionr);
		return -1;
	}
	printk("gpio %d successfull configured\n", p_gpionr);

//after request_threaded_irq()
	writel(INTERRUPT_PORT, __io_address(REG_PINT_MSK_SET)); //permits unmasking (enabling) of interrupt requests
	//writel(0x00000101 | readl(__io_address(REG_PINT_ASSIGN)), __io_address(REG_PINT_ASSIGN)); //default: 0x00000101
	writel(0x00000101, __io_address(REG_PINT_ASSIGN)); //default: 0x00000101
	writel(INTERRUPT_PORT, __io_address(REG_PINT_EDGE_SET)); //rising trigger
//	writel(INTERRUPT_PORT, __io_address(REG_PINT_LATCH)); //clear potential latches due to history:  //writing a 1 to the register clears the respective bits in the register
	writel(0xffffffff, __io_address(REG_PINT_LATCH)); //clear potential latches due to history:  //writing a 1 to the register clears the respective bits in the register
	return _rpi_irq;
}

static int driver_open(struct inode *device_file, struct file *instance){
	return 0;
}

static int driver_close(struct inode *device_file, struct file *instance){
	return 0;
}

static ssize_t driver_read(struct file *instance, char __user *user,
	size_t count, loff_t *offset){
	
//	size_t not_copied, to_copy;

	interrupt_arrived = 0;
	wait_event_interruptible(sleeping_for_ir, interrupt_arrived);	
//	to_copy = min(count, sizeof(interrupt_arrived));
//	not_copied = copy_to_user(user, &interrupt_arrived, to_copy);
//	return to_copy-not_copied;
	return 0;
}

static int sx127x_irqin_en(void){
        printk("\nInside sx127x.c: sx127x_irqin_en()!\n\n");
        writel(INTERRUPT_PORT, __io_address(REG_PINT_MSK_SET)); //permits unmasking (enabling) of interrupt requests
        writel(INTERRUPT_PORT, __io_address(REG_PORT_INEN_SET)); //enable input driver

        return 0;
}

static int sx127x_irqout_on(void){
        writel(GPO1_PORT, __io_address(REG_PORTF_DATA_SET));
        writel(GPO3_PORT, __io_address(REG_PORTB_DATA_SET));

        return 0;
}

static int sx127x_irqout_off(void){
        writel(GPO1_PORT, __io_address(REG_PORTF_DATA_CLR));
        writel(GPO3_PORT, __io_address(REG_PORTB_DATA_CLR));

        return 0;
}
 
static long sx127x_dev_ioctl(struct file *instance, unsigned int cmd, unsigned long arg){
	int ret;
	enum sx127x_ioctl_cmd ioctlcmd = cmd;
//	printk("\nInside sx127x.c: sx127x_dev_ioctl()!\n\n");
	switch(ioctlcmd){
          case SX127X_IOCTL_CMD_IRQOUTON:
            ret = sx127x_irqout_on();
            break;
          case SX127X_IOCTL_CMD_IRQOUTOFF:
            ret = sx127x_irqout_off();
            break;
          case SX127X_IOCTL_CMD_IRQINEN:
            ret = sx127x_irqin_en();
            break;
          default:
            ret = -EINVAL;
            break;
	}
	return ret;
}

static struct file_operations fops = {
	.owner   = THIS_MODULE,
	.read    = driver_read,
	.open    = driver_open,
	.release = driver_close,
  .unlocked_ioctl = sx127x_dev_ioctl
};

static int __init mod_init(void){
	dev_info(gpio_dev, "mod_init: %s\n", VERSION);
	init_waitqueue_head(&sleeping_for_ir);
	if(alloc_chrdev_region(&gpio_dev_number,0,1,MODULNAME)<0)
		return -EIO;
	driver_object = cdev_alloc();
	if(driver_object==NULL)
		goto free_device_number;
	driver_object->owner = THIS_MODULE;
	driver_object->ops = &fops;
	if(cdev_add(driver_object,gpio_dev_number,1))
		goto free_cdev;
	gpio_class = class_create(THIS_MODULE,MODULNAME);
	if(IS_ERR(gpio_class)){
		pr_err("%s: no udev support\n", MODULNAME);	
		goto free_cdev;
	}
	gpio_dev = device_create(gpio_class, NULL, gpio_dev_number,
		NULL, "%s",MODULNAME);
	if(IS_ERR(gpio_dev)){
		pr_err("%s: device_create() failed\n",MODULNAME);
		goto free_class;
	}
	rpi_irq_16 = config_gpio(INTERRUPT_NUM);
	if(rpi_irq_16<0){
		pr_err("%s: config_gpio() failed\n",MODULNAME);
		goto free_device;
	}
	writel(GPO1_PORT, __io_address(REG_PORTF_FER_CLR));
	writel(GPO1_PORT, __io_address(REG_PORTF_DIR_SET));
	writel(GPO3_PORT, __io_address(REG_PORTB_FER_CLR));
	writel(GPO3_PORT, __io_address(REG_PORTB_DIR_SET));
	return 0;
free_device:
	device_destroy(gpio_class, gpio_dev_number);
free_class:
	class_destroy(gpio_class);
free_cdev:
	cdev_del(driver_object);
	/*kobject_put(&driver_object->kobj);*/
free_device_number:
	unregister_chrdev_region(gpio_dev_number,1);
	return -EIO;
}

static void __exit mod_exit(void){
	dev_info(gpio_dev, "mod_exit");
	device_destroy(gpio_class, gpio_dev_number);
	class_destroy(gpio_class);
	cdev_del(driver_object);
	unregister_chrdev_region(gpio_dev_number,1);
	free_irq(rpi_irq_16, driver_object);
//	gpio_free(17);
	writel(INTERRUPT_PORT, __io_address(REG_PINT_MSK_CLR)); //permits masking (disabling) of interrupt requests
  writel(INTERRUPT_PORT, __io_address(REG_PORT_INEN_CLR)); //disable input driver
	return;
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(AUTHOR);
MODULE_DESCRIPTION("int-pin to irq example");
MODULE_VERSION(VERSION_NR);

