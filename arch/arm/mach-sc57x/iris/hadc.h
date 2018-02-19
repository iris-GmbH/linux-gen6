#ifndef ARM_MACH_SC57X_IRIS_HADC_H_
#define ARM_MACH_SC57X_IRIS_HADC_H_

#ifdef __KERNEL__
#include <asm/io.h> /* ioremap */
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#endif

#define SET(x,y) x |= (1 << y)
#define CLEAR(x,y) x &= ~(1<< y)

#define IOC_HADC_MAGIC 'k' // defines the magic number
#define IOCTL_TESTINPUT			_IO(IOC_HADC_MAGIC,		0)
#define READ_HADC_IOTCL			_IOR(IOC_HADC_MAGIC,	1, struct hadc0_data) //continous read needs start & stop
#define START_HADC				_IO(IOC_HADC_MAGIC,		2)
#define STOP_HADC				_IO(IOC_HADC_MAGIC,		3)
#define READ_HADC_IOTCL_SINGLE	_IOR(IOC_HADC_MAGIC,	4, struct hadc0_data) //start & stop automatically


#define HADC_CTRL_ENLS			13
#define HADC_CTRL_DOUTOREOCB	12
#define HADC_CTRL_FIXEDCNV		8 //8..11
#define HADC_CTRL_CONT			7
#define HADC_CTRL_FDIV			3 //3..6
#define HADC_CTRL_STARTCNV		2
#define HADC_CTRL_PD			1
#define HADC_CTRL_NRST			0

#endif /* ARM_MACH_SC57X_IRIS_HADC_H_ */
