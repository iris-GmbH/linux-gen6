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

#define HADC_IOC_MAGIC 		'k' // defines the magic number
#define HADC_IOCTL_TEST		_IO(HADC_IOC_MAGIC,		0)
#define HADC_START			_IO(HADC_IOC_MAGIC,		1)
#define HADC_STOP			_IO(HADC_IOC_MAGIC,		2)
#define HADC_READ_ALLCHANNEL_CONT			_IOR(HADC_IOC_MAGIC,	3, struct hadc0_data*) //continous read needs start & stop
#define HADC_READ_ALLCHANNEL_START_STOP		_IOR(HADC_IOC_MAGIC,	4, struct hadc0_data*) //start & stop automatically
#define HADC_READ_SINGLE_CHANNEL_START_STOP	_IOWR(HADC_IOC_MAGIC,	5, uint32_t*) //start & stop automatically, read single channel

#define MAX_HADC_CHANNEL		8

#define HADC_CTRL_ENLS			13
#define HADC_CTRL_DOUTOREOCB	12
#define HADC_CTRL_FIXEDCNV		8 //8..11
#define HADC_CTRL_CONT			7
#define HADC_CTRL_FDIV			3 //3..6
#define HADC_CTRL_STARTCNV		2
#define HADC_CTRL_PD			1
#define HADC_CTRL_NRST			0

struct hadc0_data{
	uint32_t data[8];
};

#endif /* ARM_MACH_SC57X_IRIS_HADC_H_ */
