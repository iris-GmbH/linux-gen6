/*
 * Analog Devices video capture driver
 *
 * Copyright (c) 2011 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
/**
 * @file
 * @brief Analog Devices video capture driver
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>
#include <media/blackfin/ppi.h>

#include <linux/dma-mapping.h>
#include <linux/dmapool.h>

#ifdef CONFIG_ARCH_HEADER_IN_MACH
#include <mach/dma.h>
#else
#include <asm/dma.h>
#endif

#define COMPATIBLE_DT_NAME	"iris,gen6-epc660"
#define CAPTURE_DRV_NAME        "epc660_capture"
#define MIN_NUM_BUF      	2
/**
 * @brief PIXEL_PER_CYCLE: With a WORDSIZE of 256, the DMA transfers 256 Bit = 32 Byte per cycle. 
 * @brief With a size of 16 Bit/Pixel = 2 Bytes/Pixel, the DMA transfers 32 Byte/Cycle * (1/2) Pixel/Byte = 16 Pixel/Cycle
 * @brief Corresponds with WDSIZE_256 (e.g. with WDSIZE_128, PIXEL_PER_CYCLE is 8)
 */
#define PIXEL_PER_CYCLE             16
#define DEBUG                       0
#define OLD_STUFF                   0
#define OLD_STUFF_OLD               0
#define NUMBER_PARTS_IN_DMA_STORAGE 1

struct imager_format {
	char *desc;
	u32 pixelformat;
	u32 mbus_code;
	int bpp; /* bits per pixel */
	int dlen; /* data length for ppi in bits */
	int channels; /* of how many interleaved sub-images is the format made of */
	int pixel_depth_bytes;
};

/**
 * @brief Points to memory that is allocated to be reacheable by the dma and holds an array of dma descriptors.
 * @brief dma_addr_t - Bus address type of allocated DMA buffers
 */ 
struct imager_dma_desc_list_item {
	dma_addr_t next_desc_addr;
	dma_addr_t start_addr;
	unsigned long cfg;
}__packed;

/**
 * @brief Basic struct for one buffer and the belonging DMA descriptors
 */
struct imager_buffer {
	struct vb2_v4l2_buffer vb; ///< Basic driver buffer, contains "struct vb2_buffer"
	struct imager_dma_desc_list_item *dma_desc; ///<  Defined locally
	dma_addr_t desc_dma_addr; ///< the dma reacheable address of the dma_desc
	struct list_head list;
};

/**
 * @brief routing information for each input
 */ 
struct imager_route {
	u32 input;
	u32 output;
};

struct capture_config {
	/* card name */
	const char *card_name;
	struct v4l2_input *inputs; ///< inputs available at the sub device. Instance of struct v4l2_input defined locally.
	int num_inputs; ///< number of inputs supported
	struct imager_route *routes; ///<  routing information for each input. Defined locally.
	/* i2c bus adapter no */
	int i2c_adapter_id;
	/* i2c subdevice board info */
	struct i2c_board_info board_info;
	/* ppi board info */
	const struct ppi_info *ppi_info;
	/* ppi control */
	unsigned long ppi_control;
	/* ppi interrupt mask */
	u32 int_mask;
	/* horizontal blanking pixels */
	int blank_pixels;
};


struct epc660_device {
	/* capture device instance */
	struct v4l2_device v4l2_dev;
	/* v4l2 control handler */
	struct v4l2_ctrl_handler ctrl_handler;
	/* device node data */
	struct video_device *video_dev;
	/* sub device instance */
	struct v4l2_subdev *sd;
	struct capture_config cfg; ///< capture config, defined locally 
	int dma_channel; ///< DMA channel
	/* ppi interface */
	struct ppi_if *ppi;
	/* current input */
	unsigned int cur_input;
	/* current selected standard */
	v4l2_std_id std;
	/* current selected dv_timings */
	struct v4l2_dv_timings dv_timings;
	/* used to store pixel format */
	struct v4l2_pix_format fmt;
	/* bits per pixel*/
	int bpp;
	/* data length for ppi in bits */
	int dlen;
	/* number of sub images */
	int pixel_channels;
	/* the size of each pixel in bytes */
	int pixel_depth_bytes;
	struct imager_format *sensor_formats; ///< used to store sensor supported formats, defined locally
	int num_sensor_formats; ///< number of sensor formats array
	/* buffer queue used in videobuf2 */
	struct vb2_queue buffer_queue;
	struct dma_pool *dma_pool; ///< something to allocate memory for dma usage from
	/* queue of filled frames */
	struct list_head dma_queue;
	struct dmasg dma_cfg_template; ///< a dma config holding information on how to paramererize the dma 
	/* used in videobuf2 callback */
	spinlock_t lock;
	/* used to access capture device */
	struct mutex mutex;
};

/**
 * @brief used to store sensor supported format
 * @brief corresponding with "static const struct epc660_datafmt epc660_monochrome_fmts[]" in epc660.c @JAHA ToDo: Change this?
 */
static const struct imager_format epc660_formats[] = {
	{
		.desc	     = "1DCS_12bpp",
		.pixelformat = V4L2_PIX_FMT_Y12,
		.mbus_code   = MEDIA_BUS_FMT_Y12_1X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 1, ///< One picture/plane */
		.pixel_depth_bytes = 2,
	},
	{
		.desc	     = "2DCS_12bpp",
		.pixelformat = v4l2_fourcc('2', 'D', 'C', 'S'),
		.mbus_code   = MEDIA_BUS_FMT_EPC660_2X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 2, ///< 2 pictures/planes */
		.pixel_depth_bytes = 4,
	},
	{
		.desc	     = "4DCS_12bpp",
		.pixelformat = v4l2_fourcc('4', 'D', 'C', 'S'),
		.mbus_code   = MEDIA_BUS_FMT_EPC660_4X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 4,   ///< 4 pictures/planes */
		.pixel_depth_bytes = 8,
	},
};
#define MAX_FMTS ARRAY_SIZE(epc660_formats)

static struct v4l2_input epc660_inputs[] = {
	{
		.index	= 0,
		.name	= CAPTURE_DRV_NAME,
		.type	= V4L2_INPUT_TYPE_CAMERA,
		.std	= V4L2_STD_UNKNOWN,
	},
};

static struct imager_route epc660_routes[] = {
	{
		.input  = 0,
		.output = 0,
	},
};

static irqreturn_t epc660_isr(int irq, void *dev_id);
static int epc660_start_transfering(struct epc660_device *epc660_dev,
				dma_addr_t descrAddr);
static void epc660_stop_transfering(struct epc660_device *epc660_dev);

/* Used by diff fcts */
static struct imager_buffer *vb2v4l2_to_imagerbuffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct imager_buffer, vb);
}

/* The queue is busy if there is a owner and you are not that owner. */
/* Used by epc660_streamon*/
static inline bool vb2_queue_is_busy(struct video_device *vdev, struct file *file)
{
	return vdev->queue->owner && vdev->queue->owner != file->private_data;
}

/* Used by epc660_probe */
static int epc660_init_sensor_formats(struct epc660_device *epc660_dev)
{

	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct imager_format *sf;
	unsigned int num_formats = 0;
	int i, j;

	while (!v4l2_subdev_call(epc660_dev->sd, pad,
				enum_mbus_code, NULL, &code)) {
		num_formats++;
		code.index++;
	}
	if (!num_formats)
		return -ENXIO;

	sf = kcalloc(num_formats, sizeof(*sf), GFP_KERNEL);
	if (!sf)
		return -ENOMEM;

	for (i = 0; i < num_formats; i++) {
		code.index = i;
		v4l2_subdev_call(epc660_dev->sd, pad,
				enum_mbus_code, NULL, &code);
		for (j = 0; j < MAX_FMTS; j++)
			if (code.code == epc660_formats[j].mbus_code)
				break;
		if (j == MAX_FMTS) {
			/* we don't allow this sensor working with our bridge */
			kfree(sf);
			return -EINVAL;
		}
		sf[i] = epc660_formats[j];
	}

	epc660_dev->sensor_formats = sf;
	epc660_dev->num_sensor_formats = num_formats;
	return 0;
}

/* Used by epc660_remove */
static void epc660_free_sensor_formats(struct epc660_device *epc660_dev)
{
	epc660_dev->num_sensor_formats = 0;
	kfree(epc660_dev->sensor_formats);
	epc660_dev->sensor_formats = NULL;
}

static int epc660_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vq);

	#if DEBUG
    printk(KERN_INFO "#### epc660_queue_setup: num_buffers: %d *nbuffers: %d\n", vq->num_buffers, *nbuffers);
	#endif /*DEBUG*/

	if (vq->num_buffers + *nbuffers < MIN_NUM_BUF)
		*nbuffers = MIN_NUM_BUF;

	if (*nplanes)
		return sizes[0] < epc660_dev->fmt.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = epc660_dev->fmt.sizeimage;

	return 0;
}

/**
 * @brief This function initializes and sets the DMA descriptor list/array for the given vb2-buffer
 * @param *vb - pointer onto allocated buffer
 * @return "0" for OK 
 */
static int epc660_buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	int dmaDescArrCount;
    #if OLD_STUFF
	int i, channel;
	dma_addr_t start_addr, channelStartAddr, nextDescrDMAAddr;
	int halfImageSizeBytes, rowSizeBytes;
    #else
    int channel;
    dma_addr_t start_addr, nextDescrDMAAddr;
    #endif /*OLD_STUFF*/
	struct imager_dma_desc_list_item *dma_desc;

	#if DEBUG
//	printk(KERN_INFO "#### epc660_buffer_init\n");
	#endif /*DEBUG*/

	INIT_LIST_HEAD(&buf->list);

    /* The DMA-Pool is organized as a list, with each buffer_init a new DMA allocation;
    then a new element is added to epc660_dev->dma_pool*/
	buf->dma_desc = dma_pool_alloc(epc660_dev->dma_pool, GFP_KERNEL, &buf->desc_dma_addr);
	if (!buf->dma_desc) {
		printk("cannot allocate memory from dma pool\n");
		return -ENOMEM;
	}

    #if OLD_STUFF
    #if OLD_STUFF_OLD
	dmaDescArrCount = epc660_dev->pixel_channels * epc660_dev->fmt.height;
    #else
	dmaDescArrCount = epc660_dev->pixel_channels * NUMBER_PARTS_IN_DMA_STORAGE;
    #endif /*OLD_STUFF_OLD*/

	printk(KERN_INFO "##buffer_init## pix-chan: %d fmt-height: %d\n", epc660_dev->pixel_channels, epc660_dev->fmt.height);
    /* With every call of this function, with a new DMA allocation a new start address is evaluated */
	start_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	rowSizeBytes       = epc660_dev->fmt.width * epc660_dev->pixel_depth_bytes;
	halfImageSizeBytes = rowSizeBytes * epc660_dev->fmt.height / 2;
	printk(KERN_INFO "##buffer_init## fmt-width: %d pix-dpth: %d halfImageSz: %d\n", epc660_dev->fmt.width, epc660_dev->pixel_depth_bytes, halfImageSizeBytes);

	dma_desc = buf->dma_desc;
	nextDescrDMAAddr = buf->desc_dma_addr + sizeof(*dma_desc);
	for (channel = 0; channel < epc660_dev->pixel_channels; ++channel) {
		channelStartAddr = start_addr + channel * ((epc660_dev->bpp + 7) / 8) * PIXEL_PER_CYCLE; // * the DMA transfers PIXEL_PER_CYCLE / per wordsize
        #if OLD_STUFF_OLD
		for (i = 0; i < epc660_dev->fmt.height/2; i += 1) {
			/* bottom half */
			dma_desc->next_desc_addr = nextDescrDMAAddr;
			dma_desc->start_addr = channelStartAddr + halfImageSizeBytes + i * rowSizeBytes;
			dma_desc->cfg = epc660_dev->dma_cfg_template.cfg;
			++dma_desc;
			nextDescrDMAAddr += sizeof(*dma_desc);

			/* top half */
			dma_desc->next_desc_addr = nextDescrDMAAddr;
			dma_desc->start_addr = channelStartAddr + halfImageSizeBytes - ((i+1) * rowSizeBytes);
			dma_desc->cfg = epc660_dev->dma_cfg_template.cfg;
			++dma_desc;
			nextDescrDMAAddr += sizeof(*dma_desc);
		}
        #else
		for (i = 0; i < NUMBER_PARTS_IN_DMA_STORAGE; i += 1) {

			dma_desc->next_desc_addr = nextDescrDMAAddr;
            /*Complete picture geteilt durch die Anzahl seiner Teile */
			dma_desc->start_addr = channelStartAddr + (halfImageSizeBytes*2*i)/NUMBER_PARTS_IN_DMA_STORAGE ;
			dma_desc->cfg = epc660_dev->dma_cfg_template.cfg;
			++dma_desc;
			nextDescrDMAAddr += sizeof(*dma_desc);

		}
        #endif /*OLD_STUFF_OLD*/
	}
	/* copy the descriptor config on the first transfer */
	buf->dma_desc->cfg |= DESCIDCPY; /*<0x02000000>*/

	/* the very last element must be a list element that makes the dma point to the beginning */
	(dma_desc-1)->next_desc_addr = buf->desc_dma_addr;
	/* the last transfer shall generate an interrupt */
	(dma_desc-1)->cfg |= DI_EN_X; /*0x00100000*/

    #else
    dmaDescArrCount = epc660_dev->pixel_channels;
	#if DEBUG
	printk(KERN_INFO "##buffer_init## fmt-width: %d fmt-height: %d pix-dpth: %d\n", epc660_dev->fmt.width, epc660_dev->fmt.height, epc660_dev->pixel_depth_bytes);
	#endif /*DEBUG*/
    /* With every call of this function, with a new DMA allocation a new start address is evaluated */
    start_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	dma_desc = buf->dma_desc;
	nextDescrDMAAddr = buf->desc_dma_addr + sizeof(*dma_desc);
	for (channel = 0; channel < epc660_dev->pixel_channels; ++channel) {
        /* ((epc660_dev->bpp + 7) / 8): We want the minimum necessary size of Bytes for a given number of bits.
        Maximum pixel line length (fmt.width) and the complete line number of one picture (fmt.height): buffer for one channel/plane/picture*/
    	dma_desc->start_addr = start_addr + channel * ((epc660_dev->bpp + 7) / 8) * epc660_dev->fmt.width * epc660_dev->fmt.height;
    	printk("##buffer_init##  start_addr %x; channel %d; dma_desc-start_addr: %x; bpp: %d\n", start_addr, channel, dma_desc->start_addr, epc660_dev->bpp );
		dma_desc->next_desc_addr = nextDescrDMAAddr;
        dma_desc->cfg = epc660_dev->dma_cfg_template.cfg;
		++dma_desc;
    	nextDescrDMAAddr += sizeof(*dma_desc);
    }
	/* copy the descriptor config on the first transfer */
	buf->dma_desc->cfg |= DESCIDCPY; /*<0x02000000>*/
    /* If channel==1, the next two lines work on the one and only DMA descriptor element and overwrite some for-loop settings*/
	/* the very last element must be a list element that makes the dma point to the beginning */
	(dma_desc-1)->next_desc_addr = buf->desc_dma_addr;
	/* the last transfer shall generate an interrupt */
	(dma_desc-1)->cfg |= DI_EN_X; /*0x00100000*/
    #endif /* OLD_STUFF */

	// print the descriptors
    #if DEBUG
    {
    int i;
//	dma_desc = buf->dma_desc;
	printk("buf->desc_dma_addr %08x\n", buf->desc_dma_addr);
	for (i = 0; i < dmaDescArrCount; ++i) {
		printk("dma_descriptor %d:"
				"\n\t next_desc_addr %x"
				"\n\t start_addr %x"
				"\n\t cfg %08lx\n",
				i,
				buf->dma_desc[i].next_desc_addr,
				buf->dma_desc[i].start_addr,
				buf->dma_desc[i].cfg
			);
	}
    }
	#endif /*DEBUG*/
	return 0;
}

/**
 * @brief This function checks if the given vb2-buffer is correctly dimensioned
 * @param *vb - pointer onto allocated buffer
 * @return "0" for OK 
 */
static int epc660_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = epc660_dev->fmt.sizeimage;

	#if DEBUG
	printk(KERN_INFO "#### epc660_buffer_prepare\n");
	#endif /*DEBUG*/

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&epc660_dev->v4l2_dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);

	vbuf->field = epc660_dev->fmt.field;

	return 0;
}

/**
 * @brief This function adds the given vb2-buffer to the descriptor queue. If the list isn't empty, the last element is added.
 * @param *vb - pointer onto allocated buffer
 * @return "0" for OK 
 */
static void epc660_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	unsigned long flags;
	int last_dma_desc_idx;

	#if DEBUG
//	printk(KERN_INFO "#### epc660_buffer_queue\n");
	#endif /*DEBUG*/
    #if OLD_STUFF 
    #if OLD_STUFF_OLD
	last_dma_desc_idx = epc660_dev->pixel_channels * epc660_dev->fmt.height - 1;
    #else
	last_dma_desc_idx = epc660_dev->pixel_channels * NUMBER_PARTS_IN_DMA_STORAGE - 1;
    #endif /* OLD_STUFF_OLD*/
    #else
	last_dma_desc_idx = epc660_dev->pixel_channels - 1;
    #endif /*OLD_STUFF*/
    buf->dma_desc[last_dma_desc_idx].next_desc_addr = buf->desc_dma_addr;
	#if DEBUG
	printk(KERN_INFO "#### epc660_buffer_queue: last_dma_desc_idx: %d desc_dma_addr: %x\n", last_dma_desc_idx, buf->desc_dma_addr);
	#endif /*DEBUG*/
	spin_lock_irqsave(&epc660_dev->lock, flags);

	// setup the dma descriptor
	if (!list_empty(&epc660_dev->dma_queue)) {
		struct imager_buffer* lastBuffer;
		lastBuffer = list_last_entry(&epc660_dev->dma_queue, struct imager_buffer, list);
		lastBuffer->dma_desc[last_dma_desc_idx].next_desc_addr = buf->desc_dma_addr;
	    #if DEBUG
    	printk(KERN_INFO "#### epc660_buffer_queue: !list_empty\n");
    	#endif /*DEBUG*/
	}


	#if DEBUG
//	{
//		struct list_head *pos;
//		int listSize = 0;
//		list_for_each(pos, &epc660_dev->dma_queue)
//		{
//			++listSize;
//		}
//		printk("listsize: %d\n", listSize);
//	}
	#endif /*DEBUG*/
    
	list_add_tail(&buf->list, &epc660_dev->dma_queue);

	spin_unlock_irqrestore(&epc660_dev->lock, flags);
}

static void epc660_buffer_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	unsigned long flags;

	#if DEBUG
	printk(KERN_INFO "#### epc660_buffer_cleanup\n");
	#endif /*DEBUG*/

	spin_lock_irqsave(&epc660_dev->lock, flags);
	list_del_init(&buf->list);
	spin_unlock_irqrestore(&epc660_dev->lock, flags);
	dma_pool_free(epc660_dev->dma_pool, buf->dma_desc, buf->desc_dma_addr);
}

/**
 * @brief The PPI params are defined and set in the ppi struct in epc660_dev. 
 * @param *vb - pointer onto queue buffer. count - unused variable.
 * @return "0" for OK 
 */
static int epc660_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = epc660_dev->ppi;
	struct ppi_params params;
	int ret;

	#if DEBUG
	printk(KERN_INFO "#### epc660_start_streaming\n");
	#endif /*DEBUG*/

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(epc660_dev->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&epc660_dev->v4l2_dev, "stream on failed in subdev\n");
		return ret;
	}

	/* set ppi params */
    /* As fmt.width==160, the length of the whole line is multipied with 2 (==320), as each Pixel contains of 2 Byte;
    As fmt.height==240, for the half picture filled it is divided by 2 (==120) */
	params.width	   = epc660_dev->fmt.width * 2;
	params.height	   = epc660_dev->fmt.height / 2;
	params.bpp	       = epc660_dev->bpp; /* 16 */
	params.dlen 	   = epc660_dev->dlen; /* 12 Bits of the 16 Bits are relevant (data word length)*/
	params.ppi_control = (EPPI_CTL_DLEN12	   |  /* Data Word Length: 12 bit */
			      EPPI_CTL_NON656	   |  /* XFRTYPE: Non-ITU656 Mode (GP Mode) */
			      EPPI_CTL_SYNC2	   |  /* 2 external frame syncs */
			      EPPI_CTL_FS1LO_FS2LO |  /* FS1 and FS2 are active low (FS - frame sync) */
			      EPPI_CTL_POLC0	   |  /* sample on falling DCLK */
			      EPPI_CTL_PACKEN	   |  /* assemble two incomming 16Bit words into one 32Bit word (reduces RAM-load a lot) */
			      EPPI_CTL_SIGNEXT);
	params.int_mask	   = 0xfc;
	params.hdelay	   = 0;
	params.vdelay	   = 0;
	params.line	   = params.width;
	params.frame	   = params.height;

	ret = ppi->ops->set_params(ppi, &params);

	if (ret < 0) {
		v4l2_err(&epc660_dev->v4l2_dev,
				"Error in setting ppi params\n");
		return ret;
	}
	return 0;
}

static void epc660_stop_streaming(struct vb2_queue *vq)
{
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vq);
	int ret;
	unsigned long flags;

	#if DEBUG
	printk(KERN_INFO "#### epc660_stop_streaming\n");
	#endif /*DEBUG*/

	spin_lock_irqsave(&epc660_dev->lock, flags);
	epc660_stop_transfering(epc660_dev);


	ret = v4l2_subdev_call(epc660_dev->sd, video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&epc660_dev->v4l2_dev,
				"stream off failed in subdev\n");

	/* release all active buffers */
	while (!list_empty(&epc660_dev->dma_queue)) {
		struct imager_buffer *buf = list_entry(epc660_dev->dma_queue.next,
						struct imager_buffer, list);
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&epc660_dev->lock, flags);
}

static struct vb2_ops epc660_video_qops =
{
	.queue_setup            = epc660_queue_setup, /* Call in Startup-Phase */
	.buf_init               = epc660_buffer_init, /* Call in Startup-Phase */
	.buf_prepare            = epc660_buffer_prepare,/* Call in all three Phases */
	.buf_cleanup            = epc660_buffer_cleanup,/* Call in Shutdown-Phase*/
	.buf_queue              = epc660_buffer_queue, /* Call in all three Phases */
	.wait_prepare           = vb2_ops_wait_prepare, /* Keine Anwendung */
	.wait_finish            = vb2_ops_wait_finish, /* Keine Anwendung */
	.start_streaming        = epc660_start_streaming,/* Call in Startup-Phase */
	.stop_streaming         = epc660_stop_streaming,/* Call in Shutdown-Phase */
};

#if DEBUG
static void printDMAState(struct epc660_device *epc660_dev)
{
	printk("dma config:"
			"\n\t next_desc_ptr %p"
			"\n\t start_addr %lx"
			"\n\t cfg %08lx"
			"\n\t x_count %lu"
			"\n\t x_modify %ld"
			"\n\t y_count %lu"
			"\n\t y_modify %ld"
			"\n\t curr_desc_ptr %p"
			"\n\t prev_desc_ptr %p"
			"\n\t curr_addr_ptr %08lx"
			"\n\t irq_status %08lx"
			"\n\t curr_x_count %ld"
			"\n\t curr_y_count %ld\n",

			get_dma_next_desc_ptr(epc660_dev->dma_channel),
			get_dma_start_addr(epc660_dev->dma_channel),
			get_dma_config(epc660_dev->dma_channel),
			get_dma_x_count(epc660_dev->dma_channel),
			get_dma_x_modify(epc660_dev->dma_channel),
			get_dma_y_count(epc660_dev->dma_channel),
			get_dma_y_modify(epc660_dev->dma_channel),
			get_dma_curr_desc_ptr(epc660_dev->dma_channel),
			get_dma_prev_desc_ptr(epc660_dev->dma_channel),
			get_dma_curr_addr(epc660_dev->dma_channel),
			get_dma_curr_irqstat(epc660_dev->dma_channel),
			get_dma_curr_xcount(epc660_dev->dma_channel),
			get_dma_curr_ycount(epc660_dev->dma_channel));
}
#endif

static irqreturn_t epc660_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	struct epc660_device *epc660_dev = ppi->priv;
	int dmaStatus;
	dma_addr_t lastDmaDescriptor;
	struct list_head* iterator;

	#if DEBUG
//	printk("epc660_isr\n");
//	printk(KERN_INFO "#### epc660_isr\n");
	#endif /*DEBUG*/

	spin_lock(&epc660_dev->lock);

	dmaStatus = get_dma_curr_irqstat(epc660_dev->dma_channel);
	clear_dma_irqstat(epc660_dev->dma_channel);

	#if DEBUG
	printk("## isr ## dmaStatus: 0x%08x\n", dmaStatus);
	#endif /*DEBUG*/
	if (dmaStatus & DMA_DONE) {
		// if there are at least two buffers in the queue we can deque one
		if (&epc660_dev->dma_queue == epc660_dev->dma_queue.next->next) {
	        #if DEBUG
			printk("buffer underrun in epc660 capture\n");
//			epc660_stop_transfering(epc660_dev);
        	#endif /*DEBUG*/
		} else {
			/* publish all buffers that are done */
			int completedCnt = 0;
			lastDmaDescriptor = (dma_addr_t)get_dma_prev_desc_ptr(epc660_dev->dma_channel);
			lastDmaDescriptor &= ~1; // the lowest bit masks when a decriptor fetch was invalid
			list_for_each(iterator, &epc660_dev->dma_queue) {
				struct imager_buffer *buf = list_entry(iterator, struct imager_buffer, list);
				if (buf->desc_dma_addr == lastDmaDescriptor) {
					++completedCnt;
					break;
				}
			}
           	#if DEBUG
			printk("lastDmaDescriptor: 0x%08x\n", lastDmaDescriptor);
           	#endif /*DEBUG*/
			if (0 == completedCnt) {
				printk("cannot find any completed buffers!\n");
            	#if DEBUG
//				epc660_stop_transfering(epc660_dev);
            	#endif /*DEBUG*/
			} else {
				struct imager_buffer* buf;
				struct vb2_buffer *vb;
            	#if DEBUG
				printk("found %d completed buffers\n", completedCnt);
            	#endif /*DEBUG*/
				while (completedCnt--) {
					if (&epc660_dev->dma_queue == epc660_dev->dma_queue.next->next) {
						break;
					}
					buf = list_entry(epc660_dev->dma_queue.next,
							struct imager_buffer, list);
					vb = &buf->vb.vb2_buf;
					vb->timestamp = ktime_get_ns(); // this has been changed from type struct timeval to -> u64 type
					if (ppi->err) {
						vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
					} else {
						vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
					}
					list_del_init(&buf->list);
				}
			}
		}
		/* clear error flag for the next frame */
		ppi->err = false;
	}

	spin_unlock(&epc660_dev->lock);

	return IRQ_HANDLED;
}

/**
 * @brief Requests the DMA channel. Sets the configuration registers and configures PPI as callback functionality.
 * @brief Starts the PPI and the DMA instance (the DMA by setting the config register and with it the DMA_ENABLE bit). 
 * @brief Used in epc660_streamon.
 * @param epc660_dev - Actual driver device structure
 * @param descrAddr - start address of the (first) DMA descriptor
 * @return ret - result of DMA allocation fct "request_dma", "0" for OK 
 */
static int epc660_start_transfering(struct epc660_device *epc660_dev, dma_addr_t descrAddr) {
	int ret;
	//printk("start DMA\n");

	ret = request_dma(epc660_dev->dma_channel, "EPC660_dma");
	if (ret) {
		printk("Unable to allocate DMA channel\n");
		return ret;
	}

	/* attach ppi DMA irq handler */
	set_dma_callback(epc660_dev->dma_channel, epc660_isr, epc660_dev->ppi);
    /* Sets the different HW registers of the DMA on chip */
	set_dma_next_desc_addr(epc660_dev->dma_channel, (void*)descrAddr);
	set_dma_x_count(epc660_dev->dma_channel, epc660_dev->dma_cfg_template.x_count),
	set_dma_x_modify(epc660_dev->dma_channel, epc660_dev->dma_cfg_template.x_modify),
	set_dma_config(epc660_dev->dma_channel, epc660_dev->dma_cfg_template.cfg);

	/* enable ppi */
	epc660_dev->ppi->ops->start(epc660_dev->ppi);

	return ret;
}

/* Used in epc660_stop_streaming*/
static void epc660_stop_transfering(struct epc660_device *epc660_dev)
{
	/* disable ppi */
	epc660_dev->ppi->ops->stop(epc660_dev->ppi);

	dma_disable_irq(epc660_dev->dma_channel);
	disable_dma(epc660_dev->dma_channel);

	clear_dma_irqstat(epc660_dev->dma_channel);

	free_dma(epc660_dev->dma_channel);
	//printk("stopped DMA\n");
}

/**
 * @brief 
 * @param *file -file descriptor of sub device
 * @param *
 * @return "0" for OK 
 */
static int epc660_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct vb2_queue *vq = &epc660_dev->buffer_queue;
	unsigned long flags;
	int ret;
	struct imager_buffer* buf;

	#if DEBUG
	printk(KERN_INFO "#### epc660_streamon\n");
	#endif /*DEBUG*/

	if (vb2_queue_is_busy(epc660_dev->video_dev, file))
		return -EBUSY;

	/* call streamon to start streaming in videobuf */
	ret = vb2_streamon(vq, buf_type);
	if (ret)
		return ret;

	/* if dma queue is empty, return error */
	if (list_empty(&epc660_dev->dma_queue)) {
		v4l2_err(&epc660_dev->v4l2_dev, "dma queue is empty\n");
		ret = -EINVAL;
		goto err;
	}

	spin_lock_irqsave(&epc660_dev->lock, flags);

	/* get the next frame from the dma queue */
	buf = list_entry(epc660_dev->dma_queue.next,
			struct imager_buffer, list);
	epc660_start_transfering(epc660_dev, buf->desc_dma_addr);
	spin_unlock_irqrestore(&epc660_dev->lock, flags);

	return 0;
err:	
	vb2_streamoff(vq, buf_type);
	return ret;
}

/**
 * @brief The function delivers the input instance according to the number which is set in struct input
 * @param *file -file descriptor of sub device
 * @param *input - instance to be filled with data according to the index value
 * @return "0" for OK 
 */
static int epc660_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct capture_config *config = &epc660_dev->cfg;
    #if 0
	int ret;
	u32 status;
    #endif
	#if DEBUG
	printk(KERN_INFO "#### epc660_enum_input\n");
	#endif /*DEBUG*/

	/* Compare the input value with the value in the driver */
	if (input->index >= config->num_inputs)
		return -EINVAL;
	/* set the input value for the required input instance */
	*input = config->inputs[input->index];

	/* As "get input status" (struct v4l2_subdev_video_ops, epc660.c) isn't implemented for EPC660 drivers,
	this call always returns -ENOIOCTLCMD (see #define v4l2_subdev_call(...) ) */
	#if 0
	ret = v4l2_subdev_call(epc660_dev->sd, video, g_input_status, &status);
	if (!ret)
		input->status = status;
	#endif	
	return 0;
}


/**
 * @brief This function sets "epc660_dev->cur_input" with the value "index" given from application
 * @param *file -file descriptor of sub device
 * @param *input - instance to be filled with data according to the index value
 * @return "0" for OK 
 */
static int epc660_s_input(struct file *file, void *priv, unsigned int index)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct vb2_queue *vq = &epc660_dev->buffer_queue;
	struct capture_config *config = &epc660_dev->cfg;
	#if 0
    struct imager_route *route;
	int ret;
    #endif
	#if DEBUG
	printk(KERN_INFO "#### epc660_s_input\n");
	#endif /*DEBUG*/

	if (vb2_is_busy(vq))
		return -EBUSY;

	if (index >= config->num_inputs)
		return -EINVAL;

	/* As "s_routing" (struct v4l2_subdev_video_ops, epc660.c) isn't implemented for EPC660 drivers, 
	this call always returns -ENOIOCTLCMD (see #define v4l2_subdev_call(...) ) 
	(tested with (ret == -ENOIOCTLCMD))*/
	#if 0
	route = &config->routes[index];
	ret = v4l2_subdev_call(epc660_dev->sd, video, s_routing, route->input,
			route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&epc660_dev->v4l2_dev, "Failed to set input\n");
		return ret;
	}
	#endif

	epc660_dev->cur_input = index;
	return 0;
}

/**
 * @brief This function tries if the format works. Called by epc660_s_fmt_vid_cap. 
 * @param *epc660_dev - Some values extracted for the other two structs
 * @param *pixfmt - Some values set in the function
 * @param *epc660_fmt - Some values set in the function
 * @return "0" for OK 
 */
static int epc660_try_format(struct epc660_device *epc660_dev,
			     struct v4l2_pix_format *pixfmt,
			     struct imager_format *epc660_fmt)
{
	struct imager_format *sf = epc660_dev->sensor_formats;
	struct imager_format *fmt = NULL;
	struct v4l2_subdev_pad_config pad_cfg;///< see comment in include/media/v4l2-subdev.h
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	int ret, i;

	for (i = 0; i < epc660_dev->num_sensor_formats; i++) {
		fmt = &sf[i];
		if (pixfmt->pixelformat == fmt->pixelformat)
			break;
	}
	if (i == epc660_dev->num_sensor_formats)
		fmt = &sf[0];

	v4l2_fill_mbus_format(&format.format, pixfmt, fmt->mbus_code);
	ret = v4l2_subdev_call(epc660_dev->sd, pad, set_fmt, &pad_cfg,
				&format);
	if (ret < 0)
		return ret;
	v4l2_fill_pix_format(pixfmt, &format.format);
	if (epc660_fmt) {
		for (i = 0; i < epc660_dev->num_sensor_formats; i++) {
			fmt = &sf[i];
			if (format.format.code == fmt->mbus_code)
				break;
		}
		*epc660_fmt = *fmt;
	}

	pixfmt->bytesperline = pixfmt->width * epc660_fmt->pixel_depth_bytes;
	pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
	return 0;
}

/* this function isn't called but necessary for kernel function (v4l2_ioctl_ops) */
static int epc660_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	#if DEBUG
	printk(KERN_INFO "#### epc660_g_fmt_vid_cap\n");
	#endif /*DEBUG*/

	fmt->fmt.pix = epc660_dev->fmt;
	return 0;
}

/**
 * @brief Sets the sensor values of *fmt for Application, and of epc660_dev. Initial DMA configuration.
 * @param *fmt: Representation of the platform device in the kernel to be set in this function
 * @return "0" for OK 
 */
static int epc660_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct vb2_queue *vq = &epc660_dev->buffer_queue;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct imager_format epc660_fmt;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int dmaPoolMemorySize;
	int ret;

	#if DEBUG
	//printk(KERN_INFO "#### epc660_s_fmt_vid_cap\n");
	#endif /*DEBUG*/

	if (vb2_is_busy(vq))
		return -EBUSY;

	/* see if format works */
	ret = epc660_try_format(epc660_dev, pixfmt, &epc660_fmt);
	if (ret < 0)
		return ret;

    /* Fills format.format for usage in v4l2_subdev_call-->set_fmt */
	v4l2_fill_mbus_format(&format.format, pixfmt, epc660_fmt.mbus_code);
    /* Sets the format parameter in the epc660.c I2C- Driver */
	ret = v4l2_subdev_call(epc660_dev->sd, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;

    /* Store the parameter from application into epc660_dev module struct */ 
	epc660_dev->fmt               = *pixfmt;
	epc660_dev->bpp               = epc660_fmt.bpp;
	epc660_dev->dlen              = epc660_fmt.dlen;
	epc660_dev->pixel_channels    = epc660_fmt.channels;
	/* align the pixels to 4 bytes */
	epc660_dev->pixel_depth_bytes = epc660_fmt.pixel_depth_bytes;

	memset(&epc660_dev->dma_cfg_template, 0, sizeof(epc660_dev->dma_cfg_template));
	epc660_dev->dma_cfg_template.cfg      = RESTART |                       ///< DMA Buffer Clear SYNC <0x00000004>
						DMATOVEN |                      ///< DMA Trigger Overrun Error Enable <0x01000000>
						WNR |                           ///< Channel Direction (W/R*) <0x00000002>
						WDSIZE_256 |                    ///< Transfer Word Size 256 Bit <0x00000500> --> Corresponds to PIXEL_PER_CYCLE
						PSIZE_32 |                      ///< Peripheral Transfer Word Size 32 Bit = 4 Byte <0x00000020>
						NDSIZE_2 |                      ///< Next Descriptor Size = 3 <0x00020000>
						DMAFLOW_LIST |                  ///< Descriptor List Mode <0x00004000>
						DMAEN;                          ///< DMA Channel Enable <0x00000001>
    /* After this setting, the cfg-value for the DMA has got the following value: 0x01024527.
    As DMA2D is not set, the DMA mode is 1D automatically.  */

    #if OLD_STUFF
    #if OLD_STUFF_OLD
	epc660_dev->dma_cfg_template.x_count  = epc660_dev->fmt.width / PIXEL_PER_CYCLE; ///< The DMA shall throw an interrupt when one line is read into storage
	epc660_dev->dma_cfg_template.x_modify = epc660_dev->pixel_depth_bytes * PIXEL_PER_CYCLE;// After 32 Bytes = 256 Bit WORDSIZE
	#else
    epc660_dev->dma_cfg_template.x_count  = epc660_dev->fmt.width * (epc660_dev->fmt.height/NUMBER_PARTS_IN_DMA_STORAGE) / PIXEL_PER_CYCLE; ///< The DMA shall throw an interrupt when one line is read into storage
	epc660_dev->dma_cfg_template.x_modify = epc660_dev->pixel_depth_bytes * PIXEL_PER_CYCLE;// After 32 Bytes = 256 Bit WORDSIZE
	#endif /*OLD_STUFF_OLD*/
    #else
    epc660_dev->dma_cfg_template.x_count  = epc660_dev->fmt.width * (epc660_dev->fmt.height) / PIXEL_PER_CYCLE; ///< The DMA shall throw an interrupt when the whole channel picture is read into storage
	epc660_dev->dma_cfg_template.x_modify = epc660_dev->pixel_depth_bytes * PIXEL_PER_CYCLE;// After 32 Bytes = 256 Bit WORDSIZE
    #endif /*OLD_STUFF*/
	#if DEBUG
    printk("#### epc660_s_fmt_vid_cap: x_count %ld, x_modify %ld\n", epc660_dev->dma_cfg_template.x_count, epc660_dev->dma_cfg_template.x_modify);
	#endif /*DEBUG*/

	if (epc660_dev->dma_pool) {
		dma_pool_destroy(epc660_dev->dma_pool);
	}

    #if OLD_STUFF
    #if OLD_STUFF_OLD
	dmaPoolMemorySize = (epc660_dev->pixel_channels * epc660_dev->fmt.height) * sizeof(struct imager_dma_desc_list_item);
    #else
   	dmaPoolMemorySize = (epc660_dev->pixel_channels * NUMBER_PARTS_IN_DMA_STORAGE) * sizeof(struct imager_dma_desc_list_item);
    #endif /*OLD_STUFF_OLD*/
    #else
	dmaPoolMemorySize = (epc660_dev->pixel_channels ) * sizeof(struct imager_dma_desc_list_item);
    #endif /*OLD_STUFF*/
	epc660_dev->dma_pool = dma_pool_create(CAPTURE_DRV_NAME, epc660_dev->v4l2_dev.dev,  dmaPoolMemorySize, 16, 0);
	return 0;
}

static int epc660_log_status(struct file *file, void *priv)
{
#if DEBUG
	struct epc660_device *epc660_dev = video_drvdata(file);
	printk(KERN_INFO "#### epc660_log_status\n");

	/* status for sub devices */
	v4l2_device_call_all(&epc660_dev->v4l2_dev, 0, core, log_status);
#endif /*DEBUG*/
	return 0;
}

static const struct v4l2_ioctl_ops epc660_ioctl_ops =
{
	.vidioc_g_fmt_vid_cap    = epc660_g_fmt_vid_cap, ///< No official usage, but driver doesn't run without the function implemented
	.vidioc_s_fmt_vid_cap    = epc660_s_fmt_vid_cap, ///< Call during start phase by EPC660_Imager.cpp
	.vidioc_enum_input       = epc660_enum_input, ///< Call during start phase by EPC660_Imager.cpp
	.vidioc_s_input          = epc660_s_input, ///< Call during start phase by  EPC660_Imager.cpp
	.vidioc_reqbufs          = vb2_ioctl_reqbufs, ///< Call during start phase by EPC660_Imager.cpp
	.vidioc_querybuf         = vb2_ioctl_querybuf, ///< Call in EPC660_Imager.cpp for V4L2_MEMORY_MMAP
	.vidioc_qbuf             = vb2_ioctl_qbuf, ///< Call in EPC660_Imager.cpp during all three phases
	.vidioc_dqbuf            = vb2_ioctl_dqbuf, ///< Call during all three phases
	.vidioc_streamon         = epc660_streamon, ///< Call during start phase by EPC660_Imager.cpp
	.vidioc_log_status       = epc660_log_status,
};


static int epc660_open(struct file* filp) {
	int ret;
	struct epc660_device *epc660_dev = video_drvdata(filp);
	#if DEBUG
	printk(KERN_INFO "#### epc660_open\n");
	#endif /*DEBUG*/

	if (!epc660_dev) {
		return -EINVAL;
	}

	ret = v4l2_subdev_call(epc660_dev->sd, core, load_fw);
	if (ret) {
		return ret;
	}
	ret = v4l2_fh_open(filp);
	return ret;
}
static int epc660_release(struct file* filp) {
	int ret;
	struct epc660_device *epc660_dev = video_drvdata(filp);
	#if DEBUG
	printk(KERN_INFO "#### epc660_release\n");
	#endif /*DEBUG*/

	if (!epc660_dev) {
		return -EINVAL;
	}

	ret = vb2_fop_release(filp);
	if (ret) {
		return ret;
	}
	// make the device enter reset
	ret = v4l2_subdev_call(epc660_dev->sd, core, reset, 1);
	return ret;
}

static struct v4l2_file_operations epc660_fops = {
	.owner = THIS_MODULE,
	.open = epc660_open, ///< Call during start phase
	.release = epc660_release, ///< Call during start and shutdown phase
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
	.poll = vb2_fop_poll
};

/**
 * @brief Wrapper for of_property_read_u32, used by diff fcts
 */
static int get_int_prop(struct device_node *dn, const char *s)
{
	int ret;
	u32 val;

	ret = of_property_read_u32(dn, s, &val);
	if (ret)
		return 0;
	return val;
}

/**
 * @brief Basic structure for registering the driver to the kernel (with MODULE_DEVICE_TABLE, of = open firmware)
 */
static const struct of_device_id cap_match[] =
{
	{ .compatible = COMPATIBLE_DT_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, cap_match);

/**
 * @brief Used by epc660_probe to initialize/configure both input structs (ppi info struct and i2c config/infos)
 * @param *pdev - representation of the platform device in the kernel
 * @param *o_config - video capture/epc660 chip information
 * @return "0" for OK or error msg
 */
static int fill_config(struct platform_device *pdev,
		       struct capture_config *o_config)
{
	struct device_node *node = pdev->dev.of_node;
	struct ppi_info *info;
	struct resource *res;

	/*Initialize ppi info struct*/
	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "failed to alloc ppi info\n");
		return -ENOMEM;
	}
	info->type = (enum ppi_type) get_int_prop(node, "type");
	info->irq_err = platform_get_irq(pdev, 0);
	info->spu = get_int_prop(node, "spu_securep_id");
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	info->base = devm_ioremap_resource(&pdev->dev, res);
	if (!(info->base)) {
		dev_err(&pdev->dev, "failed to get mem resource");
		return -ENOMEM;
	}

	o_config->i2c_adapter_id = get_int_prop(node, "i2c_bus_id");
	of_property_read_string(node, "card-name", &o_config->card_name);
	o_config->ppi_info = info;
	o_config->inputs = epc660_inputs;
	o_config->num_inputs = ARRAY_SIZE(epc660_inputs);
	o_config->routes = epc660_routes;
	return 0;
}

/**
 * @brief Basic function after initilization called by the kernel to gather structs/values of the driver device
 * @param struct platform_device - Representation of the platform device in the kernel to be set in this function
 * @return "0" for OK or error msg
 */
static int epc660_probe(struct platform_device *pdev)
{
	struct epc660_device *epc660_dev;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct vb2_queue *q;
	struct imager_route *route;
	struct of_device_id const* match;
	struct device *dev = &pdev->dev;
	struct gpio_desc *gpio;
	int ret;

	#if DEBUG
	printk(KERN_INFO "#### epc660_probe\n");
	#endif /*DEBUG*/

	match = of_match_device(cap_match, &pdev->dev);
	if (!match) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}


	epc660_dev = kzalloc(sizeof(*epc660_dev), GFP_KERNEL);
	if (!epc660_dev) {
		v4l2_err(pdev->dev.driver, "Unable to alloc epc660_dev\n");
		return -ENOMEM;
	}

	/* struct device_node *of_node : associated device tree node */
	if (dev->of_node) {
		fill_config(pdev, &epc660_dev->cfg);
	} else {
		dev_err(dev, "of node not populated\n");
		ret = -ENODEV;
		goto err_free_dev;
	}

	epc660_dev->dma_channel = get_int_prop(pdev->dev.of_node,
			"dma-channel");
	if (!epc660_dev->cfg.num_inputs || !epc660_dev->dma_channel) {
		v4l2_err(pdev->dev.driver, "Unable to get board config\n");
		ret = -ENODEV;
		goto err_free_dev;
	}

	epc660_dev->ppi = ppi_create_instance(pdev,
			epc660_dev->cfg.ppi_info);
	if (!epc660_dev->ppi) {
		v4l2_err(pdev->dev.driver, "Unable to create ppi\n");
		ret = -ENODEV;
		goto err_free_dev;
	}
	epc660_dev->ppi->priv = epc660_dev;

	vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));

	vfd = video_device_alloc();
	if (!vfd) {
		ret = -ENOMEM;
		v4l2_err(pdev->dev.driver, "Unable to alloc video device\n");
		goto err_cleanup;
	}

	/* initialize field of video device */
	vfd->release            = video_device_release;
	vfd->fops               = &epc660_fops;
	vfd->ioctl_ops          = &epc660_ioctl_ops;
	vfd->tvnorms            = 0;
	vfd->v4l2_dev           = &epc660_dev->v4l2_dev;
	strncpy(vfd->name, CAPTURE_DRV_NAME, sizeof(vfd->name));
	epc660_dev->video_dev     = vfd;

	ret = v4l2_device_register(&pdev->dev, &epc660_dev->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"Unable to register v4l2 device\n");
		goto err_release_vdev;
	}
	v4l2_info(&epc660_dev->v4l2_dev, "v4l2 device registered\n");

	epc660_dev->v4l2_dev.ctrl_handler = &epc660_dev->ctrl_handler;
	ret = v4l2_ctrl_handler_init(&epc660_dev->ctrl_handler, 0);
	if (ret) {
		v4l2_err(&epc660_dev->v4l2_dev,
				"Unable to init control handler\n");
		goto err_unreg_v4l2;
	}

	spin_lock_init(&epc660_dev->lock);
	/* initialize queue */
	q				= &epc660_dev->buffer_queue;
	q->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes			= VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	q->drv_priv			= epc660_dev;
	q->buf_struct_size	= sizeof(struct imager_buffer);
	q->ops				= &epc660_video_qops;
	q->mem_ops 			= &vb2_dma_contig_memops;
	q->timestamp_flags 		= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed 		= 1;					
	q->lock				= &epc660_dev->mutex;
	q->dev = &pdev->dev;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_handler;

	mutex_init(&epc660_dev->mutex);

	/* init video dma queues */
	INIT_LIST_HEAD(&epc660_dev->dma_queue);

	vfd->lock = &epc660_dev->mutex;
	vfd->queue = q; 

	/* register video device */
	ret = video_register_device(epc660_dev->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&epc660_dev->v4l2_dev,
				"Unable to register video device\n");
		goto err_free_handler;
	}
	video_set_drvdata(epc660_dev->video_dev, epc660_dev);
	v4l2_info(&epc660_dev->v4l2_dev, "video device registered as: %s\n",
			video_device_node_name(vfd));

	/* load up the subdevice */
	i2c_adap = i2c_get_adapter(epc660_dev->cfg.i2c_adapter_id);
	if (!i2c_adap) {
		v4l2_err(&epc660_dev->v4l2_dev,
				"Unable to find i2c adapter\n");
		ret = -ENODEV;
		goto err_unreg_vdev;

	}

	memset(&epc660_dev->cfg.board_info, 0, sizeof(epc660_dev->cfg.board_info));
	strlcpy(epc660_dev->cfg.board_info.type, epc660_dev->cfg.card_name, sizeof(epc660_dev->cfg.board_info.type));
	epc660_dev->cfg.board_info.addr = 0x22;
	epc660_dev->cfg.board_info.platform_data = NULL;

	gpio = gpiod_get(dev, "nrst", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio)) {
		v4l2_err(&epc660_dev->v4l2_dev, "Unable to get nrst gpio\n");
		goto err_free_handler;
	}
	epc660_dev->cfg.board_info.platform_data = gpio;

	epc660_dev->sd = v4l2_i2c_new_subdev_board(&epc660_dev->v4l2_dev,
						 i2c_adap,
						 &epc660_dev->cfg.board_info,
						 NULL);
	if (epc660_dev->sd) {
		int i;

		/* update tvnorms from the sub devices */
		for (i = 0; i < epc660_dev->cfg.num_inputs; i++)
			vfd->tvnorms |= epc660_dev->cfg.inputs[i].std;
	} else {
		v4l2_err(&epc660_dev->v4l2_dev,
				"Unable to register sub device\n");
		ret = -ENODEV;
		goto err_unreg_vdev;
	}

	v4l2_info(&epc660_dev->v4l2_dev, "v4l2 sub device registered\n");
	ret = v4l2_device_register_subdev_nodes(&epc660_dev->v4l2_dev);
	if (ret) {
		v4l2_err(&epc660_dev->v4l2_dev,
				"subdev nodes cannot be registered\n");
	}

	/*
	 * explicitly set input, otherwise some boards
	 * may not work at the state as we expected
	 */
	route = &epc660_dev->cfg.routes[0];
	ret = v4l2_subdev_call(epc660_dev->sd, video, s_routing,
				route->input, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&epc660_dev->v4l2_dev, "Failed to set input\n");
		goto err_unreg_vdev;
	}
	epc660_dev->cur_input = 0;
	/* if this route has specific config, update ppi control */

	/* now we can probe the default state */
	if (epc660_dev->cfg.inputs[0].capabilities & V4L2_IN_CAP_STD) {
		v4l2_std_id std;
		ret = v4l2_subdev_call(epc660_dev->sd, video, g_std, &std);
		if (ret) {
			v4l2_err(&epc660_dev->v4l2_dev,
					"Unable to get std\n");
			goto err_unreg_vdev;
		}
		epc660_dev->std = std;
	}
	if (epc660_dev->cfg.inputs[0].capabilities & V4L2_IN_CAP_DV_TIMINGS) {
		struct v4l2_dv_timings dv_timings;
		ret = v4l2_subdev_call(epc660_dev->sd, video,
				g_dv_timings, &dv_timings);
		if (ret) {
			v4l2_err(&epc660_dev->v4l2_dev,
					"Unable to get dv timings\n");
			goto err_unreg_vdev;
		}
		epc660_dev->dv_timings = dv_timings;
	}
	ret = epc660_init_sensor_formats(epc660_dev);
	if (ret) {
		v4l2_err(&epc660_dev->v4l2_dev,
				"Unable to create sensor formats table\n");
		goto err_unreg_vdev;
	}
	return 0;
err_unreg_vdev:
	video_unregister_device(epc660_dev->video_dev);
	epc660_dev->video_dev = NULL;
err_free_handler:
	v4l2_ctrl_handler_free(&epc660_dev->ctrl_handler);
err_unreg_v4l2:
	v4l2_device_unregister(&epc660_dev->v4l2_dev);
err_release_vdev:
	if (epc660_dev->video_dev)
		video_device_release(epc660_dev->video_dev);
err_cleanup:
	vb2_dma_contig_clear_max_seg_size(&pdev->dev);
	ppi_delete_instance(epc660_dev->ppi);
err_free_dev:
	kfree(epc660_dev);
	return ret;
}

/**
 * @brief Basic function before freeing the driver device called by the kernel to free storage and remove structs
 * @param struct platform_device - Representation of the platform device in the kernel 
 * @return "0" for OK 
 */
static int epc660_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct epc660_device *epc660_dev = container_of(v4l2_dev,
						struct epc660_device, v4l2_dev);

	#if DEBUG
	printk(KERN_INFO "#### epc660_remove\n");
	#endif /*DEBUG*/

	epc660_free_sensor_formats(epc660_dev);
	video_unregister_device(epc660_dev->video_dev);
	v4l2_ctrl_handler_free(&epc660_dev->ctrl_handler);
	v4l2_device_unregister(v4l2_dev);
	dma_pool_destroy(epc660_dev->dma_pool);
	vb2_dma_contig_clear_max_seg_size(&pdev->dev);
	ppi_delete_instance(epc660_dev->ppi);
	kfree(epc660_dev);
	return 0;
}

/**
 * @brief Basic structure for announcing the driver to the kernel (with "module_platform_driver")
 */
static struct platform_driver epc660_driver =
{
	.driver = { 
			.name = CAPTURE_DRV_NAME,
#ifdef CONFIG_OF
			.of_match_table = cap_match,
#endif
	},
	.probe  = epc660_probe,
	.remove = epc660_remove,
};
module_platform_driver(epc660_driver);

MODULE_DESCRIPTION("video capture driver for an EPC660 on Griffin-Lite");
MODULE_AUTHOR("Lutz Freitag <Lutz.Freitag@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
