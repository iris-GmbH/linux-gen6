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

struct imager_format {
	char *desc;
	u32 pixelformat;
	u32 mbus_code;
	int bpp; /* bits per pixel */
	int dlen; /* data length for ppi in bits */
	int channels; /* of how many interleaved sub-images is the format made of */
	int pixel_depth_bytes;
};


struct imager_dma_desc_list_item {
	dma_addr_t next_desc_addr;
	dma_addr_t start_addr;
	unsigned long cfg;
}__packed;

struct imager_buffer {
	struct vb2_v4l2_buffer vb;
	// points to memory that is allocated to be reacheable by the dma and holds an array of dma descriptors
	struct imager_dma_desc_list_item *dma_desc;
	// the dma reacheable address of the dma_desc
	dma_addr_t desc_dma_addr;
	struct list_head list;
};

struct imager_route {
	u32 input;
	u32 output;
};

struct capture_config {
	/* card name */
	const char *card_name;
	/* inputs available at the sub device */
	struct v4l2_input *inputs;
	/* number of inputs supported */
	int num_inputs;
	/* routing information for each input */
	struct imager_route *routes;
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
	/* capture config */
	struct capture_config cfg;
	/* dma channel */
	int dma_channel;
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
	/* used to store sensor supported format */
	struct imager_format *sensor_formats;
	/* number of sensor formats array */
	int num_sensor_formats;
	/* buffer queue used in videobuf2 */
	struct vb2_queue buffer_queue;
	/* something to allocate memory for dma usage from */
	struct dma_pool *dma_pool;
	/* queue of filled frames */
	struct list_head dma_queue;
	/* a dma config holding information on how to paramererize the dma */
	struct dmasg dma_cfg_template;
	/* used in videobuf2 callback */
	spinlock_t lock;
	/* used to access capture device */
	struct mutex mutex;
};

static const struct imager_format epc660_formats[] = {
	{
		.desc	     = "12bit Grey Scale",
		.pixelformat = V4L2_PIX_FMT_Y12,
		.mbus_code   = MEDIA_BUS_FMT_Y12_1X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 1,
		.pixel_depth_bytes = 2,
	},
	{
		.desc	     = "2DCS interleaved",
		.pixelformat = v4l2_fourcc('2', 'D', 'C', 'S'),
		.mbus_code   = MEDIA_BUS_FMT_EPC660_2X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 2,
		.pixel_depth_bytes = 4,
	},
	{
		.desc	     = "2DCS + gray interleaved",
		.pixelformat = v4l2_fourcc('2', 'D', 'C', 'G'),
		.mbus_code   = MEDIA_BUS_FMT_EPC660_3X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 3,
		.pixel_depth_bytes = 8,
	},
	{
		.desc	     = "4DCS interleaved",
		.pixelformat = v4l2_fourcc('4', 'D', 'C', 'S'),
		.mbus_code   = MEDIA_BUS_FMT_EPC660_4X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 4,
		.pixel_depth_bytes = 8,
	},
	{
		.desc	     = "4DCS + gray interleaved",
		.pixelformat = v4l2_fourcc('4', 'D', 'C', 'G'),
		.mbus_code   = MEDIA_BUS_FMT_EPC660_5X12,
		.bpp	     = 16,
		.dlen	     = 12, // 12 bit samples are mapped to 16 bits
		.channels    = 5,
		.pixel_depth_bytes = 16,
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

static struct imager_buffer *vb2v4l2_to_imagerbuffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct imager_buffer, vb);
}

/* The queue is busy if there is a owner and you are not that owner. */
static inline bool vb2_queue_is_busy(struct video_device *vdev, struct file *file)
{
	return vdev->queue->owner && vdev->queue->owner != file->private_data;
}

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

	if (vq->num_buffers + *nbuffers < MIN_NUM_BUF)
		*nbuffers = MIN_NUM_BUF;

	if (*nplanes)
		return sizes[0] < epc660_dev->fmt.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = epc660_dev->fmt.sizeimage;

	return 0;
}

static int epc660_buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	int dmaDescArrCount;
	int i, channel;
	dma_addr_t start_addr, channelStartAddr, nextDescrDMAAddr;
	struct imager_dma_desc_list_item *dma_desc;
	int halfImageSizeBytes, rowSizeBytes;

	INIT_LIST_HEAD(&buf->list);

	buf->dma_desc = dma_pool_alloc(epc660_dev->dma_pool, GFP_KERNEL, &buf->desc_dma_addr);
	if (!buf->dma_desc) {
		printk("cannot allocate memory from dma pool\n");
		return -ENOMEM;
	}

	dmaDescArrCount = epc660_dev->pixel_channels * epc660_dev->fmt.height;
	start_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	rowSizeBytes       = epc660_dev->fmt.width * epc660_dev->pixel_depth_bytes;
	halfImageSizeBytes = rowSizeBytes * epc660_dev->fmt.height / 2;

	dma_desc = buf->dma_desc;
	nextDescrDMAAddr = buf->desc_dma_addr + sizeof(*dma_desc);
	for (channel = 0; channel < epc660_dev->pixel_channels; ++channel) {
		channelStartAddr = start_addr + channel * ((epc660_dev->bpp + 7) / 8) * 16; // * 16 because the dma transfers sixteen pixels at a time
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
	}
	/* copy the descriptor config on the first transfer */
	buf->dma_desc->cfg |= DESCIDCPY;

	/* the very last element must be a list element that makes the dma point to the beginning */
	(dma_desc-1)->next_desc_addr = buf->desc_dma_addr;
	/* the last transfer shall generate an interrupt */
	(dma_desc-1)->cfg |= DI_EN_X;

	// print the descriptors
//	dma_desc = buf->dma_desc;
//	printk("buf->desc_dma_addr %08x\n", buf->desc_dma_addr);
//	for (i = 0; i < dmaDescArrCount; ++i) {
//		printk("dma_descriptor %d:"
//				"\n\t next_desc_addr %x"
//				"\n\t start_addr %x"
//				"\n\t cfg %08lx\n",
//				i,
//				buf->dma_desc[i].next_desc_addr,
//				buf->dma_desc[i].start_addr,
//				buf->dma_desc[i].cfg
//			);
//	}
	return 0;
}

static int epc660_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = epc660_dev->fmt.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&epc660_dev->v4l2_dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);

	vbuf->field = epc660_dev->fmt.field;

	return 0;
}

static void epc660_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	unsigned long flags;
	int last_dma_desc_idx;
	last_dma_desc_idx = epc660_dev->pixel_channels * epc660_dev->fmt.height - 1;
	buf->dma_desc[last_dma_desc_idx].next_desc_addr = buf->desc_dma_addr;
	spin_lock_irqsave(&epc660_dev->lock, flags);

	// setup the dma descriptor
	if (!list_empty(&epc660_dev->dma_queue)) {
		struct imager_buffer* lastBuffer;
		lastBuffer = list_last_entry(&epc660_dev->dma_queue, struct imager_buffer, list);
		lastBuffer->dma_desc[last_dma_desc_idx].next_desc_addr = buf->desc_dma_addr;
	}


//	{
//		struct list_head *pos;
//		int listSize = 0;
//		list_for_each(pos, &epc660_dev->dma_queue)
//		{
//			++listSize;
//		}
//		printk("listsize: %d\n", listSize);
//	}

	list_add_tail(&buf->list, &epc660_dev->dma_queue);

	spin_unlock_irqrestore(&epc660_dev->lock, flags);
}

static void epc660_buffer_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&epc660_dev->lock, flags);
	list_del_init(&buf->list);
	spin_unlock_irqrestore(&epc660_dev->lock, flags);
	dma_pool_free(epc660_dev->dma_pool, buf->dma_desc, buf->desc_dma_addr);
}

static int epc660_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct epc660_device *epc660_dev = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = epc660_dev->ppi;
	struct ppi_params params;
	int ret;

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(epc660_dev->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&epc660_dev->v4l2_dev, "stream on failed in subdev\n");
		return ret;
	}

	/* set ppi params */
	params.width	   = epc660_dev->fmt.width * 2;
	params.height	   = epc660_dev->fmt.height / 2;
	params.bpp	   = epc660_dev->bpp;
	params.dlen 	   = epc660_dev->dlen;
	params.ppi_control = (EPPI_CTL_DLEN12	   |  /* Data Word Length: 12 bit */
			      EPPI_CTL_NON656	   |  /* XFRTYPE: Non-ITU656 Mode (GP Mode) */
			      EPPI_CTL_SYNC2	   |  /* 2 external frame syncs */
			      EPPI_CTL_FS1LO_FS2LO |  /* FS1 and FS2 are active low */
			      EPPI_CTL_POLC0	   |  /* sample on falling DCLK */
			      EPPI_CTL_PACKEN	   |  /* assemble two incomming 16Bit words into one 32Bit word (reduces RAM-load a lot) */
			      EPPI_CTL_SIGNEXT);
	params.int_mask	   = 0x3c00;
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
	.queue_setup            = epc660_queue_setup,
	.buf_init               = epc660_buffer_init,
	.buf_prepare            = epc660_buffer_prepare,
	.buf_cleanup            = epc660_buffer_cleanup,
	.buf_queue              = epc660_buffer_queue,
	.wait_prepare           = vb2_ops_wait_prepare,
	.wait_finish            = vb2_ops_wait_finish,
	.start_streaming        = epc660_start_streaming,
	.stop_streaming         = epc660_stop_streaming,
};

#if 0
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

//	printk("epc660_isr\n");
	spin_lock(&epc660_dev->lock);

	dmaStatus = get_dma_curr_irqstat(epc660_dev->dma_channel);
	clear_dma_irqstat(epc660_dev->dma_channel);

	if (dmaStatus & DMA_DONE) {
		// if there are at least two buffers in the queue we can deque one
		if (&epc660_dev->dma_queue == epc660_dev->dma_queue.next->next) {
//			printk("buffer underrun in epc660 capture\n");
//			epc660_stop_transfering(epc660_dev);
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
//			printk("lastDmaDescriptor: 0x%08x\n", lastDmaDescriptor);
			if (0 == completedCnt) {
				printk("cannot find any completed buffers!\n");
//				epc660_stop_transfering(epc660_dev);
			} else {
				struct imager_buffer* buf;
				struct vb2_buffer *vb;
//				printk("found %d completed buffers\n", completedCnt);
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

	set_dma_next_desc_addr(epc660_dev->dma_channel, (void*)descrAddr);
	set_dma_x_count(epc660_dev->dma_channel, epc660_dev->dma_cfg_template.x_count),
	set_dma_x_modify(epc660_dev->dma_channel, epc660_dev->dma_cfg_template.x_modify),
	set_dma_config(epc660_dev->dma_channel, epc660_dev->dma_cfg_template.cfg);

	/* enable ppi */
	epc660_dev->ppi->ops->start(epc660_dev->ppi);

	return ret;
}

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

static int epc660_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct vb2_queue *vq = &epc660_dev->buffer_queue;
	unsigned long flags;
	int ret;
	struct imager_buffer* buf;

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

static int epc660_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	return v4l2_subdev_call(epc660_dev->sd, video, querystd, std);
}

static int epc660_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	*std = epc660_dev->std;
	return 0;
}

static int epc660_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&epc660_dev->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(epc660_dev->sd, video, s_std, std);
	if (ret < 0)
		return ret;

	epc660_dev->std = std;
	return 0;
}

static int epc660_enum_input(struct file *file, void *priv,
				struct v4l2_input *input)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct capture_config *config = &epc660_dev->cfg;

	int ret;
	u32 status;

	if (input->index >= config->num_inputs)
		return -EINVAL;

	*input = config->inputs[input->index];
	/* get input status */
	ret = v4l2_subdev_call(epc660_dev->sd, video, g_input_status, &status);
	if (!ret)
		input->status = status;
	return 0;
}

static int epc660_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	*index = epc660_dev->cur_input;
	return 0;
}

static int epc660_s_input(struct file *file, void *priv, unsigned int index)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct vb2_queue *vq = &epc660_dev->buffer_queue;
	struct capture_config *config = &epc660_dev->cfg;
	struct imager_route *route;
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	if (index >= config->num_inputs)
		return -EINVAL;

	route = &config->routes[index];
	ret = v4l2_subdev_call(epc660_dev->sd, video, s_routing, route->input,
			route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&epc660_dev->v4l2_dev, "Failed to set input\n");
		return ret;
	}
	epc660_dev->cur_input = index;
	return 0;
}

static int epc660_try_format(struct epc660_device *epc660_dev,
			     struct v4l2_pix_format *pixfmt,
			     struct imager_format *epc660_fmt)
{
	struct imager_format *sf = epc660_dev->sensor_formats;
	struct imager_format *fmt = NULL;
	struct v4l2_subdev_pad_config pad_cfg;
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

static int epc660_enum_fmt_vid_cap(struct file *file, void  *priv,
					   struct v4l2_fmtdesc *fmt)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct imager_format *sf = epc660_dev->sensor_formats;

	if (fmt->index >= epc660_dev->num_sensor_formats)
		return -EINVAL;

	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strlcpy(fmt->description, sf[fmt->index].desc, sizeof(fmt->description));
	fmt->pixelformat = sf[fmt->index].pixelformat;
	return 0;
}

static int epc660_try_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	return epc660_try_format(epc660_dev, pixfmt, NULL);
}

static int epc660_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	fmt->fmt.pix = epc660_dev->fmt;
	return 0;
}

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

	if (vb2_is_busy(vq))
		return -EBUSY;

	/* see if format works */
	ret = epc660_try_format(epc660_dev, pixfmt, &epc660_fmt);
	if (ret < 0)
		return ret;

	v4l2_fill_mbus_format(&format.format, pixfmt, epc660_fmt.mbus_code);
	ret = v4l2_subdev_call(epc660_dev->sd, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;
	epc660_dev->fmt               = *pixfmt;
	epc660_dev->bpp               = epc660_fmt.bpp;
	epc660_dev->dlen              = epc660_fmt.dlen;
	epc660_dev->pixel_channels    = epc660_fmt.channels;
	/* align the pixels to 4 bytes */
	epc660_dev->pixel_depth_bytes = epc660_fmt.pixel_depth_bytes;

	memset(&epc660_dev->dma_cfg_template, 0, sizeof(epc660_dev->dma_cfg_template));
	epc660_dev->dma_cfg_template.cfg      = RESTART | 
						DMATOVEN |
						WNR |
						WDSIZE_256 |
						PSIZE_32 |
						NDSIZE_2 |
						DMAFLOW_LIST |
						DMAEN;
	epc660_dev->dma_cfg_template.x_count  = epc660_dev->fmt.width / 16;
	epc660_dev->dma_cfg_template.x_modify = epc660_dev->pixel_depth_bytes * 16;

	if (epc660_dev->dma_pool) {
		dma_pool_destroy(epc660_dev->dma_pool);
	}

	dmaPoolMemorySize = (epc660_dev->pixel_channels * epc660_dev->fmt.height) * sizeof(struct imager_dma_desc_list_item);
	epc660_dev->dma_pool = dma_pool_create(CAPTURE_DRV_NAME, epc660_dev->v4l2_dev.dev,  dmaPoolMemorySize, 16, 0);
	return 0;
}

static int epc660_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	strlcpy(cap->driver, CAPTURE_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "Blackfin Platform", sizeof(cap->bus_info));
	strlcpy(cap->card, epc660_dev->cfg.card_name, sizeof(cap->card));
	return 0;
}

static int epc660_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return v4l2_subdev_call(epc660_dev->sd, video, g_parm, a);
}

static int epc660_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct epc660_device *epc660_dev = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return v4l2_subdev_call(epc660_dev->sd, video, s_parm, a);
}

static int epc660_log_status(struct file *file, void *priv)
{
	struct epc660_device *epc660_dev = video_drvdata(file);
	/* status for sub devices */
	v4l2_device_call_all(&epc660_dev->v4l2_dev, 0, core, log_status);
	return 0;
}

static const struct v4l2_ioctl_ops epc660_ioctl_ops =
{
	.vidioc_querycap         = epc660_querycap,
	.vidioc_g_fmt_vid_cap    = epc660_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = epc660_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap    = epc660_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap  = epc660_try_fmt_vid_cap,
	.vidioc_enum_input       = epc660_enum_input,
	.vidioc_g_input          = epc660_g_input,
	.vidioc_s_input          = epc660_s_input,
	.vidioc_querystd         = epc660_querystd,
	.vidioc_s_std            = epc660_s_std,
	.vidioc_g_std            = epc660_g_std,
	.vidioc_reqbufs          = vb2_ioctl_reqbufs,
	.vidioc_create_bufs      = vb2_ioctl_create_bufs,
	.vidioc_querybuf         = vb2_ioctl_querybuf,
	.vidioc_qbuf             = vb2_ioctl_qbuf,
	.vidioc_dqbuf            = vb2_ioctl_dqbuf,
	.vidioc_expbuf           = vb2_ioctl_expbuf,
	.vidioc_streamon         = epc660_streamon,
	.vidioc_streamoff        = vb2_ioctl_streamoff,
	.vidioc_g_parm           = epc660_g_parm,
	.vidioc_s_parm           = epc660_s_parm,
	.vidioc_log_status       = epc660_log_status,
};

static struct v4l2_file_operations epc660_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
#ifndef CONFIG_MMU
	.get_unmapped_area = vb2_fop_get_unmapped_area,
#endif
	.poll = vb2_fop_poll
};

static int get_int_prop(struct device_node *dn, const char *s)
{
	int ret;
	u32 val;

	ret = of_property_read_u32(dn, s, &val);
	if (ret)
		return 0;
	return val;
}

static const struct of_device_id cap_match[] =
{
	{ .compatible = COMPATIBLE_DT_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, cap_match);

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

static int epc660_probe(struct platform_device *pdev)
{
	struct epc660_device *epc660_dev;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct vb2_queue *q;
	struct imager_route *route;
	struct of_device_id const* match;
	struct device *dev = &pdev->dev;
	int ret;

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

static int epc660_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct epc660_device *epc660_dev = container_of(v4l2_dev,
						struct epc660_device, v4l2_dev);

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
