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

#define COMPATIBLE_DT_NAME	"iris,gen6-aptina_mt9v022"
#define CAPTURE_DRV_NAME        "aptina_mt9v022_capture" //use this string in userspace-app
#define MIN_NUM_BUF      	2

struct imager_format {
	char *desc;
	u32 pixelformat;
	u32 mbus_code;
	int bpp; /* bits per pixel */
	int dlen; /* data length for ppi in bits */
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

struct aptina_mt9v022_device {
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
	/* buffer that is currently filled by DMA and will finish next */
	struct imager_buffer * nextBufferToFinish;
	/* buffers that form a loop that keeps the DMA from overwriting buffers after dequeing them */
	struct imager_buffer * loop_buffer_a, * loop_buffer_b, * loop_buffer_c;
};

static const struct imager_format aptina_mt9v022_formats[] = {
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = MEDIA_BUS_FMT_UYVY8_2X8,
		.bpp         = 16,
		.dlen        = 8,
		.pixel_depth_bytes = 2,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved YUYV",
		.pixelformat = V4L2_PIX_FMT_YUYV,
		.mbus_code   = MEDIA_BUS_FMT_YUYV8_2X8,
		.bpp         = 16,
		.dlen        = 8,
		.pixel_depth_bytes = 2,
	},
	{
		.desc        = "YCbCr 4:2:2 Interleaved UYVY",
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.mbus_code   = MEDIA_BUS_FMT_UYVY8_1X16,
		.bpp         = 16,
		.dlen        = 16,
		.pixel_depth_bytes = 2,
	},
	{
		.desc        = "RGB 565",
		.pixelformat = V4L2_PIX_FMT_RGB565,
		.mbus_code   = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.bpp         = 16,
		.dlen        = 8,
		.pixel_depth_bytes = 2,
	},
	{
		.desc        = "RGB 444",
		.pixelformat = V4L2_PIX_FMT_RGB444,
		.mbus_code   = MEDIA_BUS_FMT_RGB444_2X8_PADHI_LE,
		.bpp         = 16,
		.dlen        = 8,
		.pixel_depth_bytes = 2,
	},
	{
		.desc 	    	   = "10bit Grey Scale",
		.pixelformat	   = V4L2_PIX_FMT_Y10,
		.mbus_code  	   = MEDIA_BUS_FMT_Y10_1X10,
		.bpp	    	   = 16,
		.dlen	     	   = 10, // number of wires
		.pixel_depth_bytes = 2,
	},

};
#define MAX_FMTS ARRAY_SIZE(aptina_mt9v022_formats)

static struct v4l2_input aptina_mt9v022_inputs[] = {
	{
		.index	= 0,
		.name	= CAPTURE_DRV_NAME,
		.type	= V4L2_INPUT_TYPE_CAMERA,
		.std	= V4L2_STD_UNKNOWN,
	},
};

static struct imager_route aptina_mt9v022_routes[] = {
	{
		.input  = 0,
		.output = 0,
	},
};

static irqreturn_t aptina_mt9v022_isr(int irq, void *dev_id);
static int aptina_mt9v022_start_transfering(
				struct aptina_mt9v022_device *aptina_mt9v022_dev,
				dma_addr_t descrAddr);
static void aptina_mt9v022_stop_transfering(
				struct aptina_mt9v022_device *aptina_mt9v022_dev);

static struct imager_buffer *vb2v4l2_to_imagerbuffer(struct vb2_v4l2_buffer *vb)
{
	return container_of(vb, struct imager_buffer, vb);
}

/* The queue is busy if there is a owner and you are not that owner. */
static inline bool vb2_queue_is_busy(struct video_device *vdev, struct file *file)
{
	return vdev->queue->owner && vdev->queue->owner != file->private_data;
}

static int aptina_mt9v022_init_sensor_formats(
				struct aptina_mt9v022_device *aptina_mt9v022_dev)
{

	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct imager_format *sf;
	unsigned int num_formats = 0;
	int i, j;

	while (!v4l2_subdev_call(aptina_mt9v022_dev->sd, pad,
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
		v4l2_subdev_call(aptina_mt9v022_dev->sd, pad,
				enum_mbus_code, NULL, &code);
		for (j = 0; j < MAX_FMTS; j++)
			if (code.code == aptina_mt9v022_formats[j].mbus_code)
				break;
		if (j == MAX_FMTS) {
			/* we don't allow this sensor working with our bridge */
			kfree(sf);
			return -EINVAL;
		}
		sf[i] = aptina_mt9v022_formats[j];
	}

	aptina_mt9v022_dev->sensor_formats = sf;
	aptina_mt9v022_dev->num_sensor_formats = num_formats;
	return 0;
}

static void aptina_mt9v022_free_sensor_formats(
				struct aptina_mt9v022_device *aptina_mt9v022_dev)
{
	aptina_mt9v022_dev->num_sensor_formats = 0;
	kfree(aptina_mt9v022_dev->sensor_formats);
	aptina_mt9v022_dev->sensor_formats = NULL;
}

static int aptina_mt9v022_queue_setup(struct vb2_queue *vq,
				unsigned int *nbuffers,
				unsigned int *nplanes,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = vb2_get_drv_priv(vq);

	if (vq->num_buffers + *nbuffers < MIN_NUM_BUF)
		*nbuffers = MIN_NUM_BUF;

	if (*nplanes)
		return sizes[0] < aptina_mt9v022_dev->fmt.sizeimage ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = aptina_mt9v022_dev->fmt.sizeimage;

	return 0;
}

static int aptina_mt9v022_buffer_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct aptina_mt9v022_device *aptina_mt9v022_dev =
						 vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	dma_addr_t start_addr;

	INIT_LIST_HEAD(&buf->list);

	buf->dma_desc = dma_pool_alloc(aptina_mt9v022_dev->dma_pool, GFP_KERNEL,
			&buf->desc_dma_addr);
	if (!buf->dma_desc) {
		printk("cannot allocate memory from dma pool\n");
		return -ENOMEM;
	}

	start_addr = vb2_dma_contig_plane_dma_addr(vb, 0);

	/* loop over the buffer */
	buf->dma_desc->next_desc_addr = buf->desc_dma_addr;
	buf->dma_desc->start_addr = start_addr;
	/* copy the descriptor config on the first transfer and generate an interrupt uppon succession */
	buf->dma_desc->cfg = aptina_mt9v022_dev->dma_cfg_template.cfg | DI_EN_Y	| DESCIDCPY;
	return 0;
}

static int aptina_mt9v022_buffer_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct aptina_mt9v022_device *aptina_mt9v022_dev = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = aptina_mt9v022_dev->fmt.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev, "buffer too small (%lu < %lu)\n",
				vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	vb2_set_plane_payload(vb, 0, size);

	vbuf->field = aptina_mt9v022_dev->fmt.field;

	return 0;
}

static void aptina_mt9v022_buffer_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct aptina_mt9v022_device *aptina_mt9v022_dev = vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	unsigned long flags;

	buf->dma_desc->next_desc_addr = buf->desc_dma_addr;
	spin_lock_irqsave(&aptina_mt9v022_dev->lock, flags);
	// setup the dma descriptor
	if (!list_empty(&aptina_mt9v022_dev->dma_queue)) {
		struct imager_buffer *nextbuf_a, *nextbuf_b;
		int dma_queue_length = 0;

		if (aptina_mt9v022_dev->nextBufferToFinish) {
			// DMA is running, get length of queue
			struct list_head * iterator;
			list_for_each(iterator, &aptina_mt9v022_dev->dma_queue) {
				dma_queue_length++;
			}
		}
		if (dma_queue_length == 3) {
			// DMA is in "loop-mode" (1-2-3-1-2-3-1-...)
			// find "2" (nextbuf_a) and "3" (nextbuf_b)
			nextbuf_a = aptina_mt9v022_dev->nextBufferToFinish;
			do {
				nextbuf_a = list_next_entry(nextbuf_a, list);
			} while (&nextbuf_a->list == &aptina_mt9v022_dev->dma_queue);
			nextbuf_b = nextbuf_a;
			do {
				nextbuf_b = list_next_entry(nextbuf_b, list);
			} while (&nextbuf_b->list == &aptina_mt9v022_dev->dma_queue);
			// Rewire "new", "2" and "3" as a loop so that "1" is free after it finishes
			nextbuf_b->dma_desc->next_desc_addr = buf->desc_dma_addr;
			nextbuf_a->dma_desc->next_desc_addr = nextbuf_b->desc_dma_addr;
			buf->dma_desc->next_desc_addr = nextbuf_a->desc_dma_addr;
		} else {
			// not in "loop-mode", just rewire the loop to a-b-new-a-b-...
			nextbuf_a = list_last_entry(&aptina_mt9v022_dev->dma_queue, struct imager_buffer, list);
			nextbuf_b = list_prev_entry(nextbuf_a, list);
			nextbuf_a->dma_desc->next_desc_addr = buf->desc_dma_addr;
			buf->dma_desc->next_desc_addr = nextbuf_b->desc_dma_addr;
		}
		aptina_mt9v022_dev->loop_buffer_a = buf;
		aptina_mt9v022_dev->loop_buffer_b = nextbuf_a;
		aptina_mt9v022_dev->loop_buffer_c = nextbuf_b;
	} else {
		aptina_mt9v022_dev->loop_buffer_a = 0;
		aptina_mt9v022_dev->loop_buffer_b = 0;
		aptina_mt9v022_dev->loop_buffer_c = 0;
		aptina_mt9v022_dev->nextBufferToFinish = 0;
	}
	list_add_tail(&buf->list, &aptina_mt9v022_dev->dma_queue);
	spin_unlock_irqrestore(&aptina_mt9v022_dev->lock, flags);
}

static void aptina_mt9v022_buffer_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct aptina_mt9v022_device *aptina_mt9v022_dev =
						vb2_get_drv_priv(vb->vb2_queue);
	struct imager_buffer *buf = vb2v4l2_to_imagerbuffer(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&aptina_mt9v022_dev->lock, flags);
	list_del_init(&buf->list);
	spin_unlock_irqrestore(&aptina_mt9v022_dev->lock, flags);
	dma_pool_free(aptina_mt9v022_dev->dma_pool, buf->dma_desc, buf->desc_dma_addr);
}

static int aptina_mt9v022_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = vb2_get_drv_priv(vq);
	struct ppi_if *ppi = aptina_mt9v022_dev->ppi;
	struct ppi_params params;
	int ret;

	/* enable streamon on the sub device */
	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_stream, 1);
	if (ret && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev, "stream on failed in subdev\n");
		return ret;
	}

	/* set ppi params */
	params.width	   = aptina_mt9v022_dev->fmt.width;
	params.height	   = aptina_mt9v022_dev->fmt.height;
	params.bpp	   = aptina_mt9v022_dev->bpp;
	params.dlen 	   = aptina_mt9v022_dev->dlen;
	params.ppi_control = (EPPI_CTL_PACKEN  |
			      EPPI_CTL_DLEN10  |
			      EPPI_CTL_NON656  |
			      EPPI_CTL_SYNC2   |
			      EPPI_CTL_POLC3   |
			      EPPI_CTL_DMAFINEN );
	params.int_mask	   = 0xfc;
	params.hdelay	   = 0;
	params.vdelay	   = 0;
	params.line	   = params.width;
	params.frame	   = params.height;

	ret = ppi->ops->set_params(ppi, &params);

	if (ret < 0) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"Error in setting ppi params\n");
		return ret;
	}
	return 0;
}

static void aptina_mt9v022_stop_streaming(struct vb2_queue *vq)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = vb2_get_drv_priv(vq);
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&aptina_mt9v022_dev->lock, flags);
	aptina_mt9v022_stop_transfering(aptina_mt9v022_dev);

	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_stream, 0);
	if (ret && (ret != -ENOIOCTLCMD))
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"stream off failed in subdev\n");

	/* release all active buffers */
	while (!list_empty(&aptina_mt9v022_dev->dma_queue)) {
		struct imager_buffer *buf = 
					list_entry(aptina_mt9v022_dev->dma_queue.next,
						struct imager_buffer, list);
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&aptina_mt9v022_dev->lock, flags);
}

static struct vb2_ops aptina_mt9v022_video_qops =
{
	.queue_setup	= aptina_mt9v022_queue_setup,
	.buf_init	= aptina_mt9v022_buffer_init,
	.buf_prepare	= aptina_mt9v022_buffer_prepare,
	.buf_cleanup	= aptina_mt9v022_buffer_cleanup,
	.buf_queue	= aptina_mt9v022_buffer_queue,
	.wait_prepare	= vb2_ops_wait_prepare,
	.wait_finish	= vb2_ops_wait_finish,
	.start_streaming = aptina_mt9v022_start_streaming,
	.stop_streaming = aptina_mt9v022_stop_streaming,
};

#if 0
static void printDMAState(struct aptina_mt9v022_device *aptina_mt9v022_dev)
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

			get_dma_next_desc_ptr(aptina_mt9v022_dev->dma_channel),
			get_dma_start_addr(aptina_mt9v022_dev->dma_channel),
			get_dma_config(aptina_mt9v022_dev->dma_channel),
			get_dma_x_count(aptina_mt9v022_dev->dma_channel),
			get_dma_x_modify(aptina_mt9v022_dev->dma_channel),
			get_dma_y_count(aptina_mt9v022_dev->dma_channel),
			get_dma_y_modify(aptina_mt9v022_dev->dma_channel),
			get_dma_curr_desc_ptr(aptina_mt9v022_dev->dma_channel),
			get_dma_prev_desc_ptr(aptina_mt9v022_dev->dma_channel),
			get_dma_curr_addr(aptina_mt9v022_dev->dma_channel),
			get_dma_curr_irqstat(aptina_mt9v022_dev->dma_channel),
			get_dma_curr_xcount(aptina_mt9v022_dev->dma_channel),
			get_dma_curr_ycount(aptina_mt9v022_dev->dma_channel));
}
#endif

static irqreturn_t aptina_mt9v022_isr(int irq, void *dev_id)
{
	struct ppi_if *ppi = dev_id;
	struct aptina_mt9v022_device *aptina_mt9v022_dev = ppi->priv;
	int dmaStatus;
	dma_addr_t lastDmaDescriptor, currDmaDescriptor;
	struct list_head* iterator;

	spin_lock(&aptina_mt9v022_dev->lock);

	dmaStatus = get_dma_curr_irqstat(aptina_mt9v022_dev->dma_channel);
	clear_dma_irqstat(aptina_mt9v022_dev->dma_channel);

	if (dmaStatus & DMA_DONE) {
		// if there are at least two buffers in the queue we can deque one
		if (&aptina_mt9v022_dev->dma_queue
				== aptina_mt9v022_dev->dma_queue.next->next) {
		} else {
			/* publish all buffers that are done */
			int completedCnt = 0, countAsCompleted = 1, dma_queue_length = 0;
			lastDmaDescriptor = (dma_addr_t) get_dma_prev_desc_ptr(aptina_mt9v022_dev->dma_channel);
			lastDmaDescriptor &= ~0x0F;
			currDmaDescriptor = (dma_addr_t) get_dma_curr_desc_ptr(aptina_mt9v022_dev->dma_channel);
			currDmaDescriptor &= ~0x0F;
			// count the buffers that might be completed and remember the buffer of the next DMA operation
			list_for_each(iterator, &aptina_mt9v022_dev->dma_queue)
			{
				struct imager_buffer *buf = list_entry(iterator, struct imager_buffer, list);
				dma_queue_length++;
				completedCnt += countAsCompleted;
				if (buf->desc_dma_addr == lastDmaDescriptor) {
					countAsCompleted = 0;
				}
				if (buf->desc_dma_addr == currDmaDescriptor) {
					aptina_mt9v022_dev->nextBufferToFinish = buf;
				}
			}
			// keep at least three buffers here
			completedCnt = (dma_queue_length - completedCnt < 3) ? dma_queue_length - 3 : completedCnt;
			dma_queue_length -= 1;
			if (completedCnt > 0) {
				struct vb2_buffer *vb;
				list_for_each(iterator, &aptina_mt9v022_dev->dma_queue)
				{
					struct imager_buffer * buf;
					if (!(dma_queue_length-- && completedCnt)) {
						break;
					}
					buf = list_entry(iterator, struct imager_buffer, list);
					// skip buffer if it is part of the loop
					if (buf == aptina_mt9v022_dev->loop_buffer_a
						|| buf == aptina_mt9v022_dev->loop_buffer_b
						|| buf == aptina_mt9v022_dev->loop_buffer_c) {
						continue;
					}
					vb = &buf->vb.vb2_buf;
					vb->timestamp = ktime_get_ns(); // this has been changed from type struct timeval to -> u64 type
					if (ppi->err) {
						vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
					} else {
						vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
					}
					list_del_init(&buf->list);
					completedCnt--;
				}
			}
		}
		/* clear error flag for the next frame */
		ppi->err = false;
	}

	spin_unlock(&aptina_mt9v022_dev->lock);

	return IRQ_HANDLED;
}

static int aptina_mt9v022_start_transfering(struct aptina_mt9v022_device
						*aptina_mt9v022_dev, dma_addr_t descrAddr)
{
	int ret;

	ret = request_dma(aptina_mt9v022_dev->dma_channel, "aptina_mt9v022_dma");
	if (ret) {
		printk("Unable to allocate DMA channel\n");
		return ret;
	}

	/* attach ppi DMA irq handler */
	set_dma_callback(aptina_mt9v022_dev->dma_channel, aptina_mt9v022_isr, aptina_mt9v022_dev->ppi);

	set_dma_next_desc_addr(aptina_mt9v022_dev->dma_channel, (void*) descrAddr);
	set_dma_x_count(aptina_mt9v022_dev->dma_channel, aptina_mt9v022_dev->dma_cfg_template.x_count);
	set_dma_x_modify(aptina_mt9v022_dev->dma_channel, aptina_mt9v022_dev->dma_cfg_template.x_modify);
	set_dma_y_count(aptina_mt9v022_dev->dma_channel, aptina_mt9v022_dev->dma_cfg_template.y_count);
	set_dma_y_modify(aptina_mt9v022_dev->dma_channel, aptina_mt9v022_dev->dma_cfg_template.y_modify);
	set_dma_config(aptina_mt9v022_dev->dma_channel, aptina_mt9v022_dev->dma_cfg_template.cfg);
#if 0
	printDMAState(aptina_mt9v022_dev);
#endif
	/* enable ppi */
	aptina_mt9v022_dev->ppi->ops->start(aptina_mt9v022_dev->ppi);

	return ret;
}

static void aptina_mt9v022_stop_transfering(struct aptina_mt9v022_device 
						*aptina_mt9v022_dev)
{
	/* disable ppi */
	aptina_mt9v022_dev->ppi->ops->stop(aptina_mt9v022_dev->ppi);

	dma_disable_irq(aptina_mt9v022_dev->dma_channel);
	disable_dma(aptina_mt9v022_dev->dma_channel);

	clear_dma_irqstat(aptina_mt9v022_dev->dma_channel);

	free_dma(aptina_mt9v022_dev->dma_channel);
}

static int aptina_mt9v022_streamon(struct file *file, void *priv,
				enum v4l2_buf_type buf_type)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct vb2_queue *vq = &aptina_mt9v022_dev->buffer_queue;
	unsigned long flags;
	int ret;
	struct imager_buffer* buf;

	if (vb2_queue_is_busy(aptina_mt9v022_dev->video_dev, file))
		return -EBUSY;

	/* call streamon to start streaming in videobuf */
	ret = vb2_streamon(vq, buf_type);
	if (ret)
		return ret;

	/* if dma queue is empty, return error */
	if (list_empty(&aptina_mt9v022_dev->dma_queue)) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev, "dma queue is empty\n");
		ret = -EINVAL;
		goto err;
	}

	spin_lock_irqsave(&aptina_mt9v022_dev->lock, flags);

	/* get the next frame from the dma queue */
	buf = list_entry(aptina_mt9v022_dev->dma_queue.next,
			struct imager_buffer, list);
	aptina_mt9v022_start_transfering(aptina_mt9v022_dev, buf->desc_dma_addr);
	spin_unlock_irqrestore(&aptina_mt9v022_dev->lock, flags);

	return 0;
err:	
	vb2_streamoff(vq, buf_type);
	return ret;
}

static int aptina_mt9v022_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	return v4l2_subdev_call(aptina_mt9v022_dev->sd, video, querystd, std);
}

static int aptina_mt9v022_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	*std = aptina_mt9v022_dev->std;
	return 0;
}

static int aptina_mt9v022_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&aptina_mt9v022_dev->buffer_queue))
		return -EBUSY;

	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_std, std);
	if (ret < 0)
		return ret;

	aptina_mt9v022_dev->std = std;
	return 0;
}

static int aptina_mt9v022_enum_dv_timings(struct file *file, void *priv,
					  struct v4l2_enum_dv_timings *timings)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	timings->pad = 0;

	return v4l2_subdev_call(aptina_mt9v022_dev->sd, pad, enum_dv_timings, timings);
}

static int aptina_mt9v022_query_dv_timings(struct file *file, void *priv,
					   struct v4l2_dv_timings *timings)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	return v4l2_subdev_call(aptina_mt9v022_dev->sd, video, query_dv_timings,timings);
}

static int aptina_mt9v022_g_dv_timings(struct file *file, void *priv,
				       struct v4l2_dv_timings *timings)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	*timings = aptina_mt9v022_dev->dv_timings;
	return 0;
}

static int aptina_mt9v022_s_dv_timings(struct file *file, void *priv,
				       struct v4l2_dv_timings *timings)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct vb2_queue *vq = &aptina_mt9v022_dev->buffer_queue;

	int ret;
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_dv_timings, timings);
	if (ret < 0)
		return ret;

	aptina_mt9v022_dev->dv_timings = *timings;
	return 0;
}

static int aptina_mt9v022_enum_input(struct file *file, void *priv,
				     struct v4l2_input *input)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct capture_config *config = &aptina_mt9v022_dev->cfg;

	int ret;
	u32 status;

	if (input->index >= config->num_inputs)
		return -EINVAL;

	*input = config->inputs[input->index];
	/* get input status */
	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, g_input_status, &status);
	if (!ret)
		input->status = status;
	return 0;
}

static int aptina_mt9v022_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	*index = aptina_mt9v022_dev->cur_input;
	return 0;
}

static int aptina_mt9v022_s_input(struct file *file, void *priv,
				  unsigned int index)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct vb2_queue *vq = &aptina_mt9v022_dev->buffer_queue;
	struct capture_config *config = &aptina_mt9v022_dev->cfg;
	struct imager_route *route;
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	if (index >= config->num_inputs)
		return -EINVAL;

	route = &config->routes[index];
	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_routing, route->input,
			route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev, "Failed to set input\n");
		return ret;
	}
	aptina_mt9v022_dev->cur_input = index;
	return 0;
}

static int aptina_mt9v022_try_format(struct aptina_mt9v022_device *aptina_mt9v022_dev,
				     struct v4l2_pix_format *pixfmt,
				     struct imager_format *aptina_mt9v022_fmt)
{
	struct imager_format *sf = aptina_mt9v022_dev->sensor_formats;
	struct imager_format *fmt = NULL;
	struct v4l2_subdev_pad_config pad_cfg;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};
	int ret, i;

	for (i = 0; i < aptina_mt9v022_dev->num_sensor_formats; i++) {
		fmt = &sf[i];
		if (pixfmt->pixelformat == fmt->pixelformat)
			break;
	}
	if (i == aptina_mt9v022_dev->num_sensor_formats)
		fmt = &sf[0];

	v4l2_fill_mbus_format(&format.format, pixfmt, fmt->mbus_code);
	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, pad, set_fmt, &pad_cfg,
				&format);
	if (ret < 0)
		return ret;
	v4l2_fill_pix_format(pixfmt, &format.format);
	if (aptina_mt9v022_fmt) {
		for (i = 0; i < aptina_mt9v022_dev->num_sensor_formats; i++) {
			fmt = &sf[i];
			if (format.format.code == fmt->mbus_code)
				break;
		}
		*aptina_mt9v022_fmt = *fmt;
	}

	pixfmt->bytesperline = pixfmt->width * aptina_mt9v022_fmt->pixel_depth_bytes;
	pixfmt->sizeimage = pixfmt->bytesperline * pixfmt->height;
	return 0;
}

static int aptina_mt9v022_enum_fmt_vid_cap(struct file *file, void *priv,
					   struct v4l2_fmtdesc *fmt)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct imager_format *sf = aptina_mt9v022_dev->sensor_formats;

	if (fmt->index >= aptina_mt9v022_dev->num_sensor_formats)
		return -EINVAL;

	fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	strlcpy(fmt->description, sf[fmt->index].desc, sizeof(fmt->description));
	fmt->pixelformat = sf[fmt->index].pixelformat;
	return 0;
}

static int aptina_mt9v022_try_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;

	return aptina_mt9v022_try_format(aptina_mt9v022_dev, pixfmt, NULL);
}

static int aptina_mt9v022_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	fmt->fmt.pix = aptina_mt9v022_dev->fmt;
	return 0;
}

static int aptina_mt9v022_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *fmt)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	struct vb2_queue *vq = &aptina_mt9v022_dev->buffer_queue;
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct imager_format aptina_mt9v022_fmt;
	struct v4l2_pix_format *pixfmt = &fmt->fmt.pix;
	int dmaPoolMemorySize;
	int ret;

	if (vb2_is_busy(vq))
		return -EBUSY;

	/* see if format works */
	ret = aptina_mt9v022_try_format(aptina_mt9v022_dev, pixfmt,
					&aptina_mt9v022_fmt);
	if (ret < 0)
		return ret;

	v4l2_fill_mbus_format(&format.format, pixfmt, aptina_mt9v022_fmt.mbus_code);
	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, pad, set_fmt, NULL, &format);
	if (ret < 0)
		return ret;
	aptina_mt9v022_dev->fmt = *pixfmt;
	aptina_mt9v022_dev->bpp = aptina_mt9v022_fmt.bpp;
	aptina_mt9v022_dev->dlen = aptina_mt9v022_fmt.dlen;
	aptina_mt9v022_dev->pixel_depth_bytes = aptina_mt9v022_fmt.pixel_depth_bytes;

	memset(&aptina_mt9v022_dev->dma_cfg_template, 0,
			sizeof(aptina_mt9v022_dev->dma_cfg_template));
	aptina_mt9v022_dev->dma_cfg_template.cfg =	RESTART |
							DMA2D |
							DMATOVEN |
							WNR |
							WDSIZE_256 |
							PSIZE_32 |
							NDSIZE_2 |
							DMAFLOW_LIST |
							DMAEN;
	aptina_mt9v022_dev->dma_cfg_template.x_count  = aptina_mt9v022_dev->fmt.width / 16;
	aptina_mt9v022_dev->dma_cfg_template.y_count  = aptina_mt9v022_dev->fmt.height;
	aptina_mt9v022_dev->dma_cfg_template.x_modify =	aptina_mt9v022_dev->pixel_depth_bytes * 16;
	aptina_mt9v022_dev->dma_cfg_template.y_modify =	aptina_mt9v022_dev->pixel_depth_bytes * 16;

	if (aptina_mt9v022_dev->dma_pool) {
		dma_pool_destroy(aptina_mt9v022_dev->dma_pool);
	}

	// we need only one dma descriptor per image
	dmaPoolMemorySize = sizeof(struct imager_dma_desc_list_item);
	aptina_mt9v022_dev->dma_pool = dma_pool_create(CAPTURE_DRV_NAME,
			aptina_mt9v022_dev->v4l2_dev.dev, dmaPoolMemorySize, 16, 0);
	return 0;
}

static int aptina_mt9v022_querycap(struct file *file, void *priv,
				   struct v4l2_capability *cap)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	strlcpy(cap->driver, CAPTURE_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "Blackfin Platform", sizeof(cap->bus_info));
	strlcpy(cap->card, aptina_mt9v022_dev->cfg.card_name, sizeof(cap->card));
	return 0;
}

static int aptina_mt9v022_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return v4l2_subdev_call(aptina_mt9v022_dev->sd, video, g_parm, a);
}

static int aptina_mt9v022_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	return v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_parm, a);
}

static int aptina_mt9v022_log_status(struct file *file, void *priv)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev = video_drvdata(file);
	/* status for sub devices */
	v4l2_device_call_all(&aptina_mt9v022_dev->v4l2_dev, 0, core, log_status);
	return 0;
}


static const struct v4l2_ioctl_ops aptina_mt9v022_ioctl_ops =
{
	.vidioc_querycap         = aptina_mt9v022_querycap,
	.vidioc_g_fmt_vid_cap    = aptina_mt9v022_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = aptina_mt9v022_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap    = aptina_mt9v022_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	 = aptina_mt9v022_try_fmt_vid_cap,
	.vidioc_enum_input	 = aptina_mt9v022_enum_input,
	.vidioc_g_input		 = aptina_mt9v022_g_input,
	.vidioc_s_input		 = aptina_mt9v022_s_input,
	.vidioc_querystd	 = aptina_mt9v022_querystd,
	.vidioc_s_std		 = aptina_mt9v022_s_std,
	.vidioc_g_std		 = aptina_mt9v022_g_std,
	.vidioc_s_dv_timings     = aptina_mt9v022_s_dv_timings,
	.vidioc_g_dv_timings     = aptina_mt9v022_g_dv_timings,
	.vidioc_query_dv_timings = aptina_mt9v022_query_dv_timings,
	.vidioc_enum_dv_timings  = aptina_mt9v022_enum_dv_timings,
	.vidioc_reqbufs          = vb2_ioctl_reqbufs,
	.vidioc_create_bufs      = vb2_ioctl_create_bufs,
	.vidioc_querybuf         = vb2_ioctl_querybuf,
	.vidioc_qbuf             = vb2_ioctl_qbuf,
	.vidioc_dqbuf            = vb2_ioctl_dqbuf,
	.vidioc_expbuf           = vb2_ioctl_expbuf,
	.vidioc_streamon         = aptina_mt9v022_streamon,
	.vidioc_streamoff        = vb2_ioctl_streamoff,
	.vidioc_g_parm		 = aptina_mt9v022_g_parm,
	.vidioc_s_parm		 = aptina_mt9v022_s_parm,
	.vidioc_log_status	 = aptina_mt9v022_log_status,
};

static struct v4l2_file_operations aptina_mt9v022_fops = {
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
	o_config->inputs = aptina_mt9v022_inputs;
	o_config->num_inputs = ARRAY_SIZE(aptina_mt9v022_inputs);
	o_config->routes = aptina_mt9v022_routes;
	return 0;
}

static int aptina_mt9v022_probe(struct platform_device *pdev)
{
	struct aptina_mt9v022_device *aptina_mt9v022_dev;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	struct vb2_queue *q;
	struct imager_route *route;
	struct of_device_id const* match;
	struct device *dev = &pdev->dev;
	struct soc_camera_subdev_desc ssdd;
	int ret;

	match = of_match_device(cap_match, &pdev->dev);
	if (!match) {
		dev_err(dev, "failed to matching of_match node\n");
		return -ENODEV;
	}

	aptina_mt9v022_dev = kzalloc(sizeof(*aptina_mt9v022_dev), GFP_KERNEL);
	if (!aptina_mt9v022_dev) {
		v4l2_err(pdev->dev.driver, "Unable to alloc aptina_mt9v022_dev\n");
		return -ENOMEM;
	}

	if (dev->of_node) {
		fill_config(pdev, &aptina_mt9v022_dev->cfg);
	} else {
		dev_err(dev, "of node not populated\n");
		ret = -ENODEV;
		goto err_free_dev;
	}

	aptina_mt9v022_dev->dma_channel = get_int_prop(pdev->dev.of_node,
			"dma-channel");
	if (!aptina_mt9v022_dev->cfg.num_inputs || !aptina_mt9v022_dev->dma_channel) {
		v4l2_err(pdev->dev.driver, "Unable to get board config\n");
		ret = -ENODEV;
		goto err_free_dev;
	}

	aptina_mt9v022_dev->ppi = ppi_create_instance(pdev,
			aptina_mt9v022_dev->cfg.ppi_info);
	if (!aptina_mt9v022_dev->ppi) {
		v4l2_err(pdev->dev.driver, "Unable to create ppi\n");
		ret = -ENODEV;
		goto err_free_dev;
	}
	aptina_mt9v022_dev->ppi->priv = aptina_mt9v022_dev;

	vb2_dma_contig_set_max_seg_size(&pdev->dev, DMA_BIT_MASK(32));

	vfd = video_device_alloc();
	if (!vfd) {
		ret = -ENOMEM;
		v4l2_err(pdev->dev.driver, "Unable to alloc video device\n");
		goto err_cleanup;
	}

	/* initialize field of video device */
	vfd->release            = video_device_release;
	vfd->fops               = &aptina_mt9v022_fops;
	vfd->ioctl_ops          = &aptina_mt9v022_ioctl_ops;
	vfd->tvnorms            = 0;
	vfd->v4l2_dev           = &aptina_mt9v022_dev->v4l2_dev;
	strncpy(vfd->name, CAPTURE_DRV_NAME, sizeof(vfd->name));
	aptina_mt9v022_dev->video_dev = vfd;

	ret = v4l2_device_register(&pdev->dev, &aptina_mt9v022_dev->v4l2_dev);
	if (ret) {
		v4l2_err(pdev->dev.driver,
				"Unable to register v4l2 device\n");
		goto err_release_vdev;
	}
	v4l2_info(&aptina_mt9v022_dev->v4l2_dev, "v4l2 device registered\n");

	aptina_mt9v022_dev->v4l2_dev.ctrl_handler = &aptina_mt9v022_dev->ctrl_handler;
	ret = v4l2_ctrl_handler_init(&aptina_mt9v022_dev->ctrl_handler, 0);
	if (ret) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"Unable to init control handler\n");
		goto err_unreg_v4l2;
	}

	spin_lock_init(&aptina_mt9v022_dev->lock);
	/* initialize queue */
	q				= &aptina_mt9v022_dev->buffer_queue;
	q->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes			= VB2_MMAP | VB2_DMABUF;
	q->drv_priv			= aptina_mt9v022_dev;
	q->buf_struct_size		= sizeof(struct imager_buffer);
	q->ops 				= &aptina_mt9v022_video_qops;
	q->mem_ops 			= &vb2_dma_contig_memops;
	q->timestamp_flags 		= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_buffers_needed 		= 4;
	q->lock 			= &aptina_mt9v022_dev->mutex;
	q->dev = &pdev->dev;							

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_handler;

	mutex_init(&aptina_mt9v022_dev->mutex);

	/* init video dma queues */
	INIT_LIST_HEAD(&aptina_mt9v022_dev->dma_queue);

	vfd->lock = &aptina_mt9v022_dev->mutex;
	vfd->queue = q; 

	/* register video device */
	ret = video_register_device(aptina_mt9v022_dev->video_dev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"Unable to register video device\n");
		goto err_free_handler;
	}
	video_set_drvdata(aptina_mt9v022_dev->video_dev, aptina_mt9v022_dev);
	v4l2_info(&aptina_mt9v022_dev->v4l2_dev, "video device registered as: %s\n",
			video_device_node_name(vfd));

	/* load up the subdevice */
	i2c_adap = i2c_get_adapter(aptina_mt9v022_dev->cfg.i2c_adapter_id);
	if (!i2c_adap) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"Unable to find i2c adapter\n");
		ret = -ENODEV;
		goto err_unreg_vdev;

	}

	memset(&ssdd, 0, sizeof(ssdd));
	memset(&aptina_mt9v022_dev->cfg.board_info, 0,
			sizeof(aptina_mt9v022_dev->cfg.board_info));
	strlcpy(aptina_mt9v022_dev->cfg.board_info.type,
			aptina_mt9v022_dev->cfg.card_name,
			sizeof(aptina_mt9v022_dev->cfg.board_info.type));
	aptina_mt9v022_dev->cfg.board_info.addr = 0x5C;
	aptina_mt9v022_dev->cfg.board_info.platform_data = &ssdd;

	aptina_mt9v022_dev->sd = v4l2_i2c_new_subdev_board(
			&aptina_mt9v022_dev->v4l2_dev, i2c_adap,
			&aptina_mt9v022_dev->cfg.board_info,
			NULL);
	if (aptina_mt9v022_dev->sd) {
		int i;

		/* update tvnorms from the sub devices */
		for (i = 0; i < aptina_mt9v022_dev->cfg.num_inputs; i++)
			vfd->tvnorms |= aptina_mt9v022_dev->cfg.inputs[i].std;
	} else {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"Unable to register sub device\n");
		ret = -ENODEV;
		goto err_unreg_vdev;
	}

	v4l2_info(&aptina_mt9v022_dev->v4l2_dev, "v4l2 sub device registered\n");
	ret = v4l2_device_register_subdev_nodes(&aptina_mt9v022_dev->v4l2_dev);
	if (ret) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"subdev nodes cannot be registered\n");
	}

	/*
	 * explicitly set input, otherwise some boards
	 * may not work at the state as we expected
	 */
	route = &aptina_mt9v022_dev->cfg.routes[0];
	ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, s_routing,
				route->input, route->output, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD)) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev, "Failed to set input\n");
		goto err_unreg_vdev;
	}
	aptina_mt9v022_dev->cur_input = 0;
	/* if this route has specific config, update ppi control */

	/* now we can probe the default state */
	if (aptina_mt9v022_dev->cfg.inputs[0].capabilities & V4L2_IN_CAP_STD) {
		v4l2_std_id std;
		ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video, g_std, &std);
		if (ret) {
			v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
					"Unable to get std\n");
			goto err_unreg_vdev;
		}
		aptina_mt9v022_dev->std = std;
	}
	if (aptina_mt9v022_dev->cfg.inputs[0].capabilities & V4L2_IN_CAP_DV_TIMINGS) {
		struct v4l2_dv_timings dv_timings;
		ret = v4l2_subdev_call(aptina_mt9v022_dev->sd, video,
				g_dv_timings, &dv_timings);
		if (ret) {
			v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
					"Unable to get dv timings\n");
			goto err_unreg_vdev;
		}
		aptina_mt9v022_dev->dv_timings = dv_timings;
	}
	ret = aptina_mt9v022_init_sensor_formats(aptina_mt9v022_dev);
	if (ret) {
		v4l2_err(&aptina_mt9v022_dev->v4l2_dev,
				"Unable to create sensor formats table\n");
		goto err_unreg_vdev;
	}
	return 0;
err_unreg_vdev:
	video_unregister_device(aptina_mt9v022_dev->video_dev);
	aptina_mt9v022_dev->video_dev = NULL;
err_free_handler:
	v4l2_ctrl_handler_free(&aptina_mt9v022_dev->ctrl_handler);
err_unreg_v4l2:
	v4l2_device_unregister(&aptina_mt9v022_dev->v4l2_dev);
err_release_vdev:
	if (aptina_mt9v022_dev->video_dev)
		video_device_release(aptina_mt9v022_dev->video_dev);
err_cleanup:
	vb2_dma_contig_clear_max_seg_size(&pdev->dev);
	ppi_delete_instance(aptina_mt9v022_dev->ppi);
err_free_dev:
	kfree(aptina_mt9v022_dev);
	return ret;
}

static int aptina_mt9v022_remove(struct platform_device *pdev)
{
	struct v4l2_device *v4l2_dev = platform_get_drvdata(pdev);
	struct aptina_mt9v022_device *aptina_mt9v022_dev =
		container_of(v4l2_dev, struct aptina_mt9v022_device, v4l2_dev);

	aptina_mt9v022_free_sensor_formats(aptina_mt9v022_dev);
	video_unregister_device(aptina_mt9v022_dev->video_dev);
	v4l2_ctrl_handler_free(&aptina_mt9v022_dev->ctrl_handler);
	v4l2_device_unregister(v4l2_dev);
	dma_pool_destroy(aptina_mt9v022_dev->dma_pool);
	vb2_dma_contig_clear_max_seg_size(&pdev->dev);
	ppi_delete_instance(aptina_mt9v022_dev->ppi);
	kfree(aptina_mt9v022_dev);
	return 0;
}

static struct platform_driver aptina_mt9v022_driver =
{
	.driver = { 
			.name = CAPTURE_DRV_NAME,
#ifdef CONFIG_OF
			.of_match_table = cap_match,
#endif
		  },
	.probe  = aptina_mt9v022_probe,
	.remove = aptina_mt9v022_remove,
};
module_platform_driver(aptina_mt9v022_driver);

MODULE_DESCRIPTION("video capture driver for an aptina_mt9v022 on Griffin-Lite");
MODULE_AUTHOR("Michael Glembottzki <Michael.Glembotzki@irisgmbh.de>");
MODULE_LICENSE("GPL v2");
