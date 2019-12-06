// SPDX-License-Identifier: GPL-2.0
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 *
 * Based on the vim2m driver, that is:
 *
 * Copyright (c) 2009-2010 Samsung Electronics Co., Ltd.
 * Pawel Osciak, <pawel@osciak.com>
 * Marek Szyprowski, <m.szyprowski@samsung.com>
 */

#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>

#include "rpivid.h"
#include "rpivid_video.h"
#include "rpivid_dec.h"

#define RPIVID_DECODE_SRC	BIT(0)
#define RPIVID_DECODE_DST	BIT(1)

#define RPIVID_MIN_WIDTH	16U
#define RPIVID_MIN_HEIGHT	16U
#define RPIVID_MAX_WIDTH	4096U
#define RPIVID_MAX_HEIGHT	2304U

static struct rpivid_format rpivid_formats[] = {
	{
		.pixelformat	= V4L2_PIX_FMT_MPEG2_SLICE,
		.directions	= RPIVID_DECODE_SRC,
	},
	{
		.pixelformat	= V4L2_PIX_FMT_H264_SLICE,
		.directions	= RPIVID_DECODE_SRC,
	},
	{
		.pixelformat	= V4L2_PIX_FMT_HEVC_SLICE,
		.directions	= RPIVID_DECODE_SRC,
		.capabilities	= RPIVID_CAPABILITY_H265_DEC,
	},
	{
		.pixelformat	= V4L2_PIX_FMT_SUNXI_TILED_NV12,
		.directions	= RPIVID_DECODE_DST,
	},
	{
		.pixelformat	= V4L2_PIX_FMT_NV12,
		.directions	= RPIVID_DECODE_DST,
		.capabilities	= RPIVID_CAPABILITY_UNTILED,
	},
};

#define RPIVID_FORMATS_COUNT	ARRAY_SIZE(rpivid_formats)

static inline struct rpivid_ctx *rpivid_file2ctx(struct file *file)
{
	return container_of(file->private_data, struct rpivid_ctx, fh);
}

static struct rpivid_format *rpivid_find_format(u32 pixelformat, u32 directions,
						unsigned int capabilities)
{
	struct rpivid_format *first_valid_fmt = NULL;
	struct rpivid_format *fmt;
	unsigned int i;

	for (i = 0; i < RPIVID_FORMATS_COUNT; i++) {
		fmt = &rpivid_formats[i];

		if ((fmt->capabilities & capabilities) != fmt->capabilities ||
		    !(fmt->directions & directions))
			continue;

		if (fmt->pixelformat == pixelformat)
			break;

		if (!first_valid_fmt)
			first_valid_fmt = fmt;
	}

	if (i == RPIVID_FORMATS_COUNT)
		return first_valid_fmt;

	return &rpivid_formats[i];
}

void rpivid_prepare_format(struct v4l2_pix_format *pix_fmt)
{
	unsigned int width = pix_fmt->width;
	unsigned int height = pix_fmt->height;
	unsigned int sizeimage = pix_fmt->sizeimage;
	unsigned int bytesperline = pix_fmt->bytesperline;

	pix_fmt->field = V4L2_FIELD_NONE;

	/* Limit to hardware min/max. */
	width = clamp(width, RPIVID_MIN_WIDTH, RPIVID_MAX_WIDTH);
	height = clamp(height, RPIVID_MIN_HEIGHT, RPIVID_MAX_HEIGHT);

	switch (pix_fmt->pixelformat) {
	case V4L2_PIX_FMT_MPEG2_SLICE:
	case V4L2_PIX_FMT_H264_SLICE:
	case V4L2_PIX_FMT_HEVC_SLICE:
		/* Zero bytes per line for encoded source. */
		bytesperline = 0;
		/* Choose some minimum size since this can't be 0 */
		sizeimage = max_t(u32, SZ_1K, sizeimage);
		break;

	case V4L2_PIX_FMT_SUNXI_TILED_NV12:
		/* 32-aligned stride. */
		bytesperline = ALIGN(width, 32);

		/* 32-aligned height. */
		height = ALIGN(height, 32);

		/* Luma plane size. */
		sizeimage = bytesperline * height;

		/* Chroma plane size. */
		sizeimage += bytesperline * height / 2;

		break;

	case V4L2_PIX_FMT_NV12:
		/* 16-aligned stride. */
		bytesperline = ALIGN(width, 16);

		/* 16-aligned height. */
		height = ALIGN(height, 16);

		/* Luma plane size. */
		sizeimage = bytesperline * height;

		/* Chroma plane size. */
		sizeimage += bytesperline * height / 2;

		break;
	}

	pix_fmt->width = width;
	pix_fmt->height = height;

	pix_fmt->bytesperline = bytesperline;
	pix_fmt->sizeimage = sizeimage;
}

static int rpivid_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strscpy(cap->driver, RPIVID_NAME, sizeof(cap->driver));
	strscpy(cap->card, RPIVID_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info),
		 "platform:%s", RPIVID_NAME);

	return 0;
}

static int rpivid_enum_fmt(struct file *file, struct v4l2_fmtdesc *f,
			   u32 direction)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);
	struct rpivid_dev *dev = ctx->dev;
	unsigned int capabilities = dev->capabilities;
	struct rpivid_format *fmt;
	unsigned int i, index;

	/* Index among formats that match the requested direction. */
	index = 0;

	for (i = 0; i < RPIVID_FORMATS_COUNT; i++) {
		fmt = &rpivid_formats[i];

		if (fmt->capabilities && (fmt->capabilities & capabilities) !=
		    fmt->capabilities)
			continue;

		if (!(rpivid_formats[i].directions & direction))
			continue;

		if (index == f->index)
			break;

		index++;
	}

	/* Matched format. */
	if (i < RPIVID_FORMATS_COUNT) {
		f->pixelformat = rpivid_formats[i].pixelformat;

		return 0;
	}

	return -EINVAL;
}

static int rpivid_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return rpivid_enum_fmt(file, f, RPIVID_DECODE_DST);
}

static int rpivid_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return rpivid_enum_fmt(file, f, RPIVID_DECODE_SRC);
}

static int rpivid_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);

	f->fmt.pix = ctx->dst_fmt;
	return 0;
}

static int rpivid_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);

	f->fmt.pix = ctx->src_fmt;
	return 0;
}

static int rpivid_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);
	struct rpivid_dev *dev = ctx->dev;
	struct v4l2_pix_format *pix_fmt = &f->fmt.pix;
	struct rpivid_format *fmt =
		rpivid_find_format(pix_fmt->pixelformat, RPIVID_DECODE_DST,
				   dev->capabilities);

	if (!fmt)
		return -EINVAL;

	pix_fmt->pixelformat = fmt->pixelformat;
	rpivid_prepare_format(pix_fmt);

	return 0;
}

static int rpivid_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);
	struct rpivid_dev *dev = ctx->dev;
	struct v4l2_pix_format *pix_fmt = &f->fmt.pix;
	struct rpivid_format *fmt =
		rpivid_find_format(pix_fmt->pixelformat, RPIVID_DECODE_SRC,
				   dev->capabilities);

	if (!fmt)
		return -EINVAL;

	pix_fmt->pixelformat = fmt->pixelformat;
	rpivid_prepare_format(pix_fmt);

	return 0;
}

static int rpivid_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);
	struct rpivid_dev *dev = ctx->dev;
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = rpivid_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	ctx->dst_fmt = f->fmt.pix;

	// #### FIXME
//	rpivid_dst_format_set(dev, &ctx->dst_fmt);

	return 0;
}

static int rpivid_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct rpivid_ctx *ctx = rpivid_file2ctx(file);
	struct vb2_queue *vq;
	int ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (vb2_is_busy(vq))
		return -EBUSY;

	ret = rpivid_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	ctx->src_fmt = f->fmt.pix;

	switch (ctx->src_fmt.pixelformat) {
	case V4L2_PIX_FMT_H264_SLICE:
		vq->subsystem_flags |=
			VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		break;
	default:
		vq->subsystem_flags &=
			~VB2_V4L2_FL_SUPPORTS_M2M_HOLD_CAPTURE_BUF;
		break;
	}

	/* Propagate colorspace information to capture. */
	ctx->dst_fmt.colorspace = f->fmt.pix.colorspace;
	ctx->dst_fmt.xfer_func = f->fmt.pix.xfer_func;
	ctx->dst_fmt.ycbcr_enc = f->fmt.pix.ycbcr_enc;
	ctx->dst_fmt.quantization = f->fmt.pix.quantization;

	return 0;
}

const struct v4l2_ioctl_ops rpivid_ioctl_ops = {
	.vidioc_querycap		= rpivid_querycap,

	.vidioc_enum_fmt_vid_cap	= rpivid_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= rpivid_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= rpivid_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= rpivid_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out	= rpivid_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out		= rpivid_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out		= rpivid_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out		= rpivid_s_fmt_vid_out,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd		= v4l2_m2m_ioctl_stateless_try_decoder_cmd,
	.vidioc_decoder_cmd		= v4l2_m2m_ioctl_stateless_decoder_cmd,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

static int rpivid_queue_setup(struct vb2_queue *vq, unsigned int *nbufs,
			      unsigned int *nplanes, unsigned int sizes[],
			      struct device *alloc_devs[])
{
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format *pix_fmt;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		pix_fmt = &ctx->src_fmt;
	else
		pix_fmt = &ctx->dst_fmt;

	if (*nplanes) {
		if (sizes[0] < pix_fmt->sizeimage)
			return -EINVAL;
	} else {
		sizes[0] = pix_fmt->sizeimage;
		*nplanes = 1;
	}

	return 0;
}

static void rpivid_queue_cleanup(struct vb2_queue *vq, u32 state)
{
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vbuf;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(vq->type))
			vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vbuf)
			return;

		v4l2_ctrl_request_complete(vbuf->vb2_buf.req_obj.req,
					   &ctx->hdl);
		v4l2_m2m_buf_done(vbuf, state);
	}
}

static int rpivid_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	vbuf->field = V4L2_FIELD_NONE;
	return 0;
}

static int rpivid_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vq);
	struct v4l2_pix_format *pix_fmt;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		pix_fmt = &ctx->src_fmt;
	else
		pix_fmt = &ctx->dst_fmt;

	if (vb2_plane_size(vb, 0) < pix_fmt->sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, pix_fmt->sizeimage);

	return 0;
}

static int rpivid_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vq);
	struct rpivid_dev *dev = ctx->dev;
	int ret = 0;

	switch (ctx->src_fmt.pixelformat) {
	case V4L2_PIX_FMT_MPEG2_SLICE:
		ctx->current_codec = RPIVID_CODEC_MPEG2;
		break;

	case V4L2_PIX_FMT_H264_SLICE:
		ctx->current_codec = RPIVID_CODEC_H264;
		break;

	case V4L2_PIX_FMT_HEVC_SLICE:
		ctx->current_codec = RPIVID_CODEC_H265;
		break;

	default:
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    dev->dec_ops[ctx->current_codec]->start)
		ret = dev->dec_ops[ctx->current_codec]->start(ctx);

	if (ret)
		rpivid_queue_cleanup(vq, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void rpivid_stop_streaming(struct vb2_queue *vq)
{
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vq);
	struct rpivid_dev *dev = ctx->dev;

	if (V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    dev->dec_ops[ctx->current_codec]->stop)
		dev->dec_ops[ctx->current_codec]->stop(ctx);

	rpivid_queue_cleanup(vq, VB2_BUF_STATE_ERROR);
}

static void rpivid_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static void rpivid_buf_request_complete(struct vb2_buffer *vb)
{
	struct rpivid_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_ctrl_request_complete(vb->req_obj.req, &ctx->hdl);
}

static struct vb2_ops rpivid_qops = {
	.queue_setup		= rpivid_queue_setup,
	.buf_prepare		= rpivid_buf_prepare,
	.buf_queue		= rpivid_buf_queue,
	.buf_out_validate	= rpivid_buf_out_validate,
	.buf_request_complete	= rpivid_buf_request_complete,
	.start_streaming	= rpivid_start_streaming,
	.stop_streaming		= rpivid_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

int rpivid_queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct rpivid_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct rpivid_buffer);
	src_vq->min_buffers_needed = 1;
	src_vq->ops = &rpivid_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->dev->dev_mutex;
	src_vq->dev = ctx->dev->dev;
	src_vq->supports_requests = true;
	src_vq->requires_requests = true;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct rpivid_buffer);
	dst_vq->min_buffers_needed = 1;
	dst_vq->ops = &rpivid_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->dev->dev_mutex;
	dst_vq->dev = ctx->dev->dev;

	return vb2_queue_init(dst_vq);
}
