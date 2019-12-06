/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef _RPIVID_H_
#define _RPIVID_H_

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/platform_device.h>

#define RPIVID_NAME			"rpivid"

#define RPIVID_CAPABILITY_UNTILED	BIT(0)
#define RPIVID_CAPABILITY_H265_DEC	BIT(1)

#define RPIVID_QUIRK_NO_DMA_OFFSET	BIT(0)

enum rpivid_codec {
	RPIVID_CODEC_MPEG2,
	RPIVID_CODEC_H264,
	RPIVID_CODEC_H265,
	RPIVID_CODEC_LAST,
};

enum rpivid_irq_status {
	RPIVID_IRQ_NONE,
	RPIVID_IRQ_ERROR,
	RPIVID_IRQ_OK,
};

enum rpivid_h264_pic_type {
	RPIVID_H264_PIC_TYPE_FRAME	= 0,
	RPIVID_H264_PIC_TYPE_FIELD,
	RPIVID_H264_PIC_TYPE_MBAFF,
};

struct rpivid_control {
	struct v4l2_ctrl_config cfg;
	enum rpivid_codec	codec;
	unsigned char		required:1;
};

struct rpivid_h264_run {
	const struct v4l2_ctrl_h264_decode_params	*decode_params;
	const struct v4l2_ctrl_h264_pps			*pps;
	const struct v4l2_ctrl_h264_scaling_matrix	*scaling_matrix;
	const struct v4l2_ctrl_h264_slice_params	*slice_params;
	const struct v4l2_ctrl_h264_sps			*sps;
};

struct rpivid_mpeg2_run {
	const struct v4l2_ctrl_mpeg2_slice_params	*slice_params;
	const struct v4l2_ctrl_mpeg2_quantization	*quantization;
};

struct rpivid_h265_run {
	const struct v4l2_ctrl_hevc_sps			*sps;
	const struct v4l2_ctrl_hevc_pps			*pps;
	const struct v4l2_ctrl_hevc_slice_params	*slice_params;
};

struct rpivid_run {
	struct vb2_v4l2_buffer	*src;
	struct vb2_v4l2_buffer	*dst;

	union {
		struct rpivid_h264_run	h264;
		struct rpivid_mpeg2_run	mpeg2;
		struct rpivid_h265_run	h265;
	};
};

struct rpivid_buffer {
	struct v4l2_m2m_buffer          m2m_buf;

	union {
		struct {
			unsigned int			position;
			enum rpivid_h264_pic_type	pic_type;
		} h264;
	} codec;
};

struct rpivid_ctx {
	struct v4l2_fh			fh;
	struct rpivid_dev		*dev;

	struct v4l2_pix_format		src_fmt;
	struct v4l2_pix_format		dst_fmt;
	enum rpivid_codec		current_codec;

	struct v4l2_ctrl_handler	hdl;
	struct v4l2_ctrl		**ctrls;

	union {
		struct {
			void		*mv_col_buf;
			dma_addr_t	mv_col_buf_dma;
			ssize_t		mv_col_buf_field_size;
			ssize_t		mv_col_buf_size;
			void		*pic_info_buf;
			dma_addr_t	pic_info_buf_dma;
			ssize_t		pic_info_buf_size;
			void		*neighbor_info_buf;
			dma_addr_t	neighbor_info_buf_dma;
			void		*deblk_buf;
			dma_addr_t	deblk_buf_dma;
			ssize_t		deblk_buf_size;
			void		*intra_pred_buf;
			dma_addr_t	intra_pred_buf_dma;
			ssize_t		intra_pred_buf_size;
		} h264;
		struct {
			void		*mv_col_buf;
			dma_addr_t	mv_col_buf_addr;
			ssize_t		mv_col_buf_size;
			ssize_t		mv_col_buf_unit_size;
			void		*neighbor_info_buf;
			dma_addr_t	neighbor_info_buf_addr;
		} h265;
	} codec;
};

struct rpivid_dec_ops {
	void (*irq_clear)(struct rpivid_ctx *ctx);
	void (*irq_disable)(struct rpivid_ctx *ctx);
	enum rpivid_irq_status (*irq_status)(struct rpivid_ctx *ctx);
	void (*setup)(struct rpivid_ctx *ctx, struct rpivid_run *run);
	int (*start)(struct rpivid_ctx *ctx);
	void (*stop)(struct rpivid_ctx *ctx);
	void (*trigger)(struct rpivid_ctx *ctx);
};

struct rpivid_variant {
	unsigned int	capabilities;
	unsigned int	quirks;
	unsigned int	mod_rate;
};

struct rpivid_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct media_device	mdev;
	struct media_pad	pad[2];
	struct platform_device	*pdev;
	struct device		*dev;
	struct v4l2_m2m_dev	*m2m_dev;
	struct rpivid_dec_ops	*dec_ops[RPIVID_CODEC_LAST];

	/* Device file mutex */
	struct mutex		dev_mutex;

	void __iomem		*base;

	struct clk		*mod_clk;
	struct clk		*ahb_clk;
	struct clk		*ram_clk;

	struct reset_control	*rstc;

	unsigned int		capabilities;
};

extern struct rpivid_dec_ops rpivid_dec_ops_mpeg2;
extern struct rpivid_dec_ops rpivid_dec_ops_h264;
extern struct rpivid_dec_ops rpivid_dec_ops_h265;

static inline void rpivid_write(struct rpivid_dev *dev, u32 reg, u32 val)
{
	writel(val, dev->base + reg);
}

static inline u32 rpivid_read(struct rpivid_dev *dev, u32 reg)
{
	return readl(dev->base + reg);
}

static inline dma_addr_t rpivid_buf_addr(struct vb2_buffer *buf,
					 struct v4l2_pix_format *pix_fmt,
					 unsigned int plane)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(buf, 0);

	return addr + (pix_fmt ? (dma_addr_t)pix_fmt->bytesperline *
	       pix_fmt->height * plane : 0);
}

static inline dma_addr_t rpivid_dst_buf_addr(struct rpivid_ctx *ctx,
					     int index, unsigned int plane)
{
	struct vb2_buffer *buf = NULL;
	struct vb2_queue *vq;

	if (index < 0)
		return 0;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (vq)
		buf = vb2_get_buffer(vq, index);

	return buf ? rpivid_buf_addr(buf, &ctx->dst_fmt, plane) : 0;
}

static inline struct rpivid_buffer *
vb2_v4l2_to_rpivid_buffer(const struct vb2_v4l2_buffer *p)
{
	return container_of(p, struct rpivid_buffer, m2m_buf.vb);
}

static inline struct rpivid_buffer *
vb2_to_rpivid_buffer(const struct vb2_buffer *p)
{
	return vb2_v4l2_to_rpivid_buffer(to_vb2_v4l2_buffer(p));
}

void *rpivid_find_control_data(struct rpivid_ctx *ctx, u32 id);

#endif
