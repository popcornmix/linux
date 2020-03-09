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

#define OPT_DEBUG_POLL_IRQ  0

#define RPIVID_NAME			"rpivid"

#define RPIVID_CAPABILITY_UNTILED	BIT(0)
#define RPIVID_CAPABILITY_H265_DEC	BIT(1)

#define RPIVID_QUIRK_NO_DMA_OFFSET	BIT(0)

#define RPIVID_SRC_PIXELFORMAT_DEFAULT	V4L2_PIX_FMT_HEVC_SLICE

enum rpivid_codec {
	RPIVID_CODEC_H265,
	RPIVID_CODEC_LAST,
};

enum rpivid_irq_status {
	RPIVID_IRQ_NONE,
	RPIVID_IRQ_ERROR,
	RPIVID_IRQ_OK,
};

struct rpivid_control {
	struct v4l2_ctrl_config cfg;
	enum rpivid_codec	codec;
	unsigned char		required:1;
};

struct rpivid_h265_run {
	const struct v4l2_ctrl_hevc_sps			*sps;
	const struct v4l2_ctrl_hevc_pps			*pps;
	const struct v4l2_ctrl_hevc_slice_params	*slice_params;
	const struct v4l2_ctrl_hevc_scaling_matrix	*scaling_matrix;
};

struct rpivid_run {
	struct vb2_v4l2_buffer	*src;
	struct vb2_v4l2_buffer	*dst;

	struct rpivid_h265_run	h265;
};

struct rpivid_buffer {
	struct v4l2_m2m_buffer          m2m_buf;

};

struct rpivid_dec_state_s;
struct rpivid_dec_env_s;
#define RPIVID_DEC_ENV_COUNT 3

struct rpivid_gptr
{
	size_t size;
	__u8 * ptr;
	dma_addr_t addr;
	unsigned long attrs;
};

struct rpivid_dev;
typedef void (*rpivid_irq_callback)(struct rpivid_dev * dev, void * ctx);

struct rpivid_q_aux_s;
#define RPIVID_AUX_ENT_COUNT VB2_MAX_FRAME

#define RPIVID_P2BUF_COUNT 2

struct rpivid_ctx {
	struct v4l2_fh			fh;
	struct rpivid_dev		*dev;

	struct v4l2_pix_format		src_fmt;
	struct v4l2_pix_format		dst_fmt;
	enum rpivid_codec		current_codec;
	int dst_fmt_set;

	struct v4l2_ctrl_handler	hdl;
	struct v4l2_ctrl		**ctrls;

	/* Decode state - stateless decoder my *** */
	/* state contains stuff that is only needed in phase0
	   it could be held in dec_env but that would be wasteful
	 */
	struct rpivid_dec_state_s * state;
	struct rpivid_dec_env_s * dec0;

	spinlock_t dec_lock;
	struct rpivid_dec_env_s * dec_free;
	struct rpivid_dec_env_s * dec_pool;
	// Some of these should be in dev
	struct rpivid_gptr bitbufs[1];  // Will be 2
	struct rpivid_gptr cmdbufs[1];  // Will be 2
	unsigned int max_pu_msgs;
	unsigned int p2idx;
	atomic_t p2out;
	struct rpivid_gptr p2bufs[RPIVID_P2BUF_COUNT];   // Will be 2

	spinlock_t aux_lock;
	struct rpivid_q_aux_s * aux_free;
	struct rpivid_q_aux_s * aux_ents[RPIVID_AUX_ENT_COUNT];

	unsigned int colmv_stride;
	unsigned int colmv_picsize;
};

struct rpivid_dec_ops {
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

struct rpivid_hw_irq_ent;
struct rpivid_hw_irq_ctrl {
	spinlock_t lock;
	struct rpivid_hw_irq_ent * claim;
	struct rpivid_hw_irq_ent * tail;

	struct rpivid_hw_irq_ent * irq;		// Ent for pending irq - also prevents sched
	int			no_sched;	// Non-zero => do not start a new job - outer layer sched pending
	bool			thread_reqed;	// Thread cb requested

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

	void __iomem		*base_irq;
	void __iomem		*base_h265;

	struct rpivid_hw_irq_ctrl ic_active1;
	struct rpivid_hw_irq_ctrl ic_active2;
};

extern struct rpivid_dec_ops rpivid_dec_ops_h265;

void *rpivid_find_control_data(struct rpivid_ctx *ctx, u32 id);

#endif
