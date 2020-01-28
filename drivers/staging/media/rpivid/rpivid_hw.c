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

#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <media/videobuf2-core.h>
#include <media/v4l2-mem2mem.h>

#include "rpivid.h"
#include "rpivid_hw.h"

#if !OPT_DEBUG_POLL_IRQ
static irqreturn_t rpivid_irq(int irq, void *data)
{
	struct rpivid_dev * const dev = data;
	struct rpivid_ctx *ctx;
	__u32 ictrl;

	/* ** The fact that this is valid implies a depressing level of single
	   threading in the m2m job Q */
	/* We keep individual ctx for each irq but this is a useful sanity
	   check on whether we are alive or not */
	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (!ctx) {
		v4l2_err(&dev->v4l2_dev,
			 "Instance released before the end of transaction\n");
		return IRQ_NONE;
	}

	ictrl = irq_read(dev, ARG_IC_ICTRL);
	if ((ictrl & ARG_IC_ICTRL_ALL_IRQ_MASK) == 0) {
		v4l2_warn(&dev->v4l2_dev,
			 "IRQ but no IRQ bits set\n");
		return IRQ_NONE;
	}

	// Cancel any/all irqs
	irq_write(dev, ARG_IC_ICTRL, ictrl & ~ARG_IC_ICTRL_SET_ZERO_MASK);

	// Service Active2 before Active1 so Phase 1 can transition to Phase 2 without delay
	if ((ictrl & ARG_IC_ICTRL_ACTIVE2_INT_SET) != 0 && dev->irq_active2)
		dev->irq_active2(dev, dev->ctx_active2);
	if ((ictrl & ARG_IC_ICTRL_ACTIVE1_INT_SET) != 0 && dev->irq_active1)
		dev->irq_active1(dev, dev->ctx_active1);

	return IRQ_HANDLED;
}
#endif

void rpivid_hw_irq_active1_release(struct rpivid_dev *dev)
{
	unsigned long flags;
	spin_lock_irqsave(&dev->irq_lock, flags);
	dev->irq_active1 = 0;
	spin_unlock_irqrestore(&dev->irq_lock, flags);
}

int rpivid_hw_irq_active1_claim(struct rpivid_dev *dev, rpivid_irq_callback cb, void * v)
{
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&dev->irq_lock, flags);
	if (dev->irq_active1)
		ret = -1;
	else
	{
		dev->irq_active1 = cb;
		dev->ctx_active1 = v;
	}
	spin_unlock_irqrestore(&dev->irq_lock, flags);
	return ret;
}

void rpivid_hw_irq_active2_release(struct rpivid_dev *dev)
{
	unsigned long flags;
	spin_lock_irqsave(&dev->irq_lock, flags);
	dev->irq_active2 = 0;
	spin_unlock_irqrestore(&dev->irq_lock, flags);
}

int rpivid_hw_irq_active2_claim(struct rpivid_dev *dev, rpivid_irq_callback cb, void * v)
{
	unsigned long flags;
	int ret = 0;
	spin_lock_irqsave(&dev->irq_lock, flags);
	if (dev->irq_active2)
		ret = -1;
	else
	{
		dev->irq_active2 = cb;
		dev->ctx_active2 = v;
	}
	spin_unlock_irqrestore(&dev->irq_lock, flags);
	return ret;
}

int rpivid_hw_probe(struct rpivid_dev *dev)
{
	int irq_dec;
	int ret = 0;

	dev->base_irq = devm_platform_ioremap_resource(dev->pdev, 0);
	if (IS_ERR(dev->base_irq)) {
		dev_err(dev->dev, "Failed to map IRQ registers\n");

		ret = PTR_ERR(dev->base_irq);
		goto err_sram;
	}

	dev->base_h265 = devm_platform_ioremap_resource(dev->pdev, 1);
	if (IS_ERR(dev->base_h265)) {
		dev_err(dev->dev, "Failed to map H265 registers\n");

		ret = PTR_ERR(dev->base_h265);
		goto err_sram;
	}

	// Disable IRQs & reset anything pending
	{
		__u32 irq_stat;
		irq_write(dev, 0, ARG_IC_ICTRL_ACTIVE1_EN_SET | ARG_IC_ICTRL_ACTIVE2_EN_SET);
		irq_stat = irq_read(dev, 0);
		irq_write(dev, 0, irq_stat);
		v4l2_info(&dev->v4l2_dev, "Initial irq=%#x\n", irq_stat);
	}

#if !OPT_DEBUG_POLL_IRQ
	irq_dec = platform_get_irq(dev->pdev, 0);
	if (irq_dec <= 0)
		return irq_dec;
	ret = devm_request_irq(dev->dev, irq_dec, rpivid_irq,
			       0, dev_name(dev->dev), dev);
	if (ret) {
		dev_err(dev->dev, "Failed to request IRQ\n");

		return ret;
	}
#endif
err_sram:
	return ret;
}

void rpivid_hw_remove(struct rpivid_dev *dev)
{
	// IRQ auto freed on unload so no need to do it here
}

