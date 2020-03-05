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

static void pre_irq(struct rpivid_dev *dev,
                              rpivid_hw_irq_ent *ient,
                              rpivid_irq_callback cb, void * v,
					 struct rpivid_hw_irq_ctrl * ictl)
{
	unsigned long flags;

	if (ictl->irq) {
		v4l2_err(&dev->v4l2_dev, "Attempt to claim IRQ when already claimed\n");
		return;
	}

	ient->cb = cb;
	ient->v = v;

	// Not sure this lock is actually required
	spin_lock_irqsave(&ictl->lock, flags);
	ictl->irq = ient;
	spin_unlock_irqrestore(&ictl->lock, flags);
}

static void sched_claim(struct rpivid_dev * const dev,
					 struct rpivid_hw_irq_ctrl * const ictl)
{
	for (;;) {
		rpivid_hw_irq_ent * ient = NULL;
		unsigned long flags;

		spin_lock_irqsave(&ictl->lock, flags);

		if (--ictl->no_sched <= 0) {
			ient = ictl->claim;
			if (!ictl->irq && ient != NULL) {
				ictl->claim = ient->next;
				ictl->no_sched = 1;
			}
		}

		spin_unlock_irqrestore(&ictl->lock, flags);

		if (ient == NULL)
			break;

		ient->cb(dev, ient->v);
	}

}

// Called in irq context
static void do_irq(struct rpivid_dev * const dev,
					 struct rpivid_hw_irq_ctrl * const ictl)
{
	rpivid_hw_irq_ent * ient;
	unsigned long flags;

	spin_lock_irqsave(&ictl->lock, flags);
	ient = ictl->irq;
	if (ient) {
		ictl->no_sched++;
		ictl->irq = NULL;
	}
	spin_unlock_irqrestore(&ictl->lock, flags);

	if (ient) {
		ient->cb(dev, ient->v);

		sched_claim(dev, ictl);
	}
}

static void do_claim(struct rpivid_dev * const dev,
                              rpivid_hw_irq_ent * ient,
                              const rpivid_irq_callback cb, void * const v,
					 struct rpivid_hw_irq_ctrl * const ictl)
{
	unsigned long flags;

	ient->next = NULL;
	ient->cb = cb;
	ient->v = v;

	spin_lock_irqsave(&ictl->lock, flags);

	// If we have a Q then add to end
	if (ictl->claim != NULL) {
		ictl->tail->next = ient;
		ictl->tail = ient;
		ient = NULL;
	}
	// Empty Q but other activity in progress so Q
	else if (ictl->no_sched || ictl->irq) {
		ictl->claim = ient;
		ictl->tail = ient;
		ient = NULL;
	}
	// Nothing else going on - schedule immediately and
	// prevent anything else scheduling claims
	else {
		ictl->no_sched = 1;
	}

	spin_unlock_irqrestore(&ictl->lock, flags);

	if (ient != NULL) {
		ient->cb(dev, ient->v);

		sched_claim(dev, ictl);
	}
}

static void ictl_init(struct rpivid_hw_irq_ctrl * const ictl)
{
	spin_lock_init(&ictl->lock);
	ictl->claim = NULL;
	ictl->tail = NULL;
	ictl->irq = NULL;
	ictl->no_sched = 0;
}

static void ictl_uninit(struct rpivid_hw_irq_ctrl * const ictl)
{
	// Nothing to do
}


#if !OPT_DEBUG_POLL_IRQ
static irqreturn_t rpivid_irq(int irq, void *data)
{
	struct rpivid_dev * const dev = data;
	__u32 ictrl;

	ictrl = irq_read(dev, ARG_IC_ICTRL);
	if ((ictrl & ARG_IC_ICTRL_ALL_IRQ_MASK) == 0) {
		v4l2_warn(&dev->v4l2_dev, "IRQ but no IRQ bits set\n");
		return IRQ_NONE;
	}

	// Cancel any/all irqs
	irq_write(dev, ARG_IC_ICTRL, ictrl & ~ARG_IC_ICTRL_SET_ZERO_MASK);

	// Service Active2 before Active1 so Phase 1 can transition to Phase 2 without delay
	if ((ictrl & ARG_IC_ICTRL_ACTIVE2_INT_SET) != 0)
		do_irq(dev, &dev->ic_active2);
	if ((ictrl & ARG_IC_ICTRL_ACTIVE1_INT_SET) != 0)
		do_irq(dev, &dev->ic_active1);

	return IRQ_HANDLED;
}
#endif


void rpivid_hw_irq_active1_claim(struct rpivid_dev *dev,
                                rpivid_hw_irq_ent *ient,
                                rpivid_irq_callback ready_cb, void * ctx)
{
	do_claim(dev, ient, ready_cb, ctx, &dev->ic_active1);
}

void rpivid_hw_irq_active1_irq(struct rpivid_dev *dev,
                              rpivid_hw_irq_ent *ient,
                              rpivid_irq_callback irq_cb, void * ctx)
{
	pre_irq(dev, ient, irq_cb, ctx, &dev->ic_active1);
}

void rpivid_hw_irq_active2_claim(struct rpivid_dev *dev,
                                rpivid_hw_irq_ent *ient,
                                rpivid_irq_callback ready_cb, void * ctx)
{
	do_claim(dev, ient, ready_cb, ctx, &dev->ic_active2);
}

void rpivid_hw_irq_active2_irq(struct rpivid_dev *dev,
                              rpivid_hw_irq_ent *ient,
                              rpivid_irq_callback irq_cb, void * ctx)
{
	pre_irq(dev, ient, irq_cb, ctx, &dev->ic_active2);
}


int rpivid_hw_probe(struct rpivid_dev *dev)
{
	int irq_dec;
	int ret = 0;

	ictl_init(&dev->ic_active1);
	ictl_init(&dev->ic_active2);

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
	ictl_uninit(&dev->ic_active1);
	ictl_uninit(&dev->ic_active2);
}

