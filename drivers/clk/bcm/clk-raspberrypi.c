// SPDX-License-Identifier: GPL-2.0+
/*
 * Raspberry Pi driver for firmware controlled clocks
 *
 * Even though clk-bcm2835 provides an interface to the hardware registers for
 * the system clocks we've had to factor out 'pllb' as the firmware 'owns' it.
 * We're not allowed to change it directly as we might race with the
 * over-temperature and under-voltage protections provided by the firmware.
 *
 * Copyright (C) 2019 Nicolas Saenz Julienne <nsaenzjulienne@suse.de>
 */

#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <soc/bcm2835/raspberrypi-firmware.h>

#define RPI_FIRMWARE_ARM_CLK_ID		0x00000003
#define RPI_FIRMWARE_V3D_CLK_ID		0x00000005

#define RPI_FIRMWARE_STATE_ENABLE_BIT	BIT(0)
#define RPI_FIRMWARE_STATE_WAIT_BIT	BIT(1)

/*
 * Even though the firmware interface alters 'pllb' the frequencies are
 * provided as per 'pllb_arm'. We need to scale before passing them trough.
 */
#define RPI_FIRMWARE_PLLB_ARM_DIV_RATE	2

#define A2W_PLL_FRAC_BITS		20

struct raspberrypi_pll {
	const char *clock_name, *parent_name, *divider_name, *dev_id;
	u32 clock_id;
	u32 mult, div;
};

struct raspberrypi_clk {
	struct device *dev;
	struct rpi_firmware *firmware;
	struct platform_device *cpufreq;

	unsigned long min_rate;
	unsigned long max_rate;

	struct clk_hw pllb;
	struct clk_hw *pllb_arm;
	struct clk_lookup *pllb_arm_lookup;
	const struct raspberrypi_pll *pll_info;
};


static const struct raspberrypi_pll plls[] = {
	{"pllb", "osc", "pllb_arm", "cpu0", RPI_FIRMWARE_ARM_CLK_ID, 1, 2},
	{"v3d", "osc", "", "", RPI_FIRMWARE_V3D_CLK_ID, 0, 0},
};

/*
 * Structure of the message passed to Raspberry Pi's firmware in order to
 * change clock rates. The 'disable_turbo' option is only available to the ARM
 * clock (pllb) which we enable by default as turbo mode will alter multiple
 * clocks at once.
 *
 * Even though we're able to access the clock registers directly we're bound to
 * use the firmware interface as the firmware ultimately takes care of
 * mitigating overheating/undervoltage situations and we would be changing
 * frequencies behind his back.
 *
 * For more information on the firmware interface check:
 * https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
 */
struct raspberrypi_firmware_prop {
	__le32 id;
	__le32 val;
	__le32 disable_turbo;
} __packed;

static int raspberrypi_clock_property(struct rpi_firmware *firmware, u32 tag,
				      u32 clk, u32 *val)
{
	struct raspberrypi_firmware_prop msg = {
		.id = cpu_to_le32(clk),
		.val = cpu_to_le32(*val),
		.disable_turbo = cpu_to_le32(1),
	};
	int ret;

	ret = rpi_firmware_property(firmware, tag, &msg, sizeof(msg));
	if (ret)
		return ret;

	*val = le32_to_cpu(msg.val);

	return 0;
}

static int raspberrypi_fw_pll_is_on(struct clk_hw *hw)
{
	struct raspberrypi_clk *rpi = container_of(hw, struct raspberrypi_clk,
						   pllb);
	u32 val = 0;
	int ret;

	ret = raspberrypi_clock_property(rpi->firmware,
					 RPI_FIRMWARE_GET_CLOCK_STATE,
					 rpi->pll_info->clock_id, &val);
	if (ret)
		return 0;

	return !!(val & RPI_FIRMWARE_STATE_ENABLE_BIT);
}


static unsigned long raspberrypi_fw_pll_get_rate(struct clk_hw *hw,
						 unsigned long parent_rate)
{
	struct raspberrypi_clk *rpi = container_of(hw, struct raspberrypi_clk,
						   pllb);
	u32 val = 0;
	int ret;

	ret = raspberrypi_clock_property(rpi->firmware,
					 RPI_FIRMWARE_GET_CLOCK_RATE,
					 rpi->pll_info->clock_id,
					 &val);
	if (ret)
		return ret;

	return val * RPI_FIRMWARE_PLLB_ARM_DIV_RATE;
}

static int raspberrypi_fw_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				       unsigned long parent_rate)
{
	struct raspberrypi_clk *rpi = container_of(hw, struct raspberrypi_clk,
						   pllb);
	u32 new_rate = rate / RPI_FIRMWARE_PLLB_ARM_DIV_RATE;
	int ret;

	ret = raspberrypi_clock_property(rpi->firmware,
					 RPI_FIRMWARE_SET_CLOCK_RATE,
					 rpi->pll_info->clock_id,
					 &new_rate);
	if (ret)
		dev_err_ratelimited(rpi->dev, "Failed to change %s frequency: %d",
				    clk_hw_get_name(hw), ret);

	return ret;
}

/*
 * Sadly there is no firmware rate rounding interface. We borrowed it from
 * clk-bcm2835.
 */
static int raspberrypi_pll_determine_rate(struct clk_hw *hw,
					  struct clk_rate_request *req)
{
	struct raspberrypi_clk *rpi = container_of(hw, struct raspberrypi_clk,
						   pllb);
	u64 div, final_rate;
	u32 ndiv, fdiv;

	/* We can't use req->rate directly as it would overflow */
	final_rate = clamp(req->rate, rpi->min_rate, rpi->max_rate);

	div = (u64)final_rate << A2W_PLL_FRAC_BITS;
	do_div(div, req->best_parent_rate);

	ndiv = div >> A2W_PLL_FRAC_BITS;
	fdiv = div & ((1 << A2W_PLL_FRAC_BITS) - 1);

	final_rate = ((u64)req->best_parent_rate *
					((ndiv << A2W_PLL_FRAC_BITS) + fdiv));

	req->rate = final_rate >> A2W_PLL_FRAC_BITS;

	return 0;
}

static const struct clk_ops raspberrypi_firmware_pll_clk_ops = {
	.is_prepared = raspberrypi_fw_pll_is_on,
	.recalc_rate = raspberrypi_fw_pll_get_rate,
	.set_rate = raspberrypi_fw_pll_set_rate,
	.determine_rate = raspberrypi_pll_determine_rate,
};

static int raspberrypi_register_pllb(struct raspberrypi_clk *rpi)
{
	u32 min_rate = 0, max_rate = 0;
	struct clk_init_data init;
	const struct raspberrypi_pll *pll = rpi->pll_info;
	int ret;
	memset(&init, 0, sizeof(init));

	/* All of the PLLs derive from the external oscillator. */
	init.parent_names = (const char *[]){ pll->parent_name };
	init.num_parents = 1;
	init.name = pll->clock_name;
	init.ops = &raspberrypi_firmware_pll_clk_ops;
	init.flags = 0x8; //CLK_GET_RATE_NOCACHE | CLK_IGNORE_UNUSED;

	/* Get min & max rates set by the firmware */
	ret = raspberrypi_clock_property(rpi->firmware,
					 RPI_FIRMWARE_GET_MIN_CLOCK_RATE,
					 rpi->pll_info->clock_id,
					 &min_rate);
	if (ret) {
		dev_err(rpi->dev, "Failed to get %s min freq: %d\n",
			init.name, ret);
		return ret;
	}

	ret = raspberrypi_clock_property(rpi->firmware,
					 RPI_FIRMWARE_GET_MAX_CLOCK_RATE,
					 rpi->pll_info->clock_id,
					 &max_rate);
	if (ret) {
		dev_err(rpi->dev, "Failed to get %s max freq: %d\n",
			init.name, ret);
		return ret;
	}

	if (!min_rate || !max_rate) {
		dev_err(rpi->dev, "Unexpected frequency range: min %u, max %u\n",
			min_rate, max_rate);
		return -EINVAL;
	}

	dev_info(rpi->dev, "CPU frequency range: min %u, max %u\n",
		 min_rate, max_rate);

	rpi->min_rate = min_rate * RPI_FIRMWARE_PLLB_ARM_DIV_RATE;
	rpi->max_rate = max_rate * RPI_FIRMWARE_PLLB_ARM_DIV_RATE;

	rpi->pllb.init = &init;

	return devm_clk_hw_register(rpi->dev, &rpi->pllb);
}

static int raspberrypi_register_pllb_arm(struct raspberrypi_clk *rpi)
{
	const struct raspberrypi_pll *pll = rpi->pll_info;

	rpi->pllb_arm = clk_hw_register_fixed_factor(rpi->dev,
				pll->divider_name, pll->clock_name,
				CLK_SET_RATE_PARENT | CLK_GET_RATE_NOCACHE,
				rpi->pll_info->mult, rpi->pll_info->div);

	if (IS_ERR(rpi->pllb_arm)) {
		dev_err(rpi->dev, "Failed to initialize %s\n", rpi->pll_info->clock_name);
		return PTR_ERR(rpi->pllb_arm);
	}

	rpi->pllb_arm_lookup = clkdev_hw_create(rpi->pllb_arm, NULL, rpi->pll_info->dev_id);
	if (!rpi->pllb_arm_lookup) {
		dev_err(rpi->dev, "Failed to initialize %s lookup\n", rpi->pll_info->clock_name);
		clk_hw_unregister_fixed_factor(rpi->pllb_arm);
		return -ENOMEM;
	}

	return 0;
}

static int raspberrypi_clk_probe(struct platform_device *pdev)
{
	struct device_node *firmware_node;
	struct device *dev = &pdev->dev;
	struct rpi_firmware *firmware;
	struct raspberrypi_clk *rpi;
	int ret;
	int i;

	firmware_node = of_find_compatible_node(NULL, NULL,
					"raspberrypi,bcm2835-firmware");
	if (!firmware_node) {
		dev_err(dev, "Missing firmware node\n");
		return -ENOENT;
	}

	firmware = rpi_firmware_get(firmware_node);
	of_node_put(firmware_node);
	if (!firmware)
		return -EPROBE_DEFER;

	for (i=0; i < ARRAY_SIZE(plls); i++) {
		rpi = devm_kzalloc(dev, sizeof(*rpi), GFP_KERNEL);
		if (!rpi)
			return -ENOMEM;

		rpi->dev = dev;
		rpi->firmware = firmware;
		rpi->pll_info = &plls[i];
		if (i == 0)
			platform_set_drvdata(pdev, rpi);



		ret = raspberrypi_register_pllb(rpi);
		if (ret) {
			dev_err(dev, "Failed to initialize %s, %d (pllb)\n", rpi->pll_info->clock_name, ret);
			//return ret;
		}

		if (rpi->pll_info->mult != rpi->pll_info->div) {
			ret = raspberrypi_register_pllb_arm(rpi);
			if (ret) {
				dev_err(dev, "Failed to initialize %s, %d (pllb_arm)\n", rpi->pll_info->clock_name, ret);
				//return ret;
			}
		}
		if (i == 0)
			rpi->cpufreq = platform_device_register_data(dev, "raspberrypi-cpufreq",
								     -1, NULL, 0);
	}
	return 0;
}

static int raspberrypi_clk_remove(struct platform_device *pdev)
{
	struct raspberrypi_clk *rpi = platform_get_drvdata(pdev);

	platform_device_unregister(rpi->cpufreq);

	return 0;
}

static struct platform_driver raspberrypi_clk_driver = {
	.driver = {
		.name = "raspberrypi-clk",
	},
	.probe          = raspberrypi_clk_probe,
	.remove		= raspberrypi_clk_remove,
};
module_platform_driver(raspberrypi_clk_driver);

MODULE_AUTHOR("Nicolas Saenz Julienne <nsaenzjulienne@suse.de>");
MODULE_DESCRIPTION("Raspberry Pi firmware clock driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:raspberrypi-clk");
