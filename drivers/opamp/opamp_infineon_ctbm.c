/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Infineon Technologies AG,
 * SPDX-FileCopyrightText: or an affiliate of Infineon Technologies AG. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Opamp driver for the Infineon M0S8 CTBm block (PSOC 4).
 *
 * Each instance is one of the two opamps of a CTBm block. The inputs use the
 * dedicated CTBm pins; the gain is either set by external components
 * (standalone mode) or fixed to 1 (follower mode), so it cannot be changed at
 * runtime.
 */

#define DT_DRV_COMPAT infineon_ctbm_opamp

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/opamp.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include <cy_ctb.h>

LOG_MODULE_REGISTER(opamp_ifx_ctbm, CONFIG_OPAMP_LOG_LEVEL);

struct opamp_ifx_ctbm_config {
	CTBM_Type *base;
	const struct pinctrl_dev_config *pcfg;
	cy_en_ctb_opamp_sel_t opamp;
	cy_en_ctb_power_t power;
	cy_en_ctb_output_t output_mode;
	uint32_t switches;
};

static int opamp_ifx_ctbm_set_gain(const struct device *dev, enum opamp_gain gain)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(gain);

	return -ENOTSUP;
}

static DEVICE_API(opamp, opamp_ifx_ctbm_api) = {
	.set_gain = opamp_ifx_ctbm_set_gain,
};

static int opamp_ifx_ctbm_init(const struct device *dev)
{
	const struct opamp_ifx_ctbm_config *cfg = dev->config;
	const cy_stc_ctb_opamp_config_t oa_cfg = {
		.power = cfg->power,
		.outputMode = cfg->output_mode,
		.pump = false,
		.compEdge = CY_CTB_COMP_EDGE_DISABLE,
		.compLevel = CY_CTB_COMP_TRIGGER_OUT_PULSE,
		.switchCtrl = cfg->switches,
	};
	int ret;

	if (cfg->pcfg != NULL) {
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
	}

	if (Cy_CTB_OpampInit(cfg->base, cfg->opamp, &oa_cfg) != CY_CTB_SUCCESS) {
		LOG_ERR("Failed to configure opamp");
		return -EIO;
	}

	/* The block enable is shared by both opamps */
	Cy_CTB_Enable(cfg->base);

	return 0;
}

#define OPAMP_IFX_CTBM_IDX(n) DT_INST_REG_ADDR(n)

#define OPAMP_IFX_CTBM_FOLLOWER(n) DT_INST_ENUM_HAS_VALUE(n, functional_mode, follower)

/* Inputs on the dedicated pins; in follower mode the inverting input is tied to the output */
#define OPAMP_IFX_CTBM_SWITCHES(n)                                                                 \
	(OPAMP_IFX_CTBM_IDX(n) == 0                                                                \
		 ? (CY_CTB_SW_OA0_POS_PIN0 |                                                       \
		    (OPAMP_IFX_CTBM_FOLLOWER(n) ? CY_CTB_SW_OA0_NEG_OUT : CY_CTB_SW_OA0_NEG_PIN1)) \
		 : (CY_CTB_SW_OA1_POS_PIN5 |                                                       \
		    (OPAMP_IFX_CTBM_FOLLOWER(n) ? CY_CTB_SW_OA1_NEG_OUT                            \
						: CY_CTB_SW_OA1_NEG_PIN4)))

#define OPAMP_IFX_CTBM_PINCTRL_DEFINE(n)                                                           \
	IF_ENABLED(DT_INST_PINCTRL_HAS_NAME(n, default), (PINCTRL_DT_INST_DEFINE(n);))

#define OPAMP_IFX_CTBM_PINCTRL_GET(n)                                                              \
	COND_CODE_1(DT_INST_PINCTRL_HAS_NAME(n, default), (PINCTRL_DT_INST_DEV_CONFIG_GET(n)),     \
		    (NULL))

#define OPAMP_IFX_CTBM_DEFINE(n)                                                                   \
	BUILD_ASSERT(OPAMP_IFX_CTBM_IDX(n) <= 1, "CTBm opamp index must be 0 or 1");               \
	BUILD_ASSERT(DT_INST_ENUM_HAS_VALUE(n, functional_mode, standalone) ||                     \
			     OPAMP_IFX_CTBM_FOLLOWER(n),                                           \
		     "CTBm opamp only supports standalone and follower modes");                    \
                                                                                                   \
	OPAMP_IFX_CTBM_PINCTRL_DEFINE(n)                                                           \
                                                                                                   \
	static const struct opamp_ifx_ctbm_config opamp_ifx_ctbm_config_##n = {                    \
		.base = (CTBM_Type *)DT_REG_ADDR(DT_INST_PARENT(n)),                               \
		.pcfg = OPAMP_IFX_CTBM_PINCTRL_GET(n),                                             \
		.opamp = OPAMP_IFX_CTBM_IDX(n) == 0 ? CY_CTB_OPAMP_0 : CY_CTB_OPAMP_1,             \
		.power = (cy_en_ctb_power_t)(CY_CTB_POWER_LOW + DT_INST_ENUM_IDX(n, power_mode)),  \
		.output_mode = DT_INST_PROP(n, out_to_pin) ? CY_CTB_MODE_OPAMP_EXTERNAL            \
							   : CY_CTB_MODE_OPAMP_INTERNAL,           \
		.switches = OPAMP_IFX_CTBM_SWITCHES(n),                                            \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, opamp_ifx_ctbm_init, NULL, NULL, &opamp_ifx_ctbm_config_##n,      \
			      POST_KERNEL, CONFIG_OPAMP_INIT_PRIORITY, &opamp_ifx_ctbm_api);

DT_INST_FOREACH_STATUS_OKAY(OPAMP_IFX_CTBM_DEFINE)
