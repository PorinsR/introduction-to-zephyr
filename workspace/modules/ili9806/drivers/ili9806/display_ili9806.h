/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_
#define ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_

#include <zephyr/device.h>

/* Commands/registers. */
#define ILI9806_GAMSET 0x26
#define ILI9806_FRMCTR1 0xB1
#define ILI9806_DISCTRL 0xB6
#define ILI9806_PWCTRL1 0xC0
#define ILI9806_PWCTRL2 0xC1
#define ILI9806_VMCTRL1 0xC5
#define ILI9806_VMCTRL2 0xC7
#define ILI9806_PGAMCTRL 0xE0
#define ILI9806_NGAMCTRL 0xE1

/* Commands/registers length. */
#define ILI9806_GAMSET_LEN 1U
#define ILI9806_FRMCTR1_LEN 2U
#define ILI9806_DISCTRL_LEN 3U
#define ILI9806_PWCTRL1_LEN 2U
#define ILI9806_PWCTRL2_LEN 1U
#define ILI9806_VMCTRL1_LEN 2U
#define ILI9806_VMCTRL2_LEN 1U
#define ILI9806_PGAMCTRL_LEN 15U
#define ILI9806_NGAMCTRL_LEN 15U

/** X resolution (pixels). */
#define ILI9806_X_RES 480U
/** Y resolution (pixels). */
#define ILI9806_Y_RES 800U

/** ILI9806 registers to be initialized. */
struct ili9806_regs {
	uint8_t gamset[ILI9806_GAMSET_LEN];
	uint8_t frmctr1[ILI9806_FRMCTR1_LEN];
	uint8_t disctrl[ILI9806_DISCTRL_LEN];
	uint8_t pwctrl1[ILI9806_PWCTRL1_LEN];
	uint8_t pwctrl2[ILI9806_PWCTRL2_LEN];
	uint8_t vmctrl1[ILI9806_VMCTRL1_LEN];
	uint8_t vmctrl2[ILI9806_VMCTRL2_LEN];
	uint8_t pgamctrl[ILI9806_PGAMCTRL_LEN];
	uint8_t ngamctrl[ILI9806_NGAMCTRL_LEN];
};

/* Initializer macro for ILI9806 registers. */
#define ILI9806_REGS_INIT(n)                                                   \
	static const struct ili9806_regs ili9xxx_regs_##n = {                  \
		.gamset = DT_PROP(DT_INST(n, ilitek_ili9806), gamset),         \
		.frmctr1 = DT_PROP(DT_INST(n, ilitek_ili9806), frmctr1),       \
		.disctrl = DT_PROP(DT_INST(n, ilitek_ili9806), disctrl),       \
		.pwctrl1 = DT_PROP(DT_INST(n, ilitek_ili9806), pwctrl1),       \
		.pwctrl2 = DT_PROP(DT_INST(n, ilitek_ili9806), pwctrl2),       \
		.vmctrl1 = DT_PROP(DT_INST(n, ilitek_ili9806), vmctrl1),       \
		.vmctrl2 = DT_PROP(DT_INST(n, ilitek_ili9806), vmctrl2),       \
		.pgamctrl = DT_PROP(DT_INST(n, ilitek_ili9806), pgamctrl),     \
		.ngamctrl = DT_PROP(DT_INST(n, ilitek_ili9806), ngamctrl),     \
	}

/**
 * @brief Initialize ILI9806 registers with DT values.
 *
 * @param dev ILI9806 device instance
 * @return 0 on success, errno otherwise.
 */
int ili9806_regs_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_ */
