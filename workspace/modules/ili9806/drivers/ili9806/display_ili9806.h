/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_
#define ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_

#include <zephyr/device.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/sys/util.h>

#define ILI9806_PIXEL_FORMAT_RGB565 0U
#define ILI9806_PIXEL_FORMAT_RGB888 1U

/* Commands/registers. */
#define ili9806_SWRESET 0x01
#define ili9806_SLPOUT 0x11
#define ili9806_DINVON 0x21
#define ili9806_GAMSET 0x26
#define ili9806_DISPOFF 0x28
#define ili9806_DISPON 0x29
#define ili9806_CASET 0x2a
#define ili9806_PASET 0x2b
#define ili9806_RAMWR 0x2c
#define ili9806_RGBSET 0x2d
#define ili9806_RAMRD 0x2e
#define ili9806_TEON 0x35
#define ili9806_MADCTL 0x36
#define ili9806_PIXSET 0x3A
#define ili9806_RAMRD_CONT 0x3e

/* MADCTL register fields. */
#define ili9806_MADCTL_MY BIT(7U)
#define ili9806_MADCTL_MX BIT(6U)
#define ili9806_MADCTL_MV BIT(5U)
#define ili9806_MADCTL_ML BIT(4U)
#define ili9806_MADCTL_BGR BIT(3U)
#define ili9806_MADCTL_MH BIT(2U)

/* PIXSET register fields. */
#define ili9806_PIXSET_RGB_18_BIT 0x60
#define ili9806_PIXSET_RGB_16_BIT 0x50
#define ili9806_PIXSET_MCU_18_BIT 0x06
#define ili9806_PIXSET_MCU_16_BIT 0x05

/** Command/data GPIO level for commands. */
#define ili9806_CMD 1U
/** Command/data GPIO level for data. */
#define ili9806_DATA 0U

/** Sleep out time (ms), ref. 8.2.12 of ili9806 manual. */
#define ili9806_SLEEP_OUT_TIME 120

/** Reset pulse time (ms), ref 15.4 of ili9806 manual. */
#define ili9806_RESET_PULSE_TIME 1

/** Reset wait time (ms), ref 15.4 of ili9806 manual. */
#define ili9806_RESET_WAIT_TIME 5

enum madctl_cmd_set
{
	CMD_SET_1, /* Default for most of ili9806 display controllers */
	CMD_SET_2, /* Used by ILI9342c */
};

struct ili9806_quirks
{
	enum madctl_cmd_set cmd_set;
};

struct ili9806_config
{
	const struct ili9806_quirks *quirks;
	const struct device *mipi_dev;
	struct mipi_dbi_config dbi_config;
	uint8_t pixel_format;
	uint16_t rotation;
	uint16_t x_resolution;
	uint16_t y_resolution;
	bool inversion;
	uint8_t te_mode;
	const void *regs;
	int (*regs_init_fn)(const struct device *dev);
};

int ili9806_transmit(const struct device *dev, uint8_t cmd,
					 const void *tx_data, size_t tx_len);

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
#define ILI9806_X_RES 240U
/** Y resolution (pixels). */
#define ILI9806_Y_RES 320U

/** ILI9806 registers to be initialized. */
struct ili9806_regs
{
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
#define ILI9806_REGS_INIT(n)                                       \
	static const struct ili9806_regs ili9806_regs_##n = {          \
		.gamset = DT_PROP(DT_INST(n, ilitek_ili9806), gamset),     \
		.frmctr1 = DT_PROP(DT_INST(n, ilitek_ili9806), frmctr1),   \
		.disctrl = DT_PROP(DT_INST(n, ilitek_ili9806), disctrl),   \
		.pwctrl1 = DT_PROP(DT_INST(n, ilitek_ili9806), pwctrl1),   \
		.pwctrl2 = DT_PROP(DT_INST(n, ilitek_ili9806), pwctrl2),   \
		.vmctrl1 = DT_PROP(DT_INST(n, ilitek_ili9806), vmctrl1),   \
		.vmctrl2 = DT_PROP(DT_INST(n, ilitek_ili9806), vmctrl2),   \
		.pgamctrl = DT_PROP(DT_INST(n, ilitek_ili9806), pgamctrl), \
		.ngamctrl = DT_PROP(DT_INST(n, ilitek_ili9806), ngamctrl), \
	}

/**
 * @brief Initialize ILI9806 registers with DT values.
 *
 * @param dev ILI9806 device instance
 * @return 0 on success, errno otherwise.
 */
int ili9806_regs_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_DISPLAY_DISPLAY_ILI9806_H_ */
