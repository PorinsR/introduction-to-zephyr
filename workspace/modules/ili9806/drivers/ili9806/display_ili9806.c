/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9806.h"
// #include "display_ili9xxx.h"
#include <zephyr/drivers/display/display_ili9xxx.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_ili9806, CONFIG_DISPLAY_LOG_LEVEL);

int ili9806_regs_init(const struct device *dev)
{
	const struct ili9xxx_config *config = dev->config;
	const struct ili9806_regs *regs = config->regs;

	int r;

	LOG_HEXDUMP_DBG(regs->gamset, ILI9806_GAMSET_LEN, "GAMSET");
	r = ili9xxx_transmit(dev, ILI9806_GAMSET, regs->gamset,
			     ILI9806_GAMSET_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->frmctr1, ILI9806_FRMCTR1_LEN, "FRMCTR1");
	r = ili9xxx_transmit(dev, ILI9806_FRMCTR1, regs->frmctr1,
			     ILI9806_FRMCTR1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->disctrl, ILI9806_DISCTRL_LEN, "DISCTRL");
	r = ili9xxx_transmit(dev, ILI9806_DISCTRL, regs->disctrl,
			     ILI9806_DISCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl1, ILI9806_PWCTRL1_LEN, "PWCTRL1");
	r = ili9xxx_transmit(dev, ILI9806_PWCTRL1, regs->pwctrl1,
			     ILI9806_PWCTRL1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl2, ILI9806_PWCTRL2_LEN, "PWCTRL2");
	r = ili9xxx_transmit(dev, ILI9806_PWCTRL2, regs->pwctrl2,
			     ILI9806_PWCTRL2_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->vmctrl1, ILI9806_VMCTRL1_LEN, "VMCTRL1");
	r = ili9xxx_transmit(dev, ILI9806_VMCTRL1, regs->vmctrl1,
			     ILI9806_VMCTRL1_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->vmctrl2, ILI9806_VMCTRL2_LEN, "VMCTRL2");
	r = ili9xxx_transmit(dev, ILI9806_VMCTRL2, regs->vmctrl2,
			     ILI9806_VMCTRL2_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pgamctrl, ILI9806_PGAMCTRL_LEN, "PGAMCTRL");
	r = ili9xxx_transmit(dev, ILI9806_PGAMCTRL, regs->pgamctrl,
			     ILI9806_PGAMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	LOG_HEXDUMP_DBG(regs->ngamctrl, ILI9806_NGAMCTRL_LEN, "NGAMCTRL");
	r = ili9xxx_transmit(dev, ILI9806_NGAMCTRL, regs->ngamctrl,
			     ILI9806_NGAMCTRL_LEN);
	if (r < 0) {
		return r;
	}

	return 0;
}
