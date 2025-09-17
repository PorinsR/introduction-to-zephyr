/*
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9806.h"

#include <zephyr/drivers/display.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(display_ili9806, CONFIG_DISPLAY_LOG_LEVEL);

#ifdef CONFIG_ILI9806
static const struct ili9806_quirks ili9806_quirks = {
	.cmd_set = CMD_SET_1,
};
#endif

struct ili9806_data
{
	uint8_t bytes_per_pixel;
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;
};

#ifdef CONFIG_ili9806_READ

/* We set this LUT directly when reads are enabled,
 * so that we can be sure the bitshift to convert GRAM data back
 * to RGB565 will result in correct data
 */
const uint8_t ili9806_rgb_lut[] = {
	0, 2, 4, 6,
	8, 10, 12, 14,
	16, 18, 20, 22,
	24, 26, 28, 30,
	32, 34, 36, 38,
	40, 42, 44, 46,
	48, 50, 52, 54,
	56, 58, 60, 62,
	0, 1, 2, 3,
	4, 5, 6, 7,
	8, 9, 10, 11,
	12, 13, 14, 15,
	16, 17, 18, 19,
	20, 21, 22, 23,
	24, 25, 26, 27,
	28, 29, 30, 31,
	32, 33, 34, 35,
	36, 37, 38, 39,
	40, 41, 42, 43,
	44, 45, 46, 47,
	48, 49, 50, 51,
	52, 53, 54, 55,
	56, 57, 58, 59,
	60, 61, 62, 63,
	0, 2, 4, 6,
	8, 10, 12, 14,
	16, 18, 20, 22,
	24, 26, 28, 30,
	32, 34, 36, 38,
	40, 42, 44, 46,
	48, 50, 52, 54,
	56, 58, 60, 62};

#endif

int ili9806_transmit(const struct device *dev, uint8_t cmd, const void *tx_data,
					 size_t tx_len)
{
	const struct ili9806_config *config = dev->config;
	printk("Here1");
	return mipi_dbi_command_write(config->mipi_dev, &config->dbi_config,
								  cmd, tx_data, tx_len);
}

static int ili9806_exit_sleep(const struct device *dev)
{
	int r;

	r = ili9806_transmit(dev, ili9806_SLPOUT, NULL, 0);
	if (r < 0)
	{
		return r;
	}

	k_sleep(K_MSEC(ili9806_SLEEP_OUT_TIME));

	return 0;
}

static void ili9806_hw_reset(const struct device *dev)
{
	const struct ili9806_config *config = dev->config;

	if (mipi_dbi_reset(config->mipi_dev, ili9806_RESET_PULSE_TIME) < 0)
	{
		return;
	};
	k_sleep(K_MSEC(ili9806_RESET_WAIT_TIME));
}

static int ili9806_set_mem_area(const struct device *dev, const uint16_t x,
								const uint16_t y, const uint16_t w,
								const uint16_t h)
{
	int r;
	uint16_t spi_data[2];

	spi_data[0] = sys_cpu_to_be16(x);
	spi_data[1] = sys_cpu_to_be16(x + w - 1U);
	r = ili9806_transmit(dev, ili9806_CASET, &spi_data[0], 4U);
	if (r < 0)
	{
		return r;
	}

	spi_data[0] = sys_cpu_to_be16(y);
	spi_data[1] = sys_cpu_to_be16(y + h - 1U);
	r = ili9806_transmit(dev, ili9806_PASET, &spi_data[0], 4U);
	if (r < 0)
	{
		return r;
	}

	return 0;
}

static int ili9806_write(const struct device *dev, const uint16_t x,
						 const uint16_t y,
						 const struct display_buffer_descriptor *desc,
						 const void *buf)
{
	const struct ili9806_config *config = dev->config;
	struct ili9806_data *data = dev->data;
	struct display_buffer_descriptor mipi_desc;

	int r;
	const uint8_t *write_data_start = (const uint8_t *)buf;
	uint16_t write_cnt;
	uint16_t nbr_of_writes;
	uint16_t write_h;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller than width");
	__ASSERT((desc->pitch * data->bytes_per_pixel * desc->height) <=
				 desc->buf_size,
			 "Input buffer too small");

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height,
			x, y);
	r = ili9806_set_mem_area(dev, x, y, desc->width, desc->height);
	if (r < 0)
	{
		return r;
	}

	if (desc->pitch > desc->width)
	{
		write_h = 1U;
		nbr_of_writes = desc->height;
		mipi_desc.height = 1;
		mipi_desc.buf_size = desc->pitch * data->bytes_per_pixel;
	}
	else
	{
		write_h = desc->height;
		mipi_desc.height = desc->height;
		mipi_desc.buf_size = desc->width * data->bytes_per_pixel * write_h;
		nbr_of_writes = 1U;
	}

	mipi_desc.width = desc->width;
	/* Per MIPI API, pitch must always match width */
	mipi_desc.pitch = desc->width;
	mipi_desc.frame_incomplete = desc->frame_incomplete;

	r = ili9806_transmit(dev, ili9806_RAMWR, NULL, 0);
	if (r < 0)
	{
		return r;
	}

	for (write_cnt = 0U; write_cnt < nbr_of_writes; ++write_cnt)
	{
		r = mipi_dbi_write_display(config->mipi_dev,
								   &config->dbi_config,
								   write_data_start,
								   &mipi_desc,
								   data->pixel_format);
		if (r < 0)
		{
			return r;
		}

		write_data_start += desc->pitch * data->bytes_per_pixel;
	}

	return 0;
}

#ifdef CONFIG_ili9806_READ

static int ili9806_read(const struct device *dev, const uint16_t x,
						const uint16_t y,
						const struct display_buffer_descriptor *desc, void *buf)
{
	const struct ili9806_config *config = dev->config;
	struct ili9806_data *data = dev->data;
	struct display_buffer_descriptor mipi_desc;
	int r;
	uint32_t gram_data, nbr_of_reads;
	uint16_t *read_data_start = (uint16_t *)buf;

	if (data->pixel_format != PIXEL_FORMAT_RGB_565)
	{
		/* Only RGB565 can be supported, see note below */
		return -ENOTSUP;
	}

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller than width");
	__ASSERT((desc->pitch * data->bytes_per_pixel * desc->height) <=
				 desc->buf_size,
			 "Output buffer too small");

	LOG_DBG("Reading %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height,
			x, y);

	r = ili9806_set_mem_area(dev, x, y, desc->width, desc->height);
	if (r < 0)
	{
		return r;
	}

	/*
	 * ili9806 stores all pixel data in graphics ram (GRAM) as 18 bit
	 * values. When using RGB565 pixel format, pixels are converted to
	 * 18 bit values via a lookup table. When using RGB888 format, the
	 * lower 2 bits of each pixel are simply dropped. When reading pixels,
	 * the response format will always look like so:
	 * | R[5:0] | x | x | G[5:0] | x | x | B[5:0] | x | x |
	 * Where x represents "don't care". The internal format of the
	 * ili9806 graphics RAM results in the following restrictions:
	 * - RGB888 mode can't be supported.
	 * - we can only read one pixel at once (since we need to do
	 *   byte manipulation on the output)
	 */

	/* Setup MIPI descriptor to read 3 bytes (one pixel in GRAM) */
	mipi_desc.width = 1;
	mipi_desc.height = 1;
	/* Per MIPI API, pitch must always match width */
	mipi_desc.pitch = 1;

	nbr_of_reads = desc->width * desc->height;

	/* Initial read command should consist of RAMRD command, plus
	 * 8 dummy clock cycles
	 */
	uint8_t cmd[] = {ili9806_RAMRD, 0xFF};

	for (uint32_t read_cnt = 0; read_cnt < nbr_of_reads; read_cnt++)
	{
		r = mipi_dbi_command_read(config->mipi_dev,
								  &config->dbi_config,
								  cmd, sizeof(cmd),
								  (uint8_t *)&gram_data, 3);
		if (r < 0)
		{
			return r;
		}

		/* Bitshift the graphics RAM data to RGB565.
		 * For more details on the formatting of this data,
		 * see "Read data through 4-line SPI mode" diagram
		 * on page 64 of datasheet.
		 */
		read_data_start[read_cnt] =
			((gram_data & 0xF80000) >> 11) | /* Blue */
			((gram_data & 0x1C00) << 3) |	 /* Green */
			((gram_data & 0xE000) >> 13) |	 /* Green */
			(gram_data & 0xF8);				 /* Red */

		/* After first read, we should use read memory continue command */
		cmd[0] = ili9806_RAMRD_CONT;
	}

	return 0;
}

#endif

static int ili9806_display_blanking_off(const struct device *dev)
{
	LOG_DBG("Turning display blanking off");
	return ili9806_transmit(dev, ili9806_DISPON, NULL, 0);
}

static int ili9806_display_blanking_on(const struct device *dev)
{
	LOG_DBG("Turning display blanking on");
	return ili9806_transmit(dev, ili9806_DISPOFF, NULL, 0);
}

static int
ili9806_set_pixel_format(const struct device *dev,
						 const enum display_pixel_format pixel_format)
{
	struct ili9806_data *data = dev->data;

	int r;
	uint8_t tx_data;
	uint8_t bytes_per_pixel;

	if (pixel_format == PIXEL_FORMAT_RGB_565)
	{
		bytes_per_pixel = 2U;
		tx_data = ili9806_PIXSET_MCU_16_BIT | ili9806_PIXSET_RGB_16_BIT;
	}
	else if (pixel_format == PIXEL_FORMAT_RGB_888)
	{
		bytes_per_pixel = 3U;
		tx_data = ili9806_PIXSET_MCU_18_BIT | ili9806_PIXSET_RGB_18_BIT;
	}
	else
	{
		LOG_ERR("Unsupported pixel format");
		return -ENOTSUP;
	}

	r = ili9806_transmit(dev, ili9806_PIXSET, &tx_data, 1U);
	if (r < 0)
	{
		return r;
	}

	data->pixel_format = pixel_format;
	data->bytes_per_pixel = bytes_per_pixel;

	return 0;
}

static int ili9806_set_orientation(const struct device *dev,
								   const enum display_orientation orientation)
{
	const struct ili9806_config *config = dev->config;
	struct ili9806_data *data = dev->data;

	int r;
	uint8_t tx_data = ili9806_MADCTL_BGR;
	if (config->quirks->cmd_set == CMD_SET_1)
	{
		if (orientation == DISPLAY_ORIENTATION_NORMAL)
		{
			tx_data |= ili9806_MADCTL_MX;
		}
		else if (orientation == DISPLAY_ORIENTATION_ROTATED_90)
		{
			tx_data |= ili9806_MADCTL_MV;
		}
		else if (orientation == DISPLAY_ORIENTATION_ROTATED_180)
		{
			tx_data |= ili9806_MADCTL_MY | ili9806_MADCTL_ML;
		}
		else if (orientation == DISPLAY_ORIENTATION_ROTATED_270)
		{
			tx_data |= ili9806_MADCTL_MV | ili9806_MADCTL_MX |
					   ili9806_MADCTL_MY;
		}
	}
	else if (config->quirks->cmd_set == CMD_SET_2)
	{
		if (orientation == DISPLAY_ORIENTATION_NORMAL)
		{
			/* Do nothing */
		}
		else if (orientation == DISPLAY_ORIENTATION_ROTATED_90)
		{
			tx_data |= ili9806_MADCTL_MV | ili9806_MADCTL_MY;
		}
		else if (orientation == DISPLAY_ORIENTATION_ROTATED_180)
		{
			tx_data |= ili9806_MADCTL_MY | ili9806_MADCTL_MX |
					   ili9806_MADCTL_ML;
		}
		else if (orientation == DISPLAY_ORIENTATION_ROTATED_270)
		{
			tx_data |= ili9806_MADCTL_MV | ili9806_MADCTL_MX;
		}
	}

	r = ili9806_transmit(dev, ili9806_MADCTL, &tx_data, 1U);
	if (r < 0)
	{
		return r;
	}

	data->orientation = orientation;

	return 0;
}

static void ili9806_get_capabilities(const struct device *dev,
									 struct display_capabilities *capabilities)
{
	struct ili9806_data *data = dev->data;
	const struct ili9806_config *config = dev->config;

	memset(capabilities, 0, sizeof(struct display_capabilities));

	capabilities->supported_pixel_formats =
		PIXEL_FORMAT_RGB_565 | PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = data->pixel_format;

	if (data->orientation == DISPLAY_ORIENTATION_NORMAL ||
		data->orientation == DISPLAY_ORIENTATION_ROTATED_180)
	{
		capabilities->x_resolution = config->x_resolution;
		capabilities->y_resolution = config->y_resolution;
	}
	else
	{
		capabilities->x_resolution = config->y_resolution;
		capabilities->y_resolution = config->x_resolution;
	}

	capabilities->current_orientation = data->orientation;
}

static int ili9806_configure(const struct device *dev)
{
	const struct ili9806_config *config = dev->config;

	int r;
	enum display_pixel_format pixel_format;
	enum display_orientation orientation;

	/* pixel format */
	if (config->pixel_format == ILI9806_PIXEL_FORMAT_RGB565)
	{
		pixel_format = PIXEL_FORMAT_RGB_565;
	}
	else
	{
		pixel_format = PIXEL_FORMAT_RGB_888;
	}

	r = ili9806_set_pixel_format(dev, pixel_format);
	if (r < 0)
	{
		return r;
	}

	/* orientation */
	if (config->rotation == 0U)
	{
		orientation = DISPLAY_ORIENTATION_NORMAL;
	}
	else if (config->rotation == 90U)
	{
		orientation = DISPLAY_ORIENTATION_ROTATED_90;
	}
	else if (config->rotation == 180U)
	{
		orientation = DISPLAY_ORIENTATION_ROTATED_180;
	}
	else
	{
		orientation = DISPLAY_ORIENTATION_ROTATED_270;
	}

	r = ili9806_set_orientation(dev, orientation);
	if (r < 0)
	{
		return r;
	}

	if (config->inversion)
	{
		r = ili9806_transmit(dev, ili9806_DINVON, NULL, 0U);
		if (r < 0)
		{
			return r;
		}
	}

	if (config->te_mode != MIPI_DBI_TE_NO_EDGE)
	{
		/* Attempt to enable TE signal */
		r = mipi_dbi_configure_te(config->mipi_dev, config->te_mode, 0);
		if (r == 0)
		{
			/* TE was enabled, send TEON, and enable vblank only */
			const uint8_t tx_data = 0x0; /* Set M bit to 0 */

			r = ili9806_transmit(dev, ili9806_TEON, &tx_data, 1U);
			if (r < 0)
			{
				return r;
			}
		}
	}

	r = config->regs_init_fn(dev);
	if (r < 0)
	{
		return r;
	}

	return 0;
}

static int ili9806_init(const struct device *dev)
{
	const struct ili9806_config *config = dev->config;

	int r;

	if (!device_is_ready(config->mipi_dev))
	{
		LOG_ERR("MIPI DBI device is not ready");
		return -ENODEV;
	}

	ili9806_hw_reset(dev);
	printk("Here0");
	r = ili9806_transmit(dev, ili9806_SWRESET, NULL, 0);
	if (r < 0)
	{
		LOG_ERR("Error transmit command Software Reset (%d)", r);
		return r;
	}

#ifdef CONFIG_ili9806_READ
	/* Set RGB LUT table to enable display read API */
	ili9806_transmit(dev, ili9806_RGBSET, ili9806_rgb_lut, sizeof(ili9806_rgb_lut));
#endif

	k_sleep(K_MSEC(ili9806_RESET_WAIT_TIME));

	ili9806_display_blanking_on(dev);

	r = ili9806_configure(dev);
	if (r < 0)
	{
		LOG_ERR("Could not configure display (%d)", r);
		return r;
	}

	r = ili9806_exit_sleep(dev);
	if (r < 0)
	{
		LOG_ERR("Could not exit sleep mode (%d)", r);
		return r;
	}

	return 0;
}

static DEVICE_API(display, ili9806_api) = {
	.blanking_on = ili9806_display_blanking_on,
	.blanking_off = ili9806_display_blanking_off,
	.write = ili9806_write,
#ifdef CONFIG_ili9806_READ
	.read = ili9806_read,
#endif
	.get_capabilities = ili9806_get_capabilities,
	.set_pixel_format = ili9806_set_pixel_format,
	.set_orientation = ili9806_set_orientation,
};

#ifdef CONFIG_ILI9340
static const struct ili9806_quirks ili9340_quirks = {
	.cmd_set = CMD_SET_1,
};
#endif

#ifdef CONFIG_ILI9341
static const struct ili9806_quirks ili9341_quirks = {
	.cmd_set = CMD_SET_1,
};
#endif

#ifdef CONFIG_ILI9342C
static const struct ili9806_quirks ili9342c_quirks = {
	.cmd_set = CMD_SET_2,
};
#endif

#ifdef CONFIG_ILI9488
static const struct ili9806_quirks ili9488_quirks = {
	.cmd_set = CMD_SET_1,
};
#endif

#define INST_DT_ili9806(n, t) DT_INST(n, ilitek_ili##t)

#define ili9806_INIT(n, t)                                              \
	ILI##t##_REGS_INIT(n);                                              \
                                                                        \
	static const struct ili9806_config ili9##t##_config_##n = {         \
		.quirks = &ili##t##_quirks,                                     \
		.mipi_dev = DEVICE_DT_GET(DT_PARENT(INST_DT_ili9806(n, t))),    \
		.dbi_config = {                                                 \
			.mode = DT_STRING_UPPER_TOKEN_OR(                           \
				INST_DT_ili9806(n, t),                                  \
				mipi_mode, MIPI_DBI_MODE_SPI_3WIRE),                    \
			.config = MIPI_DBI_SPI_CONFIG_DT(                           \
				INST_DT_ili9806(n, t),                                  \
				SPI_OP_MODE_MASTER |                                    \
					SPI_WORD_SET(9),                                    \
				0),                                                     \
		},                                                              \
		.pixel_format = DT_PROP(INST_DT_ili9806(n, t), pixel_format),   \
		.rotation = DT_PROP(INST_DT_ili9806(n, t), rotation),           \
		.x_resolution = ILI##t##_X_RES,                                 \
		.y_resolution = ILI##t##_Y_RES,                                 \
		.inversion = DT_PROP(INST_DT_ili9806(n, t), display_inversion), \
		.te_mode = MIPI_DBI_TE_MODE_DT(INST_DT_ili9806(n, t), te_mode), \
		.regs = &ili##t##_regs_##n,                                     \
		.regs_init_fn = ili##t##_regs_init,                             \
	};                                                                  \
                                                                        \
	static struct ili9806_data ili9##t##_data_##n;                      \
                                                                        \
	DEVICE_DT_DEFINE(INST_DT_ili9806(n, t), ili9806_init,               \
					 NULL, &ili9##t##_data_##n,                         \
					 &ili9##t##_config_##n, POST_KERNEL,                \
					 CONFIG_DISPLAY_INIT_PRIORITY, &ili9806_api)

#define DT_INST_FOREACH_ili9806_STATUS_OKAY(t) \
	LISTIFY(DT_NUM_INST_STATUS_OKAY(ilitek_ili##t), ili9806_INIT, (;), t)

int ili9806_regs_init(const struct device *dev)
{
	const struct ili9806_config *config = dev->config;
	const struct ili9806_regs *regs = config->regs;

	int r;

	LOG_HEXDUMP_DBG(regs->gamset, ILI9806_GAMSET_LEN, "GAMSET");
	r = ili9806_transmit(dev, ILI9806_GAMSET, regs->gamset,
						 ILI9806_GAMSET_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->frmctr1, ILI9806_FRMCTR1_LEN, "FRMCTR1");
	r = ili9806_transmit(dev, ILI9806_FRMCTR1, regs->frmctr1,
						 ILI9806_FRMCTR1_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->disctrl, ILI9806_DISCTRL_LEN, "DISCTRL");
	r = ili9806_transmit(dev, ILI9806_DISCTRL, regs->disctrl,
						 ILI9806_DISCTRL_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl1, ILI9806_PWCTRL1_LEN, "PWCTRL1");
	r = ili9806_transmit(dev, ILI9806_PWCTRL1, regs->pwctrl1,
						 ILI9806_PWCTRL1_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pwctrl2, ILI9806_PWCTRL2_LEN, "PWCTRL2");
	r = ili9806_transmit(dev, ILI9806_PWCTRL2, regs->pwctrl2,
						 ILI9806_PWCTRL2_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->vmctrl1, ILI9806_VMCTRL1_LEN, "VMCTRL1");
	r = ili9806_transmit(dev, ILI9806_VMCTRL1, regs->vmctrl1,
						 ILI9806_VMCTRL1_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->vmctrl2, ILI9806_VMCTRL2_LEN, "VMCTRL2");
	r = ili9806_transmit(dev, ILI9806_VMCTRL2, regs->vmctrl2,
						 ILI9806_VMCTRL2_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->pgamctrl, ILI9806_PGAMCTRL_LEN, "PGAMCTRL");
	r = ili9806_transmit(dev, ILI9806_PGAMCTRL, regs->pgamctrl,
						 ILI9806_PGAMCTRL_LEN);
	if (r < 0)
	{
		return r;
	}

	LOG_HEXDUMP_DBG(regs->ngamctrl, ILI9806_NGAMCTRL_LEN, "NGAMCTRL");
	r = ili9806_transmit(dev, ILI9806_NGAMCTRL, regs->ngamctrl,
						 ILI9806_NGAMCTRL_LEN);
	if (r < 0)
	{
		return r;
	}

	return 0;
}

DT_INST_FOREACH_ili9806_STATUS_OKAY(9806);
