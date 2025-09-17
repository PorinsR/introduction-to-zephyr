#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/drivers/display.h>
#include <lvgl.h>

// Settings
static const int32_t sleep_time_ms = 50; // Target 20 FPS

int main(void)
{

	uint32_t count = 0;
	char buf[11] = {0};
	lv_obj_t *hello_label;
	lv_obj_t *counter_label;
	lv_obj_t *rect;
	lv_obj_t *circle;
	lv_style_t rect_style;
	lv_style_t circle_style;
	lv_point_t rect_points[5] = {{0, 0}, {120, 0}, {120, 20}, {0, 20}, {0, 0}};
	const uint32_t circle_radius = 15;

	int ret;
	bool led_state = true;

	const struct device *const fuel_gauge = DEVICE_DT_GET_ONE(maxim_max17262);
	const struct device *const encoder = DEVICE_DT_GET_ONE(ams_as5600);
	static const struct gpio_dt_spec display_backlight = GPIO_DT_SPEC_GET(DT_ALIAS(display_backlight), gpios);
	static const struct gpio_dt_spec load_switch = GPIO_DT_SPEC_GET(DT_ALIAS(enable12v), gpios);
	const struct device *display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

	if (!gpio_is_ready_dt(&load_switch))
	{
		return 0;
	}

	ret = gpio_pin_configure_dt(&load_switch, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}
	gpio_pin_set_dt(&load_switch, 1);

	if (!gpio_is_ready_dt(&display_backlight))
	{
		return 0;
	}

	ret = gpio_pin_configure_dt(&display_backlight, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		return 0;
	}
	gpio_pin_set_dt(&display_backlight, 1);

	if (!device_is_ready(fuel_gauge))
	{
		printk("fuel gauge: device not ready.\n");
		return 0;
	}
	if (!device_is_ready(encoder))
	{
		printk("encoder: device not ready.\n");
		return 0;
	}

	if (!device_is_ready(display))
	{
		printk("Error: display not ready\r\n");
		return 0;
	}

	// Create a static label widget
	hello_label = lv_label_create(lv_scr_act());
	lv_label_set_text(hello_label, "Hello, World!");
	lv_obj_align(hello_label, LV_ALIGN_TOP_MID, 0, 5);

	// Create a dynamic label widget
	counter_label = lv_label_create(lv_scr_act());
	lv_obj_align(counter_label, LV_ALIGN_BOTTOM_MID, 0, 0);

	// Set line style
	lv_style_init(&rect_style);
	lv_style_set_line_color(&rect_style, lv_color_hex(0x0000FF));
	lv_style_set_line_width(&rect_style, 3);

	// Create a rectangle out of lines
	rect = lv_line_create(lv_scr_act());
	lv_obj_add_style(rect, &rect_style, 0);
	lv_line_set_points(rect,
					   rect_points,
					   sizeof(rect_points) / sizeof(rect_points[0]));
	lv_obj_align(rect, LV_ALIGN_TOP_MID, 0, 0);

	// Set circle style
	lv_style_init(&circle_style);
	lv_style_set_radius(&circle_style, circle_radius);
	lv_style_set_bg_opa(&circle_style, LV_OPA_100);
	lv_style_set_bg_color(&circle_style, lv_color_hex(0xFF0000));

	// Create an object with the new style
	circle = lv_obj_create(lv_scr_act());
	lv_obj_set_size(circle, circle_radius * 2, circle_radius * 2);
	lv_obj_add_style(circle, &circle_style, 0);
	lv_obj_align(circle, LV_ALIGN_CENTER, 0, 5);

	// Disable display blanking
	display_blanking_off(display);

	while (1)
	{
		struct sensor_value voltage, avg_current, temperature;
		float i_avg;

		sensor_sample_fetch(fuel_gauge);
		sensor_channel_get(fuel_gauge, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
		sensor_channel_get(fuel_gauge, SENSOR_CHAN_GAUGE_AVG_CURRENT,
						   &avg_current);
		sensor_channel_get(fuel_gauge, SENSOR_CHAN_GAUGE_TEMP, &temperature);

		i_avg = avg_current.val1 + (avg_current.val2 / 1000000.0);

		printk("V: %d.%06d V; I: %f mA; T: %d.%06d °C\n",
			   voltage.val1, voltage.val2, (double)i_avg,
			   temperature.val1, temperature.val2);

		ret = gpio_pin_toggle_dt(&display_backlight);
		if (ret < 0)
		{
			return 0;
		}

		led_state = !led_state;
		k_sleep(K_MSEC(sleep_time_ms));
	}
	return 0;
}
