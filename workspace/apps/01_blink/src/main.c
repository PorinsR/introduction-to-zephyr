#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// Settings
static const int32_t sleep_time_ms = 100;
static const struct gpio_dt_spec indicator_led = GPIO_DT_SPEC_GET(DT_ALIAS(indicator_led), gpios);
static const struct gpio_dt_spec r_led = GPIO_DT_SPEC_GET(DT_ALIAS(r_led), gpios);
static const struct gpio_dt_spec g_led = GPIO_DT_SPEC_GET(DT_ALIAS(g_led), gpios);
static const struct gpio_dt_spec b_led = GPIO_DT_SPEC_GET(DT_ALIAS(b_led), gpios);

int main(void)
{
	int ret;
	int state = 0;
	int counter = 0;

	// Make sure that the GPIO was initialized
	if (!gpio_is_ready_dt(&indicator_led)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&r_led)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&g_led)) {
		return 0;
	}
	if (!gpio_is_ready_dt(&b_led)) {
		return 0;
	}

	// Set the GPIO as output
	ret = gpio_pin_configure_dt(&indicator_led, GPIO_OUTPUT);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&r_led, GPIO_OUTPUT);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&g_led, GPIO_OUTPUT);
	if (ret < 0) {
		return 0;
	}
	ret = gpio_pin_configure_dt(&b_led, GPIO_OUTPUT);
	if (ret < 0) {
		return 0;
	}

	// Do forever
	while (1) {

		// Change the state of the pin and print
		state = !state;
		printk("LED state: %d\r\n", state);
		
		// Set pin state
		ret = gpio_pin_set_dt(&indicator_led, state);
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_set_dt(&r_led, state);
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_set_dt(&g_led, state);
		if (ret < 0) {
			return 0;
		}
		ret = gpio_pin_set_dt(&b_led, state);
		if (ret < 0) {
			return 0;
		}

		// Sleep
		k_msleep(sleep_time_ms);
	}

	return 0;
}
