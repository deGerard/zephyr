/*
 * Copyright (c) 2023 Endor AG
 * Copyright (c) 2024 Gerard Koskamp
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT issi_is31fl3242

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>

#define IS31FL3242_REG_CONFIG			0x00
#define IS31FL3242_REG_CONFIG_SSD		(1 << 0)
#define IS31FL3242_REG_CONFIG_PHC		(1 << 1)
#define IS31FL3242_REG_CONFIG_PSM		(1 << 2)
#define IS31FL3242_REG_CONFIG_PWMM		(1 << 6)
#define IS31FL3242_REG_CONFIG_D4D5  	0x30
#define IS31FL3242_REG_GCC				0x01
#define IS31FL3242_REG_SL_FIRST			0x02
#define IS31FL3242_REG_SL_LAST			0x0D
#define IS31FL3242_REG_PWM_FIRST		0x0E
#define IS31FL3242_REG_PWM_LAST			0x25
#define IS31FL3242_REG_PWM_FPS			0x61
#define IS31FL3242_REG_PWM_FPS_OEN		(1 << 0)
#define IS31FL3242_REG_PWM_FPS_SEN		(1 << 1)
#define IS31FL3242_REG_PWM_FPS_MDT_SHIFT (2)
#define IS31FL3242_REG_PWM_FPS_PFS_SHIFT (4)
#define IS31FL3242_REG_PWM_FPS_LCAI		(1 << 7)
#define IS31FL3242_REG_OPEN_SHORT_FIRST	0x62
#define IS31FL3242_REG_OPEN_SHORT_LAST	0x6D
#define IS31FL3242_REG_SPREAD_SPECTRUM	0x6E
#define IS31FL3242_REG_RESET			0x7F

#define IS31FL3242_MAX_LEDS				12
#define IS31FL3242_MAX_GLOBAL_CURRENT	48

LOG_MODULE_REGISTER(is31fl3242, CONFIG_LED_LOG_LEVEL);

struct is31fl3242_cfg {
	struct i2c_dt_spec i2c;
	uint8_t global_current;
	const uint8_t *scaling_control;
};

static int is31fl3242_write_buffer(const struct i2c_dt_spec *i2c,
				    const uint8_t *buffer, uint32_t num_bytes)
{
	int status;

	status = i2c_write_dt(i2c, buffer, num_bytes);
	if (status < 0) {
		LOG_ERR("Could not write buffer: %i", status);
		return status;
	}

	return 0;
}

static int is31fl3242_write_reg(const struct i2c_dt_spec *i2c, uint8_t reg,
				 uint8_t val)
{
	uint8_t buffer[2] = {reg, val};

	return is31fl3242_write_buffer(i2c, buffer, sizeof(buffer));
}

static uint8_t is31fl3242_brightness_to_pwm(uint8_t brightness)
{
	return (0xFFU * brightness) / 100;
}

static int is31fl3242_led_write_channels(const struct device *dev,
					  uint32_t start_channel,
					  uint32_t num_channels,
					  const uint8_t *buf)
{
	const struct is31fl3242_cfg *config = dev->config;
	uint8_t i2c_buffer[(IS31FL3242_MAX_LEDS * 2) + 1];
	int status;
	int i;

	if (num_channels == 0) {
		return 0;
	}

	if (num_channels > IS31FL3242_MAX_LEDS) {
		return -EINVAL;
	}

	/* First register pair is channel 0 */
	i2c_buffer[0] = IS31FL3242_REG_PWM_FIRST + (start_channel * 2);
	for (i = 0; i < num_channels; i++) {
		if (buf[i] > 100) {
			return -EINVAL;
		}
		/* Fill PWM_L with the brightness value */
		i2c_buffer[(i * 2) + 1] = is31fl3242_brightness_to_pwm(buf[i]);
		/* Fill PWM_H with 0 because we not use it */
		i2c_buffer[(i * 2) + 2] = 0x00;
	}

	status = is31fl3242_write_buffer(&config->i2c, i2c_buffer,
					  (num_channels  * 2) + 1);
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3242_led_set_brightness(const struct device *dev,
					  uint32_t led, uint8_t value)
{
	const struct is31fl3242_cfg *config = dev->config;
	uint8_t pwm_reg = IS31FL3242_REG_PWM_FIRST + (led * 2);
	int status;
	uint8_t pwm_value;

	if (led > IS31FL3242_MAX_LEDS - 1 || value > 100) {
		return -EINVAL;
	}

	pwm_value = is31fl3242_brightness_to_pwm(value);
	status = is31fl3242_write_reg(&config->i2c, pwm_reg, pwm_value);
	if (status < 0) {
		return status;
	}

	return 0;
}

static int is31fl3242_led_on(const struct device *dev, uint32_t led)
{
	return is31fl3242_led_set_brightness(dev, led, 100);
}

static int is31fl3242_led_off(const struct device *dev, uint32_t led)
{
	return is31fl3242_led_set_brightness(dev, led, 0);
}

static int is31fl3242_init_registers(const struct is31fl3242_cfg *config)
{
	int i;
	int status;
	const struct i2c_dt_spec *i2c = &config->i2c;

	/* Set 8 bit mode, Phase choice to mode 1, no shutdown, bit 4 and 5 must be always set */
	status = is31fl3242_write_reg(i2c, IS31FL3242_REG_CONFIG,
		IS31FL3242_REG_CONFIG_D4D5 | IS31FL3242_REG_CONFIG_PHC | IS31FL3242_REG_CONFIG_SSD);
	if (status < 0) {
		return status;
	}
	/* Global peak current is set to 24 mA */
	status = is31fl3242_write_reg(i2c, IS31FL3242_REG_GCC, config->global_current);
	if (status < 0) {
		return status;
	}
	/* Set PWM frequency to 4KHz */
	status = is31fl3242_write_reg(i2c, IS31FL3242_REG_PWM_FPS,
		(4 << IS31FL3242_REG_PWM_FPS_PFS_SHIFT));
	if (status < 0) {
		return status;
	}
	/* Write 0 to all PWM registers  */
	for (i = IS31FL3242_REG_PWM_FIRST; i <= IS31FL3242_REG_PWM_LAST; i++) {
		status = is31fl3242_write_reg(i2c, i, 0x00);
		if (status < 0) {
			return status;
		}
	}
	/* Write the scaling_control values to all SL registers, configure all outputs to max configured peak current */
	for (i = 0; i < IS31FL3242_MAX_LEDS; i++) {
		status = is31fl3242_write_reg(i2c, i + IS31FL3242_REG_SL_FIRST, config->scaling_control[i]);
		if (status < 0) {
			return status;
		}
	}
	return 0;
}

static int is31fl3242_init(const struct device *dev)
{
	const struct is31fl3242_cfg *config = dev->config;

	LOG_DBG("Initializing @0x%x...", config->i2c.addr);

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	return is31fl3242_init_registers(config);
}

static const struct led_driver_api is31fl3242_led_api = {
	.set_brightness = is31fl3242_led_set_brightness,
	.on = is31fl3242_led_on,
	.off = is31fl3242_led_off,
	.write_channels = is31fl3242_led_write_channels
};

#define IS31FL3242_INIT(id) \
	BUILD_ASSERT(DT_INST_PROP(id, global_output_current) <= 255,\
		"global-output-current must be between 0 and 255");	\
	BUILD_ASSERT(DT_INST_PROP_LEN(id, scaling_control) == IS31FL3242_MAX_LEDS, \
		"array length of scaling_control must be 12"); \
	static const uint8_t scaling_control_##id[IS31FL3242_MAX_LEDS] = {		\
		DT_INST_FOREACH_PROP_ELEM_SEP(id, scaling_control, DT_PROP_BY_IDX, (,))	\
	}; \
	static const struct is31fl3242_cfg is31fl3242_##id##_cfg = {	\
		.i2c = I2C_DT_SPEC_INST_GET(id),			\
		.global_current = DT_INST_PROP(id, global_output_current), \
		.scaling_control = scaling_control_##id \
	};								\
	DEVICE_DT_INST_DEFINE(id, &is31fl3242_init, NULL, NULL,	\
		&is31fl3242_##id##_cfg, POST_KERNEL,			\
		CONFIG_LED_INIT_PRIORITY, &is31fl3242_led_api);

DT_INST_FOREACH_STATUS_OKAY(IS31FL3242_INIT)
