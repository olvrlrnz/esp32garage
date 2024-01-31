/* SPDX-License-Identifier: GPL-2.0-only */

#include "freertos/FreeRTOS.h"
#include "ultrasound.h"
#include "freertos/event_groups.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "utils.h"

/// @brief Ultrasound sensor state
struct ultrasound_sensor {
	/// @brief The config that was passed to ``ultrasound_init``
	struct ultrasound_config config;
	/// @brief ``EventGroup`` that controls sensor start/stop
	EventGroupHandle_t control;
	/// @brief A handle to the task that does the actual measuring
	TaskHandle_t measure_task;
	/// @brief Lock to protect ``values`` from concurrent modifications
	portMUX_TYPE values_lock;
	/// @brief Sensor values (minimum, maximum, current value)
	struct ultrasound_values values;
};

/// Tag for log messages
#define ULTRASOUND_LOG_TAG "ultrasound"
/// Bit that controls start/stop
#define ULTRASOUND_RUN_BIT (1 << 0)
/// The minimum interval in milliseoncs between two measurements
#define ULTRASOUND_INTERVAL_MIN 60
/// The speed of sound in air
#define ULTRASOUND_AIR_SPEED 343

void ultrasound_free(struct ultrasound_sensor *sensor)
{
	if (!sensor) {
		return;
	}

	if (sensor->measure_task) {
		vTaskDelete(sensor->measure_task);
	}

	if (sensor->control) {
		vEventGroupDelete(sensor->control);
	}

	free(sensor);
}

static void do_measure(struct ultrasound_sensor *sensor)
{
	uint32_t start;
	uint32_t end;
	unsigned long duration;
	unsigned long distance;

	// start measurement: set trigger pin high for 10us
	gpio_set_level(sensor->config.trigger_pin, 0);
	delay_us(4);
	gpio_set_level(sensor->config.trigger_pin, 1);
	delay_us(10);
	gpio_set_level(sensor->config.trigger_pin, 0);

	while (gpio_get_level(sensor->config.echo_pin) == 0) {
	}

	start = esp_timer_get_time();
	while (gpio_get_level(sensor->config.echo_pin) == 1) {
	}
	end = esp_timer_get_time();

	// end - start is the duration it took the
	// ultrasonic wave to travel from the sensor
	// to an object and back. So the distance is
	// actually only half that
	duration = end - start;
	duration = duration / 2;

	// distance[um] = time[us] * speed [m/s]
	distance = duration * ULTRASOUND_AIR_SPEED;

	taskENTER_CRITICAL(&sensor->values_lock);
	{
		sensor->values.current = distance;
		if (distance < sensor->values.min) {
			sensor->values.min = distance;
		} else if (distance > sensor->values.max) {
			sensor->values.max = distance;
		}
	}
	taskEXIT_CRITICAL(&sensor->values_lock);
}

static void sensor_loop(void *arg)
{
	unsigned interval;
	struct ultrasound_sensor *sensor = arg;

	interval = sensor->config.interval;
	if (interval < ULTRASOUND_INTERVAL_MIN) {
		interval = ULTRASOUND_INTERVAL_MIN;
		ESP_LOGI(ULTRASOUND_LOG_TAG, "sensor (t:%d, e:%d): interval %u below minimum %u",
			 sensor->config.trigger_pin, sensor->config.echo_pin,
			 sensor->config.interval, ULTRASOUND_INTERVAL_MIN);
	}

	for (;;) {
		(void) xEventGroupWaitBits(sensor->control, ULTRASOUND_RUN_BIT, pdFALSE, pdFALSE,
					   portMAX_DELAY);

		do_measure(sensor);
		vTaskDelay(interval / portTICK_PERIOD_MS);
	}
}

void ultrasound_values(struct ultrasound_sensor *sensor, struct ultrasound_values *values)
{
	if (!sensor || !values) {
		abort();
	}

	taskENTER_CRITICAL(&sensor->values_lock);
	{
		memcpy(values, &sensor->values, sizeof(*values));
	}
	taskEXIT_CRITICAL(&sensor->values_lock);
}

void ultrasound_stop(struct ultrasound_sensor *sensor)
{
	if (!sensor || !sensor->control) {
		return;
	}

	ESP_LOGD(ULTRASOUND_LOG_TAG, "sensor (t:%d, e:%d) stopped", sensor->config.trigger_pin,
		 sensor->config.echo_pin);
	xEventGroupClearBits(sensor->control, ULTRASOUND_RUN_BIT);
}

void ultrasound_start(struct ultrasound_sensor *sensor)
{
	if (!sensor || !sensor->control) {
		return;
	}

	ESP_LOGD(ULTRASOUND_LOG_TAG, "sensor (t:%d, e:%d) started", sensor->config.trigger_pin,
		 sensor->config.echo_pin);
	xEventGroupSetBits(sensor->control, ULTRASOUND_RUN_BIT);
}

struct ultrasound_sensor *ultrasound_create(const struct ultrasound_config *config)
{
	esp_err_t rc;
	gpio_config_t io_config;
	struct ultrasound_sensor *sensor = NULL;

	// setup trigger pin
	io_config.pin_bit_mask = 1ULL << config->trigger_pin;
	io_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
	io_config.pull_up_en = GPIO_PULLUP_DISABLE;
	io_config.mode = GPIO_MODE_OUTPUT;
	io_config.intr_type = GPIO_INTR_DISABLE;

	rc = gpio_config(&io_config);
	if (rc != ESP_OK) {
		goto fail;
	}

	// setup echo pin
	io_config.pin_bit_mask = 1ULL << config->echo_pin;
	io_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
	io_config.pull_up_en = GPIO_PULLUP_DISABLE;
	io_config.mode = GPIO_MODE_INPUT;
	io_config.intr_type = GPIO_INTR_DISABLE;

	rc = gpio_config(&io_config);
	if (rc != ESP_OK) {
		goto fail;
	}

	sensor = calloc(1, sizeof(*sensor));
	if (!sensor) {
		goto fail;
	}

	memcpy(&sensor->config, config, sizeof(sensor->config));

	sensor->control = xEventGroupCreate();
	if (!sensor->control) {
		goto fail;
	}

	xTaskCreate(sensor_loop, "", 1024 * 4, sensor, tskIDLE_PRIORITY, &sensor->measure_task);

	if (!sensor->measure_task) {
		goto fail;
	}

	portMUX_INITIALIZE(&sensor->values_lock);
	sensor->values.max = 0;
	sensor->values.min = ~0UL;

	return sensor;

fail:
	ultrasound_free(sensor);
	return NULL;
}
