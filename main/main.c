#include "freertos/FreeRTOS.h"
#include "ultrasound.h"
#include "esp_log.h"

static int calculate_open_percent(const struct ultrasound_values *values)
{
	unsigned long total;
	unsigned long diff;

	if (!values->min || !values->max) {
		// never ran
		return -1;
	}

	if (values->current < values->min || values->current > values->max) {
		// invalid values
		return -1;
	}

	total = values->max - values->min;
	diff = values->current - values->min;

	return diff * 100 / total;
}

void app_main(void)
{
	struct ultrasound_sensor *sensor;

	struct ultrasound_config config = {
		.echo_pin = GPIO_NUM_37,
		.trigger_pin = GPIO_NUM_38,
	};

	sensor = ultrasound_create(&config);
	if (!sensor) {
		abort();
	}

	ultrasound_start(sensor);

	for (;;) {
		struct ultrasound_values values;

		ultrasound_values(sensor, &values);
		ESP_LOGI("main", "sensor values: cur: %lu -- min: %lu -- max: %lu -- pct: %i",
			 values.current, values.min, values.max, calculate_open_percent(&values));

		vTaskDelay(250 / portTICK_PERIOD_MS);
	}
}
