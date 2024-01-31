/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <sys/cdefs.h>
#include "esp_err.h"
#include "driver/gpio.h"

__BEGIN_DECLS

/// Ultrasound sensor configuration
struct ultrasound_config {
	/// Number of the pin the sensor's trigger input is connected to
	gpio_num_t trigger_pin;
	/// Number of the pin the sensor's echo output is connected to
	gpio_num_t echo_pin;
	/// Interval between two measurements in milliseconds
	/// @note The minimum value for ``interval`` is 60 milliseconds
	unsigned interval;
};

/// The sensor's values
struct ultrasound_values {
	/// The smallest distance measured since boot
	unsigned long min;
	/// The largest distance measured since boot
	unsigned long max;
	/// Value of the latest measurement
	unsigned long current;
};

struct ultrasound_sensor;

/**
 * Initializes a new ``ultrasound_sensor`` object
 *
 * The returned sensor is inactive and must be activated using
 * `ultrasound_start`. After that, values can be copied using
 * `ultrasound_values`.
 *
 * @param config Sensor config
 *
 * @return An initialized sensor on success, `NULL` otherwise
 */
extern struct ultrasound_sensor *ultrasound_create(const struct ultrasound_config *config)
	__attribute__((__warn_unused_result__));

/**
 * Frees memory allocated by an `ultrasound_sensor`
 * 
 * @param sensor The sensor to free
 */
extern void ultrasound_free(struct ultrasound_sensor *sensor);

/**
 * Starts an ``ultrasound_sensor``
 *
 * @param sensor The sensor to start
 *
 * @note This function does nothing if `sensor` is `NULL` or already started
 */
extern void ultrasound_start(struct ultrasound_sensor *sensor);

/**
 * Stops an ``ultrasound_sensor``
 *
 * @param sensor The sensor to stop
 *
 * @note This function does nothing if `sensor` is `NULL` or already stopped
 */
extern void ultrasound_stop(struct ultrasound_sensor *sensor);

/**
 * Copies the sensor's current values
 *
 * @param sensor The sensor whose values should be copied
 * @param values Storage space where the current values are placed
 */
extern void ultrasound_values(struct ultrasound_sensor *sensor, struct ultrasound_values *values);

__END_DECLS

#endif /* ULTRASOUND_H */
