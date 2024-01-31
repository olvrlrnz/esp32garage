/* SPDX-License-Identifier: GPL-2.0-only */

#include "utils.h"
#include "esp_timer.h"

/// NOP instruction
#define NOP asm volatile("nop")

void delay_us(uint32_t delay)
{
	uint64_t now;
	uint64_t end;

	if (!delay) {
		return;
	}

	now = (uint64_t) esp_timer_get_time();
	end = (now + delay);

	/* check for overflow */
	if (end < now) {
		while ((uint64_t) esp_timer_get_time() > end) {
			NOP;
		}
	}

	while ((uint64_t) esp_timer_get_time() < end) {
		NOP;
	}
}
