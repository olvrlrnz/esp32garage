/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef UTILS_H
#define UTILS_H

#include <sys/cdefs.h>
#include <inttypes.h>

__BEGIN_DECLS

/**
 * Delays execution for a period of time
 * 
 * @param delay The delay in microseconds
 */
extern void delay_us(uint32_t delay);

__END_DECLS

#endif /* UTILS_H */
