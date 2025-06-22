/*
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef _COBALT_KERNEL_IPIPE_CLOCK_H
#define _COBALT_KERNEL_IPIPE_CLOCK_H

#include <linux/ipipe_tickdev.h>
#include <cobalt/uapi/kernel/types.h>

struct timespec64;

static inline u64 pipeline_read_cycle_counter(void)
{
	u64 t;
	ipipe_read_tsc(t);
	return t;
}

xnticks_t pipeline_read_wallclock(void);

int pipeline_set_wallclock(xnticks_t epoch_ns);

static inline void pipeline_set_timer_shot(unsigned long cycles)
{
	ipipe_timer_set(cycles);
}

static inline const char *pipeline_timer_name(void)
{
	return ipipe_timer_name();
}

static inline const char *pipeline_clock_name(void)
{
	return ipipe_clock_name();
}

int pipeline_get_host_time(struct timespec64 *tp);

void pipeline_update_clock_freq(unsigned long long freq);

void pipeline_init_clock(void);

#endif /* !_COBALT_KERNEL_IPIPE_CLOCK_H */
