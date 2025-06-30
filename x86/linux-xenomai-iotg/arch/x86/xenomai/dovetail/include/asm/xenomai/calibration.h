/*
 * Copyright (C) 2001,2002,2003,2004,2005 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_X86_ASM_CALIBRATION_H
#define _COBALT_X86_ASM_CALIBRATION_H

static inline void xnarch_get_latencies(struct xnclock_gravity *p)
{
	unsigned long sched_latency;

#if CONFIG_XENO_OPT_TIMING_SCHEDLAT != 0
	sched_latency = CONFIG_XENO_OPT_TIMING_SCHEDLAT;
#else /* !CONFIG_XENO_OPT_TIMING_SCHEDLAT */
	sched_latency = num_online_cpus() > 1 ? 3350 : 2000;
#endif /* !CONFIG_XENO_OPT_TIMING_SCHEDLAT */

	p->user = xnclock_ns_to_ticks(&nkclock, sched_latency);
	p->kernel = xnclock_ns_to_ticks(&nkclock,
					CONFIG_XENO_OPT_TIMING_KSCHEDLAT);
	p->irq = xnclock_ns_to_ticks(&nkclock, CONFIG_XENO_OPT_TIMING_IRQLAT);
}

#endif /* !_COBALT_X86_ASM_CALIBRATION_H */
