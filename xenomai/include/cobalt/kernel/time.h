/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _COBALT_KERNEL_TIME_H
#define _COBALT_KERNEL_TIME_H

#include <linux/time.h>
#include <linux/time64.h>

/**
 * Read struct __kernel_timespec from userspace and convert to
 * struct timespec64
 *
 * @param ts The destination, will be filled
 * @param uts The source, provided by an application
 * @return 0 on success, -EFAULT otherwise
 */
int cobalt_get_timespec64(struct timespec64 *ts,
			  const struct __kernel_timespec __user *uts);

/**
 * Covert struct timespec64 to struct __kernel_timespec
 * and copy to userspace
 *
 * @param ts The source, provided by kernel
 * @param uts The destination, will be filled
 * @return 0 on success, -EFAULT otherwise
 */
int cobalt_put_timespec64(const struct timespec64 *ts,
			   struct __kernel_timespec __user *uts);

#endif //_COBALT_KERNEL_TIME_H
