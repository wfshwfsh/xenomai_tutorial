/***************************************************************************
 *   Copyright (C) 2007 by trem (Philippe Reynes)                          *
 *   tremyfr@yahoo.fr                                                      *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/**
 * This kernel driver demonstrates how an RTDM device can be set up.
 *
 * It is a simple device, only 4 operation are provided:
 *  - open:  start device usage
 *  - close: ends device usage
 *  - write: store transfered data in an internal buffer
 *  - read:  return previously stored data and erase buffer
 *  - ioctl：0x01 return max data size
 *           0x02  return valid data size
 */

#include <linux/module.h>
#include <rtdm/driver.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("will chen");


struct gpiopwm_base_signal {
	unsigned long period;
};

struct gpiopwm_duty_signal {
	unsigned int range_min;
	unsigned int range_max;
	unsigned long period;
	unsigned int cycle;
};

struct gpiopwm_control {
	struct gpiopwm_duty_signal duty;
	unsigned int configured;
	unsigned int update;
};

struct gpiopwm_priv {
	struct gpiopwm_base_signal base;
	struct gpiopwm_duty_signal duty;
	struct gpiopwm_control ctrl;

	rtdm_timer_t base_timer;
	rtdm_timer_t duty_timer;

	int gpio;
};

// ###########################################

struct gpiopwm_cfg {
	unsigned int duty_cycle;
	unsigned int range_min;
	unsigned int range_max;
	unsigned int period;
	unsigned int gpio;
};


// ###########################################

static struct gpiopwm_priv gpiopwm;
static struct gpiopwm_cfg conf = {
        .duty_cycle     =       80,
        .range_min      =       0,
        .range_max      =       20000,
        .period         =       20000000, //20ms
        .gpio           =       1,
};




static inline int div100(long long dividend)
{
        const long long divisor = 0x28f5c29;
        return ((divisor * dividend) >> 32) & 0xffffffff;
}

static inline unsigned long duty_period(struct gpiopwm_duty_signal *p)
{
        unsigned long period;

        period = p->range_min + div100((p->range_max - p->range_min) * p->cycle);
	printk("period = %lu\n", period);
        return period * 1000;
}

static void gpiopwm_handle_base_timer(rtdm_timer_t *timer)
{
	printk("[%s] 1", __FUNCTION__);
	//gpio_set_value(ctx->gpio, 1);

	/* one shot timer to avoid carrying over errors */
	rtdm_timer_start_in_handler(&gpiopwm.duty_timer, gpiopwm.duty.period, 0,
		RTDM_TIMERMODE_RELATIVE);

	if (gpiopwm.ctrl.update) {
		gpiopwm.duty.period = gpiopwm.ctrl.duty.period;
		gpiopwm.duty.cycle = gpiopwm.ctrl.duty.cycle;
		gpiopwm.ctrl.update = 0;
	}
}

static void gpiopwm_handle_duty_timer(rtdm_timer_t *timer)
{
	printk("[%s] 0", __FUNCTION__);
	//gpio_set_value(ctx->gpio, 0);
}



static void setup_timer(void)
{
	gpiopwm.duty.range_min = gpiopwm.ctrl.duty.range_min = conf.range_min;
	gpiopwm.duty.range_max = gpiopwm.ctrl.duty.range_max = conf.range_max;
	gpiopwm.duty.cycle = conf.duty_cycle;
	gpiopwm.base.period = conf.period;
	gpiopwm.gpio = conf.gpio;
	gpiopwm.duty.period = duty_period(&gpiopwm.duty);

}

static int __init simple_rtdm_init(void)
{
	printk("simple rtdm init\n");
	rtdm_timer_init(&gpiopwm.base_timer, gpiopwm_handle_base_timer, "base_timer");
	rtdm_timer_init(&gpiopwm.duty_timer, gpiopwm_handle_duty_timer, "duty_timer");

	setup_timer();

	rtdm_timer_start(&gpiopwm.base_timer, gpiopwm.base.period, gpiopwm.base.period,
			 RTDM_TIMERMODE_RELATIVE);

	return 0;
}

static void __exit simple_rtdm_exit(void)
{
	printk("simple rtdm exit\n");
	rtdm_timer_stop(&gpiopwm.base_timer);
	rtdm_timer_stop(&gpiopwm.duty_timer);

	rtdm_timer_destroy(&gpiopwm.base_timer);
	rtdm_timer_destroy(&gpiopwm.duty_timer);
}

module_init(simple_rtdm_init);
module_exit(simple_rtdm_exit);
