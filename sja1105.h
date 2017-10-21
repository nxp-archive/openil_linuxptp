/**
 * @file sja1105.h
 * @brief definition for SJA1105 specific structures
 * @note Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef HAVE_SJA1105_H
#define HAVE_SJA1105_H

#include <poll.h>

#include "config.h"

struct sja1105_sync_pi_servo {
	double kp;
	double ki;
	int64_t drift_sum;
};

struct sja1105_sync_timer {
	int     fd;
	int     valid;
	int64_t max_offset;
	int     reset_req;
	double  ratio;
	int     have_qbv;
	struct  timespec qbv_cycle_len;
	struct  sja1105_sync_pi_servo sync_pi_s;
};

int sja1105_sync_timer_is_valid();
int sja1105_sync_timer_create(struct config *config);
void sja1105_sync_fill_pollfd(struct pollfd *dest);
int sja1105_sync_timer_settime(void);
int sja1105_sync(clockid_t clkid);

#endif
