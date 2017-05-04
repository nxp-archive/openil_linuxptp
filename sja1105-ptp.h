/**
 * @file sja1105-ptp.h
 * @brief definiton for SJA1105 transparent clock specific structures
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
#ifndef HAVE_SJA1105_PTP_H
#define HAVE_SJA1105_PTP_H

#include <stdbool.h>
#include <poll.h>

#include "fd.h"
#include "ddt.h"

#define FD_NUM			2
#define MASTER_STABLE_CNT	3

struct cfg {
	char *if_name;
};

struct host_if {
	const char *name;
	struct transport *trans;
	struct fdarray fd_array;
};

struct tc {
	struct host_if *interface;
	struct pollfd fd[FD_NUM];
	struct ClockIdentity master_id;
	bool master_setup;
	int master_stable;
};

extern struct tc	tc;
extern struct cfg	tc_cfg;
extern struct host_if	tc_host_if;

#endif
