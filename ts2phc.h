/**
 * @file ts2phc.h
 * @brief Structure definitions for ts2phc
 * @note Copyright 2020 Vladimir Oltean <olteanv@gmail.com>
 * @note SPDX-License-Identifier: GPL-2.0+
 */
#ifndef HAVE_TS2PHC_H
#define HAVE_TS2PHC_H

#include <sys/queue.h>
#include <time.h>
#include "servo.h"

struct ts2phc_slave_array;

#define SERVO_SYNC_INTERVAL    1.0

struct clock {
	LIST_ENTRY(clock) list;
	LIST_ENTRY(clock) dst_list;
	clockid_t clkid;
	int phc_index;
	int state;
	int new_state;
	struct servo *servo;
	enum servo_state servo_state;
	char *name;
	int no_adj;
	int is_destination;
};

struct ts2phc_private {
	struct ts2phc_master *master;
	STAILQ_HEAD(slave_ifaces_head, ts2phc_slave) slaves;
	unsigned int n_slaves;
	struct ts2phc_slave_array *polling_array;
	struct config *cfg;
	struct clock *source;
	LIST_HEAD(clock_head, clock) clocks;
};

struct servo *servo_add(struct ts2phc_private *priv, struct clock *clock);
struct clock *clock_add(struct ts2phc_private *priv, const char *device);
void clock_add_tstamp(struct clock *clock, struct timespec ts);
void clock_destroy(struct clock *clock);

#include "ts2phc_master.h"
#include "ts2phc_slave.h"

#endif
