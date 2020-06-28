/**
 * @file ts2phc.h
 * @brief Structure definitions for ts2phc
 * @note Copyright 2020 Vladimir Oltean <olteanv@gmail.com>
 * @note SPDX-License-Identifier: GPL-2.0+
 */
#ifndef HAVE_TS2PHC_H
#define HAVE_TS2PHC_H

struct ts2phc_slave_array;

struct ts2phc_private {
	struct ts2phc_master *master;
	STAILQ_HEAD(slave_ifaces_head, ts2phc_slave) slaves;
	unsigned int n_slaves;
	struct ts2phc_slave_array *polling_array;
	struct config *cfg;
};

#include "ts2phc_master.h"
#include "ts2phc_slave.h"

#endif
