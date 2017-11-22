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

#include <sja1105/ptp.h>
#include <sja1105/dynamic-config.h>

#define FD_NUM			3
#define FD_META			2

#define MASTER_STABLE_CNT	3

#define SJA1105_PORT		0x001f
#define SJA1105_PORT_HOST	0x0010

#define PTP_E2E_ETH_MULTI_ADDR	0x011B19000000

struct cfg {
	char *if_name;
};

struct host_if {
	const char *name;
	struct transport *trans;
	struct fdarray fd_array;
	struct ptp_message *last_sync;
	struct ptp_message *last_sync_fup;
	struct ptp_message *sync;
	struct ptp_message *sync_fup;
	struct ptp_message *delay_req;
};

struct tc {
	struct host_if *interface;
	struct pollfd fd[FD_NUM];
	struct ClockIdentity master_id;
	bool master_setup;
	int master_stable;
	double cur_ratio;
	uint32_t cur_ratio_u32;
};

struct meta_data {
	char reserve;
	char rx_ts_byte2;
	char rx_ts_byte1;
	char rx_ts_byte0;
	char dst_mac_byte1;
	char dst_mac_byte0;
	char src_port;
	char switch_id;
};

struct sja1105_egress_ts {
	//assume frames are forwarded out on each port at same time
	uint64_t tx_ts;
	uint8_t ts_index;
	uint8_t available;
};

extern struct tc	tc;
extern struct cfg	tc_cfg;
extern struct host_if	tc_host_if;

extern struct sja1105_spi_setup		spi_setup;
extern struct sja1105_egress_ts		sync_tx_ts;
extern struct sja1105_egress_ts		delay_req_tx_ts;
extern struct sja1105_egress_ts		egress_ts_tmp;

#endif
