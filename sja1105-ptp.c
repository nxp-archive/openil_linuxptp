/**
 * @file sja1105-ptp.c
 * @brief definiton for SJA1105 transparent clock support
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
#include <errno.h>
#include <unistd.h>

#include "sja1105-ptp.h"
#include "raw.h"
#include "contain.h"
#include "util.h"
#include "config.h"
#include "hash.h"
#include "transport_private.h"
#include "print.h"
#include "msg.h"
#include "tlv.h"

struct tc	tc;
struct cfg	tc_cfg;
struct host_if	tc_host_if;

struct sja1105_spi_setup spi_setup = {
	.device    = "/dev/spidev0.1",
	.mode      = SPI_CPHA,
	.bits      = 8,
	.speed     = 10000000,
	.delay     = 0,
	.cs_change = 0,
	.fd        = -1,
};

static void usage(void)
{
	printf("\nusage: sja1105-ptp [options]\n\n \
		Network Interface\n \
		-i [name]   host interface name\n\n \
		-h          help\n \
		\n");
}

static int get_cfg(int argc, char *argv[], struct cfg *config)
{
	int c;

	while (EOF != (c = getopt(argc, argv, "i:h"))) {
		switch (c) {
		case 'i':
			config->if_name = optarg;
			break;
		case 'h':
			usage();
			return -1;
		}
	}

	if (!config->if_name) {
		printf("sja1105-ptp: no interface specified!\n");
		usage();
		return -1;
	}

	return 0;
}

static inline void timespec_to_timestamp(struct timespec *src, struct timestamp *dst)
{
	dst->sec = src->tv_sec;
	dst->nsec = src->tv_nsec;
}

static inline uint64_t timestamp_to_ns(struct timestamp ts)
{
	return ts.sec * NS_PER_SEC + ts.nsec;
}

#define DOUBLE_KEEP_31BIT_FRACTION_SHIFT	21
#define UINT32_LOWER_31BIT_MASK			0x7fffffff
#define UINT32_UPPER_1BIT_MASK			0x80000000

static void clock_frequency_sync(void)
{
	struct tc *clock = &tc;
	struct timestamp sync_tx = clock->interface->sync_fup->ts.pdu;
	struct timestamp last_sync_tx = clock->interface->last_sync_fup->ts.pdu;
	struct timestamp sync_rx, last_sync_rx;
	uint64_t sync_tx_interval, sync_rx_interval;
	double ratio;
	uint32_t r;
	double tmp;
	uint64_t *p = (uint64_t *)(&tmp);

	timespec_to_timestamp(&clock->interface->sync->hwts.ts, &sync_rx);
	timespec_to_timestamp(&clock->interface->last_sync->hwts.ts, &last_sync_rx);

	sync_tx_interval = timestamp_to_ns(sync_tx) - timestamp_to_ns(last_sync_tx);
	sync_rx_interval = timestamp_to_ns(sync_rx) - timestamp_to_ns(last_sync_rx);

	ratio = (double) sync_tx_interval / sync_rx_interval;
	//printf("ratio %.9f\n", ratio);
	ratio *= clock->cur_ratio;

	if (ratio <= 0 || ratio >= 2) {
		printf("ratio for frequency sync exceeded register range: %.9f", ratio);
		return;
	}

	tmp = ratio < 1 ? ratio + 1 : ratio;

	r = (*p >> DOUBLE_KEEP_31BIT_FRACTION_SHIFT) &
		UINT32_LOWER_31BIT_MASK;

	if (ratio >= 1)
		r = r | UINT32_UPPER_1BIT_MASK;

	//printf("set ratio %.9f, 0x%x, old value 0x%x\n", ratio, r, clock->cur_ratio_u32);
	if (r == clock->cur_ratio_u32)
		return;

	if (sja1105_ptp_clk_rate_set(&spi_setup, ratio) < 0) {
		printf("setting ratio failed: ratio 0x%x\n", r);
		return ;
	}

	clock->cur_ratio = ratio;
	clock->cur_ratio_u32 = r;
}

static void process_sync(struct ptp_message *m)
{
	struct tc *clock = &tc;

	if (memcmp(&clock->master_id, &m->header.sourcePortIdentity.clockIdentity,
		   sizeof(struct ClockIdentity))) {
		memcpy(&clock->master_id, &m->header.sourcePortIdentity.clockIdentity,
		       sizeof(struct ClockIdentity));

		clock->master_setup = false;
		clock->master_stable = 1;
	} else {
		if (!clock->master_setup)
			clock->master_stable += 1;

		if (clock->master_stable == MASTER_STABLE_CNT) {
			clock->master_setup = true;
			clock->master_stable = 0;
			printf("sja1105-ptp: select master clock %s\n", cid2str(&clock->master_id));
		}
	}

	if (clock->master_setup) {
		msg_get(m);
		clock->interface->sync = m;
	}
}

static void process_sync_fup(struct ptp_message *m)
{
	struct tc *clock = &tc;

	if (!clock->master_setup)
		return;

	if (!clock->interface->sync)
		return;

	if (clock->interface->sync->header.sequenceId !=
					m->header.sequenceId) {
		printf("sync_fup didn't match sync!\n");
		msg_put(clock->interface->sync);
		clock->interface->sync = NULL;
		return;
	}

	msg_get(m);
	clock->interface->sync_fup = m;

	if (clock->interface->last_sync &&
			clock->interface->last_sync_fup) {
		clock_frequency_sync();

		msg_put(clock->interface->last_sync);
		msg_put(clock->interface->last_sync_fup);
	}

	clock->interface->last_sync = clock->interface->sync;
	clock->interface->last_sync_fup = clock->interface->sync_fup;
	clock->interface->sync = NULL;
	clock->interface->sync_fup = NULL;
}

static int interface_recv(struct host_if *interface, int index)
{
	struct ptp_message *msg;
	int cnt, err;

	msg = msg_allocate();
	if (!msg) {
		printf("sja1105-ptp: msg allocate failed!\n");
		return -1;
	}

	msg->hwts.type = TS_HARDWARE;

	cnt = interface->trans->recv(interface->trans, interface->fd_array.fd[index],
		msg, sizeof(msg->data), &msg->address, &msg->hwts);
	if (cnt <= 0) {
		printf("sja1105-ptp: recv message failed\n");
		msg_put(msg);
		return -1;
	}

	err = msg_post_recv(msg, cnt);
	if (err && (err != -ETIME)) {
		switch (err) {
		case -EBADMSG:
			printf("sja1105-ptp: bad message\n");
			break;
		case -ETIME:
			printf("sja1105-ptp: received without timestamp\n");
			break;
		case -EPROTO:
			printf("sja1105-ptp: ignoring message\n");
			break;
		}
		msg_put(msg);
		return -1;
	}

	switch (msg_type(msg)) {
	case SYNC:
		//printf("sja1105-ptp: recv SYNC, timestamp %ld.%09ld\n",
		//	msg->hwts.ts.tv_sec, msg->hwts.ts.tv_nsec);
		process_sync(msg);
		break;
	case FOLLOW_UP:
		process_sync_fup(msg);
		break;
	}

	msg_put(msg);
	return 0;
}

int main(int argc, char *argv[])
{
	struct tc *clock = &tc;
	struct cfg *config = &tc_cfg;
	struct host_if *interface = &tc_host_if;
	int cnt, i;

	if (get_cfg(argc, argv, config))
		return -1;

	if (sja1105_spi_configure(&spi_setup) < 0) {
		printf("spi_configure failed");
		return -1;
	}

	if (sja1105_ptp_reset(&spi_setup)) {
		printf("sja1105: reset failed");
		return -1;
	}

	interface->name = config->if_name;
	interface->trans = raw_transport_create();
	interface->trans->is_sja1105 = true;

	if (interface->trans->open(interface->trans, interface->name,
				   &interface->fd_array, TS_HARDWARE)) {
		printf("sja1105-ptp: raw open failed!\n");
		return -1;
	}

	clock->interface = interface;
	for (i = 0; i < FD_NUM; i++) {
		clock->fd[i].fd = interface->fd_array.fd[i];
		clock->fd[i].events = POLLIN|POLLPRI;
	}

	clock->interface->sync = NULL;
	clock->interface->sync_fup = NULL;
	clock->interface->last_sync = NULL;
	clock->interface->last_sync_fup = NULL;
	clock->cur_ratio = 1.0;
	clock->cur_ratio_u32 = 0x80000000;

	printf("sja1105-ptp: start up sja1105-ptp. Listen to master ...\n");

	while (true) {
		cnt = poll(clock->fd, FD_NUM - 1, -1);
		if (cnt <=0) {
			printf("sja1105-ptp: poll failed!\n");
			return -1;
		}

		for (i = 0; i < FD_NUM - 1; i++) {
			if (!(clock->fd[i].revents & (POLLIN|POLLPRI)))
				continue;

			if (interface_recv(interface, i))
				return -1;
		}
	}

	return 0;
}
