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
	if (err) {
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

	//printf("sja11-5-tc: recv msg %s\n", msg_type_string(msg_type(msg)));
	switch (msg_type(msg)) {
	case SYNC:
		process_sync(msg);
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

	interface->name = config->if_name;
	interface->trans = raw_transport_create();
	interface->trans->is_sja1105 = true;

	if (interface->trans->open(interface->trans, interface->name,
				   &interface->fd_array, TS_HARDWARE)) {
		printf("sja1105-ptp: raw open failed!\n");
		return -1;
	}

	clock->interface = interface;
	clock->fd[0].fd = interface->fd_array.fd[FD_EVENT];
	clock->fd[0].events = POLLIN|POLLPRI;
	clock->fd[1].fd = interface->fd_array.fd[FD_GENERAL];
	clock->fd[1].events = POLLIN|POLLPRI;

	printf("sja1105-ptp: start up sja1105-ptp. Listen to master ...\n");

	while (true) {
		cnt = poll(clock->fd, FD_NUM, -1);
		if (cnt <=0) {
			printf("sja1105-ptp: poll failed!\n");
			return -1;
		}

		for (i = 0; i < FD_NUM; i++) {
			if (!(clock->fd[i].revents & (POLLIN|POLLPRI)))
				continue;

			if (interface_recv(interface, i))
				return -1;
		}
	}

	return 0;
}
