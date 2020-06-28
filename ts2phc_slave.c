/**
 * @file ts2phc_slave.c
 * @brief Utility program to synchronize the PHC clock to external events
 * @note Copyright (C) 2019 Balint Ferencz <fernya@sch.bme.hu>
 * @note SPDX-License-Identifier: GPL-2.0+
 */
#include <errno.h>
#include <linux/ptp_clock.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/queue.h>
#include <time.h>
#include <unistd.h>

#include "config.h"
#include "clockadj.h"
#include "missing.h"
#include "phc.h"
#include "print.h"
#include "servo.h"
#include "ts2phc.h"
#include "util.h"

#define NS_PER_SEC		1000000000LL
#define SAMPLE_WEIGHT		1.0
#define SERVO_SYNC_INTERVAL	1.0

struct ts2phc_slave {
	char *name;
	STAILQ_ENTRY(ts2phc_slave) list;
	struct ptp_pin_desc pin_desc;
	enum servo_state state;
	unsigned int polarity;
	int32_t correction;
	uint32_t ignore_lower;
	uint32_t ignore_upper;
	struct servo *servo;
	clockid_t clk;
	int no_adj;
	int fd;
};

struct ts2phc_slave_array {
	struct ts2phc_slave **slave;
	struct pollfd *pfd;
};

struct ts2phc_source_timestamp {
	struct timespec ts;
	bool valid;
};

enum extts_result {
	EXTTS_ERROR	= -1,
	EXTTS_OK	= 0,
	EXTTS_IGNORE	= 1,
};

static enum extts_result ts2phc_slave_offset(struct ts2phc_slave *slave,
					     struct ts2phc_source_timestamp ts,
					     int64_t *offset,
					     uint64_t *local_ts);

static int ts2phc_slave_array_create(struct ts2phc_private *priv)
{
	struct ts2phc_slave_array *polling_array;
	struct ts2phc_slave *slave;
	unsigned int i;

	polling_array = malloc(sizeof(*polling_array));
	if (!polling_array) {
		pr_err("low memory");
		return -1;
	}

	polling_array->slave = malloc(priv->n_slaves *
				      sizeof(*polling_array->slave));
	if (!polling_array->slave) {
		pr_err("low memory");
		return -1;
	}
	polling_array->pfd = malloc(priv->n_slaves *
				    sizeof(*polling_array->pfd));
	if (!polling_array->pfd) {
		pr_err("low memory");
		free(polling_array->slave);
		polling_array->slave = NULL;
		return -1;
	}
	i = 0;
	STAILQ_FOREACH(slave, &priv->slaves, list) {
		polling_array->slave[i] = slave;
		i++;
	}
	for (i = 0; i < priv->n_slaves; i++) {
		polling_array->pfd[i].events = POLLIN | POLLPRI;
		polling_array->pfd[i].fd = polling_array->slave[i]->fd;
	}

	priv->polling_array = polling_array;

	return 0;
}

static void ts2phc_slave_array_destroy(struct ts2phc_private *priv)
{
	struct ts2phc_slave_array *polling_array = priv->polling_array;

	free(polling_array->slave);
	free(polling_array->pfd);
	polling_array->slave = NULL;
	polling_array->pfd = NULL;
}

static int ts2phc_slave_clear_fifo(struct ts2phc_slave *slave)
{
	struct pollfd pfd = {
		.events = POLLIN | POLLPRI,
		.fd = slave->fd,
	};
	struct ptp_extts_event event;
	int cnt, size;

	while (1) {
		cnt = poll(&pfd, 1, 0);
		if (cnt < 0) {
			if (EINTR == errno) {
				continue;
			} else {
				pr_emerg("poll failed");
				return -1;
			}
		} else if (!cnt) {
			break;
		}
		size = read(pfd.fd, &event, sizeof(event));
		if (size != sizeof(event)) {
			pr_err("read failed");
			return -1;
		}
		pr_debug("%s SKIP extts index %u at %lld.%09u",
			 slave->name, event.index, event.t.sec, event.t.nsec);
	}

	return 0;
}

static struct ts2phc_slave *ts2phc_slave_create(struct ts2phc_private *priv,
						const char *device)
{
	enum servo_type servo = config_get_int(priv->cfg, NULL, "clock_servo");
	int err, fadj, junk, max_adj, pulsewidth;
	struct ptp_extts_request extts;
	struct ts2phc_slave *slave;

	slave = calloc(1, sizeof(*slave));
	if (!slave) {
		pr_err("low memory");
		return NULL;
	}
	slave->name = strdup(device);
	if (!slave->name) {
		pr_err("low memory");
		free(slave);
		return NULL;
	}
	slave->pin_desc.index = config_get_int(priv->cfg, device,
					       "ts2phc.pin_index");
	slave->pin_desc.func = PTP_PF_EXTTS;
	slave->pin_desc.chan = config_get_int(priv->cfg, device,
					      "ts2phc.channel");
	slave->polarity = config_get_int(priv->cfg, device,
					 "ts2phc.extts_polarity");
	slave->correction = config_get_int(priv->cfg, device,
					   "ts2phc.extts_correction");

	pulsewidth = config_get_int(priv->cfg, device,
				    "ts2phc.pulsewidth");
	pulsewidth /= 2;
	slave->ignore_upper = 1000000000 - pulsewidth;
	slave->ignore_lower = pulsewidth;

	slave->clk = posix_clock_open(device, &junk);
	if (slave->clk == CLOCK_INVALID) {
		pr_err("failed to open clock");
		goto no_posix_clock;
	}
	slave->no_adj = config_get_int(priv->cfg, NULL, "free_running");
	slave->fd = CLOCKID_TO_FD(slave->clk);

	pr_debug("PHC slave %s has ptp index %d", device, junk);

	fadj = (int) clockadj_get_freq(slave->clk);
	/* Due to a bug in older kernels, the reading may silently fail
	   and return 0. Set the frequency back to make sure fadj is
	   the actual frequency of the clock. */
	clockadj_set_freq(slave->clk, fadj);

	max_adj = phc_max_adj(slave->clk);

	slave->servo = servo_create(priv->cfg, servo, -fadj, max_adj, 0);
	if (!slave->servo) {
		pr_err("failed to create servo");
		goto no_servo;
	}
	servo_sync_interval(slave->servo, SERVO_SYNC_INTERVAL);

	if (phc_number_pins(slave->clk) > 0) {
		err = phc_pin_setfunc(slave->clk, &slave->pin_desc);
		if (err < 0) {
			pr_err("PTP_PIN_SETFUNC request failed");
			goto no_pin_func;
		}
	}

	/*
	 * Disable external time stamping, and then read out any stale
	 * time stamps.
	 */
	memset(&extts, 0, sizeof(extts));
	extts.index = slave->pin_desc.chan;
	extts.flags = 0;
	if (ioctl(slave->fd, PTP_EXTTS_REQUEST2, &extts)) {
		pr_err(PTP_EXTTS_REQUEST_FAILED);
	}
	if (ts2phc_slave_clear_fifo(slave)) {
		goto no_ext_ts;
	}

	return slave;
no_ext_ts:
no_pin_func:
	servo_destroy(slave->servo);
no_servo:
	posix_clock_close(slave->clk);
no_posix_clock:
	free(slave->name);
	free(slave);
	return NULL;
}

static void ts2phc_slave_destroy(struct ts2phc_slave *slave)
{
	struct ptp_extts_request extts;

	memset(&extts, 0, sizeof(extts));
	extts.index = slave->pin_desc.chan;
	extts.flags = 0;
	if (ioctl(slave->fd, PTP_EXTTS_REQUEST2, &extts)) {
		pr_err(PTP_EXTTS_REQUEST_FAILED);
	}
	servo_destroy(slave->servo);
	posix_clock_close(slave->clk);
	free(slave->name);
	free(slave);
}

static int ts2phc_slave_event(struct ts2phc_slave *slave,
			      struct ts2phc_source_timestamp source_ts)
{
	enum extts_result result;
	uint64_t extts_ts;
	int64_t offset;
	double adj;

	result = ts2phc_slave_offset(slave, source_ts, &offset, &extts_ts);
	switch (result) {
	case EXTTS_ERROR:
		return -1;
	case EXTTS_OK:
		break;
	case EXTTS_IGNORE:
		return 0;
	}

	if (slave->no_adj) {
		pr_info("%s master offset %10" PRId64, slave->name, offset);
		return 0;
	}

	if (!source_ts.valid) {
		pr_debug("%s ignoring invalid master time stamp", slave->name);
		return 0;
	}

	adj = servo_sample(slave->servo, offset, extts_ts,
			   SAMPLE_WEIGHT, &slave->state);

	pr_debug("%s master offset %10" PRId64 " s%d freq %+7.0f",
		 slave->name, offset, slave->state, adj);

	switch (slave->state) {
	case SERVO_UNLOCKED:
		break;
	case SERVO_JUMP:
		clockadj_set_freq(slave->clk, -adj);
		clockadj_step(slave->clk, -offset);
		break;
	case SERVO_LOCKED:
	case SERVO_LOCKED_STABLE:
		clockadj_set_freq(slave->clk, -adj);
		break;
	}
	return 0;
}

static enum extts_result ts2phc_slave_offset(struct ts2phc_slave *slave,
					     struct ts2phc_source_timestamp src,
					     int64_t *offset,
					     uint64_t *local_ts)
{
	struct timespec source_ts = src.ts;
	struct ptp_extts_event event;
	uint64_t event_ns, source_ns;
	int cnt;

	cnt = read(slave->fd, &event, sizeof(event));
	if (cnt != sizeof(event)) {
		pr_err("read extts event failed: %m");
		return EXTTS_ERROR;
	}
	if (event.index != slave->pin_desc.chan) {
		pr_err("extts on unexpected channel");
		return EXTTS_ERROR;
	}
	event_ns = event.t.sec * NS_PER_SEC;
	event_ns += event.t.nsec;

	if (slave->polarity == (PTP_RISING_EDGE | PTP_FALLING_EDGE) &&
	    source_ts.tv_nsec > slave->ignore_lower &&
	    source_ts.tv_nsec < slave->ignore_upper) {

		pr_debug("%s SKIP extts index %u at %lld.%09u src %" PRIi64 ".%ld",
		 slave->name, event.index, event.t.sec, event.t.nsec,
		 (int64_t) source_ts.tv_sec, source_ts.tv_nsec);

		return EXTTS_IGNORE;
	}
	if (source_ts.tv_nsec > 500000000) {
		source_ts.tv_sec++;
	}
	source_ns = source_ts.tv_sec * NS_PER_SEC;
	*offset = event_ns + slave->correction - source_ns;
	*local_ts = event_ns + slave->correction;

	pr_debug("%s extts index %u at %lld.%09u corr %d src %" PRIi64
		 ".%ld diff %" PRId64,
		 slave->name, event.index, event.t.sec, event.t.nsec,
		 slave->correction,
		 (int64_t) source_ts.tv_sec, source_ts.tv_nsec, *offset);

	return EXTTS_OK;
}

/* public methods */

int ts2phc_slave_add(struct ts2phc_private *priv, const char *name)
{
	struct ts2phc_slave *slave;

	/* Create each interface only once. */
	STAILQ_FOREACH(slave, &priv->slaves, list) {
		if (0 == strcmp(name, slave->name)) {
			return 0;
		}
	}
	slave = ts2phc_slave_create(priv, name);
	if (!slave) {
		pr_err("failed to create slave");
		return -1;
	}
	STAILQ_INSERT_TAIL(&priv->slaves, slave, list);
	priv->n_slaves++;

	return 0;
}

int ts2phc_slave_arm(struct ts2phc_private *priv)
{
	struct ptp_extts_request extts;
	struct ts2phc_slave *slave;
	int err;

	memset(&extts, 0, sizeof(extts));

	STAILQ_FOREACH(slave, &priv->slaves, list) {
		extts.index = slave->pin_desc.chan;
		extts.flags = slave->polarity | PTP_ENABLE_FEATURE;
		err = ioctl(slave->fd, PTP_EXTTS_REQUEST2, &extts);
		if (err < 0) {
			pr_err(PTP_EXTTS_REQUEST_FAILED);
			return -1;
		}
	}
	return 0;
}

int ts2phc_slaves_init(struct ts2phc_private *priv)
{
	int err;

	err = ts2phc_slave_array_create(priv);
	if (err)
		return err;

	return ts2phc_slave_arm(priv);
}

void ts2phc_slave_cleanup(struct ts2phc_private *priv)
{
	struct ts2phc_slave *slave;

	ts2phc_slave_array_destroy(priv);

	while ((slave = STAILQ_FIRST(&priv->slaves))) {
		STAILQ_REMOVE_HEAD(&priv->slaves, list);
		ts2phc_slave_destroy(slave);
		priv->n_slaves--;
	}
}

int ts2phc_slave_poll(struct ts2phc_private *priv)
{
	struct ts2phc_slave_array *polling_array = priv->polling_array;
	struct ts2phc_source_timestamp source_ts;
	unsigned int i;
	int cnt, err;

	cnt = poll(polling_array->pfd, priv->n_slaves, 2000);
	if (cnt < 0) {
		if (EINTR == errno) {
			return 0;
		} else {
			pr_emerg("poll failed");
			return -1;
		}
	} else if (!cnt) {
		pr_debug("poll returns zero, no events");
		return 0;
	}

	err = ts2phc_master_getppstime(priv->master, &source_ts.ts);
	source_ts.valid = err ? false : true;

	for (i = 0; i < priv->n_slaves; i++) {
		if (polling_array->pfd[i].revents & (POLLIN|POLLPRI)) {
			ts2phc_slave_event(polling_array->slave[i], source_ts);
		}
	}
	return 0;
}
