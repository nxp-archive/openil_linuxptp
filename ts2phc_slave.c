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

#include "clockadj.h"
#include "config.h"
#include "missing.h"
#include "phc.h"
#include "print.h"
#include "servo.h"
#include "ts2phc.h"
#include "util.h"

struct ts2phc_slave {
	char *name;
	STAILQ_ENTRY(ts2phc_slave) list;
	struct ptp_pin_desc pin_desc;
	unsigned int polarity;
	tmv_t correction;
	uint32_t ignore_lower;
	uint32_t ignore_upper;
	struct clock *clock;
};

struct ts2phc_slave_array {
	struct ts2phc_slave **slave;
	int *collected_events;
	struct pollfd *pfd;
};

enum extts_result {
	EXTTS_ERROR	= -1,
	EXTTS_OK	= 0,
	EXTTS_IGNORE	= 1,
};

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
	polling_array->collected_events = malloc(priv->n_slaves * sizeof(int));
	if (!polling_array->collected_events) {
		pr_err("low memory");
		free(polling_array->slave);
		free(polling_array->pfd);
		polling_array->pfd = NULL;
		polling_array->slave = NULL;
		return -1;
	}
	i = 0;
	STAILQ_FOREACH(slave, &priv->slaves, list) {
		polling_array->slave[i] = slave;
		i++;
	}
	for (i = 0; i < priv->n_slaves; i++) {
		struct ts2phc_slave *slave = polling_array->slave[i];

		polling_array->pfd[i].events = POLLIN | POLLPRI;
		polling_array->pfd[i].fd = CLOCKID_TO_FD(slave->clock->clkid);
	}

	priv->polling_array = polling_array;

	return 0;
}

static void ts2phc_slave_array_destroy(struct ts2phc_private *priv)
{
	struct ts2phc_slave_array *polling_array = priv->polling_array;

	if (!polling_array)
		return;

	free(polling_array->slave);
	free(polling_array->pfd);
	free(polling_array->collected_events);
	free(polling_array);
	priv->polling_array = NULL;
}

static int ts2phc_slave_clear_fifo(struct ts2phc_slave *slave)
{
	struct pollfd pfd = {
		.events = POLLIN | POLLPRI,
		.fd = CLOCKID_TO_FD(slave->clock->clkid),
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
	struct ptp_extts_request extts;
	struct ts2phc_slave *slave;
	int err, pulsewidth;
	int32_t correction;

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
	correction = config_get_int(priv->cfg, device,
				    "ts2phc.extts_correction");
	slave->correction = nanoseconds_to_tmv(correction);

	pulsewidth = config_get_int(priv->cfg, device,
				    "ts2phc.pulsewidth");
	pulsewidth /= 2;
	slave->ignore_upper = 1000000000 - pulsewidth;
	slave->ignore_lower = pulsewidth;

	slave->clock = clock_add(priv, device);
	if (!slave->clock) {
		pr_err("failed to open clock");
		goto no_posix_clock;
	}
	slave->clock->is_destination = 1;

	pr_debug("PHC slave %s has ptp index %d", device,
		 slave->clock->phc_index);

	if (phc_number_pins(slave->clock->clkid) > 0) {
		err = phc_pin_setfunc(slave->clock->clkid, &slave->pin_desc);
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
	if (ioctl(CLOCKID_TO_FD(slave->clock->clkid), PTP_EXTTS_REQUEST2,
		  &extts)) {
		pr_err(PTP_EXTTS_REQUEST_FAILED);
	}
	if (ts2phc_slave_clear_fifo(slave)) {
		goto no_ext_ts;
	}

	return slave;
no_ext_ts:
no_pin_func:
	clock_destroy(slave->clock);
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
	if (ioctl(CLOCKID_TO_FD(slave->clock->clkid), PTP_EXTTS_REQUEST2,
		  &extts)) {
		pr_err(PTP_EXTTS_REQUEST_FAILED);
	}
	clock_destroy(slave->clock);
	free(slave->name);
	free(slave);
}

static enum extts_result ts2phc_slave_event(struct ts2phc_private *priv,
					    struct ts2phc_slave *slave)
{
	enum extts_result result = EXTTS_OK;
	struct ptp_extts_event event;
	struct timespec source_ts;
	int err, cnt;
	tmv_t ts;

	cnt = read(CLOCKID_TO_FD(slave->clock->clkid), &event, sizeof(event));
	if (cnt != sizeof(event)) {
		pr_err("read extts event failed: %m");
		result = EXTTS_ERROR;
		goto out;
	}
	if (event.index != slave->pin_desc.chan) {
		pr_err("extts on unexpected channel");
		result = EXTTS_ERROR;
		goto out;
	}

	err = ts2phc_master_getppstime(priv->master, &source_ts);
	if (err < 0) {
		pr_debug("source ts not valid");
		return 0;
	}

	if (slave->polarity == (PTP_RISING_EDGE | PTP_FALLING_EDGE) &&
	    source_ts.tv_nsec > slave->ignore_lower &&
	    source_ts.tv_nsec < slave->ignore_upper) {

		pr_debug("%s SKIP extts index %u at %lld.%09u src %" PRIi64 ".%ld",
		 slave->name, event.index, event.t.sec, event.t.nsec,
		 (int64_t) source_ts.tv_sec, source_ts.tv_nsec);

		result = EXTTS_IGNORE;
		goto out;
	}

out:
	if (result == EXTTS_ERROR || result == EXTTS_IGNORE)
		return result;

	ts = pct_to_tmv(event.t);
	ts = tmv_add(ts, slave->correction);
	clock_add_tstamp(slave->clock, ts);

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
		err = ioctl(CLOCKID_TO_FD(slave->clock->clkid),
			    PTP_EXTTS_REQUEST2, &extts);
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
	int all_slaves_have_events = 0;
	int ignore_any = 0;
	unsigned int i;
	int cnt;

	for (i = 0; i < priv->n_slaves; i++)
		polling_array->collected_events[i] = 0;

	while (!all_slaves_have_events) {
		struct ts2phc_slave *slave;

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

		for (i = 0; i < priv->n_slaves; i++) {
			if (polling_array->pfd[i].revents & (POLLIN|POLLPRI)) {
				enum extts_result result;

				slave = polling_array->slave[i];

				result = ts2phc_slave_event(priv, slave);
				if (result == EXTTS_ERROR)
					return -EIO;
				if (result == EXTTS_IGNORE)
					ignore_any = 1;

				/*
				 * Collect the events anyway, even if we'll
				 * ignore this master edge anyway. We don't
				 * want slave events from different edges
				 * to pile up and mix.
				 */
				polling_array->collected_events[i]++;
			}
		}

		all_slaves_have_events = true;

		for (i = 0; i < priv->n_slaves; i++) {
			if (!polling_array->collected_events[i]) {
				all_slaves_have_events = false;
				break;
			}
		}
	}

	if (ignore_any)
		return 0;

	return 1;
}
