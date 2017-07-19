/**
 * @file sja1105.c
 * @brief definiton for SJA1105 sync functions
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
#include <ptp.h>

#include "missing.h"
#include "print.h"
#include "sja1105.h"

int SJA1105_VERBOSE_CONDITION = 1;
int SJA1105_DEBUG_CONDITION = 1;

struct sja1105_spi_setup spi_setup = {
	.device    = "/dev/spidev0.1",
	.mode      = SPI_CPHA,
	.bits      = 8,
	.speed     = 1000000,
	.delay     = 0,
	.cs_change = 0,
	.fd        = -1,
};

int sja1105_sync_timer_create(struct config *config)
{
	struct sja1105_sync_timer *t = &sja1105_sync_t;
	struct sja1105_sync_pi_servo *s = &sja1105_sync_pi_s;

	t->max_offset = config_get_int(config, NULL, "sja1105_max_offset");
	if (!t->max_offset) {
		pr_debug("sja1105: don't create timer for sync");
		return -1;
	}

	t->max_offset *= 1000;

	pr_debug("sja1105: initialize sja1105 and create timer for sync");

	if (sja1105_spi_configure(&spi_setup) < 0) {
		pr_err("spi_configure failed");
		return -1;
	}

	t->fd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (t->fd < 0) {
		pr_err("sja1105: failed to create timer for sync");
		return -1;
	}

	t->valid = 1;
	t->ratio = 1;
	t->reset_req = 1;

	s->kp = config_get_double(config, NULL, "sja1105_sync_kp");
	s->ki = config_get_double(config, NULL, "sja1105_sync_ki");

	return 0;
}

void sja1105_sync_fill_pollfd(struct pollfd *dest)
{
	struct sja1105_sync_timer *t = &sja1105_sync_t;

	dest[0].fd = t->fd;
	dest[0].events = POLLIN|POLLPRI;
}

int sja1105_sync_timer_settime(void)
{
	struct sja1105_sync_timer *t = &sja1105_sync_t;
	/* Sync 8 times every second */
	struct itimerspec tmo = {
		{0, 0}, {0, 125000000}
	};

	if (timerfd_settime(t->fd, 0, &tmo, NULL)) {
		pr_err("sja1105: failed to set sync timer");
		t->valid = 0;
		close(t->fd);
		return -1;
	}
	return 0;
}

#define DOUBLE_KEEP_31BIT_FRACTION_SHIFT	21
#define UINT32_LOWER_31BIT_MASK			0x7fffffff
#define UINT32_UPPER_1BIT_MASK			0x80000000

static int sja1105_set_clock_ratio(double ratio)
{
	uint32_t r;
	double tmp;
	uint64_t *p = (uint64_t *)(&tmp);

	if (ratio <= 0 || ratio >=2 ) {
		pr_err("sja1105: ratio %f exceeds register range", ratio);
		return -1;
	}

	tmp = ratio < 1 ? ratio + 1 : ratio;

	r = (*p >> DOUBLE_KEEP_31BIT_FRACTION_SHIFT) &
		UINT32_LOWER_31BIT_MASK;

	if (ratio >= 1)
		r = r | UINT32_UPPER_1BIT_MASK;

	if (sja1105_ptp_clk_rate_set(&spi_setup, r) < 0) {
		pr_err("sja1105_ptp_clk_rate_set(%" PRIu32 ") failed", r);
		return -1;
	}

	return 0;
}

static int sja1105_calculate(clockid_t clkid, int64_t *delay, int64_t *offset)
{
	struct timespec t1_spec, t3_spec;
	uint64_t t2_ns;
	int gettings;
	int64_t interval, best_interval = INT64_MAX;
	int rc1, rc2, rc3;

	memset(&t1_spec, 0, sizeof(t1_spec));
	memset(&t3_spec, 0, sizeof(t3_spec));

	/* Pick the best interval */
	for (gettings = 0; gettings < 3; gettings ++) {
		if ((rc1 = clock_gettime(clkid, &t1_spec)) ||
		    (rc2 = sja1105_ptp_clk_get(&spi_setup, &t2_ns)) < 0 ||
		    (rc3 = clock_gettime(clkid, &t3_spec))) {
			pr_err("rc1 %d rc2 %d rc3 %d", rc1, rc2, rc3);
			pr_err("sja1105: calculating got time error");
			return -1;
		}

		interval = (t3_spec.tv_sec - t1_spec.tv_sec) * NS_PER_SEC +
				(t3_spec.tv_nsec - t1_spec.tv_nsec);
		if (interval < best_interval) {
			best_interval = interval;
			*offset = t2_ns * 8 - t1_spec.tv_sec * NS_PER_SEC -
					t1_spec.tv_nsec - interval / 2;
		}
	}
	*delay = best_interval / 2;

	return 0;
}

#define ADJ_SCALE	10000000

static double sja1105_sync_run_pi_servo(int64_t offset)
{
	struct sja1105_sync_pi_servo *s = &sja1105_sync_pi_s;
	int64_t adj;

	s->drift_sum += offset * s->ki;
	if (s->drift_sum > ADJ_SCALE)
		s->drift_sum = ADJ_SCALE;
	if (s->drift_sum < - ADJ_SCALE)
		s->drift_sum = - ADJ_SCALE;

	adj = offset * s->kp + s->drift_sum;
	adj = - adj;

	return (double)adj / (double)ADJ_SCALE;
}

int sja1105_sync(clockid_t clkid)
{
	struct sja1105_sync_timer *t = &sja1105_sync_t;
	struct sja1105_sync_pi_servo *s = &sja1105_sync_pi_s;
	struct timespec cur_t;
	uint64_t cur_ns;
	int64_t delay, offset;
	double adj;

	if (t->reset_req) {
		/* Step 1, reset sja1105 ratio */
		if (sja1105_set_clock_ratio(1.0))
			return -1;

		/*
		 * Step 2, set sja1105 time prior to master about 1s.
		 * This makes we can set PTPCLKADD register with offset later.
		 */
		if (clock_gettime(clkid, &cur_t)) {
			pr_err("sja1105: clock_gettime error");
			return -1;
		}

		cur_ns = (uint64_t) (cur_t.tv_sec - 1) * NS_PER_SEC +
				(uint64_t) cur_t.tv_nsec;

		if (sja1105_ptp_clk_set(&spi_setup, cur_ns / 8) < 0) {
			pr_err("sja1105_ptp_clk_set failed");
			return -1;
		}

		/* Step 3, calculate delay and offset */
		if (sja1105_calculate(clkid, &delay, &offset))
			return -1;

		/* Step 4, set offset into PTPCLKADD */
		if (offset > 0)
			return -1;

		if (sja1105_ptp_clk_add(&spi_setup, - offset / 8) < 0) {
			pr_err("sja1105_ptp_clk_add failed");
			return -1;
		}

		t->reset_req = 0;
		s->drift_sum = 0;
	}

	if (sja1105_calculate(clkid, &delay, &offset))
		return -1;

	pr_debug("sja1105: offset %9lld ns, delay %9lld ns", offset, delay);

	if (offset >= t->max_offset || offset <= - t->max_offset) {
		pr_err("sja1105: offset from master exceeded max value %d ns", t->max_offset);

		if (offset >= NS_PER_SEC || offset <= - NS_PER_SEC)
			t->reset_req = 1;

		return 0;
	}

	adj = sja1105_sync_run_pi_servo(offset);

	if (sja1105_set_clock_ratio(t->ratio + adj))
		return -1;

	return 0;
}
