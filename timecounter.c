// SPDX-License-Identifier: GPL-2.0+
/*
 * Based on clocksource code. See commit 74d23cc704d1
 */
#include "timecounter.h"

void timecounter_init(struct timecounter *tc,
		      const struct cyclecounter *cc,
		      uint64_t start_tstamp)
{
	tc->cc = cc;
	tc->cycle_last = cc->read(cc);
	tc->nsec = start_tstamp;
	tc->mask = (1ULL << cc->shift) - 1;
	tc->frac = 0;
}

/**
 * timecounter_read_delta - get nanoseconds since last call of this function
 * @tc:         Pointer to time counter
 *
 * When the underlying cycle counter runs over, this will be handled
 * correctly as long as it does not run over more than once between
 * calls.
 *
 * The first call to this function for a new time counter initializes
 * the time tracking and returns an undefined result.
 */
static uint64_t timecounter_read_delta(struct timecounter *tc)
{
	uint64_t cycle_now, cycle_delta;
	uint64_t ns_offset;

	/* read cycle counter: */
	cycle_now = tc->cc->read(tc->cc);

	/* calculate the delta since the last timecounter_read_delta(): */
	cycle_delta = (cycle_now - tc->cycle_last) & tc->cc->mask;

	/* convert to nanoseconds: */
	ns_offset = cyclecounter_cyc2ns(tc->cc, cycle_delta,
					tc->mask, &tc->frac);

	/* update time stamp of timecounter_read_delta() call: */
	tc->cycle_last = cycle_now;

	return ns_offset;
}

uint64_t timecounter_read(struct timecounter *tc)
{
	uint64_t nsec;

	/* increment time by nanoseconds since last call */
	nsec = timecounter_read_delta(tc);
	nsec += tc->nsec;
	tc->nsec = nsec;

	return nsec;
}

/*
 * This is like cyclecounter_cyc2ns(), but it is used for computing a
 * time previous to the time stored in the cycle counter.
 */
static uint64_t cc_cyc2ns_backwards(const struct cyclecounter *cc,
			       uint64_t cycles, uint64_t mask, uint64_t frac)
{
	uint64_t ns = (uint64_t) cycles;

	ns = ((ns * cc->mult) - frac) >> cc->shift;

	return ns;
}

uint64_t timecounter_cyc2time(struct timecounter *tc,
			 uint64_t cycle_tstamp)
{
	uint64_t delta = (cycle_tstamp - tc->cycle_last) & tc->cc->mask;
	uint64_t nsec = tc->nsec, frac = tc->frac;

	/*
	 * Instead of always treating cycle_tstamp as more recent
	 * than tc->cycle_last, detect when it is too far in the
	 * future and treat it as old time stamp instead.
	 */
	if (delta > tc->cc->mask / 2) {
		delta = (tc->cycle_last - cycle_tstamp) & tc->cc->mask;
		nsec -= cc_cyc2ns_backwards(tc->cc, delta, tc->mask, frac);
	} else {
		nsec += cyclecounter_cyc2ns(tc->cc, delta, tc->mask, &frac);
	}

	return nsec;
}
