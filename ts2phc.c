/**
 * @file ts2phc.c
 * @brief Utility program to synchronize the PHC clock to external events
 * @note Copyright (C) 2013 Balint Ferencz <fernya@sch.bme.hu>
 * @note Based on the phc2sys utility
 * @note Copyright (C) 2012 Richard Cochran <richardcochran@gmail.com>
 * @note SPDX-License-Identifier: GPL-2.0+
 */
#include <stdlib.h>
#include <net/if.h>
#include <sys/types.h>
#include <unistd.h>

#include "clockadj.h"
#include "config.h"
#include "contain.h"
#include "interface.h"
#include "phc.h"
#include "print.h"
#include "ts2phc.h"
#include "version.h"

#define NS_PER_SEC		1000000000LL
#define SAMPLE_WEIGHT		1.0

struct interface {
	STAILQ_ENTRY(interface) list;
};

static void ts2phc_cleanup(struct ts2phc_private *priv)
{
	struct port *p, *tmp;

	ts2phc_slave_cleanup(priv);
	if (priv->master)
		ts2phc_master_destroy(priv->master);
	if (priv->cfg)
		config_destroy(priv->cfg);

	close_pmc_node(&priv->node);

	/*
	 * Clocks are destroyed by the cleanup methods of the individual
	 * master and slave PHC modules.
	 */
	LIST_FOREACH_SAFE(p, &priv->ports, list, tmp)
		free(p);

	msg_cleanup();
}

/* FIXME: Copied from phc2sys */
static int normalize_state(int state)
{
	if (state != PS_MASTER && state != PS_SLAVE &&
	    state != PS_PRE_MASTER && state != PS_UNCALIBRATED) {
		/* treat any other state as "not a master nor a slave" */
		state = PS_DISABLED;
	}
	return state;
}

/* FIXME: Copied from phc2sys */
static struct port *port_get(struct ts2phc_private *priv, unsigned int number)
{
	struct port *p;

	LIST_FOREACH(p, &priv->ports, list) {
		if (p->number == number)
			return p;
	}
	return NULL;
}

/* FIXME: Copied from phc2sys */
static int clock_compute_state(struct ts2phc_private *priv,
			       struct clock *clock)
{
	int state = PS_DISABLED;
	struct port *p;

	LIST_FOREACH(p, &priv->ports, list) {
		if (p->clock != clock)
			continue;
		/* PS_SLAVE takes the highest precedence, PS_UNCALIBRATED
		 * after that, PS_MASTER is third, PS_PRE_MASTER fourth and
		 * all of that overrides PS_DISABLED, which corresponds
		 * nicely with the numerical values */
		if (p->state > state)
			state = p->state;
	}
	return state;
}

#define node_to_ts2phc(node) \
	container_of(node, struct ts2phc_private, node)

static int ts2phc_recv_subscribed(struct pmc_node *node,
				  struct ptp_message *msg, int excluded)
{
	struct ts2phc_private *priv = node_to_ts2phc(node);
	int mgt_id, state;
	struct portDS *pds;
	struct port *port;
	struct clock *clock;

	mgt_id = get_mgt_id(msg);
	if (mgt_id == excluded)
		return 0;
	switch (mgt_id) {
	case TLV_PORT_DATA_SET:
		pds = get_mgt_data(msg);
		port = port_get(priv, pds->portIdentity.portNumber);
		if (!port) {
			pr_info("received data for unknown port %s",
				pid2str(&pds->portIdentity));
			return 1;
		}
		state = normalize_state(pds->portState);
		if (port->state != state) {
			pr_info("port %s changed state",
				pid2str(&pds->portIdentity));
			port->state = state;
			clock = port->clock;
			state = clock_compute_state(priv, clock);
			if (clock->state != state || clock->new_state) {
				clock->new_state = state;
				priv->state_changed = 1;
			}
		}
		return 1;
	}
	return 0;
}

struct servo *servo_add(struct ts2phc_private *priv, struct clock *clock)
{
	enum servo_type type = config_get_int(priv->cfg, NULL, "clock_servo");
	struct servo *servo;
	int fadj, max_adj;

	fadj = (int) clockadj_get_freq(clock->clkid);
	/* Due to a bug in older kernels, the reading may silently fail
	   and return 0. Set the frequency back to make sure fadj is
	   the actual frequency of the clock. */
	clockadj_set_freq(clock->clkid, fadj);

	max_adj = phc_max_adj(clock->clkid);

	servo = servo_create(priv->cfg, type, -fadj, max_adj, 0);
	if (!servo)
		return NULL;

	servo_sync_interval(servo, SERVO_SYNC_INTERVAL);

	return servo;
}

void clock_add_tstamp(struct clock *clock, tmv_t t)
{
	struct timespec ts = tmv_to_timespec(t);

	pr_debug("adding tstamp %ld.%09ld to clock %s",
		 ts.tv_sec, ts.tv_nsec, clock->name);
	clock->last_ts = t;
	clock->is_ts_available = 1;
}

static int clock_get_tstamp(struct clock *clock, tmv_t *ts)
{
	if (!clock->is_ts_available)
		return 0;
	clock->is_ts_available = 0;
	*ts = clock->last_ts;
	return 1;
}

static void clock_flush_tstamp(struct clock *clock)
{
	clock->is_ts_available = 0;
}

struct clock *clock_add(struct ts2phc_private *priv, const char *device)
{
	clockid_t clkid = CLOCK_INVALID;
	int phc_index = -1;
	struct clock *c;
	int err;

	clkid = posix_clock_open(device, &phc_index);
	if (clkid == CLOCK_INVALID)
		return NULL;

	LIST_FOREACH(c, &priv->clocks, list) {
		if (c->phc_index == phc_index) {
			/* Already have the clock, don't add it again */
			posix_clock_close(clkid);
			return c;
		}
	}

	c = calloc(1, sizeof(*c));
	if (!c) {
		pr_err("failed to allocate memory for a clock");
		return NULL;
	}
	c->clkid = clkid;
	c->phc_index = phc_index;
	c->servo_state = SERVO_UNLOCKED;
	c->servo = servo_add(priv, c);
	c->no_adj = config_get_int(priv->cfg, NULL, "free_running");
	err = asprintf(&c->name, "/dev/ptp%d", phc_index);
	if (err < 0) {
		free(c);
		posix_clock_close(clkid);
		return NULL;
	}

	LIST_INSERT_HEAD(&priv->clocks, c, list);
	return c;
}


void clock_destroy(struct clock *c)
{
	servo_destroy(c->servo);
	posix_clock_close(c->clkid);
	free(c->name);
	free(c);
}

/* FIXME: Copied from phc2sys */
static struct port *port_add(struct ts2phc_private *priv, unsigned int number,
			     char *device)
{
	struct clock *c = NULL;
	struct port *p, *tmp;

	p = port_get(priv, number);
	if (p)
		return p;
	/* port is a new one, look whether we have the device already on
	 * a different port */
	LIST_FOREACH(tmp, &priv->ports, list) {
		if (tmp->number == number) {
			c = tmp->clock;
			break;
		}
	}
	if (!c) {
		c = clock_add(priv, device);
		if (!c)
			return NULL;
	}
	p = malloc(sizeof(*p));
	if (!p) {
		pr_err("failed to allocate memory for a port");
		clock_destroy(c);
		return NULL;
	}
	p->number = number;
	p->clock = c;
	LIST_INSERT_HEAD(&priv->ports, p, list);
	return p;
}

static int auto_init_ports(struct ts2phc_private *priv)
{
	int state, timestamping;
	int number_ports, res;
	char iface[IFNAMSIZ];
	struct clock *clock;
	struct port *port;
	unsigned int i;

	while (1) {
		if (!is_running())
			return -1;
		res = run_pmc_clock_identity(&priv->node, 1000);
		if (res < 0)
			return -1;
		if (res > 0)
			break;
		/* res == 0, timeout */
		pr_notice("Waiting for ptp4l...");
	}

	number_ports = run_pmc_get_number_ports(&priv->node, 1000);
	if (number_ports <= 0) {
		pr_err("failed to get number of ports");
		return -1;
	}

	res = run_pmc_subscribe(&priv->node, 1000);
	if (res <= 0) {
		pr_err("failed to subscribe");
		return -1;
	}

	for (i = 1; i <= number_ports; i++) {
		res = run_pmc_port_properties(&priv->node, 1000, i, &state,
					      &timestamping, iface);
		if (res == -1) {
			/* port does not exist, ignore the port */
			continue;
		}
		if (res <= 0) {
			pr_err("failed to get port properties");
			return -1;
		}
		if (timestamping == TS_SOFTWARE) {
			/* ignore ports with software time stamping */
			continue;
		}
		port = port_add(priv, i, iface);
		if (!port)
			return -1;
		port->state = normalize_state(state);
	}
	if (LIST_EMPTY(&priv->clocks)) {
		pr_err("no suitable ports available");
		return -1;
	}
	LIST_FOREACH(clock, &priv->clocks, list) {
		clock->new_state = clock_compute_state(priv, clock);
	}
	priv->state_changed = 1;

	return 0;
}

static void ts2phc_reconfigure(struct ts2phc_private *priv)
{
	struct clock *c, *src = NULL, *last = NULL;
	int src_cnt = 0, dst_cnt = 0;

	pr_info("reconfiguring after port state change");
	priv->state_changed = 0;

	LIST_FOREACH(c, &priv->clocks, list) {
		if (c->new_state) {
			c->state = c->new_state;
			c->new_state = 0;
		}

		switch (c->state) {
		case PS_FAULTY:
		case PS_DISABLED:
		case PS_LISTENING:
		case PS_PRE_MASTER:
		case PS_MASTER:
		case PS_PASSIVE:
			if (!c->is_destination) {
				pr_info("selecting %s for synchronization",
					c->name);
				c->is_destination = 1;
			}
			dst_cnt++;
			break;
		case PS_UNCALIBRATED:
			src_cnt++;
			break;
		case PS_SLAVE:
			src = c;
			src_cnt++;
			break;
		}
		last = c;
	}
	if (dst_cnt >= 1 && !src) {
		priv->source = last;
		priv->source->is_destination = 0;
		/* Reset to original state in next reconfiguration. */
		priv->source->new_state = priv->source->state;
		priv->source->state = PS_SLAVE;
		pr_info("no source, selecting %s as the default clock",
			last->name);
		return;
	}
	if (src_cnt > 1) {
		pr_info("multiple source clocks available, postponing sync...");
		priv->source = NULL;
		return;
	}
	if (src_cnt > 0 && !src) {
		pr_info("source clock not ready, waiting...");
		priv->source = NULL;
		return;
	}
	if (!src_cnt && !dst_cnt) {
		pr_info("no PHC ready, waiting...");
		priv->source = NULL;
		return;
	}
	if (!src_cnt) {
		pr_info("nothing to synchronize");
		priv->source = NULL;
		return;
	}
	src->is_destination = 0;
	priv->source = src;
	pr_info("selecting %s as the source clock", src->name);
}

static int ts2phc_approximate_master_tstamp(struct ts2phc_private *priv,
					    tmv_t *master_tmv)
{
	struct timespec master_ts;
	tmv_t tmv;
	int err;

	err = ts2phc_master_getppstime(priv->master, &master_ts);
	if (err < 0) {
		pr_err("master ts not valid");
		return err;
	}

	tmv = timespec_to_tmv(master_ts);
	tmv = tmv_sub(tmv, priv->perout_phase);
	master_ts = tmv_to_timespec(tmv);

	/*
	 * As long as the kernel doesn't support a proper API for reporting
	 * a precise perout timestamp, we'll have to use this crude
	 * approximation.
	 */
	if (master_ts.tv_nsec > NS_PER_SEC / 2)
		master_ts.tv_sec++;
	master_ts.tv_nsec = 0;

	tmv = timespec_to_tmv(master_ts);
	tmv = tmv_add(tmv, priv->perout_phase);

	*master_tmv = tmv;

	return 0;
}

static void ts2phc_synchronize_clocks(struct ts2phc_private *priv, int autocfg)
{
	tmv_t source_tmv;
	struct clock *c;
	int valid, err;

	if (autocfg) {
		if (!priv->source) {
			pr_debug("no source, skipping");
			return;
		}
		valid = clock_get_tstamp(priv->source, &source_tmv);
		if (!valid) {
			pr_err("source clock (%s) timestamp not valid, skipping",
				priv->source->name);
			return;
		}
	} else {
		err = ts2phc_approximate_master_tstamp(priv, &source_tmv);
		if (err < 0)
			return;
	}

	LIST_FOREACH(c, &priv->clocks, list) {
		int64_t offset;
		double adj;
		tmv_t ts;

		if (!c->is_destination)
			continue;

		valid = clock_get_tstamp(c, &ts);
		if (!valid) {
			pr_debug("%s timestamp not valid, skipping", c->name);
			continue;
		}

		offset = tmv_to_nanoseconds(tmv_sub(ts, source_tmv));

		if (c->no_adj) {
			pr_info("%s offset %10" PRId64, c->name,
				offset);
			continue;
		}

		adj = servo_sample(c->servo, offset, tmv_to_nanoseconds(ts),
				   SAMPLE_WEIGHT, &c->servo_state);

		pr_info("%s offset %10" PRId64 " s%d freq %+7.0f",
			c->name, offset, c->servo_state, adj);

		switch (c->servo_state) {
		case SERVO_UNLOCKED:
			break;
		case SERVO_JUMP:
			clockadj_set_freq(c->clkid, -adj);
			clockadj_step(c->clkid, -offset);
			break;
		case SERVO_LOCKED:
		case SERVO_LOCKED_STABLE:
			clockadj_set_freq(c->clkid, -adj);
			break;
		}
	}
}

static int ts2phc_collect_master_tstamp(struct ts2phc_private *priv)
{
	struct clock *master_clock;
	tmv_t master_tmv;
	int err;

	master_clock = ts2phc_master_get_clock(priv->master);
	/*
	 * Master isn't a PHC (it may be a generic or a GPS master),
	 * don't error out, just don't do anything. If it doesn't have a PHC,
	 * there is nothing to synchronize, which is the only point of
	 * collecting its perout timestamp in the first place.
	 */
	if (!master_clock)
		return 0;

	err = ts2phc_approximate_master_tstamp(priv, &master_tmv);
	if (err < 0)
		return err;

	clock_add_tstamp(master_clock, master_tmv);

	return 0;
}

static void usage(char *progname)
{
	fprintf(stderr,
		"\n"
		"usage: %s [options]\n\n"
		" -a             turn on autoconfiguration\n"
		" -c [dev|name]  phc slave clock (like /dev/ptp0 or eth0)\n"
		"                (may be specified multiple times)\n"
		" -f [file]      read configuration from 'file'\n"
		" -h             prints this message and exits\n"
		" -l [num]       set the logging level to 'num'\n"
		" -m             print messages to stdout\n"
		" -q             do not print messages to the syslog\n"
		" -s [dev|name]  source of the PPS signal\n"
		"                may take any of the following forms:\n"
		"                    generic   - an external 1-PPS without ToD information\n"
		"                    /dev/ptp0 - a local PTP Hardware Clock (PHC)\n"
		"                    eth0      - a local PTP Hardware Clock (PHC)\n"
		"                    nmea      - a gps device connected by serial port or network\n"
		" -v             prints the software version and exits\n"
		"\n",
		progname);
}

int main(int argc, char *argv[])
{
	int c, err = 0, have_slave = 0, index, print_level;
	char uds_local[MAX_IFNAME_SIZE + 1];
	enum ts2phc_master_type pps_type;
	struct ts2phc_private priv = {0};
	char *config = NULL, *progname;
	const char *pps_source = NULL;
	struct config *cfg = NULL;
	struct interface *iface;
	struct option *opts;
	int autocfg = 0;

	handle_term_signals();

	cfg = config_create();
	if (!cfg) {
		ts2phc_cleanup(&priv);
		return -1;
	}

	opts = config_long_options(cfg);

	/* Process the command line arguments. */
	progname = strrchr(argv[0], '/');
	progname = progname ? 1 + progname : argv[0];
	while (EOF != (c = getopt_long(argc, argv, "ac:f:hi:l:mqs:v", opts, &index))) {
		switch (c) {
		case 0:
			if (config_parse_option(cfg, opts[index].name, optarg)) {
				ts2phc_cleanup(&priv);
				return -1;
			}
			break;
		case 'a':
			autocfg = 1;
			break;
		case 'c':
			if (!config_create_interface(optarg, cfg)) {
				fprintf(stderr, "failed to add slave\n");
				ts2phc_cleanup(&priv);
				return -1;
			}
			have_slave = 1;
			break;
		case 'f':
			config = optarg;
			break;
		case 'l':
			if (get_arg_val_i(c, optarg, &print_level,
					  PRINT_LEVEL_MIN, PRINT_LEVEL_MAX)) {
				ts2phc_cleanup(&priv);
				return -1;
			}
			config_set_int(cfg, "logging_level", print_level);
			print_set_level(print_level);
			break;
		case 'm':
			config_set_int(cfg, "verbose", 1);
			print_set_verbose(1);
			break;
		case 'q':
			config_set_int(cfg, "use_syslog", 0);
			print_set_syslog(0);
			break;
		case 's':
			if (pps_source) {
				fprintf(stderr, "too many PPS sources\n");
				ts2phc_cleanup(&priv);
				return -1;
			}
			pps_source = optarg;
			break;
		case 'v':
			ts2phc_cleanup(&priv);
			version_show(stdout);
			return 0;
		case 'h':
			ts2phc_cleanup(&priv);
			usage(progname);
			return -1;
		case '?':
		default:
			ts2phc_cleanup(&priv);
			usage(progname);
			return -1;
		}
	}
	if (config && (c = config_read(config, cfg))) {
		fprintf(stderr, "failed to read config\n");
		ts2phc_cleanup(&priv);
		return -1;
	}
	print_set_progname(progname);
	print_set_tag(config_get_string(cfg, NULL, "message_tag"));
	print_set_verbose(config_get_int(cfg, NULL, "verbose"));
	print_set_syslog(config_get_int(cfg, NULL, "use_syslog"));
	print_set_level(config_get_int(cfg, NULL, "logging_level"));

	STAILQ_INIT(&priv.slaves);
	priv.cfg = cfg;

	snprintf(uds_local, sizeof(uds_local), "/var/run/ts2phc.%d",
		 getpid());

	if (autocfg) {
		err = init_pmc_node(cfg, &priv.node, uds_local,
				    ts2phc_recv_subscribed);
		if (err) {
			ts2phc_cleanup(&priv);
			return -1;
		}
		err = auto_init_ports(&priv);
		if (err) {
			ts2phc_cleanup(&priv);
			return -1;
		}
	}

	STAILQ_FOREACH(iface, &cfg->interfaces, list) {
		const char *dev = interface_name(iface);

		if (1 == config_get_int(cfg, dev, "ts2phc.master")) {
			int perout_phase;

			if (pps_source) {
				fprintf(stderr, "too many PPS sources\n");
				ts2phc_cleanup(&priv);
				return -1;
			}
			pps_source = dev;
			perout_phase = config_get_int(cfg, dev,
						      "ts2phc.perout_phase");
			/*
			 * We use a default value of -1 to distinguish whether
			 * to use the PTP_PEROUT_PHASE API or not. But if we
			 * don't use that (and therefore we use absolute start
			 * time), the phase is still zero, by our application's
			 * convention.
			 */
			if (perout_phase < 0)
				perout_phase = 0;
			priv.perout_phase = nanoseconds_to_tmv(perout_phase);
		} else {
			if (ts2phc_slave_add(&priv, interface_name(iface))) {
				fprintf(stderr, "failed to add slave\n");
				ts2phc_cleanup(&priv);
				return -1;
			}
			have_slave = 1;
		}
	}
	if (!have_slave) {
		fprintf(stderr, "no slave clocks specified\n");
		ts2phc_cleanup(&priv);
		usage(progname);
		return -1;
	}
	if (!pps_source) {
		fprintf(stderr, "no PPS source specified\n");
		ts2phc_cleanup(&priv);
		usage(progname);
		return -1;
	}
	if (ts2phc_slaves_init(&priv)) {
		fprintf(stderr, "failed to initialize slaves\n");
		ts2phc_cleanup(&priv);
		return -1;
	}

	if (!strcasecmp(pps_source, "generic")) {
		pps_type = TS2PHC_MASTER_GENERIC;
	} else if (!strcasecmp(pps_source, "nmea")) {
		pps_type = TS2PHC_MASTER_NMEA;
	} else {
		pps_type = TS2PHC_MASTER_PHC;
	}
	priv.master = ts2phc_master_create(&priv, pps_source, pps_type);
	if (!priv.master) {
		fprintf(stderr, "failed to create master\n");
		ts2phc_cleanup(&priv);
		return -1;
	}

	while (is_running()) {
		struct clock *c;

		if (autocfg) {
			/*
			 * Make sure ptp4l sees us as alive and doesn't prune
			 * us from the list of subscribers
			 */
			err = update_pmc_node(&priv.node, 1);
			if (err < 0) {
				pr_err("update_pmc_node returned %d", err);
				break;
			}
			run_pmc_events(&priv.node);
			if (priv.state_changed)
				ts2phc_reconfigure(&priv);
		}

		LIST_FOREACH(c, &priv.clocks, list)
			clock_flush_tstamp(c);

		err = ts2phc_slave_poll(&priv);
		if (err < 0) {
			pr_err("poll failed");
			break;
		}
		if (err > 0) {
			err = ts2phc_collect_master_tstamp(&priv);
			if (err) {
				pr_err("failed to collect master tstamp");
				break;
			}

			ts2phc_synchronize_clocks(&priv, autocfg);
		}
	}

	ts2phc_cleanup(&priv);
	return err;
}
