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

static void ts2phc_synchronize_clocks(struct ts2phc_private *priv)
{
	struct timespec source_ts;
	tmv_t source_tmv;
	struct clock *c;
	int valid, err;

	err = ts2phc_master_getppstime(priv->master, &source_ts);
	if (err < 0) {
		pr_err("source ts not valid");
		return;
	}
	if (source_ts.tv_nsec > NS_PER_SEC / 2)
		source_ts.tv_sec++;
	source_ts.tv_nsec = 0;

	source_tmv = timespec_to_tmv(source_ts);

	LIST_FOREACH(c, &priv->clocks, list) {
		int64_t offset;
		double adj;
		tmv_t ts;

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
		if (1 == config_get_int(cfg, interface_name(iface), "ts2phc.master")) {
			if (pps_source) {
				fprintf(stderr, "too many PPS sources\n");
				ts2phc_cleanup(&priv);
				return -1;
			}
			pps_source = interface_name(iface);
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

		LIST_FOREACH(c, &priv.clocks, list)
			clock_flush_tstamp(c);

		err = ts2phc_slave_poll(&priv);
		if (err < 0) {
			pr_err("poll failed");
			break;
		}
		if (err > 0)
			ts2phc_synchronize_clocks(&priv);
	}

	ts2phc_cleanup(&priv);
	return err;
}
