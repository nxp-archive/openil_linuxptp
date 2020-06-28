/**
 * @file ts2phc_slave.h
 * @brief Utility program to synchronize the PHC clock to external events
 * @note Copyright (C) 2019 Balint Ferencz <fernya@sch.bme.hu>
 * @note SPDX-License-Identifier: GPL-2.0+
 */
#ifndef HAVE_TS2PHC_SLAVE_H
#define HAVE_TS2PHC_SLAVE_H

#include "ts2phc.h"

int ts2phc_slave_add(struct ts2phc_private *priv, const char *name);

int ts2phc_slaves_init(struct ts2phc_private *priv);

void ts2phc_slave_cleanup(struct ts2phc_private *priv);

int ts2phc_slave_poll(struct ts2phc_private *priv);

#endif
