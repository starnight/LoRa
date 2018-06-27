/*-
 * Copyright (c) 2018 Jian-Hong, Pan <starnight@g.ncu.edu.tw>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    similar to the "NO WARRANTY" disclaimer below ("Disclaimer") and any
 *    redistribution must be conditioned upon including a substantially
 *    similar Disclaimer requirement for further binary redistribution.
 * 3. Neither the names of the above-listed copyright holders nor the names
 *    of any contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * Alternatively, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") version 2 as published by the Free
 * Software Foundation.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 *
 */

#include <linux/lora.h>
#include "lrwreg.h"

static struct lrw_dr us902_928_drs[] = {
	{.sf = 10, .bw = 125000, .mode = LRW_LORA},
	{.sf = 9, .bw = 125000, .mode = LRW_LORA},
	{.sf = 8, .bw = 125000, .mode = LRW_LORA},
	{.sf = 7, .bw = 125000, .mode = LRW_LORA},
	{.sf = 8, .bw = 500000, .mode = LRW_LORA},
	{},
};

static struct lrw_tx_power us902_928_txpws[] = {
	{.dbm = 30},
	{.dbm = 28},
	{.dbm = 26},
	{.dbm = 24},
	{.dbm = 22},
	{.dbm = 20},
	{.dbm = 18},
	{.dbm = 16},
	{.dbm = 14},
	{.dbm = 12},
	{.dbm = 10},
	{},
};

static struct lrw_payload_len us902_928_pl_fopt[] = {
	{.m = 19, .n = 11},
	{.m = 61, .n = 53},
	{.m = 133, .n = 125},
	{.m = 250, .n = 242},
	{.m = 250, .n = 242},
	{},
};

static struct lrw_payload_len us902_928_pl_nofopt[] = {
	{.m = 19, .n = 11},
	{.m = 61, .n = 53},
	{.m = 133, .n = 125},
	{.m = 250, .n = 242},
	{.m = 250, .n = 242},
	{},
};

u32 us902_928_ch2frq(u8 dir, u8 ch)
{
	u32 frq_base;
	u32 frq;
	u32 step;

	if ((dir == LRW_UPLINK) && (ch < 63)) {
		frq_base = 902300000;
		step = 200000;
		frq = frq_base + step * ch;
	}
	else if (dir == LRW_UPLINK) {
		ch = min(ch, (u8)71);
		frq_base = 903000000;
		step = 1600000;
		frq = frq_base + step * (ch - 64);
	}
	else {
		ch = min(ch, (u8)7);
		frq_base = 923300000;
		step = 600000;
		frq = frq_base + step * ch;
	}

	return frq;
}

static struct lrw_dr as923_drs[] = {
	{.sf = 12, .bw = 125000, .mode = LRW_LORA},
	{.sf = 11, .bw = 125000, .mode = LRW_LORA},
	{.sf = 10, .bw = 125000, .mode = LRW_LORA},
	{.sf = 9, .bw = 125000, .mode = LRW_LORA},
	{.sf = 8, .bw = 125000, .mode = LRW_LORA},
	{.sf = 7, .bw = 125000, .mode = LRW_LORA},
	{},
};

static struct lrw_tx_power as923_txpws[] = {
	{.dbm = 16},
	{.dbm = 14},
	{.dbm = 12},
	{.dbm = 10},
	{.dbm = 8},
	{.dbm = 6},
	{.dbm = 4},
	{.dbm = 2},
	{},
};

static struct lrw_payload_len as923_pl_fopt[] = {
	{.m = 59, .n = 51},
	{.m = 59, .n = 51},
	{.m = 59, .n = 51},
	{.m = 123, .n = 115},
	{.m = 230, .n = 222},
	{.m = 230, .n = 222},
	{},
};

static struct lrw_payload_len as923_pl_nofopt[] = {
	{.m = 59, .n = 51},
	{.m = 59, .n = 51},
	{.m = 59, .n = 51},
	{.m = 123, .n = 115},
	{.m = 250, .n = 242},
	{.m = 250, .n = 242},
	{},
};

u32 as923_ch2frq(u8 dir, u8 ch)
{
	u32 frq_base;
	u32 frq;
	u32 step;

	ch = min(ch, (u8)1);
	frq_base = 923200000;
	step = 200000;
	frq = frq_base + step * ch;

	return frq;
}

static const struct lrw_region_parm reg_parms[] = {
	[LRW_EU863_870] = {},
	[LRW_US902_928] = {
		.sync_word = 0x34,
		.preamble_len = 8,
		.drt = us902_928_drs,
		.pwt = us902_928_txpws,
		.plt[0] = us902_928_pl_fopt,
		.plt[1] = us902_928_pl_nofopt,
		.ch_2_frq = us902_928_ch2frq,
		.rx_delay1 = HZ,
		.rx_delay2 = 2 * HZ,
		.join_accept_delay1 = 5 * HZ,
		.join_accept_delay2 = 6 * HZ,
		.ack_timeout = 2 * HZ,
	},
	[LRW_CN779_787] = {},
	[LRW_EU443] = {},
	[LRW_AU915_928] = {},
	[LRW_CN470_510] = {},
	[LRW_AS923] = {
		.sync_word = 0x34,
		.preamble_len = 8,
		.drt = as923_drs,
		.pwt = as923_txpws,
		.plt[0] = as923_pl_fopt,
		.plt[1] = as923_pl_nofopt,
		.ch_2_frq = as923_ch2frq,
		.rx_delay1 = HZ,
		.rx_delay2 = 2 * HZ,
		.join_accept_delay1 = 5 * HZ,
		.join_accept_delay2 = 6 * HZ,
		.ack_timeout = 2 * HZ,
	},
	[LRW_KR920_923] = {},
	[LRW_INDIA865_867] = {},
};

struct lrw_region_parm *
lrw_get_reg_parm(u8 region)
{
	struct lrw_region_parm *ret;

	if (region < LRW_MAX_REGION)
		ret = (struct lrw_region_parm *)&reg_parms[region];
	else
		ret = NULL;

	return ret;
}
