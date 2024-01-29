/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2010-2011 Monthadar Al Jaberi, TerraNet AB
 * All rights reserved.
 *
 * Copyright (c) 2002-2009 Sam Leffler, Errno Consulting
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
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT, MERCHANTIBILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGES.
 */

/*
 * Ioctl-related defintions for the Wireless TAP
 * based on Atheros Wireless LAN controller driver.
 */

#ifndef _DEV_WTAP_WTAPIOCTL_H
#define _DEV_WTAP_WTAPIOCTL_H

#include <sys/param.h>
#include <net80211/ieee80211_radiotap.h>

#define	SIOCGATHSTATS	_IOWR('i', 137, struct ifreq)
#define	SIOCZATHSTATS	_IOWR('i', 139, struct ifreq)

#define WTAPIOCTLCRT	_IOW('W', 1, int)
#define WTAPIOCTLDEL	_IOW('W', 2, int)

struct wtap_stats {
	uint32_t	ast_watchdog;	/* device reset by watchdog */
	uint32_t	ast_hardware;	/* fatal hardware error interrupts */
	uint32_t	ast_bmiss;	/* beacon miss interrupts */
	uint32_t	ast_bmiss_phantom;/* beacon miss interrupts */
	uint32_t	ast_bstuck;	/* beacon stuck interrupts */
	uint32_t	ast_rxorn;	/* rx overrun interrupts */
	uint32_t	ast_rxeol;	/* rx eol interrupts */
	uint32_t	ast_txurn;	/* tx underrun interrupts */
	uint32_t	ast_mib;	/* mib interrupts */
	uint32_t	ast_intrcoal;	/* interrupts coalesced */
	uint32_t	ast_tx_packets;	/* packet sent on the interface */
	uint32_t	ast_tx_mgmt;	/* management frames transmitted */
	uint32_t	ast_tx_discard;	/* frames discarded prior to assoc */
	uint32_t	ast_tx_qstop;	/* output stopped 'cuz no buffer */
	uint32_t	ast_tx_encap;	/* tx encapsulation failed */
	uint32_t	ast_tx_nonode;	/* tx failed 'cuz no node */
	uint32_t	ast_tx_nombuf;	/* tx failed 'cuz no mbuf */
	uint32_t	ast_tx_nomcl;	/* tx failed 'cuz no cluster */
	uint32_t	ast_tx_linear;	/* tx linearized to cluster */
	uint32_t	ast_tx_nodata;	/* tx discarded empty frame */
	uint32_t	ast_tx_busdma;	/* tx failed for dma resrcs */
	uint32_t	ast_tx_xretries;/* tx failed 'cuz too many retries */
	uint32_t	ast_tx_fifoerr;	/* tx failed 'cuz FIFO underrun */
	uint32_t	ast_tx_filtered;/* tx failed 'cuz xmit filtered */
	uint32_t	ast_tx_shortretry;/* tx on-chip retries (short) */
	uint32_t	ast_tx_longretry;/* tx on-chip retries (long) */
	uint32_t	ast_tx_badrate;	/* tx failed 'cuz bogus xmit rate */
	uint32_t	ast_tx_noack;	/* tx frames with no ack marked */
	uint32_t	ast_tx_rts;	/* tx frames with rts enabled */
	uint32_t	ast_tx_cts;	/* tx frames with cts enabled */
	uint32_t	ast_tx_shortpre;/* tx frames with short preamble */
	uint32_t	ast_tx_altrate;	/* tx frames with alternate rate */
	uint32_t	ast_tx_protect;	/* tx frames with protection */
	uint32_t	ast_tx_ctsburst;/* tx frames with cts and bursting */
	uint32_t	ast_tx_ctsext;	/* tx frames with cts extension */
	uint32_t	ast_rx_nombuf;	/* rx setup failed 'cuz no mbuf */
	uint32_t	ast_rx_busdma;	/* rx setup failed for dma resrcs */
	uint32_t	ast_rx_orn;	/* rx failed 'cuz of desc overrun */
	uint32_t	ast_rx_crcerr;	/* rx failed 'cuz of bad CRC */
	uint32_t	ast_rx_fifoerr;	/* rx failed 'cuz of FIFO overrun */
	uint32_t	ast_rx_badcrypt;/* rx failed 'cuz decryption */
	uint32_t	ast_rx_badmic;	/* rx failed 'cuz MIC failure */
	uint32_t	ast_rx_phyerr;	/* rx failed 'cuz of PHY err */
	uint32_t	ast_rx_phy[32];	/* rx PHY error per-code counts */
	uint32_t	ast_rx_tooshort;/* rx discarded 'cuz frame too short */
	uint32_t	ast_rx_toobig;	/* rx discarded 'cuz frame too large */
	uint32_t	ast_rx_packets;	/* packet recv on the interface */
	uint32_t	ast_rx_mgt;	/* management frames received */
	uint32_t	ast_rx_ctl;	/* rx discarded 'cuz ctl frame */
	int8_t		ast_tx_rssi;	/* tx rssi of last ack */
	int8_t		ast_rx_rssi;	/* rx rssi from histogram */
	uint8_t	ast_tx_rate;	/* IEEE rate of last unicast tx */
	uint32_t	ast_be_xmit;	/* beacons transmitted */
	uint32_t	ast_be_nombuf;	/* beacon setup failed 'cuz no mbuf */
	uint32_t	ast_per_cal;	/* periodic calibration calls */
	uint32_t	ast_per_calfail;/* periodic calibration failed */
	uint32_t	ast_per_rfgain;	/* periodic calibration rfgain reset */
	uint32_t	ast_rate_calls;	/* rate control checks */
	uint32_t	ast_rate_raise;	/* rate control raised xmit rate */
	uint32_t	ast_rate_drop;	/* rate control dropped xmit rate */
	uint32_t	ast_ant_defswitch;/* rx/default antenna switches */
	uint32_t	ast_ant_txswitch;/* tx antenna switches */
	uint32_t	ast_ant_rx[8];	/* rx frames with antenna */
	uint32_t	ast_ant_tx[8];	/* tx frames with antenna */
	uint32_t	ast_cabq_xmit;	/* cabq frames transmitted */
	uint32_t	ast_cabq_busy;	/* cabq found busy */
	uint32_t	ast_tx_raw;	/* tx frames through raw api */
	uint32_t	ast_ff_txok;	/* fast frames tx'd successfully */
	uint32_t	ast_ff_txerr;	/* fast frames tx'd w/ error */
	uint32_t	ast_ff_rx;	/* fast frames rx'd */
	uint32_t	ast_ff_flush;	/* fast frames flushed from staging q */
	uint32_t	ast_tx_qfull;	/* tx dropped 'cuz of queue limit */
	int8_t		ast_rx_noise;	/* rx noise floor */
	uint32_t	ast_tx_nobuf;	/* tx dropped 'cuz no ath buffer */
	uint32_t	ast_tdma_update;/* TDMA slot timing updates */
	uint32_t	ast_tdma_timers;/* TDMA slot update set beacon timers */
	uint32_t	ast_tdma_tsf;	/* TDMA slot update set TSF */
	uint16_t	ast_tdma_tsfadjp;/* TDMA slot adjust+ (usec, smoothed)*/
	uint16_t	ast_tdma_tsfadjm;/* TDMA slot adjust- (usec, smoothed)*/
	uint32_t	ast_tdma_ack;	/* TDMA tx failed 'cuz ACK required */
	uint32_t	ast_tx_raw_fail;/* raw tx failed 'cuz h/w down */
	uint32_t	ast_tx_nofrag;	/* tx dropped 'cuz no ath frag buffer */
	uint32_t	ast_be_missed;	/* missed beacons */
	uint32_t	ast_pad[13];
};

/*
 * Radio capture format.
 */
#define WTAP_RX_RADIOTAP_PRESENT (		\
	0)

struct wtap_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;
#if 0
	uint64_t	wr_tsf;
	uint8_t	wr_flags;
	uint8_t	wr_rate;
	int8_t		wr_antsignal;
	int8_t		wr_antnoise;
	uint8_t	wr_antenna;
	uint8_t	wr_pad[3];
	uint32_t	wr_chan_flags;
	uint16_t	wr_chan_freq;
	uint8_t	wr_chan_ieee;
	int8_t		wr_chan_maxpow;
#endif
} __packed __aligned(8);

#define WTAP_TX_RADIOTAP_PRESENT (		\
	0)

struct wtap_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
#if 0
	uint8_t	wt_flags;
	uint8_t	wt_rate;
	uint8_t	wt_txpower;
	uint8_t	wt_antenna;
	uint32_t	wt_chan_flags;
	uint16_t	wt_chan_freq;
	uint8_t	wt_chan_ieee;
	int8_t		wt_chan_maxpow;
#endif
} __packed;

#endif
