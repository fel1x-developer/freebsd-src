/*-
 * SPDX-License-Identifier: BSD-2-Clause
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
 * Ioctl-related defintions for the Atheros Wireless LAN controller driver.
 */
#ifndef _DEV_ATH_ATHIOCTL_H
#define _DEV_ATH_ATHIOCTL_H

struct ath_tx_aggr_stats {
	uint32_t	aggr_pkts[64];
	uint32_t	aggr_single_pkt;
	uint32_t	aggr_nonbaw_pkt;
	uint32_t	aggr_aggr_pkt;
	uint32_t	aggr_baw_closed_single_pkt;
	uint32_t	aggr_low_hwq_single_pkt;
	uint32_t	aggr_sched_nopkt;
	uint32_t	aggr_rts_aggr_limited;
};

#define	ATH_IOCTL_INTR_NUM_SYNC_INTR		32
struct ath_intr_stats {
	uint32_t	sync_intr[ATH_IOCTL_INTR_NUM_SYNC_INTR];
};

#define	ATH_IOCTL_STATS_NUM_RX_PHYERR		64
#define	ATH_IOCTL_STATS_NUM_TX_ANTENNA		8
#define	ATH_IOCTL_STATS_NUM_RX_ANTENNA		8
struct ath_stats {
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
	uint32_t	ast_rx_phy[ATH_IOCTL_STATS_NUM_RX_PHYERR];
					/* rx PHY error per-code counts */
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
	uint32_t	ast_ant_rx[ATH_IOCTL_STATS_NUM_RX_ANTENNA];
					/* rx frames with antenna */
	uint32_t	ast_ant_tx[ATH_IOCTL_STATS_NUM_TX_ANTENNA];
					/* tx frames with antenna */
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
	uint32_t	ast_ani_cal;	/* ANI calibrations performed */
	uint32_t	ast_rx_agg;	/* number of aggregate frames RX'ed */
	uint32_t	ast_rx_halfgi;	/* RX half-GI */
	uint32_t	ast_rx_2040;	/* RX 40mhz frame */
	uint32_t	ast_rx_pre_crc_err;	/* RX pre-delimiter CRC error */
	uint32_t	ast_rx_post_crc_err;	/* RX post-delimiter CRC error */
	uint32_t	ast_rx_decrypt_busy_err;	/* RX decrypt engine busy error */
	uint32_t	ast_rx_hi_rx_chain;
	uint32_t	ast_tx_htprotect;	/* HT tx frames with protection */
	uint32_t	ast_rx_hitqueueend;	/* RX hit descr queue end */
	uint32_t	ast_tx_timeout;		/* Global TX timeout */
	uint32_t	ast_tx_cst;		/* Carrier sense timeout */
	uint32_t	ast_tx_xtxop;	/* tx exceeded TXOP */
	uint32_t	ast_tx_timerexpired;	/* tx exceeded TX_TIMER */
	uint32_t	ast_tx_desccfgerr;	/* tx desc cfg error */
	uint32_t	ast_tx_swretries;	/* software TX retries */
	uint32_t	ast_tx_swretrymax;	/* software TX retry max limit reach */
	uint32_t	ast_tx_data_underrun;
	uint32_t	ast_tx_delim_underrun;
	uint32_t	ast_tx_aggr_failall;	/* aggregate TX failed in its entirety */
	uint32_t	ast_tx_getnobuf;
	uint32_t	ast_tx_getbusybuf;
	uint32_t	ast_tx_intr;
	uint32_t	ast_rx_intr;
	uint32_t	ast_tx_aggr_ok;		/* aggregate TX ok */
	uint32_t	ast_tx_aggr_fail;	/* aggregate TX failed */
	uint32_t	ast_tx_mcastq_overflow;	/* multicast queue overflow */
	uint32_t	ast_rx_keymiss;
	uint32_t	ast_tx_swfiltered;
	uint32_t	ast_tx_node_psq_overflow;
	uint32_t	ast_rx_stbc;		/* RX STBC frame */
	uint32_t	ast_tx_nodeq_overflow;	/* node sw queue overflow */
	uint32_t	ast_tx_ldpc;		/* TX LDPC frame */
	uint32_t	ast_tx_stbc;		/* TX STBC frame */
	uint32_t	ast_tsfoor;		/* TSFOOR interrupts */
	uint32_t	ast_pad[10];
};

#define	SIOCGATHSTATS	_IOWR('i', 137, struct ifreq)
#define	SIOCZATHSTATS	_IOWR('i', 139, struct ifreq)
#define	SIOCGATHAGSTATS	_IOWR('i', 141, struct ifreq)

struct ath_diag {
	char	ad_name[IFNAMSIZ];	/* if name, e.g. "ath0" */
	uint16_t ad_id;
#define	ATH_DIAG_DYN	0x8000		/* allocate buffer in caller */
#define	ATH_DIAG_IN	0x4000		/* copy in parameters */
#define	ATH_DIAG_OUT	0x0000		/* copy out results (always) */
#define	ATH_DIAG_ID	0x0fff
	uint16_t ad_in_size;		/* pack to fit, yech */
	caddr_t	ad_in_data;
	caddr_t	ad_out_data;
	u_int	ad_out_size;

};
#define	SIOCGATHDIAG	_IOWR('i', 138, struct ath_diag)
#define	SIOCGATHPHYERR	_IOWR('i', 140, struct ath_diag)

/*
 * The rate control ioctl has to support multiple potential rate
 * control classes.  For now, instead of trying to support an
 * abstraction for this in the API, let's just use a TLV
 * representation for the payload and let userspace sort it out.
 */
struct ath_rateioctl_tlv {
	uint16_t	tlv_id;
	uint16_t	tlv_len;	/* length excluding TLV header */
};

/*
 * This is purely the six byte MAC address.
 */
#define	ATH_RATE_TLV_MACADDR		0xaab0

/*
 * The rate control modules may decide to push a mapping table
 * of rix -> net80211 ratecode as part of the update.
 */
#define	ATH_RATE_TLV_RATETABLE_NENTRIES	64
struct ath_rateioctl_rt {
	uint16_t	nentries;
	uint16_t	pad[1];
	uint8_t		ratecode[ATH_RATE_TLV_RATETABLE_NENTRIES];
};
#define	ATH_RATE_TLV_RATETABLE		0xaab1

/*
 * This is the sample node statistics structure.
 * More in ath_rate/sample/sample.h.
 */
#define	ATH_RATE_TLV_SAMPLENODE		0xaab2

struct ath_rateioctl {
	char	if_name[IFNAMSIZ];	/* if name */
	union {
		uint8_t		macaddr[IEEE80211_ADDR_LEN];
		uint64_t	pad;
	} is_u;
	uint32_t		len;
	caddr_t			buf;
};
#define	SIOCGATHNODERATESTATS	_IOWR('i', 149, struct ath_rateioctl)
#define	SIOCGATHRATESTATS	_IOWR('i', 150, struct ath_rateioctl)

/*
 * Radio capture format.
 */
#define ATH_RX_RADIOTAP_PRESENT_BASE (		\
	(1 << IEEE80211_RADIOTAP_TSFT)		| \
	(1 << IEEE80211_RADIOTAP_FLAGS)		| \
	(1 << IEEE80211_RADIOTAP_RATE)		| \
	(1 << IEEE80211_RADIOTAP_ANTENNA)	| \
	(1 << IEEE80211_RADIOTAP_DBM_ANTSIGNAL)	| \
	(1 << IEEE80211_RADIOTAP_DBM_ANTNOISE)	| \
	(1 << IEEE80211_RADIOTAP_XCHANNEL)	| \
	0)

#ifdef	ATH_ENABLE_RADIOTAP_VENDOR_EXT
#define	ATH_RX_RADIOTAP_PRESENT \
	(ATH_RX_RADIOTAP_PRESENT_BASE		| \
	(1 << IEEE80211_RADIOTAP_VENDOREXT)	| \
	(1 << IEEE80211_RADIOTAP_EXT)		| \
	0)
#else
#define	ATH_RX_RADIOTAP_PRESENT	ATH_RX_RADIOTAP_PRESENT_BASE
#endif	/* ATH_ENABLE_RADIOTAP_PRESENT */

#ifdef	ATH_ENABLE_RADIOTAP_VENDOR_EXT
/*
 * This is higher than the vendor bitmap used inside
 * the Atheros reference codebase.
 */

/* Bit 8 */
#define	ATH_RADIOTAP_VENDOR_HEADER	8

/*
 * Using four chains makes all the fields in the
 * per-chain info header be 4-byte aligned.
 */
#define	ATH_RADIOTAP_MAX_CHAINS		4

/*
 * AR9380 and later chips are 3x3, which requires
 * 5 EVM DWORDs in HT40 mode.
 */
#define	ATH_RADIOTAP_MAX_EVM		5

/*
 * The vendor radiotap header data needs to be:
 *
 * + Aligned to a 4 byte address
 * + .. so all internal fields are 4 bytes aligned;
 * + .. and no 64 bit fields are allowed.
 *
 * So padding is required to ensure this is the case.
 *
 * Note that because of the lack of alignment with the
 * vendor header (6 bytes), the first field must be
 * two bytes so it can be accessed by alignment-strict
 * platform (eg MIPS.)
 */
struct ath_radiotap_vendor_hdr {		/* 30 bytes */
	uint8_t		vh_version;		/* 1 */
	uint8_t		vh_rx_chainmask;	/* 1 */

	/* At this point it should be 4 byte aligned */
	uint32_t	evm[ATH_RADIOTAP_MAX_EVM];	/* 5 * 4 = 20 */

	uint8_t		rssi_ctl[ATH_RADIOTAP_MAX_CHAINS];	/* 4 * 4 = 16 */
	uint8_t		rssi_ext[ATH_RADIOTAP_MAX_CHAINS];	/* 4 * 4 = 16 */

	uint8_t		vh_phyerr_code;	/* Phy error code, or 0xff */
	uint8_t		vh_rs_status;	/* RX status */
	uint8_t		vh_rssi;	/* Raw RSSI */
	uint8_t		vh_flags;	/* General flags */
#define	ATH_VENDOR_PKT_RX	0x01
#define	ATH_VENDOR_PKT_TX	0x02
#define	ATH_VENDOR_PKT_RXPHYERR	0x04
#define	ATH_VENDOR_PKT_ISAGGR	0x08
#define	ATH_VENDOR_PKT_MOREAGGR	0x10

	uint8_t		vh_rx_hwrate;	/* hardware RX ratecode */
	uint8_t		vh_rs_flags;	/* RX HAL flags */
	uint8_t		vh_pad[2];	/* pad to DWORD boundary */
} __packed;
#endif	/* ATH_ENABLE_RADIOTAP_VENDOR_EXT */

struct ath_rx_radiotap_header {
	struct ieee80211_radiotap_header wr_ihdr;

#ifdef	ATH_ENABLE_RADIOTAP_VENDOR_EXT
	/* Vendor extension header bitmap */
	uint32_t	wr_ext_bitmap;          /* 4 */

	/*
	 * This padding is needed because:
	 * + the radiotap header is 8 bytes;
	 * + the extension bitmap is 4 bytes;
	 * + the tsf is 8 bytes, so it must start on an 8 byte
	 *   boundary.
	 */
	uint32_t	wr_pad1;
#endif	/* ATH_ENABLE_RADIOTAP_VENDOR_EXT */

	/* Normal radiotap fields */
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

#ifdef	ATH_ENABLE_RADIOTAP_VENDOR_EXT
	/*
	 * Vendor header section, as required by the
	 * presence of the vendor extension bit and bitmap
	 * entry.
	 *
	 * XXX This must be aligned to a 4 byte address?
	 * XXX or 8 byte address?
	 */
	struct ieee80211_radiotap_vendor_header wr_vh;  /* 6 bytes */

	/*
	 * Because of the lack of alignment enforced by the above
	 * header, this vendor section won't be aligned in any
	 * useful way.  So, this will include a two-byte version
	 * value which will force the structure to be 4-byte aligned.
	 */
	struct ath_radiotap_vendor_hdr wr_v;
#endif	/* ATH_ENABLE_RADIOTAP_VENDOR_EXT */
} __packed __aligned(8);

#define ATH_TX_RADIOTAP_PRESENT (		\
	(1 << IEEE80211_RADIOTAP_FLAGS)		| \
	(1 << IEEE80211_RADIOTAP_RATE)		| \
	(1 << IEEE80211_RADIOTAP_DBM_TX_POWER)	| \
	(1 << IEEE80211_RADIOTAP_ANTENNA)	| \
	(1 << IEEE80211_RADIOTAP_XCHANNEL)	| \
	0)

struct ath_tx_radiotap_header {
	struct ieee80211_radiotap_header wt_ihdr;
	uint8_t	wt_flags;
	uint8_t	wt_rate;
	uint8_t	wt_txpower;
	uint8_t	wt_antenna;
	uint32_t	wt_chan_flags;
	uint16_t	wt_chan_freq;
	uint8_t	wt_chan_ieee;
	int8_t		wt_chan_maxpow;
} __packed;

/*
 * DFS ioctl commands
 */

#define	DFS_SET_THRESH		2
#define	DFS_GET_THRESH		3
#define	DFS_RADARDETECTS	6

/*
 * DFS ioctl parameter types
 */
#define DFS_PARAM_FIRPWR	1
#define DFS_PARAM_RRSSI		2
#define DFS_PARAM_HEIGHT	3
#define DFS_PARAM_PRSSI		4
#define DFS_PARAM_INBAND	5
#define DFS_PARAM_NOL		6	/* XXX not used in FreeBSD */
#define DFS_PARAM_RELSTEP_EN	7
#define DFS_PARAM_RELSTEP	8
#define DFS_PARAM_RELPWR_EN	9
#define DFS_PARAM_RELPWR	10
#define DFS_PARAM_MAXLEN	11
#define DFS_PARAM_USEFIR128	12
#define DFS_PARAM_BLOCKRADAR	13
#define DFS_PARAM_MAXRSSI_EN	14

/* FreeBSD-specific start at 32 */
#define	DFS_PARAM_ENABLE	32
#define	DFS_PARAM_EN_EXTCH	33

/*
 * Spectral ioctl parameter types
 */
#define	SPECTRAL_PARAM_FFT_PERIOD	1
#define	SPECTRAL_PARAM_SS_PERIOD	2
#define	SPECTRAL_PARAM_SS_COUNT		3
#define	SPECTRAL_PARAM_SS_SHORT_RPT	4
#define	SPECTRAL_PARAM_ENABLED		5
#define	SPECTRAL_PARAM_ACTIVE		6
#define	SPECTRAL_PARAM_SS_SPECTRAL_PRI	7

/*
 * Spectral control parameters
 */
#define	SIOCGATHSPECTRAL	_IOWR('i', 151, struct ath_diag)

#define	SPECTRAL_CONTROL_ENABLE		2
#define	SPECTRAL_CONTROL_DISABLE	3
#define	SPECTRAL_CONTROL_START		4
#define	SPECTRAL_CONTROL_STOP		5
#define	SPECTRAL_CONTROL_GET_PARAMS	6
#define	SPECTRAL_CONTROL_SET_PARAMS	7
#define	SPECTRAL_CONTROL_ENABLE_AT_RESET	8
#define	SPECTRAL_CONTROL_DISABLE_AT_RESET	9

/*
 * Bluetooth coexistence control parameters
 */
#define	SIOCGATHBTCOEX		_IOWR('i', 152, struct ath_diag)

#endif /* _DEV_ATH_ATHIOCTL_H */
