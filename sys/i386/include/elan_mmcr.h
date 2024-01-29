/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2004 John Birrell
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/* AMD Elan SC520 Memory Mapped Configuration Region (MMCR).
 *
 * The layout of this structure is documented by AMD in the Elan SC520
 * Microcontroller Register Set Manual. The field names match those
 * described in that document. The overall structure size must be 4096
 * bytes. Ignore fields with the 'pad' prefix - they are only present for
 * alignment purposes.
 */

#ifndef _MACHINE_ELAN_MMCR_H_
#define	_MACHINE_ELAN_MMCR_H_ 1

struct elan_mmcr {
	/* CPU */
	uint16_t	REVID;
	uint8_t	CPUCTL;
	uint8_t	pad_0x003[0xd];

	/* SDRAM Controller */
	uint16_t	DRCCTL;
	uint16_t	DRCTMCTL;
	uint16_t	DRCCFG;
	uint16_t	DRCBENDADR;
	uint8_t	pad_0x01a[0x6];
	uint8_t	ECCCTL;
	uint8_t	ECCSTA;
	uint8_t	ECCCKBPOS;
	uint8_t	ECCCKTEST;
	uint32_t	ECCSBADD;
	uint32_t	ECCMBADD;
	uint8_t	pad_0x02c[0x14];

	/* SDRAM Buffer */
	uint8_t	DBCTL;
	uint8_t	pad_0x041[0xf];

	/* ROM/Flash Controller */
	uint16_t	BOOTCSCTL;
	uint8_t	pad_0x052[0x2];
	uint16_t	ROMCS1CTL;
	uint16_t	ROMCS2CTL;
	uint8_t	pad_0x058[0x8];

	/* PCI Bus Host Bridge */
	uint16_t	HBCTL;
	uint16_t	HBTGTIRQCTL;
	uint16_t	HBTGTIRQSTA;
	uint16_t	HBMSTIRQCTL;
	uint16_t	HBMSTIRQSTA;
	uint8_t	pad_0x06a[0x2];
	uint32_t	MSTINTADD;

	/* System Arbitration */
	uint8_t	SYSARBCTL;
	uint8_t	PCIARBSTA;
	uint16_t	SYSARBMENB;
	uint32_t	ARBPRICTL;
	uint8_t	pad_0x078[0x8];

	/* System Address Mapping */
	uint32_t	ADDDECCTL;
	uint32_t	WPVSTA;
	uint32_t	PAR0;
	uint32_t	PAR1;
	uint32_t	PAR2;
	uint32_t	PAR3;
	uint32_t	PAR4;
	uint32_t	PAR5;
	uint32_t	PAR6;
	uint32_t	PAR7;
	uint32_t	PAR8;
	uint32_t	PAR9;
	uint32_t	PAR10;
	uint32_t	PAR11;
	uint32_t	PAR12;
	uint32_t	PAR13;
	uint32_t	PAR14;
	uint32_t	PAR15;
	uint8_t	pad_0x0c8[0xb38];

	/* GP Bus Controller */
	uint8_t	GPECHO;
	uint8_t	GPCSDW;
	uint16_t	GPCSQUAL;
	uint8_t	pad_0xc04[0x4];
	uint8_t	GPCSRT;
	uint8_t	GPCSPW;
	uint8_t	GPCSOFF;
	uint8_t	GPRDW;
	uint8_t	GPRDOFF;
	uint8_t	GPWRW;
	uint8_t	GPWROFF;
	uint8_t	GPALEW;
	uint8_t	GPALEOFF;
	uint8_t	pad_0xc11[0xf];

	/* Programmable Input/Output */
	uint16_t	PIOPFS15_0;
	uint16_t	PIOPFS31_16;
	uint8_t	CSPFS;
	uint8_t	pad_0xc25;
	uint8_t	CLKSEL;
	uint8_t	pad_0xc27;
	uint16_t	DSCTL;
	uint16_t	PIODIR15_0;
	uint16_t	PIODIR31_16;
	uint8_t	 pad_0xc2e[0x2];
	uint16_t	PIODATA15_0;
	uint16_t	PIODATA31_16;
	uint16_t	PIOSET15_0;
	uint16_t	PIOSET31_16;
	uint16_t	PIOCLR15_0;
	uint16_t	PIOCLR31_16;
	uint8_t	pad_0xc3c[0x24];

	/* Software Timer */
	uint16_t	SWTMRMILLI;
	uint16_t	SWTMRMICRO;
	uint8_t	SWTMRCFG;
	uint8_t	pad_0xc65[0xb];

	/* General-Purpose Timers */
	uint8_t	GPTMRSTA;
	uint8_t	pad_0xc71;
	uint16_t	GPTMR0CTL;
	uint16_t	GPTMR0CNT;
	uint16_t	GPTMR0MAXCMPA;
	uint16_t	GPTMR0MAXCMPB;
	uint16_t	GPTMR1CTL;
	uint16_t	GPTMR1CNT;
	uint16_t	GPTMR1MAXCMPA;
	uint16_t	GPTMR1MAXCMPB;
	uint16_t	GPTMR2CTL;
	uint16_t	GPTMR2CNT;
	uint8_t	pad_0xc86[0x8];
	uint16_t	GPTMR2MAXCMPA;
	uint8_t	pad_0xc90[0x20];

	/* Watchdog Timer */
	uint16_t	WDTMRCTL;
	uint16_t	WDTMRCNTL;
	uint16_t	WDTMRCNTH;
	uint8_t	pad_0xcb6[0xa];

	/* UART Serial Ports */
	uint8_t	UART1CTL;
	uint8_t	UART1STA;
	uint8_t	UART1FCRSHAD;
	uint8_t	pad_0xcc3;
	uint8_t	UART2CTL;
	uint8_t	UART2STA;
	uint8_t	UART2FCRSHAD;
	uint8_t	pad_0xcc7[0x9];

	/* Synchronous Serial Interface */
	uint8_t	SSICTL;
	uint8_t	SSIXMIT;
	uint8_t	SSICMD;
	uint8_t	SSISTA;
	uint8_t	SSIRCV;
	uint8_t	pad_0xcd5[0x2b];

	/* Programmable Interrupt Controller */
	uint8_t	PICICR;
	uint8_t	pad_0xd01;
	uint8_t	MPICMODE;
	uint8_t	SL1PICMODE;
	uint8_t	SL2PICMODE;
	uint8_t	pad_0xd05[0x3];
	uint16_t	SWINT16_1;
	uint8_t	SWINT22_17;
	uint8_t	pad_0xd0b[0x5];
	uint16_t	INTPINPOL;
	uint8_t	pad_0xd12[0x2];
	uint16_t	PCIHOSTMAP;
	uint8_t	pad_0xd16[0x2];
	uint16_t	ECCMAP;
	uint8_t	GPTMR0MAP;
	uint8_t	GPTMR1MAP;
	uint8_t	GPTMR2MAP;
	uint8_t	pad_0xd1d[0x3];
	uint8_t	PIT0MAP;
	uint8_t	PIT1MAP;
	uint8_t	PIT2MAP;
	uint8_t	pad_0xd23[0x5];
	uint8_t	UART1MAP;
	uint8_t	UART2MAP;
	uint8_t	pad_0xd2a[0x6];
	uint8_t	PCIINTAMAP;
	uint8_t	PCIINTBMAP;
	uint8_t	PCIINTCMAP;
	uint8_t	PCIINTDMAP;
	uint8_t	pad_0xd34[0xc];
	uint8_t	DMABCINTMAP;
	uint8_t	SSIMAP;
	uint8_t	WDTMAP;
	uint8_t	RTCMAP;
	uint8_t	WPVMAP;
	uint8_t	ICEMAP;
	uint8_t	FERRMAP;
	uint8_t	pad_0xd47[0x9];
	uint8_t	GP0IMAP;
	uint8_t	GP1IMAP;
	uint8_t	GP2IMAP;
	uint8_t	GP3IMAP;
	uint8_t	GP4IMAP;
	uint8_t	GP5IMAP;
	uint8_t	GP6IMAP;
	uint8_t	GP7IMAP;
	uint8_t	GP8IMAP;
	uint8_t	GP9IMAP;
	uint8_t	GP10IMAP;
	uint8_t	pad_0xd5b[0x15];

	/* Reset Generation */
	uint8_t	SYSINFO;
	uint8_t	pad_0xd71;
	uint8_t	RESCFG;
	uint8_t	pad_0xd73;
	uint8_t	RESSTA;
	uint8_t	pad_0xd75[0xb];

	/* GP DMA Controller */
	uint8_t	GPDMACTL;
	uint8_t	GPDMAMMIO;
	uint16_t	GPDMAEXTCHMAPA;
	uint16_t	GPDMAEXTCHMAPB;
	uint8_t	GPDMAEXTPG0;
	uint8_t	GPDMAEXTPG1;
	uint8_t	GPDMAEXTPG2;
	uint8_t	GPDMAEXTPG3;
	uint8_t	GPDMAEXTPG5;
	uint8_t	GPDMAEXTPG6;
	uint8_t	GPDMAEXTPG7;
	uint8_t	pad_0xd8d[0x3];
	uint8_t	GPDMAEXTTC3;
	uint8_t	GPDMAEXTTC5;
	uint8_t	GPDMAEXTTC6;
	uint8_t	GPDMAEXTTC7;
	uint8_t	pad_0xd94[0x4];
	uint8_t	GPDMABCCTL;
	uint8_t	GPDMABCSTA;
	uint8_t	GPDMABSINTENB;
	uint8_t	GPDMABCVAL;
	uint8_t	pad_0xd9c[0x4];
	uint16_t	GPDMANXTADDL3;
	uint16_t	GPDMANXTADDH3;
	uint16_t	GPDMANXTADDL5;
	uint16_t	GPDMANXTADDH5;
	uint16_t	GPDMANXTADDL6;
	uint16_t	GPDMANXTADDH6;
	uint16_t	GPDMANXTADDL7;
	uint16_t	GPDMANXTADDH7;
	uint16_t	GPDMANXTTCL3;
	uint8_t	GPDMANXTTCH3;
	uint8_t	pad_0xdb3;
	uint16_t	GPDMANXTTCL5;
	uint8_t	GPDMANXTTCH5;
	uint8_t	pad_0xdb7;
	uint16_t	GPDMANXTTCL6;
	uint8_t	GPDMANXTTCH6;
	uint8_t	pad_0xdbb;
	uint16_t	GPDMANXTTCL7;
	uint8_t	GPDMANXTTCH7;
	uint8_t	pad_0xdc0[0x240];
	};

CTASSERT(sizeof(struct elan_mmcr) == 4096);

extern volatile struct elan_mmcr * elan_mmcr;

#endif /* _MACHINE_ELAN_MMCR_H_ */
