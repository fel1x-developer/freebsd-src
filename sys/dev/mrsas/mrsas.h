/*
 * Copyright (c) 2015, AVAGO Tech. All rights reserved. Authors: Marian Choy
 * Copyright (c) 2014, LSI Corp. All rights reserved. Authors: Marian Choy
 * Support: freebsdraid@avagotech.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer. 2. Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution. 3. Neither the name of the
 * <ORGANIZATION> nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing
 * official policies,either expressed or implied, of the FreeBSD Project.
 *
 * Send feedback to: <megaraidfbsd@avagotech.com> Mail to: AVAGO TECHNOLOGIES, 1621
 * Barber Lane, Milpitas, CA 95035 ATTN: MegaRaid FreeBSD
 *
 */

#include <sys/cdefs.h>
#ifndef MRSAS_H
#define	MRSAS_H

#include <sys/param.h>			/* defines used in kernel.h */
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/errno.h>
#include <sys/kernel.h>			/* types used in module initialization */
#include <sys/conf.h>			/* cdevsw struct */
#include <sys/uio.h>			/* uio struct */
#include <sys/malloc.h>
#include <sys/bus.h>			/* structs, prototypes for pci bus
					 * stuff */
#include <sys/rman.h>
#include <sys/types.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/sema.h>
#include <sys/sysctl.h>
#include <sys/stat.h>
#include <sys/taskqueue.h>
#include <sys/poll.h>
#include <sys/selinfo.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/atomic.h>

#include <dev/pci/pcivar.h>		/* For pci_get macros! */
#include <dev/pci/pcireg.h>

#define	IOCTL_SEMA_DESCRIPTION	"mrsas semaphore for MFI pool"

/*
 * Device IDs and PCI
 */
#define	MRSAS_TBOLT			0x005b
#define	MRSAS_INVADER		0x005d
#define	MRSAS_FURY			0x005f
#define	MRSAS_INTRUDER		0x00ce
#define	MRSAS_INTRUDER_24	0x00cf
#define	MRSAS_CUTLASS_52	0x0052
#define	MRSAS_CUTLASS_53	0x0053
/* Gen3.5 Conroller */
#define	MRSAS_VENTURA               0x0014
#define	MRSAS_CRUSADER              0x0015
#define	MRSAS_HARPOON               0x0016
#define	MRSAS_TOMCAT                0x0017
#define	MRSAS_VENTURA_4PORT         0x001B
#define	MRSAS_CRUSADER_4PORT        0x001C
#define	MRSAS_AERO_10E0             0x10E0
#define	MRSAS_AERO_10E1             0x10E1
#define	MRSAS_AERO_10E2             0x10E2
#define	MRSAS_AERO_10E3             0x10E3
#define	MRSAS_AERO_10E4             0x10E4
#define	MRSAS_AERO_10E5             0x10E5
#define	MRSAS_AERO_10E6             0x10E6
#define	MRSAS_AERO_10E7             0x10E7

/*
 * Firmware State Defines
 */
#define	MRSAS_FWSTATE_MAXCMD_MASK		0x0000FFFF
#define	MRSAS_FWSTATE_SGE_MASK			0x00FF0000
#define	MRSAS_FW_STATE_CHNG_INTERRUPT	1

/*
 * Message Frame Defines
 */
#define	MRSAS_SENSE_LEN					96
#define	MRSAS_FUSION_MAX_RESET_TRIES	3

/*
 * Miscellaneous Defines
 */
#define	BYTE_ALIGNMENT					1
#define	MRSAS_MAX_NAME_LENGTH			32
#define	MRSAS_VERSION					"07.709.04.00-fbsd"
#define	MRSAS_ULONG_MAX					0xFFFFFFFFFFFFFFFF
#define	MRSAS_DEFAULT_TIMEOUT			0x14	/* Temporarily set */
#define	DONE							0
#define	MRSAS_PAGE_SIZE					4096
#define	MRSAS_RESET_NOTICE_INTERVAL		5
#define	MRSAS_IO_TIMEOUT				180000	/* 180 second timeout */
#define	MRSAS_LDIO_QUEUE_DEPTH			70	/* 70 percent as default */
#define	THRESHOLD_REPLY_COUNT			50
#define	MAX_MSIX_COUNT					128

#define MAX_STREAMS_TRACKED				8
#define MR_STREAM_BITMAP				0x76543210
#define BITS_PER_INDEX_STREAM			4	/* number of bits per index in U32 TrackStream */
#define STREAM_MASK						((1 << BITS_PER_INDEX_STREAM) - 1)
#define ZERO_LAST_STREAM				0x0fffffff

/*
 * Boolean types
 */
enum err {
	SUCCESS, FAIL
};

MALLOC_DECLARE(M_MRSAS);
SYSCTL_DECL(_hw_mrsas);

#define	MRSAS_INFO		(1 << 0)
#define	MRSAS_TRACE		(1 << 1)
#define	MRSAS_FAULT		(1 << 2)
#define	MRSAS_OCR		(1 << 3)
#define	MRSAS_TOUT		MRSAS_OCR
#define	MRSAS_AEN		(1 << 4)
#define	MRSAS_PRL11		(1 << 5)

#define	mrsas_dprint(sc, level, msg, args...)       \
do {                                                \
    if (sc->mrsas_debug & level)                    \
        device_printf(sc->mrsas_dev, msg, ##args);  \
} while (0)

#define	le32_to_cpus(x)	do { *((uint32_t *)(x)) = le32toh((*(uint32_t *)x)); } while (0)
#define le16_to_cpus(x) do { *((uint16_t *)(x)) = le16toh((*(uint16_t *)x)); } while (0)

/****************************************************************************
 * Raid Context structure which describes MegaRAID specific IO Paramenters
 * This resides at offset 0x60 where the SGL normally starts in MPT IO Frames
 ****************************************************************************/

typedef struct _RAID_CONTEXT {
#if _BYTE_ORDER == _LITTLE_ENDIAN
	uint8_t Type:4;
	uint8_t nseg:4;
#else
	uint8_t nseg:4;
	uint8_t Type:4;
#endif
	uint8_t resvd0;
	uint16_t timeoutValue;
	uint8_t regLockFlags;
	uint8_t resvd1;
	uint16_t VirtualDiskTgtId;
	uint64_t regLockRowLBA;
	uint32_t regLockLength;
	uint16_t nextLMId;
	uint8_t exStatus;
	uint8_t status;
	uint8_t RAIDFlags;
	uint8_t numSGE;
	uint16_t configSeqNum;
	uint8_t spanArm;
	uint8_t priority;		/* 0x1D MR_PRIORITY_RANGE */
	uint8_t numSGEExt;		/* 0x1E 1M IO support */
	uint8_t resvd2;		/* 0x1F */
}	RAID_CONTEXT;

/*
 * Raid Context structure which describes ventura MegaRAID specific IO Paramenters
 * This resides at offset 0x60 where the SGL normally starts in MPT IO Frames
 */
typedef struct _RAID_CONTEXT_G35 {
#if _BYTE_ORDER == _LITTLE_ENDIAN
	uint16_t Type:4;
	uint16_t nseg:4;
	uint16_t resvd0:8;
#else
	uint16_t resvd0:8;
	uint16_t nseg:4;
	uint16_t Type:4;
#endif
	uint16_t timeoutValue;
	union {
		struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
			uint16_t reserved:1;
			uint16_t sld:1;
			uint16_t c2f:1;
			uint16_t fwn:1;
			uint16_t sqn:1;
			uint16_t sbs:1;
			uint16_t rw:1;
			uint16_t log:1;
			uint16_t cpuSel:4;
			uint16_t setDivert:4;
#else
			uint16_t setDivert:4;
			uint16_t cpuSel:4;
			uint16_t log:1;
			uint16_t rw:1;
			uint16_t sbs:1;
			uint16_t sqn:1;
			uint16_t fwn:1;
			uint16_t c2f:1;
			uint16_t sld:1;
			uint16_t reserved:1;
#endif
		}	bits;
		uint16_t s;
	}	routingFlags;
	uint16_t VirtualDiskTgtId;
	uint64_t regLockRowLBA;
	uint32_t regLockLength;
	union {
		uint16_t nextLMId;
		uint16_t peerSMID;
	}	smid;
	uint8_t exStatus;
	uint8_t status;
	uint8_t RAIDFlags;
	uint8_t spanArm;
	uint16_t configSeqNum;
#if _BYTE_ORDER == _LITTLE_ENDIAN
	uint16_t numSGE:12;
	uint16_t reserved:3;
	uint16_t streamDetected:1;
#else
	uint16_t streamDetected:1;
	uint16_t reserved:3;
	uint16_t numSGE:12;
#endif
	uint8_t resvd2[2];
}	RAID_CONTEXT_G35;

typedef union _RAID_CONTEXT_UNION {
	RAID_CONTEXT raid_context;
	RAID_CONTEXT_G35 raid_context_g35;
}	RAID_CONTEXT_UNION, *PRAID_CONTEXT_UNION;

/*************************************************************************
 * MPI2 Defines
 ************************************************************************/

#define	MPI2_FUNCTION_IOC_INIT					(0x02)	/* IOC Init */
#define	MPI2_WHOINIT_HOST_DRIVER				(0x04)
#define	MPI2_VERSION_MAJOR						(0x02)
#define	MPI2_VERSION_MINOR						(0x00)
#define	MPI2_VERSION_MAJOR_MASK					(0xFF00)
#define	MPI2_VERSION_MAJOR_SHIFT				(8)
#define	MPI2_VERSION_MINOR_MASK					(0x00FF)
#define	MPI2_VERSION_MINOR_SHIFT				(0)
#define	MPI2_VERSION ((MPI2_VERSION_MAJOR << MPI2_VERSION_MAJOR_SHIFT) | \
                      MPI2_VERSION_MINOR)
#define	MPI2_HEADER_VERSION_UNIT				(0x10)
#define	MPI2_HEADER_VERSION_DEV					(0x00)
#define	MPI2_HEADER_VERSION_UNIT_MASK			(0xFF00)
#define	MPI2_HEADER_VERSION_UNIT_SHIFT			(8)
#define	MPI2_HEADER_VERSION_DEV_MASK			(0x00FF)
#define	MPI2_HEADER_VERSION_DEV_SHIFT			(0)
#define	MPI2_HEADER_VERSION ((MPI2_HEADER_VERSION_UNIT << 8) | MPI2_HEADER_VERSION_DEV)
#define	MPI2_IEEE_SGE_FLAGS_IOCPLBNTA_ADDR		(0x03)
#define	MPI2_SCSIIO_EEDPFLAGS_INC_PRI_REFTAG	(0x8000)
#define	MPI2_SCSIIO_EEDPFLAGS_CHECK_REFTAG		(0x0400)
#define	MPI2_SCSIIO_EEDPFLAGS_CHECK_REMOVE_OP	(0x0003)
#define	MPI2_SCSIIO_EEDPFLAGS_CHECK_APPTAG		(0x0200)
#define	MPI2_SCSIIO_EEDPFLAGS_CHECK_GUARD		(0x0100)
#define	MPI2_SCSIIO_EEDPFLAGS_INSERT_OP			(0x0004)
#define	MPI2_FUNCTION_SCSI_IO_REQUEST			(0x00)	/* SCSI IO */
#define	MPI2_FUNCTION_SCSI_TASK_MGMT			(0x01)
#define	MPI2_REQ_DESCRIPT_FLAGS_HIGH_PRIORITY	(0x03)
#define	MPI2_REQ_DESCRIPT_FLAGS_FP_IO			(0x06)
#define	MPI2_REQ_DESCRIPT_FLAGS_SCSI_IO			(0x00)
#define	MPI2_SGE_FLAGS_64_BIT_ADDRESSING		(0x02)
#define	MPI2_SCSIIO_CONTROL_WRITE				(0x01000000)
#define	MPI2_SCSIIO_CONTROL_READ				(0x02000000)
#define	MPI2_REQ_DESCRIPT_FLAGS_TYPE_MASK		(0x0E)
#define	MPI2_RPY_DESCRIPT_FLAGS_UNUSED			(0x0F)
#define	MPI2_RPY_DESCRIPT_FLAGS_SCSI_IO_SUCCESS	(0x00)
#define	MPI2_RPY_DESCRIPT_FLAGS_TYPE_MASK		(0x0F)
#define	MPI2_WRSEQ_FLUSH_KEY_VALUE				(0x0)
#define	MPI2_WRITE_SEQUENCE_OFFSET				(0x00000004)
#define	MPI2_WRSEQ_1ST_KEY_VALUE				(0xF)
#define	MPI2_WRSEQ_2ND_KEY_VALUE				(0x4)
#define	MPI2_WRSEQ_3RD_KEY_VALUE				(0xB)
#define	MPI2_WRSEQ_4TH_KEY_VALUE				(0x2)
#define	MPI2_WRSEQ_5TH_KEY_VALUE				(0x7)
#define	MPI2_WRSEQ_6TH_KEY_VALUE				(0xD)

#ifndef MPI2_POINTER
#define	MPI2_POINTER	*
#endif

/***************************************
 * MPI2 Structures
 ***************************************/

typedef struct _MPI25_IEEE_SGE_CHAIN64 {
	uint64_t Address;
	uint32_t Length;
	uint16_t Reserved1;
	uint8_t NextChainOffset;
	uint8_t Flags;
}	MPI25_IEEE_SGE_CHAIN64, MPI2_POINTER PTR_MPI25_IEEE_SGE_CHAIN64,
Mpi25IeeeSgeChain64_t, MPI2_POINTER pMpi25IeeeSgeChain64_t;

typedef struct _MPI2_SGE_SIMPLE_UNION {
	uint32_t FlagsLength;
	union {
		uint32_t Address32;
		uint64_t Address64;
	}	u;
}	MPI2_SGE_SIMPLE_UNION, MPI2_POINTER PTR_MPI2_SGE_SIMPLE_UNION,
Mpi2SGESimpleUnion_t, MPI2_POINTER pMpi2SGESimpleUnion_t;

typedef struct {
	uint8_t CDB[20];		/* 0x00 */
	uint32_t PrimaryReferenceTag;	/* 0x14 */
	uint16_t PrimaryApplicationTag;/* 0x18 */
	uint16_t PrimaryApplicationTagMask;	/* 0x1A */
	uint32_t TransferLength;	/* 0x1C */
}	MPI2_SCSI_IO_CDB_EEDP32, MPI2_POINTER PTR_MPI2_SCSI_IO_CDB_EEDP32,
Mpi2ScsiIoCdbEedp32_t, MPI2_POINTER pMpi2ScsiIoCdbEedp32_t;

typedef struct _MPI2_SGE_CHAIN_UNION {
	uint16_t Length;
	uint8_t NextChainOffset;
	uint8_t Flags;
	union {
		uint32_t Address32;
		uint64_t Address64;
	}	u;
}	MPI2_SGE_CHAIN_UNION, MPI2_POINTER PTR_MPI2_SGE_CHAIN_UNION,
Mpi2SGEChainUnion_t, MPI2_POINTER pMpi2SGEChainUnion_t;

typedef struct _MPI2_IEEE_SGE_SIMPLE32 {
	uint32_t Address;
	uint32_t FlagsLength;
}	MPI2_IEEE_SGE_SIMPLE32, MPI2_POINTER PTR_MPI2_IEEE_SGE_SIMPLE32,
Mpi2IeeeSgeSimple32_t, MPI2_POINTER pMpi2IeeeSgeSimple32_t;
typedef struct _MPI2_IEEE_SGE_SIMPLE64 {
	uint64_t Address;
	uint32_t Length;
	uint16_t Reserved1;
	uint8_t Reserved2;
	uint8_t Flags;
}	MPI2_IEEE_SGE_SIMPLE64, MPI2_POINTER PTR_MPI2_IEEE_SGE_SIMPLE64,
Mpi2IeeeSgeSimple64_t, MPI2_POINTER pMpi2IeeeSgeSimple64_t;

typedef union _MPI2_IEEE_SGE_SIMPLE_UNION {
	MPI2_IEEE_SGE_SIMPLE32 Simple32;
	MPI2_IEEE_SGE_SIMPLE64 Simple64;
}	MPI2_IEEE_SGE_SIMPLE_UNION, MPI2_POINTER PTR_MPI2_IEEE_SGE_SIMPLE_UNION,
Mpi2IeeeSgeSimpleUnion_t, MPI2_POINTER pMpi2IeeeSgeSimpleUnion_t;

typedef MPI2_IEEE_SGE_SIMPLE32 MPI2_IEEE_SGE_CHAIN32;
typedef MPI2_IEEE_SGE_SIMPLE64 MPI2_IEEE_SGE_CHAIN64;

typedef union _MPI2_IEEE_SGE_CHAIN_UNION {
	MPI2_IEEE_SGE_CHAIN32 Chain32;
	MPI2_IEEE_SGE_CHAIN64 Chain64;
}	MPI2_IEEE_SGE_CHAIN_UNION, MPI2_POINTER PTR_MPI2_IEEE_SGE_CHAIN_UNION,
Mpi2IeeeSgeChainUnion_t, MPI2_POINTER pMpi2IeeeSgeChainUnion_t;

typedef union _MPI2_SGE_IO_UNION {
	MPI2_SGE_SIMPLE_UNION MpiSimple;
	MPI2_SGE_CHAIN_UNION MpiChain;
	MPI2_IEEE_SGE_SIMPLE_UNION IeeeSimple;
	MPI2_IEEE_SGE_CHAIN_UNION IeeeChain;
}	MPI2_SGE_IO_UNION, MPI2_POINTER PTR_MPI2_SGE_IO_UNION,
Mpi2SGEIOUnion_t, MPI2_POINTER pMpi2SGEIOUnion_t;

typedef union {
	uint8_t CDB32[32];
	MPI2_SCSI_IO_CDB_EEDP32 EEDP32;
	MPI2_SGE_SIMPLE_UNION SGE;
}	MPI2_SCSI_IO_CDB_UNION, MPI2_POINTER PTR_MPI2_SCSI_IO_CDB_UNION,
Mpi2ScsiIoCdb_t, MPI2_POINTER pMpi2ScsiIoCdb_t;

/****************************************************************************
 *  *  SCSI Task Management messages
 *   ****************************************************************************/

/*SCSI Task Management Request Message */
typedef struct _MPI2_SCSI_TASK_MANAGE_REQUEST {
	uint16_t DevHandle;        /*0x00 */
	uint8_t ChainOffset;       /*0x02 */
	uint8_t Function;      /*0x03 */
	uint8_t Reserved1;     /*0x04 */
	uint8_t TaskType;      /*0x05 */
	uint8_t Reserved2;     /*0x06 */
	uint8_t MsgFlags;      /*0x07 */
	uint8_t VP_ID;     /*0x08 */
	uint8_t VF_ID;     /*0x09 */
	uint16_t Reserved3;        /*0x0A */
	uint8_t LUN[8];        /*0x0C */
	uint32_t Reserved4[7]; /*0x14 */
	uint16_t TaskMID;      /*0x30 */
	uint16_t Reserved5;        /*0x32 */
} MPI2_SCSI_TASK_MANAGE_REQUEST;

/*SCSI Task Management Reply Message */
typedef struct _MPI2_SCSI_TASK_MANAGE_REPLY {
	uint16_t DevHandle;        /*0x00 */
	uint8_t MsgLength;     /*0x02 */
	uint8_t Function;      /*0x03 */
	uint8_t ResponseCode;  /*0x04 */
	uint8_t TaskType;      /*0x05 */
	uint8_t Reserved1;     /*0x06 */
	uint8_t MsgFlags;      /*0x07 */
	uint8_t VP_ID;     /*0x08 */
	uint8_t VF_ID;     /*0x09 */
	uint16_t Reserved2;        /*0x0A */
	uint16_t Reserved3;        /*0x0C */
	uint16_t IOCStatus;        /*0x0E */
	uint32_t IOCLogInfo;       /*0x10 */
	uint32_t TerminationCount; /*0x14 */
	uint32_t ResponseInfo; /*0x18 */
} MPI2_SCSI_TASK_MANAGE_REPLY;

typedef struct _MR_TM_REQUEST {
	char request[128];
} MR_TM_REQUEST;

typedef struct _MR_TM_REPLY {
	char reply[128];
} MR_TM_REPLY;

/* SCSI Task Management Request Message */
typedef struct _MR_TASK_MANAGE_REQUEST {
	/*To be type casted to struct MPI2_SCSI_TASK_MANAGE_REQUEST */
	MR_TM_REQUEST        TmRequest;
	union {
		struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
			uint32_t isTMForLD:1;
			uint32_t isTMForPD:1;
			uint32_t reserved1:30;
#else
			uint32_t reserved1:30;
			uint32_t isTMForPD:1;
			uint32_t isTMForLD:1;
#endif
			uint32_t reserved2;
		} tmReqFlags;
		MR_TM_REPLY   TMReply;
	} uTmReqReply;
} MR_TASK_MANAGE_REQUEST;

/* TaskType values */
#define MPI2_SCSITASKMGMT_TASKTYPE_ABORT_TASK           (0x01)
#define MPI2_SCSITASKMGMT_TASKTYPE_ABRT_TASK_SET        (0x02)
#define MPI2_SCSITASKMGMT_TASKTYPE_TARGET_RESET         (0x03)
#define MPI2_SCSITASKMGMT_TASKTYPE_LOGICAL_UNIT_RESET   (0x05)
#define MPI2_SCSITASKMGMT_TASKTYPE_CLEAR_TASK_SET       (0x06)
#define MPI2_SCSITASKMGMT_TASKTYPE_QUERY_TASK           (0x07)
#define MPI2_SCSITASKMGMT_TASKTYPE_CLR_ACA              (0x08)
#define MPI2_SCSITASKMGMT_TASKTYPE_QRY_TASK_SET         (0x09)
#define MPI2_SCSITASKMGMT_TASKTYPE_QRY_ASYNC_EVENT      (0x0A)

/* ResponseCode values */
#define MPI2_SCSITASKMGMT_RSP_TM_COMPLETE               (0x00)
#define MPI2_SCSITASKMGMT_RSP_INVALID_FRAME             (0x02)
#define MPI2_SCSITASKMGMT_RSP_TM_NOT_SUPPORTED          (0x04)
#define MPI2_SCSITASKMGMT_RSP_TM_FAILED                 (0x05)
#define MPI2_SCSITASKMGMT_RSP_TM_SUCCEEDED              (0x08)
#define MPI2_SCSITASKMGMT_RSP_TM_INVALID_LUN            (0x09)
#define MPI2_SCSITASKMGMT_RSP_TM_OVERLAPPED_TAG         (0x0A)
#define MPI2_SCSITASKMGMT_RSP_IO_QUEUED_ON_IOC          (0x80)

/*
 * RAID SCSI IO Request Message Total SGE count will be one less than
 * _MPI2_SCSI_IO_REQUEST
 */
typedef struct _MPI2_RAID_SCSI_IO_REQUEST {
	uint16_t DevHandle;		/* 0x00 */
	uint8_t ChainOffset;		/* 0x02 */
	uint8_t Function;		/* 0x03 */
	uint16_t Reserved1;		/* 0x04 */
	uint8_t Reserved2;		/* 0x06 */
	uint8_t MsgFlags;		/* 0x07 */
	uint8_t VP_ID;			/* 0x08 */
	uint8_t VF_ID;			/* 0x09 */
	uint16_t Reserved3;		/* 0x0A */
	uint32_t SenseBufferLowAddress;/* 0x0C */
	uint16_t SGLFlags;		/* 0x10 */
	uint8_t SenseBufferLength;	/* 0x12 */
	uint8_t Reserved4;		/* 0x13 */
	uint8_t SGLOffset0;		/* 0x14 */
	uint8_t SGLOffset1;		/* 0x15 */
	uint8_t SGLOffset2;		/* 0x16 */
	uint8_t SGLOffset3;		/* 0x17 */
	uint32_t SkipCount;		/* 0x18 */
	uint32_t DataLength;		/* 0x1C */
	uint32_t BidirectionalDataLength;	/* 0x20 */
	uint16_t IoFlags;		/* 0x24 */
	uint16_t EEDPFlags;		/* 0x26 */
	uint32_t EEDPBlockSize;	/* 0x28 */
	uint32_t SecondaryReferenceTag;/* 0x2C */
	uint16_t SecondaryApplicationTag;	/* 0x30 */
	uint16_t ApplicationTagTranslationMask;	/* 0x32 */
	uint8_t LUN[8];		/* 0x34 */
	uint32_t Control;		/* 0x3C */
	MPI2_SCSI_IO_CDB_UNION CDB;	/* 0x40 */
	RAID_CONTEXT_UNION RaidContext;	/* 0x60 */
	MPI2_SGE_IO_UNION SGL;		/* 0x80 */
}	MRSAS_RAID_SCSI_IO_REQUEST, MPI2_POINTER PTR_MRSAS_RAID_SCSI_IO_REQUEST,
MRSASRaidSCSIIORequest_t, MPI2_POINTER pMRSASRaidSCSIIORequest_t;

/*
 * MPT RAID MFA IO Descriptor.
 */
typedef struct _MRSAS_RAID_MFA_IO_DESCRIPTOR {
	uint32_t RequestFlags:8;
	uint32_t MessageAddress1:24;	/* bits 31:8 */
	uint32_t MessageAddress2;	/* bits 61:32 */
}	MRSAS_RAID_MFA_IO_REQUEST_DESCRIPTOR, *PMRSAS_RAID_MFA_IO_REQUEST_DESCRIPTOR;

/* Default Request Descriptor */
typedef struct _MPI2_DEFAULT_REQUEST_DESCRIPTOR {
	uint8_t RequestFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint16_t LMID;			/* 0x04 */
	uint16_t DescriptorTypeDependent;	/* 0x06 */
}	MPI2_DEFAULT_REQUEST_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_DEFAULT_REQUEST_DESCRIPTOR,
Mpi2DefaultRequestDescriptor_t, MPI2_POINTER pMpi2DefaultRequestDescriptor_t;

/* High Priority Request Descriptor */
typedef struct _MPI2_HIGH_PRIORITY_REQUEST_DESCRIPTOR {
	uint8_t RequestFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint16_t LMID;			/* 0x04 */
	uint16_t Reserved1;		/* 0x06 */
}	MPI2_HIGH_PRIORITY_REQUEST_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_HIGH_PRIORITY_REQUEST_DESCRIPTOR,
Mpi2HighPriorityRequestDescriptor_t, MPI2_POINTER pMpi2HighPriorityRequestDescriptor_t;

/* SCSI IO Request Descriptor */
typedef struct _MPI2_SCSI_IO_REQUEST_DESCRIPTOR {
	uint8_t RequestFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint16_t LMID;			/* 0x04 */
	uint16_t DevHandle;		/* 0x06 */
}	MPI2_SCSI_IO_REQUEST_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_SCSI_IO_REQUEST_DESCRIPTOR,
Mpi2SCSIIORequestDescriptor_t, MPI2_POINTER pMpi2SCSIIORequestDescriptor_t;

/* SCSI Target Request Descriptor */
typedef struct _MPI2_SCSI_TARGET_REQUEST_DESCRIPTOR {
	uint8_t RequestFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint16_t LMID;			/* 0x04 */
	uint16_t IoIndex;		/* 0x06 */
}	MPI2_SCSI_TARGET_REQUEST_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_SCSI_TARGET_REQUEST_DESCRIPTOR,
Mpi2SCSITargetRequestDescriptor_t, MPI2_POINTER pMpi2SCSITargetRequestDescriptor_t;

/* RAID Accelerator Request Descriptor */
typedef struct _MPI2_RAID_ACCEL_REQUEST_DESCRIPTOR {
	uint8_t RequestFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint16_t LMID;			/* 0x04 */
	uint16_t Reserved;		/* 0x06 */
}	MPI2_RAID_ACCEL_REQUEST_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_RAID_ACCEL_REQUEST_DESCRIPTOR,
Mpi2RAIDAcceleratorRequestDescriptor_t, MPI2_POINTER pMpi2RAIDAcceleratorRequestDescriptor_t;

/* union of Request Descriptors */
typedef union _MRSAS_REQUEST_DESCRIPTOR_UNION {
	MPI2_DEFAULT_REQUEST_DESCRIPTOR Default;
	MPI2_HIGH_PRIORITY_REQUEST_DESCRIPTOR HighPriority;
	MPI2_SCSI_IO_REQUEST_DESCRIPTOR SCSIIO;
	MPI2_SCSI_TARGET_REQUEST_DESCRIPTOR SCSITarget;
	MPI2_RAID_ACCEL_REQUEST_DESCRIPTOR RAIDAccelerator;
	MRSAS_RAID_MFA_IO_REQUEST_DESCRIPTOR MFAIo;
	union {
		struct {
			uint32_t low;
			uint32_t high;
		}	u;
		uint64_t Words;
	}	addr;
}	MRSAS_REQUEST_DESCRIPTOR_UNION;

/* Default Reply Descriptor */
typedef struct _MPI2_DEFAULT_REPLY_DESCRIPTOR {
	uint8_t ReplyFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t DescriptorTypeDependent1;	/* 0x02 */
	uint32_t DescriptorTypeDependent2;	/* 0x04 */
}	MPI2_DEFAULT_REPLY_DESCRIPTOR, MPI2_POINTER PTR_MPI2_DEFAULT_REPLY_DESCRIPTOR,
Mpi2DefaultReplyDescriptor_t, MPI2_POINTER pMpi2DefaultReplyDescriptor_t;

/* Address Reply Descriptor */
typedef struct _MPI2_ADDRESS_REPLY_DESCRIPTOR {
	uint8_t ReplyFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint32_t ReplyFrameAddress;	/* 0x04 */
}	MPI2_ADDRESS_REPLY_DESCRIPTOR, MPI2_POINTER PTR_MPI2_ADDRESS_REPLY_DESCRIPTOR,
Mpi2AddressReplyDescriptor_t, MPI2_POINTER pMpi2AddressReplyDescriptor_t;

/* SCSI IO Success Reply Descriptor */
typedef struct _MPI2_SCSI_IO_SUCCESS_REPLY_DESCRIPTOR {
	uint8_t ReplyFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint16_t TaskTag;		/* 0x04 */
	uint16_t Reserved1;		/* 0x06 */
}	MPI2_SCSI_IO_SUCCESS_REPLY_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_SCSI_IO_SUCCESS_REPLY_DESCRIPTOR,
Mpi2SCSIIOSuccessReplyDescriptor_t, MPI2_POINTER pMpi2SCSIIOSuccessReplyDescriptor_t;

/* TargetAssist Success Reply Descriptor */
typedef struct _MPI2_TARGETASSIST_SUCCESS_REPLY_DESCRIPTOR {
	uint8_t ReplyFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint8_t SequenceNumber;	/* 0x04 */
	uint8_t Reserved1;		/* 0x05 */
	uint16_t IoIndex;		/* 0x06 */
}	MPI2_TARGETASSIST_SUCCESS_REPLY_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_TARGETASSIST_SUCCESS_REPLY_DESCRIPTOR,
Mpi2TargetAssistSuccessReplyDescriptor_t, MPI2_POINTER pMpi2TargetAssistSuccessReplyDescriptor_t;

/* Target Command Buffer Reply Descriptor */
typedef struct _MPI2_TARGET_COMMAND_BUFFER_REPLY_DESCRIPTOR {
	uint8_t ReplyFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint8_t VP_ID;			/* 0x02 */
	uint8_t Flags;			/* 0x03 */
	uint16_t InitiatorDevHandle;	/* 0x04 */
	uint16_t IoIndex;		/* 0x06 */
}	MPI2_TARGET_COMMAND_BUFFER_REPLY_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_TARGET_COMMAND_BUFFER_REPLY_DESCRIPTOR,
Mpi2TargetCommandBufferReplyDescriptor_t, MPI2_POINTER pMpi2TargetCommandBufferReplyDescriptor_t;

/* RAID Accelerator Success Reply Descriptor */
typedef struct _MPI2_RAID_ACCELERATOR_SUCCESS_REPLY_DESCRIPTOR {
	uint8_t ReplyFlags;		/* 0x00 */
	uint8_t MSIxIndex;		/* 0x01 */
	uint16_t SMID;			/* 0x02 */
	uint32_t Reserved;		/* 0x04 */
}	MPI2_RAID_ACCELERATOR_SUCCESS_REPLY_DESCRIPTOR,

	MPI2_POINTER PTR_MPI2_RAID_ACCELERATOR_SUCCESS_REPLY_DESCRIPTOR,
Mpi2RAIDAcceleratorSuccessReplyDescriptor_t, MPI2_POINTER pMpi2RAIDAcceleratorSuccessReplyDescriptor_t;

/* union of Reply Descriptors */
typedef union _MPI2_REPLY_DESCRIPTORS_UNION {
	MPI2_DEFAULT_REPLY_DESCRIPTOR Default;
	MPI2_ADDRESS_REPLY_DESCRIPTOR AddressReply;
	MPI2_SCSI_IO_SUCCESS_REPLY_DESCRIPTOR SCSIIOSuccess;
	MPI2_TARGETASSIST_SUCCESS_REPLY_DESCRIPTOR TargetAssistSuccess;
	MPI2_TARGET_COMMAND_BUFFER_REPLY_DESCRIPTOR TargetCommandBuffer;
	MPI2_RAID_ACCELERATOR_SUCCESS_REPLY_DESCRIPTOR RAIDAcceleratorSuccess;
	uint64_t Words;
}	MPI2_REPLY_DESCRIPTORS_UNION, MPI2_POINTER PTR_MPI2_REPLY_DESCRIPTORS_UNION,
Mpi2ReplyDescriptorsUnion_t, MPI2_POINTER pMpi2ReplyDescriptorsUnion_t;

typedef union {
	volatile unsigned int val;
	unsigned int val_rdonly;
} mrsas_atomic_t;

#define	mrsas_atomic_read(v)	atomic_load_acq_int(&(v)->val)
#define	mrsas_atomic_set(v,i)	atomic_store_rel_int(&(v)->val, i)
#define	mrsas_atomic_dec(v)	atomic_subtract_int(&(v)->val, 1)
#define	mrsas_atomic_inc(v)	atomic_add_int(&(v)->val, 1)

static inline int
mrsas_atomic_inc_return(mrsas_atomic_t *v)
{
	return 1 + atomic_fetchadd_int(&(v)->val, 1);
}

/* IOCInit Request message */
typedef struct _MPI2_IOC_INIT_REQUEST {
	uint8_t WhoInit;		/* 0x00 */
	uint8_t Reserved1;		/* 0x01 */
	uint8_t ChainOffset;		/* 0x02 */
	uint8_t Function;		/* 0x03 */
	uint16_t Reserved2;		/* 0x04 */
	uint8_t Reserved3;		/* 0x06 */
	uint8_t MsgFlags;		/* 0x07 */
	uint8_t VP_ID;			/* 0x08 */
	uint8_t VF_ID;			/* 0x09 */
	uint16_t Reserved4;		/* 0x0A */
	uint16_t MsgVersion;		/* 0x0C */
	uint16_t HeaderVersion;	/* 0x0E */
	uint32_t Reserved5;		/* 0x10 */
	uint16_t Reserved6;		/* 0x14 */
	uint8_t HostPageSize;		/* 0x16 */
	uint8_t HostMSIxVectors;	/* 0x17 */
	uint16_t Reserved8;		/* 0x18 */
	uint16_t SystemRequestFrameSize;	/* 0x1A */
	uint16_t ReplyDescriptorPostQueueDepth;	/* 0x1C */
	uint16_t ReplyFreeQueueDepth;	/* 0x1E */
	uint32_t SenseBufferAddressHigh;	/* 0x20 */
	uint32_t SystemReplyAddressHigh;	/* 0x24 */
	uint64_t SystemRequestFrameBaseAddress;	/* 0x28 */
	uint64_t ReplyDescriptorPostQueueAddress;	/* 0x30 */
	uint64_t ReplyFreeQueueAddress;/* 0x38 */
	uint64_t TimeStamp;		/* 0x40 */
}	MPI2_IOC_INIT_REQUEST, MPI2_POINTER PTR_MPI2_IOC_INIT_REQUEST,
Mpi2IOCInitRequest_t, MPI2_POINTER pMpi2IOCInitRequest_t;

/*
 * MR private defines
 */
#define	MR_PD_INVALID			0xFFFF
#define	MR_DEVHANDLE_INVALID	0xFFFF
#define	MAX_SPAN_DEPTH			8
#define	MAX_QUAD_DEPTH			MAX_SPAN_DEPTH
#define	MAX_RAIDMAP_SPAN_DEPTH	(MAX_SPAN_DEPTH)
#define	MAX_ROW_SIZE			32
#define	MAX_RAIDMAP_ROW_SIZE	(MAX_ROW_SIZE)
#define	MAX_LOGICAL_DRIVES		64
#define	MAX_LOGICAL_DRIVES_EXT	256
#define	MAX_LOGICAL_DRIVES_DYN	512

#define	MAX_RAIDMAP_LOGICAL_DRIVES	(MAX_LOGICAL_DRIVES)
#define	MAX_RAIDMAP_VIEWS			(MAX_LOGICAL_DRIVES)

#define	MAX_ARRAYS				128
#define	MAX_RAIDMAP_ARRAYS		(MAX_ARRAYS)

#define	MAX_ARRAYS_EXT			256
#define	MAX_API_ARRAYS_EXT		MAX_ARRAYS_EXT
#define	MAX_API_ARRAYS_DYN		512

#define	MAX_PHYSICAL_DEVICES	256
#define	MAX_RAIDMAP_PHYSICAL_DEVICES	(MAX_PHYSICAL_DEVICES)
#define	MAX_RAIDMAP_PHYSICAL_DEVICES_DYN	512
#define	MR_DCMD_LD_MAP_GET_INFO	0x0300e101
#define	MR_DCMD_SYSTEM_PD_MAP_GET_INFO	0x0200e102
#define MR_DCMD_PD_MFI_TASK_MGMT	0x0200e100

#define MR_DCMD_PD_GET_INFO		0x02020000
#define	MRSAS_MAX_PD_CHANNELS		1
#define	MRSAS_MAX_LD_CHANNELS		1
#define	MRSAS_MAX_DEV_PER_CHANNEL	256
#define	MRSAS_DEFAULT_INIT_ID		-1
#define	MRSAS_MAX_LUN				8
#define	MRSAS_DEFAULT_CMD_PER_LUN	256
#define	MRSAS_MAX_PD				(MRSAS_MAX_PD_CHANNELS * \
			MRSAS_MAX_DEV_PER_CHANNEL)
#define	MRSAS_MAX_LD_IDS			(MRSAS_MAX_LD_CHANNELS * \
			MRSAS_MAX_DEV_PER_CHANNEL)

#define	VD_EXT_DEBUG	0
#define TM_DEBUG		1

/*******************************************************************
 * RAID map related structures
 ********************************************************************/
#pragma pack(1)
typedef struct _MR_DEV_HANDLE_INFO {
	uint16_t curDevHdl;
	uint8_t validHandles;
	uint8_t interfaceType;
	uint16_t devHandle[2];
}	MR_DEV_HANDLE_INFO;

#pragma pack()

typedef struct _MR_ARRAY_INFO {
	uint16_t pd[MAX_RAIDMAP_ROW_SIZE];
}	MR_ARRAY_INFO;

typedef struct _MR_QUAD_ELEMENT {
	uint64_t logStart;
	uint64_t logEnd;
	uint64_t offsetInSpan;
	uint32_t diff;
	uint32_t reserved1;
}	MR_QUAD_ELEMENT;

typedef struct _MR_SPAN_INFO {
	uint32_t noElements;
	uint32_t reserved1;
	MR_QUAD_ELEMENT quad[MAX_RAIDMAP_SPAN_DEPTH];
}	MR_SPAN_INFO;

typedef struct _MR_LD_SPAN_ {
	uint64_t startBlk;
	uint64_t numBlks;
	uint16_t arrayRef;
	uint8_t spanRowSize;
	uint8_t spanRowDataSize;
	uint8_t reserved[4];
}	MR_LD_SPAN;

typedef struct _MR_SPAN_BLOCK_INFO {
	uint64_t num_rows;
	MR_LD_SPAN span;
	MR_SPAN_INFO block_span_info;
}	MR_SPAN_BLOCK_INFO;

typedef struct _MR_LD_RAID {
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t fpCapable:1;
		uint32_t raCapable:1;
		uint32_t reserved5:2;
		uint32_t ldPiMode:4;
		uint32_t pdPiMode:4;
		uint32_t encryptionType:8;
		uint32_t fpWriteCapable:1;
		uint32_t fpReadCapable:1;
		uint32_t fpWriteAcrossStripe:1;
		uint32_t fpReadAcrossStripe:1;
		uint32_t fpNonRWCapable:1;
		uint32_t tmCapable:1;
		uint32_t fpCacheBypassCapable:1;
		uint32_t reserved4:5;
#else
		uint32_t reserved4:5;
		uint32_t fpCacheBypassCapable:1;
		uint32_t tmCapable:1;
		uint32_t fpNonRWCapable:1;
		uint32_t fpReadAcrossStripe:1;
		uint32_t fpWriteAcrossStripe:1;
		uint32_t fpReadCapable:1;
		uint32_t fpWriteCapable:1;
		uint32_t encryptionType:8;
		uint32_t pdPiMode:4;
		uint32_t ldPiMode:4;
		uint32_t reserved5:2;
		uint32_t raCapable:1;
		uint32_t fpCapable:1;
#endif
	}	capability;
	uint32_t reserved6;
	uint64_t size;

	uint8_t spanDepth;
	uint8_t level;
	uint8_t stripeShift;
	uint8_t rowSize;

	uint8_t rowDataSize;
	uint8_t writeMode;
	uint8_t PRL;
	uint8_t SRL;

	uint16_t targetId;
	uint8_t ldState;
	uint8_t regTypeReqOnWrite;
	uint8_t modFactor;
	uint8_t regTypeReqOnRead;
	uint16_t seqNum;

	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t reserved:30;
		uint32_t regTypeReqOnReadLsValid:1;
		uint32_t ldSyncRequired:1;
#else
		uint32_t ldSyncRequired:1;
		uint32_t regTypeReqOnReadLsValid:1;
		uint32_t reserved:30;
#endif
	}	flags;

	uint8_t LUN[8];
	uint8_t fpIoTimeoutForLd;
	uint8_t reserved2[3];
	uint32_t logicalBlockLength;
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t reserved1:24;
		uint32_t LdLogicalBlockExp:4;
		uint32_t LdPiExp:4;
#else
		uint32_t LdPiExp:4;
		uint32_t LdLogicalBlockExp:4;
		uint32_t reserved1:24;
#endif
	}	exponent;
	uint8_t reserved3[0x80 - 0x38];
}	MR_LD_RAID;

typedef struct _MR_LD_SPAN_MAP {
	MR_LD_RAID ldRaid;
	uint8_t dataArmMap[MAX_RAIDMAP_ROW_SIZE];
	MR_SPAN_BLOCK_INFO spanBlock[MAX_RAIDMAP_SPAN_DEPTH];
}	MR_LD_SPAN_MAP;

typedef struct _MR_FW_RAID_MAP {
	uint32_t totalSize;
	union {
		struct {
			uint32_t maxLd;
			uint32_t maxSpanDepth;
			uint32_t maxRowSize;
			uint32_t maxPdCount;
			uint32_t maxArrays;
		}	validationInfo;
		uint32_t version[5];
		uint32_t reserved1[5];
	}	raid_desc;
	uint32_t ldCount;
	uint32_t Reserved1;

	/*
	 * This doesn't correspond to FW Ld Tgt Id to LD, but will purge. For
	 * example: if tgt Id is 4 and FW LD is 2, and there is only one LD,
	 * FW will populate the array like this. [0xFF, 0xFF, 0xFF, 0xFF,
	 * 0x0,.....]. This is to help reduce the entire strcture size if
	 * there are few LDs or driver is looking info for 1 LD only.
	 */
	uint8_t ldTgtIdToLd[MAX_RAIDMAP_LOGICAL_DRIVES + MAX_RAIDMAP_VIEWS];
	uint8_t fpPdIoTimeoutSec;
	uint8_t reserved2[7];
	MR_ARRAY_INFO arMapInfo[MAX_RAIDMAP_ARRAYS];
	MR_DEV_HANDLE_INFO devHndlInfo[MAX_RAIDMAP_PHYSICAL_DEVICES];
	MR_LD_SPAN_MAP ldSpanMap[1];
}	MR_FW_RAID_MAP;

typedef struct _MR_FW_RAID_MAP_EXT {
	/* Not used in new map */
	uint32_t reserved;

	union {
		struct {
			uint32_t maxLd;
			uint32_t maxSpanDepth;
			uint32_t maxRowSize;
			uint32_t maxPdCount;
			uint32_t maxArrays;
		}	validationInfo;
		uint32_t version[5];
		uint32_t reserved1[5];
	}	fw_raid_desc;

	uint8_t fpPdIoTimeoutSec;
	uint8_t reserved2[7];

	uint16_t ldCount;
	uint16_t arCount;
	uint16_t spanCount;
	uint16_t reserve3;

	MR_DEV_HANDLE_INFO devHndlInfo[MAX_RAIDMAP_PHYSICAL_DEVICES];
	uint8_t ldTgtIdToLd[MAX_LOGICAL_DRIVES_EXT];
	MR_ARRAY_INFO arMapInfo[MAX_API_ARRAYS_EXT];
	MR_LD_SPAN_MAP ldSpanMap[MAX_LOGICAL_DRIVES_EXT];
}	MR_FW_RAID_MAP_EXT;

typedef struct _MR_DRV_RAID_MAP {
	/*
	 * Total size of this structure, including this field. This field
	 * will be manupulated by driver for ext raid map, else pick the
	 * value from firmware raid map.
	 */
	uint32_t totalSize;

	union {
		struct {
			uint32_t maxLd;
			uint32_t maxSpanDepth;
			uint32_t maxRowSize;
			uint32_t maxPdCount;
			uint32_t maxArrays;
		}	validationInfo;
		uint32_t version[5];
		uint32_t reserved1[5];
	}	drv_raid_desc;

	/* timeout value used by driver in FP IOs */
	uint8_t fpPdIoTimeoutSec;
	uint8_t reserved2[7];

	uint16_t ldCount;
	uint16_t arCount;
	uint16_t spanCount;
	uint16_t reserve3;

	MR_DEV_HANDLE_INFO devHndlInfo[MAX_RAIDMAP_PHYSICAL_DEVICES_DYN];
	uint16_t ldTgtIdToLd[MAX_LOGICAL_DRIVES_DYN];
	MR_ARRAY_INFO arMapInfo[MAX_API_ARRAYS_DYN];
	MR_LD_SPAN_MAP ldSpanMap[1];

}	MR_DRV_RAID_MAP;

/*
 * Driver raid map size is same as raid map ext MR_DRV_RAID_MAP_ALL is
 * created to sync with old raid. And it is mainly for code re-use purpose.
 */

#pragma pack(1)
typedef struct _MR_DRV_RAID_MAP_ALL {
	MR_DRV_RAID_MAP raidMap;
	MR_LD_SPAN_MAP ldSpanMap[MAX_LOGICAL_DRIVES_DYN - 1];
}	MR_DRV_RAID_MAP_ALL;

#pragma pack()

typedef struct _LD_LOAD_BALANCE_INFO {
	uint8_t loadBalanceFlag;
	uint8_t reserved1;
	mrsas_atomic_t scsi_pending_cmds[MAX_PHYSICAL_DEVICES];
	uint64_t last_accessed_block[MAX_PHYSICAL_DEVICES];
}	LD_LOAD_BALANCE_INFO, *PLD_LOAD_BALANCE_INFO;

/* SPAN_SET is info caclulated from span info from Raid map per ld */
typedef struct _LD_SPAN_SET {
	uint64_t log_start_lba;
	uint64_t log_end_lba;
	uint64_t span_row_start;
	uint64_t span_row_end;
	uint64_t data_strip_start;
	uint64_t data_strip_end;
	uint64_t data_row_start;
	uint64_t data_row_end;
	uint8_t strip_offset[MAX_SPAN_DEPTH];
	uint32_t span_row_data_width;
	uint32_t diff;
	uint32_t reserved[2];
}	LD_SPAN_SET, *PLD_SPAN_SET;

typedef struct LOG_BLOCK_SPAN_INFO {
	LD_SPAN_SET span_set[MAX_SPAN_DEPTH];
}	LD_SPAN_INFO, *PLD_SPAN_INFO;

#pragma pack(1)
typedef struct _MR_FW_RAID_MAP_ALL {
	MR_FW_RAID_MAP raidMap;
	MR_LD_SPAN_MAP ldSpanMap[MAX_LOGICAL_DRIVES - 1];
}	MR_FW_RAID_MAP_ALL;

#pragma pack()

struct IO_REQUEST_INFO {
	uint64_t ldStartBlock;
	uint32_t numBlocks;
	uint16_t ldTgtId;
	uint8_t isRead;
	uint16_t devHandle;
	uint8_t pdInterface;
	uint64_t pdBlock;
	uint8_t fpOkForIo;
	uint8_t IoforUnevenSpan;
	uint8_t start_span;
	uint8_t reserved;
	uint64_t start_row;
	/* span[7:5], arm[4:0] */
	uint8_t span_arm;
	uint8_t pd_after_lb;
	boolean_t raCapable;
	uint16_t r1_alt_dev_handle;
};

/*
 * define MR_PD_CFG_SEQ structure for system PDs
 */
struct MR_PD_CFG_SEQ {
	uint16_t seqNum;
	uint16_t devHandle;
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint8_t tmCapable:1;
		uint8_t reserved:7;
#else
		uint8_t reserved:7;
		uint8_t tmCapable:1;
#endif
	} capability;
	uint8_t reserved;
	uint16_t pdTargetId;
} __packed;

struct MR_PD_CFG_SEQ_NUM_SYNC {
	uint32_t size;
	uint32_t count;
	struct MR_PD_CFG_SEQ seq[1];
} __packed;

typedef struct _STREAM_DETECT {
	uint64_t nextSeqLBA;
	struct megasas_cmd_fusion *first_cmd_fusion;
	struct megasas_cmd_fusion *last_cmd_fusion;
	uint32_t countCmdsInStream;
	uint16_t numSGEsInGroup;
	uint8_t isRead;
	uint8_t groupDepth;
	boolean_t groupFlush;
	uint8_t reserved[7];
} STREAM_DETECT, *PTR_STREAM_DETECT;

typedef struct _LD_STREAM_DETECT {
	boolean_t writeBack;
	boolean_t FPWriteEnabled;
	boolean_t membersSSDs;
	boolean_t fpCacheBypassCapable;
	uint32_t mruBitMap;
	volatile long iosToFware;
	volatile long writeBytesOutstanding;
	STREAM_DETECT streamTrack[MAX_STREAMS_TRACKED];
} LD_STREAM_DETECT, *PTR_LD_STREAM_DETECT;

typedef struct _MR_LD_TARGET_SYNC {
	uint8_t targetId;
	uint8_t reserved;
	uint16_t seqNum;
}	MR_LD_TARGET_SYNC;

/*
 * RAID Map descriptor Types.
 * Each element should uniquely idetify one data structure in the RAID map
 */
typedef enum _MR_RAID_MAP_DESC_TYPE {
	RAID_MAP_DESC_TYPE_DEVHDL_INFO = 0,	/* MR_DEV_HANDLE_INFO data */
	RAID_MAP_DESC_TYPE_TGTID_INFO = 1,	/* target to Ld num Index map */
	RAID_MAP_DESC_TYPE_ARRAY_INFO = 2,	/* MR_ARRAY_INFO data */
	RAID_MAP_DESC_TYPE_SPAN_INFO = 3,	/* MR_LD_SPAN_MAP data */
	RAID_MAP_DESC_TYPE_COUNT,
}	MR_RAID_MAP_DESC_TYPE;

/*
 * This table defines the offset, size and num elements  of each descriptor
 * type in the RAID Map buffer
 */
typedef struct _MR_RAID_MAP_DESC_TABLE {
	/* Raid map descriptor type */
	uint32_t	raidMapDescType;
	/* Offset into the RAID map buffer where descriptor data is saved */
	uint32_t	raidMapDescOffset;
	/* total size of the descriptor buffer */
	uint32_t	raidMapDescBufferSize;
	/* Number of elements contained in the descriptor buffer */
	uint32_t	raidMapDescElements;
}	MR_RAID_MAP_DESC_TABLE;

/*
 * Dynamic Raid Map Structure.
 */
typedef struct _MR_FW_RAID_MAP_DYNAMIC {
	uint32_t	raidMapSize;
	uint32_t	descTableOffset;
	uint32_t	descTableSize;
	uint32_t	descTableNumElements;
	uint64_t	PCIThresholdBandwidth;
	uint32_t	reserved2[3];

	uint8_t	fpPdIoTimeoutSec;
	uint8_t	reserved3[3];
	uint32_t	rmwFPSeqNum;
	uint16_t	ldCount;
	uint16_t	arCount;
	uint16_t	spanCount;
	uint16_t	reserved4[3];

	/*
	* The below structure of pointers is only to be used by the driver.
	* This is added in the API to reduce the amount of code changes needed in
	* the driver to support dynamic RAID map.
	* Firmware should not update these pointers while preparing the raid map
	*/
	union {
		struct {
			MR_DEV_HANDLE_INFO	*devHndlInfo;
			uint16_t			*ldTgtIdToLd;
			MR_ARRAY_INFO		*arMapInfo;
			MR_LD_SPAN_MAP		*ldSpanMap;
		} ptrStruct;
		uint64_t ptrStructureSize[RAID_MAP_DESC_TYPE_COUNT];
	} RaidMapDescPtrs;

	/*
	* RAID Map descriptor table defines the layout of data in the RAID Map.
	* The size of the descriptor table itself could change.
	*/

	/* Variable Size descriptor Table. */
	MR_RAID_MAP_DESC_TABLE raidMapDescTable[RAID_MAP_DESC_TYPE_COUNT];
	/* Variable Size buffer containing all data */
	uint32_t raidMapDescData[1];

}	MR_FW_RAID_MAP_DYNAMIC;

#define	IEEE_SGE_FLAGS_ADDR_MASK		(0x03)
#define	IEEE_SGE_FLAGS_SYSTEM_ADDR		(0x00)
#define	IEEE_SGE_FLAGS_IOCDDR_ADDR		(0x01)
#define	IEEE_SGE_FLAGS_IOCPLB_ADDR		(0x02)
#define	IEEE_SGE_FLAGS_IOCPLBNTA_ADDR	(0x03)
#define	IEEE_SGE_FLAGS_CHAIN_ELEMENT	(0x80)
#define	IEEE_SGE_FLAGS_END_OF_LIST		(0x40)

/* Few NVME flags defines*/
#define MPI2_SGE_FLAGS_SHIFT                (0x02)
#define IEEE_SGE_FLAGS_FORMAT_MASK          (0xC0)
#define IEEE_SGE_FLAGS_FORMAT_IEEE          (0x00)
#define IEEE_SGE_FLAGS_FORMAT_PQI           (0x01)
#define IEEE_SGE_FLAGS_FORMAT_NVME          (0x02)
#define IEEE_SGE_FLAGS_FORMAT_AHCI          (0x03)

#define MPI26_IEEE_SGE_FLAGS_NSF_MASK           (0x1C)
#define MPI26_IEEE_SGE_FLAGS_NSF_MPI_IEEE       (0x00)
#define MPI26_IEEE_SGE_FLAGS_NSF_PQI            (0x04)
#define MPI26_IEEE_SGE_FLAGS_NSF_NVME_PRP       (0x08)
#define MPI26_IEEE_SGE_FLAGS_NSF_AHCI_PRDT      (0x0C)
#define MPI26_IEEE_SGE_FLAGS_NSF_NVME_SGL       (0x10)

union desc_value {
	uint64_t word;
	struct {
		uint32_t low;
		uint32_t high;
	}	u;
};

/*******************************************************************
 * Temporary command
 ********************************************************************/
struct mrsas_tmp_dcmd {
	bus_dma_tag_t tmp_dcmd_tag;
	bus_dmamap_t tmp_dcmd_dmamap;
	void   *tmp_dcmd_mem;
	bus_addr_t tmp_dcmd_phys_addr;
};

#define	MR_MAX_RAID_MAP_SIZE_OFFSET_SHIFT  16
#define	MR_MAX_RAID_MAP_SIZE_MASK      0x1FF
#define	MR_MIN_MAP_SIZE                0x10000

/*******************************************************************
 * Register set, included legacy controllers 1068 and 1078,
 * structure extended for 1078 registers
 *******************************************************************/
#pragma pack(1)
typedef struct _mrsas_register_set {
	uint32_t doorbell;		/* 0000h */
	uint32_t fusion_seq_offset;	/* 0004h */
	uint32_t fusion_host_diag;	/* 0008h */
	uint32_t reserved_01;		/* 000Ch */

	uint32_t inbound_msg_0;	/* 0010h */
	uint32_t inbound_msg_1;	/* 0014h */
	uint32_t outbound_msg_0;	/* 0018h */
	uint32_t outbound_msg_1;	/* 001Ch */

	uint32_t inbound_doorbell;	/* 0020h */
	uint32_t inbound_intr_status;	/* 0024h */
	uint32_t inbound_intr_mask;	/* 0028h */

	uint32_t outbound_doorbell;	/* 002Ch */
	uint32_t outbound_intr_status;	/* 0030h */
	uint32_t outbound_intr_mask;	/* 0034h */

	uint32_t reserved_1[2];	/* 0038h */

	uint32_t inbound_queue_port;	/* 0040h */
	uint32_t outbound_queue_port;	/* 0044h */

	uint32_t reserved_2[9];	/* 0048h */
	uint32_t reply_post_host_index;/* 006Ch */
	uint32_t reserved_2_2[12];	/* 0070h */

	uint32_t outbound_doorbell_clear;	/* 00A0h */

	uint32_t reserved_3[3];	/* 00A4h */

	uint32_t outbound_scratch_pad;	/* 00B0h */
	uint32_t outbound_scratch_pad_2;	/* 00B4h */
	uint32_t outbound_scratch_pad_3;	/* 00B8h */
	uint32_t outbound_scratch_pad_4;	/* 00BCh */

	uint32_t inbound_low_queue_port;	/* 00C0h */

	uint32_t inbound_high_queue_port;	/* 00C4h */

	uint32_t inbound_single_queue_port;	/* 00C8h */
	uint32_t res_6[11];		/* CCh */
	uint32_t host_diag;
	uint32_t seq_offset;
	uint32_t index_registers[807];	/* 00CCh */
}	mrsas_reg_set;

#pragma pack()

/*******************************************************************
 * Firmware Interface Defines
 *******************************************************************
 * MFI stands for MegaRAID SAS FW Interface. This is just a moniker
 * for protocol between the software and firmware. Commands are
 * issued using "message frames".
 ******************************************************************/
/*
 * FW posts its state in upper 4 bits of outbound_msg_0 register
 */
#define	MFI_STATE_MASK					0xF0000000
#define	MFI_STATE_UNDEFINED				0x00000000
#define	MFI_STATE_BB_INIT				0x10000000
#define	MFI_STATE_FW_INIT				0x40000000
#define	MFI_STATE_WAIT_HANDSHAKE		0x60000000
#define	MFI_STATE_FW_INIT_2				0x70000000
#define	MFI_STATE_DEVICE_SCAN			0x80000000
#define	MFI_STATE_BOOT_MESSAGE_PENDING	0x90000000
#define	MFI_STATE_FLUSH_CACHE			0xA0000000
#define	MFI_STATE_READY					0xB0000000
#define	MFI_STATE_OPERATIONAL			0xC0000000
#define	MFI_STATE_FAULT					0xF0000000
#define	MFI_RESET_REQUIRED				0x00000001
#define	MFI_RESET_ADAPTER				0x00000002
#define	MEGAMFI_FRAME_SIZE				64
#define	MRSAS_MFI_FRAME_SIZE			1024
#define	MRSAS_MFI_SENSE_SIZE			128

/*
 * During FW init, clear pending cmds & reset state using inbound_msg_0
 *
 * ABORT        : Abort all pending cmds READY        : Move from OPERATIONAL to
 * READY state; discard queue info MFIMODE      : Discard (possible) low MFA
 * posted in 64-bit mode (??) CLR_HANDSHAKE: FW is waiting for HANDSHAKE from
 * BIOS or Driver HOTPLUG      : Resume from Hotplug MFI_STOP_ADP : Send
 * signal to FW to stop processing
 */

#define	WRITE_SEQUENCE_OFFSET		(0x0000000FC)
#define	HOST_DIAGNOSTIC_OFFSET		(0x000000F8)
#define	DIAG_WRITE_ENABLE			(0x00000080)
#define	DIAG_RESET_ADAPTER			(0x00000004)

#define	MFI_ADP_RESET				0x00000040
#define	MFI_INIT_ABORT				0x00000001
#define	MFI_INIT_READY				0x00000002
#define	MFI_INIT_MFIMODE			0x00000004
#define	MFI_INIT_CLEAR_HANDSHAKE	0x00000008
#define	MFI_INIT_HOTPLUG			0x00000010
#define	MFI_STOP_ADP				0x00000020
#define	MFI_RESET_FLAGS				MFI_INIT_READY|		\
									MFI_INIT_MFIMODE|	\
									MFI_INIT_ABORT

/*
 * MFI frame flags
 */
#define	MFI_FRAME_POST_IN_REPLY_QUEUE			0x0000
#define	MFI_FRAME_DONT_POST_IN_REPLY_QUEUE		0x0001
#define	MFI_FRAME_SGL32							0x0000
#define	MFI_FRAME_SGL64							0x0002
#define	MFI_FRAME_SENSE32						0x0000
#define	MFI_FRAME_SENSE64						0x0004
#define	MFI_FRAME_DIR_NONE						0x0000
#define	MFI_FRAME_DIR_WRITE						0x0008
#define	MFI_FRAME_DIR_READ						0x0010
#define	MFI_FRAME_DIR_BOTH						0x0018
#define	MFI_FRAME_IEEE							0x0020

/*
 * Definition for cmd_status
 */
#define	MFI_CMD_STATUS_POLL_MODE				0xFF

/*
 * MFI command opcodes
 */
#define	MFI_CMD_INIT							0x00
#define	MFI_CMD_LD_READ							0x01
#define	MFI_CMD_LD_WRITE						0x02
#define	MFI_CMD_LD_SCSI_IO						0x03
#define	MFI_CMD_PD_SCSI_IO						0x04
#define	MFI_CMD_DCMD							0x05
#define	MFI_CMD_ABORT							0x06
#define	MFI_CMD_SMP								0x07
#define	MFI_CMD_STP								0x08
#define	MFI_CMD_INVALID							0xff

#define	MR_DCMD_CTRL_GET_INFO					0x01010000
#define	MR_DCMD_LD_GET_LIST						0x03010000
#define	MR_DCMD_CTRL_CACHE_FLUSH				0x01101000
#define	MR_FLUSH_CTRL_CACHE						0x01
#define	MR_FLUSH_DISK_CACHE						0x02

#define	MR_DCMD_CTRL_SHUTDOWN					0x01050000
#define	MR_DCMD_HIBERNATE_SHUTDOWN				0x01060000
#define	MR_ENABLE_DRIVE_SPINDOWN				0x01

#define	MR_DCMD_CTRL_EVENT_GET_INFO				0x01040100
#define	MR_DCMD_CTRL_EVENT_GET					0x01040300
#define	MR_DCMD_CTRL_EVENT_WAIT					0x01040500
#define	MR_DCMD_LD_GET_PROPERTIES				0x03030000

#define	MR_DCMD_CLUSTER							0x08000000
#define	MR_DCMD_CLUSTER_RESET_ALL				0x08010100
#define	MR_DCMD_CLUSTER_RESET_LD				0x08010200
#define	MR_DCMD_PD_LIST_QUERY					0x02010100

#define	MR_DCMD_CTRL_MISC_CPX					0x0100e200
#define	MR_DCMD_CTRL_MISC_CPX_INIT_DATA_GET		0x0100e201
#define	MR_DCMD_CTRL_MISC_CPX_QUEUE_DATA		0x0100e202
#define	MR_DCMD_CTRL_MISC_CPX_UNREGISTER		0x0100e203
#define	MAX_MR_ROW_SIZE							32
#define	MR_CPX_DIR_WRITE						1
#define	MR_CPX_DIR_READ							0
#define	MR_CPX_VERSION							1

#define	MR_DCMD_CTRL_IO_METRICS_GET				0x01170200

#define	MR_EVT_CFG_CLEARED						0x0004

#define	MR_EVT_LD_STATE_CHANGE					0x0051
#define	MR_EVT_PD_INSERTED						0x005b
#define	MR_EVT_PD_REMOVED						0x0070
#define	MR_EVT_LD_CREATED						0x008a
#define	MR_EVT_LD_DELETED						0x008b
#define	MR_EVT_FOREIGN_CFG_IMPORTED				0x00db
#define	MR_EVT_LD_OFFLINE						0x00fc
#define	MR_EVT_CTRL_HOST_BUS_SCAN_REQUESTED		0x0152
#define	MR_EVT_CTRL_PERF_COLLECTION				0x017e

/*
 * MFI command completion codes
 */
enum MFI_STAT {
	MFI_STAT_OK = 0x00,
	MFI_STAT_INVALID_CMD = 0x01,
	MFI_STAT_INVALID_DCMD = 0x02,
	MFI_STAT_INVALID_PARAMETER = 0x03,
	MFI_STAT_INVALID_SEQUENCE_NUMBER = 0x04,
	MFI_STAT_ABORT_NOT_POSSIBLE = 0x05,
	MFI_STAT_APP_HOST_CODE_NOT_FOUND = 0x06,
	MFI_STAT_APP_IN_USE = 0x07,
	MFI_STAT_APP_NOT_INITIALIZED = 0x08,
	MFI_STAT_ARRAY_INDEX_INVALID = 0x09,
	MFI_STAT_ARRAY_ROW_NOT_EMPTY = 0x0a,
	MFI_STAT_CONFIG_RESOURCE_CONFLICT = 0x0b,
	MFI_STAT_DEVICE_NOT_FOUND = 0x0c,
	MFI_STAT_DRIVE_TOO_SMALL = 0x0d,
	MFI_STAT_FLASH_ALLOC_FAIL = 0x0e,
	MFI_STAT_FLASH_BUSY = 0x0f,
	MFI_STAT_FLASH_ERROR = 0x10,
	MFI_STAT_FLASH_IMAGE_BAD = 0x11,
	MFI_STAT_FLASH_IMAGE_INCOMPLETE = 0x12,
	MFI_STAT_FLASH_NOT_OPEN = 0x13,
	MFI_STAT_FLASH_NOT_STARTED = 0x14,
	MFI_STAT_FLUSH_FAILED = 0x15,
	MFI_STAT_HOST_CODE_NOT_FOUNT = 0x16,
	MFI_STAT_LD_CC_IN_PROGRESS = 0x17,
	MFI_STAT_LD_INIT_IN_PROGRESS = 0x18,
	MFI_STAT_LD_LBA_OUT_OF_RANGE = 0x19,
	MFI_STAT_LD_MAX_CONFIGURED = 0x1a,
	MFI_STAT_LD_NOT_OPTIMAL = 0x1b,
	MFI_STAT_LD_RBLD_IN_PROGRESS = 0x1c,
	MFI_STAT_LD_RECON_IN_PROGRESS = 0x1d,
	MFI_STAT_LD_WRONG_RAID_LEVEL = 0x1e,
	MFI_STAT_MAX_SPARES_EXCEEDED = 0x1f,
	MFI_STAT_MEMORY_NOT_AVAILABLE = 0x20,
	MFI_STAT_MFC_HW_ERROR = 0x21,
	MFI_STAT_NO_HW_PRESENT = 0x22,
	MFI_STAT_NOT_FOUND = 0x23,
	MFI_STAT_NOT_IN_ENCL = 0x24,
	MFI_STAT_PD_CLEAR_IN_PROGRESS = 0x25,
	MFI_STAT_PD_TYPE_WRONG = 0x26,
	MFI_STAT_PR_DISABLED = 0x27,
	MFI_STAT_ROW_INDEX_INVALID = 0x28,
	MFI_STAT_SAS_CONFIG_INVALID_ACTION = 0x29,
	MFI_STAT_SAS_CONFIG_INVALID_DATA = 0x2a,
	MFI_STAT_SAS_CONFIG_INVALID_PAGE = 0x2b,
	MFI_STAT_SAS_CONFIG_INVALID_TYPE = 0x2c,
	MFI_STAT_SCSI_DONE_WITH_ERROR = 0x2d,
	MFI_STAT_SCSI_IO_FAILED = 0x2e,
	MFI_STAT_SCSI_RESERVATION_CONFLICT = 0x2f,
	MFI_STAT_SHUTDOWN_FAILED = 0x30,
	MFI_STAT_TIME_NOT_SET = 0x31,
	MFI_STAT_WRONG_STATE = 0x32,
	MFI_STAT_LD_OFFLINE = 0x33,
	MFI_STAT_PEER_NOTIFICATION_REJECTED = 0x34,
	MFI_STAT_PEER_NOTIFICATION_FAILED = 0x35,
	MFI_STAT_RESERVATION_IN_PROGRESS = 0x36,
	MFI_STAT_I2C_ERRORS_DETECTED = 0x37,
	MFI_STAT_PCI_ERRORS_DETECTED = 0x38,
	MFI_STAT_CONFIG_SEQ_MISMATCH = 0x67,

	MFI_STAT_INVALID_STATUS = 0xFF
};

/*
 * Number of mailbox bytes in DCMD message frame
 */
#define	MFI_MBOX_SIZE	12

enum MR_EVT_CLASS {
	MR_EVT_CLASS_DEBUG = -2,
	MR_EVT_CLASS_PROGRESS = -1,
	MR_EVT_CLASS_INFO = 0,
	MR_EVT_CLASS_WARNING = 1,
	MR_EVT_CLASS_CRITICAL = 2,
	MR_EVT_CLASS_FATAL = 3,
	MR_EVT_CLASS_DEAD = 4,

};

enum MR_EVT_LOCALE {
	MR_EVT_LOCALE_LD = 0x0001,
	MR_EVT_LOCALE_PD = 0x0002,
	MR_EVT_LOCALE_ENCL = 0x0004,
	MR_EVT_LOCALE_BBU = 0x0008,
	MR_EVT_LOCALE_SAS = 0x0010,
	MR_EVT_LOCALE_CTRL = 0x0020,
	MR_EVT_LOCALE_CONFIG = 0x0040,
	MR_EVT_LOCALE_CLUSTER = 0x0080,
	MR_EVT_LOCALE_ALL = 0xffff,

};

enum MR_EVT_ARGS {
	MR_EVT_ARGS_NONE,
	MR_EVT_ARGS_CDB_SENSE,
	MR_EVT_ARGS_LD,
	MR_EVT_ARGS_LD_COUNT,
	MR_EVT_ARGS_LD_LBA,
	MR_EVT_ARGS_LD_OWNER,
	MR_EVT_ARGS_LD_LBA_PD_LBA,
	MR_EVT_ARGS_LD_PROG,
	MR_EVT_ARGS_LD_STATE,
	MR_EVT_ARGS_LD_STRIP,
	MR_EVT_ARGS_PD,
	MR_EVT_ARGS_PD_ERR,
	MR_EVT_ARGS_PD_LBA,
	MR_EVT_ARGS_PD_LBA_LD,
	MR_EVT_ARGS_PD_PROG,
	MR_EVT_ARGS_PD_STATE,
	MR_EVT_ARGS_PCI,
	MR_EVT_ARGS_RATE,
	MR_EVT_ARGS_STR,
	MR_EVT_ARGS_TIME,
	MR_EVT_ARGS_ECC,
	MR_EVT_ARGS_LD_PROP,
	MR_EVT_ARGS_PD_SPARE,
	MR_EVT_ARGS_PD_INDEX,
	MR_EVT_ARGS_DIAG_PASS,
	MR_EVT_ARGS_DIAG_FAIL,
	MR_EVT_ARGS_PD_LBA_LBA,
	MR_EVT_ARGS_PORT_PHY,
	MR_EVT_ARGS_PD_MISSING,
	MR_EVT_ARGS_PD_ADDRESS,
	MR_EVT_ARGS_BITMAP,
	MR_EVT_ARGS_CONNECTOR,
	MR_EVT_ARGS_PD_PD,
	MR_EVT_ARGS_PD_FRU,
	MR_EVT_ARGS_PD_PATHINFO,
	MR_EVT_ARGS_PD_POWER_STATE,
	MR_EVT_ARGS_GENERIC,
};

/*
 * Thunderbolt (and later) Defines
 */
#define	MEGASAS_CHAIN_FRAME_SZ_MIN					1024
#define	MFI_FUSION_ENABLE_INTERRUPT_MASK			(0x00000009)
#define	MRSAS_MPI2_RAID_DEFAULT_IO_FRAME_SIZE		256
#define	MRSAS_MPI2_FUNCTION_PASSTHRU_IO_REQUEST		0xF0
#define	MRSAS_MPI2_FUNCTION_LD_IO_REQUEST			0xF1
#define	MRSAS_LOAD_BALANCE_FLAG						0x1
#define	MRSAS_DCMD_MBOX_PEND_FLAG					0x1
#define	HOST_DIAG_WRITE_ENABLE						0x80
#define	HOST_DIAG_RESET_ADAPTER						0x4
#define	MRSAS_TBOLT_MAX_RESET_TRIES					3
#define MRSAS_MAX_MFI_CMDS                          16
#define MRSAS_MAX_IOCTL_CMDS                        3

/*
 * Invader Defines
 */
#define	MPI2_TYPE_CUDA								0x2
#define	MPI25_SAS_DEVICE0_FLAGS_ENABLED_FAST_PATH	0x4000
#define	MR_RL_FLAGS_GRANT_DESTINATION_CPU0			0x00
#define	MR_RL_FLAGS_GRANT_DESTINATION_CPU1			0x10
#define	MR_RL_FLAGS_GRANT_DESTINATION_CUDA			0x80
#define	MR_RL_FLAGS_SEQ_NUM_ENABLE					0x8
#define	MR_RL_WRITE_THROUGH_MODE					0x00
#define	MR_RL_WRITE_BACK_MODE						0x01

/*
 * T10 PI defines
 */
#define	MR_PROT_INFO_TYPE_CONTROLLER				0x8
#define	MRSAS_SCSI_VARIABLE_LENGTH_CMD				0x7f
#define	MRSAS_SCSI_SERVICE_ACTION_READ32			0x9
#define	MRSAS_SCSI_SERVICE_ACTION_WRITE32			0xB
#define	MRSAS_SCSI_ADDL_CDB_LEN						0x18
#define	MRSAS_RD_WR_PROTECT_CHECK_ALL				0x20
#define	MRSAS_RD_WR_PROTECT_CHECK_NONE				0x60
#define	MRSAS_SCSIBLOCKSIZE							512

/*
 * Raid context flags
 */
#define	MR_RAID_CTX_RAID_FLAGS_IO_SUB_TYPE_SHIFT	0x4
#define	MR_RAID_CTX_RAID_FLAGS_IO_SUB_TYPE_MASK		0x30
typedef enum MR_RAID_FLAGS_IO_SUB_TYPE {
	MR_RAID_FLAGS_IO_SUB_TYPE_NONE = 0,
	MR_RAID_FLAGS_IO_SUB_TYPE_SYSTEM_PD = 1,
	MR_RAID_FLAGS_IO_SUB_TYPE_RMW_DATA = 2,
	MR_RAID_FLAGS_IO_SUB_TYPE_RMW_P = 3,
	MR_RAID_FLAGS_IO_SUB_TYPE_RMW_Q = 4,
	MR_RAID_FLAGS_IO_SUB_TYPE_CACHE_BYPASS = 6,
	MR_RAID_FLAGS_IO_SUB_TYPE_LDIO_BW_LIMIT = 7
} MR_RAID_FLAGS_IO_SUB_TYPE;
/*
 * Request descriptor types
 */
#define	MRSAS_REQ_DESCRIPT_FLAGS_LD_IO		0x7
#define	MRSAS_REQ_DESCRIPT_FLAGS_MFA		0x1
#define	MRSAS_REQ_DESCRIPT_FLAGS_NO_LOCK	0x2
#define	MRSAS_REQ_DESCRIPT_FLAGS_TYPE_SHIFT	1
#define	MRSAS_FP_CMD_LEN					16
#define	MRSAS_FUSION_IN_RESET				0

#define	RAID_CTX_SPANARM_ARM_SHIFT			(0)
#define	RAID_CTX_SPANARM_ARM_MASK			(0x1f)
#define	RAID_CTX_SPANARM_SPAN_SHIFT			(5)
#define	RAID_CTX_SPANARM_SPAN_MASK			(0xE0)

/*
 * Define region lock types
 */
typedef enum _REGION_TYPE {
	REGION_TYPE_UNUSED = 0,
	REGION_TYPE_SHARED_READ = 1,
	REGION_TYPE_SHARED_WRITE = 2,
	REGION_TYPE_EXCLUSIVE = 3,
}	REGION_TYPE;

/*
 * SCSI-CAM Related Defines
 */
#define	MRSAS_SCSI_MAX_LUNS				0
#define	MRSAS_SCSI_INITIATOR_ID			255
#define	MRSAS_SCSI_MAX_CMDS				8
#define	MRSAS_SCSI_MAX_CDB_LEN			16
#define	MRSAS_SCSI_SENSE_BUFFERSIZE		96
#define	MRSAS_INTERNAL_CMDS				32
#define	MRSAS_FUSION_INT_CMDS			8

#define	MEGASAS_MAX_CHAIN_SIZE_UNITS_MASK	0x400000
#define	MEGASAS_MAX_CHAIN_SIZE_MASK		0x3E0
#define	MEGASAS_256K_IO					128
#define	MEGASAS_1MB_IO					(MEGASAS_256K_IO * 4)

/* Request types */
#define	MRSAS_REQ_TYPE_INTERNAL_CMD		0x0
#define	MRSAS_REQ_TYPE_AEN_FETCH		0x1
#define	MRSAS_REQ_TYPE_PASSTHRU			0x2
#define	MRSAS_REQ_TYPE_GETSET_PARAM		0x3
#define	MRSAS_REQ_TYPE_SCSI_IO			0x4

/* Request states */
#define	MRSAS_REQ_STATE_FREE			0
#define	MRSAS_REQ_STATE_BUSY			1
#define	MRSAS_REQ_STATE_TRAN			2
#define	MRSAS_REQ_STATE_COMPLETE		3

typedef enum _MR_SCSI_CMD_TYPE {
	READ_WRITE_LDIO = 0,
	NON_READ_WRITE_LDIO = 1,
	READ_WRITE_SYSPDIO = 2,
	NON_READ_WRITE_SYSPDIO = 3,
}	MR_SCSI_CMD_TYPE;

enum mrsas_req_flags {
	MRSAS_DIR_UNKNOWN = 0x1,
	MRSAS_DIR_IN = 0x2,
	MRSAS_DIR_OUT = 0x4,
	MRSAS_DIR_NONE = 0x8,
};

/*
 * Adapter Reset States
 */
enum {
	MRSAS_HBA_OPERATIONAL = 0,
	MRSAS_ADPRESET_SM_INFAULT = 1,
	MRSAS_ADPRESET_SM_FW_RESET_SUCCESS = 2,
	MRSAS_ADPRESET_SM_OPERATIONAL = 3,
	MRSAS_HW_CRITICAL_ERROR = 4,
	MRSAS_ADPRESET_INPROG_SIGN = 0xDEADDEAD,
};

/*
 * MPT Command Structure
 */
struct mrsas_mpt_cmd {
	MRSAS_RAID_SCSI_IO_REQUEST *io_request;
	bus_addr_t io_request_phys_addr;
	MPI2_SGE_IO_UNION *chain_frame;
	bus_addr_t chain_frame_phys_addr;
	uint32_t sge_count;
	uint8_t *sense;
	bus_addr_t sense_phys_addr;
	uint8_t retry_for_fw_reset;
	MRSAS_REQUEST_DESCRIPTOR_UNION *request_desc;
	uint32_t sync_cmd_idx;
	uint32_t index;
	uint8_t flags;
	uint8_t pd_r1_lb;
	uint8_t load_balance;
	bus_size_t length;
	uint32_t error_code;
	bus_dmamap_t data_dmamap;
	void   *data;
	union ccb *ccb_ptr;
	struct callout cm_callout;
	struct mrsas_softc *sc;
	boolean_t tmCapable;
	uint16_t r1_alt_dev_handle;
	boolean_t cmd_completed;
	struct mrsas_mpt_cmd *peer_cmd;
	bool	callout_owner;
	TAILQ_ENTRY(mrsas_mpt_cmd) next;
	uint8_t pdInterface;
};

/*
 * MFI Command Structure
 */
struct mrsas_mfi_cmd {
	union mrsas_frame *frame;
	bus_dmamap_t frame_dmamap;
	void   *frame_mem;
	bus_addr_t frame_phys_addr;
	uint8_t *sense;
	bus_dmamap_t sense_dmamap;
	void   *sense_mem;
	bus_addr_t sense_phys_addr;
	uint32_t index;
	uint8_t sync_cmd;
	uint8_t cmd_status;
	uint8_t abort_aen;
	uint8_t retry_for_fw_reset;
	struct mrsas_softc *sc;
	union ccb *ccb_ptr;
	union {
		struct {
			uint16_t smid;
			uint16_t resvd;
		}	context;
		uint32_t frame_count;
	}	cmd_id;
	TAILQ_ENTRY(mrsas_mfi_cmd) next;
};

/*
 * define constants for device list query options
 */
enum MR_PD_QUERY_TYPE {
	MR_PD_QUERY_TYPE_ALL = 0,
	MR_PD_QUERY_TYPE_STATE = 1,
	MR_PD_QUERY_TYPE_POWER_STATE = 2,
	MR_PD_QUERY_TYPE_MEDIA_TYPE = 3,
	MR_PD_QUERY_TYPE_SPEED = 4,
	MR_PD_QUERY_TYPE_EXPOSED_TO_HOST = 5,
};

#define	MR_EVT_CFG_CLEARED						0x0004
#define	MR_EVT_LD_STATE_CHANGE					0x0051
#define	MR_EVT_PD_INSERTED						0x005b
#define	MR_EVT_PD_REMOVED						0x0070
#define	MR_EVT_LD_CREATED						0x008a
#define	MR_EVT_LD_DELETED						0x008b
#define	MR_EVT_FOREIGN_CFG_IMPORTED				0x00db
#define	MR_EVT_LD_OFFLINE						0x00fc
#define	MR_EVT_CTRL_PROP_CHANGED				0x012f
#define	MR_EVT_CTRL_HOST_BUS_SCAN_REQUESTED		0x0152

enum MR_PD_STATE {
	MR_PD_STATE_UNCONFIGURED_GOOD = 0x00,
	MR_PD_STATE_UNCONFIGURED_BAD = 0x01,
	MR_PD_STATE_HOT_SPARE = 0x02,
	MR_PD_STATE_OFFLINE = 0x10,
	MR_PD_STATE_FAILED = 0x11,
	MR_PD_STATE_REBUILD = 0x14,
	MR_PD_STATE_ONLINE = 0x18,
	MR_PD_STATE_COPYBACK = 0x20,
	MR_PD_STATE_SYSTEM = 0x40,
};

/*
 * defines the physical drive address structure
 */
#pragma pack(1)
struct MR_PD_ADDRESS {
	uint16_t deviceId;
	uint16_t enclDeviceId;

	union {
		struct {
			uint8_t enclIndex;
			uint8_t slotNumber;
		}	mrPdAddress;
		struct {
			uint8_t enclPosition;
			uint8_t enclConnectorIndex;
		}	mrEnclAddress;
	}	u1;
	uint8_t scsiDevType;
	union {
		uint8_t connectedPortBitmap;
		uint8_t connectedPortNumbers;
	}	u2;
	uint64_t sasAddr[2];
};

#pragma pack()

/*
 * defines the physical drive list structure
 */
#pragma pack(1)
struct MR_PD_LIST {
	uint32_t size;
	uint32_t count;
	struct MR_PD_ADDRESS addr[1];
};

#pragma pack()

#pragma pack(1)
struct mrsas_pd_list {
	uint16_t tid;
	uint8_t driveType;
	uint8_t driveState;
};

#pragma pack()

/*
 * defines the logical drive reference structure
 */
typedef union _MR_LD_REF {
	struct {
		uint8_t targetId;
		uint8_t reserved;
		uint16_t seqNum;
	}	ld_context;
	uint32_t ref;
}	MR_LD_REF;

/*
 * defines the logical drive list structure
 */
#pragma pack(1)
struct MR_LD_LIST {
	uint32_t ldCount;
	uint32_t reserved;
	struct {
		MR_LD_REF ref;
		uint8_t state;
		uint8_t reserved[3];
		uint64_t size;
	}	ldList[MAX_LOGICAL_DRIVES_EXT];
};

#pragma pack()

/*
 * SAS controller properties
 */
#pragma pack(1)
struct mrsas_ctrl_prop {
	uint16_t seq_num;
	uint16_t pred_fail_poll_interval;
	uint16_t intr_throttle_count;
	uint16_t intr_throttle_timeouts;
	uint8_t rebuild_rate;
	uint8_t patrol_read_rate;
	uint8_t bgi_rate;
	uint8_t cc_rate;
	uint8_t recon_rate;
	uint8_t cache_flush_interval;
	uint8_t spinup_drv_count;
	uint8_t spinup_delay;
	uint8_t cluster_enable;
	uint8_t coercion_mode;
	uint8_t alarm_enable;
	uint8_t disable_auto_rebuild;
	uint8_t disable_battery_warn;
	uint8_t ecc_bucket_size;
	uint16_t ecc_bucket_leak_rate;
	uint8_t restore_hotspare_on_insertion;
	uint8_t expose_encl_devices;
	uint8_t maintainPdFailHistory;
	uint8_t disallowHostRequestReordering;
	uint8_t abortCCOnError;
	uint8_t loadBalanceMode;
	uint8_t disableAutoDetectBackplane;
	uint8_t snapVDSpace;
	/*
	 * Add properties that can be controlled by a bit in the following
	 * structure.
	 */
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t copyBackDisabled:1;
		uint32_t SMARTerEnabled:1;
		uint32_t prCorrectUnconfiguredAreas:1;
		uint32_t useFdeOnly:1;
		uint32_t disableNCQ:1;
		uint32_t SSDSMARTerEnabled:1;
		uint32_t SSDPatrolReadEnabled:1;
		uint32_t enableSpinDownUnconfigured:1;
		uint32_t autoEnhancedImport:1;
		uint32_t enableSecretKeyControl:1;
		uint32_t disableOnlineCtrlReset:1;
		uint32_t allowBootWithPinnedCache:1;
		uint32_t disableSpinDownHS:1;
		uint32_t enableJBOD:1;
		uint32_t disableCacheBypass:1;
		uint32_t useDiskActivityForLocate:1;
		uint32_t enablePI:1;
		uint32_t preventPIImport:1;
		uint32_t useGlobalSparesForEmergency:1;
		uint32_t useUnconfGoodForEmergency:1;
		uint32_t useEmergencySparesforSMARTer:1;
		uint32_t forceSGPIOForQuadOnly:1;
		uint32_t enableConfigAutoBalance:1;
		uint32_t enableVirtualCache:1;
		uint32_t enableAutoLockRecovery:1;
		uint32_t disableImmediateIO:1;
		uint32_t disableT10RebuildAssist:1;
		uint32_t ignore64ldRestriction:1;
		uint32_t enableSwZone:1;
		uint32_t limitMaxRateSATA3G:1;
		uint32_t reserved:2;
#else
		uint32_t reserved:2;
		uint32_t limitMaxRateSATA3G:1;
		uint32_t enableSwZone:1;
		uint32_t ignore64ldRestriction:1;
		uint32_t disableT10RebuildAssist:1;
		uint32_t disableImmediateIO:1;
		uint32_t enableAutoLockRecovery:1;
		uint32_t enableVirtualCache:1;
		uint32_t enableConfigAutoBalance:1;
		uint32_t forceSGPIOForQuadOnly:1;
		uint32_t useEmergencySparesforSMARTer:1;
		uint32_t useUnconfGoodForEmergency:1;
		uint32_t useGlobalSparesForEmergency:1;
		uint32_t preventPIImport:1;
		uint32_t enablePI:1;
		uint32_t useDiskActivityForLocate:1;
		uint32_t disableCacheBypass:1;
		uint32_t enableJBOD:1;
		uint32_t disableSpinDownHS:1;
		uint32_t allowBootWithPinnedCache:1;
		uint32_t disableOnlineCtrlReset:1;
		uint32_t enableSecretKeyControl:1;
		uint32_t autoEnhancedImport:1;
		uint32_t enableSpinDownUnconfigured:1;
		uint32_t SSDPatrolReadEnabled:1;
		uint32_t SSDSMARTerEnabled:1;
		uint32_t disableNCQ:1;
		uint32_t useFdeOnly:1;
		uint32_t prCorrectUnconfiguredAreas:1;
		uint32_t SMARTerEnabled:1;
		uint32_t copyBackDisabled:1;
#endif
	}	OnOffProperties;
	uint8_t autoSnapVDSpace;
	uint8_t viewSpace;
	uint16_t spinDownTime;
	uint8_t reserved[24];

};

#pragma pack()

/*
 * SAS controller information
 */
struct mrsas_ctrl_info {
	/*
	 * PCI device information
	 */
	struct {
		uint16_t vendor_id;
		uint16_t device_id;
		uint16_t sub_vendor_id;
		uint16_t sub_device_id;
		uint8_t reserved[24];
	} __packed pci;
	/*
	 * Host interface information
	 */
	struct {
		uint8_t PCIX:1;
		uint8_t PCIE:1;
		uint8_t iSCSI:1;
		uint8_t SAS_3G:1;
		uint8_t reserved_0:4;
		uint8_t reserved_1[6];
		uint8_t port_count;
		uint64_t port_addr[8];
	} __packed host_interface;
	/*
	 * Device (backend) interface information
	 */
	struct {
		uint8_t SPI:1;
		uint8_t SAS_3G:1;
		uint8_t SATA_1_5G:1;
		uint8_t SATA_3G:1;
		uint8_t reserved_0:4;
		uint8_t reserved_1[6];
		uint8_t port_count;
		uint64_t port_addr[8];
	} __packed device_interface;

	uint32_t image_check_word;
	uint32_t image_component_count;

	struct {
		char	name[8];
		char	version[32];
		char	build_date[16];
		char	built_time[16];
	} __packed image_component[8];

	uint32_t pending_image_component_count;

	struct {
		char	name[8];
		char	version[32];
		char	build_date[16];
		char	build_time[16];
	} __packed pending_image_component[8];

	uint8_t max_arms;
	uint8_t max_spans;
	uint8_t max_arrays;
	uint8_t max_lds;
	char	product_name[80];
	char	serial_no[32];

	/*
	 * Other physical/controller/operation information. Indicates the
	 * presence of the hardware
	 */
	struct {
		uint32_t bbu:1;
		uint32_t alarm:1;
		uint32_t nvram:1;
		uint32_t uart:1;
		uint32_t reserved:28;
	} __packed hw_present;

	uint32_t current_fw_time;

	/*
	 * Maximum data transfer sizes
	 */
	uint16_t max_concurrent_cmds;
	uint16_t max_sge_count;
	uint32_t max_request_size;

	/*
	 * Logical and physical device counts
	 */
	uint16_t ld_present_count;
	uint16_t ld_degraded_count;
	uint16_t ld_offline_count;

	uint16_t pd_present_count;
	uint16_t pd_disk_present_count;
	uint16_t pd_disk_pred_failure_count;
	uint16_t pd_disk_failed_count;

	/*
	 * Memory size information
	 */
	uint16_t nvram_size;
	uint16_t memory_size;
	uint16_t flash_size;

	/*
	 * Error counters
	 */
	uint16_t mem_correctable_error_count;
	uint16_t mem_uncorrectable_error_count;

	/*
	 * Cluster information
	 */
	uint8_t cluster_permitted;
	uint8_t cluster_active;

	/*
	 * Additional max data transfer sizes
	 */
	uint16_t max_strips_per_io;

	/*
	 * Controller capabilities structures
	 */
	struct {
		uint32_t raid_level_0:1;
		uint32_t raid_level_1:1;
		uint32_t raid_level_5:1;
		uint32_t raid_level_1E:1;
		uint32_t raid_level_6:1;
		uint32_t reserved:27;
	} __packed raid_levels;

	struct {
		uint32_t rbld_rate:1;
		uint32_t cc_rate:1;
		uint32_t bgi_rate:1;
		uint32_t recon_rate:1;
		uint32_t patrol_rate:1;
		uint32_t alarm_control:1;
		uint32_t cluster_supported:1;
		uint32_t bbu:1;
		uint32_t spanning_allowed:1;
		uint32_t dedicated_hotspares:1;
		uint32_t revertible_hotspares:1;
		uint32_t foreign_config_import:1;
		uint32_t self_diagnostic:1;
		uint32_t mixed_redundancy_arr:1;
		uint32_t global_hot_spares:1;
		uint32_t reserved:17;
	} __packed adapter_operations;

	struct {
		uint32_t read_policy:1;
		uint32_t write_policy:1;
		uint32_t io_policy:1;
		uint32_t access_policy:1;
		uint32_t disk_cache_policy:1;
		uint32_t reserved:27;
	} __packed ld_operations;

	struct {
		uint8_t min;
		uint8_t max;
		uint8_t reserved[2];
	} __packed stripe_sz_ops;

	struct {
		uint32_t force_online:1;
		uint32_t force_offline:1;
		uint32_t force_rebuild:1;
		uint32_t reserved:29;
	} __packed pd_operations;

	struct {
		uint32_t ctrl_supports_sas:1;
		uint32_t ctrl_supports_sata:1;
		uint32_t allow_mix_in_encl:1;
		uint32_t allow_mix_in_ld:1;
		uint32_t allow_sata_in_cluster:1;
		uint32_t reserved:27;
	} __packed pd_mix_support;

	/*
	 * Define ECC single-bit-error bucket information
	 */
	uint8_t ecc_bucket_count;
	uint8_t reserved_2[11];

	/*
	 * Include the controller properties (changeable items)
	 */
	struct mrsas_ctrl_prop properties;

	/*
	 * Define FW pkg version (set in envt v'bles on OEM basis)
	 */
	char	package_version[0x60];

	uint64_t deviceInterfacePortAddr2[8];
	uint8_t reserved3[128];

	struct {
		uint16_t minPdRaidLevel_0:4;
		uint16_t maxPdRaidLevel_0:12;

		uint16_t minPdRaidLevel_1:4;
		uint16_t maxPdRaidLevel_1:12;

		uint16_t minPdRaidLevel_5:4;
		uint16_t maxPdRaidLevel_5:12;

		uint16_t minPdRaidLevel_1E:4;
		uint16_t maxPdRaidLevel_1E:12;

		uint16_t minPdRaidLevel_6:4;
		uint16_t maxPdRaidLevel_6:12;

		uint16_t minPdRaidLevel_10:4;
		uint16_t maxPdRaidLevel_10:12;

		uint16_t minPdRaidLevel_50:4;
		uint16_t maxPdRaidLevel_50:12;

		uint16_t minPdRaidLevel_60:4;
		uint16_t maxPdRaidLevel_60:12;

		uint16_t minPdRaidLevel_1E_RLQ0:4;
		uint16_t maxPdRaidLevel_1E_RLQ0:12;

		uint16_t minPdRaidLevel_1E0_RLQ0:4;
		uint16_t maxPdRaidLevel_1E0_RLQ0:12;

		uint16_t reserved[6];
	}	pdsForRaidLevels;

	uint16_t maxPds;		/* 0x780 */
	uint16_t maxDedHSPs;		/* 0x782 */
	uint16_t maxGlobalHSPs;	/* 0x784 */
	uint16_t ddfSize;		/* 0x786 */
	uint8_t maxLdsPerArray;	/* 0x788 */
	uint8_t partitionsInDDF;	/* 0x789 */
	uint8_t lockKeyBinding;	/* 0x78a */
	uint8_t maxPITsPerLd;		/* 0x78b */
	uint8_t maxViewsPerLd;		/* 0x78c */
	uint8_t maxTargetId;		/* 0x78d */
	uint16_t maxBvlVdSize;		/* 0x78e */

	uint16_t maxConfigurableSSCSize;	/* 0x790 */
	uint16_t currentSSCsize;	/* 0x792 */

	char	expanderFwVersion[12];	/* 0x794 */

	uint16_t PFKTrialTimeRemaining;/* 0x7A0 */

	uint16_t cacheMemorySize;	/* 0x7A2 */

	struct {			/* 0x7A4 */
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t supportPIcontroller:1;
		uint32_t supportLdPIType1:1;
		uint32_t supportLdPIType2:1;
		uint32_t supportLdPIType3:1;
		uint32_t supportLdBBMInfo:1;
		uint32_t supportShieldState:1;
		uint32_t blockSSDWriteCacheChange:1;
		uint32_t supportSuspendResumeBGops:1;
		uint32_t supportEmergencySpares:1;
		uint32_t supportSetLinkSpeed:1;
		uint32_t supportBootTimePFKChange:1;
		uint32_t supportJBOD:1;
		uint32_t disableOnlinePFKChange:1;
		uint32_t supportPerfTuning:1;
		uint32_t supportSSDPatrolRead:1;
		uint32_t realTimeScheduler:1;

		uint32_t supportResetNow:1;
		uint32_t supportEmulatedDrives:1;
		uint32_t headlessMode:1;
		uint32_t dedicatedHotSparesLimited:1;

		uint32_t supportUnevenSpans:1;
		uint32_t reserved:11;
#else
		uint32_t reserved:11;
		uint32_t supportUnevenSpans:1;
		uint32_t dedicatedHotSparesLimited:1;
		uint32_t headlessMode:1;
		uint32_t supportEmulatedDrives:1;
		uint32_t supportResetNow:1;
		uint32_t realTimeScheduler:1;
		uint32_t supportSSDPatrolRead:1;
		uint32_t supportPerfTuning:1;
		uint32_t disableOnlinePFKChange:1;
		uint32_t supportJBOD:1;
		uint32_t supportBootTimePFKChange:1;
		uint32_t supportSetLinkSpeed:1;
		uint32_t supportEmergencySpares:1;
		uint32_t supportSuspendResumeBGops:1;
		uint32_t blockSSDWriteCacheChange:1;
		uint32_t supportShieldState:1;
		uint32_t supportLdBBMInfo:1;
		uint32_t supportLdPIType3:1;
		uint32_t supportLdPIType2:1;
		uint32_t supportLdPIType1:1;
		uint32_t supportPIcontroller:1;
#endif
	}	adapterOperations2;

	uint8_t driverVersion[32];	/* 0x7A8 */
	uint8_t maxDAPdCountSpinup60;	/* 0x7C8 */
	uint8_t temperatureROC;	/* 0x7C9 */
	uint8_t temperatureCtrl;	/* 0x7CA */
	uint8_t reserved4;		/* 0x7CB */
	uint16_t maxConfigurablePds;	/* 0x7CC */

	uint8_t reserved5[2];		/* 0x7CD reserved */

	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t peerIsPresent:1;
		uint32_t peerIsIncompatible:1;

		uint32_t hwIncompatible:1;
		uint32_t fwVersionMismatch:1;
		uint32_t ctrlPropIncompatible:1;
		uint32_t premiumFeatureMismatch:1;
		uint32_t reserved:26;
#else
		uint32_t reserved:26;
		uint32_t premiumFeatureMismatch:1;
		uint32_t ctrlPropIncompatible:1;
		uint32_t fwVersionMismatch:1;
		uint32_t hwIncompatible:1;
		uint32_t peerIsIncompatible:1;
		uint32_t peerIsPresent:1;
#endif
	}	cluster;

	char	clusterId[16];		/* 0x7D4 */

	char	reserved6[4];		/* 0x7E4 RESERVED FOR IOV */

	struct {			/* 0x7E8 */
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t supportPersonalityChange:2;
		uint32_t supportThermalPollInterval:1;
		uint32_t supportDisableImmediateIO:1;
		uint32_t supportT10RebuildAssist:1;
		uint32_t supportMaxExtLDs:1;
		uint32_t supportCrashDump:1;
		uint32_t supportSwZone:1;
		uint32_t supportDebugQueue:1;
		uint32_t supportNVCacheErase:1;
		uint32_t supportForceTo512e:1;
		uint32_t supportHOQRebuild:1;
		uint32_t supportAllowedOpsforDrvRemoval:1;
		uint32_t supportDrvActivityLEDSetting:1;
		uint32_t supportNVDRAM:1;
		uint32_t supportForceFlash:1;
		uint32_t supportDisableSESMonitoring:1;
		uint32_t supportCacheBypassModes:1;
		uint32_t supportSecurityonJBOD:1;
		uint32_t discardCacheDuringLDDelete:1;
		uint32_t supportTTYLogCompression:1;
		uint32_t supportCPLDUpdate:1;
		uint32_t supportDiskCacheSettingForSysPDs:1;
		uint32_t supportExtendedSSCSize:1;
		uint32_t useSeqNumJbodFP:1;
		uint32_t reserved:7;
#else
		uint32_t reserved:7;
		uint32_t useSeqNumJbodFP:1;
		uint32_t supportExtendedSSCSize:1;
		uint32_t supportDiskCacheSettingForSysPDs:1;
		uint32_t supportCPLDUpdate:1;
		uint32_t supportTTYLogCompression:1;
		uint32_t discardCacheDuringLDDelete:1;
		uint32_t supportSecurityonJBOD:1;
		uint32_t supportCacheBypassModes:1;
		uint32_t supportDisableSESMonitoring:1;
		uint32_t supportForceFlash:1;
		uint32_t supportNVDRAM:1;
		uint32_t supportDrvActivityLEDSetting:1;
		uint32_t supportAllowedOpsforDrvRemoval:1;
		uint32_t supportHOQRebuild:1;
		uint32_t supportForceTo512e:1;
		uint32_t supportNVCacheErase:1;
		uint32_t supportDebugQueue:1;
		uint32_t supportSwZone:1;
		uint32_t supportCrashDump:1;
		uint32_t supportMaxExtLDs:1;
		uint32_t supportT10RebuildAssist:1;
		uint32_t supportDisableImmediateIO:1;
		uint32_t supportThermalPollInterval:1;
		uint32_t supportPersonalityChange:2;
#endif
	}	adapterOperations3;

	uint8_t pad_cpld[16];

	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint16_t ctrlInfoExtSupported:1;
		uint16_t supportIbuttonLess:1;
		uint16_t supportedEncAlgo:1;
		uint16_t supportEncryptedMfc:1;
		uint16_t imageUploadSupported:1;
		uint16_t supportSESCtrlInMultipathCfg:1;
		uint16_t supportPdMapTargetId:1;
		uint16_t FWSwapsBBUVPDInfo:1;
		uint16_t reserved:8;
#else
		uint16_t reserved:8;
		uint16_t FWSwapsBBUVPDInfo:1;
		uint16_t supportPdMapTargetId:1;
		uint16_t supportSESCtrlInMultipathCfg:1;
		uint16_t imageUploadSupported:1;
		uint16_t supportEncryptedMfc:1;
		uint16_t supportedEncAlgo:1;
		uint16_t supportIbuttonLess:1;
		uint16_t ctrlInfoExtSupported:1;
#endif
	}	adapterOperations4;

	uint8_t pad[0x800 - 0x7FE];	/* 0x7FE */
} __packed;

/*
 * When SCSI mid-layer calls driver's reset routine, driver waits for
 * MRSAS_RESET_WAIT_TIME seconds for all outstanding IO to complete. Note
 * that the driver cannot _actually_ abort or reset pending commands. While
 * it is waiting for the commands to complete, it prints a diagnostic message
 * every MRSAS_RESET_NOTICE_INTERVAL seconds
 */
#define	MRSAS_RESET_WAIT_TIME			180
#define	MRSAS_INTERNAL_CMD_WAIT_TIME	180
#define	MRSAS_RESET_NOTICE_INTERVAL		5
#define	MRSAS_IOCTL_CMD					0
#define	MRSAS_DEFAULT_CMD_TIMEOUT		90
#define	MRSAS_THROTTLE_QUEUE_DEPTH		16

/*
 * MSI-x regsiters offset defines
 */
#define	MPI2_SUP_REPLY_POST_HOST_INDEX_OFFSET	(0x0000030C)
#define	MPI2_REPLY_POST_HOST_INDEX_OFFSET		(0x0000006C)
#define	MR_MAX_REPLY_QUEUES_OFFSET				(0x0000001F)
#define	MR_MAX_REPLY_QUEUES_EXT_OFFSET			(0x003FC000)
#define	MR_MAX_REPLY_QUEUES_EXT_OFFSET_SHIFT	14
#define	MR_MAX_MSIX_REG_ARRAY					16

/*
 * SYNC CACHE offset define
 */
#define MR_CAN_HANDLE_SYNC_CACHE_OFFSET     0X01000000

#define MR_ATOMIC_DESCRIPTOR_SUPPORT_OFFSET (1 << 24)

/*
 * FW reports the maximum of number of commands that it can accept (maximum
 * commands that can be outstanding) at any time. The driver must report a
 * lower number to the mid layer because it can issue a few internal commands
 * itself (E.g, AEN, abort cmd, IOCTLs etc). The number of commands it needs
 * is shown below
 */
#define	MRSAS_INT_CMDS			32
#define	MRSAS_SKINNY_INT_CMDS	5
#define	MRSAS_MAX_MSIX_QUEUES	128

/*
 * FW can accept both 32 and 64 bit SGLs. We want to allocate 32/64 bit SGLs
 * based on the size of bus_addr_t
 */
#define	IS_DMA64							(sizeof(bus_addr_t) == 8)

#define	MFI_XSCALE_OMR0_CHANGE_INTERRUPT	0x00000001
#define	MFI_INTR_FLAG_REPLY_MESSAGE			0x00000001
#define	MFI_INTR_FLAG_FIRMWARE_STATE_CHANGE	0x00000002
#define	MFI_G2_OUTBOUND_DOORBELL_CHANGE_INTERRUPT	0x00000004

#define	MFI_OB_INTR_STATUS_MASK				0x00000002
#define	MFI_POLL_TIMEOUT_SECS				60

#define	MFI_REPLY_1078_MESSAGE_INTERRUPT	0x80000000
#define	MFI_REPLY_GEN2_MESSAGE_INTERRUPT	0x00000001
#define	MFI_GEN2_ENABLE_INTERRUPT_MASK		0x00000001
#define	MFI_REPLY_SKINNY_MESSAGE_INTERRUPT	0x40000000
#define	MFI_SKINNY_ENABLE_INTERRUPT_MASK	(0x00000001)
#define	MFI_1068_PCSR_OFFSET				0x84
#define	MFI_1068_FW_HANDSHAKE_OFFSET		0x64
#define	MFI_1068_FW_READY					0xDDDD0000

typedef union _MFI_CAPABILITIES {
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t support_fp_remote_lun:1;
		uint32_t support_additional_msix:1;
		uint32_t support_fastpath_wb:1;
		uint32_t support_max_255lds:1;
		uint32_t support_ndrive_r1_lb:1;
		uint32_t support_core_affinity:1;
		uint32_t security_protocol_cmds_fw:1;
		uint32_t support_ext_queue_depth:1;
		uint32_t support_ext_io_size:1;
		uint32_t reserved:23;
#else
		uint32_t reserved:23;
		uint32_t support_ext_io_size:1;
		uint32_t support_ext_queue_depth:1;
		uint32_t security_protocol_cmds_fw:1;
		uint32_t support_core_affinity:1;
		uint32_t support_ndrive_r1_lb:1;
		uint32_t support_max_255lds:1;
		uint32_t support_fastpath_wb:1;
		uint32_t support_additional_msix:1;
		uint32_t support_fp_remote_lun:1;
#endif
	}	mfi_capabilities;
	uint32_t reg;
}	MFI_CAPABILITIES;

#pragma pack(1)
struct mrsas_sge32 {
	uint32_t phys_addr;
	uint32_t length;
};

#pragma pack()

#pragma pack(1)
struct mrsas_sge64 {
	uint64_t phys_addr;
	uint32_t length;
};

#pragma pack()

#pragma pack()
union mrsas_sgl {
	struct mrsas_sge32 sge32[1];
	struct mrsas_sge64 sge64[1];
};

#pragma pack()

#pragma pack(1)
struct mrsas_header {
	uint8_t cmd;			/* 00e */
	uint8_t sense_len;		/* 01h */
	uint8_t cmd_status;		/* 02h */
	uint8_t scsi_status;		/* 03h */

	uint8_t target_id;		/* 04h */
	uint8_t lun;			/* 05h */
	uint8_t cdb_len;		/* 06h */
	uint8_t sge_count;		/* 07h */

	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t timeout;		/* 12h */
	uint32_t data_xferlen;		/* 14h */
};

#pragma pack()

#pragma pack(1)
struct mrsas_init_frame {
	uint8_t cmd;			/* 00h */
	uint8_t reserved_0;		/* 01h */
	uint8_t cmd_status;		/* 02h */

	uint8_t reserved_1;		/* 03h */
	MFI_CAPABILITIES driver_operations;	/* 04h */
	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t reserved_3;		/* 12h */
	uint32_t data_xfer_len;	/* 14h */

	uint32_t queue_info_new_phys_addr_lo;	/* 18h */
	uint32_t queue_info_new_phys_addr_hi;	/* 1Ch */
	uint32_t queue_info_old_phys_addr_lo;	/* 20h */
	uint32_t queue_info_old_phys_addr_hi;	/* 24h */
	uint32_t driver_ver_lo;	/* 28h */
	uint32_t driver_ver_hi;	/* 2Ch */
	uint32_t reserved_4[4];	/* 30h */
};

#pragma pack()

#pragma pack(1)
struct mrsas_io_frame {
	uint8_t cmd;			/* 00h */
	uint8_t sense_len;		/* 01h */
	uint8_t cmd_status;		/* 02h */
	uint8_t scsi_status;		/* 03h */

	uint8_t target_id;		/* 04h */
	uint8_t access_byte;		/* 05h */
	uint8_t reserved_0;		/* 06h */
	uint8_t sge_count;		/* 07h */

	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t timeout;		/* 12h */
	uint32_t lba_count;		/* 14h */

	uint32_t sense_buf_phys_addr_lo;	/* 18h */
	uint32_t sense_buf_phys_addr_hi;	/* 1Ch */

	uint32_t start_lba_lo;		/* 20h */
	uint32_t start_lba_hi;		/* 24h */

	union mrsas_sgl sgl;		/* 28h */
};

#pragma pack()

#pragma pack(1)
struct mrsas_pthru_frame {
	uint8_t cmd;			/* 00h */
	uint8_t sense_len;		/* 01h */
	uint8_t cmd_status;		/* 02h */
	uint8_t scsi_status;		/* 03h */

	uint8_t target_id;		/* 04h */
	uint8_t lun;			/* 05h */
	uint8_t cdb_len;		/* 06h */
	uint8_t sge_count;		/* 07h */

	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t timeout;		/* 12h */
	uint32_t data_xfer_len;	/* 14h */

	uint32_t sense_buf_phys_addr_lo;	/* 18h */
	uint32_t sense_buf_phys_addr_hi;	/* 1Ch */

	uint8_t cdb[16];		/* 20h */
	union mrsas_sgl sgl;		/* 30h */
};

#pragma pack()

#pragma pack(1)
struct mrsas_dcmd_frame {
	uint8_t cmd;			/* 00h */
	uint8_t reserved_0;		/* 01h */
	uint8_t cmd_status;		/* 02h */
	uint8_t reserved_1[4];		/* 03h */
	uint8_t sge_count;		/* 07h */

	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t timeout;		/* 12h */

	uint32_t data_xfer_len;	/* 14h */
	uint32_t opcode;		/* 18h */

	union {				/* 1Ch */
		uint8_t b[12];
		uint16_t s[6];
		uint32_t w[3];
	}	mbox;

	union mrsas_sgl sgl;		/* 28h */
};

#pragma pack()

#pragma pack(1)
struct mrsas_abort_frame {
	uint8_t cmd;			/* 00h */
	uint8_t reserved_0;		/* 01h */
	uint8_t cmd_status;		/* 02h */

	uint8_t reserved_1;		/* 03h */
	MFI_CAPABILITIES driver_operations;	/* 04h */
	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t reserved_3;		/* 12h */
	uint32_t reserved_4;		/* 14h */

	uint32_t abort_context;	/* 18h */
	uint32_t pad_1;		/* 1Ch */

	uint32_t abort_mfi_phys_addr_lo;	/* 20h */
	uint32_t abort_mfi_phys_addr_hi;	/* 24h */

	uint32_t reserved_5[6];	/* 28h */
};

#pragma pack()

#pragma pack(1)
struct mrsas_smp_frame {
	uint8_t cmd;			/* 00h */
	uint8_t reserved_1;		/* 01h */
	uint8_t cmd_status;		/* 02h */
	uint8_t connection_status;	/* 03h */

	uint8_t reserved_2[3];		/* 04h */
	uint8_t sge_count;		/* 07h */

	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t timeout;		/* 12h */

	uint32_t data_xfer_len;	/* 14h */
	uint64_t sas_addr;		/* 18h */

	union {
		struct mrsas_sge32 sge32[2];	/* [0]: resp [1]: req */
		struct mrsas_sge64 sge64[2];	/* [0]: resp [1]: req */
	}	sgl;
};

#pragma pack()

#pragma pack(1)
struct mrsas_stp_frame {
	uint8_t cmd;			/* 00h */
	uint8_t reserved_1;		/* 01h */
	uint8_t cmd_status;		/* 02h */
	uint8_t reserved_2;		/* 03h */

	uint8_t target_id;		/* 04h */
	uint8_t reserved_3[2];		/* 05h */
	uint8_t sge_count;		/* 07h */

	uint32_t context;		/* 08h */
	uint32_t pad_0;		/* 0Ch */

	uint16_t flags;		/* 10h */
	uint16_t timeout;		/* 12h */

	uint32_t data_xfer_len;	/* 14h */

	uint16_t fis[10];		/* 18h */
	uint32_t stp_flags;

	union {
		struct mrsas_sge32 sge32[2];	/* [0]: resp [1]: data */
		struct mrsas_sge64 sge64[2];	/* [0]: resp [1]: data */
	}	sgl;
};

#pragma pack()

union mrsas_frame {
	struct mrsas_header hdr;
	struct mrsas_init_frame init;
	struct mrsas_io_frame io;
	struct mrsas_pthru_frame pthru;
	struct mrsas_dcmd_frame dcmd;
	struct mrsas_abort_frame abort;
	struct mrsas_smp_frame smp;
	struct mrsas_stp_frame stp;
	uint8_t raw_bytes[64];
};

#pragma pack(1)
union mrsas_evt_class_locale {
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint16_t locale;
		uint8_t reserved;
		int8_t	class;
#else
		int8_t	class;
		uint8_t reserved;
		uint16_t locale;
#endif
	} __packed members;

	uint32_t word;

} __packed;

#pragma pack()

#pragma pack(1)
struct mrsas_evt_log_info {
	uint32_t newest_seq_num;
	uint32_t oldest_seq_num;
	uint32_t clear_seq_num;
	uint32_t shutdown_seq_num;
	uint32_t boot_seq_num;

} __packed;

#pragma pack()

struct mrsas_progress {
	uint16_t progress;
	uint16_t elapsed_seconds;

} __packed;

struct mrsas_evtarg_ld {
	uint16_t target_id;
	uint8_t ld_index;
	uint8_t reserved;

} __packed;

struct mrsas_evtarg_pd {
	uint16_t device_id;
	uint8_t encl_index;
	uint8_t slot_number;

} __packed;

struct mrsas_evt_detail {
	uint32_t seq_num;
	uint32_t time_stamp;
	uint32_t code;
	union mrsas_evt_class_locale cl;
	uint8_t arg_type;
	uint8_t reserved1[15];

	union {
		struct {
			struct mrsas_evtarg_pd pd;
			uint8_t cdb_length;
			uint8_t sense_length;
			uint8_t reserved[2];
			uint8_t cdb[16];
			uint8_t sense[64];
		} __packed cdbSense;

		struct mrsas_evtarg_ld ld;

		struct {
			struct mrsas_evtarg_ld ld;
			uint64_t count;
		} __packed ld_count;

		struct {
			uint64_t lba;
			struct mrsas_evtarg_ld ld;
		} __packed ld_lba;

		struct {
			struct mrsas_evtarg_ld ld;
			uint32_t prevOwner;
			uint32_t newOwner;
		} __packed ld_owner;

		struct {
			uint64_t ld_lba;
			uint64_t pd_lba;
			struct mrsas_evtarg_ld ld;
			struct mrsas_evtarg_pd pd;
		} __packed ld_lba_pd_lba;

		struct {
			struct mrsas_evtarg_ld ld;
			struct mrsas_progress prog;
		} __packed ld_prog;

		struct {
			struct mrsas_evtarg_ld ld;
			uint32_t prev_state;
			uint32_t new_state;
		} __packed ld_state;

		struct {
			uint64_t strip;
			struct mrsas_evtarg_ld ld;
		} __packed ld_strip;

		struct mrsas_evtarg_pd pd;

		struct {
			struct mrsas_evtarg_pd pd;
			uint32_t err;
		} __packed pd_err;

		struct {
			uint64_t lba;
			struct mrsas_evtarg_pd pd;
		} __packed pd_lba;

		struct {
			uint64_t lba;
			struct mrsas_evtarg_pd pd;
			struct mrsas_evtarg_ld ld;
		} __packed pd_lba_ld;

		struct {
			struct mrsas_evtarg_pd pd;
			struct mrsas_progress prog;
		} __packed pd_prog;

		struct {
			struct mrsas_evtarg_pd pd;
			uint32_t prevState;
			uint32_t newState;
		} __packed pd_state;

		struct {
			uint16_t vendorId;
			uint16_t deviceId;
			uint16_t subVendorId;
			uint16_t subDeviceId;
		} __packed pci;

		uint32_t rate;
		char	str[96];

		struct {
			uint32_t rtc;
			uint32_t elapsedSeconds;
		} __packed time;

		struct {
			uint32_t ecar;
			uint32_t elog;
			char	str[64];
		} __packed ecc;

		uint8_t b[96];
		uint16_t s[48];
		uint32_t w[24];
		uint64_t d[12];
	}	args;

	char	description[128];

} __packed;

struct mrsas_irq_context {
	struct mrsas_softc *sc;
	uint32_t MSIxIndex;
};

enum MEGASAS_OCR_REASON {
	FW_FAULT_OCR = 0,
	MFI_DCMD_TIMEOUT_OCR = 1,
};

/* Controller management info added to support Linux Emulator */
#define	MAX_MGMT_ADAPTERS               1024

struct mrsas_mgmt_info {
	uint16_t count;
	struct mrsas_softc *sc_ptr[MAX_MGMT_ADAPTERS];
	int	max_index;
};

#define	PCI_TYPE0_ADDRESSES             6
#define	PCI_TYPE1_ADDRESSES             2
#define	PCI_TYPE2_ADDRESSES             5

typedef struct _MRSAS_DRV_PCI_COMMON_HEADER {
	uint16_t vendorID;
	      //(ro)
	uint16_t deviceID;
	      //(ro)
	uint16_t command;
	      //Device control
	uint16_t status;
	uint8_t revisionID;
	      //(ro)
	uint8_t progIf;
	      //(ro)
	uint8_t subClass;
	      //(ro)
	uint8_t baseClass;
	      //(ro)
	uint8_t cacheLineSize;
	      //(ro +)
	uint8_t latencyTimer;
	      //(ro +)
	uint8_t headerType;
	      //(ro)
	uint8_t bist;
	      //Built in self test

	union {
		struct _MRSAS_DRV_PCI_HEADER_TYPE_0 {
			uint32_t baseAddresses[PCI_TYPE0_ADDRESSES];
			uint32_t cis;
			uint16_t subVendorID;
			uint16_t subSystemID;
			uint32_t romBaseAddress;
			uint8_t capabilitiesPtr;
			uint8_t reserved1[3];
			uint32_t reserved2;
			uint8_t interruptLine;
			uint8_t interruptPin;
			      //(ro)
			uint8_t minimumGrant;
			      //(ro)
			uint8_t maximumLatency;
			      //(ro)
		}	type0;

		/*
	         * PCI to PCI Bridge
	         */

		struct _MRSAS_DRV_PCI_HEADER_TYPE_1 {
			uint32_t baseAddresses[PCI_TYPE1_ADDRESSES];
			uint8_t primaryBus;
			uint8_t secondaryBus;
			uint8_t subordinateBus;
			uint8_t secondaryLatency;
			uint8_t ioBase;
			uint8_t ioLimit;
			uint16_t secondaryStatus;
			uint16_t memoryBase;
			uint16_t memoryLimit;
			uint16_t prefetchBase;
			uint16_t prefetchLimit;
			uint32_t prefetchBaseUpper32;
			uint32_t prefetchLimitUpper32;
			uint16_t ioBaseUpper16;
			uint16_t ioLimitUpper16;
			uint8_t capabilitiesPtr;
			uint8_t reserved1[3];
			uint32_t romBaseAddress;
			uint8_t interruptLine;
			uint8_t interruptPin;
			uint16_t bridgeControl;
		}	type1;

		/*
	         * PCI to CARDBUS Bridge
	         */

		struct _MRSAS_DRV_PCI_HEADER_TYPE_2 {
			uint32_t socketRegistersBaseAddress;
			uint8_t capabilitiesPtr;
			uint8_t reserved;
			uint16_t secondaryStatus;
			uint8_t primaryBus;
			uint8_t secondaryBus;
			uint8_t subordinateBus;
			uint8_t secondaryLatency;
			struct {
				uint32_t base;
				uint32_t limit;
			}	range [PCI_TYPE2_ADDRESSES - 1];
			uint8_t interruptLine;
			uint8_t interruptPin;
			uint16_t bridgeControl;
		}	type2;
	}	u;

}	MRSAS_DRV_PCI_COMMON_HEADER, *PMRSAS_DRV_PCI_COMMON_HEADER;

#define	MRSAS_DRV_PCI_COMMON_HEADER_SIZE sizeof(MRSAS_DRV_PCI_COMMON_HEADER)   //64 bytes

typedef struct _MRSAS_DRV_PCI_LINK_CAPABILITY {
	union {
		struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
			uint32_t linkSpeed:4;
			uint32_t linkWidth:6;
			uint32_t aspmSupport:2;
			uint32_t losExitLatency:3;
			uint32_t l1ExitLatency:3;
			uint32_t rsvdp:6;
			uint32_t portNumber:8;
#else
			uint32_t portNumber:8;
			uint32_t rsvdp:6;
			uint32_t l1ExitLatency:3;
			uint32_t losExitLatency:3;
			uint32_t aspmSupport:2;
			uint32_t linkWidth:6;
			uint32_t linkSpeed:4;
#endif
		}	bits;

		uint32_t asUlong;
	}	u;
}	MRSAS_DRV_PCI_LINK_CAPABILITY, *PMRSAS_DRV_PCI_LINK_CAPABILITY;

#define	MRSAS_DRV_PCI_LINK_CAPABILITY_SIZE sizeof(MRSAS_DRV_PCI_LINK_CAPABILITY)

typedef struct _MRSAS_DRV_PCI_LINK_STATUS_CAPABILITY {
	union {
		struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
			uint16_t linkSpeed:4;
			uint16_t negotiatedLinkWidth:6;
			uint16_t linkTrainingError:1;
			uint16_t linkTraning:1;
			uint16_t slotClockConfig:1;
			uint16_t rsvdZ:3;
#else
			uint16_t rsvdZ:3;
			uint16_t slotClockConfig:1;
			uint16_t linkTraning:1;
			uint16_t linkTrainingError:1;
			uint16_t negotiatedLinkWidth:6;
			uint16_t linkSpeed:4;
#endif
		}	bits;

		uint16_t asUshort;
	}	u;
	uint16_t reserved;
}	MRSAS_DRV_PCI_LINK_STATUS_CAPABILITY, *PMRSAS_DRV_PCI_LINK_STATUS_CAPABILITY;

#define	MRSAS_DRV_PCI_LINK_STATUS_CAPABILITY_SIZE sizeof(MRSAS_DRV_PCI_LINK_STATUS_CAPABILITY)

typedef struct _MRSAS_DRV_PCI_CAPABILITIES {
	MRSAS_DRV_PCI_LINK_CAPABILITY linkCapability;
	MRSAS_DRV_PCI_LINK_STATUS_CAPABILITY linkStatusCapability;
}	MRSAS_DRV_PCI_CAPABILITIES, *PMRSAS_DRV_PCI_CAPABILITIES;

#define	MRSAS_DRV_PCI_CAPABILITIES_SIZE sizeof(MRSAS_DRV_PCI_CAPABILITIES)

/* PCI information */
typedef struct _MRSAS_DRV_PCI_INFORMATION {
	uint32_t busNumber;
	uint8_t deviceNumber;
	uint8_t functionNumber;
	uint8_t interruptVector;
	uint8_t reserved1;
	MRSAS_DRV_PCI_COMMON_HEADER pciHeaderInfo;
	MRSAS_DRV_PCI_CAPABILITIES capability;
	uint32_t domainID;
	uint8_t reserved2[28];
}	MRSAS_DRV_PCI_INFORMATION, *PMRSAS_DRV_PCI_INFORMATION;

typedef enum _MR_PD_TYPE {
	UNKNOWN_DRIVE = 0,
	PARALLEL_SCSI = 1,
	SAS_PD = 2,
	SATA_PD = 3,
	FC_PD = 4,
	NVME_PD = 5,
} MR_PD_TYPE;

typedef union	_MR_PD_REF {
	struct {
		uint16_t	 deviceId;
		uint16_t	 seqNum;
	} mrPdRef;
	uint32_t	 ref;
} MR_PD_REF;

/*
 * define the DDF Type bit structure
 */
union MR_PD_DDF_TYPE {
	struct {
		union {
			struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
				uint16_t forcedPDGUID:1;
				uint16_t inVD:1;
				uint16_t isGlobalSpare:1;
				uint16_t isSpare:1;
				uint16_t isForeign:1;
				uint16_t reserved:7;
				uint16_t intf:4;
#else
				uint16_t intf:4;
				uint16_t reserved:7;
				uint16_t isForeign:1;
				uint16_t isSpare:1;
				uint16_t isGlobalSpare:1;
				uint16_t inVD:1;
				uint16_t forcedPDGUID:1;
#endif
			} pdType;
			uint16_t type;
		};
		uint16_t reserved;
	} ddf;
	struct {
		uint32_t reserved;
	} nonDisk;
	uint32_t type;
} __packed;

/*
 * defines the progress structure
 */
union MR_PROGRESS {
	struct  {
		uint16_t progress;
		union {
			uint16_t elapsedSecs;
			uint16_t elapsedSecsForLastPercent;
		};
	} mrProgress;
	uint32_t w;
} __packed;

/*
 * defines the physical drive progress structure
 */
struct MR_PD_PROGRESS {
    struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
        uint32_t     rbld:1;
        uint32_t     patrol:1;
        uint32_t     clear:1;
        uint32_t     copyBack:1;
        uint32_t     erase:1;
        uint32_t     locate:1;
        uint32_t     reserved:26;
#else
		    uint32_t     reserved:26;
		    uint32_t     locate:1;
		    uint32_t     erase:1;
		    uint32_t     copyBack:1;
		    uint32_t     clear:1;
		    uint32_t     patrol:1;
		    uint32_t     rbld:1;
#endif
    } active;
    union MR_PROGRESS     rbld;
    union MR_PROGRESS     patrol;
    union {
        union MR_PROGRESS     clear;
        union MR_PROGRESS     erase;
    };

    struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
        uint32_t     rbld:1;
        uint32_t     patrol:1;
        uint32_t     clear:1;
        uint32_t     copyBack:1;
        uint32_t     erase:1;
        uint32_t     reserved:27;
#else
		    uint32_t     reserved:27;
		    uint32_t     erase:1;
		    uint32_t     copyBack:1;
		    uint32_t     clear:1;
		    uint32_t     patrol:1;
		    uint32_t     rbld:1;
#endif
    } pause;

    union MR_PROGRESS     reserved[3];
} __packed;

struct  mrsas_pd_info {
	 MR_PD_REF	 ref;
	 uint8_t		 inquiryData[96];
	 uint8_t		 vpdPage83[64];

	 uint8_t		 notSupported;
	 uint8_t		 scsiDevType;

	 union {
		 uint8_t		 connectedPortBitmap;
		 uint8_t		 connectedPortNumbers;
	 };

	 uint8_t		 deviceSpeed;
	 uint32_t	 mediaErrCount;
	 uint32_t	 otherErrCount;
	 uint32_t	 predFailCount;
	 uint32_t	 lastPredFailEventSeqNum;

	 uint16_t	 fwState;
	 uint8_t		 disabledForRemoval;
	 uint8_t		 linkSpeed;
	 union MR_PD_DDF_TYPE  state;

	 struct {
		 uint8_t		 count;
#if _BYTE_ORDER == _LITTLE_ENDIAN
		 uint8_t		 isPathBroken:4;
		 uint8_t		 reserved3:3;
		 uint8_t		 widePortCapable:1;
#else
		 uint8_t		 widePortCapable:1;
		 uint8_t		 reserved3:3;
		 uint8_t		 isPathBroken:4;
#endif
		 uint8_t		 connectorIndex[2];
		 uint8_t		 reserved[4];
		 uint64_t		 sasAddr[2];
		 uint8_t		 reserved2[16];
	 } pathInfo;

	 uint64_t	 rawSize;
	 uint64_t	 nonCoercedSize;
	 uint64_t	 coercedSize;
	 uint16_t	 enclDeviceId;
	 uint8_t		 enclIndex;

	 union {
		 uint8_t		 slotNumber;
		 uint8_t		 enclConnectorIndex;
	 };

	struct MR_PD_PROGRESS progInfo;
	 uint8_t		 badBlockTableFull;
	 uint8_t		 unusableInCurrentConfig;
	 uint8_t		 vpdPage83Ext[64];
	 uint8_t		 powerState;
	 uint8_t		 enclPosition;
	 uint32_t		allowedOps;
	 uint16_t	 copyBackPartnerId;
	 uint16_t	 enclPartnerDeviceId;
	struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		 uint16_t fdeCapable:1;
		 uint16_t fdeEnabled:1;
		 uint16_t secured:1;
		 uint16_t locked:1;
		 uint16_t foreign:1;
		 uint16_t needsEKM:1;
		 uint16_t reserved:10;
#else
		 uint16_t reserved:10;
		 uint16_t needsEKM:1;
		 uint16_t foreign:1;
		 uint16_t locked:1;
		 uint16_t secured:1;
		 uint16_t fdeEnabled:1;
		 uint16_t fdeCapable:1;
#endif
	 } security;
	 uint8_t		 mediaType;
	 uint8_t		 notCertified;
	 uint8_t		 bridgeVendor[8];
	 uint8_t		 bridgeProductIdentification[16];
	 uint8_t		 bridgeProductRevisionLevel[4];
	 uint8_t		 satBridgeExists;

	 uint8_t		 interfaceType;
	 uint8_t		 temperature;
	 uint8_t		 emulatedBlockSize;
	 uint16_t	 userDataBlockSize;
	 uint16_t	 reserved2;

	 struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		 uint32_t piType:3;
		 uint32_t piFormatted:1;
		 uint32_t piEligible:1;
		 uint32_t NCQ:1;
		 uint32_t WCE:1;
		 uint32_t commissionedSpare:1;
		 uint32_t emergencySpare:1;
		 uint32_t ineligibleForSSCD:1;
		 uint32_t ineligibleForLd:1;
		 uint32_t useSSEraseType:1;
		 uint32_t wceUnchanged:1;
		 uint32_t supportScsiUnmap:1;
		 uint32_t reserved:18;
#else
		 uint32_t reserved:18;
		 uint32_t supportScsiUnmap:1;
		 uint32_t wceUnchanged:1;
		 uint32_t useSSEraseType:1;
		 uint32_t ineligibleForLd:1;
		 uint32_t ineligibleForSSCD:1;
		 uint32_t emergencySpare:1;
		 uint32_t commissionedSpare:1;
		 uint32_t WCE:1;
		 uint32_t NCQ:1;
		 uint32_t piEligible:1;
		 uint32_t piFormatted:1;
		 uint32_t piType:3;
#endif
	 } properties;

	 uint64_t   shieldDiagCompletionTime;
	 uint8_t    shieldCounter;

	 uint8_t linkSpeedOther;
	 uint8_t reserved4[2];

	 struct {
#if _BYTE_ORDER == _LITTLE_ENDIAN
		uint32_t bbmErrCountSupported:1;
		uint32_t bbmErrCount:31;
#else
		uint32_t bbmErrCount:31;
		uint32_t bbmErrCountSupported:1;
#endif
	 } bbmErr;

	 uint8_t reserved1[512-428];
} __packed;

struct mrsas_target {
	uint16_t target_id;
	uint32_t queue_depth;
	uint8_t interface_type;
	uint32_t max_io_size_kb;
} __packed;

#define MR_NVME_PAGE_SIZE_MASK		0x000000FF
#define MR_DEFAULT_NVME_PAGE_SIZE	4096
#define MR_DEFAULT_NVME_PAGE_SHIFT	12

/*******************************************************************
 * per-instance data
 ********************************************************************/
struct mrsas_softc {
	device_t mrsas_dev;
	struct cdev *mrsas_cdev;
	struct intr_config_hook mrsas_ich;
	struct cdev *mrsas_linux_emulator_cdev;
	uint16_t device_id;
	struct resource *reg_res;
	int	reg_res_id;
	bus_space_tag_t bus_tag;
	bus_space_handle_t bus_handle;
	bus_dma_tag_t mrsas_parent_tag;
	bus_dma_tag_t verbuf_tag;
	bus_dmamap_t verbuf_dmamap;
	void   *verbuf_mem;
	bus_addr_t verbuf_phys_addr;
	bus_dma_tag_t sense_tag;
	bus_dmamap_t sense_dmamap;
	void   *sense_mem;
	bus_addr_t sense_phys_addr;
	bus_dma_tag_t io_request_tag;
	bus_dmamap_t io_request_dmamap;
	void   *io_request_mem;
	bus_addr_t io_request_phys_addr;
	bus_dma_tag_t chain_frame_tag;
	bus_dmamap_t chain_frame_dmamap;
	void   *chain_frame_mem;
	bus_addr_t chain_frame_phys_addr;
	bus_dma_tag_t reply_desc_tag;
	bus_dmamap_t reply_desc_dmamap;
	void   *reply_desc_mem;
	bus_addr_t reply_desc_phys_addr;
	bus_dma_tag_t ioc_init_tag;
	bus_dmamap_t ioc_init_dmamap;
	void   *ioc_init_mem;
	bus_addr_t ioc_init_phys_mem;
	bus_dma_tag_t data_tag;
	struct cam_sim *sim_0;
	struct cam_sim *sim_1;
	struct cam_path *path_0;
	struct cam_path *path_1;
	struct mtx sim_lock;
	struct mtx pci_lock;
	struct mtx io_lock;
	struct mtx ioctl_lock;
	struct mtx mpt_cmd_pool_lock;
	struct mtx mfi_cmd_pool_lock;
	struct mtx raidmap_lock;
	struct mtx aen_lock;
	struct mtx stream_lock;
	struct selinfo mrsas_select;
	uint32_t mrsas_aen_triggered;
	uint32_t mrsas_poll_waiting;

	struct sema ioctl_count_sema;
	uint32_t max_fw_cmds;
	uint16_t max_scsi_cmds;
	uint32_t max_num_sge;
	struct resource *mrsas_irq[MAX_MSIX_COUNT];
	void   *intr_handle[MAX_MSIX_COUNT];
	int	irq_id[MAX_MSIX_COUNT];
	struct mrsas_irq_context irq_context[MAX_MSIX_COUNT];
	int	msix_vectors;
	int	msix_enable;
	uint32_t msix_reg_offset[16];
	uint8_t	mask_interrupts;
	uint16_t max_chain_frame_sz;
	struct mrsas_mpt_cmd **mpt_cmd_list;
	struct mrsas_mfi_cmd **mfi_cmd_list;
	TAILQ_HEAD(, mrsas_mpt_cmd) mrsas_mpt_cmd_list_head;
	TAILQ_HEAD(, mrsas_mfi_cmd) mrsas_mfi_cmd_list_head;
	bus_addr_t req_frames_desc_phys;
	uint8_t *req_frames_desc;
	uint8_t *req_desc;
	bus_addr_t io_request_frames_phys;
	uint8_t *io_request_frames;
	bus_addr_t reply_frames_desc_phys;
	uint16_t last_reply_idx[MAX_MSIX_COUNT];
	uint32_t reply_q_depth;
	uint32_t request_alloc_sz;
	uint32_t reply_alloc_sz;
	uint32_t io_frames_alloc_sz;
	uint32_t chain_frames_alloc_sz;
	uint16_t max_sge_in_main_msg;
	uint16_t max_sge_in_chain;
	uint8_t chain_offset_io_request;
	uint8_t chain_offset_mfi_pthru;
	uint32_t map_sz;
	uint64_t map_id;
	uint64_t pd_seq_map_id;
	struct mrsas_mfi_cmd *map_update_cmd;
	struct mrsas_mfi_cmd *jbod_seq_cmd;
	struct mrsas_mfi_cmd *aen_cmd;
	uint8_t fast_path_io;
	void   *chan;
	void   *ocr_chan;
	uint8_t adprecovery;
	uint8_t remove_in_progress;
	uint8_t ocr_thread_active;
	uint8_t do_timedout_reset;
	uint32_t reset_in_progress;
	uint32_t reset_count;
	uint32_t block_sync_cache;
	uint32_t drv_stream_detection;
	uint8_t fw_sync_cache_support;
	mrsas_atomic_t target_reset_outstanding;
#define MRSAS_MAX_TM_TARGETS (MRSAS_MAX_PD + MRSAS_MAX_LD_IDS)
    struct mrsas_mpt_cmd *target_reset_pool[MRSAS_MAX_TM_TARGETS];

	bus_dma_tag_t jbodmap_tag[2];
	bus_dmamap_t jbodmap_dmamap[2];
	void   *jbodmap_mem[2];
	bus_addr_t jbodmap_phys_addr[2];

	bus_dma_tag_t raidmap_tag[2];
	bus_dmamap_t raidmap_dmamap[2];
	void   *raidmap_mem[2];
	bus_addr_t raidmap_phys_addr[2];
	bus_dma_tag_t mficmd_frame_tag;
	bus_dma_tag_t mficmd_sense_tag;
	bus_addr_t evt_detail_phys_addr;
	bus_dma_tag_t evt_detail_tag;
	bus_dmamap_t evt_detail_dmamap;
	struct mrsas_evt_detail *evt_detail_mem;
	bus_addr_t pd_info_phys_addr;
	bus_dma_tag_t pd_info_tag;
	bus_dmamap_t pd_info_dmamap;
	struct mrsas_pd_info *pd_info_mem;
	struct mrsas_ctrl_info *ctrl_info;
	bus_dma_tag_t ctlr_info_tag;
	bus_dmamap_t ctlr_info_dmamap;
	void   *ctlr_info_mem;
	bus_addr_t ctlr_info_phys_addr;
	uint32_t max_sectors_per_req;
	uint32_t disableOnlineCtrlReset;
	mrsas_atomic_t fw_outstanding;
	mrsas_atomic_t prp_count;
	mrsas_atomic_t sge_holes;

	uint32_t mrsas_debug;
	uint32_t mrsas_io_timeout;
	uint32_t mrsas_fw_fault_check_delay;
	uint32_t io_cmds_highwater;
	uint8_t UnevenSpanSupport;
	struct sysctl_ctx_list sysctl_ctx;
	struct sysctl_oid *sysctl_tree;
	struct proc *ocr_thread;
	uint32_t last_seq_num;
	bus_dma_tag_t el_info_tag;
	bus_dmamap_t el_info_dmamap;
	void   *el_info_mem;
	bus_addr_t el_info_phys_addr;
	struct mrsas_pd_list pd_list[MRSAS_MAX_PD];
	struct mrsas_pd_list local_pd_list[MRSAS_MAX_PD];
	struct mrsas_target target_list[MRSAS_MAX_TM_TARGETS];
	uint8_t ld_ids[MRSAS_MAX_LD_IDS];
	struct taskqueue *ev_tq;
	struct task ev_task;
	uint32_t CurLdCount;
	uint64_t reset_flags;
	int	lb_pending_cmds;
	LD_LOAD_BALANCE_INFO load_balance_info[MAX_LOGICAL_DRIVES_EXT];
	LD_SPAN_INFO log_to_span[MAX_LOGICAL_DRIVES_EXT];

	uint8_t mrsas_gen3_ctrl;
	uint8_t secure_jbod_support;
	uint8_t use_seqnum_jbod_fp;
	/* FW suport for more than 256 PD/JBOD */
	uint32_t support_morethan256jbod;
	uint8_t max256vdSupport;
	uint16_t fw_supported_vd_count;
	uint16_t fw_supported_pd_count;

	uint16_t drv_supported_vd_count;
	uint16_t drv_supported_pd_count;

	uint32_t max_map_sz;
	uint32_t current_map_sz;
	uint32_t old_map_sz;
	uint32_t new_map_sz;
	uint32_t drv_map_sz;

	uint32_t nvme_page_size;
	boolean_t is_ventura;
	boolean_t is_aero;
	boolean_t msix_combined;
	boolean_t atomic_desc_support;
	uint16_t maxRaidMapSize;

	/* Non dma-able memory. Driver local copy. */
	MR_DRV_RAID_MAP_ALL *ld_drv_map[2];
	PTR_LD_STREAM_DETECT  *streamDetectByLD;
};

/* Compatibility shims for different OS versions */
#define	mrsas_kproc_create(func, farg, proc_ptr, flags, stackpgs, fmtstr, arg) \
    kproc_create(func, farg, proc_ptr, flags, stackpgs, fmtstr, arg)
#define	mrsas_kproc_exit(arg)   kproc_exit(arg)

static __inline void
mrsas_clear_bit(int b, volatile void *p)
{
	atomic_clear_int(((volatile int *)p) + (b >> 5), 1 << (b & 0x1f));
}

static __inline void
mrsas_set_bit(int b, volatile void *p)
{
	atomic_set_int(((volatile int *)p) + (b >> 5), 1 << (b & 0x1f));
}

static __inline int
mrsas_test_bit(int b, volatile void *p)
{
	return ((volatile int *)p)[b >> 5] & (1 << (b & 0x1f));
}

#include "mrsas_ioctl.h"
extern int mrsas_user_command(struct mrsas_softc *, struct mfi_ioc_passthru *);

#endif					/* MRSAS_H */
