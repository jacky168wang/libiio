/*******************************************************************************
 *
 * INTEL CONFIDENTIAL
 * Copyright 2011-2012 Intel Corporation All Rights Reserved.
 * 
 * The source code contained or described herein and all documents related to the
 * source code ("Material") are owned by Intel Corporation or its suppliers or
 * licensors. Title to the Material remains with Intel Corporation or its
 * suppliers and licensors. The Material may contain trade secrets and proprietary
 * and confidential information of Intel Corporation and its suppliers and
 * licensors, and is protected by worldwide copyright and trade secret laws and
 * treaty provisions. No part of the Material may be used, copied, reproduced,
 * modified, published, uploaded, posted, transmitted, distributed, or disclosed
 * in any way without Intels prior express written permission.
 * 
 * No license under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or delivery
 * of the Materials, either expressly, by implication, inducement, estoppel or
 * otherwise. Any license under such intellectual property rights must be
 * express and approved by Intel in writing.
 * 
 * Unless otherwise agreed by Intel in writing, you may not remove or alter this
 * notice or any other notice embedded in Materials by Intel or Intels suppliers
 * or licensors in any way.
 * 
 *  version: MINI_CRAN.L.0.1.0-10
 *
 *******************************************************************************/

/*******************************************************************************
* @file		- bbu_pool.h
* @brief	- This header file defines those data used for BBU IO thread to communicate with front end
* @author	- Xuebin Yang(xuebin.yang@intel.com) Intel Labs China
*******************************************************************************/

#ifndef _BBUIO_H_
#define _BBUIO_H_

#include <linux/types.h>
#ifdef BBU_POOL_SUPPORT
#if ASTRI_TEST_PHASE > 1
#include <rte_config.h>
#include <rte_mbuf.h>

#ifdef __cplusplus
extern "C"
{
#endif
#include "rpe.h"
#include "rpe_common.h"
#include "rpe_cpri.h"
#include "rpe_lte.h"
#include "rpe_packet.h"
#include "rte_version.h"
#ifdef __cplusplus
}
#endif
#endif
#endif

#include "LTE_RAN_Parameters.h"
#include "phy_common_para.h"
#include "task_queue.h"

#define PORT_ID (0)

#define BBU_MAX_PORTS  1
#define N_MAX_BBU_IN_ONE_POOL (128)

#ifdef UE_HT_ENABLE_1
#define N_MAX_Cell_UEMACHINE 1
#else
#if defined UE_HT_ENABLE_6 || defined UE_6_HP_24CORE || defined  UE_6_HP_36CORE
#define N_MAX_Cell_UEMACHINE 6
#else
#if defined UE_HT_ENABLE_12 || defined UE_12_HP_24CORE || defined UE_12_WC_36CORE
#define N_MAX_Cell_UEMACHINE 12
#else
#ifdef UE_HT_ENABLE_24
#define N_MAX_Cell_UEMACHINE 24
#else
#define N_MAX_Cell_UEMACHINE 3
#endif
#endif
#endif
#endif

#ifdef _FAPI_DEBUG_BUILD_
#define TIME_INTERVAL (100) //ms
#elif BBU_POOL_DEBUG
#define TIME_INTERVAL (2) //ms
#elif _BBU_POOL_NIGHTLY_BUILD_
#define TIME_INTERVAL (5) //ms
#else
#define TIME_INTERVAL (1) //ms
#endif

#define TIMEOUT_THRESHOLD  (TIME_INTERVAL*100) // us
#ifdef FE_INTEGRATION
#define START_SUBFRAME     (4800) 
#endif

#define DL_MAX_PACKET 30 // depends on antenna number and BW

#define CPU_HZ g_cpuUsCycles //us

#define MAX_LOST_TIMING_PKT 10 // at most MAX_LOST_TIMING_PKT continues timing packet loss. Else exit this program

#ifdef _FAPI_DEBUG_BUILD_
#define NUM_BUFS  4096
#else
#define NUM_BUFS  16384 
#endif //_FAPI_DEBUG_BUILD_

#define MBUFSZ   (9216 + sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM) /*64 cacheline by 142 */

/* NIC interface config parameters */

#define RX_PTHRESH 8
#define RX_HTHRESH 8
#define RX_WTHRESH 4


#define TX_PTHRESH 36
#define TX_HTHRESH 0
#define TX_WTHRESH 0

#ifdef TD_CONNECT
#define MAX_PKT_BURST 60
#else
#ifdef _ANT8
#define MAX_PKT_BURST 120
#else
#define MAX_PKT_BURST 32
#endif
#endif

#define BBU_IO_RX_DESC 2048 
#define BBU_IO_TX_DESC 2048

#define BBU_FRAME_SIZE 9000

#define N_MAX_BUFFER_SEGMENT  (112 + 1)
#define N_FE_BUF_LEN (40)
#define N_SKIP_FLAG_LEN (2048) // to accomodate very WORD64 timer jitter

/* define the 32bit packet types, little endian */

#define LONG_FE_PKT_TYPE_TIMING (0x0300aaaa)
#define LONG_FE_PKT_TYPE_PRACH (0x0200aaaa)
#define LONG_FE_PKT_TYPE_PUSCH (0x0100aaaa)
#define UE_PKT_TYPE_PDSCH (0x01009999)
#define UE_PKT_TYPE_PDCCH (0x02009999)
#define UE_PKT_TYPE_PBCH  (0x03009999)
#define UE_PKT_TYPE_SS    (0x04009999)
#define UE_PKT_TYPE_CRS   (0x05009999)
//#define TD_PKT_DL   (0x0100bbbb)
//#define TD_PKT_UL   (0x0200bbbb)


/* Ferry Bridge MS3 integration */
// define the 16-bit packet subtypes, big endian for TX
#define ETHERNET_TYPE (0x0800)
#define VLAN_TAG_TPID (0x8100)
#define VLAN_TAG_VLAN_ID (0x001) // CPRI link number

// define the 16-bit packet subtypes, little endian for RX
#define RPE_TIMING_PKT_SUB_TYPE_SWREC       (0x0300)
#define RPE_TIME_DOMAIN_PKT_SUB_SYPE_SWREC  (0x0700)
#define RPE_PUSCH_PKT_SUB_TYPE_SWREC        (0x0B00)
#define RPE_PRACH_PKT_SUB_TYPE_SWREC        (0x0C00)
#define RPE_LTE_PDSCH_PKT_SUB_TYPE_SWREC    (0x0000)
#define RPE_LTE_PDCCH_PKT_SUB_TYPE_SWREC    (0x0100)

// define macro value from the structure defination of timing and time domain packet
#define RPE_RADIO_FRAME_LOW_BIT 4
#define RPE_RADIO_FRAME_LOW_MAX_NUM 16
#define RADIO_FRAME_START 2000

#define NUM_RX_QUEUES 128
#define CTRL_TX_Q_ID 0

#ifdef _FAPI_DEBUG_BUILD_
#define DATA_TX_Q_ID 0
#else
#define DATA_TX_Q_ID 1
#endif //_FAPI_DEBUG_BUILD_

#define CTRL_RX_Q_ID 10
#define DATA_RX_Q_ID 15
/* Ferry Bridge MS3 integration */

#define ETH_HEADER_SIZE (14)
#define UL_ETH_FRAME_MAX_LEN (8680)


#define BIT_PER_LINE (64)
#define DRBMAP_SEG_SIZE (2)
#define DRBMAP_PER_LINE (BIT_PER_LINE/DRBMAP_SEG_SIZE)
#define URBMAP_SEG_SIZE (3)
#define URBMAP_PER_LINE (20)
#define URBMAP_RSRV_BIT (BIT_PER_LINE - URBMAP_SEG_SIZE * URBMAP_PER_LINE)
#define N_MAX_ADDR_PER_SUBFRAME (7) /* FE delivers UL IQ sample to at most N_MAX_ADDR_PER_SUBFRAME 
                                        BBUs in one subframe  */
#define PDCCH_SEG_SIZE (2)
#define RE_PER_LINE (BIT_PER_LINE / PDCCH_SEG_SIZE)

#define RE_PER_LINE_PBCH (18)
#define BIT_RESERVE_PER_LINE_PBCH (28)

#define SS_SEG_SIZE (16)
#define BIT_RESERVE_PER_LINE_SS (32)

#define RE_IN_LAST_LINE_CRS (16)
#define BIT_RESERVE_LAST_LINE_CRS (32)

typedef struct {
    WORD32 nMacAddr;
	UWORD8 cMacAddr[N_MAX_BBU_IN_ONE_POOL][6];
	UWORD8 cBbuIpAddr[1][4];
	UWORD8 cIpAddr[N_MAX_BBU_IN_ONE_POOL][4];
} MacAddrGroupStruct;

typedef enum {
    DPDK_RX_NORMAL = 0,
    DPDK_RX_TIMEOUT
} DpdkRxStatusEnum;

/* FE Configuration access packet payloads */
struct fe_reg_access_pkt {
    uint16_t rsvd;
    uint8_t be:4;
    uint8_t tag:3;
    uint8_t rw:1;
    uint8_t addr_h:4;
    uint8_t devsel:4;
    uint16_t addr_l:16;
    uint32_t payload1;
} __attribute__((__packed__));
#if 0
/* Time domain Package header */
struct cpri_hdr_s {
    uint16_t sub_type;
    uint32_t status_h:24;    
    uint8_t  status_l:5;
    uint8_t  ant_seq:3;
    uint16_t frame_seq;
    uint8_t  symb_seq:4;
    uint8_t  subf_seq:4;
} __attribute__((packed));
#endif
/* PDSCH Packt header*/
struct pdsch_hdr_s {
    uint16_t sub_type;
    uint32_t status_h:24;
    //uint8_t status_m;
    uint8_t  blk_len_h:5;
    uint8_t  status_l:3;
    uint8_t  blk_start:3;
    uint8_t  blk_cnt:3;
    uint8_t  blk_len_l:2;
    uint16_t frame_seq;
    uint8_t  sym_seq:4;
    uint8_t  subf_seq:4;
} __attribute__((packed));

/* PDCCH Packet Header */
struct pdcch_hdr_s {
    uint16_t sub_type;
    uint32_t status_h:24;
    uint8_t  rsvd1:5;
    uint8_t  status_l:3;
    uint8_t  rsvd2;
    uint16_t frame_seq;
    uint8_t  pdcchcnt:2;
    uint8_t  urbcnt:2;
    uint8_t  subf_seq:4;
} __attribute__((packed));

struct drbmap_block_s {
    uint64_t rbmap[4]; /* each entry is divided into 32 2-bit group. 0 -- RB not used
                                          1/2/3 -RB used, decomppress table 1/2/3 */
};

struct urbmap_block_s {
    uint8_t rsvd:1; /* reserved bit */
    uint8_t SubframeSeq:4; /* subframe number */
    uint8_t NextFrame:1; /* 1 -- this URBBLOCK indexs a subframe in next frame from this PDCCH packet */    
    uint8_t SSbuframe:1; /* 1 -- one switch subframe */
    uint8_t PRACH:1; /* 1 -- do prach operation for this subframe */
    uint8_t DestAddr[7]; /* 7 index of desination address used to transfer this upstream data. 1 
        subframe data can be distributed to at most 7 servers */
    uint64_t rbmap[5]; /* each entry is divided into 20 3-bit group with last 5 bit  (LSBs) are reserved. group = 0 means 
                                              the RB is not used, 1-7 means the RB should be delived corresponding destination server */
};

typedef struct _PDCCH_BLK {
	UWORD8 x3:2;
	UWORD8 x2:2;
	UWORD8 x1:2;
	UWORD8 x0:2;

	UWORD8 x7:2;
	UWORD8 x6:2;
	UWORD8 x5:2;
	UWORD8 x4:2;

	UWORD8 x11:2;
	UWORD8 x10:2;
	UWORD8 x9:2;
	UWORD8 x8:2;

	UWORD8 Hint2:2;
	UWORD8 Hint1:2;
	UWORD8 Hint0:2;
	UWORD8 Method:2;
} PdcchBlkStruct;


struct pdcch_block_s {
    uint64_t pcfichmap;
	uint64_t hintmap[38];
    uint64_t pdcchmap[38];
};

/* PBCH Packet Header */
struct pbch_hdr_s {
    uint16_t sub_type;
} __attribute__((packed));

struct pbch_block_s {
    uint64_t pbchmap[16]; /* 4 symbol pbch data */
};

/* PSS/SSS Packet Header */
struct ss_hdr_s {
    uint16_t sub_type;
} __attribute__((packed));

struct ss_block_s {
    uint64_t pssmap[72];
	uint64_t sssmap[144];
};

/* CRS Packet Header */
struct crs_hdr_s {
    uint16_t sub_type;
    uint8_t  port0Offset;
    uint8_t  port1Offset;
	uint8_t  port2Offset;
	uint8_t  port3Offset;
    uint32_t rsvd;
} __attribute__((packed));

struct crs_ant0_block_s {
	uint64_t crsmap[275];
};
struct crs_ant1_block_s {
	uint64_t crsmap[275];
};
struct crs_ant2_block_s {
	uint64_t crsmap[138];
};
struct crs_ant3_block_s {
	uint64_t crsmap[138];
};

/* PUSCH Packet Header */
struct pusch_hdr_s {
    uint16_t sub_type;
    uint32_t status:24;
    uint8_t blk_len_h:5;
    uint8_t addr:3;
    uint8_t blk_start:3;
    uint8_t blk_cnt:3;
    uint8_t blk_len_l:2; 
    uint16_t frame_seq;
    uint8_t sym_seq:4;
    uint8_t subf_seq:4;
} __attribute__((packed));

/* PRACH Packet Header */
struct prach_hdr_s {
    uint16_t sub_type;
    uint32_t status_h:24;
    uint16_t rsvd1:5;
    uint8_t  status_l:3;
    uint8_t  rsvd2:8;
    uint16_t frame_seq;
    uint8_t ant_id:4;
    uint8_t subf_seq:4;
} __attribute__((packed));

/* Timing Packet Header */
#if !defined TD_CONNECT && !defined FB_API
struct timing_hdr_s {
    uint16_t sub_type;
    uint32_t status_h;
    uint8_t status_l;
    uint16_t frame_seq;
    uint8_t rsvd:4;
    uint8_t subf_seq:4;
} __attribute__((packed));
#endif

/* 
 * manage one cell's all Ethernet frames for one DL or UL LTE subframe
 */
typedef struct {
    /* -1-this subframe is not used in current frame format
         0-this subframe can be transmitted, i.e., data is ready
          1-this subframe is waiting transmission, i.e., data is not ready
         10 - DL transmission missing deadline. When FE needs this subframe data but bValid is still 1, 
        set bValid to 10.
    */
    WORD32 bValid ; // when UL rx, it is subframe index.
    WORD32 nSegToBeGen;
    WORD32 nSegGenerated; // how many date segment are generated by DL LTE processing or received from FE
                       // -1 means that DL packet to be transmitted is not ready in BS
    WORD32 nSegTransferred; // number of data segments has been transmitted or received
#ifdef L1_L2_DECOUPLE
	WORD32 nL1dPkgTransferred;
	WORD32 nFapiSchedulerDone;	// this flag is set after UL or DL Scheduler task is done
	WORD32 nFapiSubFrameDone;	// this flag is set after subframe indication is generated
	WORD32 nFapiTaskGenerated;
#endif
    struct rte_mbuf *pData[N_MAX_BUFFER_SEGMENT]; // point to DPDK allocated memory pool
} BbuIoBufCtrlStruct;

typedef struct  {
    UWORD64 nCoreMask;
    struct rte_mempool *bbuio_buf_pool;
#ifdef ASTRI_PHY
	struct rte_mempool *bbuio_Rxbuf_pool;
#endif
    struct rte_ring * tx_rings[UE_PHY_THREAD_NR];
    UWORD8 ucFeMac[N_MAX_CELL_PER_BBU][6]; /* Front end board MAC address */
    
#ifndef USED_BY_UE
    BbuIoBufCtrlStruct sDlBbuIoBufCtrl[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU + 1]; // PDSCH packet
    BbuIoBufCtrlStruct sPdcchBbuIoBufCtrl[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU + 1]; // PDCCH packet
    WORD16 nUlTaskGeneratedFlag[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU + 1];
    WORD16 nRachTaskGeneratedFlag[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU + 1];
    TaskSkipEnum eSubframeSkipFlag[N_SKIP_FLAG_LEN]; // skip flag for PUSCH
	TaskSkipEnum eSubframeRachSkipFlag[N_SKIP_FLAG_LEN]; // skip flag for PRACH
    TaskSkipEnum eDlTxSkipFlag[N_SKIP_FLAG_LEN];
    BbuIoBufCtrlStruct sCsrBbuIoBufCtrl[N_MAX_CELL_PER_BBU + 1]; //For sending CSR
#else
    BbuIoBufCtrlStruct sDlDataBbuIoBufCtrl[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU];
    BbuIoBufCtrlStruct sDlControlBbuIoBufCtrl[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU];
	BbuIoBufCtrlStruct sDlPbchBbuIoBufCtrl[N_MAX_CELL_PER_BBU];
	BbuIoBufCtrlStruct sDlSSBbuIoBufCtrl[N_MAX_CELL_PER_BBU];
	BbuIoBufCtrlStruct sDlCrsBbuIoBufCtrl[N_MAX_CELL_PER_BBU];
    WORD32 nSfIdx;
#endif

    // written according to received packet
    BbuIoBufCtrlStruct sUlBbuIoBufCtrl[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU + 1];
    
    BbuIoBufCtrlStruct sRachBbuIoBufCtrl[N_FE_BUF_LEN][N_MAX_CELL_PER_BBU + 1];

    WORD32  nTimingOffset[N_MAX_CELL_PER_BBU + 1];  // in us
    UWORD64 nTscTiming[N_FE_BUF_LEN]; // records the TSC when a timing packet is received.
    //UWORD8 ucBbuMacTable[N_MAX_BBU_IN_ONE_POOL]; // stores mac index of all BBUs in this pool
} BbuFeIoIfStruct;

/* function declaration */
WORD32 bbuio_module_init(BbuFeIoIfStruct *ioinfo, WORD32 eal_argc = 0, WORD8 ** eal_argv = NULL);
WORD32 bbuio_init(BbuFeIoIfStruct *ioinfo);
WORD32 bbuio_exit(void);
WORD32 bbuio_thread(void *pVoid);

WORD32 readMacAddrGroup(WORD8 *fn, MacAddrGroupStruct *pMacAddrGroup);

/* RPE setup function declaration */
#ifdef FERRY_BRIDGE_INT
#ifndef USED_BY_UE
int setupRpe(int port, int rxQueueId, int txQueueId, BbuFeIoIfStruct *ioinfo);
#else
int setupRpe(int port, int rxQueueId, int txQueueId, struct rte_mempool *pRxMbufPool, struct rte_mempool *pTxMbufPool, int nAntNum);
#endif
WORD32 repackage_small_packets(struct rte_mbuf *pRxMBuf, void *pData, UWORD8 hdrSize);
#endif

#if ASTRI_TEST_PHASE > 1
#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
#define	RTE_PKTMBUF_READ(m, off, len, buf) rte_pktmbuf_read(m, off, len, buf)
#else
#define	RTE_PKTMBUF_READ(m, off, len, buf) ((void *)((char *)(m)->pkt.data + (off)))
#endif

#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
#define	RTE_PKT_LEN(m) ((m)->pkt_len)
#else
#define	RTE_PKT_LEN(m) ((m)->pkt.data_len)
#endif
#endif

#endif /* _MAIN_H_ */
