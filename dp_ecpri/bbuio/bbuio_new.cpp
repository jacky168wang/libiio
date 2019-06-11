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
 * @file - bbuio.cpp
 * @brief -  This file defines IO thread for BBU to communicate with front-end
 * @author - Yang Xuebin(xuebin.yang@intel.com) Intel Labs China
 *******************************************************************************/
nclude public/global header files
 *******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* EAL include files */
#include "bbu_pool.h"
#include "task_timing.h"
#include "eNB_extern.h"

#if (ASTRI_GNB_PHY == 1) || (NR_UE_PHY == 1)
#include "NR_RAN_config.h"
#endif

#include <rte_config.h>
#include <rte_mbuf.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_hash.h>
#include <rte_hash_crc.h>
#include <rte_cycles.h>

#if RRU_BBU_INTF_MODE == 1
#include "rpe_ecpri_nr.h"
#endif

#if defined NR_UE || defined NR_BS
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>
#endif
/* LTE stack headers */
#include <utilities.h>

#include <xmmintrin.h> // SSE
#include <emmintrin.h> // SSE 2
#include <pmmintrin.h> // SSE 3
#include <tmmintrin.h> // SSSE 3
#include <smmintrin.h> // SSE 4 for media
#include <immintrin.h> // AVX
#include <arpa/inet.h>
#if defined NR_UE || defined NR_BS
#include "fapiWrapper.h"
#endif
#ifdef L1_L2_DECOUPLE
#include "fapiAdapter.h"
#include "L2_eNB_function.h" //eNB_Mac_Cal_subframe
#ifdef INTEL_IPC
#include "ipc.h"
#endif //INTEL_IPC

#include "astri_cdmUL.h"
#endif // L1_L2_DECOUPLE

#if FAPI_ERR_CHECK == 1
#include "PM_Cdm.h"
#endif

#if PROFILE_LV >= 1
#include "Profile.h"
#endif

#if (TIME_PROFILE_MODE > 0)
#include "TimeProfile.h"
#endif

#ifdef HUGEPAGE_BUFFER
#include "bpool.h"

#ifdef HUGEPAGE_MMAP
// mmap method
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#define FILE_NAME0 "/dev/hugepages/hugepage_buf_0"
#define FILE_NAME1 "/dev/hugepages/hugepage_buf_1"
#define PROTECTION (PROT_READ | PROT_WRITE)
// #ifdef __ia64__
#define ADDR (void *)(0x0UL)
#define FLAGS (MAP_SHARED) // | MAP_FIXED)

static WORD32 hugefd0 = open(FILE_NAME0, O_CREAT | O_RDWR, 0755);
static WORD32 hugefd1 = open(FILE_NAME1, O_CREAT | O_RDWR, 0755);
#endif
#endif
/* RPE Global Variables */
#ifdef FERRY_BRIDGE_INT
extern RpeInstanceHandle handles;
#endif

/*module level globals*/
#define MY_ARGB_LEN 5
static WORD32 nb_ports = 0;
static struct rte_eth_link g_lk_status;
struct ether_addr bbu_port_macs[BBU_MAX_PORTS];

#ifdef FE_INTEGRATION
/* set start subframe for FE integration */
WORD32 g_nFlagStart = 0;
#endif

extern WORD32 bRunFlag;

#ifdef L1_L2_DECOUPLE
#ifdef INTEL_IPC
extern char *g_pFapiMsgBuf[MAX_CELL_NUM][IPC_QUEUE_NUMBER];
extern FAPI_l1ApiMsg_st* g_pFapiMsg[MAX_CELL_NUM][IPC_QUEUE_NUMBER];
extern WORD32 g_nGenDlTasReady[MAX_CELL_NUM];
extern WORD32 g_nGenDlTaskDone[MAX_CELL_NUM];
#endif //INTEL_IPC
#endif //L1_L2_DECOUPLE

#if ASTRI_UPPTS == 1
extern UWORD16 g_UpPTS_len[];
#endif

WORD32 g_BbuioMainLoopStarted = 0;

#if defined(ASTRI_RRU) && defined(ASTRI_RRU_TEST)
#define RRT_TEST_NUM_TRAIL              (100)   
#define RRT_TEST_FLUSH_PKGCOUNT (2048)
#endif

static UWORD64 uPrevTsc;
extern WORD32 nTimeOutCnt;
static WORD32 nIsSubframeIndSent;
static WORD32 nSubrameIndSfIdx = 0;

typedef enum
{
        BBU_TIMING_STATE_UNSYNC = 0,
        BBU_TIMING_STATE_LOCK = 1,
        BBU_TIMING_STATE_SYNC = 2,
} BBU_TIMING_STATE_T;

#define NUM_BBU_RESET_COUNT (80)
#define NUM_BBU_RESYNC_COUNT (80)

static WORD32 g_BbuTimingState = BBU_TIMING_STATE_UNSYNC;
WORD32 g_IsBbuOutOfSync = 0;
WORD32 g_IsBbuUnderReset = 0;
static WORD32 g_BbuResetCount = 0;
static WORD32 g_BbuReSyncCount = 0;

/* local function declaration */
void inline bbuio_process_timing_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, uint8_t PktFmtType);
void inline bbuio_process_prach_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx, uint8_t EarlyCheck);
void inline bbuio_process_pusch_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, uint8_t PktFmtType, WORD32 cIdx, uint8_t EarlyCheck);
#ifdef TD_CONNECT
void inline bbuio_process_td_ul_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx, uint8_t EarlyCheck);
#endif
void inline timing_update(WORD32 nSfIdx, UWORD64 ulTsc, BbuPoolCtrlBlkStruct *pBbu);
#ifdef L1_L2_DECOUPLE
inline WORD32 packet_missed_exception_process_dl(WORD32 nSfIdx, BbuPoolCtrlBlkStruct *pBbu);
#endif
inline WORD32 packet_missed_exception_process(WORD32 nSfIdx, BbuPoolCtrlBlkStruct *pBbu);
inline void bbuio_process_timing_pkt_normal(LteRtInforStruct * pLteRt, WORD32 nSfIdx,
    UWORD64 ulTsc, WORD32 nFlag);

#if RRU_EMU_RX == 1
void inline bbuio_process_pdsch_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, uint8_t PktFmtType, WORD32 cIdx, uint8_t EarlyCheck);
#endif

#if RRU_BBU_INTF_MODE == 1
WORD32 sendNrCsrPacket_SlotFmt(struct rte_mempool *pMPool, WORD32 nCellIdx, WORD32 isDispRRUInfo, WORD32 isLoopBackMode, WORD32 extraPadding, uint8_t VlanTagEnable, uint8_t* SlotFmt, uint8_t NumerIdx, WORD32 nSfIdx);
#endif

#ifdef HUGEPAGE_BUFFER
void buf_alloc_in_hugepage(UWORD64 length, WORD32 socket_id)
{
    BbuBufStruct *psBbuBuf;

#ifdef HUGEPAGE_MMAP
    // input parameter check

    // map buffer on node 0

