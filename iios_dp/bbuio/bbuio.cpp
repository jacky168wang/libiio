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

/*******************************************************************************
 * Include public/global header files
 *******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* EAL include files */
#include "bbu_pool.h"
#include "task_timing.h"
#include "eNB_extern.h"

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

/* LTE stack headers */
#include <utilities.h>

#include <xmmintrin.h> // SSE
#include <emmintrin.h> // SSE 2
#include <pmmintrin.h> // SSE 3
#include <tmmintrin.h> // SSSE 3
#include <smmintrin.h> // SSE 4 for media
#include <immintrin.h> // AVX
#include <arpa/inet.h>

#ifdef L1_L2_DECOUPLE
#include "fapiAdapter.h"
#include "L2_eNB_function.h" //eNB_Mac_Cal_subframe
#ifdef INTEL_IPC
#include "ipc.h"
#endif //INTEL_IPC
#define TIME_DELAY_ON_AIR_SUBFRAME_START	 357 // refer to FAPI doc Appendix B, (2.000 - 1.643) * 1000 = 357 ns
//#define TIME_DELAY_ON_AIR_SUBFRAME_START	 57 // Give more time to DL to prevent PDSCH packet timeout at RRU program, by stanley lam 20170228


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
#define RRT_TEST_NUM_TRAIL 		(100)	
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
void inline bbuio_process_timing_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt);
void inline bbuio_process_prach_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx);
void inline bbuio_process_pusch_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx);
#ifdef TD_CONNECT
void inline bbuio_process_td_ul_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx);
#endif
void inline timing_update(WORD32 nSfIdx, UWORD64 ulTsc, BbuPoolCtrlBlkStruct *pBbu);
#ifdef L1_L2_DECOUPLE
inline WORD32 packet_missed_exception_process_dl(WORD32 nSfIdx, BbuPoolCtrlBlkStruct *pBbu);
#endif
inline WORD32 packet_missed_exception_process(WORD32 nSfIdx, BbuPoolCtrlBlkStruct *pBbu);
inline void bbuio_process_timing_pkt_normal(LteRtInforStruct * pLteRt, WORD32 nSfIdx,
    UWORD64 ulTsc, WORD32 nFlag);

#ifdef HUGEPAGE_BUFFER
void buf_alloc_in_hugepage(UWORD64 length, WORD32 socket_id)
{
    BbuBufStruct *psBbuBuf;

#ifdef HUGEPAGE_MMAP
    // input parameter check

    // map buffer on node 0
    psBbuBuf = &sBbuPhyBuf[0];
    psBbuBuf->pData = mmap(ADDR, length, PROTECTION, FLAGS, hugefd0, 0);
    if (psBbuBuf->pData == MAP_FAILED)
    {
        perror("mmap");
        unlink(FILE_NAME0);
        exit(-1);
    }
    psBbuBuf->nTotalLen = length;
    psBbuBuf->nBufOffset = 0;

    // map buffer on node 1
    bind_to_cpu(gettid(), CORE9_CPUID);
    psBbuBuf = &sBbuPhyBuf[1];
    psBbuBuf->pData = mmap(ADDR, length, PROTECTION, FLAGS, hugefd1, 0);
    if (psBbuBuf->pData == MAP_FAILED)
    {
        perror("mmap");
        unlink(FILE_NAME1);
        exit(-1);
    }
    psBbuBuf->nTotalLen = length;
    psBbuBuf->nBufOffset = 0;

    bind_to_cpu(gettid(), CORE1_CPUID);
#else
    const struct rte_memzone *mz;
    const struct rte_memseg *ms;
    WORD32 hugepage_2MB_avail = 0;
    WORD32 hugepage_1GB_avail = 0;
    WORD32 i = 0;

    ms = rte_eal_get_physmem_layout();
    for (i = 0; i < RTE_MAX_MEMSEG; i++) {
        if (ms[i].hugepage_sz == RTE_PGSIZE_2M)
            hugepage_2MB_avail = 1;
        if (ms[i].hugepage_sz == RTE_PGSIZE_1G)
            hugepage_1GB_avail = 1;
    }

    /* reserve the mz buffer */
    if (hugepage_2MB_avail)
        mz = rte_memzone_reserve("flag_zone_2M", length, socket_id, RTE_MEMZONE_2MB);
    else if (hugepage_1GB_avail)
        mz = rte_memzone_reserve("flag_zone_1G", length, socket_id, RTE_MEMZONE_1GB);

    if (mz != NULL) {
        // init the buffer pool zone
        psBbuBuf->pData = mz->addr;
        psBbuBuf->nTotalLen = length;
        psBbuBuf->nBufOffset = 0;
        psBbuBuf->nSocketId = socket_id;
        printf("###### buffer reserved sucessfully\n");
    }
    else
    {
        printf("Error, buffer reserve failed\n");
        exit(-1);
    }
#endif
}

void buf_free_in_hugepage(UWORD64 length, WORD32 socket_id)
{
    BbuBufStruct *psBbuBuf;

#ifdef HUGEPAGE_MMAP
    psBbuBuf = &sBbuPhyBuf[0];
    munmap(psBbuBuf->pData, length);
    close(hugefd0);
    unlink(FILE_NAME0);

    psBbuBuf = &sBbuPhyBuf[1];
    munmap(psBbuBuf->pData, length);
    close(hugefd1);
    unlink(FILE_NAME1);
#endif
}
#endif

/* code bbuio as an module, so we do not need all EAL standard arguments, pass our own argc argv to EAL environemtn for testing */
WORD32 bbuio_module_init(BbuFeIoIfStruct *ioinfo, WORD32 eal_argc, WORD8 ** eal_argv)
{
    WORD32 ret = 0;

#ifdef ASTRI_PHY
	WORD32 my_argc = 0;
	WORD8 * my_argv[32];
	WORD8  argvData[1024];
	WORD32 dataDataOffset = 0;
	WORD32 i;
	WORD32 isCoreMaskPresent = 0, isMemChnNumPresent = 0, isVerPresent = 0;

	printf("RTE Version used during compilation: %s\n", rte_version());
	
	memset(my_argv, 0, sizeof(my_argv));
	memset(argvData, 0, sizeof(argvData));

	// my_argv[0]: program name "bs_bbu_main"
	my_argv[my_argc] = argvData + dataDataOffset;
	sprintf(my_argv[my_argc], "bs_bbu_main");
	dataDataOffset += strlen(my_argv[my_argc]) + 1; // +1 is the length of NULL charactor
	my_argc++;

	// my_argv[1] to my_argv[N]: eal_argv from user input
	for (i = 0; i < eal_argc; i++)
	{
		my_argv[my_argc] = argvData + dataDataOffset;
		strcpy(my_argv[my_argc], eal_argv[i]);
		dataDataOffset += strlen(my_argv[my_argc]) + 1; // +1 is the length of NULL charactor
		my_argc++;

		// also check the present of "-v", "-c", "-l", "-n" option
		if (strstr(eal_argv[i], "-v") != NULL)
			isVerPresent = 1;
		if (strstr(eal_argv[i], "-c") != NULL)
			isCoreMaskPresent = 1;
		if (strstr(eal_argv[i], "-l") != NULL)
			isCoreMaskPresent = 1;
		if (strstr(eal_argv[i], "-n") != NULL)
			isMemChnNumPresent = 1;

	}

	// my_argv[N+1]: rte_version if user not provided
	if (isMemChnNumPresent == 0)
	{
		my_argv[my_argc] = argvData + dataDataOffset;
		sprintf(my_argv[my_argc], "-v");
		dataDataOffset += strlen(my_argv[my_argc]) + 1; // +1 is the length of NULL charactor
		my_argc++;
	}	

	// my_argv[N+2]: core mask if user not provided
	if (isCoreMaskPresent == 0)
	{
		my_argv[my_argc] = argvData + dataDataOffset;
#ifdef _FAPI_DEBUG_BUILD_
		sprintf(my_argv[my_argc], "-c %llx", 3);
#else
		sprintf(my_argv[my_argc], "-c %llx", ioinfo->nCoreMask);
#endif
		dataDataOffset += strlen(my_argv[my_argc]) + 1; // +1 is the length of NULL charactor
		my_argc++;
	}

	// my_argv[N+3]: memory channel number if user not provided
	if (isMemChnNumPresent == 0)
	{
		my_argv[my_argc] = argvData + dataDataOffset;
		sprintf(my_argv[my_argc], "-n 4");
		dataDataOffset += strlen(my_argv[my_argc]) + 1; // +1 is the length of NULL charactor
		my_argc++;
	}	

	printf("Call rte_eal_init() with options: ");
	for (i = 1; i < my_argc; i++)
	{	
		printf("%s ", my_argv[i]);
	}
	printf("\n");

	/* intialize EAL environment */
	ret = rte_eal_init(my_argc, my_argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "rte_eal_init failed\n");

#else
    WORD32 argc = MY_ARGB_LEN;
    UWORD64 uCoreMask = ioinfo->nCoreMask;
    WORD8 *my_argv[MY_ARGB_LEN], cTmp[100];
    my_argv[0] = (WORD8 *)malloc(100);
    sprintf(my_argv[0], "bs_bbu_main");
    my_argv[1] = (WORD8 *)malloc(100);
    sprintf(my_argv[1], "-c %llx", uCoreMask);
    my_argv[2] = (WORD8 *)malloc(100);
#ifdef HUGEPAGE_MMAP
    sprintf(my_argv[2], "-m1024");
#else
	  sprintf(my_argv[2], "");    
#endif
    my_argv[3] = (WORD8 *)malloc(100);
    snprintf(my_argv[3], sizeof(my_argv[3]), "-n4");
    my_argv[4] = (WORD8 *)malloc(100);
    sprintf(my_argv[4], "-b  0000:02:00.1");

#ifdef _FAPI_DEBUG_BUILD_
	sprintf(my_argv[1], "-c 3");
#endif //_FAPI_DEBUG_BUILD_	

    /* intialize EAL environment */
    ret = rte_eal_init(argc, my_argv);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "rte_eal_init failed\n");
	
#endif //ASTRI_PHY
	

#ifdef HUGEPAGE_BUFFER
    /* reserve buffer for physical layer mem pool */
    buf_alloc_in_hugepage(PHY_BUFFER_LENGTH, SOCKET_ID_ANY);
#endif

    /* bbuio_init must be called after EAL initializtion done! */
    ret = bbuio_init(ioinfo);
    if (ret < 0 ) {
        printf("FE TX RX module intiailization error\n");
        bbuio_exit();
    }
    return  ret;
    /*call the module destroy routine*/
    /* enable promisc */
    // rte_eth_promiscuous_enable(0);
}

static void lsi_event_callback(uint8_t port_id, enum rte_eth_event_type msg_type, void *param)
{
    struct rte_eth_link lk_status;
    RTE_SET_USED(param);
    printf(" Link Status Update ...\n");
    printf(" Event type : %s\n", msg_type == RTE_ETH_EVENT_INTR_LSC?"LSC IRQ":"not our irq");
    rte_eth_link_get(port_id, &lk_status);
    if (lk_status.link_status){
        printf("port 0 status up speed %u Mbps\n", (unsigned)lk_status.link_speed);
    } else
        printf("port 0 link down");
}

/* port config structure */
static const struct rte_eth_conf portcfg = {
    .rxmode  = {
#ifdef FERRY_BRIDGE_INT
        .mq_mode        = ETH_MQ_RX_VMDQ_DCB,
#endif
        .split_hdr_size = 0,
        .header_split = 0,
        .hw_ip_checksum = 0,
        .hw_vlan_filter = 0,
        .jumbo_frame = 1,
        .hw_strip_crc = 0,
        .max_rx_pkt_len = BBU_FRAME_SIZE,
    },
    .txmode = {
#ifdef FERRY_BRIDGE_INT
        .mq_mode = ETH_MQ_TX_NONE,
#endif
    },
    .rx_adv_conf.vmdq_dcb_conf = {
	    .nb_queue_pools = ETH_16_POOLS,
	    .enable_default_pool = 1,
	    .default_pool = 0,
	    .nb_pool_maps = 0,
	    .pool_map = {{0, 0},},
#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
	    .dcb_tc = {0},
#else
	    .dcb_queue = {0},
#endif
    },
    //.intr_conf = {
    //    .lsc = 1,
    //},
};

/* rx config structure */
static struct rte_eth_rxconf rx_cfg = {
    .rx_thresh = {
        .pthresh = RX_PTHRESH,
        .hthresh = RX_HTHRESH,
        .wthresh = RX_WTHRESH,
    },
};

/* tx config structure */
static struct rte_eth_txconf tx_cfg = {
    .tx_thresh = {
        .pthresh = TX_PTHRESH,
        .hthresh = TX_HTHRESH,
        .wthresh = TX_WTHRESH,
    },
    .tx_free_thresh = 0, /*use default */
    .tx_rs_thresh = 0, /*use default */
};

static uint16_t bbu_rx_desc_n = BBU_IO_RX_DESC;
static uint16_t bbu_tx_desc_n = BBU_IO_TX_DESC;

static enum rte_eth_nb_pools num_pools = ETH_16_POOLS;
const uint16_t vlan_tags[] = {
    0,  1,  2,  3,  4,  5,  6,  7,
    8,  9,  10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29, 30, 31
};

/* Builds up the correct configuration for vmdq+dcb based on the vlan tags array
 *  *  * given above, and the number of traffic classes available for use. */
static inline int
get_eth_conf(struct rte_eth_conf *eth_conf, enum rte_eth_nb_pools num_pools)
{
    struct rte_eth_vmdq_dcb_conf conf;
    unsigned i;

    if (num_pools != ETH_16_POOLS && num_pools != ETH_32_POOLS ) return -1;

    conf.nb_queue_pools = num_pools;
    conf.enable_default_pool = 0;
    conf.default_pool = 0; /* set explicit value, even if not used */
    conf.nb_pool_maps = sizeof( vlan_tags )/sizeof( vlan_tags[ 0 ]);
    for (i = 0; i < conf.nb_pool_maps; i++)
    {
        conf.pool_map[i].vlan_id = vlan_tags[ i ];
        conf.pool_map[i].pools = 1 << (i % num_pools);
    }
    for (i = 0; i < ETH_DCB_NUM_USER_PRIORITIES; i++)
    {
#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
        conf.dcb_tc[i] = (uint8_t)(i % (NUM_RX_QUEUES/num_pools));
#else
        conf.dcb_queue[i] = (uint8_t)(i % (NUM_RX_QUEUES/num_pools));
#endif
    }
    (void)(rte_memcpy(eth_conf, &portcfg, sizeof(*eth_conf)));
    (void)(rte_memcpy(&eth_conf->rx_adv_conf.vmdq_dcb_conf, &conf,
        sizeof(eth_conf->rx_adv_conf.vmdq_dcb_conf)));
    return 0;
}

WORD32 bbuio_init(BbuFeIoIfStruct *ioinfo)
{
    WORD32 ret = 0;
    static struct rte_mempool * bbuio_buf_pool = ioinfo->bbuio_buf_pool;
#ifdef ASTRI_PHY
	static struct rte_mempool * bbuio_Rxbuf_pool = ioinfo->bbuio_Rxbuf_pool;
#endif //ASTRI_PHY
    struct rte_eth_dev_info dev_info;
    uint8_t portID = PORT_ID;
    struct rte_eth_conf port_conf;
    /* create IO memory buffer pool */
#ifdef FERRY_BRIDGE_INT
#ifndef SMALL_PACKET
    uint16_t size = 11264;
#endif
    bbuio_buf_pool = rte_mempool_create("bbu_mem", 65536,
                 MBUFSZ,256,
                 sizeof(struct rte_pktmbuf_pool_private),
#ifdef SMALL_PACKET
                 rte_pktmbuf_pool_init, NULL,
#else
#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
                 rte_pktmbuf_pool_init, NULL,
#else
                 rte_pktmbuf_pool_init, &size,
#endif
#endif
                 rte_pktmbuf_init, NULL,
                 0, 0);
#else

#ifdef ASTRI_PHY
#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
    bbuio_buf_pool = rte_mempool_create("bbu_mem", NUM_BUFS, \
                 MBUFSZ,128,\
                 sizeof(struct rte_pktmbuf_pool_private),\
                 rte_pktmbuf_pool_init, NULL,\
                 rte_pktmbuf_init, NULL,
                 0, 0);//MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET); //MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);
    bbuio_Rxbuf_pool = rte_mempool_create("bbu_mem_rx", NUM_BUFS, \
                 MBUFSZ,128,\
                 sizeof(struct rte_pktmbuf_pool_private),\
                 rte_pktmbuf_pool_init, NULL,\
                 rte_pktmbuf_init, NULL,
                 0, 0);//MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET); //MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);
#else
    bbuio_buf_pool = rte_mempool_create("bbu_mem", NUM_BUFS, \
                 MBUFSZ,128,\
                 sizeof(struct rte_pktmbuf_pool_private),\
                 rte_pktmbuf_pool_init, (void *) MBUFSZ,\
                 rte_pktmbuf_init, NULL,
                 0, 0);//MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET); //MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);
    bbuio_Rxbuf_pool = rte_mempool_create("bbu_mem_rx", NUM_BUFS, \
                 MBUFSZ,128,\
                 sizeof(struct rte_pktmbuf_pool_private),\
                 rte_pktmbuf_pool_init, (void *) MBUFSZ,\
                 rte_pktmbuf_init, NULL,
                 0, 0);//MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET); //MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);
#endif

#else
#if RTE_VERSION >= RTE_VERSION_NUM(16,11,0,0)
    bbuio_buf_pool = rte_mempool_create("bbu_mem", NUM_BUFS, \
                 MBUFSZ,32,\
                 sizeof(struct rte_pktmbuf_pool_private),\
                 rte_pktmbuf_pool_init, NULL
                 rte_pktmbuf_init, NULL,
                 0, 0);//MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET); //MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);
#else
    bbuio_buf_pool = rte_mempool_create("bbu_mem", NUM_BUFS, \
                 MBUFSZ,32,\
                 sizeof(struct rte_pktmbuf_pool_private),\
                 rte_pktmbuf_pool_init, (void *) MBUFSZ,\
                 rte_pktmbuf_init, NULL,
                 0, 0);//MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET); //MEMPOOL_F_SP_PUT | MEMPOOL_F_SC_GET);
#endif
#endif //ASTRI_PHY
#endif

#ifdef ASTRI_PHY
	ioinfo->bbuio_Rxbuf_pool = bbuio_Rxbuf_pool;
	// rte_mempool_dump(bbuio_Rxbuf_pool);
	if (!bbuio_Rxbuf_pool)
		rte_panic("cannot init mem pool\n");
#endif //ASTRI_PHY

    ioinfo->bbuio_buf_pool = bbuio_buf_pool;
    // rte_mempool_dump(bbuio_buf_pool);
    if (!bbuio_buf_pool)
        rte_panic("cannot init mem pool\n");

    ret = get_eth_conf(&port_conf, num_pools);
    if(ret != 0)
       return ret;

    nb_ports = rte_eth_dev_count();
    if (0==nb_ports)
        rte_panic("No 10GE detected\n");

#if defined(ASTRI_DAUL_PORT_NIC)
	if (nb_ports < 2)
		rte_panic("Not enough 10GE, nb_ports = %d\n", nb_ports);
#endif


#if defined(ASTRI_DAUL_PORT_NIC)
	for (int portIdx = 0; portIdx < 2; portIdx++)
	{
		portID = portIdx;
		printf("**** Setup portID %d ****\n", portID);
#endif	

    /* check nic device info */
    rte_eth_dev_info_get(portID,&dev_info);
    printf("dev-info max tx queue = %d, max rx queue = %d\n", \
     dev_info.max_tx_queues, dev_info.max_rx_queues);

    /* configure nic port 0 */
    printf("Initializing port %d\n", portID);
#ifndef FERRY_BRIDGE_INT
    ret = rte_eth_dev_configure(portID,1,1,&port_conf);
#else
    short rxRing = 128;
    short txRing = 2;
    ret = rte_eth_dev_configure(portID, rxRing, txRing, &port_conf);
#endif
    if (ret < 0)
        return ret;

    rte_eth_dev_callback_register(portID,RTE_ETH_EVENT_INTR_LSC, lsi_event_callback, NULL);
/*  it will lead to some DPDK initialization error. Comment it.
    rte_eth_link_get(portID,&g_lk_status);
    printf("link status %s\n", g_lk_status.link_status?"UP":"Down");
 */

    /* setup rx queue */
#ifndef FERRY_BRIDGE_INT
#ifdef ASTRI_PHY
	ret = rte_eth_rx_queue_setup(portID, 0, bbu_rx_desc_n, 0,&rx_cfg\
					,bbuio_Rxbuf_pool);
	if (ret < 0) {
		printf("error setup rx queue resources \n");
		return ret;
	}
#else
    ret = rte_eth_rx_queue_setup(portID, 0, bbu_rx_desc_n, 0,&rx_cfg\
                    ,bbuio_buf_pool);
    if (ret < 0) {
        printf("error setup rx queue resources \n");
        return ret;
    }
#endif //ASTRI_PHY
#else
    for (int irx = 0; irx < rxRing; irx++){
        ret = rte_eth_rx_queue_setup(portID, irx, 128, rte_eth_dev_socket_id(portID), &rx_cfg\
                        ,bbuio_buf_pool);
        if (ret < 0) {
            printf("error setup rx queue resources \n");
            return ret;}
    }
#endif

    /* setup tx queue */
#ifndef FERRY_BRIDGE_INT
    ret = rte_eth_tx_queue_setup(portID, 0, bbu_tx_desc_n, 0, &tx_cfg);
    if (ret < 0) {
        printf("error setup tx queue resources \n");
        return ret;
    }
#else
    for (int itx = 0; itx < txRing; itx++){
        ret = rte_eth_tx_queue_setup(portID, itx, 512, rte_eth_dev_socket_id(portID), &tx_cfg);
        if (ret < 0) {
            printf("error setup tx queue resources \n");
            return ret;
        }
    }
#endif


    /*start device */
    ret = rte_eth_dev_start(portID);

    if (ret < 0) {
        printf("error startup port %d \n", portID);
        return ret;
    }

#ifdef FERRY_BRIDGE_INT
    struct ether_addr pDstEthAddr;
    pDstEthAddr.addr_bytes[0] = 0xFF;
    pDstEthAddr.addr_bytes[1] = 0xFF;
    pDstEthAddr.addr_bytes[2] = 0xFF;
    pDstEthAddr.addr_bytes[3] = 0xFF;
    pDstEthAddr.addr_bytes[4] = 0xFF;
    pDstEthAddr.addr_bytes[5] = 0xFF;
    ret = rte_eth_dev_mac_addr_add(portID, &pDstEthAddr, 1);
#endif

    rte_eth_macaddr_get(portID, &bbu_port_macs[portID]);
    printf("port %u, MAC = %02X:%02X:%02X:%02X:%02X:%02X\n", portID,
        bbu_port_macs[portID].addr_bytes[0],
        bbu_port_macs[portID].addr_bytes[1],
        bbu_port_macs[portID].addr_bytes[2],
        bbu_port_macs[portID].addr_bytes[3],
        bbu_port_macs[portID].addr_bytes[4],
        bbu_port_macs[portID].addr_bytes[5]);

#ifdef FE_INTEGRATION
    /* enable promisc */
    rte_eth_promiscuous_enable(0);
#endif

#ifdef FERRY_BRIDGE_INT
    /* enable promisc */
    //rte_eth_promiscuous_enable(portID);
    printf("port %d success\n", portID);

    sleep(3);

    ret = setupRpe(portID, CTRL_RX_Q_ID, CTRL_TX_Q_ID, ioinfo);

    if(ret != 0)
        return ret;
#endif

#if defined(ASTRI_DAUL_PORT_NIC)
	} // for loop portIdx
#endif	

    return 0;
}

WORD32 bbuio_exit(void)
{
    printf("bbu IO module exit\n");
    return 0;
}

static struct ether_addr sDstEthAddr;

/*
inline WORD32 ack_switch_fun(struct rte_mempool *pMPool) {
    struct rte_mbuf *pMBuf = rte_pktmbuf_alloc(pMPool);
    WORD32 nTx, idx, len = 120;
    UWORD8 *pChar;
    struct ether_hdr *pEthHdr;
    struct ether_addr sSrcEthAddr;
    rte_eth_macaddr_get((uint8_t)0, &sSrcEthAddr);

    if (pMBuf == NULL) return 0;

    //transmit packet to ack switch 
    pChar = rte_pktmbuf_mtod(pMBuf, UWORD8 *);
    pEthHdr = (struct ether_hdr *)pChar;
    ether_addr_copy(&sSrcEthAddr, &pEthHdr->s_addr);
    ether_addr_copy(&sDstEthAddr, &pEthHdr->d_addr);
    pEthHdr->ether_type = 0xcccc;
    for (idx = 14; idx < len; idx ++) {
        pChar[idx] = idx -  14;
    }
    rte_pktmbuf_pkt_len(pMBuf) = rte_pktmbuf_data_len(pMBuf) = len;
#ifndef FERRY_BRIDGE_INT
    nTx = rte_eth_tx_burst((uint8_t)PORT_ID, 0, &pMBuf, (uint16_t)1);
#else
    nTx = rte_eth_tx_burst((uint8_t)PORT_ID, DATA_TX_Q_ID, &pMBuf, (uint16_t)1);
#endif
    return nTx;
}
*/

/*
inline WORD32 ack_switch(BbuPoolCtrlBlkStruct *pBbu) {
    struct rte_mempool *pMPool = pBbu->sBbuIoIf.bbuio_buf_pool;
    static WORD32 nFlag = 1;
    if (0 == (pBbu->nCurrentSfIdx & 0xff)) {
        if (nFlag == 1) {
            nFlag = 0;
        } else  {
            return 0;
        }
    } else {
        nFlag = 1;
        return 0;
    }
    return ack_switch_fun(pMPool);
}
*/

inline void sleep_for_a_while(BbuPoolCtrlBlkStruct *pBbu) {
    static WORD16 nPrevSf = -1;
    if (nPrevSf == pBbu->nCurrentSfIdx) return;
    if((pBbu->nCurrentSfIdx & 0x1) == 0) {
        nPrevSf = pBbu->nCurrentSfIdx;
        usleep(15); // the actual time depends on OS. if w/RT patch, maybe 10us. w/o RT patch, maybe 70-80 us.
    }

    return;
}

inline void pause_for_a_while(WORD32 nUs) {
    WORD32 nCnt, nIdx;
    WORD32 nCyclePerLoop = 55;
    nCnt = nUs * CPU_HZ;
    for (nIdx = 0; nIdx < nCnt; nIdx += nCyclePerLoop) {
        _mm_pause();
        _mm_pause();
        _mm_pause();
        _mm_pause();
        _mm_pause();
    }
    return;
}

static WORD32 nTxFlag = 0;

inline WORD64 calc_dl_send_headroomUs(WORD16 nCurrentSfIdx, UWORD64 nCurrentSfTime, WORD16 nDlPacketSfIdx, WORD16 nDlPacketSymbol)
{
	return (astri_subframe_compare(nDlPacketSfIdx, nCurrentSfIdx) * TIME_INTERVAL * 1000) + (nDlPacketSymbol * TIME_INTERVAL * 1000 / N_SYMB_PER_SF) - ((rte_rdtsc() - nCurrentSfTime)/CPU_HZ);
}

/* return how many cells have finish DL tx */
inline WORD32 dl_transmission(BbuPoolCtrlBlkStruct *pBbu, WORD32 nType,
    WORD32 *nTxIndicator, WORD32 nTxSfIdx) {
    WORD32 nCellIdx, nTx, nTxBurstLen, nTxCell, idx;
    CellActiveEnum eCellStatus;
    UWORD64 uLatestTsc;
#if (TIME_PROFILE_MODE == 1)
    UWORD64 uDlTsc;
#endif
#ifndef TD_CONNECT
#ifndef FB_API
    struct pdsch_hdr_s *pPdsch;
    struct pdcch_hdr_s *pPdcch;
#endif
#ifdef FB_API
    RpeLtePdschPkt *pPdsch;
    RpeLtePdcchPkt *pPdcch;
#endif
#endif
#ifdef TD_CONNECT
    RpeLteTimeDomainFrameStatus *pPdsch;
#endif
    WORD32 nSfTmp;
    BbuIoBufCtrlStruct *pBbuIoBufCtrl = NULL;
    UWORD8 *pChar;
    UWORD8 bufOffset;
    for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++) {
        if (nTxIndicator[nCellIdx] == 1) {
            continue;
        }

        switch (nType) {
            case 0: // PDSCH
                pBbuIoBufCtrl = &(pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nTxSfIdx % N_FE_BUF_LEN][nCellIdx]);
                break;
            case 1: // PDCCH, including all dl ctrl info and packets
                pBbuIoBufCtrl = &(pBbu->sBbuIoIf.sPdcchBbuIoBufCtrl[nTxSfIdx % N_FE_BUF_LEN][nCellIdx]);
                break;
            default:
                pBbuIoBufCtrl = &(pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nTxSfIdx % N_FE_BUF_LEN][nCellIdx]);
                break;
        }

        eCellStatus = pBbu->eCellActive[nTxSfIdx % TASK_QUEUE_LEN][nCellIdx];
        switch (eCellStatus) {
            case CELL_CLOSE:
            case CELL_ACTIVE:
            case CELL_NEWING_2:
                if (pBbuIoBufCtrl->nSegToBeGen < 0) {
                    /* packet is not generated yet or have been counted */
                    break;
                }
                if (pBbuIoBufCtrl->nSegToBeGen == 0) {
                        nTxIndicator[nCellIdx] = 1;
                        pBbuIoBufCtrl->nSegGenerated = -1;
                        pBbuIoBufCtrl->nSegTransferred = 0;
                }
                if(pBbuIoBufCtrl->nSegGenerated > pBbuIoBufCtrl->nSegTransferred)
                {
                    switch (nType) {
                        case 0:
							nTxBurstLen = 0;
                            for (idx=pBbuIoBufCtrl->nSegTransferred; idx < pBbuIoBufCtrl->nSegGenerated;
                            idx ++) {
                                pChar = rte_pktmbuf_mtod(pBbuIoBufCtrl->pData[idx], UWORD8 *);
#if defined(ASTRI_RRU)
								WORD16 nUdpDataSize = ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1) + FB_API_PAYLOAD_OFFSET;
								pChar+= nUdpDataSize;
#endif //ASTRI_RRU

#ifndef TD_CONNECT
#ifndef FB_API
                                pChar = pChar + ETH_HEADER_SIZE;
                                pPdsch = (struct pdsch_hdr_s *)pChar;
                                nSfTmp = (ntohs(pPdsch->frame_seq) * 10 + pPdsch->subf_seq) % MAX_FRAME_NUM;
#endif
#ifdef FB_API
                                pPdsch = (RpeLtePdschPkt *)pChar;
                                nSfTmp = (ntohs(pPdsch->ltePdschHdr.frame_seq) * 10
                                    + pPdsch->ltePdschHdr.subf_seq) % MAX_FRAME_NUM;
#endif
#endif
#ifdef TD_CONNECT
                                pPdsch = &(((RpeLteTimeDomainPkt *)pChar)->lteTimeDomainFrameStatus);
                                nSfTmp = (((pPdsch->rfNumH) * RPE_RADIO_FRAME_LOW_MAX_NUM + pPdsch->rfNumL)
                                    * 10 + pPdsch->sfNum) % MAX_FRAME_NUM;
#endif

                                if (pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.eSystemBandwidth == B10M) {
                                	//pkt.data_len is 2480, expanded by lte_dl_compression_task_func hack
                                	// printf("=== PDSCH debug: %d\n", pBbuIoBufCtrl->pData[idx]->pkt.data_len);
                                	// override numRB contents to 100 RB
                                	pPdsch->ltePdschHdr.blk_len_h = 25;
                                	pPdsch->ltePdschHdr.blk_len_l = 0;
                            		// oversampling pusch_hdr to exact sampling sample locations
//                            		char *pTmp = (char *)((void *)pPdsch + sizeof(RpeLtePdschPkt));
//                            		char tmpBuf[600*2*2];
//                            		memcpy(&tmpBuf[0],&pTmp[0],600*2*2);
                            		// relocate the input buffer location
//                            		memset(&pTmp[0], 0, 1200*2*2);
//                            		memcpy(&pTmp[300*2], &tmpBuf[0],600*2);
//                            		memcpy(&pTmp[1200*2+300*2], &tmpBuf[600*2],600*2);
                                }

                                if (nSfTmp != nTxSfIdx) {
                                    /*printf("Tx PDSCH idx=%d/%d/%d sf %d at tx sf %d\n", idx,
                                    pBbuIoBufCtrl->nSegTransferred,
                                    pBbuIoBufCtrl->nSegGenerated,
                                    nSfTmp, nTxSfIdx); */
                                }
#ifdef TD_CONNECT
								WORD16 nHeadRoom = calc_dl_send_headroomUs(pBbu->nCurrentSfIdx, uPrevTsc, nSfTmp, pPdsch->symbol);
#else
                               	WORD16 nHeadRoom = calc_dl_send_headroomUs(pBbu->nCurrentSfIdx, uPrevTsc, nSfTmp, pPdsch->ltePdschHdr.sym_seq);
#endif
                                if (nHeadRoom >= (10 * TIME_INTERVAL * 1000 / N_SYMB_PER_SF))
									break;
								else
									nTxBurstLen++;
                            }
                            break;
                        case 1:
                            if (pBbuIoBufCtrl->nSegTransferred == 0)
                            {
                                pChar = rte_pktmbuf_mtod(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred], UWORD8 *);
#ifndef TD_CONNECT
#ifndef FB_API
                                pChar = pChar + ETH_HEADER_SIZE;
                                pPdcch = (struct pdcch_hdr_s *)pChar;
                                nSfTmp = (ntohs(pPdcch->frame_seq) * 10 + pPdcch->subf_seq) % MAX_FRAME_NUM;
#endif
#ifdef FB_API
                                pPdcch = (RpeLtePdcchPkt *)pChar;
                                nSfTmp = (ntohs(pPdcch->ltePdcchHdr.frame_seq) * 10
                                    + pPdcch->ltePdcchHdr.subf_seq) % MAX_FRAME_NUM;
#endif
#endif
                                if (nSfTmp != nTxSfIdx) {
                                    //printf("Tx PDCCH idx=%d sf=%d at tx sf %d\n", idx, nSfTmp, nTxSfIdx);
                                }
                            }
                            nTxBurstLen = pBbuIoBufCtrl->nSegGenerated - pBbuIoBufCtrl->nSegTransferred;
                        default:
                            break;
                    }

                    //nTxBurstLen = pBbuIoBufCtrl->nSegGenerated - pBbuIoBufCtrl->nSegTransferred;


#if defined(L1_L2_DECOUPLE)		
					// forward package to L1D 
					if (g_isL1D_Mode == L1D_MODE_ENABLE_WITHOUT_RRU)
					{
						WORD32 nL1dPkgToBeSend = pBbuIoBufCtrl->nSegGenerated; // latch the value pBbuIoBufCtrl->nSegGenerated, which can keep on increasing at same time at worker thread
						for (uint8_t idx = pBbuIoBufCtrl->nL1dPkgTransferred; idx < nL1dPkgToBeSend; idx++)
						{
							pChar = rte_pktmbuf_mtod(pBbuIoBufCtrl->pData[idx], UWORD8 *);
							unsigned int len = RTE_PKT_LEN(pBbuIoBufCtrl->pData[idx]);
							nTx = ipc_send_msg(nCellIdx, FAPI_SHMA_BBU_PKG_OUT, (char*) pChar, len);
							//fapi_print("[FAPI msg] forward data to L1D, nCellIdx = %d, nType = %d, idx = %d, pChar = %p, len = %d, nTx = %d\n", nCellIdx, nType, idx, pChar, len, nTx);
							if (nTx < 0)
							{
								MESSAGE_LOG("dl_transmission ERROR: forward data to L1D failed, nCellIdx = %d, nType = %d, idx = %d, pChar = %p, len = %d, nTx = %d\n", nCellIdx, nType, idx, pChar, len, nTx);
							}
							else
							{
								pBbuIoBufCtrl->nL1dPkgTransferred++;
							}
						}
					}
#endif // L1_L2_DECOUPLE

#ifndef FERRY_BRIDGE_INT
#if defined(ASTRI_DAUL_PORT_NIC)
					nTx = rte_eth_tx_burst((uint8_t)(PORT_ID + nCellIdx), 0,
							&(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred]),
							(uint16_t)nTxBurstLen);
#else

					nTx = rte_eth_tx_burst((uint8_t)PORT_ID, 0,
							&(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred]),
							(uint16_t)nTxBurstLen);

#if (TIME_PROFILE_MODE == 1)
					uDlTsc = rte_rdtsc();
					for ( int txIdx = 0; txIdx < nTx; txIdx++)
					{
						TimeProfileRecord( 	21,				//PDSCH_PACKET
											0,				//TIME OUT CHECK
											nCellIdx,				//Cell Index
											nTxSfIdx,		//WORKING SFN Index
											0,				//CHECKING SFN Index
											0,				//Duration
											uDlTsc / CPU_HZ,	//Start time
											0,				//Finish time
											(txIdx + pBbuIoBufCtrl->nSegTransferred)); 			//Core ID				
					}

#endif
#endif //ASTRI_DAUL_PORT_NIC
#else
                    nTx = rte_eth_tx_burst((uint8_t)PORT_ID, DATA_TX_Q_ID,
                            &(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred]),
                            (uint16_t)nTxBurstLen);
#endif
                    pBbuIoBufCtrl->nSegTransferred += nTx;

                    if (pBbuIoBufCtrl->nSegToBeGen == pBbuIoBufCtrl->nSegTransferred) {
                        /* if there is only one packet is generated, it is PDCCH */
                        /* those packets that have not been transmitted are freed by update_cell_status */
                        /*printf("%d packets are transmitted by bbuio for sf %d at Tsc(us)= %llu\n",
                                    pBbuIoBufCtrl->nSegTransferred,nTxSfIdx,
                                    rte_rdtsc()/CPU_HZ);
                        */
                        uLatestTsc = rte_rdtsc();
                        iLog_Normal(BBU_POOL_CONTROL, "%d packet are transmitted for cell %d for sf %d at global sf %d Tsc(us)=%llu\n",
                            pBbuIoBufCtrl->nSegTransferred, nCellIdx, nTxSfIdx, pBbu->nCurrentSfIdx, uLatestTsc/CPU_HZ);
                        fapi_print("%d packet are transmitted for cell %d for sf %d at global sf %d Tsc(us)=%llu, nType = %d\n",
                            pBbuIoBufCtrl->nSegTransferred, nCellIdx, nTxSfIdx, pBbu->nCurrentSfIdx, uLatestTsc/CPU_HZ, nType);
/*                        printf("%d packets are tx for cell %d sf %d at global %d\n",
                        pBbuIoBufCtrl->nSegTransferred, nCellIdx, nTxSfIdx, pBbu->nCurrentSfIdx);*/
                        nTxIndicator[nCellIdx] = 1;
                        pBbuIoBufCtrl->nSegGenerated = -1;
                        pBbuIoBufCtrl->nSegToBeGen = -1;
                        pBbuIoBufCtrl->nSegTransferred = 0;
                    }
                }
                break;
            default:
                break;
        }
    }

    /* check if sync cell transmission done */
    if (nTxIndicator[N_MAX_CELL_PER_BBU] == 1) {
        // sync cell DL compression task is not done yet or have been counted
    } else {
        switch (nType) {
            case 0:
                pBbuIoBufCtrl = &(pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nTxSfIdx % N_FE_BUF_LEN][N_MAX_CELL_PER_BBU]);
                break;
            case 1:
                pBbuIoBufCtrl = &(pBbu->sBbuIoIf.sPdcchBbuIoBufCtrl[nTxSfIdx % N_FE_BUF_LEN][N_MAX_CELL_PER_BBU]);
                break;
            default:
                pBbuIoBufCtrl = &(pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nTxSfIdx % N_FE_BUF_LEN][N_MAX_CELL_PER_BBU]);
                break;
        }

        nTxIndicator[N_MAX_CELL_PER_BBU] = 1;
        pBbuIoBufCtrl->nSegGenerated = -1;
        pBbuIoBufCtrl->nSegToBeGen = -1;
        pBbuIoBufCtrl->nSegTransferred = 0;
    }

    nTxCell = 0;
    for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
        nTxCell += nTxIndicator[nCellIdx];
    }

    return nTxCell;
}



#ifdef L1_L2_DECOUPLE
WORD32 processFapiMsg(LteRtInforStruct * pLteRt, WORD32 nOnAirSfIdx, UWORD64 uLatestTsc)
{
	const char* fapiReqMsgName[] =
	{
		"FAPI_SHMA_CH_PARA_REQ",
		"FAPI_SHMA_CH_CFG_REQ",
		"FAPI_SHMA_CH_START_REQ",
		"FAPI_SHMA_CH_STOP_REQ",
		"FAPI_SHMA_CH_DL_CFG_REQ",
		"FAPI_SHMA_CH_UL_CFG_REQ",
		"FAPI_SHMA_CH_HI_DCI0",
		"FAPI_SHMA_CH_TX_REQ",
	};

	WORD32 nCellIdx;
	WORD32 nNextSubrameIndSfIdx = (nOnAirSfIdx + 2) % MAX_FRAME_NUM;
	WORD32 nGenerateSubframeInd = 0;
#if FAPI_ERR_CHECK == 1
	int32_t result;
#endif

#if PROFILE_LV >= 1
    unsigned long long time_start;
    unsigned long long time_end;
    unsigned long long time_interval;
    unsigned long long num_of_cycles_in_1ms;
    double ratio_tti;
#endif
	
	if (((uLatestTsc - uPrevTsc) > ((TIME_INTERVAL * TIME_DELAY_ON_AIR_SUBFRAME_START) * CPU_HZ)) &&  nIsSubframeIndSent == 0)
	{

		nGenerateSubframeInd = 1;		
	}

	// Check for message arriaval
	for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
	{
		WORD32 nNewMsg = 0;

#ifdef IPC_SEMAPHORE
		if (ipc_trywait_mac_ready(nCellIdx) == 0)
		{
			for (WORD32 chIdx = FAPI_SHMA_CH_PARA_REQ; chIdx <= FAPI_SHMA_CH_TX_REQ; chIdx++)
			{
				while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0) // use while loop to flush old data
				{
					if (chIdx == FAPI_SHMA_CH_DL_CFG_REQ)
					{
						FAPI_dlConfigRequest_st* pDlConfigReq = (FAPI_dlConfigRequest_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
						if ( ((pDlConfigReq->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
							 ((pDlConfigReq->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
						{
							//Generate Error Ind and clear channel
							uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
							if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
								GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_DL_CONFIG_REQUEST, pDlConfigReq->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
#if FAPI_ERR_CHECK == 1
						else
						{
#if PROFILE_LV == 1
							time_start = __rdtsc();
#endif
							result = GetFapiAdapterPtr(nCellIdx)->RngCheckDlCfgReq(pDlConfigReq);
#if PROFILE_LV == 1
							time_end = __rdtsc();
							time_interval = time_end - time_start;

							Profile_TimeRptObj_update(&Profile_TimeRptObj_FapiMsg_0, time_interval);

							num_of_cycles_in_1ms = get_ms_cycles();
							ratio_tti = (double)(time_interval / num_of_cycles_in_1ms)*100.0f;

							ProfileLog("processFapiMsg 0: time_interval, curr = %d, cnt = %d, mean = %d, max = %d, min = %d\n", Profile_TimeRptObj_FapiMsg_0.curr, Profile_TimeRptObj_FapiMsg_0.cnt, Profile_TimeRptObj_FapiMsg_0.mean, Profile_TimeRptObj_FapiMsg_0.max, Profile_TimeRptObj_FapiMsg_0.min);
#endif
							if(result == NOT_SUCCESSFUL)
							{
								while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
								break;
							}
						}
#endif
					}
					else if (chIdx == FAPI_SHMA_CH_UL_CFG_REQ)
					{
						FAPI_ulConfigRequest_st* pUlConfigReq = (FAPI_ulConfigRequest_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
						if ( ((pUlConfigReq->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
							 ((pUlConfigReq->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
						{
							//Generate Error Ind and clear channel
							uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
							if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
								GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_UL_CONFIG_REQUEST, pUlConfigReq->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
#if FAPI_ERR_CHECK == 1
						else
						{
#if PROFILE_LV == 1
							time_start = __rdtsc();
#endif
							result = GetFapiAdapterPtr(nCellIdx)->RngCheckUlCfgReq(pUlConfigReq);
#if PROFILE_LV == 1
							time_end = __rdtsc();
							time_interval = time_end - time_start;

							Profile_TimeRptObj_update(&Profile_TimeRptObj_FapiMsg_1, time_interval);

							num_of_cycles_in_1ms = get_ms_cycles();
							ratio_tti = (double)(time_interval / num_of_cycles_in_1ms)*100.0f;

							ProfileLog("processFapiMsg 1: time_interval, curr = %d, cnt = %d, mean = %d, max = %d, min = %d\n", Profile_TimeRptObj_FapiMsg_1.curr, Profile_TimeRptObj_FapiMsg_1.cnt, Profile_TimeRptObj_FapiMsg_1.mean, Profile_TimeRptObj_FapiMsg_1.max, Profile_TimeRptObj_FapiMsg_1.min);
#endif

							if(result == NOT_SUCCESSFUL)
							{
								while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
								break;
							}
						}
#endif
					}
					else if (chIdx == FAPI_SHMA_CH_HI_DCI0)
					{
						FAPI_dlHiDCIPduInfo_st* pHiDci0PudInfo = (FAPI_dlHiDCIPduInfo_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
						if ( ((pHiDci0PudInfo->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
							 ((pHiDci0PudInfo->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
						{
							//Generate Error Ind and clear channel
							uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
							if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
								GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_DL_HI_DCI0_REQUEST, pHiDci0PudInfo->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
#if FAPI_ERR_CHECK == 1
						else
						{
#if PROFILE_LV == 1
							time_start = __rdtsc();
#endif
							result = GetFapiAdapterPtr(nCellIdx)->RngCheckHiDci0Req(pHiDci0PudInfo);
#if PROFILE_LV == 1
							time_end = __rdtsc();
							time_interval = time_end - time_start;

							Profile_TimeRptObj_update(&Profile_TimeRptObj_FapiMsg_2, time_interval);

							num_of_cycles_in_1ms = get_ms_cycles();
							ratio_tti = (double)(time_interval / num_of_cycles_in_1ms)*100.0f;

							ProfileLog("processFapiMsg 2: time_interval, curr = %d, cnt = %d, mean = %d, max = %d, min = %d\n", Profile_TimeRptObj_FapiMsg_2.curr, Profile_TimeRptObj_FapiMsg_2.cnt, Profile_TimeRptObj_FapiMsg_2.mean, Profile_TimeRptObj_FapiMsg_2.max, Profile_TimeRptObj_FapiMsg_2.min);
#endif

							if(result == NOT_SUCCESSFUL)
							{
								while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
								break;
							}
						}
#endif
					}
					else if (chIdx == FAPI_SHMA_CH_TX_REQ)
					{
						FAPI_dlDataTxRequest_st* pTxReq = (FAPI_dlDataTxRequest_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
						if ( ((pTxReq->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
							 ((pTxReq->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
						{
							//Generate Error Ind and clear channel
							uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
							if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
								GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_DL_TX_REQUEST, pTxReq->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
					}
	
					g_pFapiMsg[nCellIdx][chIdx] = (FAPI_l1ApiMsg_st*)(g_pFapiMsgBuf[nCellIdx][chIdx]);
					fapi_print("[FAPI msg] get %s nCellIdx = %d, nSubrameIndSfIdx = %d\n", fapiReqMsgName[chIdx], nCellIdx, nSubrameIndSfIdx);
					nNewMsg = 1;
	
#ifdef	_FAPI_PRINT_
					switch(chIdx)
					{
						case FAPI_SHMA_CH_DL_CFG_REQ:
							//FAPI_PrintDlCfgReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
							break;
						case FAPI_SHMA_CH_TX_REQ:
							//FAPI_PrintTxReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
							break;
						case FAPI_SHMA_CH_UL_CFG_REQ:
							//FAPI_PrintUlCfgReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
							break;
						case FAPI_SHMA_CH_HI_DCI0:
							//FAPI_PrintHiDci0Req((char*)g_pFapiMsg[nCellIdx][chIdx]);
							break;
					}
#endif //_FAPI_PRINT_
				}
			}
			g_nGenDlTasReady[nCellIdx] = 1;
		}
#else // IPC_SEMAPHORE not defined
		for (WORD32 chIdx = FAPI_SHMA_CH_PARA_REQ; chIdx <= FAPI_SHMA_CH_STOP_REQ; chIdx++)
		{
			while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0) // use while loop to flush old data
			{
				g_pFapiMsg[nCellIdx][chIdx] = (FAPI_l1ApiMsg_st*)(g_pFapiMsgBuf[nCellIdx][chIdx]);
				fapi_print("[FAPI msg] get %s nCellIdx = %d, nSubrameIndSfIdx = %d\n", fapiReqMsgName[chIdx], nCellIdx, nSubrameIndSfIdx);
				nNewMsg = 1;

#ifdef	_FAPI_PRINT_				
				switch(chIdx)
				{
					case FAPI_SHMA_CH_DL_CFG_REQ:
						//FAPI_PrintDlCfgReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
					case FAPI_SHMA_CH_TX_REQ:
						//FAPI_PrintTxReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
					case FAPI_SHMA_CH_UL_CFG_REQ:
						//FAPI_PrintUlCfgReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
					case FAPI_SHMA_CH_HI_DCI0:
						//FAPI_PrintHiDci0Req((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
				}
#endif //_FAPI_PRINT_					
			}
		}

		for (WORD32 chIdx = FAPI_SHMA_CH_DL_CFG_REQ; chIdx <= FAPI_SHMA_CH_TX_REQ; chIdx++)
		{
			while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0) // use while loop to flush old data
			{
				FAPI_l1ApiMsg_st* pL1ApiMsg = (FAPI_l1ApiMsg_st*)(g_pFapiMsgBuf[nCellIdx][chIdx]);

				if (chIdx == FAPI_SHMA_CH_DL_CFG_REQ)
				{
					FAPI_dlConfigRequest_st* pDlConfigReq = (FAPI_dlConfigRequest_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
					if ( ((pDlConfigReq->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
						 ((pDlConfigReq->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
					{
						//Generate Error Ind and clear channel
						uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
						if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
							GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_DL_CONFIG_REQUEST, pDlConfigReq->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
						while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
						break;
					}
#if FAPI_ERR_CHECK == 1
					else
					{
#if PROFILE_LV == 1
						time_start = __rdtsc();
#endif
						result = GetFapiAdapterPtr(nCellIdx)->RngCheckDlCfgReq(pDlConfigReq);
#if PROFILE_LV == 1
						time_end = __rdtsc();
						time_interval = time_end - time_start;

						Profile_TimeRptObj_update(&Profile_TimeRptObj_FapiMsg_0, time_interval);

						num_of_cycles_in_1ms = get_ms_cycles();
						ratio_tti = (double)(time_interval / num_of_cycles_in_1ms)*100.0f;

						ProfileLog("processFapiMsg 0: time_interval, curr = %d, cnt = %d, mean = %d, max = %d, min = %d\n", Profile_TimeRptObj_FapiMsg_0.curr, Profile_TimeRptObj_FapiMsg_0.cnt, Profile_TimeRptObj_FapiMsg_0.mean, Profile_TimeRptObj_FapiMsg_0.max, Profile_TimeRptObj_FapiMsg_0.min);
#endif
						if(result == NOT_SUCCESSFUL)
						{
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
					}
#endif
				}
				else if (chIdx == FAPI_SHMA_CH_UL_CFG_REQ)
				{
					FAPI_ulConfigRequest_st* pUlConfigReq = (FAPI_ulConfigRequest_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
					if ( ((pUlConfigReq->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
						 ((pUlConfigReq->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
					{
						//Generate Error Ind and clear channel
						uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
						if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
							GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_UL_CONFIG_REQUEST, pUlConfigReq->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
						while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
						break;
					}
#if FAPI_ERR_CHECK == 1
					else
					{
#if PROFILE_LV == 1
						time_start = __rdtsc();
#endif
						result = GetFapiAdapterPtr(nCellIdx)->RngCheckUlCfgReq(pUlConfigReq);
#if PROFILE_LV == 1
						time_end = __rdtsc();
						time_interval = time_end - time_start;

						Profile_TimeRptObj_update(&Profile_TimeRptObj_FapiMsg_1, time_interval);

						num_of_cycles_in_1ms = get_ms_cycles();
						ratio_tti = (double)(time_interval / num_of_cycles_in_1ms)*100.0f;

						ProfileLog("processFapiMsg 1: time_interval, curr = %d, cnt = %d, mean = %d, max = %d, min = %d\n", Profile_TimeRptObj_FapiMsg_1.curr, Profile_TimeRptObj_FapiMsg_1.cnt, Profile_TimeRptObj_FapiMsg_1.mean, Profile_TimeRptObj_FapiMsg_1.max, Profile_TimeRptObj_FapiMsg_1.min);
#endif

						if(result == NOT_SUCCESSFUL)
						{
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
					}
#endif
				}
				else if (chIdx == FAPI_SHMA_CH_HI_DCI0)
				{
					FAPI_dlHiDCIPduInfo_st* pHiDci0PudInfo = (FAPI_dlHiDCIPduInfo_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
					if ( ((pHiDci0PudInfo->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
						 ((pHiDci0PudInfo->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
					{
						//Generate Error Ind and clear channel
						uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
						if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
							GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_DL_HI_DCI0_REQUEST, pHiDci0PudInfo->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
						while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
						break;
					}
#if FAPI_ERR_CHECK == 1
					else
					{
#if PROFILE_LV == 1
						time_start = __rdtsc();
#endif
						result = GetFapiAdapterPtr(nCellIdx)->RngCheckHiDci0Req(pHiDci0PudInfo);
#if PROFILE_LV == 1
						time_end = __rdtsc();
						time_interval = time_end - time_start;

						Profile_TimeRptObj_update(&Profile_TimeRptObj_FapiMsg_2, time_interval);

						num_of_cycles_in_1ms = get_ms_cycles();
						ratio_tti = (double)(time_interval / num_of_cycles_in_1ms)*100.0f;

						ProfileLog("processFapiMsg 2: time_interval, curr = %d, cnt = %d, mean = %d, max = %d, min = %d\n", Profile_TimeRptObj_FapiMsg_2.curr, Profile_TimeRptObj_FapiMsg_2.cnt, Profile_TimeRptObj_FapiMsg_2.mean, Profile_TimeRptObj_FapiMsg_2.max, Profile_TimeRptObj_FapiMsg_2.min);
#endif

						if(result == NOT_SUCCESSFUL)
						{
							while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
							break;
						}
					}
#endif
				}
				else if (chIdx == FAPI_SHMA_CH_TX_REQ)
				{
					FAPI_dlDataTxRequest_st* pTxReq = (FAPI_dlDataTxRequest_st*)(g_pFapiMsgBuf[nCellIdx][chIdx] + FAPI_l1ApiMsg_st_LEN);
					if ( ((pTxReq->sfnsf >> 4) != (nSubrameIndSfIdx / 10)) ||
						 ((pTxReq->sfnsf & 0x0F) != (nSubrameIndSfIdx % 10)) )
					{
						//Generate Error Ind and clear channel
						uint32_t target_sfnsf =  (uint32_t)(((nSubrameIndSfIdx / 10) << 4) | (nSubrameIndSfIdx % 10));
						if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
							GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_DL_TX_REQUEST, pTxReq->sfnsf, (uint16_t) target_sfnsf, FAPI_SFN_OUT_OF_SYNC, FAPI_MSG_ERROR_NA);
						while (ipc_recv_msg(nCellIdx, (FAPI_SHMA_CH_T)chIdx, g_pFapiMsgBuf[nCellIdx][chIdx], FAPI_MSG_SIZE) > 0);
						break;
					}
				}

				g_pFapiMsg[nCellIdx][chIdx] = (FAPI_l1ApiMsg_st*)(g_pFapiMsgBuf[nCellIdx][chIdx]);
				fapi_print("[FAPI msg] get %s nCellIdx = %d, nSubrameIndSfIdx = %d\n", fapiReqMsgName[chIdx], nCellIdx, nSubrameIndSfIdx);
				nNewMsg = 1;

#ifdef	_FAPI_PRINT_
				switch(chIdx)
				{
					case FAPI_SHMA_CH_DL_CFG_REQ:
						//FAPI_PrintDlCfgReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
					case FAPI_SHMA_CH_TX_REQ:
						//FAPI_PrintTxReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
					case FAPI_SHMA_CH_UL_CFG_REQ:
						//FAPI_PrintUlCfgReq((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
					case FAPI_SHMA_CH_HI_DCI0:
						//FAPI_PrintHiDci0Req((char*)g_pFapiMsg[nCellIdx][chIdx]);
						break;
				}
#endif //_FAPI_PRINT_
			}
		}
#endif //IPC_SEMAPHORE		

		if (nNewMsg == 0 && nGenerateSubframeInd == 0)
			continue;

		UWORD8 subfCfg = g_config_data->nSubframeAssignment;
		UlScheSfStruct UlSheSf;
		eNB_Mac_Cal_subframe(nSubrameIndSfIdx, &UlSheSf, subfCfg);
		//fapi_print("[FAPI msg] UlSheSf: nSFN = %d, nSubFrame = %d, nCurrentSf = %d, nLSchePdcchSf = %d, nLSchePuschSf = %d, nLSchePhichSf = %d\n", UlSheSf.nSFN, UlSheSf.nSubFrame, UlSheSf.nCurrentSf, UlSheSf.nLSchePdcchSf, UlSheSf.nLSchePuschSf, UlSheSf.nLSchePhichSf);
		PhySectorControlBlockStruct *psPhySectorCB = eNB_GetPhySectorCtrlBlk(nCellIdx);

#ifndef IPC_SEMAPHORE	
		// trigger early processing only if both TX.req and DL_config.req are ready
		if ( (nGenerateSubframeInd == 0 && g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ] != NULL && g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_TX_REQ] != NULL))
		{
			PhyEventStruct* pPhylayerEventStructPdcch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current dl frame
			PhyEventStruct* pPhylayerEventStructDlPdsch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current dl frame

			fapi_print("[FAPI msg] processDlCfgReqAndTxReq Pdcch idx %d, DlPdsch idx %d\n", nSubrameIndSfIdx % TASK_QUEUE_LEN, nSubrameIndSfIdx % TASK_QUEUE_LEN);
			if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
				GetFapiAdapterPtr(nCellIdx)->processDlCfgReqAndTxReq(pPhylayerEventStructPdcch, pPhylayerEventStructDlPdsch, nSubrameIndSfIdx, (char*) g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ], (char*) g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_TX_REQ]);
			pPhylayerEventStructPdcch->nPDCCHSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructDlPdsch->nDLAssignmentSfIdx = nSubrameIndSfIdx;

			g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ] = NULL;
			g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_TX_REQ] = NULL;
			g_nGenDlTasReady[nCellIdx] = 1; //Fast gen dl tasks
#ifdef	_FAPI_PRINT_
/*
			FAPI_PrintPcfichInfo(pPhylayerEventStructPdcch);
			FAPI_PrintDlPdcchInfo(pPhylayerEventStructPdcch);
			FAPI_PrintDlPdschInfo(pPhylayerEventStructDlPdsch);
*/
#endif //_FAPI_PRINT_			
		}
#endif // not IPC_SEMAPHORE	


#ifdef IPC_SEMAPHORE
		if ( g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ] != NULL)
#else
		// Tx.req may not present, trigger processing anyway if deadline meets
		if ( (nGenerateSubframeInd == 1 && g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ] != NULL ) )
#endif
		{
			PhyEventStruct* pPhylayerEventStructPdcch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current dl frame
			PhyEventStruct* pPhylayerEventStructDlPdsch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current dl frame

			fapi_print("[FAPI msg] processDlCfgReqAndTxReq Pdcch idx %d, DlPdsch idx %d\n", nSubrameIndSfIdx % TASK_QUEUE_LEN, nSubrameIndSfIdx % TASK_QUEUE_LEN);
			if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
				GetFapiAdapterPtr(nCellIdx)->processDlCfgReqAndTxReq(pPhylayerEventStructPdcch, pPhylayerEventStructDlPdsch, nSubrameIndSfIdx, (char*) g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ], (char*) g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_TX_REQ]);
			pPhylayerEventStructPdcch->nPDCCHSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructDlPdsch->nDLAssignmentSfIdx = nSubrameIndSfIdx;

			g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_DL_CFG_REQ] = NULL;
			g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_TX_REQ] = NULL;

#ifdef	_FAPI_PRINT_
/*
			FAPI_PrintPcfichInfo(pPhylayerEventStructPdcch);
			FAPI_PrintDlPdcchInfo(pPhylayerEventStructPdcch);
			FAPI_PrintDlPdschInfo(pPhylayerEventStructDlPdsch);
*/
#endif //_FAPI_PRINT_
		}

		if (g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_HI_DCI0] != NULL)
		{
			if (UlSheSf.nLSchePuschSf < 0 || UlSheSf.nLSchePuschSf >= TASK_QUEUE_LEN)
			{
				if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
				{
					MESSAGE_LOG("processFapiMsg ERROR: nSFN = %d, nSubFrame = %d, nCurrentSf = %d, nLSchePdcchSf = %d, nLSchePuschSf = %d, nLSchePhichSf = %d\n", UlSheSf.nSFN, UlSheSf.nSubFrame, UlSheSf.nCurrentSf, UlSheSf.nLSchePdcchSf, UlSheSf.nLSchePuschSf, UlSheSf.nLSchePhichSf);
					g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_HI_DCI0] = NULL;
					return -1;
				}
				else
				{
					UlSheSf.nLSchePuschSf = nSubrameIndSfIdx;
				}
			}

			PhyEventStruct* pPhylayerEventStructDlPhich = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current dl frame
			PhyEventStruct* pPhylayerEventStructPdcch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current dl frame
			PhyEventStruct* pPhylayerEventStructUlPusch = &psPhySectorCB->sLowLayerEventQueue[UlSheSf.nLSchePuschSf]; // next UL frame after dci0 send out

			fapi_print("[FAPI msg] processHiDci0Req DlPhich idx %d, Pdcch idx %d, UlPusch idx %d\n", nSubrameIndSfIdx % TASK_QUEUE_LEN, nSubrameIndSfIdx % TASK_QUEUE_LEN, UlSheSf.nLSchePuschSf);
			if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
				GetFapiAdapterPtr(nCellIdx)->processHiDci0Req(pPhylayerEventStructDlPhich, pPhylayerEventStructPdcch, pPhylayerEventStructUlPusch, nSubrameIndSfIdx, (char*) g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_HI_DCI0]);
			pPhylayerEventStructDlPhich->nDLPHICHAllocateSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructPdcch->nPDCCHSfIdx = nSubrameIndSfIdx;
			// pPhylayerEventStructUlPusch->nULGrantSfIdx will be updated after ul_config.req is read

			g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_HI_DCI0] = NULL;
			
#ifdef	_FAPI_PRINT_	
/*
			FAPI_PrintPhichInfo(pPhylayerEventStructDlPhich);
			FAPI_PrintUlPdcchInfo(pPhylayerEventStructPdcch);
			FAPI_PrintUlPuschInfo(pPhylayerEventStructUlPusch);
*/			
#endif //_FAPI_PRINT_				
		}

		if (g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_UL_CFG_REQ] != NULL)
		{
			PhyEventStruct* pPhylayerEventStructPdcch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current ul frame;
			PhyEventStruct* pPhylayerEventStructUlPusch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current ul frame;;
			PhyEventStruct* pPhylayerEventStructPucch = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current ul frame;;
			PhyEventStruct* pPhylayerEventStructSrs = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN]; // current ul frame;;

			fapi_print("[FAPI msg] processUlCfgReq Pdcch idx %d, UlPusch idx %d, Pucch idx %d, Srs idx %d\n", nSubrameIndSfIdx % TASK_QUEUE_LEN, nSubrameIndSfIdx % TASK_QUEUE_LEN, nSubrameIndSfIdx % TASK_QUEUE_LEN, nSubrameIndSfIdx % TASK_QUEUE_LEN);
		
			if (g_IsBbuOutOfSync == 0 && g_IsBbuUnderReset == 0)
				GetFapiAdapterPtr(nCellIdx)->processUlCfgReq(pPhylayerEventStructPdcch, pPhylayerEventStructUlPusch, pPhylayerEventStructPucch, pPhylayerEventStructSrs, nSubrameIndSfIdx, (char*) g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_UL_CFG_REQ]);
			pPhylayerEventStructUlPusch->nULGrantSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructPucch->nPucchAllocateSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructSrs->nSRSAllocateSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructPucch->nRACHSfIdx = nSubrameIndSfIdx;

			g_pFapiMsg[nCellIdx][FAPI_SHMA_CH_UL_CFG_REQ] = NULL;
			
#ifdef	_FAPI_PRINT_
//			FAPI_PrintUlPdcchInfo(pPhylayerEventStructPdcch);
//			FAPI_PrintUlPuschInfo(pPhylayerEventStructUlPusch);
//			FAPI_PrintUlPucchInfo(pPhylayerEventStructPucch);
#ifdef PRACH_PHY_INTEGRATION
//			FAPI_PrintUlPrachInfo(pPhylayerEventStructPucch);
#endif
//			FAPI_PrintUlSrsInfo(pPhylayerEventStructSrs);
#endif //_FAPI_PRINT_			
		}


#if (TIME_OUT_CHECK_MODE == 0 || TIME_OUT_CHECK_MODE == 1)
		if (nGenerateSubframeInd == 1)
#elif (TIME_OUT_CHECK_MODE == 2)
		if (((nGenerateSubframeInd == 1) || (g_nGenDlTasReady[nCellIdx] == 1)) && (g_nGenDlTaskDone[nCellIdx] == 0))
#endif
		{
			PhyEventStruct* pPhylayerEventStructCurrent = &psPhySectorCB->sLowLayerEventQueue[nSubrameIndSfIdx % TASK_QUEUE_LEN];
			GetFapiAdapterPtr(nCellIdx)->validatePhyEventStruct(pPhylayerEventStructCurrent, nSubrameIndSfIdx);
			pPhylayerEventStructCurrent->nDLAssignmentSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructCurrent->nULGrantSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructCurrent->nPDCCHSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructCurrent->nDLPHICHAllocateSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructCurrent->nPucchAllocateSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructCurrent->nSRSAllocateSfIdx = nSubrameIndSfIdx;
			pPhylayerEventStructCurrent->nRACHSfIdx = nSubrameIndSfIdx;
		}


		if (nGenerateSubframeInd == 1)
		{
			//Update number
			pLteRt->pBbuPoolCtrlBlk->nCurrentPsSfIdx = nNextSubrameIndSfIdx;
		
			WORD32 nUlDataIndSfIdx = (MAX_FRAME_NUM + (nNextSubrameIndSfIdx - 4)) % MAX_FRAME_NUM;
			WORD32 nPrachIndSfIdx = (MAX_FRAME_NUM + (nNextSubrameIndSfIdx - 6)) % MAX_FRAME_NUM;
			WORD32 nSrsIndSfIdx = (MAX_FRAME_NUM + (nNextSubrameIndSfIdx - 6)) % MAX_FRAME_NUM;
			//fapi_print("[FAPI msg] send UL data indication messages for cell nCellIdx = %d, tti = %d, nOnAirSfIdx = %d\n", nCellIdx, nUlDataIndSfIdx, nOnAirSfIdx);
			
			PhySectorControlBlockStruct *psPhySectorCB = eNB_GetPhySectorCtrlBlk(nCellIdx);
			PhyEventStruct* pPhylayerEventUlData = &psPhySectorCB->sLowLayerEventQueue[nUlDataIndSfIdx % TASK_QUEUE_LEN];
			PhyEventStruct* pPhylayerEventRach = &psPhySectorCB->sLowLayerEventQueue[nPrachIndSfIdx % TASK_QUEUE_LEN];
			PhyEventStruct* pPhylayerEventSrs = &psPhySectorCB->sLowLayerEventQueue[nSrsIndSfIdx % TASK_QUEUE_LEN];

#ifdef	_FAPI_PRINT_
//			FAPI_PrintPuschInfo(pPhylayerEventUlData);
//			FAPI_PrintPucchInfo(pPhylayerEventUlData);
#endif //_FAPI_PRINT_		

			if (g_IsBbuOutOfSync == 0 && (g_BbuTimingState == BBU_TIMING_STATE_LOCK || g_BbuTimingState == BBU_TIMING_STATE_SYNC))
			{

				if(pPhylayerEventUlData->nChannelTimeout == 1)
				{
					GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_UL_SUBFRAME_INDICATION,
																	 pPhylayerEventUlData->nTaskSfnIndex,
																	 pPhylayerEventUlData->nCheckSfnIndex,
																	 FAPI_MSG_OUT_OF_TIME_ERR,
																	 FAPI_MSG_ERROR_TX);
					pPhylayerEventUlData->nChannelTimeout = 0;
				}
				else if(pPhylayerEventUlData->nChannelTimeout == 2)
				{
					GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_UL_SUBFRAME_INDICATION,
																	 pPhylayerEventUlData->nTaskSfnIndex,
																	 pPhylayerEventUlData->nCheckSfnIndex,
																	 FAPI_MSG_OUT_OF_TIME_ERR,
																	 FAPI_MSG_ERROR_RX);
					pPhylayerEventUlData->nChannelTimeout = 0;
				}
				else if(pPhylayerEventRach->nChannelTimeout == 3)
				{
					GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_UL_SUBFRAME_INDICATION,
																	 pPhylayerEventUlData->nTaskSfnIndex,
																	 pPhylayerEventUlData->nCheckSfnIndex,
																	 FAPI_MSG_OUT_OF_TIME_ERR,
																	 FAPI_MSG_ERROR_RACH);
					pPhylayerEventRach->nChannelTimeout = 0;
				}
				else if(pPhylayerEventSrs->nChannelTimeout == 4)
				{
					GetFapiAdapterPtr(nCellIdx)->sendErrorIndication(PHY_UL_SUBFRAME_INDICATION,
																	 pPhylayerEventUlData->nTaskSfnIndex,
																	 pPhylayerEventUlData->nCheckSfnIndex,
																	 FAPI_MSG_OUT_OF_TIME_ERR,
																	 FAPI_MSG_ERROR_SRS);
					pPhylayerEventSrs->nChannelTimeout = 0;
				}

				if (pPhylayerEventUlData->nPucchDone == 1)
				{
					if (pPhylayerEventUlData->nPucchIndSfIdx == nUlDataIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendSrIndication(pPhylayerEventUlData, nUlDataIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nPucchIndSfIdx = %d, nUlDataIndSfIdx = %d, SR indication ignored.\n", pPhylayerEventUlData->nPucchIndSfIdx, nUlDataIndSfIdx);
				}

				if (pPhylayerEventUlData->nPucchDone == 1 && pPhylayerEventUlData->nPuschDone == 1)
				{
					if (pPhylayerEventUlData->nPucchIndSfIdx == nUlDataIndSfIdx && pPhylayerEventUlData->nPuschIndSfIdx == nUlDataIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendCqiIndication(pPhylayerEventUlData, nUlDataIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nPucchIndSfIdx = %d, nPuschIndSfIdx = %d, nUlDataIndSfIdx = %d, CQI indication ignored.\n", pPhylayerEventUlData->nPucchIndSfIdx, pPhylayerEventUlData->nPuschIndSfIdx, nUlDataIndSfIdx);
				}

				if (pPhylayerEventUlData->nPucchDone == 1 && pPhylayerEventUlData->nPuschDone == 1)
				{
					if (pPhylayerEventUlData->nPucchIndSfIdx == nUlDataIndSfIdx && pPhylayerEventUlData->nPuschIndSfIdx == nUlDataIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendHarqIndication(pPhylayerEventUlData, nUlDataIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nPucchIndSfIdx = %d, nPuschIndSfIdx = %d, nUlDataIndSfIdx = %d, HARQ indication ignored.\n", pPhylayerEventUlData->nPucchIndSfIdx, pPhylayerEventUlData->nPuschIndSfIdx, nUlDataIndSfIdx);
				}

				if (pPhylayerEventUlData->nPuschDone == 1)
				{
					if (pPhylayerEventUlData->nPuschIndSfIdx == nUlDataIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendUlschIndication(pPhylayerEventUlData, nUlDataIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nPuschIndSfIdx = %d, nUlDataIndSfIdx = %d, ULSCH indication ignored.\n", pPhylayerEventUlData->nPuschIndSfIdx, nUlDataIndSfIdx);
				}

				if (pPhylayerEventSrs->nSrsDone == 1)
				{
					if (pPhylayerEventSrs->nSrsIndSfIdx == nSrsIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendSrsIndication(pPhylayerEventSrs, nSrsIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nSrsIndSfIdx = %d, nSrsIndSfIdx = %d, SRS indication ignored.\n", pPhylayerEventSrs->nSrsIndSfIdx, nSrsIndSfIdx);
				}

				if (pPhylayerEventRach->nRachDone == 1)
				{
					if (pPhylayerEventRach->nRachIndSfIdx == nPrachIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendRachIndication(pPhylayerEventRach, nPrachIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nRachIndSfIdx = %d, nPrachIndSfIdx = %d, RACH indication ignored.\n", pPhylayerEventRach->nRachIndSfIdx, nPrachIndSfIdx);
				}

				if (pPhylayerEventUlData->nPuschDone == 1)
				{
					if (pPhylayerEventUlData->nPuschIndSfIdx == nUlDataIndSfIdx)
						GetFapiAdapterPtr(nCellIdx)->sendCrcIndication(pPhylayerEventUlData, nUlDataIndSfIdx);
					else
						MESSAGE_LOG("processFapiMsg WARNING: SfIdx mismatch detected, nPuschIndSfIdx = %d, nUlDataIndSfIdx = %d, CRC indication ignored.\n", pPhylayerEventUlData->nPuschIndSfIdx, nUlDataIndSfIdx);
				}

				GetFapiAdapterPtr(nCellIdx)->sendSubframeIndication(nNextSubrameIndSfIdx);
				
#ifdef IPC_SEMAPHORE
				ipc_notify_mac_ready(nCellIdx);
#endif
			}

			// reset data for next tti
			memset(g_pFapiMsg[nCellIdx], NULL, IPC_QUEUE_NUMBER * sizeof(FAPI_l1ApiMsg_st*));

			//----------- reset data before write ---------------
			{
				PhyEventStruct* pPhylayerEventNextTTI = &psPhySectorCB->sLowLayerEventQueue[nNextSubrameIndSfIdx % TASK_QUEUE_LEN];


				pPhylayerEventNextTTI->nDLAssignmentSfIdx = INVALID_SFIDX;
				pPhylayerEventNextTTI->nULGrantSfIdx = INVALID_SFIDX;
				pPhylayerEventNextTTI->nPDCCHSfIdx = INVALID_SFIDX;
				pPhylayerEventNextTTI->nDLPHICHAllocateSfIdx = INVALID_SFIDX;
				pPhylayerEventNextTTI->nPucchAllocateSfIdx = INVALID_SFIDX;
				pPhylayerEventNextTTI->nSRSAllocateSfIdx = INVALID_SFIDX;
				pPhylayerEventNextTTI->nRACHSfIdx = INVALID_SFIDX;
				
				pPhylayerEventNextTTI->nCFI = 0;
				
				pPhylayerEventNextTTI->nDLAssignmentLen = 0;
				memset(pPhylayerEventNextTTI->sDLAssignmentState, 0, sizeof(pPhylayerEventNextTTI->sDLAssignmentState));

#ifdef TDD 
				if (get_sf_type(subfCfg, nNextSubrameIndSfIdx) == DL_SUBFRAME)
				{
					pPhylayerEventNextTTI->nULGrantLen = 0;
					memset(pPhylayerEventNextTTI->sULGrantState, 0, sizeof(pPhylayerEventNextTTI->sULGrantState));
				}
#endif				

				pPhylayerEventNextTTI->nPDCCHLen = 0;
				memset(pPhylayerEventNextTTI->sPDCCHAllocate, 0, sizeof(pPhylayerEventNextTTI->sPDCCHAllocate));

				pPhylayerEventNextTTI->nDLPHICHAllocateNum = 0;
				memset(pPhylayerEventNextTTI->sDLPHICHAllocate, 0, sizeof(pPhylayerEventNextTTI->sDLPHICHAllocate));

				pPhylayerEventNextTTI->nPucchAllocateNum = 0;
				memset(pPhylayerEventNextTTI->sPucchAllocate, 0, sizeof(pPhylayerEventNextTTI->sPucchAllocate));

				pPhylayerEventNextTTI->nSRSAllocateLen = 0;
				memset(pPhylayerEventNextTTI->sSrsAllocate, 0, sizeof(pPhylayerEventNextTTI->sSrsAllocate));

				pPhylayerEventNextTTI->bRACHEnable = 0;

				pPhylayerEventNextTTI->nPuschDone = 0;
				pPhylayerEventNextTTI->nPucchDone = 0;
				pPhylayerEventNextTTI->nRachDone = 0;
				pPhylayerEventNextTTI->nSrsDone = 0;

				pPhylayerEventNextTTI->nChannelTimeout = 0;
				pPhylayerEventNextTTI->nCheckSfnIndex = 0;
				pPhylayerEventNextTTI->nTaskSfnIndex = 0;

				pPhylayerEventNextTTI->nPuschIndicationNum = 0;
				memset(pPhylayerEventNextTTI->sPuschIndResult, 0, sizeof(pPhylayerEventNextTTI->sPuschIndResult));

				pPhylayerEventNextTTI->nPucchIndNum = 0;
				memset(pPhylayerEventNextTTI->sPucchUciIndState, 0, sizeof(pPhylayerEventNextTTI->sPucchUciIndState));

				memset(&pPhylayerEventNextTTI->sPreambleDetectionResult, 0, sizeof(pPhylayerEventNextTTI->sPreambleDetectionResult));

				pPhylayerEventNextTTI->nSRSResultNum = 0;
				memset(pPhylayerEventNextTTI->sSRSResultState, 0, sizeof(pPhylayerEventNextTTI->sSRSResultState));

				UlScheSfStruct NextUlSheSf;
				eNB_Mac_Cal_subframe(nNextSubrameIndSfIdx, &NextUlSheSf, subfCfg);
				if (NextUlSheSf.nLSchePuschSf >= 0 && NextUlSheSf.nLSchePuschSf < TASK_QUEUE_LEN)
				{
					//printf("[0413] [1] BBUIO reset PUSCH config: subfCfg = %d, nNextSubrameIndSfIdx = %d, NextUlSheSf.nLSchePuschSf = %d\n", subfCfg, nNextSubrameIndSfIdx, NextUlSheSf.nLSchePuschSf);

					PhyEventStruct* pPhylayerEventNextUlGrant = &psPhySectorCB->sLowLayerEventQueue[NextUlSheSf.nLSchePuschSf % TASK_QUEUE_LEN];
					pPhylayerEventNextUlGrant->nULGrantLen = 0;
					memset(pPhylayerEventNextUlGrant->sULGrantState, 0, sizeof(pPhylayerEventNextUlGrant->sULGrantState));
				}
			}
		}
	}


	// generate tasks for active cells
	
	for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
	{
#if (TIME_OUT_CHECK_MODE == 0 || TIME_OUT_CHECK_MODE == 1)
		if (nGenerateSubframeInd == 1)
#elif (TIME_OUT_CHECK_MODE == 2)
		if (((nGenerateSubframeInd == 1) || (g_nGenDlTasReady[nCellIdx] == 1)) && (g_nGenDlTaskDone[nCellIdx] == 0))
#endif
		{
			//fapi_print("[fapi task gen] subframe ind. generated. processFapiMsgcall gen_dl_ctrl_task and gen_dl_data_task for cell nCellIdx = %d, nTtiIdx = %d\n", nCellIdx, nSubrameIndSfIdx);
			//pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sPdcchBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
			gen_dl_ctrl_task(pLteRt->pBbuPoolCtrlBlk, nSubrameIndSfIdx, nCellIdx);
			//pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sDlBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
			gen_dl_data_task(pLteRt->pBbuPoolCtrlBlk, nSubrameIndSfIdx, nCellIdx);

			g_nGenDlTasReady[nCellIdx] = 0;
			g_nGenDlTaskDone[nCellIdx] = 1;
		}
	}

	WORD32 nGenDummyCellTask = 1;	
	for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
	{
		if (g_nGenDlTaskDone[nCellIdx] == 0)
		{
			nGenDummyCellTask = 0;
			break;
		}
	}

	// generate tasks for dummy cell
	if (nGenDummyCellTask == 1)
	{
		nCellIdx = N_MAX_CELL_PER_BBU;
		//fapi_print("[fapi task gen] subframe ind. generated. processFapiMsgcall gen_dl_ctrl_task and gen_dl_data_task for cell nCellIdx = %d, nTtiIdx = %d\n", nCellIdx, nSubrameIndSfIdx);
		//pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sPdcchBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
		gen_dl_ctrl_task(pLteRt->pBbuPoolCtrlBlk, nSubrameIndSfIdx, nCellIdx);
		//pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sDlBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
		gen_dl_data_task(pLteRt->pBbuPoolCtrlBlk, nSubrameIndSfIdx, nCellIdx);
	}

	if (nGenerateSubframeInd == 1)
	{
		//Update subframe done for traffic
		for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
		{
			pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sPdcchBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
			pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sDlBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
		}

		nCellIdx = N_MAX_CELL_PER_BBU;
		{
			pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sPdcchBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
			pLteRt->pBbuPoolCtrlBlk->sBbuIoIf.sDlBbuIoBufCtrl[nSubrameIndSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
		}


		nSubrameIndSfIdx = nNextSubrameIndSfIdx;
		nIsSubframeIndSent = 1;
		memset(g_nGenDlTasReady, NULL, sizeof(g_nGenDlTasReady));
		memset(g_nGenDlTaskDone, NULL, sizeof(g_nGenDlTaskDone));
	}
	
	return 0;
}

#endif //L1_L2_DECOUPLE

#if defined(ASTRI_RRU)
MacAddrGroupStruct g_sUeMacAddrGroup;

WORD32 checkForRruPacket(rte_mbuf *rx_pkt, WORD32 *pCellIdxDetected, WORD32* pIsLoopBackPacket)
{
	WORD16 nUdpOffset = 0;
	struct ether_hdr *pEthHdr;
	struct iphdr1 *pIpv4Hdr;
	struct udphdr1 *pUdpHdr;
	uint8_t * pUdpPayload;
	WORD32 nCellIdx;

	*pCellIdxDetected = -1;
	*pIsLoopBackPacket = 0;	

	pEthHdr = (struct ether_hdr*) RTE_PKTMBUF_READ(rx_pkt, 0, 0, 0);
	pIpv4Hdr = (struct iphdr1 *) RTE_PKTMBUF_READ(rx_pkt, ETHER_HDR_LEN, 0, 0);
	pUdpHdr = (struct udphdr1 *) RTE_PKTMBUF_READ(rx_pkt, ETHER_HDR_LEN + sizeof(struct iphdr1), 0, 0);
	pUdpPayload = (uint8_t *) RTE_PKTMBUF_READ(rx_pkt, ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1), 0, 0);

	if (pIpv4Hdr->version_ihl == 0x45 && pIpv4Hdr->next_proto_id == IPPROTO_UDP)
	{
		//fapi_print("[bbuio_thread]: UDP packet detected! src_port = %d, dst_port = %d, dgram_len=  %d\n", ntohs(pUdpHdr->src_port), ntohs(pUdpHdr->dst_port), ntohs(pUdpHdr->dgram_len));
		if (ntohs(pUdpHdr->src_port) == CLIENT_PORT && ntohs(pUdpHdr->dst_port) == HOST_PORT)
		{
			for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
			{
				if (pEthHdr->s_addr.addr_bytes[0] == g_sUeMacAddrGroup.cMacAddr[nCellIdx][0] &&
					pEthHdr->s_addr.addr_bytes[1] == g_sUeMacAddrGroup.cMacAddr[nCellIdx][1] &&
					pEthHdr->s_addr.addr_bytes[2] == g_sUeMacAddrGroup.cMacAddr[nCellIdx][2] &&
					pEthHdr->s_addr.addr_bytes[3] == g_sUeMacAddrGroup.cMacAddr[nCellIdx][3] &&
					pEthHdr->s_addr.addr_bytes[4] == g_sUeMacAddrGroup.cMacAddr[nCellIdx][4] &&
					pEthHdr->s_addr.addr_bytes[5] == g_sUeMacAddrGroup.cMacAddr[nCellIdx][5])
				{
					*pCellIdxDetected = nCellIdx;
					break;
				}
			}
			if (*pCellIdxDetected < 0 || *pCellIdxDetected >= g_config_data->sectorNum)
			{
				fapi_print("checkForRruPacket: Packet from unknown address %02X:%02X:%02X:%02X:%02X:%02X detected! \n", pEthHdr->s_addr.addr_bytes[0], pEthHdr->s_addr.addr_bytes[1], pEthHdr->s_addr.addr_bytes[2], pEthHdr->s_addr.addr_bytes[3], pEthHdr->s_addr.addr_bytes[4], pEthHdr->s_addr.addr_bytes[5]);
				*pCellIdxDetected = -1;
				return BBU_POOL_ERROR;
			}
			nUdpOffset = ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1);

			// check for loopback packet
			uint8_t * pUdpPayload_rx = (uint8_t *) RTE_PKTMBUF_READ(rx_pkt, nUdpOffset, 0, 0);
			if (pUdpPayload_rx[0] == 'A' &&
				pUdpPayload_rx[1] == 'S' &&
				pUdpPayload_rx[2] == 'T' &&
				pUdpPayload_rx[3] == 'R' &&
				pUdpPayload_rx[4] == 'I' &&
				(pUdpPayload_rx[5] & 0x0F) == 0x0F)
			{
				*pIsLoopBackPacket = 1;
				// remove RRU header
				rte_pktmbuf_adj(rx_pkt, nUdpOffset);

				return BBU_POOL_CORRECT;
			}

			uint16_t* pPktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkt, sizeof(struct ether_hdr) + sizeof(struct vlan_hdr) + nUdpOffset, 0, 0);
			WORD8 *pSrc_UE, *pEnv_UE;
			switch (*pPktType)
			{
				case RPE_TIMING_PKT_SUB_TYPE_SWREC:
					//fapi_print("[RRU Test]: RRU packet detected! pPktType = 0x%04X [Timing Packet]\n", *pPktType);
					break;
				case RPE_PRACH_PKT_SUB_TYPE_SWREC:
					//fapi_print("[RRU Test]: RRU packet detected! pPktType = 0x%04X [PRACH Packet]\n", *pPktType);
					pSrc_UE = (WORD8 *) RTE_PKTMBUF_READ(rx_pkt, 10 + nUdpOffset, 0, 0);
					pEnv_UE = (WORD8 *) RTE_PKTMBUF_READ(rx_pkt, 11 + nUdpOffset, 0, 0);
					*pSrc_UE = nCellIdx;
					*pEnv_UE = 2;
					break;
				case RPE_PUSCH_PKT_SUB_TYPE_SWREC:
					//fapi_print("[RRU Test]: RRU packet detected! pPktType = 0x%04X [PUSCH Packet]\n", *pPktType);
					pSrc_UE = (WORD8 *) RTE_PKTMBUF_READ(rx_pkt,  10 + nUdpOffset, 0, 0);
					pEnv_UE = (WORD8 *) RTE_PKTMBUF_READ(rx_pkt,  11 + nUdpOffset, 0, 0);
					*pSrc_UE = nCellIdx;
					*pEnv_UE = 2;
					break;
				default:
					//fapi_print("[RRU Test]: RRU packet detected! pPktType = 0x%04X [Unknown Type]\n", *pPktType);
					break;
			}

			// remove RRU header
			rte_pktmbuf_adj(rx_pkt, nUdpOffset);

			return BBU_POOL_CORRECT;
		}
	}

	return BBU_POOL_ERROR;
}

extern unsigned short ip_fast_csum(uint8_t *iph, unsigned int ihl);

void send_rru_init_pkt(struct rte_mempool *pMPool, WORD32 nCellIdx, WORD32	isDispRRUInfo, WORD32 isLoopBackMode, WORD32 extraPadding)
{
	rte_mbuf *tx_pkt = NULL;
	UWORD8 *pChar;
	WORD16 nUdpDataSize = ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1) + FB_API_PAYLOAD_OFFSET + extraPadding;
	struct ether_hdr *pEthHdr;
	struct iphdr1 *pIpv4Hdr;
	struct udphdr1 *pUdpHdr;
	uint8_t * pUdpPayload;
	static UWORD16 pkgId = 0;
	uint8_t src_ip[4] = {10,88,120,122};
	uint8_t dst_ip[4] = {10,88,120,133};
	dst_ip[3] += nCellIdx; // cell 0: 10.88.120.133; cell 1: 10.88.120.134; cell 2: 10.88.120.135 ....
	uint16_t src_port = HOST_PORT;
	uint16_t dst_port = CLIENT_PORT;
	struct ether_addr sSrcEthAddr, sDstEthAddr;
	WORD16 nPktSize = 0;
	UWORD8 *pCharData;
	WORD32 nTx;
	UWORD8 nidx = 0;

	sSrcEthAddr.addr_bytes[0] = bbu_port_macs[PORT_ID].addr_bytes[0];
	sSrcEthAddr.addr_bytes[1] = bbu_port_macs[PORT_ID].addr_bytes[1];
	sSrcEthAddr.addr_bytes[2] = bbu_port_macs[PORT_ID].addr_bytes[2];
	sSrcEthAddr.addr_bytes[3] = bbu_port_macs[PORT_ID].addr_bytes[3];
	sSrcEthAddr.addr_bytes[4] = bbu_port_macs[PORT_ID].addr_bytes[4];
	sSrcEthAddr.addr_bytes[5] = bbu_port_macs[PORT_ID].addr_bytes[5];



    sDstEthAddr.addr_bytes[0] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][0];
    sDstEthAddr.addr_bytes[1] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][1];
    sDstEthAddr.addr_bytes[2] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][2];
    sDstEthAddr.addr_bytes[3] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][3];
    sDstEthAddr.addr_bytes[4] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][4];
    sDstEthAddr.addr_bytes[5] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][5];

#if ASTRI_PHY == 1
    src_ip[0] = g_sUeMacAddrGroup.cBbuIpAddr[0][0];
    src_ip[1] = g_sUeMacAddrGroup.cBbuIpAddr[0][1];
    src_ip[2] = g_sUeMacAddrGroup.cBbuIpAddr[0][2];
    src_ip[3] = g_sUeMacAddrGroup.cBbuIpAddr[0][3];

    dst_ip[0] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][0];
    dst_ip[1] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][1];
    dst_ip[2] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][2];
    dst_ip[3] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][3];
#endif

	if (isDispRRUInfo == 1)
	{
		printf("[RRU %d]\n", nCellIdx);
		printf("src mac = %02X:%02X:%02X:%02X:%02X:%02X\n", sSrcEthAddr.addr_bytes[0], sSrcEthAddr.addr_bytes[1], sSrcEthAddr.addr_bytes[2], sSrcEthAddr.addr_bytes[3], sSrcEthAddr.addr_bytes[4], sSrcEthAddr.addr_bytes[5]);
		printf("dst mac = %02X:%02X:%02X:%02X:%02X:%02X\n", sDstEthAddr.addr_bytes[0], sDstEthAddr.addr_bytes[1], sDstEthAddr.addr_bytes[2], sDstEthAddr.addr_bytes[3], sDstEthAddr.addr_bytes[4], sDstEthAddr.addr_bytes[5]);
		printf("src ip = %d.%d.%d.%d, port = %d\n", src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port);
		printf("dst_ip ip = %d.%d.%d.%d, port = %d\n", dst_ip[0], dst_ip[1], dst_ip[2], dst_ip[3], dst_port);
	}

	nPktSize = nUdpDataSize;

	tx_pkt = rte_pktmbuf_alloc(pMPool);
	if (tx_pkt == NULL)
	{
		printf("send_rru_init_pkt ERROR: rte_pktmbuf_alloc error\n");
	}

	pChar = rte_pktmbuf_mtod(tx_pkt, UWORD8 *);

	// fill ETHER/IP/UDP header
	pEthHdr = (struct ether_hdr *) (pChar);
	pIpv4Hdr = (struct iphdr1 *) (pChar + ETHER_HDR_LEN);
	pUdpHdr = (struct udphdr1 *) (pChar + ETHER_HDR_LEN + sizeof(struct iphdr1));
	pUdpPayload = (uint8_t *) (pChar + ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1));
	ether_addr_copy(&sSrcEthAddr, &pEthHdr->s_addr);
	ether_addr_copy(&sDstEthAddr, &pEthHdr->d_addr);
	pEthHdr->ether_type = htons(ETHERNET_TYPE);
	pIpv4Hdr->version_ihl = 0x45;
	pIpv4Hdr->type_of_service = 0;
	pIpv4Hdr->total_length = htons(nPktSize - ETHER_HDR_LEN);
	pIpv4Hdr->packet_id = htons(pkgId++);
	pIpv4Hdr->fragment_offset = 0x0000;
	pIpv4Hdr->time_to_live = 0xFF;
	pIpv4Hdr->next_proto_id = IPPROTO_UDP;
	pIpv4Hdr->hdr_checksum = 0x0000;
	pIpv4Hdr->src_addr = src_ip[0] | src_ip[1] << 8 | src_ip[2] << 16 | src_ip[3] << 24;
	pIpv4Hdr->dst_addr = dst_ip[0] | dst_ip[1] << 8 | dst_ip[2] << 16 | dst_ip[3] << 24;

	pUdpHdr->src_port = htons(src_port);
	pUdpHdr->dst_port = htons(dst_port);
	pUdpHdr->dgram_len = htons(nPktSize - ETHER_HDR_LEN - sizeof(struct iphdr1));
	pUdpHdr->dgram_cksum = 0x0000;

	pCharData = (UWORD8 *) pUdpHdr + sizeof(struct udphdr1);
	pCharData[0] = 'A';
	pCharData[1] = 'S';
	pCharData[2] = 'T';
	pCharData[3] = 'R';
	pCharData[4] = 'I';
	pCharData[5] = (isLoopBackMode == 0) ? 0xB0 : 0xBF;
	if (isLoopBackMode == 1 && extraPadding > sizeof (uint64_t))
	{
		uint64_t *pCurTime = (uint64_t*) &pCharData[6];
		*pCurTime = rte_rdtsc();
	}

	rte_pktmbuf_pkt_len(tx_pkt) = nPktSize;
	rte_pktmbuf_data_len(tx_pkt) = nPktSize;

	pIpv4Hdr->hdr_checksum = ip_fast_csum((uint8_t*)pIpv4Hdr, 5); //rte_ipv4_phdr_cksum(pIpv4Hdr, PKT_TX_IPV4 | PKT_TX_IP_CKSUM | PKT_TX_UDP_CKSUM);

#if defined(ASTRI_DAUL_PORT_NIC)
	nTx = rte_eth_tx_burst((uint8_t)(PORT_ID + nCellIdx), 0, &tx_pkt, (uint16_t)1);
#else
    nTx = rte_eth_tx_burst((uint8_t)PORT_ID, 0, &tx_pkt, (uint16_t)1);
#endif

}

#ifdef FB_API
typedef struct{
 uint8_t reserved[2];
 uint8_t prach_freq_offset; //0 - (UL_NUMRB-6)
 uint8_t prach_config_idx; //0-63
}fe_prachconfig_t;

WORD32 sendCsrPacket_Init(struct rte_mempool *pMPool, WORD32 nCellIdx, WORD32	isDispRRUInfo, WORD32 isLoopBackMode, WORD32 extraPadding)
{
	rte_mbuf *tx_pkt = NULL;
	UWORD8 *pChar;
	WORD16 nUdpDataSize = ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1) + FB_API_PAYLOAD_OFFSET;
	struct ether_hdr *pEthHdr;
	struct iphdr1 *pIpv4Hdr;
	struct udphdr1 *pUdpHdr;
	uint8_t * pUdpPayload;
	static UWORD16 pkgId = 0;
	uint8_t src_ip[4] = {10,88,120,122};
	uint8_t dst_ip[4] = {10,88,120,133};
	dst_ip[3] += nCellIdx; // cell 0: 10.88.120.133; cell 1: 10.88.120.134; cell 2: 10.88.120.135 ....
	uint16_t src_port = HOST_PORT;
	uint16_t dst_port = CLIENT_PORT;
	struct ether_addr sSrcEthAddr, sDstEthAddr;
	WORD16 nPktSize = 0;
    RpeCsrPkt *pHeader;
	UWORD8 *pCharData;
	WORD32 nTx;
	UWORD8 nidx = 0;
	fe_prachconfig_t *crsPrachConfig;

    //---------------------------------
    //Send CSR packet for each cell
    //---------------------------------
	const FAPI_SYSTEM_PARAM* pSysPara = GetFapiAdapterPtr(nCellIdx)->getSysParaPtr();

	sSrcEthAddr.addr_bytes[0] = bbu_port_macs[PORT_ID].addr_bytes[0];
	sSrcEthAddr.addr_bytes[1] = bbu_port_macs[PORT_ID].addr_bytes[1];
	sSrcEthAddr.addr_bytes[2] = bbu_port_macs[PORT_ID].addr_bytes[2];
	sSrcEthAddr.addr_bytes[3] = bbu_port_macs[PORT_ID].addr_bytes[3];
	sSrcEthAddr.addr_bytes[4] = bbu_port_macs[PORT_ID].addr_bytes[4];
	sSrcEthAddr.addr_bytes[5] = bbu_port_macs[PORT_ID].addr_bytes[5];

	sDstEthAddr.addr_bytes[0] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][0];
	sDstEthAddr.addr_bytes[1] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][1];
	sDstEthAddr.addr_bytes[2] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][2];
	sDstEthAddr.addr_bytes[3] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][3];
	sDstEthAddr.addr_bytes[4] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][4];
	sDstEthAddr.addr_bytes[5] = g_sUeMacAddrGroup.cMacAddr[nCellIdx][5];

#if ASTRI_PHY == 1
    src_ip[0] = g_sUeMacAddrGroup.cBbuIpAddr[0][0];
    src_ip[1] = g_sUeMacAddrGroup.cBbuIpAddr[0][1];
    src_ip[2] = g_sUeMacAddrGroup.cBbuIpAddr[0][2];
    src_ip[3] = g_sUeMacAddrGroup.cBbuIpAddr[0][3];

    dst_ip[0] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][0];
    dst_ip[1] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][1];
    dst_ip[2] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][2];
    dst_ip[3] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][3];
#endif

	if (isDispRRUInfo == 1)
	{
		printf("[RRU %d]\n", nCellIdx);
		printf("src mac = %02X:%02X:%02X:%02X:%02X:%02X\n", sSrcEthAddr.addr_bytes[0], sSrcEthAddr.addr_bytes[1], sSrcEthAddr.addr_bytes[2], sSrcEthAddr.addr_bytes[3], sSrcEthAddr.addr_bytes[4], sSrcEthAddr.addr_bytes[5]);
		printf("dst mac = %02X:%02X:%02X:%02X:%02X:%02X\n", sDstEthAddr.addr_bytes[0], sDstEthAddr.addr_bytes[1], sDstEthAddr.addr_bytes[2], sDstEthAddr.addr_bytes[3], sDstEthAddr.addr_bytes[4], sDstEthAddr.addr_bytes[5]);
		printf("src ip = %d.%d.%d.%d, port = %d\n", src_ip[0], src_ip[1], src_ip[2], src_ip[3], src_port);
		printf("dst_ip ip = %d.%d.%d.%d, port = %d\n", dst_ip[0], dst_ip[1], dst_ip[2], dst_ip[3], dst_port);
	}

	nPktSize = nUdpDataSize + sizeof(RpeCsrPkt);

	tx_pkt = rte_pktmbuf_alloc(pMPool);
	if (tx_pkt == NULL)
	{
		printf("send_rru_init_pkt ERROR: rte_pktmbuf_alloc error\n");
	}

	pChar = rte_pktmbuf_mtod(tx_pkt, UWORD8 *);

	// fill ETHER/IP/UDP header
	pEthHdr = (struct ether_hdr *) (pChar);
	pIpv4Hdr = (struct iphdr1 *) (pChar + ETHER_HDR_LEN);
	pUdpHdr = (struct udphdr1 *) (pChar + ETHER_HDR_LEN + sizeof(struct iphdr1));
	pUdpPayload = (uint8_t *) (pChar + ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1));
	ether_addr_copy(&sSrcEthAddr, &pEthHdr->s_addr);
	ether_addr_copy(&sDstEthAddr, &pEthHdr->d_addr);
	pEthHdr->ether_type = htons(ETHERNET_TYPE);
	pIpv4Hdr->version_ihl = 0x45;
	pIpv4Hdr->type_of_service = 0;
	pIpv4Hdr->total_length = htons(nPktSize - ETHER_HDR_LEN);
	pIpv4Hdr->packet_id = htons(pkgId++);
	pIpv4Hdr->fragment_offset = 0x0000;
	pIpv4Hdr->time_to_live = 0xFF;
	pIpv4Hdr->next_proto_id = IPPROTO_UDP;
	pIpv4Hdr->hdr_checksum = 0x0000;
	pIpv4Hdr->src_addr = src_ip[0] | src_ip[1] << 8 | src_ip[2] << 16 | src_ip[3] << 24;
	pIpv4Hdr->dst_addr = dst_ip[0] | dst_ip[1] << 8 | dst_ip[2] << 16 | dst_ip[3] << 24;

	pUdpHdr->src_port = htons(src_port);
	pUdpHdr->dst_port = htons(dst_port);
	pUdpHdr->dgram_len = htons(nPktSize - ETHER_HDR_LEN - sizeof(struct iphdr1));
	pUdpHdr->dgram_cksum = 0x0000;

	pCharData = (UWORD8 *) pUdpHdr + sizeof(struct udphdr1);
	pCharData[0] = 'A';
	pCharData[1] = 'S';
	pCharData[2] = 'T';
	pCharData[3] = 'R';
	pCharData[4] = 'I';
	pCharData[5] = 0xA0;


	pHeader = (RpeCsrPkt *)(pChar + nUdpDataSize);
	/*
	pCharData = (UWORD8 *)pHeader + sizeof(RpeCsrPkt);

	if (isLoopBackMode == 1 && extraPadding > sizeof (uint64_t))
	{
		uint64_t *pCurTime = (uint64_t*) &pCharData[6];
		*pCurTime = rte_rdtsc();
	}
	*/

	rte_pktmbuf_pkt_len(tx_pkt) = nPktSize;
	rte_pktmbuf_data_len(tx_pkt) = nPktSize;


#ifdef FB_API

#ifndef FERRY_BRIDGE_INT
	pHeader->hdr.ethHdr.ether_type = htons(ETHERNET_TYPE);
	pHeader->hdr.vlanHdr.eth_proto = htons(VLAN_TAG_TPID);

#endif
#ifdef FERRY_BRIDGE_INT
	pHeader->hdr.ethHdr.ether_type = htons(VLAN_TAG_TPID);
	pHeader->hdr.vlanHdr.eth_proto = htons(ETHERNET_TYPE);
#endif
	pHeader->hdr.vlanHdr.vlan_tci = 0;
	pHeader->hdr.vlanHdr.vlan_tci = RPE_SET_PACKET_TYPE_BE(pHeader->hdr.vlanHdr.vlan_tci, RPE_CSR_PKT_TYPE);

	pHeader->hdr.vlanHdr.vlan_tci = RPE_SET_CPRI_LINK_NUMBER_BE(pHeader->hdr.vlanHdr.vlan_tci, 1);
	pHeader->hdr.subType = htons(RPE_CSR_PKT_SUB_TYPE);

	/* fill packet payload for Prach Config*/
	pHeader->csrPktHdr.rsvd = 0;
	pHeader->csrPktHdr.rsvd1 = 0;
	pHeader->csrPktHdr.be = 0;
	pHeader->csrPktHdr.tag = 0; //For Prach Config
	pHeader->csrPktHdr.rw = 1;	 //Write
	pHeader->csrPktHdr.addr_h = 0x0;
	pHeader->csrPktHdr.devsel = 0;
	pHeader->csrPktHdr.addr_l = htons(0x600C);

	crsPrachConfig = (fe_prachconfig_t*) &(pHeader->data);
	crsPrachConfig->reserved[0] = 0;
	crsPrachConfig->reserved[1] = 0;
	crsPrachConfig->prach_freq_offset = pSysPara->freq_offset;
	crsPrachConfig->prach_config_idx = pSysPara->configuration_index;
#endif

	pIpv4Hdr->hdr_checksum = ip_fast_csum((uint8_t*)pIpv4Hdr, 5); //rte_ipv4_phdr_cksum(pIpv4Hdr, PKT_TX_IPV4 | PKT_TX_IP_CKSUM | PKT_TX_UDP_CKSUM);

#if defined(ASTRI_DAUL_PORT_NIC)
	nTx = rte_eth_tx_burst((uint8_t)(PORT_ID + nCellIdx), 0, &tx_pkt, (uint16_t)1);
#else
	nTx = rte_eth_tx_burst((uint8_t)PORT_ID, 0, &tx_pkt, (uint16_t)1);
#endif
}


WORD32 sendCsrPacket(BbuPoolCtrlBlkStruct *pBbu)
{
	WORD32 nCellIdx, nTx, nTxBurstLen;
	UWORD8 *pChar;
	BbuFeIoIfStruct *pBbuFeIoIf;
	BbuIoBufCtrlStruct *pBbuIoBufCtrl;
	fe_prachconfig_t *crsPrachConfig;

	pBbuFeIoIf = &(pBbu->sBbuIoIf);
	pBbuIoBufCtrl = &(pBbuFeIoIf->sCsrBbuIoBufCtrl[0]);

    struct rte_mempool *pMPool = pBbuFeIoIf->bbuio_buf_pool;

    WORD16 nPktSize;
    WORD32 *pPktCnt;

    RpeCsrPkt *pHeader;

    WORD16 *pIqSample;
    UWORD8 *pCharData;
    UWORD16 pktHdrLen;

    struct ether_addr sSrcEthAddr, sDstEthAddr;

    pBbuIoBufCtrl->nSegTransferred = 0;

    //---------------------------------
    //Send CSR packet for each cell
    //---------------------------------
    for (nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
    {
    	const FAPI_SYSTEM_PARAM* pSysPara = GetFapiAdapterPtr(nCellIdx)->getSysParaPtr();

#ifndef FERRY_BRIDGE_INT
		UWORD8 *dstMac = &(pBbuFeIoIf->ucFeMac[nCellIdx][0]);
#endif

#ifndef FERRY_BRIDGE_INT
		memcpy(&(sDstEthAddr.addr_bytes[0]), dstMac, 6);
#if defined(ASTRI_RRU)
		memcpy(sSrcEthAddr.addr_bytes, bbu_port_macs[PORT_ID].addr_bytes, 6);
#else
		sSrcEthAddr.addr_bytes[0] = 0;
		sSrcEthAddr.addr_bytes[1] = 0;
		sSrcEthAddr.addr_bytes[2] = 0;
		sSrcEthAddr.addr_bytes[3] = pBbu->nBbuIdx;
		sSrcEthAddr.addr_bytes[4] = pBbu->sCellCtrlBlk[nCellIdx].nSectorID;
		sSrcEthAddr.addr_bytes[5] = pBbu->nPoolIdx;
#endif //ASTRI_RRU
#endif

#ifdef FERRY_BRIDGE_INT
		memcpy(sSrcEthAddr.addr_bytes, bbu_port_macs[0].addr_bytes, 6);
		sDstEthAddr.addr_bytes[0] = 0xFF;
		sDstEthAddr.addr_bytes[1] = 0xFF;
		sDstEthAddr.addr_bytes[2] = 0xFF;
		sDstEthAddr.addr_bytes[3] = 0xFF;
		sDstEthAddr.addr_bytes[4] = 0xFF;
		sDstEthAddr.addr_bytes[5] = 0xFF;
#endif

		/* estimate packet number */
		pBbuIoBufCtrl->nSegToBeGen = 1;
		pPktCnt = &(pBbuIoBufCtrl->nSegGenerated);
		*pPktCnt = 0;

#if defined(ASTRI_RRU)
		WORD16 nUdpDataSize = ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1) + FB_API_PAYLOAD_OFFSET;
		struct ether_hdr *pEthHdr;
		struct iphdr1 *pIpv4Hdr;
		struct udphdr1 *pUdpHdr;
		uint8_t * pUdpPayload;
		static UWORD16 pkgId = 0;
		uint8_t src_ip[4] = {10,88,120,122};
		uint8_t dst_ip[4] = {10,88,120,133};
		dst_ip[3] += nCellIdx; // cell 0: 10.88.120.133; cell 1: 10.88.120.134; cell 2: 10.88.120.135 ....
		uint16_t src_port = HOST_PORT;
		uint16_t dst_port = CLIENT_PORT;
#endif //ASTRI_RRU

#if defined(ASTRI_RRU)
#if ASTRI_PHY == 1
		src_ip[0] = g_sUeMacAddrGroup.cBbuIpAddr[0][0];
		src_ip[1] = g_sUeMacAddrGroup.cBbuIpAddr[0][1];
		src_ip[2] = g_sUeMacAddrGroup.cBbuIpAddr[0][2];
		src_ip[3] = g_sUeMacAddrGroup.cBbuIpAddr[0][3];

		dst_ip[0] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][0];
		dst_ip[1] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][1];
		dst_ip[2] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][2];
		dst_ip[3] = g_sUeMacAddrGroup.cIpAddr[nCellIdx][3];
#endif
#endif

#ifdef FB_API
#if defined(ASTRI_RRU)
		nPktSize = nUdpDataSize + sizeof(RpeCsrPkt);
#else
		nPktSize = sizeof(RpeCsrPkt);
#endif //ASTRI_RRU
#endif

		pBbuIoBufCtrl->pData[*pPktCnt] = NULL;
		pBbuIoBufCtrl->pData[*pPktCnt] = rte_pktmbuf_alloc(pMPool);
		if (NULL == pBbuIoBufCtrl->pData[*pPktCnt])
		{
			printf("Cannot get memory from huge page for TX CSR\n");
			rte_dump_stack();
			usleep(1000);
			exit(-1);
		}

		pChar = rte_pktmbuf_mtod(pBbuIoBufCtrl->pData[*pPktCnt], UWORD8 *);

#if defined(ASTRI_RRU)
		// fill ETHER/IP/UDP header
		pEthHdr = (struct ether_hdr *) (pChar);
		pIpv4Hdr = (struct iphdr1 *) (pChar + ETHER_HDR_LEN);
		pUdpHdr = (struct udphdr1 *) (pChar + ETHER_HDR_LEN + sizeof(struct iphdr1));
		pUdpPayload = (uint8_t *) (pChar + ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1));
		ether_addr_copy(&sSrcEthAddr, &pEthHdr->s_addr);
		ether_addr_copy(&sDstEthAddr, &pEthHdr->d_addr);
		pEthHdr->ether_type = htons(ETHERNET_TYPE);
		pIpv4Hdr->version_ihl = 0x45;
		pIpv4Hdr->type_of_service = 0;
		pIpv4Hdr->total_length = htons(nPktSize - ETHER_HDR_LEN);
		pIpv4Hdr->packet_id = htons(pkgId++);
		pIpv4Hdr->fragment_offset = 0x0000;
		pIpv4Hdr->time_to_live = 0xFF;
		pIpv4Hdr->next_proto_id = IPPROTO_UDP;
		pIpv4Hdr->hdr_checksum = 0x0000;
		pIpv4Hdr->src_addr = src_ip[0] | src_ip[1] << 8 | src_ip[2] << 16 | src_ip[3] << 24;
		pIpv4Hdr->dst_addr = dst_ip[0] | dst_ip[1] << 8 | dst_ip[2] << 16 | dst_ip[3] << 24;

		pUdpHdr->src_port = htons(src_port);
		pUdpHdr->dst_port = htons(dst_port);
		pUdpHdr->dgram_len = htons(nPktSize - ETHER_HDR_LEN - sizeof(struct iphdr1));
		pUdpHdr->dgram_cksum = 0x0000;

		pCharData = (UWORD8 *) pUdpHdr + sizeof(struct udphdr1);
		pCharData[0] = 'A';
		pCharData[1] = 'S';
		pCharData[2] = 'T';
		pCharData[3] = 'R';
		pCharData[4] = 'I';
		pCharData[5] = 0xA0;
#endif //ASTRI_RRU

#ifdef FB_API
#if defined(ASTRI_RRU)
		pHeader = (RpeCsrPkt *)(pChar + nUdpDataSize);
#else
		pHeader = (RpeCsrPkt *)pChar;
#endif //ASTRI_RRU
		pCharData = (UWORD8 *)pHeader + sizeof(RpeCsrPkt);
#endif

		rte_pktmbuf_pkt_len(pBbuIoBufCtrl->pData[*pPktCnt]) = nPktSize;
		rte_pktmbuf_data_len(pBbuIoBufCtrl->pData[*pPktCnt]) = nPktSize;

#ifdef FB_API
		/* fill packet header */
		ether_addr_copy(&sSrcEthAddr, &pHeader->hdr.ethHdr.s_addr);
		ether_addr_copy(&sDstEthAddr, &pHeader->hdr.ethHdr.d_addr);
#ifndef FERRY_BRIDGE_INT
		pHeader->hdr.ethHdr.ether_type = htons(ETHERNET_TYPE);
		pHeader->hdr.vlanHdr.eth_proto = htons(VLAN_TAG_TPID);
#endif
#ifdef FERRY_BRIDGE_INT
		pHeader->hdr.ethHdr.ether_type = htons(VLAN_TAG_TPID);
		pHeader->hdr.vlanHdr.eth_proto = htons(ETHERNET_TYPE);
#endif
		pHeader->hdr.vlanHdr.vlan_tci = 0;
		pHeader->hdr.vlanHdr.vlan_tci = RPE_SET_PACKET_TYPE_BE(pHeader->hdr.vlanHdr.vlan_tci, RPE_CSR_PKT_TYPE);
		pHeader->hdr.vlanHdr.vlan_tci = RPE_SET_CPRI_LINK_NUMBER_BE(pHeader->hdr.vlanHdr.vlan_tci, 1);
		pHeader->hdr.subType = htons(RPE_CSR_PKT_SUB_TYPE);

		/* fill packet payload for Prach Config*/
		pHeader->csrPktHdr.rsvd = 0;
		pHeader->csrPktHdr.rsvd1 = 0;
		pHeader->csrPktHdr.be = 0;
		pHeader->csrPktHdr.tag = 0; //For Prach Config
		pHeader->csrPktHdr.rw = 1;	 //Write
		pHeader->csrPktHdr.addr_h = 0x0;
		pHeader->csrPktHdr.devsel = 0;
		pHeader->csrPktHdr.addr_l = htons(0x600C);

		crsPrachConfig = (fe_prachconfig_t*) &(pHeader->data);
		crsPrachConfig->reserved[0] = 0;
		crsPrachConfig->reserved[1] = 0;
		crsPrachConfig->prach_freq_offset = pSysPara->freq_offset;
		crsPrachConfig->prach_config_idx = pSysPara->configuration_index;
#endif

#if defined(ASTRI_RRU)
			// todo: check if needed
			// todo: update dpdk version and call rte_ipv4_phdr_cksum()
			pIpv4Hdr->hdr_checksum = ip_fast_csum((uint8_t*)pIpv4Hdr, 5); //rte_ipv4_phdr_cksum(pIpv4Hdr, PKT_TX_IPV4 | PKT_TX_IP_CKSUM | PKT_TX_UDP_CKSUM);
#endif //ASTRI_RRU

		*pPktCnt += 1;

		nTxBurstLen = pBbuIoBufCtrl->nSegGenerated - pBbuIoBufCtrl->nSegTransferred;

		if(pBbuIoBufCtrl->nSegGenerated > pBbuIoBufCtrl->nSegTransferred)
		{

#ifndef FERRY_BRIDGE_INT
#if defined(ASTRI_DAUL_PORT_NIC)
					nTx = rte_eth_tx_burst((uint8_t)(PORT_ID + nCellIdx), 0,
							&(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred]),
							(uint16_t)nTxBurstLen);
#else
					nTx = rte_eth_tx_burst((uint8_t)PORT_ID, 0,
							&(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred]),
							(uint16_t)nTxBurstLen);
#endif //ASTRI_DAUL_PORT_NIC
#else
                    nTx = rte_eth_tx_burst((uint8_t)PORT_ID, DATA_TX_Q_ID,
                            &(pBbuIoBufCtrl->pData[pBbuIoBufCtrl->nSegTransferred]),
                            (uint16_t)nTxBurstLen);
#endif
                    pBbuIoBufCtrl->nSegTransferred += nTx;

                    if (pBbuIoBufCtrl->nSegToBeGen == pBbuIoBufCtrl->nSegTransferred) {
                        pBbuIoBufCtrl->nSegGenerated = -1;
                        pBbuIoBufCtrl->nSegToBeGen = -1;
                        pBbuIoBufCtrl->nSegTransferred = 0;

                        printf("CSR %d packet are transmitted for cell %d\n", pBbuIoBufCtrl->nSegTransferred, nCellIdx);
                    }
		}
    }
    return 0;
}
#endif
#endif //ASTRI_RRU

WORD32 bbuio_thread(void *pVoid)
{
    LteRtInforStruct *pLteRt = (LteRtInforStruct *)pVoid;
    /* set_thread_priority(pthread_self(), MAIN_PRIORITY); */
#if defined(ASTRI_PHY) && !defined(_FAPI_DEBUG_BUILD_)
	set_max_thread_priority(pthread_self());
#endif
    printf("LTE BBUIO thread started on core %d successfully at Tsc(ms)=%llu\n", pLteRt->nCoreId, (UWORD64)(rte_rdtsc()/CPU_HZ/1000));
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;
    PhyCellCtrlBlockStruct *pCellCtrlBlk = NULL;
    struct rte_mbuf *rx_pkts[MAX_PKT_BURST + 1];
    WORD32 n_rx = 0, nCellIdx, nTxSfIdx, nHarqAssocSf[NUM_OF_SUBFRAME_IN_ONE_FRAME];
    WORD32 Idx = 0, AirConfig, nTimeDiff, nActiveCellDl;
    WORD32 idx0, nFirstUlSf, nFirstRachSf, nTmpSf, nRunFlag, nProcessed;
    uint8_t nPortID = PORT_ID;
    uint16_t nQueueID = 0, nFirstCnt = 0;

	g_BbuioMainLoopStarted = 0;
  	
#ifndef TD_CONNECT
#ifndef FB_API
    struct timing_hdr_s *pTiming;
    struct pusch_hdr_s *pusch_hdr;
    uint32_t * pFePktType;
#endif
#ifdef FB_API
    RpeTimingPktPayload *pTiming;
    uint16_t * pFePktType;
    RpeLtePuschPkt *pusch_hdr;
#endif
#endif

#ifdef TD_CONNECT
    RpeTimingPktPayload *pTiming;
    RpeLteTimeDomainFrameStatus *pusch_hdr;
    uint16_t * pFePktType;
#endif
    WORD32 nPack, nPdcchTxCell, nPdschTxCell, nCsrTxCell;
    WORD8 src_UE, env_UE;
    UWORD64 uLatestTsc, t_start, t_end, tPrevTsc, tCurrentTsc;
    UWORD64 uloopTsc;
    WORD32 nPdcchTxIndicator[N_MAX_CELL_PER_BBU + 1], nPdschTxIndicator[N_MAX_CELL_PER_BBU + 1];
    sDstEthAddr.addr_bytes[0] = 0x00;
    sDstEthAddr.addr_bytes[1] = 0x1B;
    sDstEthAddr.addr_bytes[2] = 0x21;
    sDstEthAddr.addr_bytes[3] = 0x7E;
    sDstEthAddr.addr_bytes[4] = 0x2E;
    sDstEthAddr.addr_bytes[5] = 0x26;
#ifdef FERRY_BRIDGE_INT
    RpeStatus status = RPE_STATUS_FAIL;
    WORD32 localFrame = (RADIO_FRAME_START * NUM_OF_SUBFRAME_IN_ONE_FRAME) % MAX_FRAME_NUM;;
#endif

    iLog_Normal(BBU_POOL_CONTROL, "LTE BBUIO thread started on core %d successfully\n", pLteRt->nCoreId);

    //ack_switch_fun(pBbu->sBbuIoIf.bbuio_buf_pool);
   // ack_switch_fun(pBbu->sBbuIoIf.bbuio_buf_pool);

    AirConfig = -100;
#ifdef FDD
    AirConfig =-1;
#endif
#ifdef TDD 
    AirConfig = 1;
#endif
    if (AirConfig == -100) {
        iLog_Warn(BBU_POOL_CONTROL, "NO air interface definition macro\n");
        usleep(1000* 20);
        exit(-1);
    }
    for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
        nPdcchTxIndicator[nCellIdx] = 0;
        nPdschTxIndicator[nCellIdx] = 0;
    }

#ifdef FERRY_BRIDGE_INT
    /*Start the RPE instance here*/
    status = rpeStartInstance(handles);
    if(status != RPE_STATUS_SUCCESS)
    {
        printf("Error starting rpe instances: %s\n", rpeGetStatusText(status));
        free(handles);
    }
    else
    {
        printf("CPRI Link Started Ok\n");
    }

    nQueueID = DATA_RX_Q_ID;
#endif

#if defined(ASTRI_PHY)
	cpu_set_t mask;
	WORD32 NUM_PROCS = sysconf(_SC_NPROCESSORS_CONF);
	sched_getaffinity(gettid(), sizeof(mask), &mask);
	for (int i = 0; i < NUM_PROCS; i++)
	{
		if (CPU_ISSET(i, &mask))
		printf("bbuio_thread sched_getaffinity: i = %d is set\n", i); 
	}
#endif

    //ack_switch_fun(pBbu->sBbuIoIf.bbuio_buf_pool);
    //ack_switch_fun(pBbu->sBbuIoIf.bbuio_buf_pool);
    printf(" Waiting first time packet at Tsc(ms) %llu\n", (UWORD64)(rte_rdtsc()/CPU_HZ/1000));
    /* start sync cell first */
    nFirstUlSf = -1;
    pLteRt->nHeartbeat = 0;
    tPrevTsc = __rdtsc();

#if defined(ASTRI_RRU)
	UWORD64 tSendRRUPkg =  __rdtsc();
	uint8_t aIsGotFirstRRUPkg[N_MAX_CELL_PER_BBU];
	uint8_t isGotAllRRUPkg = 0;
	uint64_t sendPkgPeriodUs = 1000*500;
	WORD32	isDispRRUInfo = 1;
	memset(aIsGotFirstRRUPkg, 0, sizeof(aIsGotFirstRRUPkg));
	readMacAddrGroup("UeMacAddrGroup002.txt", &g_sUeMacAddrGroup);
#endif //ASTRI_RRU


#if defined(ASTRI_RRU) && defined(ASTRI_RRU_TEST)
	// RRU round trip time measurement
	for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
	{
		uint64_t sendPkgPeriodUs = 100;//1000*500;//100;
		WORD32	dataSize = 5000;
		uint64_t lastSendPkgTime = rte_rdtsc();
		WORD32 txCount = 0, rxCount = 0, flushCount = 0;
		double roundTripTimeUs[RRT_TEST_NUM_TRAIL];
		
	
		printf("\n");
		printf("\n");
		printf("===== Round trip time measurement for RRU %d/%d start =====\n", nCellIdx, (g_config_data->sectorNum - 1));
		printf("sendPkgPeriodUs = %d, dataSize = %d\n, RRT_TEST_NUM_TRAIL = %d\n", sendPkgPeriodUs, dataSize, RRT_TEST_NUM_TRAIL);
		while(1)
		{
			if (rxCount >= RRT_TEST_NUM_TRAIL)
				break;

			//try tx some packets
			if((rte_rdtsc() - lastSendPkgTime) >= sendPkgPeriodUs * CPU_HZ)
			{		
				lastSendPkgTime = rte_rdtsc();
				send_rru_init_pkt(pBbu->sBbuIoIf.bbuio_buf_pool, nCellIdx, 0, 1, dataSize);
				txCount++;
				
			}

			//try rx some packets
			{
				n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
#if defined(ASTRI_DAUL_PORT_NIC)
				WORD32 n_rxTmp = 0;
				n_rxTmp = rte_eth_rx_burst(nPortID + 1, nQueueID, &rx_pkts[n_rx], (MAX_PKT_BURST - n_rx));
				n_rx += n_rxTmp;
#endif
				iAssert(n_rx <= MAX_PKT_BURST);
				if (n_rx <= 0)
				{
					usleep(10);
				}
				for (idx0 = 0;idx0 < n_rx; idx0++) 
				{
					if (flushCount < RRT_TEST_FLUSH_PKGCOUNT)
					{
						flushCount++;
						continue;						
					}
					
					if (rxCount >= RRT_TEST_NUM_TRAIL)
						break;
					
					WORD32 cellIdxDetected = -1;
					WORD32 isLoopBackPacket = 0;
					if (checkForRruPacket(rx_pkts[idx0], &cellIdxDetected, &isLoopBackPacket) == BBU_POOL_CORRECT && isLoopBackPacket == 1)
					{
						uint8_t * pUdpPayload_rx = (uint8_t *) RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0);
						uint64_t *pPkgTime = (uint64_t*) &pUdpPayload_rx[6];
						uint64_t curTime = rte_rdtsc();
						//printf("*pPkgTime = %lld, curTime = %lld, diff = %lld\n", *pPkgTime, curTime, curTime - *pPkgTime);
						roundTripTimeUs[rxCount] =  (double)(curTime - *pPkgTime) / (double) CPU_HZ;;
						rxCount++;
					}
				}
 			}
		}
		double max_roundTripTimeUs = 0, min_roundTripTimeUs = 0, avg_roundTripTimeUs = 0;
		uint16_t j;
		min_roundTripTimeUs = roundTripTimeUs[0];
		for (j = 0; j < rxCount; j++)
		{
			avg_roundTripTimeUs += roundTripTimeUs[j];
			if (roundTripTimeUs[j] > max_roundTripTimeUs)
				max_roundTripTimeUs = roundTripTimeUs[j];
			if (roundTripTimeUs[j] < min_roundTripTimeUs)
				min_roundTripTimeUs = roundTripTimeUs[j];		
		}
		avg_roundTripTimeUs /= (double) rxCount;
		printf("[Round Trip Time Report]\n");
		for (j = 0; j < rxCount; j++)
		{
			printf("%03d     %0.2fus\n", j, roundTripTimeUs[j]);
		}		
		printf("Max:    %0.2fus\n", max_roundTripTimeUs);
		printf("Min:    %0.2fus\n", min_roundTripTimeUs);
		printf("Avg:    %0.2fus\n", avg_roundTripTimeUs);
		printf("===== Round trip time measurement for RRU %d/%d end =======\n", nCellIdx, g_config_data->sectorNum);
		printf("\n");
		printf("\n");
	}
#endif

    while(nFirstUlSf < 0){
        pLteRt->nHeartbeat ++;

#if defined(ASTRI_RRU)
		if (isDispRRUInfo == 1 || ((__rdtsc() - tSendRRUPkg) >= sendPkgPeriodUs * CPU_HZ && isGotAllRRUPkg == 0))
		{
			for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
				//send_rru_init_pkt(pBbu->sBbuIoIf.bbuio_buf_pool, nCellIdx, isDispRRUInfo, 0, 0);
				sendCsrPacket_Init(pBbu->sBbuIoIf.bbuio_buf_pool, nCellIdx, isDispRRUInfo, 0, 0);
			if (isDispRRUInfo == 1)
				printf("\nWait for RRU ready (Total: %d)", g_config_data->sectorNum);
			isDispRRUInfo = 0;
			tSendRRUPkg =  __rdtsc();
			printf(".");
			fflush(stdout); 
		}
#endif //ASTRI_RRU

#ifdef L1_L2_DECOUPLE
		if (g_isL1D_Mode == L1D_MODE_ENABLE_WITHOUT_RRU)
		{
			n_rx = 0;
			for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
			{		
				if (n_rx >= MAX_PKT_BURST)
					break;
				int pkgSize = ipc_recv_msg(nCellIdx, FAPI_SHMA_BBU_PKG_IN, g_txBBUBuf, MAX_BBU_PKG_SIZE);
				if (pkgSize > 0 && nCellIdx == 0) // ignore all packets from non-zero cell index
				{
#ifdef ASTRI_PHY
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_Rxbuf_pool);
#else
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_buf_pool);
#endif
					if (rx_pkts[n_rx] == NULL)
					{
						printf("bbuio_thread ERROR: rte_pktmbuf_alloc error\n");
					}
					else
					{
						uint8_t *pData = (uint8_t *) rte_pktmbuf_append(rx_pkts[n_rx], pkgSize);
						memcpy(pData, g_txBBUBuf, pkgSize);
						n_rx++;
						//fapi_print("[FAPI msg] [1] got BBU packet from L1D pkgSize = %d, rx_pkts[0] = %p, pData = %p\n", pkgSize, rx_pkts[0], pData);
					}
				}
			}
		}
		else
#endif // L1_L2_DECOUPLE			
		{
			n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
#if defined(ASTRI_DAUL_PORT_NIC)
			WORD32 n_rxTmp = 0;
			n_rxTmp = rte_eth_rx_burst(nPortID + 1, nQueueID, &rx_pkts[n_rx], (MAX_PKT_BURST - n_rx));
			n_rx += n_rxTmp;
#endif
		}
        iAssert(n_rx <= MAX_PKT_BURST);
        if (n_rx <= 0)
        {
            usleep(10);
        }
        for (idx0 = 0;idx0 < n_rx; idx0++) {
#if !defined TD_CONNECT && !defined FB_API
            pFePktType = (uint32_t *) (((WORD8 *)rx_pkts[idx0]->pkt.data) + 12);
#endif
#if defined TD_CONNECT || defined FB_API
#if defined(ASTRI_RRU)
			WORD32 cellIdxDetected = -1;
			WORD32 isLoopBackPacket = 0;
			if (checkForRruPacket(rx_pkts[idx0], &cellIdxDetected, &isLoopBackPacket) == BBU_POOL_CORRECT)
			{
				iAssert(cellIdxDetected >= 0 && cellIdxDetected < g_config_data->sectorNum);
				if (isGotAllRRUPkg == 0)
				{
					if (aIsGotFirstRRUPkg[cellIdxDetected] == 0)
						printf("(RRU %d Ready)", cellIdxDetected);
					aIsGotFirstRRUPkg[cellIdxDetected] = 1;
					uint8_t isGotAllRRUPkgChk = 1;
					for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
					{
						if (aIsGotFirstRRUPkg[nCellIdx] != 1)
						{
							isGotAllRRUPkgChk = 0;
							break;
						}
					}
					
					if (isGotAllRRUPkgChk == 1)
					{
						isGotAllRRUPkg = 1;
						printf("All Ready.\n");
					}

					rte_pktmbuf_free(rx_pkts[idx0]); // free all packets and block the reception of first timing packet before all RRU are ready
					continue;

				}

				if (cellIdxDetected != 0) // free all timing packets from non-zeor cells; free all ul packets from non-zero cells
				{
					rte_pktmbuf_free(rx_pkts[idx0]);
					continue;

				}
			}
			else
			{
				rte_pktmbuf_free(rx_pkts[idx0]);
		        continue;
			}
#endif //ASTRI_RRU
			pFePktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkts[idx0], sizeof(struct ether_hdr) + sizeof(struct vlan_hdr), 0, 0); // +14+4
#endif

            switch(*pFePktType) {
#if !defined TD_CONNECT && !defined FB_API
                case LONG_FE_PKT_TYPE_TIMING:
                    pTiming = (struct timing_hdr_s *)((WORD8 *)rx_pkts[idx0]->pkt.data + ETH_HEADER_SIZE);
#endif
#if defined TD_CONNECT || defined FB_API
                case RPE_TIMING_PKT_SUB_TYPE_SWREC:
					pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0))->message);
#ifdef FERRY_BRIDGE_INT
                    nTmpSf = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
                        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
                    if (nTmpSf != localFrame)
                    {
                        printf("eNB Error timing packet: %d %d %d %d -- %d\n", pTiming->radioFrameNumHigh,
                            pTiming->radioFrameNumLow, pTiming->subFrameNum, nTmpSf, localFrame);
                    }
                    localFrame++;
                    localFrame = (localFrame == MAX_FRAME_NUM) ? 0 : localFrame;
#endif


#endif
                    if (nFirstCnt < 50) {
                        tCurrentTsc = __rdtsc();
                        if ((tCurrentTsc - tPrevTsc) > (1000 * CPU_HZ * 0.8)) {
                            /* waiting until two succeed timing packet arrival time is about 1 ms */
                            nFirstCnt ++;
                        }
                        tPrevTsc = tCurrentTsc;
                    } else {
#if !defined TD_CONNECT && !defined FB_API
                        nFirstUlSf = (ntohs(pTiming->frame_seq) * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subf_seq)
                            %  MAX_FRAME_NUM;
#endif
#if defined TD_CONNECT || defined FB_API
                        nFirstUlSf = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
                        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) %  MAX_FRAME_NUM;
#endif
#ifdef FE_INTEGRATION
                        if (nFirstUlSf != START_SUBFRAME)
                            nFirstUlSf = -1;
#endif
                    }
                    rte_pktmbuf_free(rx_pkts[idx0]);

                    break;
                default:
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    break;
            }
        }
    }
    printf(" Get first time packet at Tsc(ms) %llu with sf=%d\n", (UWORD64)(rte_rdtsc()/CPU_HZ/1000), nFirstUlSf);

    /* get first UL subframe at least 1 seconds later */
    nFirstUlSf = (nFirstUlSf + (WORD32)(1000 / TIME_INTERVAL)) % MAX_FRAME_NUM;
    nFirstRachSf = (nFirstUlSf - 20 + MAX_FRAME_NUM) % MAX_FRAME_NUM;
    nFirstUlSf = get_next_ul_sf(nFirstUlSf);
    while (1) {
        if (nFirstUlSf < (NUM_OF_MAX_FRAME * NUM_OF_SUBFRAME_IN_ONE_FRAME - 4 * NUM_OF_SUBFRAME_IN_ONE_FRAME))
            break;
        else {
            nFirstUlSf = get_next_ul_sf(nFirstUlSf);
        }
    }
    iLog_Debug(BBU_POOL_CONTROL, "Start sync cell at subframe = %d at Tsc=%llu\n",nFirstUlSf,
    rte_rdtsc());
    printf("Start sync cell at subframe = %d at Tsc(ms)=%llu\n",nFirstUlSf, (UWORD64)(rte_rdtsc()/CPU_HZ/1000));
    /* as nFirstUlSf is not at the tail of one super frame, there is no wrap around issue */
    do {
        nFirstRachSf = get_next_rach_sf(nFirstRachSf);
    } while (subframe_compare(nFirstRachSf, nFirstUlSf, 1) < 0);
    iLog_Debug(BBU_POOL_CONTROL, "Start sync cell rach at subframe = %d\n ", nFirstRachSf);

    if (BBU_POOL_ERROR == init_task_queue(pBbu, AirConfig, nFirstUlSf, nFirstRachSf)) {
        iLog_Normal(BBU_POOL_CONTROL, "Cannot init task queue at %d in %s\n", __LINE__, __FILE__);
        usleep(10000);
        iLog_Fatal(BBU_POOL_CONTROL, "Cannot init task queue at %d in %s\n", __LINE__, __FILE__);
        return NULL;
    }

    /* initial sync cell for first subframe */
    pBbu->sCellCtrlBlk[N_MAX_CELL_PER_BBU].nTaskQueueIdx = 0;
    init_task_queue_new_cell(pBbu, N_MAX_CELL_PER_BBU, nFirstUlSf);
    for (idx0 = 0;idx0 < TASK_QUEUE_LEN; idx0 ++) {
        pBbu->eCellActive[idx0][N_MAX_CELL_PER_BBU] = CELL_INACTIVE;
    }
    pBbu->eCellActive[nFirstUlSf % TASK_QUEUE_LEN][N_MAX_CELL_PER_BBU] = CELL_NEW;

    /* active worker thread in advanced */
    pBbu->nCurrentSfIdx = (nFirstUlSf - 1 + MAX_FRAME_NUM) % MAX_FRAME_NUM;
    pBbu->nCurrentPsSfIdx = (pBbu->nCurrentSfIdx + 2 + MAX_FRAME_NUM) % MAX_FRAME_NUM;
    update_cell_status(pBbu, nFirstUlSf);
    get_harq_associated_dl_sf(nFirstUlSf,nHarqAssocSf);
    nTxSfIdx = nHarqAssocSf[0] % (NUM_OF_SUBFRAME_IN_ONE_FRAME * NUM_OF_MAX_FRAME); // this subframe is the first DL subframe

    /* process those packets before nFirstUlSf */
    nRunFlag = 1;
    while(nRunFlag){
        pLteRt->nHeartbeat ++;

#ifdef L1_L2_DECOUPLE
		if (g_isL1D_Mode == L1D_MODE_ENABLE_WITHOUT_RRU)
		{
			n_rx = 0;
			for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
			{
				if (n_rx >= MAX_PKT_BURST)
					break;
				int pkgSize = ipc_recv_msg(nCellIdx, FAPI_SHMA_BBU_PKG_IN, g_txBBUBuf, MAX_BBU_PKG_SIZE);
				if (pkgSize > 0 && nCellIdx == 0) // ignore all packets from non-zero cell index
				{
#ifdef ASTRI_PHY
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_Rxbuf_pool);
#else
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_buf_pool);
#endif
					if (rx_pkts[n_rx] == NULL)
					{
						printf("bbuio_thread ERROR: rte_pktmbuf_alloc error\n");
					}
					else
					{
						uint8_t *pData = (uint8_t *) rte_pktmbuf_append(rx_pkts[n_rx], pkgSize);
						memcpy(pData, g_txBBUBuf, pkgSize);
						n_rx++;
						//fapi_print("[FAPI msg] [2] got BBU packet from L1D pkgSize = %d, rx_pkts[0] = %p, pData = %p\n", pkgSize, rx_pkts[0], pData);
					}
				}
			}
		}
		else
#endif // L1_L2_DECOUPLE	
		{
			n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
#if defined(ASTRI_DAUL_PORT_NIC)
			WORD32 n_rxTmp = 0;
			n_rxTmp = rte_eth_rx_burst(nPortID + 1, nQueueID, &rx_pkts[n_rx], (MAX_PKT_BURST - n_rx));
			n_rx += n_rxTmp;
#endif			
		}
        iAssert(n_rx <= MAX_PKT_BURST);
        if (n_rx <= 0)
        {
            usleep(10);
        }
        nProcessed = 0;
        for (idx0 = 0;idx0 < n_rx; idx0++) {
#if !defined TD_CONNECT && !defined FB_API
            pFePktType = (uint32_t *) (((WORD8 *)rx_pkts[idx0]->pkt.data) + 12);
#endif
#if defined TD_CONNECT || defined FB_API
#if defined(ASTRI_RRU)
			WORD32 cellIdxDetected = -1;
			WORD32 isLoopBackPacket = 0;
			if (checkForRruPacket(rx_pkts[idx0], &cellIdxDetected, &isLoopBackPacket) == BBU_POOL_CORRECT)
			{
				iAssert(cellIdxDetected >= 0 && cellIdxDetected < g_config_data->sectorNum);
				if (cellIdxDetected != 0) // free all timing packets from non-zeor cells; free all ul packets from non-zero cells
				{
					rte_pktmbuf_free(rx_pkts[idx0]);
					nProcessed ++;
					continue;

				}			
			}
			else
			{
				rte_pktmbuf_free(rx_pkts[idx0]);
				nProcessed ++;
				continue;
			}
#endif //ASTRI_RRU
			pFePktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkts[idx0], sizeof(struct ether_hdr) + sizeof(struct vlan_hdr), 0, 0); 
#endif
            nProcessed ++;
            switch(*pFePktType) {
#if !defined TD_CONNECT && !defined FB_API
                case LONG_FE_PKT_TYPE_TIMING:
                    pTiming = (struct timing_hdr_s *)((WORD8 *)rx_pkts[idx0]->pkt.data + ETH_HEADER_SIZE);
                    nTmpSf = (ntohs(pTiming->frame_seq) * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subf_seq) % MAX_FRAME_NUM;
#endif
#if defined TD_CONNECT || defined FB_API
                case RPE_TIMING_PKT_SUB_TYPE_SWREC:
					pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0))->message);
                    nTmpSf = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
                        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
#endif
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    if (((nTmpSf + 1) % MAX_FRAME_NUM) == nFirstUlSf) {
                        nRunFlag = 0;
                    }
                    break;
#ifndef TD_CONNECT
#ifndef FB_API
                case LONG_FE_PKT_TYPE_PUSCH:
                    pusch_hdr = (struct pusch_hdr_s *)((WORD8 *)rx_pkts[idx0]->pkt.data + ETH_HEADER_SIZE);
                    nTmpSf = (ntohs(pusch_hdr->frame_seq) * NUM_OF_SUBFRAME_IN_ONE_FRAME + pusch_hdr->subf_seq) % MAX_FRAME_NUM;
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    if (((nTmpSf + 1) % MAX_FRAME_NUM) == nFirstUlSf) {
                        nRunFlag = 0;
                    }
                    break;
#endif

#ifdef FB_API
                case RPE_PUSCH_PKT_SUB_TYPE_SWREC:
					pusch_hdr = (RpeLtePuschPkt *) RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0);
                    nTmpSf = (ntohs(pusch_hdr->ltePuschHdr.frame_seq) * NUM_OF_SUBFRAME_IN_ONE_FRAME
                    + pusch_hdr->ltePuschHdr.subf_seq) % MAX_FRAME_NUM;
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    if (((nTmpSf + 1) % MAX_FRAME_NUM) == nFirstUlSf) {
                        nRunFlag = 0;
                    }
                    break;
#endif
#endif

#ifdef TD_CONNECT
                case RPE_TIME_DOMAIN_PKT_SUB_SYPE_SWREC:
                    pusch_hdr = &(((RpeLteTimeDomainPkt *)((WORD8 *)rx_pkts[idx0]->pkt.data))->lteTimeDomainFrameStatus);
                    nTmpSf = (((pusch_hdr->rfNumH) * RPE_RADIO_FRAME_LOW_MAX_NUM + pusch_hdr->rfNumL)
                        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pusch_hdr->sfNum) % MAX_FRAME_NUM;
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    if (((nTmpSf + 1) % MAX_FRAME_NUM) == nFirstUlSf) {
                        nRunFlag = 0;
                    }
                    break;
#endif
                default:
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    break;
            }
            if (nRunFlag == 0) {
                break; // remain packets will be processed by main loop
            }
        }
    }

    /* main loop */
    iLog_Debug(BBU_POOL_CONTROL, "DL transmission start at subframe %d Tsc(ms)=%llu\n", nTxSfIdx,
        rte_rdtsc()/CPU_HZ/1000);
    printf("DL transmission start at subframe %d Tsc(ms)=%llu\n", nTxSfIdx,
        (UWORD64)(rte_rdtsc()/CPU_HZ/1000));
    uPrevTsc = rte_rdtsc();
    nTimeOutCnt = 0;
	nIsSubframeIndSent = 0;
	g_BbuioMainLoopStarted = 1;
	g_BbuTimingState = BBU_TIMING_STATE_SYNC;
    /* should be MAX_LOST_TIMING_PKT * 2 so that there are enough time to exit worker threads */
    while(nTimeOutCnt < (MAX_LOST_TIMING_PKT * 2))
    {
        pLteRt->nHeartbeat ++;
        uLatestTsc = rte_rdtsc();
/*
#if defined(L1_L2_DECOUPLE) && !defined(ASTRI_RRU)	
        if (g_isL1D_Mode == L1D_MODE_DISABLE && (uLatestTsc - uPrevTsc) > ((TIME_INTERVAL * 1000 + TIMEOUT_THRESHOLD) * CPU_HZ)) {
#else
		if ((uLatestTsc - uPrevTsc) > ((TIME_INTERVAL * 1000 + TIMEOUT_THRESHOLD) * CPU_HZ)) {
#endif //L1_L2_DECOUPLE
#else
		if ((uLatestTsc - uPrevTsc) > ((TIME_INTERVAL * 1000 + TIMEOUT_THRESHOLD) * CPU_HZ)) {
#endif //L1_L2_DECOUPLE
            nTimeOutCnt ++;
            printf("Timing packet %d timeout at TSC(us) %llu and previous is %llu after rx nTimeOutCnt=%d\n", pBbu->nCurrentSfIdx + 1, uLatestTsc /
                CPU_HZ, uPrevTsc / CPU_HZ, nTimeOutCnt);
            iLog_Warn(BBU_POOL_CONTROL, "Timing packet %d timeout at TSC(us) %llu and previous is %llu after rx\n", pBbu->nCurrentSfIdx + 1, uLatestTsc /
                CPU_HZ, uPrevTsc / CPU_HZ);
            nTimeDiff = (WORD32)((FLOAT32)(uLatestTsc - uPrevTsc)/(TIME_INTERVAL * 1000 * CPU_HZ));
            nPack = (pBbu->nCurrentSfIdx + nTimeDiff) % MAX_FRAME_NUM;
            bbuio_process_timing_pkt_normal(pLteRt, nPack, uLatestTsc, 0);
            t_end = rte_rdtsc();
            bbuio_timing_record(t_end - uLatestTsc, T_TIMEOUT);
        }
*/        

        /* processing received packet */
        iAssert(n_rx <= MAX_PKT_BURST);
        for (Idx = nProcessed; Idx < n_rx; Idx++)
        {
            /* on rx, treat the ethernet prot field and fe subtype as a single
             *  32bit WORD32 */
#if !defined TD_CONNECT && !defined FB_API
            pFePktType = (uint32_t *) (((WORD8 *)rx_pkts[Idx]->pkt.data) + 12);
#endif

#if defined TD_CONNECT || defined FB_API
#if defined(ASTRI_RRU)
			WORD32 cellIdxDetected = -1;
			WORD32 isLoopBackPacket = 0;
			if (checkForRruPacket(rx_pkts[Idx], &cellIdxDetected, &isLoopBackPacket) == BBU_POOL_CORRECT)
			{
				iAssert(cellIdxDetected >= 0 && cellIdxDetected < g_config_data->sectorNum);
				pFePktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkts[Idx], sizeof(struct ether_hdr) + sizeof(struct vlan_hdr), 0, 0);
				// Only keep timing packet from RRU #0
				if (*pFePktType == RPE_TIMING_PKT_SUB_TYPE_SWREC && cellIdxDetected != 0) 
				{
          // check SfnSf Idx
          {
          	RpeTimingPktPayload *pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(rx_pkts[Idx], 0, 0, 0))->message);
            WORD32 nSfIdxRRU = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow) * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
            nSfIdxRRU = nSfIdxRRU % MAX_FRAME_NUM;
            
            WORD32 nTimeDiff = astri_subframe_compare(nSfIdxRRU,  pBbu->nCurrentSfIdx);
             if (abs(nTimeDiff) >= 2) 
               MESSAGE_LOG("Bbuio ERROR: Timing between RRUs are not aligned! nCurrentSfIdx = %d, nSfIdxRRU = %d, cellIdx = %d, nTimeDiff = %d\n", pBbu->nCurrentSfIdx, nSfIdxRRU, cellIdxDetected, nTimeDiff);
               
#if (TIME_PROFILE_MODE == 1)
				TimeProfileRecord( 19,				//TIME_PACKET
								   0,				//TIME OUT CHECK
								   cellIdxDetected,				//Cell Index
								   nSfIdxRRU,			//WORKING SFN Index
								   0,				//CHECKING SFN Index
								   0,				//Duration
								   rte_rdtsc() / CPU_HZ,	//Start time
								   0,				//Finish time
								   pLteRt->nCoreId); //Core ID
#endif               
               
          }
          
        
					rte_pktmbuf_free(rx_pkts[Idx]);
					continue;
				}
			}
			else
			{
				rte_pktmbuf_free(rx_pkts[Idx]);
				continue;
			}
#endif //ASTRI_RRU
			pFePktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkts[Idx], sizeof(struct ether_hdr) + sizeof(struct vlan_hdr), 0, 0);       

			// for L1D phase 2 test mode, check and handle timing packets from non-zero cells
			if (g_isL1D_Mode == L1D_MODE_ENABLE_WITHOUT_RRU && *pFePktType == RPE_TIMING_PKT_SUB_TYPE_SWREC)
			{
				uint8_t * pCellIdx = (uint8_t *) RTE_PKTMBUF_READ(rx_pkts[Idx], 0, 0, 0);       
				if (*pCellIdx != 0)
				{
					RpeTimingPktPayload *pTiming = &(((RpeTimingPkt *) RTE_PKTMBUF_READ(rx_pkts[Idx], 0, 0, 0))->message);
            		WORD32 nSfIdxL1D = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow) * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
            		nSfIdxL1D = nSfIdxL1D % MAX_FRAME_NUM;
            
            		WORD32 nTimeDiff = subframe_compare(nSfIdxL1D,  pBbu->nCurrentSfIdx, 5);

             		if (abs(nTimeDiff) > 3)
	               		printf("Bbuio ERROR: Timing between L1D instances are not aligned! nCurrentSfIdx = %d, nSfIdxL1D = %d, cellIdx = %d\n", pBbu->nCurrentSfIdx, nSfIdxL1D, *pCellIdx);
               
#if (TIME_PROFILE_MODE == 1)
					TimeProfileRecord( 19,				//TIME_PACKET
									   0,				//TIME OUT CHECK
									   *pCellIdx,				//Cell Index
									   nSfIdxL1D,			//WORKING SFN Index
									   0,				//CHECKING SFN Index
									   0,				//Duration
									   rte_rdtsc() / CPU_HZ,	//Start time
									   0,				//Finish time
									   pLteRt->nCoreId); //Core ID
#endif               
					rte_pktmbuf_free(rx_pkts[Idx]);
					continue;				
				}
			}

#endif //defined TD_CONNECT || defined FB_API

            switch(*pFePktType) {
#if !defined TD_CONNECT && !defined FB_API
                case LONG_FE_PKT_TYPE_TIMING:
#endif
#if defined TD_CONNECT || defined FB_API
                case RPE_TIMING_PKT_SUB_TYPE_SWREC:
#endif
                    /* process the timing packet */
                    t_start = rte_rdtsc();
                    bbuio_process_timing_pkt(pLteRt, rx_pkts[Idx]);
                    t_end = rte_rdtsc();
                    bbuio_timing_record(t_end - t_start, T_RX_TIME);
                    nTimeOutCnt = 0;
                    break;
#if !defined TD_CONNECT && !defined FB_API
                case LONG_FE_PKT_TYPE_PRACH:
#endif
#if defined TD_CONNECT || defined FB_API
				case RPE_PRACH_PKT_SUB_TYPE_SWREC:
#endif
					src_UE = *((WORD8 *) RTE_PKTMBUF_READ(rx_pkts[Idx], 10, 0, 0));
					env_UE = *((WORD8 *)RTE_PKTMBUF_READ(rx_pkts[Idx], 11, 0, 0));
                    if(env_UE == pBbu->nPoolIdx)
                    {
                        t_start = rte_rdtsc();
                        //search for nCellIdx
                        pCellCtrlBlk = pBbu->sCellCtrlBlk;
                        for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++)
                        {
#ifdef ASTRI_PHY
							if (nCellIdx == src_UE)
#else
                            if (pCellCtrlBlk->cellID == src_UE)
#endif								
                                break;
                            else
                                pCellCtrlBlk ++;
                        }
                        if (nCellIdx == N_MAX_CELL_PER_BBU || g_IsBbuOutOfSync != 0 || g_IsBbuUnderReset != 0) {
                            rte_pktmbuf_free(rx_pkts[Idx]);
                        } else {
                            bbuio_process_prach_pkt(pLteRt, rx_pkts[Idx], nCellIdx);
                        }
                        t_end = rte_rdtsc();
                        bbuio_timing_record(t_end - t_start, T_RX_PRACH);
                    }
                    else
                        rte_pktmbuf_free(rx_pkts[Idx]);
                    break;
#ifndef TD_CONNECT
#ifndef FB_API
                case LONG_FE_PKT_TYPE_PUSCH:
                    src_UE = *((WORD8 *)rx_pkts[Idx]->pkt.data + 10);
                    env_UE = *((WORD8 *)rx_pkts[Idx]->pkt.data + 11);
                    if(env_UE == pBbu->nPoolIdx)
                    {
                        t_start = rte_rdtsc();
                        //search for nCellIdx
                        pCellCtrlBlk = pBbu->sCellCtrlBlk;
                        for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++)
                        {
#ifdef ASTRI_PHY
							if (nCellIdx == src_UE)
#else
                            if (pCellCtrlBlk->cellID == src_UE)
#endif	                        
                                break;
                            else
                                pCellCtrlBlk ++;
                        }
                        if (nCellIdx == N_MAX_CELL_PER_BBU || g_IsBbuOutOfSync != 0 || g_IsBbuUnderReset != 0) {
                            rte_pktmbuf_free(rx_pkts[Idx]);
                        } else {
                            bbuio_process_pusch_pkt(pLteRt, rx_pkts[Idx], nCellIdx);
                        }
                        t_end = rte_rdtsc();
                        bbuio_timing_record(t_end - t_start, T_RX_PUSCH);
                    }
                    else
                        rte_pktmbuf_free(rx_pkts[Idx]);
                    break;
#endif

#ifdef FB_API
                    case RPE_PUSCH_PKT_SUB_TYPE_SWREC:
#ifndef FERRY_BRIDGE_INT
						src_UE = *((WORD8 *) RTE_PKTMBUF_READ(rx_pkts[Idx], 10, 0, 0));
						env_UE = *((WORD8 *) RTE_PKTMBUF_READ(rx_pkts[Idx], 11, 0, 0));
                        if(env_UE == pBbu->nPoolIdx)
#endif
                        {
                            t_start = rte_rdtsc();
                            //search for nCellIdx
                            pCellCtrlBlk = pBbu->sCellCtrlBlk;
#ifndef FERRY_BRIDGE_INT
                            for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++)
                            {
#ifdef ASTRI_PHY
								if (nCellIdx == src_UE)
#else
                            	if (pCellCtrlBlk->cellID == src_UE)
#endif	                            
                                    break;
                                else
                                    pCellCtrlBlk ++;
                            }
#endif
#ifdef FERRY_BRIDGE_INT
                            nCellIdx = 0;
#endif
                            if (nCellIdx == N_MAX_CELL_PER_BBU || g_IsBbuOutOfSync != 0 || g_IsBbuUnderReset != 0) {
                                rte_pktmbuf_free(rx_pkts[Idx]);
                            } else {
                                bbuio_process_pusch_pkt(pLteRt, rx_pkts[Idx], nCellIdx);
                            }
                            t_end = rte_rdtsc();
                            bbuio_timing_record(t_end - t_start, T_RX_PUSCH);
                        }
#ifndef FERRY_BRIDGE_INT
                        else
                            rte_pktmbuf_free(rx_pkts[Idx]);
#endif
                        break;
#endif
#endif

#ifdef TD_CONNECT
                case RPE_TIME_DOMAIN_PKT_SUB_SYPE_SWREC:
#ifndef FERRY_BRIDGE_INT
                    src_UE = *((WORD8 *)rx_pkts[Idx]->pkt.data + 10);
                    env_UE = *((WORD8 *)rx_pkts[Idx]->pkt.data + 11);
                    if(env_UE == pBbu->nPoolIdx)
#endif
                    {
                        t_start = rte_rdtsc();
                        //search for nCellIdx
                        pCellCtrlBlk = pBbu->sCellCtrlBlk;
#ifndef FERRY_BRIDGE_INT
                        for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++)
                        {
#ifdef ASTRI_PHY
							if (nCellIdx == src_UE)
#else
                            if (pCellCtrlBlk->cellID == src_UE)
#endif                        
                                break;
                            else
                                pCellCtrlBlk ++;
                        }
#endif
#ifdef FERRY_BRIDGE_INT
                        nCellIdx = 0;
#endif
                        if (nCellIdx == N_MAX_CELL_PER_BBU) {
                            rte_pktmbuf_free(rx_pkts[Idx]);
                        } else {
                            bbuio_process_td_ul_pkt(pLteRt, rx_pkts[Idx], nCellIdx);
                        }
                        t_end = rte_rdtsc();
                        bbuio_timing_record(t_end - t_start, T_RX_PUSCH);
                    }
#ifndef FERRY_BRIDGE_INT
                    else
                        rte_pktmbuf_free(rx_pkts[Idx]);
#endif
                    break;
#endif
                default:
                    /* unknown packet type, drop it */
                    UWORD8 *pC = (UWORD8 *)pFePktType;
                    // Stanley 20170419: when connect BBU-RRU though 10G swith this message will pop out. Hide it.
                    /*
                    MESSAGE_LOG("Unkonw pkt type %x %x %x %x, %x %x %x %x, %x %x %x %x, %x %x %x %x, %x %x %x %x at global subframe %d\n",
                        *(pC + 0), *(pC + 1) , *(pC + 2), *(pC + 3),
                        *(pC + 4), *(pC + 5) , *(pC + 6), *(pC + 7),
                        *(pC + 8), *(pC + 9) , *(pC + 10), *(pC + 11),
                        *(pC + 12), *(pC + 13) , *(pC + 14), *(pC + 15),
                        *(pC + 16), *(pC + 17) , *(pC + 18), *(pC + 19),
                        pBbu->nCurrentSfIdx);
                        */
                    rte_pktmbuf_free(rx_pkts[Idx]);
            }
        }

#ifdef L1_L2_DECOUPLE
        uLatestTsc = rte_rdtsc();
		processFapiMsg(pLteRt, pBbu->nCurrentSfIdx, uLatestTsc);
#endif //L1_L2_DECOUPLE

        //ack_switch(pBbu);
        sleep_for_a_while(pBbu);

        /* try transmit DL packets */
        /* only tx at this subframe or 1 subframe in advance */
#ifdef FERRY_BRIDGE_INT
        if (((nTxSfIdx - 1 + MAX_FRAME_NUM) % MAX_FRAME_NUM) == pBbu->nCurrentSfIdx)
#else
        if ((nTxSfIdx  == pBbu->nCurrentSfIdx) ||
            (((nTxSfIdx - 1 + MAX_FRAME_NUM) % MAX_FRAME_NUM) == pBbu->nCurrentSfIdx))
#endif
        {
            t_start = rte_rdtsc();
            nPdcchTxCell = dl_transmission(pBbu, 1,
                nPdcchTxIndicator, nTxSfIdx);
            t_end = rte_rdtsc();
            bbuio_timing_record(t_end - t_start, T_TX_PDCCH);


            t_start = rte_rdtsc();
            nPdschTxCell = dl_transmission(pBbu, 0,
                nPdschTxIndicator, nTxSfIdx);
            t_end = rte_rdtsc();
            bbuio_timing_record(t_end - t_start, T_TX_PDSCH);

            nActiveCellDl = pBbu->sTaskQueue.nTotalActiveCellDl[nTxSfIdx % TASK_QUEUE_LEN];

#ifdef PDCCH_PACKET
            if ( (nPdcchTxCell == nActiveCellDl) &&
                (nPdschTxCell == nActiveCellDl) &&
                (nPdcchTxCell > 0) &&
                (nPdschTxCell > 0)) {
#else
            if ( (nPdschTxCell == nActiveCellDl) &&
                (nPdschTxCell > 0)) {
#endif
                /* packet counters are reset at by update_cell_status */
                // all data from all active cells at this subframe are transmitted, go to next subframe
                iLog_Debug(BBU_POOL_CONTROL, "nTxCell = %5d Subframe %3d data transimssion done\n",
                        nPdcchTxCell + nPdschTxCell, nTxSfIdx);
                nTxSfIdx = (nTxSfIdx + 1) % MAX_FRAME_NUM;
                for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
                    nPdcchTxIndicator[nCellIdx] = 0;
                    nPdschTxIndicator[nCellIdx] = 0;
                }
            }
        }
        else
        {
#ifdef FERRY_BRIDGE_INT
            if (subframe_compare(pBbu->nCurrentSfIdx, nTxSfIdx, 2) >= 0)
#else
            if (subframe_compare(pBbu->nCurrentSfIdx, nTxSfIdx, 2) > 0)
#endif
            {
                /* Only one reason to generate this problem:
                                DL packet for one or more sectors are not ready on time.
                            */
                uLatestTsc = rte_rdtsc();
                nPdcchTxCell = nPdschTxCell = 0;
                for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++)
                {
                    nPdcchTxCell += nPdcchTxIndicator[nCellIdx];
                    nPdschTxCell += nPdschTxIndicator[nCellIdx];
                }
                /*
                printf("Transmission error: nTxSfIdx= %5d Tsc(us)= %llu global = %5d Tsc(u)= %llu, %3d cells PDCCH and %3d cells PDSCH done and nActiveDlCell=%3d\n",
                    nTxSfIdx, uLatestTsc/CPU_HZ, pBbu->nCurrentSfIdx, uPrevTsc/CPU_HZ, nPdcchTxCell, nPdschTxCell, nActiveCellDl);
                printf("nTxIndicator=%d/%d %d/%d %d/%d nTxPktCnt=%d/%d %d/%d %d/%d\n",
                    nPdcchTxIndicator[0], nPdschTxIndicator[0],
                    nPdcchTxIndicator[1], nPdschTxIndicator[1],
                    nPdcchTxIndicator[2], nPdschTxIndicator[2],
                    (pBbuIoBufCtrl + 0)->nSegTransferred, (pBbuIoBufCtrl + 0)->nSegGenerated,
                    (pBbuIoBufCtrl + 1)->nSegTransferred, (pBbuIoBufCtrl + 1)->nSegGenerated,
                    (pBbuIoBufCtrl + 2)->nSegTransferred, (pBbuIoBufCtrl + 2)->nSegGenerated);
*/
                for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++)
                {
                    nPdcchTxIndicator[nCellIdx] = 0;
                    nPdschTxIndicator[nCellIdx] = 0;
                }

                /* those packets that have not been transmitted are freed by update_cell_status */
#ifdef FERRY_BRIDGE_INT
                nTxSfIdx = pBbu->nCurrentSfIdx + 1;
#else
                nTxSfIdx = pBbu->nCurrentSfIdx;
#endif
            }
        }

        uLatestTsc = rte_rdtsc();
/*		
#if defined(L1_L2_DECOUPLE) && !defined(ASTRI_RRU)			
        if (g_isL1D_Mode == L1D_MODE_DISABLE && (uLatestTsc - uPrevTsc) > ((TIME_INTERVAL * 1000 + TIMEOUT_THRESHOLD) * CPU_HZ)) {
#else
		if ((uLatestTsc - uPrevTsc) > ((TIME_INTERVAL * 1000 + TIMEOUT_THRESHOLD) * CPU_HZ)) {
#endif //L1_L2_DECOUPLE
            nTimeOutCnt ++;
            printf("Timing packet %d timeout at TSC(us) %llu and previous is %llu after tx nTimeOutCnt=%d\n", pBbu->nCurrentSfIdx + 1, uLatestTsc /
                CPU_HZ, uPrevTsc / CPU_HZ, nTimeOutCnt);
            iLog_Warn(BBU_POOL_CONTROL, "Timing packet %d timeout at TSC(us) %llu and previous is %llu after tx\n", pBbu->nCurrentSfIdx + 1, uLatestTsc /
                CPU_HZ, uPrevTsc / CPU_HZ);
            nTimeDiff = (WORD32)((FLOAT32)(uLatestTsc - uPrevTsc)/(TIME_INTERVAL * 1000 * CPU_HZ));
            nPack = (pBbu->nCurrentSfIdx + nTimeDiff) % MAX_FRAME_NUM;
            bbuio_process_timing_pkt_normal(pLteRt, nPack, uLatestTsc, 0);
            t_end = rte_rdtsc();
            bbuio_timing_record(t_end - uLatestTsc, T_TIMEOUT);
        }
*/        

        /* try rx some packets */
        t_start = rte_rdtsc();

#ifdef L1_L2_DECOUPLE
		if (g_isL1D_Mode == L1D_MODE_ENABLE_WITHOUT_RRU)
		{
			n_rx = 0;
			for(nCellIdx = 0; nCellIdx < g_config_data->sectorNum; nCellIdx++)
			{
				if (n_rx >= MAX_PKT_BURST)
					break;
				int pkgSize = ipc_recv_msg(nCellIdx, FAPI_SHMA_BBU_PKG_IN, g_txBBUBuf, MAX_BBU_PKG_SIZE);
				if (pkgSize > 0)
				{
#ifdef ASTRI_PHY			
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_Rxbuf_pool);
#else
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_buf_pool);
#endif
					if (rx_pkts[n_rx] == NULL)
					{
						printf("bbuio_thread ERROR: rte_pktmbuf_alloc error\n");
					}
					else
					{
						uint8_t *pData = (uint8_t *) rte_pktmbuf_append(rx_pkts[n_rx], pkgSize);
						memcpy(pData, g_txBBUBuf, pkgSize);
						n_rx++;
						//fapi_print("[FAPI msg] [3] got BBU packet from L1D pkgSize = %d, rx_pkts[0] = %p, pData = %p\n", pkgSize, rx_pkts[0], pData);
					}
				}
			}
		}
		else
#endif //INTEL_IPC			
		{
			n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
#if defined(ASTRI_DAUL_PORT_NIC)
			WORD32 n_rxTmp = 0;
			n_rxTmp = rte_eth_rx_burst(nPortID + 1, nQueueID, &rx_pkts[n_rx], (MAX_PKT_BURST - n_rx));
			n_rx += n_rxTmp;
#endif				
		}

        t_end = rte_rdtsc();
        if(n_rx > 0)
            bbuio_timing_record(t_end - t_start, T_RX_DPDK);
        else            
            // sleep for a while so to allow DPDK to handle interrupt at eal_intr_thread_main() thread
            usleep(20);

        nProcessed = 0;
    }

    bRunFlag = 0;

    return NULL;
}

void inline timing_update(WORD32 nSfIdx, UWORD64 ulTsc, BbuPoolCtrlBlkStruct *pBbu) {
    WORD32 nCellIdx, nDlSf, nDlSfIdx, nAdditionTsc;
    SubframeTimingStruct *pSfTiming;
    WORD32 nHarqDlSf[NUM_OF_SUBFRAME_IN_ONE_FRAME], nSfIdxMod;
#if defined TDD && defined ASTRI_TDD_MODE
    UWORD8 subfCfg;
#endif	
    nSfIdxMod = nSfIdx % TASK_QUEUE_LEN;
#if defined TDD && defined ASTRI_TDD_MODE
    for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
         subfCfg = pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg;
	     if ((BBU_POOL_CORRECT == is_a_ul_sf(nSfIdx)) |
            (BBU_POOL_CORRECT == is_a_switch_sf(subfCfg, nSfIdx))) {
             pSfTiming = &(pBbu->sUlSubframeTiming[nSfIdxMod][0]);
             switch (pBbu->eCellActive[nSfIdxMod][nCellIdx]) {
                case CELL_NEW:
                case CELL_NEWING_1:
                case CELL_NEWING_2:
                case CELL_TO_BE_ACTIVE:
                case CELL_ACTIVE:
                    (pSfTiming + nCellIdx)->nSubframeIdx = nSfIdx;
                    (pSfTiming + nCellIdx)->ulTsc = ulTsc;
                    break;
                default:
                    break;
            }
        }
	} 
        for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
           subfCfg = pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg;
           if ((BBU_POOL_CORRECT == is_a_ul_sf(nSfIdx)) |
              (BBU_POOL_CORRECT == is_a_switch_sf(subfCfg, nSfIdx))) {
				   nDlSf = get_harq_associated_dl_sf(nSfIdx, nHarqDlSf);
                   ulTsc = rte_rdtsc();
             for (nDlSfIdx = 0; nDlSfIdx < nDlSf; nDlSfIdx ++) {
                pSfTiming = &(pBbu->sDlSubframeTiming[nHarqDlSf[nDlSfIdx] % TASK_QUEUE_LEN][0]);
                nAdditionTsc = (nHarqDlSf[nDlSfIdx] - 1.6) * TIME_INTERVAL * 1000 * CPU_HZ;
				    	switch (pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][nCellIdx]) {
                            case CELL_NEW:
                            case CELL_NEWING_1:
                            case CELL_NEWING_2:
                            case CELL_TO_BE_ACTIVE:
                            case CELL_ACTIVE:
                                (pSfTiming + nCellIdx)->nSubframeIdx = nHarqDlSf[nDlSfIdx] % MAX_FRAME_NUM;
                                (pSfTiming + nCellIdx)->ulTsc = ulTsc + nAdditionTsc +
                                        pBbu->sBbuIoIf.nTimingOffset[nCellIdx] * CPU_HZ;
                                 break;
                            default:
                                break;
                    }
                 }
			   }
        }
#else
    if ((BBU_POOL_CORRECT == is_a_ul_sf(nSfIdx)) |
        (BBU_POOL_CORRECT == is_a_switch_sf(nSfIdx))) {
        pSfTiming = &(pBbu->sUlSubframeTiming[nSfIdxMod][0]);
        for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
            switch (pBbu->eCellActive[nSfIdxMod][nCellIdx]) {
                case CELL_NEW:
                case CELL_NEWING_1:
                case CELL_NEWING_2:
                case CELL_TO_BE_ACTIVE:
                case CELL_ACTIVE:
                    (pSfTiming + nCellIdx)->nSubframeIdx = nSfIdx;
                    (pSfTiming + nCellIdx)->ulTsc = ulTsc;
                    break;
                default:
                    break;
            }
        }

        nDlSf = get_harq_associated_dl_sf(nSfIdx, nHarqDlSf);
        ulTsc = rte_rdtsc();
        for (nDlSfIdx = 0; nDlSfIdx < nDlSf; nDlSfIdx ++) {
            pSfTiming = &(pBbu->sDlSubframeTiming[nHarqDlSf[nDlSfIdx] % TASK_QUEUE_LEN][0]);
            nAdditionTsc = (nHarqDlSf[nDlSfIdx] - 1.6) * TIME_INTERVAL * 1000 * CPU_HZ;
            for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
                switch (pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][nCellIdx]) {
                    case CELL_NEW:
                    case CELL_NEWING_1:
                    case CELL_NEWING_2:
                    case CELL_TO_BE_ACTIVE:
                    case CELL_ACTIVE:
                        (pSfTiming + nCellIdx)->nSubframeIdx = nHarqDlSf[nDlSfIdx] % MAX_FRAME_NUM;
                        (pSfTiming + nCellIdx)->ulTsc = ulTsc + nAdditionTsc +
                                pBbu->sBbuIoIf.nTimingOffset[nCellIdx] * CPU_HZ;
                        break;
                    default:
                        break;
                }
            }
        }
    }
#endif	

    return;
}

#ifdef L1_L2_DECOUPLE
inline WORD32 packet_missed_exception_process_dl(WORD32 nSfIdx, BbuPoolCtrlBlkStruct *pBbu) 
{
    WORD32 nAtenna;
    WORD32 nBlk;
    WORD32 nExceptionFlag = 0;
    BbuIoBufCtrlStruct *pBbuIo;
    WORD32 nCellIdx;

    pBbuIo = &(pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nSfIdx % TASK_QUEUE_LEN][0]);
    for (nCellIdx =0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) { // from 0 to N_MAX_CELL_PER_BBU inculsive
        switch (pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][nCellIdx]) {
            case CELL_NEW:
            case CELL_NEWING_1:
            case CELL_NEWING_2:
            case CELL_IMMIGRATION_2:
            case CELL_IMMIGRATION_3:
            case CELL_TO_BE_ACTIVE:
            case CELL_ACTIVE:
				pBbu->sBbuIoIf.sPdcchBbuIoBufCtrl[nSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
				if (pBbu->sBbuIoIf.sPdcchBbuIoBufCtrl[nSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSchedulerDone == 1 && pBbu->sBbuIoIf.sPdcchBbuIoBufCtrl[nSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiTaskGenerated == 0)
				{
					gen_dl_ctrl_task(pBbu, nSfIdx, nCellIdx);
				}
				pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSubFrameDone = 1;
				if (pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiSchedulerDone == 1 && pBbu->sBbuIoIf.sDlBbuIoBufCtrl[nSfIdx % N_FE_BUF_LEN][nCellIdx].nFapiTaskGenerated == 0)
				{
					gen_dl_data_task(pBbu, nSfIdx, nCellIdx);
				}
                break;
            default:
                break;
        }
    }

    return nExceptionFlag;
}

#endif //L1_L2_DECOUPLE

/* this function check if all cells' UL subframe at one UL subframe have been trigged.
   If not, trig it no matter if UL data is ready */
inline WORD32 packet_missed_exception_process(WORD32 nSfIdx, BbuPoolCtrlBlkStruct *pBbu) {
    WORD32 nAtenna;
    WORD32 nBlk;
    WORD32 nExceptionFlag = 0;
    BbuIoBufCtrlStruct *pBbuIo;
    WORD32 nCellIdx;
    WORD32 sf_type;

    /* exception process is done only when nSfIdx is one UL subframe */
    // check one UL subframe processing after 2 subframe
    nSfIdx = (nSfIdx - 2 + MAX_FRAME_NUM) % MAX_FRAME_NUM;
#ifdef TDD
#ifdef ASTRI_TDD_MODE
    nCellIdx=0;
	//fapi_print("[task gen check] - packet_missed_exception_process called nSfIdx = %d \n", nSfIdx);
    if (get_sf_type(pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg, nSfIdx) == DL_SUBFRAME)
#else
	//fapi_print("[task gen check] -- packet_missed_exception_process called nSfIdx = %d \n", nSfIdx);
    if (get_sf_type(nSfIdx) == DL_SUBFRAME)
#endif
        return nExceptionFlag;
#else
    if ((BBU_POOL_CORRECT != is_a_ul_sf(nSfIdx)) && (BBU_POOL_CORRECT != is_a_switch_sf(nSfIdx)))
        return nExceptionFlag;
#endif

    pBbuIo = &(pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nSfIdx % TASK_QUEUE_LEN][0]);
    for (nCellIdx =0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++) {
        switch (pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][nCellIdx]) {
            case CELL_NEW:
            case CELL_NEWING_1:
            case CELL_NEWING_2:
            case CELL_IMMIGRATION_2:
            case CELL_IMMIGRATION_3:
            case CELL_TO_BE_ACTIVE:
            case CELL_ACTIVE:
                /* exception processing for PUSCH */
#ifdef TDD
				sf_type = get_sf_type(pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg, nSfIdx);
#ifdef ASTRI_TDD_MODE
                if ((sf_type == UL_SUBFRAME)
#if ASTRI_UPPTS == 1
                || (sf_type == SPECIAL_SUBFRAME)
#endif
                )
                {
#else
                if (get_sf_type(nSfIdx) == UL_SUBFRAME) {
#endif  /* ASTRI_TDD_MODE */
#endif
                if (nSfIdx != (pBbuIo + nCellIdx)->bValid) {
                    /* all packets are missed */
                    iLog_Warn(BBU_POOL_CONTROL, "all UL packets are missed for cell %d at subframe %d\n",
                        nCellIdx, nSfIdx);
                    /* printf("all UL packets are missed for cell %d at subframe %d\n",
                        nCellIdx, nSfIdx); */
                    nExceptionFlag = 1;
                    (pBbuIo +nCellIdx)->bValid = nSfIdx;
                    //printf("all UL packets are missed for cell %d at subframe %d\n", nCellIdx, nSfIdx);
                    pBbu->sBbuIoIf.eSubframeSkipFlag[nSfIdx % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
                    gen_pusch_task(pBbu, nSfIdx, nCellIdx, 0);
					//fapi_print("[task gen check] case 6: missing UL -- gen_pusch_task nSfTmp = %d, nCellIdx = %d, nCaller = 0\n", nSfIdx, nCellIdx);
                } else {
#if ASTRI_PHY == 1
                	nAtenna = 1 << (pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eRxAntennaPortsCount);
#else
                	nAtenna = 1 << (pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eAntennaPortsCount);
#endif

#if ASTRI_UPPTS == 1
                	if(sf_type == SPECIAL_SUBFRAME)
                	{
                		nBlk = g_UpPTS_len[pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.specSubfCfg] * nAtenna;
                	}
                	else
#endif
                	{
                		nBlk = N_SYMB_PER_SF * nAtenna;
                	}

                    if (nBlk > ((pBbuIo + nCellIdx)->nSegGenerated)) {
                        if ((pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][nCellIdx] == CELL_ACTIVE) ||
                            (pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][nCellIdx] == CELL_TO_BE_ACTIVE))
                        {
                            /* partial packets are missed */
                            iLog_Warn(BBU_POOL_CONTROL, "partial UL packets are missed for cell %d at subframe %d\n",
                                nCellIdx, nSfIdx);
                            nExceptionFlag = 2;
                            /* printf("partial UL packets are missed for cell %d at subframe %d\n",
                                nCellIdx, nSfIdx);  */
                            //printf("partial UL packets are missed for cell %d at subframe %d\n", nCellIdx, nSfIdx);
                            pBbu->sBbuIoIf.eSubframeSkipFlag[nSfIdx % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
                            gen_pusch_task(pBbu, nSfIdx, nCellIdx, 1);
							//fapi_print("[task gen check] case 7: missing UL -- gen_pusch_task nSfTmp = %d, nCellIdx = %d, nCaller = 1\n", nSfIdx, nCellIdx);
                        }
                    } else {
                        /* all packets are received or at least last packet is received. gen_pusch_task was called in
                                            process pusch function*/
                       //fapi_print("[task gen check] all packets are received or at least last packet is received. \n");
                    }
                }
#ifdef TDD
                }
#endif
                //change PRACH task generation to follow PUSCH if FFilter is applied
#if PRACHFFILTER == 0
                /* exception for PRACH, i.e., RACH packet is missed */
#if ASTRI_PHY == 0
                if (BBU_POOL_CORRECT == is_a_rach_sf(nSfIdx))
#endif
                {
                    if (pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nSfIdx % TASK_QUEUE_LEN][nCellIdx].bValid
                         != nSfIdx) {
                        iLog_Warn(BBU_POOL_CONTROL, "all RACH packets are missed for cell %d at subframe %d\n",
                            nCellIdx, nSfIdx);
                        nExceptionFlag = 4;
                        pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfIdx % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
                        gen_prach_task(pBbu, nSfIdx, nCellIdx, 1);
                    }
                }
#endif
                break;
            default:
                break;
        }
    }

    return nExceptionFlag;
}

/*
nCallerFlag 0: timeout
                1: normal
                2: by PUSCH
                3: by PRACH
*/
inline void bbuio_process_timing_pkt_normal(LteRtInforStruct * pLteRt, WORD32 nSfIdx,
        UWORD64 ulTsc, WORD32 nCallerFlag) {
    WORD32 nSfTmp, nSfTmpMod, nCellIdx, nIdx;
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;
    CellActiveEnum *peCSCur;
    UWORD64 uLatestTsc;
    WORD32 nTimeDiff;
    WORD32 nHarqAssocSf[NUM_OF_SUBFRAME_IN_ONE_FRAME], nSfCnt;

	if (g_BbuTimingState == BBU_TIMING_STATE_SYNC && g_IsBbuOutOfSync == 1)
	{
		MESSAGE_LOG("Enter BBU timing recovery state at global sf %d\n", pBbu->nCurrentSfIdx);
		g_BbuTimingState = BBU_TIMING_STATE_UNSYNC;
		g_IsBbuUnderReset = 1;
	}

	// ignore all update timing request during timing recovery
	if (g_BbuTimingState != BBU_TIMING_STATE_SYNC && nCallerFlag != 1) 
	{
		return;
	}

	if (g_BbuTimingState == BBU_TIMING_STATE_UNSYNC)
	{
		if ((nSfIdx % TASK_QUEUE_LEN) != (pBbu->nCurrentSfIdx + 1) % TASK_QUEUE_LEN)
		{
			g_BbuResetCount = 0; //reset counter repeatively if timing packet not aligned with internal timing
			return;
		}

		if (g_IsBbuOutOfSync == 1)
		{
			g_IsBbuOutOfSync = 0;
			g_BbuResetCount = 0; //reset counter repeatively if we receive more bbu out of sync signal during recovery
		}

		// after we receive no more out of sycn singal and reset enough time, try to realign with rru timing
		if (g_BbuResetCount >= NUM_BBU_RESET_COUNT) 
		{
			g_BbuTimingState = BBU_TIMING_STATE_LOCK;
			g_BbuReSyncCount = 0;
			pBbu->nCurrentSfIdx = (nSfIdx + MAX_FRAME_NUM - 1) % MAX_FRAME_NUM;
		}
	}
	else if (g_BbuTimingState == BBU_TIMING_STATE_LOCK)
	{
		if (nSfIdx == (pBbu->nCurrentSfIdx + 1) % MAX_FRAME_NUM)
		{
			g_BbuReSyncCount++;			
		}
		else
		{
			// lost track again
			g_BbuTimingState = BBU_TIMING_STATE_UNSYNC;
			g_IsBbuOutOfSync = 0;
			g_BbuResetCount = 0;			
		}

		if (g_BbuReSyncCount >= NUM_BBU_RESYNC_COUNT)
		{
			MESSAGE_LOG("BBU timing recovery state finished at global sf %d\n", pBbu->nCurrentSfIdx);
			g_BbuTimingState = BBU_TIMING_STATE_SYNC;
			g_IsBbuUnderReset = 0;
		}
	}

    switch (nCallerFlag) {
        case 0:
            iLog_Normal(BBU_POOL_CONTROL, "Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by timeout\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            MESSAGE_LOG("Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by timeout\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            break;
        case 1:
            iLog_Normal(BBU_POOL_CONTROL, "Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by normal\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            break;
        case 2:
            if ((ulTsc - uPrevTsc) < (TIME_INTERVAL * 1000 * CPU_HZ))  {
				MESSAGE_LOG("Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by pusch (Ignored)\n",
					nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
                /* timing cannot be updated as time is not past enough */
                return;
            }
            MESSAGE_LOG("Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by pusch\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            iLog_Normal(BBU_POOL_CONTROL, "Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by pusch\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            break;
        case 3:
            if ((ulTsc - uPrevTsc) < (TIME_INTERVAL * 1000 * CPU_HZ))  {
                /* timing cannot be updated as time is not past enough */
                return;
            }
            MESSAGE_LOG("Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by prach\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            iLog_Normal(BBU_POOL_CONTROL, "Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by prach\n",
                nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx);
            break;
        default:
            iLog_Normal(BBU_POOL_CONTROL, "Timing processing nSfIdx=%d Tsc(us)= %llu at global sf %d called by %d\n",
                    nSfIdx, ulTsc/CPU_HZ, pBbu->nCurrentSfIdx, nCallerFlag);
            return;
            break;
    }

	if (g_BbuTimingState == BBU_TIMING_STATE_UNSYNC)
	{
		nSfIdx = (pBbu->nCurrentSfIdx + 1) % MAX_FRAME_NUM;
		g_BbuResetCount++;		
	}
	else
	{
		nSfIdx = nSfIdx % MAX_FRAME_NUM;
	}

	fapi_print("======================== Timing packet received, On-Air subframe nSfIdx = %d ========================\n", nSfIdx);

	//Timing packet received too late
	//if ((ulTsc - uPrevTsc) > (TIME_INTERVAL * 1100 * CPU_HZ))
		//MESSAGE_LOG(">>>>>> Timing packet for subframe %d received with a delay over 10 percent of a subframe !!!!\n", nSfIdx);

    nSfTmp = (pBbu->nCurrentSfIdx + 1) % MAX_FRAME_NUM;
    nTimeDiff = astri_subframe_compare(nSfIdx,  nSfTmp);//subframe_compare(nSfIdx,  nSfTmp, 3);
    uLatestTsc = rte_rdtsc();
    //if (abs(nTimeDiff) > 3) {
    if (abs(nTimeDiff) > 10) {
        /* even 2 succeed timing packets are missed, the code will not run */
        iLog_Warn(BBU_POOL_CONTROL, "An ambiguous subframe index %d is received at TSC(us) %llu, global sfidx=%d\n",
                nSfIdx, uLatestTsc / CPU_HZ, pBbu->nCurrentSfIdx);
        MESSAGE_LOG("An ambiguous subframe index %d is received at TSC(us) %llu, global sfidx=%d\n",
                nSfIdx, uLatestTsc / CPU_HZ, pBbu->nCurrentSfIdx);
        return;
    }
    if (nTimeDiff < 0) {
        iLog_Warn(BBU_POOL_CONTROL, "%d sf timing packet is repeated at TSC(us) %llu when global subframe=%d\n",
            nSfIdx, uLatestTsc / CPU_HZ, pBbu->nCurrentSfIdx % MAX_FRAME_NUM);
        MESSAGE_LOG("%d sf timing packet is repeated at TSC(us) %llu when global subframe=%d\n",
            nSfIdx, uLatestTsc / CPU_HZ, pBbu->nCurrentSfIdx % MAX_FRAME_NUM);
        return;
    }
    if (nTimeDiff > 0) {
        iLog_Warn(BBU_POOL_CONTROL, "%d sf timing packet is missed at TSC(us) %llu, received sfidx=%d\n",
                nSfTmp, uLatestTsc / CPU_HZ, nSfIdx);
        MESSAGE_LOG("%d sf timing packet is missed at TSC(us) %llu, received sfidx=%d\n",
                nSfTmp, uLatestTsc / CPU_HZ, nSfIdx);
    }

    if (1 != nCallerFlag)
    {
        MESSAGE_LOG("update sfidx by %d to %d from %d at TSC(us) %llu with prev Tsc(us)= %llu\n",
            nCallerFlag, nSfIdx, pBbu->nCurrentSfIdx, uLatestTsc / CPU_HZ, uPrevTsc/CPU_HZ);
    }
    //rte_mempool_dump(pBbu->sBbuIoIf.bbuio_buf_pool);
    do {
        pBbu->nCurrentSfIdx = nSfTmp;
        pBbu->sBbuIoIf.nTscTiming[nSfTmp % N_FE_BUF_LEN] = ulTsc;
        uPrevTsc = rte_rdtsc();
		nIsSubframeIndSent = 0;
        nSfTmpMod = nSfTmp % TASK_QUEUE_LEN;
        iLog_Debug(BBU_POOL_CONTROL, "Processing Timing at subframe %d  cell 0 state=%d cell 3 state=%d\n ",
            nSfTmp, pBbu->eCellActive[nSfTmpMod][0], pBbu->eCellActive[nSfTmpMod][N_MAX_CELL_PER_BBU]);

        update_cell_status(pBbu,(nSfTmp + 1) % MAX_FRAME_NUM);
        //timing_update(nSfTmp2, ulTsc, pBbu);

        /* check if one cell's UL data packet is not received.
                * if previous subframes are missed or not processed, generate exception first so that
                    other threads can process those subframe immediately.
                */
#ifdef L1_L2_DECOUPLE
		packet_missed_exception_process_dl(nSfTmp, pBbu);
#endif                
        packet_missed_exception_process(nSfTmp, pBbu);

#if PROFILE_LV >= 2
        for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++)
        {
            peCSCur = &(pBbu->eCellActive[nSfTmpMod][nCellIdx]);

            switch (pBbu->eCellActive[nSfTmpMod][N_MAX_CELL_PER_BBU])
            {
                case CELL_ACTIVE:
                    ProfileCntInc(nCellIdx, nSfTmp);
                	break;

                default:
                	break;
            }
        }
#endif
		// For dummy cell index N_MAX_CELL_PER_BBU, generate uplink tasks immediately after timing updated
        if (BBU_POOL_CORRECT == is_a_ul_sf(nSfTmp)) {
            nSfTmpMod = nSfTmp % TASK_QUEUE_LEN;
            /* generate dummy job */
            for (nCellIdx = 0; nCellIdx <= N_MAX_CELL_PER_BBU; nCellIdx ++) {
                peCSCur = &(pBbu->eCellActive[nSfTmpMod][nCellIdx]);
                switch (*peCSCur) {
                case CELL_NEW:
                case CELL_NEWING_1:
                case CELL_NEWING_2:
                case CELL_IMMIGRATION_2:
                case CELL_IMMIGRATION_3:
            iLog_Debug(BBU_POOL_CONTROL, "generate dummy ul subframe %d for cell %d at state %d at global sf=%d\n",
                nSfTmp, nCellIdx, *peCSCur, pBbu->nCurrentSfIdx);
                    gen_pusch_task(pBbu, nSfTmp,nCellIdx, 2);

                    //change PRACH task generation to follow PUSCH if FFilter is applied
#if PRACHFFILTER == 0
                    //do PRACH check inside the task later in order to ensure the FAPI PRACH flags are received
#if ASTRI_PHY == 0
                    if (BBU_POOL_CORRECT == is_a_rach_sf(nSfTmp))
#endif
                    {
                        gen_prach_task(pBbu, nSfTmp,nCellIdx, 2);
                    }
#endif
                    break;
                default:
                    break;
                }
            }

            switch (pBbu->eCellActive[nSfTmpMod][N_MAX_CELL_PER_BBU]) {
                case CELL_ACTIVE:
                case CELL_TO_BE_ACTIVE:
                    gen_pusch_task(pBbu, nSfTmp, N_MAX_CELL_PER_BBU, 3);

                    //change PRACH task generation to follow PUSCH if FFilter is applied
#if PRACHFFILTER == 0
                    //do PRACH check inside the task later in order to ensure the FAPI PRACH flags are received
#if ASTRI_PHY == 0
                    if (BBU_POOL_CORRECT == is_a_rach_sf(nSfTmp))
#endif
                    {
                        gen_prach_task(pBbu, nSfTmp, N_MAX_CELL_PER_BBU, 3);
                    }
#endif
                    break;
                default:
                    break;
            }
        }

		// For normal cell index 0 to (N_MAX_CELL_PER_BBU - 1), DL subframe, generate uplink tasks immediately as well
#ifdef TDD
#ifdef  ASTRI_TDD_MODE
	    nCellIdx=0;
	    if(get_sf_type(pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg, nSfTmp) != UL_SUBFRAME && get_sf_type(pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg, nSfTmp) != SPECIAL_SUBFRAME) {
#else
#if ASTRI_PHY == 1
		if(get_sf_type(nSfTmp) != UL_SUBFRAME && get_sf_type(nSfTmp) != SPECIAL_SUBFRAME) {
#else
		if(get_sf_type(nSfTmp) != UL_SUBFRAME) {
#endif
#endif
	         for (nCellIdx =0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++) {
	            switch (pBbu->eCellActive[nSfTmp % TASK_QUEUE_LEN][nCellIdx]) {
	                case CELL_TO_BE_ACTIVE:
	                case CELL_ACTIVE:
	                    pBbu->sBbuIoIf.eSubframeSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
	                    gen_pusch_task(pBbu, nSfTmp, nCellIdx, 6);

#if ASTRI_PHY == 1
	                    //change PRACH task generation to follow PUSCH if FFilter is applied
#if PRACHFFILTER == 0
	                    if(get_sf_type(pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.subfCfg, nSfTmp) != SPECIAL_SUBFRAME)
	                    {
	                    	pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
	                    	gen_prach_task(pBbu, nSfTmp, nCellIdx, 6);
	                    }
#endif
#endif
	                    break;
	                default:
	                    break;
	            }
	        }
	    }
#endif
		// For normal cell index 0 to (N_MAX_CELL_PER_BBU - 1), UL subframe, uplink tasks will be generated when: 
		// i) last symbol UL packet received, at bbuio_process_pusch_pkt(), or
		// ii) 2 subframe after time expired, at packet_missed_exception_process()

		// Set task skip flag during recovery
		if (g_BbuTimingState == BBU_TIMING_STATE_UNSYNC || g_BbuTimingState == BBU_TIMING_STATE_LOCK)
		{
			pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
			pBbu->sBbuIoIf.eSubframeSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
			nSfCnt = get_harq_associated_dl_sf(nSfTmp, nHarqAssocSf);
			for (nIdx = 0; nIdx < nSfCnt; nIdx ++) {
				pBbu->sBbuIoIf.eDlTxSkipFlag[nHarqAssocSf[nIdx] % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
			}
		}
		else
		{
			switch (nCallerFlag) {
				case 0:
					pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
					pBbu->sBbuIoIf.eSubframeSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
					nSfCnt = get_harq_associated_dl_sf(nSfTmp, nHarqAssocSf);
					for (nIdx = 0; nIdx < nSfCnt; nIdx ++) {
						pBbu->sBbuIoIf.eDlTxSkipFlag[nHarqAssocSf[nIdx] % N_SKIP_FLAG_LEN] = TASK_ALL_SKIP;
					}
					break;
				case 2:
				case 3:
					/* timing packet is not received, then start cell status too late, only do mandotary work */
					pBbu->sBbuIoIf.eSubframeSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_UL_SKIP;
#if ASTRI_PHY == 1
					pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_UL_SKIP;
#endif
					nSfCnt = get_harq_associated_dl_sf(nSfTmp, nHarqAssocSf);
					for (nIdx = 0; nIdx < nSfCnt; nIdx ++) {
						pBbu->sBbuIoIf.eDlTxSkipFlag[nHarqAssocSf[nIdx] % N_SKIP_FLAG_LEN] = TASK_UL_SKIP;
					}
					break;
				case 1:
				default:
					pBbu->sBbuIoIf.eSubframeSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_NO_SKIP;
					pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfTmp % N_SKIP_FLAG_LEN] = TASK_NO_SKIP;
					break;
			}
		}

        nSfTmp  = (nSfTmp + 1) % MAX_FRAME_NUM;
        if (unlikely(nTimeDiff > 0)) {
            nTimeDiff --;
            usleep(100); // reserve time for LTE worker thread do exception processing
        }
        if (unlikely(g_BbuTimingState == BBU_TIMING_STATE_UNSYNC)) {
            usleep(100); // reserve time for LTE worker thread do exception processing
        }		
    }while (subframe_compare(nSfTmp, nSfIdx, 4) <= 0);
}

void inline bbuio_process_timing_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt)
{
    UWORD64 ulTsc = rte_rdtsc();
    WORD32 nSfIdx = 0;
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;

#if !defined TD_CONNECT && !defined FB_API
    struct timing_hdr_s *pTiming = (struct timing_hdr_s *)((WORD8 *)pkt->pkt.data + ETH_HEADER_SIZE);
    // generate latest global subframe index
    nSfIdx = ntohs(pTiming->frame_seq) * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subf_seq;
#endif
#if defined TD_CONNECT || defined FB_API
	RpeTimingPktPayload *pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(pkt, 0, 0, 0))->message);
    nSfIdx = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
#endif
    rte_pktmbuf_free(pkt);

#ifdef FE_INTEGRATION
    WORD32 nRfIdx = 0;
#ifndef TD_CONNECT
    nRfIdx = ntohs(pTiming->frame_seq);
#endif
#ifdef TD_CONNECT
    nRfIdx = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)) % NUM_OF_MAX_FRAME;
#endif
    if ((nRfIdx & 0xC00) == 0xC00){
        g_nFlagStart = 1;
        //printf("Timing packet %d is processed\n", nSfIdx);
    }
#endif
    nSfIdx = nSfIdx % MAX_FRAME_NUM;
    static WORD32 nLastSfIdx = -1;
    if (nLastSfIdx >= 0 && g_BbuTimingState == BBU_TIMING_STATE_SYNC)
    {
      if (((nLastSfIdx + 1) % MAX_FRAME_NUM) != nSfIdx && ((pBbu->nCurrentSfIdx + 1) % MAX_FRAME_NUM) != nSfIdx)
      {
        MESSAGE_LOG("*** sudden change of timing index detected!! nSfIdx = %d, nLastSfIdx = %d, Tsc(us)= %llu ***\n ", nSfIdx, nLastSfIdx, ulTsc/CPU_HZ);
		g_IsBbuOutOfSync = 1;
      }
    }
    nLastSfIdx = nSfIdx;
    
    pBbu->nTimePacketArrivalTime[nSfIdx % 10] = ulTsc;
    bbuio_process_timing_pkt_normal(pLteRt, nSfIdx, ulTsc, 1);

#if (TIME_PROFILE_MODE == 1)
	TimeProfileRecord( 19,				//TIME_PACKET
					   0,				//TIME OUT CHECK
					   0,				//Cell Index
					   nSfIdx,			//WORKING SFN Index
					   0,				//CHECKING SFN Index
					   0,				//Duration
					   ulTsc / CPU_HZ,	//Start time
					   0,				//Finish time
					   pLteRt->nCoreId); //Core ID
#endif

	// move gen_pusch_task()/gen_prach_task() by caller 6 code inside bbuio_process_timing_pkt_normal() since it can handle multiple steps of subframe increment

};

void inline bbuio_process_prach_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx)
{
    UWORD64 ulTsc = rte_rdtsc();
#ifndef FB_API	
    struct prach_hdr_s *prach_hdr;
	prach_hdr = (struct prach_hdr_s *)((WORD8 *)pkt->pkt.data + ETH_HEADER_SIZE);
#endif //not FB_API
#ifdef FB_API
	RpeLtePrachPkt *prach_hdr;
	prach_hdr = (RpeLtePrachPkt *) RTE_PKTMBUF_READ(pkt, 0, 0, 0);
#endif
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;

#if ASTRI_PHY == 1
    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eRxAntennaPortsCount);
#else
    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eAntennaPortsCount);
#endif
#ifdef _ANT8
    nAtenna = g_config_data->UL_nRx;
#endif

#ifndef FB_API
	uint16_t nFrame = (ntohs(prach_hdr->frame_seq))&0x3FF;
	uint8_t nSubF = prach_hdr->subf_seq;
#endif
#ifdef FB_API
#if ASTRI_PHY == 1
	uint16_t nFrame = (ntohs(prach_hdr->ltePrachHdr.frame_seq))&0x3FF;
	uint8_t nSubF = prach_hdr->ltePrachHdr.subf_seq;
    uint8_t nSym = 0; //prach_hdr->ltePrachHdr.sym_seq; //This field may be used for multi-PRACH resource in the future
    uint8_t nBlkStart = prach_hdr->ltePrachHdr.blk_start;
#else
	uint16_t nFrame = (ntohs(prach_hdr->ltePrachConfiguration.rfNum)) & 0x3FF;
	uint8_t nSubF = prach_hdr->ltePrachConfiguration.sfNum;
#endif
#endif
    uint8_t nBlkCnt;
    uint8_t nBufSlot = (nFrame%4)*10+nSubF;
    uint8_t nBufOffset = 0;
    WORD32 nDataReadyFlag, nRcveBlock;
    WORD32 nSfIdx = nFrame * 10 + nSubF, nSfDiff;
    CellActiveEnum eCellStatus;

#if defined(FB_API) && ASTRI_PHY == 1
    prach_hdr->ltePrachHdr.blk_cnt &= 0x3;
    nBlkCnt = prach_hdr->ltePrachHdr.blk_cnt + 1;
#endif

    iLog_Debug(BBU_POOL_CONTROL, "into Processing PRACH Packet at nSfIdx=%d with global sf %d\n",
    nSfIdx, pBbu->nCurrentSfIdx);

    /* check if this subframe timing packet is received */
    //nSfDiff = subframe_compare(nSfIdx, pBbu->nCurrentSfIdx, 5);

    /* check if this packet is a delayed too much */
	/*
    if (abs(nSfDiff) > 2) {
        rte_pktmbuf_free(pkt);
        return;
    }
    */

	nSfDiff = astri_subframe_compare(nSfIdx, pBbu->nCurrentSfIdx);
    if (nSfDiff < -1 || // free all expired packets
		nSfDiff > 10)  // the allowed recovery limit have to large enough to across DL subframe
    {
    	MESSAGE_LOG("bbuio_process_prach_pkt: cIdx = %d, nSfIdx = %d, pBbu->nCurrentSfIdx = %d, nSfDiff = %d, free packet\n", cIdx, nSfIdx, pBbu->nCurrentSfIdx, nSfDiff);
        rte_pktmbuf_free(pkt);
        return;
   	}

    if (nSfDiff > 0) {
        MESSAGE_LOG("bbuio_process_prach_pkt: cIdx = %d, nSfIdx = %d, nSfDiff = %d, update timing\n", cIdx, nSfIdx, nSfDiff);
         bbuio_process_timing_pkt_normal(pLteRt, nSfIdx, ulTsc, 3);
    }
	

    if (pBbu->sBbuIoIf.eSubframeRachSkipFlag[nSfIdx % N_SKIP_FLAG_LEN] != TASK_NO_SKIP) {
        rte_pktmbuf_free(pkt);
        return;
    }

    eCellStatus = pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][cIdx];
    switch (eCellStatus) {
    //discard PRACH packet from RRU when FFilter is enabled
#if PRACHFFILTER == 0
        case CELL_TO_BE_ACTIVE:
        case CELL_ACTIVE:
            iLog_Debug(BBU_POOL_CONTROL, "Processing PRACH Packet at subframe %d cell %d state=%d \n ",
                nSfIdx, cIdx, pBbu->eCellActive[nSfIdx%TASK_QUEUE_LEN][cIdx]);

            nBufOffset = (pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegTransferred) ++;
#if defined(FB_API) && ASTRI_PHY == 1
            pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated += nBlkCnt;
#else
            pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated += 2;//1;
#endif
            (*pBbu).sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].pData[nBufOffset] = pkt;
            pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].bValid= nSfIdx;

#if defined(FB_API) && ASTRI_PHY == 1
            nDataReadyFlag = 0;
			if ((1 * nAtenna) ==
				pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated)
			{
				nDataReadyFlag = 1;

#if 0
				task_history_record(rte_rdtsc() -
					pBbu->sBbuIoIf.nTscTiming[nSfIdx % N_FE_BUF_LEN],
					nSfIdx % NUM_OF_SUBFRAME_IN_ONE_FRAME, cIdx,
					TASK_STAT_LAST_UL_PKT_TIMING); //invalid enum for PRACH
#endif

			}
			else
			{
				/*
				 * 20161115, astri_walter
				 * Below packet loss handling is imported from PUSCH packet handling.
				 * Btw, it is not suitable for PRACH, as packet loss in PUSCH affect the data plane only,
				 * but packet loss in PRACH may cause error in control plane.
				 * so I recommend it is preferred to send all the antenna data within one packet to prevetn from such error.
				 */

				if ((0 == nSym)&&(nAtenna == (nBlkStart+nBlkCnt)))
				{
					nRcveBlock = pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated;
					pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated =
						1 * nAtenna;
					nDataReadyFlag = 2;
				}
			}

			switch (nDataReadyFlag)
			{
				case 1:
		            iLog_Normal(BBU_POOL_CONTROL, "PRACH Frame =%d, SubFrame= %d are received\n",\
		                nFrame,nSubF);
		            gen_prach_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 4);
					break;
				case 2:
					iLog_Normal(BBU_POOL_CONTROL, "PRACH Frame=%d, SubFrame= %d are partially received %d\n",\
						nFrame,nSubF, nRcveBlock);
					gen_prach_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 5);
					break;
				default:
					break;
			}
#else // #if defined(FB_API) && ASTRI_PHY == 1

            iLog_Normal(BBU_POOL_CONTROL, "PRACH Frame =%d, SubFrame= %d are received\n",\
                                    nFrame,nSubF);

//#ifndef TD_CONNECT
#ifndef FE_INTEGRATION
//            gen_prach_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 4);
#endif

//#else
            if(nAtenna == pBbu->sBbuIoIf.sRachBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated)
            {
                gen_prach_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 4);
            }
//#endif
#endif //#if defined(FB_API) && ASTRI_PHY == 1

            break;
#endif // #if PRACHFFILTER == 0
        default:
            iLog_Debug(BBU_POOL_CONTROL, "Discard PRACH Packet at subframe %d cell %d state=%d \n ",
                nSfIdx, cIdx, pBbu->eCellActive[nSfIdx%TASK_QUEUE_LEN][cIdx]);
            rte_pktmbuf_free(pkt);
            break;
    }
    return;
}

void inline bbuio_process_pusch_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx)
{
    UWORD64 ulTsc = rte_rdtsc();
#ifndef FB_API
    struct pusch_hdr_s *pusch_hdr;
    pusch_hdr = (struct pusch_hdr_s *)((WORD8 *)pkt->pkt.data + ETH_HEADER_SIZE);
#endif
#ifdef FB_API
    RpeLtePuschPkt *pusch_hdr;
	pusch_hdr = (RpeLtePuschPkt *) RTE_PKTMBUF_READ(pkt, 0, 0, 0);
#endif
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;

/*
    if (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.eSystemBandwidth == B10M) {
		// oversampling pusch_hdr to exact sampling sample locations
		char *pTmp = (char *)((void *)pusch_hdr + sizeof(RpeLtePuschPkt));
		char tmpBuf[600*2*2];
		memcpy(&tmpBuf[0],&pTmp[300*2],600*2);
		memcpy(&tmpBuf[600*2],&pTmp[1200*2+300*2],600*2);
		// relocate the input buffer location
		memset(&pTmp[0], 0, 1200*2*2);
		memcpy(&pTmp[0], &tmpBuf[0],600*2);
		memcpy(&pTmp[1200*2], &tmpBuf[600*2],600*2);
    }
*/

#if ASTRI_PHY == 1
    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eRxAntennaPortsCount);
#else
    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eAntennaPortsCount);
#endif

#ifdef _ANT8
    nAtenna = g_config_data->UL_nRx;
#endif

#ifndef FB_API
    uint16_t nFrame = (ntohs(pusch_hdr->frame_seq))&0x3FF;
    uint8_t nSubF = pusch_hdr->subf_seq;
    uint8_t nSym = pusch_hdr->sym_seq;
    uint8_t nBlkStart = pusch_hdr->blk_start;
#endif
#ifdef FB_API
    uint16_t nFrame = (ntohs(pusch_hdr->ltePuschHdr.frame_seq))&0x3FF;
    uint8_t nSubF = pusch_hdr->ltePuschHdr.subf_seq;
    uint8_t nSym = pusch_hdr->ltePuschHdr.sym_seq;
    uint8_t nBlkStart = pusch_hdr->ltePuschHdr.blk_start;
#endif
    uint8_t nBlkCnt;
    uint8_t nBufSlot = (nFrame%4)*10+nSubF;
    uint8_t nBufOffset = 0;
    WORD32 nDataReadyFlag, nRcveBlock;
    WORD32 nSfIdx = nFrame * 10 + nSubF, nSfDiff;
    CellActiveEnum eCellStatus;
#if ASTRI_UPPTS == 1
    WORD32 sf_type;
#endif
    uint8_t nSymUl_max;

#ifdef FE_INTEGRATION
    /* due to RTL code only support 8 antenna and 1/2/4 antenna cases are hacked to support,
       need to limit cnt range to 4 to correct support 1/2/4 antenna */
    pusch_hdr->blk_cnt &= 0x3;
#endif

#ifndef FB_API
    nBlkCnt = pusch_hdr->blk_cnt + 1;
#endif
#ifdef FB_API
    pusch_hdr->ltePuschHdr.blk_cnt &= 0x3;
    nBlkCnt = pusch_hdr->ltePuschHdr.blk_cnt + 1;
#endif

    iLog_Debug(BBU_POOL_CONTROL, "into Processing PUSCH Packet at nSfIdx=%d with global sf %d\n",
    nSfIdx, pBbu->nCurrentSfIdx);

    /* check if this subframe timing packet is received */
    //nSfDiff = subframe_compare(nSfIdx, pBbu->nCurrentSfIdx, 5);
    /* check if this packet is a delayed too much */
	/*
    if (abs(nSfDiff) > 2) {
        rte_pktmbuf_free(pkt);
        return;
    }
    */


#if (TIME_PROFILE_MODE == 1)
	TimeProfileRecord( 20,				//LAST_PUSCH_PACKET
					   0,				//TIME OUT CHECK
					   cIdx,			//Cell Index
					   nSfIdx,			//WORKING SFN Index
					   0,				//CHECKING SFN Index
					   0,				//Duration
					   ulTsc / CPU_HZ,	//Start time
					   0,				//Finish time
					   nSym); //Core ID
#endif	

	nSfDiff = astri_subframe_compare(nSfIdx, pBbu->nCurrentSfIdx);
    if (nSfDiff < -1 || // free all expired packets
		nSfDiff > 10)  // the allowed recovery limit have to large enough to across DL subframe
    {
    	MESSAGE_LOG("bbuio_process_pusch_pkt: cIdx = %d, nSfIdx = %d, pBbu->nCurrentSfIdx = %d, nSfDiff = %d, free packet\n", cIdx, nSfIdx, pBbu->nCurrentSfIdx, nSfDiff);
        rte_pktmbuf_free(pkt);
        return;
   	}

    if (nSfDiff > 0) {
         MESSAGE_LOG("bbuio_process_pusch_pkt: cIdx = %d, nSfIdx = %d, pBbu->nCurrentSfIdx = %d, nSfDiff = %d, update timing\n", cIdx, nSfIdx, pBbu->nCurrentSfIdx, nSfDiff);
         bbuio_process_timing_pkt_normal(pLteRt, nSfIdx, ulTsc, 2);
    }

    if (pBbu->sBbuIoIf.eSubframeSkipFlag[nSfIdx % N_SKIP_FLAG_LEN] != TASK_NO_SKIP) {
        rte_pktmbuf_free(pkt);
        return;
    }
    
#if ASTRI_UPPTS == 1
    sf_type = get_sf_type(pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.subfCfg, nSfIdx);
    if (sf_type == SPECIAL_SUBFRAME)
    {
    	nSymUl_max = g_UpPTS_len[pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.specSubfCfg];
    }
    else
#endif
    {
    	nSymUl_max = N_SYMB_PER_SF;
    }
    
#if ASTRI_UPPTS == 1
    uint8_t firstSymIdx = N_SYMB_PER_SF - nSymUl_max;
    if (sf_type == SPECIAL_SUBFRAME && nSym < firstSymIdx) // ignore DwPTS and GP packets
    {
        rte_pktmbuf_free(pkt);
        return;
    }
#endif        

    eCellStatus = pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][cIdx];
    switch (eCellStatus) {
        case CELL_TO_BE_ACTIVE:
        case CELL_ACTIVE:
            iLog_Debug(BBU_POOL_CONTROL, "Processing PUSCH Packet at subframe %d cell %d state=%d \n ",
                nSfIdx, cIdx, pBbu->eCellActive[nSfIdx%TASK_QUEUE_LEN][cIdx]);
            nBufOffset = (pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegTransferred) ++;
            pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated += nBlkCnt;
            (*pBbu).sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].pData[nBufOffset] = pkt;
            pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].bValid= nSfIdx;
			/*
#if (TIME_PROFILE_MODE == 1)
				TimeProfileRecord( 20,				//LAST_PUSCH_PACKET
								   0,				//TIME OUT CHECK
								   cIdx,			//Cell Index
								   nSfIdx,			//WORKING SFN Index
								   0,				//CHECKING SFN Index
								   0,				//Duration
								   ulTsc / CPU_HZ,	//Start time
								   0,				//Finish time
								   nSym); //Core ID
#endif
*/

			//Check whether last Pusch symbol arrived to late (Arrived after half of the subframe)
			//if ((ulTsc - uPrevTsc) > ((TIME_INTERVAL * 500) * CPU_HZ) &&  nSym == 13)
                //MESSAGE_LOG(">>>>>> PUSCH Frame=%d, SubFrame= %d, nSym = %d, received with a delay over 50 percent of a subframe !!!\n", nFrame,nSubF,nSym);


            nDataReadyFlag = 0;

            if ((nSymUl_max * nAtenna) ==
                pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated) {
                nDataReadyFlag = 1;
                task_history_record(rte_rdtsc() -
                    pBbu->sBbuIoIf.nTscTiming[nSfIdx % N_FE_BUF_LEN],
                    nSfIdx % NUM_OF_SUBFRAME_IN_ONE_FRAME, cIdx,
                    TASK_STAT_LAST_UL_PKT_TIMING);

            } else {
            
                if ((13 == nSym)&&(nAtenna == (nBlkStart+nBlkCnt)))  {
                    nRcveBlock = pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated;
                    pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated =
                    		nSymUl_max * nAtenna;
                    nDataReadyFlag = 2;
                }
            }
            switch (nDataReadyFlag) {
                case 1:
                    iLog_Normal(BBU_POOL_CONTROL, "PUSCH Frame =%d, SubFrame= %d are received\n",\
                        nFrame,nSubF);
                    gen_pusch_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 4);
                    break;
                case 2:
                    iLog_Normal(BBU_POOL_CONTROL, "PUSCH Frame=%d, SubFrame= %d are partially received %d\n",\
                        nFrame,nSubF, nRcveBlock);
                    MESSAGE_LOG("PUSCH Frame=%d, SubFrame= %d are partially received %d, nSymUl_max = %d, nAtenna = %d, nSym = %d, nBlkStart = %d, nBlkCnt = %d, nSegGenerated = %d, nDataReadyFlag = %d\n",\
                        nFrame,nSubF, nRcveBlock, nSymUl_max, nAtenna, nSym, nBlkStart, nBlkCnt, pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated, nDataReadyFlag);
                    gen_pusch_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 5);
                    break;
                default:
                    break;
            }
            break;
        default:
            iLog_Debug(BBU_POOL_CONTROL, "Discard PUSCH Packet at subframe %d cell %d state=%d \n ",
                nSfIdx, cIdx, pBbu->eCellActive[nSfIdx%TASK_QUEUE_LEN][cIdx]);
            rte_pktmbuf_free(pkt);
            break;
    }
    return;
}

#ifdef TD_CONNECT
void inline bbuio_process_td_ul_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx)
{
    UWORD64 ulTsc = rte_rdtsc();
    RpeLteTimeDomainFrameStatus *pusch_hdr;
    pusch_hdr = &(((RpeLteTimeDomainPkt *)((WORD8 *)pkt->pkt.data))->lteTimeDomainFrameStatus);
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;

#if ASTRI_PHY == 1
    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eRxAntennaPortsCount);
#else
    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eAntennaPortsCount);
#endif

#ifdef _ANT8
    nAtenna = g_config_data->UL_nRx;
#endif
    uint16_t nFrame = ((pusch_hdr->rfNumH) * RPE_RADIO_FRAME_LOW_MAX_NUM + pusch_hdr->rfNumL) % NUM_OF_MAX_FRAME;
    uint8_t nSubF = pusch_hdr->sfNum;
    uint8_t nSymIdx = pusch_hdr->symbol;
    uint8_t nAntIdx = pusch_hdr->antNum;
    uint8_t nBufSlot = (nFrame%4)*10+nSubF;
    uint8_t nBufOffset = 0;
    WORD32 nDataReadyFlag, nRcveBlock;
    WORD32 nSfIdx = nFrame * 10 + nSubF, nSfDiff;
    CellActiveEnum eCellStatus;

    iLog_Debug(BBU_POOL_CONTROL, "into Processing PUSCH Packet at nSfIdx=%d with global sf %d\n",
    nSfIdx, pBbu->nCurrentSfIdx);

    /* check if this subframe timing packet is received */
    nSfDiff = subframe_compare(nSfIdx, pBbu->nCurrentSfIdx, 5);
    /* check if this packet is a delayed too much */
    if (abs(nSfDiff) > 2) {
        rte_pktmbuf_free(pkt);
        return;
    }

    if (nSfDiff > 0) {
         bbuio_process_timing_pkt_normal(pLteRt, nSfIdx, ulTsc, 2);
    }

    if (pBbu->sBbuIoIf.eSubframeSkipFlag[nSfIdx % N_SKIP_FLAG_LEN] != TASK_NO_SKIP) {
        rte_pktmbuf_free(pkt);
        return;
    }

    eCellStatus = pBbu->eCellActive[nSfIdx % TASK_QUEUE_LEN][cIdx];
    switch (eCellStatus) {
        case CELL_TO_BE_ACTIVE:
        case CELL_ACTIVE:
            iLog_Debug(BBU_POOL_CONTROL, "Processing PUSCH Packet at subframe %d cell %d state=%d \n ",
                nSfIdx, cIdx, pBbu->eCellActive[nSfIdx%TASK_QUEUE_LEN][cIdx]);
            nBufOffset = (pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegTransferred) ++;
            pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated ++;
            (*pBbu).sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].pData[nBufOffset] = pkt;
            pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].bValid= nSfIdx;

            nDataReadyFlag = 0;
            if ((N_SYMB_PER_SF * nAtenna) ==
                pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated) {
                nDataReadyFlag = 1;
            } else {
                if ((13 == nSymIdx)&&(nAtenna == nAntIdx))  {
                    nRcveBlock = pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated;
                    pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated =
                        N_SYMB_PER_SF * nAtenna;
                    nDataReadyFlag = 2;
                }
            }
            switch (nDataReadyFlag) {
                case 1:
                    iLog_Normal(BBU_POOL_CONTROL, "PUSCH Frame =%d, SubFrame= %d are received\n",\
                        nFrame,nSubF);
                    gen_pusch_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 4);
                    break;
                case 2:
                    iLog_Normal(BBU_POOL_CONTROL, "PUSCH Frame=%d, SubFrame= %d are partially received %d\n",\
                        nFrame,nSubF, nRcveBlock);
                    gen_pusch_task(pBbu, nFrame * NUM_OF_SUBFRAME_IN_ONE_FRAME + nSubF, cIdx, 5);
                    break;
                default:
                    break;
            }
            break;
        default:
            iLog_Debug(BBU_POOL_CONTROL, "Discard PUSCH Packet at subframe %d cell %d state=%d \n ",
                nSfIdx, cIdx, pBbu->eCellActive[nSfIdx%TASK_QUEUE_LEN][cIdx]);
            rte_pktmbuf_free(pkt);
            break;
    }
    return;
}
#endif

