void inline bbuio_process_pusch_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt, WORD32 cIdx)
{
    UWORD64 ulTsc = rte_rdtsc();

    RpeLtePuschPkt *pusch_hdr;
	pusch_hdr = (RpeLtePuschPkt *) RTE_PKTMBUF_READ(pkt, 0, 0, 0);
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;

    WORD32 nAtenna = 1 << (pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.sAntennaInfoCommon.eRxAntennaPortsCount);

    uint16_t nFrame = (ntohs(pusch_hdr->ltePuschHdr.frame_seq))&0x3FF;
    uint8_t nSubF = pusch_hdr->ltePuschHdr.subf_seq;
    uint8_t nSym = pusch_hdr->ltePuschHdr.sym_seq;
    uint8_t nBlkStart = pusch_hdr->ltePuschHdr.blk_start;
    uint8_t nBlkCnt;
    uint8_t nBufSlot = (nFrame%4)*10+nSubF;
    uint8_t nBufOffset = 0;
    WORD32 nDataReadyFlag, nRcveBlock;
    WORD32 nSfIdx = nFrame * 10 + nSubF, nSfDiff;
    CellActiveEnum eCellStatus;
    WORD32 sf_type;
    uint8_t nSymUl_max;

    pusch_hdr->ltePuschHdr.blk_cnt &= 0x3;
    nBlkCnt = pusch_hdr->ltePuschHdr.blk_cnt + 1;

    iLog_Debug(BBU_POOL_CONTROL, "into Processing PUSCH Packet at nSfIdx=%d with global sf %d\n",
    nSfIdx, pBbu->nCurrentSfIdx);

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
    
    sf_type = get_sf_type(pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.subfCfg, nSfIdx);
    if (sf_type == SPECIAL_SUBFRAME)
    {
    	nSymUl_max = g_UpPTS_len[pBbu->sCellCtrlBlk[cIdx].sRadioResourceConfigCommon.specSubfCfg];
    }
    else
    {
    	nSymUl_max = N_SYMB_PER_SF;
    }
    
    uint8_t firstSymIdx = N_SYMB_PER_SF - nSymUl_max;
    if (sf_type == SPECIAL_SUBFRAME && nSym < firstSymIdx) // ignore DwPTS and GP packets
    {
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
            pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].nSegGenerated += nBlkCnt;
            (*pBbu).sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].pData[nBufOffset] = pkt;
            pBbu->sBbuIoIf.sUlBbuIoBufCtrl[nBufSlot][cIdx].bValid= nSfIdx;

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

void inline bbuio_process_timing_pkt(LteRtInforStruct * pLteRt, rte_mbuf * pkt)
{
    UWORD64 ulTsc = rte_rdtsc();
    WORD32 nSfIdx = 0;
    BbuPoolCtrlBlkStruct *pBbu = pLteRt->pBbuPoolCtrlBlk;

	RpeTimingPktPayload *pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(pkt, 0, 0, 0))->message);
    nSfIdx = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
    rte_pktmbuf_free(pkt);

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

inline WORD32 dl_transmission(BbuPoolCtrlBlkStruct *pBbu, WORD32 nType,
    WORD32 *nTxIndicator, WORD32 nTxSfIdx) {
    WORD32 nCellIdx, nTx, nTxBurstLen, nTxCell, idx;
    CellActiveEnum eCellStatus;
    UWORD64 uLatestTsc;
#if (TIME_PROFILE_MODE == 1)
    UWORD64 uDlTsc;
#endif

    RpeLtePdschPkt *pPdsch;
    RpeLtePdcchPkt *pPdcch;
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
								WORD16 nUdpDataSize = ETHER_HDR_LEN + sizeof(struct iphdr1) + sizeof(struct udphdr1) + FB_API_PAYLOAD_OFFSET;
								pChar+= nUdpDataSize;

								pPdsch = (RpeLtePdschPkt *)pChar;
                                nSfTmp = (ntohs(pPdsch->ltePdschHdr.frame_seq) * 10
                                    + pPdsch->ltePdschHdr.subf_seq) % MAX_FRAME_NUM;

                                if (pBbu->sCellCtrlBlk[nCellIdx].sRadioResourceConfigCommon.eSystemBandwidth == B10M) {
                                	//pkt.data_len is 2480, expanded by lte_dl_compression_task_func hack
                                	// printf("=== PDSCH debug: %d\n", pBbuIoBufCtrl->pData[idx]->pkt.data_len);
                                	// override numRB contents to 100 RB
                                	pPdsch->ltePdschHdr.blk_len_h = 25;
                                	pPdsch->ltePdschHdr.blk_len_l = 0;
                            		// oversampling pusch_hdr to exact sampling sample locations
                                }

                                if (nSfTmp != nTxSfIdx) {
                                    /*printf("Tx PDSCH idx=%d/%d/%d sf %d at tx sf %d\n", idx,
                                    pBbuIoBufCtrl->nSegTransferred,
                                    pBbuIoBufCtrl->nSegGenerated,
                                    nSfTmp, nTxSfIdx); */
                                }
                               	WORD16 nHeadRoom = calc_dl_send_headroomUs(pBbu->nCurrentSfIdx, uPrevTsc, nSfTmp, pPdsch->ltePdschHdr.sym_seq);
                                if (nHeadRoom >= (10 * TIME_INTERVAL * 1000 / N_SYMB_PER_SF))
									break;
								else
									nTxBurstLen++;
                            }
                            break;

                        default:
                            break;
                    }

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
  	
    RpeTimingPktPayload *pTiming;
    uint16_t * pFePktType;
    RpeLtePuschPkt *pusch_hdr;


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

	cpu_set_t mask;
	WORD32 NUM_PROCS = sysconf(_SC_NPROCESSORS_CONF);
	sched_getaffinity(gettid(), sizeof(mask), &mask);
	for (int i = 0; i < NUM_PROCS; i++)
	{
		if (CPU_ISSET(i, &mask))
		printf("bbuio_thread sched_getaffinity: i = %d is set\n", i); 
	}

    printf(" Waiting first time packet at Tsc(ms) %llu\n", (UWORD64)(rte_rdtsc()/CPU_HZ/1000));
    /* start sync cell first */
    nFirstUlSf = -1;
    pLteRt->nHeartbeat = 0;
    tPrevTsc = __rdtsc();

	UWORD64 tSendRRUPkg =  __rdtsc();
	uint8_t aIsGotFirstRRUPkg[N_MAX_CELL_PER_BBU];
	uint8_t isGotAllRRUPkg = 0;
	uint64_t sendPkgPeriodUs = 1000*500;
	WORD32	isDispRRUInfo = 1;
	memset(aIsGotFirstRRUPkg, 0, sizeof(aIsGotFirstRRUPkg));
	readMacAddrGroup("UeMacAddrGroup002.txt", &g_sUeMacAddrGroup);

    while(nFirstUlSf < 0){
        pLteRt->nHeartbeat ++;

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
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_Rxbuf_pool);

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
		{
			n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
		}
        iAssert(n_rx <= MAX_PKT_BURST);
        if (n_rx <= 0)
        {
            usleep(10);
        }
        for (idx0 = 0;idx0 < n_rx; idx0++) {

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
			pFePktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkts[idx0], sizeof(struct ether_hdr) + sizeof(struct vlan_hdr), 0, 0); // +14+4

            switch(*pFePktType) {
                case RPE_TIMING_PKT_SUB_TYPE_SWREC:
					pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0))->message);
                    if (nFirstCnt < 50) {
                        tCurrentTsc = __rdtsc();
                        if ((tCurrentTsc - tPrevTsc) > (1000 * CPU_HZ * 0.8)) {
                            /* waiting until two succeed timing packet arrival time is about 1 ms */
                            nFirstCnt ++;
                        }
                        tPrevTsc = tCurrentTsc;
                    } else {
                        nFirstUlSf = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
                        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) %  MAX_FRAME_NUM;
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
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_Rxbuf_pool);
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
		{
			n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
		}
        iAssert(n_rx <= MAX_PKT_BURST);
        if (n_rx <= 0)
        {
            usleep(10);
        }
        nProcessed = 0;
        for (idx0 = 0;idx0 < n_rx; idx0++) {

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
			pFePktType = (uint16_t *) RTE_PKTMBUF_READ(rx_pkts[idx0], sizeof(struct ether_hdr) + sizeof(struct vlan_hdr), 0, 0); 
            nProcessed ++;
            switch(*pFePktType) {
                case RPE_TIMING_PKT_SUB_TYPE_SWREC:
					pTiming = &(((RpeTimingPkt *)RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0))->message);
                    nTmpSf = (((pTiming->radioFrameNumHigh) * RPE_RADIO_FRAME_LOW_MAX_NUM + pTiming->radioFrameNumLow)
                        * NUM_OF_SUBFRAME_IN_ONE_FRAME + pTiming->subFrameNum) % MAX_FRAME_NUM;
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    if (((nTmpSf + 1) % MAX_FRAME_NUM) == nFirstUlSf) {
                        nRunFlag = 0;
                    }
                    break;
                case RPE_PUSCH_PKT_SUB_TYPE_SWREC:
					pusch_hdr = (RpeLtePuschPkt *) RTE_PKTMBUF_READ(rx_pkts[idx0], 0, 0, 0);
                    nTmpSf = (ntohs(pusch_hdr->ltePuschHdr.frame_seq) * NUM_OF_SUBFRAME_IN_ONE_FRAME
                    + pusch_hdr->ltePuschHdr.subf_seq) % MAX_FRAME_NUM;
                    rte_pktmbuf_free(rx_pkts[idx0]);
                    if (((nTmpSf + 1) % MAX_FRAME_NUM) == nFirstUlSf) {
                        nRunFlag = 0;
                    }
                    break;
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

        /* processing received packet */
        iAssert(n_rx <= MAX_PKT_BURST);
        for (Idx = nProcessed; Idx < n_rx; Idx++)
        {
            /* on rx, treat the ethernet prot field and fe subtype as a single
             *  32bit WORD32 */

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


            switch(*pFePktType) {
                case RPE_TIMING_PKT_SUB_TYPE_SWREC:
                    /* process the timing packet */
                    t_start = rte_rdtsc();
                    bbuio_process_timing_pkt(pLteRt, rx_pkts[Idx]);
                    t_end = rte_rdtsc();
                    bbuio_timing_record(t_end - t_start, T_RX_TIME);
                    nTimeOutCnt = 0;
                    break;
				case RPE_PRACH_PKT_SUB_TYPE_SWREC:
					src_UE = *((WORD8 *) RTE_PKTMBUF_READ(rx_pkts[Idx], 10, 0, 0));
					env_UE = *((WORD8 *)RTE_PKTMBUF_READ(rx_pkts[Idx], 11, 0, 0));
                    if(env_UE == pBbu->nPoolIdx)
                    {
                        t_start = rte_rdtsc();
                        //search for nCellIdx
                        pCellCtrlBlk = pBbu->sCellCtrlBlk;
                        for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++)
                        {
							if (nCellIdx == src_UE)
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

                    case RPE_PUSCH_PKT_SUB_TYPE_SWREC:
						src_UE = *((WORD8 *) RTE_PKTMBUF_READ(rx_pkts[Idx], 10, 0, 0));
						env_UE = *((WORD8 *) RTE_PKTMBUF_READ(rx_pkts[Idx], 11, 0, 0));
                        if(env_UE == pBbu->nPoolIdx)
                        {
                            t_start = rte_rdtsc();
                            //search for nCellIdx
                            pCellCtrlBlk = pBbu->sCellCtrlBlk;
                            for (nCellIdx = 0; nCellIdx < N_MAX_CELL_PER_BBU; nCellIdx ++)
                            {
								if (nCellIdx == src_UE)

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

        uLatestTsc = rte_rdtsc();
		processFapiMsg(pLteRt, pBbu->nCurrentSfIdx, uLatestTsc);

        //ack_switch(pBbu);
        sleep_for_a_while(pBbu);

        /* try transmit DL packets */
        /* only tx at this subframe or 1 subframe in advance */
        if ((nTxSfIdx  == pBbu->nCurrentSfIdx) ||
            (((nTxSfIdx - 1 + MAX_FRAME_NUM) % MAX_FRAME_NUM) == pBbu->nCurrentSfIdx))
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

            if ( (nPdschTxCell == nActiveCellDl) &&
                (nPdschTxCell > 0)) {
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
            if (subframe_compare(pBbu->nCurrentSfIdx, nTxSfIdx, 2) > 0)
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
                nTxSfIdx = pBbu->nCurrentSfIdx;
            }
        }

        uLatestTsc = rte_rdtsc();

        /* try rx some packets */
        t_start = rte_rdtsc();

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
					rx_pkts[n_rx] = rte_pktmbuf_alloc(pBbu->sBbuIoIf.bbuio_Rxbuf_pool);
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
		{
			n_rx = rte_eth_rx_burst(nPortID, nQueueID, rx_pkts, MAX_PKT_BURST);
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
