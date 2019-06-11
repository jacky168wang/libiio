/*
 * @file       RPE_API.h
 *
 * @copyright  Copyright (c) 2016 Hong Kong Applied Science and Technology
 *             Research Institute Company Limited (ASTRI)
 *             Proprietary and Confidential Information.\n
 *             Not to be copied or distributed.
 *
 * @author     Eric Leung (ASTRI)
 * @version    1.0
 * @date       Sep 5, 2016
 * 
 * @brief      Declaration for packets handler and generators for IPC channels.
 */

#ifndef RPE_API_H_
#define RPE_API_H_

#include "SymbolTracker.h"

#define ENABLE_DUMMY_PDSCH 0

#define ENABLE_DUMMY_CSR_PRACH 0

#define ENABLE_DUMMY_CSR_TDDCONFIG 0

//-------------------------------------
#define RPE_NUM_SYMBOL_VALID 39 //with in two subframes in the future still ok

typedef enum {
	RPE_CSR_DEVSELECT_LOCAL = 0xF
}RpeCsrDevSelect_e;

typedef enum {
	RPE_CSR_READ = 0,
	RPE_CSR_WRITE = 1,
}RpeCsrRW_e;

typedef enum {
	RPE_RETURN_GOT_RECORD = 0,      //got written to the symbol tracker
	RPE_RETURN_TOO_OLD_TOO_FAR = 1, //symbols too old or too far away from current tti symbol
	RPE_RETURN_WAIT = 2,            //very close but not close enough to write to symbol tracker yet
}RpePdschReturnType_e;

typedef enum {
	CSR_ADDR_RS_QPSK        = 0x00001000,
	CSR_ADDR_BCH_QPSK       = 0x00001008,
	CSR_ADDR_RS_OS_A0_S0    = 0x0000100C,
	CSR_ADDR_RS_OS_A0_S1    = 0x00001010,
	CSR_ADDR_RS_OS_A0_S2    = 0x00001014,
	CSR_ADDR_RS_OS_A0_S3    = 0x00001018,
	CSR_ADDR_RS_OS_A1_S0    = 0x0000101C,
	CSR_ADDR_RS_OS_A1_S1    = 0x00001020,
	CSR_ADDR_RS_OS_A1_S2    = 0x00001024,
	CSR_ADDR_RS_OS_A1_S3    = 0x00001028,
	CSR_ADDR_RS_ENABLE      = 0x0000102C,
	CSR_ADDR_BCH_ENABLE     = 0x00001030,
	CSR_ADDR_PSS_ENABLE     = 0x00001034,
	//CSR_ADDR_TIMING_PKT_OS  = 0x00001038,
	//CSR_ADDR_TMINGG_PKT_EN  = 0x0000103C,
	//CSR_ADDR_TX_START_EN    = 0x00005004,
	//CSR_ADDR_PRESENT_PKT_EN = 0x00001040,
	//ASTRI DEFINED REGISTERS TO SUPPORT EXTNORMCP, NumTxAnt, NumRxAnt, TDD/FDD, ULDL CONFIG, SPSF CONFIG
	CSR_ADDR_INITCONFIG     = 0x00006000, //Ext/Norm CP, NumTxAnt, NumRxAnt, BW
	CSR_ADDR_TDDCONFIG      = 0x00006004, //FDD/TDD, ULDL Config, Spsf Config
	CSR_ADDR_SYSTIMECONFIG  = 0x00006008, //type, system time % 256, radio frame offset
	CSR_ADDR_PRACHCONFIG    = 0x0000600C, //prach config index
}Rpe_CSR_Addr_e;

typedef struct{
	uint8_t ExtNormCP;  //0: Extended, 1: Normal
	uint8_t NumTxAnt;   //1 or 2 tx antenna
	uint8_t NumRxAnt;   //2 rx antenna
	uint8_t BW;         //0/1/2/3/4/5 = 1.4/3/5/10/15/20 MHz
}fe_initconfig_t;

typedef struct{
	uint8_t TDDFDD;     //1:FDD, 2:TDD
	uint8_t ULDLConfig; //0-6
	uint8_t SPSFConfig; //0-8
	uint8_t reserve;
}fe_tddfddconfig_t;

typedef struct{
	uint8_t SysTimeSrc;        //system time source (1588: type = 4 ) referred to PM_FeSch.h's e_palign_t
	uint8_t SysTimeMod256;     //system time % 256 in second
	uint16_t RadioFrameOffSet; //radio frame offset
}fe_systimeconfig_t;

typedef struct{
	uint8_t reserved[2];
	uint8_t prach_freq_offset; //0 - (UL_NUMRB-6)
	uint8_t prach_config_idx; //0-63
}fe_prachconfig_t;

typedef struct{
	int16_t QPSK0;
	int16_t QPSK1;
}qpsk_t;

typedef struct{
	uint32_t reserved:29;
	uint32_t rs_offset:3;
}rs_os_t;

typedef struct{
	uint32_t reserved:31;
	uint32_t enable:1;
}en_t;

typedef struct{
	uint32_t reserved:15;
	uint32_t txEnable:1;
	uint32_t reserved1:4;
	uint32_t rfstartnum:12;
}tx_start_en_t;

//typedef struct{
//	Complex16 RS_QPSK;
//	Complex16 BCH_QPSK;
//	rs_os_t rs_os[2][4]; //antennas, symbols
//	en_t rs_enable;
//	en_t bch_enable;
//	en_t pss_enable;
//	en_t timing_enable;
//	tx_start_en_t tx_start_enable;
//	//ASTRI Defined Register start here
//	fe_initconfig_t fe_initialconfig;
//	fe_tddfddconfig_t fe_tddfddconfig;
//	fe_prachconfig_t fe_prachconfig;
//}Rpe_CSR_Register_t;

//-------------------------------------

#define SUPPORT_PDSCH		1
#define SUPPORT_PDCCH		 0
#define SUPPORT_PBCH		1
#define SUPPORT_TIMING		1
#define SUPPORT_PSS			1
#define SUPPORT_RS			1
#define RPE_SUPPORT_CPRI		 0
#define SUPPORT_TIME_DOMAIN  0
#define SUPPORT_CSR			1
#define SUPPORT_PRESENT		 0
#define SUPPORT_MGNT_ST		 0
#define SUPPORT_PUSCH		1
#define SUPPORT_PRACH		1


typedef enum
{
	PKT_SUBTYPE_PDSCH	= 0x0,
	PKT_SUBTYPE_PBCH    = 0x2,
	PKT_SUBTYPE_TIMING	= 0x3,
	PKT_SUBTYPE_PSS    	= 0x4,
	PKT_SUBTYPE_RS     	= 0x5,
	PKT_SUBTYPE_CSR    	= 0x8,
	PKT_SUBTYPE_PUSCH  	= 0xB,
	PKT_SUBTYPE_PRACH  	= 0xC,
	NUM_SUBTYPE	  		= (SUPPORT_PDSCH   + SUPPORT_PDCCH       + SUPPORT_PBCH  + \
			               SUPPORT_TIMING  + SUPPORT_PSS         + SUPPORT_RS    + \
			               RPE_SUPPORT_CPRI    + SUPPORT_TIME_DOMAIN + SUPPORT_CSR   + \
			               SUPPORT_PRESENT + SUPPORT_MGNT_ST     + SUPPORT_PUSCH +
			               SUPPORT_PRACH)
}rpe_subtype_e;

typedef struct {
	uint32_t DestinationMAC_h;
	
	uint16_t DestinationMAC_l;
	uint16_t SourceMAC_h;
	
	uint32_t SourceMAC_l;
	
	uint16_t TPID;
	uint16_t PCP_VLANID;
	
	uint16_t reserved; //don't care
	uint16_t sub_type; //rpe_subtype_e
}rpe_common_t;

typedef struct {
	uint32_t reserved;
	
	uint32_t rw:1; //assume 0 as read; 1 as write
	uint32_t tag:3;
	uint32_t be:4;
	uint32_t devsel:4;
	uint32_t csr_addr:20;
	uint32_t csr_data;
} rpe_csrPkt_t; //0x0008

typedef struct {
	uint32_t reserved:8;
	uint32_t dl_underflow_cnt:4; //4 bit to report how many dl underflow count
	uint32_t ul_overflow:1; //1: ul overflow error has occured - hardware not fast enough to process ul
	uint32_t dl_overflow:1; //1:dl overflow error - too many symbols; 0: ok
	uint32_t dl_underflow:1; //1:dl underflow error - not enough symbols; 0: ok
	uint32_t dl_sync:1; //1:sync; 0:not sync
	uint32_t rfnum:12;
	uint32_t sfnum:4;
} rpe_timingPkt_t; //0x0003

typedef struct __attribute__((packed)) {
	int16_t mantissa: 9; 
	int16_t reserved: 1;
	int16_t exponent: 6;
} gain_t;

typedef struct {
	uint32_t reserved;
	uint32_t reserved1:22;
	uint32_t is_S_Frame:1;
	uint32_t reserved2:1;
	uint32_t rb_num:7;
	uint32_t reserved3:1;
	
	uint32_t reserved4:2;
	uint32_t ant_num:3;
	uint32_t ant_start:3;
	uint32_t rf_num:16;
	uint32_t sf_num:4;
	uint32_t sym_num:4;
	
	gain_t gain[8];
	
	uint8_t sample[];
} rpe_puschPkt_t; //0x0003

//typedef struct {
//	uint32_t reserved:8;
//	uint32_t rf_num:16; //rf index this antenna data belongs to
//	uint32_t sf_num:4; //sf index this antenna data belongs to
//	uint32_t ant_num:4; //ant index this antenna data belongs to
//	
//	uint16_t sample[];
//} rpe_prachPkt_t; //0x000C

#define rpe_prachPkt_t rpe_puschPkt_t

typedef struct {
	uint32_t reserved;
	
	uint32_t reserved1:25;
	uint32_t is_S_Frame:1;
	uint32_t enable_rb_sel_table:1;
	uint32_t numrb_h: 5; //Hi part of NumRB - 20/15/10/5/3/1.4 MHz = 100/75/50/25/15/6 NumRB
	
	uint32_t numrb_l: 2; //Lo part of NumRB
	uint32_t ant_num: 3;  //Number of Antennas: 0-7, 0/1/3/7 represents 1/2/4/8 ants 
	uint32_t ant_start: 3; //Antenna Start: 0-1
	uint32_t rf_num: 16; //0-1023
	uint32_t sf_num: 4; //0-9
	uint32_t sym_num: 4; //0-13
	
	uint8_t sample[]; //variable array of dataBlock when NumRB or NumAnt are non-zero.
	
} rpe_pdschPkt_t;

typedef struct  __attribute__((packed)) {
	union{
		uint8_t all;
		struct {
			uint8_t re0:2;
			uint8_t re1:2;
			uint8_t re2:2;
			uint8_t re3:2;
		}res;
	};
} qpsk_re_t;

typedef struct {
	uint32_t reserved;
	struct {
		struct{
			uint8_t re0:2;
			uint8_t re1:2;
			uint8_t re2:2;
			uint8_t re3:2;
			
			uint8_t re4:2;
			uint8_t re5:2;
			uint8_t re6:2;
			uint8_t re7:2;
			
			uint8_t re8:2;
			uint8_t re9:2;
			uint8_t re10:2;
			uint8_t re11:2;
			
			uint8_t re12:2;
			uint8_t re13:2;
			uint8_t re14:2;
			uint8_t re15:2;
			
			uint8_t re16:2;
			uint8_t re17:2;
			uint8_t reserved1:4;
			
			uint8_t reserved2[3];
		}RE[4];
	}Symbol[4];
} rpe_pbchPkt_t;

typedef struct {
	uint32_t reserved;
	
	struct{
		Complex16 value;
		uint32_t reserved;
	}PSS_RE[72];
	
	struct{
		Complex16 value;
		uint32_t reserved;
	}SSS_RE[72];
		
} rpe_psssssPkt_t;

typedef struct {
	uint8_t reserved[3];
	uint8_t AntBitMask;
	
	struct{ //2048 Bytes aligned for every Antenna
		uint8_t RS[10][4][50]; //each of this element contains 4 RS, 2 bits each
		uint8_t dummy[48];
	}Antenna[MAX_NUM_RX_ANT];
		
} rpe_rsPkt_t;

typedef struct {
	rpe_common_t header; //from the sub_type in the header,
	union {
		rpe_csrPkt_t    rpe_csrPkt;
		rpe_timingPkt_t rpe_timingPkt;
		rpe_puschPkt_t  rpe_puschPkt;
		rpe_prachPkt_t  rpe_prachPkt;
		rpe_pdschPkt_t  rpe_pdschPkt;
		rpe_pbchPkt_t   rpe_bchPkt;
		rpe_psssssPkt_t rpe_psssssPkt;
		rpe_rsPkt_t     rpe_rsPkt;
	};
}rpe_Pkt_t;

RpePdschReturnType_e Packet_Handler(rpe_Pkt_t* handle);

RpePdschReturnType_e Pdsch_Valid_Checker(rpe_Pkt_t* handle, int* diff);

uint32_t Timing_Generator(uint16_t l_sfnsf, uint32_t exception, uint32_t num_dlsym_missed);

void Pusch_Generator(SymSfnSf_t tempSymSfnSf, Vector_Type32 **sample_ptr, int8_t *scale_ptr);

void Send_PuschPkt(SymSfnSf_t tempSymSfnSf);

uint32_t Prach_Generator(SymSfnSf_t* tempSymSfnSf, Complex8 **sample_ptr, uint8_t **scale_ptr);

#if (ENABLE_DUMMY_PDSCH==1)
void dummy_PdschGenerator(uint8_t** packet_ptr);

void dummy_PdschGenerator_inc();

#elif (ENABLE_DUMMY_PDSCH==2)
void dummy_PdschGenerator_insert_packet(SymSfnSf_t temp);

uint32_t dummy_PdschGenerator_packet_available(uint8_t** packet_ptr);

void dummy_PdschGenerator_update_idx();
#endif

#if (ENABLE_DUMMY_CSR_PRACH==1)
void dummy_PrachConfig_update();
#endif

#if (ENABLE_DUMMY_CSR_TDDCONFIG==1)
void dummy_TddConfig_update();
#endif

#endif /* RPE_API_H_ */
