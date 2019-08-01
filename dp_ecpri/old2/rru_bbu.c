#include "rru_bbu.h"

void display_buffer(uint8_t *buf, size_t size)
{
	size_t i;

	printf("\r\n---- Whole rawsocket Buffer (%lu):", size);
	for (i=0; i<size; i++) {
		if (0 == i % 16) printf("\r\n");
		printf("%02x ", buf[i]);
	}
	printf("\r\n\r\n");	
	fflush(stdout);
}

uint8_t rawst_cbuf[RAWSTX_BUF_SZ][RAWST_BUF_NB_NOW];
volatile sig_atomic_t rawst_iw = 0;
volatile sig_atomic_t rawst_ir = 0;
uint8_t rawsr_cbuf[RAWSRX_BUF_SZ][RAWSR_BUF_NB_NOW];
volatile sig_atomic_t rawsr_ir = 0;
volatile sig_atomic_t rawsr_iw = RAWSR_BUF_NB_NOW;

struct timespec tm_ts, tm_te;
struct timespec tm_rs, tm_re;

volatile sig_atomic_t pusch_tx_cnt;
volatile sig_atomic_t pusch_rx_cnt;
volatile sig_atomic_t tmpps_tx_cnt;
volatile sig_atomic_t tmpps_rx_cnt;
volatile sig_atomic_t pdsch_tx_cnt;
volatile sig_atomic_t pdsch_rx_cnt;

volatile sig_atomic_t taskstop_rawsr;
volatile sig_atomic_t taskstop_rawst;
#if defined(TMPPS_TRG_UGPIOIRQ)
volatile sig_atomic_t taskstop_tmirq;
#endif

uint32_t iiopr_ovfw;
uint32_t iiopt_udfw;
uint32_t iiopt_ovfw;
uint32_t rawst_ovfw;
uint32_t rawsr_udfw;
uint32_t rawsr_ovfw;

void clear_counters(void)
{
	pusch_tx_cnt = 0;
	pusch_rx_cnt = 0;
	tmpps_tx_cnt = 0;
	tmpps_rx_cnt = 0;
	pdsch_tx_cnt = 0;
	pdsch_rx_cnt = 0;
	
	iiopr_ovfw = 0;
	iiopt_udfw = 0;
	rawst_ovfw = 0;
	rawsr_udfw = 0;
	rawsr_ovfw = 0;
}

#if 0
void display_tx_counters(void)
{
#if defined(BUILDING_BBU_DL)
	printf("\rntb_of=%10u, tx_pdsch=%10u", 
		 rawst_ovfw, pdsch_tx_cnt);
	if (rawst_ovfw)
		printf("\r\n");
#endif
#if defined(BUILDING_RRU_UL)
	printf("\rirb_of=%10u, ntb_of=%10u, tx_pusch=%10u; tm_tx=%10u, tm_rx=%10u", 
		iiopr_ovfw, rawst_ovfw, pusch_tx_cnt, 
		tmpps_tx_cnt, tmpps_rx_cnt);
	if (iiopr_ovfw || rawst_ovfw)
		printf("\r\n");
#endif
}

void display_rx_counters(void)
{
#if defined(BUILDING_BBU_UL)
	printf("\rnrb_uf=%10u, nrb_of=%10u, rx_pusch=%10u, rx_tmpps=%10u", 
		 rawsr_ovfw, rawsr_ovfw, pusch_rx_cnt, tmpps_rx_cnt);
	if (rawsr_ovfw)
		printf("\r\n");
	fflush(stdout);
#endif
#if defined(BUILDING_RRU_DL)
	printf("\rnrb_uf=%10u, nrb_of=%10u, rx_pdsch=%10u, itb_uf=%10u", 
		rawsr_udfw, rawsr_ovfw, pdsch_rx_cnt, iiopt_udfw);
	if (rawsr_ovfw)
		printf("\r\n");
#endif
}
#else
void display_tx_counters(void)
{
#if defined(BUILDING_BBU_DL)
	printf("\rntb_of=%u, tx_pdsch=%u", 
		 rawst_ovfw, pdsch_tx_cnt);
	if (rawst_ovfw)
		printf("\r\n");
#endif
#if defined(BUILDING_RRU_UL)
	printf("\rirb_of=%u, ntb_of=%u, tx_pusch=%u; tm_tx=%u, tm_rx=%u", 
		iiopr_ovfw, rawst_ovfw, pusch_tx_cnt, 
		tmpps_tx_cnt, tmpps_rx_cnt);
	if (iiopr_ovfw || rawst_ovfw)
		printf("\r\n");
#endif
	fflush(stdout);
}

void display_rx_counters(void)
{
#if defined(BUILDING_BBU_UL)
	printf("\rnrb_uf=%u, nrb_of=%u, rx_pusch=%u, rx_tmpps=%u", 
		 rawsr_ovfw, rawsr_ovfw, pusch_rx_cnt, tmpps_rx_cnt);
	if (rawsr_ovfw)
		printf("\r\n");
#endif
#if defined(BUILDING_RRU_DL)
	printf("\rnrb_uf=%u, nrb_of=%u, rx_pdsch=%u, itb_uf=%u", 
		rawsr_udfw, rawsr_ovfw, pdsch_rx_cnt, iiopt_udfw);
	if (rawsr_ovfw)
		printf("\r\n");
#endif
	fflush(stdout);
}

void display_counters(void)
{
#if defined(BUILDING_BBU_DL) || defined(BUILDING_BBU_UL)/*
	if (rawsr_ovfw || rawst_ovfw || rawsr_udfw)
		printf("\r\nntb_of=%u, tx_pdsch=%u; "
			"nrb_uf=%u, nrb_of=%u, rx_pusch=%u, rx_tmpps=%u", 
			 rawst_ovfw, pdsch_tx_cnt,
			 rawsr_udfw, rawsr_ovfw, pusch_rx_cnt, tmpps_rx_cnt);
	else if (0 == pdsch_tx_cnt % 70)*/
		printf("\rtx_pdsch=%u; rx_pusch=%u, rx_tmpps=%u", 
			 pdsch_tx_cnt,
			 pusch_rx_cnt, tmpps_rx_cnt);
#endif
#if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)/*
	if (iiopr_ovfw || rawst_ovfw || rawsr_ovfw)
		printf("\rtm_tx=%u, tm_rx=%u; "
			"irb_of=%u, ntb_of=%u, tx_pusch=%u; "
			"nrb_uf=%u, nrb_of=%u, rx_pdsch=%u, itb_uf=%u", 
			tmpps_tx_cnt, tmpps_rx_cnt, 
			iiopr_ovfw, rawst_ovfw, pusch_tx_cnt, 
			rawsr_udfw, rawsr_ovfw, pdsch_rx_cnt, iiopt_udfw);
	else if (0 == pusch_tx_cnt % 70)*/
		printf("\rtm_tx=%u, tm_rx=%u; "
			"tx_pusch=%u; "
			"rx_pdsch=%u, itb_uf=%u", 
			tmpps_tx_cnt, tmpps_rx_cnt, 
			pusch_tx_cnt, 
			pdsch_rx_cnt, iiopt_udfw);
#endif
	fflush(stdout);
}

#endif

void display_tx_perf(struct timespec *e, struct timespec *s)
{
	double tm_us = elapse_us(e, s);
#if defined(BUILDING_RRU_UL)
	if (pusch_tx_cnt > 0) {
	printf("\r\nTX PUSCH %lu in %.0f us (%7.3f Mbps, %7.3f us)\r\n",
		pusch_tx_cnt, tm_us,
		tm_us ? pusch_tx_cnt*PUSCH_UBUF_LEN/tm_us*8 : 0,
		tm_us/pusch_tx_cnt);
	}
#endif
#if defined(BUILDING_BBU_DL)
	if (pdsch_tx_cnt > 0) {
	printf("\r\nTX PDSCH %lu in %.0f us (%7.3f Mbps, %7.3f us)\r\n",
		pdsch_tx_cnt, tm_us,
		tm_us ? pdsch_tx_cnt*PDSCH_UBUF_LEN/tm_us*8 : 0,
		tm_us/pdsch_tx_cnt);
	}
#endif
}

void display_rx_perf(struct timespec *e, struct timespec *s)
{
	double tm_us = elapse_us(e, s);
#if defined(BUILDING_RRU_DL)
	if (pdsch_rx_cnt > 0) {
	printf("\r\nRX PDSCH %lu in %.0f us (%7.3f Mbps, %7.3f us)\r\n",
		pdsch_rx_cnt, tm_us,
		tm_us ? pdsch_rx_cnt*PDSCH_UBUF_LEN/tm_us*8 : 0,
		tm_us/pdsch_rx_cnt);
	}
#endif
#if defined(BUILDING_BBU_UL)
	if (pusch_rx_cnt > 0) {
	printf("\r\nRX PUSCH %lu in %.0f us (%7.3f Mbps, %7.3f us)\r\n",
		pusch_rx_cnt, tm_us,
		tm_us ? pusch_rx_cnt*PUSCH_UBUF_LEN/tm_us*8 : 0,
		tm_us/pusch_rx_cnt);
	}
#endif
}

/*----------------------------------
*/

void display_pusch(uint8_t *buf)
{
	int iprt, ismp;
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;
	struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pusch_hdr));

	printf("\r\n---- PUSCH HEADER (%lu):\r\n", sizeof(*ph));
	//printf("sub_type[15:0]=0x%04x\r\n", ph->sub_type);

	//printf("rsvd1[31:0]=0x%08x\r\n", ph->rsvd1);

	//printf("rsvd2d1[31:10]=0x%06x ", ph->rsvd2d1);
	printf("special[9]=0x%x ", ph->special);
	//printf("rsvd2d2[8]=0x%x ", ph->rsvd2d2);
	printf("rb_seq[7:1]=0x%02x ", ph->rb_seq);
	//printf("rsvd2d3[0]=0x%x\r\n", ph->rsvd2d3);

	//printf("rsvd3[31:30]=0x%x ", ph->rsvd3);
	printf("ant_cnt[29:27]=0x%x ", ph->ant_cnt);
	printf("ant_start[26:24]=0x%x ", ph->ant_start);
	printf("frame_seq[23:8]=0x%04x ", ph->frame_seq);
	printf("subf_seq[7:4]=0x%x ", ph->subf_seq);
	printf("sym_seq[3:0]=0x%x\r\n", ph->sym_seq);
	printf("ant0_gain[31:16]=0x%04x ", *(int16_t *)(&(ph->gain[0])));
	printf("ant1_gain[15:0]=0x%04x\r\n", *(int16_t *)(&(ph->gain[1])));
    /*
	for (iprt = 0; iprt < TOT_PORT_NB; iprt++) {
		printf("---- PUSCH PAYLOAD: I/Q pairs for Port%d (%lu):\r\n",
		    iprt, sizeof(*pl)/TOT_PORT_NB);
		for (ismp = 0; ismp < RADIO_SYM2SMP; ismp++) {
			printf("%02x %02x ", (uint8_t)(pl->port[iprt][ismp].i16), 
				(uint8_t)(pl->port[iprt][ismp].q16));
		}
	}*/

    printf("\r\n");
	fflush(stdout);
}

void fill_pusch_pld(uint8_t *buf)
{
	int iprt, ismp;
	struct bbu_payload *pl = (struct bbu_payload *)buf;
	//struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pusch_hdr));

	for (iprt = 0; iprt < TOT_PORT_NB; iprt++) {
		for(ismp = 0; ismp < RADIO_SYM2SMP; ismp++) {
			pl->iqs[iprt][ismp].i8 = 0x01*(2*iprt+1);
			pl->iqs[iprt][ismp].q8 = 0x01*(2*iprt+2);
		}
	}
}

void pack_pusch_hdr(uint8_t *buf)
{
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pusch_hdr, rsvd1));

	//memset(ph, 0, sizeof(struct pusch_hdr));

	ph->sub_type = htons(PUSCH_SUBTYPE);

    //1st transfer
	//*p32 = htonl(*p32);

    //2nd transfer
	ph->rb_seq  = RADIO_RB_NB;
	ph->special = 0;
	p32++; *p32 = htonl(*p32);

    //3rd transfer
	ph->sym_seq = RADIO_UL_SYM_OF + (pusch_tx_cnt % RADIO_UL_SYM_NB);
	ph->subf_seq = (pusch_tx_cnt/RADIO_UL_SYM_NB) % RADIO_FRM2SUB;
	ph->frame_seq = ((pusch_tx_cnt/RADIO_UL_SYM_NB) / RADIO_FRM2SUB) & 0x3ff;
	ph->ant_start = 0;
	ph->ant_cnt = TOT_PORT_NB - 1;
	p32++; *p32 = htonl(*p32);

#if 0
    //4th transfer
	ph->ant0_gain = htons(0);
	ph->ant1_gain = htons(0);

    //5th transfer
	ph->ant2_gain = htons(0);
	ph->ant3_gain = htons(0);

	//6th transfer
	ph->ant4_gain = htons(0);
	ph->ant5_gain = htons(0);

	//7th transfer
	ph->ant6_gain = htons(0);
	ph->ant7_gain = htons(0);
#endif

    pusch_tx_cnt++;
#if defined(BUILDING_RRU_UL)
    if (1 == pusch_tx_cnt) clock_gettime(CLOCK_MONOTONIC, &tm_ts);
#endif
	//display_pusch(buf);
}

#if defined(BUILDING_RRU_PUSCH_B2B)
extern int datapath_handler_tx(int idev);
#endif
void unpack_pusch(uint8_t *buf)
{
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pusch_hdr, rsvd1));

	pusch_rx_cnt++;
#if defined(BUILDING_BBU_UL)
    if (1 == pusch_rx_cnt) clock_gettime(CLOCK_MONOTONIC, &tm_rs);
#endif

	ph->sub_type = ntohs(ph->sub_type);

    //1st transfer
	//*p32 = ntohl(*p32);

    //2nd transfer
	p32++; *p32 = ntohl(*p32);

    //3rd transfer
	p32++; *p32 = ntohl(*p32);
#if 0
    //4th transfer
    ph->ant0_gain = ntohs(ph->ant0_gain);
    ph->ant1_gain = ntohs(ph->ant1_gain);

    //5th transfer
    ph->ant2_gain = ntohs(ph->ant2_gain);
    ph->ant3_gain = ntohs(ph->ant3_gain);

    //6th transfer
    ph->ant4_gain = ntohs(ph->ant4_gain);
    ph->ant5_gain = ntohs(ph->ant5_gain);

    //7th transfer
    ph->ant6_gain = ntohs(ph->ant6_gain);
    ph->ant7_gain = ntohs(ph->ant7_gain);
#endif

	//display_pusch(buf);
#if defined(BUILDING_RRU_PUSCH_B2B)
	do {
		if (0 == datapath_handler_tx(0))
			break;
	} while (1);
#endif
}

/*----------------------------------
*/

void display_pdsch(uint8_t *buf)
{
	int iprt, ismp;
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;
	struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pdsch_hdr));

	printf("\r\n---- PDSCH HEADER (%lu):\r\n", sizeof(*ph));
	//printf("sub_type[15:0]=0x%04x\r\n", ph->sub_type);

	//printf("rsvd1[31:0]=0x%08x\r\n", ph->rsvd1);

	//printf("rsvd2[31:7]=0x%07x ", ph->rsvd2);
	printf("special[6]=0x%x ", ph->special);
	printf("rb_sel[5]=0x%x ", ph->rb_sel);
	printf("rb_seq_h[4:0]=0x%02x\r\n", ph->rb_seq_h);

	printf("rb_seq_l[31:30]=0x%x ", ph->rb_seq_l);
	printf("ant_cnt[29:27]=0x%x ", ph->ant_cnt);
	printf("ant_start[26:24]=0x%x ", ph->ant_start);
	printf("frame_seq[23:8]=0x%02x ", ph->frame_seq);
	printf("subf_seq[7:4]=0x%x ", ph->subf_seq);
	printf("sym_seq[3:0]=0x%x\r\n", ph->sym_seq);
    /*
	for (iprt = 0; iprt < TOT_PORT_NB; iprt++) {
		printf("---- PDSCH PAYLOAD: I/Q pairs for Port%d (%lu):\r\n",
		    iprt, sizeof(*pl)/TOT_PORT_NB);
		for (ismp = 0; ismp < RADIO_SYM2SMP; ismp++) {
			printf("%02x %02x ", (uint8_t)(pl->port[iprt][ismp].i16), 
				(uint8_t)(pl->port[iprt][ismp].q16));
		}
	}*/

    printf("\r\n");
	fflush(stdout);
}

void fill_pdsch_pld(uint8_t *buf)
{
	int iprt, ismp;
	struct bbu_payload *pl = (struct bbu_payload *)buf;
	//struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pdsch_hdr));

	for (iprt = 0; iprt < TOT_PORT_NB; iprt++) {
		for(ismp = 0; ismp < RADIO_SYM2SMP; ismp++) {
			pl->iqs[iprt][ismp].i8 = 0x10*(2*iprt+1);
			pl->iqs[iprt][ismp].q8 = 0x10*(2*iprt+1);
		}
	}
}

void pack_pdsch_hdr(uint8_t *buf)
{
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pdsch_hdr, rsvd1));

	memset(ph, 0, sizeof(struct pdsch_hdr));
	ph->sub_type = htons(PDSCH_SUBTYPE);

	//1st transfer
	//*p32 = htonl(*p32);

    //2nd transfer
	ph->rb_seq_h = (RADIO_RB_NB >> 2) & 0x7c;
	ph->rb_sel = 0;
	ph->special = 0;	//TODO: when should be 1?
	p32++; *p32 = htonl(*p32);

    //3rd transfer
	ph->sym_seq = RADIO_DL_SYM_OF + (pdsch_tx_cnt % RADIO_DL_SYM_NB);
	ph->subf_seq = (pdsch_tx_cnt/RADIO_DL_SYM_NB) % RADIO_FRM2SUB;
	ph->frame_seq = (pdsch_tx_cnt/RADIO_DL_SYM_NB/RADIO_FRM2SUB) & 0x3ff;
	ph->ant_start = 0;
	ph->ant_cnt = TOT_PORT_NB;
	ph->rb_seq_l = RADIO_RB_NB & 0x03;
	p32++; *p32 = htonl(*p32);

    pdsch_tx_cnt++;
#if defined(BUILDING_BBU_DL)
    if (1 == pdsch_tx_cnt) clock_gettime(CLOCK_MONOTONIC, &tm_ts);
#endif
	//display_pdsch(buf);
}

void unpack_pdsch(uint8_t *buf)
{
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pdsch_hdr, rsvd1));

    pdsch_rx_cnt++;
#if defined(BUILDING_RRU_UL)
    if (1 == pdsch_rx_cnt) clock_gettime(CLOCK_MONOTONIC, &tm_rs);
#endif
	//display_pdsch(buf);

	ph->sub_type = ntohs(ph->sub_type);

    //1st transfer
	//*p32 = ntohl(*p32);

    //2nd transfer
	p32++; *p32 = ntohl(*p32);

    //3rd transfer
	p32++; *p32 = ntohl(*p32);
}

/*----------------------------------
*/
sem_t g_sem_pps;

void display_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;

	printf("\r\n---- TIMING HEADER (%lu):\r\n", sizeof(*ph));
	//printf("sub_type[15:0]=0x%04x\r\n", ph->sub_type);

	printf("dl_bof_cnt[23:20]=0x%x ", ph->dl_bof_cnt);
	printf("ul_bof_flg[19]=0x%x ", ph->ul_bof_flg);
	printf("dl_bof_flg[18]=0x%x ", ph->dl_bof_flg);
	printf("dl_buf_flg[17]=0x%x ", ph->dl_buf_flg);
	printf("symbol_sync[16]=0x%x ", ph->symbol_sync);
	printf("frame_seq[15:4]=0x%03x ", ph->frame_seq);
	printf("ant_cnt[3:0]=0x%x\r\n", ph->subf_seq);
    printf("\r\n");

	fflush(stdout);
}

/*
-- UL iio_rxdma overflow
  definition: rx_fifo shift new input samples in before
    kernel iio_buffer <- rx_dma <- rx_fifo
  judgement: jesd/iio_txdma register 0x80000088
-- UL libiio2rawsock overflow
  definition: new data overwrite rawst_cbuf[] before sendto() finish
  judgement: impossible by current design

-- DL iio_rxdma underflow
  definition: kernel iio_buffer -> tx_dma -> tx_fifo before
    tx_fifo shift new output samples out
  judgement: jesd/iio_rxdma register 0x80000088
-- DL libiio2rawsock underflow
  definition: new data overwrite rawsr_cbuf[] before recvfrom() finish
  judgement: impossible by current design
*/
void pack_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + 
        offsetof(struct tmpps_hdr, sub_type) + sizeof(ph->sub_type));

	//memset(ph, 0, sizeof(struct tmpps_hdr));
	ph->sub_type = htons(TMPPS_SUBTYPE);

    //1st transfer
	if (is_iiorxdma_overflow(0)) iiopr_ovfw++;
    if (iiopr_ovfw | rawst_ovfw) ph->ul_bof_flg = 1;

	if (is_iiotxdma_underflow(0)) iiopt_udfw++;
    if (iiopt_udfw | rawsr_udfw) ph->dl_buf_flg = 1;

	ph->dl_bof_flg = rawsr_ovfw > 0 ? 1 : 0;
	ph->dl_bof_cnt = rawsr_ovfw & 0xff;

	ph->symbol_sync = 1;
#if !defined(BUILDING_TMPPS_SOLO)
	ph->frame_seq = ((pusch_tx_cnt/RADIO_UL_SYM_NB) / RADIO_FRM2SUB) & 0x3ff;
	ph->subf_seq = (pusch_tx_cnt/RADIO_UL_SYM_NB) % RADIO_FRM2SUB;
#else
    ph->frame_seq = (tmpps_tx_cnt / RADIO_FRM2SUB) & 0x3ff;
    ph->subf_seq = tmpps_tx_cnt % RADIO_FRM2SUB;
    tmpps_tx_cnt++;
#endif
	*p32 = htonl(*p32);

	//display_tmpps(buf);
}

void unpack_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + 
        offsetof(struct tmpps_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = ntohs(ph->sub_type);

    //1st transfer
	*p32 = ntohl(*p32);

	tmpps_rx_cnt++;
	//display_tmpps(buf);
}

/*---- globals ----*/
char *str_devname = "eth0";
#if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)
uint8_t g_peer_mac[6] = { 0xa0, 0x36, 0x9f, 0x3b, 0xf3, 0x74 };
//char    g_peer_ips[16] = "10.88.120.122";
char    g_peer_ips[16] = "10.88.120.52";
#else
uint8_t g_peer_mac[6] = { 0xa0, 0x36, 0x9f, 0x42, 0x74, 0x90 };
//char    g_peer_ips[16] = "10.88.120.133";
char    g_peer_ips[16] = "10.88.120.32";
#endif
short   g_rru_txport = 2152;
short   g_rru_rxport = 2152;
short   g_bbu_txport = 2153;
short   g_bbu_rxport = 2153;

void common_getargs(int argc, char* argv[])
{
	int i = 1;
	printf("%s\r\n", 0x00 == *((char *)&i) ? "BigEndian" : "LittleEndian");

	if (argc > 1) {
		str_devname = argv[1];
	}
	printf("to use the ethernet IF %s\r\n", str_devname);

	if (argc > 2) {
		macaddr_s2a(g_peer_mac, argv[2]);
	}
	printf("RAW interact with the remote ");
	macaddr_print(g_peer_mac);
	printf("\r\n");

#if defined(BUILDING_PFRAWS_UDP)
	if (argc > 3) {
		strncpy(g_peer_ips, argv[3], sizeof(g_peer_ips) - 1);
	}
	printf("UDP tx->%s:%d\r\n", g_peer_ips, g_rru_txport);
	printf("UDP rx<-any:%d\r\n", g_rru_rxport);
#endif
}

/*---- leaf functions ----*/
void display_ethhdr(uint8_t *buf)
{
#ifdef BUILDING_PFRAWS_VLAN
	struct vlan_ethhdr *phdr = (struct vlan_ethhdr *)buf;
	printf("\r\n---- VLAN HEADER (%lu):\r\n", sizeof(*phdr));
	printf("DMAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", phdr->h_dest[0], phdr->h_dest[1],
		phdr->h_dest[2],phdr->h_dest[3],phdr->h_dest[4],phdr->h_dest[5]);
	printf("SMAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", phdr->h_source[0], phdr->h_source[1],
		phdr->h_source[2],phdr->h_source[3],phdr->h_source[4],phdr->h_source[5]);
	printf("VLAN_PROTO: 0x%04x\r\n", htons(phdr->h_vlan_proto));
	//printf("VLAN_TCI: 0x%04x\r\n", htons(phdr->h_vlan_TCI));
	printf("VLAN_PCP/DCI: 0x%x\r\n", htons(phdr->h_vlan_TCI >> 12));
	printf("VLAN_ID: 0x%03x\r\n", htons(phdr->h_vlan_TCI & 0x0FFF));
	printf("ENCAPSULATED_PROTO: 0x%04x\r\n", htons(phdr->h_vlan_encapsulated_proto));
#else
	struct ethhdr *phdr = (struct ethhdr *)buf;
	printf("\r\n---- RAW HEADER (%lu):\r\n", sizeof(*phdr));
	printf("DMAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", phdr->h_dest[0], phdr->h_dest[1],
		phdr->h_dest[2],phdr->h_dest[3],phdr->h_dest[4],phdr->h_dest[5]);
	printf("SMAC: %02x:%02x:%02x:%02x:%02x:%02x\r\n", phdr->h_source[0], phdr->h_source[1],
		phdr->h_source[2],phdr->h_source[3],phdr->h_source[4],phdr->h_source[5]);
	printf("TYPE: 0x%04x\r\n", htons(phdr->h_proto));
#endif
	fflush(stdout);
}

int pack_ethhdr(int fd, uint8_t *buf)
{
	int ret;
	struct ifreq req;
#ifdef BUILDING_PFRAWS_VLAN
	struct vlan_ethhdr *phdr = (struct vlan_ethhdr *)buf;
#else
	struct ethhdr *phdr = (struct ethhdr *)buf;
#endif

	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, str_devname, sizeof(req.ifr_name));
	ret = ioctl(fd, SIOCGIFHWADDR, &req);
	if (ret < 0) {
		perror("socket ioctl() SIOCGIFHWADDR");
		return ret;
	}
#ifdef BUILDING_PFRAWS_VLAN
	memcpy(phdr->h_source, req.ifr_hwaddr.sa_data, ETH_ALEN);
	memcpy(phdr->h_dest, g_peer_mac, ETH_ALEN);
	phdr->h_vlan_proto = htons(ETH_P_8021Q);
	phdr->h_vlan_TCI = htons(0xE << 12 | BBU_RRU_VLANID);
	phdr->h_vlan_encapsulated_proto = htons(ETH_P_IP);
#else
	memcpy(phdr->h_source, req.ifr_hwaddr.sa_data, ETH_ALEN);
	memcpy(phdr->h_dest, g_peer_mac, ETH_ALEN);
	phdr->h_proto = htons(ETH_P_IP);
#endif

	//display_ethhdr(buf);
	return ret;
}

int unpack_ethhdr(uint8_t *buf)
{
#ifdef BUILDING_PFRAWS_VLAN
	struct vlan_ethhdr *phdr = (struct vlan_ethhdr *)buf;
#else
	struct ethhdr *phdr = (struct ethhdr *)buf;
#endif

	//display_ethhdr(buf);

	/* match the expected source MAC? */
	if ((buf[6]  != g_peer_mac[0]) || (buf[7]  != g_peer_mac[1]) ||
		(buf[8]  != g_peer_mac[2]) || (buf[9]  != g_peer_mac[3]) ||
		(buf[10] != g_peer_mac[4]) || (buf[11] != g_peer_mac[5])) {
		return -1;
	}

	/* NOTICE:
	vlan header is removed by the Ethernet Device Driver
	even if the right vlan interface is created */
	if (
#ifdef BUILDING_PFRAWS_VLAN
		(phdr->h_vlan_proto != htons(ETH_P_8021Q)) || \
		(phdr->h_vlan_TCI != htons(0xE << 12 | BBU_RRU_VLANID)) || \
		(phdr->h_vlan_encapsulated_proto != htons(ETH_P_IP))
#else
		phdr->h_proto != htons(ETH_P_IP)
#endif
	) {
		return -2;
	}

	return 0;
}

void display_iphdr_udphdr(uint8_t *buf)
{
	fflush(stdout);
}

/*
RFC791: The checksum field is the 16 bit one's complement of the one's
  complement sum of all 16 bit words in the header. For purposes of 
  computing the checksum, the value of the checksum field is zero.
*/
static uint16_t ip_csum16(uint16_t *buf, size_t len)
{
    uint32_t sum = 0;
    uint16_t last = 0;

    while (len > 1) {
        sum += *buf++;
        len -= 2;
    }
    if (len == 1) {
        *(uint8_t *)(&last) = *(uint8_t *)buf;
        sum += last;
    }

    sum  = (sum >> 16) + (sum & 0xffff); /* add hi-16 to low-16 */
    sum += (sum >> 16);	/* add carry */
    return (uint16_t)(~sum); /* truncate to 16 bits */
}

int pack_iphdr_udphdr(int fd, uint8_t *buf, size_t l5len)
{
	int ret;
	struct ifreq req;
	struct udphdr *udph;
	struct pseudohdr *psh;
	struct iphdr *iph;

	ASSERT(fd != -1);
	ASSERT(buf != NULL);
	//ASSERT(l5len <= );

	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, str_devname, sizeof(req.ifr_name));
	ret = ioctl(fd, SIOCGIFADDR, &req);
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCGIFADDR");
		return -1;
	}

	/* fill udph.header.checksum AT FIRST!!!
	   then overwrite pseudo_header with the real ip_header */
	udph = (struct udphdr *)(buf + sizeof(struct iphdr));
#if defined(BUILDING_RRU_UL)
	udph->source = htons(g_rru_txport);
	udph->dest   = htons(g_bbu_rxport);
#else
	udph->source = htons(g_bbu_txport);
	udph->dest   = htons(g_rru_rxport);
#endif
	udph->len    = htons(sizeof(struct udphdr) + l5len);
#if 1
	udph->check  = 0;/* udph.checksum is optional for IPv4 */
#else
	/* RFC768, IPv4 Pseudo Header
	   https://en.wikipedia.org/wiki/User_Datagram_Protocol#Checksum_computation
	*/
	psh = (struct pseudohdr *)(buf + sizeof(struct iphdr) - sizeof(struct pseudohdr));
	psh->saddr = inet_addr(inet_ntoa(((struct sockaddr_in *)&req.ifr_addr)->sin_addr));
	psh->daddr = inet_addr(g_peer_ips);
	psh->zeros = 0;
	psh->protocol = htons(IPPROTO_UDP);
	psh->len = udph->len;
	/* calculating checksum for "pseudo.header + udph.header + udph.payload" */
	udph->check = ip_csum16((uint16_t *)psh, sizeof(struct pseudohdr) + UDP_HDR_LEN + l5len);
#endif

	/* fill iph.header */
	iph = (struct iphdr *)buf;
	iph->ihl = 5;
	iph->version = 4;
	iph->tos = 0;
	iph->tot_len = htons(sizeof(struct iphdr) + sizeof(struct udphdr) + l5len);
	iph->id = htons(0);				/* sequence number */
	iph->frag_off = htons(0);		/* no fragment */
	iph->ttl = 64;					/* default value */
	iph->protocol = IPPROTO_UDP;	/* protocol at L4 */
	iph->check = 0;/* zero before calculating checksum */
	iph->saddr = inet_addr(inet_ntoa(((struct sockaddr_in *)&req.ifr_addr)->sin_addr));
	iph->daddr = inet_addr(g_peer_ips);
	/* calculating checksum for "iph.header" only:
	it's a must! otherwise packets will be dropped */
	iph->check = ip_csum16((uint16_t *)buf, sizeof(struct iphdr));

	return 0;
}

int unpack_iphdr_udphdr(uint8_t *buf)
{
	struct udphdr *udp;
	struct pseudohdr *psh;
	struct iphdr *iph;
    uint16_t sub_type;
    int iswrong = 0;

	/* verify udp.header */
	udp = (struct udphdr *)(buf + sizeof(struct iphdr));
    udp->source = ntohs(udp->source);
	if (g_rru_rxport+1 != udp->source) iswrong = 1;
    udp->dest = ntohs(udp->dest);
	if (g_rru_rxport != udp->dest) iswrong = 1;
    udp->len = ntohs(udp->len);
    sub_type = *(uint16_t *)(buf +IP_HDR_LEN+UDP_HDR_LEN +6+18);
    sub_type = ntohs(sub_type);
#if defined(BUILDING_RRU_DL)
	if ((0x0000 == sub_type) && (sizeof(struct udphdr)+PDSCH_SIZE != udp->len))
        iswrong = 1;
#else
	if ((0x0000 == sub_type) && (sizeof(struct udphdr)+TMPPS_SIZE != udp->len))
        iswrong = 1;
	if ((0x0000 == sub_type) && sizeof(struct udphdr)+PUSCH_SIZE != udp->len)
        iswrong = 1;
#endif
#if 0
	/* fill udp.header.checksum
	https://en.wikipedia.org/wiki/User_Datagram_Protocol#Checksum_computation
	RFC768, IPv4 Pseudo Header */
	psh = (struct pseudohdr *)(buf + sizeof(struct iphdr) - sizeof(struct pseudohdr));
	psh->saddr ;
	psh->daddr ;
	psh->zeros ;
	psh->protocol = htons(IPPROTO_UDP);
	psh->len = udp->len;	
	/* calculating checksum for "pseudo.header + udp.header + udp.payload" */
	udp->check = ip_csum16((uint16_t *)psh, sizeof(struct pseudohdr) + sizeof(struct udphdr)+PUSCH_SIZE);
#endif
    if (iswrong) printf("wrong udp_hdr!\n");

	/* verify ip.header */
	iph = (struct iphdr *)buf;
	//iph->ihl = 5;
	//iph->tos;
	//iph->tot_len;
	//iph->id;
	//iph->frag_off;
	//iph->ttl;
	//iph->check;
	if (4 != iph->version) iswrong = 1;
    if (IPPROTO_UDP != iph->protocol) iswrong = 1;
    if (iph->saddr != inet_addr(g_peer_ips)) iswrong = 1;
    //if (iph->daddr != inet_addr(g_local_ips)) iswrong = 1;
    if (iswrong) printf("wrong ip_hdr!\n");

	return 0;
}

/* set sockaddr info then bind it with the socket */
int sock_bind_ipport(int fd, int tx_rx)
{
	int ret = 0;

#if defined(BUILDING_AFINETS_UDP)
	struct sockaddr_in ipme;
	bzero(&ipme, sizeof(ipme));
	ipme.sin_family = AF_INET;
# if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)
	ipme.sin_port = tx_rx? htons(g_rru_txport) : htons(g_rru_rxport);
# else
	ipme.sin_port = tx_rx? htons(g_bbu_txport) : htons(g_bbu_rxport);
# endif
	ipme.sin_addr.s_addr = htonl(INADDR_ANY);
	ret = bind(fd, (struct sockaddr *)&ipme, sizeof(ipme));
#else
# if defined(BUILDING_RAWS_WITH_BIND)
	struct sockaddr_ll llme;
	memset(&llme, 0, sizeof(llme));
	llme.sll_family = AF_PACKET;
#  if defined(BUILDING_PFRAWS_UDP)
	llme.sll_protocol = htons(ETH_P_IP);
#  elif defined(BUILDING_PFRAWS_VLAN)
	llme.sll_protocol = htons(ETH_P_8021Q);
#  else
	llme.sll_protocol = htons(ETH_P_802_3);
#  endif
	llme.sll_ifindex = if_nametoindex(str_devname);
	ret = bind(fd, (struct sockaddr *)&llme, sizeof(llme));
# endif
#endif
	if (-1 == ret) {
		perror("bind");
		return -1;;
	}
}

/* fill peer sockaddr */
#if defined(BUILDING_AFINETS_UDP)
struct sockaddr_in g_txaddrip;
#else
struct sockaddr_ll g_txaddrll;
#endif
int sock_fill_txaddr(void)
{
#if defined(BUILDING_AFINETS_UDP)
	bzero(&g_txaddrip, sizeof(g_txaddrip));
	g_txaddrip.sin_family = AF_INET;
# if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)
	g_txaddrip.sin_port   = htons(g_bbu_rxport);
# else
	g_txaddrip.sin_port   = htons(g_rru_rxport);
# endif
	if (inet_aton(g_peer_ips, &(g_txaddrip.sin_addr)) == 0) {
		perror("inet_aton()");
		return -1;
	}
	//printf("UDP TX -> %s:%d\r\n", inet_ntoa(g_txaddrip.sin_addr), ntohs(g_txaddrip.sin_port));
#else
	bzero(&g_txaddrll, sizeof(g_txaddrll));
	g_txaddrll.sll_ifindex  = if_nametoindex(str_devname);
	g_txaddrll.sll_family   = AF_PACKET;
	g_txaddrll.sll_protocol = htons(ETH_P_IP);
# if defined(BUILDING_PFRAWS_UDP)
	g_txaddrll.sll_halen    = ETH_ALEN;
	memcpy(&g_txaddrll.sll_addr, g_peer_mac, ETH_ALEN);
# endif
#endif
	return 0;
}

