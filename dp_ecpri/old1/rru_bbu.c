#include <stdbool.h>
#include <stdint.h>
#include <stddef.h> /* size_t, offsetof(type, member) */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h> /* GNU getopt() */

#include <errno.h>
#include <fcntl.h>
//#include <sys/types.h>
//#include <sys/mman.h>
//#include <sys/stat.h>
#include <sys/socket.h>	/* socket() */
#include <sys/ioctl.h>	/* ioctl() */
#include <arpa/inet.h>	/* htons/htonl() */

#include "common_jacky.h"
#include "rru_bbu.h"

extern int is_iiorxdma_overflow(int idev);
extern int is_iiotxdma_underflow(int idev);

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

uint32_t pusch_tx_cnt_symbl;
uint32_t pusch_rx_cnt;

uint32_t pdsch_tx_cnt_symbl;
uint32_t pdsch_rx_cnt;

uint32_t iio_rx_overflow;
uint32_t iio_tx_underflow;
uint32_t iio_tx_overflow;

uint32_t raws_txbuf_overflow;
uint32_t raws_rxbuf_underflow;
uint32_t raws_rxbuf_overflow;
uint32_t tmpps_tx_cnt;
uint32_t tmpps_rx_cnt;

void clear_counters(void)
{
	pusch_tx_cnt_symbl = 0;
	pusch_rx_cnt = 0;
	
	pdsch_tx_cnt_symbl = 0;
	pdsch_rx_cnt = 0;
	
	iio_rx_overflow = 0;
	iio_tx_underflow = 0;
	raws_txbuf_overflow = 0;
	raws_rxbuf_underflow = 0;
	raws_rxbuf_overflow = 0;
	tmpps_tx_cnt = 0;
	tmpps_rx_cnt = 0;
}

#if 0
void display_tx_counters(void)
{
#if defined(BUILDING_BBU_DL)
	printf("\rntb_of=%10u, tx_pdsch=%10u", 
		 raws_txbuf_overflow, pdsch_tx_cnt_symbl);
	if (raws_txbuf_overflow)
		printf("\r\n");
#endif
#if defined(BUILDING_RRU_UL)
	printf("\rirb_of=%10u, ntb_of=%10u, tx_pusch=%10u; tm_tx=%10u, tm_rx=%10u", 
		iio_rx_overflow, raws_txbuf_overflow, pusch_tx_cnt_symbl, 
		tmpps_tx_cnt, tmpps_rx_cnt);
	if (iio_rx_overflow || raws_txbuf_overflow)
		printf("\r\n");
#endif
}

void display_rx_counters(void)
{
#if defined(BUILDING_BBU_UL)
	printf("\rnrb_uf=%10u, nrb_of=%10u, rx_pusch=%10u, rx_tmpps=%10u", 
		 raws_rxbuf_overflow, raws_rxbuf_overflow, pusch_rx_cnt, tmpps_rx_cnt);
	if (raws_rxbuf_overflow)
		printf("\r\n");
	fflush(stdout);
#endif
#if defined(BUILDING_RRU_DL)
	printf("\rnrb_uf=%10u, nrb_of=%10u, rx_pdsch=%10u, itb_uf=%10u", 
		raws_rxbuf_underflow, raws_rxbuf_overflow, pdsch_rx_cnt, iio_tx_underflow);
	if (raws_rxbuf_overflow)
		printf("\r\n");
#endif
}
#else
void display_tx_counters(void)
{
#if defined(BUILDING_BBU_DL)
	printf("\rntb_of=%u, tx_pdsch=%u", 
		 raws_txbuf_overflow, pdsch_tx_cnt_symbl);
	if (raws_txbuf_overflow)
		printf("\r\n");
#endif
#if defined(BUILDING_RRU_UL)
	printf("\rirb_of=%u, ntb_of=%u, tx_pusch=%u; tm_tx=%u, tm_rx=%u", 
		iio_rx_overflow, raws_txbuf_overflow, pusch_tx_cnt_symbl, 
		tmpps_tx_cnt, tmpps_rx_cnt);
	if (iio_rx_overflow || raws_txbuf_overflow)
		printf("\r\n");
#endif
	fflush(stdout);
}

void display_rx_counters(void)
{
#if defined(BUILDING_BBU_UL)
	printf("\rnrb_uf=%u, nrb_of=%u, rx_pusch=%u, rx_tmpps=%u", 
		 raws_rxbuf_overflow, raws_rxbuf_overflow, pusch_rx_cnt, tmpps_rx_cnt);
	if (raws_rxbuf_overflow)
		printf("\r\n");
#endif
#if defined(BUILDING_RRU_DL)
	printf("\rnrb_uf=%u, nrb_of=%u, rx_pdsch=%u, itb_uf=%u", 
		raws_rxbuf_underflow, raws_rxbuf_overflow, pdsch_rx_cnt, iio_tx_underflow);
	if (raws_rxbuf_overflow)
		printf("\r\n");
#endif
	fflush(stdout);
}

void display_counters(void)
{
#if defined(BUILDING_BBU_DL) || defined(BUILDING_BBU_UL)/*
	if (raws_rxbuf_overflow || raws_txbuf_overflow || raws_rxbuf_underflow)
		printf("\r\nntb_of=%u, tx_pdsch=%u; "
			"nrb_uf=%u, nrb_of=%u, rx_pusch=%u, rx_tmpps=%u", 
			 raws_txbuf_overflow, pdsch_tx_cnt_symbl,
			 raws_rxbuf_underflow, raws_rxbuf_overflow, pusch_rx_cnt, tmpps_rx_cnt);
	else if (0 == pdsch_tx_cnt_symbl % 70)*/
		printf("\rtx_pdsch=%u; rx_pusch=%u, rx_tmpps=%u", 
			 pdsch_tx_cnt_symbl,
			 pusch_rx_cnt, tmpps_rx_cnt);
#endif
#if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)/*
	if (iio_rx_overflow || raws_txbuf_overflow || raws_rxbuf_overflow)
		printf("\rtm_tx=%u, tm_rx=%u; "
			"irb_of=%u, ntb_of=%u, tx_pusch=%u; "
			"nrb_uf=%u, nrb_of=%u, rx_pdsch=%u, itb_uf=%u", 
			tmpps_tx_cnt, tmpps_rx_cnt, 
			iio_rx_overflow, raws_txbuf_overflow, pusch_tx_cnt_symbl, 
			raws_rxbuf_underflow, raws_rxbuf_overflow, pdsch_rx_cnt, iio_tx_underflow);
	else if (0 == pusch_tx_cnt_symbl % 70)*/
		printf("\rtm_tx=%u, tm_rx=%u; "
			"tx_pusch=%u; "
			"rx_pdsch=%u, itb_uf=%u", 
			tmpps_tx_cnt, tmpps_rx_cnt, 
			pusch_tx_cnt_symbl, 
			pdsch_rx_cnt, iio_tx_underflow);
#endif
	fflush(stdout);
}

#endif

void display_tx_perf(struct timespec *e, struct timespec *s)
{
	double tm_us = elapse_us(e, s);
#if defined(BUILDING_BBU_DL)
	if (pdsch_tx_cnt_symbl > 0) {
	printf("\r\n#### TX PDSCH %u in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\r\n",
		pdsch_tx_cnt_symbl, tm_us,
		pdsch_tx_cnt_symbl/tm_us*PDSCH_UBUF_LEN*8, tm_us/pdsch_tx_cnt_symbl);
	}
#endif
#if defined(BUILDING_RRU_UL)
	if (pusch_tx_cnt_symbl > 0) {
	printf("\r\n#### TX PUSCH %u in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\r\n",
		pusch_tx_cnt_symbl, tm_us,
		pusch_tx_cnt_symbl/tm_us*PUSCH_UBUF_LEN*8, tm_us/pusch_tx_cnt_symbl);
	}
#endif
}

void display_rx_perf(struct timespec *e, struct timespec *s)
{
	double tm_us = elapse_us(e, s);
#if defined(BUILDING_BBU_UL)
	if (pusch_rx_cnt > 0) {
	printf("\r\n#### RX PUSCH %u in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\r\n",
		pusch_rx_cnt, tm_us,
		pusch_rx_cnt/tm_us*PUSCH_UBUF_LEN*8, tm_us/pusch_rx_cnt);
	}
#endif
#if defined(BUILDING_RRU_DL)
	if (pdsch_rx_cnt > 0) {
	printf("\r\n#### RX PDSCH %u in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\r\n",
		pdsch_rx_cnt, tm_us,
		pdsch_rx_cnt/tm_us*PDSCH_UBUF_LEN*8, tm_us/pdsch_rx_cnt);
	}
#endif
}

/*----------------------------------
*/

void display_pusch(uint8_t *buf)
{
	int iport, isample;
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
	printf("ant1_gain[15:0]=0x%04x\r\n", *(int16_t *)(&(ph->gain[1])));/*
	for (iport = 0; iport < PORT_NUM_TOTAL; iport++) {
		printf("---- PUSCH PAYLOAD: I/Q pairs for Port%d (%lu):\r\n", iport, sizeof(*pl)/PORT_NUM_TOTAL);
		for (isample = 0; isample < SUBCARRIER_SAMPLES; isample++) {
			printf("%02x %02x ", (uint8_t)(pl->port[iport][isample].i16), 
				(uint8_t)(pl->port[iport][isample].q16));
		}
		printf("\r\n");
	}*/

	fflush(stdout);
}

static inline void hton_pusch(uint8_t *buf)
{
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pusch_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = htons(ph->sub_type);

	//*p32 = htonl(*p32);//1st transfer
	p32++;//2nd transfer
	*p32 = htonl(*p32);
	p32++;//3rd transfer
	*p32 = htonl(*p32);
	p32++;//4th transfer
	*p32 = htonl(*p32);
	p32++;//5th transfer
	*p32 = htonl(*p32);
	p32++;//6th transfer
	*p32 = htonl(*p32);
	p32++;//7th transfer
	*p32 = htonl(*p32);
}

void fill_pusch_pld(uint8_t *buf)
{
	int iport, isample;
	struct bbu_payload *pl = (struct bbu_payload *)buf;
	//struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pusch_hdr));

	for (iport = 0; iport < PORT_NUM_TOTAL; iport++) {
		for(isample = 0; isample < SUBCARRIER_SAMPLES; isample++) {
			pl->iqs[iport][isample].i8 = 0x01*(2*iport+1);
			pl->iqs[iport][isample].q8 = 0x01*(2*iport+2);
		}
	}
}

void pack_pusch_hdr(uint8_t *buf)
{
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;

	memset(ph, 0, sizeof(struct pusch_hdr));

	ph->sub_type = PUSCH_SUBTYPE;

	ph->special = 0;
	ph->rb_seq  = 100;	// RB=100 when using 20MHz bandwidth

	ph->ant_cnt = PORT_NUM_TOTAL;
	ph->ant_start = 0;
	ph->frame_seq = (pusch_tx_cnt_symbl/140) & 0x3ff;
	ph->subf_seq = (pusch_tx_cnt_symbl/14) % 10;
	ph->sym_seq = pusch_tx_cnt_symbl % 14;

	//ph->ant0_gain = htons(0);
	//ph->ant1_gain = htons(0);

	display_pusch(buf);
	hton_pusch(buf);
	//display_pusch(buf);
}

static inline void ntoh_pusch(uint8_t *buf)
{
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pusch_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = ntohs(ph->sub_type);

	//*p32 = ntohl(*p32);//1st transfer
	p32++;//2nd transfer
	*p32 = ntohl(*p32);
	p32++;//3rd transfer
	*p32 = ntohl(*p32);
	p32++;//4th transfer
	*p32 = ntohl(*p32);
	p32++;//5th transfer
	*p32 = ntohl(*p32);
	p32++;//6th transfer
	*p32 = ntohl(*p32);
	p32++;//7th transfer
	*p32 = ntohl(*p32);
}

#if defined(BUILDING_RRU_PUSCH_B2B)
extern int datapath_handler_tx(int idev);
#endif
void unpack_pusch(uint8_t *buf)
{
	struct pusch_hdr *ph = (struct pusch_hdr *)buf;
	//struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pusch_hdr));

	pusch_rx_cnt++;

	//display_pusch(buf); //to debug before byte-swap
	ntoh_pusch(buf);
	display_pusch(buf);

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
	int iport, isample;
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
	printf("sym_seq[3:0]=0x%x\r\n", ph->sym_seq);/*
	for (iport = 0; iport < PORT_NUM_TOTAL; iport++) {
		printf("---- PDSCH PAYLOAD: I/Q pairs for Port%d (%lu):\r\n", iport, sizeof(*pl)/PORT_NUM_TOTAL);
		for (isample = 0; isample < SUBCARRIER_SAMPLES; isample++) {
			printf("%02x %02x ", (uint8_t)(pl->port[iport][isample].i16), 
				(uint8_t)(pl->port[iport][isample].q16));
		}
		printf("\r\n");
	}*/
	
	fflush(stdout);
}

static inline void ntoh_pdsch(uint8_t *buf)
{
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pdsch_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = ntohs(ph->sub_type);

	//*p32 = ntohl(*p32);//1st transfer
	p32++;//2nd transfer
	*p32 = ntohl(*p32);
	p32++;//3rd transfer
	*p32 = ntohl(*p32);
}

void unpack_pdsch(uint8_t *buf)
{
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;

	pdsch_rx_cnt++;

	//display_pdsch(buf); //to debug before byte-swap
	ntoh_pdsch(buf);
	display_pdsch(buf);
}

static inline void hton_pdsch(uint8_t *buf)
{
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct pdsch_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = htons(ph->sub_type);

	//*p32 = htonl(*p32);//1st transfer
	p32++;//2nd transfer
	*p32 = htonl(*p32);
	p32++;//3rd transfer
	*p32 = htonl(*p32);
}

void fill_pdsch_pld(uint8_t *buf)
{
	int iport, isample;
	struct bbu_payload *pl = (struct bbu_payload *)buf;
	//struct bbu_payload *pl = (struct bbu_payload *)(buf + sizeof(struct pdsch_hdr));

	for (iport = 0; iport < PORT_NUM_TOTAL; iport++) {
		for(isample = 0; isample < SUBCARRIER_SAMPLES; isample++) {
			pl->iqs[iport][isample].i8 = 0x10*(2*iport+1);
			pl->iqs[iport][isample].q8 = 0x10*(2*iport+1);
		}
	}
}

void pack_pdsch_hdr(uint8_t *buf)
{
	struct pdsch_hdr *ph = (struct pdsch_hdr *)buf;

	memset(ph, 0, sizeof(struct pdsch_hdr));

	ph->sub_type = PDSCH_SUBTYPE;
	
	ph->special = 0;	//TODO: when should be 1?
	ph->rb_sel = 0;
	ph->rb_seq_h = (100 >> 2) & 0x7c; //[4:0] High part of number of RB (25,50,75,100)

	ph->rb_seq_l = 100 & 0x03;
	ph->ant_cnt = PORT_NUM_TOTAL;
	ph->ant_start = 0;
	ph->frame_seq = (pdsch_tx_cnt_symbl/140) & 0x3ff;
	ph->subf_seq = (pdsch_tx_cnt_symbl/14) % 10;
	ph->sym_seq = pdsch_tx_cnt_symbl % 14;

	display_pdsch(buf);
	hton_pdsch(buf);
	//display_pdsch(buf);
}

/*----------------------------------
*/

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

	fflush(stdout);
}

static inline void hton_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct tmpps_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = htons(ph->sub_type);

	*p32 = htonl(*p32);//1st transfer
}

void pack_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;

	memset(ph, 0, sizeof(struct tmpps_hdr));

	ph->sub_type = TMPPS_SUBTYPE;

	// -- UL iio_rxdma overflow
	//	definition: rx_fifo shift new input samples in before kernel iio_buffer <- rx_dma <- rx_fifo
	//	 judgement: jesd/iio_txdma register 0x80000088
	// -- UL libiio2rawsock overflow
	//	definition: new data overwrite socktxbuf[] before sendto() finish
	//	 judgement: impossible by current design
	if (1 == is_iiorxdma_overflow(0)) iio_rx_overflow++;
	ph->ul_bof_flg = iio_rx_overflow | raws_txbuf_overflow;

	// -- DL iio_rxdma underflow
	//	definition: kernel iio_buffer -> tx_dma -> tx_fifo before tx_fifo shift new output samples out
	//	 judgement: jesd/iio_rxdma register 0x80000088
	// -- DL libiio2rawsock underflow
	//	definition: new data overwrite sockrxbuf[] before recvfrom() finish
	//	 judgement: impossible by current design
	iio_tx_underflow = is_iiotxdma_underflow(0);
	ph->dl_buf_flg = iio_tx_underflow | raws_rxbuf_underflow;

	ph->dl_bof_flg = raws_rxbuf_overflow > 0 ? 1 : 0;
	ph->dl_bof_cnt = raws_rxbuf_overflow & 0xff;

	ph->symbol_sync = 1; //[16]	0-1	1 to indicate the BBU to RRU symbol packets are in sync
#if 0
	ph->frame_seq = (pusch_tx_cnt_symbl/140) & 0x3ff; //[15:4]	0-1023	Radio frame number
	//ph->subf_seq = (pusch_tx_cnt_symbl/14)%10; //[3:0]	0-9	Subframe number
	ph->subf_seq = (pusch_tx_cnt_symbl/14)%10 + 1;
#else
	ph->frame_seq = (tmpps_tx_cnt+1)/10;
	ph->subf_seq = (tmpps_tx_cnt+1)%10;
#endif
	display_tmpps(buf);
	hton_tmpps(buf);
}

static inline void ntoh_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;
	int32_t *p32 = (int32_t *)(buf + offsetof(struct tmpps_hdr, sub_type) + sizeof(ph->sub_type));

	ph->sub_type = ntohs(ph->sub_type);

	*p32 = ntohl(*p32);//1st transfer
}

void unpack_tmpps(uint8_t *buf)
{
	struct tmpps_hdr *ph = (struct tmpps_hdr *)buf;

	tmpps_rx_cnt++;

	//display_tmpps(buf); //to debug before byte-swap
	ntoh_tmpps(buf);
	display_tmpps(buf);
}

/*---- globals ----*/
#if 0
char    str_devname[32] = "eth0";
#else
extern char * str_devname;
#endif
#if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)
uint8_t g_peer_mac[6] = { 0xa0, 0x36, 0x9f, 0x42, 0x74, 0x90 };
char    g_peer_ips[16] = "10.88.120.122";
short   g_rxport = 2153;
short   g_txport = 2152;
#else
uint8_t g_peer_mac[6] = { 0x1C, 0x76, 0xCA, 0x80, 0x80, 0x80 };
char    g_peer_ips[16] = "10.88.120.133";
short   g_rxport = 2152;
short   g_txport = 2153;
#endif

/*---- leaf functions ----*/
static void display_ethhdr(uint8_t *buf)
{
#ifdef BUILDING_RAWS_VLAN
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

int raws_pack_ethhdr(int fd, uint8_t *buf)
{
	int ret;
	struct ifreq req;
#ifdef BUILDING_RAWS_VLAN
	struct vlan_ethhdr *phdr = (struct vlan_ethhdr *)buf;
#else
	struct ethhdr *phdr = (struct ethhdr *)buf;
#endif

	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, str_devname, sizeof(req.ifr_name));
	ret = ioctl(fd, SIOCGIFHWADDR, &req);
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCGIFHWADDR");
		return ret;
	}
#ifdef BUILDING_RAWS_VLAN
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
	display_ethhdr(buf);

	return ret;
}

int raws_unpack_ethhdr(uint8_t *buf)
{
#ifdef BUILDING_RAWS_VLAN
	struct vlan_ethhdr *phdr = (struct vlan_ethhdr *)buf;
#else
	struct ethhdr *phdr = (struct ethhdr *)buf;
#endif

	display_ethhdr(buf);

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
#ifdef BUILDING_RAWS_VLAN
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

void display_iphdr_udphdr(uint8_t *buf, size_t l5len)
{
	fflush(stdout);
}

/*
RFC791: The checksum field is the 16 bit one's complement of the one's
  complement sum of all 16 bit words in the header. For purposes of 
  computing the checksum, the value of the checksum field is zero.
*/
static uint16_t calcuate_checksum(uint16_t *buf, size_t len)
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
	struct pseudoiphdr *psh;
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

	/* fill udph.header.checksum AT FIRST */
	udph = (struct udphdr *)(buf + sizeof(struct iphdr));
	udph->source = htons(g_txport - 1);
	udph->dest = htons(g_txport);
	udph->len = htons(sizeof(struct udphdr) + l5len);
#if 1
	udph->check = 0;/* udph.checksum is optional for IPv4 */
#else
	/* RFC768, IPv4 Pseudo Header
	   https://en.wikipedia.org/wiki/User_Datagram_Protocol#Checksum_computation
	*/
	psh = (struct pseudoiphdr *)(buf + sizeof(struct iphdr) - sizeof(struct pseudoiphdr));
	psh->saddr = inet_addr(inet_ntoa(((struct sockaddr_in *)&req.ifr_addr)->sin_addr));
	psh->daddr = inet_addr(g_peer_ips);
	psh->zeros = 0;
	psh->protocol = htons(IPPROTO_UDP);
	psh->len = udph->len;
	/* calculating checksum for "pseudo.header + udph.header + udph.payload" */
	udph->check = calcuate_checksum((uint16_t *)psh, 
		sizeof(struct pseudoiphdr) + UDP_HDR_LEN + l5len);
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
#if 1
	/* calculating checksum for "iph.header" only */
	//iph->check = ip_fast_csum(ip, iph->ihl);
	iph->check = calcuate_checksum((uint16_t *)buf, sizeof(struct iphdr));
#else
	/* calculating checksum for "iph.header + iph.payload" */
	iph->check = calcuate_checksum((uint16_t *)buf, sizeof(struct iphdr) + UDP_HDR_LEN + l5len);
#endif

	return 0;
}

int unpack_iphdr_udphdr(uint8_t *buf)
{
#if 1
	struct udphdr *udp;
	struct pseudoiphdr *psh;
	struct iphdr *iph;

	/* fill udp.header */
	udp = (struct udphdr *)(buf + sizeof(struct iphdr));
	ASSERT(htons(g_rxport) == udp->source);
	ASSERT(htons(g_rxport) == udp->dest);
	ASSERT(htons(sizeof(struct udphdr) + PUSCH_SIZE) == udp->len);
	/* fill udp.header.checksum
	https://en.wikipedia.org/wiki/User_Datagram_Protocol#Checksum_computation
	RFC768, IPv4 Pseudo Header */
#if 1
	//udp->check;/* udp.checksum is optional for IPv4 */
#else
	psh = (struct pseudoiphdr *)(buf + sizeof(struct iphdr) - sizeof(struct pseudoiphdr));
	psh->saddr ;
	psh->daddr ;
	psh->zeros ;
	psh->protocol = htons(IPPROTO_UDP);
	psh->len = udp->len;	
	/* calculating checksum for "pseudo.header + udp.header + udp.payload" */
	udp->check = calcuate_checksum((uint16_t *)psh, 
		sizeof(struct pseudoiphdr) + sizeof(struct udphdr) + PUSCH_SIZE);
#endif
	/* fill ip.header */
	iph = (struct iphdr *)buf;
	//iph->ihl = 5;
	ASSERT(4 == iph->version);
	//iph->tos;
	//iph->tot_len;
	//iph->id;
	//iph->frag_off;
	//iph->ttl;
	ASSERT(IPPROTO_UDP == iph->protocol);
	ASSERT(iph->saddr == inet_addr(g_peer_ips));
	//ASSERT(iph->daddr);	
	//iph->check;
#endif
	return 0;
}

