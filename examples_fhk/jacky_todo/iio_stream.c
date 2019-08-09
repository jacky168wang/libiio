/*
 * applications based on libiio
 *   - RFIC IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Jacky Wang <kenwj@sina.com>
 *
 * License: GPL, version 2.1
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h> /* size_t, offsetof(type, member) */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h> /* GNU getopt() */

#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif
#include "common_jacky.h"
#include "common_libiio.h"
//#include "rru_bbu.h"

/*
------------------------------------------------------------------------------*/

/* IIO structs required for streaming */
static struct iio_context *ctx;

// Streaming devices
static struct iio_device *iio_rxdev[TOT_CHIP_NB];
static struct iio_device *iio_txdev[TOT_CHIP_NB];
static struct iio_device *ordev[TOT_CHIP_NB];

// one libiio buffer for each device
       struct iio_buffer *iio_rxbuf[TOT_CHIP_NB];
       struct iio_buffer *iio_txbuf[TOT_CHIP_NB];
       struct iio_buffer *iio_orbuf[TOT_CHIP_NB];

// sample counters per device
static size_t orsnb[TOT_CHIP_NB];
static size_t rxsnb[TOT_CHIP_NB];
static size_t txsnb[TOT_CHIP_NB];

// 2x2 Streaming channels (2 Streaming ports) for each device
struct iio_channel *iiopr_chni[TOT_PORT_NB];
struct iio_channel *iiopr_chnq[TOT_PORT_NB];
struct iio_channel *iiopt_chni[TOT_PORT_NB];
struct iio_channel *iiopt_chnq[TOT_PORT_NB];

/*
------------------------------------------------------------------------------*/

#define EN_IIO_PATH_RX BIT(RX)
#define EN_IIO_PATH_TX BIT(TX)
       unsigned int g_pth_msk = EN_IIO_PATH_TX | EN_IIO_PATH_RX;
static unsigned int g_m_verbo = 0;

static unsigned int g_prt_msk = BIT(0); // 2T2R
//static unsigned int g_prt_msk = BIT(0) | BIT(1); // 4T4R
//static unsigned int g_prt_msk = BIT(0) | BIT(1) | BIT(2) | BIT(3); // 8T8R

static unsigned int g_rsmp_nb = IIOPR_SMP_NB_NOW;
static unsigned int g_dst_way = 0;
static FILE *g_wfd[TOT_PORT_NB];

static unsigned int g_tsmp_nb = IIOPT_SMP_NB_NOW;
static unsigned int g_word_pn = 0xffff;
static unsigned int g_src_way = 0;
static FILE *g_rfd[TOT_PORT_NB];

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION]\r\n"
		" -h\tshow this help\r\n"
		" -v\tbe verbose [0,1,2,3]\r\n"
		" -x\tbit-mask of the datapath to be enabled\r\n"
		"   \t\t0) disable iio_rxpath & iio_txpath\r\n"
		"   \t\t1) enable iio_rxpath only\r\n"
		"   \t\t2) enable iio_txpath only\r\n"
		"   \t\t3) enable iio_rxpath and iio_txpath\r\n"
		" -c\tbit-mask of the channels to be enabled\r\n"
		"   \t\t1) disable all channels\r\n"
		"   \t\t1) enable channel1 only\r\n"
		"   \t\t2) enable channel2 only\r\n"
		"   \t\t3) enable channel2&1\r\n"
		"   \t\t4) enable channel3 only\r\n"
		"   \t\t5) enable channel3&1\r\n"
		"   \t\t6) enable channel3&2\r\n"
		"   \t\t7) enable channel3&2&1\r\n"
		"   \t\t......\r\n"
		" ==== Options for iio_rxpath only"
		" -m\tset samples number [4~1048576] for iio_rxpath/rxbuf each time\r\n"
		" -d\tselect one way to deal with Destination samples\r\n"
		"   \t\t0) fwrite to one file named rxsmp[channel-mask]\r\n"
		"   \t\t1) memcpy to one user buffer\r\n"
		"   \t\t2) call iio_channel_read_raw()\r\n"
		"   \t\t3) call iio_channel_read()\r\n"
		"   \t\t4) foreach the received sample frankly\r\n"
		"   \t\t5) foreach the received sample with callback()\r\n"
		" ==== Options for iio_txpath only"
		" -n\tset samples number [4~1048576] for iio_txpath/txbuf each time\r\n"
		" -p\tone sample(16-bit) pattern to be filled\r\n"
		" -s\tselect one way to deal with Source samples\r\n"
		"   \t\t0) fread from one file named txsmp[channel-mask]\r\n"
		"   \t\t1) memcpy from one user buffer\r\n"
		"   \t\t2) call iio_channel_write_raw()\r\n"
		"   \t\t3) call iio_channel_write()\r\n"
		"   \t\t4) foreach the specified sample pattern frankly\r\n"
		"   \t\t5) foreach the specified sample pattern with callback()\r\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
static void get_args(int argc, char **argv)
{
	int c;
	while ((c = getopt(argc, argv, "x:p:m:d:n:p:s:v:h")) != EOF) {
		switch (c) {
		case 'x': g_pth_msk = strtoul(optarg, NULL, 16);			break;
		case 'p': g_prt_msk = strtoul(optarg, NULL, 16);			break;
		case 'm': g_rsmp_nb = strtoul(optarg, NULL,  0);			break;
		case 'd': g_dst_way = strtoul(optarg, NULL,  0);			break;
		case 'n': g_tsmp_nb = strtoul(optarg, NULL,  0);			break;
		case 'w': g_word_pn = strtoul(optarg, NULL, 16);			break;
		case 's': g_src_way = strtoul(optarg, NULL,  0);			break;
		case 'v': g_m_verbo = strtoul(optarg, NULL,  0);			break;
		default : usage(); exit(EXIT_FAILURE);						break;
		}
	}
#if 0
	/* take first residual non option argv element as interface name. */
	if ( optind < argc ) {
		str_devname = argv[ optind ];
	}
	if( !str_devname ) {
		fprintf( stderr, "ERROR: No interface was specified\r\n");
		usage();
		exit( EXIT_FAILURE );
	}
#endif
	printf( "CURRENT SETTINGS:\r\n" );
	printf( "g_m_verbo(-v):   %u\r\n", g_m_verbo );
	printf( "g_pth_msk(-x):   %u\r\n", g_pth_msk );
	printf( "g_prt_msk(-x):   %u\r\n", g_prt_msk );
	printf( "g_rsmp_nb(-m):   %u\r\n", g_rsmp_nb );
	printf( "g_dst_way(-d):   %u\r\n", g_dst_way );
	printf( "g_tsmp_nb(-n):   %u\r\n", g_tsmp_nb );
	printf( "g_word_pn(-p):   %u\r\n", g_word_pn );
	printf( "g_src_way(-s):   %u\r\n", g_src_way );
	printf( "\r\n" );
}

/*
------------------------------------------------------------------------------*/
static volatile sig_atomic_t stop = 0;
static void handle_sig(int sig)
{
	printf("\r\nWaiting for process to finish...\r\n");
	stop = 1;
}

/* finds datalink IIO devices */
static struct iio_device *get_dlk_dev(struct iio_context *ctx, enum iopath p, int ig)
{
	struct iio_device *dev;
	const char *devname_dlk[] = {
#if 0
        "axi-ad9371-rx-obs-hpc",
        "axi-ad9371-tx-hpc",
        "axi-ad9371-rx-hpc",
#else
        "axi-adrv9009-rx-obs-hpc",
        "axi-adrv9009-tx-hpc",
        "axi-adrv9009-rx-hpc",
#endif
	};

	if (ig == 0) {
		dev = iio_context_find_device(ctx, devname_dlk[path]);
	} else {
		char tmpstr[64];
		sscanf(tmpstr, "%s-n%d", devname_dlk, ig+1);
		dev = iio_context_find_device(ctx, tmpstr);
	}
	ASSERT((NULL != dev) && "No iio-device found");
	return dev;
}

/* finds datalink IIO channels */
static struct iio_channel *get_dlk_chn_ss(struct iio_device *dev, enum iopath p, int ip, char modify)
{
	struct iio_channel *chn;

	if (g_m_verbo > 3) printf("'DLK-%s' Acquiring channel-%s-%d ... ", dev->name, str_path[p], ip);
#if 0
	*chn = iio_device_find_channel(dev, GET_CHN_NAME("voltage", ip, modify), p == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, GET_CHN_NAME("voltage", ip, modify), p == TX);
#else
	chn = iio_device_find_channel(dev, GET_CHN_NAME("voltage", ip, modify), p == TX);
#endif
	ASSERT((NULL != chn) && "No iio-channel found");
	if (g_m_verbo > 3) printf("Succeed\r\n");
	return chn;
}

/* JESD_RX(iio:device4) */
	/* JESD_RX[0/1]_I(iio:device4/in_voltage[0/1]_i_xxx) */
	/* JESD_RX[0/1]_Q(iio:device4/in_voltage[0/1]_q_xxx) */
int init_dlk_rxpath(int ig, size_t smp_nb)
{
	int ip;
	
	if (g_m_verbo > 2) printf("'DLK%d' Acquiring iio_rxdev ... ", ig);
	iio_rxdev[ig] = get_dlk_dev(ctx, RX, ig);
	ASSERT((NULL != iio_rxdev[ig]) && "No iio_rxdev found");
	if (g_m_verbo > 2) printf("Succeed\r\n");

	// Clear all DMA status bits
	iio_device_reg_write(iio_rxdev[ig], 0x80000088, 0x6);

	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		if (g_prt_msk & (1 << ip)) {
			if (0 == g_dst_way) {
				if (g_m_verbo > 2) printf("'DLK%d' Create rxfiles for RX-port%d ... ", ig, ip);
				char str_rxfile[16];
				sscanf(str_rxfile, "rxsmp%d", ip);
				g_wfd[ig*DEV_PORT_NB + ip] = fopen(str_rxfile, "wb");
				ASSERT(g_wfd[ig*DEV_PORT_NB+ip] && "fopen g_wfd failed");
				if (g_m_verbo > 2) printf("Succeed\r\n");
			}

			if (g_m_verbo > 2) printf("'DLK%d' Enable i/q channels of RX-port%d ... ", ig, ip);
			iiopr_chni[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(iio_rxdev[ig], RX, ip, 'i');
			ASSERT(iiopr_chni[ig*DEV_PORT_NB+ip] && "No rxdev_p0i found");
			iiopr_chnq[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(iio_rxdev[ig], RX, ip, 'q');
			ASSERT(iiopr_chnq[ig*DEV_PORT_NB+ip] && "No rxdev_p0q found");
			iio_channel_enable(iiopr_chni[ig*DEV_PORT_NB + ip]);
			iio_channel_enable(iiopr_chnq[ig*DEV_PORT_NB + ip]);
			if (g_m_verbo > 2) printf("Succeed\r\n");
		}
	}

    if (g_m_verbo > 2) printf("'DLK%d' Creating iio_rxbuf with %u samples ... ", ig, smp_nb);
	iio_rxbuf[ig] = iio_device_create_buffer(iio_rxdev[ig], smp_nb, false);
	ASSERT((NULL != iio_rxbuf[ig]) && "Create iio_rxbuf failed");
	if (g_m_verbo > 2) printf("Succeed\r\n");

	ASSERT(4*DEV_PORT_NB <= iio_device_get_sample_size(iio_rxdev[ig]));
	return 0;
}

/* JESD_TX(iio:device3)*/
/* JESD_TX[0/1]_I(iio:device3/out_voltage[0/2]_xxx)*/
/* JESD_TX[0/1]_Q(iio:device3/out_voltage[1/3]_xxx)*/
int init_dlk_txpath(int ig, size_t smp_nb)
{
	int ip;

	if (g_m_verbo > 2) printf("'DLK%d' Acquiring iio_txdev ... ", ig);
	iio_txdev[ig] = get_dlk_dev(ctx, TX, ig);
	ASSERT(iio_txdev[ig] && "No iio_txdev found");
	if (g_m_verbo > 2) printf("Succeed\r\n");

	// Clear all DMA status bits
	iio_device_reg_write(iio_txdev[ig], 0x80000088, 0x6);

	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		if (g_prt_msk & (1 << ip)) {
			if (0 == g_src_way) {
				if (g_m_verbo > 2) printf("'DLK%d' Create txfiles for RX-port%d ... ", ig, ip);
				char str_txfile[16];
				sscanf(str_txfile, "txsmp%d", ip);
				g_rfd[ig*DEV_PORT_NB + ip] = fopen(str_txfile, "rb");
				ASSERT(g_rfd[ig*DEV_PORT_NB+ip] && "fopen g_rfd failed");
				if (g_m_verbo > 2) printf("Succeed\r\n");
			}

			if (g_m_verbo > 2) printf("'DLK%d' Enable i/q channels of TX-port%d ... ", ig, ip);
			iiopt_chni[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(iio_txdev[ig], TX, 2*ip, 0);
			ASSERT(iiopt_chni[ig*DEV_PORT_NB+ip] && "No txdev_p0i found");
			iiopt_chnq[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(iio_txdev[ig], TX, 2*ip+1, 0);
			ASSERT(iiopt_chnq[ig*DEV_PORT_NB+ip] && "No txdev_p0q found");
			iio_channel_enable(iiopt_chni[ig*DEV_PORT_NB+ip]);
			iio_channel_enable(iiopt_chnq[ig*DEV_PORT_NB+ip]);
			if (g_m_verbo > 2) printf("Succeed\r\n");
		}
	}

    if (g_m_verbo > 2) printf("'DLK%d' Creating iio_txbuf with %u samples ... ", ig, smp_nb);
	iio_txbuf[ig] = iio_device_create_buffer(iio_txdev[ig], smp_nb, false);
	ASSERT((NULL != iio_txbuf[ig]) && "Create iio_txbuf failed");
	if (g_m_verbo > 2) printf("Succeed\r\n");

	ASSERT(4*DEV_PORT_NB <= iio_device_get_sample_size(iio_txdev[ig]));
	return 0;
}

static void rxbuf_dump(int ig)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int ip;
	uint16_t *p_d16;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	printf("<-- iio_rxbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			ip, iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB+ip]),
			ip, iio_buffer_first(iio_rxbuf[ig], iiopr_chnq[ig*DEV_PORT_NB+ip]));
	}
	printf("\r\n");	//fflush(stdout);

	printf"<-- iio_rxbuf: RAW:");
	pbyte = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB]);
	for (i=0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\r\n");
		printf("%02x ", *pbyte);
	}
	printf("\r\n"); //fflush(stdout);

	printf("<-- iio_rxbuf: Samples:");
	for (ip = 0; ip < DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	printf("\t");
	for (ip = 0; ip < DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iio_rxbuf[ig]->data_length;
	psample = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB]);
	for (i = 0; (void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\r\n");
		for (ip = 0; ip < DEV_PORT_NB; ip++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
	printf("\r\n"); //fflush(stdout);
#if 0
	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		printf("<-- iio_rxbuf: P%dI: ", ip);
		p_d16 = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB+ip]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
		printf("\r\n"); //fflush(stdout);
		printf("<-- iio_rxbuf: P%dQ: ", ip);
		p_d16 = iio_buffer_first(iio_rxbuf[ig], iiopr_chnq[ig*DEV_PORT_NB+ip]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
	}
	printf("\r\n"); //fflush(stdout);
#endif
}

/* Method0: */
static void rxbuf_filew(int ig, ssize_t nbytes_from_k)
{
	fwrite(iio_rxbuf[ig], 1, nbytes_from_k, g_wfd);
	fflush(g_wfd);
}

// Method1: sample by sample
// Example: copy Real(I) and Imag(Q) to an array of userspace buffers
// The HARDWARE format of each sample :
// cat /sys/bus/iio/devices/iio:deviceX/scan_elements/in_voltage0_type
// le:s12/16>>4
static void rxbuf_memcopy(int ig, ssize_t nbytes_from_k)
{
	char *p_d16;
	ptrdiff_t p_inc;
	size_t nb;

	ASSERT(1 == DEV_PORT_NB); /* samples of P0 and P1 are interleaved */

	p_d16 = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB]);
	do {
		nb = min(nbytes_from_k, RADIO_SYM2SMP*4);
		memcpy((void *)rawst_cbuf[rawst_iw] + PUSCH_PLDOF, p_d16, nb/2);
		p_d16 += nb;
		rawst_iw++;
		if (RAWST_BUF_NB_NOW == rawst_iw) rawst_iw = 0;
		nbytes_from_k -= nb;
	} while (nbytes_from_k > 0);
}

// Method2: iio_channel_read_raw()
// Method3: iio_channel_read()
static void rxbuf_apir(int ig, ssize_t nbytes_from_k)
{
	int ip, iblk;
	uint8_t *testbuf;

	/* p0i0, p0i1, ...	p0q0, p0q1, ... */
	testbuf = malloc(nbytes_from_k);
	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		if (3 == g_dst_way) {
			iio_channel_read_raw(iiopr_chni[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read_raw(iiopr_chnq[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
		} else {
			iio_channel_read(iiopr_chni[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read(iiopr_chnq[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
		}
	}
	free(testbuf);
}

/* Method4: sample by sample, Example: swap Real(I) and Imag(Q)
	p_inc == iio_device_get_sample_size(iio_rxdev[ig]) == DEV_PORT_NB*4
	ismp == (p_end-p_d16)/p_inc == nbytes_from_k/(PORTS*4) <= IIO_SMP_MAX
*/
static void rxbuf_foreach(int ig, ssize_t nbytes_from_k)
{
	ptrdiff_t p_inc;
	void *p_end;
	int16_t *p_d16;
	int ip, ismp;
	int16_t i16, q16;

	p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	p_d16 = (int16_t *)iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB]);
#if 1
	do {
		for (ip = 0; ip < DEV_PORT_NB; ip++) {
			if (g_prt_msk & (1 << ip)) {
				i16 = *(p_d16 + 0);
				q16 = *(p_d16 + 1);
				*p_d16++ = q16;
				*p_d16++ = i16;
			}
		}
	} while ((void *)p_d16 < p_end);
#else //defined(BUILDING_RRU_UL)
	struct bbu_payload *pl;
	do {
		pl = (struct bbu_payload *)txring_pusch_offset(rawst_iw);
		for (ismp=0; ismp<min(nbytes_from_k/DEV_PORT_NB*4, RADIO_SYM2SMP); ismp++) {
			for (ip = 0; ip < DEV_PORT_NB; ip++) {
				pl->iqs[ip][ismp].i8 = *p_d16++;	// LSB of I[ismp][ip]
				p_d16++;							// MSB of I[ismp][ip]
				pl->iqs[ip][ismp].q8 = *p_d16++;	// LSB of Q[ismp][ip]
				p_d16++;							// MSB of Q[ismp][ip]
			}
		}
		nbytes_from_k -= ismp*p_inc;
		rawst_iw++;
	} while (0 != nbytes_from_k);
#endif
}

// Method5: iio_buffer_foreach_sample() with callback() : TODO
static ssize_t rxbuf_callback(struct iio_channel *chn, void *p_smp, size_t smp_n, void *opt)
{
	if (!iio_channel_is_enabled(chn))
		return 0;
#if 0
	static int ismp=0;
	static uint8_t *p_dst;

	if (0 == ismp % RADIO_SYM2SMP) {
		p_dst = (uint8_t *)rawst_cbuf[rawst_iw] + PUSCH_PLDOF;
		switch (chn->index) {
		// IIO[ismp][p0i+p0q] -> RRU[p0i+p0q][ismp]
		case 0: break;
		case 1: p_dst += 1; break;
		// IIO[ismp][p1i+p1q] -> RRU[p1i+p1q][ismp]
		case 2: p_dst += RADIO_SYM2SMP*2; break;
		case 3: p_dst += 1; break;
		default:
			printf("unknown channel id %d\r\n", chn->index);
			return -1;
		}
		rawst_iw++;
		if (RAWST_BUF_NB_NOW == rawst_iw) rawst_iw = 0;
	}

	*p_dst = *p_smp & 0xff; /* for example, only LSB */
	p_dst += 2;	/* p0i0, p0q0, p0i1, p0q1, ... */
	//p_smp += iio_buffer_step(iio_rxbuf[0]);
	//smp_n--;

#endif
	return g_rsmp_nb;
}

int handle_iio_rxpath(int ig)
{
	ssize_t nbytes_from_k;
	uint32_t val;

	nbytes_from_k = iio_buffer_refill(iio_rxbuf[ig]);
	if (g_m_verbo > 1) {
		iio_device_reg_read(iio_rxdev[ig], 0x80000088, &val);
		printf("\r\n<-- read rxdma[%d] status=0x%08x\r\n", val);
	}
	if (nbytes_from_k < 0) {
		fprintf(stderr, "<-- iio_buffer_refill(): error %s\r\n",
			strerror(-(int)nbytes_from_k));
		return -1;
	}

	if (g_m_verbo > 0) {
		ASSERT(nbytes_from_k <= IIO_SMP_MAX*4*DEV_PORT_NB);
		printf("<-- iio_buffer_refill(): k_iio => u_libiio %u Bytes\r\n",
			(size_t)nbytes_from_k);
	}
	if (g_m_verbo > 2) {
		rxbuf_dump(ig);
	}

	switch (g_dst_way) {
	case 0:
		rxbuf_filew(ig, nbytes_from_k);
		break;
	case 1:
		rxbuf_memcopy(ig, nbytes_from_k);
		break;
	case 2:
	case 3:
		rxbuf_apir(ig, nbytes_from_k);
		break;
	default:
	case 4:
		rxbuf_foreach(ig, nbytes_from_k);
		break;
	case 5:
		iio_buffer_foreach_sample(iio_rxbuf[ig], rxbuf_callback, NULL);
		break;
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	rxsnb[ig] += nbytes_from_k/iio_buffer_step(iio_rxbuf[ig])*2;
#else
    rxsnb[ig] += nbytes_from_k/iio_buffer_step(iio_rxbuf[ig]);
#endif
	return 0;
}

static void txbuf_dump(int ig)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int ip;
	uint16_t *p_d16;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	printf("--> iio_txbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			ip, iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB+ip]),
			ip, iio_buffer_first(iio_txbuf[ig], iiopt_chnq[ig*DEV_PORT_NB+ip]));
	}
	printf("\r\n"); //fflush(stdout);

	printf("--> iio_txbuf: RAW:");
	pbyte = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	for (i = 0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\r\n");
		printf("%02x ", *pbyte);
	}
	printf("\r\n"); //fflush(stdout);

	printf("--> iio_txbuf: Samples:");
	for (ip = 0; ip < DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	printf("\t");
	for (ip = 0; ip < DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iio_txbuf[ig]->data_length;
	psample = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	for (i = 0;	(void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\r\n");
		for (ip = 0; ip < DEV_PORT_NB; ip++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
	printf("\r\n"); //fflush(stdout);
#if 0
	for (ip = 0; ip < DEV_PORT_NB; ip++) {	
		printf("--> iio_txbuf: P%dI: ", ip);
		p_d16 = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB+ip]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
		printf("\r\n"); //fflush(stdout);
		printf("--> iio_txbuf: P%dQ: ", ip);
		p_d16 = iio_buffer_first(iio_txbuf[ig], iiopt_chnq[ig*DEV_PORT_NB+ip]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
	}
	printf("\r\n"); //fflush(stdout);
#endif
}

/* Method0: */
static void txbuf_filer(int ig, ssize_t nb2k)
{
	fread(iio_txbuf[ig], 1, nb2k, g_rfd);
	fflush(g_rfd);
}

/* Method1: */
static void txbuf_memcopy(int ig, ssize_t nb2k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_d16;
	int ip, ismp, nb, tmp;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	p_d16 = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	nb2k = p_end - p_d16;
#if 0
	ASSERT(1 == DEV_PORT_NB);
	do {
		if (rawsr_iw < rawsr_ir + RAWSR_BUF_NB_NOW/2) {
			sleep(0);
			continue;
		}
		tmp = rawsr_ir + RAWSR_BUF_NB_NOW/2;
		nb = RADIO_SYM2SMP*DEV_PORT_NB*4;
		do {
			memcpy(p_d16, rawsr_cbuf[rawsr_ir] + PDSCH_PLDOF, nb/2);
			p_d16 += nb/2;
			memcpy(p_d16, rawsr_cbuf[rawsr_ir] + PDSCH_PLDOF, nb/2);
			p_d16 += nb/2;
			rawsr_ir++;
		} while (rawsr_ir == tmp);
		txsnb[ig] += ismp;
		break;
	} while (1);
#endif
}

// Method2: iio_channel_write_raw()
// Method3: iio_channel_write()
#if 0
static void rxbuf_apiw(int ig, ssize_t nb2k)
{
	int ip, iblk;
	uint8_t *testbuf;

	/* p0i0, p0i1, ...	p0q0, p0q1, ... */
	testbuf = malloc(nbytes_from_k);
	/*for (ip = 0; ip < DEV_PORT_NB; ip++) {
		if (3 == g_dst_way) {
			iio_channel_read_raw(iiopr_chni[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read_raw(iiopr_chnq[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
		} else {
			iio_channel_read(iiopr_chni[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read(iiopr_chnq[ip], iio_rxbuf[0], testbuf, RADIO_SYM2SMP*2);
		}
	}*/
	free(testbuf);
}
#endif

/* Example: fill with 0xff
  p_inc == iio_device_get_sample_size(iio_txdev[ig]) == DEV_PORT_NB*4
   ismp == nb2k/p_inc == nb2k/(DEV_PORT_NB*4) <= IIO_SMP_MAX
AD9361:
  14-bit sample needs to be MSB alligned so shift by 2
  https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
AD9371:
  a10ad9371> cat iio:device4/scan_elements/in_voltage0_i_type
  le:S16/16>>0
  a10ad9371> cat iio:device3/scan_elements/out_voltage0_type
  le:S16/16>>0
*/
static void txbuf_foreach(int ig, ssize_t nb2k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_stt;
	int16_t *p_d16;
	int ip, ismp;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	p_d16 = (int16_t *)iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	nb2k = p_end - (void *)p_d16;
#if 1
	do {
		for (ip = 0; ip < DEV_PORT_NB; ip++) {
			if (g_prt_msk & (1 << ip)) {
				*p_d16++ = g_word_pn;
				*p_d16++ = g_word_pn;
			}
		}
	} while ((void *)p_d16 < p_end);
#else //defined(BUILDING_RRU_DL)
	struct bbu_payload *pl;
	int nb, tmp;
	do {
		if (rawsr_iw < rawsr_ir + RAWSR_BUF_NB_NOW/2) {
			sleep(0);
			continue;
		}
		tmp = rawsr_ir + RAWSR_BUF_NB_NOW/2;
		nb = RADIO_SYM2SMP*DEV_PORT_NB*4;
		do {
#if defined(BUILDING_RRU_PUSCH_B2B)// board circle-pusch for testing
			pl = (struct bbu_payload *)(&rawsr_cbuf[rawsr_ir][0] + PUSCH_PLDOF);
#else
			pl = (struct bbu_payload *)(&rawsr_cbuf[rawsr_ir][0] + PDSCH_PLDOF);
#endif
			for (ismp=0; ismp<RADIO_SYM2SMP; ismp++) {
				for (ip = 0; ip < DEV_PORT_NB; ip++) {
					*p_d16++ = pl->iqs[ip][ismp].i8;	// LSB of I[ismp][ip]
					p_d16++;							// MSB of I[ismp][ip]
					*p_d16++ = pl->iqs[ip][ismp].q8;	// LSB of I[ismp][ip]
					p_d16++;							// MSB of I[ismp][ip]
				}
			}
			rawsr_ir++;
		} while (rawsr_ir == tmp)
	} while (rawsr_iw > rawsr_ir + RAWSR_BUF_NB_NOW/2);
#endif
}

// Method5: iio_buffer_foreach_sample() with callback() : TODO
#if 0
static ssize_t txbuf_callback(struct iio_channel *chn, void *p_smp, size_t smp_n, void *opt)
{
	if (!iio_channel_is_enabled(chn))
		return 0;
	printf("TODO");
}
#endif

int handle_iio_txpath(int ig)
{
	ssize_t nb2k;
	int ip;
	uint32_t val;

	nb2k = iio_buffer_push(iio_txbuf[ig]);
	if (g_m_verbo > 1) {
		iio_device_reg_read(iio_txdev[ig], 0x80000088, &val);
		printf("\r\n--> read txdma[%d] status=0x%08x\r\n",
			ig, val);
	}
	if (nb2k < 0) {
		fprintf(stderr, "--> iio_buffer_push(): error %s\r\n",
			strerror(-(int)nb2k));
		return -1;
	}
	if (g_m_verbo > 0) {
		ASSERT(nb2k <= IIO_SMP_MAX*4*DEV_PORT_NB);
		printf("--> iio_buffer_push(): u_libiio => k_iio %u Bytes\r\n",
			(size_t)nb2k);
	}

	switch (g_src_way) {
	case 0:
		txbuf_foreach(ig, nb2k);
		break;
	case 1:
		txbuf_memcopy(ig, nb2k);
		break;
	case 2:
	case 3:
		printf("TODO: txbuf_apiw \r\n");
		break;
	case 3:
		txbuf_foreach(ig, nb2k);
		break;
	case 4:
		printf("TODO: txbuf_foreach\r\n");
		break;
	default:
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	txsnb[ig] += nb2k/iio_buffer_step(iio_txbuf[ig])*2;
#else
	txsnb[ig] += nb2k/iio_buffer_step(iio_txbuf[ig]);
#endif

	if (g_m_verbo > 2) txbuf_dump(ig);

	return 0;
}

//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiorxdma_overflow(int ig)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(iio_rxdev[ig], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "rxdma[%d] read-status failed %s\r\n", ig, strerror(-ret));
		return -1;
	}
	if (g_m_verbo > 1) printf("rxdma[%d] status=0x%08x\r\n", ig, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(iio_rxdev[ig], 0x80000088, val);
	if (val & 0x04) {
		if (g_m_verbo > 1) fprintf(stderr, "rxdma[%d] Overflow detected!\r\n", ig);
		return 1;
	}

	return 0;
}

//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiotxdma_underflow(int ig)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(iio_txdev[ig], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "txdma[%d] read-status failed %s\r\n", ig, strerror(-ret));
		return -1;
	}
	if (g_m_verbo > 1) printf("txdma[%d] status=0x%08x\r\n", ig, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(iio_txdev[ig], 0x80000088, val);
	if (val & 0x01) {
		if (g_m_verbo > 1) fprintf(stderr, "txdma[%d] Underflow detected\r\n", ig);
		return 1;
	}

	return 0;
}

int libiio_app_startup(int rxdev_smp_nb, int txdev_smp_nb)
{
	int ret, ig, ip;

    if (NULL == ctx) {
		ctx = iio_create_local_context();
    	ASSERT((NULL != ctx) && "context-local failed");

		ret = iio_context_get_devices_count(ctx);
    	ASSERT((ret > 0) && "No devices found");
    }

	for (ig = 0; ig < TOT_CHIP_NB; ig++) {
		init_phy(ig);
	}

	if (g_pth_msk & EN_IIO_PATH_RX) {
		for (ig = 0; ig < TOT_CHIP_NB; ig++) {
            ASSERT((rxdev_smp_nb > 0) && (rxdev_smp_nb <= IIO_SMP_MAX));
            init_dlk_rxpath(ig, rxdev_smp_nb);
        }
	}

	if (g_pth_msk & EN_IIO_PATH_TX) {
		for (ig = 0; ig < TOT_CHIP_NB; ig++) {
            ASSERT((txdev_smp_nb > 0) && (txdev_smp_nb <= IIO_SMP_MAX));
            init_dlk_txpath(ig, txdev_smp_nb);
        }
	}

	return 0;
}

/* cleanup and exit */
int libiio_app_shutdown(void)
{
	int ip, ig;

	if (g_pth_msk & EN_IIO_PATH_RX) {
		for (ig = 0; ig < TOT_CHIP_NB; ig++) {
			if (0 == g_dst_way) {
				fclose(g_wfd[ig*DEV_PORT_NB + ip]);
			}

			if (g_m_verbo > 2) printf("'DLK%d' Destroying iio_rxbuf ... ", ig);
			if (iio_rxbuf[ig]) iio_buffer_destroy(iio_rxbuf[ig]);
			if (g_m_verbo > 2) printf("Succeed\r\n");

			if (g_m_verbo > 2) printf("'DLK%d' Disabling iio_rxchn ...", ig);
			for (ip = 0; ip < TOT_PORT_NB; ip++) {
				if (iiopr_chni[ip]) iio_channel_disable(iiopr_chni[ip]);
				if (iiopr_chnq[ip]) iio_channel_disable(iiopr_chnq[ip]);
			}
			if (g_m_verbo > 2) printf("Succeed\r\n");
		}
	}

	if (g_pth_msk & EN_IIO_PATH_TX) {
		for (ig = 0; ig < TOT_CHIP_NB; ig++) {
			if (0 == g_src_way) {
				fclose(g_rfd[ig*DEV_PORT_NB + ip]);
			}

			if (g_m_verbo > 2) printf("'DLK%d' Destroying iio_txbuf ... ", ig);
			if (iio_txbuf[ig]) iio_buffer_destroy(iio_txbuf[ig]);
			if (g_m_verbo > 2) printf("Succeed\r\n");

			if (g_m_verbo > 2) printf("'DLK%d' Disabling iio_txchn ...", ig);
			for (ip = 0; ip < TOT_PORT_NB; ip++) {
				if (iiopt_chni[ip]) iio_channel_disable(iiopt_chni[ip]);
				if (iiopt_chnq[ip]) iio_channel_disable(iiopt_chnq[ip]);
			}
			if (g_m_verbo > 2) printf("Succeed\r\n");
		}
	}

	if (ctx) { iio_context_destroy(ctx); ctx = NULL; }
	return 0;
}

#if 1//!defined(BUILDING_RRU_UL) && !defined(BUILDING_RRU_DL)
int main(int argc, char *argv[])
{
	int ret, ig, ip;
	struct timespec ts, te;
	double tm_us;
	struct timespec old, new;

	get_args(argc, argv);

#if defined(FPGA_COMPRESS_SAMPLES)
	libiio_app_startup(g_rsmp_nb/2, g_tsmp_nb/2);
#else
    libiio_app_startup(g_rsmp_nb, g_tsmp_nb);
#endif

	signal(SIGINT, handle_sig);
	if (g_m_verbo > 0)
		printf("* Starting IO streaming (press CTRL+C to cancel)\r\n");

	clock_gettime(CLOCK_MONOTONIC, &ts);
	do {
		if (g_pth_msk & EN_IIO_PATH_RX) {
			for (ig = 0; ig < TOT_CHIP_NB; ig++) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_iio_rxpath(ig);
                clock_gettime(CLOCK_MONOTONIC, &new);
                printf("handle_iio_rxpath() take %.0f us REAL-TIME\r\n", 
                    elapse_us(&new, &old));
    		}
		}

    	if (g_pth_msk & EN_IIO_PATH_TX) {
			for (ig = 0; ig < TOT_CHIP_NB; ig++) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_iio_txpath(ig);
                clock_gettime(CLOCK_MONOTONIC, &new);
                printf("handle_iio_txpath() take %.0f us REAL-TIME\r\n", 
                    elapse_us(&new, &old));
    		}
    	}
        printf("RX %4.6f MiSmp; TX %4.6f MiSmp\r\n",
            rxsnb[0]/1e6, txsnb[0]/1e6);
	} while (!stop);

	clock_gettime(CLOCK_MONOTONIC, &te);
	tm_us = elapse_us(&te, &ts);
	printf("\r\n");
	for (ip = 0; ip < DEV_PORT_NB; ip++) {
		if (g_pth_msk & (EN_IIO_PATH_RX << ig))
			printf("RX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\r\n",
				ip, rxsnb[0]/1e6, tm_us, rxsnb[0]/tm_us*32);
		if (g_pth_msk & (EN_IIO_PATH_TX << ig))
			printf("TX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\r\n",
				ip, txsnb[0]/1e6, tm_us, txsnb[0]/tm_us*32);
	}

	libiio_app_shutdown();
	return 0;
}
#endif
