/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
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
#include "rru_bbu.h"

/* IIO structs required for streaming */
static struct iio_context *ctx;
// Streaming devices
static struct iio_device *ordev[TOT_CHIP_NB];
static struct iio_device *txdev[TOT_CHIP_NB];
static struct iio_device *rxdev[TOT_CHIP_NB];
// one libiio buffer for each device
       struct iio_buffer *iio_orbuf[TOT_CHIP_NB];
       struct iio_buffer *iio_txbuf[TOT_CHIP_NB];
       struct iio_buffer *iio_rxbuf[TOT_CHIP_NB];
// sample counters per device
static size_t orsnb[TOT_CHIP_NB];
static size_t rxsnb[TOT_CHIP_NB];
static size_t txsnb[TOT_CHIP_NB];
// 2x2 Streaming channels (2 Streaming channels) for each device
struct iio_channel *iio_rxchi[TOT_CHN_NB];
struct iio_channel *iio_rxchq[TOT_CHN_NB];
struct iio_channel *iio_txchi[TOT_CHN_NB];
struct iio_channel *iio_txchq[TOT_CHN_NB];

/* RX is input, TX is output */
enum iopath { RX, TX, OR };
static const char *str_path[] = {
	"RX", "TX", "OR"
};
#define EN_IIO_PATH_RX (1 << RX)
#define EN_IIO_PATH_TX (1 << TX)
       unsigned int g_pth_msk = EN_IIO_PATH_TX | EN_IIO_PATH_RX;
static unsigned int g_m_verbo = 0;
//#define BIT(n) (1 << n)
static unsigned int g_chn_msk = BIT(0); // 2T2R
//static unsigned int g_chn_msk = BIT(0) | BIT(1); // 4T4R
//static unsigned int g_chn_msk = BIT(0) | BIT(1) | BIT(2) | BIT(3); // 8T8R

static unsigned int g_rsmp_nb = IIOPR_SMP_NB_NOW;
static unsigned int g_dst_way = 0;
static unsigned int g_tsmp_nb = IIOPT_SMP_NB_NOW;
static unsigned int g_pattern = 0xffff;
static unsigned int g_src_way = 0;
static int wfd, rfd;


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
	while ((c = getopt(argc, argv, "x:c:m:d:n:p:s:v:h")) != EOF) {
		switch (c) {
		case 'x': g_pth_msk = strtoul(optarg, NULL, 16);			break;
		case 'c': g_chn_msk = strtoul(optarg, NULL, 16);			break;
		case 'm': g_rsmp_nb = strtoul(optarg, NULL,  0);			break;
		case 'd': g_dst_way = strtoul(optarg, NULL,  0);			break;
		case 'n': g_tsmp_nb = strtoul(optarg, NULL,  0);			break;
		case 'p': g_pattern = strtoul(optarg, NULL, 16);			break;
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
	printf( "g_chn_msk(-x):   %u\r\n", g_chn_msk );
	printf( "g_rsmp_nb(-m):   %u\r\n", g_rsmp_nb );
	printf( "g_dst_way(-d):   %u\r\n", g_dst_way );
	printf( "g_tsmp_nb(-n):   %u\r\n", g_tsmp_nb );
	printf( "g_pattern(-p):   %u\r\n", g_pattern );
	printf( "g_src_way(-s):   %u\r\n", g_src_way );
	printf( "\r\n" );
}

/* cleanup and exit */
static void shutdown(void)
{
	int ic, ig;

	if (g_m_verbo > 0)
		printf("* Destroying iio-buffers\r\n");
	for (ig = 0; ig < TOT_CHIP_NB; ig++) {
		if (iio_txbuf[ig]) iio_buffer_destroy(iio_txbuf[ig]);
		if (iio_rxbuf[ig]) iio_buffer_destroy(iio_rxbuf[ig]);

		if (g_m_verbo > 0)
			printf("* Disabling streaming channels\r\n");
		for (ic = 0; ic < TOT_CHN_NB; ic++) {
			if (iio_rxchi[ic]) iio_channel_disable(iio_rxchi[ic]);
			if (iio_rxchq[ic]) iio_channel_disable(iio_rxchq[ic]);
			if (iio_txchi[ic]) iio_channel_disable(iio_txchi[ic]);
			if (iio_txchq[ic]) iio_channel_disable(iio_txchq[ic]);
		}
	}

	if (g_m_verbo > 0)
		printf("* Destroying context\r\n");
	if (ctx) iio_context_destroy(ctx);
	exit(0);
}

static volatile sig_atomic_t stop = 0;
static void handle_sig(int sig)
{
	printf("\r\nWaiting for process to finish...\r\n");
	stop = 1;
}

#if 0
/* check return value of attr_write function */
static void errchk(int v, const char* what)
{
	if (v < 0) {
		fprintf(stderr, "Error %d writing to channel \"%s\""
			"\r\nvalue may not be supported.\r\n", v, what);
		exit(0);
	}
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: long long int */
static long long rd_ch_lli(struct iio_channel *chn, const char* what)
{
	long long val;

	errchk(iio_channel_attr_read_longlong(chn, what, &val), what);
	printf("\t %s: %lld\r\n", what, val);
	return val;
}

#if 0
/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}
#endif

/* helper function generating channel names */
static char *get_chn_name(const char* type, int id, char modify)
{
	if ('\0' == modify)
		snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	else
		snprintf(tmpstr, sizeof(tmpstr), "%s%d_%c", type, id, modify);
	return tmpstr;
}

/* helper function to check return value of attr_write function */
/* int chn, const char* type_string, const char* what, type_key value */
#define IIO_ATTR(A, C, T, W, V) \
do {\
	int ret;\
	ret = iio_channel_attr_##A_##T(C, W, V);\
	if (ret < 0) {\
		fprintf(stderr, "Error %d "#A" to channel \"%s\"\r\n", W);\
		abort();\
	}\
} while (0);
#endif

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
	ASSERT((NULL != dev) && "the specified iio device not found");
	return dev;
}

/* finds datalink IIO channels */
static struct iio_channel *get_dlk_chn_ss(struct iio_device *dev, enum iopath p, int ic, char modify)
{
	struct iio_channel *chn;
#if 0
	*chn = iio_device_find_channel(dev, get_chn_name("voltage", ic, modify), p == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, get_chn_name("voltage", ic, modify), p == TX);
#else
	if (g_m_verbo > 0)
		printf("* Acquiring dlk ss channel %s%d\r\n", p == TX ? "TX" : "RX", ic);
	chn = iio_device_find_channel(dev, get_chn_name("voltage", ic, modify), p == TX);
#endif
	ASSERT((NULL != chn) && "get_dlk_chn_ss: No channel found");
	return chn;
}

/* JESD_RX(iio:device4) */
	/* JESD_RX[0/1]_I(iio:device4/in_voltage[0/1]_i_xxx) */
	/* JESD_RX[0/1]_Q(iio:device4/in_voltage[0/1]_q_xxx) */
int init_iio_rxpath(int ig, size_t smp_nb)
{
	if (g_m_verbo > 0)
		printf("RX(g%d): Acquiring L3RX-JESD device\r\n", ig);
	rxdev[ig] = iio_context_find_device(ctx, str_devname[L3RX]);
	ASSERT((NULL != rxdev[ig]) && "No rxdev found");
	iio_device_reg_write(rxdev[ig], 0x80000088, 0x6);// Clear all status bits

	for (ic=0; ic<DEV_CHN_NB; ic++) {
		if (g_chn_msk & (1 << ic)) {
            if (g_m_verbo > 0)
                printf("RX(g%dc%d): Initializing channels of L3RX-JESD device\r\n", ig, ic);
			iio_rxchi[ig*DEV_CHN_NB+ic] = get_dlk_chn_ss(rxdev[ig], L3RX, ic, 'i');
			ASSERT(ret && "No rxdev_p0i found");
			iio_rxchq[ig*DEV_CHN_NB+ic] = get_dlk_chn_ss(rxdev[ig], L3RX, ic, 'q');
			ASSERT(ret && "No rxdev_p0q found");
			iio_channel_enable(iio_rxchi[ig*DEV_CHN_NB + ic]);
			iio_channel_enable(iio_rxchq[ig*DEV_CHN_NB + ic]);
		}
	}

    if (g_m_verbo > 0)
    	printf("RX(g%d): Creating non-cyclic buffers with %u samples\r\n", ig, smp_nb);
	iio_txbuf[ig] = iio_device_create_buffer(rxdev[ig], smp_nb, false);
	if (NULL == iio_txbuf[ig]) {
		perror("RX: Creating buffers failed");
		return -1;
	}
	ASSERT(4*DEV_CHN_NB == iio_device_get_sample_size(rxdev[ig]));

	return 0;
}

/* JESD_TX(iio:device3)*/
/* JESD_TX[0/1]_I(iio:device3/out_voltage[0/2]_xxx)*/
/* JESD_TX[0/1]_Q(iio:device3/out_voltage[1/3]_xxx)*/
int init_iio_txpath(int ig, size_t smp_nb)
{
	if (g_m_verbo > 0)
		printf("TX(g%d): Acquiring TX-JESD device\r\n", ig);
	txdev[ig] = iio_context_find_device(ctx, str_devname[TX]);
	ASSERT((NULL != txdev[ig]) && "No txdev found");
	//iio_device_reg_write(txdev[ig], 0x80000088, 0x6);// Clear all status bits

	for (ic=0; ic<DEV_CHN_NB; ic++) {
		if (g_chn_msk & (1 << ic)) {
            if (g_m_verbo > 0)
                printf("TX(g%d): Initializing channels of TX-JESD port%d\r\n", ig, ic);
			ret = get_dlk_chn_ss(ctx, TX, txdev[ig], 2*ic,   0, 
				&iio_txchi[ig*DEV_CHN_NB+ic]);
			ASSERT(ret && "No txdev_p0i found");
			ret = get_dlk_chn_ss(ctx, TX, txdev[ig], 2*ic+1, 0, 
				&iio_txchq[ig*DEV_CHN_NB+ic]);
			ASSERT(ret && "No txdev_p0q found");
			iio_channel_enable(iio_txchi[ig*DEV_CHN_NB+ic]);
			iio_channel_enable(iio_txchq[ig*DEV_CHN_NB+ic]);
		}
	}
    if (g_m_verbo > 0)
    	printf("TX(g%d): Creating non-cyclic buffers with %u samples\r\n",
    	    ig, smp_nb);
	iio_rxbuf[ig] = iio_device_create_buffer(txdev[ig], smp_nb, false);
	if (NULL == iio_rxbuf[ig]) {
		perror("TX: Create buffers failed");
		return -1;
	}
	ASSERT(4*DEV_CHN_NB == iio_device_get_sample_size(txdev[ig]));

	return 0;
}

void rxbuf_dump(int ig)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int ic;
	uint16_t *p_d16;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	printf("<-- iio_rxbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (ic=0; ic<DEV_CHN_NB; ic++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			ic, iio_buffer_first(iio_txbuf[ig], iio_rxchi[ig*DEV_CHN_NB+ic]),
			ic, iio_buffer_first(iio_txbuf[ig], iio_rxchq[ig*DEV_CHN_NB+ic]));
	}
	printf("\r\n<-- iio_rxbuf: RAW:");
	pbyte = iio_buffer_first(iio_txbuf[ig], iio_rxchi[ig*DEV_CHN_NB]);
	for (i=0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\r\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);

	printf("\r\n<-- iio_rxbuf: Samples:");
	for (ic=0; ic<DEV_CHN_NB; ic++) printf(" P%dI, P%dQ", ic, ic);
	printf("\t");
	for (ic=0; ic<DEV_CHN_NB; ic++) printf(" P%dI, P%dQ", ic, ic);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iio_txbuf[ig]->data_length;
	psample = iio_buffer_first(iio_txbuf[ig], iio_rxchi[ig*DEV_CHN_NB]);
	for (i = 0; (void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\r\n");
		for (ic=0; ic<DEV_CHN_NB; ic++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
#if 0
	for (ic=0; ic<DEV_CHN_NB; ic++) {
		printf("\r\n<-- iio_rxbuf: P%dI: ", ic);
		p_d16 = iio_buffer_first(iio_txbuf[ig], iio_rxchi[ig*DEV_CHN_NB+ic]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
		printf("\r\n<-- iio_rxbuf: P%dQ: ", ic);
		p_d16 = iio_buffer_first(iio_txbuf[ig], iio_rxchq[ig*DEV_CHN_NB+ic]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
	}
#endif
	printf("\r\n");
	fflush(stdout);
}

void txbuf_dump(int ig)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int ic;
	uint16_t *p_d16;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	printf("--> iio_txbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (ic=0; ic<DEV_CHN_NB; ic++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			ic, iio_buffer_first(iio_rxbuf[ig], iio_txchi[ig*DEV_CHN_NB+ic]),
			ic, iio_buffer_first(iio_rxbuf[ig], iio_txchq[ig*DEV_CHN_NB+ic]));
	}

	printf("\r\n--> iio_txbuf: RAW:");
	pbyte = iio_buffer_first(iio_rxbuf[ig], iio_txchi[ig*DEV_CHN_NB]);
	for (i = 0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\r\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);

	printf("\r\n--> iio_txbuf: Samples:");
	for (ic=0; ic<DEV_CHN_NB; ic++) printf(" P%dI, P%dQ", ic, ic);
	printf("\t");
	for (ic=0; ic<DEV_CHN_NB; ic++) printf(" P%dI, P%dQ", ic, ic);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iio_rxbuf[ig]->data_length;
	psample = iio_buffer_first(iio_rxbuf[ig], iio_txchi[ig*DEV_CHN_NB]);
	for (i = 0;	(void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\r\n");
		for (ic=0; ic<DEV_CHN_NB; ic++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
#if 0
	for (ic=0; ic<DEV_CHN_NB; ic++) {	
		printf("\r\n--> iio_txbuf: P%dI: ", ic);
		p_d16 = iio_buffer_first(iio_rxbuf[ig], iio_txchi[ig*DEV_CHN_NB+ic]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
		printf("\r\n--> iio_txbuf: P%dQ: ", ic);
		p_d16 = iio_buffer_first(iio_rxbuf[ig], iio_txchq[ig*DEV_CHN_NB+ic]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
	}
#endif
	printf("\r\n");
	fflush(stdout);
}

/* Method0: */
static void rxbuf_filew(int ig, ssize_t nbytes_from_k)
{
	fwrite(iio_txbuf[ig], 1, nbytes_from_k, wfd);
	fflush(wfd);
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

	ASSERT(1 == DEV_CHN_NB); /* samples of P0 and P1 are interleaved */

	p_d16 = iio_buffer_first(iio_txbuf[ig], iio_rxchi[ig*DEV_CHN_NB]);
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
// TODO: get known
static void rxbuf_apiread(int ig, ssize_t nbytes_from_k)
{
	int ic, iblk;
	uint8_t *testbuf;

	/* p0i0, p0i1, ...	p0q0, p0q1, ... */
	testbuf = malloc(nbytes_from_k);
	for (ic=0; ic<DEV_CHN_NB; ic++) {
		if (3 == g_dst_way) {
			iio_channel_read_raw(iio_rxchi[ic], iio_txbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read_raw(iio_rxchq[ic], iio_txbuf[0], testbuf, RADIO_SYM2SMP*2);
		} else {
			iio_channel_read(iio_rxchi[ic], iio_txbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read(iio_rxchq[ic], iio_txbuf[0], testbuf, RADIO_SYM2SMP*2);
		}
	}
	free(testbuf);
}

/* Method4: sample by sample, Example: swap Real(I) and Imag(Q)
	p_inc == iio_device_get_sample_size(rxdev[ig]) == DEV_CHN_NB*4
	ismp == (p_end-p_d16)/p_inc == nbytes_from_k/(PORTS*4) <= IIO_SMP_MAX
*/
static void rxbuf_foreach(int ig, ssize_t nbytes_from_k)
{
	ptrdiff_t p_inc;
	void *p_end;
	int16_t *p_d16;
	int ic, ismp;
	int16_t i16, q16;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	p_d16 = (int16_t *)iio_buffer_first(iio_txbuf[ig], iio_rxchi[ig*DEV_CHN_NB]);
#if 1
	do {
		for (ic=0; ic<DEV_CHN_NB; ic++) {
			if (g_chn_msk & (1 << ic)) {
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
		for (ismp=0; ismp<min(nbytes_from_k/DEV_CHN_NB*4, RADIO_SYM2SMP); ismp++) {
			for (ic=0; ic<DEV_CHN_NB; ic++) {
				pl->iqs[ic][ismp].i8 = *p_d16++;	// LSB of I[ismp][ic]
				p_d16++;							// MSB of I[ismp][ic]
				pl->iqs[ic][ismp].q8 = *p_d16++;	// LSB of Q[ismp][ic]
				p_d16++;							// MSB of Q[ismp][ic]
			}
		}
		nbytes_from_k -= ismp*p_inc;
		rawst_iw++;
	} while (0 != nbytes_from_k);
#endif
}

// Method5: iio_buffer_foreach_sample() with callback()
// TODO: get known
static ssize_t rxbuf_callback(const struct iio_channel *chn,
			void *p_smp, size_t smp_n, void *opt)
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
	//p_smp += iio_buffer_step(iio_txbuf[0]);
	//smp_n--;

#endif
	return g_rsmp_nb;
}

int handle_iio_rxpath(int ig)
{
	ssize_t nbytes_from_k;
	uint32_t val;

	nbytes_from_k = iio_buffer_refill(iio_txbuf[ig]);
	if (g_m_verbo > 1) {
		iio_device_reg_read(rxdev[ig], 0x80000088, &val);
		printf("\r\n<-- read rxdma[%d] status=0x%08x\r\n", val);
	}
	if (nbytes_from_k < 0) {
		fprintf(stderr, "<-- iio_buffer_refill(): error %s\r\n",
			strerror(-(int)nbytes_from_k));
		return -1;
	}

	if (g_m_verbo > 0) {
		ASSERT(nbytes_from_k <= IIO_SMP_MAX*4*DEV_CHN_NB);
		printf("<-- iio_buffer_refill(): k_iio => u_libiio %u Bytes\r\n",
			(size_t)nbytes_from_k);
	}
	if (g_m_verbo > 2) {
		rxbuf_dump(ig);
	}

	switch (g_dst_way) {
	case 5:
		iio_buffer_foreach_sample(iio_txbuf[ig], rxbuf_callback, NULL);
		break;
	case 4:
		rxbuf_foreach(ig, nbytes_from_k);
		break;
	case 3:
	case 2:
		rxbuf_apiread(ig, nbytes_from_k);
		break;
	case 1:
		rxbuf_memcopy(ig, nbytes_from_k);
		break;
	case 0:
	default:
		rxbuf_filew(ig, nbytes_from_k);
		break;
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	rxsnb[ig] += nbytes_from_k/iio_buffer_step(iio_txbuf[ig])*2;
#else
    rxsnb[ig] += nbytes_from_k/iio_buffer_step(iio_txbuf[ig]);
#endif
	return 0;
}

/* Method0: */
static void txbuf_filer(int ig, ssize_t nbytes_to_k)
{
	fread(iio_rxbuf[ig], 1, nbytes_to_k, rfd);
	fflush(rfd);
}

/* Example: fill with 0xff
  iio_buffer_foreach_sample();
  p_inc == iio_device_get_sample_size(txdev[ig]) == DEV_CHN_NB*4
  ismp == nbytes_to_k/p_inc == nbytes_to_k/(DEV_CHN_NB*4) <= IIO_SMP_MAX
AD9361:
  14-bit sample needs to be MSB alligned so shift by 2
  https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
AD9371:
  a10ad9371> cat iio:device4/scan_elements/in_voltage0_i_type
  le:S16/16>>0
  a10ad9371> cat iio:device3/scan_elements/out_voltage0_type
  le:S16/16>>0
*/
static void txbuf_foreach(int ig, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_stt;
	int16_t *p_d16;
	int ic, ismp;

	p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	p_d16 = (int16_t *)iio_buffer_first(iio_rxbuf[ig], iio_txchi[ig*DEV_CHN_NB]);
	nbytes_to_k = p_end - (void *)p_d16;
#if 1
	do {
		for (ic=0; ic<DEV_CHN_NB; ic++) {
			if (g_chn_msk & (1 << ic)) {
				*p_d16++ = g_pattern;
				*p_d16++ = g_pattern;
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
		nb = RADIO_SYM2SMP*DEV_CHN_NB*4;
		do {
#if defined(BUILDING_RRU_PUSCH_B2B)// board circle-pusch for testing
			pl = (struct bbu_payload *)(&rawsr_cbuf[rawsr_ir][0] + PUSCH_PLDOF);
#else
			pl = (struct bbu_payload *)(&rawsr_cbuf[rawsr_ir][0] + PDSCH_PLDOF);
#endif
			for (ismp=0; ismp<RADIO_SYM2SMP; ismp++) {
				for (ic=0; ic<DEV_CHN_NB; ic++) {
					*p_d16++ = pl->iqs[ic][ismp].i8;	// LSB of I[ismp][ic]
					p_d16++;							// MSB of I[ismp][ic]
					*p_d16++ = pl->iqs[ic][ismp].q8;	// LSB of I[ismp][ic]
					p_d16++;							// MSB of I[ismp][ic]
				}
			}
			rawsr_ir++;
		} while (rawsr_ir == tmp)
	} while (rawsr_iw > rawsr_ir + RAWSR_BUF_NB_NOW/2);
#endif
}

static void txbuf_memcopy(int ig, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_d16;
	int ic, ismp, nb, tmp;

	p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	p_d16 = iio_buffer_first(iio_rxbuf[ig], iio_txchi[ig*DEV_CHN_NB]);
	nbytes_to_k = p_end - p_d16;
#if 0
	ASSERT(1 == DEV_CHN_NB);
	do {
		if (rawsr_iw < rawsr_ir + RAWSR_BUF_NB_NOW/2) {
			sleep(0);
			continue;
		}
		tmp = rawsr_ir + RAWSR_BUF_NB_NOW/2;
		nb = RADIO_SYM2SMP*DEV_CHN_NB*4;
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

int handle_iio_txpath(int ig)
{
	ssize_t nbytes_to_k;
	int ic;
	uint32_t val;

	nbytes_to_k = iio_buffer_push(iio_rxbuf[ig]);
	if (g_m_verbo > 1) {
		iio_device_reg_read(txdev[ig], 0x80000088, &val);
		printf("\r\n--> read txdma[%d] status=0x%08x\r\n",
			ig, val);
	}
	if (nbytes_to_k < 0) {
		fprintf(stderr, "--> iio_buffer_push(): error %s\r\n",
			strerror(-(int)nbytes_to_k));
		return -1;
	}
	if (g_m_verbo > 0) {
		ASSERT(nbytes_to_k <= IIO_SMP_MAX*4*DEV_CHN_NB);
		printf("--> iio_buffer_push(): u_libiio => k_iio %u Bytes\r\n",
			(size_t)nbytes_to_k);
	}

	switch (g_src_way) {
	case 4:
	case 3:
		printf("\r\ncase3/4 TODO\r\n");
		break;
	case 2:
		printf("\r\ncase2 TODO\r\n");
		break;
	case 1:
		txbuf_memcopy(ig, nbytes_to_k);
		break;
	default:
	case 0:
		txbuf_foreach(ig, nbytes_to_k);
		break;
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	txsnb[ig] += nbytes_to_k/iio_buffer_step(iio_rxbuf[ig])*2;
#else
	txsnb[ig] += nbytes_to_k/iio_buffer_step(iio_rxbuf[ig]);
#endif

	if (g_m_verbo > 2) {
		txbuf_dump(ig);
	}

	return 0;
}

//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiorxdma_overflow(int ig)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(rxdev[ig], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "<-- read rxdma[%d] status failed %s\r\n",
			ig, strerror(-ret));
		return -1;
	}
	if (g_m_verbo > 1)
		printf("<-- rxdma[%d] status=0x%08x\r\n", ig, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(rxdev[ig], 0x80000088, val);
	if (val & 0x04) {
		if (g_m_verbo > 1)
			fprintf(stderr, "<-- rxdma[%d] Overflow detected!\r\n", ig);
		return 1;
	}

	return 0;
}

//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiotxdma_underflow(int ig)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(txdev[ig], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "--> read txdma[%d] status failed %s\r\n",
			ig, strerror(-ret));
		return -1;
	}
	if (g_m_verbo > 1)
		printf("--> txdma[%d] status=0x%08x\r\n", ig, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(txdev[ig], 0x80000088, val);
	if (val & 0x01) {
		if (g_m_verbo > 1)
			fprintf(stderr, "--> txdma[%d] Underflow detected\r\n", ig);
		return 1;
	}

	return 0;
}

void libiio_app_startup(int rxdev_smp_nb, int txdev_smp_nb)
{
	int ret, ig, ic;
	struct iio_device *phy;

    if (NULL == ctx) {
        if (g_m_verbo > 0)
            printf("* Acquiring IIO context\r\n");
		ctx = iio_create_local_context();
    	ASSERT((NULL != ctx) && "No context");
		ret = iio_context_get_devices_count(ctx);
    	ASSERT((ret > 0) && "No devices");
    }

	for (ig=0; ig<TOT_CHIP_NB; ig++) {
#if 1
        if (g_m_verbo > 0)
            printf("LIBIIO_APP(g%d): Acquiring PHY device\r\n", ig);
        phy = get_iio_dev(ctx, "adrv9371-phy");
		ASSERT((ret > 0) && "No devices");
        if (g_m_verbo > 0)
            printf("LIBIIO_APP(g%d): Profiling PHY device\r\n", ig);
        profile_ad9371_phy_array(phy);
#endif
		if (g_pth_msk & (EN_IIO_PATH_RX << ig)) {
			init_phy_rxpath(ig);
            ASSERT((rxdev_smp_nb > 0) && (rxdev_smp_nb <= IIO_SMP_MAX));
            init_iio_rxpath(ig, rxdev_smp_nb);
			if (0 == g_dst_way) {
				char str_rxfile[16];
				sscanf(str_rxfile, "%s%d", "rxsmp", g_chn_msk);
				wfd = fopen(str_rxfile, "wb");
			}
        }
		if (g_pth_msk & (EN_IIO_PATH_TX << ig)) {
			init_phy_txpath(ig);
            ASSERT((txdev_smp_nb > 0) && (txdev_smp_nb <= IIO_SMP_MAX));
            init_iio_txpath(ig, txdev_smp_nb);
			if (0 == g_src_way) {
				char str_txfile[16];
				sscanf(str_txfile, "%s%d", "txsmp", g_chn_msk);
				rfd = fopen(str_txfile, "wb");
			}
        }
	}
}

#if 1//!defined(BUILDING_RRU_UL) && !defined(BUILDING_RRU_DL)
int main(int argc, char *argv[])
{
	int ig, ic;
	struct timespec tm_xs, tm_xe;
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

	clock_gettime(CLOCK_MONOTONIC, &tm_xs);
	do {
        for (ig=0; ig<TOT_CHIP_NB; ig++) {
    		if (g_pth_msk & (EN_IIO_PATH_RX << ig)) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_iio_rxpath(ig);
                clock_gettime(CLOCK_MONOTONIC, &new);
                printf("handle_iio_rxpath() take %.0f us REAL-TIME\r\n", 
                    elapse_us(&new, &old));
    		}
    		if (g_pth_msk & (EN_IIO_PATH_TX << ig)) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_iio_txpath(ig);
                clock_gettime(CLOCK_MONOTONIC, &new);
                printf("handle_iio_txpath() take %.0f us REAL-TIME\r\n", 
                    elapse_us(&new, &old));
    		}
            //printf("\r");
            printf("RX[0] %4.6f MiSmp; TX[0] %4.6f MiSmp",
                rxsnb[0]/1e6, txsnb[0]/1e6);
            printf("\r\n");
            fflush(stdout);
        }
	} while (!stop);

	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
	tm_us = elapse_us(&tm_xe, &tm_xs);
	printf("\r\n");
	for (ic=0; ic<DEV_CHN_NB; ic++) {
		if (g_pth_msk & (EN_IIO_PATH_RX << ig))
			printf("RX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\r\n",
				ic, rxsnb[0]/1e6, tm_us, rxsnb[0]/tm_us*32);
		if (g_pth_msk & (EN_IIO_PATH_TX << ig))
			printf("TX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\r\n",
				ic, txsnb[0]/1e6, tm_us, txsnb[0]/tm_us*32);
	}

	ig = 0;
	if (g_pth_msk & (EN_IIO_PATH_RX << ig)) {
		if (0 == g_dst_way) {
			char str_rxfile[16];
			sscanf(str_rxfile, "%s%d", "rxsmp", g_chn_msk);
			fclose(wfd);
		}
	}
	if (g_pth_msk & (EN_IIO_PATH_TX << ig)) {
		if (0 == g_src_way) {
			char str_txfile[16];
			sscanf(str_txfile, "%s%d", "txsmp", g_chn_msk);
			fclose(rfd);
		}
	}

	shutdown();
	return 0;
}
#endif
