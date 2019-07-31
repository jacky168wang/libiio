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
#include "rru_bbu.h"

#define AD9009_DEBUG (1)
#if(AD9009_DEBUG)
//printf("=====> %s(%d): %s() "#fmt"\r\n",__FILE__,__LINE__,__func__,##args)
#define printf(fmt,args...)  printf("==> (%d): %s()>>->> "#fmt"\r\n",__LINE__,__func__,##args)
#else
#define printf(fmt,args...)
#endif

/* IIO structs required for streaming */
static struct iio_context *ctx;

// Streaming devices
static struct iio_device *rxdev[TOT_CHIP_NB];
static struct iio_device *txdev[TOT_CHIP_NB];
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

#define EN_IIO_PATH_RX (1 << RX)
#define EN_IIO_PATH_TX (1 << TX)
       unsigned int g_pth_msk = EN_IIO_PATH_TX | EN_IIO_PATH_RX;

static unsigned int g_prt_msk = BIT(0); // 2T2R
//static unsigned int g_prt_msk = BIT(0) | BIT(1); // 4T4R
//static unsigned int g_prt_msk = BIT(0) | BIT(1) | BIT(2) | BIT(3); // 8T8R

static unsigned int g_rsmp_nb = 0; //IIOPR_SMP_NB_NOW;
static unsigned int g_dst_way = 0;

static unsigned int g_tsmp_nb = 0; //IIOPT_SMP_NB_NOW;
static unsigned int g_word_pn = 0x7832;
static unsigned int cyc_mode = 0;

static FILE* fd = NULL;
static FILE* writefd = NULL;
static int bypassDatasize = 0;
static int outputFormat = 0;
static int outCounter = 0;

static char in0Filename[MAXSLEN];
static char in1Filename[MAXSLEN];
static char out0Filename[MAXSLEN];
static char out1Filename[MAXSLEN];


static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION]\r\n"
		" -h\tshow this help\r\n"
		" -n\tset samples number[4~1048576] for iio TX/RX each time\r\n"
		" -t\tenable TX datapath only\r\n"
		" -r\tenable RX datapath only\r\n"
        " -f\tTX file what you want to send by tx1.\r\n"
        " -s\tTX file what you want to send by tx2\r\n"
        " -c\tiio create buffer cycle.\r\n"
        " -w\tselect one way to deal with samples\r\n"
		"   \t\t0) memcpy to user buffer in device level\r\n"
		"   \t\t1) for each sample frankly in device level\r\n"
		"   \t\t2) for each sample with callback() in channel level\r\n"
		"   \t\t3) call iio_channel_read_raw() for all samples of one channel\r\n"
		"   \t\t4) call iio_channel_read() for all samples of one channel\r\n"
		"   \t\t5) don't convert data format, send big file\r\n"
		"   \t\t6) read/write file to tx/rx\r\n"
		" -v\tbe verbose [0,1,2,3]\r\n"
		" -o\tchoose module what u wana get, and then generate module.txt\r\n"
		"   \t\t tx_loopback, scm_data,dec_data, ifft_data,cpi_data\r\n"
		"   \t\t rx_loopback, scd_data,com_data,  fft_data,cpr_data\r\n"
		"   \t./stream-test -w 6 -f filename1 -s filename2 -p port -o module  -->module.txt\r\n"
		"   \t./stream-test -w 6 -f filename1 -s filename2 -p 3 -o loopback   -->tx_loop_back.txt\r\n"
		"   \t./stream-test -w 6 -f filename1 -s filename2 -p 3 -o scm_data   -->scm_data.txt\r\n"
		"   \t./stream-test -w 6 -f filename1 -p 1 -o ifft_data               -->ifft_data.txt\r\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
static void get_args(int argc, char **argv)
{
	int c;
    int tmpsize;
	opterr = 0;
    memset(in0Filename, 0x00, sizeof(in0Filename));
    memset(in1Filename, 0x00, sizeof(in1Filename));
    memset(out0Filename, 0x00, sizeof(out0Filename));
    memset(out1Filename, 0x00, sizeof(out1Filename));
    
	while ((c = getopt(argc, argv, "p:m:n:v:s:w:b:f:o:x:t:r:ch")) != EOF) {
		switch (c) {
		case 'x': g_pth_msk = strtoul(optarg, NULL, 16);	break;
		case 'p': g_prt_msk = strtoul(optarg, NULL, 16);	break;
		case 'm': g_rsmp_nb = strtoul(optarg, NULL,  0);	break;
		case 'p': g_dst_way = strtoul(optarg, NULL,  0);	break;
		case 'n': g_tsmp_nb = strtoul(optarg, NULL,  0);	break;
		case 'w': g_word_pn = strtoul(optarg, NULL, 0);		break;
        case 't': g_pth_msk = 2;                           
                  g_tsmp_nb = strtoul(optarg, NULL, 0);
                  break;
        case 'r': g_pth_msk = 1;
                  g_rsmp_nb = strtoul(optarg, NULL, 0);
                  break;
        case 'c': cyc_mode = 1;                             break;

        case 'f': tmpsize = strlen(optarg); 
                  if(tmpsize > 0)
                    memcpy(in0Filename, optarg, tmpsize);
                  printf("%s-->%d", in0Filename, tmpsize);
                  break;
                  
        case 's': tmpsize = strlen(optarg); 
                  if(tmpsize > 0)
                    memcpy(in1Filename, optarg, tmpsize);
                  printf("%s-->%d", in1Filename, tmpsize);
                  break;

        case 'o': 
                  tmpsize = strlen(optarg); 
                  if(tmpsize > 0)
                    memcpy(out0Filename, optarg, tmpsize);
                  printf("%s-->%d", out0Filename, tmpsize);
                  break;
		case 'v': g_m_verbo = strtoul(optarg, NULL,  0);	break;
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
	printf( "g_rsmp_nb(-n):   %u\r\n", g_rsmp_nb );
	printf( "g_tsmp_nb(-n):   %u\r\n", g_tsmp_nb );
	printf( "g_pth_msk(-x):   %u\r\n", g_pth_msk );
	printf( "g_dst_way(-w):   %u\r\n", g_dst_way );
	printf( "g_word_pn(-b):   %u\r\n\r\n", g_word_pn);
    printf( "cyc_mode(-b):   %u\r\n\r\n", cyc_mode);
    printf( "g_pth_msk:   %u\r\n\r\n", g_pth_msk);
	printf( "\r\n" );
}

/* cleanup and exit */
static void shutdown(void)
{
	int ip, ig;

	if (g_m_verbo > 0) printf("* Destroying buffers ...");
	for (ig = 0; ig < TOT_CHIP_NB; ig++) {
		if (iio_rxbuf[ig]) iio_buffer_destroy(iio_rxbuf[ig]);
		if (iio_txbuf[ig]) iio_buffer_destroy(iio_txbuf[ig]);
	}
	if (g_m_verbo > 0) printf(" Done\r\n");

	if (g_m_verbo > 0) printf("* Disabling streaming channels ...");
	for (ig = 0; ig < TOT_CHIP_NB; ig++) {
		for (ip = 0; ip < TOT_PORT_NB; ip++) {
			if (iiopr_chni[ip]) iio_channel_disable(iiopr_chni[ip]);
			if (iiopr_chnq[ip]) iio_channel_disable(iiopr_chnq[ip]);
			if (iiopt_chni[ip]) iio_channel_disable(iiopt_chni[ip]);
			if (iiopt_chnq[ip]) iio_channel_disable(iiopt_chnq[ip]);
		}
	}
	if (g_m_verbo > 0) printf(" Done\r\n");

	if (g_m_verbo > 0) printf("* Destroying context ...");
	if (ctx) iio_context_destroy(ctx);
	if (g_m_verbo > 0) printf(" Done\r\n");
	exit(0);
}

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
	ASSERT((NULL != dev) && "the specified iio device not found");
	return dev;
}

/* finds datalink IIO channels */
static struct iio_channel *get_dlk_chn_ss(struct iio_device *dev, enum iopath p, int ip, char modify)
{
	struct iio_channel *chn;
#if 0
	*chn = iio_device_find_channel(dev, GET_CHN_NAME("voltage", ip, modify), p == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, GET_CHN_NAME("voltage", ip, modify), p == TX);
#else
	if (g_m_verbo > 0)
		printf("* Acquiring dlk ss channel %s%d\r\n", p == TX ? "TX" : "RX", ip);
	chn = iio_device_find_channel(dev, GET_CHN_NAME("voltage", ip, modify), p == TX);
#endif
	ASSERT((NULL != chn) && "get_dlk_chn_ss: No channel found");
	return chn;
}




/************************************************************************
#   slot_num  * 14 * 3276
#   slot_num  * 14 * 4096
#   slot_num  * 14 * 4096
#   slot_num  * 14 * 4096
#   slot_num  * 61440
#   slot_num  * 14 * 4096
#   slot_num  * 14 * 4096
#   slot_num  * 14 * 4096
#   slot_num  * 14 * 3276
************************************************************************/
static int getOutFileNameSize(char* filename, int srcFileSize)
{
    int size = 0;
    
    if(filename == NULL){
        printf("u must input a out file name to store data.\r\n");
        return 0;
    }
    
    if (strcmp(filename, "scm_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "dec_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "ifft_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "isca_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "cpi_data") == 0) {
        if (g_prt_msk == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "cpr_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "fft_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "sca_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "com_data") == 0) {
        if (g_prt_msk == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "scd_data") == 0) {
        if (g_prt_msk == 3)
            size = 14 * 3276 / 2;
        else size = 14 * 3276 / 2; 
        outputFormat = 64; //128;
    }else if (strcmp(filename, "tx_loopback") == 0) {
        size = srcFileSize / 2;
        outputFormat = 64; 
    }else if (strcmp(filename, "rx_loopback") == 0) {
        size = 614400;
        outputFormat = 128; 
        
    }else if (strcmp(filename, "ducinput") == 0) {
        if (g_prt_msk == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "lpf") == 0) {
        if (g_prt_msk == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "up-fifo") == 0) {
        if (g_prt_msk == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "hbf1") == 0) {
        if (g_prt_msk == 3)
            size =  61440;
        else size = 61440;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "hbf2") == 0) {
        if (g_prt_msk == 3)
            size =  61440*2;
        else size = 61440*2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "cfr") == 0) {
        if (g_prt_msk == 3)
            size =  61440*2;
        else size = 61440*2;
        outputFormat = 64; //128;
    }else {
        printf("output file must is: loopback\r\n \
            \tscm_data, dec_data, ifft_data, isca_data, cpi_data\r\n \
            \tcpr_data, fft_data, sca_data, com_data, scd_data\r\n");
        outputFormat = 0;
    
    }
    if (size == 0 && g_rsmp_nb == 0) {
        size = 100000;
        outputFormat = 64;
    }
    printf("%s OUTPUT FILE FORMAT IS: %d", filename, outputFormat);
    
    return size;
}

/* JESD_RX(iio:device4) */
	/* JESD_RX[0/1]_I(iio:device4/in_voltage[0/1]_i_xxx) */
	/* JESD_RX[0/1]_Q(iio:device4/in_voltage[0/1]_q_xxx) */
int init_iio_rxpath(int ig, size_t smp_nb)
{
	int ip;

	if (g_m_verbo > 0)
		printf("RX(g%d): Acquiring RX-JESD device\r\n", ig);
	rxdev[ig] = get_dlk_dev(ctx, RX, ig);
	ASSERT((NULL != rxdev[ig]) && "No rxdev found");
	iio_device_reg_write(rxdev[ig], 0x80000088, 0x6);// Clear all status bits

	for (ip=0; ip<DEV_PORT_NB; ip++) {
		if (g_prt_msk & (1 << ip)) {
            if (g_m_verbo > 0) printf("RX(g%dp%d): Init channels of 'JESD-RX'\r\n", ig, ip);
			iiopr_chni[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(rxdev[ig], RX, ip, 'i');
			ASSERT(iiopr_chni[ig*DEV_PORT_NB+ip] && "No rxdev_p0i found");
			iiopr_chnq[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(rxdev[ig], RX, ip, 'q');
			ASSERT(iiopr_chnq[ig*DEV_PORT_NB+ip] && "No rxdev_p0q found");
			iio_channel_enable(iiopr_chni[ig*DEV_PORT_NB + ip]);
			iio_channel_enable(iiopr_chnq[ig*DEV_PORT_NB + ip]);
		}
	}

    iio_device_set_kernel_buffers_count(rxdev[ig], 40);
    if (g_m_verbo > 0)
    	printf("RX(g%d): Creating non-cyclic buffers with %u samples\r\n", ig, smp_nb);
	iio_rxbuf[ig] = iio_device_create_buffer(rxdev[ig], smp_nb, false);
	if (NULL == iio_rxbuf[ig]) {
		perror("RX: Creating buffers failed");
		return -1;
	}
    printf("%s-->%s", rxdev[ig]->name, txdev[ig]->id);
	ASSERT(4*DEV_PORT_NB == iio_device_get_sample_size(rxdev[ig]));

	return 0;
}

/* JESD_TX(iio:device3)*/
/* JESD_TX[0/1]_I(iio:device3/out_voltage[0/2]_xxx)*/
/* JESD_TX[0/1]_Q(iio:device3/out_voltage[1/3]_xxx)*/
int init_iio_txpath(int ig, size_t smp_nb)
{
	int ip;

	if (g_m_verbo > 0) printf("TX(g%d): Acquiring TX-JESD device\r\n", ig);
	txdev[ig] = get_dlk_dev(ctx, TX, ig);
	ASSERT(txdev[ig] && "No txdev found");
	iio_device_reg_write(txdev[ig], 0x80000088, 0x6); // Clear all status bits

	for (ip=0; ip<DEV_PORT_NB; ip++) {
		if (g_prt_msk & (1 << ip)) {
            if (g_m_verbo > 0) printf("TX(g%dp%d): Init channels of TX-JESD\r\n", ig, ip);
			iiopt_chni[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(txdev[ig], TX, 2*ip, 0);
			ASSERT(iiopt_chni[ig*DEV_PORT_NB+ip] && "No txdev_p0i found");
			iiopt_chnq[ig*DEV_PORT_NB+ip] = get_dlk_chn_ss(txdev[ig], TX, 2*ip+1, 0);
			ASSERT(iiopt_chnq[ig*DEV_PORT_NB+ip] && "No txdev_p0q found");
			iio_channel_enable(iiopt_chni[ig*DEV_PORT_NB+ip]);
			iio_channel_enable(iiopt_chnq[ig*DEV_PORT_NB+ip]);
            printf("i%d, q%d be enable", ig*DEV_PORT_NB+ip, ig*DEV_PORT_NB+ip);
		}
	}
	iio_txbuf[ig] = iio_device_create_buffer(txdev[ig], smp_nb, cyc_mode ? true : false);
    printf("check0 buffer length----> %d", iio_txbuf[ig]->data_length);
	if (NULL == iio_txbuf[ig]) {
		printf("TX: Create buffers failed");
		return -1;
	}
    printf("%s-->%s", txdev[ig]->name, txdev[ig]->id);
	ASSERT(4*DEV_PORT_NB == iio_device_get_sample_size(txdev[ig]));

	return 0;
}

void rxbuf_dump(int ig)
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
	for (ip=0; ip<DEV_PORT_NB; ip++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			ip, iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB+ip]),
			ip, iio_buffer_first(iio_rxbuf[ig], iiopr_chnq[ig*DEV_PORT_NB+ip]));
	}


	printf("\r\n<-- iio_rxbuf: Samples:");
	for (ip=0; ip<DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	printf("\t");
	for (ip=0; ip<DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);

    
	psample = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB]);
	for (i = 0; (void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\r\n");
		for (ip=0; ip<DEV_PORT_NB; ip++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
    
	printf("\r\n");
	fflush(stdout);
}

void txbuf_dump(int ig)
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
	for (ip=0; ip<DEV_PORT_NB; ip++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			ip, iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB+ip]),
			ip, iio_buffer_first(iio_txbuf[ig], iiopt_chnq[ig*DEV_PORT_NB+ip]));
	}

	printf("\r\n--> iio_txbuf: RAW:");
	pbyte = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	for (i = 0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\r\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);

	printf("\r\n--> iio_txbuf: Samples:");
	for (ip=0; ip<DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	printf("\t");
	for (ip=0; ip<DEV_PORT_NB; ip++) printf(" P%dI, P%dQ", ip, ip);
	psample = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	for (i = 0;	(void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\r\n");
		for (ip=0; ip<DEV_PORT_NB; ip++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}

	printf("\r\n");
	fflush(stdout);
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
// TODO: get known
static void rxbuf_apiread(int ig, ssize_t nbytes_from_k)
{
	int ip, iblk;
	uint8_t *testbuf;

	/* p0i0, p0i1, ...	p0q0, p0q1, ... */
	testbuf = malloc(nbytes_from_k);
	for (ip=0; ip<DEV_PORT_NB; ip++) {
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
	p_inc == iio_device_get_sample_size(rxdev[ig]) == DEV_PORT_NB*4
	ismp == (p_end-p_d16)/p_inc == nbytes_from_k/(PORTS*4) <= IIO_SMP_MAX
*/
static void rxbuf_foreach(int ig, ssize_t nbytes_from_k)
{
	ptrdiff_t p_inc;
	void *p_end;
	int16_t *p_d16;
	int ip, ismp;
	int16_t i16, q16;

    FILE* fdw = NULL;

	p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	p_d16 = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[ig*DEV_PORT_NB]);
#if 0
	do {
		for (ip=0; ip<DEV_PORT_NB; ip++) {
			if (g_prt_msk & (1 << ip)) {
				i16 = *(p_d16 + 0);
				q16 = *(p_d16 + 1);
				*p_d16++ = q16;
				*p_d16++ = i16;
			}
		}
	} while ((void *)p_d16 < p_end);
//#else //defined(BUILDING_RRU_UL)
	struct bbu_payload *pl;
	do {
		pl = (struct bbu_payload *)txring_pusch_offset(rawst_iw);
		for (ismp=0; ismp<min(nbytes_from_k/DEV_PORT_NB*4, RADIO_SYM2SMP); ismp++) {
			for (ip=0; ip<DEV_PORT_NB; ip++) {
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

// Method5: iio_buffer_foreach_sample() with callback()
// TODO: get known
static ssize_t rxbuf_callback(const struct iio_channel *chn, void *p_smp, size_t smp_n, void *opt)
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

static FILE* phy_stream_analyze_rxfile(char *rxfilename)
{
    FILE* writefd = NULL;

    if(rxfilename == NULL || strlen(rxfilename) == 0)
        return writefd;

    printf("rxfilename is %s\r\n", rxfilename);
	if (!(writefd = fopen(rxfilename, "wb+"))){  // ab+
        fprintf(stderr, "Cannot open output rxfile\r\n");
        return writefd;
    }
    return writefd;
}

static int ad9009_RxPath_toFile(int ig, char *outfilename)
{
    char *p_d16, *p_end;
    ptrdiff_t p_inc;
    size_t nbytes_tx;
    int frame_size = 0;
    FILE* writefd = NULL;

    writefd = phy_stream_analyze_rxfile(outfilename);
    if (writefd == NULL) {
        printf("file: %s open faile", outfilename);
        return -1;
    }
    int channel_opt = (g_prt_msk == 0x2) ? 1 : 0;

    p_d16 = iio_buffer_first(iio_rxbuf[ig], iiopr_chni[channel_opt]);
    p_inc = iio_buffer_step(iio_rxbuf[ig]);
	p_end = iio_buffer_end(iio_rxbuf[ig]);
	//p_d16 = iio_buffer_first(iio_rxbuf[ig],iiopr_chni[ig*DEV_PORT_NB]);

    /*calcate buffer len*/
    frame_size = iio_buffer_end(iio_rxbuf[ig]) - iio_buffer_start(iio_rxbuf[ig]);
    fwrite(p_d16, p_inc, frame_size, writefd);
    fflush(writefd);
    printf("%d bytes be writed to %s\r\n", frame_size, outfilename);
    fclose(writefd);    
    return 0;
}

static int ad9009_RxPath_W4_toFile(int ig, char *outfilename)
{
    int counter = 0;
    FILE* writetxtfd = NULL;
    char tempFilename[MAXSLEN];
    char *p_d16, *p_end;
    ptrdiff_t p_inc;
    int nbytes_rx = 0;
    int fileBr_flag = 0;

    sprintf(tempFilename,"%s.txt", outfilename);
    if (!(writetxtfd = fopen(tempFilename, "at+"))){  // ab+
        fprintf(stderr, "Cannot open output rxfile\r\n");
        return -1;
    }

    int channel_opt = (g_prt_msk == 0x2) ? 1 : 0;
    while (!stop) {
        nbytes_rx = iio_buffer_refill(iio_rxbuf[ig]); 
        printf("step: %d, refill size %d", iio_buffer_step(iio_rxbuf[ig]),nbytes_rx);
        if (nbytes_rx <= 0)
            continue;

        p_inc = iio_buffer_step(iio_rxbuf[ig]); 
        p_end = iio_buffer_end(iio_rxbuf[ig]); 
        if (p_inc == 4) {
            for (p_d16 = iio_buffer_first(iio_rxbuf[ig],iiopr_chni[channel_opt]); p_d16 < p_end; p_d16 += p_inc) {
                fprintf(writetxtfd,"%02x%02x%02x%02x", p_d16[3],p_d16[2],p_d16[1],p_d16[0]);
                counter += p_inc;
                if ((fileBr_flag++ % 2 == 1) && (outputFormat == 64)) {
                    printf("\r\n");  // 8byte br
                }else if ((fileBr_flag++ % 4 == 3) && (outputFormat == 128)) {
                    printf("\r\n"); // 16byte br
                }
            }
        } else if (p_inc == 8) {
            for (p_d16 = iio_buffer_first(iio_rxbuf[ig],iiopr_chni[channel_opt]); p_d16 < p_end; p_d16 += p_inc) {
                fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x", \
                    p_d16[7],p_d16[6],p_d16[5],p_d16[4],p_d16[3],p_d16[2],p_d16[1],p_d16[0]);
                counter += p_inc;
                if (outputFormat == 64) {
                    printf("\r\n");  // 8byte br
                }else if ((fileBr_flag++ % 2 == 1) && (outputFormat == 128)) {
                    printf("\r\n"); // 16byte br
                }
            }
        }
    }
    
    fclose(writetxtfd);
    return counter;
}

static int ad9009_RxPath_W6_toFile(int ig, char *outfilename)
{
    char *p_d16, *p_end;
    ptrdiff_t p_inc;
    int frame_size = 0;
    int counter = 0;
    int pkg_cnt = 0;
    FILE* writetxtfd = NULL;
    char tempFilename[MAXSLEN];
    char *tempMem = NULL;
    char *readTemp = NULL;
    int mallocBuffsize = 0;
    
    if (g_dst_way == 6) {
        mallocBuffsize = 1024*1024*200;           
    }else {
        mallocBuffsize = 1024*1024*50;
    }
    tempMem = malloc(mallocBuffsize);
    if (tempMem == NULL)
        fprintf(stderr, "Can't malloc %d bytes\r\n", mallocBuffsize);
    readTemp = tempMem;
    memset(tempMem, 0, mallocBuffsize);
    
    sprintf(tempFilename,"/root/iqdata_out/%s.txt", outfilename);
    if (!(writetxtfd = fopen(tempFilename, "at+"))){  // ab+
        fprintf(stderr, "Can't open output rxfile, %s\r\n", tempFilename);
        return -1;
    }


    int channel_opt = (g_prt_msk == 0x2) ? 1 : 0; 
    while (!stop){
        frame_size = iio_buffer_refill(iio_rxbuf[ig]);
        printf("step: %d, refill size %d -->(%d)", iio_buffer_step(iio_rxbuf[ig]), \
            frame_size, (frame_size > 0) ? ++pkg_cnt: pkg_cnt);
        
        if (frame_size <= 0) 
            continue;

        p_d16 = iio_buffer_first(iio_rxbuf[ig],iiopr_chni[channel_opt]);
        p_inc = iio_buffer_step(iio_rxbuf[ig]);
    	p_end = iio_buffer_end(iio_rxbuf[ig]);
        counter += frame_size;
        if (counter >= mallocBuffsize)
            break;
        
        memcpy(tempMem, p_d16, frame_size);
        tempMem += frame_size;
    }

    printf("%d bytes will be writed to %s ......", counter, outfilename);
    if (outputFormat == 32) {
        for (int i = 0; i < counter; i=i+4) {
           fprintf(writetxtfd,"%02x%02x%02x%02x\r\n",readTemp[i+3],readTemp[i+2],readTemp[i+1],readTemp[i]);
        }
    }else if (outputFormat == 64) {
        for (int i = 0; i < counter; i=i+8) {
           fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x\r\n",readTemp[i+7],readTemp[i+6], \
                readTemp[i+5],readTemp[i+4],readTemp[i+3],readTemp[i+2],readTemp[i+1],readTemp[i]);
        }
    }else if (outputFormat == 128) {
        for (int i = 0; i < counter; i=i+16) {
            fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x",readTemp[i+15],readTemp[i+14], \
                readTemp[i+13],readTemp[i+12],readTemp[i+11],readTemp[i+10],readTemp[i+9],readTemp[i+8]);
            fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x\r\n",readTemp[i+7],readTemp[i+6], \
                readTemp[i+5],readTemp[i+4],readTemp[i+3],readTemp[i+2],readTemp[i+1],readTemp[i]);
        }
    } 
    fflush(writetxtfd);
    fclose(writetxtfd);
    if (readTemp) free(readTemp);

    return counter;
}

int handle_iio_rxpath(int ig, char *outfilename)
{
	ssize_t nbytes_from_k;
	uint32_t val;
    static int flag = 0;

	switch (g_dst_way) {
    case 6:
        ad9009_RxPath_W6_toFile(ig, outfilename);
        break;
    case 5:
        /*receive data to files*/
        ad9009_RxPath_W6_toFile(ig, outfilename);
        break;
	case 4:
        ad9009_RxPath_W4_toFile(ig, outfilename);
        break;
	case 3:
		rxbuf_apiread(ig, nbytes_from_k);
		break;
	case 2:
		iio_buffer_foreach_sample(iio_rxbuf[ig], rxbuf_callback, NULL);
		break;
	case 1:
		rxbuf_memcopy(ig, nbytes_from_k);
		break;
	default:
	case 0:
		rxbuf_foreach(ig, nbytes_from_k);
		break;
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	rxsnb[ig] += nbytes_from_k/iio_buffer_step(iio_rxbuf[ig])*2;
#else
    rxsnb[ig] += nbytes_from_k/iio_buffer_step(iio_rxbuf[ig]);
#endif
	return 0;
}


/* Example: fill with 0xff
  iio_buffer_foreach_sample();
  p_inc == iio_device_get_sample_size(txdev[ig]) == DEV_PORT_NB*4
  ismp == nbytes_to_k/p_inc == nbytes_to_k/(DEV_PORT_NB*4) <= IIO_SMP_MAX
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
	int ip, ismp;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	p_d16 = (int16_t *)iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	nbytes_to_k = p_end - (void *)p_d16;
#if 1
	do {
		for (ip=0; ip<DEV_PORT_NB; ip++) {
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
				for (ip=0; ip<DEV_PORT_NB; ip++) {
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

static void txbuf_memcopy(int ig, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_d16;
	int ip, ismp, nb, tmp;

	p_inc = iio_buffer_step(iio_txbuf[ig]);
	p_end = iio_buffer_end(iio_txbuf[ig]);
	p_d16 = iio_buffer_first(iio_txbuf[ig], iiopt_chni[ig*DEV_PORT_NB]);
	nbytes_to_k = p_end - p_d16;
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

static char* phy_stream_analyze_txfile(char *txfilename, int *frame_size)
{
    FILE* readfd = NULL;
    char *buf = NULL;
    int16_t i16, q16;

    if(txfilename == NULL || strlen(txfilename) == 0)
        return NULL;
    printf("txfilename is %s\r\n", txfilename);

    if (!(readfd = fopen(txfilename, "rb"))) {
        fprintf(stderr, "Cannot open input txfile\r\n");
        return NULL;
    }
    fseek(readfd, 0, SEEK_END);
    *frame_size = ftell(readfd) / 4; //FIXME: preset sample size = 4; a nasty trick is needed to get real-time sample size
    printf("Tx frame size is set to %d samples\r\n", *frame_size);
    fseek(readfd, 0, SEEK_SET);


    buf = malloc((*frame_size*4));
    *frame_size = fread(buf, 4, *frame_size, readfd);
    printf("%s len is %d bytes.", txfilename, *frame_size);    
    fclose(readfd);

#if 1   /*if data is 8bit i, 8bit q, then exchane,  if file is 16bit i, 16bit q, then not exchange*/
    for (int i = 0; i < (*frame_size)*4; i=i+4) {
        //i_dat = temp[i];
        //temp[i] = temp[i+3];
        //temp[i+3] = i_dat;
        q16 = buf[i+1];
        buf[i+1] = buf[i+2];
        buf[i+2] = q16;
    }
#endif
    return buf;
}

static int ad9009_TxPath_fromFile(int ig)
{
    FILE* readfd = NULL;
    char *p_d16, *p_end;
    ptrdiff_t p_inc;
    size_t nbytes_tx;
    int frame_size0 = 0;
    int frame_size1 = 0;
    char exTemp;

    char *filebuf0 = NULL;
    char *filebuf1 = NULL;
    uint16_t *p_d16 = NULL;
    char *buffertmp=NULL;

    if (readfd = fopen(in0Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);
		frame_size0 = ftell(readfd);
        printf("Tx frame size is set to %d samples\r\n", frame_size0/4);
        fseek(readfd, 0, SEEK_SET);
        filebuf0 = malloc(frame_size0);
        fread(filebuf0, 4, frame_size0/4, readfd);
        printf("%s len is %d bytes.", in0Filename, frame_size0); 
        fclose(readfd);
        for (int i = 0; i < frame_size0; i=i+4) {
            //exTemp = filebuf0[i];filebuf0[i] = filebuf0[i+3];filebuf0[i+3] = exTemp;
            exTemp = filebuf0[i+1];
			filebuf0[i+1] = filebuf0[i+2];
			filebuf0[i+2] = exTemp;
        }
    }

    if (readfd = fopen(in1Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);
		frame_size1 = ftell(readfd);
        printf("Tx frame size is set to %d samples\r\n", frame_size1/4);
        fseek(readfd, 0, SEEK_SET);
        filebuf1 = malloc((frame_size1));
        fread(filebuf1, 4, frame_size1/4, readfd);
        printf("%s len is %d bytes.", in1Filename, frame_size1); 
        fclose(readfd);
        for (int i = 0; i < frame_size1; i=i+4) {
            //exTemp = filebuf1[i];filebuf1[i] = filebuf1[i+3];filebuf1[i+3] = exTemp;
            exTemp = filebuf1[i+1];
			filebuf1[i+1] = filebuf1[i+2];
			filebuf1[i+2] = exTemp;
        }
    }

    /* only tx1 or have tx1&tx2, channel_opt = 0, only tx2, channel_opt 1*/    
    int channel_opt = (g_prt_msk == 0x2) ? 1 : 0;
    int counter = 0;

    if (g_prt_msk == 0x3)
        buffertmp = malloc((g_tsmp_nb*4*2));

    while (4*g_tsmp_nb*counter < frame_size0) {
        p_d16 = iio_buffer_first(iio_txbuf[ig], iiopt_chni[channel_opt]);
        p_end = iio_buffer_end(iio_txbuf[ig]);
        p_inc = iio_buffer_step(iio_txbuf[ig]);
        printf("step: %d, sample_size = %d,channel = %d", p_inc, iio_device_get_sample_size(txdev[ig]), \
                iio_device_get_channels_count(txdev[ig]));

        if (g_prt_msk == 0x1) {
            if (filebuf0 ==  NULL){
                fprintf(stderr, "tx1 buffer is NULL\r\n");
                goto lable1;
            }
            memcpy(p_d16, filebuf0 + 4*g_tsmp_nb*counter, 4*g_tsmp_nb);
            printf("memcopy %d bytes to  port%d iio-buffer", 4*g_tsmp_nb, g_prt_msk);
        } else if (g_prt_msk == 0x2) {
            if (filebuf1 == NULL){
                fprintf(stderr, "tx2 buffer is NULL\r\n");
                goto lable2;
            }
            memcpy(p_d16, filebuf1 + 4*g_tsmp_nb*counter, 4*g_tsmp_nb);
            printf("memcopy %d bytes to  port%d iio-buffer", 4*g_tsmp_nb, g_prt_msk);
        } else if (g_prt_msk == 0x3) {
            if ((frame_size0 != frame_size1 || frame_size0 == 0 || frame_size1 == 0) && \
                (filebuf1 == NULL || filebuf0 == NULL)) {
                printf("%s or %s is NULL, or two files is unequal", in0Filename, in1Filename);
                goto lable3;
            }
            for(int i=0; i<g_tsmp_nb; i++){
                memcpy(buffertmp+i*8,   filebuf0 + 4*g_tsmp_nb*counter +i*4, 4);
                memcpy(buffertmp+i*8+4, filebuf1 + 4*g_tsmp_nb*counter +i*4, 4);
            }
            memcpy(p_d16, buffertmp, g_tsmp_nb*4*2);
            printf("memcopy %d bytes to  port%x iio-buffer", 4*g_tsmp_nb*2, g_prt_msk);
        }

        nbytes_tx = iio_buffer_push(iio_txbuf[ig]);
    	if (nbytes_tx < 0) {
    		fprintf(stderr, "Error pushing buf %d\r\n", (int) nbytes_tx);
    		return -1;  //CleanUP();
    	} else {
    		printf("push %d bytes of data\r\n", (int) nbytes_tx);
    	}
        while(!stop);
        counter++;
    }


lable3:    
    if (buffertmp)
        free(buffertmp);buffertmp = NULL;
lable1:    
    if (filebuf0)
        free(filebuf0);filebuf0 = NULL;
lable2:    
    if (filebuf1)
        free(filebuf1);filebuf1 = NULL;

    return nbytes_tx;
}

static int ad9009_TxPath_W5_fromFile(int ig)
{
    FILE* readfd = NULL;
    char *p_d16, *p_end;
    ptrdiff_t p_inc;
    size_t nbytes_tx;
    int frame_size0 = 0;
    int frame_size1 = 0;
    char exTemp;
    int times = 0;
    struct timespec old, new;
    char *filebuf0 = NULL;
    char *filebuf1 = NULL;
    uint16_t *p_d16 = NULL;
    char *buffertmp=NULL;
    
     /* only tx1 or have tx1&tx2, channel_opt = 0, only tx2, channel_opt 1*/    
    int channel_opt = (g_prt_msk == 0x2) ? 1 : 0;
    int counter = 0;
    
    if (readfd = fopen(in0Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);frame_size0 = ftell(readfd);
        printf("Tx frame size is set to %d samples\r\n", frame_size0/4);
        fseek(readfd, 0, SEEK_SET);
        
        filebuf0 = malloc((frame_size0));
        fread(filebuf0, 4, frame_size0/4, readfd);
        //fread(p_d16, 4, frame_size0/4, readfd);
        printf("%s len is %d bytes.", in0Filename, frame_size0); 
        fclose(readfd);
    }


    if (readfd = fopen(in1Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);frame_size1 = ftell(readfd);
        printf("Tx frame size is set to %d samples\r\n", frame_size1/4);
        fseek(readfd, 0, SEEK_SET);
        filebuf1 = malloc((frame_size1));
        fread(filebuf1, 4, frame_size1/4, readfd);
        //fread(p_d16, 4, frame_size1/4, readfd);
        printf("%s len is %d bytes.", in1Filename, frame_size1); 
        fclose(readfd);
    }

    if (g_prt_msk == 0x3) {
        buffertmp = malloc((g_tsmp_nb*4*2));
    }
    
#if 1        
    while (4*g_tsmp_nb*counter < frame_size0) {

        clock_gettime(CLOCK_MONOTONIC, &old);
        p_d16 = iio_buffer_first(iio_txbuf[ig], iiopt_chni[channel_opt]);
        
        if (g_prt_msk == 0x1) {
            if (filebuf0 ==  NULL){
                fprintf(stderr, "tx1 buffer is NULL\r\n");
                goto lable1;
            }
            memcpy(p_d16, filebuf0 + 4*g_tsmp_nb*counter, 4*g_tsmp_nb);
            
        }else if (g_prt_msk == 0x2) {
            if (filebuf1 == NULL){
                fprintf(stderr, "tx2 buffer is NULL\r\n");
                goto lable2;
            }
            memcpy(p_d16, filebuf1 + 4*g_tsmp_nb*counter, 4*g_tsmp_nb);
            
        }else if (g_prt_msk == 0x3) {
            if ((frame_size0 != frame_size1 || frame_size0 == 0 || frame_size1 == 0) && \
                (filebuf1 == NULL || filebuf0 == NULL)) {
                printf("%s or %s is NULL, or two files is unequal", in0Filename, in1Filename);
                goto lable3;
            }
            for(int i=0; i<g_tsmp_nb; i++){
                memcpy(buffertmp+i*8,   filebuf0 + 4*g_tsmp_nb*counter +i*4, 4);
                memcpy(buffertmp+i*8+4, filebuf1 + 4*g_tsmp_nb*counter +i*4, 4);
            }
            memcpy(p_d16, buffertmp, g_tsmp_nb*4*2);
        }
       
        /*commit buffer to system  ..................*/
        nbytes_tx = iio_buffer_push(iio_txbuf[ig]);
        while(!stop);
        counter++;
      }
#endif      


lable3:    
    if (buffertmp)
        free(buffertmp);buffertmp = NULL;
lable1:    
    if (filebuf0)
        free(filebuf0);filebuf0 = NULL;
lable2:    
    if (filebuf1)
        free(filebuf1);filebuf1 = NULL;
    
    return nbytes_tx;
}

int handle_iio_txpath(int ig)
{
	ssize_t nbytes_to_k;
	int ip;
	uint32_t val;

	switch (g_src_way) {
    case 6:
        /*send data from files*/
		nbytes_to_k = ad9009_TxPath_fromFile(ig);
        break;
    case 5:
        /*send data from files*/
		nbytes_to_k = ad9009_TxPath_W5_fromFile(ig);
        break;
	case 4:
        nbytes_to_k = ad9009_TxPath_fromFile(ig);
        break;
	case 3:
		printf("\r\ncase3/4 TODO\r\n");
		break;
	case 2:
        ad9009_TxPath_fromFile(ig);
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
	txsnb[ig] += nbytes_to_k/iio_buffer_step(iio_txbuf[ig])*2;
#else
	txsnb[ig] += nbytes_to_k/iio_buffer_step(iio_txbuf[ig]);
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
	int ig = 0, ip;

    if (NULL == ctx) {
        if (g_m_verbo > 0)
            printf("* Acquiring IIO context\r\n");
		ctx = iio_create_local_context();
    	ASSERT((NULL != ctx) && "No context");
		ret = iio_context_get_devices_count(ctx);
    	ASSERT((ret > 0) && "No devices");
    }

    ASSERT(cfg_stream_chn(ctx, &txchn_rfcfg, TX, 0)&& "No phy_txport found");
 
	for (ig=0; ig<TOT_CHIP_NB; ig++) {

		init_phy(ig);

        if (g_pth_msk & (EN_IIO_PATH_TX << ig)) {
            //ASSERT((txdev_smp_nb > 0) && (txdev_smp_nb < (1024*1024))); //IIO_SMP_MAX
            init_iio_txpath(ig, txdev_smp_nb);
        }
        
		if (g_pth_msk & (EN_IIO_PATH_RX << ig)) {
            //ASSERT((rxdev_smp_nb > 0) && (rxdev_smp_nb <= (1024*1024)));
            init_iio_rxpath(ig, rxdev_smp_nb);
        }
        //iio_context_set_timeout(ctx, 500);

#if 0       //set trigger
        if (!phy || !iio_device_is_trigger(phy)) {
    		fprintf(stderr,"No trigger found (try setting up the iio-trig-hrtimer module)");
    		shutdown();;
	    }
        if (iio_device_set_trigger(phy, rxdev[ig])) {
    		fprintf(stderr,"Could not set trigger\r\n");
    		shutdown();
	    }
#endif
	}
}


static int getFileSize(char* filename)
{
    int size = 0;
    FILE* readfd = NULL;

    if(filename == NULL)
        return 0;

    if (!(readfd = fopen(filename, "rb"))) {
        fprintf(stderr, "------>Cannot open input txfile %s\r\n", filename);
        return size;
    }
    fseek(readfd, 0, SEEK_END);
    size = ftell(readfd) / 4;
    fseek(readfd, 0, SEEK_SET);
    fclose(readfd);
    printf("Tx frame size is set to %d samples\r\n", size);
    
    return size;
}

static int initConfig()
{
    int temp;

    if (g_pth_msk & (EN_IIO_PATH_TX)) {
            if (in0Filename != NULL) {
                g_tsmp_nb = getFileSize(in0Filename);
                printf("really file size is %d", g_tsmp_nb*4);
                if (g_tsmp_nb <= 0 && (g_tsmp_nb * 4 > 39 * 1024 * 1024)) { //39M
                    printf("file: %d, can't send more than 39M", g_tsmp_nb*4);
                    return -1;
                }
            }
            /*calculate tx buffer size .........*/
            temp = g_tsmp_nb;
            while (1) {
                if (temp > 64*1024*1024) {  // less than 1M break
                    temp = temp / 2;
                    continue;
                }
                if (g_prt_msk == 0x3)
                    g_tsmp_nb = (temp > 1024*1024/2) ? (temp/2) : temp;
                else g_tsmp_nb = temp;
                printf("will set %d samples(%d bytes) as tx buffer", g_tsmp_nb, g_tsmp_nb*4);
                break;
            }
    }

    if (g_pth_msk & (EN_IIO_PATH_RX)) {
        /*calculate rx buffer size .........*/
        if (out0Filename != NULL) {
            bypassDatasize = getOutFileNameSize(out0Filename, g_tsmp_nb);
            if (bypassDatasize == 0) {
                printf("output filename error!");
                fprintf(stderr, "must select a name\r\n");
                return -1;
            }
        }
    }
    printf("txbuff = %d, rxbuff = %d", g_tsmp_nb, bypassDatasize);
    return 0;
}

/*receive thread, monitor rx dma data, and then write data to file*/
static void *monitor_thread_fn(void *data)
{
    printf("RX thread_fn......................");
    /*handle rxpath ........*/
    if (g_pth_msk & (EN_IIO_PATH_RX << 0)) {
        printf("RX......");
        handle_iio_rxpath(0, out0Filename);
    }
    
    return (void *)0;
}

int main(int argc, char *argv[])
{
	int ig, ip;
	struct timespec tm_xs, tm_xe;
	double tm_us;
	struct timespec old, new;
    pthread_t monitor_thread;
    int ret;

    signal(SIGINT, handle_sig);
	get_args(argc, argv); 
    dmp_libiio_version();
    ret = initConfig();
    if (ret < 0){
        fprintf(stderr, "Some inappropriate parameters may have been passed in\r\n");
        return 0;
    }
    
    /*init devices and then create buffer size .........*/
    libiio_app_startup(bypassDatasize, g_tsmp_nb);
    

    /*create a thread, use to monitor receive..........*/
    if (g_pth_msk & (EN_IIO_PATH_RX)) {
        ret = pthread_create(&monitor_thread, NULL, monitor_thread_fn, NULL);
    	if (ret) {
    		fprintf(stderr, "Failed to create monitor thread: %s\r\n",strerror(-ret));
    	}
    }

/******************************************************************/
/******************************************************************/
    clock_gettime(CLOCK_MONOTONIC, &tm_xs);
    do {
        for (ig=0; ig<TOT_CHIP_NB; ig++) {

            /*handle txpath ........*/
    		if (g_pth_msk & (EN_IIO_PATH_TX << ig)) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_iio_txpath(ig);
                clock_gettime(CLOCK_MONOTONIC, &new);
                //printf("handle_iio_txpath() take %.0f us REAL-TIME\r\n", elapse_us(&new, &old));
		    }            
            //printf("RX[0] %4.6f MiSmp; TX[0] %4.6f MiSmp\r\n",rxsnb[0]/1e6, txsnb[0]/1e6);
            //fflush(stdout);
            
        }
    }while(0); //while(!stop);


    pthread_join(monitor_thread, NULL);
    
/******************************************************************/
	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
	tm_us = elapse_us(&tm_xe, &tm_xs);
	printf("\r\n");
	for (ip=0; ip<DEV_PORT_NB; ip++) {
		if (g_pth_msk & (EN_IIO_PATH_RX << ig))
			printf("RX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\r\n",
				ip, rxsnb[0]/1e6, tm_us, rxsnb[0]/tm_us*32);        
	}
    if (g_dst_way == 2)
        printf("w 2 mode: receive %d\r\n", outCounter);
	shutdown();
    fclose(writefd);
    
	return 0;
}

