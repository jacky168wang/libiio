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

#include "iio-private.h"



#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif
#include "common_jacky.h"
#include "rru_bbu.h"

#define AD9009_DEBUG (1)
#if(AD9009_DEBUG)
//printf("=====> %s(%d): %s() "#fmt"\n",__FILE__,__LINE__,__func__,##args)
#define DBG_Printf(fmt,args...)  printf("==> (%d): %s()>>->> "#fmt"\n",__LINE__,__func__,##args)
#else
#define DBG_Printf(fmt,args...)
#endif

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
#define MAXLEN_FILENAME  128
/* RX is input, TX is output */
enum iodev { RX, TX };

/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
    double gain_or_atten;
    bool qtracking;
    bool loltracking;//TX ONLY
#if 0
    const char *filename;
    bool on;
    bool nostop;
    size_t frame_num;
#endif
};

// Stream configurations
struct stream_cfg rxcfg = {
	0,
	0,
	GHZ(3.5), //long long rx_lo_freq;
	0, //double rx_gain;
	1, //bool rx_qtracking;
	0, //bool unused! rx_loltracking;
#if 0
	"./rx_data.bin", //dst file path
	1, //rx is on;
	0, //non-stop is disabled
	10 //10 frames
#endif
};

struct stream_cfg txcfg = {
	0,
	0,
	GHZ(3.5), //long long tx_lo_freq;
	-20, //double tx_atten;
	1, //bool tx_qtracking;
	1, //bool tx_loltracking;
#if 0
	NULL, //src file path
	0, //tx is off;
	0, //non-stop is disabled
	10 //10 frames
#endif
};

/* static scratch mem for strings */
static char tmpstr[64];
/* IIO structs required for streaming */
static struct iio_context *ctx;
// Streaming devices
static struct iio_device *txdev[TOT_CHIP_NB];
static struct iio_device *rxdev[TOT_CHIP_NB];
// one libiio buffer for each device
struct iio_buffer *iiopr_cbuf[TOT_CHIP_NB];
struct iio_buffer *iiopt_cbuf[TOT_CHIP_NB];
// sample counters per device
static size_t rxsnb[TOT_CHIP_NB];
static size_t txsnb[TOT_CHIP_NB];
// 2x2 Streaming channels (2 Streaming ports) for each device
struct iio_channel *iiopr_chni[TOT_PORT_NB];
struct iio_channel *iiopr_chnq[TOT_PORT_NB];
struct iio_channel *iiopt_chni[TOT_PORT_NB];
struct iio_channel *iiopt_chnq[TOT_PORT_NB];

#define EN_IIO_PATH_RX 0x01
#define EN_IIO_PATH_TX 0x02
static unsigned int iiopr_smp_nb = 0; //IIOPR_SMP_NB_NOW;
static unsigned int iiopt_smp_nb = 0; //IIOPT_SMP_NB_NOW;
       unsigned int iio_path_x = EN_IIO_PATH_TX | EN_IIO_PATH_RX;
static unsigned int do_smp_way = 0;
static unsigned int mode_verbo = 0;
static unsigned int bytetofill = 0x7832;
static unsigned int ports_mask = 0x1;
static unsigned int cyc_mode = 0;

static FILE* fd = NULL;
static FILE* writefd = NULL;
static int bypassDatasize = 0;
static int outputFormat = 0;
static int outCounter = 0;

static char in0Filename[MAXLEN_FILENAME];
static char in1Filename[MAXLEN_FILENAME];
static char out0Filename[MAXLEN_FILENAME];
static char out1Filename[MAXLEN_FILENAME];


static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION]\n"
		" -h\tshow this help\n"
		" -n\tset samples number[4~1048576] for iio TX/RX each time\n"
		" -t\tenable TX datapath only\n"
		" -r\tenable RX datapath only\n"
        " -f\tTX file what you want to send by tx1.\n"
        " -s\tTX file what you want to send by tx2\n"
        " -c\tiio create buffer cycle.\n"
        " -w\tselect one way to deal with samples\n"
		"   \t\t0) memcpy to user buffer in device level\n"
		"   \t\t1) for each sample frankly in device level\n"
		"   \t\t2) for each sample with callback() in channel level\n"
		"   \t\t3) call iio_channel_read_raw() for all samples of one channel\n"
		"   \t\t4) call iio_channel_read() for all samples of one channel\n"
		"   \t\t5) don't convert data format, send big file\n"
		"   \t\t6) read/write file to tx/rx\n"
		" -v\tbe verbose [0,1,2,3]\n"
		" -o\tchoose module what u wana get, and then generate module.txt\n"
		"   \t\t tx_loopback, scm_data,dec_data, ifft_data,cpi_data\n"
		"   \t\t rx_loopback, scd_data,com_data,  fft_data,cpr_data\n"
		"   \t./stream-test -w 6 -f filename1 -s filename2 -p port -o module  -->module.txt\n"
		"   \t./stream-test -w 6 -f filename1 -s filename2 -p 3 -o loopback   -->tx_loop_back.txt\n"
		"   \t./stream-test -w 6 -f filename1 -s filename2 -p 3 -o scm_data   -->scm_data.txt\n"
		"   \t./stream-test -w 6 -f filename1 -p 1 -o ifft_data               -->ifft_data.txt\n"
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
		case 'p': ports_mask = strtoul(optarg, NULL, 16);	break;
		case 'm': iiopr_smp_nb = strtoul(optarg, NULL,  0);	break;
		case 'n': iiopt_smp_nb = strtoul(optarg, NULL,  0);	break;
		case 'w': do_smp_way = strtoul(optarg, NULL,  0);	break;
		case 'v': mode_verbo = strtoul(optarg, NULL,  0);	break;
		case 'b': bytetofill = strtoul(optarg, NULL, 0);	break;
		case 'x': iio_path_x = strtoul(optarg, NULL, 16);	break;
        case 't': iio_path_x = 2;                           
                  iiopt_smp_nb = strtoul(optarg, NULL, 0);
                  break;
        case 'r': iio_path_x = 1;
                  iiopr_smp_nb = strtoul(optarg, NULL, 0);
                  break;
        case 'c': cyc_mode = 1;                             break;
		case 'h': usage(); exit( EXIT_FAILURE );			break;
        case 'f': tmpsize = strlen(optarg); 
                  if(tmpsize > 0)
                    memcpy(in0Filename, optarg, tmpsize);
                  DBG_Printf("%s-->%d", in0Filename, tmpsize);
                  break;
                  
        case 's': tmpsize = strlen(optarg); 
                  if(tmpsize > 0)
                    memcpy(in1Filename, optarg, tmpsize);
                  DBG_Printf("%s-->%d", in1Filename, tmpsize);
                  break;

        case 'o': 
                  tmpsize = strlen(optarg); 
                  if(tmpsize > 0)
                    memcpy(out0Filename, optarg, tmpsize);
                  DBG_Printf("%s-->%d", out0Filename, tmpsize);
                  break;
                  
		case '?':
			if (isprint(optopt)) {
				fprintf(stderr,
						"ERROR: unrecognised option \"%c\"\n",
						(char) optopt);
				exit(EXIT_FAILURE);
			}
			break;
		default:
			fprintf(stderr, "ERROR: unrecognised command line option\n");
			exit(EXIT_FAILURE);
			break;
		}
	}
#if 0
	/* take first residual non option argv element as interface name. */
	if ( optind < argc ) {
		str_devname = argv[ optind ];
	}
	if( !str_devname ) {
		fprintf( stderr, "ERROR: No interface was specified\n");
		usage();
		exit( EXIT_FAILURE );
	}
#endif
	printf( "CURRENT SETTINGS:\n" );
	printf( "iiopr_smp_nb(-n):   %u\n", iiopr_smp_nb );
	printf( "iiopt_smp_nb(-n):   %u\n", iiopt_smp_nb );
	printf( "iio_path_x(-x):   %u\n", iio_path_x );
	printf( "do_smp_way(-w):   %u\n", do_smp_way );
	printf( "bytetofill(-b):   %u\n\n", bytetofill);
    printf( "cyc_mode(-b):   %u\n\n", cyc_mode);
    printf( "iio_path_x:   %u\n\n", iio_path_x);
}





/* cleanup and exit */
void libiio_app_shutdown(void)
{
	int iprt, igrp;

	if (mode_verbo > 0)
		printf("* Destroying buffers\n");

	for (igrp = 0; igrp < TOT_CHIP_NB; igrp++) {
		if (iiopr_cbuf[igrp]){
          iio_buffer_destroy(iiopr_cbuf[igrp]);
          printf("disable rx buffer.........\n");
		}
		if (iiopt_cbuf[igrp]) {
           iio_buffer_destroy(iiopt_cbuf[igrp]);
           printf("disable tx buffer.........\n");
        }
        
		if (mode_verbo > 0)
			printf("* Disabling streaming channels\n");
		for (iprt = 0; iprt < TOT_PORT_NB; iprt++) {
			if (iiopr_chni[iprt]) iio_channel_disable(iiopr_chni[iprt]);
			if (iiopr_chnq[iprt]) iio_channel_disable(iiopr_chnq[iprt]);
			if (iiopt_chni[iprt]) iio_channel_disable(iiopt_chni[iprt]);
			if (iiopt_chnq[iprt]) iio_channel_disable(iiopt_chnq[iprt]);
            printf("disable channel.........\n");
		}
	}

	if (mode_verbo > 0)
		printf("* Destroying context\n");
	if (ctx) iio_context_destroy(ctx);
	exit(0);
}





static volatile sig_atomic_t process_stop = 0;
static void handle_sig(int sig)
{
	printf("\nWaiting for process to finish...\n");
	process_stop = 1;
}





/* check return value of attr_write function */
static void errchk(int v, const char* what)
{
	if (v < 0) {
		fprintf(stderr, "Error %d writing to channel \"%s\"" \
			"\nvalue may not be supported.\n", v, what);
		libiio_app_shutdown();
	}
}





/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}





/* read attribute: long long int */
static long long rd_ch_lli(struct iio_channel *chn, const char* what)
{
	long long val;

	errchk(iio_channel_attr_read_longlong(chn, what, &val), what);

	//printf("\tchn[%s]: %s=%lld\n", chn->name, what, val);
	printf("\tiio_channel: %s=%lld\n", what, val);
	return val;
}





/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}





/* helper function generating channel names */
static char* get_ch_name_mod(const char* type, int id, char modify)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d_%c", type, id, modify);
	return tmpstr;
}





/* helper function generating channel names */
static char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}





/* returns ad9371 phy device */
static struct iio_device* get_phy_dev(struct iio_context *ctx)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "adrv9009-phy");
	ASSERT(dev && "No adrv9009-phy found");
	return dev;
}





/* finds AD9371 streaming IIO devices */
static bool get_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
	switch (d) {
	    case TX: *dev = iio_context_find_device(ctx, "jesd204tx-layer3"); return *dev != NULL;
	    case RX: *dev = iio_context_find_device(ctx, "jesd204rx-layer3"); return *dev != NULL;
	    //adolph case ORX: *dev = iio_context_find_device(ctx, "jesd204or-layer3"); return *dev != NULL;
		default: ASSERT(0); return false;
	}
}





/* finds streaming IIO channels */
static bool get_stream_chn(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, char modify, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	return *chn != NULL;
}





/* finds phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
		case RX: *chn = iio_device_find_channel(get_phy_dev(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
		case TX: *chn = iio_device_find_channel(get_phy_dev(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
		default: ASSERT(0); return false;
	}
}





static bool set_phy_chn(struct iio_context *ctx, enum iodev d, struct iio_channel *chn)
{
	struct stream_cfg *cfg;
	switch (d) {
	case RX:
		cfg = &rxcfg; //TODO: cfg = (struct stream_cfg *)(chn->userdata);
		iio_channel_attr_write_double(chn, "hardwaregain", cfg->gain_or_atten);
		iio_channel_attr_write_bool(chn, "quadrature_tracking_en", cfg->qtracking);
		break;
	case TX:
		cfg = &txcfg; //TODO: cfg = (struct stream_cfg *)(chn->userdata);
		iio_channel_attr_write_double(chn, "hardwaregain", cfg->gain_or_atten);
		iio_channel_attr_write_bool(chn, "quadrature_tracking_en", cfg->qtracking);
		iio_channel_attr_write_bool(chn, "lo_leakage_tracking_en", cfg->loltracking);
		break;
	default: ASSERT(0); return false;
	}
}





/* finds AD9371 local oscillator IIO configuration channels */
static bool get_lo_chn(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_phy_dev(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_phy_dev(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: ASSERT(0); return false;
	}
}





static bool set_lo_chn(struct iio_context *ctx, enum iodev d, struct iio_channel *chn)
{
	struct stream_cfg *cfg;

	switch (d) {
	case RX:
		cfg = &rxcfg; //TODO: cfg = (struct stream_cfg *)(chn->userdata);
		iio_channel_attr_write_longlong(chn, "RX_LO_frequency", cfg->lo_hz);
		break;
	case TX:
		cfg = &txcfg; //TODO: cfg = (struct stream_cfg *)(chn->userdata);
		iio_channel_attr_write_longlong(chn, "TX_LO_frequency", cfg->lo_hz);
		break;
	default: ASSERT(0); return false;
	}
	return true;
}





/* applies streaming configuration through IIO */
static bool cfg_stream_chn(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;

	// Configure phy and lo channels
	if (!get_phy_chan(ctx, type, chid, &chn)) { return false; }
    //set_phy_chn(ctx, type, chn);
	rd_ch_lli(chn, "rf_bandwidth");
	rd_ch_lli(chn, "sampling_frequency");
    
	return true;
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
        DBG_Printf("u must input a out file name to store data.\n");
        return 0;
    }
    
    if (strcmp(filename, "scm_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "dec_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "ifft_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "isca_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "cpi_data") == 0) {
        if (ports_mask == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "cpr_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "fft_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "sca_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "com_data") == 0) {
        if (ports_mask == 3)
            size = 16 * 14 * 4096 / 8 / 2; // 16:128bit  14*4096
        else size = 8 * 14 * 4096 / 4 / 2; // / 4 only one layer
        outputFormat = 32; //64;
    }else if (strcmp(filename, "scd_data") == 0) {
        if (ports_mask == 3)
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
        if (ports_mask == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "lpf") == 0) {
        if (ports_mask == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "up-fifo") == 0) {
        if (ports_mask == 3)
            size =  61440/2;
        else size = 61440/2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "hbf1") == 0) {
        if (ports_mask == 3)
            size =  61440;
        else size = 61440;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "hbf2") == 0) {
        if (ports_mask == 3)
            size =  61440*2;
        else size = 61440*2;
        outputFormat = 64; //128;
    }else if (strcmp(filename, "cfr") == 0) {
        if (ports_mask == 3)
            size =  61440*2;
        else size = 61440*2;
        outputFormat = 64; //128;
    }else {
        printf("output file must is: loopback\n \
            \tscm_data, dec_data, ifft_data, isca_data, cpi_data\n \
            \tcpr_data, fft_data, sca_data, com_data, scd_data\n");
        outputFormat = 0;
    
    }
    if (size == 0 && iiopr_smp_nb == 0) {
        size = 100000;
        outputFormat = 64;
    }
    DBG_Printf("%s OUTPUT FILE FORMAT IS: %d", filename, outputFormat);
    
    return size;
}







/* JESD_RX(iio:device4) */
/* JESD_RX[0/1]_I(iio:device4/in_voltage[0/1]_i_xxx) */
/* JESD_RX[0/1]_Q(iio:device4/in_voltage[0/1]_q_xxx) */
int init_ad9009_rxpath(int igrp, size_t smp_nb)
{
	int iprt;

#if 0
	/* ad9371-phy(iio:device1) */
	/* PHY_RX[0/1/2/3](iio:device1/in_voltage[n]_xxx */
	/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
        if (mode_verbo > 0)
            printf("AD9371_RX(group%d): Initializing PHY port%d\n",
                igrp, iprt);
		ASSERT(cfg_stream_chn(ctx, &rxcfg, RX, iprt) && "No phy_rxport found");
	}

#endif
	if (mode_verbo > 0)
		printf("AD9371_RX(group%d): Acquiring RX-JESD device\n", igrp);    
	ASSERT(get_stream_dev(ctx, RX, &rxdev[igrp]) && "No rxdev found");


	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (ports_mask & (1 << iprt)) {
            if (mode_verbo > 0)
                printf("AD9371_RX(group%d): Initilizing channels of RX-JESD port%d\n",
                    igrp, iprt);
			ASSERT(get_stream_chn(ctx, RX, rxdev[igrp], iprt, 'i',
				&iiopr_chni[igrp*DEV_PORT_NB+iprt]) && "No rxdev_p0i found");
			ASSERT(get_stream_chn(ctx, RX, rxdev[igrp], iprt, 'q', 
				&iiopr_chnq[igrp*DEV_PORT_NB+iprt]) && "No rxdev_p0q found");
			iio_channel_enable(iiopr_chni[igrp*DEV_PORT_NB+iprt]);
			iio_channel_enable(iiopr_chnq[igrp*DEV_PORT_NB+iprt]);
            DBG_Printf("rx i%d, q%d", igrp*DEV_PORT_NB+iprt, igrp*DEV_PORT_NB+iprt);
		}
	}

    iio_device_set_kernel_buffers_count(rxdev[igrp], 40);
    if (mode_verbo > 0)
    	printf("AD9371_RX(group%d): Creating non-cyclic buffers with %u samples\n",
    	    igrp, smp_nb);
	iiopr_cbuf[igrp] = iio_device_create_buffer(rxdev[igrp], smp_nb, false);
	if (NULL == iiopr_cbuf[igrp]) {
		perror("AD9371_RX: Creating buffers failed");
		return -1;
	}
     DBG_Printf("%s-->%s", rxdev[igrp]->name, txdev[igrp]->id);
	//ASSERT(4*DEV_PORT_NB == iio_device_get_sample_size(rxdev[igrp]));


	return 0;
}





/* JESD_TX(iio:device3)*/
/* JESD_TX[0/1]_I(iio:device3/out_voltage[0/2]_xxx)*/
/* JESD_TX[0/1]_Q(iio:device3/out_voltage[1/3]_xxx)*/
int init_ad9009_txpath(int igrp, size_t smp_nb)
{
	int iprt;

	if (mode_verbo > 0)
		printf("AD9371_TX(group%d): Acquiring TX-JESD device\n", igrp);
	ASSERT(get_stream_dev(ctx, TX, &txdev[igrp]) && "No txdev found");
	//iio_device_reg_write(txdev[igrp], 0x80000088, 0x6);// Clear all status bits

    DBG_Printf("DEV_PORT_NB = %d", DEV_PORT_NB);
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (ports_mask & (1 << iprt)) {
            if (mode_verbo > 0)
                printf("AD9009_TX(group%d): Initilizing channels of TX-JESD port%d\n",
                    igrp, iprt);
			ASSERT(get_stream_chn(ctx, TX, txdev[igrp], 2*iprt,   0, 
				&iiopt_chni[igrp*DEV_PORT_NB+iprt]) && "No txdev_p0i found");
			ASSERT(get_stream_chn(ctx, TX, txdev[igrp], 2*iprt+1, 0, 
				&iiopt_chnq[igrp*DEV_PORT_NB+iprt]) && "No txdev_p0q found");
			iio_channel_enable(iiopt_chni[igrp*DEV_PORT_NB+iprt]);
			iio_channel_enable(iiopt_chnq[igrp*DEV_PORT_NB+iprt]);
            DBG_Printf("i%d, q%d be enable", igrp*DEV_PORT_NB+iprt, igrp*DEV_PORT_NB+iprt);
		}
	}
    
	iiopt_cbuf[igrp] = iio_device_create_buffer(txdev[igrp], smp_nb, cyc_mode ? true : false);
    DBG_Printf("check0 buffer length----> %d", iiopt_cbuf[igrp]->data_length);
	if (NULL == iiopt_cbuf[igrp]) {
		DBG_Printf("AD9371_TX: Create buffers failed");
		return -1;
	}
    DBG_Printf("%s-->%s", txdev[igrp]->name, txdev[igrp]->id);
	//ASSERT(4*DEV_PORT_NB == iio_device_get_sample_size(txdev[igrp]));


	return 0;
}





void show_libiio_rxbuf(int igrp)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int iprt;
	uint16_t *p_d16;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iiopr_cbuf[igrp]);
	p_end = iio_buffer_end(iiopr_cbuf[igrp]);
	printf("<-- iio_rxbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			iprt, iio_buffer_first(iiopr_cbuf[igrp], iiopr_chni[igrp*DEV_PORT_NB+iprt]),
			iprt, iio_buffer_first(iiopr_cbuf[igrp], iiopr_chnq[igrp*DEV_PORT_NB+iprt]));
	}


	printf("\n<-- iio_rxbuf: Samples:");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) printf(" P%dI, P%dQ", iprt, iprt);
	printf("\t");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) printf(" P%dI, P%dQ", iprt, iprt);

    
	psample = iio_buffer_first(iiopr_cbuf[igrp], iiopr_chni[igrp*DEV_PORT_NB]);
	for (i = 0; (void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\n");
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
    
	printf("\n");
	fflush(stdout);
}





void show_libiio_txbuf(int igrp)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int iprt;
	uint16_t *p_d16;
	struct iio_sample *psample;


	p_inc = iio_buffer_step(iiopt_cbuf[igrp]);
	p_end = iio_buffer_end(iiopt_cbuf[igrp]);
	printf("--> iio_txbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			iprt, iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB+iprt]),
			iprt, iio_buffer_first(iiopt_cbuf[igrp], iiopt_chnq[igrp*DEV_PORT_NB+iprt]));
	}


	printf("\n--> iio_txbuf: RAW:");
	pbyte = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB]);
	for (i = 0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);


	printf("\n--> iio_txbuf: Samples:");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) printf(" P%dI, P%dQ", iprt, iprt);
	printf("\t");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) printf(" P%dI, P%dQ", iprt, iprt);
	psample = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB]);
	for (i = 0;	(void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\n");
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}

	printf("\n");
	fflush(stdout);
}





/* Method0: sample by sample, Example: swap Real(I) and Imag(Q)
	p_inc == iio_device_get_sample_size(rxdev[igrp]) == DEV_PORT_NB*4
	ismp == (p_end-p_d16)/p_inc == nbytes_from_k/(PORTS*4) <= IIO_SMP_MAX
*/
static void foreach_samples_rxbuf(int igrp, ssize_t nbytes_from_k)
{
	ptrdiff_t p_inc;
	char *p_end;
	char *p_dat;
	int iprt, ismp;
	int16_t i16, q16;

    FILE* fdw = NULL;

	p_inc = iio_buffer_step(iiopr_cbuf[igrp]);
	p_end = iio_buffer_end(iiopr_cbuf[igrp]);
	p_dat = iio_buffer_first(iiopr_cbuf[igrp],
				iiopr_chni[igrp*DEV_PORT_NB]);
#if 0
	do {
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			if (ports_mask & (1 << iprt)) {
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
			for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
				pl->iqs[iprt][ismp].i8 = *p_d16++;	// LSB of I[ismp][iprt]
				p_d16++;							// MSB of I[ismp][iprt]
				pl->iqs[iprt][ismp].q8 = *p_d16++;	// LSB of Q[ismp][iprt]
				p_d16++;							// MSB of Q[ismp][iprt]
			}
		}
		nbytes_from_k -= ismp*p_inc;
		rawst_iw++;
	} while (0 != nbytes_from_k);
#endif
}





// Method1: sample by sample
// Example: copy Real(I) and Imag(Q) to an array of userspace buffers
// The HARDWARE format of each sample :
// cat /sys/bus/iio/devices/iio:device1/scan_elements/in_voltage0_type
// le:s12/16>>4
static void memcopy_samples_rxbuf(int igrp, ssize_t nbytes_from_k)
{
	char *p_d16;
	ptrdiff_t p_inc;
	size_t nb;

	ASSERT(1 == DEV_PORT_NB); /* samples of P0 and P1 are interleaved */

	p_d16 = iio_buffer_first(iiopr_cbuf[igrp], iiopr_chni[igrp*DEV_PORT_NB]);
	do {
		nb = min(nbytes_from_k, RADIO_SYM2SMP*4);
		memcpy((void *)rawst_cbuf[rawst_iw] + PUSCH_PLDOF, p_d16, nb/2);
		p_d16 += nb;
		rawst_iw++;
		if (RAWST_BUF_NB_NOW == rawst_iw) rawst_iw = 0;
		nbytes_from_k -= nb;
	} while (nbytes_from_k > 0);
}





// Method2: iio_buffer_foreach_sample() with callback()
//  TODO: get known
static ssize_t rxbuf_callback(const struct iio_channel *chn,
			void *p_smp, size_t smp_n, void *opt)
{
    char * p_dat = NULL;
    
	if (!iio_channel_is_enabled(chn))
		return 0;

    outCounter += smp_n;
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
			printf("unknown channel id %d\n", chn->index);
			return -1;
		}
		rawst_iw++;
		if (RAWST_BUF_NB_NOW == rawst_iw) rawst_iw = 0;
	}

	*p_dst = *p_smp & 0xff; /* for example, only LSB */
	p_dst += 2;	/* p0i0, p0q0, p0i1, p0q1, ... */
	//p_smp += iio_buffer_step(iiopr_cbuf[0]);
	//smp_n--;
#else
    //printf("%d\n", (uint32_t*)p_smp);
    //fwrite(p_dat, 1, smp_n);
#endif
	return smp_n;
}





// Method3: iio_channel_read_raw()
// Method4: iio_channel_read()
//  TODO: get known
static void chnread_samples_rxbuf(int igrp, ssize_t nbytes_from_k)
{
	int iprt, iblk;
	uint8_t *testbuf;

	/* p0i0, p0i1, ...	p0q0, p0q1, ... */
	testbuf = malloc(nbytes_from_k);
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (3 == do_smp_way) {
			iio_channel_read_raw(iiopr_chni[iprt], 
				iiopr_cbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read_raw(iiopr_chnq[iprt], 
				iiopr_cbuf[0], testbuf, RADIO_SYM2SMP*2);
		} else {
			iio_channel_read(iiopr_chni[iprt], 
				iiopr_cbuf[0], testbuf, RADIO_SYM2SMP*2);
			testbuf += RADIO_SYM2SMP*2;
			iio_channel_read(iiopr_chnq[iprt], 
				iiopr_cbuf[0], testbuf, RADIO_SYM2SMP*2);
		}
	}
	free(testbuf);
}




static FILE* phy_stream_analyze_rxfile(char *rxfilename)
{
    FILE* writefd = NULL;

    if(rxfilename == NULL || strlen(rxfilename) == 0)
        return writefd;

    DBG_Printf("rxfilename is %s\n", rxfilename);
	if (!(writefd = fopen(rxfilename, "wb+"))){  // ab+
        fprintf(stderr, "Cannot open output rxfile\n");
        return writefd;
    }
    return writefd;
}




static int ad9009_RxPath_toFile(int igrp, char *outfilename)
{

    char *p_dat, *p_end;
    ptrdiff_t p_inc;
    size_t nbytes_tx;
    int frame_size = 0;
    FILE* writefd = NULL;
    
    
    writefd = phy_stream_analyze_rxfile(outfilename);
    if (writefd == NULL) {
        printf("file: %s open faile", outfilename);
        return -1;
    }
    int channel_opt = (ports_mask == 0x2) ? 1 : 0;

    p_dat = iio_buffer_first(iiopr_cbuf[igrp],iiopr_chni[channel_opt]);
    p_inc = iio_buffer_step(iiopr_cbuf[igrp]);
	p_end = iio_buffer_end(iiopr_cbuf[igrp]);
	//p_dat = iio_buffer_first(iiopr_cbuf[igrp],iiopr_chni[igrp*DEV_PORT_NB]);
	

    /*calcate buffer len*/
    frame_size = iio_buffer_end(iiopr_cbuf[igrp]) - iio_buffer_start(iiopr_cbuf[igrp]);
    fwrite(p_dat, p_inc, frame_size, writefd);
    fflush(writefd);
    DBG_Printf("%d bytes be writed to %s\n", frame_size, outfilename);
    fclose(writefd);    
    return 0;
}





static int ad9009_RxPath_W4_toFile(int igrp, char *outfilename)
{
    int counter = 0;
    FILE* writetxtfd = NULL;
    char tempFilename[MAXLEN_FILENAME];
    char *p_dat, *p_end;
    ptrdiff_t p_inc;
    int nbytes_rx = 0;
    int fileBr_flag = 0;

    sprintf(tempFilename,"%s.txt", outfilename);
    if (!(writetxtfd = fopen(tempFilename, "at+"))){  // ab+
        fprintf(stderr, "Cannot open output rxfile\n");
        return -1;
    }


    int channel_opt = (ports_mask == 0x2) ? 1 : 0;
    while (!process_stop) {
        nbytes_rx = iio_buffer_refill(iiopr_cbuf[igrp]); 
        DBG_Printf("step: %d, refill size %d", iio_buffer_step(iiopr_cbuf[igrp]),nbytes_rx);
        if (nbytes_rx <= 0)
            continue;

        p_inc = iio_buffer_step(iiopr_cbuf[igrp]); 
        p_end = iio_buffer_end(iiopr_cbuf[igrp]); 
        if (p_inc == 4) {
            for (p_dat = iio_buffer_first(iiopr_cbuf[igrp],iiopr_chni[channel_opt]); p_dat < p_end; p_dat += p_inc) {
                fprintf(writetxtfd,"%02x%02x%02x%02x", p_dat[3],p_dat[2],p_dat[1],p_dat[0]);
                counter += p_inc;
                if ((fileBr_flag++ % 2 == 1) && (outputFormat == 64)) {
                    printf("\n");  // 8byte br
                }else if ((fileBr_flag++ % 4 == 3) && (outputFormat == 128)) {
                    printf("\n"); // 16byte br
                }
            }
            
        }else if (p_inc == 8) {
            for (p_dat = iio_buffer_first(iiopr_cbuf[igrp],iiopr_chni[channel_opt]); p_dat < p_end; p_dat += p_inc) {
                fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x", \
                    p_dat[7],p_dat[6],p_dat[5],p_dat[4],p_dat[3],p_dat[2],p_dat[1],p_dat[0]);
                counter += p_inc;
                if (outputFormat == 64) {
                    printf("\n");  // 8byte br
                }else if ((fileBr_flag++ % 2 == 1) && (outputFormat == 128)) {
                    printf("\n"); // 16byte br
                }
            }
        }
    }
    
    fclose(writetxtfd);
    return counter;
}





static int ad9009_RxPath_W6_toFile(int igrp, char *outfilename)
{

    char *p_dat, *p_end;
    ptrdiff_t p_inc;
    int frame_size = 0;
    int counter = 0;
    int pkg_cnt = 0;
    FILE* writetxtfd = NULL;
    char tempFilename[MAXLEN_FILENAME];
    char *tempMem = NULL;
    char *readTemp = NULL;
    int mallocBuffsize = 0;

    
    if (do_smp_way == 6) {
        mallocBuffsize = 1024*1024*200;           
    }else {
        mallocBuffsize = 1024*1024*50;
    }
    tempMem = malloc(mallocBuffsize);
    if (tempMem == NULL)
        fprintf(stderr, "Can't malloc %d bytes\n", mallocBuffsize);
    readTemp = tempMem;
    memset(tempMem, 0, mallocBuffsize);
    
    sprintf(tempFilename,"/root/iqdata_out/%s.txt", outfilename);
    if (!(writetxtfd = fopen(tempFilename, "at+"))){  // ab+
        fprintf(stderr, "Can't open output rxfile, %s\n", tempFilename);
        return -1;
    }


    int channel_opt = (ports_mask == 0x2) ? 1 : 0; 
    while (!process_stop){
        frame_size = iio_buffer_refill(iiopr_cbuf[igrp]);
        DBG_Printf("step: %d, refill size %d -->(%d)", iio_buffer_step(iiopr_cbuf[igrp]), \
            frame_size, (frame_size > 0) ? ++pkg_cnt: pkg_cnt);
        
        if (frame_size <= 0) 
            continue;

        p_dat = iio_buffer_first(iiopr_cbuf[igrp],iiopr_chni[channel_opt]);
        p_inc = iio_buffer_step(iiopr_cbuf[igrp]);
    	p_end = iio_buffer_end(iiopr_cbuf[igrp]);
        counter += frame_size;
        if (counter >= mallocBuffsize)
            break;
        
        memcpy(tempMem, p_dat, frame_size);
        tempMem += frame_size;
    }

     
    DBG_Printf("%d bytes will be writed to %s ......", counter, outfilename);
    if (outputFormat == 32) {
        for (int i = 0; i < counter; i=i+4) {
           fprintf(writetxtfd,"%02x%02x%02x%02x\n",readTemp[i+3],readTemp[i+2],readTemp[i+1],readTemp[i]);
        }
    }else if (outputFormat == 64) {
        for (int i = 0; i < counter; i=i+8) {
           fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x\n",readTemp[i+7],readTemp[i+6], \
                readTemp[i+5],readTemp[i+4],readTemp[i+3],readTemp[i+2],readTemp[i+1],readTemp[i]);
        }
    }else if (outputFormat == 128) {
        for (int i = 0; i < counter; i=i+16) {
            fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x",readTemp[i+15],readTemp[i+14], \
                readTemp[i+13],readTemp[i+12],readTemp[i+11],readTemp[i+10],readTemp[i+9],readTemp[i+8]);
            fprintf(writetxtfd,"%02x%02x%02x%02x%02x%02x%02x%02x\n",readTemp[i+7],readTemp[i+6], \
                readTemp[i+5],readTemp[i+4],readTemp[i+3],readTemp[i+2],readTemp[i+1],readTemp[i]);
        }
    } 
    fflush(writetxtfd); 
    fclose(writetxtfd);
    if (readTemp) free(readTemp);
    
    return counter;
}



int handle_ad9009_rxpath(int igrp, char *outfilename)
{
	ssize_t nbytes_from_k;
	uint32_t val;
    static int flag = 0;

	switch (do_smp_way) {
     
    case 6:
        ad9009_RxPath_W6_toFile(igrp, outfilename);
        break;
        
    case 5:
        /*receive data to files*/
        ad9009_RxPath_W6_toFile(igrp, outfilename);
        break;
	case 4:
        ad9009_RxPath_W4_toFile(igrp, outfilename);
        break;
	case 3:
		chnread_samples_rxbuf(igrp, nbytes_from_k);
		break;
	case 2:
		iio_buffer_foreach_sample(iiopr_cbuf[igrp], rxbuf_callback, NULL);
		break;
	case 1:
		memcopy_samples_rxbuf(igrp, nbytes_from_k);
		break;
	default:
	case 0:
		foreach_samples_rxbuf(igrp, nbytes_from_k);
		break;
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	rxsnb[igrp] += nbytes_from_k/iio_buffer_step(iiopr_cbuf[igrp])*2;
#else
    rxsnb[igrp] += nbytes_from_k/iio_buffer_step(iiopr_cbuf[igrp]);
#endif
	return 0;
}





/* Example: fill with 0xff
  iio_buffer_foreach_sample();
  p_inc == iio_device_get_sample_size(txdev[igrp]) == DEV_PORT_NB*4
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
static int foreach_samples_txbuf(int igrp, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_stt;
	int16_t *p_d16;
	int iprt, ismp;
    int size = 0;

	p_inc = iio_buffer_step(iiopt_cbuf[igrp]);
	p_end = iio_buffer_end(iiopt_cbuf[igrp]);
	p_d16 = (int16_t *)iio_buffer_first(iiopt_cbuf[igrp],iiopt_chni[igrp*DEV_PORT_NB]);
	nbytes_to_k = p_end - (void *)p_d16;
    
#if 1
	do {
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			if (ports_mask & (1 << iprt)) {
				*p_d16++ = bytetofill;
				*p_d16++ = bytetofill;
			}
		}
	} while ((void *)p_d16 < p_end);

    size = iio_buffer_push(iiopt_cbuf[igrp]);
    DBG_Printf("push %d bytes of data\n", (int)size);
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
				for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
					*p_d16++ = pl->iqs[iprt][ismp].i8;	// LSB of I[ismp][iprt]
					p_d16++;							// MSB of I[ismp][iprt]
					*p_d16++ = pl->iqs[iprt][ismp].q8;	// LSB of I[ismp][iprt]
					p_d16++;							// MSB of I[ismp][iprt]
				}
			}
			rawsr_ir++;
		} while (rawsr_ir == tmp)
	} while (rawsr_iw > rawsr_ir + RAWSR_BUF_NB_NOW/2);
#endif
    return size;
}





static int memcopy_samples_txbuf(int igrp, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_d16;
	int iprt, ismp, nb, tmp;
    int size = 0;

	p_inc = iio_buffer_step(iiopt_cbuf[igrp]);
	p_end = iio_buffer_end(iiopt_cbuf[igrp]);
	p_d16 = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB]);
	nbytes_to_k = p_end - p_d16;

    size = iio_buffer_push(iiopt_cbuf[igrp]);
    DBG_Printf("push %d bytes of data\n", (int)size);
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
		txsnb[igrp] += ismp;
		break;
	} while (1);
#endif
    return size;
}




static char* phy_stream_analyze_txfile(char *txfilename, int *frame_size)
{
    FILE* readfd = NULL;
    char *buf = NULL;
    int16_t i16, q16;
    

    if(txfilename == NULL || strlen(txfilename) == 0)
        return NULL;
    DBG_Printf("txfilename is %s\n", txfilename);


    if (!(readfd = fopen(txfilename, "rb"))) {
        fprintf(stderr, "Cannot open input txfile\n");
        return NULL;
    }
    fseek(readfd, 0, SEEK_END);
    *frame_size = ftell(readfd) / 4; //FIXME: preset sample size = 4; a nasty trick is needed to get real-time sample size
    DBG_Printf("Tx frame size is set to %d samples\n", *frame_size);
    fseek(readfd, 0, SEEK_SET);


    buf = malloc((*frame_size*4));
    *frame_size = fread(buf, 4, *frame_size, readfd);
    DBG_Printf("%s len is %d bytes.", txfilename, *frame_size);    
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




static int ad9009_TxPath_fromFile(int igrp)
{
    FILE* readfd = NULL;
    char *p_dat, *p_end;
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
        fseek(readfd, 0, SEEK_END);frame_size0 = ftell(readfd);
        DBG_Printf("Tx frame size is set to %d samples\n", frame_size0/4);
        fseek(readfd, 0, SEEK_SET);
        filebuf0 = malloc((frame_size0));
        fread(filebuf0, 4, frame_size0/4, readfd);
        DBG_Printf("%s len is %d bytes.", in0Filename, frame_size0); 
        fclose(readfd);
        for (int i = 0; i < frame_size0; i=i+4) {
            //exTemp = filebuf0[i];filebuf0[i] = filebuf0[i+3];filebuf0[i+3] = exTemp;
            exTemp = filebuf0[i+1];filebuf0[i+1] = filebuf0[i+2];filebuf0[i+2] = exTemp;
        }
    }


    if (readfd = fopen(in1Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);frame_size1 = ftell(readfd);
        DBG_Printf("Tx frame size is set to %d samples\n", frame_size1/4);
        fseek(readfd, 0, SEEK_SET);
        filebuf1 = malloc((frame_size1));
        fread(filebuf1, 4, frame_size1/4, readfd);
        DBG_Printf("%s len is %d bytes.", in1Filename, frame_size1); 
        fclose(readfd);
        for (int i = 0; i < frame_size1; i=i+4) {
            //exTemp = filebuf1[i];filebuf1[i] = filebuf1[i+3];filebuf1[i+3] = exTemp;
            exTemp = filebuf1[i+1];filebuf1[i+1] = filebuf1[i+2];filebuf1[i+2] = exTemp;
        }
    }
    /* only tx1 or have tx1&tx2, channel_opt = 0, only tx2, channel_opt 1*/    
    int channel_opt = (ports_mask == 0x2) ? 1 : 0;
    int counter = 0;
    
    if (ports_mask == 0x3)
        buffertmp = malloc((iiopt_smp_nb*4*2));

    
    while (4*iiopt_smp_nb*counter < frame_size0) {
        
        p_dat = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[channel_opt]);
        p_end = iio_buffer_end(iiopt_cbuf[igrp]);
        p_inc = iio_buffer_step(iiopt_cbuf[igrp]);
        DBG_Printf("step: %d, sample_size = %d,channel = %d", p_inc, iio_device_get_sample_size(txdev[igrp]), \
                iio_device_get_channels_count(txdev[igrp]));

        
        if (ports_mask == 0x1) {
            if (filebuf0 ==  NULL){
                fprintf(stderr, "tx1 buffer is NULL\n");
                goto lable1;
            }
            memcpy(p_dat, filebuf0 + 4*iiopt_smp_nb*counter, 4*iiopt_smp_nb);
            DBG_Printf("memcopy %d bytes to  port%d iio-buffer", 4*iiopt_smp_nb, ports_mask);
            
        }else if (ports_mask == 0x2) {
            if (filebuf1 == NULL){
                fprintf(stderr, "tx2 buffer is NULL\n");
                goto lable2;
            }
            memcpy(p_dat, filebuf1 + 4*iiopt_smp_nb*counter, 4*iiopt_smp_nb);
            DBG_Printf("memcopy %d bytes to  port%d iio-buffer", 4*iiopt_smp_nb, ports_mask);
            
        }else if (ports_mask == 0x3) {
            if ((frame_size0 != frame_size1 || frame_size0 == 0 || frame_size1 == 0) && \
                (filebuf1 == NULL || filebuf0 == NULL)) {
                DBG_Printf("%s or %s is NULL, or two files is unequal", in0Filename, in1Filename);
                goto lable3;
            }
            for(int i=0; i<iiopt_smp_nb; i++){
                memcpy(buffertmp+i*8,   filebuf0 + 4*iiopt_smp_nb*counter +i*4, 4);
                memcpy(buffertmp+i*8+4, filebuf1 + 4*iiopt_smp_nb*counter +i*4, 4);
            }
            memcpy(p_dat, buffertmp, iiopt_smp_nb*4*2);
            DBG_Printf("memcopy %d bytes to  port%x iio-buffer", 4*iiopt_smp_nb*2, ports_mask);
        }


        /*commit buffer to system  ..................*/
        nbytes_tx = iio_buffer_push(iiopt_cbuf[igrp]);
    	if (nbytes_tx < 0) {
    		fprintf(stderr, "Error pushing buf %d\n", (int) nbytes_tx);
    		return -1;  //CleanUP();
    	} else {
    		DBG_Printf("push %d bytes of data\n", (int) nbytes_tx);
    	}
        while(!process_stop);
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





static int ad9009_TxPath_W5_fromFile(int igrp)
{
    FILE* readfd = NULL;
    char *p_dat, *p_end;
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
    int channel_opt = (ports_mask == 0x2) ? 1 : 0;
    int counter = 0;
    
    if (readfd = fopen(in0Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);frame_size0 = ftell(readfd);
        DBG_Printf("Tx frame size is set to %d samples\n", frame_size0/4);
        fseek(readfd, 0, SEEK_SET);
        
        filebuf0 = malloc((frame_size0));
        fread(filebuf0, 4, frame_size0/4, readfd);
        //fread(p_dat, 4, frame_size0/4, readfd);
        DBG_Printf("%s len is %d bytes.", in0Filename, frame_size0); 
        fclose(readfd);
    }


    if (readfd = fopen(in1Filename, "rb")) {
        fseek(readfd, 0, SEEK_END);frame_size1 = ftell(readfd);
        DBG_Printf("Tx frame size is set to %d samples\n", frame_size1/4);
        fseek(readfd, 0, SEEK_SET);
        filebuf1 = malloc((frame_size1));
        fread(filebuf1, 4, frame_size1/4, readfd);
        //fread(p_dat, 4, frame_size1/4, readfd);
        DBG_Printf("%s len is %d bytes.", in1Filename, frame_size1); 
        fclose(readfd);
    }

    if (ports_mask == 0x3) {
        buffertmp = malloc((iiopt_smp_nb*4*2));
    }
    
#if 1        
    while (4*iiopt_smp_nb*counter < frame_size0) {

        clock_gettime(CLOCK_MONOTONIC, &old);
        p_dat = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[channel_opt]);
        
        if (ports_mask == 0x1) {
            if (filebuf0 ==  NULL){
                fprintf(stderr, "tx1 buffer is NULL\n");
                goto lable1;
            }
            memcpy(p_dat, filebuf0 + 4*iiopt_smp_nb*counter, 4*iiopt_smp_nb);
            
        }else if (ports_mask == 0x2) {
            if (filebuf1 == NULL){
                fprintf(stderr, "tx2 buffer is NULL\n");
                goto lable2;
            }
            memcpy(p_dat, filebuf1 + 4*iiopt_smp_nb*counter, 4*iiopt_smp_nb);
            
        }else if (ports_mask == 0x3) {
            if ((frame_size0 != frame_size1 || frame_size0 == 0 || frame_size1 == 0) && \
                (filebuf1 == NULL || filebuf0 == NULL)) {
                DBG_Printf("%s or %s is NULL, or two files is unequal", in0Filename, in1Filename);
                goto lable3;
            }
            for(int i=0; i<iiopt_smp_nb; i++){
                memcpy(buffertmp+i*8,   filebuf0 + 4*iiopt_smp_nb*counter +i*4, 4);
                memcpy(buffertmp+i*8+4, filebuf1 + 4*iiopt_smp_nb*counter +i*4, 4);
            }
            memcpy(p_dat, buffertmp, iiopt_smp_nb*4*2);
        }
       
        /*commit buffer to system  ..................*/
        nbytes_tx = iio_buffer_push(iiopt_cbuf[igrp]);
        while(!process_stop);
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







int handle_ad9009_txpath(int igrp)
{
	ssize_t nbytes_to_k;
	int iprt;
	uint32_t val;


	switch (do_smp_way) {
    case 6:
        /*send data from files*/
		nbytes_to_k = ad9009_TxPath_fromFile(igrp);
        break;
    case 5:
        /*send data from files*/
		nbytes_to_k = ad9009_TxPath_W5_fromFile(igrp);
        break;
	case 4:
         nbytes_to_k = ad9009_TxPath_fromFile(igrp);
        break;
	case 3:
		printf("\ncase3/4 TODO\n");
		break;
        
	case 2:
        ad9009_TxPath_fromFile(igrp);
		break;
        
	case 1:
		nbytes_to_k = memcopy_samples_txbuf(igrp, nbytes_to_k);
		break;
        
	default:
	case 0:
		nbytes_to_k = foreach_samples_txbuf(igrp, nbytes_to_k);
		break;
	}

#if defined(FPGA_COMPRESS_SAMPLES)
	txsnb[igrp] += nbytes_to_k/iio_buffer_step(iiopt_cbuf[igrp])*2;
#else
	txsnb[igrp] += nbytes_to_k/iio_buffer_step(iiopt_cbuf[igrp]);
#endif

	if (mode_verbo > 2) {
		show_libiio_txbuf(igrp);
	}

	return 0;
}





//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiorxdma_overflow(int igrp)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(rxdev[igrp], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "<-- read rxdma[%d] status failed %s\n",
			igrp, strerror(-ret));
		return -1;
	}
	if (mode_verbo > 1)
		printf("<-- rxdma[%d] status=0x%08x\n", igrp, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(rxdev[igrp], 0x80000088, val);
	if (val & 0x04) {
		if (mode_verbo > 1)
			fprintf(stderr, "<-- rxdma[%d] Overflow detected!\n", igrp);
		return 1;
	}

	return 0;
}





//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiotxdma_underflow(int igrp)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(txdev[igrp], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "--> read txdma[%d] status failed %s\n",
			igrp, strerror(-ret));
		return -1;
	}
	if (mode_verbo > 1)
		printf("--> txdma[%d] status=0x%08x\n", igrp, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(txdev[igrp], 0x80000088, val);
	if (val & 0x01) {
		if (mode_verbo > 1)
			fprintf(stderr, "--> txdma[%d] Underflow detected\n", igrp);
		return 1;
	}

	return 0;
}




void libiio_app_startup(int rxdev_smp_nb, int txdev_smp_nb)
{
	int igrp = 0, iprt;


    if (NULL == ctx) {
        if (mode_verbo > 0)
            printf("* Acquiring IIO context\n");
    	ASSERT((ctx = iio_create_local_context()) && "No context");
    	ASSERT((iio_context_get_devices_count(ctx) > 0) && "No devices");
    }
    
    ASSERT(cfg_stream_chn(ctx, &txcfg, TX, 0)&& "No phy_txport found");
 
	for (igrp=0; igrp<TOT_CHIP_NB; igrp++) {

        if (mode_verbo > 0)
            printf("LIBIIO_APP(group%d): Acquiring PHY device\n", igrp);
        struct iio_device *phy = get_phy_dev(ctx);


        if (mode_verbo > 0)
            printf("LIBIIO_APP(group%d): Profiling PHY device\n", igrp);

        if (iio_path_x & (EN_IIO_PATH_TX << igrp)) {
            //ASSERT((txdev_smp_nb > 0) && (txdev_smp_nb < (1024*1024))); //IIO_SMP_MAX
            init_ad9009_txpath(igrp, txdev_smp_nb);
        }
        
		if (iio_path_x & (EN_IIO_PATH_RX << igrp)) {
            //ASSERT((rxdev_smp_nb > 0) && (rxdev_smp_nb <= (1024*1024)));
            init_ad9009_rxpath(igrp, rxdev_smp_nb);
        }
        //iio_context_set_timeout(ctx, 500);

#if 0       //set trigger
        if (!phy || !iio_device_is_trigger(phy)) {
    		fprintf(stderr,"No trigger found (try setting up the iio-trig-hrtimer module)");
    		libiio_app_shutdown();;
	    }
        if (iio_device_set_trigger(phy, rxdev[igrp])) {
    		fprintf(stderr,"Could not set trigger\n");
    		libiio_app_shutdown();
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
        fprintf(stderr, "------>Cannot open input txfile %s\n", filename);
        return size;
    }
    fseek(readfd, 0, SEEK_END);
    size = ftell(readfd) / 4;
    fseek(readfd, 0, SEEK_SET);
    fclose(readfd);
    DBG_Printf("Tx frame size is set to %d samples\n", size);
    
    return size;
}



void printLibbiioVersion()
{
    unsigned int major, minor;
	char git_tag[8];
    
	iio_library_get_version(&major, &minor, git_tag);
	printf("===>Library version: %u.%u (git tag: %s)\n\n", major, minor, git_tag);
}





static int initConfig()
{
    int temp;
    

    if (iio_path_x & (EN_IIO_PATH_TX)) {

        
            if (in0Filename != NULL) {
                iiopt_smp_nb = getFileSize(in0Filename);
                DBG_Printf("really file size is %d", iiopt_smp_nb*4);
                if (iiopt_smp_nb <= 0 && (iiopt_smp_nb * 4 > 39 * 1024 * 1024)) { //39M
                    DBG_Printf("file: %d, can't send more than 39M", iiopt_smp_nb*4);
                    return -1;
                }
            }
            /*calculate tx buffer size .........*/
            temp = iiopt_smp_nb;
            while (1) {
                if (temp > 64*1024*1024) {  // less than 1M break
                    temp = temp / 2;
                    continue;
                }
                if (ports_mask == 0x3)
                    iiopt_smp_nb = (temp > 1024*1024/2) ? (temp/2) : temp;
                else iiopt_smp_nb = temp;
                DBG_Printf("will set %d samples(%d bytes) as tx buffer", iiopt_smp_nb, iiopt_smp_nb*4);
                break;
            }
    }

    if (iio_path_x & (EN_IIO_PATH_RX)) {

        /*calculate rx buffer size .........*/
        if (out0Filename != NULL) {
            bypassDatasize = getOutFileNameSize(out0Filename, iiopt_smp_nb);
            if (bypassDatasize == 0) {
                DBG_Printf("output filename error!");
                fprintf(stderr, "must select a name\n");
                return -1;
            }
        }
    }
    DBG_Printf("txbuff = %d, rxbuff = %d", iiopt_smp_nb, bypassDatasize);
    return 0;
}




/*receive thread, monitor rx dma data, and then write data to file*/
static void *monitor_thread_fn(void *data)
{

    DBG_Printf("RX thread_fn......................");
    /*handle rxpath ........*/
    if (iio_path_x & (EN_IIO_PATH_RX << 0)) {
        DBG_Printf("RX......");
        handle_ad9009_rxpath(0, out0Filename);
    }
    
    return (void *)0;
}





int main(int argc, char *argv[])
{

	int igrp, iprt;
	struct timespec tm_xs, tm_xe;
	double tm_us;
	struct timespec old, new;
    pthread_t monitor_thread;
    int ret;

    
    signal(SIGINT, handle_sig);
	get_args(argc, argv); 
    printLibbiioVersion();
    ret = initConfig();
    if (ret < 0){
        fprintf(stderr, "Some inappropriate parameters may have been passed in\n");
        return 0;
    }
    
    /*init devices and then create buffer size .........*/
    libiio_app_startup(bypassDatasize, iiopt_smp_nb);
    

    /*create a thread, use to monitor receive..........*/
    if (iio_path_x & (EN_IIO_PATH_RX)) {
        ret = pthread_create(&monitor_thread, NULL, monitor_thread_fn, NULL);
    	if (ret) {
    		fprintf(stderr, "Failed to create monitor thread: %s\n",strerror(-ret));
    	}
    }
        

/******************************************************************/
/******************************************************************/
    clock_gettime(CLOCK_MONOTONIC, &tm_xs);
    do {
        for (igrp=0; igrp<TOT_CHIP_NB; igrp++) {

            /*handle txpath ........*/
    		if (iio_path_x & (EN_IIO_PATH_TX << igrp)) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_ad9009_txpath(igrp);
                clock_gettime(CLOCK_MONOTONIC, &new);
                //printf("handle_ad9009_txpath() take %.0f us REAL-TIME\n", elapse_us(&new, &old));
		    }            
            //printf("RX[0] %4.6f MiSmp; TX[0] %4.6f MiSmp\n",rxsnb[0]/1e6, txsnb[0]/1e6);
            //fflush(stdout);
            
        }
    }while(0); //while(!process_stop);


    pthread_join(monitor_thread, NULL);
    
/******************************************************************/
	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
	tm_us = elapse_us(&tm_xe, &tm_xs);
	printf("\n");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (iio_path_x & (EN_IIO_PATH_RX << igrp))
			printf("RX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\n",
				iprt, rxsnb[0]/1e6, tm_us, rxsnb[0]/tm_us*32);        
	}
    if (do_smp_way == 2)
        DBG_Printf("w 2 mode: receive %d\n", outCounter);
	libiio_app_shutdown();
    fclose(writefd);  
    
	return 0;
}

