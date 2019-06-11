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

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

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
static unsigned int iiopr_smp_nb = IIOPR_SMP_NB_NOW;
static unsigned int iiopt_smp_nb = IIOPT_SMP_NB_NOW;
       unsigned int iio_path_x = EN_IIO_PATH_TX | EN_IIO_PATH_RX;
static unsigned int do_smp_way = 0;
static unsigned int mode_verbo = 0;
static unsigned int bytetofill = 0xffff;
static unsigned int ports_mask = 0x1;

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION]\n"
		" -h\tshow this help\n"
		" -n\tset samples number[4~1048576] for iio TX/RX each time\n"
		" -t\tenable TX datapath only\n"
		" -r\tenable RX datapath only\n"
		" -w\tselect one way to deal with samples\n"
		"   \t\t0) memcpy to user buffer in device level\n"
		"   \t\t1) for each sample frankly in device level\n"
		"   \t\t2) for each sample with callback() in channel level\n"
		"   \t\t3) call iio_channel_read_raw() for all samples of one channel\n"
		"   \t\t4) call iio_channel_read() for all samples of one channel\n"
		" -v\tbe verbose [0,1,2,3]\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
static void get_args(int argc, char **argv)
{
	int c;
	opterr = 0;
	while ((c = getopt(argc, argv, "p:m:n:v:w:b:xh")) != EOF) {
		switch (c) {
		case 'p': ports_mask = strtoul(optarg, NULL, 16);	break;
		case 'm': iiopr_smp_nb = strtoul(optarg, NULL,  0);	break;
		case 'n': iiopt_smp_nb = strtoul(optarg, NULL,  0);	break;
		case 'w': do_smp_way = strtoul(optarg, NULL,  0);	break;
		case 'v': mode_verbo = strtoul(optarg, NULL,  0);	break;
		case 'b': bytetofill = strtoul(optarg, NULL, 16);	break;
		case 'x': iio_path_x = strtoul(optarg, NULL, 16);	break;
		case 'h': usage(); exit( EXIT_FAILURE );			break;
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
	printf( "mode_verbo(-v):   %u\n\n", mode_verbo );
}

/* cleanup and exit */
void libiio_app_shutdown(void)
{
	int iprt, igrp;

	if (mode_verbo > 0)
		printf("* Destroying buffers\n");

	for (igrp = 0; igrp < TOT_CHIP_NB; igrp++) {
		if (iiopr_cbuf[igrp]) iio_buffer_destroy(iiopr_cbuf[igrp]);
		if (iiopt_cbuf[igrp]) iio_buffer_destroy(iiopt_cbuf[igrp]);

		if (mode_verbo > 0)
			printf("* Disabling streaming channels\n");
		for (iprt = 0; iprt < TOT_PORT_NB; iprt++) {
			if (iiopr_chni[iprt]) iio_channel_disable(iiopr_chni[iprt]);
			if (iiopr_chnq[iprt]) iio_channel_disable(iiopr_chnq[iprt]);
			if (iiopt_chni[iprt]) iio_channel_disable(iiopt_chni[iprt]);
			if (iiopt_chnq[iprt]) iio_channel_disable(iiopt_chnq[iprt]);
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
static struct iio_device* get_ad9371_phy(struct iio_context *ctx)
{
	struct iio_device *dev =  iio_context_find_device(ctx, "ad9371-phy");
	ASSERT(dev && "No ad9371-phy found");
	return dev;
}

/* finds AD9371 streaming IIO devices */
static bool get_ad9371_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
	switch (d) {
	case TX: *dev = iio_context_find_device(ctx, "axi-ad9371-tx-hpc"); return *dev != NULL;
	case RX: *dev = iio_context_find_device(ctx, "axi-ad9371-rx-hpc");  return *dev != NULL;
	default: ASSERT(0); return false;
	}
}

/* finds AD9371 streaming IIO channels */
static bool get_ad9371_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, char modify, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	return *chn != NULL;
}

/* finds AD9371 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
	case RX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

static bool set_phy_chan(struct iio_context *ctx, enum iodev d, struct iio_channel *chn)
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
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
	switch (d) {
	 // LO chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

static bool set_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel *chn)
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
static bool cfg_ad9371_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
	struct iio_channel *chn = NULL;

	// Configure phy and lo channels
	if (mode_verbo > 0)
		printf("* Acquiring AD9371 phy %s voltage channel %d\n", type?"TX":"RX", chid);
	if (!get_phy_chan(ctx, type, chid, &chn)) { return false; }
	//wr_ch_lli(chn, "rf_bandwidth",       cfg->bw_hz);
	//wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);
    set_phy_chan(ctx, type, chn);
	rd_ch_lli(chn, "rf_bandwidth");
	rd_ch_lli(chn, "sampling_frequency");

	// Configure LO channel
	if (mode_verbo > 0)
		printf("* Acquiring AD9371 phy %s lo channel\n", type?"TX":"RX");
	if (!get_lo_chan(ctx, type, &chn)) { return false; }
	//wr_ch_lli(chn, type == TX ? "TX_LO_frequency" : "RX_LO_frequency" , cfg->lo_hz);
	set_lo_chan(ctx, type, chn);

	return true;
}
/*
static bool dump_ad9371_phy(int igrp, enum iodev type, int iprt)
{
    int i;
    struct iio_channel *chn = NULL;

    // Configure voltage channels
	if (mode_verbo > 0)
	    printf("* Acquiring AD9371 phy %s voltage channel %d\n", type?"TX":"RX", iprt);
    if (!get_phy_chan(ctx, type, iprt, &chn)) { return false; }
    printf("Phy channel attributes list, %d item in total\n", chn->nb_attrs);
    for (i = 0; i < chn->nb_attrs; i++) {
        printf("%s %s\n", chn->attrs[i].name, chn->attrs[i].filename);
    }

    // Configure LO channel
	if (mode_verbo > 0)
	    printf("* Acquiring AD9371 phy %s LO channel\n", type?"TX":"RX");
    if (!get_lo_chan(ctx, type, &chn)) { return false; }
    printf("LO channel attributes list, %d item in total\n", chn->nb_attrs);
    for (i = 0; i < chn->nb_attrs; i++) {
        printf("%s %s\n", chn->attrs[i].name, chn->attrs[i].filename);
    }

    return true;
}*/

/* JESD_RX(iio:device4) */
/* JESD_RX[0/1]_I(iio:device4/in_voltage[0/1]_i_xxx) */
/* JESD_RX[0/1]_Q(iio:device4/in_voltage[0/1]_q_xxx) */
int init_ad9371_rxpath(int igrp, size_t smp_nb)
{
	int iprt;

	/* ad9371-phy(iio:device1) */
		/* PHY_RX[0/1/2/3](iio:device1/in_voltage[n]_xxx */
		/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
        if (mode_verbo > 0)
            printf("AD9371_RX(group%d): Initializing PHY port%d\n",
                igrp, iprt);
		ASSERT(cfg_ad9371_streaming_ch(ctx, &rxcfg, RX, iprt) \
			&& "No phy_rxport found");
	}

	if (mode_verbo > 0)
		printf("AD9371_RX(group%d): Acquiring RX-JESD device\n", igrp);
	ASSERT(get_ad9371_stream_dev(ctx, RX, &rxdev[igrp]) && "No rxdev found");
	//iio_device_reg_write(rxdev[igrp], 0x80000088, 0x6);// Clear all status bits

	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (ports_mask & (1 << iprt)) {
            if (mode_verbo > 0)
                printf("AD9371_RX(group%d): Initilizing channels of RX-JESD port%d\n",
                    igrp, iprt);
			ASSERT(get_ad9371_stream_ch(ctx, RX, rxdev[igrp], iprt, 'i',
				&iiopr_chni[igrp*DEV_PORT_NB+iprt]) && "No rxdev_p0i found");
			ASSERT(get_ad9371_stream_ch(ctx, RX, rxdev[igrp], iprt, 'q', 
				&iiopr_chnq[igrp*DEV_PORT_NB+iprt]) && "No rxdev_p0q found");
			iio_channel_enable(iiopr_chni[igrp*DEV_PORT_NB+iprt]);
			iio_channel_enable(iiopr_chnq[igrp*DEV_PORT_NB+iprt]);
		}
	}

    if (mode_verbo > 0)
    	printf("AD9371_RX(group%d): Creating non-cyclic buffers with %u samples\n",
    	    igrp, smp_nb);
	iiopr_cbuf[igrp] = iio_device_create_buffer(rxdev[igrp], smp_nb, false);
	if (NULL == iiopr_cbuf[igrp]) {
		perror("AD9371_RX: Creating buffers failed");
		return -1;
	}
	ASSERT(4*DEV_PORT_NB == iio_device_get_sample_size(rxdev[igrp]));

	return 0;
}

/* JESD_TX(iio:device3)*/
/* JESD_TX[0/1]_I(iio:device3/out_voltage[0/2]_xxx)*/
/* JESD_TX[0/1]_Q(iio:device3/out_voltage[1/3]_xxx)*/
int init_ad9371_txpath(int igrp, size_t smp_nb)
{
	int iprt;

	/* ad9371-phy(iio:device1) */
		/* PHY_RX[0/1/2/3](iio:device1/in_voltage[n]_xxx */
		/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
        if (mode_verbo > 0)
            printf("AD9371_TX(group%d): Initializing PHY port%d\n",
                igrp, iprt);
		ASSERT(cfg_ad9371_streaming_ch(ctx, &txcfg, TX, iprt) \
			&& "No phy_txport found");
	}

	if (mode_verbo > 0)
		printf("AD9371_TX(group%d): Acquiring TX-JESD device\n", igrp);
	ASSERT(get_ad9371_stream_dev(ctx, TX, &txdev[igrp]) && "No txdev found");
	//iio_device_reg_write(txdev[igrp], 0x80000088, 0x6);// Clear all status bits

	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (ports_mask & (1 << iprt)) {
            if (mode_verbo > 0)
                printf("AD9371_TX(group%d): Initilizing channels of TX-JESD port%d\n",
                    igrp, iprt);
			ASSERT(get_ad9371_stream_ch(ctx, TX, txdev[igrp], 2*iprt,   0, 
				&iiopt_chni[igrp*DEV_PORT_NB+iprt]) && "No txdev_p0i found");
			ASSERT(get_ad9371_stream_ch(ctx, TX, txdev[igrp], 2*iprt+1, 0, 
				&iiopt_chnq[igrp*DEV_PORT_NB+iprt]) && "No txdev_p0q found");
			iio_channel_enable(iiopt_chni[igrp*DEV_PORT_NB+iprt]);
			iio_channel_enable(iiopt_chnq[igrp*DEV_PORT_NB+iprt]);
		}
	}
    if (mode_verbo > 0)
    	printf("AD9371_TX(group%d): Creating non-cyclic buffers with %u samples\n",
    	    igrp, smp_nb);
	iiopt_cbuf[igrp] = iio_device_create_buffer(txdev[igrp], smp_nb, false);
	if (NULL == iiopt_cbuf[igrp]) {
		perror("AD9371_TX: Create buffers failed");
		return -1;
	}
	ASSERT(4*DEV_PORT_NB == iio_device_get_sample_size(txdev[igrp]));

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
	printf("\n<-- iio_rxbuf: RAW:");
	pbyte = iio_buffer_first(iiopr_cbuf[igrp], iiopr_chni[igrp*DEV_PORT_NB]);
	for (i=0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);

	printf("\n<-- iio_rxbuf: Samples:");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) printf(" P%dI, P%dQ", iprt, iprt);
	printf("\t");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) printf(" P%dI, P%dQ", iprt, iprt);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iiopr_cbuf[igrp]->data_length;
	psample = iio_buffer_first(iiopr_cbuf[igrp], iiopr_chni[igrp*DEV_PORT_NB]);
	for (i = 0; (void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\n");
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
#if 0
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		printf("\n<-- iio_rxbuf: P%dI: ", iprt);
		p_d16 = iio_buffer_first(iiopr_cbuf[igrp], iiopr_chni[igrp*DEV_PORT_NB+iprt]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
		printf("\n<-- iio_rxbuf: P%dQ: ", iprt);
		p_d16 = iio_buffer_first(iiopr_cbuf[igrp], iiopr_chnq[igrp*DEV_PORT_NB+iprt]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
	}
#endif
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
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iiopt_cbuf[igrp]->data_length;
	psample = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB]);
	for (i = 0;	(void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\n");
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
#if 0
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {	
		printf("\n--> iio_txbuf: P%dI: ", iprt);
		p_d16 = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB+iprt]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
		printf("\n--> iio_txbuf: P%dQ: ", iprt);
		p_d16 = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chnq[igrp*DEV_PORT_NB+iprt]);
		for (i = 0; (void *)p_d16 < p_end; p_d16 += p_inc/sizeof(*p_d16), i++) {
			printf("%04x ", p_d16[0]);
		}
	}
#endif
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
	void *p_end;
	int16_t *p_d16;
	int iprt, ismp;
	int16_t i16, q16;

	p_inc = iio_buffer_step(iiopr_cbuf[igrp]);
	p_end = iio_buffer_end(iiopr_cbuf[igrp]);
	p_d16 = (int16_t *)iio_buffer_first(iiopr_cbuf[igrp],
				iiopr_chni[igrp*DEV_PORT_NB]);
#if 1
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
#else //defined(BUILDING_RRU_UL)
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

#endif
	return iiopr_smp_nb;
}

// Method3: iio_channel_read_raw()
// Method4: iio_channel_read()
// TODO: get known
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

int handle_ad9371_rxpath(int igrp)
{
	ssize_t nbytes_from_k;
	uint32_t val;

	nbytes_from_k = iio_buffer_refill(iiopr_cbuf[igrp]);
	if (mode_verbo > 1) {
		iio_device_reg_read(rxdev[igrp], 0x80000088, &val);
		printf("\n<-- read rxdma[%d] status=0x%08x\n", val);
	}
	if (nbytes_from_k < 0) {
		fprintf(stderr, "<-- iio_buffer_refill(): error %s\n",
			strerror(-(int)nbytes_from_k));
		return -1;
	}

	if (mode_verbo > 0) {
		ASSERT(nbytes_from_k <= IIO_SMP_MAX*4*DEV_PORT_NB);
		printf("<-- iio_buffer_refill(): k_iio => u_libiio %u Bytes\n",
			(size_t)nbytes_from_k);
	}
	if (mode_verbo > 2) {
		show_libiio_rxbuf(igrp);
	}

	switch (do_smp_way) {
	case 4:
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
static void foreach_samples_txbuf(int igrp, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_stt;
	int16_t *p_d16;
	int iprt, ismp;

	p_inc = iio_buffer_step(iiopt_cbuf[igrp]);
	p_end = iio_buffer_end(iiopt_cbuf[igrp]);
	p_d16 = (int16_t *)iio_buffer_first(iiopt_cbuf[igrp],
				iiopt_chni[igrp*DEV_PORT_NB]);
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
}

static void memcopy_samples_txbuf(int igrp, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	void *p_end, *p_d16;
	int iprt, ismp, nb, tmp;

	p_inc = iio_buffer_step(iiopt_cbuf[igrp]);
	p_end = iio_buffer_end(iiopt_cbuf[igrp]);
	p_d16 = iio_buffer_first(iiopt_cbuf[igrp], iiopt_chni[igrp*DEV_PORT_NB]);
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
		txsnb[igrp] += ismp;
		break;
	} while (1);
#endif
}

int handle_ad9371_txpath(int igrp)
{
	ssize_t nbytes_to_k;
	int iprt;
	uint32_t val;

	nbytes_to_k = iio_buffer_push(iiopt_cbuf[igrp]);
	if (mode_verbo > 1) {
		iio_device_reg_read(txdev[igrp], 0x80000088, &val);
		printf("\n--> read txdma[%d] status=0x%08x\n",
			igrp, val);
	}
	if (nbytes_to_k < 0) {
		fprintf(stderr, "--> iio_buffer_push(): error %s\n",
			strerror(-(int)nbytes_to_k));
		return -1;
	}
	if (mode_verbo > 0) {
		ASSERT(nbytes_to_k <= IIO_SMP_MAX*4*DEV_PORT_NB);
		printf("--> iio_buffer_push(): u_libiio => k_iio %u Bytes\n",
			(size_t)nbytes_to_k);
	}

	switch (do_smp_way) {
	case 4:
	case 3:
		printf("\ncase3/4 TODO\n");
		break;
	case 2:
		printf("\ncase2 TODO\n");
		break;
	case 1:
		memcopy_samples_txbuf(igrp, nbytes_to_k);
		break;
	default:
	case 0:
		foreach_samples_txbuf(igrp, nbytes_to_k);
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

void profile_ad9371_phy_array(struct iio_device *phy);
void libiio_app_startup(int rxdev_smp_nb, int txdev_smp_nb)
{
	int igrp, iprt;

    if (NULL == ctx) {
        if (mode_verbo > 0)
            printf("* Acquiring IIO context\n");
    	ASSERT((ctx = iio_create_local_context()) && "No context");
    	ASSERT((iio_context_get_devices_count(ctx) > 0) && "No devices");
    }

	for (igrp=0; igrp<TOT_CHIP_NB; igrp++) {
#if 1
        if (mode_verbo > 0)
            printf("LIBIIO_APP(group%d): Acquiring PHY device\n", igrp);
        struct iio_device *phy = get_ad9371_phy(ctx);
        if (mode_verbo > 0)
            printf("LIBIIO_APP(group%d): Profiling PHY device\n", igrp);
        profile_ad9371_phy_array(phy);
#endif
		if (iio_path_x & (EN_IIO_PATH_RX << igrp)) {
            ASSERT((rxdev_smp_nb > 0) && (rxdev_smp_nb <= IIO_SMP_MAX));
            init_ad9371_rxpath(igrp, rxdev_smp_nb);
        }
		if (iio_path_x & (EN_IIO_PATH_TX << igrp)) {
            ASSERT((txdev_smp_nb > 0) && (txdev_smp_nb <= IIO_SMP_MAX));
            init_ad9371_txpath(igrp, txdev_smp_nb);
        }
	}
}

#if 0//!defined(BUILDING_RRU_UL) && !defined(BUILDING_RRU_DL)
int main(int argc, char *argv[])
{
	int igrp, iprt;
	struct timespec tm_xs, tm_xe;
	double tm_us;
	struct timespec old, new;

	get_args(argc, argv);

#if defined(FPGA_COMPRESS_SAMPLES)
	libiio_app_startup(iiopr_smp_nb/2, iiopt_smp_nb/2);
#else
    libiio_app_startup(iiopr_smp_nb, iiopt_smp_nb);
#endif

	signal(SIGINT, handle_sig);
	if (mode_verbo > 0)
		printf("* Starting IO streaming (press CTRL+C to cancel)\n");

	clock_gettime(CLOCK_MONOTONIC, &tm_xs);
	do {
        for (igrp=0; igrp<TOT_CHIP_NB; igrp++) {
    		if (iio_path_x & (EN_IIO_PATH_RX << igrp)) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_ad9371_rxpath(igrp);
                clock_gettime(CLOCK_MONOTONIC, &new);
                printf("handle_ad9371_rxpath() take %.0f us REAL-TIME\n", 
                    elapse_us(&new, &old));
    		}
    		if (iio_path_x & (EN_IIO_PATH_TX << igrp)) {
                clock_gettime(CLOCK_MONOTONIC, &old);
    			handle_ad9371_txpath(igrp);
                clock_gettime(CLOCK_MONOTONIC, &new);
                printf("handle_ad9371_txpath() take %.0f us REAL-TIME\n", 
                    elapse_us(&new, &old));
    		}
            //printf("\r");
            printf("RX[0] %4.6f MiSmp; TX[0] %4.6f MiSmp",
                rxsnb[0]/1e6, txsnb[0]/1e6);
            printf("\n");
            fflush(stdout);
        }
	} while (!process_stop);

	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
	tm_us = elapse_us(&tm_xe, &tm_xs);
	printf("\n");
	for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
		if (iio_path_x & (EN_IIO_PATH_RX << igrp))
			printf("RX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\n",
				iprt, rxsnb[0]/1e6, tm_us, rxsnb[0]/tm_us*32);
		if (iio_path_x & (EN_IIO_PATH_TX << igrp))
			printf("TX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\n",
				iprt, txsnb[0]/1e6, tm_us, txsnb[0]/tm_us*32);
	}

	libiio_app_shutdown();
	return 0;
}
#endif

#include "t_mykonos.h"
#include "t_mykonos_gpio.h"
#include "ad9371profile_20MHz.c"
//Loading the ad9371 profile generated by the TES
extern mykonosDevice_t mykDevice;
void profile_ad9371_phy_array(struct iio_device *phy)
{
    mykonosTxSettings_t *tx = mykDevice.tx;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-channels-enable",	tx->txChannels);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-pll-use-external-lo", tx->txPllUseExternalLo);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-pll-lo-frequency_hz", tx->txPllLoFrequency_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,adi,tx-settings-tx-atten-step-size", tx->txAttenStepSize);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx2-atten_mdb", tx->tx2Atten_mdB);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx1-atten_mdb", tx->tx1Atten_mdB);

    mykonosTxProfile_t *tp = mykDevice.tx->txProfile;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-bbf-3db-corner_khz", tp->txBbf3dBCorner_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-dac-3db-corner_khz", tp->txDac3dBCorner_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-rf-bandwidth_hz", tp->rfBandwidth_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-primary-sig-bandwidth_hz", tp->primarySigBandwidth_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-iq-rate_khz", tp->iqRate_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-input-hb-interpolation", tp->txInputHbInterpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb2-interpolation", tp->thb1Interpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb1-interpolation", tp->thb2Interpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-fir-interpolation", tp->txFirInterpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-dac-div", tp->dacDiv);

	mykonosRxSettings_t *rx = mykDevice.rx;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-lo-frequency_hz", rx->rxPllLoFrequency_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-use-external-lo", rx->rxPllUseExternalLo);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-channels-enable", rx->rxChannels);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-real-if-data", rx->realIfData);

    mykonosRxProfile_t *rp = mykDevice.rx->rxProfile;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-bbf-3db-corner_khz", rp->rxBbf3dBCorner_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rf-bandwidth_hz", rp->rfBandwidth_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-iq-rate_khz", rp->iqRate_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rhb1-decimation", rp->rhb1Decimation);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-en-high-rej-dec5", rp->enHighRejDec5);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-dec5-decimation", rp->rxDec5Decimation);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-fir-decimation", rp->rxFirDecimation);
} 

void profile_ad9371_phy_file(int igrp, char *fn)
{
	int ret;
	size_t len;
	FILE *fd = NULL;
	char *buf = NULL;
    struct iio_device *phy = get_ad9371_phy(ctx);

	printf("Load profile from %s\n", fn);
	fd = fopen(fn, "r");
	if (fd < 0) {
		perror("fopen");
		exit(-1);
	}

	fseek(fd, 0, SEEK_END);
	len = ftell(fd);
	fseek(fd, 0, SEEK_SET);
	buf = malloc(len);
	if (NULL == buf) {
		perror("malloc");
		exit(-1);
	}
	ret = fread(buf, len, 1, fd);
	if (ret != len) {
		perror("fread");
		exit(-1);
	}
	ret = fclose(fd);
	if (0 != ret) {
		perror("fclose");
		exit(-1);
	}

	iio_context_set_timeout(ctx, 30000);
	printf("Profile write start\n");
	iio_device_attr_write_raw(phy, "profile_config", buf, len);
	printf("Profile write   end\n");
	free(buf);
}
