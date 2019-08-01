/*
 * libiio - AD9371 IIO streaming example
 *
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 * Copyright (C) 2017 Analog Devices Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 **/
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
static struct iio_device *txdev[CHIP_NUM_TOTAL];
static struct iio_device *rxdev[CHIP_NUM_TOTAL];
// one libiio buffer for each device
struct iio_buffer *iiorxcbuf[CHIP_NUM_TOTAL];
struct iio_buffer *iiotxcbuf[CHIP_NUM_TOTAL];
// 2x2 Streaming channels (2 Streaming ports) for each device
struct iio_channel *rxpi[PORT_NUM_TOTAL];
struct iio_channel *rxpq[PORT_NUM_TOTAL];
struct iio_channel *txpi[PORT_NUM_TOTAL];
struct iio_channel *txpq[PORT_NUM_TOTAL];
// sample counters per port
static size_t rxsn[PORT_NUM_TOTAL];
static size_t txsn[PORT_NUM_TOTAL];

static bool b_process_stop;
static unsigned int iio_smp_nb = IIO_PORT_SAMPLES;
static unsigned int iio_rxp_en = 1;
static unsigned int iio_txp_en = 1;
static unsigned int do_smp_way = 0;
static unsigned int mode_verbo = 0;

uint8_t socktxbuf[SUBCARRIER_NTXBUF_NB][SUBCARRIER_NTXBUF_SZ];
uint8_t sockrxbuf[SUBCARRIER_NRXBUF_NB][SUBCARRIER_NRXBUF_SZ];
volatile int iw_socktxbuf = 0;
volatile int ir_socktxbuf = 0;
volatile int ir_sockrxbuf = 0;
volatile int iw_sockrxbuf = SUBCARRIER_NRXBUF_NB/2;

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./iiojacky [OPTION]\n"
		" -h\tshow this help\n"
		" -n\tset samples number[4~1048576] for iio TX/RX each time\n"
		" -t\tenable TX datapath only\n"
		" -r\tenable RX datapath only\n"
		" -m\tselect one way to deal with samples\n"
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
	while ( (c = getopt( argc, argv, "n:v:rtch")) != EOF) {
		switch ( c ) {
		case 'n': iio_smp_nb = strtoul( optarg, NULL, 0 );	break;
		case 't': iio_txp_en = 1; iio_rxp_en = 0; 			break;
		case 'r': iio_rxp_en = 1; iio_txp_en = 0;			break;
		case 'c': do_smp_way = strtoul( optarg, NULL, 0 );	break;
		case 'v': mode_verbo = strtoul( optarg, NULL, 0 );	break;
		case 'h': usage(); exit( EXIT_FAILURE );			break;
		case '?':
			if ( isprint (optopt) ) {
				fprintf ( stderr,
				          "ERROR: unrecognised option \"%c\"\n",
				          (char) optopt );
				exit( EXIT_FAILURE );
			}
			break;
		default:
			fprintf( stderr, "ERROR: unrecognised command line option\n");
			exit( EXIT_FAILURE );
			break;
		}
	}
	printf( "CURRENT SETTINGS:\n" );
	printf( "iio_smp_nb(-n):   %u\n", iio_smp_nb );
	printf( "iio_rxp_en(-r):   %u\n", iio_rxp_en );
	printf( "iio_txp_en(-t):   %u\n", iio_txp_en );
	printf( "do_smp_way(-c):   %u\n", do_smp_way );
	printf( "mode_verbo(-v):   %u\n\n", mode_verbo );
}

/* cleanup and exit */
void libiio_app_shutdown(void)
{
	int iport, idev;

	if (mode_verbo > 0)
		printf("* Destroying buffers\n");

	for (idev = 0; idev < CHIP_NUM_TOTAL; idev++) {
		if (iiorxcbuf[idev]) iio_buffer_destroy(iiorxcbuf[idev]);
		if (iiotxcbuf[idev]) iio_buffer_destroy(iiotxcbuf[idev]);

		if (mode_verbo > 0)
			printf("* Disabling streaming channels\n");
		for (iport = 0; iport < PORT_NUM_TOTAL; iport++) {
			if (rxpi[iport]) iio_channel_disable(rxpi[iport]);
			if (rxpq[iport]) iio_channel_disable(rxpq[iport]);
			if (txpi[iport]) iio_channel_disable(txpi[iport]);
			if (txpq[iport]) iio_channel_disable(txpq[iport]);
		}
	}

	if (mode_verbo > 0)
		printf("* Destroying context\n");
	if (ctx) iio_context_destroy(ctx);
	exit(0);
}

static void handle_sig(int sig)
{
	printf("\nWaiting for process to finish...\n");
	b_process_stop = true;
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
static bool dump_ad9371_phy(int idev, enum iodev type, int iport)
{
    int i;
    struct iio_channel *chn = NULL;

    // Configure voltage channels
	if (mode_verbo > 0)
	    printf("* Acquiring AD9371 phy %s voltage channel %d\n", type?"TX":"RX", iport);
    if (!get_phy_chan(ctx, type, iport, &chn)) { return false; }
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
int init_ad9371_rxpath(int idev, size_t iios)
{
	int iport;
	int ret;

	if (mode_verbo > 0)
		printf("* Acquiring AD9371-JESD rx-streaming device\n");
	ASSERT(get_ad9371_stream_dev(ctx, RX, &rxdev[idev]) && "No rxdev found");
	//iio_device_reg_write(rxdev[idev], 0x80000088, 0x6);// Clear all status bits

	printf("* Initilizing AD9371-JESD rx-streaming channels\n");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		ASSERT(get_ad9371_stream_ch(ctx, RX, rxdev[idev], iport, 'i',
			&rxpi[idev*PORT_PER_DEVICE+iport]) && "No rxdev_p0i found");
		ASSERT(get_ad9371_stream_ch(ctx, RX, rxdev[idev], iport, 'q', 
			&rxpq[idev*PORT_PER_DEVICE+iport]) && "No rxdev_p0q found");
		iio_channel_enable(rxpi[idev*PORT_PER_DEVICE+iport]);
		iio_channel_enable(rxpq[idev*PORT_PER_DEVICE+iport]);
	}

	printf("* Creating non-cyclic rx-buffers with %u Samples\n", iios);
	iiorxcbuf[idev] = iio_device_create_buffer(rxdev[idev], iios, false);
	if (NULL == iiorxcbuf[idev]) {
		perror("Could not create RX buffer");
		return -1;
	}
	ASSERT(4*PORT_PER_DEVICE == iio_device_get_sample_size(rxdev[idev]));

	return 0;
}

/* JESD_TX(iio:device3)*/
/* JESD_TX[0/1]_I(iio:device3/out_voltage[0/2]_xxx)*/
/* JESD_TX[0/1]_Q(iio:device3/out_voltage[1/3]_xxx)*/
int init_ad9371_txpath(int idev, size_t iios)
{
	int ret, iport;

	if (mode_verbo > 0)
		printf("* Acquiring AD9371-JESD tx-streaming devices\n");
	ASSERT(get_ad9371_stream_dev(ctx, TX, &txdev[idev]) && "No txdev found");
	//iio_device_reg_write(txdev[idev], 0x80000088, 0x6);// Clear all status bits

	printf("* Initilizing AD9371-JESD tx-streaming channels\n");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		ASSERT(get_ad9371_stream_ch(ctx, TX, txdev[idev], 2*iport,   0, 
			&txpi[idev*PORT_PER_DEVICE+iport]) && "No txdev_p0i found");
		ASSERT(get_ad9371_stream_ch(ctx, TX, txdev[idev], 2*iport+1, 0, 
			&txpq[idev*PORT_PER_DEVICE+iport]) && "No txdev_p0q found");
		iio_channel_enable(txpi[idev*PORT_PER_DEVICE+iport]);
		iio_channel_enable(txpq[idev*PORT_PER_DEVICE+iport]);
	}

	printf("* Creating non-cyclic tx-buffers with %u Samples\n", iios);
	iiotxcbuf[idev] = iio_device_create_buffer(txdev[idev], iios, false);
	if (NULL == iiotxcbuf[idev]) {
		perror("Could not create TX buffer");
		return -1;
	}
	ASSERT(4*PORT_PER_DEVICE == iio_device_get_sample_size(txdev[idev]));

	return 0;
}

void show_libiio_rxbuf(int idev)
{
	void *p_end;
	ptrdiff_t p_inc;	
	uint8_t *pbyte;
	size_t i;
	int iport;
	uint16_t *p_dat;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iiorxcbuf[idev]);
	p_end = iio_buffer_end(iiorxcbuf[idev]);
	printf("iio_rxbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			iport, iio_buffer_first(iiorxcbuf[idev], rxpi[idev*PORT_PER_DEVICE+iport]),
			iport, iio_buffer_first(iiorxcbuf[idev], rxpq[idev*PORT_PER_DEVICE+iport]));
	}
	printf("\n==== iio_rxbuf: RAW:");
	pbyte = iio_buffer_first(iiorxcbuf[idev], rxpi[idev*PORT_PER_DEVICE]);
	for (i=0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);

	printf("\n==== iio_rxbuf: Samples:");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) printf(" P%dI, P%dQ", iport, iport);
	printf("\t");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) printf(" P%dI, P%dQ", iport, iport);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iiorxcbuf[idev]->data_length;
	psample = iio_buffer_first(iiorxcbuf[idev], rxpi[idev*PORT_PER_DEVICE]);
	for (i = 0; (void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\n");
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
#if 0
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		printf("\n==== iio_rxbuf: P%dI: ", iport);
		p_dat = iio_buffer_first(iiorxcbuf[idev], rxpi[idev*PORT_PER_DEVICE+iport]);
		for (i = 0; (void *)p_dat < p_end; p_dat += p_inc/sizeof(*p_dat), i++) {
			printf("%04x ", p_dat[0]);
		}
		printf("\n==== iio_rxbuf: P%dQ: ", iport);
		p_dat = iio_buffer_first(iiorxcbuf[idev], rxpq[idev*PORT_PER_DEVICE+iport]);
		for (i = 0; (void *)p_dat < p_end; p_dat += p_inc/sizeof(*p_dat), i++) {
			printf("%04x ", p_dat[0]);
		}
	}
#endif
	printf("\n");
	fflush(stdout);
}

void show_libiio_txbuf(int idev)
{
	void *p_end;
	ptrdiff_t p_inc;
	uint8_t *pbyte;
	size_t i;
	int iport;
	uint16_t *p_dat;
	struct iio_sample *psample;

	p_inc = iio_buffer_step(iiotxcbuf[idev]);
	p_end = iio_buffer_end(iiotxcbuf[idev]);
	printf("iio_txbuf: step=%d, end=0x%08X", (int)p_inc, p_end);
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		printf(" P%dI_start=0x%08X, P%dQ_start=0x%08X", 
			iport, iio_buffer_first(iiotxcbuf[idev], txpi[idev*PORT_PER_DEVICE+iport]),
			iport, iio_buffer_first(iiotxcbuf[idev], txpq[idev*PORT_PER_DEVICE+iport]));
	}

	printf("\n==== iio_txbuf: RAW:");
	pbyte = iio_buffer_first(iiotxcbuf[idev], txpi[idev*PORT_PER_DEVICE]);
	for (i = 0; (void *)pbyte < p_end; pbyte++, i++) {
		if (0 == i % 16) printf("\n");
		printf("%02x ", *pbyte);
	}
	fflush(stdout);

	printf("\n==== iio_txbuf: Samples:");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) printf(" P%dI, P%dQ", iport, iport);
	printf("\t");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) printf(" P%dI, P%dQ", iport, iport);
	//psample = (struct iio_sample *)(pbuf->buffer);
	//iiotxcbuf[idev]->data_length;
	psample = iio_buffer_first(iiotxcbuf[idev], txpi[idev*PORT_PER_DEVICE]);
	for (i = 0;	(void *)psample < p_end; i++) {
		if (i & 1) printf("\t"); else printf("\n");
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			printf("%04x %04x ", (uint16_t)psample->i16, (uint16_t)psample->q16);
			psample++;
		}
	}
#if 0
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {	
		printf("\n==== iio_txbuf: P%dI: ", iport);
		p_dat = iio_buffer_first(iiotxcbuf[idev], txpi[idev*PORT_PER_DEVICE+iport]);
		for (i = 0; (void *)p_dat < p_end; p_dat += p_inc/sizeof(*p_dat), i++) {
			printf("%04x ", p_dat[0]);
		}
		printf("\n==== iio_txbuf: P%dQ: ", iport);
		p_dat = iio_buffer_first(iiotxcbuf[idev], txpq[idev*PORT_PER_DEVICE+iport]);
		for (i = 0; (void *)p_dat < p_end; p_dat += p_inc/sizeof(*p_dat), i++) {
			printf("%04x ", p_dat[0]);
		}
	}
#endif
	printf("\n");
	fflush(stdout);
}

// Method0: sample by sample
// Example: copy Real(I) and Imag(Q) to an array of userspace buffers
// The HARDWARE format of each sample :
// cat /sys/bus/iio/devices/iio:device1/scan_elements/in_voltage0_type
// le:s12/16>>4
void memcopy_samples_rxbuf(int idev, ssize_t nbytes_from_k)
{
	char *p_dat;
	ptrdiff_t p_inc;
	size_t nb;

	ASSERT(1 == PORT_PER_DEVICE); /* samples of P0 and P1 are interleaved */

	p_dat = iio_buffer_first(iiorxcbuf[idev], rxpi[idev*PORT_PER_DEVICE]);
	do {
		nb = min(nbytes_from_k, SUBCARRIER_SAMPLES*4);
		memcpy((void *)socktxbuf[iw_socktxbuf] + PUSCH_PLDOF, p_dat, nb/2);
		p_dat += nb;
		iw_socktxbuf++;
		if (SUBCARRIER_NTXBUF_NB == iw_socktxbuf) iw_socktxbuf = 0;
		nbytes_from_k -= nb;
	} while (nbytes_from_k > 0);
}

// Method1: sample by sample
// p_inc == iio_device_get_sample_size(rxdev[idev]) == PORT_PER_DEVICE*4
// ismp == (p_end-p_dat)/p_inc == nbytes_from_k/(PORT_PER_DEVICE*4) <= IIO_PORT_SAMPLES
// Example: swap Real(I) and Imag(Q)
void foreach_samples_rxbuf(int idev, ssize_t nbytes_from_k)
{
	ptrdiff_t p_inc;
	char *p_end, *p_dat;
	int iport, ismp;

	p_inc = iio_buffer_step(iiorxcbuf[idev]);
	p_end = iio_buffer_end(iiorxcbuf[idev]);
	p_dat = iio_buffer_first(iiorxcbuf[idev], rxpi[idev*PORT_PER_DEVICE]);
#if defined(BUILDING_ORGINAL_ADI)
	for (ismp=0; p_dat<p_end; p_dat+=p_inc, ismp++) {
		const int16_t i16 = ((int16_t*)p_dat)[0];
		const int16_t q16 = ((int16_t*)p_dat)[1];
		((int16_t*)p_dat)[0] = q16;
		((int16_t*)p_dat)[1] = i16;
	}
#elif defined(BUILDING_SIMILAR_ADI)
# if 1
	for (ismp=0; p_dat<p_end; p_dat+=p_inc, ismp++) {
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			const int16_t i16 = ((int16_t*)p_dat)[0];
			const int16_t q16 = ((int16_t*)p_dat)[1];
			((int16_t*)p_dat)[0] = q16;
			((int16_t*)p_dat)[1] = i16;
		}
	}
# else
	static struct iio_sample *psmp = (struct iio_sample *)p_dat;
	for (ismp=0; ismp<(p_end-p_dat)/p_inc; ismp++) {
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			int16_t tmp = psmp->i16;
			psmp->i16 = psmp->q16;
			psmp->q16 = tmp;
			psmp++;
		}
	}/*
	struct iio_iqsmap *pmap = (struct iio_iqsmap *)p_dat;
	for (ismp=0; ismp<(p_end-p_dat)/p_inc; ismp++) {
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			int16_t tmp = pmap->iqs[ismp][iport].i16;
			pmap->iqs[ismp][iport].i16 = pmap->iqs[ismp][iport].q16;
			pmap->iqs[ismp][iport].q16 = tmp;
		}
	}*/
# endif
#elif 0//defined(BUILDING_RRU_UL)
	struct bbu_payload *pl;
	do {
		pl = (struct bbu_payload *)txring_pusch_offset(iw_socktxbuf);
		for (ismp=0; ismp<min(nbytes_from_k/PORT_PER_DEVICE*4, SUBCARRIER_SAMPLES); ismp++) {
			for (iport=0; iport<PORT_PER_DEVICE; iport++) {
				pl->iqs[iport][ismp].i8 = *p_dat++;	// LSB of I[ismp][iport]
				p_dat++;							// MSB of I[ismp][iport]
				pl->iqs[iport][ismp].q8 = *p_dat++;	// LSB of Q[ismp][iport]
				p_dat++;							// MSB of Q[ismp][iport]
			}
		}
		nbytes_from_k -= ismp*p_inc;
		iw_socktxbuf++;
	} while (0 != nbytes_from_k);
#endif
}

// Method2: iio_buffer_foreach_sample() with callback()
// TODO: get known
ssize_t rxbuf_callback(const struct iio_channel *chn,
			void *p_smp, size_t smp_n, void *opt)
{
	if (!iio_channel_is_enabled(chn))
		return 0;
#if 0
	static int ismp=0;
	static uint8_t *p_dst;

	if (0 == ismp % SUBCARRIER_SAMPLES) {
		p_dst = (uint8_t *)socktxbuf[iw_socktxbuf] + PUSCH_PLDOF;
		switch (chn->index) {
		// IIO[ismp][p0i+p0q] -> RRU[p0i+p0q][ismp]
		case 0: break;
		case 1: p_dst += 1; break;
		// IIO[ismp][p1i+p1q] -> RRU[p1i+p1q][ismp]
		case 2: p_dst += SUBCARRIER_SAMPLES*2; break;
		case 3: p_dst += 1; break;
		default:
			printf("unknown channel id %d\n", chn->index);
			return -1;
		}
		iw_socktxbuf++;
		if (SUBCARRIER_NTXBUF_NB == iw_socktxbuf) iw_socktxbuf = 0;
	}

	*p_dst = *p_smp & 0xff; /* for example, only LSB */
	p_dst += 2;	/* p0i0, p0q0, p0i1, p0q1, ... */
	//p_smp += iio_buffer_step(iiorxcbuf[0]);
	//smp_n--;

#endif
	return iio_smp_nb;
}

// Method3: iio_channel_read_raw()
// Method4: iio_channel_read()
// TODO: get known
void chnread_samples_rxbuf(int idev, ssize_t nbytes_from_k)
{
	int iport, iblk;
	uint8_t *testbuf;

	/* p0i0, p0i1, ...	p0q0, p0q1, ... */
	testbuf = malloc(nbytes_from_k);
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		if (3 == do_smp_way) {
			iio_channel_read_raw(rxpi[iport], iiorxcbuf[0], testbuf, SUBCARRIER_SAMPLES*2);
			testbuf += SUBCARRIER_SAMPLES*2;
			iio_channel_read_raw(rxpq[iport], iiorxcbuf[0], testbuf, SUBCARRIER_SAMPLES*2);
		} else {
			iio_channel_read(rxpi[iport], iiorxcbuf[0], testbuf, SUBCARRIER_SAMPLES*2);
			testbuf += SUBCARRIER_SAMPLES*2;
			iio_channel_read(rxpq[iport], iiorxcbuf[0], testbuf, SUBCARRIER_SAMPLES*2);
		}
	}
	free(testbuf);
}

int handle_ad9371_rxpath(int idev)
{
	ssize_t nbytes_from_k;
	uint32_t val;

	nbytes_from_k = iio_buffer_refill(iiorxcbuf[idev]);
	if (mode_verbo > 1) {
		iio_device_reg_read(rxdev[idev], 0x80000088, &val);
		fprintf(stderr, "after iio_buffer_refill(): rx_dma_status=0x%02x\n", val);
	}
	if (nbytes_from_k < 0) {
		fprintf(stderr, "iio_buffer_refill() %s\n",
			strerror(-(int)nbytes_from_k));
		return -1;
	}

	if (mode_verbo > 0) {
		ASSERT(nbytes_from_k <= IIO_PORT_SAMPLES*4*PORT_PER_DEVICE);
		printf("k_iiorx -> u_libiio %u Bytes\n", (size_t)nbytes_from_k);
	}
	if (mode_verbo > 2) {
		show_libiio_rxbuf(idev);
	}

	switch (do_smp_way) {
	default:
	case 0:
		memcopy_samples_rxbuf(idev, nbytes_from_k);
		break;
	case 1:
		foreach_samples_rxbuf(idev, nbytes_from_k);
		break;
	case 2:
		iio_buffer_foreach_sample(iiorxcbuf[idev], rxbuf_callback, NULL);
		break;
	case 3:
	case 4:
		chnread_samples_rxbuf(idev, nbytes_from_k);
		break;
	}

#if defined(BUILDING_ORGINAL_ADI)
	rxsn[idev*PORT_PER_DEVICE] += nbytes_from_k/4;
	//rxsn[idev*PORT_PER_DEVICE] += nbytes_from_k/iio_buffer_step(iiorxcbuf[idev]);
#else
	for (int iport=0; iport<PORT_PER_DEVICE; iport++)
		rxsn[idev*PORT_PER_DEVICE] += nbytes_from_k/(4*PORT_PER_DEVICE);
		//rxsn[idev*PORT_PER_DEVICE] += nbytes_from_k/iio_buffer_step(iiorxcbuf[idev]);
#endif
	return 0;
}

// Example: fill with zeros
// 14-bit sample needs to be MSB alligned so shift by 2
// https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
// iio_buffer_foreach_sample();
// p_inc == iio_device_get_sample_size(txdev[idev]) == PORT_PER_DEVICE*4
// ismp == nbytes_to_k/p_inc == nbytes_to_k/(PORT_PER_DEVICE*4) <= IIO_PORT_SAMPLES
void foreach_samples_txbuf(int idev, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	char *p_end, *p_dat;
	int iport, ismp;

	p_inc = iio_buffer_step(iiotxcbuf[idev]);
	p_end = iio_buffer_end(iiotxcbuf[idev]);
	p_dat = iio_buffer_first(iiotxcbuf[idev], txpi[idev*PORT_PER_DEVICE]);
	nbytes_to_k = p_end - p_dat;
#if defined(BUILDING_ORGINAL_ADI)
	for (ismp=0; p_dat<p_end; p_dat+=p_inc, ismp++) {
		((int16_t*)p_dat)[0] = 0 << 2; // Real (I)
		((int16_t*)p_dat)[1] = 0 << 2; // Imag (Q)
	}
#elif defined(BUILDING_SIMILAR_ADI)
#if 1
	for (ismp=0; p_dat<p_end; p_dat+=p_inc, ismp++) {
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			((int16_t*)p_dat)[0] = 0 << 2; // Real (I)
			((int16_t*)p_dat)[1] = 0 << 2; // Imag (Q)
		}
	}
#else
	struct iio_sample *psmp = (struct iio_sample *)p_dat;
	for (ismp=0; ismp<(p_end-p_dat)/p_inc; ismp++) {
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			psmp->i16 = 0 << 2; // Real (I)
			psmp->q16 = 0 << 2; // Imag (Q)
			psmp++;
		}
	}/*
	struct iio_iqsmap *pmap = (struct iio_iqsmap *)p_dat;
	for (ismp=0; ismp<(p_end-p_dat)/p_inc; ismp++) {
		for (iport=0; iport<PORT_PER_DEVICE; iport++) {
			pmap->iqs[ismp][iport].i16 = 0 << 2; // Real (I)
			pmap->iqs[ismp][iport].q16 = 0 << 2; // Imag (Q)
		}
	}*/
#endif
#elif 0//defined(BUILDING_RRU_DL)
	struct bbu_payload *pl;
	int nb, tmp;
	do {
		if (iw_sockrxbuf < ir_sockrxbuf + SUBCARRIER_NRXBUF_NB/2) {
			sleep(0);
			continue;
		}
		tmp = ir_sockrxbuf + SUBCARRIER_NRXBUF_NB/2;
		nb = SUBCARRIER_NRXBUF_SZ*PORT_PER_DEVICE*4;
		do {
#if defined(BUILDING_RRU_PUSCH_B2B)// board circle-pusch for testing
			pl = (struct bbu_payload *)(&sockrxbuf[ir_sockrxbuf][0] + PUSCH_PLDOF);
#else
			pl = (struct bbu_payload *)(&sockrxbuf[ir_sockrxbuf][0] + PDSCH_PLDOF);
#endif
			for (ismp=0; ismp<SUBCARRIER_SAMPLES; ismp++) {
				for (iport=0; iport<PORT_PER_DEVICE; iport++) {
					*p_dat++ = pl->iqs[iport][ismp].i8;	// LSB of I[ismp][iport]
					p_dat++;							// MSB of I[ismp][iport]
					*p_dat++ = pl->iqs[iport][ismp].q8;	// LSB of I[ismp][iport]
					p_dat++;							// MSB of I[ismp][iport]
				}
			}
			ir_sockrxbuf++;
		} while (ir_sockrxbuf == tmp)
	} while (iw_sockrxbuf > ir_sockrxbuf + SUBCARRIER_NRXBUF_NB/2);
#endif
}

void memcopy_samples_txbuf(int idev, ssize_t nbytes_to_k)
{
	ptrdiff_t p_inc;
	char *p_end, *p_dat;
	int iport, ismp, nb, tmp;

	p_inc = iio_buffer_step(iiotxcbuf[idev]);
	p_end = iio_buffer_end(iiotxcbuf[idev]);
	p_dat = iio_buffer_first(iiotxcbuf[idev], txpi[idev*PORT_PER_DEVICE]);
	nbytes_to_k = p_end - p_dat;
#if 0
	ASSERT(1 == PORT_PER_DEVICE);
	do {
		if (iw_sockrxbuf < ir_sockrxbuf + SUBCARRIER_NRXBUF_NB/2) {
			sleep(0);
			continue;
		}
		tmp = ir_sockrxbuf + SUBCARRIER_NRXBUF_NB/2;
		nb = SUBCARRIER_NRXBUF_SZ*PORT_PER_DEVICE*4;
		do {
			memcpy(p_dat, sockrxbuf[ir_sockrxbuf] + PDSCH_PLDOF, nb/2);
			p_dat += nb/2;
			memcpy(p_dat, sockrxbuf[ir_sockrxbuf] + PDSCH_PLDOF, nb/2);
			p_dat += nb/2;
			ir_sockrxbuf++;
		} while (ir_sockrxbuf == tmp);
		for (iport=0; iport<PORT_PER_DEVICE; iport++) txsn[iport] += ismp;
		break;
	} while (1);
#endif
}

int handle_ad9371_txpath(int idev)
{
	ssize_t nbytes_to_k;
	int iport;
	uint32_t val;

	nbytes_to_k = iio_buffer_push(iiotxcbuf[idev]);
	if (mode_verbo > 1) {
		iio_device_reg_read(txdev[idev], 0x80000088, &val);
		fprintf(stderr, "after iio_buffer_push(): tx_dma_status=0x%02x\n", val);
	}
	if (nbytes_to_k < 0) {
		fprintf(stderr, "Unable to push buffer: %s\n",
			strerror(-(int)nbytes_to_k));
		return -1;
	}
	if (mode_verbo > 0) {
		ASSERT(nbytes_to_k <= IIO_PORT_SAMPLES*4*PORT_PER_DEVICE);
		printf("u_libiio -> k_iiotx %u Bytes\n", (size_t)nbytes_to_k);
	}

	switch (do_smp_way) {
	default:
	case 0:
		memcopy_samples_txbuf(idev, nbytes_to_k);
		break;
	case 1:
		foreach_samples_txbuf(idev, nbytes_to_k);
		break;
	case 2:
		printf("\ncase2 TODO\n");
		break;
	case 3:
	case 4:
		printf("\ncase3/4 TODO\n");
		break;
	}

#if defined(BUILDING_ORGINAL_ADI)
	txsn[idev*PORT_PER_DEVICE] += nbytes_to_k/4;
	//txsn[idev*PORT_PER_DEVICE] += nbytes_from_k/iio_buffer_step(iiorxcbuf[idev]);
#else
	for (iport=0; iport<PORT_PER_DEVICE; iport++)
		txsn[idev*PORT_PER_DEVICE] += nbytes_to_k/(4*PORT_PER_DEVICE);
		//txsn[idev*PORT_PER_DEVICE] += nbytes_to_k/iio_buffer_step(iiorxcbuf[idev]);
#endif

	if (mode_verbo > 2) {
		show_libiio_txbuf(idev);
	}

	return 0;
}

//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiorxdma_overflow(int idev)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(rxdev[idev], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "rxdev[%d] read status register failed %s\n",
			idev, strerror(-ret));
		return -1;
	}
	printf("rxdev[%d]: reg[0x80000088]=0x%08x\n", idev, val);

	//Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(rxdev[idev], 0x80000088, val);
	if (val & 0x04) {
		fprintf(stderr, "rxdev[%d] Overflow detected!\n", idev);
		return 1;
	}

	return 0;
}

//refer to: monitor_thread_fn() in "iio_adi_xflow_check.c"
int is_iiotxdma_underflow(int idev)
{
	int ret;
	uint32_t val;

	ret = iio_device_reg_read(txdev[idev], 0x80000088, &val);
	if (0 != ret) {
		fprintf(stderr, "txdev[%d] read status register failed: %s\n",
			idev, strerror(-ret));
		return -1;
	}
	printf("txdev[%d]: reg[0x80000088]=0x%08x\n", idev, val);

	// Clear status bits by writting value back
	if (0 != val) iio_device_reg_write(txdev[idev], 0x80000088, val);
	if (val & 0x01) {
		fprintf(stderr, "txdev[%d] Underflow detected\n", idev);
		return 1;
	}

	return 0;
}

void profile_ad9371_phy_array(struct iio_device *phy);
void libiio_app_startup(int iios)
{
	int idev, iport;

	printf("* Acquiring IIO context\n");
#if 0
	ASSERT((ctx = iio_create_default_context()) && "No context");
#else
	ASSERT((ctx = iio_create_local_context()) && "No context");
#endif
	ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

	/* ad9371-phy(iio:device1) */
		/* PHY_RX[0/1/2/3](iio:device1/in_voltage[n]_xxx */
		/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
	if (mode_verbo > 0)
		printf("* Acquiring AD9371 PHY device\n");
	struct iio_device *phy = get_ad9371_phy(ctx);
	printf("* Profiling AD9371 PHY device\n");
	profile_ad9371_phy_array(phy);
	printf("* Initializing AD9371-PHY channels\n");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		if (iio_rxp_en)
			ASSERT(cfg_ad9371_streaming_ch(ctx, &rxcfg, RX, iport) \
				&& "No phy_rxport found");
		if (iio_txp_en)
			ASSERT(cfg_ad9371_streaming_ch(ctx, &txcfg, TX, iport) \
				&& "No phy_txport found");
	}

	for (idev=0; idev<CHIP_NUM_TOTAL; idev++) {
		if (iio_rxp_en)
			init_ad9371_rxpath(idev, iios);
		if (iio_txp_en)
			init_ad9371_txpath(idev, iios);
	}
}

#if defined(BUILDING_ORGINAL_ADI) || defined(BUILDING_SIMILAR_ADI)
int main(int argc, char *argv[])
{
	int iport;
	struct timespec tm_xs, tm_xe;
	double tm_us;

	get_args(argc, argv);

	libiio_app_startup(iio_smp_nb);

	// Listen to ctrl+c and ASSERT
	signal(SIGINT, handle_sig);

	if (mode_verbo > 0)
		printf("* Starting IO streaming (press CTRL+C to cancel)\n");
	clock_gettime(CLOCK_MONOTONIC, &tm_xs);

	do {
		if (iio_rxp_en) {
			handle_ad9371_rxpath(0);
			printf("\r\tRX[0] %4.6f MSmp (+ %u)", rxsn[0]/1e6, iio_smp_nb);
			fflush(stdout);
		}
		if (iio_txp_en) {
			handle_ad9371_txpath(0);
			printf("\r\tTX[0] %4.6f MSmp (+ %u)", txsn[0]/1e6, iio_smp_nb);
			fflush(stdout);
		}
	} while (!b_process_stop);

	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
	tm_us = elapse_us(&tm_xe, &tm_xs);
	printf("\n");
	for (iport=0; iport<PORT_PER_DEVICE; iport++) {
		if (iio_rxp_en)
			printf("RX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\n",
				iport, rxsn[iport]/1e6, tm_us, rxsn[iport]/tm_us*32);
		if (iio_txp_en)
			printf("TX[%d] Total %4.6f MSmp in %f us (Throughput %7.3f Mbps)\n",
				iport, txsn[iport]/1e6, tm_us, txsn[iport]/tm_us*32);
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

void profile_ad9371_phy_file(int idev, char *fn)
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
	printf("Profile write end\n");
	free(buf);
}
