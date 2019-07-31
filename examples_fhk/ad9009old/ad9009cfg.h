/**
 * \file Talise_adrv9009_stream.c
 * \brief Contains Talise default iio-stream func.
 *
 * Copyright (C) 2014 IABG mbH
 * Original Author: Michael Feilen <feilen_at_iabg.de>
 * Modified Author: Yang Yang <page.y.yang._at_foxconn.com>
 * Copyright (C) 2017 Analog Devices Inc.
 * Copyright (C) 2018 Foxconn 5G Research Institute
 *
*/

#ifndef TALISE_ADRV9009_H_
#define TALISE_ADRV9009_H_


#include "iio-private.h"
#include "talise_types.h"
#include "talise_config.h"
#include "talise_error.h"
#include "ad9009profile100mhz.h"
#include <time.h>
#include "comp.h"
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

//#define DEBUG

//GPIO Control Memeory Address
//#define TXMEM 0xff220000
//#define OFFSET1 0xe030 //mode number (4bits) + division (4bits)
//#define OFFSET2 0xe020 //Select channel
//#define MEM_MAP_SIZE 0x10000
//#define MEM_MAP_MASK MEM_MAP_SIZE-1

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

/* if expr=0, assertion failed and abort */
#define ASSERT(expr) { \
    if (!(expr)) { \
        (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
        (void) abort(); \
    } \
}

/* if expr<0, assertion failed and abort */
#define ASSERT_ATTR(expr) { \
    if (expr < 0) { \
        (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
        (void) abort(); \
    } \
}

/* RX and ORX is input, TX is output */
enum iodev { RX, TX, ORX};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_channel *tx1_i = NULL;
static struct iio_channel *tx1_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

static bool stop;

/* config structs required for channel */
typedef struct channel_cfg{
    long long rx_gain;
    long long tx_atten;
    bool tx_qtracking;
    bool tx_loltracking;
    bool rx_qtracking;
    bool rx_hd2tracking;
    bool pa_protection;
    long long lo_freq;    //tx & rx frequency for TDD
    bool enb_txFPGA;
    bool enb_rxFPGA;
    bool loop;
    bool cycle;
    bool tcal;
    bool rcal;
    uint8_t RFICseq;
}channel_cfg;

/* config structs required for stream */
typedef struct stream_cfg{
    char *src_fn;
    char *dst_fn;
    bool tx_on;
    bool rx_on;
    bool no_stop;
    size_t frame_num;
    bool tx1;
    bool tx2;
    uint8_t rx1;
}stream_cfg;

typedef struct timespec timespec;


int memfd = -1;
int runtime = 0;
void *mapped_dev_base = NULL;

void command_parse(int argc, char** argv, channel_cfg *cfg, stream_cfg *scfg){
    int i = 1;
    for (;i < argc; i++){
        if (strcmp(argv[i], "-tx") == 0)
            scfg->tx_on = 1;
        else if (strcmp(argv[i], "-tx1") == 0)
            scfg->tx1 = 1;
        else if (strcmp(argv[i], "-tx2") == 0)
            scfg->tx2 = 1;
        else if (strcmp(argv[i], "-nrx") == 0)
            scfg->rx_on = 0;
        else if (strcmp(argv[i], "-rx1") == 0){
            scfg->rx_on = 1;
            scfg->rx1 = 0;}
        else if (strcmp(argv[i], "-rx2") == 0){
            scfg->rx_on = 1;
            scfg->rx1 = 1;}
        else if (strcmp(argv[i], "-orx1") == 0){
            scfg->rx_on = 1;
            scfg->rx1 = 2;}
        else if (strcmp(argv[i], "-orx2") == 0){
            scfg->rx_on = 1;
            scfg->rx1 = 3;}
        else if (strcmp(argv[i], "-stop") == 0)
            scfg->no_stop = 0;
        else if (strcmp(argv[i], "-ten") == 0)
            cfg->enb_txFPGA = 1;
        else if (strcmp(argv[i], "-ren") == 0)
            cfg->enb_rxFPGA = 1;
        else if (strcmp(argv[i], "-loop") == 0)
            cfg->loop = 1;
        else if (strcmp(argv[i], "-cyc") == 0)
            cfg->cycle = 1;
        else if (strcmp(argv[i], "-rcal") == 0)
            cfg->rcal = 1;
        else if (strcmp(argv[i], "-tcal") == 0)
            cfg->tcal = 1;
        else if (strcmp(argv[i], "-txat") == 0)
            cfg->tx_atten = -atof(argv[++i]);
        else if((strcmp(argv[i], "-rxgain") == 0)){
            cfg->rx_gain = atof(argv[++i]);
            scfg->rx_on = 1;}
        else if((strcmp(argv[i], "-num") == 0))
            cfg->RFICseq = atoi(argv[++i]);
        else if((strcmp(argv[i], "-txqt") == 0))
            cfg->tx_qtracking = (atoi(argv[++i]) > 0);
        else if((strcmp(argv[i], "-lo") == 0))
            cfg->tx_loltracking = (atoi(argv[++i]) > 0);
        else if((strcmp(argv[i], "-rxqt") == 0))
            cfg->rx_qtracking = (atoi(argv[++i]) > 0);
        else if((strcmp(argv[i], "-rxhd2t") == 0))
            cfg->rx_hd2tracking = (atoi(argv[++i]) > 0);
        else if((strcmp(argv[i], "-papt") == 0))
            cfg->pa_protection = (atoi(argv[++i]) > 0);
        else if((strcmp(argv[i], "-lofq") == 0))
            cfg->lo_freq = GHZ(atof(argv[++i]));
        else if((strcmp(argv[i], "-fno") == 0)){
            //printf("-fno %s %d\n", argv[i+1], atoi(argv[i+1]));
            scfg->frame_num = atoi(argv[++i]);
            scfg->rx_on = 1;}
        else if((strcmp(argv[i], "-src") == 0)){
            scfg->src_fn = argv[++i];
            scfg->tx_on = 1;
            scfg->no_stop = 1;
            scfg->rx_on = 0;}
        else if((strcmp(argv[i], "-dst") == 0))
            scfg->dst_fn = argv[++i];
        else if(strcmp(argv[i], "-help") == 0){
            printf("Options:\n");
            printf("\t-tx1 or -tx2\t\tEnable tx1 or tx2\n");
            printf("\t-rx1 or -rx2\t\tEnable rx1 or rx2\n");
            printf("\t-orx1 or -orx2\t\tEnable orx1 or orx2\n");
            printf("\t-enb\t\tEnable FPGA BBP modules\n");
            printf("\t-cyc\t\tEnable cycle transmission\n");
            printf("\t-txat\tTx attenuation (dB)\n");
            printf("\t-rxgain\t\tRx gain (dB)\n");
            printf("\t-txqt\t\tTx quadrature tracking enable (bool)\n");
            printf("\t-rxqt\t\tRx quadrature tracking enable (bool)\n");
            printf("\t-rxhd2t\t\tRx hd2 tracking enable (bool)\n");
            printf("\t-papt\t\tPA protection enable (bool)\n");
            printf("\t-txlolt\t\tTx LO leakage tracking enable (bool)\n");
            printf("\t-lofq\t\tTx&Rx Lo Frequency (GHz)\n");
            printf("\t-src\t\tsource data file path\n");
            printf("\t-dst\t\tdstination data file path\n");
            exit(0);}
        else if (strcmp(argv[i], "-rt") == 0)
            runtime = atof(argv[++i]);
        else
        {
            printf("Wrong Option\n");
            exit(0);
        }
    }
}


/* cleanup and exit */
void shutdown()
{
    //if (mapped_dev_base){
    //    printf("* Close GPIO Memory\n");
    //    if (munmap(mapped_dev_base, MEM_MAP_SIZE) == -1)
    //    {
    //        printf("Can't unmap memory from user space.\n");
    //        exit(0);
    //    }
    //}
    //if (memfd != -1)
    //    close(memfd);

    printf("* Destroying buffers\n");
    if (rxbuf) { iio_buffer_destroy(rxbuf); }
    if (txbuf) { iio_buffer_destroy(txbuf); }

    printf("* Disabling streaming channels\n");
    if (rx0_i) { iio_channel_disable(rx0_i); }
    if (rx0_q) { iio_channel_disable(rx0_q); }
    if (tx0_i) { iio_channel_disable(tx0_i); }
    if (tx0_q) { iio_channel_disable(tx0_q); }
    if (tx1_i) { iio_channel_disable(tx1_i); }
    if (tx1_q) { iio_channel_disable(tx1_q); }

    printf("* Destroying context\n");
    if (ctx) { iio_context_destroy(ctx); }
    exit(0);
}

/* check return value of attr_write function */
void errchk(int v, const char* what) {
     if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown(); }
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

	printf("\t %s: %lld\n", what, val);
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

/* returns adrv9009 phy device */
static struct iio_device* get_adrv9009_phy(struct iio_context *ctx)
{
#if 1
	struct iio_device *dev =  iio_context_find_device(ctx, "adrv9009-phy");
#else
	struct iio_device *dev =  iio_context_find_device(ctx, "adrv9009n2-phy");
#endif
	ASSERT(dev && "No adrv9009-phy found");
	return dev;
}

/* finds adrv9009 streaming IIO devices, JESD204B device, eg: Tx-link, Rx-link, ORx-link*/
static bool get_adrv9009_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
	switch (d) {
#if 0
	case RX : *dev = iio_context_find_device(ctx, "jesd204rx-layer3"); return *dev != NULL;
	case TX : *dev = iio_context_find_device(ctx, "jesd204tx-layer3"); return *dev != NULL;
	case ORX: *dev = iio_context_find_device(ctx, "jesd204ro-layer3"); return *dev != NULL;
#endif
#if 1
	case RX : *dev = iio_context_find_device(ctx, "axi-adrv9009-rx-hpc"); return *dev != NULL;
	case TX : *dev = iio_context_find_device(ctx, "axi-adrv9009-tx-hpc"); return *dev != NULL;
	case ORX: *dev = iio_context_find_device(ctx, "axi-adrv9009-rx-obs-hpc"); return *dev != NULL;
#else
	case RX : *dev = iio_context_find_device(ctx, "axi-adrv9009n2-rx-hpc"); return *dev != NULL;
	case TX : *dev = iio_context_find_device(ctx, "axi-adrv9009n2-tx-hpc"); return *dev != NULL;
	case ORX: *dev = iio_context_find_device(ctx, "axi-adrv9009n2-rx-obs-hpc"); return *dev != NULL;
#endif
	default: ASSERT(0); return false;
	}
}

/* finds adrv9009 streaming IIO channels, JESD204B Link-channel, eg: Tx-link, Rx-link, ORx-link */
static bool get_adrv9009_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, char modify, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	return *chn != NULL;
}

/* finds adrv9009 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
      // Rx chan is always input, i.e. false
      // Tx chan is always output, i.e. true
	case RX: *chn = iio_device_find_channel(get_adrv9009_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
	case TX: *chn = iio_device_find_channel(get_adrv9009_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
	default: ASSERT(0); return false;
	}
}

/* finds adrv9009 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, struct iio_channel **chn)
{
    // TRx_LO chan is always output, i.e. true
    *chn = iio_device_find_channel(get_adrv9009_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
}

/* power down channel*/
static bool power_down_chan(struct iio_context *ctx, int chid)
{
    struct iio_channel *chn = NULL;
    bool powerdown;
    printf("* Power down Rx channel %d\n", chid);
    if (!get_phy_chan(ctx, 0, chid, &chn)) {return false;} //rx
    ASSERT_ATTR(iio_channel_attr_write_bool(chn, "powerdown", 1));
    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    printf("powerdown status is %d\n", powerdown);
    return true;
}

/* power up channel*/
static bool power_up_chan(struct iio_context *ctx, int chid)
{
    struct iio_channel *chn = NULL;
    bool powerdown;
    printf("* Power up Rx channel %d\n", chid);
    if (!get_phy_chan(ctx, 0, chid, &chn)) {return false;} //rx
    ASSERT_ATTR(iio_channel_attr_write_bool(chn, "powerdown", 0));
    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    printf("channel powerdown status is %d\n", powerdown);
    return true;
}


/* applies streaming configuration through IIO */
bool cfg_adrv9009_streaming_ch(struct iio_context *ctx, struct channel_cfg *cfg,  int chid, int tx1rx0)
{
    struct iio_channel *chn = NULL;
    double gain, atten;
    long long val;

    if(tx1rx0==1)	//tx1rx0=1: config Tx
    {
        // get phy
        printf("* Acquiring ADRV9009 phy Tx channel %d\n",  chid);
        if (!get_phy_chan(ctx, 1, chid, &chn)) {return false;}
        // Configure channels parameter
        printf("* Configure hardwaregain ADRV9009 Tx channel: %d\n",  chid);
        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->tx_atten));

        printf("Read Channel %d Tx attenuation\n", chid);
        iio_channel_attr_read_double(chn, "hardwaregain", &atten);
        printf("Channel %s Tx attenuation is %f\n", iio_channel_get_name(chn), atten);

        printf("* Acquiring ADRV9009 TX-LO channel\n");
        if (!get_lo_chan(ctx, &chn)) { return false;}
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "frequency", cfg->lo_freq));
        iio_channel_attr_read_longlong(chn, "frequency", &val);
        printf("TX frequency is %lld\n",val);
    }
    if(tx1rx0==0)    //tx1rx0=0: config Rx
    {
        // Configure phy RX
        printf("* Acquiring ADRV9009 phy RX channel %d\n", chid);
        if (!get_phy_chan(ctx, 0, chid, &chn)) {return false;}
        // Configure channels parameter
        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->rx_gain));

        printf("Read Channel %d Rx attenuation\n", chid);
        iio_channel_attr_read_double(chn, "hardwaregain", &gain);
        printf("Channel %s Rx gain is %f\n", iio_channel_get_name(chn), gain);

        printf("* Acquiring ADRV9009 RX-LO channel\n");
        if (!get_lo_chan(ctx, &chn)) { return false;}
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "frequency", cfg->lo_freq));
        iio_channel_attr_read_longlong(chn, "frequency", &val);
        printf("RX frequency is %lld\n",val);
    }
    if(tx1rx0==2)    //tx1rx0=0: config ORx
    {
        // Configure phy ORX
        printf("* Acquiring ADRV9009 phy ORX channel %d\n", chid);
        if (!get_phy_chan(ctx, 0, chid, &chn)) {return false;}
        // Configure channels parameter

        power_down_chan(ctx, 0);  //power down rx1
        power_down_chan(ctx, 1);  //power down rx2

        power_up_chan(ctx, chid);  //power up orx2
        printf("* power up ADRV9009 ORX2 phy channel\n");

        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->rx_gain));

        iio_channel_attr_read_double(chn, "hardwaregain", &gain);
        printf("Channel %s ORx gain is %f\n", iio_channel_get_name(chn), gain);

        printf("* Acquiring ADRV9009 ORX-LO channel\n");
        if (!get_lo_chan(ctx, &chn)) { return false;}
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "frequency", cfg->lo_freq));
        iio_channel_attr_read_longlong(chn, "frequency", &val);
        printf("ORX frequency is %lld\n",val);
    }

	return true;

}

void print_channel_name(struct iio_channel *chn){
    printf("Channe name is %s \n", iio_channel_get_name(chn));
}

bool print_adrv9009_channel_attr(struct iio_context *ctx, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    printf("* Acquiring ADRV9009 phy %s channel %d\n", type == TX ? "TX" : "RX", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) {return false;}

    int i;
    printf("\nPhy channel attributes list, %d item in total\n",chn->nb_attrs);
    for (i = 0; i < chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

    printf("* Acquiring ADRV9009 TRX-LO channel\n");
    if (!get_lo_chan(ctx, &chn)) { return false;}

    printf("\nTRX-LO channel attributes list, %d item in total\n",chn->nb_attrs);
    for (i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

    printf("* Acquiring ADRV9009 ORX1 lo channel\n");
    chn = iio_device_find_channel(get_adrv9009_phy(ctx), get_ch_name("altvoltage", 2), true);
    for (i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

	printf("* Acquiring ADRV9009 ORX2 lo channel\n");
    chn = iio_device_find_channel(get_adrv9009_phy(ctx), get_ch_name("altvoltage", 3), true);
    for (i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));
    return true;
}

bool print_adrv9009_channel_attr2(struct iio_channel *chn)
{
    ASSERT(chn);
    printf("Channel name is %s, channel type is %d\n", iio_channel_get_name(chn),iio_channel_get_type(chn));
    for (int i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    return true;
}

//Loading the adrv9009 profile generated by the TES
void load_adrv9009_profile(struct iio_device *phy, taliseInit_t *talInit)
{
    long long val;

    //Tx setting data structure
    taliseTxSettings_t *txsetting = &talInit->tx;

    //enabled channels
    val = txsetting->txChannels;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-channels-enable", val);

    //tx data options if pll unlock
    val = txsetting->disTxDataIfPllUnlock;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-data-options-if-pll-unlock", val);

    //PLL Lo Frequency_Hz
    //val = txsetting->txPllLoFrequency_Hz;
    //iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-pll-lo-frequency_hz", val);

    //JESD204b deframer select
    val = txsetting->deframerSel;
    iio_device_debug_attr_write_longlong(phy, "adi,adi,tx-settings-deframer-sel", val);

    //tx atten step size
    val = txsetting->txAttenStepSize;
    iio_device_debug_attr_write_longlong(phy, "adi,adi,tx-settings-tx-atten-step-size", val);

    //tx1, tx2 atten, mdB
    val = txsetting->tx2Atten_mdB;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx2-atten_mdb", val);
    val = txsetting->tx1Atten_mdB;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx1-atten_mdb", val);

    //txprofile
    taliseTxProfile_t *txp = &txsetting->txProfile;
    val = txp->txBbf3dBCorner_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-bbf-3db-corner_khz", val);
    val = txp->txDac3dBCorner_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-dac-3db-corner_khz", val);
    val = txp->rfBandwidth_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-rf-bandwidth_hz", val);
    val = txp->primarySigBandwidth_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-primary-sig-bandwidth_hz", val);
    val = txp->txInputRate_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-input-rate_khz", val);
    val = txp->txInt5Interpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-int5-interpolation", val);
    val = txp->thb1Interpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb1-interpolation", val);
    val = txp->thb2Interpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb2-interpolation", val);
    val = txp->thb3Interpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb3-interpolation", val);
    val = txp->txFirInterpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-fir-interpolation", val);
    val = txp->dacDiv;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-dac-div", val);


    //Rx
    taliseRxSettings_t *rxsetting = &talInit->rx;

    //val = rxsetting->rxPllLoFrequency_Hz;
    //iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-lo-frequency_hz", val);
    //val = rxsetting->rxPllUseExternalLo;
    //iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-use-external-lo", val);
    val = rxsetting->rxChannels;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-channels-enable", val);
    //val = rxsetting->realIfData;
    //iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-real-if-data", val);
    //JESD204b framer select
    val = rxsetting->framerSel;
    iio_device_debug_attr_write_longlong(phy, "adi,adi,rx-settings-framer-sel", val);

    taliseRxProfile_t *rxp = &rxsetting->rxProfile;
    val = rxp->rxBbf3dBCorner_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-bbf-3db-corner_khz", val);
    val = rxp->rfBandwidth_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rf-bandwidth_hz", val);
    val = rxp->rxOutputRate_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-output-rate_khz", val);
    val = rxp->rhb1Decimation;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rhb1-decimation", val);
    val = rxp->rxDec5Decimation;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-dec5-decimation", val);
    val = rxp->rxFirDecimation;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-fir-decimation", val);

}


void load_adrv9009_profile_simplified(struct iio_context *ctx, struct iio_device *phy, char *fn){
    printf("Load profile from %s\n",fn);
    FILE *fd = fopen(fn, "r");
    if(!fd)
        printf("Cannot open the profile file\n");
    fseek(fd, 0, SEEK_END);  //fp pointer back to end-file
    size_t len = ftell(fd);      //get file size by pointer offset
    //printf("%d\n", len);
    char *buf = malloc(len);
    fseek(fd,0,SEEK_SET);
    len = fread(buf,1,len,fd);
    fclose(fd);

    printf("the talise profile's length is:%d\n", len);
    iio_context_set_timeout(ctx, 30000);
    ASSERT_ATTR(iio_device_attr_write_raw(phy,"profile_config",buf,len));
    printf("Profile write finished.\n");
    iio_context_set_timeout(ctx, 3000);
    free(buf);
}


#if 0
void read_ad9371_profile(struct iio_context *ctx, struct iio_device *phy, char *fn){
    printf("Read profile to %s\n",fn);
    FILE *fd = fopen(fn, "w");
    if(!fd)
        printf("Cannot open the profile file\n");
    size_t len = 4096;
    char *buf = malloc(len);

    //printf("%d\n", len);
    ASSERT_ATTR(iio_device_attr_read(phy,"profile_config",buf,len));
    printf("Profile read finished.\n");
    iio_context_set_timeout(ctx, 3000);
    len = fwrite(buf,1,len,fd);
    free(buf);
    fclose(fd);
}
#endif

void handle_sig(int sig)
{
    printf("Waiting for process to finish...\n");
    stop = true;
}

long long timediff(timespec a, timespec b){
    return (1000000000*(b.tv_sec-a.tv_sec) + b.tv_nsec-a.tv_nsec );
}

/*
void* OpenMemory(off_t dev_base, size_t size){
    size_t mask = size-1;
    void *mapped_base;
    if (memfd == -1){
        memfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (memfd == -1)
        {
            printf("Can't open /dev/mem.\n");
            exit(0);
        }
        printf("/dev/mem opened.\n");
    }

    // Map one page of memory into user space such that the device is in that page, but it may not
    // be at the start of the page.

    mapped_base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, dev_base & ~mask);
    if (mapped_base == (void *) -1)
    {
        printf("Can't map the memory to user space.\n");
        exit(0);
    }
    printf("Memory mapped at address %p.\n", mapped_base);
    if (dev_base != TXMEM)
        return (mapped_base + (dev_base & mask));
    else
        mapped_dev_base = mapped_base + (dev_base & mask);
}
*/

/*int print_debug_attr(struct iio_device *dev, const char *attr, const char *val, size_t len, void *d){
    long long attr_val;
    memcpy(&attr_val, val, len);
    printf("%s = %lld\n", attr, attr_val);
    return 1;
}*/
#endif //TALISE_ADRV9009
