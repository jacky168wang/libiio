/*
 * applications based on libiio
 *   - RFIC IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
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

/* common PHY RF params */
struct rfcfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
    double gain;
    bool qec_tcal;
    bool lol_tcal;//TX-ONLY
	bool pwrdown;
};

struct rfcfg rxchn_rfcfg = {
	0, 0, GHZ(3.5), 0, 1, 0
};

struct rfcfg txchn_rfcfg = {
	0, 0, GHZ(3.5), -20, 1, 0
};

/* IIO structs required for streaming */
static struct iio_context *ctx;

// PHY devices
static struct iio_device *rfdev[TOT_CHIP_NB];

#define EN_IIO_PATH_RX (1 << RX)
#define EN_IIO_PATH_TX (1 << TX)
       unsigned int g_pth_msk = EN_IIO_PATH_TX | EN_IIO_PATH_RX;
static unsigned int g_m_verbo = 0;

static unsigned int g_chn_msk = BIT(0); // 2T2R
//static unsigned int g_chn_msk = BIT(0) | BIT(1); // 4T4R
//static unsigned int g_chn_msk = BIT(0) | BIT(1) | BIT(2) | BIT(3); // 8T8R

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
		" -p\tprofile RFIC with RX/TX/OR signal-path, clock/lo-path, digital/jesd-path\r\n"
        " -f\tset RFIC frequency for RX or TX\r\n"
        " -g\tset RFIC Gain for RX or Attention for TX\r\n"
		" -q\tdisable RFIC QEC-TrackingCalibration for RX or TX\r\n"
		" -t\tdisable RFIC LOL-TrackingCalibration for TX only\r\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
static void get_args(int argc, char **argv)
{
	int c;
	while ((c = getopt(argc, argv, "x:c:p:f:g:q:t:v:h")) != EOF) {
		switch (c) {
		case 'x': g_pth_msk = strtoul(optarg, NULL, 16);			break;
		case 'c': g_chn_msk = strtoul(optarg, NULL, 16);			break;
	    case 'p':
	        memset(profilename, 0x00, tmpsize);
	        tmpsize -= 1;
	        if(tmpsize > strlen(optarg))
	            tmpsize = strlen(optarg);
	        memcpy(profilename, optarg, tmpsize);
	        break;
	    case 'f':
			rxchn_rfcfg->lo_hz = GHZ(strtoul(optarg, NULL, 0));
			txchn_rfcfg->lo_hz = GHZ(strtoul(optarg, NULL, 0));		break;
	    case 'g':
			rxchn_rfcfg->gain = strtoul(optarg, NULL,  0);	
	    	txchn_rfcfg->gain = strtoul(optarg, NULL,  0);			break;
	    case 'q':
			rxchn_rfcfg->qec_tcal = strtoul(optarg, NULL,  0);	
			txchn_rfcfg->qec_tcal = strtoul(optarg, NULL,  0);		break;
	    case 't': 	
	    	txchn_rfcfg->lol_tcal = strtoul(optarg, NULL,  0);		break;
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
	printf( "\r\n" );
}

/* cleanup and exit */
static void shutdown(void)
{
	if (NULL != ctx) {
		iio_context_destroy(ctx);
		ctx = NULL;
	}
	exit(0);
}

static volatile sig_atomic_t stop = 0;
static void handle_sig(int sig)
{
	printf("\r\nWaiting for process to finish...\r\n");
	stop = 1;
}

/* finds rfic IIO devices */
static struct iio_device *get_phy_dev(struct iio_context *ctx, int ig)
{
	struct iio_device *dev;
	const char *devname_phy = {
#if 0
        "ad9371-phy",
#else
		"adrv9009-phy",
#endif
	};

	if (ig == 0) {
		dev = iio_context_find_device(ctx, devname_phy);
	} else {
		sscanf(tmpstr, "%s-n%d", devname_phy, ig+1);
		dev = iio_context_find_device(ctx, tmpstr);
	}
	ASSERT((NULL != dev) && "iio device not found");
	return dev;
}

/* finds rfic datapath channels */
/* ad9371-phy(iio:device1) */
	/* PHY_RX[0/1] (iio:device1/ in_voltage[0/1] */
	/* PHY_TX[0/1] (iio:device1/out_voltage[0/1] */
static struct iio_channel *get_phy_chn_rf(struct iio_device *phy, enum iopath p, int ip)
{
	struct iio_channel *chn;

	chn = iio_device_find_channel(phy, GET_CHN_NAME("voltage", ip, '\0'), p == TX);
	ASSERT((NULL != chn) && "No channel found");
	if (g_m_verbo > 0)
        printf("* Acquiring PHY channel %s\r\n", iio_channel_get_name(chn));
	return chn;
}

static bool dmp_phy_chn_rf(struct iio_channel *chn, enum iopath p, struct rfcfg *cfg)
{
	int ret;

	R_CHN_BOO(chn, "powerdown", &cfg->pwrdown);
	R_CHN_LLI(chn, "rf_bandwidth", &cfg->bw_hz);
	R_CHN_LLI(chn, "sampling_frequency", &cfg->fs_hz);
	R_CHN_DBL(chn, "hardwaregain", &cfg->gain);
    R_CHN_BOO(chn, "quadrature_tracking_en", &cfg->qec_tcal);
	if (TX == p)
		R_CHN_BOO(chn, "lo_leakage_tracking_en", &cfg->lol_tcal);

	return true;
}

static bool cfg_phy_chn_rf(struct iio_channel *chn, enum iopath p, struct rfcfg *cfg)
{
	int ret;
#if 1
	W_CHN_BOO(chn, "powerdown", false);
	W_CHN_LLI(chn, "rf_bandwidth", cfg->bw_hz);
	W_CHN_LLI(chn, "sampling_frequency", cfg->fs_hz);
	wr_ch_dbl(chn, "hardwaregain", cfg->gain);
	W_CHN_BOO(chn, "quadrature_tracking_en", cfg->qec_tcal);
	if (TX == p)
		W_CHN_BOO(chn, "lo_leakage_tracking_en", cfg->lol_tcal);
#else
	IIO_ATTR(chn, "powerdown", write, bool, false);
	IIO_ATTR(chn, "rf_bandwidth", write, longlong, cfg->bw_hz);
	IIO_ATTR(chn, "sampling_frequency", write, longlong, cfg->fs_hz);
	IIO_ATTR(chn, "hardwaregain", write, double, cfg->gain);
	if (TX == p)
		IIO_ATTR(chn, "lo_leakage_tracking_en", write, bool, cfg->qec_tcal);
#endif

	return true;
}

/* finds local-oscillator configuration channels: always output, i.e. true */
static struct iio_channel *get_phy_chn_lo(struct iio_device *phy, enum iopath p)
{
	struct iio_channel *chn;

	if (g_m_verbo > 0)
		printf("* Acquiring device 'PHY' lo-channel '%s'\r\n", p == TX ? "TX" : "RX");
	chn = iio_device_find_channel(phy, GET_CHN_NAME("altvoltage", p, '\0'), true);
	ASSERT((NULL != chn) && "No channel found");
	return chn;
}

static bool dmp_phy_chn_lo(struct iio_channel *chn, enum iopath p, struct rfcfg *cfg)
{
	int ret;

    switch (p) {
	case RX:
#if 0
        R_CHN_LLI(chn, "RX_LO_frequency", &cfg->lo_hz);
#else
        R_CHN_LLI(chn, "frequency", &cfg->lo_hz);
#endif
		break;
	case TX:
#if 0
		R_CHN_LLI(chn, "TX_LO_frequency", &cfg->lo_hz);
#else
        R_CHN_LLI(chn, "frequency", &cfg->lo_hz);
#endif
		break;
	case OR:
#if 0
        //chn = iio_device_find_channel(phy, chnname("altvoltage", 2), true);
        R_CHN_LLI(chn, "RX_SN_LO_frequency", &cfg->lo_hz);
#else
        R_CHN_LLI(chn, "frequency", &cfg->lo_hz);
#endif
		break;
	}
}

static bool cfg_phy_chn_lo(struct iio_channel *chn, enum iopath p, struct rfcfg *cfg)
{
	int ret;
    switch (p) {
	case RX:
#if 0
        W_CHN_LLI(chn, "RX_LO_frequency", cfg->lo_hz);
#else
        W_CHN_LLI(chn, "frequency", cfg->lo_hz);
#endif
		break;
	case TX:
#if 0
		W_CHN_LLI(chn, "TX_LO_frequency", cfg->lo_hz);
#else
        W_CHN_LLI(chn, "frequency", cfg->lo_hz);
#endif
		break;
	case OR:
#if 0
        //chn = iio_device_find_channel(phy, chnname("altvoltage", 2), true);
        W_CHN_LLI(chn, "RX_SN_LO_frequency", cfg->lo_hz);
#else
        W_CHN_LLI(chn, "frequency", cfg->lo_hz);
#endif
		break;
	}
}

static void dmp_phy_dbg(struct iio_device *phy)
{
    long long val;

	W_DBG_LLI(phy[i],"adi,tx-settings-tx-pll-lo-frequency_hz", &val);
	W_DBG_LLI(phy[i],"adi,rx-settings-rx-pll-lo-frequency_hz", &val);
	R_DBG_LLI(phy[i],"adi,tx-settings-tx-pll-lo-frequency_hz", &val);
	R_DBG_LLI(phy[i],"adi,rx-settings-rx-pll-lo-frequency_hz", &val);
	R_DBG_LLI(phy[i],"adi,tx-settings-tx1-atten_mdb", &val);
	R_DBG_LLI(phy[i],"adi,tx-settings-tx2-atten_mdb", &val);
	R_DBG_LLI(phy[i],"adi,tx-profile-iq-rate_khz", &val);
	R_DBG_LLI(phy[i],"adi,rx-profile-iq-rate_khz", &val);
}


#if 0
#include "t_mykonos.h"
#include "t_mykonos_gpio.h"
#include "ad9371profile20mhz.c"
extern mykonosDevice_t mykDevice;
#else
#include "talise_types.h"
#include "talise_config.h"
#include "talise_error.h"
#include "ad9009profile100mhz.h"
#include "ad9xxxcommon.h"
extern taliseInit_t talInit;
#endif
static void profile_phy_array(struct iio_device *phy)
{
#if 0
    mykonosTxSettings_t *tx = mykDevice.tx;
    W_DBG_LLI(phy, "adi,tx-settings-tx-channels-enable",	tx->txChannels);
    W_DBG_LLI(phy, "adi,tx-settings-tx-pll-use-external-lo", tx->txPllUseExternalLo);
    W_DBG_LLI(phy, "adi,tx-settings-tx-pll-lo-frequency_hz", tx->txPllLoFrequency_Hz);
    W_DBG_LLI(phy, "adi,adi,tx-settings-tx-atten-step-size", tx->txAttenStepSize);
    W_DBG_LLI(phy, "adi,tx-settings-tx2-atten_mdb", tx->tx2Atten_mdB);
    W_DBG_LLI(phy, "adi,tx-settings-tx1-atten_mdb", tx->tx1Atten_mdB);

    mykonosTxProfile_t *tp = mykDevice.tx->txProfile;
    W_DBG_LLI(phy, "adi,tx-profile-tx-bbf-3db-corner_khz", tp->txBbf3dBCorner_kHz);
    W_DBG_LLI(phy, "adi,tx-profile-tx-dac-3db-corner_khz", tp->txDac3dBCorner_kHz);
    W_DBG_LLI(phy, "adi,tx-profile-rf-bandwidth_hz", tp->rfBandwidth_Hz);
    W_DBG_LLI(phy, "adi,tx-profile-primary-sig-bandwidth_hz", tp->primarySigBandwidth_Hz);
    W_DBG_LLI(phy, "adi,tx-profile-iq-rate_khz", tp->iqRate_kHz);
    W_DBG_LLI(phy, "adi,tx-profile-tx-input-hb-interpolation", tp->txInputHbInterpolation);
    W_DBG_LLI(phy, "adi,tx-profile-thb2-interpolation", tp->thb1Interpolation);
    W_DBG_LLI(phy, "adi,tx-profile-thb1-interpolation", tp->thb2Interpolation);
    W_DBG_LLI(phy, "adi,tx-profile-tx-fir-interpolation", tp->txFirInterpolation);
    W_DBG_LLI(phy, "adi,tx-profile-dac-div", tp->dacDiv);

	mykonosRxSettings_t *rx = mykDevice.rx;
    W_DBG_LLI(phy, "adi,rx-settings-rx-pll-lo-frequency_hz", rx->rxPllLoFrequency_Hz);
    W_DBG_LLI(phy, "adi,rx-settings-rx-pll-use-external-lo", rx->rxPllUseExternalLo);
    W_DBG_LLI(phy, "adi,rx-settings-rx-channels-enable", rx->rxChannels);
    W_DBG_LLI(phy, "adi,rx-settings-real-if-data", rx->realIfData);

    mykonosRxProfile_t *rp = mykDevice.rx->rxProfile;
    W_DBG_LLI(phy, "adi,rx-profile-rx-bbf-3db-corner_khz", rp->rxBbf3dBCorner_kHz);
    W_DBG_LLI(phy, "adi,rx-profile-rf-bandwidth_hz", rp->rfBandwidth_Hz);
    W_DBG_LLI(phy, "adi,rx-profile-iq-rate_khz", rp->iqRate_kHz);
    W_DBG_LLI(phy, "adi,rx-profile-rhb1-decimation", rp->rhb1Decimation);
    W_DBG_LLI(phy, "adi,rx-profile-en-high-rej-dec5", rp->enHighRejDec5);
    W_DBG_LLI(phy, "adi,rx-profile-rx-dec5-decimation", rp->rxDec5Decimation);
    W_DBG_LLI(phy, "adi,rx-profile-rx-fir-decimation", rp->rxFirDecimation);
#else
    taliseTxSettings_t *tx = talInit.tx;

    W_DBG_LLI(phy, "adi,tx-settings-tx-channels-enable", tx->txChannels);
    W_DBG_LLI(phy, "adi,tx-settings-tx-data-options-if-pll-unlock", tx->disTxDataIfPllUnlock);
    //W_DBG_LLI(phy, "adi,tx-settings-tx-pll-lo-frequency_hz", tx->txPllLoFrequency_Hz);
    W_DBG_LLI(phy, "adi,adi,tx-settings-deframer-sel", tx->deframerSel); 
    W_DBG_LLI(phy, "adi,adi,tx-settings-tx-atten-step-size", tx->txAttenStepSize);
    W_DBG_LLI(phy, "adi,tx-settings-tx2-atten_mdb", tx->tx2Atten_mdB);
    W_DBG_LLI(phy, "adi,tx-settings-tx1-atten_mdb", tx->tx1Atten_mdB);

    taliseTxProfile_t *tp = tx->txProfile;
    W_DBG_LLI(phy, "adi,tx-profile-tx-bbf-3db-corner_khz", tp->txBbf3dBCorner_kHz);
    W_DBG_LLI(phy, "adi,tx-profile-tx-dac-3db-corner_khz", tp->txDac3dBCorner_kHz);
    W_DBG_LLI(phy, "adi,tx-profile-rf-bandwidth_hz", tp->rfBandwidth_Hz);
    W_DBG_LLI(phy, "adi,tx-profile-primary-sig-bandwidth_hz", tp->primarySigBandwidth_Hz);
    W_DBG_LLI(phy, "adi,tx-profile-tx-input-rate_khz", tp->txInputRate_kHz);
    W_DBG_LLI(phy, "adi,tx-profile-tx-int5-interpolation", tp->txInt5Interpolation);
    W_DBG_LLI(phy, "adi,tx-profile-thb1-interpolation", tp->thb1Interpolation);
    W_DBG_LLI(phy, "adi,tx-profile-thb2-interpolation", tp->thb2Interpolation);
    W_DBG_LLI(phy, "adi,tx-profile-thb3-interpolation", tp->thb3Interpolation);
    W_DBG_LLI(phy, "adi,tx-profile-tx-fir-interpolation", tp->txFirInterpolation); 
    W_DBG_LLI(phy, "adi,tx-profile-dac-div", tp->dacDiv);

    taliseRxSettings_t *rxs = talInit.rx;
    //W_DBG_LLI(phy, "adi,rx-settings-rx-pll-lo-frequency_hz", rx->rxPllLoFrequency_Hz);
    //W_DBG_LLI(phy, "adi,rx-settings-rx-pll-use-external-lo", rx->rxPllUseExternalLo);
    W_DBG_LLI(phy, "adi,rx-settings-rx-channels-enable", rx->rxChannels);
    //W_DBG_LLI(phy, "adi,rx-settings-real-if-data", rx->realIfData);
    W_DBG_LLI(phy, "adi,adi,rx-settings-framer-sel", rx->framerSel);

    taliseRxProfile_t *rp = &rx->rxProfile;
    W_DBG_LLI(phy, "adi,rx-profile-rx-bbf-3db-corner_khz", rp->rxBbf3dBCorner_kHz);
    W_DBG_LLI(phy, "adi,rx-profile-rf-bandwidth_hz", rp->rfBandwidth_Hz);
    W_DBG_LLI(phy, "adi,rx-profile-rx-output-rate_khz", rp->rxOutputRate_kHz);
    W_DBG_LLI(phy, "adi,rx-profile-rhb1-decimation", rp->rhb1Decimation);
    W_DBG_LLI(phy, "adi,rx-profile-rx-dec5-decimation", rp->rxDec5Decimation);
    W_DBG_LLI(phy, "adi,rx-profile-rx-fir-decimation", rp->rxFirDecimation);
#endif
    printf("%s finished\r\n", __func__);
}

static void profile_phy_file(struct iio_context *ctx, struct iio_device *phy, const char *fn)
{
	int ret;
	FILE *fd;
	char *buf;
	size_t len;

    printf("Load profile from %s\r\n", fn);
    FILE *fd = fopen(fn, "r");
    if (!fd) {
        fprintf(stderr, "Cannot open the profile file\r\n");
	 	return;
    }

	fseek(fd, 0, SEEK_END);
    len = ftell(fd);
	//printf("the profile's length is:%d\r\n", (int)len);
    buf = malloc(len);
	if (NULL == buf) {
		perror("malloc");
		exit(-1);
	}
	fseek(fd, 0, SEEK_SET);
    ret = fread(buf, 1, len, fd);
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
	printf("Profile write start\r\n");
	iio_device_attr_write_raw(phy, "profile_config", buf, len);
	printf("Profile write   end\r\n");
    iio_context_set_timeout(ctx, 3000);
	free(buf);
}

int init_phy(int ig)
{
	int ip;
	struct iio_channel *chn;

    if (g_m_verbo > 2) printf("'PHY%d' acquiring ... ", ig);
	rfdev[ig] = get_phy_dev(ctx, ig);
	ASSERT((NULL != rfdev[ig]) && "No devices");
    if (g_m_verbo > 2) printf("succeed\r\n");

    if (g_m_verbo > 2) printf("'PHY%d' profiling ... ", ig);
    profile_phy_array(rfdev[ig]);
    if (g_m_verbo > 2) printf("succeed\r\n");

	if (g_pth_msk & EN_IIO_PATH_RX) {
		if (g_m_verbo > 2) printf("'PHY%d' Init RX i/q channels of port%d ... ", ig, ip);
		for (ip=0; ip<DEV_PORT_NB; ip++) {
			chn = get_phy_chn_rf(rfdev[ig], RX, ip);
			ASSERT((NULL != chn) && "'PHY': No RF-RX channel found");
			cfg_phy_chn_rf(chn, RX, &rxchn_rfcfg);
			chn = get_phy_chn_lo(rfdev[ig], RX);
			ASSERT((NULL != chn) && "'PHY': No LO-RX channel found");
			cfg_phy_chn_lo(chn, RX, &rxchn_rfcfg);
		}
		if (g_m_verbo > 2) printf("succeed\r\n");
	}

	if ((g_pth_msk & EN_IIO_PATH_TX)) {
		if (g_m_verbo > 2) printf("'PHY%d' Init TX i/q channels of port%d ... ", ig, ip);
		for (ip=0; ip<DEV_PORT_NB; ip++) {
			chn = get_phy_chn_rf(rfdev[ig], TX, ip);
			ASSERT((NULL != chn) && "'PHY': No RF-TX channel found");
			cfg_phy_chn_rf(chn, TX, &txchn_rfcfg);
			chn = get_phy_chn_lo(rfdev[ig], TX);
			ASSERT((NULL != chn) && "'PHY': No LO-TX channel found");
			cfg_phy_chn_lo(chn, TX, &txchn_rfcfg);
		}
		if (g_m_verbo > 2) printf("succeed\r\n");
	}
}

int main(int argc, char *argv[])
{
	int ret;
	int ig;

    if (NULL == ctx) {
		ctx = iio_create_local_context();
    	ASSERT((NULL != ctx) && "context-local failed");

		ret = iio_context_get_devices_count(ctx);
    	ASSERT((ret > 0) && "No iio-devices found");
    }

	get_args(argc, argv);

    //signal(SIGINT, handle_sig);

	for (ig=0; ig<TOT_CHIP_NB; ig++) {
		init_phy(ig);
	}

	shutdown();
	return 0;
}

