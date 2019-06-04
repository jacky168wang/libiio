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
enum iodev { RX, TX, OR };

/* static scratch mem for strings */
static char tmpstr[64];
/* IIO structs required for streaming */
static struct iio_context *ctx;
struct iio_device *phy[TOT_CHIP_NB];

static unsigned int mode_verbo;

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION]\n"
        " -a\tset Tx attenuation\n"
        " -f\tset carrier frequency\n"
        " -g\tset Rx Gain\n"
		" -P/-p\tTxRx cfg profile"
		" -v\tbe verbose [0,1,2,3]\n"
		" -l\tenable JESD204B loopback mode\n"
		" -c\tDISABLE AD9371 tracking calibration\n"
		" -h\tshow this help\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
static void get_args(int argc, char **argv)
{
	int c;
	opterr = 0;

	while ((c = getopt(argc, argv, "a:f:g:l:p:v:h")) != EOF) {
		switch (c) {
	    case 'a': cfg->tx_gain = strtoul(optarg, NULL,  0);		break;
	    case 'f': cfg->rx_freq = GHZ(strtoul(optarg, NULL, 0));
				  cfg->tx_freq = GHZ(strtoul(optarg, NULL, 0));	break;
	    case 'g': cfg->rx_gain = strtoul(optarg, NULL,  0);		break;
	    case 'l': cfg->loop = 1;								break;
	    case 'p':
	        memset(profilename, 0x00, tmpsize);
	        tmpsize -= 1;
	        if(tmpsize > strlen(optarg))
	            tmpsize = strlen(optarg);
	        memcpy(profilename, optarg, tmpsize);
	        break;
	    case 'v': mode_verbo = strtoul(optarg, NULL,  0);		break;
	    case 'h': usage(); exit(EXIT_FAILURE);					break;
		case '?':
			if (isprint(optopt)) {
				fprintf(stderr, "ERROR: unrecognised option \"%c\"\n", optopt);
				exit(EXIT_FAILURE);
			}
			break;
		default:
			fprintf(stderr, "ERROR: unrecognised command line option\n");
			exit(EXIT_FAILURE);
			break;
		}
	}

	printf( "CURRENT SETTINGS:\n" );
	printf( "\tTODO:   %u\n");
}

/* cleanup and exit */
static void shutdown(void)
{
	if (mode_verbo > 0)
		printf("* Destroying context\n");
	if (NULL != ctx) {
		iio_context_destroy(ctx);
		ctx = NULL;
	}
	exit(0);
}

static volatile sig_atomic_t process_stop = 0;
static void handle_sig(int sig)
{
	printf("\nWaiting for process to finish...\n");
	process_stop = 1;
}

/* helper function generating channel names */
static void chnname(const char* type, int id, char modify)
{
	if ('\0' = modify)
		snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	else
		snprintf(tmpstr, sizeof(tmpstr), "%s%d_%c", type, id, modify);
}

/* helper function to check return value of attr_write function */
/* int chn, const char* type_string, const char* what, type_key value */
#define IIO_ATTR(A, C, T, W, V) \
do {\
	int ret;\
	ret = iio_channel_attr_##A_##T(C, W, V);\
	if (ret < 0) {\
		fprintf(stderr, "Error %d "#A" to channel \"%s\"\n", W);\
		abort();\
	}\
} while (0);

static struct iio_device *get_iio_dev(struct iio_context *ctx, enum iodev d)
{
	struct iio_device *dev;
	char devname[][SLEN_MAX] = {
		"afe-phy",
		"jesd204tx-layer3",
		"jesd204rx-layer3",
		"jesd204or-layer3"
	};
	dev = iio_context_find_device(ctx, devname[d][SLEN_MAX]);
	ASSERT((NULL != dev) && "the specified iio device not found");
	return dev;
}

/* finds streaming IIO channels */
static struct iio_channel *get_jesd_chn_ss(struct iio_device *dev, enum iodev d, int ich, char modify)
{
	struct iio_channel *chn;

	if (mode_verbo > 0)
		printf("* Acquiring device 'JESD' SS-channel '%s%d'\n",
			d == TX ? "TX" : "RX", ich);
	//chn = iio_device_find_channel(dev, chnname("voltage", ich, '\0'), d == TX);
	chn = iio_device_find_channel(dev, chnname("voltage", ich, modify), d == TX);
	ASSERT((NULL != chn) && "No SS-channel found");
	return chn;
}

/* finds phy IIO configuration channel with id ich */
static struct iio_channel *get_phy_chn_rf(struct iio_device *phy, enum iodev d, int ich)
{
	struct iio_channel *chn;
	chn = iio_device_find_channel(phy, chnname("voltage", ich), d == TX);
	ASSERT((NULL != chn) && "No RF-channel found");
	return chn;
}

// LO chan is always output, i.e. true
static struct iio_channel *get_phy_chn_lo(struct iio_device *phy, enum iodev d)
{
	struct iio_channel *chn;

	if (mode_verbo > 0)
		printf("* Acquiring device 'PHY' lo-channel '%s'\n",
			d == TX ? "TX" : "RX");

	chn = iio_device_find_channel(phy, chnname("altvoltage", TX == d), true);

	ASSERT((NULL != chn) && "No LO-channel found");
	return chn;
}

static bool set_phy_chn_lo(enum iodev d, struct iio_channel *chn, struct stream_cfg *cfg)
{
	switch (d) {
	case RX:
#if 0
		IIO_ATTR(chn, "RX_LO_frequency", write, longlong, cfg->lo_hz);
#else
		IIO_ATTR(chn, "out_altvoltage0", write, longlong, cfg->lo_hz);
#endif
		break;
	case TX:
#if 0
		IIO_ATTR(chn, "TX_LO_frequency", write, longlong, cfg->lo_hz);
#else
		IIO_ATTR(chn, "out_altvoltage0", write, longlong, cfg->lo_hz);
#endif
		break;
	default:
		ASSERT(0);
	}

	return true;
}

/* applies streaming configuration through IIO */
/* ad9371-phy(iio:device1) */
	/* PHY_RX[0/1/2/3](iio:device1/ in_voltage[n]_xxx */
	/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
static bool cfg_phy_chn_ss(enum iodev d, struct iio_channel *chn, struct stream_cfg *cfg)
{
	int ret;
	int iprt;
	struct iio_device *dev;
	struct iio_channel *chn;

	dev = get_iio_dev(ctx, PHY);

	chn = get_phy_chn_rf(dev, d, ich);

	switch (d) {
	case RX:
		IIO_ATTR(chn, "rf_bandwidth", write, longlong, cfg->bw_hz);
		IIO_ATTR(chn, "sampling_frequency", write, longlong, cfg->fs_hz);
		IIO_ATTR(chn, "hardwaregain", write, double, cfg->gain_or_atten);
		IIO_ATTR(chn, "quadrature_tracking_en", write, bool, cfg->qtracking);
		break;
	case TX:
		IIO_ATTR(chn, "rf_bandwidth", write, longlong, cfg->bw_hz);
		IIO_ATTR(chn, "sampling_frequency", write, longlong, cfg->fs_hz);
		IIO_ATTR(chn, "hardwaregain", write, double, cfg->gain_or_atten);
		IIO_ATTR(chn, "lo_leakage_tracking_en", write, bool, cfg->qtracking);
		break;
	default:
		ASSERT(0);
		return false;
	}

	/* configure local oscillator channels */
#if 0
	if (mode_verbo > 0)
		printf("* Acquiring device 'PHY' lo-channel '%s'\n", type ? "TX" : "RX");
	chn = iio_device_find_channel(dev, chnname("altvoltage", ich), false);
	ASSERT((NULL != chn) && "No channel 'RX-LO' found");
	IIO_ATTR(chn, "RX_LO_frequency", write, longlong, rxcfg.lo_hz);
	chn = iio_device_find_channel(dev, chnname("altvoltage", ich), true);
	ASSERT((NULL != chn) && "No channel 'TX-LO' found");
	IIO_ATTR(chn, "TX_LO_frequency", write, longlong, txcfg.lo_hz);
#else
	chn = iio_device_find_channel(dev, chnname("altvoltage", ich), true);
	IIO_ATTR(chn, "out_altvoltage0", write, longlong, rxcfg.lo_hz);
#endif
}

/* power down channel*/
static bool set_phy_chn_power(struct iio_device *phy, int ich, bool up)
{
    bool powerdown;
    struct iio_channel *chn = NULL;

    if (!get_phy_chn_rf(phy, 0, ich, &chn)) {
        printf("* Power down Rx channel %d failed for get_phy_chn_rf\n", ich);
        return false;
    }

    printf("* Power down Rx channel %d\n", ich);
    iio_channel_attr_write_bool(chn, "powerdown", !up);
    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    printf("powerdown status is %d\n", powerdown);

    return true;
}

static bool get_phy_chn_power(struct iio_device *phy, int ich)
{
    bool powerdown;
    struct iio_channel *chn = NULL;

    if (!get_phy_chn_rf(phy, 0, ich, &chn)) {
        printf("* Power down Rx channel %d failed for get_phy_chn_rf\n", ich);
        return true;
    }

    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    return !powerdown;
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
#else
    taliseTxSettings_t *tx = talInit.tx;

    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-channels-enable", tx->txChannels);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-data-options-if-pll-unlock", tx->disTxDataIfPllUnlock);
    //iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-pll-lo-frequency_hz", tx->txPllLoFrequency_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,adi,tx-settings-deframer-sel", tx->deframerSel); 
    iio_device_debug_attr_write_longlong(phy, "adi,adi,tx-settings-tx-atten-step-size", tx->txAttenStepSize);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx2-atten_mdb", tx->tx2Atten_mdB);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx1-atten_mdb", tx->tx1Atten_mdB);

    taliseTxProfile_t *tp = tx->txProfile;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-bbf-3db-corner_khz", tp->txBbf3dBCorner_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-dac-3db-corner_khz", tp->txDac3dBCorner_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-rf-bandwidth_hz", tp->rfBandwidth_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-primary-sig-bandwidth_hz", tp->primarySigBandwidth_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-input-rate_khz", tp->txInputRate_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-int5-interpolation", tp->txInt5Interpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb1-interpolation", tp->thb1Interpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb2-interpolation", tp->thb2Interpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb3-interpolation", tp->thb3Interpolation);
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-fir-interpolation", tp->txFirInterpolation); 
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-dac-div", tp->dacDiv);

    taliseRxSettings_t *rxs = talInit.rx;
    //iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-lo-frequency_hz", rx->rxPllLoFrequency_Hz);
    //iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-use-external-lo", rx->rxPllUseExternalLo);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-channels-enable", rx->rxChannels);
    //iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-real-if-data", rx->realIfData);
    iio_device_debug_attr_write_longlong(phy, "adi,adi,rx-settings-framer-sel", rx->framerSel);

    taliseRxProfile_t *rp = &rx->rxProfile;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-bbf-3db-corner_khz", rp->rxBbf3dBCorner_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rf-bandwidth_hz", rp->rfBandwidth_Hz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-output-rate_khz", rp->rxOutputRate_kHz);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rhb1-decimation", rp->rhb1Decimation);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-dec5-decimation", rp->rxDec5Decimation);
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-fir-decimation", rp->rxFirDecimation);
#endif
    printf("%s finished\r\n", __func__);
}

static void profile_phy_file(struct iio_context *ctx, struct iio_device *phy, const char *fn)
{
    printf("Load profile from %s\n", fn);
    FILE *fd = fopen(fn, "r");
    if (!fd) {
        fprintf(stderr, "Cannot open the profile file\n");
	 	return;
    }

	fseek(fd, 0, SEEK_END);
    size_t len = ftell(fd);
	//printf("the profile's length is:%d\n", (int)len);
    char *buf = malloc(len);
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
	printf("Profile write start\n");
	iio_device_attr_write_raw(phy, "profile_config", buf, len);
	printf("Profile write   end\n");
    iio_context_set_timeout(ctx, 3000);
	free(buf);
}

void main(int argc, char *argv[])
{
	int ret, igrp, iprt;
	struct iio_device *phy;

    if (NULL == ctx) {
        if (mode_verbo > 0) {
            printf("* Acquiring IIO context\n");
        }
		ctx = iio_create_local_context();
    	ASSERT((NULL != ctx) && "No context");
		ret = iio_context_get_devices_count(ctx);
    	ASSERT((ret > 0) && "No devices");
    }

	for (igrp=0; igrp<TOT_CHIP_NB; igrp++) {
        if (mode_verbo > 0) {
            printf("LIBIIO_APP(group%d): Acquiring PHY device\n", igrp);
        }
        phy = get_iio_dev(ctx, "adrv9371-phy");
		ASSERT((ret > 0) && "No devices");
        if (mode_verbo > 0) {
            printf("LIBIIO_APP(group%d): Profiling PHY device\n", igrp);
        }
        profile_phy_array(phy);

		/* ad9371-phy(iio:device1) */
			/* PHY_RX[0/1/2/3](iio:device1/in_voltage[n]_xxx */
			/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
		for (iprt=0; iprt<DEV_PORT_NB; iprt++) {
			if (mode_verbo > 0) {
				printf("AD9371_RX(group%d): Initializing PHY port%d\n",
					igrp, iprt);
			}
			ret = cfg_phy_chn_stream(ctx, &rxcfg, RX, iprt);
			ASSERT(ret && "No phy_rxport found");

			if (mode_verbo > 0) {
				printf("AD9371_TX(group%d): Initializing PHY port%d\n",
					igrp, iprt);
			}
			ret = cfg_phy_chn_stream(ctx, &txcfg, TX, iprt);
			ASSERT(ret && "No phy_txport found");
		}
	}
}

