/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
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

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

/* RX is input, TX is output */
enum iodev { RX, TX, OR };

/* static scratch mem for strings */
#define SLEN_MAX 64
static char tmpstr[SLEN_MAX];

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

/* check return value of attr_write function */
static void errchk(int v, const char* what)
{
	if (v < 0) {
		fprintf(stderr, "Error %d writing to channel \"%s\"" \
			"\nvalue may not be supported.\n", v, what);
		exit(0);
	}
}

/* helper function generating channel names */
static char* chnname_mod(const char* type, int id, char modify)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d_%c", type, id, modify);
	return tmpstr;
}

static char* chnname(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

static void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

static long long rd_ch_lli(struct iio_channel *chn, const char* what)
{
	long long val;

	errchk(iio_channel_attr_read_longlong(chn, what, &val), what);
	printf("\t %s: %lld\n", what, val);
	return val;
}

static struct iio_device *get_phy_dev(struct iio_context *ctx, int idev)
{
	struct iio_device *dev;
	char devname[][SLEN_MAX] = {
#if 0
        "ad9371-phy",
        "ad9371-phy-n2",
#else
		"adrv9009-phy",
		"adrv9009-phy-n2",
#endif
#if 0
		"jesd204tx-layer3",
		"jesd204rx-layer3",
		"jesd204or-layer3"
#endif
	};

	dev = iio_context_find_device(ctx, devname[idev][SLEN_MAX]);
	ASSERT((NULL != dev) && "the specified iio device not found");
	return dev;
}

/* finds streaming IIO channels */
static struct iio_channel *get_jesd_chn_ss(struct iio_device *dev, enum iodev d, int ich, char modify)
{
	struct iio_channel *chn;

	*chn = iio_device_find_channel(dev, \
		modify ? chnname_mod("voltage", ich, modify) : \
			chnname("voltage", ich), \
		d == TX);

	if (!*chn) {
		*chn = iio_device_find_channel(dev, \
			modify ? chnname_mod("voltage", ich, modify) : \
				chnname("voltage", ich), \
			d == TX);
	}

	return *chn != NULL;
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

	switch (d) {
	case RX:
		chn = iio_device_find_channel(phy, chnname("altvoltage", 0), true);
	case TX:
		chn = iio_device_find_channel(phy, chnname("altvoltage", 1), true);
	default:
		ASSERT(0);
	}

	ASSERT((NULL != chn) && "No LO-channel found");
	return chn;
}

/* ad9371-phy(iio:device1) */
	/* PHY_RX[0/1/2/3](iio:device1/ in_voltage[n]_xxx */
	/* PHY_TX[0/1/2/3](iio:device1/out_voltage[n]_xxx */
static bool cfg_phy_chn_ss(struct iio_device *phy, int ich, enum iodev d, struct phy_cfg *cfg)
{
    long long val;
    double gain;
    struct iio_channel *chn;

    switch (d) {
	default:
	case TX:
        printf("* Acquiring TX-RF channel %d\n", ich);
        chn = get_phy_chn_rf(phy, d, ich);
        iio_channel_attr_write_bool(chn, "quadrature_tracking_en", cfg->tx_qtracking);
        iio_channel_attr_write_bool(chn, "lo_leakage_tracking_en", cfg->tx_loltracking);

        iio_channel_attr_write_double(chn, "hardwaregain", cfg->tx_gain);
        iio_channel_attr_read_double(chn, "hardwaregain", &gain);
        printf("Channel %s Tx attenuation is %f\n", iio_channel_get_name(chn), gain);

        printf("* Acquiring TX-LO channel\n");
        chn = get_phy_chn_lo(phy, TX);
#if 0
        iio_channel_attr_write_longlong(chn, "TX_LO_frequency", cfg->tx_freq);
        iio_channel_attr_read_longlong (chn, "TX_LO_frequency", &val);
#else
        iio_channel_attr_write_longlong(chn, "frequency", cfg->tx_freq);
        iio_channel_attr_read_longlong (chn, "frequency", &val);
#endif
        printf("TX-LO frequency is %lld\n",val);
	   	break;

	case ORX:
        printf("* Acquiring OR-LO channel\n");
        chn = get_phy_chn_rf(phy, d, ich);

		set_phy_chn_power(phy, ich, true);

		iio_channel_attr_write_double(chn, "hardwaregain", cfg->rx_gain);
		iio_channel_attr_read_double (chn, "hardwaregain", &gain);
		printf("Channel %s ORx gain is %f\n", iio_channel_get_name(chn), gain);

        chn = iio_device_find_channel(phy, chnname("altvoltage", 2), true);
#if 0
        iio_channel_attr_write_longlong(chn, "RX_SN_LO_frequency", cfg->tx_freq);
        iio_channel_attr_read_longlong (chn, "RX_SN_LO_frequency", &val);
#else
		iio_channel_attr_write_longlong(chn, "frequency", cfg->tx_freq);
		iio_channel_attr_read_longlong(chn, "frequency", &val);
#endif
        printf("OR-LO frequency is %lld\n", val);
		break;

	case RX:
        printf("* Acquiring RX-RF channel %d\n", ich);
        chn = get_phy_chn_rf(phy, d, ich);
        iio_channel_attr_write_bool(chn, "quadrature_tracking_en", cfg->rx_qtracking);

        iio_channel_attr_write_double(chn, "hardwaregain", cfg->rx_gain);
        iio_channel_attr_read_double (chn, "hardwaregain", &gain);
        printf("Channel %s Rx gain is %f\n", iio_channel_get_name(chn), gain);

        printf("* Acquiring RX-LO channel\n");
        chn = get_phy_chn_lo(phy, RX);
#if 0
        iio_channel_attr_write_longlong(chn, "RX_LO_frequency", cfg->rx_freq);
        iio_channel_attr_read_longlong (chn, "RX_LO_frequency", &val);
#else
        iio_channel_attr_write_longlong(chn, "frequency", cfg->rx_freq);
        iio_channel_attr_read_longlong (chn, "frequency", &val);
#endif
        printf("RX-LO frequency is %lld\n", val);
		break;
    }

	return true;
}

static void show_phy_chn_ss(struct iio_device *phy)
{
    long long val;

	iio_device_debug_attr_write_longlong(phy[i],"adi,tx-settings-tx-pll-lo-frequency_hz", cfg->tx_freq);
	iio_device_debug_attr_write_longlong(phy[i],"adi,rx-settings-rx-pll-lo-frequency_hz", cfg->rx_freq);
	iio_device_debug_attr_read_longlong (phy[i],"adi,tx-settings-tx-pll-lo-frequency_hz", &tempvar1);
	iio_device_debug_attr_read_longlong (phy[i],"adi,rx-settings-rx-pll-lo-frequency_hz", &val);
	printf("Tx frequency is %lld, Rx frequency is %lld\r\n",tempvar1, val);
	
	iio_device_debug_attr_read_longlong (phy[i],"adi,tx-settings-tx1-atten_mdb", &tempvar1);
	iio_device_debug_attr_read_longlong (phy[i],"adi,tx-settings-tx2-atten_mdb", &val);
	printf("tx1_atten is %lld, tx2_atten is %lld\r\n",tempvar1, val);
	
	iio_device_debug_attr_read_longlong (phy[i],"adi,tx-profile-iq-rate_khz", &tempvar1);
	iio_device_debug_attr_read_longlong (phy[i],"adi,rx-profile-iq-rate_khz", &val);
	printf("tx sample rate is %lld, rx sample rate is %lld\r\n",tempvar1, val);

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


static void phy_init_profilead9xxx(struct iio_context *ctx, char *profilename)
{
    unsigned char i;
    char devicead9009name[][32]= {
        "adrv9009-phy"
    };
    char devicead9371name[][32]= {
        "ad9371-phy",
        "ad9371-phy-n2"
    };

    if (ctx == NULL) {
        printf("phy_init_profilead9xxx failed for ctx NULL\r\n");
        return;
    }

	/* try ad9371 */
    for (i=0; i<(sizeof(devicead9371name)/sizeof(devicead9371name[0])); i++) {
        phy[i] = get_phy_dev(ctx, i);
        if (profilename != NULL && strlen(profilename))
            profile_phy_file(ctx, phy[i], profilename);
        else
            profile_phy_array(phy[i]);
    }

    if (i != 0) {
        return;
    }

    for (i=0; i<sizeof(phy)/sizeof(phy[0]); i++) {
        phy[i] = NULL;
    }

    /* AD9009 */
    for (i=0; i<(sizeof(devicead9009name)/sizeof(devicead9009name[0])); i++) {
        phy[i] = get_phy_dev(ctx, i);
        if (profilename != NULL && strlen(profilename))
            profile_phy_file(ctx, phy[i], profilename);
        else
            load_ad9009_profile(phy[i], &tal);
    }
}


/* Configure AD9XXXX*/
static void phy_init_cfgad9xxx(struct iio_context *ctx, struct phy_cfg* cfg)
{
    int i;
    bool val = 0; 
    long long tempvar1;
    long long tempvar2;
    char ifad9371 = -1; /*not ad9371 or ad9009*/
    struct iio_device *phy[4] = {NULL,NULL,NULL,NULL};
    struct iio_channel *obschn[4] = {NULL,NULL,NULL,NULL};
    char devicead9009name[][32]= {
        "adrv9009-phy"
    };

    char devicead9371name[][32]={
        "ad9371-phy",
        "ad9371-phy-n2"
    };

    if (ctx == NULL || cfg == NULL) {
        printf("phy_init_cfgad9371 failed for ctx NULL or cfg NULL\r\n");
        return;
    }

    /* AD9371 */
    for (i=0; i<(sizeof(devicead9371name)/sizeof(devicead9371name[0])); i++) {
		phy[i] = get_phy_dev(ctx, i);
        ifad9371 = 1;

        cfg_phy_chn_ss(phy[i], 0, 0, cfg);
        cfg_phy_chn_ss(phy[i], 1, 0, cfg);
        cfg_phy_chn_ss(phy[i], 0, 1, cfg);
        cfg_phy_chn_ss(phy[i], 1, 1, cfg);

        /*move from stream*/
        get_phy_chn_rf(phy[i], RX, 2, &obschn[i]);
        iio_channel_attr_write(obschn[i], "rf_port_select", "INTERNALCALS");
        iio_device_debug_attr_write_bool(phy[i],"loopback_tx_rx", cfg->loop);
        iio_device_debug_attr_read_bool (phy[i],"loopback_tx_rx", &val);
        LOG_PRINT_L1(mode_verbo,printf("Tx_RX loopback is %d\r\n",val));
        /*move from stream end*/
    }
    /* if chip is AD9371,then end of process AD9371, else find AD9009 */
    if (ifad9371 == 1) {
        return;
    }

    for (i=0; i<sizeof(phy)/sizeof(phy[0]); i++) {
        phy[i] = NULL;
    }
    /* AD9009 */
    for (i=0; i<(sizeof(devicead9009name)/sizeof(devicead9009name[0])); i++) {
        phy[i] = get_phy_dev(ctx, i);
        get_phy_chn_rf(phy[i], RX, 2, &obschn[i]);
        ifad9371 = 0;
		cfg_phy_chn_ss(phy[i], cfg, 0, 0);
        cfg_phy_chn_ss(phy[i], cfg, 1, 0);
        cfg_phy_chn_ss(phy[i], cfg, 0, 1);
        cfg_phy_chn_ss(phy[i], cfg, 1, 1);
    }
}

/*for extern used*/
int phy_init_chgsetting(struct iio_context *ctx, int argc, char **argv)
{
    int ret = 0;
    char profilename[SLEN_MAX];
    struct iio_context *tmpctx = ctx;

    memset(profilename, 0x00, sizeof(profilename));

    if (tmpctx == NULL) {
        tmpctx = iio_create_local_context();
        if (tmpctx == NULL) {
            printf("phy_init_chgsetting failed for no context\r\n");
            return -1;
        }
    }

    if (iio_context_get_devices_count(tmpctx) == 0) {
        printf("phy_init_chgsetting failed for no devices\r\n");
        return -1;
    }

    //parse input command
    ret = get_args(argc, argv, &cfg, profilename, SLEN_MAX);
    if (ret != 0) return ret;

    phy_init_cfgad9xxx(tmpctx,&cfg);

    if (ctx == NULL && tmpctx != NULL) {
        iio_context_destroy(tmpctx);
        tmpctx = NULL;
    }

    return 0;
}

int main(int argc, char **argv)
{
    char profilename[SLEN_MAX];

    memset(profilename, 0x00, sizeof(profilename));

    if (mode_verbo < 2)
    	printf("* Acquiring IIO context\n");

    ctx = iio_create_local_context();
    if (ctx == NULL) { 
		printf("No context\r\n");
		return -1;
	}

    if (mode_verbo < 2)
		printf("Get %d devices.\n", iio_context_get_devices_count(ctx));
    if (0 == iio_context_get_devices_count(ctx)) {
        printf("No devices\r\n");
        return -1;
    }

    signal(SIGINT, handle_sig);

    ret = get_args(argc, argv, &cfg, profilename, SLEN_MAX);
    if (ret != 0) return -1;

    phy_init_profilead9xxx(ctx, profilename);

    phy_init_cfgad9xxx(ctx, &cfg);

    if (ctx != NULL) {
        iio_context_destroy(ctx);
        ctx = NULL;
    }

    return 0;
}

