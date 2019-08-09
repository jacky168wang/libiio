/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
 *
 * License: GPL, version 2.1
 */
#include <signal.h>
#include <stdio.h>
#include <ctype.h>

#include <fcntl.h>

#include <sys/types.h>
#include <stdlib.h>
#include "ad9371profile20mhz.h"
#include "ad9009profile100mhz.h"
#include "ad9xxxcommon.h"

extern unsigned int mode_verbo;

/* IIO structs required for streaming */
struct iio_context *phy_init_ctx = NULL;

static void phy_init_usage(void)
{
	fprintf( stderr,
        "phy_init_usage : ./phy_init [OPTION]\n"
        " -a\tset Tx attenuation\n"
        " -f\tset carrier frequency\n"
        " -g\tset Rx Gain\n"
		" -P/-p\tTxRx cfg profile\n"
		" -v\tbe verbose [0,1,2,3]\n"
		" -l\tenable JESD204B loopback mode\n"
		" -c\tRecycle to send or receive\n"
		" -h\tshow this help\n"
		);
}

static void phy_init_handle_sig(int sig)
{
	printf("Waiting for process to finish...\n");
	if (phy_init_ctx != NULL)
    {
        iio_context_destroy(phy_init_ctx);
        phy_init_ctx = NULL;
    }
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
static int phy_init_get_args(int argc, char **argv, struct phy_cfg *cfg, char *profilename, unsigned short filenamesize)
{
    int c;
    int errorflag = 0;
    int tmpsize = 0;
    if(cfg == 0 || profilename == NULL || filenamesize > MAXLEN_FILENAME || filenamesize == 0)
    {
        fprintf(stderr, "ERROR: phy_init_get_args error for cfg or profilename or filenamesize error\n");
        return -1;
        // exit(EXIT_FAILURE);
    }
    /* it is inited when it is defined, so don't init cfg and scfg to 0 */
    while ((c = getopt(argc, argv, "a:f:g:P:p:v:lch")) != EOF)
    {
        switch (c) 
        {
        case 'a':
            cfg->tx_gain = strtoul(optarg, NULL,  0);
            break;
        case 'f':
            cfg->rx_freq = GHZ(strtoul(optarg, NULL,  0)); 
            cfg->tx_freq = GHZ(strtoul(optarg, NULL,  0)); 
            break;
        case 'g':
            cfg->rx_gain= strtoul(optarg, NULL,  0);
            break;
        case 'P':
        case 'p':
            memset(profilename, 0x00, tmpsize);
            tmpsize -= 1;
            if(tmpsize > strlen(optarg))
                tmpsize = strlen(optarg);
            memcpy(profilename, optarg, tmpsize);
            break;
        case 'v':
            mode_verbo = strtoul(optarg, NULL,  0);
            break;
        case 'l':
            cfg->loop = 1;
            break;
        case 'c':
            break;
        case 'h':
            phy_init_usage();
            errorflag = -1;
            //exit( EXIT_FAILURE );
            break;
        case '?':
            if (isprint(optopt))
            {
                fprintf(stderr,"ERROR: unrecognised option \"%c\"\n",(char) optopt);
                errorflag = -1;
                //exit(EXIT_FAILURE);
            }
            break;
        default:
            fprintf(stderr,"ERROR: unrecognised option \"%c\"\n",(char) optopt);
            errorflag = -1;
            //exit(EXIT_FAILURE);
            break;
        }
        if(errorflag == -1)
            break;
    }
    return errorflag;
}

/* applies streaming configuration through IIO */
static bool phy_init_cfg_ad9371_stream_ch(struct iio_device *ptrphy, struct phy_cfg *pcfg,  int chid, enum iodev type)
{
    long long val;
    double gain;
    struct iio_channel *phychn = NULL;
    struct iio_channel *logchn = NULL;

    if(pcfg == NULL || ptrphy == NULL || type >= IO_BUTT)
    {
        return false;
    }

    // Configure phy and lo channels
    if (!get_phy_chan(ptrphy, type, chid, &phychn))
    {
        return false;
    }

    if(type == TX)
    {
        printf("* Acquiring AD9371 phy TX channel %d\n", chid);
        iio_channel_attr_write_double(phychn, "hardwaregain", pcfg->tx_gain);
        iio_channel_attr_write_bool(phychn, "quadrature_tracking_en", pcfg->tx_qtracking);
        iio_channel_attr_write_bool(phychn, "lo_leakage_tracking_en", pcfg->tx_loltracking);

        iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
        printf("Channel %s Tx attenuation is %f\n", iio_channel_get_name(phychn), gain);
        // Configure LO channel
        printf("* Acquiring AD9371 TX lo channel\n");
        if (!get_lo_chan(ptrphy, type, &logchn)) { return false; }
        iio_channel_attr_write_longlong(logchn, "TX_LO_frequency", pcfg->tx_freq);
        iio_channel_attr_read_longlong(logchn, "TX_LO_frequency", &val);
        printf("Tx %lld frequency is %lld\n",pcfg->tx_freq, val);

        printf("* Acquiring AD9371 ORx lo channel\n");
        logchn = iio_device_find_channel(ptrphy, get_ch_name("altvoltage", 2), true);
        iio_channel_attr_write_longlong(logchn, "RX_SN_LO_frequency", pcfg->tx_freq);
        iio_channel_attr_read_longlong(logchn, "RX_SN_LO_frequency", &val);
        printf("OBS %lld frequency is %lld\n", pcfg->tx_freq, val);
    }
    else
    {
        printf("* Acquiring AD9371 phy RX channel %d\n", chid);
        iio_channel_attr_write_double(phychn, "hardwaregain", pcfg->rx_gain);
        iio_channel_attr_write_bool(phychn, "quadrature_tracking_en", pcfg->rx_qtracking);

        iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
        printf("Channel %s Rx gain is %f\n", iio_channel_get_name(phychn), gain);
        // Configure LO channel
        printf("* Acquiring AD9371 RX lo channel\n");
        if (!get_lo_chan(ptrphy, type, &logchn)) { return false; }
        iio_channel_attr_write_longlong(logchn, "RX_LO_frequency", pcfg->rx_freq);
        iio_channel_attr_read_longlong(logchn, "RX_LO_frequency", &val);
        printf("Rx %lld frequency is %lld\n",pcfg->rx_freq, val);
    }
    return true;
}

/* applies streaming configuration through IIO */
static bool phy_init_adrv9009_streaming_ch(struct iio_device *ptrphy, struct phy_cfg *pcfg,  int chid, enum iodev type)
{
    long long val;
    double gain;
    struct iio_channel *phychn = NULL;
    struct iio_channel *logchn = NULL;

    if(pcfg == NULL || ptrphy == NULL || type >= IO_BUTT)
    {
        return false;
    }

    // Configure phy and lo channels
    if (!get_phy_chan(ptrphy, type, chid, &phychn))
    {
        return false;
    }

    if(type == TX)	//tx1rx0=1: config Tx
    {
        // Configure channels parameter
        printf("* Configure hardwaregain ADRV9009 Tx channel: %d\n",  chid);
        iio_channel_attr_write_double(phychn, "hardwaregain", pcfg->tx_gain);
        iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
        printf("Channel %s Tx attenuation is %f\n", iio_channel_get_name(phychn), gain);

        if (!get_lo_chan(ptrphy, type, &logchn)) { return false;}
        iio_channel_attr_write_longlong(logchn, "frequency", pcfg->tx_freq);
        iio_channel_attr_read_longlong(logchn, "frequency", &val);
        printf("TX frequency is %lld\n",val);
    }
    if(type == RX)
    {
        if(is_chan_power_down(ptrphy, chid))
            power_up_chan(ptrphy, chid);
        // Configure channels parameter
        printf("* Configure hardwaregain ADRV9009 RX channel: %d\n",  chid);
        iio_channel_attr_write_double(phychn, "hardwaregain", pcfg->rx_gain);
        iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
        printf("Channel %s Rx gain is %f\n", iio_channel_get_name(phychn), gain);

        if (!get_lo_chan(ptrphy, type, &logchn)) { return false;}
        iio_channel_attr_write_longlong(logchn, "frequency", pcfg->rx_freq);
        iio_channel_attr_read_longlong(logchn, "frequency", &val);
        printf("RX frequency is %lld\n",val);
    }
    if(type == ORX)
    {
        power_down_chan(ptrphy, 0);  //power down rx1
        power_down_chan(ptrphy, 1);  //power down rx2
        power_up_chan(ptrphy, chid);  //power up orx

        // Configure channels parameter
        printf("* Configure hardwaregain ADRV9009 ORX channel: %d\n",  chid);
        iio_channel_attr_write_double(phychn, "hardwaregain", pcfg->rx_gain);
        iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
        printf("Channel %s ORx gain is %f\n", iio_channel_get_name(phychn), gain);

        if (!get_lo_chan(ptrphy, type, &logchn)) { return false;}
        iio_channel_attr_write_longlong(logchn, "frequency", pcfg->tx_freq);
        iio_channel_attr_read_longlong(logchn, "frequency", &val);
        printf("ORX frequency is %lld\n",val);
    }
	return true;
}

static void phy_init_profilead9xxx(struct iio_context *ctx, char *profilename)
{
    unsigned char i;
    struct iio_device *phy[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
    char devicead9009name[][32]=
    {
        "adrv9009-phy"
    };

    char devicead9371name[][32]=
    {
        "ad9371-phy",
        "ad9371-phy-n2"
    };

    if (ctx == NULL)
    {
        printf("phy_init_profilead9xxx failed for ctx NULL\r\n");
        return;
    }

    /*œ»’“AD9371*/
    for(i=0; i<(sizeof(phy)/sizeof(phy[0])); i++)
    {
        phy[i] = NULL;
    }
    
    for(i=0; i<(sizeof(devicead9371name)/sizeof(devicead9371name[0])); i++)
    {
        phy[i] = get_phy_dev(ctx, devicead9371name[i]);
        if(phy[i] == NULL)
            break;
        if(profilename != NULL && strlen(profilename))
            load_profile_simplified(ctx, phy[i], profilename);
        else
            load_ad9371_profile(phy[i], &mykDevice);
    }
    if(i != 0)
    {
        return;
    }

    for(i=0; i<sizeof(phy)/sizeof(phy[0]); i++)
    {
        phy[i] = NULL;
    }
    /*AD9009 Process, wait for yangyang*/
    for(i=0; i<(sizeof(devicead9009name)/sizeof(devicead9009name[0])); i++)
    {
        phy[i] = get_phy_dev(ctx, devicead9009name[i]);
        if(phy[i] == NULL)
            break;
        if(profilename != NULL && strlen(profilename))
            load_profile_simplified(ctx, phy[i], profilename);
        else
            load_adrv9009_profile(phy[i], &talInit);
    }
}

/* Configure AD9XXXX*/
static void phy_init_cfgad9xxx(struct iio_context *ctx, struct phy_cfg* pcfg)
{
    int i, j;
    bool errflag = 0;
    bool val = 0; 
    long long tempvar1;
    long long tempvar2;
    char ifad9371 = -1; /*not ad9371 or ad9009*/
    struct iio_device *phy[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
    struct iio_channel *obschn[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
    char devicead9009name[][32]=
    {
        "adrv9009-phy"
    };

    char devicead9371name[][32]=
    {
        "ad9371-phy",
        "ad9371-phy-n2"
    };

    if (ctx == NULL || pcfg == NULL)
    {
        printf("phy_init_cfgad9371 failed for ctx NULL or pcfg NULL\r\n");
        return;
    }

    /*œ»’“AD9371*/
    for(i=0; i<(sizeof(phy)/sizeof(phy[0])); i++)
    {
        phy[i] = NULL;
        obschn[i] = NULL;
    }

    for(i=0; i<(sizeof(devicead9371name)/sizeof(devicead9371name[0])); i++)
    {
      phy[i] = get_phy_dev(ctx, devicead9371name[i]);
        if(phy[i] == NULL)
            break;
        ifad9371 = 1;
        errflag = 0;
        for(j=0; j<MAX_TRNUM_PER_CHIP; j++)
        {
            if(true != phy_init_cfg_ad9371_stream_ch(phy[i], pcfg, j, RX))
            {
                errflag = 1;
                printf("Ad9371 chip %d RX port %d not found\r\n", i, j);
                if( i > 0 )
                    i--;
                break;
            }
            if(true != phy_init_cfg_ad9371_stream_ch(phy[i], pcfg, j, TX))
            {
                errflag = 1;
                printf("Ad9371 chip %d TX port %d not found\r\n", i, j);
                break;
            }
        }
        if(errflag)
        {
            if( i > 0 )
                i--;
            break;
        }
        iio_device_debug_attr_write_longlong(phy[i],"adi,tx-settings-tx-pll-lo-frequency_hz", pcfg->tx_freq);
        iio_device_debug_attr_write_longlong(phy[i],"adi,rx-settings-rx-pll-lo-frequency_hz", pcfg->rx_freq);
        iio_device_debug_attr_read_longlong(phy[i],"adi,tx-settings-tx-pll-lo-frequency_hz", &tempvar1);
        iio_device_debug_attr_read_longlong(phy[i],"adi,rx-settings-rx-pll-lo-frequency_hz", &tempvar2);
        printf("Tx frequency is %lld, Rx frequency is %lld\r\n",tempvar1, tempvar2);

        iio_device_debug_attr_read_longlong(phy[i],"adi,tx-settings-tx1-atten_mdb", &tempvar1);
        iio_device_debug_attr_read_longlong(phy[i],"adi,tx-settings-tx2-atten_mdb", &tempvar2);
        printf("tx1_atten is %lld, tx2_atten is %lld\r\n",tempvar1, tempvar2);

        iio_device_debug_attr_read_longlong(phy[i],"adi,tx-profile-iq-rate_khz", &tempvar1);
        iio_device_debug_attr_read_longlong(phy[i],"adi,rx-profile-iq-rate_khz", &tempvar2);
        printf("tx sample rate is %lld, rx sample rate is %lld\r\n",tempvar1, tempvar2);

        /*move from stream*/
        get_phy_chan(phy[i], RX, 2, &obschn[i]);
        iio_channel_attr_write(obschn[i], "rf_port_select", "INTERNALCALS");
        iio_device_debug_attr_write_bool(phy[i],"loopback_tx_rx", pcfg->loop);
        iio_device_debug_attr_read_bool(phy[i],"loopback_tx_rx", &val);
        syslog_level(LOG_LEVEL1, "Tx_RX loopback is %d\r\n",val);
        /*move from stream end*/
    }
    /*if chip is AD9371,then end of process AD9371, else find AD9009*/
    if(ifad9371 == 1)
    {
        return;
    }

    for(i=0; i<sizeof(phy)/sizeof(phy[0]); i++)
    {
        phy[i] = NULL;
        obschn[i] = NULL;
    }
    /*AD9009 Process, wait for yangyang*/
    for(i=0; i<(sizeof(devicead9009name)/sizeof(devicead9009name[0])); i++)
    {
        phy[i] = get_phy_dev(ctx, devicead9009name[i]);
        if(phy[i] == NULL)
            break;
        get_phy_chan(phy[i], RX, 2, &obschn[i]);
        ifad9371 = 0;
        errflag = 0;
        for(j=0; j<MAX_TRNUM_PER_CHIP; j++)
        {
            if(true != phy_init_adrv9009_streaming_ch(phy[i], pcfg, j, RX))
            {
                errflag = 1;
                printf("Ad9009 chip %d RX port %d not found\r\n", i, j);
                if( i > 0 )
                    i--;
                break;
            }
            if(true != phy_init_adrv9009_streaming_ch(phy[i], pcfg, j, TX))
            {
                errflag = 1;
                printf("Ad9009 chip %d TX port %d not found\r\n", i, j);
                break;
            }
        }
        if(errflag)
        {
            if( i > 0 )
                i--;
            break;
        }
    }
}

/*for extern used*/
int phy_init_chgsetting(struct iio_context *ctx, int argc, char **argv)
{
    int errorflag = 0;
    char profilename[MAXLEN_FILENAME];
    /* define global variables*/
    struct phy_cfg cfg = {
       0, //double rx_gain;
       -30, //double tx_atten;
       1, //bool tx_qtracking;
       0, //bool tx_loltracking;
       1, //bool rx_qtracking;
       GHZ(3.5), //long long tx_freq;
       GHZ(3.5), //long long rx_freq;
       0 //not enable loopback
    };
    struct iio_context *tmpctx = ctx;

    memset(profilename, 0x00, sizeof(profilename));

    if(tmpctx == NULL)
    {
        tmpctx = iio_create_local_context();
        if(tmpctx == NULL)
        {
            printf("phy_init_chgsetting failed for no context\r\n");
            return -1;
        }
    }

    if(iio_context_get_devices_count(tmpctx) == 0)
    {
        printf("phy_init_chgsetting failed for no devices\r\n");
        return -1;
    }

    //parse input command
    errorflag = phy_init_get_args(argc, argv, &cfg, profilename, MAXLEN_FILENAME);
    if(errorflag != 0)
        return errorflag;

    phy_init_cfgad9xxx(tmpctx,&cfg);

    if (ctx == NULL && tmpctx != NULL)
    {
        iio_context_destroy(tmpctx);
        tmpctx = NULL;
    }
    return 0;
}

/* main function */
int main (int argc, char **argv)
{
    int errorflag = 0;
    char profilename[MAXLEN_FILENAME];
    /* define global variables*/
    struct phy_cfg cfg = {
       0, //double rx_gain;
       -20, //double tx_atten;
       1, //bool tx_qtracking;
       0, //bool tx_loltracking;
       1, //bool rx_qtracking;
       GHZ(3.5), //long long tx_freq;
       GHZ(3.5), //long long rx_freq;
       0 //not enable loopback
    };

    memset(profilename, 0x00, sizeof(profilename));

    printf("* Acquiring IIO context\r\n");
    phy_init_ctx = iio_create_local_context();
    if(phy_init_ctx == NULL)
    {
        printf("No context\r\n");
        return -1;
    }

    //parse input command
    errorflag = phy_init_get_args(argc, argv, &cfg, profilename, MAXLEN_FILENAME);
    if(errorflag != 0)
        return errorflag;

    syslog_level(LOG_LEVEL2,"Get %d devices.\r\n",iio_context_get_devices_count(phy_init_ctx));
    if(iio_context_get_devices_count(phy_init_ctx) == 0)
    {
        iio_context_destroy(phy_init_ctx);    
        printf("No devices\r\n");
        return -1;
    }

    // Listen to ctrl+c and ASSERT
    signal(SIGINT, phy_init_handle_sig);

    phy_init_profilead9xxx(phy_init_ctx, profilename);

    phy_init_cfgad9xxx(phy_init_ctx, &cfg);

    if (phy_init_ctx != NULL)
    {
        iio_context_destroy(phy_init_ctx);
        phy_init_ctx = NULL;
    }
    return 0;
}

