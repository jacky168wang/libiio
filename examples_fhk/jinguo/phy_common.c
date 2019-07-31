/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
 *
 * License: GPL, version 2.1
 */
#include "ad9371profile20mhz.h"
#include "ad9009profile100mhz.h"
#include "ad9xxxcommon.h"
/* static scratch mem for strings */
static char tmpstr[64];

bool stop;

int runtime = 0;
unsigned int mode_verbo = 0;


/* check return value of attr_write function */
void errchk(int v, const char* what)
{
	if (v < 0) {
		fprintf(stderr, "Error %d writing to channel \"%s\"" \
			"\nvalue may not be supported.\n", v, what);
		exit(0);
	}
}

/* write attribute: long long int */
void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
	errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* read attribute: long long int */
inline long long rd_ch_lli(struct iio_channel *chn, const char* what)
{
	long long val;

	errchk(iio_channel_attr_read_longlong(chn, what, &val), what);

	printf("\t %s: %lld\n", what, val);
	return val;
}

#if 0
/* write attribute: string */
void wr_ch_str(struct iio_channel *chn, const char* what, const char* str)
{
	errchk(iio_channel_attr_write(chn, what, str), what);
}
#endif

/* helper function generating channel names */
char* get_ch_name_mod(const char* type, int id, char modify)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d_%c", type, id, modify);
	return tmpstr;
}

/* helper function generating channel names */
char* get_ch_name(const char* type, int id)
{
	snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
	return tmpstr;
}

/* returns ad9371 phy device */
struct iio_device* get_phy_dev(struct iio_context *ctx, char *devicename)
{
    struct iio_device *dev = NULL;
    if(ctx == NULL || devicename == 0)
        return dev;
    dev =  iio_context_find_device(ctx, devicename);
    if(dev != NULL)
        printf("Phy device name is %s\n",devicename);
    return dev;
}

/* finds streaming IIO channels */
bool get_stream_chn(struct iio_device *dev, enum iodev d, int chid, char modify, struct iio_channel **chn)
{
	*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	if (!*chn)
		*chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
	return *chn != NULL;
}

/* finds phy IIO configuration channel with id chid */
bool get_phy_chan(struct iio_device *phy, enum iodev d, int chid, struct iio_channel **chn)
{
	switch (d) {
		case RX: *chn = iio_device_find_channel(phy, get_ch_name("voltage", chid), false); return *chn != NULL;
		case TX: *chn = iio_device_find_channel(phy, get_ch_name("voltage", chid), true);  return *chn != NULL;
		default: ASSERT(0); return false;
	}
}

/* finds AD9371 local oscillator IIO configuration channels */
bool get_lo_chan(struct iio_device *phy, enum iodev d, struct iio_channel **chn)
{
	switch (d) {
		// LO chan is always output, i.e. true
		case RX: *chn = iio_device_find_channel(phy, get_ch_name("altvoltage", 0), true); return *chn != NULL;
		case TX: *chn = iio_device_find_channel(phy, get_ch_name("altvoltage", 1), true); return *chn != NULL;
		default: ASSERT(0); return false;
	}
}

void print_channel_name(struct iio_channel *chn)
{
    printf("Channe name is %s \n", iio_channel_get_name(chn));
}

bool print_channel_attr(struct iio_device *phy, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    printf("* Acquiring phy %s channel %d\n", type == TX ? "TX" : "RX", chid);
    if (!get_phy_chan(phy, type, chid, &chn)) return false;

    int i;
	//chn->nb_attrs = iio_channel_get_attrs_count(&chn);
    printf("\nPhy channel attributes list, %d item in total\n",chn->nb_attrs);
    for (i=0; i<chn->nb_attrs; i++) {
        printf("%s %s\n", chn->attrs[i].name, chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

    printf("* Acquiring %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(phy, type, &chn)) return false;

    printf("\nLO channel attributes list, %d item in total\n", chn->nb_attrs);
    for (i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

    printf("* Acquiring ORx lo channel\n");
    chn = iio_device_find_channel(phy, get_ch_name("altvoltage", 2), true);
    for (i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));
    return true;
}

bool print_ad9371_channel_attr2(struct iio_channel *chn)
{
    ASSERT(chn);
    printf("Channel name is %s, channel type is %d\n", iio_channel_get_name(chn),iio_channel_get_type(chn));
	for (int i = 0; i<chn->nb_attrs; i++) {
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
	}
    return true;
}

/* power down channel*/
bool power_down_chan(struct iio_device *ptrphy, int chid)
{
    bool powerdown;
    struct iio_channel *chn = NULL;

    if (!get_phy_chan(ptrphy, 0, chid, &chn))
    {
        printf("* Power down Rx channel %d failed for get_phy_chan\n", chid);
        return false;
    }

    printf("* Power down Rx channel %d\n", chid);
    iio_channel_attr_write_bool(chn, "powerdown", 1);
    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    printf("powerdown status is %d\n", powerdown);
    
    return true;
}

/* power up channel*/
bool power_up_chan(struct iio_device *ptrphy, int chid)
{
    bool powerdown;
    struct iio_channel *chn = NULL;

    if (!get_phy_chan(ptrphy, 0, chid, &chn))
    {
        printf("* Power up Rx channel %d failed for get_phy_chan\n", chid);
        return false;
    }

    printf("* Power up Rx channel %d\n", chid);
    iio_channel_attr_write_bool(chn, "powerdown", 0);
    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    printf("channel powerdown status is %d\n", powerdown);
    
    return true;
}

bool is_chan_power_down(struct iio_device *ptrphy, int chid)
{
    bool powerdown;
    struct iio_channel *chn = NULL;

    if (!get_phy_chan(ptrphy, 0, chid, &chn))
    {
        printf("* Power down Rx channel %d failed for get_phy_chan\n", chid);
        return true;
    }

    iio_channel_attr_read_bool(chn, "powerdown", &powerdown);
    return powerdown;
}

//Loading the ad9371 profile generated by the TES
void load_ad9371_profile(struct iio_device *phy, mykonosDevice_t *mykDevice)
{
    long long val;

    //Tx
    mykonosTxSettings_t *txsetting = mykDevice->tx;

    //enabled channels
    val = txsetting->txChannels;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-channels-enable", val);

    //PLL use external Lo
    val = txsetting->txPllUseExternalLo;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-pll-use-external-lo", val);

    //PLL Lo Frequency_Hz
    val = txsetting->txPllLoFrequency_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-pll-lo-frequency_hz", val);

    //tx atten step size
    val = txsetting->txAttenStepSize;
    iio_device_debug_attr_write_longlong(phy, "adi,adi,tx-settings-tx-atten-step-size", val);

    //tx1, tx2 atten, mdB
    val = txsetting->tx2Atten_mdB;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx2-atten_mdb", val);
    val = txsetting->tx1Atten_mdB;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx1-atten_mdb", val);

    //txprofile
    mykonosTxProfile_t *txp = txsetting->txProfile;
    val = txp->txBbf3dBCorner_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-bbf-3db-corner_khz", val);
    val = txp->txDac3dBCorner_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-dac-3db-corner_khz", val);
    val = txp->rfBandwidth_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-rf-bandwidth_hz", val);
    val = txp->primarySigBandwidth_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-primary-sig-bandwidth_hz", val);
    val = txp->iqRate_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-iq-rate_khz", val);
    val = txp->txInputHbInterpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-input-hb-interpolation", val);
    val = txp->thb1Interpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb2-interpolation", val);
    val = txp->thb2Interpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-thb1-interpolation", val);
    val = txp->txFirInterpolation;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-tx-fir-interpolation", val);
    val = txp->dacDiv;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-profile-dac-div", val);


    //Rx
    mykonosRxSettings_t *rxsetting = mykDevice->rx;

    val = rxsetting->rxPllLoFrequency_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-lo-frequency_hz", val);
    val = rxsetting->rxPllUseExternalLo;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-pll-use-external-lo", val);
    val = rxsetting->rxChannels;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-channels-enable", val);
    val = rxsetting->realIfData;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-real-if-data", val);

    mykonosRxProfile_t *rxp = rxsetting->rxProfile;
    val = rxp->rxBbf3dBCorner_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-bbf-3db-corner_khz", val);
    val = rxp->rfBandwidth_Hz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rf-bandwidth_hz", val);
    val = rxp->iqRate_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-iq-rate_khz", val);
    val = rxp->rhb1Decimation;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rhb1-decimation", val);
    val = rxp->enHighRejDec5;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-en-high-rej-dec5", val);
    val = rxp->rxDec5Decimation;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-dec5-decimation", val);
    val = rxp->rxFirDecimation;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-profile-rx-fir-decimation", val);
}

//Loading the adrv9009 profile generated by the TES
void load_adrv9009_profile(struct iio_device *phy, taliseInit_t *talInit)
{
    long long val;    //Tx setting data structure
    taliseTxSettings_t *txsetting = &talInit->tx;
    taliseRxSettings_t *rxsetting = &talInit->rx;
    taliseDigClocks_t  *pclocks = &talInit->clocks;

    val = txsetting->txChannels;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-channels-enable", val);
    //tx data options if pll unlock
    val = txsetting->disTxDataIfPllUnlock;
    iio_device_debug_attr_write_longlong(phy, "adi,tx-settings-tx-data-options-if-pll-unlock", val);
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

    val = rxsetting->rxChannels;
    iio_device_debug_attr_write_longlong(phy, "adi,rx-settings-rx-channels-enable", val);
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

    val = pclocks->deviceClock_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,clocks-device-clock_khz", val);
    val = pclocks->clkPllVcoFreq_kHz;
    iio_device_debug_attr_write_longlong(phy, "adi,clocks-clk-pll-vco-freq_khz", val);
    val = pclocks->clkPllHsDiv;
    iio_device_debug_attr_write_longlong(phy, "adi,clocks-clk-pll-hs-div", val);

    printf("\r\nload_adrv9009_profile succes\r\n");
}

void load_profile_simplified(struct iio_context *ctx, struct iio_device *phy, char *fn)
{
    printf("Load profile from %s\n",fn);
    FILE *fd = fopen(fn, "r");
    if(!fd) {
        fprintf(stderr, "Cannot open the profile file\n");
	 	return;
    }
    fseek(fd, 0, SEEK_END);
    size_t len = ftell(fd);
    //printf("%d\n", len);
    char *buf = malloc(len);
    fseek(fd,0,SEEK_SET);
    len = fread(buf,1,len,fd);
    fclose(fd);

    printf("the profile's length is:%d\n", (int)len);
    iio_context_set_timeout(ctx, 30000);
    ASSERT_ATTR(iio_device_attr_write_raw(phy,"profile_config",buf,len));
    printf("Profile write finished.\n");
    iio_context_set_timeout(ctx, 3000);
    free(buf);
}


long long timediff(timespec a, timespec b)
{
    return (1000000000*(b.tv_sec-a.tv_sec) + b.tv_nsec-a.tv_nsec );
}

void* OpenMemory(int *memfd, off_t dev_base, size_t size)
{
    size_t mask = size-1;
    void *mapped_base;
    if (*memfd == -1){
        *memfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (*memfd == -1)
        {
            printf("Can't open /dev/mem.\n");
            return NULL;
        }
        syslog_level(LOG_LEVEL1, "/dev/mem opened.\n");
    }

    // Map one page of memory into user space such that the device is in that page, but it may not
    // be at the start of the page.

    mapped_base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, *memfd, dev_base & ~mask);
    if (mapped_base == (void *) -1) {
        printf("Can't map the memory to user space.\n");
        return NULL;
    }
    syslog_level(LOG_LEVEL1, "Memory mapped at address %p.\n", mapped_base);
    return (mapped_base + (dev_base & mask));
}

/*judge rf chip type : -1 no rf chip, 0 ad9009, 1 ad9371*/
char judge_rfchip_type(struct iio_context *ctx)
{
    char rfchiptype = -1;
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
        printf("judge_rfchip_type failed for ctx NULL\r\n");
        return rfchiptype;
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
    }
    if(i != 0)
    {
        rfchiptype = 1;
        return rfchiptype;
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
    }
    if(i != 0)
    {
        rfchiptype = 0;
    }
    return rfchiptype;
}

void syslog_level(unsigned int level, const char *pFormat, ... )
{
    va_list     ArgList;
    char        chEosPrintfString[EOS_PRINTF_MAX_LEN+1];

    if(mode_verbo < level)
        return;

    /* °‰®∞®Æ?????°¡?°§?°‰?2??®∫D®™1y3°Ë, °§®§?1?®≤°‰????? */
    if( strlen( pFormat ) >= ( EOS_PRINTF_MAX_LEN * 3 / 4 ) )
    {
        printf( "EOS_Printf(): format too long " );
        printf( "%s", *pFormat);
        return;
    }
    memset(chEosPrintfString, 0x00, sizeof(chEosPrintfString));

    va_start ( ArgList, pFormat );
    (void)vsprintf ( chEosPrintfString, pFormat, ArgList );
    va_end ( ArgList );

    /* °‰®∞®Æ?¶Ã?°¡?°§?°‰?®∫?°§?1y3°Ë */
    if( strlen( chEosPrintfString ) >=  EOS_PRINTF_MAX_LEN )
    {
        chEosPrintfString[EOS_PRINTF_MAX_LEN - 1] = '\0';
    }

    printf( "%s", chEosPrintfString );
}
