/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
 *
 * License: GPL, version 2.1
 */
#include "ad9371common_junyi.h"

/* static scratch mem for strings */
static char tmpstr[64];

bool stop;

int runtime = 0;
unsigned int mode_verbo = 0;

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./stream [OPTION]\n"
		" -h\tshow this help\n"
		" -s\tset source file of the transmission. This indicates a Tx call and must not be used together with -d\n"
		" -d\tset dst file of the receiving. This indicates a Rx call and must not be used together with -s\n"
		" -a\tset Tx attenuation\n"
		" -g\tset Rx Gain\n"
		" -n\tset Rx frame length (samples)\n"
		" -N\tset Rx Frame Number\n"
		" -f\tset carrier frequency\n"
		" -m\tchoose Tx/Rx channel\n"
		" -l\tenable JESD204B loopback mode\n"
		" -c\tDISABLE AD9371 tracking calibration\n"
		" -F\tenable FPGA DSP\n"
		" -C\tcircularly transmission\n"
		" -D\tenable iio buffer\n"
		" -O\tenable ORx\n"
		" -v\tbe verbose [0,1,2,3]\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
void get_args(int argc, char **argv, struct phy_cfg *cfg, struct stream_cfg *scfg)
{
	int c;
	opterr = 0;
	while ((c = getopt(argc, argv, "s:d:a:g:n:N:f:m:v:lcFCDOh")) != EOF) {
		switch (c) {
		case 's': 
			scfg->filename = malloc(strlen(optarg));
			strcpy(scfg->filename, optarg);
			scfg->tx_on = 1; scfg->rx_on = 0;
			break;
		case 'd':
			scfg->filename = malloc(strlen(optarg));
			strcpy(scfg->filename, optarg);
			scfg->tx_on = 0; scfg->rx_on = 1;
			break;
		case 'a': 
			cfg->tx_atten = strtoul(optarg, NULL,  0); break;
		case 'g':
			cfg->rx_gain= strtoul(optarg, NULL,  0);	break;
		case 'n':
			scfg->frame_size= strtoul(optarg, NULL,  0);	break;
		case 'N':
			scfg->rx_on = 1;
			scfg->frame_num= strtoul(optarg, NULL,  0); break;
		case 'f':
			cfg->freq= GHZ(strtoul(optarg, NULL,  0)); break;
		case 'm':
			scfg->map = strtoul(optarg, NULL,  0); break;
		case 'v': 
			mode_verbo = strtoul(optarg, NULL,  0);	break;
		case 'l': cfg->loop = 1; break;
		case 'c': cfg->tracking_cal= 0; break;
		case 'F': scfg->enb_FPGA= 1; break;
		case 'C': scfg->cycle= 1; break;
		case 'D': scfg->enb_DMA= 1; break;
		case 'O': 
			scfg->tx_on = 0; scfg->rx_on = 0; scfg->orx_on= 1; break;
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
}


/* check return value of attr_write function */
void errchk(int v, const char* what) {
     if (v < 0) { fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what); shutdown_iio(); }
}

/* write attribute: long long int */
void wr_ch_lli(struct iio_channel *chn, const char* what, long long val)
{
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* read attribute: long long int */
long long rd_ch_lli(struct iio_channel *chn, const char* what)
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
struct iio_device* get_ad9371_phy(struct iio_context *ctx)
{
    struct iio_device *dev =  iio_context_find_device(ctx, "ad9371-phy");
    ASSERT(dev && "No ad9371-phy found");
    return dev;
}

/* finds AD9371 streaming IIO devices */
bool get_ad9371_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
    switch (d) {
    case TX: *dev = iio_context_find_device(ctx, "axi-ad9371-tx-hpc"); return *dev != NULL;
    case RX: *dev = iio_context_find_device(ctx, "axi-ad9371-rx-hpc");  return *dev != NULL;
    case ORX: *dev = iio_context_find_device(ctx, "axi-ad9371-rx-obs-hpc");  return *dev != NULL;
    default: ASSERT(0); return false;
    }
}

/* finds AD9371 streaming IIO channels */
bool get_ad9371_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, char modify, struct iio_channel **chn)
{
    *chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
#ifdef DEBUG
    //printf("Channel name is %s\n", modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid));
#endif
    if (!*chn)
        *chn = iio_device_find_channel(dev, modify ? get_ch_name_mod("voltage", chid, modify) : get_ch_name("voltage", chid), d == TX);
    return *chn != NULL;
}

/* finds AD9371 phy IIO configuration channel with id chid */
bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
    switch (d) {
    case RX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("voltage", chid), false); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("voltage", chid), true);  return *chn != NULL;
    default: ASSERT(0); return false;
    }
}

/* finds AD9371 local oscillator IIO configuration channels */
bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
    switch (d) {
     // LO chan is always output, i.e. true
    case RX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 0), true); return *chn != NULL;
    case TX: *chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 1), true); return *chn != NULL;
    default: ASSERT(0); return false;
    }
}

/* applies streaming configuration through IIO */
bool cfg_ad9371_streaming_ch(struct iio_context *ctx, struct phy_cfg *cfg, int chid, int tx1rx0)
{
    struct iio_channel *chn = NULL;
    double gain, atten;
    long long val;

    if (tx1rx0){
        // Configure phy and lo channels
        LOG_PRINT_L2(mode_verbo,printf("* Acquiring AD9371 phy TX channel %d\n", chid));
        if (!get_phy_chan(ctx, 1, chid, &chn)) {	return false; }

        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->tx_atten));
	if (cfg->tracking_cal){
        	ASSERT_ATTR(iio_channel_attr_write_bool(chn, "quadrature_tracking_en", 1));
        	ASSERT_ATTR(iio_channel_attr_write_bool(chn, "lo_leakage_tracking_en", 1));
			}
	else{
		ASSERT_ATTR(iio_channel_attr_write_bool(chn, "quadrature_tracking_en", 0));
        	ASSERT_ATTR(iio_channel_attr_write_bool(chn, "lo_leakage_tracking_en", 0));
	}

        iio_channel_attr_read_double(chn, "hardwaregain", &atten);
        LOG_PRINT_L1(mode_verbo,printf("Channel %s Tx attenuation is %f\n", iio_channel_get_name(chn), atten));
        // Configure LO channel
        LOG_PRINT_L2(mode_verbo,printf("* Acquiring AD9371 TX lo channel\n"));
        if (!get_lo_chan(ctx, 1, &chn)) { return false; }
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "TX_LO_frequency", cfg->freq));
        iio_channel_attr_read_longlong(chn, "TX_LO_frequency", &val);
        LOG_PRINT_L1(mode_verbo,printf("Tx frequency is %lld\n",val));

        LOG_PRINT_L2(mode_verbo,printf("* Acquiring AD9371 ORx lo channel\n"));
        chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 2), true);
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "RX_SN_LO_frequency", cfg->freq));
        iio_channel_attr_read_longlong(chn, "RX_SN_LO_frequency", &val);
        LOG_PRINT_L1(mode_verbo,printf("OBS frequency is %lld\n",val));
    }

    else {

        LOG_PRINT_L2(mode_verbo,printf("* Acquiring AD9371 phy RX channel %d\n", chid));
        if (!get_phy_chan(ctx, 0, chid, &chn)) {	return false; }

        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->rx_gain));
	if(cfg->tracking_cal){
        	ASSERT_ATTR(iio_channel_attr_write_bool(chn, "quadrature_tracking_en", 1));
		}
	else{
	       ASSERT_ATTR(iio_channel_attr_write_bool(chn, "quadrature_tracking_en", 0));
		}

        iio_channel_attr_read_double(chn, "hardwaregain", &gain);
        LOG_PRINT_L1(mode_verbo,printf("Channel %s Rx gain is %f\n", iio_channel_get_name(chn), gain));

        // Configure LO channel
        LOG_PRINT_L2(mode_verbo,printf("* Acquiring AD9371 RX lo channel\n"));
        if (!get_lo_chan(ctx, 0, &chn)) { return false; }
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "RX_LO_frequency", cfg->freq));
        iio_channel_attr_read_longlong(chn, "RX_LO_frequency", &val);
        LOG_PRINT_L1(mode_verbo,printf("Rx frequency is %lld\n",val));
    }
    return true;
}

void print_channel_name(struct iio_channel *chn){
    printf("Channe name is %s \n", iio_channel_get_name(chn));
}

bool print_ad9371_channel_attr(struct iio_context *ctx, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    printf("* Acquiring AD9371 phy %s channel %d\n", type == TX ? "TX" : "RX", chid);
    if (!get_phy_chan(ctx, type, chid, &chn)) {	return false; }

    int i;
	//chn->nb_attrs = iio_channel_get_attrs_count(&chn);
    printf("\nPhy channel attributes list, %d item in total\n",chn->nb_attrs);
    for (i=0; i<chn->nb_attrs; i++){
        printf("%s %s\n", chn->attrs[i].name, chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

    printf("* Acquiring AD9371 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn)) { return false; }

    printf("\nLO channel attributes list, %d item in total\n", chn->nb_attrs);
    for (i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    }
    printf("Channel type is %d\n", iio_channel_get_type(chn));

    printf("* Acquiring AD9371 ORx lo channel\n");
    chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 2), true);
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
    for (int i = 0; i<chn->nb_attrs; i++){
        printf("%s %s\n",chn->attrs[i].name,chn->attrs[i].filename);
    	}
    return true;
}

/*//Loading the ad9371 profile generated by the TES
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

}*/

void load_ad9371_profile_simplified(struct iio_context *ctx, struct iio_device *phy, char *fn){
    printf("Load profile from %s\n",fn);
    FILE *fd = fopen(fn, "r");
    if(!fd){
        fprintf(stderr, "Cannot open the profile file\n");
	 shutdown_iio();
    	}
    fseek(fd, 0, SEEK_END);
    size_t len = ftell(fd);
    //printf("%d\n", len);
    char *buf = malloc(len);
    fseek(fd,0,SEEK_SET);
    len = fread(buf,1,len,fd);
    fclose(fd);

    //printf("%d\n", len);
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
    shutdown_iio();
}

long long timediff(timespec a, timespec b){
    return (1000000000*(b.tv_sec-a.tv_sec) + b.tv_nsec-a.tv_nsec );
}

void* OpenMemory(int *memfd, off_t dev_base, size_t size){
    size_t mask = size-1;
    void *mapped_base;
    if (*memfd == -1){
        *memfd = open("/dev/mem", O_RDWR | O_SYNC);
        if (*memfd == -1)
        {
            printf("Can't open /dev/mem.\n");
            return NULL;
        }
        LOG_PRINT_L1(mode_verbo,printf("/dev/mem opened.\n"));
    }

    // Map one page of memory into user space such that the device is in that page, but it may not
    // be at the start of the page.

    mapped_base = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, *memfd, dev_base & ~mask);
    if (mapped_base == (void *) -1)
    {
        printf("Can't map the memory to user space.\n");
        return NULL;
    }
    LOG_PRINT_L1(mode_verbo,printf("Memory mapped at address %p.\n", mapped_base));
    return (mapped_base + (dev_base & mask));
}

/*int print_debug_attr(struct iio_device *dev, const char *attr, const char *val, size_t len, void *d){
    long long attr_val;
    memcpy(&attr_val, val, len);
    printf("%s = %lld\n", attr, attr_val);
    return 1;
}*/