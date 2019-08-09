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

#define ORX_ATTEN 0x2000
/* define the memory map address of GPIO*/
#define MEM_BASE 0xff220000
/*OFFSET1*/
#define MEM_MODE_SELECT 0xe030 //mode number (4bits) + division (4bits)
/*OFFSET2*/
#define MEM_CHANNEL_SELECT 0xe020 //Select channel
/*DMA_RECONFIG*/
#define MEM_FPGA_SET 0xe050
/*SYNC_ENB*/
#define MEM_SYNC_SET 0xe040

#define MEM_MAP_SIZE 0x10000
#define MEM_MAP_MASK MEM_MAP_SIZE-1

/* \delta T_r and \delta T_t*/
#define DELTATR 163
#define DELTATT 163

/* declaration of corresponding pointer*/
static int memfd=-1;
static void *mem_base = NULL;
static void *mem_orx_atten = NULL;
static void *mem_mode_select = NULL;
static void *mem_fpga_set = NULL;
static void *mem_sync_set = NULL;

static char default_dst[] = "./rx_data.bin";

static struct stream_cfg scfg = {
      0, //tx is on;
      0, //rx is on;
      0, //or is off;
      0, //not cycle iio_buffer
      0, //not enable FPGA
      0, //not enable DMA
      3, //using No.1 and No.2 AD9371
      30720*20, //sample number in a frame
      1 //receiving one frame
};


/* IIO structs required for streaming */
static struct iio_context *ctx = NULL;

static int ad9xxxchip = 0;
static char ifad9371 = -1; /*1 ad9371; 0 ad9009; -1 unkown*/
static char testtxrxchip = 0;

//axi-ad9371 streaming devices
static struct iio_device *tx[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_device *rx[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_device *or[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *rx0_i[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *rx0_q[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *rx1_i[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *rx1_q[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *orx0_i[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *orx0_q[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *orx1_i[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *orx1_q[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *tx0_i[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *tx0_q[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *tx1_i[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *tx1_q[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *obschn0[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_channel *obschn1[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_buffer  *rxbuf[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};
static struct iio_buffer  *txbuf[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};

void phy_stream_init_global(void)
{
    char i;
    for(i=0; i<MAX_CHIP_NUM; i++)
    {
        tx[i] = NULL;
        rx[i] = NULL;
        or[i] = NULL;
        rx0_i[i] = NULL;
        rx0_q[i] = NULL;
        rx1_i[i] = NULL;
        rx1_q[i] = NULL;
        orx0_i[i] = NULL;
        orx0_q[i] = NULL;
        orx1_i[i] = NULL;
        orx1_q[i] = NULL;
        tx0_i[i] = NULL;
        tx0_q[i] = NULL;
        tx1_i[i] = NULL;
        tx1_q[i] = NULL;
        obschn0[i] = NULL;
        obschn1[i] = NULL;
        rxbuf[i] = NULL;
        txbuf[i] = NULL;
    }
}

/* finds AD9371 streaming IIO devices */
bool phy_stream_get_dev(struct iio_context *ctx, char ifad9371, enum iodev d, char index, struct iio_device **dev)
{
    char ad9371txname[][30] = 
    {
        "axi-ad9371-tx-hpc",
        "axi-ad9371-tx-hpc-n2",
        "axi-ad9371-tx-hpc-n3",
        "axi-ad9371-tx-hpc-n4"
    };
    char ad9371rxname[][30] = 
    {
        "axi-ad9371-rx-hpc",
        "axi-ad9371-rx-hpc-n2",
        "axi-ad9371-rx-hpc-n3",
        "axi-ad9371-rx-hpc-n4"
    };
    char ad9371obsname[][30] = 
    {
        "axi-ad9371-rx-obs-hpc",
        "axi-ad9371-rx-obs-hpc-n2",
        "axi-ad9371-rx-obs-hpc-n3",
        "axi-ad9371-rx-obs-hpc-n4"
    };
    char ad9009txname[][30] = 
    {
        "jesd204tx-layer3",
        "jesd204tx-layer3-n2",
        "jesd204tx-layer3-n3",
        "jesd204tx-layer3-n4"
    };
    char ad9009rxname[][30] = 
    {
        "jesd204rx-layer3",
        "jesd204rx-layer3-n2",
        "jesd204rx-layer3-n3",
        "jesd204rx-layer3-n4"
    };
    char ad9009obsname[][30] = 
    {
        "jesd204ro-layer3",
        "jesd204ro-layer3-n2",
        "jesd204ro-layer3-n3",
        "jesd204ro-layer3-n4"
    };

    if(index < 4)
    {
        switch (d) {
            case TX: 
                if(ifad9371 == 1)
                    *dev = iio_context_find_device(ctx, ad9371txname[index]);
                else
                    *dev = iio_context_find_device(ctx, ad9009txname[index]);
                return *dev != NULL;

            case RX:
                if(ifad9371 == 1)
                    *dev = iio_context_find_device(ctx, ad9371rxname[index]);
                else
                    *dev = iio_context_find_device(ctx, ad9009rxname[index]);
                return *dev != NULL;

            case ORX:
                if(ifad9371 == 1)
                    *dev = iio_context_find_device(ctx, ad9371obsname[index]);
                else
                    *dev = iio_context_find_device(ctx, ad9009obsname[index]);
                return *dev != NULL;

            default:
                return false;
	    }
    }
    else
    {
        return false;
    }
}

void phy_stream_usage(void)
{
	fprintf( stderr,
		"phy_stream_usage : ./phy_stream [OPTION]\n"
		" -h\tshow this help\n"
		" -m\tchoose Tx/Rx channel\n"
		" -n\tset Rx frame length (samples)\n"
		" -N\tset Rx Frame Number\n"
		" -R/-r\tRx stream datefile\n"
		" -T/-t\tTx stream datefile\n"
		" -v\tbe verbose [0,1,2,3]\n"
		" -F\tenable FPGA DSP\n"
		" -C\tcircularly transmission\n"
		" -D\tenable iio buffer\n"
		" -O\tenable ORx\n"
		);
}

/*
unsigned long int strtoul(const char *nptr, char **endptr, int base);
*/
void phy_stream_get_args(int argc, char **argv, struct stream_cfg *scfg, char *rxfilename, char *txfilename, unsigned short filenamesize)
{
    int c;
    int tmpsize = 0;
    if(scfg == 0 || rxfilename == NULL || txfilename == NULL || filenamesize > MAXLEN_FILENAME || filenamesize == 0)
    {
        fprintf(stderr, "ERROR: phy_stream_get_args error for scfg or rxfilename or txfilename error\n");
        exit(EXIT_FAILURE);
    }
    /* it is inited when it is defined, so don't init cfg and scfg to 0 */
    while ((c = getopt(argc, argv, "m:n:N:R:r:T:t:v:V:FcCDOh")) != EOF)
    {
        switch (c) 
        {
        case 'n':
            scfg->frame_size= strtoul(optarg, NULL,  0);
            break;
        case 'N':
            scfg->rx_on = 1;
            scfg->frame_num= strtoul(optarg, NULL,  0); break;
        case 'm':
            scfg->RFICseq = strtoul(optarg, NULL,  0);
            break;
        case 'R':
        case 'r':
            scfg->rx_on = 1;
            memset(rxfilename, 0x00, tmpsize);
            tmpsize -= 1;
            if(tmpsize > strlen(optarg))
                tmpsize = strlen(optarg);
            memcpy(rxfilename, optarg, tmpsize);
            break;
        case 'T':
        case 't':
            scfg->tx_on = 1;
            memset(txfilename, 0x00, tmpsize);
            tmpsize -= 1;
            if(tmpsize > strlen(optarg))
                tmpsize = strlen(optarg);
            memcpy(txfilename, optarg, tmpsize);
            break;
        case 'v':
        case 'V':
            mode_verbo = strtoul(optarg, NULL,  0);
            break;
        case 'F':
            scfg->enb_FPGA= 1;
            break;
        case 'c':
        case 'C':
            scfg->cycle= 1;
            break;
        case 'D':
            scfg->enb_DMA= 1;
            break;
        case 'O':
            scfg->tx_on = 0;
            scfg->rx_on = 0; 
            scfg->orx_on= 1;
            break;
        case 'h':
            phy_stream_usage();
            exit( EXIT_FAILURE );
            break;
        case '?':
            if (isprint(optopt))
            {
                fprintf(stderr,"ERROR: unrecognised option \"%c\"\n",(char) optopt);
                exit(EXIT_FAILURE);
            }
            break;
        default:
            fprintf(stderr,"ERROR: unrecognised option \"%c\"\n",(char) optopt);
            exit(EXIT_FAILURE);
            break;
        }
    }
}


/* Configure AD9371*/
static void phy_stream_getchip_info()
{
    int i;
    bool val = 0; 
    long long tempvar1;
    long long tempvar2;
    struct iio_device *phy[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};

    char devicead9371name[][32]=
    {
        "ad9371-phy",
        "ad9371-phy-n2"
    };

    char devicead9009name[][32]=
    {
        "adrv9009-phy"
    };

    ad9xxxchip = 0;

    syslog_level(LOG_LEVEL2, "* Acquiring IIO context\n");
    ASSERT((ctx = iio_create_local_context()) && "No context");
    syslog_level(LOG_LEVEL2, "Get %d devices.\n",iio_context_get_devices_count(ctx));
    ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    /*先找AD9371*/
    for(i=0; i<sizeof(phy)/sizeof(phy[0]); i++)
    {
        phy[i] = NULL;
    }
    for(i=0; i<(sizeof(devicead9371name)/sizeof(devicead9371name[0])); i++)
    {
      phy[i] = get_phy_dev(ctx, devicead9371name[i]);
        if(phy[i] == NULL)
            break;
        get_phy_chan(phy[i], RX, 2, &obschn0[i]);
        ifad9371 = 1;
    }
    /*找到AD9371,就表示射频板是AD9371,无需找AD9009*/
    if(ifad9371 == 1)
    {
        syslog_level(LOG_LEVEL2, "*Phy stream find %d ad9371\n", i);
        ad9xxxchip = i;
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
        get_phy_chan(phy[i], RX, 2, &obschn0[i]);
        get_phy_chan(phy[i], RX, 3, &obschn1[i]);
        ifad9371 = 0;
    }
    syslog_level(LOG_LEVEL2, "*Phy stream find %d ad9009\n", i);
    ad9xxxchip = i;
}

/* applies streaming configuration through IIO */
bool phy_stream_cfg_adrv9009_ch(struct iio_device *ptrphy, int chid, enum iodev type)
{
    long long val = 0;
    long long freq = 0;
    double gain = 0;
    struct iio_channel *phychn = NULL;
    struct iio_channel *tmpchn = NULL;
    struct iio_channel *logchn = NULL;

    if(ptrphy == NULL || type >= IO_BUTT)
    {
        return false;
    }

    // Configure phy and lo channels
    if (!get_phy_chan(ptrphy, type, chid, &phychn)||!get_lo_chan(ptrphy, type, &logchn))
    {
        return false;
    }


    /*Because set ORX need to power down the rx, so only need to process RX and ORX*/
    if(type == RX)
    {
        if(is_chan_power_down(ptrphy, chid))
        {
            gain = -20;
            freq = GHZ(3.5);
            power_up_chan(ptrphy, chid);
            iio_channel_attr_write_double(phychn, "hardwaregain", gain);
            iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
            printf("Channel %s Rx gain is %f\n", iio_channel_get_name(phychn), gain);
            iio_channel_attr_write_longlong(logchn, "frequency", freq);
            iio_channel_attr_read_longlong(logchn, "frequency", &freq);
            printf("Channel %s frequency is %f\n", iio_channel_get_name(phychn), freq);
        }
    }
    else if(type == ORX)
    {
        if (!get_phy_chan(ptrphy, RX, 0, &tmpchn))
        {
            gain = -20;
            freq = GHZ(3.5);
        }
        else
        {
            iio_channel_attr_read_double(tmpchn, "hardwaregain", &gain);
            iio_channel_attr_read_longlong(logchn, "frequency", &freq);
        }
        power_down_chan(ptrphy, 0);  //power down rx1
        power_down_chan(ptrphy, 1);  //power down rx2
        power_up_chan(ptrphy, chid);  //power up orx

        // Configure channels parameter
        printf("* Configure hardwaregain ADRV9009 ORX channel: %d\n",  chid);
        iio_channel_attr_write_double(phychn, "hardwaregain", gain);
        iio_channel_attr_read_double(phychn, "hardwaregain", &gain);
        printf("Channel %s ORx gain is %f\n", iio_channel_get_name(phychn), gain);

        iio_channel_attr_write_longlong(logchn, "frequency", freq);
        iio_channel_attr_read_longlong(logchn, "frequency", &freq);
        printf("Channel %s frequency is %f\n", iio_channel_get_name(logchn), freq);
    }
	return true;
}
static void phy_stream_cfg_axi()
{
    int i;
    char devicead9009name[][32]=
    {
        "adrv9009-phy"
    };
    struct iio_device *phy[MAX_CHIP_NUM] = {NULL,NULL,NULL,NULL};

    for(i=0; i<sizeof(phy)/sizeof(phy[0]); i++)
    {
        phy[i] = NULL;
    }
    for(i=0; i<ad9xxxchip; i++)
    {
        if(ctx != NULL)
        {
            syslog_level(LOG_LEVEL2, "* Acquiring stream devices\n");
            if(false == phy_stream_get_dev(ctx, ifad9371, TX, i, &tx[i]))
            {
                printf("ad9xxxchip i %d have no TX dev!\r\n", i);
                tx[i] = NULL;
                continue;
            }
            if(false == phy_stream_get_dev(ctx, ifad9371, RX, i, &rx[i]))
            {
                printf("ad9xxxchip i %d have no RX dev!\r\n", i);
                rx[i] = NULL;
                continue;
            }
            if(false == phy_stream_get_dev(ctx, ifad9371, ORX, i, &or[i]))
            {
                printf("ad9xxxchip i %d have no ORX dev!\r\n", i);
                or[i] = NULL;
                continue;
            }
            if(ifad9371 == 1)
            {
                if (scfg.tx_on) 
                {
                    ASSERT(get_stream_chn(tx[i], TX, 0, 0, &tx0_i[i]) && "TX chan i not found");
                    ASSERT(get_stream_chn(tx[i], TX, 1, 0, &tx0_q[i]) && "TX chan q not found");
                    ASSERT(get_stream_chn(tx[i], TX, 2, 0, &tx1_i[i]) && "TX chan i not found");
                    ASSERT(get_stream_chn(tx[i], TX, 3, 0, &tx1_q[i]) && "TX chan q not found");
                    if ((scfg.RFICseq>>i)&0x01)
                    {
                        syslog_level(LOG_LEVEL1, "chip %d two tx channels are enabled!\r\n", i);
                        iio_channel_enable(tx0_i[i]);
                        iio_channel_enable(tx0_q[i]);
                        iio_channel_enable(tx1_i[i]);
                        iio_channel_enable(tx1_q[i]);
                    }
                }
                /*when orx is on, then you can not use rx*/
            	if (scfg.rx_on && scfg.orx_on == 0)
                {
                    ASSERT(get_stream_chn(rx[i], RX, 0, 'i', &rx0_i[i]) && "RX chan i not found");
                    ASSERT(get_stream_chn(rx[i], RX, 0, 'q', &rx0_q[i]) && "RX chan q not found");
                    ASSERT(get_stream_chn(rx[i], RX, 1, 'i', &rx1_i[i]) && "RX chan i not found");
                    ASSERT(get_stream_chn(rx[i], RX, 1, 'q', &rx1_q[i]) && "RX chan q not found");
                    if ((scfg.RFICseq>>i)&0x01)
                    {
                        syslog_level(LOG_LEVEL1, "chip %d two rx channels are enabled!\r\n", i);
                        iio_channel_enable(rx0_i[i]);
                        iio_channel_enable(rx0_q[i]);
                        iio_channel_enable(rx1_i[i]);
                        iio_channel_enable(rx1_q[i]);
                    }
                }

                else if (scfg.orx_on) 
                {
                    char attrval[40] = "ORX1_TX_LO";
                    if(obschn0[i] != NULL)
                    {
                        ASSERT_ATTR(iio_channel_attr_write(obschn0[i], "rf_port_select", attrval));
                        ASSERT(get_stream_chn(or[i], RX, 0, 'i', &orx0_i[i]) && "ORX chan i not found");
                        ASSERT(get_stream_chn(or[i], RX, 0, 'q', &orx0_q[i]) && "ORX chan q not found");
                        if ((scfg.RFICseq>>i)&0x01)
                        {
                            iio_channel_enable(orx0_i[i]);
                            iio_channel_enable(orx0_q[i]);
                        }
                    }
                }
            }
            else
            {
                phy[i] = get_phy_dev(ctx, devicead9009name[i]);
                if(phy[i] == NULL)
                    break;

                if (scfg.tx_on) 
                {
                    ASSERT(get_stream_chn(tx[i], TX, 0, 0, &tx0_i[i]) && "TX chan i not found");
                    ASSERT(get_stream_chn(tx[i], TX, 1, 0, &tx0_q[i]) && "TX chan q not found");
                    ASSERT(get_stream_chn(tx[i], TX, 2, 0, &tx1_i[i]) && "TX chan i not found");
                    ASSERT(get_stream_chn(tx[i], TX, 3, 0, &tx1_q[i]) && "TX chan q not found");
                    if ((scfg.RFICseq>>i)&0x01)
                    {
                        syslog_level(LOG_LEVEL1, "chip %d two tx channels are enabled!\r\n", i);
                        iio_channel_enable(tx0_i[i]);
                        iio_channel_enable(tx0_q[i]);
                        iio_channel_enable(tx1_i[i]);
                        iio_channel_enable(tx1_q[i]);
                    }
                }
                /*when orx is on, then you can not use rx*/
            	if (scfg.rx_on && scfg.orx_on == 0)
                {
                    phy_stream_cfg_adrv9009_ch(phy[i], 0, RX);
                    phy_stream_cfg_adrv9009_ch(phy[i], 1, RX);
                    ASSERT(get_stream_chn(rx[i], RX, 0, 'i', &rx0_i[i]) && "RX chan i not found");
                    ASSERT(get_stream_chn(rx[i], RX, 0, 'q', &rx0_q[i]) && "RX chan q not found");
                    ASSERT(get_stream_chn(rx[i], RX, 1, 'i', &rx1_i[i]) && "RX chan i not found");
                    ASSERT(get_stream_chn(rx[i], RX, 1, 'q', &rx1_q[i]) && "RX chan q not found");
                    if ((scfg.RFICseq>>i)&0x01)
                    {
                        syslog_level(LOG_LEVEL1, "chip %d two rx channels are enabled!\r\n", i);
                        iio_channel_enable(rx0_i[i]);
                        iio_channel_enable(rx0_q[i]);
                        iio_channel_enable(rx1_i[i]);
                        iio_channel_enable(rx1_q[i]);
                    }
                }

                if (scfg.orx_on) 
                {
                    char attrval[40] = "OBS_TX_LO";

                    phy_stream_cfg_adrv9009_ch(phy[i], 2, ORX);
                    phy_stream_cfg_adrv9009_ch(phy[i], 3, ORX);
                    if(obschn0[i] != NULL)
                    {
                        ASSERT_ATTR(iio_channel_attr_write(obschn0[i], "rf_port_select", attrval));
                        ASSERT(get_stream_chn(or[i], RX, 0, 'i', &orx0_i[i]) && "ORX chan i not found");
                        ASSERT(get_stream_chn(or[i], RX, 0, 'q', &orx0_q[i]) && "ORX chan q not found");
                        if ((scfg.RFICseq>>i)&0x01)
                        {
                            iio_channel_enable(orx0_i[i]);
                            iio_channel_enable(orx0_q[i]);
                        }
                    }
                    if(obschn1[i] != NULL)
                    {
                        ASSERT_ATTR(iio_channel_attr_write(obschn1[i], "rf_port_select", attrval));
                        ASSERT(get_stream_chn(or[i], RX, 1, 'i', &orx1_i[i]) && "ORX chan i not found");
                        ASSERT(get_stream_chn(or[i], RX, 1, 'q', &orx1_q[i]) && "ORX chan q not found");
                        if ((scfg.RFICseq>>i)&0x01)
                        {
                            iio_channel_enable(orx1_i[i]);
                            iio_channel_enable(orx1_q[i]);
                        }
                    }
                }

            }
        }
    }
    return;
}

void phy_stream_handle_sig(int sig)
{
    printf("Waiting for process to finish...\n");
    phy_stream_shutdown_iio();
}

/* Map GPIO related memory to virtual memory space*/
static int phy_stream_open_gpio()
{
    mem_base = OpenMemory(&memfd, MEM_BASE, MEM_MAP_SIZE);
    if (mem_base)
    {
        mem_mode_select = mem_base + MEM_MODE_SELECT;
        mem_orx_atten = mem_base + MEM_CHANNEL_SELECT;
        mem_fpga_set = mem_base + MEM_FPGA_SET;
        mem_sync_set = mem_base + MEM_SYNC_SET;
    }
    else
    {
        return -1;
    }
}

/* Unmap GPIO memory*/
static void phy_stream_close_gpio()
{
    if (mem_base)
    {
        printf("* Close GPIO Memory\n");
        if (munmap(mem_base, MEM_MAP_SIZE) == -1)
        {
            printf("Can't unmap memory from user space.\n");
            exit(0);
        }
        mem_base = NULL;
    }
    if (memfd != -1)
    {
        close(memfd);
        memfd = -1;
    }
}

/* Setup GPIO*/
static void phy_stream_setup_gpio()
{
    int reg = 0;

    if(mem_orx_atten != NULL)
    {
        reg = ORX_ATTEN;
        memcpy(mem_orx_atten, &reg, 2); //set ORx attenuation
    }

    if (scfg.tx_on)
    {
        reg = 0x00000000;
        memcpy(mem_sync_set, &reg, 4); //disable synchronization block

        reg = 0x80000000; //open PA power
        if (scfg.enb_FPGA) 
        {
            if (scfg.tx_on)
                reg |= (0x80 << 8);
            else if(scfg.rx_on)
                reg |= 0xa0;
        }
        memcpy(mem_fpga_set, &reg, 4);
    }
    reg = 0;
    memcpy(mem_mode_select, &reg, 1); //0xff22e030
    memcpy(mem_mode_select+1, &reg, 1); //0xff22e031

    if (scfg.tx_on)
    {
    	reg |= scfg.RFICseq;
    	memcpy(mem_mode_select+1, &reg, 1); //0xff22e031
    }
}

static void phy_stream_en_sync_block()
{
	int reg = 3;
	reg |= ((DELTATR) << 8);
	memcpy(mem_sync_set, &reg, 4);
}

/* cleanup and exit; declared in Ad9371_common.h */
void phy_stream_shutdown_iio()
{
    int i;
    printf("* Destroying buffers\n");

    for(i=0; i<ad9xxxchip; i++)
    {
        printf("* Disabling streaming channels\n");
        if (rxbuf[i])
        {
            iio_buffer_destroy(rxbuf[i]);
            rxbuf[i] = NULL;
        }
        if (txbuf[i])
        {
            iio_buffer_destroy(txbuf[i]);
            txbuf[i] = NULL;
        }
        if (rx0_i[i])
        {
            iio_channel_disable(rx0_i[i]);
            rx0_i[i] = NULL;
        }
        if (rx0_q[i])
        {
            iio_channel_disable(rx0_q[i]);
            rx0_q[i] = NULL;
        }
        if (tx0_i[i])
        {
            iio_channel_disable(tx0_i[i]);
            tx0_i[i] = NULL;
        }
        if (tx0_q[i])
        {
            iio_channel_disable(tx0_q[i]);
            tx0_q[i] = NULL;
        }
        if (tx1_i[i])
        {
            iio_channel_disable(tx1_i[i]);
            tx1_i[i] = NULL;
        }
        if (tx1_q[i])
        {
            iio_channel_disable(tx1_q[i]);
            tx1_q[i] = NULL;
        }
    }
    printf("* Destroying context\n");
    if (ctx)
    {
        iio_context_destroy(ctx);
        ctx = NULL;
    }
    exit(0);
}

int phy_stream_rxdma_overflow(int igrp)
{
    int ret;
    uint32_t val;

    if(igrp > ad9xxxchip)
    {
        fprintf(stderr, "<-- read rxdma[%d] status failed %s\n", 
            igrp, strerror(-ret));
        return -1;
    }

    ret = iio_device_reg_read(rx[igrp], 0x80000088, &val);
    if (0 != ret) {
        fprintf(stderr, "<-- read rxdma[%d] status failed %s\n",
            igrp, strerror(-ret)); 
        return -1;
    }
    if (mode_verbo > 1)
        printf("<-- rxdma[%d] status=0x%08x\n", igrp, val);
                                                                                 
    // Clear status bits by writting value back                                  
    if (0 != val) iio_device_reg_write(rx[igrp], 0x80000088, val); 
    if (val & 0x04) {
        if (mode_verbo > 1)
            fprintf(stderr, "<-- rxdma[%d] Overflow detected!\n", igrp);
        return 1;
    }
    return 0;
}

int phy_stream_txdma_underflow(int igrp)
{
    int ret;                                                                     
    uint32_t val;

    ret = iio_device_reg_read(tx[igrp], 0x80000088, &val);
    if (0 != ret) {
        fprintf(stderr, "--> read txdma[%d] status failed %s\n",
            igrp, strerror(-ret));
        return -1;
    }
    if (mode_verbo > 1)
        printf("--> txdma[%d] status=0x%08x\n", igrp, val);
                                                                                 
    // Clear status bits by writting value back
    if (0 != val) iio_device_reg_write(tx[igrp], 0x80000088, val);
    if (val & 0x01) {
        if (mode_verbo > 1)
            fprintf(stderr, "--> txdma[%d] Underflow detected\n", igrp);
        return 1;
    }

    return 0;                                                                    
}

static FILE* phy_stream_analyze_rxfile(char *rxfilename)
{
    FILE* writefd = NULL;

    if(rxfilename == NULL || strlen(rxfilename) == 0)
        return writefd;

    syslog_level(LOG_LEVEL1, "rxfilename is %s\n", rxfilename);

    if (scfg.rx_on)
    {
    	if (!(writefd = fopen(rxfilename, "wb+")))
       {
            fprintf(stderr, "Cannot open output rxfile\n");
            return writefd;
        }
    }
    return writefd;
}

static FILE* phy_stream_analyze_txfile(char *txfilename)
{
    FILE* readfd = NULL;

    if(txfilename == NULL || strlen(txfilename) == 0)
        return readfd;

    syslog_level(LOG_LEVEL1, "txfilename is %s\n", txfilename);
    if (scfg.tx_on)
    {
        if (!(readfd = fopen(txfilename, "rb")))
        {
            fprintf(stderr, "Cannot open input txfile\n");
            return readfd;
        }
        fseek(readfd, 0, SEEK_END);
        scfg.frame_size = ftell(readfd) / 4; //FIXME: preset sample size = 4; a nasty trick is needed to get real-time sample size
        syslog_level(LOG_LEVEL1, "Tx frame size is set to %d samples\n", (int)(scfg.frame_size));
        fseek(readfd, 0, SEEK_SET);
    }
    return readfd;
}

/* create libiio buffer*/
static void phy_stream_create_libiio_buffer(enum iodev direction)
{
    int i;
    for(i=0; i<ad9xxxchip; i++)
    {
        if(direction == TX)
        {
        	if (scfg.tx_on) {
            	txbuf[i] = iio_device_create_buffer(tx[i], scfg.frame_size, scfg.cycle);
            	if (!txbuf[i]) {
                	perror("Could not create TX buffer");
                } else {
                	syslog_level(LOG_LEVEL1, "Open Tx buffer succeeds");
                }
            }
        }

        if(direction == RX)
        {
        	if (scfg.rx_on) {
            	rxbuf[i] = iio_device_create_buffer(rx[i], scfg.frame_size, scfg.cycle);
            	if (!rxbuf[i]) {
                	perror("Could not create RX buffer");
                } else {
                	syslog_level(LOG_LEVEL1, "Open Rx buffer succeeds");
                }
            }
        	else if (scfg.orx_on) {
            	rxbuf[i] = iio_device_create_buffer(or[i], scfg.frame_size, scfg.cycle);
            	if (!rxbuf[i]) {
                	perror("Could not create OR buffer");
                } else {
                	syslog_level(LOG_LEVEL1, "Open ORx buffer succeeds");
                }
            }
        }
    }
}

/* handle iio buffer for Tx*/
static void phy_stream_push_iio_tx_buffer(FILE* readfd)
{
    int i, j;
    char *p_dat, *p_end;
    char *temp = NULL;
    char *temp2 = NULL;

    ptrdiff_t p_inc;
    size_t nbytes_tx;

    temp = malloc(scfg.frame_size * 4);
    temp2 = malloc(scfg.frame_size* 8);

    i = testtxrxchip;
    if(i < ad9xxxchip && txbuf[i])
    {
        //p_dat = iio_buffer_first(txbuf[i], tx0_i[i]);
        p_dat = iio_buffer_first(txbuf[i], tx1_i[i]);
        p_end = iio_buffer_end(txbuf[i]);
        p_inc = iio_buffer_step(txbuf[i]);

        syslog_level(LOG_LEVEL2, "AdiChip %d begin: %x, end: %x, step: %x\n", i, (unsigned int)p_dat, (unsigned int)p_end, (int)p_inc);
        fread(temp, 4, scfg.frame_size, readfd);
        syslog_level(LOG_LEVEL1, "AdiChip %d Read %d bytes from %d\n", i, (int)scfg.frame_size, (int)readfd);

        if ((scfg.RFICseq>>i)&0x01)
        {
            for(j=0; j<scfg.frame_size; j++)
            {
                memcpy(temp2+j*8, temp+j*4, 4);
                memcpy(temp2+j*8+4, temp+j*4, 4);
            }
            memcpy(p_dat, temp2, scfg.frame_size*8);
        } 
        else
        {
            memcpy(p_dat, temp, p_inc * scfg.frame_size);
        }

        nbytes_tx = iio_buffer_push(txbuf[i]);
        if (nbytes_tx < 0) 
        {
            fprintf(stderr, "AdiChip %d Error pushing buf %d\n", i, (int)nbytes_tx);
            if (temp)
                free(temp);
            if (temp2)
                free(temp2);
            return;
        } 
        else
        {
            syslog_level(LOG_LEVEL1, "AdiChip %d push %d bytes of data\n", i, (int)nbytes_tx);
        }
    }
    if (temp)
        free(temp);
    if (temp2)
        free(temp2);
}

static void phy_stream_refill_iio_rx_buffer(FILE* writefd)
{
    int i;
    char *p_dat;
    char *p_end;
    ptrdiff_t p_inc;
    size_t count = 0;
    size_t nbytes_rx = 0;
    size_t bytes_count[MAX_CHIP_NUM] = {0,0,0,0};

    //printf("frame number is %d\n", scfg.frame_num);
    memset(bytes_count, 0x00, sizeof(bytes_count));
    while(count < scfg.frame_num)
    {
        i = testtxrxchip;
        if(i < ad9xxxchip && rxbuf[i])
        {
            nbytes_rx = iio_buffer_refill(rxbuf[i]);
            if (nbytes_rx < 0)
            {
                fprintf(stderr, "Error refilling buf %d\n", (int)nbytes_rx);
                count = scfg.frame_num;
                break;
            }
            else
            {
                syslog_level(LOG_LEVEL1, "Refill %d bytes of data\n", (int)nbytes_rx);
            }

            bytes_count[i] += nbytes_rx;

            p_inc = iio_buffer_step(rxbuf[i]);
            p_end = iio_buffer_end(rxbuf[i]);
            p_dat = iio_buffer_first(rxbuf[i], rx0_i[i]);

            fwrite(p_dat, p_inc, scfg.frame_size, writefd);
            fflush(writefd);
        }
        count++;            
    }
}

/* main function */
int main (int argc, char **argv)
{
    int tmpsize = 0;
    FILE* readfd = NULL;
    FILE* writefd = NULL;
    char rxfilename[MAXLEN_FILENAME];
    char txfilename[MAXLEN_FILENAME];

    memset(rxfilename, 0x00, sizeof(rxfilename));
    memset(txfilename, 0x00, sizeof(txfilename));

    // Listen to ctrl+c and ASSERT
    signal(SIGINT, phy_stream_handle_sig);

    //parse input command
    phy_stream_get_args(argc, argv, &scfg, rxfilename, txfilename, MAXLEN_FILENAME);

    phy_stream_init_global();
    phy_stream_getchip_info();

    //find the specific AD9371 chip and channel selected;
    phy_stream_cfg_axi();

    if(ifad9371 == 1 && (strlen(rxfilename) || strlen(txfilename)))
    {
        phy_stream_open_gpio();
        phy_stream_setup_gpio();
        if (scfg.enb_DMA)
        {
            if (scfg.tx_on && strlen(txfilename)) 
            {
                readfd = phy_stream_analyze_txfile(txfilename);
                if(readfd != NULL)
                {
                    phy_stream_create_libiio_buffer(TX);

                    phy_stream_push_iio_tx_buffer(readfd);

                    //sychronization block is enabled AFTER tx buffer is ready
                    phy_stream_en_sync_block();
                }
            }

            if (scfg.rx_on) 
            {
                puts("rx is on");
                if(!strlen(rxfilename))
                {
                    tmpsize = sizeof(rxfilename)-1;
                    if(tmpsize > strlen(default_dst))
                        tmpsize = strlen(default_dst);
                    memcpy(rxfilename, default_dst, tmpsize);
                }
                writefd = phy_stream_analyze_rxfile(rxfilename);
                if(writefd != NULL)
                {
                    phy_stream_create_libiio_buffer(RX);

                    //synchronization block is enabled BEFORE rx buffer is ready
                    phy_stream_en_sync_block();

                    phy_stream_refill_iio_rx_buffer(writefd);
                }
            }
        }
        pause();
    }
    else if(ifad9371 == 0)
    {
        if (scfg.tx_on && strlen(txfilename)) 
        {
            readfd = phy_stream_analyze_txfile(txfilename);
            if(readfd != NULL)
            {
                phy_stream_create_libiio_buffer(TX);
                phy_stream_push_iio_tx_buffer(readfd);
            }
        }

        if (scfg.rx_on) 
        {
            puts("rx is on");
            if(!strlen(rxfilename))
            {
                tmpsize = sizeof(rxfilename)-1;
                if(tmpsize > strlen(default_dst))
                    tmpsize = strlen(default_dst);
                memcpy(rxfilename, default_dst, tmpsize);
            }
            writefd = phy_stream_analyze_rxfile(rxfilename);
            if(writefd != NULL)
            {
                phy_stream_create_libiio_buffer(RX);
                phy_stream_refill_iio_rx_buffer(writefd);
            }
        }
        if(scfg.cycle == 1)
            pause();
    }

    if (writefd != NULL)
    {
        fclose (writefd);
        writefd = NULL;
    }
    if(readfd != NULL)
    {
        fclose(readfd);
        readfd = NULL;
    }
    if(ifad9371 == 1)
        phy_stream_close_gpio();
    phy_stream_shutdown_iio();

    return 0;
}

