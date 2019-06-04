/*
 * libiio - AD9371 IIO streaming application
 * modified based on AD9371 IIO streaming example
 *
 *
 * Copyright (C) 2014 IABG mbH
 * Original Author: Michael Feilen <feilen_at_iabg.de>
 * Modified Author: Junyi Zhang <jun-yi.zhang_at_foxconn.com>
 * Copyright (C) 2017 Analog Devices Inc.
 * Copyright (C) 2017 Foxconn 5G Research Institute
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
#include <string.h>

#include <signal.h>
#include <stdio.h>
#include <ctype.h>

#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

#include <sys/types.h>  
#include <sys/socket.h>	/* socket() */
#include <sys/ioctl.h>	/* ioctl() */
#include <arpa/inet.h>	/* htons/htonl() */ 
#include <pthread.h>
#include <stdlib.h>

//#include <iio-private.h>

#include "t_mykonos.h"
#include "t_mykonos_gpio.h"
#include "ad9371profile_20MHz.h"
#include <time.h>
//#include "comp.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <getopt.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

#define ORX_ATTEN 0x3000
#define CTRL_MEM 0xff220000
#define DMA_RESET 0xe050
#define SYNC_ENB 0xe040
#define DELTATR 163
#define QUEUE_SIZE 6        
#define SERVER_PORT 8000    //服务器的端口号
#define SERVER_IP  "10.180.8.21"   //服务器的IP地址
static int g_rxloop_stop;
static int ret_rx;

//GPIO Control Memeory Address
#define TXMEM 0xff220000
#define OFFSET1 0xe030 //mode number (4bits) + division (4bits)
#define OFFSET2 0xe020 //Select channel
#define MEM_MAP_SIZE 0x10000
#define MEM_MAP_MASK MEM_MAP_SIZE-1

/* helper macros */
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))

#define ASSERT(expr) { \
    if (!(expr)) { \
        (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
        (void) abort(); \
    } \
}

#define ASSERT_ATTR(expr) { \
    if (expr < 0) { \
        (void) fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
        (void) abort(); \
    } \
}

/* RX is input, TX is output */
enum iodev { RX, TX, ORX };
typedef struct timespec timespec;

/* static scratch mem for strings */
static char tmpstr[64];

struct iio_context *ctx   = NULL;
struct iio_channel *rx0_i = NULL;
struct iio_channel *rx0_q = NULL;
struct iio_channel *tx0_i = NULL;
struct iio_channel *tx0_q = NULL;
struct iio_channel *tx1_i = NULL;
struct iio_channel *tx1_q = NULL;
struct iio_buffer  *rxbuf = NULL;
struct iio_buffer  *txbuf = NULL;

void *mapped_dev_base = NULL;
int memfd = -1;
int runtime = 0;

typedef struct tx_rx_config{
    long long rx_gain;
    long long tx_atten;
    bool tx_qtracking;
    bool tx_loltracking;
    bool rx_qtracking;
    long long tx_freq;
    long long rx_freq;
    bool enb_txFPGA;
    bool enb_rxFPGA;
    bool loop;
    bool cycle;
    bool tcal;
    bool rcal;
    uint8_t RFICseq;
}tx_rx_config;


typedef struct stream_cfg{
    char *src_fn;
    char *dst_fn;
    bool tx_on;
    bool rx_on;
    bool no_stop;
    size_t frame_num;
    bool tx1;
    uint8_t rx1;
    bool tx2;
}stream_cfg;


static void usage(void)
{
	printf("Options:\n");
	printf("\t-tx1 or -tx2\t\tEnable tx1 or tx2\n");
	printf("\t-rx1 or -rx2\t\tEnable rx1 or rx2\n");
	printf("\t-enb\t\tEnable FPGA BBP modules\n");
	printf("\t-cyc\t\tEnable cycle transmission\n");
	printf("\t-txat\tTx attenuation (dB)\n");
	printf("\t-rxgain\t\tRx gain (dB)\n");
	printf("\t-txqt\t\tTx quadrature tracking enable (bool)\n");
	printf("\t-rxqt\t\tRx quadrature tracking enable (bool)\n");
	printf("\t-txlolt\t\tTx LO leakage tracking enable (bool)\n");
	printf("\t-txfq\t\tTx Frequency (GHz)\n");
	printf("\t-rxfq\t\tRx Frequency (GHz)\n");
	printf("\t-src\t\tsource data file path\n");
	printf("\t-dst\t\tdstination data file path\n");
           
}


int  get_args(int argc, char **argv, tx_rx_config *cfg, stream_cfg *scfg)
{	
	
	bool case_flag = true;
	const char *optstring="d:g:a:k:n:m:o:p:q:0h";  
	int c,index;
	int tx_noarg = 0;
	int nrx_noarg = 0;
	int tx2_noarg = 0;
	//for(int i = 0; i <= argc ; i++ )
		//printf("argv [%d] = %s \n",i,argv[i]);
	struct option opts[]={
			  {"tx",no_argument,0,0},//0  
              {"nrx",no_argument,0,0},  
              {"tx2",no_argument,0,0}, 
              {"tx1",no_argument,0,0},   
              {"rx2",no_argument,0,0},  
              {"rx1",no_argument,NULL,0},//5  
              {"orx",no_argument,0,0},
              {"stop",no_argument,0,0},  
              {"ten",no_argument,0,0},  
              {"ren",no_argument,0,0},  
              {"loop",no_argument,0,0}, //10 
              {"cyc",no_argument,0,0},  
              {"rcal",no_argument,0,0},  
              {"tcal",no_argument,0,0},
              {"txat",required_argument,0,'a'},
              {"rxgain",required_argument,0,'g'},//15
              {"num",required_argument,0,'n'}, 
              {"txqt",required_argument,0,'t'},
              {"lo",required_argument,0,0},
              {"rxqt",required_argument,0,0},
              {"txfq",required_argument,0,0},//20
              {"rxfq",required_argument,0,'f'},
              {"fno",required_argument,0,'o'}, 
              {"src",required_argument,0,0}, //23
              {"dst",required_argument,0,'d'},
              {"rt",required_argument,0,0},   
              {"help",no_argument,0,0},//26
              {0,0,0,27}};
	printf("Enter get_args() fountion\n");
	int cnt = 1 ;
	while(1/*(c=getopt_long_only(argc,argv,optstring,opts,&index))!=-1*/)  
	{  
		c=getopt_long_only(argc,argv,optstring,opts,&index);
		//printf("argv [%d] = %s, c = [%c]\n",index,argv[cnt++],c);
		//printf("opt = %c\n", c);
       
       // printf("option_index = %d\n",index);
		if (c == -1)
		{
			//printf("index = [%d], c = [%d]\n",index,c);
			
			break;
		}
		printf("opt = %c   index = %d  argv[argc%d] =%s\n", c , index ,cnt,argv[cnt]);
		cnt++;
		if(optarg)
        {
            printf("arguis %s\n", optarg);
			cnt++;
        }
		//printf("optarg = %s  ",optarg);
        //printf("optind = %d  ",optind);
       // printf("argv[optind] =%s\n", argv[optind]);
	    switch(index){  
			case 0:
				scfg->tx_on = 1;
				break;  
			case 1: 
				scfg->rx_on = 0; 
			    break;  
			case 2:
			  	 scfg->tx2 = 1;
			  	 break ;
			case 3:  
			    scfg->tx1 = 1;  
			    break;   
			case 4:
				scfg->rx1 = 0;
				scfg->rx_on = 1;
				break;
			case 5:
		    	scfg->rx1 = 1;
				scfg->rx_on = 1;
				break;
			case 6:
		    	scfg->rx1 = 2;
				scfg->rx_on = 1;
				break;
			case 7:  
			    scfg->no_stop = 0;  
			    break;   
			case 8:
				cfg->enb_txFPGA = 1;
				break;  
			case 9:  
			    cfg->enb_rxFPGA = 1;  
			    break;
			case 10:  
			    cfg->loop = 1;  
			    break; 
			case 11:  
		        cfg->cycle = 1;  
		        break;
			case 12:  
		        cfg->rcal = 1;  
		        break; 
			case 13:  
		        cfg->tcal = 1;  
		        break;
			case 14:  
		        cfg->tx_atten = -atof(optarg); 
		        break;
			case 15:
				cfg->rx_gain = atof(optarg);
				scfg->rx_on = 1;
		        break;
			case 16:
				cfg->RFICseq = atoi(optarg);
		        break;
			case 17:
				cfg->tx_qtracking = (atoi(optarg) > 0);
		        break;
			case 18:
				cfg->tx_loltracking = (atoi(optarg) > 0);
		        break;
			case 19:
				cfg->rx_qtracking = (atoi(optarg) > 0);
		        break;
			case 20:
				cfg->tx_freq = GHZ(atof(optarg));
		        break;
			case 21:
				 cfg->rx_freq = GHZ(atof(optarg));
			     break;
			case 22:
				 scfg->frame_num = atoi(optarg);
				 scfg->rx_on = 1;
			     break;
			case 23:
				 scfg->src_fn = optarg;
				 scfg->tx_on = 1;
				 scfg->no_stop = 1;
				 scfg->rx_on = 0;
			     break;
			case 24:
				scfg->dst_fn = optarg;
			    break;
			case 25:
				runtime = atof(optarg);
			    break;
			case 26:
				case_flag = false;
				usage(); 
				exit( EXIT_FAILURE );
			    break;
			default:  
				case_flag = false;
			    fprintf(stderr, "ERROR: unrecognised command line option\n");
				exit(EXIT_FAILURE);
			    break;  
			}  
	} 
	printf("End parser argv \n");
	if( case_flag ) return 0;  
	else return -1;
} 
#if 1

/* cleanup and exit */
void shutdown_gpio()
{
    if (mapped_dev_base){
        printf("* Close GPIO Memory\n");
        if (munmap(mapped_dev_base, MEM_MAP_SIZE) == -1)
        {
            printf("Can't unmap memory from user space.\n");
            exit(0);
        }
    }
    if (memfd != -1)
        close(memfd);

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
bool cfg_ad9371_streaming_ch(struct iio_context *ctx, struct tx_rx_config *cfg, int chid, int tx1rx0)
{
    struct iio_channel *chn = NULL;
    double gain, atten;
    long long val;

    if (tx1rx0){
        // Configure phy and lo channels
        printf("* Acquiring AD9371 phy TX channel %d\n", chid);
        if (!get_phy_chan(ctx, 1, chid, &chn)) {	return false; }

        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->tx_atten));
        ASSERT_ATTR(iio_channel_attr_write_bool(chn, "quadrature_tracking_en", cfg->tx_qtracking));
        ASSERT_ATTR(iio_channel_attr_write_bool(chn, "lo_leakage_tracking_en", cfg->tx_loltracking));

        iio_channel_attr_read_double(chn, "hardwaregain", &atten);
        printf("Channel %s Tx attenuation is %f\n", iio_channel_get_name(chn), atten);
        // Configure LO channel
        printf("* Acquiring AD9371 TX lo channel\n");
        if (!get_lo_chan(ctx, 1, &chn)) { return false; }
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "TX_LO_frequency", cfg->tx_freq));
        iio_channel_attr_read_longlong(chn, "TX_LO_frequency", &val);
        printf("Tx frequency is %lld\n",val);

        printf("* Acquiring AD9371 ORx lo channel\n");
        chn = iio_device_find_channel(get_ad9371_phy(ctx), get_ch_name("altvoltage", 2), true);
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "RX_SN_LO_frequency", cfg->tx_freq));
        iio_channel_attr_read_longlong(chn, "RX_SN_LO_frequency", &val);
        printf("OBS frequency is %lld\n",val);
    }

    else {

        printf("* Acquiring AD9371 phy RX channel %d\n", chid);
        if (!get_phy_chan(ctx, 0, chid, &chn)) {	return false; }

        ASSERT_ATTR(iio_channel_attr_write_double(chn, "hardwaregain", cfg->rx_gain));
        ASSERT_ATTR(iio_channel_attr_write_bool(chn, "quadrature_tracking_en", cfg->rx_qtracking));

        iio_channel_attr_read_double(chn, "hardwaregain", &gain);
        printf("Channel %s Rx gain is %f\n", iio_channel_get_name(chn), gain);

        // Configure LO channel
        printf("* Acquiring AD9371 RX lo channel\n");
        if (!get_lo_chan(ctx, 0, &chn)) { return false; }
        ASSERT_ATTR(iio_channel_attr_write_longlong(chn, "RX_LO_frequency", cfg->rx_freq));
        iio_channel_attr_read_longlong(chn, "RX_LO_frequency", &val);
        printf("Rx frequency is %lld\n",val);
    }
    return true;
}

long long timediff(timespec a, timespec b){
    return (1000000000*(b.tv_sec-a.tv_sec) + b.tv_nsec-a.tv_nsec );
}

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


#endif


tx_rx_config cfg =
{
	0, //double rx_gain;
	-26, //double tx_atten;
	0, //bool tx_qtracking;
	0, //bool tx_loltracking;
	0, //bool rx_qtracking;
	GHZ(3.5), //long long tx_freq;
	GHZ(3.5), //long long rx_freq;
	0, //not enable tx FPGA processing
	0, //not enable rx FPGA processing
	0, //not enable loopback
	0, //not cycle iio buffer
	0, //not doing tx calibration
	0, //not doing rx calibration
	2, //using No.2 AD9371
};

char default_dst[] = "./rx_data.bin";

stream_cfg scfg =
{
	NULL, //src file path
	default_dst, //dst file path
	0, //tx is off;
	0, //rx is off;
	0, //non-stop is disabled
	10, //10 frames
	0, //not using tx1
	0, //not using rx1
	0 //not using tx2
};

void sighandler_abort(int signo)
{
  g_rxloop_stop = 1; 
  if (SIGINT != signo)
	  return;

  printf("\nWaiting for process to finish...\n");
  
}

static char* myStrtok_origin(char* str_arr,const char* delimiters,char**temp_str)
{
  //定义一个指针来指向待分解串
  char*b_temp;
  /*
  * 1、判断参数str_arr是否为空，如果是NULL就以传递进来的temp_str作为起始位置；
  * 若不是NULL，则以str为起始位置开始切分。
  */
  if(str_arr == NULL)
  {
	  str_arr =*temp_str;
  }
  //2、跳过待分解字符串
  //扫描delimiters字符开始的所有分解符
  str_arr += strspn(str_arr, delimiters);
  //3、判断当前待分解的位置是否为'\0'，若是则返回NULL，否则继续
  if(*str_arr =='\0')
  {
	  return NULL;
  }
  /*
  * 4、保存当前的待分解串的指针b_temp，调用strpbrk()在b_temp中找分解符，
  * 如果找不到，则将temp_str赋值为待分解字符串末尾部'\0'的位置，
  * b_temp没有发生变化；若找到则将分解符所在位置赋值为'\0',
  * b_temp相当于被截断了，temp_str指向分解符的下一位置。
  */
  b_temp = str_arr;
  str_arr = strpbrk(str_arr, delimiters);
  if(str_arr == NULL)
  {
	  *temp_str = strchr(b_temp,'\0');
  }
  else
  {
	  *str_arr ='\0';
	  *temp_str = str_arr +1;
  }
  //5、函数最后部分无论找没找到分解符，都将b_temp返回。
  return b_temp;
}

//使用myStrtok来简化myStrtok_origin函数
static char* myStrtok(char* str_arr,const char* delimiters)
{
  static char*last;
  return myStrtok_origin(str_arr, delimiters,&last);
}

#if 1
void reset_cfg()
{	
	cfg.rx_gain = 0;
	cfg.tx_atten = -26;
	cfg.tx_qtracking   = 0 ;
	cfg.tx_loltracking = 0 ;
	cfg.rx_qtracking   = 0;
	cfg.tx_freq = GHZ(3.5);
	cfg.rx_freq = GHZ(3.5);
	cfg.enb_txFPGA = 0,
	cfg.enb_rxFPGA = 0;
	cfg.loop = 0;
	cfg.cycle = 0;
	cfg.tcal = 0;
	cfg.rcal = 0;
	cfg.RFICseq = 2;
		
	scfg.src_fn = NULL;
	scfg.dst_fn = default_dst ;
	scfg.tx_on = 0;
	scfg.rx_on = 0;
	scfg.no_stop = 0;
	scfg.frame_num = 10;
	scfg.tx1 = 0;
	scfg.rx1 = 0;
	scfg.tx2 = 0;  


   /* MEMSET_ZERO(rx0_i, struct iio_channel *, 1);
    MEMSET_ZERO(rx0_q, struct iio_channel *, 1);
    MEMSET_ZERO(tx0_i, struct iio_channel *, 1);
    MEMSET_ZERO(tx0_q, struct iio_channel *, 1);
	MEMSET_ZERO(tx1_i, struct iio_channel *, 1);
    MEMSET_ZERO(tx1_q, struct iio_channel *, 1);
    MEMSET_ZERO(txbuf, struct iio_buffer *, 1);
    MEMSET_ZERO(rxbuf, struct iio_buffer *, 1);
	ctx	 = NULL;
	*/
	
}

//static int cfg_ad9371_alg( tx_rx_config cfg, stream_cfg scfg)
static int cfg_ad9371_alg()
{
  
  uint8_t rfgpio1, rfgpio2;
  rfgpio1 = 0x50 + cfg.RFICseq*2-2;
  rfgpio2 = 0x50 + cfg.RFICseq*2-1;

  void *ctrl1, *ctrl2, *dmactl, *sync_enb;
  mapped_dev_base = NULL ;
  dmactl = NULL;
  sync_enb = NULL;
  
   //open GPIO for FPGA control
  OpenMemory(TXMEM, MEM_MAP_SIZE);
  ctrl1 = mapped_dev_base + OFFSET1;
  ctrl2 = mapped_dev_base + OFFSET2;

  dmactl = mapped_dev_base + DMA_RESET;
  sync_enb = mapped_dev_base + SYNC_ENB;

  if(scfg.tx_on == 1 && scfg.src_fn == NULL){
	  printf("Need source data file for TX\n");
	  exit(0);
  }

  // Streaming devices
  struct iio_device *tx = NULL;
  struct iio_device *rx = NULL;
  struct iio_device *orx = NULL;

  // Listen to ctrl+c and ASSERT
  signal(SIGINT, sighandler_abort);

  printf("* Acquiring IIO context\n");
  ASSERT((ctx = iio_create_default_context()) && "No context");
  //ASSERT((ctx = iio_create_local_context()) && "No context");
  printf("Get %d devices.\n",iio_context_get_devices_count(ctx));
  ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

  struct iio_device *phy;
  phy = get_ad9371_phy(ctx);
  printf("* Acquiring AD9371 streaming devices\n");
  ASSERT(get_ad9371_stream_dev(ctx, TX, &tx) && "No tx dev found");
  ASSERT(get_ad9371_stream_dev(ctx, RX, &rx) && "No rx dev found");
  ASSERT(get_ad9371_stream_dev(ctx, ORX, &orx) && "No rx dev found");

  struct iio_channel *obschn = NULL;
  get_phy_chan(ctx, RX, 2, &obschn);
  ASSERT(obschn!=NULL);
  char attrval[] = "INTERNALCALS";
  ASSERT_ATTR(iio_channel_attr_write(obschn, "rf_port_select", attrval));
  
  int reg, reg_tmp;
  uint8_t tmp;
  printf("scfg.rx_on = %d\n",scfg.rx_on);
  printf("scfg.tx_on = %d\n",scfg.tx_on);
  if (scfg.tx_on){
	  reg = 0;
	  memcpy(sync_enb, &reg, 4);

	  reg = 0x80000000;
	  memcpy(dmactl, &reg, 4);
	  //memset(dmactl+1,0,1);
	  sleep(1);
	
	  if (cfg.enb_txFPGA){
		  reg |= 0x80 << 8; //0x84 for 0309 image
	  }
	  if (cfg.tcal)
		  //reg |= (0xc8 << 8);
		  reg |= (0xc0<< 8);
	  //reg |= 0x80; //FIXME: for temperary use 0xcc
	  printf("Register value is %x\n", reg);
	  memcpy(dmactl, &reg, 4);
	
	  printf("* Configuring AD9371 for streaming\n");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 0, 1) && "TX port 0 not found");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 1, 1) && "TX port 1 not found");

	  bool val;
	  iio_device_debug_attr_write_bool(phy,"loopback_tx_rx", cfg.loop);
	  iio_device_debug_attr_read_bool(phy,"loopback_tx_rx", &val);
	  printf("Tx_RX loopback is %d\n",val);
  }

  if (scfg.rx_on){
	  memcpy(&reg, dmactl, 4);
	  if (cfg.rcal)
		  reg |= 0xc0;
	  else{
		  reg &= ~(0xcc);
	  }
	  if (cfg.enb_rxFPGA)
		  reg |= 0xa0;
	  else
		  reg &= ~(0xa0);

	  printf("Register value is %x\n", reg);
	  memcpy(dmactl, &reg, 4);

	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 0, 0) && "RX port 0 not found");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 1, 0) && "RX port 1 not found");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 2, 0) && "ORX port 0 not found");
  }


  //iio_device_debug_attr_write_longlong(phy,"adi,tx-settings-tx1-atten_mdb", -cfg.tx_atten*1000);
  //iio_device_debug_attr_write_longlong(phy,"adi,tx-settings-tx2-atten_mdb", -cfg.tx_atten*1000);

  /*bool temp;
  struct iio_channel *tx_dds= iio_device_find_channel(tx, "altvoltage0", true);
  printf("TX device name is %s\n", iio_device_get_name(tx));
  printf("DDS channel name is %s\n", iio_channel_get_name(tx_dds));
  iio_channel_attr_read_bool(tx_dds,"raw",&temp);
  printf("DDS channel raw attr is %d\n", temp);
  ASSERT_ATTR(iio_channel_attr_write_bool(tx_dds, "raw", false));

  temp = iio_channel_is_enabled(tx0_q);
  printf("DDS channel enable is %d\n", temp);
  iio_channel_enable(tx_dds);
  temp = iio_channel_is_scan_element(tx0_q);
  printf("DDS channel enable is %d\n", temp);*/


  size_t tx_samp_num;

  //Open source and destination file
  FILE* src_fd = NULL, *dst_fd = NULL;
  //char* buf = malloc(tx_samp_num * tx_samp_size);
  if (scfg.tx_on){
	  printf("src file is %s\n", scfg.src_fn);
	  if(!(src_fd = fopen(scfg.src_fn, "rb"))){
		  printf("Cannot open src file\n");
		  shutdown_gpio();
		  return -1;
	  }
	  //fread(buf, tx_samp_size, tx_samp_num, src_fd);
	  //fclose(src_fd);
	  fseek(src_fd, 0, SEEK_END);
	  tx_samp_num = ftell(src_fd) / 4; //FIXME: preset sample size = 4; a nasty trick is needed to get real-time sample size
	  fseek(src_fd, 0, SEEK_SET);
  }
  if (scfg.rx_on){
	  printf("dst file is %s\n", scfg.dst_fn);
	  if(!(dst_fd = fopen(scfg.dst_fn, "wb+")))
		  printf("Cannot open dst file\n");
  }
  //Create tx and rx buffer
  size_t rx_samp_num = 5*30720;//10*1024;
  printf("scfg.rx_on = %d\n",scfg.rx_on);
  if (scfg.rx_on){
  		printf("scfg.rx1 = %d\n",scfg.rx1);
	  if(scfg.rx1==1){
		  ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 0, 'i', &rx0_i) && "RX chan i not found");
		  ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 0, 'q', &rx0_q) && "RX chan q not found");
		  memcpy(&tmp, ctrl1, 1);
		  if (tmp == rfgpio1){
			  memset(ctrl1,rfgpio2,1);
			  printf("Set register to %d\n",rfgpio2);
		  }
	  }else if (scfg.rx1 == 0){
		  ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 1, 'i', &rx0_i) && "RX chan i not found");
		  ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 1, 'q', &rx0_q) && "RX chan q not found");
		  memcpy(&tmp,ctrl1, 1);
		  if (tmp == rfgpio2){
			  memset(ctrl1,rfgpio1,1);
			  printf("Set register to %d\n",rfgpio1);
		  }
	  }else if (scfg.rx1 == 2){
		  char attrval[] = "ORX1_TX_LO";
		  //char attrval[128];
		  //ASSERT_ATTR(iio_channel_attr_read(obschn, "rf_port_select", attrval, 128));
		  ASSERT_ATTR(iio_channel_attr_write(obschn, "rf_port_select", attrval));
		  //puts(attrval);
		  ASSERT(get_ad9371_stream_ch(ctx, RX, orx, 0, 'i', &rx0_i) && "RX chan i not found");
		  ASSERT(get_ad9371_stream_ch(ctx, RX, orx, 0, 'q', &rx0_q) && "RX chan q not found");
		  memcpy(&tmp,ctrl1, 1);
		  if (tmp == rfgpio2){
			  memset(ctrl1,rfgpio1,1);
			  printf("Set register to %d\n",rfgpio1);
		  }
	  }
	  iio_channel_enable(rx0_i);
	  iio_channel_enable(rx0_q);
	  printf("* Creating Rx buffers\n");
	  if (scfg.rx1 != 2)
		  rxbuf = iio_device_create_buffer(rx, rx_samp_num, false);
	  else
		  rxbuf = iio_device_create_buffer(orx, rx_samp_num, false);
	  if (!rxbuf) {
		  perror("Could not create RX buffer");
		  shutdown_gpio();
	  }

	  memcpy(&reg, sync_enb, 4);
	  reg_tmp = 0;
	  memcpy(&reg_tmp, sync_enb, 4);
	  reg |= 3;
	  reg &= ~(0xff << 8);
	  reg |= ((DELTATR) << 8);//+2048+160
	  memcpy(sync_enb, &reg, 4);
  }    

  reg = ORX_ATTEN;
  memcpy(ctrl2, &reg, 2); //set ORx attenuation

  if (scfg.tx_on){
	  if (scfg.tx1){
		  ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 0, 0, &tx0_i) && "TX chan i not found");
		  ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 1, 0, &tx0_q) && "TX chan q not found");
		  memset(ctrl1, rfgpio1, 1);
		  //memset(ctrl1, 0x21, 1);
		  iio_channel_enable(tx0_i);
		  iio_channel_enable(tx0_q);
	  }
	  if (scfg.tx2){
		  ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 2, 0, &tx1_i) && "TX chan i not found");
		  ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 3, 0, &tx1_q) && "TX chan q not found");
		  memset(ctrl1, rfgpio2, 1); //0x51 for No.1 AD9371, 0x53 for No.2
		  iio_channel_enable(tx1_i);
		  iio_channel_enable(tx1_q);
	  }

	  printf("* Creating Tx buffers\n");
	  //printf("%x, %d, %d\n",tx, tx_samp_num, cfg.cycle );
	  //printf("%d\n",cfg.cycle);
	  txbuf = iio_device_create_buffer(tx, tx_samp_num, cfg.cycle);
	  if (!txbuf) {
		  perror("Could not create TX buffer");
		  shutdown_gpio();
	  }
	  //ssize_t tx_samp_size = iio_device_get_sample_size(tx);
	  //printf("%d bytes \n", tx_samp_size);
  }

  // Schedule TX buffer
  ssize_t nbytes_tx, nbytes_rx;
  char *p_dat, *p_end;
  ptrdiff_t p_inc;

  if (scfg.tx_on){
	  if (scfg.tx1)
		  p_dat = iio_buffer_first(txbuf, tx0_i);
	  else
		  p_dat = iio_buffer_first(txbuf, tx1_i);
	  p_end = iio_buffer_end(txbuf);
	  p_inc = iio_buffer_step(txbuf);

	  printf("begin: %x, end: %x, step: %x\n", (uint) p_dat,(uint) p_end, p_inc);

	  /*p_dat = iio_buffer_first(txbuf, tx1_i);
	  p_end = iio_buffer_end(txbuf);
	  p_inc = iio_buffer_step(txbuf);
	  printf("begin: %x, end: %x, step: %x\n", (uint) p_dat,(uint) p_end, p_inc);*/

	  char *temp=NULL,*temp2=NULL;
	  temp = malloc(tx_samp_num * 4);
	  if (src_fd){
		  fread(temp, 4, tx_samp_num, src_fd);
		  printf("Read %d bytes from %s\n", tx_samp_num*p_inc, scfg.src_fn);

	  }
	  if (scfg.tx1 && scfg.tx2){
		  temp2 = malloc(tx_samp_num * 8);
		  for(int i=0; i<tx_samp_num; i++){
			  memcpy(temp2+i*8, temp+i*4, 4);
			  memcpy(temp2+i*8+4, temp+i*4, 4);
		  }
		  memcpy(p_dat, temp2, tx_samp_num*8);
	  }else{
		  memcpy(p_dat, temp, p_inc * tx_samp_num);
	  }
	  //memcpy(p_dat, buf, p_end - p_dat);
	  /*for (int j=0; j < 10; j++) {
		  // Example: swap I and Q
		  const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)
		  const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q);
		  printf("I: %d, Q: %d\n",i, q);
		  p_dat += p_inc;
	  }*/

	  nbytes_tx = iio_buffer_push(txbuf);	  
	  if (nbytes_tx < 0) { printf("Error pushing buf %d\n", (int) nbytes_tx); shutdown_gpio(); }
	  else printf("push %d bytes of data\n", (int) nbytes_tx);

	  //iio_channel_enable(tx0_i);
	  //iio_channel_enable(tx0_q);

	  if (temp)  {free(temp); temp = NULL;}
	  if (temp2) {free(temp2); temp2 = NULL;}

	  usleep(100000);

	  reg = 3;
	  reg |= ((DELTATR) << 8);//+2048+160
	  memcpy(sync_enb, &reg, 4);
	  
	  iio_buffer_destroy(txbuf);
	  txbuf = NULL;
	  reg = 0 ;
	  
  }


  struct timespec t0, t1, t2;
  if (scfg.rx_on){
	  memcpy(&tmp, ctrl1, 1);
	  printf("%x\n",tmp);
	  size_t count = 0, bytes_count = 0;
	  printf("scfg.frame_num = %d\n",scfg.frame_num);
	  while (count < scfg.frame_num)
	  {
		  // Refill RX buffer

		  t2 = t0;
		  clock_gettime(CLOCK_MONOTONIC, &t0);
		  nbytes_rx = iio_buffer_refill(rxbuf);
		  clock_gettime(CLOCK_MONOTONIC, &t1);
		  printf("refill interval %lld us, refill takes %lld us, the throughput is %fMbps\n",
				 timediff(t2,t0)/1000, timediff(t0,t1)/1000,nbytes_rx * 8.0 / timediff(t0,t1)*1000);
		  if (nbytes_rx < 0) { 
		  	printf("Error refilling buf %d\n",(int) nbytes_rx);
		  	shutdown_gpio(); 
		  }
		  
		  //else  printf("Refill %d bytes data\n", (int) nbytes_rx);
		  bytes_count += nbytes_rx;

		  // READ: Get pointers to RX buf and read IQ from RX buf port 0
		  //printf("%s, %d\n", __FILE__, __LINE__);
		  p_inc = iio_buffer_step(rxbuf);
		  p_end = iio_buffer_end(rxbuf);
		  p_dat = iio_buffer_first(rxbuf, rx0_i);
		  //printf("%s, %d\n", __FILE__, __LINE__);
		  //printf("begin: %x, end: %x, step: %x\n", (uint) p_dat,(uint) p_end, p_inc);

		  if(dst_fd){
			  fwrite(p_dat, p_inc, rx_samp_num, dst_fd);
			  //printf("Write %d bytes to %s\n", rx_samp_num*p_inc, scfg.dst_fn);
			  fflush(dst_fd);
		  }

		  /*size_t rx_bytes = 0;
		  for (p_dat = iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc) {
			  // Example: swap I and Q
			  const int16_t i = ((int16_t*)p_dat)[0]; // Real (I)
			  const int16_t q = ((int16_t*)p_dat)[1]; // Imag (Q)
			  //((int16_t*)p_dat)[0] = q;
			  //((int16_t*)p_dat)[1] = i;
			  fwrite(p_dat,4,1,dst_fd);
			  if(rx_bytes < 40)
				  printf("I: %d, Q: %d\n",i, q);
			  rx_bytes += 4;
		  }
		  printf("%x bytes received\n", rx_bytes);

		  // WRITE: Get pointers to TX buf and write IQ to TX buf port 0
		  /*p_inc = iio_buffer_step(txbuf);
		  p_end = iio_buffer_end(txbuf);
		  for (p_dat = iio_buffer_first(txbuf, tx0_i); p_dat < p_end; p_dat += p_inc) {
			  // Example: fill with zeros
			  // 14-bit sample needs to be MSB alligned so shift by 2
			  // https://wiki.analog.com/resources/eval/user-guides/ad-fmcomms2-ebz/software/basic_iq_datafiles#binary_format
			  ((int16_t*)p_dat)[0] = 0 << 2; // Real (I)
			  ((int16_t*)p_dat)[1] = 0 << 2; // Imag (Q)
		  }

		  // Sample counter increment and status output
		  nrx += nbytes_rx / iio_device_get_sample_size(rx);
		  ntx += nbytes_tx / iio_device_get_sample_size(tx);
		  printf("\tRX %8.2f MSmp, TX %8.2f MSmp\n", nrx/1e6, ntx/1e6);*/
		  count++;
	  }

	  iio_buffer_destroy(rxbuf);
	  rxbuf = NULL;
	  printf("End receiving %d frames, %d btyes in total\n Destroy rx buffer, rxbuf=%x\n", scfg.frame_num, bytes_count, (uint) rxbuf);
	  if (dst_fd)
		  fclose(dst_fd);
  }

  /*while(1){
	  char cmd[256];
	  double cmd_val;
	  scanf("%s %d", cmd, &cmd_val);
  }*/
  /*if(!scfg.no_stop){
	  printf("Press Ctrl+C to stop transmission\n");
	  pause();
  }*/

  if  (src_fd)
	  fclose(src_fd);
  //if (!scfg.no_stop)
	 // pause();
	if (scfg.tx_on){
	  reg = 0;
	  memcpy(sync_enb, &reg, 4);
	}

	
	printf("* End the rx or tx\n");
  //printf("Tx2 is %s\n ", iio_channel_is_enabled(tx0_i)? "on":"off");
  	return 0;
}
#endif

static void *socket_rx_thread(void *parg)
{   
  /* Listen to ctrl+c and assert */
  signal(SIGINT, sighandler_abort);
  
  int sock = socket(AF_INET,SOCK_STREAM,0);  
  if(sock < 0)	
  { 	
	  perror("scok");  
	  ret_rx = sock;
	  return (void *)&ret_rx;  
  }
  
  struct sockaddr_in addrser;	//服务器的地址结构
  addrser.sin_family = AF_INET;   //设置服务器的协议家族
  addrser.sin_port = htons(SERVER_PORT);  //设置服务器的端口
  addrser.sin_addr.s_addr = inet_addr(SERVER_IP);  //设置服务器的IP地址
  socklen_t addrlen = sizeof(struct sockaddr);	 //得到服务器的地址的大小
  int res ;
  
  //设置接收超时
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 200;/* TODO: How does it affect the throughput/latency? */
  setsockopt(sock ,SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,sizeof(struct timeval));
  
  do{
	  //阻塞直到有连接服务器连接，不然多浪费CPU资源。	  
	  if( res = connect(sock, (struct sockaddr*)&addrser, addrlen) == -1){	  
		  //printf("accept socket error: %s(errno: %d)\n",strerror(errno),errno);
		  if (g_rxloop_stop) goto rx_exit;			  
		  continue;
	  }else{
		  printf("RRU connect server ok.\n");
		  break;
	  }

  }while(1);
  
  do{	  
	   //服务器和客户端进行通信
	  char recvbuf[256];
rx_revedo:
	  res = recv(sock, recvbuf, 256, MSG_DONTWAIT/*0*/);   //客户端先接收服务器发来的消息 	  
#if 0
	  printf("\nRecv From Server Date:%s\n",recvbuf);
	  printf("buf_lenth = [%d]\n",res);
#endif		
	  if (res < 0){

		  if (EINTR == errno){
		  	  
			  if (g_rxloop_stop)  goto rx_exit;
			  else goto rx_revedo;
		  }else if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
		  	  
			  if (g_rxloop_stop)goto rx_exit;
			  else goto rx_revedo;
		  }else{   
			  //fprintf(stderr, "\nerrno=[%d] in socket_rx_thread() ", errno);
			  goto rx_exit;
		  }
	  }  
	  else{  
		  if(1 == res || 0 == res){   
			  printf("Recvbuf has no Date\n");
			  break;
			  //goto rx_revedo;
		  } 		   
		  recvbuf[res-1] = '\0';  //具体看消息格式处理
										
		  if (strcmp(recvbuf, "quit") == 0){  
			  printf("quit!\n");  
			  goto rx_exit; 			  
		  }
#if 1
		  printf("\nParse cmd date\n");
		  printf("Recv From Server CMD:%s\n",recvbuf); 
#endif						
		  int i = 0,j = 0 ;
		  char *split_cmd[100];			  
		  const char *sep = " "; //可按多个字符来分割				  
		  char *p = myStrtok(recvbuf, sep);
		  split_cmd[i++]=  "start";
		  while(p != NULL)//获取命令
		  { 			  
			  split_cmd[i] = p;
			  //printf("split_cmd : i = %d, %s\n",i,split_cmd[i]);
			  //printf("%s\n",p);
			  p = myStrtok(NULL, sep);
			  i++;				
		  }
#if 1						
		  split_cmd[i] ="\0";
		  for(int t =0 ;t < i ;t++ ){
			  printf("split_cmd[ %d], %s\n",t,split_cmd[t]);
		  }
		  printf("%d\n",i);
		  printf("parse cmd end!\n"); 
		  
#endif						
		  res = get_args(i, split_cmd, &cfg, &scfg);//parse_input_command(i, split_cmd, &cfg, &scfg);
		  if(res !=0){
			  send(sock, "quit", strlen("quit") + 1, 0);   //客户端发送消
			  //printf("socket send quit\n");
			  shutdown_gpio();
			  break;
		  }

		  //profile the ad9371
		  //cfg_ad9371_alg(cfg, scfg);

		  cfg_ad9371_alg();
		  shutdown_gpio();
		  reset_cfg();
	  }
	  
  }while(!g_rxloop_stop);
	  
 //printf("leave do while()\n");  
rx_exit:
  g_rxloop_stop = 1;
  close(sock);	//通信结束，关闭客户端的套接字
  ret_rx = res;
  return (void *)&ret_rx; 
}

/* simple configuration and streaming */
int main (int argc, char **argv)
{
	/*****************socket revc data  from	BBU *********/
	int ret;
	pthread_t  pid_rx;
	g_rxloop_stop = 0;

	/*---- wait for thread-terminate */
	ret = pthread_create(&pid_rx, NULL, socket_rx_thread, &g_rxloop_stop);
	if (ret != 0){
	perror("pthread_create() for raws_rx_thread\n");	  
	} 

	pthread_exit(&pid_rx);
	return ret;
}
