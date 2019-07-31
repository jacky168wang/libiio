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

#include "ad9371common_junyi.h"
#include "ad9371stream_junyi.h"

extern unsigned int mode_verbo;

/* declaration of corresponding pointer*/
static int memfd=-1;
static void *mem_base;
static void *mem_orx_atten;
static void *mem_mode_select;
static void *mem_channel_select;
static void *mem_fpga_set;
static void *mem_sync_set;


/* define global variables*/
static struct phy_cfg cfg = {
	  30, //rx_gain;
	  -30, //tx_atten;
	  1, //tracking_cal;
	  GHZ(3.5), //freq;
	  0, //not enable loopback
};

static struct stream_cfg scfg = {
	  NULL, //file path
	  0, //tx is off;
	  0, //rx is off;
	  0, //orx is off;
	  0, //not cycle iio_buffer
	  0, //not enable FPGA
	  0, //not enable DMA
	  30720*20, //sample number in a frame
	  1, //receiving one frame
	  4, //using channel 1 of No.2 AD9371 
  };


/* IIO structs required for streaming */
static struct iio_context *ctx   = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *rx1_i = NULL;
static struct iio_channel *rx1_q = NULL;
static struct iio_channel *orx0_i = NULL;
static struct iio_channel *orx0_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_channel *tx1_i = NULL;
static struct iio_channel *tx1_q = NULL;
static struct iio_buffer  *rxbuf = NULL;
static struct iio_buffer  *txbuf = NULL;

  // Streaming devices
static struct iio_device *tx = NULL;
static struct iio_device *rx = NULL;
static struct iio_device *orx = NULL;
static struct iio_device *phy = NULL;
static struct iio_channel *obschn = NULL;


static char default_dst[] = "./rx_data.bin"; //default rx file save path
static uint8_t RFICseq=255; //RFIC sequence number
static bool chn1=0; // chn1 == 1 indicates channel 1 is selected
static bool chn2=0; // chn2 == 1 indicates channel 2 is selected
static FILE* fd = NULL;


/* Map GPIO related memory to virtual memory space*/
static int openGPIO(){
	mem_base = OpenMemory(&memfd, MEM_BASE, MEM_MAP_SIZE);
	if (mem_base){
		mem_orx_atten = mem_base + MEM_ORX_ATTEN;
		mem_mode_select = mem_base + MEM_MODE_SELECT;
		mem_channel_select = mem_base + MEM_CHANNEL_SELECT;
		mem_fpga_set = mem_base + MEM_FPGA_SET;
		mem_sync_set = mem_base + MEM_SYNC_SET;
		}
	else
		return -1;
}

/* Unmap GPIO memory*/
static void CloseGPIO(){
    if (fd!= NULL)
		fclose (fd);

    if (mem_base){
        printf("* Close GPIO Memory\n");
        if (munmap(mem_base, MEM_MAP_SIZE) == -1)
        {
            printf("Can't unmap memory from user space.\n");
            exit(0);
        }
    }
    if (memfd != -1)
        close(memfd);
}

/* clean up function*/
static void CleanUP(){
	CloseGPIO();
	shutdown_iio();
}

/* cleanup and exit; declared in Ad9371_common.h */
void shutdown_iio()
{
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

/* convert scfg.map bitmap to specific RFIC and Channel select*/
void ConvertBitMap(){
	uint8_t map = scfg.map;
	int i = 0;
	while(map){
		if (map & 1){
			if (RFICseq == 255){
				RFICseq = i / 2 + 1;
				}
			else{
				if (RFICseq != i/2 + 1){
					fprintf(stderr, "Wrong channel select input; exit. \n");
					CleanUP();
					}
			}
			//printf("i = %d, channel = %d\n", i, i%2);
			if (!(i%2))
				chn1 = 1;
			else
				chn2 = 1;
			//printf("%d, %d\n", chn1, chn2);
			}
		map >>= 1;
		i++;
		}
}


/* Configure AD9371*/
static void cfg_ad9371()
{
  LOG_PRINT_L2(mode_verbo, printf("* Acquiring IIO context\n"));
  ASSERT((ctx = iio_create_local_context()) && "No context");
  LOG_PRINT_L2(mode_verbo,printf("Get %d devices.\n",iio_context_get_devices_count(ctx)));
  ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

  phy = get_ad9371_phy(ctx);
  LOG_PRINT_L2(mode_verbo,printf("* Acquiring AD9371 streaming devices\n"));
  ASSERT(get_ad9371_stream_dev(ctx, TX, &tx) && "No tx dev found");
  ASSERT(get_ad9371_stream_dev(ctx, RX, &rx) && "No rx dev found");
  ASSERT(get_ad9371_stream_dev(ctx, ORX, &orx) && "No rx dev found");

  get_phy_chan(ctx, RX, 2, &obschn);
  ASSERT(obschn!=NULL);
  char attrval[] = "INTERNALCALS";
  ASSERT_ATTR(iio_channel_attr_write(obschn, "rf_port_select", attrval));

  if (scfg.tx_on){
         LOG_PRINT_L2(mode_verbo,printf("* Configuring AD9371 for streaming\n"));
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 0, 1) && "TX port 0 not found");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 1, 1) && "TX port 1 not found");
	  bool val;
	  iio_device_debug_attr_write_bool(phy,"loopback_tx_rx", cfg.loop);
	  iio_device_debug_attr_read_bool(phy,"loopback_tx_rx", &val);
	  LOG_PRINT_L1(mode_verbo,printf("Tx_RX loopback is %d\n",val));
  	}

  if (scfg.rx_on || scfg.orx_on){
  	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 0, 0) && "RX port 0 not found");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 1, 0) && "RX port 1 not found");
	  ASSERT(cfg_ad9371_streaming_ch(ctx, &cfg, 2, 0) && "ORX port 0 not found");
  	}
}

/* Open streaming Channel*/
static void open_ad9371_streaming_channel(){
	if (scfg.tx_on){
		ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 0, 0, &tx0_i) && "TX chan i not found");
		ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 1, 0, &tx0_q) && "TX chan q not found");
		ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 2, 0, &tx1_i) && "TX chan i not found");
		ASSERT(get_ad9371_stream_ch(ctx, TX, tx, 3, 0, &tx1_q) && "TX chan q not found");

		if (chn1){
			LOG_PRINT_L1(mode_verbo,puts("channel1 is enabled"));
			iio_channel_enable(tx0_i);
			iio_channel_enable(tx0_q);
			}

		if (chn2){
			LOG_PRINT_L1(mode_verbo,puts("channel2 is enabled"));
			iio_channel_enable(tx1_i);
			iio_channel_enable(tx1_q);
			}
		}

	if (scfg.rx_on){
		ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 0, 'i', &rx0_i) && "RX chan i not found");
		ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 0, 'q', &rx0_q) && "RX chan q not found");
		ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 1, 'i', &rx0_i) && "RX chan i not found");
		ASSERT(get_ad9371_stream_ch(ctx, RX, rx, 1, 'q', &rx0_q) && "RX chan q not found");

		if (chn1){
			LOG_PRINT_L1(mode_verbo,puts("channel1 is enabled"));
			iio_channel_enable(rx0_i);
			iio_channel_enable(rx0_q);
			}

		if (chn2){	
			LOG_PRINT_L1(mode_verbo,puts("channel2 is enabled"));
			iio_channel_enable(rx0_i);
			iio_channel_enable(rx0_q);
			}
		}

	if (scfg.orx_on){
		char attrval[] = "ORX1_TX_LO";            //char attrval[128];
		ASSERT_ATTR(iio_channel_attr_write(obschn, "rf_port_select", attrval));
		ASSERT(get_ad9371_stream_ch(ctx, RX, orx, 0, 'i', &orx0_i) && "RX chan i not found"); 
		ASSERT(get_ad9371_stream_ch(ctx, RX, orx, 0, 'q', &orx0_q) && "RX chan q not found");
		iio_channel_enable(orx0_i);
		iio_channel_enable(orx0_q);
		}
	
}

/* create libiio buffer*/
static void create_libiio_buffer (){
	if (scfg.tx_on){
		txbuf = iio_device_create_buffer(tx, scfg.frame_size, scfg.cycle);
		if (!txbuf) {
			perror("Could not create TX buffer");
			CleanUP();
			}
		else{
			LOG_PRINT_L1(mode_verbo, puts("Open Tx buffer succeeds"));
			}
		}
	
	if (scfg.rx_on){
		rxbuf = iio_device_create_buffer(rx, scfg.frame_size, scfg.cycle);
		if (!rxbuf) {
			perror("Could not create RX buffer");
			CleanUP();
			}
		else{
			LOG_PRINT_L1(mode_verbo, puts("Open Rx buffer succeeds"));
			}
		}

	if (scfg.orx_on){
		rxbuf = iio_device_create_buffer(orx, scfg.frame_size, scfg.cycle);
		if (!rxbuf) {
			perror("Could not create ORX buffer");
			CleanUP();
			}
		else{
			LOG_PRINT_L1(mode_verbo, puts("Open ORx buffer succeeds"));
			}
		}
}

/* Setup GPIO*/
static void setup_gpio(){
	int reg = 0;
	memcpy(mem_sync_set, &reg, 4); //disable synchronization block

	reg = 0x80000000; //open PA power
	if (scfg.enb_FPGA){
		if (scfg.tx_on)
			reg |= (0x80 << 8);
		else if(scfg.rx_on){
			reg |= 0xa0;
			}
		}
	memcpy(mem_fpga_set, &reg, 4);

	reg = 0;
	memcpy(mem_mode_select, &reg, 1); //0xff22e030
	memcpy(mem_mode_select+1, &reg, 1); //0xff22e031

	if (scfg.tx_on){
		reg |= scfg.map;
		memcpy(mem_mode_select+1, &reg, 1); //0xff22e031		
		}

	//FIXME: add GPIO setup for ORx. Including ORx attenuation
}

static void enable_sync_block(){
	int reg = 3;
	reg |= ((DELTATR) << 8);
	memcpy(mem_sync_set, &reg, 4);
}


/*analyze input/output file*/
static void analyze_filename(){
	LOG_PRINT_L1(mode_verbo, printf("filename is %s\n", scfg.filename));
	if (scfg.tx_on){
		if (!(fd = fopen(scfg.filename, "rb"))){
			fprintf(stderr, "Cannot open input file\n");
			CleanUP();
			}
		fseek(fd, 0, SEEK_END);
		scfg.frame_size = ftell(fd) / 4; //FIXME: preset sample size = 4; a nasty trick is needed to get real-time sample size
		LOG_PRINT_L1(mode_verbo,printf("Tx frame size is set to %d samples\n", scfg.frame_size));
		fseek(fd, 0, SEEK_SET);
		}

	if (scfg.rx_on){
		if (!(fd = fopen(scfg.filename, "wb+"))){
			fprintf(stderr, "Cannot open output file\n");
			CleanUP();
			}
		}
}

/* handle iio buffer for Tx*/
static void push_iio_tx_buffer(){
	char *p_dat, *p_end;
	ptrdiff_t p_inc;
	size_t nbytes_tx;

	if (chn1)
		p_dat = iio_buffer_first(txbuf, tx0_i);
	else
		p_dat = iio_buffer_first(txbuf, tx1_i);
	p_end = iio_buffer_end(txbuf);
	p_inc = iio_buffer_step(txbuf);
	LOG_PRINT_L2(mode_verbo, printf("begin: %x, end: %x, step: %x\n", (uint) p_dat,(uint) p_end, p_inc));

	char *temp = NULL, *temp2 = NULL;
	temp = malloc(scfg.frame_size * 4);
	
	fread(temp, 4, scfg.frame_size, fd);
	LOG_PRINT_L1(mode_verbo, printf("Read %d bytes from %s\n", scfg.frame_size*p_inc, scfg.filename));

	if (chn1 && chn2){
		temp2 = malloc(scfg.frame_size* 8);
		for(int i=0; i<scfg.frame_size; i++){
			memcpy(temp2+i*8, temp+i*4, 4);
			memcpy(temp2+i*8+4, temp+i*4, 4);
			}
		memcpy(p_dat, temp2, scfg.frame_size*8);
	}else{
		memcpy(p_dat, temp, p_inc * scfg.frame_size);
	}

	nbytes_tx = iio_buffer_push(txbuf); 
	if (nbytes_tx < 0) { fprintf(stderr, "Error pushing buf %d\n", (int) nbytes_tx); CleanUP(); }
	else{ 
		LOG_PRINT_L1(mode_verbo, printf("push %d bytes of data\n", (int) nbytes_tx));
		}

	if (temp) free(temp);
	if(temp2) free(temp2);
}

static void refill_iio_rx_buffer(){
	char *p_dat, *p_end;
	ptrdiff_t p_inc;
	size_t count = 0, bytes_count = 0, nbytes_rx = 0;

	//printf("frame number is %d\n", scfg.frame_num);	
	while(count < scfg.frame_num){
		nbytes_rx = iio_buffer_refill(rxbuf);
		if (nbytes_rx < 0) { fprintf(stderr, "Error refilling buf %d\n",(int) nbytes_rx); CleanUP(); }
		else{
				LOG_PRINT_L1(mode_verbo,printf("Refill %d bytes of data\n",nbytes_rx));
		}

		bytes_count += nbytes_rx;

		p_inc = iio_buffer_step(rxbuf);
		p_end = iio_buffer_end(rxbuf);
		p_dat = iio_buffer_first(rxbuf, rx0_i);

		fwrite(p_dat, p_inc, scfg.frame_size, fd);
		fflush(fd);
		count++;
		}
}



/* main function */
int main (int argc, char **argv)
{
	// Listen to ctrl+c and ASSERT
	signal(SIGINT, handle_sig);
	
	//parse input command
	get_args(argc, argv, &cfg, &scfg);

	openGPIO();
	setup_gpio();

	//find the specific AD9371 chip and channel selected;
	ConvertBitMap();

	cfg_ad9371();
	
	open_ad9371_streaming_channel();

	if (scfg.enb_DMA){
		if (scfg.tx_on){
			if (!scfg.filename){
				fprintf(stderr, "Tx DMA mode must have input source file!\n");
				CleanUP();
				}
			analyze_filename();

			create_libiio_buffer();
			
			push_iio_tx_buffer();

			//sychronization block is enabled AFTER tx buffer is ready
			enable_sync_block();
			}

		if (scfg.rx_on){
			puts("rx is on");
			if(!scfg.filename){
				scfg.filename = default_dst;
				}
			analyze_filename();

			create_libiio_buffer();

			//synchronization block is enabled BEFORE rx buffer is ready
			enable_sync_block();
			
			refill_iio_rx_buffer();
		}
	}
	pause();
}