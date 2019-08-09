/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
 *
 * License: GPL, version 2.1
 */
#ifndef __Ad9xxx_common_h
#define __Ad9xxx_common_h

#include <time.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif
#include <iio-private.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h> /* size_t, offsetof(type, member) */
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

//GPIO Control Memeory Address
//mode number (4bits) + division (4bits)
#define OFFSET1 0xe030
//Obs and snf gain
#define OFFSET2 0xe020

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

#define LOG_PRINT_L1(a,b)  if(a>=1) b;

#define LOG_PRINT_L2(a,b) if(a>=2) b;

#define MAXLEN_FILENAME  128
#define EOS_PRINTF_MAX_LEN 128
#define MAX_CHIP_NUM 4
#define MAX_TRNUM_PER_CHIP 2
/* RX is input, TX is output */
enum iodev 
{ 
    RX,
    TX,
    ORX,
    IO_BUTT
};

enum LOG_LEVEL
{
    LOG_LEVEL0,
    LOG_LEVEL1,
    LOG_LEVEL2,
    LOG_LEVEL_BUTT
};

typedef struct timespec timespec;


struct phy_cfg {
    long long rx_gain;
    long long tx_gain;
    bool tx_qtracking;
    bool tx_loltracking;
    bool rx_qtracking;
    long long tx_freq;
    long long rx_freq;
    bool loop;
};

struct stream_cfg{
    bool tx_on;
    bool rx_on;
    bool orx_on;
    bool cycle;
    bool enb_FPGA;
    bool enb_DMA;
    uint8_t RFICseq;
    size_t frame_size;
    size_t frame_num;
};

/* print out application usage*/
void phyinitusage(void);

/* cleanup and exit */
void phy_stream_shutdown_iio();

/* check return value of attr_write function */
void errchk(int v, const char* what);

/* write attribute: long long int */
void wr_ch_lli(struct iio_channel *chn, const char* what, long long val);

/* read attribute: long long int */
long long rd_ch_lli(struct iio_channel *chn, const char* what);

/* helper function generating channel names */
char* get_ch_name_mod(const char* type, int id, char modify);

/* helper function generating channel names */
char* get_ch_name(const char* type, int id);

/* returns ad9371 phy device */
struct iio_device* get_ad9371_phy(struct iio_context *ctx);

/* finds AD9371 streaming IIO devices */
bool get_ad9371_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev);

/* finds AD9371 streaming IIO channels */
bool get_ad9371_stream_ch(struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, char modify, struct iio_channel **chn);

/* finds AD9371 phy IIO configuration channel with id chid */
bool get_phy_chan(struct iio_device *phy, enum iodev d, int chid, struct iio_channel **chn);

/* finds AD9371 local oscillator IIO configuration channels */
bool get_lo_chan(struct iio_device *phy, enum iodev d, struct iio_channel **chn);

/* print the name of a libiio channel*/
void print_channel_name(struct iio_channel *chn);

/* print all the attributes of a libiio channel*/
bool print_ad9371_channel_attr(struct iio_context *ctx, enum iodev type, int chid);

/* print all the attributes of a libiio channel v2*/
bool print_ad9371_channel_attr2(struct iio_channel *chn);

bool power_down_chan(struct iio_device *ptrphy, int chid);

bool power_up_chan(struct iio_device *ptrphy, int chid);

bool is_chan_power_down(struct iio_device *ptrphy, int chid);

/* calculate the time difference between the two input, return in ns*/
long long timediff(timespec a, timespec b);

/* Open a page of memeory start with dev_base with size size*/
void* OpenMemory(int *memfd, off_t dev_base, size_t size);

char judge_rfchip_type(struct iio_context *ctx);

void syslog_level(unsigned int level, const char *pFormat, ... );

char* get_ch_name(const char* type, int id);

struct iio_device* get_phy_dev(struct iio_context *ctx, char *devicename);

void load_ad9371_profile(struct iio_device *phy, mykonosDevice_t *mykDevice);

void load_adrv9009_profile(struct iio_device *phy, taliseInit_t *talInit);

void load_profile_simplified(struct iio_context *ctx, struct iio_device *phy, char *fn);

int phy_stream_rxdma_overflow(int igrp);

int phy_stream_txdma_underflow(int igrp);

bool get_stream_chn(struct iio_device *dev, enum iodev d, int chid, char modify, struct iio_channel **chn);

bool get_phy_chan(struct iio_device *phy, enum iodev d, int chid, struct iio_channel **chn);

bool phy_stream_get_dev(struct iio_context *ctx, char ifad9371, enum iodev d, char index, struct iio_device **dev);

#endif //ifdef __Ad9371_common_h
