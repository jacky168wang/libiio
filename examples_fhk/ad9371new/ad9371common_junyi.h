/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
 *
 * License: GPL, version 2.1
 */
#ifndef __Ad9371_common_h
#define __Ad9371_common_h

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
#include <ctype.h>


//#define DEBUG

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

#define LOG_PRINT_L1(a,b) if(a>=1) b; 
#define LOG_PRINT_L2(a,b) if(a>=2) b; 


/* RX is input, TX is output */
enum iodev { RX, TX, ORX};

typedef struct timespec timespec;

/* IIO structs required for streaming 
extern struct iio_context *ctx   = NULL;
extern struct iio_channel *rx0_i = NULL;
extern struct iio_channel *rx0_q = NULL;
extern struct iio_channel *tx0_i = NULL;
extern struct iio_channel *tx0_q = NULL;
extern struct iio_channel *tx1_i = NULL;
extern struct iio_channel *tx1_q = NULL;
extern struct iio_buffer  *rxbuf = NULL;
extern struct iio_buffer  *txbuf = NULL;*/

struct phy_cfg {
    long long rx_gain;
    long long tx_atten;
    bool tracking_cal;
    long long freq;
    bool loop;
};

struct stream_cfg{
    char *filename;
    bool tx_on;
    bool rx_on;
    bool orx_on;
    bool cycle;
    bool enb_FPGA;
    bool enb_DMA;
    size_t frame_size;
    size_t frame_num;
    uint8_t map;
};

/* print out application usage*/
static void usage(void);

/* input command parse*/
void get_args(int argc, char **argv, struct phy_cfg *cfg, struct stream_cfg *scfg);

/* cleanup and exit */
void shutdown_iio();

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
bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn);

/* finds AD9371 local oscillator IIO configuration channels */
bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn);

/* applies streaming configuration through IIO */
bool cfg_ad9371_streaming_ch(struct iio_context *ctx, struct phy_cfg *cfg, int chid, int tx1rx0);

/* print the name of a libiio channel*/
void print_channel_name(struct iio_channel *chn);

/* print all the attributes of a libiio channel*/
bool print_ad9371_channel_attr(struct iio_context *ctx, enum iodev type, int chid);

/* print all the attributes of a libiio channel v2*/
bool print_ad9371_channel_attr2(struct iio_channel *chn);

/* load ad9371 profile from a text file*/
void load_ad9371_profile_simplified(struct iio_context *ctx, struct iio_device *phy, char *fn);

/* function to handle the interrupt signal*/
void handle_sig(int sig);

/* calculate the time difference between the two input, return in ns*/
long long timediff(timespec a, timespec b);

/* Open a page of memeory start with dev_base with size size*/
void* OpenMemory(int *memfd, off_t dev_base, size_t size);

#endif //ifdef __Ad9371_common_h