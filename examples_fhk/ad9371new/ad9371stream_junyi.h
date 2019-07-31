/*
 * applications based on libiio
 *   - AD9371 IIO streaming example
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Junyi Zhang <jun-yi.zhang@foxconn.com>
 *
 * License: GPL, version 2.1
 */
#ifndef __Ad9371_stream_h
#define __Ad9371_stream_h

/* define the memory map address of GPIO*/
#define MEM_BASE 0xff220000
#define MEM_ORX_ATTEN 0x3000
#define MEM_MODE_SELECT 0xe030 //mode number (4bits) + division (4bits)
#define MEM_CHANNEL_SELECT 0xe020 //Select channel
#define MEM_FPGA_SET 0xe050
#define MEM_SYNC_SET 0xe040

#define MEM_MAP_SIZE 0x10000
#define MEM_MAP_MASK MEM_MAP_SIZE-1

/* \delta T_r and \delta T_t*/
#define DELTATR 163
#define DELTATT 163

/* Map GPIO related memory to virtual memory space*/
static int openGPIO();

/* Unmap GPIO memory*/
static void CloseGPIO();

/* clean up function*/
static void CleanUp();

/* convert scfg.map bitmap to specific RFIC and Channel select*/
void ConvertBitMap();

/* Configure AD9371*/
static void cfg_ad9371();

/* Open streaming Channel*/
static void open_ad9371_streaming_channel();

/* create libiio buffer*/
static void create_libiio_buffer ();

/* Setup GPIO*/
static void setup_gpio();

static void enable_sync_block();

/*analyze input/output file*/
static void analyze_filename();

/* handle iio buffer for Tx*/
static void push_iio_tx_buffer();
static void refill_iio_rx_buffer();

#endif