/*
 * libiio - ADRV9009 IIO streaming application
 * modified based on AD9371 IIO streaming example
 *
 *
 * Copyright (C) 2014 IABG mbH
 * Original Author: Michael Feilen <feilen_at_iabg.de>
 * Modified Author: Yang Yang <page.y.yang._at_foxconn.com>
 * Copyright (C) 2017 Analog Devices Inc.
 * Copyright (C) 2018 Foxconn 5G Research Institute
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
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#ifdef __APPLE__
#include <iio/iio.h>
#else
#include <iio.h>
#endif

#include "ad9009cfg.h"


/* simple configuration and streaming */
int main (int argc, char **argv)
{
    channel_cfg cfg =
    {
        0, //double rx_gain;
        -30, //double tx_atten;
        0, //bool tx_qtracking;
        0, //bool tx_loltracking;
        0, //bool rx_qtracking;
        0, //bool rx_hd2tracking;
        0, // pa_protection;
        GHZ(3.5), //long long TRx_LO_freq;
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
        0, //not using tx2
        0  //not using rx1
    };

    command_parse(argc, argv, &cfg, &scfg);

    if(scfg.tx_on == 1 && scfg.src_fn == NULL){
        printf("Need source data file for TX\n");
        exit(0);
    }

    // Listen to ctrl+c and ASSERT
    signal(SIGINT, handle_sig);

    // Acquiring IIO context
    printf("* Acquiring IIO context\n");
    ASSERT((ctx = iio_create_local_context()) && "No context");
    printf("Get %d devices.\n",iio_context_get_devices_count(ctx));
    ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    // Acquiring IIO phy device & stream device
    struct iio_device *phy;
    // Streaming devices
    struct iio_device *tx;
    struct iio_device *rx;
    struct iio_device *orx;
    phy = get_adrv9009_phy(ctx);
    printf("* Acquiring ADRV9009 streaming devices\n");
    ASSERT(get_adrv9009_stream_dev(ctx, TX, &tx) && "No tx dev found");
    ASSERT(get_adrv9009_stream_dev(ctx, RX, &rx) && "No rx dev found");
    ASSERT(get_adrv9009_stream_dev(ctx, ORX, &orx) && "No orx dev found");

    struct iio_channel *obschn1 = NULL;
    struct iio_channel *obschn2 = NULL;
    get_phy_chan(ctx, RX, 2, &obschn1); //Observation RX1
    ASSERT(obschn1!=NULL);
    get_phy_chan(ctx, RX, 3, &obschn2); //Observation RX2
    ASSERT(obschn2!=NULL);

    //"OBS_TX_LO": ORX select TRX-LO as LO
    //"OBS_AUX_LO": ORX select Aux-LO as LO
    char attrval[] = "OBS_TX_LO";
    ASSERT_ATTR(iio_channel_attr_write(obschn1, "rf_port_select", attrval));
    ASSERT_ATTR(iio_channel_attr_write(obschn2, "rf_port_select", attrval));

    //if (scfg.rx_on){
    //    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 0, 0) && "RX port 0 not found"); //RX1
    //    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 1, 0) && "RX port 1 not found"); //RX2
    //    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 2, 2) && "ORX port 0 not found"); //ORX1
    //    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 3, 2) && "ORX port 1 not found"); //ORX2
    //}

    size_t tx_samp_num;

    //Open source and destination file
    FILE* src_fd = NULL, *dst_fd = NULL;
    //char* buf = malloc(tx_samp_num * tx_samp_size);
    if (scfg.tx_on){
        printf("src file is %s\n", scfg.src_fn);
        if(!(src_fd = fopen(scfg.src_fn, "rb"))){
            printf("Cannot open src file\n");
            shutdown();
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
    if(scfg.rx_on){
        //rx1=0: RX1; rx1=1: RX2; rx1=2: ORX1; rx1=3: ORX2
        if(scfg.rx1==0){//rx1
            printf("* Configuring ADRV9009 for Rx1 streaming\n");
            ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 0, 0) && "RX port 0 not found"); //RX1
            printf("* Initializing ADRV9009 IIO Rx1 streaming channels\n");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, rx, 0, 'i', &rx0_i) && "RX chan i not found");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, rx, 0, 'q', &rx0_q) && "RX chan q not found");
            iio_channel_enable(rx0_i);
            iio_channel_enable(rx0_q);
        }
        if (scfg.rx1==1){//rx2
            printf("* Configuring ADRV9009 for Rx2 streaming\n");
            ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 1, 0) && "RX port 0 not found"); //RX2
            printf("* Initializing ADRV9009 IIO Rx2 streaming channels\n");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, rx, 1, 'i', &rx0_i) && "RX chan i not found");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, rx, 1, 'q', &rx0_q) && "RX chan q not found");
            iio_channel_enable(rx0_i);
            iio_channel_enable(rx0_q);
        }
        if (scfg.rx1==2){//orx1
            printf("* Configuring ADRV9009 for ORx1 streaming\n");
            ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 2, 2) && "ORX port 0 not found"); //ORX1
            printf("* Initializing ADRV9009 IIO ORx1 streaming channels\n");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, orx, 0, 'i', &rx0_i) && "ORX chan i not found");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, orx, 0, 'q', &rx0_q) && "ORX chan q not found");
            iio_channel_enable(rx0_i);
            iio_channel_enable(rx0_q);
        }
        if (scfg.rx1==3){//orx2
            printf("* Configuring ADRV9009 for ORx2 streaming\n");
            ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 3, 2) && "ORX2 port 0 not found"); //ORX2
            printf("* Initializing ADRV9009 IIO ORx2 streaming channels\n");
            //ASSERT(get_adrv9009_stream_ch(ctx, RX, orx, 1, 'i', &rx0_i) && "ORX chan i not found");
            //ASSERT(get_adrv9009_stream_ch(ctx, RX, orx, 1, 'q', &rx0_q) && "ORX chan q not found");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, orx, 0, 'i', &rx0_i) && "ORX chan i not found");
            ASSERT(get_adrv9009_stream_ch(ctx, RX, orx, 0, 'q', &rx0_q) && "ORX chan q not found");
            iio_channel_enable(rx0_i);
            iio_channel_enable(rx0_q);
        }

        //rxbuf used by RX&ORX
        if (scfg.rx1 < 2) { //one device only one buffer
            printf("* Creating Rx buffers\n");
            rxbuf = iio_device_create_buffer(rx, rx_samp_num, false);
        } else {
            printf("* Creating Rx buffers\n");
            rxbuf = iio_device_create_buffer(orx, rx_samp_num, false);
        }
            if (!rxbuf) {
                perror("Could not create RX buffer");
                shutdown();
            }
    }

    if(scfg.tx_on){
        if (scfg.tx1){
            printf("* Configuring ADRV9009 for Tx1 streaming\n");
            ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 0, 1) && "TX port 0 not found"); //TX1
            ASSERT(get_adrv9009_stream_ch(ctx, TX, tx, 0, 0, &tx0_i) && "TX chan i not found");
            ASSERT(get_adrv9009_stream_ch(ctx, TX, tx, 1, 0, &tx0_q) && "TX chan q not found");
            iio_channel_enable(tx0_i);
            iio_channel_enable(tx0_q);
        }
        if (scfg.tx2){
            ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 1, 1) && "TX port 1 not found"); //TX2
            ASSERT(get_adrv9009_stream_ch(ctx, TX, tx, 2, 0, &tx1_i) && "TX chan i not found");
            ASSERT(get_adrv9009_stream_ch(ctx, TX, tx, 3, 0, &tx1_q) && "TX chan q not found");
            iio_channel_enable(tx1_i);
            iio_channel_enable(tx1_q);
        }
        printf("* Creating Tx buffers\n"); //one device only one buffer
        txbuf = iio_device_create_buffer(tx, tx_samp_num, cfg.cycle);
        if (!txbuf) {
            perror("Could not create TX buffer");
            shutdown();
        }
        //ssize_t tx_samp_size = iio_device_get_sample_size(tx);
        //printf("%d bytes \n", tx_samp_size);
    }

    // Schedule TX buffer
    ssize_t nbytes_tx, nbytes_rx;
    char *p_dat, *p_end;
    ptrdiff_t p_inc;

    if (scfg.tx_on){
        if(scfg.tx1) //only tx1 or have tx1&tx2
            p_dat = iio_buffer_first(txbuf, tx0_i);
        else //only tx2
            p_dat = iio_buffer_first(txbuf, tx1_i);

        p_end = iio_buffer_end(txbuf);
        p_inc = iio_buffer_step(txbuf);
        printf("begin: %x, end: %x, step: %x\n", (uint) p_dat,(uint) p_end, p_inc);

        char *temp=NULL,*temp2=NULL;
        temp = malloc(tx_samp_num * 4);
        if (src_fd){
            fread(temp, 4, tx_samp_num, src_fd); //read 'tx_samp_num' times data form 'src_fd' file, every time read '4 bytes '
            printf("Read %d bytes from %s\n", tx_samp_num*p_inc, scfg.src_fn);
        }
        if (scfg.tx1 && scfg.tx2){ //tx1 & tx2
            temp2 = malloc(tx_samp_num * 8);
            for(int i=0; i<tx_samp_num; i++){
                memcpy(temp2+i*8, temp+i*4, 4);
                memcpy(temp2+i*8+4, temp+i*4, 4);
            }
            memcpy(p_dat, temp2, tx_samp_num*8);
        }
        else{ //only tx1 or tx2
            memcpy(p_dat, temp, p_inc * tx_samp_num);
        }

        nbytes_tx = iio_buffer_push(txbuf);  //the number of bytes written is returned that send to the hardware.
        if (nbytes_tx < 0) { printf("Error pushing buf %d\n", (int) nbytes_tx); shutdown(); }
        else printf("push %d bytes of data\n", (int) nbytes_tx);

        if (temp) free(temp);
        if (temp2) free(temp2);

        usleep(100000);  //hang 100ms

    }

    struct timespec t0, t1, t2;
    if (scfg.rx_on){
        size_t count = 0, bytes_count = 0;
        while (count < scfg.frame_num){
            //Refill Rx buffer
            t2 = t0;
            clock_gettime(CLOCK_MONOTONIC, &t0);
            nbytes_rx = iio_buffer_refill(rxbuf);
            clock_gettime(CLOCK_MONOTONIC, &t1);
            printf("refill interval %lld us, refill takes %lld us, the throughput is %fMbps\n",
                    timediff(t0,t2)/1000, timediff(t0,t1)/1000,nbytes_rx * 8.0 / timediff(t0,t1)*1000);
            if (nbytes_rx < 0) { printf("Error refilling buf %d\n",(int) nbytes_rx); shutdown();}
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
            count++;
        }

        iio_buffer_destroy(rxbuf);
        rxbuf = NULL;
        printf("End receiving %d frames, %d btyes in total\n Destroy rx buffer, rxbuf=%x\n", scfg.frame_num, bytes_count, (uint) rxbuf);
        if (dst_fd)
            fclose(dst_fd);
    }

    if  (src_fd)
        fclose(src_fd);

    //if (!scfg.no_stop)
    pause();

    shutdown();

    //printf("Tx2 is %s\n ", iio_channel_is_enabled(tx0_i)? "on":"off");
    return 0;

}

