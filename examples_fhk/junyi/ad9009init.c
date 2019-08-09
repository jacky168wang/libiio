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
        -26, //double tx_atten;
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

    // phy debug device
    struct iio_device *phy;

    printf("* Acquiring IIO context\n");
    ASSERT((ctx = iio_create_local_context()) && "No context");
    printf("Get %d devices.\n",iio_context_get_devices_count(ctx));
    ASSERT(iio_context_get_devices_count(ctx) > 0 && "No devices");

    phy = get_adrv9009_phy(ctx);
    if (argc > 1)
    {
        char *fn = argv[1];
        load_adrv9009_profile_simplified(ctx, phy, fn);
    }else
        load_adrv9009_profile(phy, &talInit);

    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 0, 0) && "RX port 0 not found");//rx1
    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 1, 0) && "RX port 1 not found");//rx2
    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 0, 1) && "TX port 0 not found");//tx1
    ASSERT(cfg_adrv9009_streaming_ch(ctx, &cfg, 1, 1) && "TX port 1 not found");//tx2

    //long long val;
    //iio_device_debug_attr_write_longlong(phy,"adi,tx-settings-trx-pll-lo-frequency_hz", cfg.lo_freq);
    //iio_device_debug_attr_read_longlong(phy,"adi,tx-settings-trx-pll-lo-frequency_hz", &val);
    //printf("TRx frequency is %lld\n",val);

    //long long gain1, gain2;
    //iio_device_debug_attr_read_longlong(phy,"adi,tx-settings-tx1-atten_mdb", &gain1);
    //iio_device_debug_attr_read_longlong(phy,"adi,tx-settings-tx2-atten_mdb", &gain2);
    //printf("tx1_atten is %lld, tx2_atten is %lld\n",gain1, gain2);

    //long long sr_tx, sr_rx;
    //iio_device_debug_attr_read_longlong(phy,"adi,tx-profile-iq-rate_khz", &sr_tx);
    //iio_device_debug_attr_read_longlong(phy,"adi,rx-profile-iq-rate_khz", &sr_rx);
    //printf("tx sample rate is %lld, rx sample rate is %lld\n",sr_tx, sr_rx);

    shutdown();
    return 0;
}


