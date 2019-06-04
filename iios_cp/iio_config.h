/* common RX and TX streaming params */
struct stream_cfg {
	long long bw_hz; // Analog banwidth in Hz
	long long fs_hz; // Baseband sample rate in Hz
	long long lo_hz; // Local oscillator frequency in Hz
    double gain_or_atten;
    bool qtracking;
    bool loltracking;//TX ONLY
#if 0
    const char *filename;
    bool on;
    bool nostop;
    size_t frame_num;
#endif
};

// Stream configurations
struct stream_cfg rxcfg = {
	0,
	0,
	GHZ(3.5), //long long rx_lo_freq;
	0, //double rx_gain;
	1, //bool rx_qtracking;
	0, //bool unused! rx_loltracking;
#if 0
	"./rx_data.bin", //dst file path
	1, //rx is on;
	0, //non-stop is disabled
	10 //10 frames
#endif
};

struct stream_cfg txcfg = {
	0,
	0,
	GHZ(3.5), //long long tx_lo_freq;
	-20, //double tx_atten;
	1, //bool tx_qtracking;
	1, //bool tx_loltracking;
#if 0
	NULL, //src file path
	0, //tx is off;
	0, //non-stop is disabled
	10 //10 frames
#endif
};

struct phy_cfg cfg = {
   0, //double rx_gain;
   -20, //double tx_atten;
   1, //bool tx_qtracking;
   1, //bool tx_loltracking;
   1, //bool rx_qtracking;
   GHZ(3.5), //long long tx_freq;
   GHZ(3.5), //long long rx_freq;
   0 //not enable loopback
};

