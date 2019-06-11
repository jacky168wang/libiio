/*
 * https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
 * https://sites.google.com/site/packetmmap/
 *   Userspace example for "package-socket with mmap, tx" to make
 *   transmission with a zero-copy mechanism to increase TX bandwidth
 *   of raw socket.
 * Author: Johann Baudy <johann.baudy@gnu-log.net>
 * Usage:
 *   Sending packet of 7000bytes, socket buffer size to 100 packets (7000*100)
 *     "./packet_mmap -s7000 -m7200 -z700000 eth0"
 *   Using multi-thread: "./packet_mmap -t eth0"
 *   Using SOCK_DGRAM to compare: "./packet_mmap -g eth0"
 *   Sending packet of 2490bytes, default socket buffer size, total 655360 packages:
 *     "./psock_mmap_tx -s2490 -m3000 -c655360 eth0":    515.704Mbps on a10ad9371
 *     "./psock_mmap_tx -s2490 -m3000 -c655360 -g eth0": 512.691Mbps on a10ad9371
 *
 * License: GPL, version 2.0
 *-----------------------------------------------------------------------------
+ Why use PACKET_MMAP
+ How to use mmap() directly to improve transmission process
+ PACKET_MMAP settings and constraints
+ Mapping and use of the circular buffer (ring)
+ What TPACKET versions are available and when to use them?
  int val = tpacket_version;
  setsockopt(fd, SOL_PACKET, PACKET_VERSION, &val, sizeof(val));
  TPACKET_V1:
  	- Default if not otherwise specified by setsockopt(2)
  	- RX_RING, TX_RING available
  TPACKET_V1 --> TPACKET_V2:
  	- Made 64 bit clean due to unsigned long usage in TPACKET_V1
  	  structures, thus this also works on 64 bit kernel with 32 bit
  	  userspace and the like
  	- Timestamp resolution in nanoseconds instead of microseconds
  	- RX_RING, TX_RING available
  	- VLAN metadata information available for packets
  	  (TP_STATUS_VLAN_VALID, TP_STATUS_VLAN_TPID_VALID),
  	  in the tpacket2_hdr structure:
  		- TP_STATUS_VLAN_VALID bit being set into the tp_status field indicates
  		  that the tp_vlan_tci field has valid VLAN TCI value
  		- TP_STATUS_VLAN_TPID_VALID bit being set into the tp_status field
  		  indicates that the tp_vlan_tpid field has valid VLAN TPID value
  	- How to switch to TPACKET_V2:
  		1. Replace struct tpacket_hdr by struct tpacket2_hdr
  		2. Query header len and save
  		3. Set protocol version to 2, set up ring as usual
  		4. For getting the sockaddr_ll,
  		   use (void *)hdr + TPACKET_ALIGN(hdrlen) instead of
  		   (void *)hdr + TPACKET_ALIGN(sizeof(struct tpacket_hdr))
  TPACKET_V2 --> TPACKET_V3:
  	- Flexible buffer implementation for RX_RING:
  		1. Blocks can be configured with non-static frame-size
  		2. Read/poll is at a block-level (as opposed to packet-level)
  		3. Added poll timeout to avoid indefinite user-space wait
  		   on idle links
  		4. Added user-configurable knobs:
  			4.1 block::timeout
  			4.2 tpkt_hdr::sk_rxhash
  	- RX Hash data available in user space
  	- TX_RING semantics are conceptually similar to TPACKET_V2;
  	  use tpacket3_hdr instead of tpacket2_hdr, and TPACKET3_HDRLEN
  	  instead of TPACKET2_HDRLEN. In the current implementation,
  	  the tp_next_offset field in the tpacket3_hdr MUST be set to
  	  zero, indicating that the ring does not hold variable sized frames.
  	  Packets with non-zero values of tp_next_offset will be dropped.
 *-----------------------------------------------------------------------------
+ How to use mmap() directly to improve transmission process
++ Transmission process
[setup]          socket() -------> creation of the transmission socket
                 setsockopt() ---> allocation of the circular buffer (ring)
                                   option: PACKET_TX_RING
                 bind() ---------> bind transmission socket with a interface
                 mmap() ---------> mapping of the allocated buffer to the
                                   user process

[transmission]   poll() ---------> wait for free packets (optional)
                 send() ---------> send all packets that are set as ready in
                                   the ring. The flag MSG_DONTWAIT can be used
                                   to return before end of transfer.
  #define TP_STATUS_AVAILABLE 	   0 // Frame is available
  #define TP_STATUS_SEND_REQUEST	   1 // Frame will be sent on next send()
  #define TP_STATUS_SENDING		   2 // Frame is currently in transmission
  #define TP_STATUS_WRONG_FORMAT	   4 // Frame format is not correct
  First, the kernel initializes all frames to TP_STATUS_AVAILABLE. To send a
  packet, the user fills a data buffer of an available frame, sets tp_len to
  current data buffer size and sets its status field to TP_STATUS_SEND_REQUEST.
  This can be done on multiple frames. Once the user is ready to transmit, it
  calls send(). Then all buffers with status equal to TP_STATUS_SEND_REQUEST are
  forwarded to the network device. The kernel updates each status of sent
  frames with TP_STATUS_SENDING until the end of transfer.
  At the end of each transfer, buffer status returns to TP_STATUS_AVAILABLE.
[shutdown]  close() --------> destruction of the transmission socket and
                              deallocation of all associated resources.
Summary example-codes:
    int fd_tx = socket(PF_PACKET, mode, 0);
    setsockopt(fd_tx, SOL_PACKET, PACKET_TX_RING, &foo, sizeof(foo));
    mmap(0, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd_tx, 0);
    #if 0
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.revents = 0;
        pfd.events = POLLOUT;
        retval = poll(&pfd, 1, timeout);
    #endif
    header->tp_len = in_i_size;
    header->tp_status = TP_STATUS_SEND_REQUEST;
    ec = send(this->socket, NULL, 0, 0);
 *-----------------------------------------------------------------------------
+ PACKET_QDISC_BYPASS
If there is a requirement to load the network with many packets in a similar
  fashion as pktgen does, you might set the following option after socket
  creation:
	int one = 1;
	setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS, &one, sizeof(one));
  This has the side-effect, that packets sent through PF_PACKET will bypass the
  kernel's qdisc layer and are forcedly pushed to the driver directly. Meaning,
  packet are not buffered, tc disciplines are ignored, increased loss can occur
  and such packets are also not visible to other PF_PACKET sockets anymore. So,
  you have been warned; generally, this can be useful for stress testing various
  components of a system.
 *-----------------------------------------------------------------------------
"man package"
  PACKET_TX_RING (since Linux 2.6.31)
	Create a memory-mapped ring buffer for packet transmission.
	This option is similar to PACKET_RX_RING and takes the same
	arguments.  The application writes packets into slots with
	tp_status equal to TP_STATUS_AVAILABLE and schedules them for
	transmission by changing tp_status to TP_STATUS_SEND_REQUEST.
	When packets are ready to be transmitted, the application
	calls send(2) or a variant thereof.	The buf and len fields of
	this call are ignored.  If an address is passed using
	sendto(2) or sendmsg(2), then that overrides the socket
	default.  On successful transmission, the socket resets
	tp_status to TP_STATUS_AVAILABLE.  It immediately aborts the
	transmission on error unless PACKET_LOSS is set.
  PACKET_LOSS (with PACKET_TX_RING)
	When a malformed packet is encountered on a transmit ring, the
	default is to reset its tp_status to TP_STATUS_WRONG_FORMAT
	and abort the transmission immediately.  The malformed packet
	blocks itself and subsequently enqueued packets from being
	sent.  The format error must be fixed, the associated tp_status
	reset to TP_STATUS_SEND_REQUEST, and the transmission process
	restarted via send(2).  However, if PACKET_LOSS is set, any
	malformed packet will be skipped, its tp_status reset to 
	TP_STATUS_AVAILABLE, and the transmission process continued.
  PACKET_QDISC_BYPASS (since Linux 3.14)
    By default, packets sent through packet sockets pass through
    the kernel's qdisc (traffic control) layer, which is fine for
    the vast majority of use cases.  For traffic generator appli©\
    ances using packet sockets that intend to brute-force flood
    the network¡ªfor example, to test devices under load in a simi©\
    lar fashion to pktgen¡ªthis layer can be bypassed by setting
    this integer option to 1.  A side effect is that packet
    buffering in the qdisc layer is avoided, which will lead to
    increased drops when network device transmit queues are busy;
    therefore, use at your own risk.
 *-----------------------------------------------------------------------------
History for "txring_patch" in Kernel 2.6 later, "af_packet.c -> tpacket_snd()"
	Version 2.4.24: (Dec 28, 2010)
		TO BE ADDED
	......
	Version 1.4: (May 11 2009) (selected for 2.6.31 merge window)
		Fix trailing white spaces 
	Version 1.3: (May 3 2009)
		Fix RX poll issue 
	Version 1.2: (May 2 2009)
		Clean code according to Linux coding rules
		Remove static function not used 
	Version 1.1: (April 21 2009)
		Move destructor argument to shared info
		Rename tp_status values for TX_RING 
	Version 1.0: (March 21 2009)
		Replace busy flag with pgvec mutex
		Remove useless while loop
		Remove shutdown flag
		Skb allocation is now always blocking 
	Version 0.9: (March 19 2009)
		Add setsockopt(PACKET_LOSS):
		1: ignore wrong formatted packet and continue
		0 : set status to TPSTATUS_LOSING and exit from send() procedure 
		Remove useless while() loop on release
		Add schedule() call on blocking loop 
	Version 0.8:
		Remove warning on 64 bit: warning: cast from pointer to integer of different size 
	Version 0.7:
		Remove useless debug printf
		Update documentation 
	Version 0.6:
		First frag wrongly calculated
		Dcache flushed when read or write frame status 
	Version 0.5:
		Align TX ring data with header as RX ring (TPACKET_ALIGN(..))
		Fix some critical issues 
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h> /* size_t, offsetof(type, member) */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h> /* GNU getopt() */

//#include <inttypes.h>
#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#include <poll.h>

//#include <sys/types.h>
//#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
//#include <sys/select.h>
#include <arpa/inet.h>
//#include <netinet/in.h>
#include <netdb.h>
#include <net/if.h>
#include <linux/if_ether.h>
#include <linux/if_packet.h>
//#include <linux/ip.h>
#if 0
#include <sys/user.h> /* PAGE_SIZE */
#include <linux/mmzone.h> /* MAX_ORDER */
#else
#define MAX_ORDER 11
#endif

#include "common_jacky.h"
#include "rru_bbu.h"
#if defined(BUILDING_RRU_UL)
#include <iio.h>
# if (1 != DEV_PORT_NB)
#  error "memcpy(samples) should be changed into for-each-sample for DEV_PORT_NB>1"
# endif
#endif

/* params */
#if defined(BUILDING_RRU_UL)
extern char *str_devname;
static unsigned int n_tx_pkt_sz = PUSCH_UBUF_LEN;
static unsigned int n_tx_pkt_nb = RAWST_BUF_NB_NOW;
//extern unsigned int n_rx_pkt_nb;
static unsigned int n_rx_pkt_nb = RAWSR_BUF_NB_NOW;
#else
static char * str_devname = NULL;
static unsigned int n_tx_pkt_sz = 1500;
static unsigned int n_tx_pkt_nb	= 1000;
#endif
// tp_block_size <= getpagesize() << MAX_ORDER
// tp_block_size <= 4K << 11
// tp_frame_size >= MTU + 32; MTU >= n_tx_pkt_sz
static unsigned int n_tx_blk_sz	= 1024*8;
// tp_block_nr <= "kmalloc-size-max" per "/proc/slabinfo"
// ARMv7+Linux4.9.0: 
// tp_frame_nr = tp_block_size/tp_frame_size*tp_block_nr;
static unsigned int n_tx_blk_nb	= 1024;

static unsigned int n_eth_mtu   = 0;
static unsigned int n_sndbuf_sz	= 512*1024;
static unsigned int m_tx_dgram	= 0;
static unsigned int m_tx_loss	= 0;
static unsigned int m_tx_tcbyps = 0;
static unsigned int m_tx_thread = 0;
#if defined(BUILDING_RRU_UL)
static unsigned int n_tx_oneshot= RADIO_UL_SYM_NB + 1;
#else
static unsigned int n_tx_oneshot= 127;
#endif
static unsigned int m_tx_repeat	= 1;
static unsigned int n_tx_verbos	= 0;

#if !defined(BUILDING_RRU_UL)
static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION] [INTERFACE]\n"
		" -h\tshow this help\n"
		" -m\tset mtu\n"
		" -b\tset buffer size\n"
		" -n\tset buffer count\n"
		" -s\tset packet size\n"
		" -c\tset packet count\n"
		" -j\tset send() period (mask==0)\n"
		" -z\tset socket buffer size\n"
		" -t\tuse dual thread\n"
		" -g\tuse SOCK_DGRAM\n"
		" -l\tPACKET_LOSS(discard wrong packets)\n"
		" -p\tPACKET_QDISC_BYPASS(bypass tc layer)\n"
		" -e\tgenerate error [num]\n"
		" -r\trepeating rather than one-short\n"
		" -v\tbe verbose\n"
		);
}

static void get_args(int argc, char **argv)
{
	int c;
	opterr = 0;
	while ( (c = getopt( argc, argv, "m:b:n:s:c:z:j:e:v:tglprh")) != EOF ) {
		switch ( c ) {
		case 'm': n_eth_mtu   = strtoul( optarg, NULL, 0 ); break;
		case 'b': n_tx_blk_sz = strtoul( optarg, NULL, 0 ); break;
		case 'n': n_tx_blk_nb = strtoul( optarg, NULL, 0 ); break;
		case 's': n_tx_pkt_sz = strtoul( optarg, NULL, 0 ); break;
		case 'c': n_tx_pkt_nb = strtoul( optarg, NULL, 0 ); break;
		case 'z': n_sndbuf_sz = strtoul( optarg, NULL, 0 ); break;
		case 'j': n_tx_oneshot= strtoul( optarg, NULL, 0 ); break;
		case 'v': n_tx_verbos = strtoul( optarg, NULL, 0 ); break;
		case 't': m_tx_thread = 1;                          break;
		case 'g': m_tx_dgram  = 1;                          break;
		case 'l': m_tx_loss   = 1;                          break;
		case 'p': m_tx_tcbyps = 1;                          break;
		case 'r': m_tx_repeat = 1;                          break;
		case 'h': usage(); exit( EXIT_FAILURE );            break;
		case '?':
			if ( isprint (optopt) ) {
				fprintf ( stderr,
				          "ERROR: unrecognised option \"%c\"\n",
				          (char) optopt );
				exit( EXIT_FAILURE );
			}
			break;
		default:
			fprintf( stderr, "ERROR: unrecognised command line option\n");
			exit( EXIT_FAILURE );
			break;
		}
	}

#if 1
	/* take first residual non option argv element as interface name. */
	if ( optind < argc ) {
		str_devname = argv[ optind ];
	}
	if( !str_devname ) {
		fprintf( stderr, "ERROR: No interface was specified\n");
		usage();
		exit( EXIT_FAILURE );
	}
#else
    strncpy(str_devname, argv[1], 31);
#endif
}
#endif

static void print_args(void)
{
	printf( "CURRENT SETTINGS:\n" );
	printf( "str_devname:       %s\n", str_devname );
	printf( "n_eth_mtu(-m):     %d\n", n_eth_mtu );
	printf( "n_tx_blk_sz(-b):   %u\n", n_tx_blk_sz );
	printf( "n_tx_blk_nb(-n):   %u\n", n_tx_blk_nb );
	printf( "n_tx_pkt_sz(-s):   %u\n", n_tx_pkt_sz );
	printf( "n_tx_pkt_nb(-c):   %u\n", n_tx_pkt_nb );
	printf( "n_tx_oneshot(-j):  %d\n", n_tx_oneshot );
	printf( "n_sndbuf_sz(-z):   %u\n", n_sndbuf_sz );
	printf( "m_tx_thread(-t):   %d\n", m_tx_thread );
	printf( "m_tx_dgram(-g):    %d\n", m_tx_dgram );
	printf( "m_tx_loss(-l):     %d\n", m_tx_loss );
	printf( "m_tx_tcbyps(-p):   %d\n", m_tx_tcbyps );
	printf( "m_tx_repeat(-r):   %d\n", m_tx_repeat );
	printf( "n_tx_verbos(-v):   %d\n\n", n_tx_verbos );  
}

static int verify_args(void)
{
	int pgsz = getpagesize();

	printf("current arch: page_size=%d, MAX_ORDER=%d\n", pgsz, MAX_ORDER);
	if (n_tx_blk_sz > getpagesize() << MAX_ORDER) {
		printf("assert-fail: tp_block_size <= getpagesize(%d) << MAX_ORDER(%d)\n");
		return -1;
	}
	if (n_tx_blk_sz & (pgsz - 1) != 0) {
		printf("assert-fail: tp_block_size & (pgsz - 1) == 0\n");
		return -1;
	}

	printf("current arch+platform: /proc/slabinfo: <kmalloc-min>*<min>\n");
	if (n_tx_blk_nb & (sizeof(void *) - 1) != 0) {
		printf("assert-fail: tp_block_size & (sizeof(void *) - 1) == 0\n");
		return -1;
	}

#if defined(BUILDING_RRU_UL)
    printf("RAWST_BUF_NB_MAX=%d, RADIO_UL_SYM_NB=%d\n", RAWST_BUF_NB_MAX, RADIO_UL_SYM_NB);
	if (n_tx_pkt_nb > RAWST_BUF_NB_MAX) {
		printf("assert-fail: n_tx_pkt_nb <= RAWST_BUF_NB_MAX\n");
		return -1;
	}
#endif

    if (!m_tx_thread) {
        if (0 == n_tx_oneshot) {
            printf("assert-fail: 0 != n_tx_oneshot\n");
            return -1;
        }
    } else {
#if defined(BUILDING_RRU_UL)
        if (0 == n_tx_oneshot) {
            printf("assert-fail: 0 != n_tx_oneshot\n");
            return -1;
        }
#endif
    }
	return 0;
}

/* globals */
static int fd_txsock = -1;
static int data_offset = 0;
static struct tpacket_hdr *ps_hdr = NULL;
static struct tpacket_req s_pkt_req;

static unsigned long shot_sent_nb = 0;
static unsigned long shot_tout_nb = 0;
static unsigned long etht_pkts_nb = 0;
static unsigned long etht_byts_nb = 0;
static unsigned long etht_pusch_nb = 0;

#if defined(BUILDING_RRU_UL)
static unsigned long iior_loop_nb = 0;
static ssize_t iior_byts_nb = 0;
static char   *iior_byts_pt = NULL;
# if defined(TMPPS_TRG_UTIMER)
extern void tmpps_tmr_init(void);
extern void tmpps_tmr_exit(void);
# endif
# if defined(BUILDING_RRU_DL)
extern void *main_rx(void *parg);
# endif
#endif

static void handle_sig(int sig)
{
	printf("\nWaiting for process to finish...\n");
    taskstop_rawst = 1;
# if defined(TMPPS_TRG_UGPIOIRQ)
    taskstop_tmirq = 1;
# endif
    taskstop_rawsr = 1;
}

static int setup_txsocket(void)
{
	int ec, fd, opt;
	socklen_t optlen;
	struct ifreq s_ifr;
	struct sockaddr_ll llme;

	fd = socket(PF_PACKET, m_tx_dgram ? SOCK_DGRAM : SOCK_RAW, htons(ETH_P_ALL));
	if (fd == -1) {
		perror("socket");
		return -1;
	}

	/* update the interface mtu */
	strncpy(s_ifr.ifr_name, str_devname, sizeof(s_ifr.ifr_name));
	ec = ioctl(fd, SIOCGIFMTU, &s_ifr);
	if (ec == -1) {
		perror("iotcl SIOCGIFMTU");
		return -2;
	}
	printf("get: the current actual mtu = %d\n", s_ifr.ifr_mtu);
	if (n_eth_mtu < 64) {
		n_eth_mtu = s_ifr.ifr_mtu;
	} else if (n_eth_mtu != s_ifr.ifr_mtu) {
		/*s_ifr.ifr_flags &= ~IFF_UP;
		ec = ioctl(fd, SIOCSIFFLAGS, &s_ifr); */
		s_ifr.ifr_mtu = n_eth_mtu;
		printf("set: mtu = %d\n", s_ifr.ifr_mtu);
		ec = ioctl(fd, SIOCSIFMTU, &s_ifr);
		if (ec == -1) {
			perror("iotcl SIOCSIFMTU");
			return -2;
		}
		/* s_ifr.ifr_flags |= IFF_UP;
		ec = ioctl(fd, SIOCSIFFLAGS, &s_ifr); */
	}
	if (n_tx_blk_sz < n_eth_mtu+TPACKET_HDRLEN-sizeof(struct sockaddr_ll)) {
		printf("assert-fail: buffer_sz >= mtu + data_offset)\n");
		return -2;
	}
	if (n_tx_pkt_sz > n_eth_mtu) {
		printf("assert-fail: packet_sz <= mtu)\n");
		return -2;
	}

	/* bind the socket with a specific interface (man packet:
	   only sll_family, sll_protocol, sll_ifindex is valid) */
	/* SOCK_RAW, TX:
	  if bind() is used (and the out-going interface is specified)
	    then sendto() without 'sockaddr_ll' is permitted as
	    the src_mac and dst_mac are user-defined in the buffer;
	  if bind() is not used then sendto() with 'sockaddr_ll'
	    is a must to specify the out-going interface. */
	/* SOCK_DGRAM, TX:
	  whatever bind() is used or not, sendto() must specify
	    'sockaddr_ll' with 'ifindex' and 'dst_mac' so that
	    the socket itself can fill the src_mac and dst_mac! */
	if (0) {
		memset(&llme, 0, sizeof(llme));
		llme.sll_family  = AF_PACKET;
		llme.sll_protocol= htons(ETH_P_ALL);
		llme.sll_ifindex = if_nametoindex(str_devname);
		ec = bind(fd, (struct sockaddr *)&llme, sizeof(llme));
		if (ec == -1) {
			perror("bind");
			return -2;
		}
	}

	/* set packet loss option */
	opt = m_tx_loss;
	ec = setsockopt(fd, SOL_PACKET, PACKET_LOSS, &opt, sizeof(opt));
	if (ec < 0) {
		perror("setsockopt: PACKET_LOSS");
		return -2;
	}

	/* bypass the QDISC layer */
	opt = m_tx_tcbyps;
	ec = setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS, &opt, sizeof(opt));
	if (ec < 0) {
		perror("setsockopt: PACKET_LOSS");
		return -2;
	}

	/* send TX ring request */
	s_pkt_req.tp_block_size = n_tx_blk_sz;
	s_pkt_req.tp_frame_size = n_tx_blk_sz;
	s_pkt_req.tp_block_nr = n_tx_blk_nb;
	s_pkt_req.tp_frame_nr = n_tx_blk_nb;
	ec = setsockopt(fd, SOL_PACKET, PACKET_TX_RING, &s_pkt_req, sizeof(s_pkt_req));
	if (ec < 0) {
		perror("setsockopt: PACKET_TX_RING");
		return -2;
	}

	/* change send buffer size */
	if (n_sndbuf_sz) {
		printf("set: sndbuf size = %d\n", n_sndbuf_sz);
		ec = setsockopt(fd, SOL_SOCKET, SO_SNDBUF,
				&n_sndbuf_sz, sizeof(n_sndbuf_sz));
		if (ec < 0) {
			perror("setsockopt: SO_SNDBUF");
			return -2;
		}
	} else {
		optlen = sizeof(n_sndbuf_sz);
		ec = getsockopt(fd, SOL_SOCKET, SO_SNDBUF,
				&n_sndbuf_sz, &optlen);
		if (ec < 0) {
			perror("getsockopt: SO_SNDBUF");
			return -2;
		}
		printf("get: send buff size = %d\n", n_sndbuf_sz);
	}

	/* get data offset */
	data_offset = TPACKET_HDRLEN - sizeof(struct sockaddr_ll);
	printf("data offset = %d bytes\n", data_offset);

	/* mmap Tx ring buffers memory */
	ps_hdr = mmap(NULL, s_pkt_req.tp_block_size * s_pkt_req.tp_block_nr,
			PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (ps_hdr == MAP_FAILED) {
		perror("mmap");
		return -3;
	}

	return fd;
}

static int prefill_txblock(void)
{
    int ec, ibuf;
    struct tpacket_hdr *ps;
    char *pdu;

    for (ibuf=0; ibuf<n_tx_blk_nb; ibuf++) {
        ps = (struct tpacket_hdr *)((void *)ps_hdr + n_tx_blk_sz*ibuf);
        pdu = (void *)ps + data_offset;

        /* ethernet header : ETH_HLEN */
        /* UDP socket: ethernet header is built by SOCK_DGRAM based on
           "struct sockaddr_ll" in the sendto() */
        if (!m_tx_dgram)
            pack_ethhdr(fd_txsock, pdu);
        
        /* CANNOT be used because we cannot make sure that 
         "n_tx_blk_nb%n_tx_oneshot==0" */
#if 1
        if (0 == (ibuf % n_tx_oneshot)) {
            /* TMPPS  header: sizeof(struct tmpps_hdr) */
            //pack_tmpps(pdu + PKG_OFFSET);
            /* IP+UDP header: IP_HLEN+UDP_HLEN */
            pack_iphdr_udphdr(fd_txsock, pdu + ETH_HLEN, TMPPS_SIZE);
            continue;
        }
        /* PUSCH  header: sizeof(struct pusch_hdr) */
        //pack_pusch_hdr(pdu + PKG_OFFSET);
        /* IP+UDP header: IP_HLEN+UDP_HLEN */
        pack_iphdr_udphdr(fd_txsock, pdu + ETH_HLEN, PUSCH_SIZE);
#endif
    }
}

static void pack_packet(char *pdu, int ipkt)
{
#if !defined(BUILDING_RRU_UL)
    int j;
    for (j=ETH_ALEN; j<n_tx_pkt_sz; j+=8) {
        *(int *)&pdu[j]   = ipkt;
        *(int *)&pdu[j+4] = (j-ETH_ALEN)/8;
    }
#else
    /* PUSCH  header: sizeof(struct pusch_hdr) */
    pack_pusch_hdr(pdu + PKG_OFFSET);
	/* PUSCH payload: PUSCH_SIZE */
    size_t nb_dst = RADIO_SYM2SMP*sizeof(struct bbu_sample);
	memcpy(pdu + PUSCH_PLDOF, iior_byts_pt, nb_dst);
# if defined(FPGA_COMPRESS_SAMPLES)
    /* FPGA compress samples from 16bit to 8bit */
	iior_byts_pt += nb_dst;
	iior_byts_nb -= nb_dst;
# else
    /* the following 1 line is NOT for function BUT for throughput-test;
       real function codes here should narrow each sample from 16bit to 8bit */
	//memcpy(pdu + PUSCH_PLDOF, iior_byts_pt + asked_nb, asked_nb);
	iior_byts_pt += nb_dst*2;
	iior_byts_nb -= nb_dst*2;
# endif
    /* IP+UDP header: IP_HLEN+UDP_HLEN */
    pack_iphdr_udphdr(fd_txsock, pdu + ETH_HLEN, PUSCH_SIZE);
#endif
}

static int update_txblock(int ibuf, int ipkt)
{
	struct tpacket_hdr *ps;
    char *pdu;

	/* for each packet, wait until the current buffer become free */
	ps = (struct tpacket_hdr *)((void *)ps_hdr + n_tx_blk_sz*ibuf);
    pdu = (void *)ps + data_offset;
	int wait = 1;
	do {
		switch ((volatile uint32_t)ps->tp_status) {
		case TP_STATUS_AVAILABLE:
            if (0 == (ipkt % n_tx_oneshot)) {
                /* TMPPS  header: sizeof(struct tmpps_hdr) */
                pack_tmpps(pdu + PKG_OFFSET);
                /* IP+UDP header: IP_HLEN+UDP_HLEN */
                pack_iphdr_udphdr(fd_txsock, pdu + ETH_HLEN, TMPPS_SIZE);
                ps->tp_len = TMPPS_UBUF_LEN; /* update packet len */
                if (n_tx_verbos > 1) {
                    printf("    rawsr[ibuf%d ipkt%d] tmpps\n",
                        ibuf, ipkt);
                    fflush(stdout);
                }
                wait = 0; /* exit polling */
                break;
            }
			pack_packet(pdu, ipkt);
            ps->tp_len = n_tx_pkt_sz; /* update packet len */
            if (n_tx_verbos > 1) {
                printf("    rawsr[ibuf%d ipkt%d] iiopr[nB=%u, pB=%p]\n",
                    ibuf, ipkt, iior_byts_nb, iior_byts_pt);
                fflush(stdout);
            }
			wait = 0; /* exit polling */
			break;
		case TP_STATUS_WRONG_FORMAT:
            fprintf(stderr, "task_fill: iiopr2rawst[%lu]: "
                "rawsr[ibuf%d ipkt%d] TP_STATUS_WRONG_FORMAT\n",
                iior_loop_nb, ibuf, ipkt);
            return EXIT_FAILURE;
			break;
		default:
			/* nothing to do => schedule : useful if no SMP */
			usleep(0);
			if (taskstop_rawst) {
				printf("task_fill: iiopr2rawst[%lu]: rawsr[ibuf%d ipkt%d] "
					"Abort(CTL+C) from waiting tp_status\n",
					iior_loop_nb, ibuf, ipkt);
				return 1;
			}
			break;
		}
	} while (wait);

	/* set header flag (trigs xmit)*/
	ps->tp_status = TP_STATUS_SEND_REQUEST;
	return 0;
}

static void *task_send(void *arg);

static int switch_to_send(int ibuf, int ipkt)
{
	int ec;
    struct timespec abs = {0, 3000000};/* 3ms */

# if defined(TMPPS_TRG_UGPIOIRQ) \
    || defined(TMPPS_TRG_UTIMER) \
    || defined(TMPPS_TRG_KSIGIO_NOWAIT)
sync_tmpps:
    ec = sem_wait(&g_sem_pps);
    //ec = sem_timedwait(&g_sem_pps, &abs);
    if (-1 == ec) {
        if ((EINTR == errno) && !taskstop_rawst) goto sync_tmpps;
        /*if (ETIMEDOUT == errno) {
            if (0 == (shot_tout_nb++ & 0x7f))
                printf("      sem_timedwait(&g_sem_pps, 4ms) timeout %lu\n",
                    shot_tout_nb);
                fflush(stdout);
        }*/
        fprintf(stderr, "task_fill: iiopr2rawst[%lu]: ibuf%d ipkt%d ",
            iior_loop_nb, ibuf, ipkt);
        perror("sem_wait(&g_sem_pps)");
        return ec;
    }
# endif
# if defined(TMPPS_TRG_KSIGIO_WAIT)
    /* 'CANWAIT' means blocking */
    ec = pps_fetch_src(0, &pps_handle, &pps_mode);
    if ((ec < 0) && (errno != ETIMEDOUT))
        return ec;
# endif
}

/* fill circular buffers
   one buffer holds one packet */
static void *task_fill(void *arg)
{
	int ec;
    unsigned long ipkt=0, ibuf=0;
#if defined(BUILDING_CTM0_VERSION) || defined(BUILDING_CTM1_VERSION)
	struct timespec old, new;
#endif

	printf("task_fill: start\n");

redo:
#if defined(BUILDING_RRU_UL)
	if (0 == iior_byts_nb) {
        if (n_tx_verbos > 0) {
            printf("  iiopr2rawst[%lu] start\n", iior_loop_nb);
            fflush(stdout);
        }
#if defined(BUILDING_CTM0_VERSION) || defined(BUILDING_CTM1_VERSION)
        clock_gettime(CLOCK_MONOTONIC, &old);
#endif
		iior_byts_nb = iio_buffer_refill(iiopr_cbuf[0]);
		if (iior_byts_nb < 0) {
			fprintf(stderr, "task_fill: iiopr2rawst[%lu]: libiio_refill %s\n",
                iior_loop_nb, strerror(-iior_byts_nb));
			return (void *)EXIT_FAILURE;
		}/*
        if (iior_byts_nb != IIOPR_SMP_NB_NOW*sizeof(struct bbu_sample)) {
            printf("assert-fail: iiopr2rawst[%lu]: "\
                "libiio_refill(%lu) != RAWST_BUF_NB_NOW*2400\n",
                iior_loop_nb, iior_byts_nb);
			return (void *)EXIT_FAILURE;
        }*/
		iior_byts_pt = iio_buffer_first(iiopr_cbuf[0], iiopr_chni[0]);
#if defined(BUILDING_CTM0_VERSION) || defined(BUILDING_CTM1_VERSION)
        clock_gettime(CLOCK_MONOTONIC, &new);
        printf("tm_us: iio_buffer_refill[%lu] %.0f\n", 
            iior_loop_nb, elapse_us(&new, &old));
#endif
		if (n_tx_verbos > 0) {
			printf("  libiio_refill()=%u, iiobuf_first()=%p\n",
				iior_byts_nb, iior_byts_pt);
            fflush(stdout);
        }
	}
#endif
    if (n_tx_verbos > 0) {
        printf("  fill_cbuf start: ibuf=%d\n", ibuf);
        fflush(stdout);
    }
	for (ipkt=0; ipkt < n_tx_pkt_nb; ipkt++) {
#if defined(BUILDING_CTM0_VERSION)
        clock_gettime(CLOCK_MONOTONIC, &old);
#endif
#if defined(BUILDING_CTM1_VERSION)
        if (0 == ipkt % n_tx_oneshot)
            clock_gettime(CLOCK_MONOTONIC, &old);
#endif
        ec = update_txblock(ibuf, ipkt);
        if (0 != ec) return (void *)ec;
#if defined(BUILDING_CTM0_VERSION)
        clock_gettime(CLOCK_MONOTONIC, &new);
        printf("tm_us: update_txblock[%lu,%lu] %.0f\n",
            iior_loop_nb, ipkt, elapse_us(&new, &old));
#endif
        if ((n_tx_oneshot-1 == ipkt % n_tx_oneshot) || (ipkt == n_tx_pkt_nb)) {
            if (n_tx_verbos > 0) {
                printf("  wait the signal to send shot[%lu]\n",
                    (iior_loop_nb*n_tx_pkt_nb + ipkt)/n_tx_oneshot);
                fflush(stdout);
            }
#if defined(BUILDING_CTM1_VERSION)
            clock_gettime(CLOCK_MONOTONIC, &new);
            printf("tm_us: update_txblock[%lu, %lu] %.0f\n",
                iior_loop_nb, ipkt/n_tx_oneshot, elapse_us(&new, &old));
#endif
#if defined(BUILDING_CTM0_VERSION) || defined(BUILDING_CTM1_VERSION)
            clock_gettime(CLOCK_MONOTONIC, &old);
#endif
            ec = switch_to_send(ibuf, ipkt);
            if (ec < 0) return (void *)ec;
#if defined(BUILDING_CTM0_VERSION) || defined(BUILDING_CTM1_VERSION)
            clock_gettime(CLOCK_MONOTONIC, &new);
            printf("tm_us: switch_to_send[%lu, %lu] %.0f\n", 
                iior_loop_nb, ipkt/n_tx_oneshot, elapse_us(&new, &old));
            clock_gettime(CLOCK_MONOTONIC, &old);
#endif
            if (!m_tx_thread) {
                do {
                    ec = (int) task_send((void*)0);
                } while (ec < 0);
            }
#if defined(BUILDING_CTM0_VERSION) || defined(BUILDING_CTM1_VERSION)
            clock_gettime(CLOCK_MONOTONIC, &new);
            printf("tm_us: task_send[%lu, %lu] %.0f\n",
                iior_loop_nb, ipkt/n_tx_oneshot, elapse_us(&new, &old));
#endif
        }

        if (++ibuf == n_tx_blk_nb) ibuf = 0; /* circling ibuf */
	}
    if (n_tx_verbos > 0) {
        printf("  fill_cbuf end: ibuf=%d\n", ibuf);
        fflush(stdout);
    }/*
    if (0 != iior_byts_nb) {
        printf("assert-failed: iior_byts_nb!=0 after\n");
        return (void *)1;
    }*/
    if (n_tx_verbos > 0) {
        printf("  iiopr2rawst[%lu]   end\n", iior_loop_nb);
        fflush(stdout);
    }
    iior_loop_nb++;

	if (m_tx_repeat) {
		if (!taskstop_rawst) goto redo;
	}

	printf("task_fill:   end\n");
	return (void *)0;
}

/* send those buffers with TP_STATUS_SEND_REQUEST
   THREAD:
     sendto() works in blocking mode:
     keep waiting until end of kernel transfering
   Function:
     sendto() works in non-blocking mode:
     Don't wait end of transfer
     triger kernel to scans over ring then return
*/
static void *task_send(void *arg)
{
	int ec;
	int blocking = (int)arg;
#if defined(BUILDING_RRU_UL)
    unsigned long pattern_bytes = n_tx_pkt_sz*(n_tx_oneshot-1) + TMPPS_UBUF_LEN;
#endif
	if (blocking) printf("task_send: start\n");

    /* one time: send all buffers with TP_STATUS_SEND_REQUEST */
	do {
        // NOT re-enter already called in tmpps_handler()
        //measure_interval(TMPPS_INTERVAL_US, "task_send");
        ec = sendto(fd_txsock, NULL, 0, blocking ? 0 : MSG_DONTWAIT,
                (struct sockaddr *)&g_txaddrll, sizeof(struct sockaddr_ll));
		if (ec < 0) {
            /* blocking  mode */
            if ((EINTR == errno) && !taskstop_rawst) continue;
            /* non-block mode */
            //if (ENOBUFS == errno) { usleep(100); continue; }
            if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) { usleep(40); continue; }
            fprintf(stderr, "%s_send[shot%lu]: sendto(%s)=%d ",
                blocking ? "task" : "func", shot_sent_nb,
                blocking ? "WAIT" : "NOWAIT", ec);
			perror("");
			break;
		} else if (ec == 0) {
			/* nothing to do => schedule : useful for thread_mode */
			usleep(0);
		} else {
		    shot_sent_nb++;
            if (n_tx_verbos > 0) {
                printf("%s_send[shot%lu]: sent %dB\n",
                    blocking ? "task" : "func", shot_sent_nb, ec);
                fflush(stdout);
            }
		    etht_byts_nb += ec;
#if defined(BUILDING_RRU_UL)
            if (0 != (ec % pattern_bytes)) {
                printf("%s_send[shot%lu]: unexpected executing pattern\n",
                    blocking ? "task" : "func", shot_sent_nb);
                fflush(stdout);
            }
            etht_pkts_nb += ec/pattern_bytes*n_tx_oneshot;
#else
			etht_pkts_nb += ec/n_tx_pkt_sz;
#endif
		}
	} while (blocking && !taskstop_rawst);

	if (blocking) printf("task_send:   end\n");
	return (void *)ec;
}

/* After TX done, check errors of circular buffers */
/* TODO: display header of all blocks if error occurs */
static void check_txring(void)
{
	unsigned int err_nb, ibuf;
	struct tpacket_hdr *ps;

	err_nb = 0;
	for (ibuf=0; ibuf<n_tx_blk_nb; ibuf++) {
		ps = (struct tpacket_hdr *)((void *)ps_hdr + (n_tx_blk_sz*ibuf));
		switch ((volatile uint32_t)ps->tp_status) {
		case TP_STATUS_SEND_REQUEST:
			printf("check_txring: ibuf%d not been sent\n", ibuf);
			err_nb++;
			break;
		case TP_STATUS_LOSING:
			printf("check_txring: ibuf%d transfer with errors\n", ibuf);
			err_nb++;
			break;
		default:
			break;
		}
	}
	printf("check_txring: number of error: %u\n", err_nb);
}

static void close_txsocket(void)
{
	munmap((void *)ps_hdr, n_tx_blk_sz * n_tx_blk_nb);
	close(fd_txsock);
}

/*
Optimize(2018-05-02) for RealTime application
0, Kernel:
    disable power-management; 
    disable hyperthreading and out-of-order execution
1, call mlockall() as soon as possible from main()
2, avoid dynamic memory allocation/freeing while in RT critical path by
   reserving a memory pool to do malloc/free
3, create all threads with explicit secheduler attributes at startup time
4, not configure threads with priority 99
5, touch each page of the entire stack of each thread
6, never use system calls which generates page faults, such as fopen()
7, use mmap to pass data around
*/
#define RT_TASKS_SCHED SCHED_RR
//#define RT_TASKS_SCHED SCHED_FIFO /* !!! NOT GOOD for faster task-switch */
int main(int argc, char ** argv)
{
	int ec;
	pthread_t pid_send, pid_fill;
	pthread_attr_t attr_send, attr_fill;
	struct sched_param para_send, para_fill;

#if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)
    common_getargs(argc, argv);
    iio_path_x = 0x03; /* IIO_RX and IIO_TX */
# if defined(FPGA_COMPRESS_SAMPLES)
    libiio_app_startup(n_tx_pkt_nb*RADIO_SYM2SMP/2, n_rx_pkt_nb*RADIO_SYM2SMP/2);
# else
    libiio_app_startup(n_tx_pkt_nb*RADIO_SYM2SMP, n_rx_pkt_nb*RADIO_SYM2SMP);
# endif
#else
    get_args(argc, argv);
#endif

    print_args();
    ec = verify_args();
    if (ec < 0) goto exit0;
#if defined(BUILDING_RRU_UL)
    n_tx_pkt_nb += n_tx_pkt_nb/RADIO_UL_SYM_NB;
#endif

    /* prevent that memory is paged to the swap area */
    //mlockall(MCL_CURRENT | MCL_FUTURE);
    
    signal(SIGINT, handle_sig);/* Listen to ctrl+c */
    
# if defined(BUILDING_RRU_DL)
    pthread_t pid_rawsrx;
    pthread_attr_t attr_rawsrx;
    struct sched_param para_rawsrx;
    /* Set thread priorities, scheduler, ... */
    pthread_attr_init(&attr_rawsrx);
    pthread_attr_setschedpolicy(&attr_rawsrx, RT_TASKS_SCHED);
    para_rawsrx.sched_priority = 50 + 1;
    pthread_attr_setschedparam(&attr_rawsrx, &para_rawsrx);
    /* force to use the policy and priority specified by the attributes */
    pthread_attr_setinheritsched(&attr_rawsrx, PTHREAD_EXPLICIT_SCHED);
    ec = pthread_create(&pid_rawsrx, &attr_rawsrx, main_rx, NULL);
    if (ec != 0) {
        perror("task_rawsr: create");
        goto exit1;
    }
# endif

	fd_txsock = setup_txsocket();
	switch (fd_txsock) {
	case -1: ec = EXIT_FAILURE; goto exit1;
	case -2: ec = EXIT_FAILURE; goto exit2;
	case -3: ec = EXIT_FAILURE; goto exit3;
	default: break;
	}

	sock_fill_txaddr();
    prefill_txblock();

#if defined(BUILDING_RRU_UL)
    ec = sem_init(&g_sem_pps, 0, 0);
    if (ec == -1) {
        perror("sem_init(&g_sem_pps)");
        goto exit3;
    }
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_init();
# endif
# if defined(TMPPS_TRG_UGPIOIRQ) \
    || defined(TMPPS_TRG_KSIGIO_NOWAIT)
    pthread_t pid_tmirq;
    pthread_attr_t attr_tmirq;
    struct sched_param para_tmirq;
    /* Set thread priorities, scheduler, ... */
    pthread_attr_init(&attr_tmirq);
    pthread_attr_setschedpolicy(&attr_tmirq, RT_TASKS_SCHED);
    para_tmirq.sched_priority = 60; /* higher */
    pthread_attr_setschedparam(&attr_tmirq, &para_tmirq);
    /* force to use the policy and priority specified by the attributes */
    pthread_attr_setinheritsched(&attr_tmirq, PTHREAD_EXPLICIT_SCHED);
#  if defined(TMPPS_TRG_UGPIOIRQ)
    ec = pthread_create(&pid_tmirq, &attr_tmirq, task_tmpps_irq, 0);
    if (ec != 0) {
        perror("task_tmpps_irq: create");
        goto exit4;
    }
#  endif
#  if defined(TMPPS_TRG_KSIGIO_NOWAIT)
    ec = pthread_create(&pid_tmirq, &attr_tmirq, task_tmpps_pps, 0);
    if (ec != 0) {
        perror("task_tmpps_pps: create");
        goto exit4;
    }
#  endif
# endif
# if defined(TMPPS_TRG_KSIGIO_WAIT)
    ec = pps_find_src("/dev/pps0", &pps_handle, &pps_mode);
    if (ec < 0) {
        goto exit4;
    }
# endif
#endif

	/* Set thread priorities, scheduler, ... */
	pthread_attr_init(&attr_fill);
	pthread_attr_setschedpolicy(&attr_fill, RT_TASKS_SCHED);
	para_fill.sched_priority = 50;
	pthread_attr_setschedparam(&attr_fill, &para_fill);
    /* force to use the policy and priority specified by the attributes */
    pthread_attr_setinheritsched(&attr_fill, PTHREAD_EXPLICIT_SCHED);
	/* Start fill_thread that fills dummy data in circular buffers */
	ec = pthread_create(&pid_fill, &attr_fill, task_fill, (void *)ps_hdr);
	if (ec != 0) {
		perror("task_fill: create");
		ec = EXIT_FAILURE;
		goto exit5;
	}

	/* Start send_THREAD in blocking mode only on SMP mode */
	if (m_tx_thread) {
		pthread_attr_init(&attr_send);
		pthread_attr_setschedpolicy(&attr_send, RT_TASKS_SCHED);
		para_send.sched_priority = 50 + 10;
		pthread_attr_setschedparam(&attr_send, &para_send);
        /* force to use the policy and priority specified by the attributes */
        pthread_attr_setinheritsched(&attr_send, PTHREAD_EXPLICIT_SCHED);
		ec = pthread_create(&pid_send, &attr_send, task_send, (void *)1);
		if (ec != 0) {
			perror("task_send: create");
			ec = EXIT_FAILURE;
			goto exit6;
		}
	}

	/* Wait/blocking end of fill_thread */
	pthread_join(pid_fill, NULL);
	if (m_tx_thread) {
		/* Wait end of send_THREAD */
		pthread_join(pid_send, NULL);
        printf("task_send exit\n");
        pthread_attr_destroy(&attr_send);
    	pthread_exit(&pid_send);
	}

	/* in case any un-drawn packages after stopping send_THREAD */
	do {
		ec = (int) task_send((void*)0);
		printf("Loop until queue empty (%d)\n", ec);
	} while (ec != 0);

	clock_gettime(CLOCK_MONOTONIC, &tm_te);
	check_txring();

    printf("\n");
#if 0
	struct tpacket_stats_v3 stats;
	int len = sizeof(stats);
	ec = getsockopt(fd_txsock, SOL_PACKET, PACKET_STATISTICS, &stats, &len);
	if (ec < 0) {
		perror("getsockopt PACKET_STATISTICS");
		goto exit4;
	}
	if (stats.tp_packets > 0) {
		printf("TX PACKET_STATISTICS: %u pkts, %u dropped, freeze_q_cnt %u\n",
			stats.tp_packets, stats.tp_drops, stats.tp_freeze_q_cnt);
	}
#endif
	if (etht_pkts_nb > 0) {
		double tm_us = elapse_us(&tm_te, &tm_ts);
	    printf("TX %lu in %.0f us (%7.3f Mbps, %7.3f us)\n",
		    etht_pkts_nb, tm_us,
		    tm_us ? etht_pkts_nb*n_tx_pkt_sz/tm_us*8 : 0,
		    tm_us/etht_pkts_nb);
	}
#if defined(BUILDING_RRU_UL)
    display_tx_perf(&tm_te, &tm_ts);
#endif

	ec = EXIT_SUCCESS;
exit6:
# if !defined(TMPPS_TRG_UTIMER)
    printf("task_fill exit\n");
    pthread_attr_destroy(&attr_fill);
	pthread_exit(&pid_fill);
# endif
exit5:
#if defined(BUILDING_RRU_UL)
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_exit();
# endif
# if defined(TMPPS_TRG_UGPIOIRQ)
    printf("task_tmpps_irq: exit\n");
    pthread_attr_destroy(&attr_tmirq);
    pthread_exit(&pid_tmirq);
# endif
# if defined(TMPPS_TRG_KSIGIO_WAIT)
    time_pps_destroy(pps_handle);
# endif
exit4:
    sem_destroy(&g_sem_pps);
#endif
exit3:
    printf("exit3: munmap\n");
	munmap((void *)ps_hdr, n_tx_blk_sz * n_tx_blk_nb);
exit2:
    printf("exit2: close(fd_txsock)\n");
	close(fd_txsock);
exit1:
# if defined(BUILDING_RRU_DL)
    printf("task_rawsr: exit\n");
    pthread_attr_destroy(&attr_rawsrx);
    pthread_exit(&pid_rawsrx);
# endif
exit0:
# if defined(BUILDING_RRU_UL) || defined(BUILDING_RRU_DL)
    libiio_app_shutdown();
# endif
	return ec;
}
