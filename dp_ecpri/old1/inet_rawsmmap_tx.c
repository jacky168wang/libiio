/*
-------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
-------------------------------------------------------------------------------
+ Why use PACKET_MMAP
+ How to use mmap() directly to improve transmission process
+ PACKET_MMAP settings and constraints
+ Mapping and use of the circular buffer (ring)

-------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ What TPACKET versions are available and when to use them?
-------------------------------------------------------------------------------
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

-------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ How to use mmap() directly to improve transmission process
++ Transmission process
--------------------------------------------------------------------------------
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

-------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ PACKET_QDISC_BYPASS
-------------------------------------------------------------------------------
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

-------------------------------------------------------------------------------
man package
-------------------------------------------------------------------------------
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

-------------------------------------------------------------------------------
https://sites.google.com/site/packetmmap/
kernel: af_packet.c -> tpacket_snd()
Author:
-------------------------------------------------------------------------------
  Kernel 2.6 : txring_patch
    Version 1.4: (May 11 2009) (selected for 2.6.31 merge window)
        Fixed trailing white spaces 
    Version 1.3: (May 3 2009)
        Fixed RX poll issue 
    Version 1.2: (May 2 2009)
        cleaned code according to Linux coding rules
        removed static function not used 
    Version 1.1: (April 21 2009)
        moved destructor argument to shared info
        renamed tp_status values for TX_RING 
    Version 1.0: (March 21 2009)
        replaced busy flag with pgvec mutex
        removed useless while loop
        removed shutdown flag
        skb allocation is now always blocking 
    Version 0.9: (March 19 2009)
        added setsockopt(PACKET_LOSS):
            1: ignore wrong formatted packet and continue
            0 : set status to TPSTATUS_LOSING and exit from send() procedure 
        Removed useless while() loop on release
        Added schedule() call on blocking loop 
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

  Userspace example
	Usage:
	  Sending packet of 7000bytes, socket buffer size to 100 packets (7000*100)
	    "./packet_mmap -s7000 -m7200 -z700000 eth0"
      Using multi-thread: "./packet_mmap -t eth0"
      Using SOCK_DGRAM to compare: "./packet_mmap -g eth0"

      Sending packet of 2490bytes, default socket buffer size, total 655360 packages:
        "./inet_rawsmmap_tx -s2490 -m3000 -c655360 eth0":    515.704Mbps on a10ad9371
        "./inet_rawsmmap_tx -s2490 -m3000 -c655360 -g eth0": 512.691Mbps on a10ad9371
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


/* params */
char * str_devname = NULL;
#if defined(BUILDING_RRU_UL)
static unsigned int c_packet_sz = SUBCARRIER_NTXBUF_SZ; //2462, 2490
static unsigned int c_packet_nb = IIO_PORT_SAMPLES/SUBCARRIER_SAMPLES; //873
#else
static unsigned int c_packet_sz = 1500;
static unsigned int c_packet_nb	= 1000;
#endif
// tp_block_size <= getpagesize() << MAX_ORDER
// tp_block_size <= 4K << 11
// tp_frame_size >= MTU + 32; MTU >= c_packet_sz
static unsigned int c_buffer_sz	= 1024*8;
// tp_block_nr <= "kmalloc-size-max" per "/proc/slabinfo"
// ARMv7+Linux4.9.0: 
// tp_frame_nr = tp_block_size/tp_frame_size*tp_block_nr;
static unsigned int c_buffer_nb	= 1024;

static unsigned int c_mtu		= 0;
static unsigned int c_sndbuf_sz	= 0;
static unsigned int c_send_mask	= 127;
static unsigned int c_error		= 0;
static int mode_dgram	= 0;
static int mode_thread	= 0;
static int mode_loss	= 0;
static int mode_tcbypass= 0;
static int mode_repeat  = 0;
static int mode_verbose	= 0;

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./packet_mmap [OPTION] [INTERFACE]\n"
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

void get_args(int argc, char **argv)
{
	int c;
	opterr = 0;
	while ( (c = getopt( argc, argv, "m:b:n:s:c:z:j:e:tglprvh")) != EOF ) {
		switch ( c ) {
		case 'm': c_mtu       = strtoul( optarg, NULL, 0 ); break;
		case 'b': c_buffer_sz = strtoul( optarg, NULL, 0 ); break;
		case 'n': c_buffer_nb = strtoul( optarg, NULL, 0 ); break;
		case 's': c_packet_sz = strtoul( optarg, NULL, 0 ); break;
		case 'c': c_packet_nb = strtoul( optarg, NULL, 0 ); break;
		case 'z': c_sndbuf_sz = strtoul( optarg, NULL, 0 ); break;
		case 'j': c_send_mask = strtoul( optarg, NULL, 0 ); break;
		case 'e': c_error     = strtoul( optarg, NULL, 0 ); break;
		case 't': mode_thread = 1;                          break;
		case 'g': mode_dgram  = 1;                          break;
		case 'l': mode_loss   = 1;                          break;
		case 'p': mode_tcbypass = 1;                        break;
		case 'r': mode_repeat = 1;                          break;
		case 'v': mode_verbose= strtoul( optarg, NULL, 0 ); break;
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
	/* take first residual non option argv element as interface name. */
	if ( optind < argc ) {
		str_devname = argv[ optind ];
	}
	if( !str_devname ) {
		fprintf( stderr, "ERROR: No interface was specified\n");
		usage();
		exit( EXIT_FAILURE );
	}

	printf( "CURRENT SETTINGS:\n" );
	printf( "str_devname:       %s\n", str_devname );
	printf( "c_mtu(-m):         %d\n", c_mtu );
	printf( "c_buffer_sz(-b):   %u\n", c_buffer_sz );
	printf( "c_buffer_nb(-n):   %u\n", c_buffer_nb );
	printf( "c_packet_sz(-s):   %u\n", c_packet_sz );
	printf( "c_packet_nb(-c):   %u\n", c_packet_nb );
	printf( "c_send_mask(-j):   %d\n", c_send_mask );
	printf( "c_sndbuf_sz(-z):   %u\n", c_sndbuf_sz );
	printf( "mode_thread(-t):   %d\n", mode_thread );
	printf( "mode_dgram(-g):    %d\n", mode_dgram );
	printf( "mode_loss(-l):     %d\n", mode_loss );
	printf( "mode_tcbypass(-p): %d\n", mode_tcbypass );
	printf( "mode_repeat(-r):   %d\n", mode_repeat );
	printf( "mode_verbose(-v):  %d\n\n", mode_verbose );
}

int verify_args(void)
{
	int pgsz = getpagesize();
	printf("current arch: page_size=%d, MAX_ORDER=%d\n", pgsz, MAX_ORDER);
	if (c_buffer_sz > getpagesize() << MAX_ORDER) {
		printf("assert failed (buffer_sz <= getpagesize() << MAX_ORDER)\n");
		return -1;
	}
	if (c_buffer_sz & (pgsz - 1) != 0) {
		printf("assert failed (buffer_sz & (pgsz - 1) == 0)\n");
		return -1;
	}

	printf("current arch+platform: /proc/slabinfo: <kmalloc-min>*<min>\n");
	if (c_buffer_nb & (sizeof(void *) - 1) != 0) {
		printf("assert failed (buffer_nb & (sizeof(void *) - 1) == 0)\n");
		return -1;
	}

	if (c_buffer_sz < c_mtu + TPACKET_HDRLEN - sizeof(struct sockaddr_ll)) {
		printf("assert failed (buffer_sz >= mtu + data_offset)\n");
		return -1;
	}

	if (c_packet_sz > c_mtu) {
		printf("assert failed (packet_sz <= mtu)\n");
		return -1;
	}
#if defined(BUILDING_RRU_UL)
	if (c_packet_nb > IIO_PORT_SAMPLES/SUBCARRIER_SAMPLES) {
		printf("assert failed (packet_nb <= IIO_PORT_SAMPLES/SUBCARRIER_SAMPLES)\n");
		return -1;
	}
#endif

	return 0;
}

/* globals */
static volatile int fd_sock;
static volatile int data_offset = 0;
static struct sockaddr_ll peer_addr;
static struct sockaddr_ll *ps_peer_addr = NULL;
static volatile struct tpacket_hdr *ps_header = NULL;
static volatile int shutdown_flag = 0;
static struct tpacket_req s_packet_req;

static unsigned long mainloop_nb = 0;
static unsigned long packets_total = 0;

static volatile bool process_stop;
static void handle_sig(int sig)
{
	printf("\nWaiting for process to finish...\n");
	process_stop = true;
}

static int setup_socket(char *netdev)
{
	int ec, fd, opt;
	socklen_t optlen;
	struct ifreq s_ifr;
	struct sockaddr_ll llme;

	fd = socket(PF_PACKET, mode_dgram? SOCK_DGRAM : SOCK_RAW, htons(ETH_P_ALL));
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
	if (c_mtu < 64) {
		c_mtu = s_ifr.ifr_mtu;
	} else if (c_mtu != s_ifr.ifr_mtu) {
		/*s_ifr.ifr_flags &= ~IFF_UP;
		ec = ioctl(fd, SIOCSIFFLAGS, &s_ifr); */
		s_ifr.ifr_mtu = c_mtu;
		printf("set: mtu = %d\n", s_ifr.ifr_mtu);
		ec = ioctl(fd, SIOCSIFMTU, &s_ifr);
		if (ec == -1) {
			perror("iotcl SIOCSIFMTU");
			return -2;
		}
		/* s_ifr.ifr_flags |= IFF_UP;
		ec = ioctl(fd, SIOCSIFFLAGS, &s_ifr); */
	}

	ec = verify_args();
	if (ec < 0) return -2;

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
		llme.sll_ifindex = if_nametoindex(netdev);
		ec = bind(fd, (struct sockaddr *)&llme, sizeof(llme));
		if (ec == -1) {
			perror("bind");
			return -2;
		}
	}

	/* set packet loss option */
	opt = mode_loss;
	ec = setsockopt(fd, SOL_PACKET, PACKET_LOSS, &opt, sizeof(opt));
	if (ec < 0) {
		perror("setsockopt: PACKET_LOSS");
		return -2;
	}

	/* bypass the QDISC layer */
	opt = mode_tcbypass;
	ec = setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS, &opt, sizeof(opt));
	if (ec < 0) {
		perror("setsockopt: PACKET_LOSS");
		return -2;
	}

	/* send TX ring request */
	s_packet_req.tp_block_size = c_buffer_sz;
	s_packet_req.tp_frame_size = c_buffer_sz;
	s_packet_req.tp_block_nr = c_buffer_nb;
	s_packet_req.tp_frame_nr = c_buffer_nb;
	ec = setsockopt(fd, SOL_PACKET, PACKET_TX_RING, &s_packet_req, sizeof(s_packet_req));
	if (ec < 0) {
		perror("setsockopt: PACKET_TX_RING");
		return -2;
	}

	/* change send buffer size */
	if (c_sndbuf_sz) {
		printf("set: send buff size = %d\n", c_sndbuf_sz);
		ec = setsockopt(fd, SOL_SOCKET, SO_SNDBUF,
				&c_sndbuf_sz, sizeof(c_sndbuf_sz));
		if (ec < 0) {
			perror("setsockopt: SO_SNDBUF");
			return -2;
		}
	} else {
		ec = getsockopt(fd, SOL_SOCKET, SO_SNDBUF,
				&c_sndbuf_sz, &optlen);
		if (ec < 0) {
			perror("getsockopt: SO_SNDBUF");
			return -2;
		}
		printf("get: send buff size = %d\n", c_sndbuf_sz);
	}

	/* get data offset */
	data_offset = TPACKET_HDRLEN - sizeof(struct sockaddr_ll);
	printf("data offset = %d bytes\n", data_offset);

	/* mmap Tx ring buffers memory */
	ps_header = mmap(NULL, s_packet_req.tp_block_size * s_packet_req.tp_block_nr,
			PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (ps_header == MAP_FAILED) {
		perror("mmap");
		return -3;
	}

	return fd;
}

/* fill peer sockaddr for SOCK_DGRAM */
void fill_peeraddr(void)
{
	memset(&peer_addr, 0, sizeof(peer_addr));
	peer_addr.sll_ifindex = if_nametoindex(str_devname);
	peer_addr.sll_family  = AF_PACKET;
	peer_addr.sll_protocol= htons(mode_dgram?ETH_P_IP:ETH_P_ALL);
	if (mode_dgram) {
		peer_addr.sll_halen   = ETH_ALEN;
		peer_addr.sll_addr[0] = 0xff; peer_addr.sll_addr[1] = 0xff;
		peer_addr.sll_addr[2] = 0xff; peer_addr.sll_addr[3] = 0xff;
		peer_addr.sll_addr[4] = 0xff; peer_addr.sll_addr[5] = 0xff;
	}
	ps_peer_addr = &peer_addr;
}

int fill_ethhdr(char *pdu)
{
	int j;

	/* ethernet header: dst mac */
	for (j=0; j<ETH_ALEN; j++) pdu[j] = 0xff;

	/* ethernet header: src mac */
	int ec;
	struct ifreq s_ifr;
	memset(&s_ifr, 0, sizeof(s_ifr));
	strncpy(s_ifr.ifr_name, str_devname, sizeof(s_ifr.ifr_name));
	ec = ioctl(fd_sock, SIOCGIFHWADDR, &s_ifr);
	if (ec < 0) {
		perror("ioctl() SIOCGIFHWADDR");
		return ec;
	}
	memcpy(pdu + j, s_ifr.ifr_hwaddr.sa_data, ETH_ALEN);
	j += ETH_ALEN;

	/* ethernet header: protocol */
	pdu[j++] = (ETH_P_IP >> 8) & 0xff;/* network/big-endian */
	pdu[j++] = (ETH_P_IP >> 0) & 0xff;

	return 0;
}

#if defined(BUILDING_RRU_UL)
extern int raws_pack_ethhdr(int fd, uint8_t *buf);
extern int pack_iphdr_udphdr(int fd, uint8_t *buf, size_t l5len);
#endif
void prefill_txring(void)
{
	int ec, ibuf;
	char *pdu;

	for (ibuf=0; ibuf<c_buffer_nb; ibuf++) {

		pdu = (void *)ps_header + c_buffer_sz*ibuf + data_offset;

#if defined(BUILDING_RRU_UL)
		/* ethernet header : ETH_HLEN */
		ec = raws_pack_ethhdr(fd_sock, pdu);

		/* IP+UDP	header : IP_HLEN+UDP_HLEN */
		ec = pack_iphdr_udphdr(fd_sock, pdu + ETH_HLEN, PUSCH_SIZE);
#else
		/* ethernet header : ETH_HLEN */
		ec = fill_ethhdr(pdu);
#endif
	}
}

#if defined(BUILDING_RRU_UL)
#include <iio.h>
extern struct iio_buffer *iiorxcbuf[CHIP_NUM_TOTAL];
extern struct iio_channel *rxpi[PORT_NUM_TOTAL];
static ssize_t iiorxbytes = 0;
static char *piiorxbuf = NULL;
#endif
void format_packet(char *pdu) /* ethernet PDU */
{
	int j;

	if (mode_dgram) {
		/* ethernet header is built by SOCK_DGRAM based on
		   "struct sockaddr_ll" in the sendto() */
	} else {
		/* ethernet header : ETH_HLEN -> prebuilt  */
		//fill_ethhdr(pdu);
	}

#if defined(BUILDING_RRU_UL)
	/* PUSCH	header : sizeof(struct pusch_hdr) */
	pack_pusch_hdr(pdu + PKG_OFFSET);
	/* PUSCH   payload : PUSCH_SIZE */
	ASSERT(1 == PORT_PER_DEVICE);
	size_t nb = SUBCARRIER_SAMPLES*PORT_PER_DEVICE*2;
# if 0 /* FPGA compress samples from 16bit to 8bit */
	piiorxbuf += nb;
	iiorxbytes -= nb;
# else
	piiorxbuf += nb*2;
	iiorxbytes -= nb*2;
# endif
	if (mode_verbose > 1)
		printf("format_packet() update iiorxbytes=%u, piiorxbuf=%p\n",
			iiorxbytes, piiorxbuf);
#else
	/* ethernet payload */
	for (j=ETH_HLEN; j<c_packet_sz; j++)
		pdu[j] = j-ETH_HLEN;
#endif
}

/* THREAD:
     sendto() works in blocking mode:
     keep waiting until end of kernel transfering
   Function:
     sendto() works in non-blocking mode:
     triger kernel to scans over ring then return
*/
void *task_send(void *arg)
{
	int ec_send;
	int blocking = (int)arg;

	if (blocking) printf("start send() thread\n");

	do {
		/* send all buffers with TP_STATUS_SEND_REQUEST */
		if (mode_verbose > 0) printf("send() start\n");
		ec_send = sendto(fd_sock, NULL, 0, blocking?0:MSG_DONTWAIT,
				(struct sockaddr *)ps_peer_addr, sizeof(struct sockaddr_ll));
		if (mode_verbose > 0) printf("send() end (ec=%d)\n", ec_send);
		if (ec_send < 0) {
			perror("send");
			break;
		} else if (ec_send == 0) {
			/* nothing to do => schedule : useful if no SMP */
			usleep(0);
		} else {
			packets_total += ec_send/c_packet_sz;
			if (mode_verbose > 0) printf("sent %lu packets (+%d-packet)\n",
				packets_total, ec_send/c_packet_sz);
			fflush(0);
		}
	} while (blocking && !shutdown_flag);

	if (blocking) printf("endof send() thread\n");

	return (void *)ec_send;
}

/* fill circular buffers
   one buffer holds one packet */
void *task_fill(void *arg)
{
	int ec, ipkt, ibuf=0, first_loop=1;
	struct tpacket_hdr *ps;

	printf("start fill() thread\n");

task_fill_loop:
#if defined(BUILDING_RRU_UL)
	iiorxbytes = iio_buffer_refill(iiorxcbuf[0]);
	if (iiorxbytes < 0) {
		fprintf(stderr, "iio_buffer_refill() %s\r\n", strerror(-(int)iiorxbytes));
		return (void *)-1;
	}
	piiorxbuf = iio_buffer_first(iiorxcbuf[0], rxpi[0]);
	if (mode_verbose > 1)
		printf("iio_buffer_refill()=%u, iio_buffer_first()=%p\n",
			iiorxbytes, piiorxbuf);
	ASSERT(iiorxbytes == c_packet_nb*SUBCARRIER_SAMPLES*PORT_PER_DEVICE*4);
#endif
	for (ipkt=1; ipkt<=c_packet_nb; ipkt++) {
		/* for each packet, wait until the current buffer become free */
		ps = (struct tpacket_hdr *)((void *)ps_header + c_buffer_sz*ibuf);
		int wait = 1;
		do {
			switch ((volatile uint32_t)ps->tp_status) {
			case TP_STATUS_AVAILABLE:
				if (first_loop) {
					format_packet((void *)ps + data_offset);
				}
				wait = 0;
				break;
			case TP_STATUS_WRONG_FORMAT:
				printf("An error has occured during transfer\n");
				exit(EXIT_FAILURE);
				break;
			default:
				/* nothing to do => schedule : useful if no SMP */
				usleep(0);
				if (mode_repeat) {
					if (process_stop) {
						printf("\nfill-repeatly: abort from waiting tp_status\n");
						goto task_fill_exit;
					}
				}
				break;
			}
		} while (wait);

		/* update packet len */
		ps->tp_len = c_packet_sz;
		/* set header flag (trigs xmit)*/
		ps->tp_status = TP_STATUS_SEND_REQUEST;

		ibuf++;
		/* when c_packet_nb > c_buffer_nb */
		if (ibuf >= c_buffer_nb) {
			ibuf = 0;
			first_loop = 0;
		}

		if (!mode_thread) {
			if (((ipkt & c_send_mask) == 0) || (ec < 0) || (ipkt == c_packet_nb)) {
				/* send all buffers with TP_STATUS_SEND_REQUEST */
				/* Don't wait end of transfer : FUNCTION: non-blocking sendto() */
				ec = (int) task_send((void*)0);
			}
		} else {
			if (c_error) {
				if (ipkt == c_packet_nb/2) {
					if (mode_verbose > 0) printf("close() start\n");
	 
					if (c_error == 1) ec = close(fd_sock);
					if (c_error == 2) {
						ec = setsockopt(fd_sock, SOL_PACKET, PACKET_TX_RING,
								(char *)&s_packet_req, sizeof(s_packet_req));
						if (ec < 0) {
							perror("setsockopt: PACKET_TX_RING");
							return (void *)EXIT_FAILURE;
						}
					}

					if (mode_verbose > 0) printf("close end (ec:%d)\n", ec);
					break;
				}
			}
		}
	}

	if (mode_repeat) {
		mainloop_nb++;
		printf("\rfill-repeatly: complete loop %lu", mainloop_nb); fflush(stdout);
		if (!process_stop) goto task_fill_loop;
	}
task_fill_exit:	
	/* send_THREAD will be terminated later by shutdown_flag */
	printf("endof fill() thread\n");
	return (void *)0;
}

/* After TX done, check errors of circular buffers */
void check_txring(void)
{
	unsigned int i_nb_error, i;
	struct tpacket_hdr *ps;

	i_nb_error = 0;
	for (i=0; i<c_buffer_nb; i++) {
		ps = (struct tpacket_hdr *)((void *)ps_header + (c_buffer_sz*i));
		switch ((volatile uint32_t)ps->tp_status) {
		case TP_STATUS_SEND_REQUEST:
			printf("A frame has not been sent %p\n", ps_header);
			i_nb_error++;
			break;
		case TP_STATUS_LOSING:
			printf("An error has occured during transfer\n");
			i_nb_error++;
			break;
		default:
			break;
		}
	}
	printf("END (number of error:%u)\n", i_nb_error);
}

int main(int argc, char ** argv)
{
	int ec;
	//struct pollfd s_pfd;
	pthread_t t_send, t_fill;
	pthread_attr_t t_attr_send, t_attr_fill;
	struct sched_param para_send, para_fill;
	struct timespec tm_xs, tm_xe;
	double tm_us;

	get_args(argc, argv);

#if defined(BUILDING_RRU_UL)
	libiio_app_startup( c_packet_nb * SUBCARRIER_SAMPLES );
#endif

	printf("\nSTARTING TEST:\n");

	fd_sock = setup_socket(str_devname);
	switch (fd_sock) {
	case -1: ec = EXIT_FAILURE; goto exit0;
	case -2: ec = EXIT_FAILURE; goto exit1;
	case -3: ec = EXIT_FAILURE; goto exit2;
	default: break;
	}

	fill_peeraddr();

	prefill_txring();

	/* Listen to ctrl+c */
	signal(SIGINT, handle_sig);

	/* Set thread priorities, scheduler, ... */
	pthread_attr_init(&t_attr_fill);
	pthread_attr_setschedpolicy(&t_attr_fill, SCHED_RR);
	para_fill.sched_priority = 20;
	pthread_attr_setschedparam(&t_attr_fill, &para_fill);
	/* Start fill_thread that fills dummy data in circular buffers */
	ec = pthread_create(&t_fill, &t_attr_fill, task_fill, (void *)ps_header);
	if (ec != 0) {
		perror("pthread_create() for task_fill");
		ec = EXIT_FAILURE;
		goto exit2;
	}

	clock_gettime(CLOCK_MONOTONIC, &tm_xs);

	/* Start send_THREAD in blocking mode only on SMP mode */
	if (mode_thread) {
		pthread_attr_init(&t_attr_send);
		pthread_attr_setschedpolicy(&t_attr_send, SCHED_RR);
		para_send.sched_priority = 20;
		pthread_attr_setschedparam(&t_attr_send, &para_send);
		ec = pthread_create(&t_send, &t_attr_send, task_send, (void *)1);
		if (ec != 0) {
			perror("pthread_create() for task_send");
			ec = EXIT_FAILURE;
			goto exit3;
		}
	}

	/* Wait/blocking end of fill_thread */
	pthread_join(t_fill, NULL);
	/* terminate send_THREAD by setting shutdown_flag */
	if (mode_thread) {
		shutdown_flag = 1;
		printf("Shutdown requested (%d)\n", shutdown_flag);
		/* Wait end of send_THREAD */
		pthread_join(t_send, NULL);
	}
	/* in case any un-drawn packages after stopping send_THREAD */
	do {
		ec = (int) task_send((void*)0);
		printf("Loop until queue empty (%d)\n", ec);
	} while ((ec != 0) && (c_error == 0));

	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
	if (packets_total > 0) {
		tm_us = elapse_us(&tm_xe, &tm_xs);
		printf("#### TX %lu in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\n",
			packets_total, tm_us,
			packets_total/tm_us*c_packet_sz*8, tm_us/packets_total);
	}

	check_txring();
	/* TODO: display header of all blocks */

	ec = EXIT_SUCCESS;
	/* later than pthread_join(), no matter mode_thread true or not */
	goto exit2;

exit4:
	pthread_exit(&t_send);
exit3:
	pthread_exit(&t_fill);
exit2:
	munmap((void *)ps_header, c_buffer_sz * c_buffer_nb);
exit1:
	close(fd_sock);
exit0:
#if defined(BUILDING_RRU_UL)
	libiio_app_shutdown();
#endif

	return ec;
}

