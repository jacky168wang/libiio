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

--------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ How to use mmap() directly to improve capture process
++ Capture process
--------------------------------------------------------------------------------
[setup]     socket() -------> creation of the capture socket
            setsockopt() ---> allocation of the circular buffer (ring)
                              option: PACKET_RX_RING
            mmap() ---------> mapping of the allocated buffer to the
                              user process

[capture]   poll() ---------> to wait for incoming packets
  At the beginning of each frame there is an status field (see 
  struct tpacket_hdr). If this field is 0 means that the frame is ready
  to be used for the kernel, If not, there is a frame the user can read 
  and the following flags (linux/if_packet.h) apply: 
  #define TP_STATUS_COPY			(1 << 1)
    This flag indicates that the frame (and associated meta information) has been
    truncated because it's larger than tp_frame_size. This packet can be read 
    entirely with recvfrom().
    In order to make this work it must to be enabled previously with setsockopt()
    and the PACKET_COPY_THRESH option. 
    The number of frames that can be buffered to be read with recvfrom is limited
    like a normal socket. See the SO_RCVBUF option after "man socket".
  #define TP_STATUS_LOSING		(1 << 2)
    indicates there were packet drops from last time statistics where checked with
    getsockopt() and the PACKET_STATISTICS option.
  #define TP_STATUS_CSUMNOTREADY	(1 << 3)
    currently it's used for outgoing IP packets which its checksum will be done in 
    hardware. So while reading the packet we should not try to check the checksum.
  #define TP_STATUS_CSUM_VALID	(1 << 7)
    This flag indicates that at least the transport header checksum of the packet
    has been already validated on the kernel side. If the flag is not set then we 
    are free to check the checksum by ourselves provided that
    TP_STATUS_CSUMNOTREADY is also not set.
  The kernel initializes all frames to TP_STATUS_KERNEL, when the kernel
  receives a packet it puts in the buffer and updates the status with
  at least the TP_STATUS_USER flag. Then the user can read the packet,
  once the packet is read the user must zero the status field, so the kernel 
  can use again that frame buffer.

[shutdown]  close() --------> destruction of the capture socket and
                              deallocation of all associated resources.
Summary example-codes:
    int fd_rx = socket(PF_PACKET, mode, htons(ETH_P_ALL));
    setsockopt(fd_rx, SOL_PACKET, PACKET_RX_RING, &foo, sizeof(foo));
    mmap(0, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd_rx, 0);
    ...
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.revents = 0;
    pfd.events = POLLIN|POLLRDNORM|POLLERR;
    do {
      if (status == TP_STATUS_KERNEL)
    	  ec = poll(&pfd, 1, timeout);
    } while (1);

-------------------------------------------------------------------------------
man package
-------------------------------------------------------------------------------
  PACKET_RX_RING
	Create a memory-mapped ring buffer for asynchronous packet
	reception.	The packet socket reserves a contiguous region of
	application address space, lays it out into an array of packet
	slots and copies packets (up to tp_snaplen) into subsequent
	slots.	Each packet is preceded by a metadata structure simi©\
	lar to tpacket_auxdata.  The protocol fields encode the offset
	to the data from the start of the metadata header.	tp_net
	stores the offset to the network layer.  If the packet socket
	is of type SOCK_DGRAM, then tp_mac is the same.  If it is of
	type SOCK_RAW, then that field stores the offset to the link-
	layer frame.  Packet socket and application communicate the
	head and tail of the ring through the tp_status field.	The
	packet socket owns all slots with tp_status equal to TP_STA©\
	TUS_KERNEL.  After filling a slot, it changes the status of
	the slot to transfer ownership to the application.	During
	normal operation, the new tp_status value has at least the
	TP_STATUS_USER bit set to signal that a received packet has
	been stored.  When the application has finished processing a
	packet, it transfers ownership of the slot back to the socket
	by setting tp_status equal to TP_STATUS_KERNEL.
	Packet sockets implement multiple variants of the packet ring.
	The implementation details are described in Documentation/net-
	working/packet_mmap.txt in the Linux kernel source tree.

  PACKET_VERSION (with PACKET_RX_RING; since Linux 2.6.27)
	By default, PACKET_RX_RING creates a packet receive ring of
	variant TPACKET_V1.  To create another variant, configure the
	desired variant by setting this integer option before creating
	the ring.

  PACKET_STATISTICS
	Retrieve packet socket statistics in the form of a structure
	  struct tpacket_stats {
	      unsigned int tp_packets;  // Total packet count
	      unsigned int tp_drops;    // Dropped packet count
	  };
	Receiving statistics resets the internal counters.  The sta©\
	tistics structure differs when using a ring of variant TPACKET_V3.

  PACKET_TIMESTAMP (with PACKET_RX_RING; since Linux 2.6.36)
	The packet receive ring always stores a timestamp in the meta©\
	data header.  By default, this is a software generated time©\
	stamp generated when the packet is copied into the ring.  This
	integer option selects the type of timestamp.  Besides the
	default, it support the two hardware formats described in Doc©\
	umentation/networking/timestamping.txt in the Linux kernel
	source tree.

-------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ AF_PACKET TPACKET_V3 example
-------------------------------------------------------------------------------
  AF_PACKET's TPACKET_V3 ring buffer can be configured to use non-static frame
  sizes by doing it's own memory management. It is based on blocks where polling
  works on a per block basis instead of per ring as in TPACKET_V2 and predecessor.
  It is said that TPACKET_V3 brings the following benefits:
    *) ~15 - 20% reduction in CPU-usage
    *) ~20% increase in packet capture rate
    *) ~2x increase in packet density
    *) Port aggregation analysis
    *) Non static frame size to capture entire packet payload
  So it seems to be a good candidate to be used with packet fanout.

  Minimal example code
    Copyright 2011, Chetan Loke <loke.chetan@gmail.com>
    kernel-to-user space API usage dissected from Chetan Loke's lolpcap
    License: GPL, version 2.0
    Compile: gcc -Wall -O2 blob.c
    Usage: ./a.out eth0
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
#if defined(BUILDING_RRU_DL)
#include <iio.h>
# if (1 != DEV_PORT_NB)
#error "memcpy(samples) should be changed into for-each-sample for DEV_PORT_NB>1"
# endif
#endif

#ifndef likely
# define likely(x)		__builtin_expect(!!(x), 1)
#endif
#ifndef unlikely
# define unlikely(x)	__builtin_expect(!!(x), 0)
#endif

struct block_desc {
	uint32_t version;
	uint32_t offset_to_priv;
	struct tpacket_hdr_v1 h1;
};

struct tpring {
	struct iovec *rd;
	uint8_t *map;
	struct tpacket_req3 req;
};

/* params */
#if !defined(BUILDING_RRU_DL)
static char * str_devname = NULL;
static unsigned int n_rx_pkt_sz = 1500;
static unsigned int n_rx_pkt_nb	= 1000;
#else
extern char *str_devname;
static unsigned int n_rx_pkt_sz = RAWSRX_BUF_SZ;
       unsigned int n_rx_pkt_nb = RAWSR_BUF_NB_NOW;
#endif
/* MTU >= n_rx_pkt_sz */
static unsigned int n_eth_mtu = 0;
/* tp_frame_size >= MTU + 32 */
/* PDSCH==4890 -> MTU>=4890 -> frame_size=8K */
static unsigned int n_eframe_sz = 1 << 13; /* 8K */
/* tp_frame_nr = tp_block_size/tp_frame_size*tp_block_nr; */

/* frame_size=8K and n_rx_pkt_nb==873
   it's better if one block is able to hold >=873 frames
   -> n_rx_blk_sz = 8K*1024=8M */
/* tp_block_size <= getpagesize() << MAX_ORDER;
   test-on-board: 4K << 11, i.e. 1<<23 will fail */
static unsigned int n_rx_blk_sz = 1 << 22; /* 4M */
/* tp_block_nr <= "kmalloc-size-max" per "/proc/slabinfo" */
static unsigned int n_rx_blk_nb = 64;

static unsigned int m_rx_repeat = 1;
static unsigned int n_rx_verbos = 0;

#if !defined(BUILDING_RRU_DL)
static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION] [INTERFACE]\n"
		" -h\tshow this help\n"
		" -m\tset mtu\n"
		" -f\tset frame  size\n"
		" -b\tset block  size\n"
		" -n\tset block  count\n"
		" -s\tset wanted IP packet size\n"
		" -c\tset wanted IP packet count\n"
		" -t\tuse dual thread\n"
		" -r\trepeating rather than one-short\n"
		" -v\tbe verbose\n"
		);
}

static void get_args(int argc, char **argv)
{
	int c;
	opterr = 0;
	while ( (c = getopt( argc, argv, "m:f:b:n:s:c:v:rh")) != EOF ) {
		switch (c) {
		case 'm': n_eth_mtu   = strtoul( optarg, NULL, 0 ); break;
		case 'f': n_eframe_sz = strtoul( optarg, NULL, 0 ); break;
		case 'b': n_rx_blk_sz = strtoul( optarg, NULL, 0 ); break;
		case 'n': n_rx_blk_nb = strtoul( optarg, NULL, 0 ); break;
		case 's': n_rx_pkt_sz = strtoul( optarg, NULL, 0 ); break;
		case 'c': n_rx_pkt_nb = strtoul( optarg, NULL, 0 ); break;
		case 'v': n_rx_verbos = strtoul( optarg, NULL, 0 );	break;
		case 'r': m_rx_repeat = 1;                          break;
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
	if ( optind < argc )
		str_devname = argv[ optind ];
	if ( NULL == str_devname ) {
		fprintf( stderr, "ERROR: No interface was specified\n");
		usage();
		exit( EXIT_FAILURE );
	}
}
#endif

static void print_args(void)
{
	printf( "CURRENT SETTINGS:\n" );
	printf( "str_devname:       %s\n", str_devname );
	printf( "n_eth_mtu(-m):     %d\n", n_eth_mtu );
	printf( "n_eframe_sz(-f):   %u\n", n_eframe_sz );
	printf( "n_rx_blk_sz(-b):   %u\n", n_rx_blk_sz );
	printf( "n_rx_blk_nb(-n):   %u\n", n_rx_blk_nb );
	printf( "n_rx_pkt_sz(-s):   %u\n", n_rx_pkt_sz );
	printf( "n_rx_pkt_nb(-c):   %u\n", n_rx_pkt_nb );
	printf( "m_rx_repeat(-r):   %d\n", m_rx_repeat );
	printf( "n_rx_verbos(-v):   %d\n\n", n_rx_verbos );
}

static int verify_args(void)
{
	int pgsz = getpagesize();
	printf("current arch: page_size=%d, MAX_ORDER=%d\n", pgsz, MAX_ORDER);
	if (n_rx_blk_sz > getpagesize() << MAX_ORDER) {
		printf("tp_block_size must be a multiple of PAGE_SIZE\n");
		return -1;
	}
	if (n_rx_blk_sz & (pgsz - 1) != 0) {
		printf("tp_block_size must be a multiple of PAGE_SIZE\n");
		return -1;
	}

	printf("current arch+platform: /proc/slabinfo: <kmalloc-min>*<min>\n");
	if (n_rx_blk_nb & (sizeof(void *) - 1) != 0) {
		printf("tp_block_nr must be a multiple of sizeof(void *)\n");
		return -1;
	}

	if (n_rx_blk_sz % n_eframe_sz != 0) {
		printf("tp_frame_size must be a divider of tp_block_size\n");
		return -1;
	}
	if (n_eframe_sz & (TPACKET_ALIGNMENT-1) != 0) {
		printf("tp_frame_size must be a multiple of TPACKET_ALIGNMENT\n");
		return -1;
	}

#if defined(BUILDING_RRU_DL)
    printf("RAWSR_BUF_NB_MAX = %d, RADIO_DL_SYM_NB = %d", RAWSR_BUF_NB_MAX, RADIO_DL_SYM_NB);
	if (n_rx_pkt_nb > RAWSR_BUF_NB_MAX) {
		printf("assert failed (packet_nb <= RAWSR_BUF_NB_MAX)\n");
		return -1;
	}
#endif

	return 0;
}

/* globals */
static struct timespec tm_xs_tot;
static unsigned long ethr_pkts_nb = 0;
static unsigned long ethr_byts_nb = 0;
static struct timespec tm_rs_spe;
static unsigned long ethr_spec_nb;
#if defined(BUILDING_RRU_DL)
static unsigned long ethr_pdsch_nb;
static unsigned long ethr_bscsr_nb;
static ssize_t iiot_byts_nb;
static char   *iiot_byts_pt;
#endif

static void handle_sig(int sig)
{
	taskstop_rawsr = 1;
}

static int setup_rxsocket(struct tpring *ring)
{
	int ec, fd, opt;
	socklen_t optlen;
	struct ifreq s_ifr;
	struct sockaddr_ll llme;
#if 0
	short protocol = ETH_P_ALL;
#else
	short protocol = ETH_P_IP;
#endif

	fd = socket(AF_PACKET, SOCK_RAW, htons(protocol));
	if (fd < 0) {
		perror("socket");
		return (-1);
	}

	/* update the interface mtu */
	strncpy(s_ifr.ifr_name, str_devname, sizeof(s_ifr.ifr_name));
	ec = ioctl(fd, SIOCGIFMTU, &s_ifr);
	if (ec == -1) {
		perror("iotcl SIOCGIFMTU");
		return (-2);
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
			return (-2);
		}
		/* s_ifr.ifr_flags |= IFF_UP;
		ec = ioctl(fd, SIOCSIFFLAGS, &s_ifr); */
	}
	if (n_eframe_sz < n_eth_mtu + TPACKET_HDRLEN - sizeof(struct sockaddr_ll)) {
		printf("assert failed (frame_sz >= mtu + data_offset)\n");
		return (-2);
	}
	if (n_rx_pkt_sz > n_eth_mtu) {
		printf("assert failed (packet_sz <= mtu)\n");
		return (-2);
	}

	memset(&llme, 0, sizeof(llme));
	llme.sll_family = AF_PACKET;
	llme.sll_protocol = htons(protocol);
	llme.sll_ifindex = if_nametoindex(str_devname);
	ec = bind(fd, (struct sockaddr *)&llme, sizeof(llme));
	if (ec < 0) {
		perror("bind");
		return (-2);
	}

	opt = TPACKET_V3;
	ec = setsockopt(fd, SOL_PACKET, PACKET_VERSION, &opt, sizeof(opt));
	if (ec < 0) {
		perror("setsockopt TPACKET_V3");
		return (-2);
	}

	memset(&ring->req, 0, sizeof(ring->req));
	ring->req.tp_block_size = n_rx_blk_sz;
	ring->req.tp_frame_size = n_eframe_sz;
	ring->req.tp_block_nr = n_rx_blk_nb;
	ring->req.tp_frame_nr = (n_rx_blk_sz * n_rx_blk_nb) / n_eframe_sz;
	ring->req.tp_retire_blk_tov = 60;
	ring->req.tp_feature_req_word = TP_FT_REQ_FILL_RXHASH;

	ec = setsockopt(fd, SOL_PACKET, PACKET_RX_RING, &ring->req, sizeof(ring->req));
	if (ec < 0) {
		perror("setsockopt PACKET_RX_RING");
		return (-2);
	}

	ring->map = mmap(NULL, ring->req.tp_block_size * ring->req.tp_block_nr,
		  PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
	if (ring->map == MAP_FAILED) {
		perror("mmap");
		return (-3);
	}

	ring->rd = malloc(ring->req.tp_block_nr * sizeof(*ring->rd));
	assert(ring->rd);
	if (NULL == ring->rd) {
		perror("malloc");
		return (-4);
	}
	for (int i = 0; i < ring->req.tp_block_nr; ++i) {
		ring->rd[i].iov_base = ring->map + (i * ring->req.tp_block_size);
		ring->rd[i].iov_len = ring->req.tp_block_size;
	}

	return fd;
}

static void display_packet(uint8_t *pdu)
{
	struct ethhdr *eth = (struct ethhdr *)pdu;
	struct iphdr *ip = (struct iphdr *)(pdu + ETH_HLEN);

	macaddr_print((uint8_t *)eth + 6);
	printf(" => ");
	macaddr_print((uint8_t *)eth);
	printf(", ");

	if (eth->h_proto == htons(ETH_P_IP)) {
		struct sockaddr_in ss, sd;
		char sbuff[NI_MAXHOST], dbuff[NI_MAXHOST];

		memset(&ss, 0, sizeof(ss));
		ss.sin_family = PF_INET;
		ss.sin_addr.s_addr = ip->saddr;
		getnameinfo((struct sockaddr *)&ss, sizeof(ss),
			 sbuff, sizeof(sbuff), NULL, 0, NI_NUMERICHOST);

		memset(&sd, 0, sizeof(sd));
		sd.sin_family = PF_INET;
		sd.sin_addr.s_addr = ip->daddr;
		getnameinfo((struct sockaddr *)&sd, sizeof(sd),
			 dbuff, sizeof(dbuff), NULL, 0, NI_NUMERICHOST);

		printf("%s -> %s\n", sbuff, dbuff);
	}
}

#if defined(BUILDING_RRU_DL)
static inline int walk_pdsch(uint8_t *pdu)
{
	if (n_rx_verbos > 0) {
		unpack_ethhdr(pdu);
		unpack_iphdr_udphdr(pdu + ETH_HDR_LEN);
	}

	/* PDSCH	header : sizeof(struct pusch_hdr) */
	unpack_pdsch(pdu + PKG_OFFSET);
	/* PDSCH   payload : PDSCH_SIZE */
    size_t nb_src = RADIO_SYM2SMP*sizeof(struct bbu_sample);
	memcpy(iiot_byts_pt, pdu + PDSCH_PLDOF, nb_src);
#if defined(FPGA_COMPRESS_SAMPLES)
    /* FPGA compress samples from 16bit to 8bit */
	iiot_byts_pt += nb_src;
	iiot_byts_nb -= nb_src;
#else
    /* the following 1 line are NOT for function BUT for throughput-test
       real function codes here should expend each sample from 8bit to 16bit */
	//memcpy(iiot_byts_pt + nb_src, pdu + PDSCH_PLDOF, nb_src);
	iiot_byts_pt += nb_src*2;
	iiot_byts_nb -= nb_src*2;
#endif
	if (0 == ethr_spec_nb % n_rx_pkt_nb) {
		iiot_byts_nb = iio_buffer_push(iiopt_cbuf[0]);
		if (iiot_byts_nb < 0) {
			fprintf(stderr,
                "task_rawsr: rawsr2iiopt[%lu]: iio_buffer_refill() %s\n",
                ethr_spec_nb/n_rx_pkt_nb, strerror(-(int)iiot_byts_nb));
			return -1;
		}
# if 0
		if (iiot_byts_nb != IIOPT_SMP_NB_NOW*sizeof(struct bbu_sample)) {
			fprintf(stderr, "assert-fail: rawsr2iiopt[%lu]: "\
                "iio_buffer_push() %u != RAWSR_BUF_NB_NOW*2400\n",
                ethr_spec_nb/n_rx_pkt_nb, iiot_byts_nb);
			return -2;
		}
# endif
		iiot_byts_pt = iio_buffer_first(iiopt_cbuf[0], iiopt_chni[0]);
		if (n_rx_verbos > 0) {
			printf("task_rawsr: rawsr2iiopt[%lu]: "\
                "iio_buffer_push()=%u, iio_buffer_first()=%p\n", 
                ethr_spec_nb/n_rx_pkt_nb, iiot_byts_nb, iiot_byts_pt);
		}
	}
	return 0;
}
#endif

static int walk_rxblock(struct block_desc *pbd)
{
	int iframe;
	struct tpacket3_hdr *ppd;
	unsigned int bytes = 0;

	ppd = (struct tpacket3_hdr *)((uint8_t *)pbd + pbd->h1.offset_to_first_pkt);
	for (iframe = 0; iframe < pbd->h1.num_pkts; iframe++) {
		bytes += ppd->tp_snaplen;
		if (n_rx_verbos > 1) {
			printf("\tRxPkt[%d]: len=%u, hash=0x%x: ",
				iframe, ppd->tp_snaplen, ppd->hv1.tp_rxhash);
			display_packet((uint8_t *)ppd + ppd->tp_mac);
			fflush(stdout);
		}
		if (n_rx_pkt_sz == ppd->tp_snaplen) {
			ethr_spec_nb++;
			if (1 == ethr_spec_nb)
				clock_gettime(CLOCK_MONOTONIC, &tm_rs_spe);
		}

#if defined(BUILDING_RRU_DL)
		/* in case n_rx_pkt_sz user input other value */
		if (PDSCH_UBUF_LEN == ppd->tp_snaplen) {
			if (0 != walk_pdsch((uint8_t *)ppd + ppd->tp_mac))
				return -1;
		} else if (BSCSR_UBUF_LEN == ppd->tp_snaplen)
			ethr_bscsr_nb++;
#endif
		ppd = (struct tpacket3_hdr *)((uint8_t *)ppd + ppd->tp_next_offset);
	}
	ethr_pkts_nb += pbd->h1.num_pkts;
	ethr_byts_nb += bytes;

	return 0;
}

static void flush_rxblock(struct block_desc *pbd)
{
	pbd->h1.block_status = TP_STATUS_KERNEL;
}

static void close_rxsocket(struct tpring *ring, int fd)
{
	munmap((void *)ring->map, ring->req.tp_block_size * ring->req.tp_block_nr);
	free(ring->rd);
	close(fd);
}

//int raws_rx_main(void)
void *main_rx(void *parg)
{
	int fd_rxsock, ec;
	struct tpring rx_ring;
	struct pollfd pfd;
	unsigned int iblk;
	struct block_desc *pbd;
	int events_nb;

	memset(&rx_ring, 0, sizeof(rx_ring));
	fd_rxsock = setup_rxsocket(&rx_ring);
	assert(fd_rxsock > 0);
	switch (fd_rxsock) {
	case -1: ec = EXIT_FAILURE; goto exit0;
	case -2: ec = EXIT_FAILURE; goto exit1;
	case -3: ec = EXIT_FAILURE; goto exit2;
	case -4: ec = EXIT_FAILURE; goto exit3;
	default: break;
	}

	printf("task_rawsr: start\n"); fflush(stdout);

	memset(&pfd, 0, sizeof(pfd));
	pfd.fd = fd_rxsock;
	pfd.events = POLLIN | POLLERR;
	pfd.revents = 0;

#if defined(BUILDING_RRU_DL)
	iiot_byts_nb = iio_buffer_push(iiopt_cbuf[0]);
	if (iiot_byts_nb < 0) {
		fprintf(stderr, "task_rawsr: iio_buffer_refill() %s\n",
						strerror(-(int)iiot_byts_nb));
		ec = 1;
		goto exit0;
	}
	iiot_byts_pt = iio_buffer_first(iiopt_cbuf[0], iiopt_chni[0]);
#endif

	iblk = 0;
	events_nb = 0;
	while (likely(!taskstop_rawsr)) {
		pbd = (struct block_desc *) rx_ring.rd[iblk].iov_base;
		/* if current owner of this block is kernel */
		if ((pbd->h1.block_status & TP_STATUS_USER) == 0) {
			if (n_rx_verbos > 1)
				printf("task_rawsr: block[%d]: poll(blocking) until available\n",
				    iblk);

			poll(&pfd, 1, -1);/* negative value means infinite */

			events_nb++;
			if (n_rx_verbos > 1)
				printf("task_rawsr: block[%d]: poll(blocking) %d times\n",
				    iblk, events_nb);

			continue;
		}
		events_nb = 0;
		if (0 == ethr_pkts_nb)
			clock_gettime(CLOCK_MONOTONIC, &tm_xs_tot);

		/* wait/kill all POLLIN events until kernel_rx on that block is done */
		ec = walk_rxblock(pbd);
		flush_rxblock(pbd);
#if defined(BUILDING_RRU_DL)
		if (ec < 0)
            break; /* terminate ahead */
#endif
		if ((0 == m_rx_repeat) && (ethr_spec_nb >= n_rx_pkt_nb))
			break;	/* terminate ahead */

		if (n_rx_verbos > 0) printf("task_rawsr: block[%d] end\n", iblk);
		iblk = (iblk + 1) % n_rx_blk_nb;
	}

	if (ethr_pkts_nb > 0)
		clock_gettime(CLOCK_MONOTONIC, &tm_re);

    printf("\n");
	struct tpacket_stats_v3 stats;
	socklen_t len = sizeof(stats);
	ec = getsockopt(fd_rxsock, SOL_PACKET, PACKET_STATISTICS, &stats, &len);
	if (ec < 0) {
		perror("getsockopt PACKET_STATISTICS");
		goto exit4;
	}
	if (stats.tp_packets > 0) {
		printf("RX PACKET_STATISTICS: %u pkts, %u dropped, freeze_q_cnt %u\n",
			stats.tp_packets, stats.tp_drops, stats.tp_freeze_q_cnt);
	}

exit4:
	if (ethr_pkts_nb > 0) {
        double tm_us = elapse_us(&tm_re, &tm_xs_tot);
		printf("RX total: %lu/%lu pkts/bytes in %.0f us (%7.3f Mbps, %7.3f us)\n",
			ethr_pkts_nb, ethr_byts_nb, tm_us,
			tm_us ? ethr_byts_nb/tm_us*8 : 0,
			tm_us/ethr_pkts_nb);
	}
	if (ethr_spec_nb > 0) {
		double tm_us = elapse_us(&tm_re, &tm_rs_spe);
		printf("RX specific: %lu pkts in %.0f us (%7.3f Mbps, %7.3f us)\n",
			ethr_spec_nb, tm_us,
			tm_us ? ethr_spec_nb*n_rx_pkt_sz/tm_us*8 : 0,
			tm_us/ethr_spec_nb);
	}
#if defined(BUILDING_RRU_DL)
    display_rx_perf(&tm_re, &tm_rs);
#endif

exit3:
	free(rx_ring.rd);
exit2:
	munmap((void *)rx_ring.map, rx_ring.req.tp_block_size * rx_ring.req.tp_block_nr);
exit1:
	close(fd_rxsock);
exit0:
	printf("task_rawsr: end\n");
	return (void *)ec;
}

#if 0
#if defined(BUILDING_RRU_DL)
extern int raws_tmpps_startup(void);
# if defined(TMPPS_TRG_UGPIOIRQ)
extern void *raws_tmpps_main(void *parg);
# endif
#endif
int main(int argc, char **argv)
{
    int ec;
#if !defined(BUILDING_RRU_DL)
	get_args(argc, argv);
#else
    common_getargs(argc, argv);
#endif
    print_args();
    ec = verify_args();
    if (ec < 0) return ec;

#if defined(BUILDING_RRU_DL)
    iio_path_x = 0x03;
#if defined(FPGA_COMPRESS_SAMPLES)
    libiio_app_startup(IIOPR_SMP_NB_NOW/2, n_rx_pkt_nb*RADIO_SYM2SMP/2);
#else
    libiio_app_startup(IIOPR_SMP_NB_NOW, n_rx_pkt_nb*RADIO_SYM2SMP);
#endif

# if defined(TMPPS_TRG_UTIMER)
    ec = raws_tmpps_startup(str_devname);
    if (ec < 0) return -1;
    tmpps_tmr_init();
# endif
# if defined(TMPPS_TRG_UGPIOIRQ)
    pthread_t pid_tmpps;
    ec = pthread_create(&pid_tmpps, NULL, raws_tmpps_main, NULL);
    if (ec != 0) {
        perror("pthread_create() for raws_rx_thread\n");
        return -1;
    }
# endif
# if defined(TMPPS_TRG_KSIGIO_NOWAIT)
    signal(PPS_CAPTUREASSERT, tmpps_handler);/* Listen to PPS_ASSERT */
# endif
#endif

    /* Listen to ctrl+c */
    signal(SIGINT, handle_sig);

    ec = (int)main_rx(NULL);

#if defined(BUILDING_RRU_DL)
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_exit();
    raws_tmpps_close();
# endif
# if defined(TMPPS_TRG_UTIMER)
    pthread_exit(&pid_tmpps);
# endif

	libiio_app_shutdown();
#endif

    return ec;
}
#endif
