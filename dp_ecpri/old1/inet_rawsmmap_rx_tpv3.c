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
char * str_devname = NULL;
#if defined(BUILDING_RRU_DL)
static unsigned int c_packet_sz = SUBCARRIER_NRXBUF_SZ; //4862, 4890
static unsigned int c_packet_nb = IIO_PORT_SAMPLES/SUBCARRIER_SAMPLES; //873
#else
static unsigned int c_packet_sz = 1500;
static unsigned int c_packet_nb	= 1000;
#endif
// tp_block_size <= getpagesize() << MAX_ORDER
// tp_block_size <= 4K << 11
static unsigned int c_block_sz = 1 << 22; /* == 4K << 10 */
// tp_block_nr <= "kmalloc-size-max" per "/proc/slabinfo"
// ARMv7+Linux4.9.0: 
static unsigned int c_block_nb = 64;
// tp_frame_size >= MTU + 32; MTU >= c_packet_sz
static unsigned int c_frame_sz = 1 << 13; /* c_frame_sz  */
// tp_frame_nr = tp_block_size/tp_frame_size*tp_block_nr;

static unsigned int c_mtu = 0;
static unsigned int mode_thread = 0;
static unsigned int mode_verbose = 0;

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION] [INTERFACE]\n"
		" -h\tshow this help\n"
		" -m\tset mtu\n"
		" -f\tset frame  size\n"
		" -b\tset block  size\n"
		" -n\tset block  count\n"
		" -s\tset packet size\n"
		" -c\tset packet count\n"
		" -t\tuse dual thread\n"
		" -v\tbe verbose\n"
		);
}

void get_args(int argc, char **argv)
{
	int c;
	opterr = 0;
	while ( (c = getopt( argc, argv, "m:f:b:n:s:c:tvh")) != EOF ) {
		switch (c) {
		case 'm': c_mtu       = strtoul( optarg, NULL, 0 ); break;
		case 'f': c_frame_sz  = strtoul( optarg, NULL, 0 ); break;
		case 'b': c_block_sz  = strtoul( optarg, NULL, 0 ); break;
		case 'n': c_block_nb  = strtoul( optarg, NULL, 0 ); break;
		case 's': c_packet_sz = strtoul( optarg, NULL, 0 ); break;
		case 'c': c_packet_nb = strtoul( optarg, NULL, 0 ); break;
		case 't': mode_thread = 1;                          break;
		case 'v': mode_verbose= strtoul( optarg, NULL, 0 );	break;
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
	printf( "c_frame_sz(-f):    %u\n", c_frame_sz );
	printf( "c_block_sz(-b):    %u\n", c_block_sz );
	printf( "c_block_nb(-n):    %u\n", c_block_nb );
	printf( "c_packet_sz(-s):   %u\n", c_packet_sz );
	printf( "c_packet_nb(-c):   %u\n", c_packet_nb );
	printf( "mode_thread(-t):   %d\n", mode_thread );
	printf( "mode_verbose(-v):  %d\n\n", mode_verbose );
}

int verify_args(void)
{
	int pgsz = getpagesize();
	printf("current arch: page_size=%d, MAX_ORDER=%d\n", pgsz, MAX_ORDER);
	if (c_block_sz > getpagesize() << MAX_ORDER) {
		printf("tp_block_size must be a multiple of PAGE_SIZE\n");
		return -1;
	}
	if (c_block_sz & (pgsz - 1) != 0) {
		printf("tp_block_size must be a multiple of PAGE_SIZE\n");
		return -1;
	}

	printf("current arch+platform: /proc/slabinfo: <kmalloc-min>*<min>\n");
	if (c_block_nb & (sizeof(void *) - 1) != 0) {
		printf("tp_block_nr must be a multiple of sizeof(void *)\n");
		return -1;
	}

	if (c_block_sz % c_frame_sz != 0) {
		printf("tp_frame_size must be a divider of tp_block_size\n");
		return -1;
	}
	if (c_frame_sz & (TPACKET_ALIGNMENT-1) != 0) {
		printf("tp_frame_size must be a multiple of TPACKET_ALIGNMENT\n");
		return -1;
	}

	if (c_frame_sz < c_mtu + TPACKET_HDRLEN - sizeof(struct sockaddr_ll)) {
		printf("assert failed (frame_sz >= mtu + data_offset)\n");
		return -1;
	}

	if (c_packet_sz > c_mtu) {
		printf("assert failed (packet_sz <= mtu)\n");
		return -1;
	}

#if defined(BUILDING_RRU_UL)
	if (1 != PORT_PER_DEVICE) {
		printf("assert failed (1 == PORT_PER_DEVICE)\n");
		return -1;
	}
	if (c_packet_nb > SUBCARRIER_NRXBUF_NB) {
		printf("assert failed (packet_nb <= SUBCARRIER_NRXBUF_NB)\n");
		return -1;
	}
#endif

	return 0;
}

/* globals */
static unsigned long packets_total = 0;
static unsigned long bytes_total = 0;

static sig_atomic_t sigint = 0;
static void handle_sig(int sig)
{
	sigint = 1;
}

static int setup_socket(struct tpring *ring, char *netdev)
{
	int ec, fd, opt;
	socklen_t optlen;
	struct ifreq s_ifr;
	struct sockaddr_ll llme;

	fd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
	if (fd < 0) {
		perror("socket");
		exit(-1);
	}

	/* update the interface mtu */
	strncpy(s_ifr.ifr_name, str_devname, sizeof(s_ifr.ifr_name));
	ec = ioctl(fd, SIOCGIFMTU, &s_ifr);
	if (ec == -1) {
		perror("iotcl SIOCGIFMTU");
		exit(-2);
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
			exit(-2);
		}
		/* s_ifr.ifr_flags |= IFF_UP;
		ec = ioctl(fd, SIOCSIFFLAGS, &s_ifr); */
	}

	ec = verify_args();
	if (ec < 0) exit(-2);

	memset(&llme, 0, sizeof(llme));
	llme.sll_family = AF_PACKET;
	llme.sll_protocol = htons(ETH_P_ALL);
	llme.sll_ifindex = if_nametoindex(netdev);
	ec = bind(fd, (struct sockaddr *)&llme, sizeof(llme));
	if (ec < 0) {
		perror("bind");
		exit(-2);
	}

	opt = TPACKET_V3;
	ec = setsockopt(fd, SOL_PACKET, PACKET_VERSION, &opt, sizeof(opt));
	if (ec < 0) {
		perror("setsockopt TPACKET_V3");
		exit(-2);
	}

	memset(&ring->req, 0, sizeof(ring->req));
	ring->req.tp_block_size = c_block_sz;
	ring->req.tp_frame_size = c_frame_sz;
	ring->req.tp_block_nr = c_block_nb;
	ring->req.tp_frame_nr = (c_block_sz * c_block_nb) / c_frame_sz;
	ring->req.tp_retire_blk_tov = 60;
	ring->req.tp_feature_req_word = TP_FT_REQ_FILL_RXHASH;

	ec = setsockopt(fd, SOL_PACKET, PACKET_RX_RING, &ring->req, sizeof(ring->req));
	if (ec < 0) {
		perror("setsockopt PACKET_RX_RING");
		exit(-2);
	}

	ring->map = mmap(NULL, ring->req.tp_block_size * ring->req.tp_block_nr,
		  PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
	if (ring->map == MAP_FAILED) {
		perror("mmap");
		exit(-3);
	}

	ring->rd = malloc(ring->req.tp_block_nr * sizeof(*ring->rd));
	assert(ring->rd);
	if (NULL == ring->rd) {
		perror("malloc");
		exit(-4);
	}
	for (int i = 0; i < ring->req.tp_block_nr; ++i) {
		ring->rd[i].iov_base = ring->map + (i * ring->req.tp_block_size);
		ring->rd[i].iov_len = ring->req.tp_block_size;
	}

	return fd;
}

#if defined(BUILDING_RRU_DL)
#include <iio.h>
extern struct iio_buffer *iiotxcbuf[CHIP_NUM_TOTAL];
extern struct iio_channel *txpi[PORT_NUM_TOTAL];
static ssize_t iiotxbytes = 0;
static char *piiotxbuf = NULL;

extern int raws_unpack_ethhdr(uint8_t *buf);
extern int unpack_iphdr_udphdr(uint8_t *buf);
#endif

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

		printf("%s -> %s, ", sbuff, dbuff);
	}
}

static void walk_block(struct block_desc *pbd)
{
	int ipkt;
	size_t bytes = 0;
	struct tpacket3_hdr *ppd;

	ppd = (struct tpacket3_hdr *)((uint8_t *)pbd + pbd->h1.offset_to_first_pkt);
	for (ipkt = 0; ipkt < pbd->h1.num_pkts; ipkt++) {
		bytes += ppd->tp_snaplen;
		if ((0 == ipkt) || (pbd->h1.num_pkts-1 == ipkt)) {
			if (mode_verbose > 1) {
				printf("\tRxPkt[%d]: len=%u, hash=0x%x: ",
					ipkt, ppd->tp_snaplen, ppd->hv1.tp_rxhash);
				display_packet((uint8_t *)ppd + ppd->tp_mac);
				fflush(stdout);
			}
		}
#if defined(BUILDING_RRU_DL)
		ASSERT(PDSCH_UBUF_LEN == ppd->tp_snaplen);
		if (mode_verbose > 0) {
			raws_unpack_ethhdr((uint8_t *)ppd + ppd->tp_mac);
			unpack_iphdr_udphdr((uint8_t *)ppd + ppd->tp_net);
		}
		/* PDSCH	header : sizeof(struct pusch_hdr) */
		unpack_pdsch((uint8_t *)ppd + ppd->tp_mac + PKG_OFFSET);
		/* PDSCH   payload : PDSCH_SIZE */
		size_t nb = SUBCARRIER_SAMPLES*2;
		memcpy(piiotxbuf, (uint8_t *)ppd + ppd->tp_mac + PUSCH_PLDOF, nb);
		piiotxbuf += nb;
		iiotxbytes -= nb;
#if 0 /* FPGA decompress samples from 8bit to 16bit */
		// do nothing
#else
		memcpy(piiotxbuf, (uint8_t *)ppd + ppd->tp_mac + PUSCH_PLDOF, nb);		
		piiotxbuf += nb;
		iiotxbytes -= nb;
#endif
		//printf("format_packet() update iiotxbytes=%u, piiotxbuf=%p\n", iiotxbytes, piiotxbuf);
#endif
		ppd = (struct tpacket3_hdr *)((uint8_t *)ppd + ppd->tp_next_offset);
	}

	packets_total += pbd->h1.num_pkts;
	bytes_total += bytes;
}

static void flush_block(struct block_desc *pbd)
{
	pbd->h1.block_status = TP_STATUS_KERNEL;
}

static void close_socket(struct tpring *ring, int fd)
{
	munmap((void *)ring->map, ring->req.tp_block_size * ring->req.tp_block_nr);
	free(ring->rd);
	close(fd);
}

int main(int argc, char **argv)
{
	int fd_sock, ec;
	socklen_t len;
	struct tpring rx_ring;
	struct pollfd pfd;
	unsigned int iblk;
	struct block_desc *pbd;
	int events_nb;
	struct timespec tm_xs, tm_xe;
	double tm_us;

	get_args(argc, argv);

#if defined(BUILDING_RRU_DL)
	libiio_app_startup( c_packet_nb * SUBCARRIER_SAMPLES );
#endif

	memset(&rx_ring, 0, sizeof(rx_ring));
	fd_sock = setup_socket(&rx_ring, str_devname);
	assert(fd_sock > 0);
	switch (fd_sock) {
	case -1: ec = EXIT_FAILURE; goto exit0;
	case -2: ec = EXIT_FAILURE; goto exit1;
	case -3: ec = EXIT_FAILURE; goto exit2;
	case -4: ec = EXIT_FAILURE; goto exit3;
	default: break;
	}

	memset(&pfd, 0, sizeof(pfd));
	pfd.fd = fd_sock;
	pfd.events = POLLIN | POLLERR;
	pfd.revents = 0;

	/* Listen to ctrl+c */
	signal(SIGINT, handle_sig);

	iblk = 0;
	events_nb = 0;
	while (likely(!sigint)) {
		if (mode_verbose > 1) printf("\nblock[%d] start\n", iblk);

		pbd = (struct block_desc *) rx_ring.rd[iblk].iov_base;
		if ((pbd->h1.block_status & TP_STATUS_USER) == 0) {
			if ((0 != iblk) && (0 == events_nb))
				clock_gettime(CLOCK_MONOTONIC, &tm_xe);

			poll(&pfd, 1, -1);/* negative value means infinite timeout */

			if ((0 == iblk) && (0 == events_nb))
				clock_gettime(CLOCK_MONOTONIC, &tm_xs);

			events_nb++;
			if (mode_verbose > 1) printf("\tblocking-poll() %d times\n", events_nb);

			continue;
		}

#if defined(BUILDING_RRU_DL)
		if (0 == packets_total % c_packet_nb) {
			iiotxbytes = iio_buffer_push(iiotxcbuf[0]);
			if (iiotxbytes < 0) {
				fprintf(stderr, "iio_buffer_refill() %s\r\n",
					strerror(-(int)iiotxbytes));
				ec = 1;
				goto exit0;
			}
			piiotxbuf = iio_buffer_first(iiotxcbuf[0], txpi[0]);
			if (mode_verbose > 1) {
				printf("iio_buffer_push()=%u, iio_buffer_first()=%p\n", 
					iiotxbytes, piiotxbuf);
			}
			ASSERT(iiotxbytes == c_packet_nb*SUBCARRIER_SAMPLES*4);
			//ASSERT(SUBCARRIER_NRXBUF_NB == pbd->h1.num_pkts);
		}
#endif

		/* wait/kill all POLLIN events until kernel_rx on that block is done */
		walk_block(pbd);

		flush_block(pbd);

		if (mode_verbose > 1) printf("block[%d] end\n", iblk);
		iblk = (iblk + 1) % c_block_nb;
		events_nb = 0;

		if (packets_total >= c_packet_nb)
			break;
	}

	tm_us = elapse_us(&tm_xe, &tm_xs);

	struct tpacket_stats_v3 stats;
	len = sizeof(stats);
	ec = getsockopt(fd_sock, SOL_PACKET, PACKET_STATISTICS, &stats, &len);
	if (ec < 0) {
		perror("getsockopt PACKET_STATISTICS");
		goto exit4;
	}
	if (stats.tp_packets > 0) {
		printf("\nRX %u packets, %u dropped, freeze_q_cnt %u\n",
			stats.tp_packets, stats.tp_drops, stats.tp_freeze_q_cnt);
		printf("%lu-p/%lu-B in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\n",
			packets_total, bytes_total, tm_us,
			bytes_total/tm_us*8, tm_us/packets_total);
		goto exit3;
	}

exit4:
	if (packets_total > 0) {
		printf("#### RX %lu packets in %f us; throughput %7.3f Mbps, avg.lentency %7.3f us\n",
			packets_total, tm_us, bytes_total/tm_us*8, tm_us/packets_total);
	}
exit3:
	free(rx_ring.rd);
exit2:
	munmap((void *)rx_ring.map, rx_ring.req.tp_block_size * rx_ring.req.tp_block_nr);
exit1:
	close(fd_sock);
exit0:
#if defined(BUILDING_RRU_DL)
	libiio_app_shutdown();
#endif

	return ec;
}

