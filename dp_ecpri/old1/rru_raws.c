/*
--------------------------------------------------------------------------------
man socket:	int socket(int domain, int type, int protocol); <sys/socket.h>
  https://docs.oracle.com/cd/E19253-01/817-4415/sockets-85885/index.html
--------------------------------------------------------------------------------
domain and protocol
  Name				Purpose 					 	Man page	Protocol
  AF_UNIX, AF_LOCAL	Local communication 			 unix(7)
  AF_INET 			IPv4 Internet protocols 		 ip(7)		/etc/protocols
  AF_INET6			IPv6 Internet protocols 		 ipv6(7)	/etc/protocols
  AF_IPX			IPX - Novell protocols
  AF_NETLINK		Kernel user interface device	 netlink(7)
  AF_X25			ITU-T X.25 / ISO-8208 protocol	 x25(7)
  AF_AX25 			Amateur radio AX.25 protocol
  AF_ATMPVC			Access to raw ATM PVCs
  AF_APPLETALK		AppleTalk						 ddp(7)
  AF_PACKET			Low level packet interface		 packet(7)	linux/if_ether.h
  AF_ALG			Interface to kernel crypto API

type
  SOCK_STREAM: Provides sequenced, reliable, two-way, connection-based byte streams.
  	An out-of-band data transmission mechanism may be supported.
  SOCK_DGRAM: Supports datagrams (connectionless, unreliable messages of a fixed maximum length).
  	PF_PACKET+SOCK_DGRAM: The cooked interface where link level info capture is not 
  	  supported and a link level pseudo-header is provided by the kernel.
  SOCK_SEQPACKET:	Provides a sequenced, reliable, two-way connection-based 
    data transmission path for datagrams of fixed maximum length;
    a consumer is required to read  an  entire packet with each input system call.
  SOCK_RDM: Provides a reliable datagram layer that does not guarantee ordering.
  SOCK_RAW: The raw interface where link level info can be captured.
  	AF_PACKET+SOCK_RAW:
  	PF_PACKET+SOCK_RAW:

type OR status flag
	| SOCK_NONBLOCK: same as fcntl(2) with O_NONBLOCK file status flag
	| SOCK_CLOEXEC:	 same as open(2) with close-on-exec (FD_CLOEXEC) file status flag

--------------------------------------------------------------------------------
AF_PACKET + SOCK_RAW : Basic
http://sock-raw.org/papers/sock_raw
http://opensourceforu.com/2015/03/a-guide-to-using-raw-sockets/
http://www.binarytides.com/raw-udp-sockets-c-linux/
--------------------------------------------------------------------------------
  0, usage:
	i)	capture  network traffic with utilities like tcpdump
	ii) transmit network traffic, or any other that needs raw access to NIC

  1, system-calls copy user-space frame buffer to kernel-space skb
  	| struct ethhdr | user-data without CRC |

  2, raw sockets deliver all packets to "all registered socket" user-processes, 
  so your process will get a copy of the specified <protocol>(e.g. UDP) packet
  as soon as it is got into the system, and other process using a socket with 
  higher layer <protocol>(e.g. UDP) will receive it also.

--------------------------------------------------------------------------------
AF_PACKET + SOCK_RAW : PACKET_MMAP
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ Why use PACKET_MMAP
+ How to use mmap() directly to improve transmission process
+ PACKET_MMAP settings and constraints
+ Mapping and use of the circular buffer (ring)
--------------------------------------------------------------------------------
  0, if PACKET_MMAP is not enabled, the capture process is very inefficient. 
  It uses very limited buffers and requires one system call to
  capture each packet, it requires two if you want to get packet's timestamp
  (like libpcap always does).
  PACKET_MMAP provides a size configurable circular buffer mapped in user space
  that can be used to either send or receive packets.
  i) This way reading packets just needs to wait for them, most of the time
  there is no need to issue a single system call; transmissing multiple packets
  can be sent through one system call to get the highest bandwidth.
  ii) By using a shared buffer between kernel and user also has the benefit of
  minimizing packet copies. ("zero-copy")

  1, Frame structure: <linux/if_packet.h>
   - Start. Frame must be aligned to TPACKET_ALIGNMENT=16
   - struct tpacket_hdr
   - pad to TPACKET_ALIGNMENT=16
   - struct sockaddr_ll
   - Gap, chosen so that packet data (Start+tp_net) aligns to 
     TPACKET_ALIGNMENT=16
   - Start+tp_mac: [ Optional MAC header ]
   - Start+tp_net: Packet data, aligned to TPACKET_ALIGNMENT=16.
   - Pad to align to TPACKET_ALIGNMENT=16

  2, Mapping and use of the circular buffer (ring) for one shared socket by RX/TX
    Even the circular buffer is compound of several physically discontiguous memory
    blocks, they are contiguous to user-space, hence just one call to mmap is needed.
    To use one and only socket for capture and transmission, the mapping of both the
    RX and TX buffer ring has to be done with one call to mmap:
  	setsockopt(fd, SOL_PACKET, PACKET_RX_RING, &foo, sizeof(foo));
  	setsockopt(fd, SOL_PACKET, PACKET_TX_RING, &bar, sizeof(bar));
  	rx_ring = mmap(0, size * 2, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  	tx_ring = rx_ring + size;
    RX must be the first as the kernel maps the TX ring memory right
    after the RX one.

--------------------------------------------------------------------------------
AF_PACKET + SOCK_RAW : optimize further
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
--------------------------------------------------------------------------------
  RX: check the device driver of your NIC
    if it supports some sort of interrupt load mitigation;
    or (even better) if it supports NAPI, also make sure it is enabled. 
  TX: check the MTU used and supported by your NIC;
    CPU IRQ pinning of your NIC can also be an advantage.
*/
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h> /* size_t, offsetof(type, member) */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>	/* GNU getopt(), close() */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <sys/socket.h>	/* socket() */
#include <sys/ioctl.h>	/* ioctl() */
#include <arpa/inet.h>	/* htons/htonl() */

#include "common_jacky.h"
#include "rru_bbu.h"


#define BUILDING_THREAD
//#define BUILDING_RAWS_WITH_BIND
#define BUILDING_RAWS_MMAP
//#define BUILDING_SYNC_PUSCH_TMPPS
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
//#define BUILDING_SOCK_RXQUNFL
//#define BUILDING_SOCK_RXQOVFL
#endif
#define SOCK_EINTR_US4TMPPS 200 /* How does it affect the throughput/latency? */

#if defined(BUILDING_RAWS_MMAP)
struct tpring {
	struct iovec *rd;
	uint8_t *map;
	struct tpacket_req req;
};
#endif

/*---- globals ----*/
#if defined(BUILDING_RRU_DL) || defined(BUILDING_BBU_UL)
static int fd_sockrx;
static struct timespec tm_rs;
static int g_rxloop_stop;
static int ret_rx;
#endif
#if defined(BUILDING_RRU_UL) || defined(BUILDING_BBU_DL)
static int fd_socktx;
# if defined(BUILDING_IP_UDP)
static struct sockaddr_in g_txaddrip;
# else
static struct sockaddr_ll g_txaddrll;
# endif
static struct timespec tm_ts;
static int g_txloop_stop;
static int ret_tx;
#endif
#if defined(BUILDING_RRU_UL)
static int fd_tx_tmpps;
static uint8_t buf_tmpps[TMPPS_UBUF_LEN];
# if defined(BUILDING_SYNC_PUSCH_TMPPS)
static uint32_t testcounter;
# endif
#endif

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./packet_mmap [OPTION] [INTERFACE]\n"
		" -h\tshow this help\n"
		" -g\tuse SOCK_DGRAM\n"
		" -t\tuse dual thread\n"
		" -s\tset packet size\n"
		" -c\tset packet count\n"
		" -m\tset mtu\n"
		" -b\tset buffer size\n"
		" -n\tset buffer count\n"
		" -j\tset send() period (mask==0)\n"
		" -z\tset socket buffer size\n"
		" -l\tPACKET_LOSS(discard wrong packets)\n"
		" -p\tPACKET_QDISC_BYPASS(bypass tc layer)"
		" -e\tgenerate error [num]\n"
		" -v\tbe verbose\n"
		);
}

void getargs(int argc, char **argv)
{
	int c;
	opterr = 0;
	while ( (c = getopt( argc, argv, "e:s:m:b:B:n:c:z:j:vhgtlp")) != EOF) {
		switch ( c ) {
		case 's': c_packet_sz = strtoul( optarg, NULL, 0 ); break;
		case 'c': c_packet_nb = strtoul( optarg, NULL, 0 ); break;
		case 'b': c_buffer_sz = strtoul( optarg, NULL, 0 ); break;
		case 'n': c_buffer_nb = strtoul( optarg, NULL, 0 ); break;
		case 'z': c_sndbuf_sz = strtoul( optarg, NULL, 0 ); break;
		case 'm': c_mtu       = strtoul( optarg, NULL, 0 ); break;
		case 'j': c_send_mask = strtoul( optarg, NULL, 0 ); break;
		case 'e': c_error     = strtoul( optarg, NULL, 0 ); break;
		case 'g': mode_dgram  = 1;                          break;
		case 't': mode_thread    = 1;                          break;
		case 'l': mode_loss   = 1;                          break;
		case 'p': mode_tcbypass = 1;                        break;
		case 'v': mode_verbose= 1;                          break;
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
	printf( "c_packet_sz(-s):   %u\n", c_packet_sz );
	printf( "c_packet_nb(-c):   %u\n", c_packet_nb );
	printf( "c_buffer_sz(-b):   %u\n", c_buffer_sz );
	printf( "c_buffer_nb(-n):   %u\n", c_buffer_nb );
	printf( "c_mtu(-m):         %d\n", c_mtu );
	printf( "c_send_mask(-j):   %d\n", c_send_mask );
	printf( "c_sndbuf_sz(-z):   %u\n", c_sndbuf_sz );
	printf( "mode_loss(-l):     %d\n", mode_loss );
	printf( "mode_tcbypass(-p): %d\n", mode_tcbypass );
	printf( "mode_thread(-t):      %d\n", mode_thread );
	printf( "mode_dgram(-g):    %d\n", mode_dgram );
	printf( "mode_verbose(-v):  %d\n", mode_verbose );
}


/* RX: read "inet_rawsmmap_rx_tpv3.c" for more reference
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
*/
#if defined(BUILDING_RRU_DL) || defined(BUILDING_BBU_UL)
static int raws_rx_startup(void)
{
	int ret, fd;
	struct ifreq req;

#if defined(BUILDING_IP_UDP)
	fd = socket(AF_INET, SOCK_DGRAM, htons(IPPROTO_UDP));
	if (fd < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() AF_INET-SOCK_DGRAM-IPPROTO_UDP");
		return -1;
	}
#elif defined(BUILDING_RAWS_UDP)
	//fd = socket(PF_PACKET, SOCK_RAW, htons(IPPROTO_UDP)); /* IP+UDP */
	//fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL)); /* ETH+IP */
	//fd = socket(PF_PACKET, SOCK_RAW, 0);
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP)); /* ETH+IP */
	if (fd < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() PF_PACKET-SOCK_RAW-ETH_P_IP");
		return -1;
	}
#elif defined(BUILDING_RAWS_VLAN)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_8021Q)); /* ETH */
	if (fd < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() PF_PACKET-SOCK_RAW-ETH_P_8021Q");
		return -1;
	}
#else
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_802_3)); /* ETH */
	if (fd < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() PF_PACKET-SOCK_RAW-ETH_P_802_3");
		return -1;
	}
#endif

	/* set sockaddr info then bind it with the socket */
#if defined(BUILDING_IP_UDP)
	struct sockaddr_in ipme;
	bzero(&ipme, sizeof(ipme));
	ipme.sin_family = AF_INET;
	ipme.sin_port = htons(g_rxport);
	ipme.sin_addr.s_addr = htonl(INADDR_ANY);
	ret = bind(fd, (struct sockaddr *)&ipme, sizeof(ipme));
	if (-1 == ret) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("bind in udps_rx_thread()");
		goto rs_exit0;
	}
#else
# if defined(BUILDING_RAWS_WITH_BIND)
	struct sockaddr_ll llme;
	memset(&llme, 0, sizeof(llme));
	llme.sll_family = AF_PACKET;
#  if defined(BUILDING_IP_UDP) || defined(BUILDING_RAWS_UDP)
	llme.sll_protocol = htons(ETH_P_IP);
#  elif defined(BUILDING_RAWS_VLAN)
	llme.sll_protocol = htons(ETH_P_8021Q);
#  else
	llme.sll_protocol = htons(ETH_P_802_3);
#  endif
	llme.sll_ifindex = if_nametoindex(g_ifname);
	ret = bind(fd, (struct sockaddr *) &llme, sizeof(llme));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("bind");
		goto rs_exit0;
	}
# endif //defined(BUILDING_RAWS_WITH_BIND)
#endif //defined(BUILDING_IP_UDP)

#if defined(BUILDING_SOCK_RXQUNFL)
	struct timeval tm;
	tm.tv_sec = 0;
	tm.tv_usec = 200;/* TODO: How does it affect the throughput/latency? */
	ret = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tm, sizeof(tm));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("setsockopt() with SO_RCVTIMEO");
		goto rs_exit0;
	}
#endif

#if !defined(BUILDING_IP_UDP)
#if 0
	/* RX only, promisc mode in case as a network sniffer */
    memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, g_ifname, sizeof(req.ifr_name));
	ret = ioctl(fd, SIOCGIFFLAGS, &req);
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCGIFFLAGS");
		goto rs_exit0;
	}
	req.ifr_flags |= IFF_PROMISC;
	ret = ioctl(fd, SIOCSIFFLAGS, &req);
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCSIFFLAGS");
		goto rs_exit0;
	}
#endif
#if 0
	/* ifdown <eth0> at first.
	the EMAC hardware and driver should support such MTU length
	current kernel's ability, setting in devicetree */
	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, g_ifname, sizeof(req.ifr_name));
	req.ifr_mtu = 5000;
	ret = ioctl(fd, SIOCSIFMTU, &req);
	if (-1 == ret) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCSIFMTU");
		goto rs_exit0;
	}
#endif
#if defined(BUILDING_RAWS_MMAP)
	/* TPACKET_V3 is valid for RX only */
	int v = TPACKET_V3;
	ret = setsockopt(fd, SOL_PACKET, PACKET_VERSION, &v, sizeof(v));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("setsockopt TPACKET_V3");
		goto rs_exit0;
	}

	struct tpring rx_ring;
	unsigned int bsize;
	memset(&rx_ring.req, 0, sizeof(rx_ring.req));
# if defined(BUILDING_RRU_DL)
	rx_ring.req.tp_frame_size = PDSCH_UBUF_LEN;
# endif
# if defined(BUILDING_BBU_UL)
	rx_ring.req.tp_frame_size = PUSCH_UBUF_LEN;
# endif
	rx_ring.req.tp_block_size = 1024*8;
	rx_ring.req.tp_block_nr = 1024;
	bsize = rx_ring.req.tp_block_size * rx_ring.req.tp_block_nr;
	rx_ring.req.tp_frame_nr = bsize / rx_ring.req.tp_frame_size;
	if (TPACKET_V3 == v) {
		rx_ring.req.tp_retire_blk_tov = 60;
		rx_ring.req.tp_feature_req_word = TP_FT_REQ_FILL_RXHASH;
	}
	ret = setsockopt(fd, SOL_PACKET, PACKET_RX_RING,
			&rx_ring.req, sizeof(rx_ring.req));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("setsockopt");
		goto rs_exit0;
	}

	rx_ring.map = mmap(NULL, bsize,
		  PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
	if (rx_ring.map == MAP_FAILED) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("mmap");
		goto rs_exit0;
	}

	int i;
	rx_ring.rd = malloc(rx_ring.req.tp_block_nr * sizeof(*rx_ring.rd));
	for (i = 0; i < rx_ring.req.tp_block_nr; ++i) {
		rx_ring.rd[i].iov_base = rx_ring.map + (i * rx_ring.req.tp_block_size);
		rx_ring.rd[i].iov_len = rx_ring.req.tp_block_size;
	}
#endif //defined(BUILDING_RAWS_MMAP)
#endif //!defined(BUILDING_IP_UDP)
	return fd;

rs_exit1:
#if !defined(BUILDING_IP_UDP)
#if defined(BUILDING_RAWS_MMAP)
	free(rx_ring.rd);
	munmap(rx_ring.map, rx_ring.req.tp_block_size * rx_ring.req.tp_block_nr);
#endif
#endif
rs_exit0:
	close(fd);
	return ret;
}

static void raws_rx_handler_rru(void)
{
	uint16_t subtype;
	int ret;

	subtype = *(uint16_t *)(sockrxbuf + PKG_OFFSET + offsetof(struct pdsch_hdr, sub_type));
	if (ntohs(PDSCH_SUBTYPE) == subtype) {
		if (0 == pdsch_rx_cnt) clock_gettime(CLOCK_MONOTONIC, &tm_rs);
		unpack_pdsch(sockrxbuf + PKG_OFFSET);
#if defined(BUILDING_IIO_TX)
		if (0 == fill_libiio_txbuf(0)) {
			ret = datapath_handler_tx(0);
			if (-1 == ret) {
				iio_tx_overflow++;
				usleep(iio_tx_overflow*100);
			}
		}
#endif
	} else {
#ifdef BUILDING_DEBUG
		printf("#### UNKNOWN FRAME 0x%04x ####\r\n", subtype);
#endif
	}
}

static void raws_rx_handler_bbu(void)
{
	uint16_t subtype;

	subtype = *(uint16_t *)(sockrxbuf + PKG_OFFSET + offsetof(struct pusch_hdr, sub_type));
	if (ntohs(PUSCH_SUBTYPE) == subtype) {
		if (0 == pusch_rx_cnt) clock_gettime(CLOCK_MONOTONIC, &tm_rs);
		unpack_pusch(sockrxbuf + PKG_OFFSET);
	} else if (ntohs(TMPPS_SUBTYPE) == subtype) {
		unpack_tmpps(sockrxbuf + PKG_OFFSET);
	} else {
#ifdef BUILDING_DEBUG
		printf("#### UNKNOWN FRAME 0x%04x ####\r\n", subtype);
#endif
	}
}

static void *raws_rx_thread(void *parg)
{
	int ret;
#if defined(BUILDING_IP_UDP)
	struct sockaddr_in iprx;
	socklen_t slen = sizeof(iprx);
#else
	struct sockaddr_ll llrx;
	socklen_t slen = sizeof(llrx);
#endif
#if defined(BUILDING_SOCK_RXQOVFL)
	struct iovec iov;
	struct msghdr msg;
	struct cmsghdr *pcmsg;
	uint8_t ctrlmsg[sizeof(__u32)];
	const int dropmonitor = 1;
#endif

#if defined(BUILDING_SOCK_RXQOVFL)
	ret = setsockopt(fd_sockrx, SOL_SOCKET, SO_RXQ_OVFL,
			&dropmonitor, sizeof(dropmonitor));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("setsockopt() with SO_RXQ_OVFL");
		goto rx_exit;
	}
	/* these settings are static */
	iov.iov_base = (void *)sockrxbuf;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
# if defined(BUILDING_IP_UDP)
	msg.msg_name = &iprx;/* optional address */
	msg.msg_namelen = sizeof(iprx);/* size of address */
# else
	msg.msg_name = &llrx;/* optional address */
	msg.msg_namelen = sizeof(llrx);/* size of address */
# endif
	msg.msg_control = &ctrlmsg;
#endif

	/* don't use 'continue' as no-loop when 'function' style */
	do {
rx_redo:
		//display_rx_counters();
#if defined(BUILDING_SOCK_RXQOVFL)
		/* these settings may be modified by recvmsg() */
		iov.iov_len = SUBCARRIER_NRXBUF_SZ;
		msg.msg_controllen = sizeof(ctrlmsg); 
		msg.msg_flags = 0;
		ret = recvmsg(fd_sockrx, &msg, 0/*MSG_DONTWAIT*/);
#else
# if 0
#  if defined(BUILDING_IP_UDP)
		ret = recvfrom(fd_sockrx, sockrxbuf, SUBCARRIER_NRXBUF_SZ,
			0/*MSG_DONTWAIT*/, (struct sockaddr *)&iprx, &slen);
#  else
		ret = recvfrom(fd_sockrx, sockrxbuf, SUBCARRIER_NRXBUF_SZ,
			0/*MSG_DONTWAIT*/, (struct sockaddr *)&llrx, &slen);
#  endif
# else
		ret = recvfrom(fd_sockrx, sockrxbuf, SUBCARRIER_NRXBUF_SZ,
			0/*MSG_DONTWAIT*/, NULL, NULL);
# endif
#endif
		if (ret < 0) {
			/* SIGINT(ctrl-c) CAN be detected while recvfrom() is blocking */
			//fprintf(stderr, "\r\nerrno=%d: ", errno);
			//perror("socket recvfrom()");
			if (EINTR == errno) {
				/* signals are detected while blocking recvfrom()
				the process received a signal while recvfrom() is running before any data was received.
				in our case, it means that SIGALRM have been asserted by ITIMER_REAL. */
				//usleep(SOCK_EINTR_US4TMPPS);/* give way to send timing package at first */
				if (g_rxloop_stop) goto rx_exit;
				else goto rx_redo;
#if defined(BUILDING_SOCK_RXQUNFL)
			} else if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
				/* read timeout due to no more data arriving
				it also means that the sender could send faster */
				raws_rxbuf_underflow++;
				//usleep(1000);/* Wait longer */
    			goto rx_redo;
#endif
			} else {
				fprintf(stderr, "\r\nerrno=%d: ", errno);
				perror("socket recvfrom()");
				goto rx_exit;
			}
		}

		/* reset so that a non-zero value could mean continuous underflow */
		raws_rxbuf_underflow = 0;
#if defined(BUILDING_SOCK_RXQOVFL)
		for (pcmsg = CMSG_FIRSTHDR(&msg);
			pcmsg && (pcmsg->cmsg_level == SOL_SOCKET);
			pcmsg = CMSG_NXTHDR(&msg, pcmsg)) {
				if (pcmsg->cmsg_type == SO_RXQ_OVFL) {
					//memcpy(&dropcnt, CMSG_DATA(pcmsg), sizeof(__u32));
					raws_rxbuf_overflow = *(__u32 *)CMSG_DATA(pcmsg);
				}
		}
#endif

		//display_buffer(sockrxbuf, ret);

#if !defined(BUILDING_RAWS_WITH_BIND)
		ret = raws_unpack_ethhdr(sockrxbuf);
		if (0 != ret) goto rx_redo;
#endif

#if defined(BUILDING_RAWS_UDP)
		unpack_iphdr_udphdr(fd_sockrx, sockrxbuf + ETH_HDR_LEN);
		if (0 != ret) goto rx_redo;
#endif

#if defined(BUILDING_RRU_DL)
		raws_rx_handler_rru();
#endif
#if defined(BUILDING_BBU_UL)
		raws_rx_handler_bbu();
#endif
	} while (!g_rxloop_stop);

rx_exit:
	g_rxloop_stop = 1;
	ret_rx = ret;
	return (void *)&ret_rx;
}
#endif

/* TX: read "inet_rawsmmap_tx.c" for more reference
--------------------------------------------------------------------------------
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ How to use mmap() directly to improve transmission process
--------------------------------------------------------------------------------
[setup]          socket() -------> creation of the transmission socket
                 setsockopt() ---> allocation of the circular buffer (ring)
                                   option: PACKET_TX_RING
                 bind() ---------> bind transmission socket with an interface
                 mmap() ---------> mapping of the allocated buffer to the
                                   user process

[transmission]   poll() ---------> wait for free packets (optional)
                 send() ---------> send all packets that are set as ready in
                                   the ring. The flag MSG_DONTWAIT can be used
                                   to return before end of transfer.
  #define TP_STATUS_AVAILABLE        0 // Frame is available
  #define TP_STATUS_SEND_REQUEST     1 // Frame will be sent on next send()
  #define TP_STATUS_SENDING          2 // Frame is currently in transmission
  #define TP_STATUS_WRONG_FORMAT     4 // Frame format is not correct
  First, the kernel initializes all frames to TP_STATUS_AVAILABLE. To send a
  packet, the user fills a data buffer of an available frame, sets tp_len to
  current data buffer size and sets its status field to TP_STATUS_SEND_REQUEST.
  This can be done on multiple frames. Once the user is ready to transmit, it
  calls send(). Then all buffers with status equal to TP_STATUS_SEND_REQUEST
  are forwarded to the network device. The kernel updates each status of sent
  frames with TP_STATUS_SENDING until the end of transfer.
  At the end of each transfer, buffer status returns to TP_STATUS_AVAILABLE.

[shutdown]  close() --------> destruction of the transmission socket and
                              deallocation of all associated resources.
*/
#if defined(BUILDING_RRU_UL) || defined(BUILDING_BBU_DL)
static int raws_tx_startup(void)
{
	int ret, fd, tmp;
	struct ifreq req;

#if defined(BUILDING_IP_UDP)
	fd = socket(AF_INET, SOCK_DGRAM, htons(IPPROTO_UDP));
	if (fd < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() AF_INET-SOCK_DGRAM-IPPROTO_UDP");
		return -1;
	}
#elif defined(BUILDING_RAWS_UDP)
	//fd = socket(PF_PACKET, SOCK_RAW, htons(IPPROTO_UDP)); /* IP+UDP */
	//fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ALL)); /* ETH+IP */
	//fd = socket(PF_PACKET, SOCK_RAW, 0);
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP)); /* ETH+IP */
	if (-1 == fd) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() PF_PACKET-SOCK_RAW-ETH_P_IP");
		return -1;
	}
#elif defined(BUILDING_RAWS_VLAN)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_8021Q)); /* ETH */
	if (-1 == fd) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() PF_PACKET-SOCK_RAW-ETH_P_8021Q");
		return -1;
	}
#else
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_802_3)); /* ETH */
	if (-1 == fd) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket() PF_PACKET-SOCK_RAW-ETH_P_802_3");
		return -1;
	}
#endif

	/* set sockaddr info then bind it with the socket */
#if defined(BUILDING_IP_UDP)
	struct sockaddr_in ipme;
	bzero(&ipme, sizeof(ipme));
	ipme.sin_family = AF_INET;
	ipme.sin_port = htons(g_txport);
	ipme.sin_addr.s_addr = htonl(INADDR_ANY);
	ret = bind(fd, (struct sockaddr *)&ipme, sizeof(ipme));
	if (-1 == ret) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("bind in udps_rx_thread()");
		goto rs_exit0;
	}
#else
# if defined(BUILDING_RAWS_WITH_BIND)
	struct sockaddr_ll llme;
	memset(&llme, 0, sizeof(llme));
	llme.sll_family = AF_PACKET;
#  if defined(BUILDING_RAWS_UDP)
	llme.sll_protocol = htons(ETH_P_IP);
#  elif defined(BUILDING_RAWS_VLAN)
	llme.sll_protocol = htons(ETH_P_8021Q);
#  else
	llme.sll_protocol = htons(ETH_P_802_3);
#  endif
	llme.sll_ifindex = if_nametoindex(g_ifname);
	ret = bind(fd, (struct sockaddr *)&llme, sizeof(llme));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("bind");
		goto ts_exit0;
	}
# endif //defined(BUILDING_RAWS_WITH_BIND)
#endif //defined(BUILDING_IP_UDP)

#if !defined(BUILDING_IP_UDP)
#if 0
	/* ifdown <eth0> at first
	the EMAC hardware and driver should support such MTU length
	current kernel's ability, setting in devicetree */
	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, g_ifname, sizeof(req.ifr_name));
	req.ifr_mtu = 5000;
	ret = ioctl(fd, SIOCSIFMTU, &req);
	if (-1 == ret) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCSIFMTU");
		goto ts_exit0;
	}
#endif
#if defined(BUILDING_RAWS_MMAP)
	/* TPACKET_V3 is valid for RX only */
	int v = TPACKET_V2;
	ret = setsockopt(fd, SOL_PACKET, PACKET_VERSION, &v, sizeof(v));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("setsockopt TPACKET_V2");
		goto ts_exit0;
	}

	struct tpring tx_ring;
	unsigned int bsize;
	memset(&tx_ring.req, 0, sizeof(tx_ring.req));
# if defined(BUILDING_RRU_UL)
	tx_ring.req.tp_frame_size = 1024*4; //PUSCH_UBUF_LEN;
# endif
# if defined(BUILDING_BBU_DL)
	tx_ring.req.tp_frame_size = 1024*4; //PDSCH_UBUF_LEN;
# endif
	tx_ring.req.tp_block_size = 1024*8;
	tx_ring.req.tp_block_nr = 1024;
	bsize = tx_ring.req.tp_block_size * tx_ring.req.tp_block_nr;
	tx_ring.req.tp_frame_nr = bsize / tx_ring.req.tp_frame_size;

	ret = setsockopt(fd, SOL_PACKET, PACKET_TX_RING,
			&tx_ring.req, sizeof(tx_ring.req));
	if (ret < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("setsockopt");
		goto ts_exit0;
	}

	tx_ring.map = mmap(NULL, bsize,
			PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
	if (tx_ring.map == MAP_FAILED) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("mmap");
		goto ts_exit0;
	}

	/*int i;
	tx_ring.rd = malloc(tx_ring.req.tp_block_nr * sizeof(*tx_ring.rd));
	assert(tx_ring.rd);
	for (i = 0; i < tx_ring.req.tp_block_nr; ++i) {
		tx_ring.rd[i].iov_base = tx_ring.map + (i * tx_ring.req.tp_block_size);
		tx_ring.rd[i].iov_len = tx_ring.req.tp_block_size;
	}*/

# if defined(BUILDING_RAWS_TXPLOSS)
	tmp = 1;
	ret = setsockopt(fd_socket, SOL_PACKET, PACKET_LOSS,
			(char *)&tmp, sizeof(tmp));
	if (ret < 0) {
		perror("setsockopt: PACKET_LOSS");
		goto ts_exit1;
	}
# endif
# if defined(BUILDING_RAWS_TXBYPASSQC)
	tmp = 1;
	ret = setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS,
			(char *)&tmp, sizeof(tmp));
	if (ret < 0) {
		perror("setsockopt: QDISC_BYPASS");
		goto ts_exit1;
	}
# endif
#endif //defined(BUILDING_RAWS_MMAP)
#endif //!defined(BUILDING_IP_UDP)
	return fd;

ts_exit1:
#if !defined(BUILDING_IP_UDP)
#if defined(BUILDING_RAWS_MMAP)
	//free(tx_ring.rd);
	munmap(tx_ring.map, tx_ring.req.tp_block_size * tx_ring.req.tp_block_nr);
#endif
#endif
ts_exit0:
	close(fd);
	return 1;
}

/* fill peer sockaddr for SOCK_DGRAM */
static void raws_tx_fill_peeraddr(void)
{
#if defined(BUILDING_IP_UDP)
	bzero(&g_txaddrip, sizeof(g_txaddrip));
	g_txaddrip.sin_family = AF_INET;
	g_txaddrip.sin_port   = htons(g_txport);
	if (inet_aton(g_peer_ips, &(g_txaddrip.sin_addr)) == 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("inet_aton()");
		return -1;
	}
	//printf("UDP TX -> %s:%d\r\n", inet_ntoa(g_txaddrip.sin_addr), ntohs(g_txaddrip.sin_port));
#else
	bzero(&g_txaddrll, sizeof(g_txaddrll));
#if 0
	struct ifreq req;
	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, g_ifname, sizeof(req.ifr_name));
	if (ioctl(fd, SIOCGIFINDEX, &req) < 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("socket ioctl() SIOCGIFINDEX");
		return ret;
	}
	g_txaddrll.sll_ifindex  = req.ifr_ifindex;
#else
	g_txaddrll.sll_ifindex  = if_nametoindex(g_ifname);
#endif
	g_txaddrll.sll_family   = AF_PACKET;
	g_txaddrll.sll_protocol = htons(ETH_P_IP);
	g_txaddrll.sll_halen    = ETH_ALEN;
	memcpy(&g_txaddrll.sll_addr, g_peer_mac, ETH_ALEN);
#endif
}

int raws_tx_prepare_txbuf(void)
{
	char *p;
	int ret, i;

	/* prepare headers before start */
	for (i=0, p=socktxbuf; i<SUBCARRIER_NTXBUF_NB; i++, p+=SUBCARRIER_NTXBUF_SZ) {
# if !defined(BUILDING_IP_UDP)
#  if !defined(BUILDING_RAWS_WITH_BIND)
		ret = raws_pack_ethhdr(fd_socktx, p);
		if (ret < 0) return ret;
#  endif
# endif
# if defined(BUILDING_RAWS_UDP)
#  if defined(BUILDING_RRU_UL)
		ret = pack_iphdr_udphdr(fd_socktx, p + ETH_HDR_LEN, PUSCH_SIZE);
#  else
		ret = pack_iphdr_udphdr(fd_socktx, p + ETH_HDR_LEN, PDSCH_SIZE);
#  endif
		if (0 != ret) return ret;
# endif
	}

{
	int ec, j;
	static int sleft = 0;
	static char *p_iiosample;

	/* ethernet header : buf+=ETH_HLEN */
	ec = raw_ethhdr_fill(buf);

	/* IP+UDP   header : buf+=IP_HLEN+UDP_HLEN */
	ec = pack_iphdr_udphdr(fd_socktx, buf + ETH_HDR_LEN, PUSCH_SIZE);

	/* PUSCH    header : buf+=PKG_OFFSET */
	pack_pusch_hdr(buf + PKG_OFFSET);

	/* PUSCH   payload : buf+=PUSCH_SIZE */
	if (0 == sleft) {
		sleft = iio_buffer_refill(iiorxcbuf[0]);
		if (sleft < 0) {
			fprintf(stderr, "iio_buffer_refill() %s\r\n", strerror(-sleft));
			return -1;
		}
		p_iiosample = iio_buffer_first(rxbuf[0], rxport_i[0*PORT_PER_DEVICE]);
	}
	memcpy(buf, p_iiosample, p_iiosample*2);
	p_iiosample += ;
	sleft -= ;
	if (sleft == 0) datapath_handler_rx(0); /* I[IIO_PORT_SAMPLES] + Q[IIO_PORT_SAMPLES] */
	sleft = dump_libiio_rxbuf(0); /* I[SUBCARRIER_SAMPLES] + Q[SUBCARRIER_SAMPLES] */

}

}

/*---- leaf function ----*/
static int raws_tx_one(int fd, uint8_t *buf, size_t tlen)
{
	int ret;
	size_t lsent;

	assert(fd != -1);
	assert(buf != NULL);
	assert(tlen <= SUBCARRIER_NTXBUF_SZ);

	display_buffer(buf, tlen);
	lsent = 0;
tx_redo:
	//display_tx_counters();
#if defined(BUILDING_IP_UDP)
	ret = sendto(fd, buf + lsent, tlen - lsent,
			0, (struct sockaddr *)&g_txaddrip, sizeof(g_txaddrip));
#else
	ret = sendto(fd, buf + lsent, tlen - lsent,
			0, (struct sockaddr *)&g_txaddrll, sizeof(g_txaddrll));
#endif
	if (ret < 0) {
		/* SIGINT(ctrl-c) cannot be detected while blocking sendto() */
		//fprintf(stderr, "\r\nerrno=%d: ", errno);
		//perror("socket sendto() PUSCH");
		if (EINTR == errno) {
			/* signals are detected while executing a blocking system call
			the process received a signal while sendto() is running before any data was transmitted.
			in our case, it means that SIGALRM have been asserted by ITIMER_REAL.*/
			//usleep(SOCK_EINTR_US4TMPPS);/* give way to send timing package at first */
			if (g_txloop_stop) return ret;
			else goto tx_redo;
		} else if (ENOBUFS == errno) {
			/* ENOBUFS: The output queue was full (transient congestion). 
			(Normally, this doesn't occur in Linux as packets are just silently dropped
			when a device queue overflows. In other words, Linux device driver doesn't report overflow) */
			raws_txbuf_overflow++;/* continuous overflow */
			//sleep(raws_txbuf_overflow);/* long sleep */
			goto tx_redo;
		} else if (EMSGSIZE == errno) {
			fprintf(stderr, "\r\nETH_PUSCH_ALL_LEN > MTU");
			return ret;
		} else {
			fprintf(stderr, "\r\nerrno=%d: ", errno);
			perror("socket sendto()");
			return ret;
		}
	}

	lsent += ret; if (lsent < tlen) goto tx_redo;

	/* reset so that a non-zero value could mean continuous overflow */
	raws_txbuf_overflow = 0;
	return 0;
}

static void *raws_tx_thread(void *parg)
{
	int ret;
#if defined(BUILDING_RRU_UL)
	int sleft = 0;
#endif

	do {
#if defined(BUILDING_RRU_UL)
		//fill_pusch_pld(socktxbuf + PUSCH_PLDOF);
		pack_pusch_hdr(socktxbuf + PKG_OFFSET);
		ret = raws_tx_one(fd_socktx, socktxbuf, PUSCH_UBUF_LEN);
		if (ret < 0) goto tx_exit;
		if (0 == pusch_tx_cnt_symbl) clock_gettime(CLOCK_MONOTONIC, &tm_ts);
		pusch_tx_cnt_symbl++;
#else //defined(BUILDING_BBU_DL)
		fill_pdsch_pld(socktxbuf + PDSCH_PLDOF);
		pack_pdsch_hdr(socktxbuf + PKG_OFFSET);
		ret = raws_tx_one(fd_socktx, socktxbuf, PDSCH_UBUF_LEN);
		if (ret < 0) goto tx_exit;
		if (0 == pdsch_tx_cnt_symbl) clock_gettime(CLOCK_MONOTONIC, &tm_ts);
		pdsch_tx_cnt_symbl++;
#endif
	} while (!g_txloop_stop);

tx_exit:
	g_txloop_stop = 1;
	ret_tx = ret;
	return (void *)&ret_tx;
}

/* TX in a signal handler context
--------------------------------------------------------------------------------
*/
#if defined(BUILDING_RRU_UL)
#if 0
pthread_t pid_tmpps;
extern void *rru_gpioirq_thread(void *parg);
#endif
void s_tmpps_handler(int signo)
{
	int ret;
	int sleft;

	/*if (SIGALRM != signo) {
		fprintf(stderr, "\r\n%s Unknown signal %d\r\n", __func__, signo);
		return;
	}*/
#if defined(BUILDING_SYNC_PUSCH_TMPPS)
	if (12 != testcounter % 15) {
		if (sleft == 0)	datapath_handler_rx(0);
		sleft = dump_libiio_rxbuf(0);
		//fill_pusch_pld(socktxbuf + PUSCH_PLDOF);
		pack_pusch_hdr(socktxbuf + PKG_OFFSET);
		ret = raws_tx_one(fd_socktx, socktxbuf, PUSCH_UBUF_LEN);
		if (ret < 0) goto tmpps_exit;
		if (0 == pusch_tx_cnt_symbl) clock_gettime(CLOCK_MONOTONIC, &tm_ts);
		pusch_tx_cnt_symbl++;
		testcounter++;
		return; // succeed
	}
#endif
	pack_tmpps(buf_tmpps + PKG_OFFSET);
	ret = raws_tx_one(fd_tx_tmpps, buf_tmpps, TMPPS_UBUF_LEN);
	if (ret < 0) goto tmpps_exit;
	tmpps_tx_cnt++;
#if defined(BUILDING_SYNC_PUSCH_TMPPS)
	testcounter++;
#endif
	return; // succeed

tmpps_exit:
	return; // fail
}

static void hrtimer_init(void)
{
	struct sigaction tact;
	struct itimerval ttv;

	tact.sa_handler = s_tmpps_handler;
	tact.sa_flags = 0;
	sigemptyset(&tact.sa_mask);
	sigaction(SIGALRM, &tact, NULL);

	ttv.it_value.tv_sec = 0;
	ttv.it_value.tv_usec = 1000;
	ttv.it_interval.tv_sec = 0; // 2 for debug
#if defined(BUILDING_SYNC_PUSCH_TMPPS)
	ttv.it_interval.tv_usec = 1000/16; // 1000/15; // 1ms PPS
#else
	ttv.it_interval.tv_usec = 1000; // 1ms PPS
#endif
	setitimer(ITIMER_REAL, &ttv, NULL);
}

static void hrtimer_exit(void)
{
	struct itimerval ttv;

	ttv.it_value.tv_sec = 0;
	ttv.it_value.tv_usec = 0;
	ttv.it_interval.tv_sec = 0;
	ttv.it_interval.tv_usec = 0;
	setitimer(ITIMER_REAL, &ttv, NULL);
}
#endif

#endif

/*----  */
void sighandler_abort(int signo)
{
	if (SIGINT != signo)
		return;

	printf("\r\nWaiting for process to finish...\r\n");

#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	g_rxloop_stop = 1;
#endif
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	g_txloop_stop = 1;
#endif
}

#if 1
/* sync/blocking mode and asyn/non-block mode
*/
int main(int argc, char *argv[])
{
	char *p;
	int ret, i;
	struct timespec tm_xe;
	pthread_t pid_tx, pid_rx;

	i = 1;
	printf("%s\r\n", 0x00 == *((char *)&i) ? "BigEndian" : "LittleEndian");

	if (argc > 1) {
		strncpy(g_ifname, argv[1], 31);
	}
	printf("to use the ethernet IF %s\r\n", g_ifname);

	if (argc > 2) {
		macaddr_s2a(g_peer_mac, argv[2]);
	}
	printf("RAW interact with the remote ");
	macaddr_print(g_peer_mac);
	printf("\r\n");

#if defined(BUILDING_RAWS_UDP)
# if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	if (argc > 3) {
		strncpy(g_peer_ips, argv[3], sizeof(g_peer_ips) - 1);
	}
	printf("UDP tx->%s:%d\r\n", g_peer_ips, g_txport);
# endif
# if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	printf("UDP rx<-any:%d\r\n", g_rxport);
# endif
#endif

	clear_counters();

	libiio_app_startup(IIO_PORT_SAMPLES);

	/* Listen to ctrl+c and assert */
	signal(SIGINT, sighandler_abort);

	/*---- rru_dl rx pdsch or bbu_ul rx pdsch && tmpps */
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	fd_sockrx = raws_rx_startup();
	if (fd_sockrx < 0) goto m_exit0;

# if defined(BUILDING_THREAD)
	g_rxloop_stop = 0;
	ret = pthread_create(&pid_rx, NULL, raws_rx_thread, NULL);
	if (ret != 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("pthread_create() for raws_rx_thread\r\n");
		close(fd_sockrx);
		goto m_exit1;
	}
# endif
#endif

	/*---- rru_ul tx pusch or bbu_dl tx pdsch */
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	fd_socktx = raws_tx_startup();
	if (fd_socktx < 0) goto m_exit1;

	ret = raws_tx_prepare_txbuf();
	if (ret < 0) { close(fd_socktx); goto m_exit1; }

# if defined(BUILDING_THREAD)
#  if !defined(BUILDING_SYNC_PUSCH_TMPPS)
	g_txloop_stop = 0;
	ret = pthread_create(&pid_tx, NULL, raws_tx_thread, NULL);
	if (ret != 0) {
		fprintf(stderr, "\r\nerrno=%d: ", errno);
		perror("pthread_create() for raws_tx_thread\r\n");
		close(fd_socktx);
		goto m_exit1;
	}
#  endif
# endif

#endif //defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)

	/*---- rru_ul tx tmpps */
#if 0//defined(BUILDING_RRU_UL)
	fd_tx_tmpps = raws_tx_startup(); // TODO change package size into TMPPS_SIZE
	if (fd_tx_tmpps < 0) goto m_exit2;

# if !defined(BUILDING_IP_UDP)
#  if !defined(BUILDING_RAWS_WITH_BIND)
	ret = raws_pack_ethhdr(fd_tx_tmpps, buf_tmpps, &g_txaddrll);
	if (ret < 0) { close(fd_tx_tmpps); goto m_exit2; }
#  endif
# endif
# if defined(BUILDING_RAWS_UDP)
	ret = pack_iphdr_udphdr(fd_tx_tmpps, buf_tmpps + ETH_HDR_LEN, TMPPS_SIZE);
	if (ret < 0) { close(fd_tx_tmpps); goto m_exit2; }
# endif

	hrtimer_init(); /* start timer to for tmpps */
#endif

	/*---- wait for thread-terminate */
#if defined(BUILDING_THREAD)
	pthread_join(pid_rx, NULL);
	pthread_join(pid_tx, NULL);
#else
	do {
# if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
		raws_tx_thread(NULL);
		if (g_txloop_stop) break;
# endif
# if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
		raws_rx_thread(NULL);
		if (g_rxloop_stop) break;
# endif
		//display_counters();
	} while (1);
#endif

	/*---- performance */
	clock_gettime(CLOCK_MONOTONIC, &tm_xe);
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	display_tx_perf(&tm_xe, &tm_ts);
#endif
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	display_rx_perf(&tm_xe, &tm_rs);
#endif

m_exit3:
#if 0//defined(BUILDING_RRU_UL)
	hrtimer_exit();
	close(fd_tx_tmpps);
#endif
m_exit2:
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
# if defined(BUILDING_THREAD)
#  ifndef BUILDING_SYNC_PUSCH_TMPPS
	pthread_exit(&pid_tx);
#  endif
# endif
	close(fd_socktx);
#endif
m_exit1:
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
# if defined(BUILDING_THREAD)
	pthread_exit(&pid_rx);
# endif
	close(fd_sockrx);
#endif
m_exit0:
	libiio_app_shutdown();
	return ret;
}
#else
/*	Input/Output Multiplexing
1, select:
----
	#include <sys/time.h>
	#include <sys/types.h>
	#include <sys/select.h>
	...
	fd_set readmask, writemask, exceptmask;
	struct timeval timeout;
	...
	select(nfds, &readmask, &writemask, &exceptmask, &timeout);
	If the timeout pointer is NULL, select() blocks until a descriptor is selectable, or until a signal is received.
	If the fields in timeout are set to 0, select() polls and returns immediately. 

2, poll:
----

3, epoll:
----
*/
#define PPS_IRQ_GPIO_NUM 511
int main(int argc, char *argv[])
{
	int ret, fd_socktx, fd_sockrx, fd_gpio;
	struct timespec tm_xe;
	struct sockaddr_ll lladdr;

	int i = 1;
	printf("%s\r\n", 0x00 == *((char *)&i) ? "BigEndian" : "LittleEndian");

	if (argc > 1) {
		strncpy(g_ifname, argv[1], 31);
	}
	printf("to use the ethernet IF %s\r\n", g_ifname);

	if (argc > 2) {
		macaddr_s2a(g_peer_mac, argv[2]);
	}
	printf("to interact with the remote ip node ");
	macaddr_print(g_peer_mac);
	printf("\r\n");

#if defined(BUILDING_RAWS_UDP)
	if (argc > 3) {
		strncpy(g_peer_ips, argv[3], sizeof(g_peer_ips) - 1);
	}
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	printf("UDP tx->%s:%d\r\n", g_peer_ips, g_txport);
#endif
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	printf("UDP rx<-any:%d\r\n", g_rxport);
#endif
#endif

	clear_counters();
	libiio_app_startup(IIO_PORT_SAMPLES);

	signal(SIGINT, sighandler_abort);// Listen to ctrl+c and assert
	g_txloop_stop = 1;
	g_rxloop_stop = 1;

#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	fd_socktx = raws_tx_startup(g_ifname);
	if (fd_socktx < 0)
		return -1;
	ret = raws_pack_ethhdr(fd_socktx, socktxbuf, &lladdr);
	if (ret < 0) {
		close(fd_socktx);
		return -1;
	}
#endif
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	fd_sockrx = raws_rx_startup(g_ifname);
	if (fd_sockrx < 0)
		return -1;
#endif

#if defined(BUILDING_RRU_UL)
	gpio_export(PPS_IRQ_GPIO_NUM);
	gpio_set_dir(PPS_IRQ_GPIO_NUM, 0);//input
	gpio_set_edge(PPS_IRQ_GPIO_NUM, "rising");
	//gpio_set_level(PPS_IRQ_GPIO_NUM, 1);//active-high
	fd_gpio = gpio_fd_open(PPS_IRQ_GPIO_NUM);
	if (fd_gpio < 0) {
		return -1;
	}
#endif

#if 0
	fd_set rset, wset;
	int maxfd;
	struct timeval tv;

	maxfd = max(fd_sockrx, fd_socktx);
	// Initialize time out struct
	tv.tv_sec = 0;
	tv.tv_usec = 4000;//4ms per timing-package requirement

	do { // TODO: debug
		// Initialize the set
		FD_ZERO(&rset);
		FD_SET(fd_gpio, &rset);
		FD_SET(fd_sockrx, &rset);
		FD_ZERO(&wset);
		FD_SET(fd_socktx, &wset);

		ret = select(maxfd+1, &rset, &wset, NULL, NULL/*&tv*/);
		if (ret < 0) {
			fprintf(stderr, "\r\nerrno=%d: ", errno);
			perror("select");
			break;
		}
		if (ret == 0) {//timeout
			//printf(".");
			continue;
		}

#if defined(BUILDING_RRU_UL)
		if (FD_ISSET(fd_gpio, &rset) && FD_ISSET(fd_socktx, &wset)) {
			pack_tmpps(socktxbuf + PKG_OFFSET);
			ret = raws_tx_one(fd_socktx, socktxbuf, TMPPS_UBUF_LEN);
			if (ret < 0) break;
			tmpps_tx_cnt++;
		}
#endif
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
		if (FD_ISSET(fd_socktx, &wset)) {
			ret = *(int *)raws_tx_thread(&fd_socktx);
			if ((ret < 0) || (g_txloop_stop)) break;
		}
#endif
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
		if (FD_ISSET(fd_sockrx, &rset)) {
			ret = *(int *)raws_rx_thread(&fd_sockrx);
			if ((ret < 0) || (g_rxloop_stop)) break;
		}
#endif
	} while (1);
#else
	struct pollfd fdset[3];
	int nfds = 3;
	int timeout = 4; // ms

	do {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = fd_gpio;
		fdset[0].events = POLLPRI;
		//fdset[1].fd = fd_socktx;
		//fdset[1].events = POLLOUT;
		fdset[2].fd = fd_sockrx;
		fdset[2].events = POLLIN;

		ret = poll(fdset, nfds, timeout);
		if (ret < 0) {
			fprintf(stderr, "\r\nerrno=%d: ", errno);
			perror("poll()");
			//assert((errno == EINVAL) && (errno == EBADF));
			if (errno == ENOMEM) return -1;
			if (errno == EINTR) {
				return -1;
			}
		} else if (ret == 0) {//timeout
			//printf(".");
			continue;
		}

#if defined(BUILDING_RRU_UL)
		if ((fdset[0].revents & POLLPRI)) {
			//lseek(fdset[0].fd, 0, SEEK_SET);
			//len = read(fdset[0].fd, buf, MAX_BUF);
			//printf("\r\npoll() GPIO %d interrupt occurred\r\n", gpio);
			pack_tmpps(socktxbuf + PKG_OFFSET);
			ret = raws_tx_one(fd_socktx, socktxbuf, TMPPS_UBUF_LEN);
			if (ret < 0) break;
			tmpps_tx_cnt++;
		}
#endif
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
		if (fdset[1].revents & POLLOUT) {
			ret = *(int *)raws_tx_thread(&fd_socktx);
			if ((ret < 0) || (g_txloop_stop)) break;
		}
#endif
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
		if (fdset[1].revents & POLLIN) {
			ret = *(int *)raws_rx_thread(&fd_sockrx);
			if ((ret < 0) || (g_rxloop_stop)) break;
		}
#endif
	} while (!g_process_stop);
#endif

m_exit:
#if defined(BUILDING_BBU_UL) || defined(BUILDING_RRU_DL)
	if (-1 != fd_sockrx) { close(fd_sockrx); fd_sockrx = -1; }
#endif
#if defined(BUILDING_RRU_UL)
	gpio_fd_close(fd_gpio);
#endif
#if defined(BUILDING_BBU_DL) || defined(BUILDING_RRU_UL)
	if (-1 != fd_socktx) { close(fd_socktx); fd_socktx = -1; }
#endif
	libiio_app_shutdown();
	return ret;
}
#endif
