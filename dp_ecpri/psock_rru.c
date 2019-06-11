/*
 * applications based on libiio
 *   - RRU2BBU datapath based on package-socket with mmap
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Jacky Wang <kenwj@sina.com>
 *
 * License: GPL, version 2.1
 *
 *------------------------------------------------------------------------------
"man socket":	int socket(int domain, int type, int protocol); <sys/socket.h>
https://docs.oracle.com/cd/E19253-01/817-4415/sockets-85885/index.html
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
  SOCK_DGRAM: Supports datagrams (connectionless, unreliable messages of a fixed max length).
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
 *------------------------------------------------------------------------------
AF_PACKET + SOCK_RAW : Basic
http://sock-raw.org/papers/sock_raw
http://opensourceforu.com/2015/03/a-guide-to-using-raw-sockets/
http://www.binarytides.com/raw-udp-sockets-c-linux/
  0, usage:
	i)	capture  network traffic with utilities like tcpdump
	ii) transmit network traffic, or any other that needs raw access to NIC
  1, system-calls copy user-space frame buffer to kernel-space skb
  	| struct ethhdr | user-data without CRC |
  2, raw sockets deliver all packets to "all registered socket" user-processes, 
  so your process will get a copy of the specified <protocol>(e.g. UDP) packet
  as soon as it is got into the system, and other process using a socket with 
  higher layer <protocol>(e.g. UDP) will receive it also.
 *------------------------------------------------------------------------------
AF_PACKET + SOCK_RAW : PACKET_MMAP
https://www.kernel.org/doc/Documentation/networking/packet_mmap.txt
+ Why use PACKET_MMAP
+ How to use mmap() directly to improve transmission process
+ PACKET_MMAP settings and constraints
+ Mapping and use of the circular buffer (ring)
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
+ How to use mmap() directly to improve capture process
+ How to use mmap() directly to improve transmission process
 *------------------------------------------------------------------------------
+ AF_PACKET + SOCK_RAW : optimize further
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

#include <sys/select.h>
#include <sys/stropts.h>
#include <poll.h>
#include <sys/epoll.h>

#if defined(BUILDING_RRU_DL) || defined(BUILDING_RRU_UL)
#include <iio.h>
#endif
#include "common_jacky.h"

#include "rru_bbu.h"


//#define BUILDING_RAWS_WITH_BIND
//#define BUILDING_RAWS_MMAP

#if defined(BUILDING_RRU_DL)
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
static int fd_rxsock, fd_txsock;
static uint8_t buf_tmpps[TMPPS_UBUF_LEN];

/* params */
extern char *str_devname;
static unsigned int mode_verbose = 0;
#if 0
static unsigned int c_packet_sz = RAWSRX_BUF_SZ; //4862, 4890
static unsigned int c_packet_nb = IIO_PORT_SAMPLES/RADIO_SYM2SMP; //873
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

static void usage(void)
{
	fprintf( stderr,
		"Usage: ./a.out [OPTION] [INTERFACE]\n"
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

static void get_args(int argc, char **argv)
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
		case 't': mode_thread = 1;                          break;
		case 'l': mode_loss   = 1;                          break;
		case 'p': mode_tcbypass = 1;                        break;
		case 'v': mode_verbose = 1;                         break;
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
#endif

static volatile sig_atomic_t procstop = 0;
static void handle_sig(int signo)
{
	if (SIGINT != signo) return;

	printf("\nWaiting for process to finish...\n");
    taskstop_rawst = 1;
# if defined(TMPPS_TRG_UGPIOIRQ)
    taskstop_tmirq = 1;
# endif
    taskstop_rawsr = 1;
	procstop = 1;
}

static int raws_rx_startup(void)
{
	int ec, fd;
	struct ifreq req;

#if defined(BUILDING_AFINETS_UDP)
	fd = socket(AF_INET, SOCK_DGRAM, htons(IPPROTO_UDP));
#elif defined(BUILDING_PFRAWS_UDP)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP)); /* ETH+IP */
#elif defined(BUILDING_PFRAWS_VLAN)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_8021Q)); /* ETH */
#else //defined(BUILDING_PFRAWS_ETH)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_802_3)); /* ETH */
#endif
	if (fd < 0) {
		perror("socket()");
		return -1;
	}

	ec = sock_bind_ipport(fd, 0);
	if (ec < 0) { return -1; }

#if !defined(BUILDING_AFINETS_UDP)
# if 0
	/* RX only, promisc mode in case as a network sniffer */
    memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, str_devname, sizeof(req.ifr_name));
	ec = ioctl(fd, SIOCGIFFLAGS, &req);
	if (ec < 0) {
		perror("socket ioctl() SIOCGIFFLAGS");
		goto rs_exit0;
	}
	req.ifr_flags |= IFF_PROMISC;
	ec = ioctl(fd, SIOCSIFFLAGS, &req);
	if (ec < 0) {
		perror("socket ioctl() SIOCSIFFLAGS");
		goto rs_exit0;
	}
# endif
# if 0
	/* ifdown <eth0> at first.
	the EMAC hardware and driver should support such MTU length
	current kernel's ability, setting in devicetree */
	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, str_devname, sizeof(req.ifr_name));
	req.ifr_mtu = 5000;
	ec = ioctl(fd, SIOCSIFMTU, &req);
	if (-1 == ec) {
		perror("socket ioctl() SIOCSIFMTU");
		goto rs_exit0;
	}
# endif
# if defined(BUILDING_RAWS_MMAP)
	/* TPACKET_V3 is valid for RX only */
	int v = TPACKET_V3;
	ec = setsockopt(fd, SOL_PACKET, PACKET_VERSION, &v, sizeof(v));
	if (ec < 0) {
		perror("setsockopt TPACKET_V3");
		goto rs_exit0;
	}

	struct tpring rx_ring;
	memset(&rx_ring.req, 0, sizeof(rx_ring.req));
	rx_ring.req.tp_frame_size = 1024*8;
	rx_ring.req.tp_block_size = 1024*8;
	rx_ring.req.tp_block_nr = 1024;
	rx_ring.req.tp_frame_nr = bsize / rx_ring.req.tp_frame_size;
	if (TPACKET_V3 == v) {
		rx_ring.req.tp_retire_blk_tov = 60;
		rx_ring.req.tp_feature_req_word = TP_FT_REQ_FILL_RXHASH;
	}
	ec = setsockopt(fd, SOL_PACKET, PACKET_RX_RING,
			&rx_ring.req, sizeof(rx_ring.req));
	if (ec < 0) {
		perror("setsockopt");
		goto rs_exit0;
	}

	rx_ring.map = mmap(NULL, rx_ring.req.tp_block_size * rx_ring.req.tp_block_nr,
		  PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
	if (rx_ring.map == MAP_FAILED) {
		perror("mmap");
		goto rs_exit0;
	}

	rx_ring.rd = malloc(rx_ring.req.tp_block_nr * sizeof(*rx_ring.rd));
	for (int i = 0; i < rx_ring.req.tp_block_nr; ++i) {
		rx_ring.rd[i].iov_base = rx_ring.map + (i * rx_ring.req.tp_block_size);
		rx_ring.rd[i].iov_len = rx_ring.req.tp_block_size;
	}
# endif
#endif

#if defined(BUILDING_SOCK_RXQUNFL)
	struct timeval tm;
	tm.tv_sec = 0;
	tm.tv_usec = 200;/* TODO: How does it affect the throughput/latency? */
	ec = setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tm, sizeof(tm));
	if (ec < 0) {
		perror("setsockopt() with SO_RCVTIMEO");
		goto rs_exit0;
	}
#endif
	return fd;

rs_exit1:
#if !defined(BUILDING_AFINETS_UDP)
# if defined(BUILDING_RAWS_MMAP)
	free(rx_ring.rd);
	munmap(rx_ring.map, rx_ring.req.tp_block_size * rx_ring.req.tp_block_nr);
# endif
#endif
rs_exit0:
	close(fd);
	return ec;
}

static void *raws_rx_main(void *parg)
{
	int ec;
#if defined(BUILDING_AFINETS_UDP)
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
	ec = setsockopt(fd_rxsock, SOL_SOCKET, SO_RXQ_OVFL,
			&dropmonitor, sizeof(dropmonitor));
	if (ec < 0) {
		perror("setsockopt() with SO_RXQ_OVFL");
		goto rx_exit;
	}
	/* these settings are static */
	iov.iov_base = (void *)&rawsr_cbuf[0][0];
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
# if defined(BUILDING_AFINETS_UDP)
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
		iov.iov_len = RAWSRX_BUF_SZ;
		msg.msg_controllen = sizeof(ctrlmsg); 
		msg.msg_flags = 0;
		ec = recvmsg(fd_rxsock, &msg, 0/*MSG_DONTWAIT*/);
#else
#  if defined(BUILDING_AFINETS_UDP)
		ec = recvfrom(fd_rxsock, &rawsr_cbuf[0][0], RAWSRX_BUF_SZ,
			0/*MSG_DONTWAIT*/, (struct sockaddr *)&iprx, &slen);
#  else
		ec = recvfrom(fd_rxsock, &rawsr_cbuf[0][0], RAWSRX_BUF_SZ,
			0/*MSG_DONTWAIT*/, (struct sockaddr *)&llrx, &slen);
#  endif
#endif
		if (ec < 0) {
			/* SIGINT(ctrl-c) CAN be detected while recvfrom() is blocking */
			//fprintf(stderr, "\nerrno=%d: ", errno);
			//perror("socket recvfrom()");
			if (EINTR == errno) {
				/* signals are detected while blocking recvfrom()
				the process received a signal while recvfrom() is running before any data was received.
				in our case, it means that SIGALRM have been asserted by ITIMER_REAL. */
				//usleep(SOCK_EINTR_US4TMPPS);/* give way to send timing package at first */
				if (taskstop_rawsr) goto rx_exit;
				else goto rx_redo;
#if defined(BUILDING_SOCK_RXQUNFL)
			} else if ((EAGAIN == errno) || (EWOULDBLOCK == errno)) {
				/* read timeout due to no more data arriving
				it also means that the sender could send faster */
				rawsr_udfw++;
				//usleep(1000);/* Wait longer */
    			goto rx_redo;
#endif
			} else {
				perror("socket recvfrom()");
				goto rx_exit;
			}
		}

		/* reset so that a non-zero value could mean continuous underflow */
		rawsr_udfw = 0;
#if defined(BUILDING_SOCK_RXQOVFL)
		for (pcmsg = CMSG_FIRSTHDR(&msg);
			pcmsg && (pcmsg->cmsg_level == SOL_SOCKET);
			pcmsg = CMSG_NXTHDR(&msg, pcmsg)) {
				if (pcmsg->cmsg_type == SO_RXQ_OVFL) {
					//memcpy(&dropcnt, CMSG_DATA(pcmsg), sizeof(__u32));
					rawsr_ovfw = *(__u32 *)CMSG_DATA(pcmsg);
				}
		}
#endif

		//display_buffer(&rawsr_cbuf[0][0], ec);

#if !defined(BUILDING_RAWS_WITH_BIND)
		ec = unpack_ethhdr(&rawsr_cbuf[0][0]);
		if (0 != ec) goto rx_redo;
#endif

#if defined(BUILDING_PFRAWS_UDP)
		unpack_iphdr_udphdr(&rawsr_cbuf[0][0]);
		if (0 != ec) goto rx_redo;
#endif

		uint16_t subtype;
		subtype = *(uint16_t *)(&rawsr_cbuf[0][0] + 
            PKG_OFFSET + offsetof(struct pdsch_hdr, sub_type));
		if (ntohs(PDSCH_SUBTYPE) == subtype) {
			unpack_pdsch(&rawsr_cbuf[0][0] + PKG_OFFSET);
#if 0
			if (0 == fill_libiio_txbuf(0)) {
				ec = datapath_handler_tx(0);
				if (-1 == ec) {
					iiopt_ovfw++;
					usleep(iiopt_ovfw*100);
				}
			}
#endif
		} else {
			if (mode_verbose > 1)
				printf("#### UNKNOWN FRAME 0x%04x ####\n", subtype);
		}
	} while (!taskstop_rawsr);

rx_exit:
	return (void *)ec;
}

static int raws_tx_startup(void)
{
	int ec, fd, tmp;
	struct ifreq req;

#if defined(BUILDING_AFINETS_UDP)
	fd = socket(AF_INET, SOCK_DGRAM, htons(IPPROTO_UDP));
#elif defined(BUILDING_PFRAWS_UDP)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_IP)); /* ETH+IP */
#elif defined(BUILDING_PFRAWS_VLAN)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_8021Q)); /* ETH */
#else //defined(BUILDING_PFRAWS_ETH)
	fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_802_3)); /* ETH */
#endif
	if (-1 == fd) {
		perror("socket()");
		return -1;
	}

	ec = sock_bind_ipport(fd, 1);
	if (ec < 0) { return -1; }

#if !defined(BUILDING_AFINETS_UDP)
# if 0
	/* ifdown <eth0> at first
	the EMAC hardware and driver should support such MTU length
	current kernel's ability, setting in devicetree */
	memset(&req, 0, sizeof(req));
	strncpy(req.ifr_name, str_devname, sizeof(req.ifr_name));
	req.ifr_mtu = 5000;
	ec = ioctl(fd, SIOCSIFMTU, &req);
	if (-1 == ec) {
		perror("socket ioctl() SIOCSIFMTU");
		goto ts_exit0;
	}
# endif
# if defined(BUILDING_RAWS_MMAP)
	/* TPACKET_V3 is valid for RX only */
	int v = TPACKET_V2;
	ec = setsockopt(fd, SOL_PACKET, PACKET_VERSION, &v, sizeof(v));
	if (ec < 0) {
		perror("setsockopt TPACKET_V2");
		goto ts_exit0;
	}

	struct tpring tx_ring;
	unsigned int bsize;
	memset(&tx_ring.req, 0, sizeof(tx_ring.req));
	tx_ring.req.tp_frame_size = 1024*8;
	tx_ring.req.tp_block_size = 1024*8;
	tx_ring.req.tp_block_nr = 1024;
	bsize = tx_ring.req.tp_block_size * tx_ring.req.tp_block_nr;
	tx_ring.req.tp_frame_nr = bsize / tx_ring.req.tp_frame_size;

	ec = setsockopt(fd, SOL_PACKET, PACKET_TX_RING,
			&tx_ring.req, sizeof(tx_ring.req));
	if (ec < 0) {
		perror("setsockopt");
		goto ts_exit0;
	}

	tx_ring.map = mmap(NULL, bsize,
			PROT_READ | PROT_WRITE, MAP_SHARED | MAP_LOCKED, fd, 0);
	if (tx_ring.map == MAP_FAILED) {
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

#  if defined(BUILDING_RAWS_TXPLOSS)
	tmp = 1;
	ec = setsockopt(fd_socket, SOL_PACKET, PACKET_LOSS,
			(char *)&tmp, sizeof(tmp));
	if (ec < 0) {
		perror("setsockopt: PACKET_LOSS");
		goto ts_exit1;
	}
#  endif
#  if defined(BUILDING_RAWS_TXBYPASSQC)
	tmp = 1;
	ec = setsockopt(fd, SOL_PACKET, PACKET_QDISC_BYPASS,
			(char *)&tmp, sizeof(tmp));
	if (ec < 0) {
		perror("setsockopt: QDISC_BYPASS");
		goto ts_exit1;
	}
#  endif
# endif
#endif
	return fd;

ts_exit1:
#if !defined(BUILDING_AFINETS_UDP)
# if defined(BUILDING_RAWS_MMAP)
	//free(tx_ring.rd);
	munmap(tx_ring.map, tx_ring.req.tp_block_size * tx_ring.req.tp_block_nr);
# endif
#endif
ts_exit0:
	close(fd);
	return 1;
}

int raws_tx_prepare(void)
{
	char *p;
	int ec, i;

# if !defined(BUILDING_AFINETS_UDP)
#  if !defined(BUILDING_RAWS_WITH_BIND)
    ec = pack_ethhdr(fd_txsock, buf_tmpps);
    if (ec < 0) return -1;
#  endif
# endif
# if defined(BUILDING_PFRAWS_UDP)
    ec = pack_iphdr_udphdr(fd_txsock, buf_tmpps + ETH_HDR_LEN, TMPPS_SIZE);
    if (ec < 0) return -1;
# endif

	p = &rawst_cbuf[0][0];
	for (i=0; i<RAWST_BUF_NB_NOW; i++) {
#if !defined(BUILDING_AFINETS_UDP)
# if !defined(BUILDING_RAWS_WITH_BIND)
		ec = pack_ethhdr(fd_txsock, p);
		if (ec < 0) return ec;
# endif
#endif
#if defined(BUILDING_PFRAWS_UDP)
# if defined(BUILDING_RRU_UL)
		ec = pack_iphdr_udphdr(fd_txsock, p + ETH_HDR_LEN, PUSCH_SIZE);
# endif
# if defined(BUILDING_BBU_DL)
		ec = pack_iphdr_udphdr(fd_txsock, p + ETH_HDR_LEN, PDSCH_SIZE);
# endif
		if (0 != ec) return ec;
#endif
		p += sizeof(*p);
	}

    return 0;
}

/*---- leaf function ----*/
static int raws_tx_one(int fd, uint8_t *buf, size_t tlen)
{
	int ec;
	size_t lsent;

	assert(fd != -1);
	assert(buf != NULL);
#if defined(BUILDING_RRU_UL)
	assert(tlen <= RAWSTX_BUF_SZ);
#endif
#if defined(BUILDING_BBU_DL)
	assert(tlen <= PDSCH_UBUF_LEN);
#endif
	lsent = 0;
tx_redo:
	//display_tx_counters();
#if defined(BUILDING_AFINETS_UDP)
	ec = sendto(fd, buf + lsent, tlen - lsent,
			0, (struct sockaddr *)&g_txaddrip, sizeof(g_txaddrip));
#else
	ec = sendto(fd, buf + lsent, tlen - lsent,
			0, (struct sockaddr *)&g_txaddrll, sizeof(g_txaddrll));
#endif
	if (ec < 0) {
		/* SIGINT(ctrl-c) cannot be detected while blocking sendto() */
		//fprintf(stderr, "\nerrno=%d: ", errno);
		//perror("socket sendto() PUSCH");
		if (EINTR == errno) {
			/* signals are detected while executing a blocking system call
			the process received a signal while sendto() is running before any data was transmitted.
			in our case, it means that SIGALRM have been asserted by ITIMER_REAL.*/
			//usleep(SOCK_EINTR_US4TMPPS);/* give way to send timing package at first */
			if (taskstop_rawst) return ec;
			else goto tx_redo;
		} else if (ENOBUFS == errno) {
			/* ENOBUFS: The output queue was full (transient congestion). 
			(Normally, this doesn't occur in Linux as packets are just silently dropped
			when a device queue overflows. In other words, Linux device driver doesn't report overflow) */
			rawst_ovfw++;/* continuous overflow */
			//sleep(rawst_ovfw);/* long sleep */
			goto tx_redo;
		} else if (EMSGSIZE == errno) {
			return ec;
		} else {
			fprintf(stderr, "\nerrno=%d: ", errno);
			perror("socket sendto()");
			return ec;
		}
	}

	lsent += ec;
	if (lsent < tlen) goto tx_redo;

	/* reset so that a non-zero value could mean continuous overflow */
	rawst_ovfw = 0;
	return 0;
}

// TODO:
static void *raws_tx_main(void *parg)
{
	int ec, i;
#if defined(BUILDING_RRU_UL)
	ssize_t iior_byte_nb = 0;
	char *iior_byte_pt;
#endif
    char *ethw_byte_pt;

	do {
#if defined(BUILDING_RRU_UL)
        pack_tmpps(buf_tmpps + PKG_OFFSET);
        pack_iphdr_udphdr(fd_txsock, buf_tmpps + ETH_HDR_LEN, TMPPS_SIZE);
# if 1
		if (0 == iior_byte_nb) {
			iior_byte_nb = iio_buffer_refill(iiopr_cbuf[0]);
			if (iior_byte_nb < 0) {
				fprintf(stderr, "iio_buffer_refill() %s\n", strerror(-iior_byte_nb));
				goto tx_exit;
			}
			iior_byte_pt = iio_buffer_first(iiopr_cbuf[0], iiopr_chni[0]);
        }
        ethw_byte_pt = &rawst_cbuf[0][0];
        for (i=0; i<RAWST_BUF_NB_NOW; i++) {
    		memcpy(ethw_byte_pt + PUSCH_PLDOF, iior_byte_pt, RADIO_SYM2SMP*2);
            ethw_byte_pt += RADIO_SYM2SMP*2;
#  if defined(FPGA_COMPRESS_SAMPLES)
            iior_byte_pt += RADIO_SYM2SMP*2;
            iior_byte_nb -= RADIO_SYM2SMP*2;
#  else
            iior_byte_pt += RADIO_SYM2SMP*4;
            iior_byte_nb -= RADIO_SYM2SMP*4;
#  endif
            pack_pusch_hdr(ethw_byte_pt + PKG_OFFSET);
        }
# else
        ethw_byte_pt = &rawst_cbuf[0][0];
        for (i=0; i<RAWST_BUF_NB_MIN; i++) {
		    fill_pusch_pld(ethw_byte_pt + PUSCH_PLDOF);
            pack_pusch_hdr(ethw_byte_pt + PKG_OFFSET);
        }
# endif
        sem_wait(&g_sem_pps);
        ec = raws_tx_one(fd_txsock, buf_tmpps, TMPPS_UBUF_LEN);
        if (ec < 0) goto tx_exit;
		ec = raws_tx_one(fd_txsock, &rawst_cbuf[0][0], RAWSTX_BUF_SZ*RAWST_BUF_NB_NOW);
		if (ec < 0) goto tx_exit;
#endif
#if defined(BUILDING_BBU_DL)
		fill_pdsch_pld(rawst_cbuf + PDSCH_PLDOF);
		pack_pdsch_hdr(rawst_cbuf + PKG_OFFSET);
		ec = raws_tx_one(fd_txsock, &rawst_cbuf[0][0], PDSCH_UBUF_LEN);
		if (ec < 0) goto tx_exit;
#endif
	} while (!taskstop_rawst);

tx_exit:
	return (void *)ec;
}

void tmpps_handler(int signo)
{
	int ec;

	/*if (SIGALRM != signo) {
		fprintf(stderr, "\n%s Unknown signal %d\n", __func__, signo);
		goto tmpps_exit;
	}*/
    measure_interval(TMPPS_INTERVAL_US, "tmpps_handler");
    //if (mode_verbose > 2) { printf("."); fflush(stdout); }

#if defined(BUILDING_TMPPS_SOLO)
    pack_tmpps(buf_tmpps + PKG_OFFSET);
    raws_tx_one(fd_txsock, buf_tmpps, TMPPS_UBUF_LEN);
#else
    if (pusch_tx_cnt < RADIO_UL_SYM_NB) return;
	ec = sem_post(&g_sem_pps);
    if (ec < 0) { perror("sem_post()"); return; }
#endif
}

#if defined(TMPPS_TRG_UTIMER)
void tmpps_tmr_init(void)
{
	struct sigaction tact;
	struct itimerval ttv;

	tact.sa_handler = tmpps_handler;
	tact.sa_flags = 0;
	sigemptyset(&tact.sa_mask);
	sigaction(SIGALRM, &tact, NULL);

	ttv.it_value.tv_sec = 0;
	ttv.it_value.tv_usec = 1000;
	ttv.it_interval.tv_sec = 0;
	ttv.it_interval.tv_usec = TMPPS_INTERVAL_US; /* 1ms interval */
	setitimer(ITIMER_REAL, &ttv, NULL);
}

void tmpps_tmr_exit(void)
{
	struct itimerval ttv;

	ttv.it_value.tv_sec = 0;
	ttv.it_value.tv_usec = 0;
	ttv.it_interval.tv_sec = 0;
	ttv.it_interval.tv_usec = 0;
	setitimer(ITIMER_REAL, &ttv, NULL);
}
#endif

/*	Input/Output Multiplexing
1, select:
----
	fd_set readmask, writemask, exceptmask;
	struct timeval timeout;
	...
	select(nfds, &readmask, &writemask, &exceptmask, &timeout);
	If the timeout pointer is NULL, select() blocks until a descriptor is selectable, or until a signal is received.
	If the fields in timeout are set to 0, select() polls and returns immediately. 
*/
int do_select(int fd_gpio, int fd_tx, int fd_rx)
{
	int ec;
	fd_set rset, wset;
	int maxfd;
	struct timeval tv;
    char buf[64];

	maxfd = max(fd_rx, fd_tx);

	tv.tv_sec = 0;
	tv.tv_usec = 4000;//4ms per timing-package requirement

	do { // TODO: debug
		FD_ZERO(&rset);
		FD_SET(fd_gpio, &rset);
		FD_SET(fd_rx, &rset);
		FD_ZERO(&wset);
		FD_SET(fd_tx, &wset);

		ec = select(maxfd+1, &rset, &wset, NULL, &tv);
		if (ec < 0) {
			perror("select");
			break;
		}
		if (ec == 0) {/* timeout */
			printf(".");
			continue;
		}

		if (FD_ISSET(fd_gpio, &rset)) {
			lseek(fd_gpio, 0, SEEK_SET);
			read(fd_gpio, buf, sizeof(buf));
            tmpps_handler(0);
		}
		if (FD_ISSET(fd_tx, &wset)) {
			if (*(int *)raws_tx_main(&fd_tx) < 0) break;
		}
		if (FD_ISSET(fd_rx, &rset)) {
			if (*(int *)raws_rx_main(&fd_rx) < 0) break;
		}
	} while (!procstop);
}

/*	Input/Output Multiplexing
2, poll:
----
*/
int do_poll(int fd_gpio, int fd_tx, int fd_rx)
{
	int ec;
	struct pollfd fdset[3];
	int nfds = sizeof(fdset)/sizeof(fdset[0]);
	int timeout = 3;
    char buf[64];

	do {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = fd_gpio;
		fdset[0].events = POLLPRI;
		fdset[1].fd = fd_tx;
		fdset[1].events = POLLOUT;
		fdset[2].fd = fd_rx;
		fdset[2].events = POLLIN;

		ec = poll(fdset, nfds, timeout);
		if (ec < 0) {
			perror("poll()");
			//assert((errno == EINVAL) && (errno == EBADF));
			if (errno == ENOMEM) return -1;
			else if (errno == EINTR) return -1;
		} else if (ec == 0) {/* timeout */
			printf(".");
			continue;
		}

		if (fdset[0].revents & POLLPRI) {
			lseek(fdset[0].fd, 0, SEEK_SET);
			read(fdset[0].fd, buf, sizeof(buf));
            tmpps_handler(0);
		}
		if (fdset[1].revents & POLLOUT) {
			if (*(int *)raws_tx_main(&fd_tx) < 0) break;
		}
		if (fdset[1].revents & POLLIN) {
			if (*(int *)raws_rx_main(&fd_rx) < 0) break;
		}
	} while (!procstop);
}


/*	Input/Output Multiplexing
3, epoll:
----
*/
static inline void add_event(int epollfd, int fd, int state)
{
    struct epoll_event ev;
    ev.events = state;
    ev.data.fd = fd;
    epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev);
}
static inline void del_event(int epollfd, int fd, int state)
{
    struct epoll_event ev;
    ev.events = state;
    ev.data.fd = fd;
    epoll_ctl(epollfd, EPOLL_CTL_DEL, fd, &ev);
}
static inline void mod_event(int epollfd, int fd, int state)
{
    struct epoll_event ev;
    ev.events = state;
    ev.data.fd = fd;
    epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &ev);
}

void do_epoll(int fd_gpio, int fd_tx, int fd_rx)
{
    int i, fd_epoll, ec_epoll;
    struct epoll_event events[3];
    char buf[64];

    if ((fd_gpio < 0) || (fd_tx < 0) || (fd_rx < 0))
        return;

    fd_epoll = epoll_create(3);
	if (fd_epoll < 0) {
		perror("epoll_create");
		return;
	}

    add_event(fd_epoll, fd_gpio, EPOLLPRI);
    add_event(fd_epoll, fd_tx,   EPOLLOUT);
    add_event(fd_epoll, fd_rx,   EPOLLIN);

    do {
        ec_epoll = epoll_wait(fd_epoll, events, 3, -1);
		if (ec_epoll < 0) {
			perror("epoll_wait");
			return;
		}

        for (i = 0; i < ec_epoll; i++) {
            if ((events[i].data.fd == fd_gpio) && (events[i].events & EPOLLPRI)) {
                lseek(fd_gpio, 0, SEEK_SET);
                read(fd_gpio, buf, sizeof(buf));
	            tmpps_handler(0);
            }
			if ((events[i].data.fd == fd_tx) && (events[i].events & EPOLLOUT)) { 	
                printf("TX EPOLLOUT is up!\n");
                if (*(int *)raws_tx_main(&fd_tx) < 0) break;
            }
			if ((events[i].data.fd == fd_rx) && (events[i].events & EPOLLIN)) {
                printf("RX EPOLLIN is up!\n");
                if (*(int *)raws_rx_main(&fd_rx) < 0) break;
            }
        }
    }while (!procstop);
    close(fd_epoll);
}

int do_functions(void)
{
	int ec;
	pthread_t pid_tmpps;

#if defined(BUILDING_RRU_UL)
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_init(); /* start timer to for tmpps */
# endif
# if defined(TMPPS_TRG_UGPIOIRQ)
    ec = pthread_create(&pid_tmpps, NULL, task_tmpps_irq, NULL);
    if (ec != 0) {
        perror("pthread_create() for raws_rx_main\n");
        return -1;
    }
# endif
# if defined(TMPPS_TRG_KSIGIO_NOWAIT)
    /* create one thread to fetch pps signal from the kernel because
       send() without psock_mmap takes long time */
    ec = pthread_create(&pid_tmpps, NULL, task_tmpps_pps, NULL);
    if (ec != 0) {
        perror("pthread_create() for raws_rx_main\n");
        return -1;
    }
# endif
#endif

	taskstop_rawst = 1;
	taskstop_rawsr = 1;
	do {
#if defined(BUILDING_RRU_UL)
		raws_tx_main(NULL);
#endif
#if defined(BUILDING_RRU_DL)
		raws_rx_main(NULL);
#endif
		//display_counters();
	} while (!procstop);

#if defined(BUILDING_RRU_UL)
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_exit();
# endif
# if defined(TMPPS_TRG_UGPIOIRQ) || defined(TMPPS_TRG_KSIGIO_NOWAIT)
    pthread_exit(&pid_tmpps);
# endif
#endif
	return 0;
}

int do_threads(int fd_tx, int fd_rx)
{
	int ec;
	pthread_t pid_ethrx, pid_ethtx, pid_tmpps;

#if defined(BUILDING_RRU_DL)
    taskstop_rawsr = 0;
    ec = pthread_create(&pid_ethrx, NULL, raws_rx_main, NULL);
    if (ec != 0) {
        perror("pthread_create() for raws_rx_main\n");
        return -1;
    }
#endif
#if defined(BUILDING_RRU_UL)
# if !defined(BUILDING_TMPPS_SOLO)
    taskstop_rawst = 0;
    ec = pthread_create(&pid_ethtx, NULL, raws_tx_main, NULL);
    if (ec != 0) {
        perror("pthread_create() for raws_tx_main\n");
        return -1;
    }
# endif
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_init();
# endif
# if defined(TMPPS_TRG_UGPIOIRQ)
	ec = pthread_create(&pid_tmpps, NULL, task_tmpps_irq, NULL);
	if (ec != 0) {
		perror("pthread_create() for raws_rx_main\n");
		return -1;
	}
# endif
# if defined(TMPPS_TRG_KSIGIO_NOWAIT)
    ec = pps_find_src("/dev/pps0", &pps_handle, &pps_mode);
    if (ec < 0)
        exit(EXIT_FAILURE);
# endif
#endif

    do {
#if defined(BUILDING_RRU_UL)
# if defined(TMPPS_TRG_KSIGIO_NOWAIT) && defined(BUILDING_TMPPS_SOLO)
		ec = pps_fetch_src(0, &pps_handle, &pps_mode);
		if (ec < 0 && errno != ETIMEDOUT)
			exit(EXIT_FAILURE);
        tmpps_handler(0);
# endif
#endif
    } while (!procstop);

#if defined(BUILDING_RRU_UL)
# if defined(TMPPS_TRG_UTIMER)
    tmpps_tmr_exit();
# endif
# if defined(TMPPS_TRG_UGPIOIRQ)
    pthread_exit(&pid_tmpps);
# endif
# if defined(TMPPS_TRG_KSIGIO_NOWAIT)
    time_pps_destroy(pps_handle);
# endif
# if !defined(BUILDING_TMPPS_SOLO)
    pthread_exit(&pid_ethtx);
# endif
#endif
#if defined(BUILDING_RRU_DL)
	pthread_exit(&pid_ethrx);
#endif
	return 0;
}

#if defined(BUILDING_TMPPS_SOLO)
int main(int argc, char *argv[])
{
	int ec;

	common_getargs(argc, argv);

	clear_counters();
#if defined(BUILDING_RRU_UL) || defined(BUILDING_BBU_DL)
    iio_path_x = 0x03;
# if defined(FPGA_COMPRESS_SAMPLES)
    libiio_app_startup(IIOPR_SMP_NB_NOW/2, IIOPT_SMP_NB_NOW/2);
# else
    libiio_app_startup(IIOPR_SMP_NB_NOW, IIOPT_SMP_NB_NOW);
# endif
#endif

#if defined(BUILDING_RRU_DL)
	fd_rxsock = raws_rx_startup();
	if (fd_rxsock < 0) goto exit0;
#endif

#if defined(BUILDING_RRU_UL)
    fd_txsock = raws_tx_startup();
	if (fd_txsock < 0) goto exit1;

    ec = sock_fill_txaddr();
    if (ec < 0) goto exit2;

    ec = raws_tx_prepare();
    if (ec < 0) goto exit2;

# if !defined(BUILDING_TMPPS_SOLO)
    ec = sem_init(&g_sem_pps, 0, 0);
    if (ec == -1) {
        perror("sem_init(&g_sem_pps)");
        goto exit3;
    }
# endif
#endif

	/* Listen to ctrl+c and assert */
	signal(SIGINT, handle_sig);

    ec = do_threads(fd_txsock, fd_rxsock);

	/*---- performance */
	clock_gettime(CLOCK_MONOTONIC, &tm_te);
	display_tx_perf(&tm_te, &tm_ts);
	display_rx_perf(&tm_te, &tm_rs);

#if defined(BUILDING_RRU_UL)
# if !defined(BUILDING_TMPPS_SOLO)
exit3:
    sem_destroy(&g_sem_pps);
# endif
exit2:
	if (fd_txsock > 0) { close(fd_txsock); fd_txsock = 0; }
#endif
exit1:
#if defined(BUILDING_RRU_DL)
	if (fd_rxsock > 0) { close(fd_rxsock); fd_rxsock = 0; }
#endif
exit0:
#if defined(BUILDING_RRU_UL) || defined(BUILDING_BBU_DL)
	libiio_app_shutdown();
#endif
	return ec;
}
#endif
