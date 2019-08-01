#ifndef _RRU_BBU_H
#define _RRU_BBU_H

#include <stdbool.h>
#include <stdint.h>

/*==================================
	for RRU only
*/

#define CHIP_NUM_TOTAL 1	/* total chip number: TODO: 4 */
#define PORT_PER_DEVICE 1	/* each chip has 2-RX & 2-TX ports: TODO: 2 (then FIXME profile_ad9371_phy()) */
#define CHANNEL_PER_PORT 2	/* each RX/TX port has one pair of I/Q channel */
#define PORT_NUM_TOTAL (CHIP_NUM_TOTAL*PORT_PER_DEVICE)

/*
iio-internal-buffer is 4MiB at most so 
	  1 MiSample when 1-port(i.e. 2-channel)
	512 KiSample when 2-port(i.e. 4-channel)
*/
#define LIBIIO_PORT_MAX_SAMPLES (1024*1024/PORT_PER_DEVICE)
#if (LIBIIO_PORT_MAX_SAMPLES*PORT_PER_DEVICE > 1024*1024)
#error "libiio internal buffer limit to 4MiB"
#endif

/* for 20/15/10/5MHz subcarrier, samples are 1200/900/600/300 */
#define SUBCARRIER_SAMPLES 1200

#if defined(BUILDING_ORGINAL_ADI)
#define IIO_PORT_SAMPLES LIBIIO_PORT_MAX_SAMPLES
#elif defined(BUILDING_SIMILAR_ADI)
#define IIO_PORT_SAMPLES LIBIIO_PORT_MAX_SAMPLES
#else //if defined(BUILDING_RRU_DL) || defined(BUILDING_RRU_UL)
#define IIO_PORT_SAMPLES ((LIBIIO_PORT_MAX_SAMPLES/SUBCARRIER_SAMPLES)*SUBCARRIER_SAMPLES)
#endif
#define SUBCARRIER_NTXBUF_NB (IIO_PORT_SAMPLES/SUBCARRIER_SAMPLES)
#define SUBCARRIER_NRXBUF_NB (IIO_PORT_SAMPLES/SUBCARRIER_SAMPLES)

/*
	CHIP: ADI ADC(AD9371):	 				one sample
	FPGA: ADI AXI_AD9371:
	FPGA: ADI_JESD204B:					16bit-I + 16bit-Q
	FPGA_USER_FFT:
	FPGA_USER_COMPRESS:			compressed(8bit-zero_8bit-I + 8bit-zero_8bit-Q)
	FPGA_ADI_CPACK(UTIL_CPACK):			parallel N x 32-bit FIFO
	FPGA_ADI_DMA(AXI_DMAC):			interleaved 1 x 128-bit FIFO

https://wiki.analog.com/resources/fpga/docs/util_cpack
Four channels enabled (4'b1111)
offset:               0;                       2;                       4;                       6;
---------------------------------------------------------------------------------------------------
channel0_sample0/p0i.s0; channel1_sample0/p0q.s0; channel2_sample0/p1i.s0; channel0_sample0/p1q.s0;
channel0_sample1/p0i.s1; channel1_sample0/p0q.s1; channel2_sample0/p1i.s1; channel0_sample0/p1q.s1;
channel0_sample2/p0i.s1; channel1_sample0/p0q.s2; channel2_sample0/p1i.s2; channel0_sample0/p1q.s2;
......
Consult:
	item: array[isample][ichannel] -> I_and_Q[isample][iport]
	step=8 for each channel
*/
struct iio_sample {
	int16_t i16;/* LSB after CPACK */
	int16_t q16;/* MSB after CPACK */
} __attribute__((packed));

struct iio_iqsmap {
	struct iio_sample iqs[IIO_PORT_SAMPLES][PORT_NUM_TOTAL];
} __attribute__((packed));

/*==================================
	for both RRU and BBU
*/

//((packed, aligned(1)))

/* Timing Packet Header */
#define TMPPS_SUBTYPE 0x03
struct tmpps_hdr {
	uint8_t zero[18];
    uint16_t sub_type;			/* TIME: 0003 */

#ifdef ENDIAN_MODE_BE
	uint32_t rsvd1:8;			/* [31:24]	NA 	Reserved */
	uint32_t dl_bof_cnt:4;		/* [23:20]	NA	DL symbol buffer overflow count */
	uint32_t ul_bof_flg:1;		/* [19]	0-1	1 to indicate UL buffer overflow occured */
	uint32_t dl_bof_flg:1;		/* [18]	0-1	1 to indicate DL buffer overflow occured */
	uint32_t dl_buf_flg:1;		/* [17]	0-1	1 to indicate DL buffer underflow occured */
	uint32_t symbol_sync:1;		/* [16]	0-1	1 to indicate the BBU to RRU symbol packets are in sync */
	uint32_t frame_seq:12;		/* [15:4] 0-1023	Radio frame number */
	uint32_t subf_seq:4;		/* [3:0]	0-9	Subframe number */
#else
	uint32_t subf_seq:4;		/* [3:0]	0-9	Subframe number */
	uint32_t frame_seq:12;		/* [15:4]	0-1023	Radio frame number */
	uint32_t symbol_sync:1;		/* [16]	0-1	1 to indicate the BBU to RRU symbol packets are in sync */
	uint32_t dl_buf_flg:1;		/* [17]	0-1	1 to indicate DL buffer underflow occured */
	uint32_t dl_bof_flg:1;		/* [18]	0-1	1 to indicate DL buffer overflow occured */
	uint32_t ul_bof_flg:1;		/* [19]	0-1	1 to indicate UL buffer overflow occured */
	uint32_t dl_bof_cnt:4;		/* [23:20]	NA	DL symbol buffer overflow count */
	uint32_t rsvd1:8;			/* [31:24]	NA 	Reserved */
#endif
} __attribute__((packed));
#define TMPPS_SIZE	(sizeof(struct tmpps_hdr))	/* 24 */

/* Payload,
one sample occupy 16bit (8bitI+8bitQ), represent RE for all samples of Ant 0/1/...
N transfers/words for Ant 0: for n = [0 to N-1] (N = SUBCARRIER_SAMPLES/2)
[31:24] NA	8-bit compressed I component of sample n*2
[23:16] NA	8-bit compressed Q component of sample n*2
[15:8]	NA	8-bit compressed I component of sample n*2+1
[7:0]	NA	8-bit compressed Q component of sample n*2+1
*/
//#define BBU_RRU_MTU ((1200*2*2)+PUSCH_PLDOF) /* 4890-PUSCH/4880-PDSCH */
#define EMAC_MAX_MTU 3800 /* per arria10soc_fpga's emac */
//#define EMAC_MAX_SAMPLES (EMAC_MAX_MTU-PUSCH_PLDOF)/(2*PORT_PER_DEVICE) /* (3800-90)/2*2=927 */
#if EMAC_MAX_MTU < 4880
//#warnning "don't forget to change MTU of the ethernet interface"
#endif

struct bbu_sample {
#if 1 /* FIXME: trick for test only */
	int8_t i8;	/* LSB */
	int8_t q8;	/* MSB */
#else
	int16_t i8;	/* LSB */
	int16_t q8;	/* MSB */
#endif
} __attribute__((packed));

struct bbu_payload {
	struct bbu_sample iqs[PORT_NUM_TOTAL][SUBCARRIER_SAMPLES];
} __attribute__((packed));

/* PUSCH Packet Header */
struct gain_t {
	int16_t mantissa: 9; 
	int16_t reserved: 1;
	int16_t exponent: 6;
} __attribute__((packed));

#define PUSCH_SUBTYPE 0x0b
struct pusch_hdr {
	uint8_t zero[18];
    uint16_t sub_type;		/* PUSCH: 000b */

	uint32_t rsvd1;			/* [31:0] Reserved */

#ifdef ENDIAN_MODE_BE
	uint32_t rsvd2d1:22;	/* [31:10] Reserved */
	uint32_t special:1; 	/* [9] 0-1	1 to indicate it is special subframe */
	uint32_t rsvd2d2:1; 	/* [8] Reserved */
	uint32_t rb_seq:7;		/* [7:1] 25,50,75,100 RB Number, system bandwidth dependent */
	uint32_t rsvd2d3:1; 	/* [0] Reserved */
#else
	uint32_t rsvd2d3:1; 	/* [0] Reserved */
	uint32_t rb_seq:7;		/* [7:1] 25,50,75,100 RB Number, system bandwidth dependent */
	uint32_t rsvd2d1:1; 	/* [8] Reserved */
	uint32_t special:1; 	/* [9] 0-1	1 to indicate it is special subframe */
	uint32_t rsvd2d2:22;	/* [31:10] Reserved */
#endif

#ifdef ENDIAN_MODE_BE
    uint32_t rsvd3:2;		/* [31:30]Reserved */
    uint32_t ant_cnt:3;		/* [29:27] 2 Antenna number */
    uint32_t ant_start:3;	/* [26:24] 0 Start antenna index */
    uint32_t frame_seq:16;	/* [23:8] Radio frame number */
    uint32_t subf_seq:4;	/* [7:4] 0-9 Subframe number */
    uint32_t sym_seq:4;		/* [3:0] 0-13 Symbol number in subframe */
#else
    uint32_t sym_seq:4;		/* [3:0] 0-13 Symbol number in subframe */
    uint32_t subf_seq:4;	/* [7:4] 0-9 Subframe number */
    uint32_t frame_seq:16;	/* [23:8] Radio frame number */
    uint32_t ant_start:3;	/* [26:24] 0 Start antenna index */
    uint32_t ant_cnt:3;		/* [29:27] 2 Antenna number */
    uint32_t rsvd3:2;		/* [31:30]Reserved */
#endif
	/* 4th to 7th */
	struct gain_t gain[8];	/* NA Gain-factor for signal from ant 0/1; 2/3; 4/5; 6/7 */
} __attribute__((packed));
#define PUSCH_SIZE	(sizeof(struct pusch_hdr) + sizeof(struct bbu_payload)) /* 48+ */

/* PDSCH Packt header*/
#define PDSCH_SUBTYPE 0x00
struct pdsch_hdr {
	uint8_t flag[6];	/* "ASTRI." 41 53 54 52 49 A0 */

	uint8_t zero[18];	/* dmac[6], smac[6], 0800, vlanTCI(e001), 8100 */
    uint16_t sub_type;		/* PDSCH: 0000 */

	uint32_t rsvd1;			/* [31:0] Reserved */

#ifdef ENDIAN_MODE_BE
	uint32_t rsvd2:25;		/* [31:7] Reserved */
	uint32_t special:1;		/* [6] 0-1 1 to indicate it is special subframe */
	uint32_t rb_sel:1;		/* [5] RB selection table is always disabled */
	uint32_t rb_seq_h:5;	/* [4:0] High part of number of RB (25,50,75,100) */
#else
	uint32_t rb_seq_h:5;	/* [4:0] High part of number of RB (25,50,75,100) */
	uint32_t rb_sel:1;		/* [5] RB selection table is always disabled */
	uint32_t special:1;		/* [6] 0-1 1 to indicate it is special subframe */
	uint32_t rsvd2:25;		/* /[31:7] Reserved */
#endif

#ifdef ENDIAN_MODE_BE
    uint32_t rb_seq_l:2;	/* [31:30] Reserved */
    uint32_t ant_cnt:3;		/* [29:27] 2 Antenna number */
    uint32_t ant_start:3;	/* [26:24] 0 Start antenna index */
    uint32_t frame_seq:16;	/* [23:8] Radio frame number */
    uint32_t subf_seq:4;	/* [7:4] 0-9 Subframe number */
    uint32_t sym_seq:4;		/* [3:0] 0-13 Symbol number in subframe */
#else
    uint32_t sym_seq:4;		/* [3:0] 0-13 Symbol number in subframe */
    uint32_t subf_seq:4;	/* [7:4] 0-9 Subframe number */
    uint32_t frame_seq:16;	/* [23:8] Radio frame number */
    uint32_t ant_start:3;	/* [26:24] 0 Start antenna index */
    uint32_t ant_cnt:3;		/* [29:27] 2 Antenna number */
    uint32_t rb_seq_l:2;	/* [31:30] Reserved */
#endif
} __attribute__((packed));
#define PDSCH_SIZE	(sizeof(struct pdsch_hdr) + sizeof(struct bbu_payload)) // 38+ */

struct bscsr {
	uint8_t flag[6];	/* "ASTRI." 41 53 54 52 49 A0 */

	uint8_t zero[18];	/* dmac[6], smac[6], 0800, vlanTCI(4001), 8100 */
    uint16_t sub_type;	/* PDSCH: 0008 */

	uint32_t rsvd1;

#ifdef ENDIAN_MODE_BE
	uint32_t rw:1;
	uint32_t tag:3;
	uint32_t be:4;
	uint32_t devsel:4;
	uint32_t csr_addr:20;
#else
	uint32_t csr_addr:20;	/* [19:0] */
	uint32_t devsel:4;		/* [23:20] */
	uint32_t be:4;			/* [27:24] */
	uint32_t tag:3;			/* [30:28] */
	uint32_t rw:1;	/* [31] assume 0 as read; 1 as write */
#endif

	uint32_t csr_data;
} __attribute__((packed));
#define BSCSR_SIZE	(sizeof(struct bscsr))

#if 1
#include <linux/if_ether.h>
#else
/*
1> promisc mode could receive any DMAC
2> Ethernet Protocols */
#define ETH_P_ALL	0x0003			/* all defined and undefined L2 protocol */
#define ETH_P_802_3 0x0001 			/* Dummy type for 802.3 frames  */ 
#define ETH_P_LOCALTALK 0x0009      /* Localtalk pseudo type    */ 
#define ETH_P_IP	0x0800
#define ETH_P_ARP	0x0806
#define ETH_P_RARP	0x8035
#define ETH_P_8021Q 	0x8100
#define ETH_P_8021AD	0x0x88A8	/* 802.1ad Service VLAN 	*/
#define ETH_P_8021AH	0x88E7		/* 802.1ah Backbone Service Tag */
#define ETH_P_QINQ1 	0x9100		/* deprecated QinQ VLAN [ NOT AN OFFICIALLY REGISTERED ID ] */
#define ETH_P_QINQ2 	0x9200		/* deprecated QinQ VLAN [ NOT AN OFFICIALLY REGISTERED ID ] */
#define ETH_P_QINQ3 	0x9300		/* deprecated QinQ VLAN [ NOT AN OFFICIALLY REGISTERED ID ] */
#define ETH_P_EDSA		0xDADA		/* Ethertype DSA [ NOT AN OFFICIALLY REGISTERED ID ] */
#define ETH_P_AF_IUCV	0xFBFB		/* IBM af_iucv [ NOT AN OFFICIALLY REGISTERED ID ] */

struct ethhdr {                                                                  
    unsigned char   h_dest[ETH_ALEN];   /* destination eth addr */               
    unsigned char   h_source[ETH_ALEN]; /* source ether addr    */               
    __be16      h_proto;        /* packet type ID field */                       
} __attribute__((packed));
#endif
#if 0
#include <net/ethernet.h>
struct ether_addr                                                                
{                                                                                
	u_int8_t ether_addr_octet[ETH_ALEN];                                           
} __attribute__ ((__packed__));

/* | 6 byte	| 6 byte | 2 byte | 46~1500 byte | */
struct ethhdr
{
	u_int8_t  ether_dhost[ETH_ALEN];	/* destination eth addr    */
	u_int8_t  ether_shost[ETH_ALEN];	/* source ether addr	*/
	u_int16_t ether_type;				/* packet type ID field    */
} __attribute__ ((__packed__));
#endif
#if 0
#include <netinet/ether.h>
//char *ether_ntoa (const struct ether_addr *__addr);
//struct ether_addr *ether_aton (const char *__asc);
#endif

#if 1
#include <linux/if_packet.h>
#else
struct sockaddr_ll {
	unsigned short int sll_family;	/* Always AF_PACKET */
	unsigned short int sll_protocol;/* Physical-layer protocol, <linux/if_ether.h> */
	int            sll_ifindex;		/* Interface number */
	unsigned short int sll_hatype;	/* ARP hardware type, <linux/if_arp.h>, set by socket RX */
	unsigned char  sll_pkttype;		/* Packet type, set by socket RX */
	unsigned char  sll_halen;		/* Length of address */
	unsigned char  sll_addr[8];		/* Physical-layer address */
};
#endif

/*
struct tpacket_req establishes a circular buffer (ring) of unswappable memory.
Being mapped in the capture process allows reading the captured frames and 
related meta-info like timestamps without requiring a system call.

Frames are grouped in blocks. Each block is a physically contiguous
region of memory and holds tp_block_size/tp_frame_size frames.
  tp_frame_nr == tp_block_size/tp_frame_size*tp_block_nr
# memory regions are allocated with calls to the __get_free_pages().
# The maximum size of a region (tp_block_size) = PAGE_SIZE << MAX_ORDER
  In a i386 architecture PAGE_SIZE is 4096 bytes. 
  PAGE_SIZE can also be determined dynamically with getpagesize() system call.
  In a 2.4/i386 kernel MAX_ORDER is 10; In a 2.6/i386 kernel MAX_ORDER is 11.
  MAX_ORDER is arch specific and defined in <sys/user.h>.
# The maximum number of allocable regions (tp_block_nr) = <size-max>/sizeof(void *)
  struct pg_vec {
  	pointer -> each block(memory region)
  };
  __get_free_pages() {
    kmalloc allocates any number of bytes of physically contiguous memory from
    a pool of pre-determined sizes. This pool is maintained by slab-allocator.
    In 2.4/2.6 kernel and i386 architecture, the size limit is 131072 bytes.
    Generally, the size limit is indicated by "size-<bytes>" of /proc/slabinfo.
  }
*/
#if 1
#include <linux/if_packet.h> /*  struct tpacket_req */
#include <sys/user.h> /* PAGE_SIZE */
//#include <linux/mmzone.h> /* MAX_ORDER */
#else

struct tpacket_req { /* for both TPACKET_V1 and TPACKET_V2 */
    unsigned int    tp_block_size;  /* Minimal size of contiguous block */
    unsigned int    tp_block_nr;    /* Number of blocks */
    unsigned int    tp_frame_size;  /* Size of frame */
    unsigned int    tp_frame_nr;    /* Total number of frames */
};

struct tpacket_req3 { /* for TPACKET_V3 */
    unsigned int    tp_block_size;  /* Minimal size of contiguous block */
    unsigned int    tp_block_nr;    /* Number of blocks */
    unsigned int    tp_frame_size;  /* Size of frame */
    unsigned int    tp_frame_nr;    /* Total number of frames */
    unsigned int    tp_retire_blk_tov; /* timeout in msecs */
    unsigned int    tp_sizeof_priv; /* offset to private data area */
    unsigned int    tp_feature_req_word;
};

union tpacket_req_u {
    struct tpacket_req  req;
    struct tpacket_req3 req3;
};

/*---- each circle-buffer/frame/slot have one tpacket_hdr ----*/

struct tpacket_hdr { /* TPACKET_V1*/
    unsigned long   tp_status;
    unsigned int    tp_len;
    unsigned int    tp_snaplen;
    unsigned short  tp_mac;
    unsigned short  tp_net;
    unsigned int    tp_sec;
    unsigned int    tp_usec;
};

struct tpacket2_hdr { /* TPACKET_V2*/
    __u32       tp_status;
    __u32       tp_len;
    __u32       tp_snaplen;
    __u16       tp_mac;
    __u16       tp_net;
    __u32       tp_sec;
    __u32       tp_nsec;
    __u16       tp_vlan_tci;
    __u16       tp_vlan_tpid;
    __u8        tp_padding[4];
};

struct tpacket_hdr_variant1 {
    __u32   tp_rxhash;
    __u32   tp_vlan_tci;
    __u16   tp_vlan_tpid;
    __u16   tp_padding;
};
struct tpacket3_hdr { /* TPACKET_V3*/
    __u32       tp_next_offset;
    __u32       tp_sec;
    __u32       tp_nsec;
    __u32       tp_snaplen;
    __u32       tp_len;
    __u32       tp_status;
    __u16       tp_mac;
    __u16       tp_net;
    /* pkt_hdr variants */
    union {
        struct tpacket_hdr_variant1 hv1;
    };
    __u8        tp_padding[8];
};

/* Frame structure 
 - Start. Frame must be aligned to TPACKET_ALIGNMENT=16
 - struct tpacket_hdr
 - pad to TPACKET_ALIGNMENT=16
 - struct sockaddr_ll
 - Gap, chosen so that packet data (Start+tp_net) aligns to 
   TPACKET_ALIGNMENT=16
 - Start+tp_mac: [ Optional MAC header ]
 - Start+tp_net: Packet data, aligned to TPACKET_ALIGNMENT=16.
 - Pad to align to TPACKET_ALIGNMENT=16
*/
#define TPACKET_ALIGNMENT   16
#define TPACKET_ALIGN(x)    (((x)+TPACKET_ALIGNMENT-1)&~(TPACKET_ALIGNMENT-1))
#define TPACKET_HDRLEN      (TPACKET_ALIGN(sizeof(struct tpacket_hdr)) + sizeof(struct sockaddr_ll))
#define TPACKET2_HDRLEN     (TPACKET_ALIGN(sizeof(struct tpacket2_hdr)) + sizeof(struct sockaddr_ll))
#define TPACKET3_HDRLEN     (TPACKET_ALIGN(sizeof(struct tpacket3_hdr)) + sizeof(struct sockaddr_ll))

/* for PACKET_ADD_MEMBERSHIP/PACKET_DROP_MEMBERSHIP
   to configure physical-layer multicasting and promiscuous mode */
struct packet_mreq {
    int     mr_ifindex;
    unsigned short  mr_type;
    unsigned short  mr_alen;
    unsigned char   mr_address[8];
};
#define PACKET_MR_MULTICAST 0
#define PACKET_MR_PROMISC   1
#define PACKET_MR_ALLMULTI  2
#define PACKET_MR_UNICAST   3

#endif

#if 0
#include <linux/if_vlan.h>
#else
struct vlan_ethhdr {
   unsigned char h_dest[ETH_ALEN]; /* destination eth addr */
   unsigned char h_source[ETH_ALEN]; /* source ether addr */
   __be16 h_vlan_proto; /* Should always be 0x8100 */
   __be16 h_vlan_TCI; /* Encapsulates priority and VLAN ID */
   __be16 h_vlan_encapsulated_proto; /* packet type ID field (or len) */
} __attribute__((packed));
#endif

#if 1
#include <linux/ip.h>	/*  IP protocols */
#else
struct iphdr {
#if defined(__LITTLE_ENDIAN_BITFIELD)
    __u8    ihl:4,
        version:4;
#elif defined (__BIG_ENDIAN_BITFIELD)
    __u8    version:4,
        ihl:4;
#else
#error  "Please fix <asm/byteorder.h>"
#endif
    __u8    tos;
    __be16  tot_len;
    __be16  id;
    __be16  frag_off;
    __u8    ttl;
    __u8    protocol;
    __sum16 check;
    __be32  saddr;
    __be32  daddr;
    /*The options start here. */
};
#endif
struct pseudoiphdr {
	uint32_t saddr;
	uint32_t daddr;	
	uint8_t  zeros;
	uint8_t  protocol;
	uint16_t len;
};

#if 1
#include <linux/in.h>
#else
/* Standard well-defined IP protocols. */
enum {
	IPPROTO_IP = 0,			/* Dummy protocol for TCP		*/
	IPPROTO_ICMP = 1,		/* Internet Control Message Protocol	*/
	IPPROTO_IGMP = 2,		/* Internet Group Management Protocol	*/
	IPPROTO_IPIP = 4,		/* IPIP tunnels (older KA9Q tunnels use 94) */
	IPPROTO_TCP = 6,		/* Transmission Control Protocol	*/
	IPPROTO_EGP = 8,		/* Exterior Gateway Protocol		*/
	IPPROTO_PUP = 12,		/* PUP protocol				*/
	IPPROTO_UDP = 17,		/* User Datagram Protocol		*/
	IPPROTO_IDP = 22,		/* XNS IDP protocol			*/
	IPPROTO_DCCP = 33,		/* Datagram Congestion Control Protocol */
	IPPROTO_RSVP = 46,		/* RSVP protocol			*/
	IPPROTO_GRE = 47,		/* Cisco GRE tunnels (rfc 1701,1702)	*/
	IPPROTO_IPV6 = 41,		/* IPv6-in-IPv4 tunnelling		*/
	IPPROTO_ESP = 50,       	/* Encapsulation Security Payload protocol */
	IPPROTO_AH = 51,             	/* Authentication Header protocol       */
	IPPROTO_BEETPH = 94,	       	/* IP option pseudo header for BEET */
	IPPROTO_PIM    = 103,		/* Protocol Independent Multicast	*/
	IPPROTO_COMP   = 108,           /* Compression Header protocol */
	IPPROTO_SCTP   = 132,		/* Stream Control Transport Protocol	*/
	IPPROTO_UDPLITE = 136,		/* UDP-Lite (RFC 3828)			*/
	IPPROTO_RAW	 = 255,		/* Raw IP packets			*/
	IPPROTO_MAX
};
/* Internet address. */
struct in_addr {
	__be32	s_addr;
};

struct sockaddr_in {
  __kernel_sa_family_t  sin_family; /* Address family       */
  __be16        		sin_port;   /* Port number          */
  struct in_addr    	sin_addr;   /* Internet address     */
  /* Pad to size of `struct sockaddr'. */
  unsigned char     __pad[__SOCK_SIZE__ - sizeof(short int) -
            sizeof(unsigned short int) - sizeof(struct in_addr)];
};

#endif

#if 1
#include <linux/udp.h>
#else
struct udphdr {
    __be16  source;
    __be16  dest;
    __be16  len;
    __sum16 check;
};
#endif

#if 1
#include <linux/tcp.h>
#else
struct tcphdr {
    __be16  source;
    __be16  dest;
    __be32  seq;
    __be32  ack_seq;
#if defined(__LITTLE_ENDIAN_BITFIELD)
    __u16   res1:4,
        doff:4,
        fin:1,
        syn:1,
        rst:1,
        psh:1,
        ack:1,
        urg:1,
        ece:1,
        cwr:1;
#elif defined(__BIG_ENDIAN_BITFIELD)                                             
    __u16   doff:4,
        res1:4,
        cwr:1,
        ece:1,
        urg:1,
        ack:1,
        psh:1,
        rst:1,
        syn:1,
        fin:1;
#else
#error  "Adjust your <asm/byteorder.h> defines"
#endif
    __be16  window;
    __sum16 check;
    __be16  urg_ptr;
};
#endif

#if 1
#include <net/if.h>
#else
struct ifreq
{
# define IFHWADDRLEN    6
# define IFNAMSIZ    IF_NAMESIZE
	union
	{
		char ifrn_name[IFNAMSIZ];	 /* Interface name, e.g. "en0".  */
	} ifr_ifrn;

	union
	{
		struct sockaddr ifru_addr;
		struct sockaddr ifru_dstaddr;
		struct sockaddr ifru_broadaddr;
		struct sockaddr ifru_netmask;
		struct sockaddr ifru_hwaddr;
		short int ifru_flags;
		int ifru_ivalue;
		int ifru_mtu;
		struct ifmap ifru_map;
		char ifru_slave[IFNAMSIZ];	  /* Just fits the size */
		char ifru_newname[IFNAMSIZ];
		__caddr_t ifru_data;
	} ifr_ifru;
}
#endif

#ifdef BUILDING_IP_UDP
#define ETH_HDR_LEN 0
#else
#ifdef BUILDING_RAWS_VLAN
#define BBU_RRU_VLANID 0x001
#define ETH_HDR_LEN sizeof(struct vlan_ethhdr)
#else
#define ETH_HDR_LEN sizeof(struct ethhdr)
#endif
#endif

#ifdef BUILDING_RAWS_UDP
#define IP_HDR_LEN	sizeof(struct iphdr)
#define UDP_HDR_LEN sizeof(struct udphdr)
#else
#define IP_HDR_LEN	0
#define UDP_HDR_LEN 0
#endif
/*  ethhdr(14)=14
	ethhdr(14)+iphdr(20)+udphdr(8)=42
	vlanhdr(18)=18
	vlanhdr(18)+iphdr(20)+udphdr(8)=46 */
#define PKG_OFFSET	(ETH_HDR_LEN + IP_HDR_LEN + UDP_HDR_LEN)
/* 14+48=62; 42+48=90 */
#define PUSCH_PLDOF	(PKG_OFFSET + sizeof(struct pusch_hdr))
/* 14+38=52; 42+38=80 */
#define PDSCH_PLDOF	(PKG_OFFSET + sizeof(struct pdsch_hdr))
/* 14+24=38; 42+24=66 */
#define TMPPS_PLDOF	(PKG_OFFSET + sizeof(struct tmpps_hdr))

#if 0
#define PUSCH_UBUF_LEN	max(PKG_OFFSET + PUSCH_SIZE, 64)
#define PDSCH_UBUF_LEN	max(PKG_OFFSET + PDSCH_SIZE, 64)
#define TMPPS_UBUF_LEN  max(PKG_OFFSET + TMPPS_SIZE, 64)
#else
/* 62+1200*2*PORT_PER_DEVICE=2462/4862 */
/* 90+1200*2*PORT_PER_DEVICE=2490/4890 */
#define PUSCH_UBUF_LEN	(PKG_OFFSET + PUSCH_SIZE)
/* 52+1200*2*PORT_PER_DEVICE=2452/4852 */
/* 80+1200*2*PORT_PER_DEVICE=2480/4880 */
#define PDSCH_UBUF_LEN	(PKG_OFFSET + PDSCH_SIZE)
/* 38; 66 */
#define TMPPS_UBUF_LEN  (PKG_OFFSET + TMPPS_SIZE)
#endif

#define SUBCARRIER_NTXBUF_SZ PUSCH_UBUF_LEN /* max(PUSCH_UBUF_LEN, PDSCH_UBUF_LEN) */
#define SUBCARRIER_NRXBUF_SZ 4890 /* BBU alwasy send 4880 pdsch */

/*==================================
	shared interfaces from "rru_bbu.c"
*/

extern uint32_t pusch_tx_cnt_symbl;
extern uint32_t pusch_rx_cnt;

extern uint32_t pdsch_tx_cnt_symbl;
extern uint32_t pdsch_rx_cnt;

extern uint32_t iio_rx_overflow;
extern uint32_t iio_tx_underflow;
extern uint32_t iio_tx_overflow;
extern uint32_t raws_txbuf_overflow;
extern uint32_t raws_rxbuf_underflow;
extern uint32_t raws_rxbuf_overflow;
extern uint32_t tmpps_tx_cnt;
extern uint32_t tmpps_rx_cnt;

extern void clear_counters(void);

extern void display_tx_counters(void);
extern void display_rx_counters(void);
extern void display_counters(void);

extern void display_tx_perf(struct timespec *e, struct timespec *s);
extern void display_rx_perf(struct timespec *e, struct timespec *s);

extern void display_buffer(uint8_t *buf, size_t size);

extern void display_pusch(uint8_t *buf);
extern void fill_pusch_pld(uint8_t *buf);
extern void pack_pusch_hdr(uint8_t *buf);
extern void unpack_pusch(uint8_t *buf);

extern void display_pdsch(uint8_t *buf);
extern void fill_pdsch_pld(uint8_t *buf);
extern void pack_pdsch_hdr(uint8_t *buf);
extern void unpack_pdsch(uint8_t *buf);

extern void display_tmpps(uint8_t *buf);
extern void pack_tmpps(uint8_t *buf);
extern void unpack_tmpps(uint8_t *buf);

/*==================================
	shared interfaces from "ad9371-iiostream-jacky.c"
*/

extern void libiio_app_startup(int iios);
extern void libiio_app_shutdown(void);
extern int datapath_handler_tx(int idev);
extern int datapath_handler_rx(int idev);

#if 0
extern uint8_t sockrxbuf[SUBCARRIER_NRXBUF_SZ][SUBCARRIER_NRXBUF_NB];
extern volatile int ir_netrxbuf;
extern volatile int iw_netrxbuf;
extern uint8_t socktxbuf[SUBCARRIER_NTXBUF_SZ][SUBCARRIER_NTXBUF_NB];
extern volatile int iw_socktxbuf;
extern volatile int ir_socktxbuf;
#endif

#endif
