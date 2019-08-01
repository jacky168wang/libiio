/*
	https://docs.oracle.com/cd/E19253-01/817-4415/sockets-85885/index.html

	In a one-to-many style socket, each socket handles multiple SCTP associations.
	Each association has an association identifier called sctp_assoc_t. 
	socket(AF_INET[6], SOCK_SEQPACKET, IPPROTO_SCTP); 

*/

#if 0

int sctp_bindx(int sock, void *addrs, int addrcnt, int flags); 

#endif

#if 0
struct sctp_rtoinfo {
    sctp_assoc_t    srto_assoc_id;
    uint32_t        srto_initial;
    uint32_t        srto_max; 
    uint32_t        srto_min;
};

struct sctp_assocparams {
    sctp_assoc_t    sasoc_assoc_id;
    uint16_t        sasoc_asocmaxrxt;
    uint16_t        sasoc_number_peer_destinations;
    uint32_t        sasoc_peer_rwnd;
    uint32_t        sasoc_local_rwnd;
    uint32_t        sasoc_cookie_life;
};

struct sctp_sndrcvinfo {
    uint16_t        sinfo_stream;
    uint16_t        sinfo_ssn;
    uint16_t        sinfo_flags;
    uint32_t        sinfo_ppid;
    uint32_t        sinfo_context;
    uint32_t        sinfo_timetolive;
    uint32_t        sinfo_tsn;
    uint32_t        sinfo_cumtsn;
    sctp_assoc_t    sinfo_assoc_id;
};

struct sctp_paddrparams {
    sctp_assoc_t               spp_assoc_id;
    struct sockaddr_storage    spp_address;
    uint32_t                   spp_hbinterval;
    uint16_t                   spp_pathmaxrxt;
};

struct sctp_status {
    sctp_assoc_t             sstat_assoc_id;
    int32_t                  sstat_state;
    uint32_t                 sstat_rwnd;
    uint16_t                 sstat_unackdata;
    uint16_t                 sstat_penddata;
    uint16_t                 sstat_instrms;
    uint16_t                 sstat_outstrms;
    uint32_t                 sstat_fragmentation_point;
    struct sctp_paddrinfo    sstat_primary;
};

struct sctp_paddrinfo {
    sctp_assoc_t               spinfo_assoc_id;
    struct sockaddr_storage    spinfo_address;
    int32_t                    spinfo_state;
    uint32_t                   spinfo_cwnd;
    uint32_t                   spinfo_srtt;
    uint32_t                   spinfo_rto;
    uint32_t                   spinfo_mtu;
};

int sctp_opt_info(int sock, sctp_assoc_id_t id, int opt, void *arg, socklen_t *len);

#endif

#if 0

ssize_t sctp_recvmsg(int s, void *msg, size_t len, struct sockaddr *from, socklen_t *fromlen, struct sctp_sndrcvinfo *sinfo, int *msg_flags); 

ssize_t sctp_sendmsg(int s, const void *msg, size_t len, const struct sockaddr *to, socklen_t tolen, uint32_t ppid, uint32_t flags, uint16_t stream_no, uint32_t timetolive, uint32_t context);

ssize_t sctp_send(int s, const void *msg, size_t len, const struct sctp_sndrcvinfo *sinfo, int flags);

#endif

#if 0


int sctp_peeloff(int sock, sctp_assoc_t id);

int sctp_getpaddrs(int sock, sctp_assoc_t id, void **addrs); 

void sctp_freepaddrs(void *addrs);

int sctp_getladdrs(int sock, sctp_assoc_t id, void **addrs);

void sctp_freeladdrs(void *addrs); 


#endif
