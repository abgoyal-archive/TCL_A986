

#ifndef _UDP_H_
#define _UDP_H_

#define UDP_TTL 0x60
#define UDP_UDPDATALEN 16192

extern struct socket *udp_last_so;

struct udphdr {
	u_int16_t	uh_sport;		/* source port */
	u_int16_t	uh_dport;		/* destination port */
	int16_t	uh_ulen;		/* udp length */
	u_int16_t	uh_sum;			/* udp checksum */
};

struct udpiphdr {
	        struct  ipovly ui_i;            /* overlaid ip structure */
	        struct  udphdr ui_u;            /* udp header */
};
#define ui_mbuf         ui_i.ih_mbuf.mptr
#define ui_x1           ui_i.ih_x1
#define ui_pr           ui_i.ih_pr
#define ui_len          ui_i.ih_len
#define ui_src          ui_i.ih_src
#define ui_dst          ui_i.ih_dst
#define ui_sport        ui_u.uh_sport
#define ui_dport        ui_u.uh_dport
#define ui_ulen         ui_u.uh_ulen
#define ui_sum          ui_u.uh_sum

#ifdef LOG_ENABLED
struct udpstat {
	                                /* input statistics: */
	        u_long  udps_ipackets;          /* total input packets */
	        u_long  udps_hdrops;            /* packet shorter than header */
	        u_long  udps_badsum;            /* checksum error */
	        u_long  udps_badlen;            /* data length larger than packet */
	        u_long  udps_noport;            /* no socket on port */
	        u_long  udps_noportbcast;       /* of above, arrived as broadcast */
	        u_long  udps_fullsock;          /* not delivered, input socket full */
	        u_long  udpps_pcbcachemiss;     /* input packets missing pcb cache */
	                                /* output statistics: */
	        u_long  udps_opackets;          /* total output packets */
};
#endif

#define UDPCTL_CHECKSUM         1       /* checksum UDP packets */
#define UDPCTL_MAXID            2

#ifdef LOG_ENABLED
extern struct udpstat udpstat;
#endif

extern struct socket udb;
struct mbuf;

void udp_init _P((void));
void udp_input _P((register struct mbuf *, int));
int udp_output _P((struct socket *, struct mbuf *, struct sockaddr_in *));
int udp_attach _P((struct socket *));
void udp_detach _P((struct socket *));
struct socket * udp_listen _P((u_int, u_int32_t, u_int, int));
int udp_output2(struct socket *so, struct mbuf *m,
                struct sockaddr_in *saddr, struct sockaddr_in *daddr,
                int iptos);
#endif
