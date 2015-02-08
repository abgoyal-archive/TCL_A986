

#ifndef _TCPIP_H_
#define _TCPIP_H_

struct tcpiphdr {
	struct 	ipovly ti_i;		/* overlaid ip structure */
	struct	tcphdr ti_t;		/* tcp header */
};
#define ti_mbuf         ti_i.ih_mbuf.mptr
#define	ti_x1		ti_i.ih_x1
#define	ti_pr		ti_i.ih_pr
#define	ti_len		ti_i.ih_len
#define	ti_src		ti_i.ih_src
#define	ti_dst		ti_i.ih_dst
#define	ti_sport	ti_t.th_sport
#define	ti_dport	ti_t.th_dport
#define	ti_seq		ti_t.th_seq
#define	ti_ack		ti_t.th_ack
#define	ti_x2		ti_t.th_x2
#define	ti_off		ti_t.th_off
#define	ti_flags	ti_t.th_flags
#define	ti_win		ti_t.th_win
#define	ti_sum		ti_t.th_sum
#define	ti_urp		ti_t.th_urp

#define tcpiphdr2qlink(T) ((struct qlink*)(((char*)(T)) - sizeof(struct qlink)))
#define qlink2tcpiphdr(Q) ((struct tcpiphdr*)(((char*)(Q)) + sizeof(struct qlink)))
#define tcpiphdr_next(T) qlink2tcpiphdr(tcpiphdr2qlink(T)->next)
#define tcpiphdr_prev(T) qlink2tcpiphdr(tcpiphdr2qlink(T)->prev)
#define tcpfrag_list_first(T) qlink2tcpiphdr((T)->seg_next)
#define tcpfrag_list_end(F, T) (tcpiphdr2qlink(F) == (struct qlink*)(T))
#define tcpfrag_list_empty(T) ((T)->seg_next == (struct tcpiphdr*)(T))

struct tcpiphdr_2 {
	struct tcpiphdr dummy;
	char first_char;
};

#endif
