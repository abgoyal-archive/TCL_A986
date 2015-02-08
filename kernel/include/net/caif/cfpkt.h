

#ifndef CFPKT_H_
#define CFPKT_H_
#include <net/caif/caif_layer.h>
#include <linux/types.h>
struct cfpkt;

struct cfpkt *cfpkt_create(u16 len);

struct cfpkt *cfpkt_create_uplink(const unsigned char *data, unsigned int len);
void cfpkt_destroy(struct cfpkt *pkt);

int cfpkt_extr_head(struct cfpkt *pkt, void *data, u16 len);

int cfpkt_peek_head(struct cfpkt *pkt, void *data, u16 len);

int cfpkt_extr_trail(struct cfpkt *pkt, void *data, u16 len);

int cfpkt_add_head(struct cfpkt *pkt, const void *data, u16 len);

int cfpkt_add_trail(struct cfpkt *pkt, const void *data, u16 len);

int cfpkt_pad_trail(struct cfpkt *pkt, u16 len);

int cfpkt_addbdy(struct cfpkt *pkt, const u8 data);

int cfpkt_add_body(struct cfpkt *pkt, const void *data, u16 len);

bool cfpkt_more(struct cfpkt *pkt);

bool cfpkt_erroneous(struct cfpkt *pkt);

u16 cfpkt_getlen(struct cfpkt *pkt);

int cfpkt_setlen(struct cfpkt *pkt, u16 len);

struct cfpkt *cfpkt_append(struct cfpkt *dstpkt, struct cfpkt *addpkt,
		      u16 expectlen);

struct cfpkt *cfpkt_split(struct cfpkt *pkt, u16 pos);


u16 cfpkt_iterate(struct cfpkt *pkt,
		u16 (*iter_func)(u16 chks, void *buf, u16 len),
		u16 data);

int cfpkt_raw_append(struct cfpkt *cfpkt, void **buf, unsigned int buflen);

int cfpkt_raw_extract(struct cfpkt *cfpkt, void **buf, unsigned int buflen);

struct cfpkt *cfpkt_fromnative(enum caif_direction dir, void *nativepkt);

void *cfpkt_tonative(struct cfpkt *pkt);

void cfpkt_queue(struct cfpktq *pktq, struct cfpkt *pkt,
		 unsigned short prio);

struct cfpkt *cfpkt_dequeue(struct cfpktq *pktq);

struct cfpkt *cfpkt_qpeek(struct cfpktq *pktq);

struct cfpktq *cfpktq_create(void);

int cfpkt_qcount(struct cfpktq *pktq);

char *cfpkt_log_pkt(struct cfpkt *pkt, char *buf, int buflen);

struct cfpkt *cfpkt_clone_release(struct cfpkt *pkt);


struct caif_payload_info *cfpkt_info(struct cfpkt *pkt);
/*! @} */
#endif				/* CFPKT_H_ */
