

#ifndef ST_CORE_H
#define ST_CORE_H

#include <linux/skbuff.h>
#include "st.h"

/* states of protocol list */
#define ST_NOTEMPTY	1
#define ST_EMPTY	0

#define ST_INITIALIZING		1
#define ST_REG_IN_PROGRESS	2
#define ST_REG_PENDING		3
#define ST_WAITING_FOR_RESP	4

struct st_data_s {
	unsigned long st_state;
	struct tty_struct *tty;
	struct tty_ldisc_ops *ldisc_ops;
	struct sk_buff *tx_skb;
#define ST_TX_SENDING	1
#define ST_TX_WAKEUP	2
	unsigned long tx_state;
	struct st_proto_s *list[ST_MAX];
	unsigned long rx_state;
	unsigned long rx_count;
	struct sk_buff *rx_skb;
	struct sk_buff_head txq, tx_waitq;
	spinlock_t lock;	/* ST LL state lock  */
	unsigned char	protos_registered;
	unsigned long ll_state;	/* ST LL power state */
};

int st_int_write(struct st_data_s*, const unsigned char*, int);
long st_write(struct sk_buff *);
void st_ll_send_frame(enum proto_type, struct sk_buff *);
/* internal wake up function */
void st_tx_wakeup(struct st_data_s *st_data);

int st_core_init(struct st_data_s **);
void st_core_exit(struct st_data_s *);
void st_kim_ref(struct st_data_s **);

#define GPS_STUB_TEST
#ifdef GPS_STUB_TEST
int gps_chrdrv_stub_write(const unsigned char*, int);
void gps_chrdrv_stub_init(void);
#endif

#endif /*ST_CORE_H */
