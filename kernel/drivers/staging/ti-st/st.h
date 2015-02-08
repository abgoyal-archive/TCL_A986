

#ifndef ST_H
#define ST_H

#include <linux/skbuff.h>

#define N_TI_WL	20	/* Ldisc for TI's WL BT, FM, GPS combo chips */

enum kim_gpio_state {
	KIM_GPIO_INACTIVE,
	KIM_GPIO_ACTIVE,
};
enum proto_type {
	ST_BT,
	ST_FM,
	ST_GPS,
	ST_MAX,
};

enum {
	ST_ERR_FAILURE = -1,	/* check struct */
	ST_SUCCESS,
	ST_ERR_PENDING = -5,	/* to call reg_complete_cb */
	ST_ERR_ALREADY,		/* already registered */
	ST_ERR_INPROGRESS,
	ST_ERR_NOPROTO,		/* protocol not supported */
};

struct st_proto_s {
	enum proto_type type;
	long (*recv) (struct sk_buff *);
	unsigned char (*match_packet) (const unsigned char *data);
	void (*reg_complete_cb) (char data);
	long (*write) (struct sk_buff *skb);
};

extern long st_register(struct st_proto_s *new_proto);
extern long st_unregister(enum proto_type type);

#endif /* ST_H */
