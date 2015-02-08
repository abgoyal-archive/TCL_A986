

#ifndef __SCO_H
#define __SCO_H

/* SCO defaults */
#define SCO_DEFAULT_MTU		500
#define SCO_DEFAULT_FLUSH_TO	0xFFFF

#define SCO_CONN_TIMEOUT	(HZ * 40)
#define SCO_DISCONN_TIMEOUT	(HZ * 2)
#define SCO_CONN_IDLE_TIMEOUT	(HZ * 60)

/* SCO socket address */
struct sockaddr_sco {
	sa_family_t	sco_family;
	bdaddr_t	sco_bdaddr;
	__u16		sco_pkt_type;
};

/* SCO socket options */
#define SCO_OPTIONS	0x01
struct sco_options {
	__u16 mtu;
};

#define SCO_CONNINFO	0x02
struct sco_conninfo {
	__u16 hci_handle;
	__u8  dev_class[3];
};

/* ---- SCO connections ---- */
struct sco_conn {
	struct hci_conn	*hcon;

	bdaddr_t 	*dst;
	bdaddr_t 	*src;
	
	spinlock_t	lock;
	struct sock 	*sk;

	unsigned int    mtu;
};

#define sco_conn_lock(c)	spin_lock(&c->lock);
#define sco_conn_unlock(c)	spin_unlock(&c->lock);

/* ----- SCO socket info ----- */
#define sco_pi(sk) ((struct sco_pinfo *) sk)

struct sco_pinfo {
	struct bt_sock	bt;
	__u16		pkt_type;

	struct sco_conn	*conn;
};

#endif /* __SCO_H */
