

#ifndef DISCOVERY_H
#define DISCOVERY_H

#include <asm/param.h>

#include <net/irda/irda.h>
#include <net/irda/irqueue.h>		/* irda_queue_t */
#include <net/irda/irlap_event.h>	/* LAP_REASON */

#define DISCOVERY_EXPIRE_TIMEOUT (2*sysctl_discovery_timeout*HZ)
#define DISCOVERY_DEFAULT_SLOTS  0

typedef union {
	__u16 word;
	__u8  byte[2];
} __u16_host_order;

/* Types of discovery */
typedef enum {
	DISCOVERY_LOG,		/* What's in our discovery log */
	DISCOVERY_ACTIVE,	/* Doing our own discovery on the medium */
	DISCOVERY_PASSIVE,	/* Peer doing discovery on the medium */
	EXPIRY_TIMEOUT,		/* Entry expired due to timeout */
} DISCOVERY_MODE;

#define NICKNAME_MAX_LEN 21

/* Basic discovery information about a peer */
typedef struct irda_device_info		discinfo_t;	/* linux/irda.h */

typedef struct discovery_t {
	irda_queue_t	q;		/* Must be first! */

	discinfo_t	data;		/* Basic discovery information */
	int		name_len;	/* Length of nickname */

	LAP_REASON	condition;	/* More info about the discovery */
	int		gen_addr_bit;	/* Need to generate a new device
					 * address? */
	int		nslots;		/* Number of slots to use when
					 * discovering */
	unsigned long	timestamp;	/* Last time discovered */
	unsigned long	firststamp;	/* First time discovered */
} discovery_t;

void irlmp_add_discovery(hashbin_t *cachelog, discovery_t *discovery);
void irlmp_add_discovery_log(hashbin_t *cachelog, hashbin_t *log);
void irlmp_expire_discoveries(hashbin_t *log, __u32 saddr, int force);
struct irda_device_info *irlmp_copy_discoveries(hashbin_t *log, int *pn,
						__u16 mask, int old_entries);

#endif
