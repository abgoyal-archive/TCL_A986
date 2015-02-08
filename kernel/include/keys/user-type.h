

#ifndef _KEYS_USER_TYPE_H
#define _KEYS_USER_TYPE_H

#include <linux/key.h>
#include <linux/rcupdate.h>

/*****************************************************************************/
struct user_key_payload {
	struct rcu_head	rcu;		/* RCU destructor */
	unsigned short	datalen;	/* length of this data */
	char		data[0];	/* actual data */
};

extern struct key_type key_type_user;

extern int user_instantiate(struct key *key, const void *data, size_t datalen);
extern int user_update(struct key *key, const void *data, size_t datalen);
extern int user_match(const struct key *key, const void *criterion);
extern void user_revoke(struct key *key);
extern void user_destroy(struct key *key);
extern void user_describe(const struct key *user, struct seq_file *m);
extern long user_read(const struct key *key,
		      char __user *buffer, size_t buflen);


#endif /* _KEYS_USER_TYPE_H */
