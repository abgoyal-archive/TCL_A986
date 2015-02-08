

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/err.h>
#include <keys/user-type.h>
#include <asm/uaccess.h>
#include "internal.h"

struct key_type key_type_user = {
	.name		= "user",
	.instantiate	= user_instantiate,
	.update		= user_update,
	.match		= user_match,
	.revoke		= user_revoke,
	.destroy	= user_destroy,
	.describe	= user_describe,
	.read		= user_read,
};

EXPORT_SYMBOL_GPL(key_type_user);

/*****************************************************************************/
int user_instantiate(struct key *key, const void *data, size_t datalen)
{
	struct user_key_payload *upayload;
	int ret;

	ret = -EINVAL;
	if (datalen <= 0 || datalen > 32767 || !data)
		goto error;

	ret = key_payload_reserve(key, datalen);
	if (ret < 0)
		goto error;

	ret = -ENOMEM;
	upayload = kmalloc(sizeof(*upayload) + datalen, GFP_KERNEL);
	if (!upayload)
		goto error;

	/* attach the data */
	upayload->datalen = datalen;
	memcpy(upayload->data, data, datalen);
	rcu_assign_pointer(key->payload.data, upayload);
	ret = 0;

error:
	return ret;

} /* end user_instantiate() */

EXPORT_SYMBOL_GPL(user_instantiate);

/*****************************************************************************/
static void user_update_rcu_disposal(struct rcu_head *rcu)
{
	struct user_key_payload *upayload;

	upayload = container_of(rcu, struct user_key_payload, rcu);

	kfree(upayload);

} /* end user_update_rcu_disposal() */

/*****************************************************************************/
int user_update(struct key *key, const void *data, size_t datalen)
{
	struct user_key_payload *upayload, *zap;
	int ret;

	ret = -EINVAL;
	if (datalen <= 0 || datalen > 32767 || !data)
		goto error;

	/* construct a replacement payload */
	ret = -ENOMEM;
	upayload = kmalloc(sizeof(*upayload) + datalen, GFP_KERNEL);
	if (!upayload)
		goto error;

	upayload->datalen = datalen;
	memcpy(upayload->data, data, datalen);

	/* check the quota and attach the new data */
	zap = upayload;

	ret = key_payload_reserve(key, datalen);

	if (ret == 0) {
		/* attach the new data, displacing the old */
		zap = key->payload.data;
		rcu_assign_pointer(key->payload.data, upayload);
		key->expiry = 0;
	}

	call_rcu(&zap->rcu, user_update_rcu_disposal);

error:
	return ret;

} /* end user_update() */

EXPORT_SYMBOL_GPL(user_update);

/*****************************************************************************/
int user_match(const struct key *key, const void *description)
{
	return strcmp(key->description, description) == 0;

} /* end user_match() */

EXPORT_SYMBOL_GPL(user_match);

/*****************************************************************************/
void user_revoke(struct key *key)
{
	struct user_key_payload *upayload = key->payload.data;

	/* clear the quota */
	key_payload_reserve(key, 0);

	if (upayload) {
		rcu_assign_pointer(key->payload.data, NULL);
		call_rcu(&upayload->rcu, user_update_rcu_disposal);
	}

} /* end user_revoke() */

EXPORT_SYMBOL(user_revoke);

/*****************************************************************************/
void user_destroy(struct key *key)
{
	struct user_key_payload *upayload = key->payload.data;

	kfree(upayload);

} /* end user_destroy() */

EXPORT_SYMBOL_GPL(user_destroy);

/*****************************************************************************/
void user_describe(const struct key *key, struct seq_file *m)
{
	seq_puts(m, key->description);

	seq_printf(m, ": %u", key->datalen);

} /* end user_describe() */

EXPORT_SYMBOL_GPL(user_describe);

/*****************************************************************************/
long user_read(const struct key *key, char __user *buffer, size_t buflen)
{
	struct user_key_payload *upayload;
	long ret;

	upayload = rcu_dereference_protected(
		key->payload.data, rwsem_is_locked(&((struct key *)key)->sem));
	ret = upayload->datalen;

	/* we can return the data as is */
	if (buffer && buflen > 0) {
		if (buflen > upayload->datalen)
			buflen = upayload->datalen;

		if (copy_to_user(buffer, upayload->data, buflen) != 0)
			ret = -EFAULT;
	}

	return ret;

} /* end user_read() */

EXPORT_SYMBOL_GPL(user_read);
