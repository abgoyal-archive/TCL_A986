

#include <linux/dcache.h>

#if defined(CONFIG_ISDN_CAPI_CAPIFS) || defined(CONFIG_ISDN_CAPI_CAPIFS_MODULE)

struct dentry *capifs_new_ncci(unsigned int num, dev_t device);
void capifs_free_ncci(struct dentry *dentry);

#else

static inline struct dentry *capifs_new_ncci(unsigned int num, dev_t device)
{
	return NULL;
}

static inline void capifs_free_ncci(struct dentry *dentry)
{
}

#endif
