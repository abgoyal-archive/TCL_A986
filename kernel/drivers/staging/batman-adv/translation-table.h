

#include "types.h"

int hna_local_init(void);
void hna_local_add(uint8_t *addr);
void hna_local_remove(uint8_t *addr, char *message);
int hna_local_fill_buffer(unsigned char *buff, int buff_len);
int hna_local_fill_buffer_text(struct net_device *net_dev, char *buff,
			       size_t count, loff_t off);
void hna_local_purge(struct work_struct *work);
void hna_local_free(void);
int hna_global_init(void);
void hna_global_add_orig(struct orig_node *orig_node, unsigned char *hna_buff,
			 int hna_buff_len);
int hna_global_fill_buffer_text(struct net_device *net_dev, char *buff,
				size_t count, loff_t off);
void _hna_global_del_orig(struct hna_global_entry *hna_global_entry,
			  char *orig_str);
void hna_global_del_orig(struct orig_node *orig_node, char *message);
void hna_global_free(void);
struct orig_node *transtable_search(uint8_t *addr);

extern spinlock_t hna_local_hash_lock;
extern struct hashtable_t *hna_local_hash;
extern atomic_t hna_local_changed;
