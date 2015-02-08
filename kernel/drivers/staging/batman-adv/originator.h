

int originator_init(void);
void free_orig_node(void *data);
void originator_free(void);
void purge_orig(struct work_struct *work);
struct orig_node *orig_find(char *mac);
struct orig_node *get_orig_node(uint8_t *addr);
struct neigh_node *
create_neighbor(struct orig_node *orig_node, struct orig_node *orig_neigh_node,
		uint8_t *neigh, struct batman_if *if_incoming);
ssize_t orig_fill_buffer_text(struct net_device *net_dev, char *buff,
			      size_t count, loff_t off);
int orig_hash_add_if(struct batman_if *batman_if, int max_if_num);
int orig_hash_del_if(struct batman_if *batman_if, int max_if_num);
