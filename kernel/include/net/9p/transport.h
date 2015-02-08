

#ifndef NET_9P_TRANSPORT_H
#define NET_9P_TRANSPORT_H


struct p9_trans_module {
	struct list_head list;
	char *name;		/* name of transport */
	int maxsize;		/* max message size of transport */
	int def;		/* this transport should be default */
	struct module *owner;
	int (*create)(struct p9_client *, const char *, char *);
	void (*close) (struct p9_client *);
	int (*request) (struct p9_client *, struct p9_req_t *req);
	int (*cancel) (struct p9_client *, struct p9_req_t *req);
};

void v9fs_register_trans(struct p9_trans_module *m);
void v9fs_unregister_trans(struct p9_trans_module *m);
struct p9_trans_module *v9fs_get_trans_by_name(const substring_t *name);
struct p9_trans_module *v9fs_get_default_trans(void);
void v9fs_put_trans(struct p9_trans_module *m);
#endif /* NET_9P_TRANSPORT_H */
