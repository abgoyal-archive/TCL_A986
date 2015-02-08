

#include <linux/types.h>
#include <linux/slab.h>
#include <asm/debug.h>

#define IUCV_IPRMDATA	0x80
#define IUCV_IPQUSCE	0x40
#define IUCV_IPBUFLST	0x40
#define IUCV_IPPRTY	0x20
#define IUCV_IPANSLST	0x08
#define IUCV_IPSYNC	0x04
#define IUCV_IPLOCAL	0x01

struct iucv_array {
	u32 address;
	u32 length;
} __attribute__ ((aligned (8)));

extern struct bus_type iucv_bus;
extern struct device *iucv_root;

struct iucv_path {
	u16 pathid;
	u16 msglim;
	u8  flags;
	void *private;
	struct iucv_handler *handler;
	struct list_head list;
};

struct iucv_message {
	u32 id;
	u32 audit;
	u32 class;
	u32 tag;
	u32 length;
	u32 reply_size;
	u8  rmmsg[8];
	u8  flags;
};

struct iucv_handler {
	 /*
	  * The path_pending function is called after an iucv interrupt
	  * type 0x01 has been received. The base code allocates a path
	  * structure and "asks" the handler if this path belongs to the
	  * handler. To accept the path the path_pending function needs
	  * to call iucv_path_accept and return 0. If the callback returns
	  * a value != 0 the iucv base code will continue with the next
	  * handler. The order in which the path_pending functions are
	  * called is the order of the registration of the iucv handlers
	  * to the base code.
	  */
	int  (*path_pending)(struct iucv_path *, u8 ipvmid[8], u8 ipuser[16]);
	/*
	 * The path_complete function is called after an iucv interrupt
	 * type 0x02 has been received for a path that has been established
	 * for this handler with iucv_path_connect and got accepted by the
	 * peer with iucv_path_accept.
	 */
	void (*path_complete)(struct iucv_path *, u8 ipuser[16]);
	 /*
	  * The path_severed function is called after an iucv interrupt
	  * type 0x03 has been received. The communication peer shutdown
	  * his end of the communication path. The path still exists and
	  * remaining messages can be received until a iucv_path_sever
	  * shuts down the other end of the path as well.
	  */
	void (*path_severed)(struct iucv_path *, u8 ipuser[16]);
	/*
	 * The path_quiesced function is called after an icuv interrupt
	 * type 0x04 has been received. The communication peer has quiesced
	 * the path. Delivery of messages is stopped until iucv_path_resume
	 * has been called.
	 */
	void (*path_quiesced)(struct iucv_path *, u8 ipuser[16]);
	/*
	 * The path_resumed function is called after an icuv interrupt
	 * type 0x05 has been received. The communication peer has resumed
	 * the path.
	 */
	void (*path_resumed)(struct iucv_path *, u8 ipuser[16]);
	/*
	 * The message_pending function is called after an icuv interrupt
	 * type 0x06 or type 0x07 has been received. A new message is
	 * availabe and can be received with iucv_message_receive.
	 */
	void (*message_pending)(struct iucv_path *, struct iucv_message *);
	/*
	 * The message_complete function is called after an icuv interrupt
	 * type 0x08 or type 0x09 has been received. A message send with
	 * iucv_message_send2way has been replied to. The reply can be
	 * received with iucv_message_receive.
	 */
	void (*message_complete)(struct iucv_path *, struct iucv_message *);

	struct list_head list;
	struct list_head paths;
};

int iucv_register(struct iucv_handler *handler, int smp);

void iucv_unregister(struct iucv_handler *handle, int smp);

static inline struct iucv_path *iucv_path_alloc(u16 msglim, u8 flags, gfp_t gfp)
{
	struct iucv_path *path;

	path = kzalloc(sizeof(struct iucv_path), gfp);
	if (path) {
		path->msglim = msglim;
		path->flags = flags;
	}
	return path;
}

static inline void iucv_path_free(struct iucv_path *path)
{
	kfree(path);
}

int iucv_path_accept(struct iucv_path *path, struct iucv_handler *handler,
		     u8 userdata[16], void *private);

int iucv_path_connect(struct iucv_path *path, struct iucv_handler *handler,
		      u8 userid[8], u8 system[8], u8 userdata[16],
		      void *private);

int iucv_path_quiesce(struct iucv_path *path, u8 userdata[16]);

int iucv_path_resume(struct iucv_path *path, u8 userdata[16]);

int iucv_path_sever(struct iucv_path *path, u8 userdata[16]);

int iucv_message_purge(struct iucv_path *path, struct iucv_message *msg,
		       u32 srccls);

int iucv_message_receive(struct iucv_path *path, struct iucv_message *msg,
			 u8 flags, void *buffer, size_t size, size_t *residual);

int __iucv_message_receive(struct iucv_path *path, struct iucv_message *msg,
			   u8 flags, void *buffer, size_t size,
			   size_t *residual);

int iucv_message_reject(struct iucv_path *path, struct iucv_message *msg);

int iucv_message_reply(struct iucv_path *path, struct iucv_message *msg,
		       u8 flags, void *reply, size_t size);

int iucv_message_send(struct iucv_path *path, struct iucv_message *msg,
		      u8 flags, u32 srccls, void *buffer, size_t size);

int __iucv_message_send(struct iucv_path *path, struct iucv_message *msg,
			u8 flags, u32 srccls, void *buffer, size_t size);

int iucv_message_send2way(struct iucv_path *path, struct iucv_message *msg,
			  u8 flags, u32 srccls, void *buffer, size_t size,
			  void *answer, size_t asize, size_t *residual);
