

#ifndef LINUX_RIO_DRV_H
#define LINUX_RIO_DRV_H

#include <linux/types.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/rio.h>

extern int __rio_local_read_config_32(struct rio_mport *port, u32 offset,
				      u32 * data);
extern int __rio_local_write_config_32(struct rio_mport *port, u32 offset,
				       u32 data);
extern int __rio_local_read_config_16(struct rio_mport *port, u32 offset,
				      u16 * data);
extern int __rio_local_write_config_16(struct rio_mport *port, u32 offset,
				       u16 data);
extern int __rio_local_read_config_8(struct rio_mport *port, u32 offset,
				     u8 * data);
extern int __rio_local_write_config_8(struct rio_mport *port, u32 offset,
				      u8 data);

extern int rio_mport_read_config_32(struct rio_mport *port, u16 destid,
				    u8 hopcount, u32 offset, u32 * data);
extern int rio_mport_write_config_32(struct rio_mport *port, u16 destid,
				     u8 hopcount, u32 offset, u32 data);
extern int rio_mport_read_config_16(struct rio_mport *port, u16 destid,
				    u8 hopcount, u32 offset, u16 * data);
extern int rio_mport_write_config_16(struct rio_mport *port, u16 destid,
				     u8 hopcount, u32 offset, u16 data);
extern int rio_mport_read_config_8(struct rio_mport *port, u16 destid,
				   u8 hopcount, u32 offset, u8 * data);
extern int rio_mport_write_config_8(struct rio_mport *port, u16 destid,
				    u8 hopcount, u32 offset, u8 data);

static inline int rio_local_read_config_32(struct rio_mport *port, u32 offset,
					   u32 * data)
{
	return __rio_local_read_config_32(port, offset, data);
}

static inline int rio_local_write_config_32(struct rio_mport *port, u32 offset,
					    u32 data)
{
	return __rio_local_write_config_32(port, offset, data);
}

static inline int rio_local_read_config_16(struct rio_mport *port, u32 offset,
					   u16 * data)
{
	return __rio_local_read_config_16(port, offset, data);
}


static inline int rio_local_write_config_16(struct rio_mport *port, u32 offset,
					    u16 data)
{
	return __rio_local_write_config_16(port, offset, data);
}

static inline int rio_local_read_config_8(struct rio_mport *port, u32 offset,
					  u8 * data)
{
	return __rio_local_read_config_8(port, offset, data);
}

static inline int rio_local_write_config_8(struct rio_mport *port, u32 offset,
					   u8 data)
{
	return __rio_local_write_config_8(port, offset, data);
}

static inline int rio_read_config_32(struct rio_dev *rdev, u32 offset,
				     u32 * data)
{
	u8 hopcount = 0xff;
	u16 destid = rdev->destid;

	if (rdev->rswitch) {
		destid = rdev->rswitch->destid;
		hopcount = rdev->rswitch->hopcount;
	}

	return rio_mport_read_config_32(rdev->net->hport, destid, hopcount,
					offset, data);
};

static inline int rio_write_config_32(struct rio_dev *rdev, u32 offset,
				      u32 data)
{
	u8 hopcount = 0xff;
	u16 destid = rdev->destid;

	if (rdev->rswitch) {
		destid = rdev->rswitch->destid;
		hopcount = rdev->rswitch->hopcount;
	}

	return rio_mport_write_config_32(rdev->net->hport, destid, hopcount,
					 offset, data);
};

static inline int rio_read_config_16(struct rio_dev *rdev, u32 offset,
				     u16 * data)
{
	u8 hopcount = 0xff;
	u16 destid = rdev->destid;

	if (rdev->rswitch) {
		destid = rdev->rswitch->destid;
		hopcount = rdev->rswitch->hopcount;
	}

	return rio_mport_read_config_16(rdev->net->hport, destid, hopcount,
					offset, data);
};

static inline int rio_write_config_16(struct rio_dev *rdev, u32 offset,
				      u16 data)
{
	u8 hopcount = 0xff;
	u16 destid = rdev->destid;

	if (rdev->rswitch) {
		destid = rdev->rswitch->destid;
		hopcount = rdev->rswitch->hopcount;
	}

	return rio_mport_write_config_16(rdev->net->hport, destid, hopcount,
					 offset, data);
};

static inline int rio_read_config_8(struct rio_dev *rdev, u32 offset, u8 * data)
{
	u8 hopcount = 0xff;
	u16 destid = rdev->destid;

	if (rdev->rswitch) {
		destid = rdev->rswitch->destid;
		hopcount = rdev->rswitch->hopcount;
	}

	return rio_mport_read_config_8(rdev->net->hport, destid, hopcount,
				       offset, data);
};

static inline int rio_write_config_8(struct rio_dev *rdev, u32 offset, u8 data)
{
	u8 hopcount = 0xff;
	u16 destid = rdev->destid;

	if (rdev->rswitch) {
		destid = rdev->rswitch->destid;
		hopcount = rdev->rswitch->hopcount;
	}

	return rio_mport_write_config_8(rdev->net->hport, destid, hopcount,
					offset, data);
};

extern int rio_mport_send_doorbell(struct rio_mport *mport, u16 destid,
				   u16 data);

static inline int rio_send_doorbell(struct rio_dev *rdev, u16 data)
{
	return rio_mport_send_doorbell(rdev->net->hport, rdev->destid, data);
};

static inline void rio_init_mbox_res(struct resource *res, int start, int end)
{
	memset(res, 0, sizeof(struct resource));
	res->start = start;
	res->end = end;
	res->flags = RIO_RESOURCE_MAILBOX;
}

static inline void rio_init_dbell_res(struct resource *res, u16 start, u16 end)
{
	memset(res, 0, sizeof(struct resource));
	res->start = start;
	res->end = end;
	res->flags = RIO_RESOURCE_DOORBELL;
}

#define RIO_DEVICE(dev,ven) \
	.did = (dev), .vid = (ven), \
	.asm_did = RIO_ANY_ID, .asm_vid = RIO_ANY_ID

/* Mailbox management */
extern int rio_request_outb_mbox(struct rio_mport *, void *, int, int,
				 void (*)(struct rio_mport *, void *,int, int));
extern int rio_release_outb_mbox(struct rio_mport *, int);

static inline int rio_add_outb_message(struct rio_mport *mport,
				       struct rio_dev *rdev, int mbox,
				       void *buffer, size_t len)
{
	return rio_hw_add_outb_message(mport, rdev, mbox, buffer, len);
}

extern int rio_request_inb_mbox(struct rio_mport *, void *, int, int,
				void (*)(struct rio_mport *, void *, int, int));
extern int rio_release_inb_mbox(struct rio_mport *, int);

static inline int rio_add_inb_buffer(struct rio_mport *mport, int mbox,
				     void *buffer)
{
	return rio_hw_add_inb_buffer(mport, mbox, buffer);
}

static inline void *rio_get_inb_message(struct rio_mport *mport, int mbox)
{
	return rio_hw_get_inb_message(mport, mbox);
}

/* Doorbell management */
extern int rio_request_inb_dbell(struct rio_mport *, void *, u16, u16,
				 void (*)(struct rio_mport *, void *, u16, u16, u16));
extern int rio_release_inb_dbell(struct rio_mport *, u16, u16);
extern struct resource *rio_request_outb_dbell(struct rio_dev *, u16, u16);
extern int rio_release_outb_dbell(struct rio_dev *, struct resource *);

/* Memory region management */
int rio_claim_resource(struct rio_dev *, int);
int rio_request_regions(struct rio_dev *, char *);
void rio_release_regions(struct rio_dev *);
int rio_request_region(struct rio_dev *, int, char *);
void rio_release_region(struct rio_dev *, int);

/* Port-Write management */
extern int rio_request_inb_pwrite(struct rio_dev *,
			int (*)(struct rio_dev *, union rio_pw_msg*, int));
extern int rio_release_inb_pwrite(struct rio_dev *);
extern int rio_inb_pwrite_handler(union rio_pw_msg *pw_msg);

/* LDM support */
int rio_register_driver(struct rio_driver *);
void rio_unregister_driver(struct rio_driver *);
struct rio_dev *rio_dev_get(struct rio_dev *);
void rio_dev_put(struct rio_dev *);

static inline const char *rio_name(struct rio_dev *rdev)
{
	return dev_name(&rdev->dev);
}

static inline void *rio_get_drvdata(struct rio_dev *rdev)
{
	return dev_get_drvdata(&rdev->dev);
}

static inline void rio_set_drvdata(struct rio_dev *rdev, void *data)
{
	dev_set_drvdata(&rdev->dev, data);
}

/* Misc driver helpers */
extern u16 rio_local_get_device_id(struct rio_mport *port);
extern struct rio_dev *rio_get_device(u16 vid, u16 did, struct rio_dev *from);
extern struct rio_dev *rio_get_asm(u16 vid, u16 did, u16 asm_vid, u16 asm_did,
				   struct rio_dev *from);

#endif				/* LINUX_RIO_DRV_H */
