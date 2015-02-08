
#ifndef _LINUX_VIRTIO_RING_H
#define _LINUX_VIRTIO_RING_H
#include <linux/types.h>

/* This marks a buffer as continuing via the next field. */
#define VRING_DESC_F_NEXT	1
/* This marks a buffer as write-only (otherwise read-only). */
#define VRING_DESC_F_WRITE	2
/* This means the buffer contains a list of buffer descriptors. */
#define VRING_DESC_F_INDIRECT	4

#define VRING_USED_F_NO_NOTIFY	1
#define VRING_AVAIL_F_NO_INTERRUPT	1

/* We support indirect buffer descriptors */
#define VIRTIO_RING_F_INDIRECT_DESC	28

/* Virtio ring descriptors: 16 bytes.  These can chain together via "next". */
struct vring_desc {
	/* Address (guest-physical). */
	__u64 addr;
	/* Length. */
	__u32 len;
	/* The flags as indicated above. */
	__u16 flags;
	/* We chain unused descriptors via this, too */
	__u16 next;
};

struct vring_avail {
	__u16 flags;
	__u16 idx;
	__u16 ring[];
};

/* u32 is used here for ids for padding reasons. */
struct vring_used_elem {
	/* Index of start of used descriptor chain. */
	__u32 id;
	/* Total length of the descriptor chain which was used (written to) */
	__u32 len;
};

struct vring_used {
	__u16 flags;
	__u16 idx;
	struct vring_used_elem ring[];
};

struct vring {
	unsigned int num;

	struct vring_desc *desc;

	struct vring_avail *avail;

	struct vring_used *used;
};

static inline void vring_init(struct vring *vr, unsigned int num, void *p,
			      unsigned long align)
{
	vr->num = num;
	vr->desc = p;
	vr->avail = p + num*sizeof(struct vring_desc);
	vr->used = (void *)(((unsigned long)&vr->avail->ring[num] + align-1)
			    & ~(align - 1));
}

static inline unsigned vring_size(unsigned int num, unsigned long align)
{
	return ((sizeof(struct vring_desc) * num + sizeof(__u16) * (2 + num)
		 + align - 1) & ~(align - 1))
		+ sizeof(__u16) * 2 + sizeof(struct vring_used_elem) * num;
}

#ifdef __KERNEL__
#include <linux/irqreturn.h>
struct virtio_device;
struct virtqueue;

struct virtqueue *vring_new_virtqueue(unsigned int num,
				      unsigned int vring_align,
				      struct virtio_device *vdev,
				      void *pages,
				      void (*notify)(struct virtqueue *vq),
				      void (*callback)(struct virtqueue *vq),
				      const char *name);
void vring_del_virtqueue(struct virtqueue *vq);
/* Filter out transport-specific feature bits. */
void vring_transport_features(struct virtio_device *vdev);

irqreturn_t vring_interrupt(int irq, void *_vq);
#endif /* __KERNEL__ */
#endif /* _LINUX_VIRTIO_RING_H */
