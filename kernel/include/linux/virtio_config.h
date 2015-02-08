
#ifndef _LINUX_VIRTIO_CONFIG_H
#define _LINUX_VIRTIO_CONFIG_H

#include <linux/types.h>

/* Status byte for guest to report progress, and synchronize features. */
/* We have seen device and processed generic fields (VIRTIO_CONFIG_F_VIRTIO) */
#define VIRTIO_CONFIG_S_ACKNOWLEDGE	1
/* We have found a driver for the device. */
#define VIRTIO_CONFIG_S_DRIVER		2
/* Driver has used its parts of the config, and is happy */
#define VIRTIO_CONFIG_S_DRIVER_OK	4
/* We've given up on this device. */
#define VIRTIO_CONFIG_S_FAILED		0x80

#define VIRTIO_TRANSPORT_F_START	28
#define VIRTIO_TRANSPORT_F_END		32

#define VIRTIO_F_NOTIFY_ON_EMPTY	24

#ifdef __KERNEL__
#include <linux/err.h>
#include <linux/virtio.h>

typedef void vq_callback_t(struct virtqueue *);
struct virtio_config_ops {
	void (*get)(struct virtio_device *vdev, unsigned offset,
		    void *buf, unsigned len);
	void (*set)(struct virtio_device *vdev, unsigned offset,
		    const void *buf, unsigned len);
	u8 (*get_status)(struct virtio_device *vdev);
	void (*set_status)(struct virtio_device *vdev, u8 status);
	void (*reset)(struct virtio_device *vdev);
	int (*find_vqs)(struct virtio_device *, unsigned nvqs,
			struct virtqueue *vqs[],
			vq_callback_t *callbacks[],
			const char *names[]);
	void (*del_vqs)(struct virtio_device *);
	u32 (*get_features)(struct virtio_device *vdev);
	void (*finalize_features)(struct virtio_device *vdev);
};

/* If driver didn't advertise the feature, it will never appear. */
void virtio_check_driver_offered_feature(const struct virtio_device *vdev,
					 unsigned int fbit);

static inline bool virtio_has_feature(const struct virtio_device *vdev,
				      unsigned int fbit)
{
	/* Did you forget to fix assumptions on max features? */
	MAYBE_BUILD_BUG_ON(fbit >= 32);

	if (fbit < VIRTIO_TRANSPORT_F_START)
		virtio_check_driver_offered_feature(vdev, fbit);

	return test_bit(fbit, vdev->features);
}

#define virtio_config_val(vdev, fbit, offset, v) \
	virtio_config_buf((vdev), (fbit), (offset), (v), sizeof(*v))

static inline int virtio_config_buf(struct virtio_device *vdev,
				    unsigned int fbit,
				    unsigned int offset,
				    void *buf, unsigned len)
{
	if (!virtio_has_feature(vdev, fbit))
		return -ENOENT;

	vdev->config->get(vdev, offset, buf, len);
	return 0;
}

static inline
struct virtqueue *virtio_find_single_vq(struct virtio_device *vdev,
					vq_callback_t *c, const char *n)
{
	vq_callback_t *callbacks[] = { c };
	const char *names[] = { n };
	struct virtqueue *vq;
	int err = vdev->config->find_vqs(vdev, 1, &vq, callbacks, names);
	if (err < 0)
		return ERR_PTR(err);
	return vq;
}
#endif /* __KERNEL__ */
#endif /* _LINUX_VIRTIO_CONFIG_H */
