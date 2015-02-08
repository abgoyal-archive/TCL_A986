

#ifndef MLX4_DRIVER_H
#define MLX4_DRIVER_H

#include <linux/device.h>

struct mlx4_dev;

enum mlx4_dev_event {
	MLX4_DEV_EVENT_CATASTROPHIC_ERROR,
	MLX4_DEV_EVENT_PORT_UP,
	MLX4_DEV_EVENT_PORT_DOWN,
	MLX4_DEV_EVENT_PORT_REINIT,
};

struct mlx4_interface {
	void *			(*add)	 (struct mlx4_dev *dev);
	void			(*remove)(struct mlx4_dev *dev, void *context);
	void			(*event) (struct mlx4_dev *dev, void *context,
					  enum mlx4_dev_event event, int port);
	struct list_head	list;
};

int mlx4_register_interface(struct mlx4_interface *intf);
void mlx4_unregister_interface(struct mlx4_interface *intf);

#endif /* MLX4_DRIVER_H */
