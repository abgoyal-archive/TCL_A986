

#include <linux/rwsem.h>

struct i2c_devinfo {
	struct list_head	list;
	int			busnum;
	struct i2c_board_info	board_info;
};

extern struct rw_semaphore	__i2c_board_lock;
extern struct list_head	__i2c_board_list;
extern int		__i2c_first_dynamic_bus_num;

