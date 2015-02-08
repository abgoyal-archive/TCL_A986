

#ifndef __W1_INT_H
#define __W1_INT_H

#include <linux/kernel.h>
#include <linux/device.h>

#include "w1.h"

int w1_add_master_device(struct w1_bus_master *);
void w1_remove_master_device(struct w1_bus_master *);
void __w1_remove_master_device(struct w1_master *);

#endif /* __W1_INT_H */
