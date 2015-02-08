

#ifndef __DEVICE_TREE_H__
#define __DEVICE_TREE_H__

void *load_device_tree(const char *filename_path, int *sizep);

int qemu_devtree_setprop(void *fdt, const char *node_path,
                         const char *property, uint32_t *val_array, int size);
int qemu_devtree_setprop_cell(void *fdt, const char *node_path,
                              const char *property, uint32_t val);
int qemu_devtree_setprop_string(void *fdt, const char *node_path,
                                const char *property, const char *string);

#endif /* __DEVICE_TREE_H__ */
