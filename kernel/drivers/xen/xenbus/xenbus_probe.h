

#ifndef _XENBUS_PROBE_H
#define _XENBUS_PROBE_H

#define XEN_BUS_ID_SIZE			20

#ifdef CONFIG_XEN_BACKEND
extern void xenbus_backend_suspend(int (*fn)(struct device *, void *));
extern void xenbus_backend_resume(int (*fn)(struct device *, void *));
extern void xenbus_backend_probe_and_watch(void);
extern int xenbus_backend_bus_register(void);
extern void xenbus_backend_bus_unregister(void);
#else
static inline void xenbus_backend_suspend(int (*fn)(struct device *, void *)) {}
static inline void xenbus_backend_resume(int (*fn)(struct device *, void *)) {}
static inline void xenbus_backend_probe_and_watch(void) {}
static inline int xenbus_backend_bus_register(void) { return 0; }
static inline void xenbus_backend_bus_unregister(void) {}
#endif

struct xen_bus_type
{
	char *root;
	unsigned int levels;
	int (*get_bus_id)(char bus_id[XEN_BUS_ID_SIZE], const char *nodename);
	int (*probe)(const char *type, const char *dir);
	struct bus_type bus;
};

extern int xenbus_match(struct device *_dev, struct device_driver *_drv);
extern int xenbus_dev_probe(struct device *_dev);
extern int xenbus_dev_remove(struct device *_dev);
extern int xenbus_register_driver_common(struct xenbus_driver *drv,
					 struct xen_bus_type *bus,
					 struct module *owner,
					 const char *mod_name);
extern int xenbus_probe_node(struct xen_bus_type *bus,
			     const char *type,
			     const char *nodename);
extern int xenbus_probe_devices(struct xen_bus_type *bus);

extern void xenbus_dev_changed(const char *node, struct xen_bus_type *bus);

#endif
