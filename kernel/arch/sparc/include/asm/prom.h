
#include <linux/of.h>	/* linux/of.h gets to determine #include ordering */
#ifndef _SPARC_PROM_H
#define _SPARC_PROM_H
#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/mutex.h>
#include <asm/atomic.h>

#define OF_ROOT_NODE_ADDR_CELLS_DEFAULT	2
#define OF_ROOT_NODE_SIZE_CELLS_DEFAULT	1

#define of_compat_cmp(s1, s2, l)	strncmp((s1), (s2), (l))
#define of_prop_cmp(s1, s2)		strcasecmp((s1), (s2))
#define of_node_cmp(s1, s2)		strcmp((s1), (s2))

struct of_irq_controller {
	unsigned int	(*irq_build)(struct device_node *, unsigned int, void *);
	void		*data;
};

extern struct device_node *of_find_node_by_cpuid(int cpuid);
extern int of_set_property(struct device_node *node, const char *name, void *val, int len);
extern struct mutex of_set_property_mutex;
extern int of_getintprop_default(struct device_node *np,
				 const char *name,
				 int def);
extern int of_find_in_proplist(const char *list, const char *match, int len);
#ifdef CONFIG_NUMA
extern int of_node_to_nid(struct device_node *dp);
#else
#define of_node_to_nid(dp)	(-1)
#endif

extern void prom_build_devicetree(void);
extern void of_populate_present_mask(void);
extern void of_fill_in_cpu_data(void);

extern unsigned int irq_of_parse_and_map(struct device_node *node, int index);
static inline void irq_dispose_mapping(unsigned int virq)
{
}

extern struct device_node *of_console_device;
extern char *of_console_path;
extern char *of_console_options;

extern void (*prom_build_more)(struct device_node *dp, struct device_node ***nextp);
extern char *build_full_name(struct device_node *dp);

#endif /* __KERNEL__ */
#endif /* _SPARC_PROM_H */
