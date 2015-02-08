#ifndef _LINUX_NODE_H_
#define _LINUX_NODE_H_

#include <linux/sysdev.h>
#include <linux/cpumask.h>

struct node {
	struct sys_device	sysdev;
};

extern struct node node_devices[];

extern int register_node(struct node *, int, struct node *);
extern void unregister_node(struct node *node);
#ifdef CONFIG_NUMA
extern int register_one_node(int nid);
extern void unregister_one_node(int nid);
extern int register_cpu_under_node(unsigned int cpu, unsigned int nid);
extern int unregister_cpu_under_node(unsigned int cpu, unsigned int nid);
#else
static inline int register_one_node(int nid)
{
	return 0;
}
static inline int unregister_one_node(int nid)
{
	return 0;
}
static inline int register_cpu_under_node(unsigned int cpu, unsigned int nid)
{
	return 0;
}
static inline int unregister_cpu_under_node(unsigned int cpu, unsigned int nid)
{
	return 0;
}
#endif

#define to_node(sys_device) container_of(sys_device, struct node, sysdev)

#endif /* _LINUX_NODE_H_ */
