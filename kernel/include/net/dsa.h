

#ifndef __LINUX_NET_DSA_H
#define __LINUX_NET_DSA_H

#define DSA_MAX_SWITCHES	4
#define DSA_MAX_PORTS		12

struct dsa_chip_data {
	/*
	 * How to access the switch configuration registers.
	 */
	struct device	*mii_bus;
	int		sw_addr;

	/*
	 * The names of the switch's ports.  Use "cpu" to
	 * designate the switch port that the cpu is connected to,
	 * "dsa" to indicate that this port is a DSA link to
	 * another switch, NULL to indicate the port is unused,
	 * or any other string to indicate this is a physical port.
	 */
	char		*port_names[DSA_MAX_PORTS];

	/*
	 * An array (with nr_chips elements) of which element [a]
	 * indicates which port on this switch should be used to
	 * send packets to that are destined for switch a.  Can be
	 * NULL if there is only one switch chip.
	 */
	s8		*rtable;
};

struct dsa_platform_data {
	/*
	 * Reference to a Linux network interface that connects
	 * to the root switch chip of the tree.
	 */
	struct device	*netdev;

	/*
	 * Info structs describing each of the switch chips
	 * connected via this network interface.
	 */
	int		nr_chips;
	struct dsa_chip_data	*chip;
};

extern bool dsa_uses_dsa_tags(void *dsa_ptr);
extern bool dsa_uses_trailer_tags(void *dsa_ptr);


#endif
