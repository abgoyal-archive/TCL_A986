


#ifndef __CVMX_HELPER_H__
#define __CVMX_HELPER_H__

#include "cvmx-config.h"
#include "cvmx-fpa.h"
#include "cvmx-wqe.h"

typedef enum {
	CVMX_HELPER_INTERFACE_MODE_DISABLED,
	CVMX_HELPER_INTERFACE_MODE_RGMII,
	CVMX_HELPER_INTERFACE_MODE_GMII,
	CVMX_HELPER_INTERFACE_MODE_SPI,
	CVMX_HELPER_INTERFACE_MODE_PCIE,
	CVMX_HELPER_INTERFACE_MODE_XAUI,
	CVMX_HELPER_INTERFACE_MODE_SGMII,
	CVMX_HELPER_INTERFACE_MODE_PICMG,
	CVMX_HELPER_INTERFACE_MODE_NPI,
	CVMX_HELPER_INTERFACE_MODE_LOOP,
} cvmx_helper_interface_mode_t;

typedef union {
	uint64_t u64;
	struct {
		uint64_t reserved_20_63:44;
		uint64_t link_up:1;	    /**< Is the physical link up? */
		uint64_t full_duplex:1;	    /**< 1 if the link is full duplex */
		uint64_t speed:18;	    /**< Speed of the link in Mbps */
	} s;
} cvmx_helper_link_info_t;

#include "cvmx-helper-fpa.h"

#include <asm/octeon/cvmx-helper-errata.h>
#include "cvmx-helper-loop.h"
#include "cvmx-helper-npi.h"
#include "cvmx-helper-rgmii.h"
#include "cvmx-helper-sgmii.h"
#include "cvmx-helper-spi.h"
#include "cvmx-helper-util.h"
#include "cvmx-helper-xaui.h"

extern void (*cvmx_override_pko_queue_priority) (int pko_port,
						 uint64_t priorities[16]);

extern void (*cvmx_override_ipd_port_setup) (int ipd_port);

extern int cvmx_helper_ipd_and_packet_input_enable(void);

extern int cvmx_helper_initialize_packet_io_global(void);

extern int cvmx_helper_initialize_packet_io_local(void);

extern int cvmx_helper_ports_on_interface(int interface);

extern int cvmx_helper_get_number_of_interfaces(void);

extern cvmx_helper_interface_mode_t cvmx_helper_interface_get_mode(int
								   interface);

extern cvmx_helper_link_info_t cvmx_helper_link_autoconf(int ipd_port);

extern cvmx_helper_link_info_t cvmx_helper_link_get(int ipd_port);

extern int cvmx_helper_link_set(int ipd_port,
				cvmx_helper_link_info_t link_info);

extern int cvmx_helper_interface_probe(int interface);

extern int cvmx_helper_configure_loopback(int ipd_port, int enable_internal,
					  int enable_external);

#endif /* __CVMX_HELPER_H__ */
