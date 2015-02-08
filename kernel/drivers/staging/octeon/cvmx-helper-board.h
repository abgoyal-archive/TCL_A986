

#ifndef __CVMX_HELPER_BOARD_H__
#define __CVMX_HELPER_BOARD_H__

#include "cvmx-helper.h"

typedef enum {
	USB_CLOCK_TYPE_REF_12,
	USB_CLOCK_TYPE_REF_24,
	USB_CLOCK_TYPE_REF_48,
	USB_CLOCK_TYPE_CRYSTAL_12,
} cvmx_helper_board_usb_clock_types_t;

typedef enum {
	set_phy_link_flags_autoneg = 0x1,
	set_phy_link_flags_flow_control_dont_touch = 0x0 << 1,
	set_phy_link_flags_flow_control_enable = 0x1 << 1,
	set_phy_link_flags_flow_control_disable = 0x2 << 1,
	set_phy_link_flags_flow_control_mask = 0x3 << 1,	/* Mask for 2 bit wide flow control field */
} cvmx_helper_board_set_phy_link_flags_types_t;

extern cvmx_helper_link_info_t(*cvmx_override_board_link_get) (int ipd_port);

extern int cvmx_helper_board_get_mii_address(int ipd_port);

int cvmx_helper_board_link_set_phy(int phy_addr,
				   cvmx_helper_board_set_phy_link_flags_types_t
				   link_flags,
				   cvmx_helper_link_info_t link_info);

extern cvmx_helper_link_info_t __cvmx_helper_board_link_get(int ipd_port);

extern int __cvmx_helper_board_interface_probe(int interface,
					       int supported_ports);

extern int __cvmx_helper_board_hardware_enable(int interface);

cvmx_helper_board_usb_clock_types_t
__cvmx_helper_board_usb_get_clock_type(void);

int __cvmx_helper_board_usb_get_num_ports(int supported_ports);

#endif /* __CVMX_HELPER_BOARD_H__ */
