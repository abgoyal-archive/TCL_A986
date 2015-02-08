

#include <asm/octeon/octeon.h>

#include "cvmx-config.h"

#include "cvmx-helper.h"
#include "cvmx-pip-defs.h"

int __cvmx_helper_loop_probe(int interface)
{
	union cvmx_ipd_sub_port_fcs ipd_sub_port_fcs;
	int num_ports = 4;
	int port;

	/* We need to disable length checking so packet < 64 bytes and jumbo
	   frames don't get errors */
	for (port = 0; port < num_ports; port++) {
		union cvmx_pip_prt_cfgx port_cfg;
		int ipd_port = cvmx_helper_get_ipd_port(interface, port);
		port_cfg.u64 = cvmx_read_csr(CVMX_PIP_PRT_CFGX(ipd_port));
		port_cfg.s.maxerr_en = 0;
		port_cfg.s.minerr_en = 0;
		cvmx_write_csr(CVMX_PIP_PRT_CFGX(ipd_port), port_cfg.u64);
	}

	/* Disable FCS stripping for loopback ports */
	ipd_sub_port_fcs.u64 = cvmx_read_csr(CVMX_IPD_SUB_PORT_FCS);
	ipd_sub_port_fcs.s.port_bit2 = 0;
	cvmx_write_csr(CVMX_IPD_SUB_PORT_FCS, ipd_sub_port_fcs.u64);
	return num_ports;
}

int __cvmx_helper_loop_enable(int interface)
{
	/* Do nothing. */
	return 0;
}
