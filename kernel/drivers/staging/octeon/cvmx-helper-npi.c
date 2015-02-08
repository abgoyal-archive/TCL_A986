

#include <asm/octeon/octeon.h>

#include "cvmx-config.h"

#include "cvmx-helper.h"

#include "cvmx-pip-defs.h"

int __cvmx_helper_npi_probe(int interface)
{
#if CVMX_PKO_QUEUES_PER_PORT_PCI > 0
	if (OCTEON_IS_MODEL(OCTEON_CN38XX) || OCTEON_IS_MODEL(OCTEON_CN58XX))
		return 4;
	else if (OCTEON_IS_MODEL(OCTEON_CN56XX)
		 && !OCTEON_IS_MODEL(OCTEON_CN56XX_PASS1_X))
		/* The packet engines didn't exist before pass 2 */
		return 4;
	else if (OCTEON_IS_MODEL(OCTEON_CN52XX)
		 && !OCTEON_IS_MODEL(OCTEON_CN52XX_PASS1_X))
		/* The packet engines didn't exist before pass 2 */
		return 4;
#if 0
	/*
	 * Technically CN30XX, CN31XX, and CN50XX contain packet
	 * engines, but nobody ever uses them. Since this is the case,
	 * we disable them here.
	 */
	else if (OCTEON_IS_MODEL(OCTEON_CN31XX)
		 || OCTEON_IS_MODEL(OCTEON_CN50XX))
		return 2;
	else if (OCTEON_IS_MODEL(OCTEON_CN30XX))
		return 1;
#endif
#endif
	return 0;
}

int __cvmx_helper_npi_enable(int interface)
{
	/*
	 * On CN50XX, CN52XX, and CN56XX we need to disable length
	 * checking so packet < 64 bytes and jumbo frames don't get
	 * errors.
	 */
	if (!OCTEON_IS_MODEL(OCTEON_CN3XXX) &&
	    !OCTEON_IS_MODEL(OCTEON_CN58XX)) {
		int num_ports = cvmx_helper_ports_on_interface(interface);
		int port;
		for (port = 0; port < num_ports; port++) {
			union cvmx_pip_prt_cfgx port_cfg;
			int ipd_port =
			    cvmx_helper_get_ipd_port(interface, port);
			port_cfg.u64 =
			    cvmx_read_csr(CVMX_PIP_PRT_CFGX(ipd_port));
			port_cfg.s.maxerr_en = 0;
			port_cfg.s.minerr_en = 0;
			cvmx_write_csr(CVMX_PIP_PRT_CFGX(ipd_port),
				       port_cfg.u64);
		}
	}

	/* Enables are controlled by the remote host, so nothing to do here */
	return 0;
}
