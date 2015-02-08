


#include <asm/octeon/octeon.h>

#include "cvmx-asxx-defs.h"
#include "cvmx-gmxx-defs.h"

#ifndef PRINT_ERROR
#define PRINT_ERROR(format, ...)
#endif

void __cvmx_interrupt_gmxx_rxx_int_en_enable(int index, int block);

void __cvmx_interrupt_asxx_enable(int block)
{
	int mask;
	union cvmx_asxx_int_en csr;
	/*
	 * CN38XX and CN58XX have two interfaces with 4 ports per
	 * interface. All other chips have a max of 3 ports on
	 * interface 0
	 */
	if (OCTEON_IS_MODEL(OCTEON_CN38XX) || OCTEON_IS_MODEL(OCTEON_CN58XX))
		mask = 0xf;	/* Set enables for 4 ports */
	else
		mask = 0x7;	/* Set enables for 3 ports */

	/* Enable interface interrupts */
	csr.u64 = cvmx_read_csr(CVMX_ASXX_INT_EN(block));
	csr.s.txpsh = mask;
	csr.s.txpop = mask;
	csr.s.ovrflw = mask;
	cvmx_write_csr(CVMX_ASXX_INT_EN(block), csr.u64);
}
void __cvmx_interrupt_gmxx_enable(int interface)
{
	union cvmx_gmxx_inf_mode mode;
	union cvmx_gmxx_tx_int_en gmx_tx_int_en;
	int num_ports;
	int index;

	mode.u64 = cvmx_read_csr(CVMX_GMXX_INF_MODE(interface));

	if (OCTEON_IS_MODEL(OCTEON_CN56XX) || OCTEON_IS_MODEL(OCTEON_CN52XX)) {
		if (mode.s.en) {
			switch (mode.cn56xx.mode) {
			case 1:	/* XAUI */
				num_ports = 1;
				break;
			case 2:	/* SGMII */
			case 3:	/* PICMG */
				num_ports = 4;
				break;
			default:	/* Disabled */
				num_ports = 0;
				break;
			}
		} else
			num_ports = 0;
	} else {
		if (mode.s.en) {
			if (OCTEON_IS_MODEL(OCTEON_CN38XX)
			    || OCTEON_IS_MODEL(OCTEON_CN58XX)) {
				/*
				 * SPI on CN38XX and CN58XX report all
				 * errors through port 0.  RGMII needs
				 * to check all 4 ports
				 */
				if (mode.s.type)
					num_ports = 1;
				else
					num_ports = 4;
			} else {
				/*
				 * CN30XX, CN31XX, and CN50XX have two
				 * or three ports. GMII and MII has 2,
				 * RGMII has three
				 */
				if (mode.s.type)
					num_ports = 2;
				else
					num_ports = 3;
			}
		} else
			num_ports = 0;
	}

	gmx_tx_int_en.u64 = 0;
	if (num_ports) {
		if (OCTEON_IS_MODEL(OCTEON_CN38XX)
		    || OCTEON_IS_MODEL(OCTEON_CN58XX))
			gmx_tx_int_en.s.ncb_nxa = 1;
		gmx_tx_int_en.s.pko_nxa = 1;
	}
	gmx_tx_int_en.s.undflw = (1 << num_ports) - 1;
	cvmx_write_csr(CVMX_GMXX_TX_INT_EN(interface), gmx_tx_int_en.u64);
	for (index = 0; index < num_ports; index++)
		__cvmx_interrupt_gmxx_rxx_int_en_enable(index, interface);
}
