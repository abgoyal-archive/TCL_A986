

#ifdef __KERNEL__
#ifndef __POWERPC_FSL_PCI_H
#define __POWERPC_FSL_PCI_H

#define PCIE_LTSSM	0x0404		/* PCIE Link Training and Status */
#define PCIE_LTSSM_L0	0x16		/* L0 state */
#define PIWAR_EN		0x80000000	/* Enable */
#define PIWAR_PF		0x20000000	/* prefetch */
#define PIWAR_TGI_LOCAL		0x00f00000	/* target - local memory */
#define PIWAR_READ_SNOOP	0x00050000
#define PIWAR_WRITE_SNOOP	0x00005000

/* PCI/PCI Express outbound window reg */
struct pci_outbound_window_regs {
	__be32	potar;	/* 0x.0 - Outbound translation address register */
	__be32	potear;	/* 0x.4 - Outbound translation extended address register */
	__be32	powbar;	/* 0x.8 - Outbound window base address register */
	u8	res1[4];
	__be32	powar;	/* 0x.10 - Outbound window attributes register */
	u8	res2[12];
};

/* PCI/PCI Express inbound window reg */
struct pci_inbound_window_regs {
	__be32	pitar;	/* 0x.0 - Inbound translation address register */
	u8	res1[4];
	__be32	piwbar;	/* 0x.8 - Inbound window base address register */
	__be32	piwbear;	/* 0x.c - Inbound window base extended address register */
	__be32	piwar;	/* 0x.10 - Inbound window attributes register */
	u8	res2[12];
};

/* PCI/PCI Express IO block registers for 85xx/86xx */
struct ccsr_pci {
	__be32	config_addr;		/* 0x.000 - PCI/PCIE Configuration Address Register */
	__be32	config_data;		/* 0x.004 - PCI/PCIE Configuration Data Register */
	__be32	int_ack;		/* 0x.008 - PCI Interrupt Acknowledge Register */
	__be32	pex_otb_cpl_tor;	/* 0x.00c - PCIE Outbound completion timeout register */
	__be32	pex_conf_tor;		/* 0x.010 - PCIE configuration timeout register */
	u8	res2[12];
	__be32	pex_pme_mes_dr;		/* 0x.020 - PCIE PME and message detect register */
	__be32	pex_pme_mes_disr;	/* 0x.024 - PCIE PME and message disable register */
	__be32	pex_pme_mes_ier;	/* 0x.028 - PCIE PME and message interrupt enable register */
	__be32	pex_pmcr;		/* 0x.02c - PCIE power management command register */
	u8	res3[3024];

	struct pci_outbound_window_regs pow[5];

	u8	res14[256];

	struct pci_inbound_window_regs piw[3];

	__be32	pex_err_dr;		/* 0x.e00 - PCI/PCIE error detect register */
	u8	res21[4];
	__be32	pex_err_en;		/* 0x.e08 - PCI/PCIE error interrupt enable register */
	u8	res22[4];
	__be32	pex_err_disr;		/* 0x.e10 - PCI/PCIE error disable register */
	u8	res23[12];
	__be32	pex_err_cap_stat;	/* 0x.e20 - PCI/PCIE error capture status register */
	u8	res24[4];
	__be32	pex_err_cap_r0;		/* 0x.e28 - PCIE error capture register 0 */
	__be32	pex_err_cap_r1;		/* 0x.e2c - PCIE error capture register 0 */
	__be32	pex_err_cap_r2;		/* 0x.e30 - PCIE error capture register 0 */
	__be32	pex_err_cap_r3;		/* 0x.e34 - PCIE error capture register 0 */
};

extern int fsl_add_bridge(struct device_node *dev, int is_primary);
extern void fsl_pcibios_fixup_bus(struct pci_bus *bus);
extern int mpc83xx_add_bridge(struct device_node *dev);

#endif /* __POWERPC_FSL_PCI_H */
#endif /* __KERNEL__ */
