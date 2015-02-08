


#ifndef	_h_pcicfg_
#define	_h_pcicfg_


#define	PCI_CFG_VID		0
#define	PCI_CFG_CMD		4
#define	PCI_CFG_REV		8
#define	PCI_CFG_BAR0		0x10
#define	PCI_CFG_BAR1		0x14
#define	PCI_BAR0_WIN		0x80	
#define	PCI_INT_STATUS		0x90	
#define	PCI_INT_MASK		0x94	

#define PCIE_EXTCFG_OFFSET	0x100
#define	PCI_BAR0_PCIREGS_OFFSET	(6 * 1024)	
#define	PCI_BAR0_PCISBR_OFFSET	(4 * 1024)	

#define PCI_BAR0_WINSZ		(16 * 1024)	


#define	PCI_16KB0_PCIREGS_OFFSET (8 * 1024)	
#define	PCI_16KB0_CCREGS_OFFSET	(12 * 1024)	
#define PCI_16KBB0_WINSZ	(16 * 1024)	

#endif	
