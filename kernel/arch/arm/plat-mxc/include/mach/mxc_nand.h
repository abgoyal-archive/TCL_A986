

#ifndef __ASM_ARCH_NAND_H
#define __ASM_ARCH_NAND_H

struct mxc_nand_platform_data {
	int width;	/* data bus width in bytes */
	int hw_ecc:1;	/* 0 if supress hardware ECC */
	int flash_bbt:1; /* set to 1 to use a flash based bbt */
};
#endif /* __ASM_ARCH_NAND_H */
