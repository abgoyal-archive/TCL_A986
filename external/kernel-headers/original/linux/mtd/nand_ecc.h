
#ifndef __MTD_NAND_ECC_H__
#define __MTD_NAND_ECC_H__

struct mtd_info;

int nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code);

int nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc);

#endif /* __MTD_NAND_ECC_H__ */
