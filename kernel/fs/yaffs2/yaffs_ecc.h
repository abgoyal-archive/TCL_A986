


#ifndef __YAFFS_ECC_H__
#define __YAFFS_ECC_H__

typedef struct {
	unsigned char colParity;
	unsigned lineParity;
	unsigned lineParityPrime;
} yaffs_ECCOther;

void yaffs_ECCCalculate(const unsigned char *data, unsigned char *ecc);
int yaffs_ECCCorrect(unsigned char *data, unsigned char *read_ecc,
		const unsigned char *test_ecc);

void yaffs_ECCCalculateOther(const unsigned char *data, unsigned nBytes,
			yaffs_ECCOther *ecc);
int yaffs_ECCCorrectOther(unsigned char *data, unsigned nBytes,
			yaffs_ECCOther *read_ecc,
			const yaffs_ECCOther *test_ecc);
#endif
