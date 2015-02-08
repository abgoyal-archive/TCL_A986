
 /*
  * This code implements the ECC algorithm used in SmartMedia.
  *
  * The ECC comprises 22 bits of parity information and is stuffed into 3 bytes. 
  * The two unused bit are set to 1.
  * The ECC can correct single bit errors in a 256-byte page of data. Thus, two such ECC 
  * blocks are used on a 512-byte NAND page.
  *
  */

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
			     yaffs_ECCOther * ecc);
int yaffs_ECCCorrectOther(unsigned char *data, unsigned nBytes,
			  yaffs_ECCOther * read_ecc,
			  const yaffs_ECCOther * test_ecc);
#endif
