

#ifndef ATMEL_NAND_ECC_H
#define ATMEL_NAND_ECC_H

#define ATMEL_ECC_CR		0x00			/* Control register */
#define		ATMEL_ECC_RST		(1 << 0)		/* Reset parity */

#define ATMEL_ECC_MR		0x04			/* Mode register */
#define		ATMEL_ECC_PAGESIZE	(3 << 0)		/* Page Size */
#define			ATMEL_ECC_PAGESIZE_528		(0)
#define			ATMEL_ECC_PAGESIZE_1056		(1)
#define			ATMEL_ECC_PAGESIZE_2112		(2)
#define			ATMEL_ECC_PAGESIZE_4224		(3)

#define ATMEL_ECC_SR		0x08			/* Status register */
#define		ATMEL_ECC_RECERR		(1 << 0)		/* Recoverable Error */
#define		ATMEL_ECC_ECCERR		(1 << 1)		/* ECC Single Bit Error */
#define		ATMEL_ECC_MULERR		(1 << 2)		/* Multiple Errors */

#define ATMEL_ECC_PR		0x0c			/* Parity register */
#define		ATMEL_ECC_BITADDR	(0xf << 0)		/* Bit Error Address */
#define		ATMEL_ECC_WORDADDR	(0xfff << 4)		/* Word Error Address */

#define ATMEL_ECC_NPR		0x10			/* NParity register */
#define		ATMEL_ECC_NPARITY	(0xffff << 0)		/* NParity */

#endif
