

#ifndef __MD4_H__
#define __MD4_H__

/* MD4 context. */
typedef	struct	_MD4_CTX_	{
	unsigned long	state[4];        /* state (ABCD) */
	unsigned long	count[2];        /* number of bits, modulo 2^64 (lsb first) */
	u8	buffer[64];      /* input buffer */
}	MD4_CTX;

void MD4Init(MD4_CTX *);
void MD4Update(MD4_CTX *, u8 *, UINT);
void MD4Final(u8 [16], MD4_CTX *);

#endif /*__MD4_H__*/
