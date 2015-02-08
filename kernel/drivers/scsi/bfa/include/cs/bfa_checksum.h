


#ifndef __BFA_CHECKSUM_H__
#define __BFA_CHECKSUM_H__

static inline u32
bfa_checksum_u32(u32 *buf, int sz)
{
	int		i, m = sz >> 2;
	u32	sum = 0;

	for (i = 0; i < m; i++)
		sum ^= buf[i];

	return sum;
}

static inline u16
bfa_checksum_u16(u16 *buf, int sz)
{
	int             i, m = sz >> 1;
	u16        sum = 0;

	for (i = 0; i < m; i++)
		sum ^= buf[i];

	return sum;
}

static inline u8
bfa_checksum_u8(u8 *buf, int sz)
{
	int             i;
	u8         sum = 0;

	for (i = 0; i < sz; i++)
		sum ^= buf[i];

	return sum;
}
#endif
