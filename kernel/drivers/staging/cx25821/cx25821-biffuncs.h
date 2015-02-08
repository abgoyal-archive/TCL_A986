

#ifndef _BITFUNCS_H
#define _BITFUNCS_H

#define SetBit(Bit)  (1 << Bit)

inline u8 getBit(u32 sample, u8 index)
{
	return (u8) ((sample >> index) & 1);
}

inline u32 clearBitAtPos(u32 value, u8 bit)
{
	return value & ~(1 << bit);
}

inline u32 setBitAtPos(u32 sample, u8 bit)
{
	sample |= (1 << bit);
	return sample;

}

#endif
