
#ifndef	__RTMP_CKIPMIC_H__
#define	__RTMP_CKIPMIC_H__

struct rt_mic_context {
	/* --- MMH context                            */
	u8 CK[16];		/* the key                                    */
	u8 coefficient[16];	/* current aes counter mode coefficients      */
	unsigned long long accum;	/* accumulated mic, reduced to u32 in final() */
	u32 position;		/* current position (byte offset) in message  */
	u8 part[4];		/* for conversion of message to u32 for mmh   */
};

void xor_128(u8 *a, u8 *b, u8 *out);

u8 RTMPCkipSbox(u8 a);

void xor_32(u8 *a, u8 *b, u8 *out);

void next_key(u8 *key, int round);

void byte_sub(u8 *in, u8 *out);

void shift_row(u8 *in, u8 *out);

void mix_column(u8 *in, u8 *out);

#endif /*__RTMP_CKIPMIC_H__ */
