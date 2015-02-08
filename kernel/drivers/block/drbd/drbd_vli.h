

#ifndef _DRBD_VLI_H
#define _DRBD_VLI_H





/* fibonacci data 1, 1, ... */
#define VLI_L_1_1() do { \
	LEVEL( 2, 1, 0x00); \
	LEVEL( 3, 2, 0x01); \
	LEVEL( 5, 3, 0x03); \
	LEVEL( 7, 4, 0x07); \
	LEVEL(10, 5, 0x0f); \
	LEVEL(14, 6, 0x1f); \
	LEVEL(21, 8, 0x3f); \
	LEVEL(29, 8, 0x7f); \
	LEVEL(42, 8, 0xbf); \
	LEVEL(64, 8, 0xff); \
	} while (0)

static inline int vli_decode_bits(u64 *out, const u64 in)
{
	u64 adj = 1;

#define LEVEL(t,b,v)					\
	do {						\
		if ((in & ((1 << b) -1)) == v) {	\
			*out = ((in & ((~0ULL) >> (64-t))) >> b) + adj;	\
			return t;			\
		}					\
		adj += 1ULL << (t - b);			\
	} while (0)

	VLI_L_1_1();

	/* NOT REACHED, if VLI_LEVELS code table is defined properly */
	BUG();
#undef LEVEL
}

static inline int __vli_encode_bits(u64 *out, const u64 in)
{
	u64 max = 0;
	u64 adj = 1;

	if (in == 0)
		return -EINVAL;

#define LEVEL(t,b,v) do {		\
		max += 1ULL << (t - b);	\
		if (in <= max) {	\
			if (out)	\
				*out = ((in - adj) << b) | v;	\
			return t;	\
		}			\
		adj = max + 1;		\
	} while (0)

	VLI_L_1_1();

	return -EOVERFLOW;
#undef LEVEL
}

#undef VLI_L_1_1

/* code from here down is independend of actually used bit code */


/* for the bitstream, we need a cursor */
struct bitstream_cursor {
	/* the current byte */
	u8 *b;
	/* the current bit within *b, nomalized: 0..7 */
	unsigned int bit;
};

/* initialize cursor to point to first bit of stream */
static inline void bitstream_cursor_reset(struct bitstream_cursor *cur, void *s)
{
	cur->b = s;
	cur->bit = 0;
}

static inline void bitstream_cursor_advance(struct bitstream_cursor *cur, unsigned int bits)
{
	bits += cur->bit;
	cur->b = cur->b + (bits >> 3);
	cur->bit = bits & 7;
}

/* the bitstream itself knows its length */
struct bitstream {
	struct bitstream_cursor cur;
	unsigned char *buf;
	size_t buf_len;		/* in bytes */

	/* for input stream:
	 * number of trailing 0 bits for padding
	 * total number of valid bits in stream: buf_len * 8 - pad_bits */
	unsigned int pad_bits;
};

static inline void bitstream_init(struct bitstream *bs, void *s, size_t len, unsigned int pad_bits)
{
	bs->buf = s;
	bs->buf_len = len;
	bs->pad_bits = pad_bits;
	bitstream_cursor_reset(&bs->cur, bs->buf);
}

static inline void bitstream_rewind(struct bitstream *bs)
{
	bitstream_cursor_reset(&bs->cur, bs->buf);
	memset(bs->buf, 0, bs->buf_len);
}

static inline int bitstream_put_bits(struct bitstream *bs, u64 val, const unsigned int bits)
{
	unsigned char *b = bs->cur.b;
	unsigned int tmp;

	if (bits == 0)
		return 0;

	if ((bs->cur.b + ((bs->cur.bit + bits -1) >> 3)) - bs->buf >= bs->buf_len)
		return -ENOBUFS;

	/* paranoia: strip off hi bits; they should not be set anyways. */
	if (bits < 64)
		val &= ~0ULL >> (64 - bits);

	*b++ |= (val & 0xff) << bs->cur.bit;

	for (tmp = 8 - bs->cur.bit; tmp < bits; tmp += 8)
		*b++ |= (val >> tmp) & 0xff;

	bitstream_cursor_advance(&bs->cur, bits);
	return bits;
}

static inline int bitstream_get_bits(struct bitstream *bs, u64 *out, int bits)
{
	u64 val;
	unsigned int n;

	if (bits > 64)
		return -EINVAL;

	if (bs->cur.b + ((bs->cur.bit + bs->pad_bits + bits -1) >> 3) - bs->buf >= bs->buf_len)
		bits = ((bs->buf_len - (bs->cur.b - bs->buf)) << 3)
			- bs->cur.bit - bs->pad_bits;

	if (bits == 0) {
		*out = 0;
		return 0;
	}

	/* get the high bits */
	val = 0;
	n = (bs->cur.bit + bits + 7) >> 3;
	/* n may be at most 9, if cur.bit + bits > 64 */
	/* which means this copies at most 8 byte */
	if (n) {
		memcpy(&val, bs->cur.b+1, n - 1);
		val = le64_to_cpu(val) << (8 - bs->cur.bit);
	}

	/* we still need the low bits */
	val |= bs->cur.b[0] >> bs->cur.bit;

	/* and mask out bits we don't want */
	val &= ~0ULL >> (64 - bits);

	bitstream_cursor_advance(&bs->cur, bits);
	*out = val;

	return bits;
}

static inline int vli_encode_bits(struct bitstream *bs, u64 in)
{
	u64 code = code;
	int bits = __vli_encode_bits(&code, in);

	if (bits <= 0)
		return bits;

	return bitstream_put_bits(bs, code, bits);
}

#endif
