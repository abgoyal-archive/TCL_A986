

#ifdef LBL_ALIGN
#ifdef HAVE___ATTRIBUTE__
typedef struct {
	u_int16_t	val;
} __attribute__((packed)) unaligned_u_int16_t;

typedef struct {
	u_int32_t	val;
} __attribute__((packed)) unaligned_u_int32_t;

#define EXTRACT_16BITS(p) \
	((u_int16_t)ntohs(((const unaligned_u_int16_t *)(p))->val))
#define EXTRACT_32BITS(p) \
	((u_int32_t)ntohl(((const unaligned_u_int32_t *)(p))->val))
#define EXTRACT_64BITS(p) \
	((u_int64_t)(((u_int64_t)ntohl(((const unaligned_u_int32_t *)(p) + 0)->val)) << 32 | \
		     ((u_int64_t)ntohl(((const unaligned_u_int32_t *)(p) + 1)->val)) << 0))

#else /* HAVE___ATTRIBUTE__ */
#define EXTRACT_16BITS(p) \
	((u_int16_t)((u_int16_t)*((const u_int8_t *)(p) + 0) << 8 | \
		     (u_int16_t)*((const u_int8_t *)(p) + 1)))
#define EXTRACT_32BITS(p) \
	((u_int32_t)((u_int32_t)*((const u_int8_t *)(p) + 0) << 24 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 1) << 16 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 2) << 8 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 3)))
#define EXTRACT_64BITS(p) \
	((u_int64_t)((u_int64_t)*((const u_int8_t *)(p) + 0) << 56 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 1) << 48 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 2) << 40 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 3) << 32 | \
	             (u_int64_t)*((const u_int8_t *)(p) + 4) << 24 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 5) << 16 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 6) << 8 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 7)))
#endif /* HAVE___ATTRIBUTE__ */
#else /* LBL_ALIGN */
#define EXTRACT_16BITS(p) \
	((u_int16_t)ntohs(*(const u_int16_t *)(p)))
#define EXTRACT_32BITS(p) \
	((u_int32_t)ntohl(*(const u_int32_t *)(p)))
#define EXTRACT_64BITS(p) \
	((u_int64_t)(((u_int64_t)ntohl(*((const u_int32_t *)(p) + 0))) << 32 | \
		     ((u_int64_t)ntohl(*((const u_int32_t *)(p) + 1))) << 0))
#endif /* LBL_ALIGN */

#define EXTRACT_24BITS(p) \
	((u_int32_t)((u_int32_t)*((const u_int8_t *)(p) + 0) << 16 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 1) << 8 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 2)))

#define EXTRACT_LE_8BITS(p) (*(p))
#define EXTRACT_LE_16BITS(p) \
	((u_int16_t)((u_int16_t)*((const u_int8_t *)(p) + 1) << 8 | \
		     (u_int16_t)*((const u_int8_t *)(p) + 0)))
#define EXTRACT_LE_32BITS(p) \
	((u_int32_t)((u_int32_t)*((const u_int8_t *)(p) + 3) << 24 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 2) << 16 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 1) << 8 | \
		     (u_int32_t)*((const u_int8_t *)(p) + 0)))
#define EXTRACT_LE_64BITS(p) \
	((u_int64_t)((u_int64_t)*((const u_int8_t *)(p) + 7) << 56 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 6) << 48 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 5) << 40 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 4) << 32 | \
	             (u_int64_t)*((const u_int8_t *)(p) + 3) << 24 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 2) << 16 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 1) << 8 | \
		     (u_int64_t)*((const u_int8_t *)(p) + 0)))
