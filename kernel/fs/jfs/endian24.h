
#ifndef _H_ENDIAN24
#define	_H_ENDIAN24

#define __swab24(x) \
({ \
	__u32 __x = (x); \
	((__u32)( \
		((__x & (__u32)0x000000ffUL) << 16) | \
		 (__x & (__u32)0x0000ff00UL)	    | \
		((__x & (__u32)0x00ff0000UL) >> 16) )); \
})

#if (defined(__KERNEL__) && defined(__LITTLE_ENDIAN)) || (defined(__BYTE_ORDER) && (__BYTE_ORDER == __LITTLE_ENDIAN))
	#define __cpu_to_le24(x) ((__u32)(x))
	#define __le24_to_cpu(x) ((__u32)(x))
#else
	#define __cpu_to_le24(x) __swab24(x)
	#define __le24_to_cpu(x) __swab24(x)
#endif

#ifdef __KERNEL__
	#define cpu_to_le24 __cpu_to_le24
	#define le24_to_cpu __le24_to_cpu
#endif

#endif				/* !_H_ENDIAN24 */
