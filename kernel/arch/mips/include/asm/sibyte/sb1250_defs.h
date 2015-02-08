

#ifndef _SB1250_DEFS_H
#define _SB1250_DEFS_H

#if !defined(__STDC__) && !defined(_MSC_VER)
#error SiByte headers require ANSI C89 support
#endif



#define	SIBYTE_HDR_FMASK_1250_ALL		0x000000ff
#define	SIBYTE_HDR_FMASK_1250_PASS1		0x00000001
#define	SIBYTE_HDR_FMASK_1250_PASS2		0x00000002
#define	SIBYTE_HDR_FMASK_1250_PASS3		0x00000004

#define	SIBYTE_HDR_FMASK_112x_ALL		0x00000f00
#define	SIBYTE_HDR_FMASK_112x_PASS1		0x00000100

#define SIBYTE_HDR_FMASK_1480_ALL		0x0000f000
#define SIBYTE_HDR_FMASK_1480_PASS1		0x00001000
#define SIBYTE_HDR_FMASK_1480_PASS2		0x00002000

/* Bit mask for chip/revision.  (use _ALL for all revisions of a chip).  */
#define	SIBYTE_HDR_FMASK(chip, pass)					\
    (SIBYTE_HDR_FMASK_ ## chip ## _ ## pass)
#define	SIBYTE_HDR_FMASK_ALLREVS(chip)					\
    (SIBYTE_HDR_FMASK_ ## chip ## _ALL)

/* Default constant value for all chips, all revisions */
#define	SIBYTE_HDR_FMASK_ALL						\
    (SIBYTE_HDR_FMASK_1250_ALL | SIBYTE_HDR_FMASK_112x_ALL		\
     | SIBYTE_HDR_FMASK_1480_ALL)

#define SIBYTE_HDR_FMASK_1250_112x_ALL					\
    (SIBYTE_HDR_FMASK_1250_ALL | SIBYTE_HDR_FMASK_112x_ALL)
#define SIBYTE_HDR_FMASK_1250_112x SIBYTE_HDR_FMASK_1250_112x_ALL

#ifndef SIBYTE_HDR_FEATURES
#define	SIBYTE_HDR_FEATURES			SIBYTE_HDR_FMASK_ALL
#endif


/* Bit mask for revisions of chip exclusively before the named revision.  */
#define	SIBYTE_HDR_FMASK_BEFORE(chip, pass)				\
    ((SIBYTE_HDR_FMASK(chip, pass) - 1) & SIBYTE_HDR_FMASK_ALLREVS(chip))

/* Bit mask for revisions of chip exclusively after the named revision.  */
#define	SIBYTE_HDR_FMASK_AFTER(chip, pass)				\
    (~(SIBYTE_HDR_FMASK(chip, pass)					\
     | (SIBYTE_HDR_FMASK(chip, pass) - 1)) & SIBYTE_HDR_FMASK_ALLREVS(chip))


/* True if header features enabled for (any revision of) that chip type.  */
#define SIBYTE_HDR_FEATURE_CHIP(chip)					\
    (!! (SIBYTE_HDR_FMASK_ALLREVS(chip) & SIBYTE_HDR_FEATURES))

#define SIBYTE_HDR_FEATURE_1250_112x \
      (SIBYTE_HDR_FEATURE_CHIP(1250) || SIBYTE_HDR_FEATURE_CHIP(112x))
/*    (!!  (SIBYTE_HDR_FEATURES & SIBYHTE_HDR_FMASK_1250_112x)) */

/* True if header features enabled for that rev or later, inclusive.  */
#define SIBYTE_HDR_FEATURE(chip, pass)					\
    (!! ((SIBYTE_HDR_FMASK(chip, pass)					\
	  | SIBYTE_HDR_FMASK_AFTER(chip, pass)) & SIBYTE_HDR_FEATURES))

/* True if header features enabled for exactly that rev.  */
#define SIBYTE_HDR_FEATURE_EXACT(chip, pass)				\
    (!! (SIBYTE_HDR_FMASK(chip, pass) & SIBYTE_HDR_FEATURES))

/* True if header features enabled for that rev or before, inclusive.  */
#define SIBYTE_HDR_FEATURE_UP_TO(chip, pass)				\
    (!! ((SIBYTE_HDR_FMASK(chip, pass)					\
	 | SIBYTE_HDR_FMASK_BEFORE(chip, pass)) & SIBYTE_HDR_FEATURES))







#if !defined(__ASSEMBLY__)
#define _SB_MAKE64(x) ((uint64_t)(x))
#define _SB_MAKE32(x) ((uint32_t)(x))
#else
#define _SB_MAKE64(x) (x)
#define _SB_MAKE32(x) (x)
#endif



#define _SB_MAKEMASK1(n) (_SB_MAKE64(1) << _SB_MAKE64(n))
#define _SB_MAKEMASK1_32(n) (_SB_MAKE32(1) << _SB_MAKE32(n))


#define _SB_MAKEMASK(v, n) (_SB_MAKE64((_SB_MAKE64(1)<<(v))-1) << _SB_MAKE64(n))
#define _SB_MAKEMASK_32(v, n) (_SB_MAKE32((_SB_MAKE32(1)<<(v))-1) << _SB_MAKE32(n))


#define _SB_MAKEVALUE(v, n) (_SB_MAKE64(v) << _SB_MAKE64(n))
#define _SB_MAKEVALUE_32(v, n) (_SB_MAKE32(v) << _SB_MAKE32(n))

#define _SB_GETVALUE(v, n, m) ((_SB_MAKE64(v) & _SB_MAKE64(m)) >> _SB_MAKE64(n))
#define _SB_GETVALUE_32(v, n, m) ((_SB_MAKE32(v) & _SB_MAKE32(m)) >> _SB_MAKE32(n))



#if defined(__mips64) && !defined(__ASSEMBLY__)
#define SBWRITECSR(csr, val) *((volatile uint64_t *) PHYS_TO_K1(csr)) = (val)
#define SBREADCSR(csr) (*((volatile uint64_t *) PHYS_TO_K1(csr)))
#endif /* __ASSEMBLY__ */

#endif
