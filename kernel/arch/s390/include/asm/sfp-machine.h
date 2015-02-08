

#ifndef _SFP_MACHINE_H
#define _SFP_MACHINE_H
   

#define _FP_W_TYPE_SIZE		32
#define _FP_W_TYPE		unsigned int
#define _FP_WS_TYPE		signed int
#define _FP_I_TYPE		int

#define _FP_MUL_MEAT_S(R,X,Y)					\
  _FP_MUL_MEAT_1_wide(_FP_WFRACBITS_S,R,X,Y,umul_ppmm)
#define _FP_MUL_MEAT_D(R,X,Y)					\
  _FP_MUL_MEAT_2_wide(_FP_WFRACBITS_D,R,X,Y,umul_ppmm)
#define _FP_MUL_MEAT_Q(R,X,Y)					\
  _FP_MUL_MEAT_4_wide(_FP_WFRACBITS_Q,R,X,Y,umul_ppmm)

#define _FP_DIV_MEAT_S(R,X,Y)	_FP_DIV_MEAT_1_udiv(S,R,X,Y)
#define _FP_DIV_MEAT_D(R,X,Y)	_FP_DIV_MEAT_2_udiv(D,R,X,Y)
#define _FP_DIV_MEAT_Q(R,X,Y)	_FP_DIV_MEAT_4_udiv(Q,R,X,Y)

#define _FP_NANFRAC_S		((_FP_QNANBIT_S << 1) - 1)
#define _FP_NANFRAC_D		((_FP_QNANBIT_D << 1) - 1), -1
#define _FP_NANFRAC_Q		((_FP_QNANBIT_Q << 1) - 1), -1, -1, -1
#define _FP_NANSIGN_S		0
#define _FP_NANSIGN_D		0
#define _FP_NANSIGN_Q		0

#define _FP_KEEPNANFRACP 1

#define _FP_CHOOSENAN(fs, wc, R, X, Y, OP)                      \
  do {                                                          \
    if ((_FP_FRAC_HIGH_RAW_##fs(X) & _FP_QNANBIT_##fs)          \
        && !(_FP_FRAC_HIGH_RAW_##fs(Y) & _FP_QNANBIT_##fs))     \
      {                                                         \
        R##_s = Y##_s;                                          \
        _FP_FRAC_COPY_##wc(R,Y);                                \
      }                                                         \
    else                                                        \
      {                                                         \
        R##_s = X##_s;                                          \
        _FP_FRAC_COPY_##wc(R,X);                                \
      }                                                         \
    R##_c = FP_CLS_NAN;                                         \
  } while (0)

/* Some assembly to speed things up. */
#define __FP_FRAC_ADD_3(r2,r1,r0,x2,x1,x0,y2,y1,y0) ({		\
	unsigned int __r2 = (x2) + (y2);			\
	unsigned int __r1 = (x1);				\
	unsigned int __r0 = (x0);				\
	asm volatile(						\
		"	alr	%2,%3\n"			\
		"	brc	12,0f\n"			\
		"	lhi	0,1\n"				\
		"	alr	%1,0\n"				\
		"	brc	12,0f\n"			\
		"	alr	%0,0\n"				\
		"0:"						\
		: "+&d" (__r2), "+&d" (__r1), "+&d" (__r0)	\
		: "d" (y0), "i" (1) : "cc", "0" );		\
	asm volatile(						\
		"	alr	%1,%2\n"			\
		"	brc	12,0f\n"			\
		"	ahi	%0,1\n"				\
		"0:"						\
		: "+&d" (__r2), "+&d" (__r1)			\
		: "d" (y1) : "cc");				\
	(r2) = __r2;						\
	(r1) = __r1;						\
	(r0) = __r0;						\
})

#define __FP_FRAC_SUB_3(r2,r1,r0,x2,x1,x0,y2,y1,y0) ({		\
	unsigned int __r2 = (x2) - (y2);			\
	unsigned int __r1 = (x1);				\
	unsigned int __r0 = (x0);				\
	asm volatile(						\
		"	slr   %2,%3\n"				\
		"	brc	3,0f\n"				\
		"	lhi	0,1\n"				\
		"	slr	%1,0\n"				\
		"	brc	3,0f\n"				\
		"	slr	%0,0\n"				\
		"0:"						\
		: "+&d" (__r2), "+&d" (__r1), "+&d" (__r0)	\
		: "d" (y0) : "cc", "0");			\
	asm volatile(						\
		"	slr	%1,%2\n"			\
		"	brc	3,0f\n"				\
		"	ahi	%0,-1\n"			\
		"0:"						\
		: "+&d" (__r2), "+&d" (__r1)			\
		: "d" (y1) : "cc");				\
	(r2) = __r2;						\
	(r1) = __r1;						\
	(r0) = __r0;						\
})

#define __FP_FRAC_DEC_3(x2,x1,x0,y2,y1,y0) __FP_FRAC_SUB_3(x2,x1,x0,x2,x1,x0,y2,y1,y0)

/* Obtain the current rounding mode. */
#define FP_ROUNDMODE	mode

/* Exception flags. */
#define FP_EX_INVALID		0x800000
#define FP_EX_DIVZERO		0x400000
#define FP_EX_OVERFLOW		0x200000
#define FP_EX_UNDERFLOW		0x100000
#define FP_EX_INEXACT		0x080000

/* We write the results always */
#define FP_INHIBIT_RESULTS 0

#endif
