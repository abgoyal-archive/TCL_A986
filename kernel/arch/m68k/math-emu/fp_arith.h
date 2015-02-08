

#ifndef FP_ARITH_H
#define FP_ARITH_H

/* easy ones */
struct fp_ext *
fp_fabs(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fneg(struct fp_ext *dest, struct fp_ext *src);

/* straightforward arithmetic */
struct fp_ext *
fp_fadd(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fsub(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fcmp(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_ftst(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fmul(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fdiv(struct fp_ext *dest, struct fp_ext *src);

/* ones that do rounding and integer conversions */
struct fp_ext *
fp_fmod(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_frem(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fint(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fintrz(struct fp_ext *dest, struct fp_ext *src);
struct fp_ext *
fp_fscale(struct fp_ext *dest, struct fp_ext *src);

#endif	/* FP_ARITH__H */
