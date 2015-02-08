
#ifndef __CMD_H
#define __CMD_H

#include "ar9170.h"

/* basic HW access */
int ar9170_write_mem(struct ar9170 *ar, const __le32 *data, size_t len);
int ar9170_write_reg(struct ar9170 *ar, const u32 reg, const u32 val);
int ar9170_read_reg(struct ar9170 *ar, u32 reg, u32 *val);
int ar9170_read_mreg(struct ar9170 *ar, int nregs, const u32 *regs, u32 *out);
int ar9170_echo_test(struct ar9170 *ar, u32 v);

#define ar9170_regwrite_begin(ar)					\
do {									\
	int __nreg = 0, __err = 0;					\
	struct ar9170 *__ar = ar;

#define ar9170_regwrite(r, v) do {					\
	__ar->cmdbuf[2 * __nreg + 1] = cpu_to_le32(r);			\
	__ar->cmdbuf[2 * __nreg + 2] = cpu_to_le32(v);			\
	__nreg++;							\
	if ((__nreg >= PAYLOAD_MAX/2)) {				\
		if (IS_ACCEPTING_CMD(__ar))				\
			__err = ar->exec_cmd(__ar, AR9170_CMD_WREG,	\
					     8 * __nreg,		\
					     (u8 *) &__ar->cmdbuf[1],	\
					     0, NULL);			\
		__nreg = 0;						\
		if (__err)						\
			goto __regwrite_out;				\
	}								\
} while (0)

#define ar9170_regwrite_finish()					\
__regwrite_out :							\
	if (__nreg) {							\
		if (IS_ACCEPTING_CMD(__ar))				\
			__err = ar->exec_cmd(__ar, AR9170_CMD_WREG,	\
					     8 * __nreg,		\
					     (u8 *) &__ar->cmdbuf[1],	\
					     0, NULL);			\
		__nreg = 0;						\
	}

#define ar9170_regwrite_result()					\
	__err;								\
} while (0);

#endif /* __CMD_H */
