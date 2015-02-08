
#ifndef _ASM_XTENSA_UNALIGNED_H
#define _ASM_XTENSA_UNALIGNED_H

#include <asm/byteorder.h>

#ifdef __LITTLE_ENDIAN
# include <linux/unaligned/le_struct.h>
# include <linux/unaligned/be_byteshift.h>
# include <linux/unaligned/generic.h>
# define get_unaligned	__get_unaligned_le
# define put_unaligned	__put_unaligned_le
#else
# include <linux/unaligned/be_struct.h>
# include <linux/unaligned/le_byteshift.h>
# include <linux/unaligned/generic.h>
# define get_unaligned	__get_unaligned_be
# define put_unaligned	__put_unaligned_be
#endif

#endif	/* _ASM_XTENSA_UNALIGNED_H */
