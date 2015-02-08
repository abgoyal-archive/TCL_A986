

#ifndef __ASM_REGOPS_H__
#define __ASM_REGOPS_H__

#include <linux/types.h>

#include <asm/war.h>

#ifndef R10000_LLSC_WAR
#define R10000_LLSC_WAR 0
#endif

#if R10000_LLSC_WAR == 1
#define __beqz	"beqzl	"
#else
#define __beqz	"beqz	"
#endif

#ifndef _LINUX_TYPES_H
typedef unsigned int u32;
#endif

static inline void set_value_reg32(volatile u32 *const addr,
					u32 const mask,
					u32 const value)
{
	u32 temp;

	__asm__ __volatile__(
	"	.set	push				\n"
	"	.set	mips3				\n"
	"1:	ll	%0, %1	# set_value_reg32	\n"
	"	and	%0, %2				\n"
	"	or	%0, %3				\n"
	"	sc	%0, %1				\n"
	"	"__beqz"%0, 1b				\n"
	"	nop					\n"
	"	.set	pop				\n"
	: "=&r" (temp), "=m" (*addr)
	: "ir" (~mask), "ir" (value), "m" (*addr));
}

static inline void set_reg32(volatile u32 *const addr,
				u32 const mask)
{
	u32 temp;

	__asm__ __volatile__(
	"	.set	push				\n"
	"	.set	mips3				\n"
	"1:	ll	%0, %1		# set_reg32	\n"
	"	or	%0, %2				\n"
	"	sc	%0, %1				\n"
	"	"__beqz"%0, 1b				\n"
	"	nop					\n"
	"	.set	pop				\n"
	: "=&r" (temp), "=m" (*addr)
	: "ir" (mask), "m" (*addr));
}

static inline void clear_reg32(volatile u32 *const addr,
				u32 const mask)
{
	u32 temp;

	__asm__ __volatile__(
	"	.set	push				\n"
	"	.set	mips3				\n"
	"1:	ll	%0, %1		# clear_reg32	\n"
	"	and	%0, %2				\n"
	"	sc	%0, %1				\n"
	"	"__beqz"%0, 1b				\n"
	"	nop					\n"
	"	.set	pop				\n"
	: "=&r" (temp), "=m" (*addr)
	: "ir" (~mask), "m" (*addr));
}

static inline void toggle_reg32(volatile u32 *const addr,
				u32 const mask)
{
	u32 temp;

	__asm__ __volatile__(
	"	.set	push				\n"
	"	.set	mips3				\n"
	"1:	ll	%0, %1		# toggle_reg32	\n"
	"	xor	%0, %2				\n"
	"	sc	%0, %1				\n"
	"	"__beqz"%0, 1b				\n"
	"	nop					\n"
	"	.set	pop				\n"
	: "=&r" (temp), "=m" (*addr)
	: "ir" (mask), "m" (*addr));
}

static inline u32 read_reg32(volatile u32 *const addr,
				u32 const mask)
{
	u32 temp;

	__asm__ __volatile__(
	"	.set	push				\n"
	"	.set	noreorder			\n"
	"	lw	%0, %1		# read		\n"
	"	and	%0, %2		# mask		\n"
	"	.set	pop				\n"
	: "=&r" (temp)
	: "m" (*addr), "ir" (mask));

	return temp;
}

static inline u32 blocking_read_reg32(volatile u32 *const addr)
{
	u32 temp;

	__asm__ __volatile__(
	"	.set	push				\n"
	"	.set	noreorder			\n"
	"	lw	%0, %1		# read		\n"
	"	move	%0, %0		# block		\n"
	"	.set	pop				\n"
	: "=&r" (temp)
	: "m" (*addr));

	return temp;
}

#define custom_read_reg32(address, tmp)				\
	__asm__ __volatile__(					\
	"	.set	push				\n"	\
	"	.set	mips3				\n"	\
	"1:	ll	%0, %1	#custom_read_reg32	\n"	\
	"	.set	pop				\n"	\
	: "=r" (tmp), "=m" (*address)				\
	: "m" (*address))

#define custom_write_reg32(address, tmp)			\
	__asm__ __volatile__(					\
	"	.set	push				\n"	\
	"	.set	mips3				\n"	\
	"	sc	%0, %1	#custom_write_reg32	\n"	\
	"	"__beqz"%0, 1b				\n"	\
	"	nop					\n"	\
	"	.set	pop				\n"	\
	: "=&r" (tmp), "=m" (*address)				\
	: "0" (tmp), "m" (*address))

#endif  /* __ASM_REGOPS_H__ */
