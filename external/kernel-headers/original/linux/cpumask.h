#ifndef __LINUX_CPUMASK_H
#define __LINUX_CPUMASK_H


#include <linux/kernel.h>
#include <linux/threads.h>
#include <linux/bitmap.h>

typedef struct { DECLARE_BITMAP(bits, NR_CPUS); } cpumask_t;
extern cpumask_t _unused_cpumask_arg_;

#define cpu_set(cpu, dst) __cpu_set((cpu), &(dst))
static inline void __cpu_set(int cpu, volatile cpumask_t *dstp)
{
	set_bit(cpu, dstp->bits);
}

#define cpu_clear(cpu, dst) __cpu_clear((cpu), &(dst))
static inline void __cpu_clear(int cpu, volatile cpumask_t *dstp)
{
	clear_bit(cpu, dstp->bits);
}

#define cpus_setall(dst) __cpus_setall(&(dst), NR_CPUS)
static inline void __cpus_setall(cpumask_t *dstp, int nbits)
{
	bitmap_fill(dstp->bits, nbits);
}

#define cpus_clear(dst) __cpus_clear(&(dst), NR_CPUS)
static inline void __cpus_clear(cpumask_t *dstp, int nbits)
{
	bitmap_zero(dstp->bits, nbits);
}

/* No static inline type checking - see Subtlety (1) above. */
#define cpu_isset(cpu, cpumask) test_bit((cpu), (cpumask).bits)

#define cpu_test_and_set(cpu, cpumask) __cpu_test_and_set((cpu), &(cpumask))
static inline int __cpu_test_and_set(int cpu, cpumask_t *addr)
{
	return test_and_set_bit(cpu, addr->bits);
}

#define cpus_and(dst, src1, src2) __cpus_and(&(dst), &(src1), &(src2), NR_CPUS)
static inline void __cpus_and(cpumask_t *dstp, const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}

#define cpus_or(dst, src1, src2) __cpus_or(&(dst), &(src1), &(src2), NR_CPUS)
static inline void __cpus_or(cpumask_t *dstp, const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}

#define cpus_xor(dst, src1, src2) __cpus_xor(&(dst), &(src1), &(src2), NR_CPUS)
static inline void __cpus_xor(cpumask_t *dstp, const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}

#define cpus_andnot(dst, src1, src2) \
				__cpus_andnot(&(dst), &(src1), &(src2), NR_CPUS)
static inline void __cpus_andnot(cpumask_t *dstp, const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}

#define cpus_complement(dst, src) __cpus_complement(&(dst), &(src), NR_CPUS)
static inline void __cpus_complement(cpumask_t *dstp,
					const cpumask_t *srcp, int nbits)
{
	bitmap_complement(dstp->bits, srcp->bits, nbits);
}

#define cpus_equal(src1, src2) __cpus_equal(&(src1), &(src2), NR_CPUS)
static inline int __cpus_equal(const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	return bitmap_equal(src1p->bits, src2p->bits, nbits);
}

#define cpus_intersects(src1, src2) __cpus_intersects(&(src1), &(src2), NR_CPUS)
static inline int __cpus_intersects(const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}

#define cpus_subset(src1, src2) __cpus_subset(&(src1), &(src2), NR_CPUS)
static inline int __cpus_subset(const cpumask_t *src1p,
					const cpumask_t *src2p, int nbits)
{
	return bitmap_subset(src1p->bits, src2p->bits, nbits);
}

#define cpus_empty(src) __cpus_empty(&(src), NR_CPUS)
static inline int __cpus_empty(const cpumask_t *srcp, int nbits)
{
	return bitmap_empty(srcp->bits, nbits);
}

#define cpus_full(cpumask) __cpus_full(&(cpumask), NR_CPUS)
static inline int __cpus_full(const cpumask_t *srcp, int nbits)
{
	return bitmap_full(srcp->bits, nbits);
}

#define cpus_weight(cpumask) __cpus_weight(&(cpumask), NR_CPUS)
static inline int __cpus_weight(const cpumask_t *srcp, int nbits)
{
	return bitmap_weight(srcp->bits, nbits);
}

#define cpus_shift_right(dst, src, n) \
			__cpus_shift_right(&(dst), &(src), (n), NR_CPUS)
static inline void __cpus_shift_right(cpumask_t *dstp,
					const cpumask_t *srcp, int n, int nbits)
{
	bitmap_shift_right(dstp->bits, srcp->bits, n, nbits);
}

#define cpus_shift_left(dst, src, n) \
			__cpus_shift_left(&(dst), &(src), (n), NR_CPUS)
static inline void __cpus_shift_left(cpumask_t *dstp,
					const cpumask_t *srcp, int n, int nbits)
{
	bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}

#ifdef CONFIG_SMP
int __first_cpu(const cpumask_t *srcp);
#define first_cpu(src) __first_cpu(&(src))
int __next_cpu(int n, const cpumask_t *srcp);
#define next_cpu(n, src) __next_cpu((n), &(src))
#else
#define first_cpu(src)		0
#define next_cpu(n, src)	1
#endif

#define cpumask_of_cpu(cpu)						\
({									\
	typeof(_unused_cpumask_arg_) m;					\
	if (sizeof(m) == sizeof(unsigned long)) {			\
		m.bits[0] = 1UL<<(cpu);					\
	} else {							\
		cpus_clear(m);						\
		cpu_set((cpu), m);					\
	}								\
	m;								\
})

#define CPU_MASK_LAST_WORD BITMAP_LAST_WORD_MASK(NR_CPUS)

#if NR_CPUS <= BITS_PER_LONG

#define CPU_MASK_ALL							\
(cpumask_t) { {								\
	[BITS_TO_LONGS(NR_CPUS)-1] = CPU_MASK_LAST_WORD			\
} }

#else

#define CPU_MASK_ALL							\
(cpumask_t) { {								\
	[0 ... BITS_TO_LONGS(NR_CPUS)-2] = ~0UL,			\
	[BITS_TO_LONGS(NR_CPUS)-1] = CPU_MASK_LAST_WORD			\
} }

#endif

#define CPU_MASK_NONE							\
(cpumask_t) { {								\
	[0 ... BITS_TO_LONGS(NR_CPUS)-1] =  0UL				\
} }

#define CPU_MASK_CPU0							\
(cpumask_t) { {								\
	[0] =  1UL							\
} }

#define cpus_addr(src) ((src).bits)

#define cpumask_scnprintf(buf, len, src) \
			__cpumask_scnprintf((buf), (len), &(src), NR_CPUS)
static inline int __cpumask_scnprintf(char *buf, int len,
					const cpumask_t *srcp, int nbits)
{
	return bitmap_scnprintf(buf, len, srcp->bits, nbits);
}

#define cpumask_parse(ubuf, ulen, dst) \
			__cpumask_parse((ubuf), (ulen), &(dst), NR_CPUS)
static inline int __cpumask_parse(const char __user *buf, int len,
					cpumask_t *dstp, int nbits)
{
	return bitmap_parse(buf, len, dstp->bits, nbits);
}

#define cpulist_scnprintf(buf, len, src) \
			__cpulist_scnprintf((buf), (len), &(src), NR_CPUS)
static inline int __cpulist_scnprintf(char *buf, int len,
					const cpumask_t *srcp, int nbits)
{
	return bitmap_scnlistprintf(buf, len, srcp->bits, nbits);
}

#define cpulist_parse(buf, dst) __cpulist_parse((buf), &(dst), NR_CPUS)
static inline int __cpulist_parse(const char *buf, cpumask_t *dstp, int nbits)
{
	return bitmap_parselist(buf, dstp->bits, nbits);
}

#define cpu_remap(oldbit, old, new) \
		__cpu_remap((oldbit), &(old), &(new), NR_CPUS)
static inline int __cpu_remap(int oldbit,
		const cpumask_t *oldp, const cpumask_t *newp, int nbits)
{
	return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits);
}

#define cpus_remap(dst, src, old, new) \
		__cpus_remap(&(dst), &(src), &(old), &(new), NR_CPUS)
static inline void __cpus_remap(cpumask_t *dstp, const cpumask_t *srcp,
		const cpumask_t *oldp, const cpumask_t *newp, int nbits)
{
	bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits);
}

#if NR_CPUS > 1
#define for_each_cpu_mask(cpu, mask)		\
	for ((cpu) = first_cpu(mask);		\
		(cpu) < NR_CPUS;		\
		(cpu) = next_cpu((cpu), (mask)))
#else /* NR_CPUS == 1 */
#define for_each_cpu_mask(cpu, mask)		\
	for ((cpu) = 0; (cpu) < 1; (cpu)++, (void)mask)
#endif /* NR_CPUS */


extern cpumask_t cpu_possible_map;
extern cpumask_t cpu_online_map;
extern cpumask_t cpu_present_map;

#if NR_CPUS > 1
#define num_online_cpus()	cpus_weight(cpu_online_map)
#define num_possible_cpus()	cpus_weight(cpu_possible_map)
#define num_present_cpus()	cpus_weight(cpu_present_map)
#define cpu_online(cpu)		cpu_isset((cpu), cpu_online_map)
#define cpu_possible(cpu)	cpu_isset((cpu), cpu_possible_map)
#define cpu_present(cpu)	cpu_isset((cpu), cpu_present_map)
#else
#define num_online_cpus()	1
#define num_possible_cpus()	1
#define num_present_cpus()	1
#define cpu_online(cpu)		((cpu) == 0)
#define cpu_possible(cpu)	((cpu) == 0)
#define cpu_present(cpu)	((cpu) == 0)
#endif

#ifdef CONFIG_SMP
int highest_possible_processor_id(void);
#define any_online_cpu(mask) __any_online_cpu(&(mask))
int __any_online_cpu(const cpumask_t *mask);
#else
#define highest_possible_processor_id()	0
#define any_online_cpu(mask)		0
#endif

#define for_each_possible_cpu(cpu)  for_each_cpu_mask((cpu), cpu_possible_map)
#define for_each_online_cpu(cpu)  for_each_cpu_mask((cpu), cpu_online_map)
#define for_each_present_cpu(cpu) for_each_cpu_mask((cpu), cpu_present_map)

#endif /* __LINUX_CPUMASK_H */
