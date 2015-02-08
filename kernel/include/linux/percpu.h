
#ifndef __LINUX_PERCPU_H
#define __LINUX_PERCPU_H

#include <linux/preempt.h>
#include <linux/smp.h>
#include <linux/cpumask.h>
#include <linux/pfn.h>
#include <linux/init.h>

#include <asm/percpu.h>

/* enough to cover all DEFINE_PER_CPUs in modules */
#ifdef CONFIG_MODULES
#define PERCPU_MODULE_RESERVE		(8 << 10)
#else
#define PERCPU_MODULE_RESERVE		0
#endif

#ifndef PERCPU_ENOUGH_ROOM
#define PERCPU_ENOUGH_ROOM						\
	(ALIGN(__per_cpu_end - __per_cpu_start, SMP_CACHE_BYTES) +	\
	 PERCPU_MODULE_RESERVE)
#endif

#define get_cpu_var(var) (*({				\
	preempt_disable();				\
	&__get_cpu_var(var); }))

#define put_cpu_var(var) do {				\
	(void)&(var);					\
	preempt_enable();				\
} while (0)

#ifdef CONFIG_SMP

/* minimum unit size, also is the maximum supported allocation size */
#define PCPU_MIN_UNIT_SIZE		PFN_ALIGN(64 << 10)

#if BITS_PER_LONG > 32
#define PERCPU_DYNAMIC_RESERVE		(20 << 10)
#else
#define PERCPU_DYNAMIC_RESERVE		(12 << 10)
#endif

extern void *pcpu_base_addr;
extern const unsigned long *pcpu_unit_offsets;

struct pcpu_group_info {
	int			nr_units;	/* aligned # of units */
	unsigned long		base_offset;	/* base address offset */
	unsigned int		*cpu_map;	/* unit->cpu map, empty
						 * entries contain NR_CPUS */
};

struct pcpu_alloc_info {
	size_t			static_size;
	size_t			reserved_size;
	size_t			dyn_size;
	size_t			unit_size;
	size_t			atom_size;
	size_t			alloc_size;
	size_t			__ai_size;	/* internal, don't use */
	int			nr_groups;	/* 0 if grouping unnecessary */
	struct pcpu_group_info	groups[];
};

enum pcpu_fc {
	PCPU_FC_AUTO,
	PCPU_FC_EMBED,
	PCPU_FC_PAGE,

	PCPU_FC_NR,
};
extern const char *pcpu_fc_names[PCPU_FC_NR];

extern enum pcpu_fc pcpu_chosen_fc;

typedef void * (*pcpu_fc_alloc_fn_t)(unsigned int cpu, size_t size,
				     size_t align);
typedef void (*pcpu_fc_free_fn_t)(void *ptr, size_t size);
typedef void (*pcpu_fc_populate_pte_fn_t)(unsigned long addr);
typedef int (pcpu_fc_cpu_distance_fn_t)(unsigned int from, unsigned int to);

extern struct pcpu_alloc_info * __init pcpu_alloc_alloc_info(int nr_groups,
							     int nr_units);
extern void __init pcpu_free_alloc_info(struct pcpu_alloc_info *ai);

extern struct pcpu_alloc_info * __init pcpu_build_alloc_info(
				size_t reserved_size, ssize_t dyn_size,
				size_t atom_size,
				pcpu_fc_cpu_distance_fn_t cpu_distance_fn);

extern int __init pcpu_setup_first_chunk(const struct pcpu_alloc_info *ai,
					 void *base_addr);

#ifdef CONFIG_NEED_PER_CPU_EMBED_FIRST_CHUNK
extern int __init pcpu_embed_first_chunk(size_t reserved_size, ssize_t dyn_size,
				size_t atom_size,
				pcpu_fc_cpu_distance_fn_t cpu_distance_fn,
				pcpu_fc_alloc_fn_t alloc_fn,
				pcpu_fc_free_fn_t free_fn);
#endif

#ifdef CONFIG_NEED_PER_CPU_PAGE_FIRST_CHUNK
extern int __init pcpu_page_first_chunk(size_t reserved_size,
				pcpu_fc_alloc_fn_t alloc_fn,
				pcpu_fc_free_fn_t free_fn,
				pcpu_fc_populate_pte_fn_t populate_pte_fn);
#endif

#define per_cpu_ptr(ptr, cpu)	SHIFT_PERCPU_PTR((ptr), per_cpu_offset((cpu)))

extern void __percpu *__alloc_reserved_percpu(size_t size, size_t align);
extern bool is_kernel_percpu_address(unsigned long addr);

#ifndef CONFIG_HAVE_SETUP_PER_CPU_AREA
extern void __init setup_per_cpu_areas(void);
#endif

#else /* CONFIG_SMP */

#define per_cpu_ptr(ptr, cpu) ({ (void)(cpu); (ptr); })

/* can't distinguish from other static vars, always false */
static inline bool is_kernel_percpu_address(unsigned long addr)
{
	return false;
}

static inline void __init setup_per_cpu_areas(void) { }

static inline void *pcpu_lpage_remapped(void *kaddr)
{
	return NULL;
}

#endif /* CONFIG_SMP */

extern void __percpu *__alloc_percpu(size_t size, size_t align);
extern void free_percpu(void __percpu *__pdata);
extern phys_addr_t per_cpu_ptr_to_phys(void *addr);

#define alloc_percpu(type)	\
	(typeof(type) __percpu *)__alloc_percpu(sizeof(type), __alignof__(type))

#ifndef percpu_read
# define percpu_read(var)						\
  ({									\
	typeof(var) *pr_ptr__ = &(var);					\
	typeof(var) pr_ret__;						\
	pr_ret__ = get_cpu_var(*pr_ptr__);				\
	put_cpu_var(*pr_ptr__);						\
	pr_ret__;							\
  })
#endif

#define __percpu_generic_to_op(var, val, op)				\
do {									\
	typeof(var) *pgto_ptr__ = &(var);				\
	get_cpu_var(*pgto_ptr__) op val;				\
	put_cpu_var(*pgto_ptr__);					\
} while (0)

#ifndef percpu_write
# define percpu_write(var, val)		__percpu_generic_to_op(var, (val), =)
#endif

#ifndef percpu_add
# define percpu_add(var, val)		__percpu_generic_to_op(var, (val), +=)
#endif

#ifndef percpu_sub
# define percpu_sub(var, val)		__percpu_generic_to_op(var, (val), -=)
#endif

#ifndef percpu_and
# define percpu_and(var, val)		__percpu_generic_to_op(var, (val), &=)
#endif

#ifndef percpu_or
# define percpu_or(var, val)		__percpu_generic_to_op(var, (val), |=)
#endif

#ifndef percpu_xor
# define percpu_xor(var, val)		__percpu_generic_to_op(var, (val), ^=)
#endif


extern void __bad_size_call_parameter(void);

#define __pcpu_size_call_return(stem, variable)				\
({	typeof(variable) pscr_ret__;					\
	__verify_pcpu_ptr(&(variable));					\
	switch(sizeof(variable)) {					\
	case 1: pscr_ret__ = stem##1(variable);break;			\
	case 2: pscr_ret__ = stem##2(variable);break;			\
	case 4: pscr_ret__ = stem##4(variable);break;			\
	case 8: pscr_ret__ = stem##8(variable);break;			\
	default:							\
		__bad_size_call_parameter();break;			\
	}								\
	pscr_ret__;							\
})

#define __pcpu_size_call(stem, variable, ...)				\
do {									\
	__verify_pcpu_ptr(&(variable));					\
	switch(sizeof(variable)) {					\
		case 1: stem##1(variable, __VA_ARGS__);break;		\
		case 2: stem##2(variable, __VA_ARGS__);break;		\
		case 4: stem##4(variable, __VA_ARGS__);break;		\
		case 8: stem##8(variable, __VA_ARGS__);break;		\
		default: 						\
			__bad_size_call_parameter();break;		\
	}								\
} while (0)


#define _this_cpu_generic_read(pcp)					\
({	typeof(pcp) ret__;						\
	preempt_disable();						\
	ret__ = *this_cpu_ptr(&(pcp));					\
	preempt_enable();						\
	ret__;								\
})

#ifndef this_cpu_read
# ifndef this_cpu_read_1
#  define this_cpu_read_1(pcp)	_this_cpu_generic_read(pcp)
# endif
# ifndef this_cpu_read_2
#  define this_cpu_read_2(pcp)	_this_cpu_generic_read(pcp)
# endif
# ifndef this_cpu_read_4
#  define this_cpu_read_4(pcp)	_this_cpu_generic_read(pcp)
# endif
# ifndef this_cpu_read_8
#  define this_cpu_read_8(pcp)	_this_cpu_generic_read(pcp)
# endif
# define this_cpu_read(pcp)	__pcpu_size_call_return(this_cpu_read_, (pcp))
#endif

#define _this_cpu_generic_to_op(pcp, val, op)				\
do {									\
	preempt_disable();						\
	*__this_cpu_ptr(&(pcp)) op val;					\
	preempt_enable();						\
} while (0)

#ifndef this_cpu_write
# ifndef this_cpu_write_1
#  define this_cpu_write_1(pcp, val)	_this_cpu_generic_to_op((pcp), (val), =)
# endif
# ifndef this_cpu_write_2
#  define this_cpu_write_2(pcp, val)	_this_cpu_generic_to_op((pcp), (val), =)
# endif
# ifndef this_cpu_write_4
#  define this_cpu_write_4(pcp, val)	_this_cpu_generic_to_op((pcp), (val), =)
# endif
# ifndef this_cpu_write_8
#  define this_cpu_write_8(pcp, val)	_this_cpu_generic_to_op((pcp), (val), =)
# endif
# define this_cpu_write(pcp, val)	__pcpu_size_call(this_cpu_write_, (pcp), (val))
#endif

#ifndef this_cpu_add
# ifndef this_cpu_add_1
#  define this_cpu_add_1(pcp, val)	_this_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef this_cpu_add_2
#  define this_cpu_add_2(pcp, val)	_this_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef this_cpu_add_4
#  define this_cpu_add_4(pcp, val)	_this_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef this_cpu_add_8
#  define this_cpu_add_8(pcp, val)	_this_cpu_generic_to_op((pcp), (val), +=)
# endif
# define this_cpu_add(pcp, val)		__pcpu_size_call(this_cpu_add_, (pcp), (val))
#endif

#ifndef this_cpu_sub
# define this_cpu_sub(pcp, val)		this_cpu_add((pcp), -(val))
#endif

#ifndef this_cpu_inc
# define this_cpu_inc(pcp)		this_cpu_add((pcp), 1)
#endif

#ifndef this_cpu_dec
# define this_cpu_dec(pcp)		this_cpu_sub((pcp), 1)
#endif

#ifndef this_cpu_and
# ifndef this_cpu_and_1
#  define this_cpu_and_1(pcp, val)	_this_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef this_cpu_and_2
#  define this_cpu_and_2(pcp, val)	_this_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef this_cpu_and_4
#  define this_cpu_and_4(pcp, val)	_this_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef this_cpu_and_8
#  define this_cpu_and_8(pcp, val)	_this_cpu_generic_to_op((pcp), (val), &=)
# endif
# define this_cpu_and(pcp, val)		__pcpu_size_call(this_cpu_and_, (pcp), (val))
#endif

#ifndef this_cpu_or
# ifndef this_cpu_or_1
#  define this_cpu_or_1(pcp, val)	_this_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef this_cpu_or_2
#  define this_cpu_or_2(pcp, val)	_this_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef this_cpu_or_4
#  define this_cpu_or_4(pcp, val)	_this_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef this_cpu_or_8
#  define this_cpu_or_8(pcp, val)	_this_cpu_generic_to_op((pcp), (val), |=)
# endif
# define this_cpu_or(pcp, val)		__pcpu_size_call(this_cpu_or_, (pcp), (val))
#endif

#ifndef this_cpu_xor
# ifndef this_cpu_xor_1
#  define this_cpu_xor_1(pcp, val)	_this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef this_cpu_xor_2
#  define this_cpu_xor_2(pcp, val)	_this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef this_cpu_xor_4
#  define this_cpu_xor_4(pcp, val)	_this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef this_cpu_xor_8
#  define this_cpu_xor_8(pcp, val)	_this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# define this_cpu_xor(pcp, val)		__pcpu_size_call(this_cpu_or_, (pcp), (val))
#endif

#ifndef __this_cpu_read
# ifndef __this_cpu_read_1
#  define __this_cpu_read_1(pcp)	(*__this_cpu_ptr(&(pcp)))
# endif
# ifndef __this_cpu_read_2
#  define __this_cpu_read_2(pcp)	(*__this_cpu_ptr(&(pcp)))
# endif
# ifndef __this_cpu_read_4
#  define __this_cpu_read_4(pcp)	(*__this_cpu_ptr(&(pcp)))
# endif
# ifndef __this_cpu_read_8
#  define __this_cpu_read_8(pcp)	(*__this_cpu_ptr(&(pcp)))
# endif
# define __this_cpu_read(pcp)	__pcpu_size_call_return(__this_cpu_read_, (pcp))
#endif

#define __this_cpu_generic_to_op(pcp, val, op)				\
do {									\
	*__this_cpu_ptr(&(pcp)) op val;					\
} while (0)

#ifndef __this_cpu_write
# ifndef __this_cpu_write_1
#  define __this_cpu_write_1(pcp, val)	__this_cpu_generic_to_op((pcp), (val), =)
# endif
# ifndef __this_cpu_write_2
#  define __this_cpu_write_2(pcp, val)	__this_cpu_generic_to_op((pcp), (val), =)
# endif
# ifndef __this_cpu_write_4
#  define __this_cpu_write_4(pcp, val)	__this_cpu_generic_to_op((pcp), (val), =)
# endif
# ifndef __this_cpu_write_8
#  define __this_cpu_write_8(pcp, val)	__this_cpu_generic_to_op((pcp), (val), =)
# endif
# define __this_cpu_write(pcp, val)	__pcpu_size_call(__this_cpu_write_, (pcp), (val))
#endif

#ifndef __this_cpu_add
# ifndef __this_cpu_add_1
#  define __this_cpu_add_1(pcp, val)	__this_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef __this_cpu_add_2
#  define __this_cpu_add_2(pcp, val)	__this_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef __this_cpu_add_4
#  define __this_cpu_add_4(pcp, val)	__this_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef __this_cpu_add_8
#  define __this_cpu_add_8(pcp, val)	__this_cpu_generic_to_op((pcp), (val), +=)
# endif
# define __this_cpu_add(pcp, val)	__pcpu_size_call(__this_cpu_add_, (pcp), (val))
#endif

#ifndef __this_cpu_sub
# define __this_cpu_sub(pcp, val)	__this_cpu_add((pcp), -(val))
#endif

#ifndef __this_cpu_inc
# define __this_cpu_inc(pcp)		__this_cpu_add((pcp), 1)
#endif

#ifndef __this_cpu_dec
# define __this_cpu_dec(pcp)		__this_cpu_sub((pcp), 1)
#endif

#ifndef __this_cpu_and
# ifndef __this_cpu_and_1
#  define __this_cpu_and_1(pcp, val)	__this_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef __this_cpu_and_2
#  define __this_cpu_and_2(pcp, val)	__this_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef __this_cpu_and_4
#  define __this_cpu_and_4(pcp, val)	__this_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef __this_cpu_and_8
#  define __this_cpu_and_8(pcp, val)	__this_cpu_generic_to_op((pcp), (val), &=)
# endif
# define __this_cpu_and(pcp, val)	__pcpu_size_call(__this_cpu_and_, (pcp), (val))
#endif

#ifndef __this_cpu_or
# ifndef __this_cpu_or_1
#  define __this_cpu_or_1(pcp, val)	__this_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef __this_cpu_or_2
#  define __this_cpu_or_2(pcp, val)	__this_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef __this_cpu_or_4
#  define __this_cpu_or_4(pcp, val)	__this_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef __this_cpu_or_8
#  define __this_cpu_or_8(pcp, val)	__this_cpu_generic_to_op((pcp), (val), |=)
# endif
# define __this_cpu_or(pcp, val)	__pcpu_size_call(__this_cpu_or_, (pcp), (val))
#endif

#ifndef __this_cpu_xor
# ifndef __this_cpu_xor_1
#  define __this_cpu_xor_1(pcp, val)	__this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef __this_cpu_xor_2
#  define __this_cpu_xor_2(pcp, val)	__this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef __this_cpu_xor_4
#  define __this_cpu_xor_4(pcp, val)	__this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef __this_cpu_xor_8
#  define __this_cpu_xor_8(pcp, val)	__this_cpu_generic_to_op((pcp), (val), ^=)
# endif
# define __this_cpu_xor(pcp, val)	__pcpu_size_call(__this_cpu_xor_, (pcp), (val))
#endif

#define irqsafe_cpu_generic_to_op(pcp, val, op)				\
do {									\
	unsigned long flags;						\
	local_irq_save(flags);						\
	*__this_cpu_ptr(&(pcp)) op val;					\
	local_irq_restore(flags);					\
} while (0)

#ifndef irqsafe_cpu_add
# ifndef irqsafe_cpu_add_1
#  define irqsafe_cpu_add_1(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef irqsafe_cpu_add_2
#  define irqsafe_cpu_add_2(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef irqsafe_cpu_add_4
#  define irqsafe_cpu_add_4(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), +=)
# endif
# ifndef irqsafe_cpu_add_8
#  define irqsafe_cpu_add_8(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), +=)
# endif
# define irqsafe_cpu_add(pcp, val) __pcpu_size_call(irqsafe_cpu_add_, (pcp), (val))
#endif

#ifndef irqsafe_cpu_sub
# define irqsafe_cpu_sub(pcp, val)	irqsafe_cpu_add((pcp), -(val))
#endif

#ifndef irqsafe_cpu_inc
# define irqsafe_cpu_inc(pcp)	irqsafe_cpu_add((pcp), 1)
#endif

#ifndef irqsafe_cpu_dec
# define irqsafe_cpu_dec(pcp)	irqsafe_cpu_sub((pcp), 1)
#endif

#ifndef irqsafe_cpu_and
# ifndef irqsafe_cpu_and_1
#  define irqsafe_cpu_and_1(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef irqsafe_cpu_and_2
#  define irqsafe_cpu_and_2(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef irqsafe_cpu_and_4
#  define irqsafe_cpu_and_4(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), &=)
# endif
# ifndef irqsafe_cpu_and_8
#  define irqsafe_cpu_and_8(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), &=)
# endif
# define irqsafe_cpu_and(pcp, val) __pcpu_size_call(irqsafe_cpu_and_, (val))
#endif

#ifndef irqsafe_cpu_or
# ifndef irqsafe_cpu_or_1
#  define irqsafe_cpu_or_1(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef irqsafe_cpu_or_2
#  define irqsafe_cpu_or_2(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef irqsafe_cpu_or_4
#  define irqsafe_cpu_or_4(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), |=)
# endif
# ifndef irqsafe_cpu_or_8
#  define irqsafe_cpu_or_8(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), |=)
# endif
# define irqsafe_cpu_or(pcp, val) __pcpu_size_call(irqsafe_cpu_or_, (val))
#endif

#ifndef irqsafe_cpu_xor
# ifndef irqsafe_cpu_xor_1
#  define irqsafe_cpu_xor_1(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef irqsafe_cpu_xor_2
#  define irqsafe_cpu_xor_2(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef irqsafe_cpu_xor_4
#  define irqsafe_cpu_xor_4(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), ^=)
# endif
# ifndef irqsafe_cpu_xor_8
#  define irqsafe_cpu_xor_8(pcp, val) irqsafe_cpu_generic_to_op((pcp), (val), ^=)
# endif
# define irqsafe_cpu_xor(pcp, val) __pcpu_size_call(irqsafe_cpu_xor_, (val))
#endif

#endif /* __LINUX_PERCPU_H */
