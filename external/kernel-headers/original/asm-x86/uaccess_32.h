#ifndef __i386_UACCESS_H
#define __i386_UACCESS_H

#include <linux/errno.h>
#include <linux/thread_info.h>
#include <linux/prefetch.h>
#include <linux/string.h>
#include <asm/page.h>

#define VERIFY_READ 0
#define VERIFY_WRITE 1


#define MAKE_MM_SEG(s)	((mm_segment_t) { (s) })


#define KERNEL_DS	MAKE_MM_SEG(0xFFFFFFFFUL)
#define USER_DS		MAKE_MM_SEG(PAGE_OFFSET)

#define get_ds()	(KERNEL_DS)
#define get_fs()	(current_thread_info()->addr_limit)
#define set_fs(x)	(current_thread_info()->addr_limit = (x))

#define segment_eq(a,b)	((a).seg == (b).seg)

#ifdef CONFIG_X86_INTEL_USERCOPY
extern struct movsl_mask {
	int mask;
} ____cacheline_aligned_in_smp movsl_mask;
#endif

#define __addr_ok(addr) ((unsigned long __force)(addr) < (current_thread_info()->addr_limit.seg))

#define __range_ok(addr,size) ({ \
	unsigned long flag,roksum; \
	__chk_user_ptr(addr); \
	asm("addl %3,%1 ; sbbl %0,%0; cmpl %1,%4; sbbl $0,%0" \
		:"=&r" (flag), "=r" (roksum) \
		:"1" (addr),"g" ((int)(size)),"rm" (current_thread_info()->addr_limit.seg)); \
	flag; })

#define access_ok(type,addr,size) (likely(__range_ok(addr,size) == 0))


struct exception_table_entry
{
	unsigned long insn, fixup;
};

extern int fixup_exception(struct pt_regs *regs);


extern void __get_user_1(void);
extern void __get_user_2(void);
extern void __get_user_4(void);

#define __get_user_x(size,ret,x,ptr) \
	__asm__ __volatile__("call __get_user_" #size \
		:"=a" (ret),"=d" (x) \
		:"0" (ptr))


/* Careful: we have to cast the result to the type of the pointer for sign reasons */
#define get_user(x,ptr)							\
({	int __ret_gu;							\
	unsigned long __val_gu;						\
	__chk_user_ptr(ptr);						\
	switch(sizeof (*(ptr))) {					\
	case 1:  __get_user_x(1,__ret_gu,__val_gu,ptr); break;		\
	case 2:  __get_user_x(2,__ret_gu,__val_gu,ptr); break;		\
	case 4:  __get_user_x(4,__ret_gu,__val_gu,ptr); break;		\
	default: __get_user_x(X,__ret_gu,__val_gu,ptr); break;		\
	}								\
	(x) = (__typeof__(*(ptr)))__val_gu;				\
	__ret_gu;							\
})

extern void __put_user_bad(void);

extern void __put_user_1(void);
extern void __put_user_2(void);
extern void __put_user_4(void);
extern void __put_user_8(void);

#define __put_user_1(x, ptr) __asm__ __volatile__("call __put_user_1":"=a" (__ret_pu):"0" ((typeof(*(ptr)))(x)), "c" (ptr))
#define __put_user_2(x, ptr) __asm__ __volatile__("call __put_user_2":"=a" (__ret_pu):"0" ((typeof(*(ptr)))(x)), "c" (ptr))
#define __put_user_4(x, ptr) __asm__ __volatile__("call __put_user_4":"=a" (__ret_pu):"0" ((typeof(*(ptr)))(x)), "c" (ptr))
#define __put_user_8(x, ptr) __asm__ __volatile__("call __put_user_8":"=a" (__ret_pu):"A" ((typeof(*(ptr)))(x)), "c" (ptr))
#define __put_user_X(x, ptr) __asm__ __volatile__("call __put_user_X":"=a" (__ret_pu):"c" (ptr))

#ifdef CONFIG_X86_WP_WORKS_OK

#define put_user(x,ptr)						\
({	int __ret_pu;						\
	__typeof__(*(ptr)) __pu_val;				\
	__chk_user_ptr(ptr);					\
	__pu_val = x;						\
	switch(sizeof(*(ptr))) {				\
	case 1: __put_user_1(__pu_val, ptr); break;		\
	case 2: __put_user_2(__pu_val, ptr); break;		\
	case 4: __put_user_4(__pu_val, ptr); break;		\
	case 8: __put_user_8(__pu_val, ptr); break;		\
	default:__put_user_X(__pu_val, ptr); break;		\
	}							\
	__ret_pu;						\
})

#else
#define put_user(x,ptr)						\
({								\
 	int __ret_pu;						\
	__typeof__(*(ptr)) __pus_tmp = x;			\
	__ret_pu=0;						\
	if(unlikely(__copy_to_user_ll(ptr, &__pus_tmp,		\
				sizeof(*(ptr))) != 0))		\
 		__ret_pu=-EFAULT;				\
 	__ret_pu;						\
 })


#endif

#define __get_user(x,ptr) \
  __get_user_nocheck((x),(ptr),sizeof(*(ptr)))


#define __put_user(x,ptr) \
  __put_user_nocheck((__typeof__(*(ptr)))(x),(ptr),sizeof(*(ptr)))

#define __put_user_nocheck(x,ptr,size)				\
({								\
	long __pu_err;						\
	__put_user_size((x),(ptr),(size),__pu_err,-EFAULT);	\
	__pu_err;						\
})


#define __put_user_u64(x, addr, err)				\
	__asm__ __volatile__(					\
		"1:	movl %%eax,0(%2)\n"			\
		"2:	movl %%edx,4(%2)\n"			\
		"3:\n"						\
		".section .fixup,\"ax\"\n"			\
		"4:	movl %3,%0\n"				\
		"	jmp 3b\n"				\
		".previous\n"					\
		".section __ex_table,\"a\"\n"			\
		"	.align 4\n"				\
		"	.long 1b,4b\n"				\
		"	.long 2b,4b\n"				\
		".previous"					\
		: "=r"(err)					\
		: "A" (x), "r" (addr), "i"(-EFAULT), "0"(err))

#ifdef CONFIG_X86_WP_WORKS_OK

#define __put_user_size(x,ptr,size,retval,errret)			\
do {									\
	retval = 0;							\
	__chk_user_ptr(ptr);						\
	switch (size) {							\
	case 1: __put_user_asm(x,ptr,retval,"b","b","iq",errret);break;	\
	case 2: __put_user_asm(x,ptr,retval,"w","w","ir",errret);break; \
	case 4: __put_user_asm(x,ptr,retval,"l","","ir",errret); break;	\
	case 8: __put_user_u64((__typeof__(*ptr))(x),ptr,retval); break;\
	  default: __put_user_bad();					\
	}								\
} while (0)

#else

#define __put_user_size(x,ptr,size,retval,errret)			\
do {									\
	__typeof__(*(ptr)) __pus_tmp = x;				\
	retval = 0;							\
									\
	if(unlikely(__copy_to_user_ll(ptr, &__pus_tmp, size) != 0))	\
		retval = errret;					\
} while (0)

#endif
struct __large_struct { unsigned long buf[100]; };
#define __m(x) (*(struct __large_struct __user *)(x))

#define __put_user_asm(x, addr, err, itype, rtype, ltype, errret)	\
	__asm__ __volatile__(						\
		"1:	mov"itype" %"rtype"1,%2\n"			\
		"2:\n"							\
		".section .fixup,\"ax\"\n"				\
		"3:	movl %3,%0\n"					\
		"	jmp 2b\n"					\
		".previous\n"						\
		".section __ex_table,\"a\"\n"				\
		"	.align 4\n"					\
		"	.long 1b,3b\n"					\
		".previous"						\
		: "=r"(err)						\
		: ltype (x), "m"(__m(addr)), "i"(errret), "0"(err))


#define __get_user_nocheck(x,ptr,size)				\
({								\
	long __gu_err;						\
	unsigned long __gu_val;					\
	__get_user_size(__gu_val,(ptr),(size),__gu_err,-EFAULT);\
	(x) = (__typeof__(*(ptr)))__gu_val;			\
	__gu_err;						\
})

extern long __get_user_bad(void);

#define __get_user_size(x,ptr,size,retval,errret)			\
do {									\
	retval = 0;							\
	__chk_user_ptr(ptr);						\
	switch (size) {							\
	case 1: __get_user_asm(x,ptr,retval,"b","b","=q",errret);break;	\
	case 2: __get_user_asm(x,ptr,retval,"w","w","=r",errret);break;	\
	case 4: __get_user_asm(x,ptr,retval,"l","","=r",errret);break;	\
	default: (x) = __get_user_bad();				\
	}								\
} while (0)

#define __get_user_asm(x, addr, err, itype, rtype, ltype, errret)	\
	__asm__ __volatile__(						\
		"1:	mov"itype" %2,%"rtype"1\n"			\
		"2:\n"							\
		".section .fixup,\"ax\"\n"				\
		"3:	movl %3,%0\n"					\
		"	xor"itype" %"rtype"1,%"rtype"1\n"		\
		"	jmp 2b\n"					\
		".previous\n"						\
		".section __ex_table,\"a\"\n"				\
		"	.align 4\n"					\
		"	.long 1b,3b\n"					\
		".previous"						\
		: "=r"(err), ltype (x)					\
		: "m"(__m(addr)), "i"(errret), "0"(err))


unsigned long __must_check __copy_to_user_ll(void __user *to,
				const void *from, unsigned long n);
unsigned long __must_check __copy_from_user_ll(void *to,
				const void __user *from, unsigned long n);
unsigned long __must_check __copy_from_user_ll_nozero(void *to,
				const void __user *from, unsigned long n);
unsigned long __must_check __copy_from_user_ll_nocache(void *to,
				const void __user *from, unsigned long n);
unsigned long __must_check __copy_from_user_ll_nocache_nozero(void *to,
				const void __user *from, unsigned long n);


static __always_inline unsigned long __must_check
__copy_to_user_inatomic(void __user *to, const void *from, unsigned long n)
{
	if (__builtin_constant_p(n)) {
		unsigned long ret;

		switch (n) {
		case 1:
			__put_user_size(*(u8 *)from, (u8 __user *)to, 1, ret, 1);
			return ret;
		case 2:
			__put_user_size(*(u16 *)from, (u16 __user *)to, 2, ret, 2);
			return ret;
		case 4:
			__put_user_size(*(u32 *)from, (u32 __user *)to, 4, ret, 4);
			return ret;
		}
	}
	return __copy_to_user_ll(to, from, n);
}

static __always_inline unsigned long __must_check
__copy_to_user(void __user *to, const void *from, unsigned long n)
{
       might_sleep();
       return __copy_to_user_inatomic(to, from, n);
}

static __always_inline unsigned long
__copy_from_user_inatomic(void *to, const void __user *from, unsigned long n)
{
	/* Avoid zeroing the tail if the copy fails..
	 * If 'n' is constant and 1, 2, or 4, we do still zero on a failure,
	 * but as the zeroing behaviour is only significant when n is not
	 * constant, that shouldn't be a problem.
	 */
	if (__builtin_constant_p(n)) {
		unsigned long ret;

		switch (n) {
		case 1:
			__get_user_size(*(u8 *)to, from, 1, ret, 1);
			return ret;
		case 2:
			__get_user_size(*(u16 *)to, from, 2, ret, 2);
			return ret;
		case 4:
			__get_user_size(*(u32 *)to, from, 4, ret, 4);
			return ret;
		}
	}
	return __copy_from_user_ll_nozero(to, from, n);
}

static __always_inline unsigned long
__copy_from_user(void *to, const void __user *from, unsigned long n)
{
	might_sleep();
	if (__builtin_constant_p(n)) {
		unsigned long ret;

		switch (n) {
		case 1:
			__get_user_size(*(u8 *)to, from, 1, ret, 1);
			return ret;
		case 2:
			__get_user_size(*(u16 *)to, from, 2, ret, 2);
			return ret;
		case 4:
			__get_user_size(*(u32 *)to, from, 4, ret, 4);
			return ret;
		}
	}
	return __copy_from_user_ll(to, from, n);
}

#define ARCH_HAS_NOCACHE_UACCESS

static __always_inline unsigned long __copy_from_user_nocache(void *to,
				const void __user *from, unsigned long n)
{
	might_sleep();
	if (__builtin_constant_p(n)) {
		unsigned long ret;

		switch (n) {
		case 1:
			__get_user_size(*(u8 *)to, from, 1, ret, 1);
			return ret;
		case 2:
			__get_user_size(*(u16 *)to, from, 2, ret, 2);
			return ret;
		case 4:
			__get_user_size(*(u32 *)to, from, 4, ret, 4);
			return ret;
		}
	}
	return __copy_from_user_ll_nocache(to, from, n);
}

static __always_inline unsigned long
__copy_from_user_inatomic_nocache(void *to, const void __user *from, unsigned long n)
{
       return __copy_from_user_ll_nocache_nozero(to, from, n);
}

unsigned long __must_check copy_to_user(void __user *to,
				const void *from, unsigned long n);
unsigned long __must_check copy_from_user(void *to,
				const void __user *from, unsigned long n);
long __must_check strncpy_from_user(char *dst, const char __user *src,
				long count);
long __must_check __strncpy_from_user(char *dst,
				const char __user *src, long count);

#define strlen_user(str) strnlen_user(str, LONG_MAX)

long strnlen_user(const char __user *str, long n);
unsigned long __must_check clear_user(void __user *mem, unsigned long len);
unsigned long __must_check __clear_user(void __user *mem, unsigned long len);

#endif /* __i386_UACCESS_H */
