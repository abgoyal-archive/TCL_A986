

#ifndef __S390_KVM_PARA_H
#define __S390_KVM_PARA_H

#ifdef __KERNEL__


static inline long kvm_hypercall0(unsigned long nr)
{
	register unsigned long __nr asm("1") = nr;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr): "memory", "cc");
	return __rc;
}

static inline long kvm_hypercall1(unsigned long nr, unsigned long p1)
{
	register unsigned long __nr asm("1") = nr;
	register unsigned long __p1 asm("2") = p1;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr), "0" (__p1) : "memory", "cc");
	return __rc;
}

static inline long kvm_hypercall2(unsigned long nr, unsigned long p1,
			       unsigned long p2)
{
	register unsigned long __nr asm("1") = nr;
	register unsigned long __p1 asm("2") = p1;
	register unsigned long __p2 asm("3") = p2;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr), "0" (__p1), "d" (__p2)
		      : "memory", "cc");
	return __rc;
}

static inline long kvm_hypercall3(unsigned long nr, unsigned long p1,
			       unsigned long p2, unsigned long p3)
{
	register unsigned long __nr asm("1") = nr;
	register unsigned long __p1 asm("2") = p1;
	register unsigned long __p2 asm("3") = p2;
	register unsigned long __p3 asm("4") = p3;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr), "0" (__p1), "d" (__p2),
			"d" (__p3) : "memory", "cc");
	return __rc;
}


static inline long kvm_hypercall4(unsigned long nr, unsigned long p1,
			       unsigned long p2, unsigned long p3,
			       unsigned long p4)
{
	register unsigned long __nr asm("1") = nr;
	register unsigned long __p1 asm("2") = p1;
	register unsigned long __p2 asm("3") = p2;
	register unsigned long __p3 asm("4") = p3;
	register unsigned long __p4 asm("5") = p4;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr), "0" (__p1), "d" (__p2),
			"d" (__p3), "d" (__p4) : "memory", "cc");
	return __rc;
}

static inline long kvm_hypercall5(unsigned long nr, unsigned long p1,
			       unsigned long p2, unsigned long p3,
			       unsigned long p4, unsigned long p5)
{
	register unsigned long __nr asm("1") = nr;
	register unsigned long __p1 asm("2") = p1;
	register unsigned long __p2 asm("3") = p2;
	register unsigned long __p3 asm("4") = p3;
	register unsigned long __p4 asm("5") = p4;
	register unsigned long __p5 asm("6") = p5;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr), "0" (__p1), "d" (__p2),
			"d" (__p3), "d" (__p4), "d" (__p5)  : "memory", "cc");
	return __rc;
}

static inline long kvm_hypercall6(unsigned long nr, unsigned long p1,
			       unsigned long p2, unsigned long p3,
			       unsigned long p4, unsigned long p5,
			       unsigned long p6)
{
	register unsigned long __nr asm("1") = nr;
	register unsigned long __p1 asm("2") = p1;
	register unsigned long __p2 asm("3") = p2;
	register unsigned long __p3 asm("4") = p3;
	register unsigned long __p4 asm("5") = p4;
	register unsigned long __p5 asm("6") = p5;
	register unsigned long __p6 asm("7") = p6;
	register long __rc asm("2");

	asm volatile ("diag 2,4,0x500\n"
		      : "=d" (__rc) : "d" (__nr), "0" (__p1), "d" (__p2),
			"d" (__p3), "d" (__p4), "d" (__p5), "d" (__p6)
		      : "memory", "cc");
	return __rc;
}

/* kvm on s390 is always paravirtualization enabled */
static inline int kvm_para_available(void)
{
	return 1;
}

/* No feature bits are currently assigned for kvm on s390 */
static inline unsigned int kvm_arch_para_features(void)
{
	return 0;
}

#endif

#endif /* __S390_KVM_PARA_H */
