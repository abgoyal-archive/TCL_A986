
#ifndef _ASM_IA64_RSE_H
#define _ASM_IA64_RSE_H


static __inline__ unsigned long
ia64_rse_slot_num (unsigned long *addr)
{
	return (((unsigned long) addr) >> 3) & 0x3f;
}

static __inline__ unsigned long
ia64_rse_is_rnat_slot (unsigned long *addr)
{
	return ia64_rse_slot_num(addr) == 0x3f;
}

static __inline__ unsigned long *
ia64_rse_rnat_addr (unsigned long *slot_addr)
{
	return (unsigned long *) ((unsigned long) slot_addr | (0x3f << 3));
}

static __inline__ unsigned long
ia64_rse_num_regs (unsigned long *bspstore, unsigned long *bsp)
{
	unsigned long slots = (bsp - bspstore);

	return slots - (ia64_rse_slot_num(bspstore) + slots)/0x40;
}

static __inline__ unsigned long *
ia64_rse_skip_regs (unsigned long *addr, long num_regs)
{
	long delta = ia64_rse_slot_num(addr) + num_regs;

	if (num_regs < 0)
		delta -= 0x3e;
	return addr + num_regs + delta/0x3f;
}

#endif /* _ASM_IA64_RSE_H */
