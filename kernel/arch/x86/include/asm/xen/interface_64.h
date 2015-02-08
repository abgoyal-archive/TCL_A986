
#ifndef _ASM_X86_XEN_INTERFACE_64_H
#define _ASM_X86_XEN_INTERFACE_64_H


#define FLAT_RING3_CS32 0xe023  /* GDT index 260 */
#define FLAT_RING3_CS64 0xe033  /* GDT index 261 */
#define FLAT_RING3_DS32 0xe02b  /* GDT index 262 */
#define FLAT_RING3_DS64 0x0000  /* NULL selector */
#define FLAT_RING3_SS32 0xe02b  /* GDT index 262 */
#define FLAT_RING3_SS64 0xe02b  /* GDT index 262 */

#define FLAT_KERNEL_DS64 FLAT_RING3_DS64
#define FLAT_KERNEL_DS32 FLAT_RING3_DS32
#define FLAT_KERNEL_DS   FLAT_KERNEL_DS64
#define FLAT_KERNEL_CS64 FLAT_RING3_CS64
#define FLAT_KERNEL_CS32 FLAT_RING3_CS32
#define FLAT_KERNEL_CS   FLAT_KERNEL_CS64
#define FLAT_KERNEL_SS64 FLAT_RING3_SS64
#define FLAT_KERNEL_SS32 FLAT_RING3_SS32
#define FLAT_KERNEL_SS   FLAT_KERNEL_SS64

#define FLAT_USER_DS64 FLAT_RING3_DS64
#define FLAT_USER_DS32 FLAT_RING3_DS32
#define FLAT_USER_DS   FLAT_USER_DS64
#define FLAT_USER_CS64 FLAT_RING3_CS64
#define FLAT_USER_CS32 FLAT_RING3_CS32
#define FLAT_USER_CS   FLAT_USER_CS64
#define FLAT_USER_SS64 FLAT_RING3_SS64
#define FLAT_USER_SS32 FLAT_RING3_SS32
#define FLAT_USER_SS   FLAT_USER_SS64

#define __HYPERVISOR_VIRT_START 0xFFFF800000000000
#define __HYPERVISOR_VIRT_END   0xFFFF880000000000
#define __MACH2PHYS_VIRT_START  0xFFFF800000000000
#define __MACH2PHYS_VIRT_END    0xFFFF804000000000

#ifndef HYPERVISOR_VIRT_START
#define HYPERVISOR_VIRT_START mk_unsigned_long(__HYPERVISOR_VIRT_START)
#define HYPERVISOR_VIRT_END   mk_unsigned_long(__HYPERVISOR_VIRT_END)
#endif

#define MACH2PHYS_VIRT_START  mk_unsigned_long(__MACH2PHYS_VIRT_START)
#define MACH2PHYS_VIRT_END    mk_unsigned_long(__MACH2PHYS_VIRT_END)
#define MACH2PHYS_NR_ENTRIES  ((MACH2PHYS_VIRT_END-MACH2PHYS_VIRT_START)>>3)
#ifndef machine_to_phys_mapping
#define machine_to_phys_mapping ((unsigned long *)HYPERVISOR_VIRT_START)
#endif

#define SEGBASE_FS          0
#define SEGBASE_GS_USER     1
#define SEGBASE_GS_KERNEL   2
#define SEGBASE_GS_USER_SEL 3 /* Set user %gs specified in base[15:0] */

/* Guest exited in SYSCALL context? Return to guest with SYSRET? */
#define _VGCF_in_syscall 8
#define VGCF_in_syscall  (1<<_VGCF_in_syscall)
#define VGCF_IN_SYSCALL  VGCF_in_syscall

#ifndef __ASSEMBLY__

struct iret_context {
    /* Top of stack (%rsp at point of hypercall). */
    uint64_t rax, r11, rcx, flags, rip, cs, rflags, rsp, ss;
    /* Bottom of iret stack frame. */
};

#if defined(__GNUC__) && !defined(__STRICT_ANSI__)
/* Anonymous union includes both 32- and 64-bit names (e.g., eax/rax). */
#define __DECL_REG(name) union { \
    uint64_t r ## name, e ## name; \
    uint32_t _e ## name; \
}
#else
/* Non-gcc sources must always use the proper 64-bit name (e.g., rax). */
#define __DECL_REG(name) uint64_t r ## name
#endif

struct cpu_user_regs {
    uint64_t r15;
    uint64_t r14;
    uint64_t r13;
    uint64_t r12;
    __DECL_REG(bp);
    __DECL_REG(bx);
    uint64_t r11;
    uint64_t r10;
    uint64_t r9;
    uint64_t r8;
    __DECL_REG(ax);
    __DECL_REG(cx);
    __DECL_REG(dx);
    __DECL_REG(si);
    __DECL_REG(di);
    uint32_t error_code;    /* private */
    uint32_t entry_vector;  /* private */
    __DECL_REG(ip);
    uint16_t cs, _pad0[1];
    uint8_t  saved_upcall_mask;
    uint8_t  _pad1[3];
    __DECL_REG(flags);      /* rflags.IF == !saved_upcall_mask */
    __DECL_REG(sp);
    uint16_t ss, _pad2[3];
    uint16_t es, _pad3[3];
    uint16_t ds, _pad4[3];
    uint16_t fs, _pad5[3]; /* Non-zero => takes precedence over fs_base.     */
    uint16_t gs, _pad6[3]; /* Non-zero => takes precedence over gs_base_usr. */
};
DEFINE_GUEST_HANDLE_STRUCT(cpu_user_regs);

#undef __DECL_REG

#define xen_pfn_to_cr3(pfn) ((unsigned long)(pfn) << 12)
#define xen_cr3_to_pfn(cr3) ((unsigned long)(cr3) >> 12)

struct arch_vcpu_info {
    unsigned long cr2;
    unsigned long pad; /* sizeof(vcpu_info_t) == 64 */
};

typedef unsigned long xen_callback_t;

#define XEN_CALLBACK(__cs, __rip)				\
	((unsigned long)(__rip))

#endif /* !__ASSEMBLY__ */


#endif /* _ASM_X86_XEN_INTERFACE_64_H */
