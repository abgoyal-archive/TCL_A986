

#ifndef __ASM_44X_H__
#define __ASM_44X_H__

#include <linux/kvm_host.h>

#define PPC44x_TLB_SIZE 64

#define KVM44x_GUEST_TLB_SIZE 64

struct kvmppc_44x_tlbe {
	u32 tid; /* Only the low 8 bits are used. */
	u32 word0;
	u32 word1;
	u32 word2;
};

struct kvmppc_44x_shadow_ref {
	struct page *page;
	u16 gtlb_index;
	u8 writeable;
	u8 tid;
};

struct kvmppc_vcpu_44x {
	/* Unmodified copy of the guest's TLB. */
	struct kvmppc_44x_tlbe guest_tlb[KVM44x_GUEST_TLB_SIZE];

	/* References to guest pages in the hardware TLB. */
	struct kvmppc_44x_shadow_ref shadow_refs[PPC44x_TLB_SIZE];

	/* State of the shadow TLB at guest context switch time. */
	struct kvmppc_44x_tlbe shadow_tlb[PPC44x_TLB_SIZE];
	u8 shadow_tlb_mod[PPC44x_TLB_SIZE];

	struct kvm_vcpu vcpu;
};

static inline struct kvmppc_vcpu_44x *to_44x(struct kvm_vcpu *vcpu)
{
	return container_of(vcpu, struct kvmppc_vcpu_44x, vcpu);
}

void kvmppc_set_pid(struct kvm_vcpu *vcpu, u32 new_pid);
void kvmppc_44x_tlb_put(struct kvm_vcpu *vcpu);
void kvmppc_44x_tlb_load(struct kvm_vcpu *vcpu);

#endif /* __ASM_44X_H__ */
