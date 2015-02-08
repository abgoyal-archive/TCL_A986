

#ifndef __ASM_KVM_BOOK3S_64_H__
#define __ASM_KVM_BOOK3S_64_H__

static inline struct kvmppc_book3s_shadow_vcpu *to_svcpu(struct kvm_vcpu *vcpu)
{
	return &get_paca()->shadow_vcpu;
}

#endif /* __ASM_KVM_BOOK3S_64_H__ */
