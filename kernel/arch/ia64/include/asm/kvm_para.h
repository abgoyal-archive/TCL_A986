
#ifndef __IA64_KVM_PARA_H
#define __IA64_KVM_PARA_H


#ifdef __KERNEL__

static inline unsigned int kvm_arch_para_features(void)
{
	return 0;
}

#endif

#endif
