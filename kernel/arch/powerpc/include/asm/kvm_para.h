

#ifndef __POWERPC_KVM_PARA_H__
#define __POWERPC_KVM_PARA_H__

#ifdef __KERNEL__

static inline int kvm_para_available(void)
{
	return 0;
}

static inline unsigned int kvm_arch_para_features(void)
{
	return 0;
}

#endif /* __KERNEL__ */

#endif /* __POWERPC_KVM_PARA_H__ */
