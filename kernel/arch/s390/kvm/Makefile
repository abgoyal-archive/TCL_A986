common-objs = $(addprefix ../../../virt/kvm/, kvm_main.o)

EXTRA_CFLAGS += -Ivirt/kvm -Iarch/s390/kvm

kvm-objs := $(common-objs) kvm-s390.o sie64a.o intercept.o interrupt.o priv.o sigp.o diag.o
obj-$(CONFIG_KVM) += kvm.o
