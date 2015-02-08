obj-y := setup.o
ifeq ($(CONFIG_DMAR), y)
obj-$(CONFIG_IA64_GENERIC) += machvec.o machvec_vtd.o
else
obj-$(CONFIG_IA64_GENERIC) += machvec.o
endif

