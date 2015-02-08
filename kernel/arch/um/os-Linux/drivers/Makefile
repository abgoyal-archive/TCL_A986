ethertap-objs := ethertap_kern.o ethertap_user.o
tuntap-objs := tuntap_kern.o tuntap_user.o

obj-y = 
obj-$(CONFIG_UML_NET_ETHERTAP) += ethertap.o
obj-$(CONFIG_UML_NET_TUNTAP) += tuntap.o

include arch/um/scripts/Makefile.rules
