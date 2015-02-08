ifneq ($(GCOV),)
  EXTRA_CFLAGS += -fprofile-arcs -ftest-coverage
  EXTRA_CFLAGS += -O0
endif

obj-$(CONFIG_SCSI_LPFC) := lpfc.o

lpfc-objs := lpfc_mem.o lpfc_sli.o lpfc_ct.o lpfc_els.o lpfc_hbadisc.o	\
	lpfc_init.o lpfc_mbox.o lpfc_nportdisc.o lpfc_scsi.o lpfc_attr.o \
	lpfc_vport.o lpfc_debugfs.o lpfc_bsg.o
