ifeq ($(CONFIG_SCSI_MVSAS_DEBUG),y)
	EXTRA_CFLAGS += -DMV_DEBUG
endif

obj-$(CONFIG_SCSI_MVSAS) += mvsas.o
mvsas-y +=  mv_init.o  \
           mv_sas.o   \
           mv_64xx.o  \
           mv_94xx.o
