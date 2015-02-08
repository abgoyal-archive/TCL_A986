ifeq ($(CONFIG_SCSI_SAS_LIBSAS_DEBUG),y)
	EXTRA_CFLAGS += -DSAS_DEBUG
endif

obj-$(CONFIG_SCSI_SAS_LIBSAS) += libsas.o
libsas-y +=  sas_init.o     \
		sas_phy.o      \
		sas_port.o     \
		sas_event.o    \
		sas_dump.o     \
		sas_discover.o \
		sas_expander.o \
		sas_scsi_host.o \
		sas_task.o
libsas-$(CONFIG_SCSI_SAS_ATA) +=	sas_ata.o
libsas-$(CONFIG_SCSI_SAS_HOST_SMP) +=	sas_host_smp.o