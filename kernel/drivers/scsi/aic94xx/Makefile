ifeq ($(CONFIG_AIC94XX_DEBUG),y)
	EXTRA_CFLAGS += -DASD_DEBUG -DASD_ENTER_EXIT
endif

obj-$(CONFIG_SCSI_AIC94XX) += aic94xx.o
aic94xx-y += aic94xx_init.o \
	     aic94xx_hwi.o  \
	     aic94xx_reg.o  \
	     aic94xx_sds.o  \
	     aic94xx_seq.o  \
	     aic94xx_dump.o \
	     aic94xx_scb.o  \
	     aic94xx_dev.o  \
	     aic94xx_tmf.o  \
	     aic94xx_task.o
