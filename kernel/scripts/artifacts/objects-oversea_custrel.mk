obj_to_install := \
 ../mediatek/source/kernel/drivers/combo/common/core:mtkstp.o \
 ../mediatek/source/kernel/drivers/combo/common/core:mtkwmt.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:stp_chrdev_wmt.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:stp_drv.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:stp_psm.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:stp_sdio.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:stp_uart.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:wmt_lib.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:wmt_lib_conf.o \
 ../mediatek/source/kernel/drivers/combo/common/linux/pri:wmt_lib_plat.o \
 ../mediatek/platform/mt6573/kernel/drivers/mflexvideo:mt6573_mflexvideo_kernel_driver.o \
 ../mediatek/platform/mt6573/kernel/drivers/mflexvideo/codec/common/hal/linux/kernel:hal_hw.o \
 ../mediatek/platform/mt6573/kernel/drivers/mflexvideo/codec/common/val/linux/kernel:val_hw.o \
 ../mediatek/platform/mt6573/kernel/drivers/mflexvideo/codec/common/val/linux/kernel:val.o \
 ../mediatek/platform/mt6573/kernel/drivers/mflexvideo/codec/common/val/linux/kernel:mfv_load_packet.o \
 ../mediatek/platform/mt6573/kernel/drivers/mflexvideo/codec/common/api/src:mfv_drv_base.o

ifeq ($(MTK_WAPI_SUPPORT),yes)
obj_to_install +=  ../mediatek/source/kernel/drivers/net/mt592x/wlan:gl_sec.o \

endif

file_to_touch := \
  arch/arm/mach-mt3351/Kconfig \
  arch/arm/mach-mt3351/Makefile \

