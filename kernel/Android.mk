$(info using $(KERNEL_CONFIG_FILE) .... )
ifeq ($(TARGET_KMODULES),true)
ALL_PREBUILT += $(TARGET_OUT)/lib/modules/modules.order
$(BUILT_SYSTEMIMAGE): kernel_modules
$(TARGET_OUT)/lib/modules/modules.order: kernel_modules
kernel_modules:
	@echo "building linux kernel modules..."
#ifneq (,$(KERNEL_CONFIG_FILE))
#	@cat kernel/$(KERNEL_CONFIG_FILE) > kernel/.config
#endif
	make MTK_PROJECT=$(MTK_PROJECT) -C  kernel modules
	INSTALL_MOD_STRIP=1 MTK_PROJECT=$(MTK_PROJECT) INSTALL_MOD_PATH=../$(TARGET_OUT) INSTALL_MOD_DIR=../$(TARGET_OUT) make -C kernel android_modules_install
endif
