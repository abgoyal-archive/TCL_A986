$(call add-clean-step, rm -rf $(PRODUCT_OUT)/obj/STATIC_LIBRARIES/libv8_intermediates)
$(call add-clean-step, rm -rf $(OUT_DIR)/host/$(HOST_OS)-$(HOST_ARCH)/bin/mksnapshot)
$(call add-clean-step, rm -rf $(OUT_DIR)/host/$(HOST_OS)-$(HOST_ARCH)/obj/EXECUTABLES/mksnapshot_intermediates)


# ************************************************
# NEWER CLEAN STEPS MUST BE AT THE END OF THE LIST
# ************************************************
