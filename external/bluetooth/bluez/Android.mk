ifneq ($(TARGET_SIMULATOR),true)
ifeq ($(BOARD_HAVE_BLUETOOTH),true)
  include $(all-subdir-makefiles)
endif
endif
