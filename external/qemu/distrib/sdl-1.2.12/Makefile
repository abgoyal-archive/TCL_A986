OBJS_DIR     := objs
CONFIG_MAKE  := $(OBJS_DIR)/config.make
CONFIG_H     := $(OBJS_DIR)/config-host.h

ifeq ($(wildcard $(CONFIG_MAKE)),)
    $(error "The configuration file '$(CONFIG_MAKE)' doesnt' exist, please run the "android-configure.sh" script)
endif

include $(CONFIG_MAKE)

ifeq ($(strip $(BUILD_SYSTEM)),)
    $(error "The BUILD_SYSTEM variable was not defined by your configuration sub-makefile. Aborting")
endif

include $(BUILD_SYSTEM)/definitions.make

VPATH := $(OBJS_DIR)
VPATH += :$(SRC_PATH)/include
VPATH += :$(SRC_PATH)

.PHONY: all libraries executables clean clean-config clean-objs-dir \
        clean-executables clean-libraries

CLEAR_VARS                := $(BUILD_SYSTEM)/clear_vars.make
BUILD_HOST_EXECUTABLE     := $(BUILD_SYSTEM)/host_executable.make
BUILD_HOST_STATIC_LIBRARY := $(BUILD_SYSTEM)/host_static_library.make

DEPENDENCY_DIRS :=

all: libraries executables
EXECUTABLES :=
LIBRARIES   :=

clean: clean-intermediates

distclean: clean clean-config

# let's roll
include Makefile.android

libraries: $(LIBRARIES)
executables: $(EXECUTABLES)

clean-intermediates:
	rm -rf $(OBJS_DIR)/intermediates $(EXECUTABLES) $(LIBRARIES)

clean-config:
	rm -f $(CONFIG_MAKE) $(CONFIG_H)

# include dependency information
DEPENDENCY_DIRS := $(sort $(DEPENDENCY_DIRS))
-include $(wildcard $(DEPENDENCY_DIRS:%=%/*.d))
