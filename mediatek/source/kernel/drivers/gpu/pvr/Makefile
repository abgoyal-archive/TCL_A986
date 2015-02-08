ifeq ($(CONFIG_ARCH_MT6573), y)
PVR_SYSTEM = mt6573
PVR_BUILD_DIR = mt6573
DISPLAY_CONTROLLER = mtklfb
SGXCORE = 531
SGXCOREREV = 101
SYS_SGX_ACTIVE_POWER_LATENCY_MS = 100
OPTIM = -Os
endif

ccflags-y := \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/include4 \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/include \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/srvkm/env/linux \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/srvkm/include \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/srvkm/bridged \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/system/$(PVR_SYSTEM) \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/system/include \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/srvkm/hwdefs \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/srvkm/bridged/sgx \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/srvkm/devices/sgx \
	-I$(MTK_PATH_SOURCE)/drivers/gpu/pvr/services4/3rdparty/mtklfb

ccflags-y += \
	-Wall \
	-Wdeclaration-after-statement \
	-Wpointer-arith \
	-Wmissing-format-attribute \
	-Wno-format-zero-length \
	-Wmissing-prototypes \
	-Wstrict-prototypes \
	-Wno-unused-parameter \
	-Wno-sign-compare \
	-Werror \
	-fno-strict-aliasing \
	-Wno-pointer-arith

ccflags-y += \
	-DLINUX \
	-DPVR_BUILD_DIR="\"$(PVR_BUILD_DIR)\"" \
	-DPVR_BUILD_DATE="\"$(DATE)\""

ccflags-$(CONFIG_PVR_BUILD_RELEASE) += \
	-DPVR_BUILD_TYPE="\"release\"" -DRELEASE $(OPTIM) -g

ccflags-$(CONFIG_PVR_BUILD_DEBUG) += \
	-DPVR_BUILD_TYPE="\"debug\"" -DDEBUG -g -O0 -DDLL_METRIC=1

ccflags-y += \
	-DANDROID \
	-D__linux__ \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSYS_SGX_ACTIVE_POWER_LATENCY_MS=$(SYS_SGX_ACTIVE_POWER_LATENCY_MS) \
	-DUSE_ARGB8888_FB \
	-DPVRSRV_MODNAME="\"pvrsrvkm\"" \
	-DSERVICES4 \
	-D_XOPEN_SOURCE=600 \
	-D_POSIX_C_SOURCE=199309 \
	-DPVR2D_VALIDATE_INPUT_PARAMS \
	-DDISPLAY_CONTROLLER=$(DISPLAY_CONTROLLER)

ccflags-$(CONFIG_ARCH_MT6573) += \
	-DSUPPORT_SRVINIT \
	-DSUPPORT_SGX \
	-DSUPPORT_XWS \
	-DDISABLE_SGX_PB_GROW_SHRINK \
	-DSUPPORT_LINUX_X86_WRITECOMBINE \
	-DTRANSFER_QUEUE -DSYS_USING_INTERRUPTS \
	-DSUPPORT_HW_RECOVERY \
	-DSUPPORT_ACTIVE_POWER_MANAGEMENT \
	-DPVR_SECURE_HANDLES \
	-DPVR_SECURE_FD_EXPORT \
	-DUSE_PTHREADS \
	-DSUPPORT_SGX_EVENT_OBJECT \
	-DLDM_PLATFORM \
	-DSUPPORT_SGX_HWPERF \
	-DSUPPORT_SGX_LOW_LATENCY_SCHEDULING \
	-DSUPPORT_LINUX_X86_PAT \
	-DSUPPORT_MEMINFO_IDS \
	-DPVR_PDP_LINUX_FB \
	-DPVR_LINUX_USING_WORKQUEUES \
	-DPVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE \
	-DPVR_LINUX_TIMERS_USING_WORKQUEUES \
	-DSYS_CUSTOM_POWERLOCK_WRAP \
	-DSUPPORT_SGX_NEW_STATUS_VALS

ccflags-y += \
	-DSGX$(SGXCORE) \
	-DSUPPORT_SGX$(SGXCORE) \
	-DSGX_CORE_REV=$(SGXCOREREV)

pvrsrvkm-y := \
	services4/srvkm/env/linux/osfunc.o \
	services4/srvkm/env/linux/mutils.o \
	services4/srvkm/env/linux/mmap.o \
	services4/srvkm/env/linux/module.o \
	services4/srvkm/env/linux/pdump.o \
	services4/srvkm/env/linux/proc.o \
	services4/srvkm/env/linux/pvr_bridge_k.o \
	services4/srvkm/env/linux/pvr_debug.o \
	services4/srvkm/env/linux/mm.o \
	services4/srvkm/env/linux/mutex.o \
	services4/srvkm/env/linux/event.o \
	services4/srvkm/env/linux/osperproc.o \
	services4/srvkm/common/buffer_manager.o \
	services4/srvkm/common/devicemem.o \
	services4/srvkm/common/deviceclass.o \
	services4/srvkm/common/handle.o \
	services4/srvkm/common/hash.o \
	services4/srvkm/common/metrics.o \
	services4/srvkm/common/pvrsrv.o \
	services4/srvkm/common/queue.o \
	services4/srvkm/common/ra.o \
	services4/srvkm/common/resman.o \
	services4/srvkm/common/power.o \
	services4/srvkm/common/mem.o \
	services4/srvkm/common/pdump_common.o \
	services4/srvkm/bridged/bridged_support.o \
	services4/srvkm/bridged/bridged_pvr_bridge.o \
	services4/srvkm/common/perproc.o \
	services4/system/$(PVR_SYSTEM)/sysconfig.o \
	services4/system/$(PVR_SYSTEM)/sysutils.o \
	services4/srvkm/common/lists.o \
	services4/srvkm/common/mem_debug.o \
	services4/srvkm/common/osfunc_common.o \
	services4/srvkm/bridged/sgx/bridged_sgx_bridge.o \
	services4/srvkm/devices/sgx/sgxinit.o \
	services4/srvkm/devices/sgx/sgxpower.o \
	services4/srvkm/devices/sgx/sgxreset.o \
	services4/srvkm/devices/sgx/sgxutils.o \
	services4/srvkm/devices/sgx/sgxkick.o \
	services4/srvkm/devices/sgx/sgxtransfer.o \
	services4/srvkm/devices/sgx/mmu.o \
	services4/srvkm/devices/sgx/pb.o

mtklfb-y := \
	services4/3rdparty/mtklfb/mtklfb_displayclass.o \
	services4/3rdparty/mtklfb/mtklfb_linux.o

obj-$(CONFIG_PVR_SGX) += pvrsrvkm.o

obj-$(CONFIG_PVR_SGX) += mtklfb.o
