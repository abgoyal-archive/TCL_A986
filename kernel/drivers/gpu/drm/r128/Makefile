ccflags-y := -Iinclude/drm
r128-y   := r128_drv.o r128_cce.o r128_state.o r128_irq.o

r128-$(CONFIG_COMPAT)   += r128_ioc32.o

obj-$(CONFIG_DRM_R128)	+= r128.o
