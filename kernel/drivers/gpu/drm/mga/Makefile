ccflags-y := -Iinclude/drm
mga-y := mga_drv.o mga_dma.o mga_state.o mga_warp.o mga_irq.o

mga-$(CONFIG_COMPAT) += mga_ioc32.o

obj-$(CONFIG_DRM_MGA)	+= mga.o

