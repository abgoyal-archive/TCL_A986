targets := zImage
subdir- := compressed

obj-y	:= setup.o

$(obj)/zImage: $(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)
	@echo 'Kernel: $@ is ready'

$(obj)/compressed/vmlinux: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed $@

