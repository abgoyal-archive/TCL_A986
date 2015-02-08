targets		:= vmlinux.bin zImage

subdir- 	:= compressed

# ---------------------------------------------------------------------------


$(obj)/zImage: $(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)
	@echo 'Kernel: $@ is ready'

$(obj)/vmlinux.bin: $(obj)/compressed/vmlinux FORCE
	$(call if_changed,objcopy)

$(obj)/compressed/vmlinux: FORCE
	$(Q)$(MAKE) $(build)=$(obj)/compressed IMAGE_OFFSET=$(IMAGE_OFFSET) $@
