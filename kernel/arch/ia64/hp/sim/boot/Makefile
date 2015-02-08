targets-$(CONFIG_IA64_HP_SIM)  += bootloader
targets := vmlinux.bin vmlinux.gz $(targets-y)

quiet_cmd_cptotop = LN      $@
      cmd_cptotop = ln -f $< $@

vmlinux.gz: $(obj)/vmlinux.gz $(addprefix $(obj)/,$(targets-y))
	$(call cmd,cptotop)
	@echo '  Kernel: $@ is ready'

boot: bootloader

bootloader: $(obj)/bootloader
	$(call cmd,cptotop)

$(obj)/vmlinux.gz: $(obj)/vmlinux.bin FORCE
	$(call if_changed,gzip)

$(obj)/vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)


LDFLAGS_bootloader = -static -T

$(obj)/bootloader: $(src)/bootloader.lds $(obj)/bootloader.o $(obj)/boot_head.o $(obj)/fw-emu.o \
                   lib/lib.a arch/ia64/lib/built-in.o arch/ia64/lib/lib.a FORCE
	$(call if_changed,ld)
