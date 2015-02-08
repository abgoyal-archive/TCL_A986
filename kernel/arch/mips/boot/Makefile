ifdef CONFIG_MACH_DECSTATION
  E2EFLAGS = -a
else
  E2EFLAGS =
endif

#
# Drop some uninteresting sections in the kernel.
# This is only relevant for ELF kernels but doesn't hurt a.out
#
drop-sections	= .reginfo .mdebug .comment .note .pdr .options .MIPS.options
strip-flags	= $(addprefix --remove-section=,$(drop-sections))

VMLINUX = vmlinux

all: vmlinux.ecoff vmlinux.srec

vmlinux.ecoff: $(obj)/elf2ecoff $(VMLINUX)
	$(obj)/elf2ecoff $(VMLINUX) $(obj)/vmlinux.ecoff $(E2EFLAGS)

$(obj)/elf2ecoff: $(obj)/elf2ecoff.c
	$(HOSTCC) -o $@ $^

vmlinux.bin: $(VMLINUX)
	$(OBJCOPY) -O binary $(strip-flags) $(VMLINUX) $(obj)/vmlinux.bin

vmlinux.srec: $(VMLINUX)
	$(OBJCOPY) -S -O srec $(strip-flags) $(VMLINUX) $(obj)/vmlinux.srec

clean-files += elf2ecoff \
	       vmlinux.bin \
	       vmlinux.ecoff \
	       vmlinux.srec
