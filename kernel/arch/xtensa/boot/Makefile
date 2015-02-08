KBUILD_CFLAGS	+= -fno-builtin -Iarch/$(ARCH)/boot/include
HOSTFLAGS	+= -Iarch/$(ARCH)/boot/include

BIG_ENDIAN	:= $(shell echo -e __XTENSA_EB__ | $(CC) -E - | grep -v "\#")

export EXTRA_CFLAGS
export BIG_ENDIAN

subdir-y	:= lib

# Subdirs for the boot loader(s)

bootdir-$(CONFIG_XTENSA_PLATFORM_ISS)	 += boot-elf
bootdir-$(CONFIG_XTENSA_PLATFORM_XT2000) += boot-redboot boot-elf


zImage zImage.initrd Image Image.initrd: $(bootdir-y)

$(bootdir-y): $(addprefix $(obj)/,$(subdir-y)) \
	      $(addprefix $(obj)/,$(host-progs))
	$(Q)$(MAKE) $(build)=$(obj)/$@ $(MAKECMDGOALS)

