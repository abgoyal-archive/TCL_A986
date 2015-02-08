hostprogs-$(CONFIG_KALLSYMS)     += kallsyms
hostprogs-$(CONFIG_LOGO)         += pnmtologo
hostprogs-$(CONFIG_VT)           += conmakehash
hostprogs-$(CONFIG_IKCONFIG)     += bin2c

always		:= $(hostprogs-y) $(hostprogs-m)

# The following hostprogs-y programs are only build on demand
hostprogs-y += unifdef

subdir-$(CONFIG_MODVERSIONS) += genksyms
subdir-y                     += mod
subdir-$(CONFIG_SECURITY_SELINUX) += selinux
subdir-$(CONFIG_DTC)         += dtc

# Let clean descend into subdirs
subdir-	+= basic kconfig package selinux
