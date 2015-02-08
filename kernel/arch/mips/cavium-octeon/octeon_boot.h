

#ifndef __OCTEON_BOOT_H__
#define __OCTEON_BOOT_H__

#include <linux/types.h>

struct boot_init_vector {
	uint32_t stack_addr;
	uint32_t code_addr;
	uint32_t app_start_func_addr;
	uint32_t k0_val;
	uint32_t flags;
	uint32_t boot_info_addr;
	uint32_t pad;
	uint32_t pad2;
};

/* similar to bootloader's linux_app_boot_info but without global data */
struct linux_app_boot_info {
	uint32_t labi_signature;
	uint32_t start_core0_addr;
	uint32_t avail_coremask;
	uint32_t pci_console_active;
	uint32_t icache_prefetch_disable;
	uint32_t InitTLBStart_addr;
	uint32_t start_app_addr;
	uint32_t cur_exception_base;
	uint32_t no_mark_private_data;
	uint32_t compact_flash_common_base_addr;
	uint32_t compact_flash_attribute_base_addr;
	uint32_t led_display_base_addr;
};

#define AVAIL_COREMASK_OFFSET_IN_LINUX_APP_BOOT_BLOCK    0x765c

/* hardcoded in bootloader */
#define  LABI_ADDR_IN_BOOTLOADER                         0x700

#define LINUX_APP_BOOT_BLOCK_NAME "linux-app-boot"

#define LABI_SIGNATURE 0xAABBCCDD

/*  from uboot-headers/octeon_mem_map.h */
#define EXCEPTION_BASE_INCR     (4 * 1024)
			       /* Increment size for exception base addresses (4k minimum) */
#define EXCEPTION_BASE_BASE     0
#define BOOTLOADER_PRIV_DATA_BASE       (EXCEPTION_BASE_BASE + 0x800)
#define BOOTLOADER_BOOT_VECTOR          (BOOTLOADER_PRIV_DATA_BASE)

#endif /* __OCTEON_BOOT_H__ */
