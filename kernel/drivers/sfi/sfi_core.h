
/* sfi_core.h Simple Firmware Interface, internal header */

struct sfi_table_key{
	char	*sig;
	char	*oem_id;
	char	*oem_table_id;
};

/* sysfs interface */
struct sfi_table_attr {
	struct bin_attribute attr;
	char name[8];
};

#define SFI_ANY_KEY { .sig = NULL, .oem_id = NULL, .oem_table_id = NULL }

extern int __init sfi_acpi_init(void);
extern  struct sfi_table_header *sfi_check_table(u64 paddr,
					struct sfi_table_key *key);
struct sfi_table_header *sfi_get_table(struct sfi_table_key *key);
extern void sfi_put_table(struct sfi_table_header *table);
extern struct sfi_table_attr __init *sfi_sysfs_install_table(u64 pa);
extern void __init sfi_acpi_sysfs_init(void);
