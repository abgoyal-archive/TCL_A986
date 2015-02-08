
/* sfi.h Simple Firmware Interface */


#ifndef _LINUX_SFI_ACPI_H
#define _LINUX_SFI_ACPI_H

#ifdef CONFIG_SFI
#include <acpi/acpi.h>		/* struct acpi_table_header */

extern int sfi_acpi_table_parse(char *signature, char *oem_id,
				char *oem_table_id,
				int (*handler)(struct acpi_table_header *));

static inline int acpi_sfi_table_parse(char *signature,
				int (*handler)(struct acpi_table_header *))
{
	if (!acpi_table_parse(signature, handler))
		return 0;

	return sfi_acpi_table_parse(signature, NULL, NULL, handler);
}
#else /* !CONFIG_SFI */

static inline int sfi_acpi_table_parse(char *signature, char *oem_id,
				char *oem_table_id,
				int (*handler)(struct acpi_table_header *))
{
	return -1;
}

static inline int acpi_sfi_table_parse(char *signature,
				int (*handler)(struct acpi_table_header *))
{
	return acpi_table_parse(signature, handler);
}
#endif /* !CONFIG_SFI */

#endif /*_LINUX_SFI_ACPI_H*/
