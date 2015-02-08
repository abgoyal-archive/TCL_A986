


#include <acpi/acpi.h>
#include "accommon.h"
#include "acresrc.h"

#define _COMPONENT          ACPI_RESOURCES
ACPI_MODULE_NAME("rsinfo")

/* Dispatch table for resource-to-AML (Set Resource) conversion functions */
struct acpi_rsconvert_info *acpi_gbl_set_resource_dispatch[] = {
	acpi_rs_set_irq,	/* 0x00, ACPI_RESOURCE_TYPE_IRQ */
	acpi_rs_convert_dma,	/* 0x01, ACPI_RESOURCE_TYPE_DMA */
	acpi_rs_set_start_dpf,	/* 0x02, ACPI_RESOURCE_TYPE_START_DEPENDENT */
	acpi_rs_convert_end_dpf,	/* 0x03, ACPI_RESOURCE_TYPE_END_DEPENDENT */
	acpi_rs_convert_io,	/* 0x04, ACPI_RESOURCE_TYPE_IO */
	acpi_rs_convert_fixed_io,	/* 0x05, ACPI_RESOURCE_TYPE_FIXED_IO */
	acpi_rs_set_vendor,	/* 0x06, ACPI_RESOURCE_TYPE_VENDOR */
	acpi_rs_convert_end_tag,	/* 0x07, ACPI_RESOURCE_TYPE_END_TAG */
	acpi_rs_convert_memory24,	/* 0x08, ACPI_RESOURCE_TYPE_MEMORY24 */
	acpi_rs_convert_memory32,	/* 0x09, ACPI_RESOURCE_TYPE_MEMORY32 */
	acpi_rs_convert_fixed_memory32,	/* 0x0A, ACPI_RESOURCE_TYPE_FIXED_MEMORY32 */
	acpi_rs_convert_address16,	/* 0x0B, ACPI_RESOURCE_TYPE_ADDRESS16 */
	acpi_rs_convert_address32,	/* 0x0C, ACPI_RESOURCE_TYPE_ADDRESS32 */
	acpi_rs_convert_address64,	/* 0x0D, ACPI_RESOURCE_TYPE_ADDRESS64 */
	acpi_rs_convert_ext_address64,	/* 0x0E, ACPI_RESOURCE_TYPE_EXTENDED_ADDRESS64 */
	acpi_rs_convert_ext_irq,	/* 0x0F, ACPI_RESOURCE_TYPE_EXTENDED_IRQ */
	acpi_rs_convert_generic_reg	/* 0x10, ACPI_RESOURCE_TYPE_GENERIC_REGISTER */
};

/* Dispatch tables for AML-to-resource (Get Resource) conversion functions */

struct acpi_rsconvert_info *acpi_gbl_get_resource_dispatch[] = {
	/* Small descriptors */

	NULL,			/* 0x00, Reserved */
	NULL,			/* 0x01, Reserved */
	NULL,			/* 0x02, Reserved */
	NULL,			/* 0x03, Reserved */
	acpi_rs_get_irq,	/* 0x04, ACPI_RESOURCE_NAME_IRQ */
	acpi_rs_convert_dma,	/* 0x05, ACPI_RESOURCE_NAME_DMA */
	acpi_rs_get_start_dpf,	/* 0x06, ACPI_RESOURCE_NAME_START_DEPENDENT */
	acpi_rs_convert_end_dpf,	/* 0x07, ACPI_RESOURCE_NAME_END_DEPENDENT */
	acpi_rs_convert_io,	/* 0x08, ACPI_RESOURCE_NAME_IO */
	acpi_rs_convert_fixed_io,	/* 0x09, ACPI_RESOURCE_NAME_FIXED_IO */
	NULL,			/* 0x0A, Reserved */
	NULL,			/* 0x0B, Reserved */
	NULL,			/* 0x0C, Reserved */
	NULL,			/* 0x0D, Reserved */
	acpi_rs_get_vendor_small,	/* 0x0E, ACPI_RESOURCE_NAME_VENDOR_SMALL */
	acpi_rs_convert_end_tag,	/* 0x0F, ACPI_RESOURCE_NAME_END_TAG */

	/* Large descriptors */

	NULL,			/* 0x00, Reserved */
	acpi_rs_convert_memory24,	/* 0x01, ACPI_RESOURCE_NAME_MEMORY24 */
	acpi_rs_convert_generic_reg,	/* 0x02, ACPI_RESOURCE_NAME_GENERIC_REGISTER */
	NULL,			/* 0x03, Reserved */
	acpi_rs_get_vendor_large,	/* 0x04, ACPI_RESOURCE_NAME_VENDOR_LARGE */
	acpi_rs_convert_memory32,	/* 0x05, ACPI_RESOURCE_NAME_MEMORY32 */
	acpi_rs_convert_fixed_memory32,	/* 0x06, ACPI_RESOURCE_NAME_FIXED_MEMORY32 */
	acpi_rs_convert_address32,	/* 0x07, ACPI_RESOURCE_NAME_ADDRESS32 */
	acpi_rs_convert_address16,	/* 0x08, ACPI_RESOURCE_NAME_ADDRESS16 */
	acpi_rs_convert_ext_irq,	/* 0x09, ACPI_RESOURCE_NAME_EXTENDED_IRQ */
	acpi_rs_convert_address64,	/* 0x0A, ACPI_RESOURCE_NAME_ADDRESS64 */
	acpi_rs_convert_ext_address64	/* 0x0B, ACPI_RESOURCE_NAME_EXTENDED_ADDRESS64 */
};

#ifdef ACPI_FUTURE_USAGE
#if defined(ACPI_DEBUG_OUTPUT) || defined(ACPI_DEBUGGER)

/* Dispatch table for resource dump functions */

struct acpi_rsdump_info *acpi_gbl_dump_resource_dispatch[] = {
	acpi_rs_dump_irq,	/* ACPI_RESOURCE_TYPE_IRQ */
	acpi_rs_dump_dma,	/* ACPI_RESOURCE_TYPE_DMA */
	acpi_rs_dump_start_dpf,	/* ACPI_RESOURCE_TYPE_START_DEPENDENT */
	acpi_rs_dump_end_dpf,	/* ACPI_RESOURCE_TYPE_END_DEPENDENT */
	acpi_rs_dump_io,	/* ACPI_RESOURCE_TYPE_IO */
	acpi_rs_dump_fixed_io,	/* ACPI_RESOURCE_TYPE_FIXED_IO */
	acpi_rs_dump_vendor,	/* ACPI_RESOURCE_TYPE_VENDOR */
	acpi_rs_dump_end_tag,	/* ACPI_RESOURCE_TYPE_END_TAG */
	acpi_rs_dump_memory24,	/* ACPI_RESOURCE_TYPE_MEMORY24 */
	acpi_rs_dump_memory32,	/* ACPI_RESOURCE_TYPE_MEMORY32 */
	acpi_rs_dump_fixed_memory32,	/* ACPI_RESOURCE_TYPE_FIXED_MEMORY32 */
	acpi_rs_dump_address16,	/* ACPI_RESOURCE_TYPE_ADDRESS16 */
	acpi_rs_dump_address32,	/* ACPI_RESOURCE_TYPE_ADDRESS32 */
	acpi_rs_dump_address64,	/* ACPI_RESOURCE_TYPE_ADDRESS64 */
	acpi_rs_dump_ext_address64,	/* ACPI_RESOURCE_TYPE_EXTENDED_ADDRESS64 */
	acpi_rs_dump_ext_irq,	/* ACPI_RESOURCE_TYPE_EXTENDED_IRQ */
	acpi_rs_dump_generic_reg,	/* ACPI_RESOURCE_TYPE_GENERIC_REGISTER */
};
#endif

#endif				/* ACPI_FUTURE_USAGE */
const u8 acpi_gbl_aml_resource_sizes[] = {
	sizeof(struct aml_resource_irq),	/* ACPI_RESOURCE_TYPE_IRQ (optional Byte 3 always created) */
	sizeof(struct aml_resource_dma),	/* ACPI_RESOURCE_TYPE_DMA */
	sizeof(struct aml_resource_start_dependent),	/* ACPI_RESOURCE_TYPE_START_DEPENDENT (optional Byte 1 always created) */
	sizeof(struct aml_resource_end_dependent),	/* ACPI_RESOURCE_TYPE_END_DEPENDENT */
	sizeof(struct aml_resource_io),	/* ACPI_RESOURCE_TYPE_IO */
	sizeof(struct aml_resource_fixed_io),	/* ACPI_RESOURCE_TYPE_FIXED_IO */
	sizeof(struct aml_resource_vendor_small),	/* ACPI_RESOURCE_TYPE_VENDOR */
	sizeof(struct aml_resource_end_tag),	/* ACPI_RESOURCE_TYPE_END_TAG */
	sizeof(struct aml_resource_memory24),	/* ACPI_RESOURCE_TYPE_MEMORY24 */
	sizeof(struct aml_resource_memory32),	/* ACPI_RESOURCE_TYPE_MEMORY32 */
	sizeof(struct aml_resource_fixed_memory32),	/* ACPI_RESOURCE_TYPE_FIXED_MEMORY32 */
	sizeof(struct aml_resource_address16),	/* ACPI_RESOURCE_TYPE_ADDRESS16 */
	sizeof(struct aml_resource_address32),	/* ACPI_RESOURCE_TYPE_ADDRESS32 */
	sizeof(struct aml_resource_address64),	/* ACPI_RESOURCE_TYPE_ADDRESS64 */
	sizeof(struct aml_resource_extended_address64),	/*ACPI_RESOURCE_TYPE_EXTENDED_ADDRESS64 */
	sizeof(struct aml_resource_extended_irq),	/* ACPI_RESOURCE_TYPE_EXTENDED_IRQ */
	sizeof(struct aml_resource_generic_register)	/* ACPI_RESOURCE_TYPE_GENERIC_REGISTER */
};

const u8 acpi_gbl_resource_struct_sizes[] = {
	/* Small descriptors */

	0,
	0,
	0,
	0,
	ACPI_RS_SIZE(struct acpi_resource_irq),
	ACPI_RS_SIZE(struct acpi_resource_dma),
	ACPI_RS_SIZE(struct acpi_resource_start_dependent),
	ACPI_RS_SIZE_MIN,
	ACPI_RS_SIZE(struct acpi_resource_io),
	ACPI_RS_SIZE(struct acpi_resource_fixed_io),
	0,
	0,
	0,
	0,
	ACPI_RS_SIZE(struct acpi_resource_vendor),
	ACPI_RS_SIZE_MIN,

	/* Large descriptors */

	0,
	ACPI_RS_SIZE(struct acpi_resource_memory24),
	ACPI_RS_SIZE(struct acpi_resource_generic_register),
	0,
	ACPI_RS_SIZE(struct acpi_resource_vendor),
	ACPI_RS_SIZE(struct acpi_resource_memory32),
	ACPI_RS_SIZE(struct acpi_resource_fixed_memory32),
	ACPI_RS_SIZE(struct acpi_resource_address32),
	ACPI_RS_SIZE(struct acpi_resource_address16),
	ACPI_RS_SIZE(struct acpi_resource_extended_irq),
	ACPI_RS_SIZE(struct acpi_resource_address64),
	ACPI_RS_SIZE(struct acpi_resource_extended_address64)
};
