



#include <acpi/acpi.h>
#include "accommon.h"

#define _COMPONENT          ACPI_HARDWARE
ACPI_MODULE_NAME("hwvalid")

/* Local prototypes */
static acpi_status
acpi_hw_validate_io_request(acpi_io_address address, u32 bit_width);

static const struct acpi_port_info acpi_protected_ports[] = {
	{"DMA", 0x0000, 0x000F, ACPI_OSI_WIN_XP},
	{"PIC0", 0x0020, 0x0021, ACPI_ALWAYS_ILLEGAL},
	{"PIT1", 0x0040, 0x0043, ACPI_OSI_WIN_XP},
	{"PIT2", 0x0048, 0x004B, ACPI_OSI_WIN_XP},
	{"RTC", 0x0070, 0x0071, ACPI_OSI_WIN_XP},
	{"CMOS", 0x0074, 0x0076, ACPI_OSI_WIN_XP},
	{"DMA1", 0x0081, 0x0083, ACPI_OSI_WIN_XP},
	{"DMA1L", 0x0087, 0x0087, ACPI_OSI_WIN_XP},
	{"DMA2", 0x0089, 0x008B, ACPI_OSI_WIN_XP},
	{"DMA2L", 0x008F, 0x008F, ACPI_OSI_WIN_XP},
	{"ARBC", 0x0090, 0x0091, ACPI_OSI_WIN_XP},
	{"SETUP", 0x0093, 0x0094, ACPI_OSI_WIN_XP},
	{"POS", 0x0096, 0x0097, ACPI_OSI_WIN_XP},
	{"PIC1", 0x00A0, 0x00A1, ACPI_ALWAYS_ILLEGAL},
	{"IDMA", 0x00C0, 0x00DF, ACPI_OSI_WIN_XP},
	{"ELCR", 0x04D0, 0x04D1, ACPI_ALWAYS_ILLEGAL},
	{"PCI", 0x0CF8, 0x0CFF, ACPI_OSI_WIN_XP}
};

#define ACPI_PORT_INFO_ENTRIES  ACPI_ARRAY_LENGTH (acpi_protected_ports)


static acpi_status
acpi_hw_validate_io_request(acpi_io_address address, u32 bit_width)
{
	u32 i;
	u32 byte_width;
	acpi_io_address last_address;
	const struct acpi_port_info *port_info;

	ACPI_FUNCTION_TRACE(hw_validate_io_request);

	/* Supported widths are 8/16/32 */

	if ((bit_width != 8) && (bit_width != 16) && (bit_width != 32)) {
		return AE_BAD_PARAMETER;
	}

	port_info = acpi_protected_ports;
	byte_width = ACPI_DIV_8(bit_width);
	last_address = address + byte_width - 1;

	ACPI_DEBUG_PRINT((ACPI_DB_IO, "Address %p LastAddress %p Length %X",
			  ACPI_CAST_PTR(void, address), ACPI_CAST_PTR(void,
								      last_address),
			  byte_width));

	/* Maximum 16-bit address in I/O space */

	if (last_address > ACPI_UINT16_MAX) {
		ACPI_ERROR((AE_INFO,
			    "Illegal I/O port address/length above 64K: %p/0x%X",
			    ACPI_CAST_PTR(void, address), byte_width));
		return_ACPI_STATUS(AE_LIMIT);
	}

	/* Exit if requested address is not within the protected port table */

	if (address > acpi_protected_ports[ACPI_PORT_INFO_ENTRIES - 1].end) {
		return_ACPI_STATUS(AE_OK);
	}

	/* Check request against the list of protected I/O ports */

	for (i = 0; i < ACPI_PORT_INFO_ENTRIES; i++, port_info++) {
		/*
		 * Check if the requested address range will write to a reserved
		 * port. Four cases to consider:
		 *
		 * 1) Address range is contained completely in the port address range
		 * 2) Address range overlaps port range at the port range start
		 * 3) Address range overlaps port range at the port range end
		 * 4) Address range completely encompasses the port range
		 */
		if ((address <= port_info->end)
		    && (last_address >= port_info->start)) {

			/* Port illegality may depend on the _OSI calls made by the BIOS */

			if (acpi_gbl_osi_data >= port_info->osi_dependency) {
				ACPI_DEBUG_PRINT((ACPI_DB_IO,
						  "Denied AML access to port 0x%p/%X (%s 0x%.4X-0x%.4X)",
						  ACPI_CAST_PTR(void, address),
						  byte_width, port_info->name,
						  port_info->start,
						  port_info->end));

				return_ACPI_STATUS(AE_AML_ILLEGAL_ADDRESS);
			}
		}

		/* Finished if address range ends before the end of this port */

		if (last_address <= port_info->end) {
			break;
		}
	}

	return_ACPI_STATUS(AE_OK);
}


acpi_status acpi_hw_read_port(acpi_io_address address, u32 *value, u32 width)
{
	acpi_status status;
	u32 one_byte;
	u32 i;

	/* Truncate address to 16 bits if requested */

	if (acpi_gbl_truncate_io_addresses) {
		address &= ACPI_UINT16_MAX;
	}

	/* Validate the entire request and perform the I/O */

	status = acpi_hw_validate_io_request(address, width);
	if (ACPI_SUCCESS(status)) {
		status = acpi_os_read_port(address, value, width);
		return status;
	}

	if (status != AE_AML_ILLEGAL_ADDRESS) {
		return status;
	}

	/*
	 * There has been a protection violation within the request. Fall
	 * back to byte granularity port I/O and ignore the failing bytes.
	 * This provides Windows compatibility.
	 */
	for (i = 0, *value = 0; i < width; i += 8) {

		/* Validate and read one byte */

		if (acpi_hw_validate_io_request(address, 8) == AE_OK) {
			status = acpi_os_read_port(address, &one_byte, 8);
			if (ACPI_FAILURE(status)) {
				return status;
			}

			*value |= (one_byte << i);
		}

		address++;
	}

	return AE_OK;
}


acpi_status acpi_hw_write_port(acpi_io_address address, u32 value, u32 width)
{
	acpi_status status;
	u32 i;

	/* Truncate address to 16 bits if requested */

	if (acpi_gbl_truncate_io_addresses) {
		address &= ACPI_UINT16_MAX;
	}

	/* Validate the entire request and perform the I/O */

	status = acpi_hw_validate_io_request(address, width);
	if (ACPI_SUCCESS(status)) {
		status = acpi_os_write_port(address, value, width);
		return status;
	}

	if (status != AE_AML_ILLEGAL_ADDRESS) {
		return status;
	}

	/*
	 * There has been a protection violation within the request. Fall
	 * back to byte granularity port I/O and ignore the failing bytes.
	 * This provides Windows compatibility.
	 */
	for (i = 0; i < width; i += 8) {

		/* Validate and write one byte */

		if (acpi_hw_validate_io_request(address, 8) == AE_OK) {
			status =
			    acpi_os_write_port(address, (value >> i) & 0xFF, 8);
			if (ACPI_FAILURE(status)) {
				return status;
			}
		}

		address++;
	}

	return AE_OK;
}
