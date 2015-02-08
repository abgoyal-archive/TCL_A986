



#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"
#include "acevents.h"

#define _COMPONENT          ACPI_HARDWARE
ACPI_MODULE_NAME("hwregs")

/* Local Prototypes */
static acpi_status
acpi_hw_read_multiple(u32 *value,
		      struct acpi_generic_address *register_a,
		      struct acpi_generic_address *register_b);

static acpi_status
acpi_hw_write_multiple(u32 value,
		       struct acpi_generic_address *register_a,
		       struct acpi_generic_address *register_b);


acpi_status
acpi_hw_validate_register(struct acpi_generic_address *reg,
			  u8 max_bit_width, u64 *address)
{

	/* Must have a valid pointer to a GAS structure */

	if (!reg) {
		return (AE_BAD_PARAMETER);
	}

	/*
	 * Copy the target address. This handles possible alignment issues.
	 * Address must not be null. A null address also indicates an optional
	 * ACPI register that is not supported, so no error message.
	 */
	ACPI_MOVE_64_TO_64(address, &reg->address);
	if (!(*address)) {
		return (AE_BAD_ADDRESS);
	}

	/* Validate the space_iD */

	if ((reg->space_id != ACPI_ADR_SPACE_SYSTEM_MEMORY) &&
	    (reg->space_id != ACPI_ADR_SPACE_SYSTEM_IO)) {
		ACPI_ERROR((AE_INFO,
			    "Unsupported address space: 0x%X", reg->space_id));
		return (AE_SUPPORT);
	}

	/* Validate the bit_width */

	if ((reg->bit_width != 8) &&
	    (reg->bit_width != 16) &&
	    (reg->bit_width != 32) && (reg->bit_width != max_bit_width)) {
		ACPI_ERROR((AE_INFO,
			    "Unsupported register bit width: 0x%X",
			    reg->bit_width));
		return (AE_SUPPORT);
	}

	/* Validate the bit_offset. Just a warning for now. */

	if (reg->bit_offset != 0) {
		ACPI_WARNING((AE_INFO,
			      "Unsupported register bit offset: 0x%X",
			      reg->bit_offset));
	}

	return (AE_OK);
}


acpi_status acpi_hw_read(u32 *value, struct acpi_generic_address *reg)
{
	u64 address;
	acpi_status status;

	ACPI_FUNCTION_NAME(hw_read);

	/* Validate contents of the GAS register */

	status = acpi_hw_validate_register(reg, 32, &address);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Initialize entire 32-bit return value to zero */

	*value = 0;

	/*
	 * Two address spaces supported: Memory or IO. PCI_Config is
	 * not supported here because the GAS structure is insufficient
	 */
	if (reg->space_id == ACPI_ADR_SPACE_SYSTEM_MEMORY) {
		status = acpi_os_read_memory((acpi_physical_address)
					     address, value, reg->bit_width);
	} else {		/* ACPI_ADR_SPACE_SYSTEM_IO, validated earlier */

		status = acpi_hw_read_port((acpi_io_address)
					   address, value, reg->bit_width);
	}

	ACPI_DEBUG_PRINT((ACPI_DB_IO,
			  "Read:  %8.8X width %2d from %8.8X%8.8X (%s)\n",
			  *value, reg->bit_width, ACPI_FORMAT_UINT64(address),
			  acpi_ut_get_region_name(reg->space_id)));

	return (status);
}


acpi_status acpi_hw_write(u32 value, struct acpi_generic_address *reg)
{
	u64 address;
	acpi_status status;

	ACPI_FUNCTION_NAME(hw_write);

	/* Validate contents of the GAS register */

	status = acpi_hw_validate_register(reg, 32, &address);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/*
	 * Two address spaces supported: Memory or IO. PCI_Config is
	 * not supported here because the GAS structure is insufficient
	 */
	if (reg->space_id == ACPI_ADR_SPACE_SYSTEM_MEMORY) {
		status = acpi_os_write_memory((acpi_physical_address)
					      address, value, reg->bit_width);
	} else {		/* ACPI_ADR_SPACE_SYSTEM_IO, validated earlier */

		status = acpi_hw_write_port((acpi_io_address)
					    address, value, reg->bit_width);
	}

	ACPI_DEBUG_PRINT((ACPI_DB_IO,
			  "Wrote: %8.8X width %2d   to %8.8X%8.8X (%s)\n",
			  value, reg->bit_width, ACPI_FORMAT_UINT64(address),
			  acpi_ut_get_region_name(reg->space_id)));

	return (status);
}


acpi_status acpi_hw_clear_acpi_status(void)
{
	acpi_status status;
	acpi_cpu_flags lock_flags = 0;

	ACPI_FUNCTION_TRACE(hw_clear_acpi_status);

	ACPI_DEBUG_PRINT((ACPI_DB_IO, "About to write %04X to %8.8X%8.8X\n",
			  ACPI_BITMASK_ALL_FIXED_STATUS,
			  ACPI_FORMAT_UINT64(acpi_gbl_xpm1a_status.address)));

	lock_flags = acpi_os_acquire_lock(acpi_gbl_hardware_lock);

	/* Clear the fixed events in PM1 A/B */

	status = acpi_hw_register_write(ACPI_REGISTER_PM1_STATUS,
					ACPI_BITMASK_ALL_FIXED_STATUS);
	if (ACPI_FAILURE(status)) {
		goto unlock_and_exit;
	}

	/* Clear the GPE Bits in all GPE registers in all GPE blocks */

	status = acpi_ev_walk_gpe_list(acpi_hw_clear_gpe_block, NULL);

      unlock_and_exit:
	acpi_os_release_lock(acpi_gbl_hardware_lock, lock_flags);
	return_ACPI_STATUS(status);
}


struct acpi_bit_register_info *acpi_hw_get_bit_register_info(u32 register_id)
{
	ACPI_FUNCTION_ENTRY();

	if (register_id > ACPI_BITREG_MAX) {
		ACPI_ERROR((AE_INFO, "Invalid BitRegister ID: 0x%X",
			    register_id));
		return (NULL);
	}

	return (&acpi_gbl_bit_register_info[register_id]);
}


acpi_status acpi_hw_write_pm1_control(u32 pm1a_control, u32 pm1b_control)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(hw_write_pm1_control);

	status =
	    acpi_hw_write(pm1a_control, &acpi_gbl_FADT.xpm1a_control_block);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	if (acpi_gbl_FADT.xpm1b_control_block.address) {
		status =
		    acpi_hw_write(pm1b_control,
				  &acpi_gbl_FADT.xpm1b_control_block);
	}
	return_ACPI_STATUS(status);
}

acpi_status
acpi_hw_register_read(u32 register_id, u32 * return_value)
{
	u32 value = 0;
	acpi_status status;

	ACPI_FUNCTION_TRACE(hw_register_read);

	switch (register_id) {
	case ACPI_REGISTER_PM1_STATUS:	/* PM1 A/B: 16-bit access each */

		status = acpi_hw_read_multiple(&value,
					       &acpi_gbl_xpm1a_status,
					       &acpi_gbl_xpm1b_status);
		break;

	case ACPI_REGISTER_PM1_ENABLE:	/* PM1 A/B: 16-bit access each */

		status = acpi_hw_read_multiple(&value,
					       &acpi_gbl_xpm1a_enable,
					       &acpi_gbl_xpm1b_enable);
		break;

	case ACPI_REGISTER_PM1_CONTROL:	/* PM1 A/B: 16-bit access each */

		status = acpi_hw_read_multiple(&value,
					       &acpi_gbl_FADT.
					       xpm1a_control_block,
					       &acpi_gbl_FADT.
					       xpm1b_control_block);

		/*
		 * Zero the write-only bits. From the ACPI specification, "Hardware
		 * Write-Only Bits": "Upon reads to registers with write-only bits,
		 * software masks out all write-only bits."
		 */
		value &= ~ACPI_PM1_CONTROL_WRITEONLY_BITS;
		break;

	case ACPI_REGISTER_PM2_CONTROL:	/* 8-bit access */

		status =
		    acpi_hw_read(&value, &acpi_gbl_FADT.xpm2_control_block);
		break;

	case ACPI_REGISTER_PM_TIMER:	/* 32-bit access */

		status = acpi_hw_read(&value, &acpi_gbl_FADT.xpm_timer_block);
		break;

	case ACPI_REGISTER_SMI_COMMAND_BLOCK:	/* 8-bit access */

		status =
		    acpi_hw_read_port(acpi_gbl_FADT.smi_command, &value, 8);
		break;

	default:
		ACPI_ERROR((AE_INFO, "Unknown Register ID: 0x%X", register_id));
		status = AE_BAD_PARAMETER;
		break;
	}

	if (ACPI_SUCCESS(status)) {
		*return_value = value;
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_hw_register_write(u32 register_id, u32 value)
{
	acpi_status status;
	u32 read_value;

	ACPI_FUNCTION_TRACE(hw_register_write);

	switch (register_id) {
	case ACPI_REGISTER_PM1_STATUS:	/* PM1 A/B: 16-bit access each */
		/*
		 * Handle the "ignored" bit in PM1 Status. According to the ACPI
		 * specification, ignored bits are to be preserved when writing.
		 * Normally, this would mean a read/modify/write sequence. However,
		 * preserving a bit in the status register is different. Writing a
		 * one clears the status, and writing a zero preserves the status.
		 * Therefore, we must always write zero to the ignored bit.
		 *
		 * This behavior is clarified in the ACPI 4.0 specification.
		 */
		value &= ~ACPI_PM1_STATUS_PRESERVED_BITS;

		status = acpi_hw_write_multiple(value,
						&acpi_gbl_xpm1a_status,
						&acpi_gbl_xpm1b_status);
		break;

	case ACPI_REGISTER_PM1_ENABLE:	/* PM1 A/B: 16-bit access */

		status = acpi_hw_write_multiple(value,
						&acpi_gbl_xpm1a_enable,
						&acpi_gbl_xpm1b_enable);
		break;

	case ACPI_REGISTER_PM1_CONTROL:	/* PM1 A/B: 16-bit access each */

		/*
		 * Perform a read first to preserve certain bits (per ACPI spec)
		 * Note: This includes SCI_EN, we never want to change this bit
		 */
		status = acpi_hw_read_multiple(&read_value,
					       &acpi_gbl_FADT.
					       xpm1a_control_block,
					       &acpi_gbl_FADT.
					       xpm1b_control_block);
		if (ACPI_FAILURE(status)) {
			goto exit;
		}

		/* Insert the bits to be preserved */

		ACPI_INSERT_BITS(value, ACPI_PM1_CONTROL_PRESERVED_BITS,
				 read_value);

		/* Now we can write the data */

		status = acpi_hw_write_multiple(value,
						&acpi_gbl_FADT.
						xpm1a_control_block,
						&acpi_gbl_FADT.
						xpm1b_control_block);
		break;

	case ACPI_REGISTER_PM2_CONTROL:	/* 8-bit access */

		/*
		 * For control registers, all reserved bits must be preserved,
		 * as per the ACPI spec.
		 */
		status =
		    acpi_hw_read(&read_value,
				 &acpi_gbl_FADT.xpm2_control_block);
		if (ACPI_FAILURE(status)) {
			goto exit;
		}

		/* Insert the bits to be preserved */

		ACPI_INSERT_BITS(value, ACPI_PM2_CONTROL_PRESERVED_BITS,
				 read_value);

		status =
		    acpi_hw_write(value, &acpi_gbl_FADT.xpm2_control_block);
		break;

	case ACPI_REGISTER_PM_TIMER:	/* 32-bit access */

		status = acpi_hw_write(value, &acpi_gbl_FADT.xpm_timer_block);
		break;

	case ACPI_REGISTER_SMI_COMMAND_BLOCK:	/* 8-bit access */

		/* SMI_CMD is currently always in IO space */

		status =
		    acpi_hw_write_port(acpi_gbl_FADT.smi_command, value, 8);
		break;

	default:
		ACPI_ERROR((AE_INFO, "Unknown Register ID: 0x%X", register_id));
		status = AE_BAD_PARAMETER;
		break;
	}

      exit:
	return_ACPI_STATUS(status);
}


static acpi_status
acpi_hw_read_multiple(u32 *value,
		      struct acpi_generic_address *register_a,
		      struct acpi_generic_address *register_b)
{
	u32 value_a = 0;
	u32 value_b = 0;
	acpi_status status;

	/* The first register is always required */

	status = acpi_hw_read(&value_a, register_a);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Second register is optional */

	if (register_b->address) {
		status = acpi_hw_read(&value_b, register_b);
		if (ACPI_FAILURE(status)) {
			return (status);
		}
	}

	/*
	 * OR the two return values together. No shifting or masking is necessary,
	 * because of how the PM1 registers are defined in the ACPI specification:
	 *
	 * "Although the bits can be split between the two register blocks (each
	 * register block has a unique pointer within the FADT), the bit positions
	 * are maintained. The register block with unimplemented bits (that is,
	 * those implemented in the other register block) always returns zeros,
	 * and writes have no side effects"
	 */
	*value = (value_a | value_b);
	return (AE_OK);
}


static acpi_status
acpi_hw_write_multiple(u32 value,
		       struct acpi_generic_address *register_a,
		       struct acpi_generic_address *register_b)
{
	acpi_status status;

	/* The first register is always required */

	status = acpi_hw_write(value, register_a);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/*
	 * Second register is optional
	 *
	 * No bit shifting or clearing is necessary, because of how the PM1
	 * registers are defined in the ACPI specification:
	 *
	 * "Although the bits can be split between the two register blocks (each
	 * register block has a unique pointer within the FADT), the bit positions
	 * are maintained. The register block with unimplemented bits (that is,
	 * those implemented in the other register block) always returns zeros,
	 * and writes have no side effects"
	 */
	if (register_b->address) {
		status = acpi_hw_write(value, register_b);
	}

	return (status);
}
