




#define DEFINE_AML_GLOBALS

#include <acpi/acpi.h>
#include "accommon.h"
#include "acinterp.h"
#include "amlcode.h"

#define _COMPONENT          ACPI_EXECUTER
ACPI_MODULE_NAME("exutils")

/* Local prototypes */
static u32 acpi_ex_digits_needed(u64 value, u32 base);

#ifndef ACPI_NO_METHOD_EXECUTION

void acpi_ex_enter_interpreter(void)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ex_enter_interpreter);

	status = acpi_ut_acquire_mutex(ACPI_MTX_INTERPRETER);
	if (ACPI_FAILURE(status)) {
		ACPI_ERROR((AE_INFO,
			    "Could not acquire AML Interpreter mutex"));
	}

	return_VOID;
}


void acpi_ex_reacquire_interpreter(void)
{
	ACPI_FUNCTION_TRACE(ex_reacquire_interpreter);

	/*
	 * If the global serialized flag is set, do not release the interpreter,
	 * since it was not actually released by acpi_ex_relinquish_interpreter.
	 * This forces the interpreter to be single threaded.
	 */
	if (!acpi_gbl_all_methods_serialized) {
		acpi_ex_enter_interpreter();
	}

	return_VOID;
}


void acpi_ex_exit_interpreter(void)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ex_exit_interpreter);

	status = acpi_ut_release_mutex(ACPI_MTX_INTERPRETER);
	if (ACPI_FAILURE(status)) {
		ACPI_ERROR((AE_INFO,
			    "Could not release AML Interpreter mutex"));
	}

	return_VOID;
}


void acpi_ex_relinquish_interpreter(void)
{
	ACPI_FUNCTION_TRACE(ex_relinquish_interpreter);

	/*
	 * If the global serialized flag is set, do not release the interpreter.
	 * This forces the interpreter to be single threaded.
	 */
	if (!acpi_gbl_all_methods_serialized) {
		acpi_ex_exit_interpreter();
	}

	return_VOID;
}


void acpi_ex_truncate_for32bit_table(union acpi_operand_object *obj_desc)
{

	ACPI_FUNCTION_ENTRY();

	/*
	 * Object must be a valid number and we must be executing
	 * a control method. NS node could be there for AML_INT_NAMEPATH_OP.
	 */
	if ((!obj_desc) ||
	    (ACPI_GET_DESCRIPTOR_TYPE(obj_desc) != ACPI_DESC_TYPE_OPERAND) ||
	    (obj_desc->common.type != ACPI_TYPE_INTEGER)) {
		return;
	}

	if (acpi_gbl_integer_byte_width == 4) {
		/*
		 * We are running a method that exists in a 32-bit ACPI table.
		 * Truncate the value to 32 bits by zeroing out the upper 32-bit field
		 */
		obj_desc->integer.value &= (u64) ACPI_UINT32_MAX;
	}
}


void acpi_ex_acquire_global_lock(u32 field_flags)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ex_acquire_global_lock);

	/* Only use the lock if the always_lock bit is set */

	if (!(field_flags & AML_FIELD_LOCK_RULE_MASK)) {
		return_VOID;
	}

	/* Attempt to get the global lock, wait forever */

	status = acpi_ex_acquire_mutex_object(ACPI_WAIT_FOREVER,
					      acpi_gbl_global_lock_mutex,
					      acpi_os_get_thread_id());

	if (ACPI_FAILURE(status)) {
		ACPI_EXCEPTION((AE_INFO, status,
				"Could not acquire Global Lock"));
	}

	return_VOID;
}


void acpi_ex_release_global_lock(u32 field_flags)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ex_release_global_lock);

	/* Only use the lock if the always_lock bit is set */

	if (!(field_flags & AML_FIELD_LOCK_RULE_MASK)) {
		return_VOID;
	}

	/* Release the global lock */

	status = acpi_ex_release_mutex_object(acpi_gbl_global_lock_mutex);
	if (ACPI_FAILURE(status)) {

		/* Report the error, but there isn't much else we can do */

		ACPI_EXCEPTION((AE_INFO, status,
				"Could not release Global Lock"));
	}

	return_VOID;
}


static u32 acpi_ex_digits_needed(u64 value, u32 base)
{
	u32 num_digits;
	u64 current_value;

	ACPI_FUNCTION_TRACE(ex_digits_needed);

	/* u64 is unsigned, so we don't worry about a '-' prefix */

	if (value == 0) {
		return_UINT32(1);
	}

	current_value = value;
	num_digits = 0;

	/* Count the digits in the requested base */

	while (current_value) {
		(void)acpi_ut_short_divide(current_value, base, &current_value,
					   NULL);
		num_digits++;
	}

	return_UINT32(num_digits);
}


void acpi_ex_eisa_id_to_string(char *out_string, u64 compressed_id)
{
	u32 swapped_id;

	ACPI_FUNCTION_ENTRY();

	/* The EISAID should be a 32-bit integer */

	if (compressed_id > ACPI_UINT32_MAX) {
		ACPI_WARNING((AE_INFO,
			      "Expected EISAID is larger than 32 bits: 0x%8.8X%8.8X, truncating",
			      ACPI_FORMAT_UINT64(compressed_id)));
	}

	/* Swap ID to big-endian to get contiguous bits */

	swapped_id = acpi_ut_dword_byte_swap((u32)compressed_id);

	/* First 3 bytes are uppercase letters. Next 4 bytes are hexadecimal */

	out_string[0] =
	    (char)(0x40 + (((unsigned long)swapped_id >> 26) & 0x1F));
	out_string[1] = (char)(0x40 + ((swapped_id >> 21) & 0x1F));
	out_string[2] = (char)(0x40 + ((swapped_id >> 16) & 0x1F));
	out_string[3] = acpi_ut_hex_to_ascii_char((u64) swapped_id, 12);
	out_string[4] = acpi_ut_hex_to_ascii_char((u64) swapped_id, 8);
	out_string[5] = acpi_ut_hex_to_ascii_char((u64) swapped_id, 4);
	out_string[6] = acpi_ut_hex_to_ascii_char((u64) swapped_id, 0);
	out_string[7] = 0;
}


void acpi_ex_integer_to_string(char *out_string, u64 value)
{
	u32 count;
	u32 digits_needed;
	u32 remainder;

	ACPI_FUNCTION_ENTRY();

	digits_needed = acpi_ex_digits_needed(value, 10);
	out_string[digits_needed] = 0;

	for (count = digits_needed; count > 0; count--) {
		(void)acpi_ut_short_divide(value, 10, &value, &remainder);
		out_string[count - 1] = (char)('0' + remainder);
	}
}

#endif
