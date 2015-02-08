


#include <acpi/acpi.h>
#include "accommon.h"
#include "acparser.h"
#include "acinterp.h"
#include "acevents.h"
#include "amlcode.h"

#define _COMPONENT          ACPI_EXECUTER
ACPI_MODULE_NAME("exoparg2")

acpi_status acpi_ex_opcode_2A_0T_0R(struct acpi_walk_state *walk_state)
{
	union acpi_operand_object **operand = &walk_state->operands[0];
	struct acpi_namespace_node *node;
	u32 value;
	acpi_status status = AE_OK;

	ACPI_FUNCTION_TRACE_STR(ex_opcode_2A_0T_0R,
				acpi_ps_get_opcode_name(walk_state->opcode));

	/* Examine the opcode */

	switch (walk_state->opcode) {
	case AML_NOTIFY_OP:	/* Notify (notify_object, notify_value) */

		/* The first operand is a namespace node */

		node = (struct acpi_namespace_node *)operand[0];

		/* Second value is the notify value */

		value = (u32) operand[1]->integer.value;

		/* Are notifies allowed on this object? */

		if (!acpi_ev_is_notify_object(node)) {
			ACPI_ERROR((AE_INFO,
				    "Unexpected notify object type [%s]",
				    acpi_ut_get_type_name(node->type)));

			status = AE_AML_OPERAND_TYPE;
			break;
		}

		/*
		 * Dispatch the notify to the appropriate handler
		 * NOTE: the request is queued for execution after this method
		 * completes.  The notify handlers are NOT invoked synchronously
		 * from this thread -- because handlers may in turn run other
		 * control methods.
		 */
		status = acpi_ev_queue_notify_request(node, value);
		break;

	default:

		ACPI_ERROR((AE_INFO, "Unknown AML opcode 0x%X",
			    walk_state->opcode));
		status = AE_AML_BAD_OPCODE;
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_ex_opcode_2A_2T_1R(struct acpi_walk_state *walk_state)
{
	union acpi_operand_object **operand = &walk_state->operands[0];
	union acpi_operand_object *return_desc1 = NULL;
	union acpi_operand_object *return_desc2 = NULL;
	acpi_status status;

	ACPI_FUNCTION_TRACE_STR(ex_opcode_2A_2T_1R,
				acpi_ps_get_opcode_name(walk_state->opcode));

	/* Execute the opcode */

	switch (walk_state->opcode) {
	case AML_DIVIDE_OP:

		/* Divide (Dividend, Divisor, remainder_result quotient_result) */

		return_desc1 =
		    acpi_ut_create_internal_object(ACPI_TYPE_INTEGER);
		if (!return_desc1) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		return_desc2 =
		    acpi_ut_create_internal_object(ACPI_TYPE_INTEGER);
		if (!return_desc2) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		/* Quotient to return_desc1, remainder to return_desc2 */

		status = acpi_ut_divide(operand[0]->integer.value,
					operand[1]->integer.value,
					&return_desc1->integer.value,
					&return_desc2->integer.value);
		if (ACPI_FAILURE(status)) {
			goto cleanup;
		}
		break;

	default:

		ACPI_ERROR((AE_INFO, "Unknown AML opcode 0x%X",
			    walk_state->opcode));
		status = AE_AML_BAD_OPCODE;
		goto cleanup;
	}

	/* Store the results to the target reference operands */

	status = acpi_ex_store(return_desc2, operand[2], walk_state);
	if (ACPI_FAILURE(status)) {
		goto cleanup;
	}

	status = acpi_ex_store(return_desc1, operand[3], walk_state);
	if (ACPI_FAILURE(status)) {
		goto cleanup;
	}

      cleanup:
	/*
	 * Since the remainder is not returned indirectly, remove a reference to
	 * it. Only the quotient is returned indirectly.
	 */
	acpi_ut_remove_reference(return_desc2);

	if (ACPI_FAILURE(status)) {

		/* Delete the return object */

		acpi_ut_remove_reference(return_desc1);
	}

	/* Save return object (the remainder) on success */

	else {
		walk_state->result_obj = return_desc1;
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_ex_opcode_2A_1T_1R(struct acpi_walk_state *walk_state)
{
	union acpi_operand_object **operand = &walk_state->operands[0];
	union acpi_operand_object *return_desc = NULL;
	u64 index;
	acpi_status status = AE_OK;
	acpi_size length;

	ACPI_FUNCTION_TRACE_STR(ex_opcode_2A_1T_1R,
				acpi_ps_get_opcode_name(walk_state->opcode));

	/* Execute the opcode */

	if (walk_state->op_info->flags & AML_MATH) {

		/* All simple math opcodes (add, etc.) */

		return_desc = acpi_ut_create_internal_object(ACPI_TYPE_INTEGER);
		if (!return_desc) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		return_desc->integer.value =
		    acpi_ex_do_math_op(walk_state->opcode,
				       operand[0]->integer.value,
				       operand[1]->integer.value);
		goto store_result_to_target;
	}

	switch (walk_state->opcode) {
	case AML_MOD_OP:	/* Mod (Dividend, Divisor, remainder_result (ACPI 2.0) */

		return_desc = acpi_ut_create_internal_object(ACPI_TYPE_INTEGER);
		if (!return_desc) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		/* return_desc will contain the remainder */

		status = acpi_ut_divide(operand[0]->integer.value,
					operand[1]->integer.value,
					NULL, &return_desc->integer.value);
		break;

	case AML_CONCAT_OP:	/* Concatenate (Data1, Data2, Result) */

		status = acpi_ex_do_concatenate(operand[0], operand[1],
						&return_desc, walk_state);
		break;

	case AML_TO_STRING_OP:	/* to_string (Buffer, Length, Result) (ACPI 2.0) */

		/*
		 * Input object is guaranteed to be a buffer at this point (it may have
		 * been converted.)  Copy the raw buffer data to a new object of
		 * type String.
		 */

		/*
		 * Get the length of the new string. It is the smallest of:
		 * 1) Length of the input buffer
		 * 2) Max length as specified in the to_string operator
		 * 3) Length of input buffer up to a zero byte (null terminator)
		 *
		 * NOTE: A length of zero is ok, and will create a zero-length, null
		 *       terminated string.
		 */
		length = 0;
		while ((length < operand[0]->buffer.length) &&
		       (length < operand[1]->integer.value) &&
		       (operand[0]->buffer.pointer[length])) {
			length++;
		}

		/* Allocate a new string object */

		return_desc = acpi_ut_create_string_object(length);
		if (!return_desc) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		/*
		 * Copy the raw buffer data with no transform.
		 * (NULL terminated already)
		 */
		ACPI_MEMCPY(return_desc->string.pointer,
			    operand[0]->buffer.pointer, length);
		break;

	case AML_CONCAT_RES_OP:

		/* concatenate_res_template (Buffer, Buffer, Result) (ACPI 2.0) */

		status = acpi_ex_concat_template(operand[0], operand[1],
						 &return_desc, walk_state);
		break;

	case AML_INDEX_OP:	/* Index (Source Index Result) */

		/* Create the internal return object */

		return_desc =
		    acpi_ut_create_internal_object(ACPI_TYPE_LOCAL_REFERENCE);
		if (!return_desc) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		/* Initialize the Index reference object */

		index = operand[1]->integer.value;
		return_desc->reference.value = (u32) index;
		return_desc->reference.class = ACPI_REFCLASS_INDEX;

		/*
		 * At this point, the Source operand is a String, Buffer, or Package.
		 * Verify that the index is within range.
		 */
		switch ((operand[0])->common.type) {
		case ACPI_TYPE_STRING:

			if (index >= operand[0]->string.length) {
				status = AE_AML_STRING_LIMIT;
			}

			return_desc->reference.target_type =
			    ACPI_TYPE_BUFFER_FIELD;
			break;

		case ACPI_TYPE_BUFFER:

			if (index >= operand[0]->buffer.length) {
				status = AE_AML_BUFFER_LIMIT;
			}

			return_desc->reference.target_type =
			    ACPI_TYPE_BUFFER_FIELD;
			break;

		case ACPI_TYPE_PACKAGE:

			if (index >= operand[0]->package.count) {
				status = AE_AML_PACKAGE_LIMIT;
			}

			return_desc->reference.target_type = ACPI_TYPE_PACKAGE;
			return_desc->reference.where =
			    &operand[0]->package.elements[index];
			break;

		default:

			status = AE_AML_INTERNAL;
			goto cleanup;
		}

		/* Failure means that the Index was beyond the end of the object */

		if (ACPI_FAILURE(status)) {
			ACPI_EXCEPTION((AE_INFO, status,
					"Index (0x%8.8X%8.8X) is beyond end of object",
					ACPI_FORMAT_UINT64(index)));
			goto cleanup;
		}

		/*
		 * Save the target object and add a reference to it for the life
		 * of the index
		 */
		return_desc->reference.object = operand[0];
		acpi_ut_add_reference(operand[0]);

		/* Store the reference to the Target */

		status = acpi_ex_store(return_desc, operand[2], walk_state);

		/* Return the reference */

		walk_state->result_obj = return_desc;
		goto cleanup;

	default:

		ACPI_ERROR((AE_INFO, "Unknown AML opcode 0x%X",
			    walk_state->opcode));
		status = AE_AML_BAD_OPCODE;
		break;
	}

      store_result_to_target:

	if (ACPI_SUCCESS(status)) {
		/*
		 * Store the result of the operation (which is now in return_desc) into
		 * the Target descriptor.
		 */
		status = acpi_ex_store(return_desc, operand[2], walk_state);
		if (ACPI_FAILURE(status)) {
			goto cleanup;
		}

		if (!walk_state->result_obj) {
			walk_state->result_obj = return_desc;
		}
	}

      cleanup:

	/* Delete return object on error */

	if (ACPI_FAILURE(status)) {
		acpi_ut_remove_reference(return_desc);
		walk_state->result_obj = NULL;
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_ex_opcode_2A_0T_1R(struct acpi_walk_state *walk_state)
{
	union acpi_operand_object **operand = &walk_state->operands[0];
	union acpi_operand_object *return_desc = NULL;
	acpi_status status = AE_OK;
	u8 logical_result = FALSE;

	ACPI_FUNCTION_TRACE_STR(ex_opcode_2A_0T_1R,
				acpi_ps_get_opcode_name(walk_state->opcode));

	/* Create the internal return object */

	return_desc = acpi_ut_create_internal_object(ACPI_TYPE_INTEGER);
	if (!return_desc) {
		status = AE_NO_MEMORY;
		goto cleanup;
	}

	/* Execute the Opcode */

	if (walk_state->op_info->flags & AML_LOGICAL_NUMERIC) {

		/* logical_op (Operand0, Operand1) */

		status = acpi_ex_do_logical_numeric_op(walk_state->opcode,
						       operand[0]->integer.
						       value,
						       operand[1]->integer.
						       value, &logical_result);
		goto store_logical_result;
	} else if (walk_state->op_info->flags & AML_LOGICAL) {

		/* logical_op (Operand0, Operand1) */

		status = acpi_ex_do_logical_op(walk_state->opcode, operand[0],
					       operand[1], &logical_result);
		goto store_logical_result;
	}

	switch (walk_state->opcode) {
	case AML_ACQUIRE_OP:	/* Acquire (mutex_object, Timeout) */

		status =
		    acpi_ex_acquire_mutex(operand[1], operand[0], walk_state);
		if (status == AE_TIME) {
			logical_result = TRUE;	/* TRUE = Acquire timed out */
			status = AE_OK;
		}
		break;

	case AML_WAIT_OP:	/* Wait (event_object, Timeout) */

		status = acpi_ex_system_wait_event(operand[1], operand[0]);
		if (status == AE_TIME) {
			logical_result = TRUE;	/* TRUE, Wait timed out */
			status = AE_OK;
		}
		break;

	default:

		ACPI_ERROR((AE_INFO, "Unknown AML opcode 0x%X",
			    walk_state->opcode));
		status = AE_AML_BAD_OPCODE;
		goto cleanup;
	}

      store_logical_result:
	/*
	 * Set return value to according to logical_result. logical TRUE (all ones)
	 * Default is FALSE (zero)
	 */
	if (logical_result) {
		return_desc->integer.value = ACPI_UINT64_MAX;
	}

      cleanup:

	/* Delete return object on error */

	if (ACPI_FAILURE(status)) {
		acpi_ut_remove_reference(return_desc);
	}

	/* Save return object on success */

	else {
		walk_state->result_obj = return_desc;
	}

	return_ACPI_STATUS(status);
}
