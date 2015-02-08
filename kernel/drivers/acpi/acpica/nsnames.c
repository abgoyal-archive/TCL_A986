


#include <acpi/acpi.h>
#include "accommon.h"
#include "amlcode.h"
#include "acnamesp.h"

#define _COMPONENT          ACPI_NAMESPACE
ACPI_MODULE_NAME("nsnames")

acpi_status
acpi_ns_build_external_path(struct acpi_namespace_node *node,
			    acpi_size size, char *name_buffer)
{
	acpi_size index;
	struct acpi_namespace_node *parent_node;

	ACPI_FUNCTION_ENTRY();

	/* Special case for root */

	index = size - 1;
	if (index < ACPI_NAME_SIZE) {
		name_buffer[0] = AML_ROOT_PREFIX;
		name_buffer[1] = 0;
		return (AE_OK);
	}

	/* Store terminator byte, then build name backwards */

	parent_node = node;
	name_buffer[index] = 0;

	while ((index > ACPI_NAME_SIZE) && (parent_node != acpi_gbl_root_node)) {
		index -= ACPI_NAME_SIZE;

		/* Put the name into the buffer */

		ACPI_MOVE_32_TO_32((name_buffer + index), &parent_node->name);
		parent_node = acpi_ns_get_parent_node(parent_node);

		/* Prefix name with the path separator */

		index--;
		name_buffer[index] = ACPI_PATH_SEPARATOR;
	}

	/* Overwrite final separator with the root prefix character */

	name_buffer[index] = AML_ROOT_PREFIX;

	if (index != 0) {
		ACPI_ERROR((AE_INFO,
			    "Could not construct external pathname; index=%u, size=%u, Path=%s",
			    (u32) index, (u32) size, &name_buffer[size]));

		return (AE_BAD_PARAMETER);
	}

	return (AE_OK);
}


char *acpi_ns_get_external_pathname(struct acpi_namespace_node *node)
{
	acpi_status status;
	char *name_buffer;
	acpi_size size;

	ACPI_FUNCTION_TRACE_PTR(ns_get_external_pathname, node);

	/* Calculate required buffer size based on depth below root */

	size = acpi_ns_get_pathname_length(node);
	if (!size) {
		return_PTR(NULL);
	}

	/* Allocate a buffer to be returned to caller */

	name_buffer = ACPI_ALLOCATE_ZEROED(size);
	if (!name_buffer) {
		ACPI_ERROR((AE_INFO, "Could not allocate %u bytes", (u32)size));
		return_PTR(NULL);
	}

	/* Build the path in the allocated buffer */

	status = acpi_ns_build_external_path(node, size, name_buffer);
	if (ACPI_FAILURE(status)) {
		ACPI_FREE(name_buffer);
		return_PTR(NULL);
	}

	return_PTR(name_buffer);
}


acpi_size acpi_ns_get_pathname_length(struct acpi_namespace_node *node)
{
	acpi_size size;
	struct acpi_namespace_node *next_node;

	ACPI_FUNCTION_ENTRY();

	/*
	 * Compute length of pathname as 5 * number of name segments.
	 * Go back up the parent tree to the root
	 */
	size = 0;
	next_node = node;

	while (next_node && (next_node != acpi_gbl_root_node)) {
		if (ACPI_GET_DESCRIPTOR_TYPE(next_node) != ACPI_DESC_TYPE_NAMED) {
			ACPI_ERROR((AE_INFO,
				    "Invalid Namespace Node (%p) while traversing namespace",
				    next_node));
			return 0;
		}
		size += ACPI_PATH_SEGMENT_LENGTH;
		next_node = acpi_ns_get_parent_node(next_node);
	}

	if (!size) {
		size = 1;	/* Root node case */
	}

	return (size + 1);	/* +1 for null string terminator */
}


acpi_status
acpi_ns_handle_to_pathname(acpi_handle target_handle,
			   struct acpi_buffer * buffer)
{
	acpi_status status;
	struct acpi_namespace_node *node;
	acpi_size required_size;

	ACPI_FUNCTION_TRACE_PTR(ns_handle_to_pathname, target_handle);

	node = acpi_ns_validate_handle(target_handle);
	if (!node) {
		return_ACPI_STATUS(AE_BAD_PARAMETER);
	}

	/* Determine size required for the caller buffer */

	required_size = acpi_ns_get_pathname_length(node);
	if (!required_size) {
		return_ACPI_STATUS(AE_BAD_PARAMETER);
	}

	/* Validate/Allocate/Clear caller buffer */

	status = acpi_ut_initialize_buffer(buffer, required_size);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	/* Build the path in the caller buffer */

	status =
	    acpi_ns_build_external_path(node, required_size, buffer->pointer);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	ACPI_DEBUG_PRINT((ACPI_DB_EXEC, "%s [%X]\n",
			  (char *)buffer->pointer, (u32) required_size));
	return_ACPI_STATUS(AE_OK);
}
