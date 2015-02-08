


#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"

#define _COMPONENT          ACPI_NAMESPACE
ACPI_MODULE_NAME("nsxfobj")

acpi_status acpi_get_id(acpi_handle handle, acpi_owner_id * ret_id)
{
	struct acpi_namespace_node *node;
	acpi_status status;

	/* Parameter Validation */

	if (!ret_id) {
		return (AE_BAD_PARAMETER);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Convert and validate the handle */

	node = acpi_ns_validate_handle(handle);
	if (!node) {
		(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
		return (AE_BAD_PARAMETER);
	}

	*ret_id = node->owner_id;

	status = acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_get_id)

acpi_status acpi_get_type(acpi_handle handle, acpi_object_type * ret_type)
{
	struct acpi_namespace_node *node;
	acpi_status status;

	/* Parameter Validation */

	if (!ret_type) {
		return (AE_BAD_PARAMETER);
	}

	/*
	 * Special case for the predefined Root Node
	 * (return type ANY)
	 */
	if (handle == ACPI_ROOT_OBJECT) {
		*ret_type = ACPI_TYPE_ANY;
		return (AE_OK);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Convert and validate the handle */

	node = acpi_ns_validate_handle(handle);
	if (!node) {
		(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
		return (AE_BAD_PARAMETER);
	}

	*ret_type = node->type;

	status = acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_get_type)

acpi_status acpi_get_parent(acpi_handle handle, acpi_handle * ret_handle)
{
	struct acpi_namespace_node *node;
	struct acpi_namespace_node *parent_node;
	acpi_status status;

	if (!ret_handle) {
		return (AE_BAD_PARAMETER);
	}

	/* Special case for the predefined Root Node (no parent) */

	if (handle == ACPI_ROOT_OBJECT) {
		return (AE_NULL_ENTRY);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Convert and validate the handle */

	node = acpi_ns_validate_handle(handle);
	if (!node) {
		status = AE_BAD_PARAMETER;
		goto unlock_and_exit;
	}

	/* Get the parent entry */

	parent_node = acpi_ns_get_parent_node(node);
	*ret_handle = ACPI_CAST_PTR(acpi_handle, parent_node);

	/* Return exception if parent is null */

	if (!parent_node) {
		status = AE_NULL_ENTRY;
	}

      unlock_and_exit:

	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_get_parent)

acpi_status
acpi_get_next_object(acpi_object_type type,
		     acpi_handle parent,
		     acpi_handle child, acpi_handle * ret_handle)
{
	acpi_status status;
	struct acpi_namespace_node *node;
	struct acpi_namespace_node *parent_node = NULL;
	struct acpi_namespace_node *child_node = NULL;

	/* Parameter validation */

	if (type > ACPI_TYPE_EXTERNAL_MAX) {
		return (AE_BAD_PARAMETER);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* If null handle, use the parent */

	if (!child) {

		/* Start search at the beginning of the specified scope */

		parent_node = acpi_ns_validate_handle(parent);
		if (!parent_node) {
			status = AE_BAD_PARAMETER;
			goto unlock_and_exit;
		}
	} else {
		/* Non-null handle, ignore the parent */
		/* Convert and validate the handle */

		child_node = acpi_ns_validate_handle(child);
		if (!child_node) {
			status = AE_BAD_PARAMETER;
			goto unlock_and_exit;
		}
	}

	/* Internal function does the real work */

	node = acpi_ns_get_next_node_typed(type, parent_node, child_node);
	if (!node) {
		status = AE_NOT_FOUND;
		goto unlock_and_exit;
	}

	if (ret_handle) {
		*ret_handle = ACPI_CAST_PTR(acpi_handle, node);
	}

      unlock_and_exit:

	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_get_next_object)
