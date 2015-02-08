


#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"
#include "acdispat.h"
#include "actables.h"

#define _COMPONENT          ACPI_NAMESPACE
ACPI_MODULE_NAME("nsload")

/* Local prototypes */
#ifdef ACPI_FUTURE_IMPLEMENTATION
acpi_status acpi_ns_unload_namespace(acpi_handle handle);

static acpi_status acpi_ns_delete_subtree(acpi_handle start_handle);
#endif

#ifndef ACPI_NO_METHOD_EXECUTION

acpi_status
acpi_ns_load_table(u32 table_index, struct acpi_namespace_node *node)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ns_load_table);

	/*
	 * Parse the table and load the namespace with all named
	 * objects found within.  Control methods are NOT parsed
	 * at this time.  In fact, the control methods cannot be
	 * parsed until the entire namespace is loaded, because
	 * if a control method makes a forward reference (call)
	 * to another control method, we can't continue parsing
	 * because we don't know how many arguments to parse next!
	 */
	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	/* If table already loaded into namespace, just return */

	if (acpi_tb_is_table_loaded(table_index)) {
		status = AE_ALREADY_EXISTS;
		goto unlock;
	}

	ACPI_DEBUG_PRINT((ACPI_DB_INFO,
			  "**** Loading table into namespace ****\n"));

	status = acpi_tb_allocate_owner_id(table_index);
	if (ACPI_FAILURE(status)) {
		goto unlock;
	}

	status = acpi_ns_parse_table(table_index, node);
	if (ACPI_SUCCESS(status)) {
		acpi_tb_set_table_loaded_flag(table_index, TRUE);
	} else {
		(void)acpi_tb_release_owner_id(table_index);
	}

      unlock:
	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);

	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	/*
	 * Now we can parse the control methods.  We always parse
	 * them here for a sanity check, and if configured for
	 * just-in-time parsing, we delete the control method
	 * parse trees.
	 */
	ACPI_DEBUG_PRINT((ACPI_DB_INFO,
			  "**** Begin Table Method Parsing and Object Initialization\n"));

	status = acpi_ds_initialize_objects(table_index, node);

	ACPI_DEBUG_PRINT((ACPI_DB_INFO,
			  "**** Completed Table Method Parsing and Object Initialization\n"));

	return_ACPI_STATUS(status);
}

#ifdef ACPI_OBSOLETE_FUNCTIONS

acpi_status acpi_ns_load_namespace(void)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(acpi_load_name_space);

	/* There must be at least a DSDT installed */

	if (acpi_gbl_DSDT == NULL) {
		ACPI_ERROR((AE_INFO, "DSDT is not in memory"));
		return_ACPI_STATUS(AE_NO_ACPI_TABLES);
	}

	/*
	 * Load the namespace.  The DSDT is required,
	 * but the SSDT and PSDT tables are optional.
	 */
	status = acpi_ns_load_table_by_type(ACPI_TABLE_ID_DSDT);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	/* Ignore exceptions from these */

	(void)acpi_ns_load_table_by_type(ACPI_TABLE_ID_SSDT);
	(void)acpi_ns_load_table_by_type(ACPI_TABLE_ID_PSDT);

	ACPI_DEBUG_PRINT_RAW((ACPI_DB_INIT,
			      "ACPI Namespace successfully loaded at root %p\n",
			      acpi_gbl_root_node));

	return_ACPI_STATUS(status);
}
#endif

#ifdef ACPI_FUTURE_IMPLEMENTATION

static acpi_status acpi_ns_delete_subtree(acpi_handle start_handle)
{
	acpi_status status;
	acpi_handle child_handle;
	acpi_handle parent_handle;
	acpi_handle next_child_handle;
	acpi_handle dummy;
	u32 level;

	ACPI_FUNCTION_TRACE(ns_delete_subtree);

	parent_handle = start_handle;
	child_handle = NULL;
	level = 1;

	/*
	 * Traverse the tree of objects until we bubble back up
	 * to where we started.
	 */
	while (level > 0) {

		/* Attempt to get the next object in this scope */

		status = acpi_get_next_object(ACPI_TYPE_ANY, parent_handle,
					      child_handle, &next_child_handle);

		child_handle = next_child_handle;

		/* Did we get a new object? */

		if (ACPI_SUCCESS(status)) {

			/* Check if this object has any children */

			if (ACPI_SUCCESS
			    (acpi_get_next_object
			     (ACPI_TYPE_ANY, child_handle, NULL, &dummy))) {
				/*
				 * There is at least one child of this object,
				 * visit the object
				 */
				level++;
				parent_handle = child_handle;
				child_handle = NULL;
			}
		} else {
			/*
			 * No more children in this object, go back up to
			 * the object's parent
			 */
			level--;

			/* Delete all children now */

			acpi_ns_delete_children(child_handle);

			child_handle = parent_handle;
			status = acpi_get_parent(parent_handle, &parent_handle);
			if (ACPI_FAILURE(status)) {
				return_ACPI_STATUS(status);
			}
		}
	}

	/* Now delete the starting object, and we are done */

	acpi_ns_remove_node(child_handle);
	return_ACPI_STATUS(AE_OK);
}


acpi_status acpi_ns_unload_namespace(acpi_handle handle)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ns_unload_name_space);

	/* Parameter validation */

	if (!acpi_gbl_root_node) {
		return_ACPI_STATUS(AE_NO_NAMESPACE);
	}

	if (!handle) {
		return_ACPI_STATUS(AE_BAD_PARAMETER);
	}

	/* This function does the real work */

	status = acpi_ns_delete_subtree(handle);

	return_ACPI_STATUS(status);
}
#endif
#endif
