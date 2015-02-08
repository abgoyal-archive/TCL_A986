


#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"
#include "acinterp.h"

#define _COMPONENT          ACPI_NAMESPACE
ACPI_MODULE_NAME("nsxfeval")

/* Local prototypes */
static void acpi_ns_resolve_references(struct acpi_evaluate_info *info);


acpi_status
acpi_evaluate_object_typed(acpi_handle handle,
			   acpi_string pathname,
			   struct acpi_object_list *external_params,
			   struct acpi_buffer *return_buffer,
			   acpi_object_type return_type)
{
	acpi_status status;
	u8 must_free = FALSE;

	ACPI_FUNCTION_TRACE(acpi_evaluate_object_typed);

	/* Return buffer must be valid */

	if (!return_buffer) {
		return_ACPI_STATUS(AE_BAD_PARAMETER);
	}

	if (return_buffer->length == ACPI_ALLOCATE_BUFFER) {
		must_free = TRUE;
	}

	/* Evaluate the object */

	status =
	    acpi_evaluate_object(handle, pathname, external_params,
				 return_buffer);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	/* Type ANY means "don't care" */

	if (return_type == ACPI_TYPE_ANY) {
		return_ACPI_STATUS(AE_OK);
	}

	if (return_buffer->length == 0) {

		/* Error because caller specifically asked for a return value */

		ACPI_ERROR((AE_INFO, "No return value"));
		return_ACPI_STATUS(AE_NULL_OBJECT);
	}

	/* Examine the object type returned from evaluate_object */

	if (((union acpi_object *)return_buffer->pointer)->type == return_type) {
		return_ACPI_STATUS(AE_OK);
	}

	/* Return object type does not match requested type */

	ACPI_ERROR((AE_INFO,
		    "Incorrect return type [%s] requested [%s]",
		    acpi_ut_get_type_name(((union acpi_object *)return_buffer->
					   pointer)->type),
		    acpi_ut_get_type_name(return_type)));

	if (must_free) {

		/* Caller used ACPI_ALLOCATE_BUFFER, free the return buffer */

		ACPI_FREE(return_buffer->pointer);
		return_buffer->pointer = NULL;
	}

	return_buffer->length = 0;
	return_ACPI_STATUS(AE_TYPE);
}

ACPI_EXPORT_SYMBOL(acpi_evaluate_object_typed)

acpi_status
acpi_evaluate_object(acpi_handle handle,
		     acpi_string pathname,
		     struct acpi_object_list *external_params,
		     struct acpi_buffer *return_buffer)
{
	acpi_status status;
	struct acpi_evaluate_info *info;
	acpi_size buffer_space_needed;
	u32 i;

	ACPI_FUNCTION_TRACE(acpi_evaluate_object);

	/* Allocate and initialize the evaluation information block */

	info = ACPI_ALLOCATE_ZEROED(sizeof(struct acpi_evaluate_info));
	if (!info) {
		return_ACPI_STATUS(AE_NO_MEMORY);
	}

	info->pathname = pathname;

	/* Convert and validate the device handle */

	info->prefix_node = acpi_ns_validate_handle(handle);
	if (!info->prefix_node) {
		status = AE_BAD_PARAMETER;
		goto cleanup;
	}

	/*
	 * If there are parameters to be passed to a control method, the external
	 * objects must all be converted to internal objects
	 */
	if (external_params && external_params->count) {
		/*
		 * Allocate a new parameter block for the internal objects
		 * Add 1 to count to allow for null terminated internal list
		 */
		info->parameters = ACPI_ALLOCATE_ZEROED(((acpi_size)
							 external_params->
							 count +
							 1) * sizeof(void *));
		if (!info->parameters) {
			status = AE_NO_MEMORY;
			goto cleanup;
		}

		/* Convert each external object in the list to an internal object */

		for (i = 0; i < external_params->count; i++) {
			status =
			    acpi_ut_copy_eobject_to_iobject(&external_params->
							    pointer[i],
							    &info->
							    parameters[i]);
			if (ACPI_FAILURE(status)) {
				goto cleanup;
			}
		}
		info->parameters[external_params->count] = NULL;
	}

	/*
	 * Three major cases:
	 * 1) Fully qualified pathname
	 * 2) No handle, not fully qualified pathname (error)
	 * 3) Valid handle
	 */
	if ((pathname) && (acpi_ns_valid_root_prefix(pathname[0]))) {

		/* The path is fully qualified, just evaluate by name */

		info->prefix_node = NULL;
		status = acpi_ns_evaluate(info);
	} else if (!handle) {
		/*
		 * A handle is optional iff a fully qualified pathname is specified.
		 * Since we've already handled fully qualified names above, this is
		 * an error
		 */
		if (!pathname) {
			ACPI_DEBUG_PRINT((ACPI_DB_INFO,
					  "Both Handle and Pathname are NULL"));
		} else {
			ACPI_DEBUG_PRINT((ACPI_DB_INFO,
					  "Null Handle with relative pathname [%s]",
					  pathname));
		}

		status = AE_BAD_PARAMETER;
	} else {
		/* We have a namespace a node and a possible relative path */

		status = acpi_ns_evaluate(info);
	}

	/*
	 * If we are expecting a return value, and all went well above,
	 * copy the return value to an external object.
	 */
	if (return_buffer) {
		if (!info->return_object) {
			return_buffer->length = 0;
		} else {
			if (ACPI_GET_DESCRIPTOR_TYPE(info->return_object) ==
			    ACPI_DESC_TYPE_NAMED) {
				/*
				 * If we received a NS Node as a return object, this means that
				 * the object we are evaluating has nothing interesting to
				 * return (such as a mutex, etc.)  We return an error because
				 * these types are essentially unsupported by this interface.
				 * We don't check up front because this makes it easier to add
				 * support for various types at a later date if necessary.
				 */
				status = AE_TYPE;
				info->return_object = NULL;	/* No need to delete a NS Node */
				return_buffer->length = 0;
			}

			if (ACPI_SUCCESS(status)) {

				/* Dereference Index and ref_of references */

				acpi_ns_resolve_references(info);

				/* Get the size of the returned object */

				status =
				    acpi_ut_get_object_size(info->return_object,
							    &buffer_space_needed);
				if (ACPI_SUCCESS(status)) {

					/* Validate/Allocate/Clear caller buffer */

					status =
					    acpi_ut_initialize_buffer
					    (return_buffer,
					     buffer_space_needed);
					if (ACPI_FAILURE(status)) {
						/*
						 * Caller's buffer is too small or a new one can't
						 * be allocated
						 */
						ACPI_DEBUG_PRINT((ACPI_DB_INFO,
								  "Needed buffer size %X, %s\n",
								  (u32)
								  buffer_space_needed,
								  acpi_format_exception
								  (status)));
					} else {
						/* We have enough space for the object, build it */

						status =
						    acpi_ut_copy_iobject_to_eobject
						    (info->return_object,
						     return_buffer);
					}
				}
			}
		}
	}

	if (info->return_object) {
		/*
		 * Delete the internal return object. NOTE: Interpreter must be
		 * locked to avoid race condition.
		 */
		acpi_ex_enter_interpreter();

		/* Remove one reference on the return object (should delete it) */

		acpi_ut_remove_reference(info->return_object);
		acpi_ex_exit_interpreter();
	}

      cleanup:

	/* Free the input parameter list (if we created one) */

	if (info->parameters) {

		/* Free the allocated parameter block */

		acpi_ut_delete_internal_object_list(info->parameters);
	}

	ACPI_FREE(info);
	return_ACPI_STATUS(status);
}

ACPI_EXPORT_SYMBOL(acpi_evaluate_object)

static void acpi_ns_resolve_references(struct acpi_evaluate_info *info)
{
	union acpi_operand_object *obj_desc = NULL;
	struct acpi_namespace_node *node;

	/* We are interested in reference objects only */

	if ((info->return_object)->common.type != ACPI_TYPE_LOCAL_REFERENCE) {
		return;
	}

	/*
	 * Two types of references are supported - those created by Index and
	 * ref_of operators. A name reference (AML_NAMEPATH_OP) can be converted
	 * to an union acpi_object, so it is not dereferenced here. A ddb_handle
	 * (AML_LOAD_OP) cannot be dereferenced, nor can it be converted to
	 * an union acpi_object.
	 */
	switch (info->return_object->reference.class) {
	case ACPI_REFCLASS_INDEX:

		obj_desc = *(info->return_object->reference.where);
		break;

	case ACPI_REFCLASS_REFOF:

		node = info->return_object->reference.object;
		if (node) {
			obj_desc = node->object;
		}
		break;

	default:
		return;
	}

	/* Replace the existing reference object */

	if (obj_desc) {
		acpi_ut_add_reference(obj_desc);
		acpi_ut_remove_reference(info->return_object);
		info->return_object = obj_desc;
	}

	return;
}


acpi_status
acpi_walk_namespace(acpi_object_type type,
		    acpi_handle start_object,
		    u32 max_depth,
		    acpi_walk_callback pre_order_visit,
		    acpi_walk_callback post_order_visit,
		    void *context, void **return_value)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(acpi_walk_namespace);

	/* Parameter validation */

	if ((type > ACPI_TYPE_LOCAL_MAX) ||
	    (!max_depth) || (!pre_order_visit && !post_order_visit)) {
		return_ACPI_STATUS(AE_BAD_PARAMETER);
	}

	/*
	 * Need to acquire the namespace reader lock to prevent interference
	 * with any concurrent table unloads (which causes the deletion of
	 * namespace objects). We cannot allow the deletion of a namespace node
	 * while the user function is using it. The exception to this are the
	 * nodes created and deleted during control method execution -- these
	 * nodes are marked as temporary nodes and are ignored by the namespace
	 * walk. Thus, control methods can be executed while holding the
	 * namespace deletion lock (and the user function can execute control
	 * methods.)
	 */
	status = acpi_ut_acquire_read_lock(&acpi_gbl_namespace_rw_lock);
	if (ACPI_FAILURE(status)) {
		return status;
	}

	/*
	 * Lock the namespace around the walk. The namespace will be
	 * unlocked/locked around each call to the user function - since the user
	 * function must be allowed to make ACPICA calls itself (for example, it
	 * will typically execute control methods during device enumeration.)
	 */
	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		goto unlock_and_exit;
	}

	status = acpi_ns_walk_namespace(type, start_object, max_depth,
					ACPI_NS_WALK_UNLOCK, pre_order_visit,
					post_order_visit, context,
					return_value);

	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);

      unlock_and_exit:
	(void)acpi_ut_release_read_lock(&acpi_gbl_namespace_rw_lock);
	return_ACPI_STATUS(status);
}

ACPI_EXPORT_SYMBOL(acpi_walk_namespace)

static acpi_status
acpi_ns_get_device_callback(acpi_handle obj_handle,
			    u32 nesting_level,
			    void *context, void **return_value)
{
	struct acpi_get_devices_info *info = context;
	acpi_status status;
	struct acpi_namespace_node *node;
	u32 flags;
	struct acpica_device_id *hid;
	struct acpica_device_id_list *cid;
	u32 i;
	u8 found;
	int no_match;

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	node = acpi_ns_validate_handle(obj_handle);
	status = acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	if (!node) {
		return (AE_BAD_PARAMETER);
	}

	/*
	 * First, filter based on the device HID and CID.
	 *
	 * 01/2010: For this case where a specific HID is requested, we don't
	 * want to run _STA until we have an actual HID match. Thus, we will
	 * not unnecessarily execute _STA on devices for which the caller
	 * doesn't care about. Previously, _STA was executed unconditionally
	 * on all devices found here.
	 *
	 * A side-effect of this change is that now we will continue to search
	 * for a matching HID even under device trees where the parent device
	 * would have returned a _STA that indicates it is not present or
	 * not functioning (thus aborting the search on that branch).
	 */
	if (info->hid != NULL) {
		status = acpi_ut_execute_HID(node, &hid);
		if (status == AE_NOT_FOUND) {
			return (AE_OK);
		} else if (ACPI_FAILURE(status)) {
			return (AE_CTRL_DEPTH);
		}

		no_match = ACPI_STRCMP(hid->string, info->hid);
		ACPI_FREE(hid);

		if (no_match) {
			/*
			 * HID does not match, attempt match within the
			 * list of Compatible IDs (CIDs)
			 */
			status = acpi_ut_execute_CID(node, &cid);
			if (status == AE_NOT_FOUND) {
				return (AE_OK);
			} else if (ACPI_FAILURE(status)) {
				return (AE_CTRL_DEPTH);
			}

			/* Walk the CID list */

			found = 0;
			for (i = 0; i < cid->count; i++) {
				if (ACPI_STRCMP(cid->ids[i].string, info->hid)
				    == 0) {
					found = 1;
					break;
				}
			}
			ACPI_FREE(cid);
			if (!found)
				return (AE_OK);
		}
	}

	/* Run _STA to determine if device is present */

	status = acpi_ut_execute_STA(node, &flags);
	if (ACPI_FAILURE(status)) {
		return (AE_CTRL_DEPTH);
	}

	if (!(flags & ACPI_STA_DEVICE_PRESENT) &&
	    !(flags & ACPI_STA_DEVICE_FUNCTIONING)) {
		/*
		 * Don't examine the children of the device only when the
		 * device is neither present nor functional. See ACPI spec,
		 * description of _STA for more information.
		 */
		return (AE_CTRL_DEPTH);
	}

	/* We have a valid device, invoke the user function */

	status = info->user_function(obj_handle, nesting_level, info->context,
				     return_value);
	return (status);
}


acpi_status
acpi_get_devices(const char *HID,
		 acpi_walk_callback user_function,
		 void *context, void **return_value)
{
	acpi_status status;
	struct acpi_get_devices_info info;

	ACPI_FUNCTION_TRACE(acpi_get_devices);

	/* Parameter validation */

	if (!user_function) {
		return_ACPI_STATUS(AE_BAD_PARAMETER);
	}

	/*
	 * We're going to call their callback from OUR callback, so we need
	 * to know what it is, and their context parameter.
	 */
	info.hid = HID;
	info.context = context;
	info.user_function = user_function;

	/*
	 * Lock the namespace around the walk.
	 * The namespace will be unlocked/locked around each call
	 * to the user function - since this function
	 * must be allowed to make Acpi calls itself.
	 */
	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return_ACPI_STATUS(status);
	}

	status = acpi_ns_walk_namespace(ACPI_TYPE_DEVICE, ACPI_ROOT_OBJECT,
					ACPI_UINT32_MAX, ACPI_NS_WALK_UNLOCK,
					acpi_ns_get_device_callback, NULL,
					&info, return_value);

	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return_ACPI_STATUS(status);
}

ACPI_EXPORT_SYMBOL(acpi_get_devices)

acpi_status
acpi_attach_data(acpi_handle obj_handle,
		 acpi_object_handler handler, void *data)
{
	struct acpi_namespace_node *node;
	acpi_status status;

	/* Parameter validation */

	if (!obj_handle || !handler || !data) {
		return (AE_BAD_PARAMETER);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Convert and validate the handle */

	node = acpi_ns_validate_handle(obj_handle);
	if (!node) {
		status = AE_BAD_PARAMETER;
		goto unlock_and_exit;
	}

	status = acpi_ns_attach_data(node, handler, data);

      unlock_and_exit:
	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_attach_data)

acpi_status
acpi_detach_data(acpi_handle obj_handle, acpi_object_handler handler)
{
	struct acpi_namespace_node *node;
	acpi_status status;

	/* Parameter validation */

	if (!obj_handle || !handler) {
		return (AE_BAD_PARAMETER);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Convert and validate the handle */

	node = acpi_ns_validate_handle(obj_handle);
	if (!node) {
		status = AE_BAD_PARAMETER;
		goto unlock_and_exit;
	}

	status = acpi_ns_detach_data(node, handler);

      unlock_and_exit:
	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_detach_data)

acpi_status
acpi_get_data(acpi_handle obj_handle, acpi_object_handler handler, void **data)
{
	struct acpi_namespace_node *node;
	acpi_status status;

	/* Parameter validation */

	if (!obj_handle || !handler || !data) {
		return (AE_BAD_PARAMETER);
	}

	status = acpi_ut_acquire_mutex(ACPI_MTX_NAMESPACE);
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/* Convert and validate the handle */

	node = acpi_ns_validate_handle(obj_handle);
	if (!node) {
		status = AE_BAD_PARAMETER;
		goto unlock_and_exit;
	}

	status = acpi_ns_get_attached_data(node, handler, data);

      unlock_and_exit:
	(void)acpi_ut_release_mutex(ACPI_MTX_NAMESPACE);
	return (status);
}

ACPI_EXPORT_SYMBOL(acpi_get_data)
