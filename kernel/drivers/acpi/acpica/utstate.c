


#include <acpi/acpi.h>
#include "accommon.h"

#define _COMPONENT          ACPI_UTILITIES
ACPI_MODULE_NAME("utstate")

acpi_status
acpi_ut_create_pkg_state_and_push(void *internal_object,
				  void *external_object,
				  u16 index,
				  union acpi_generic_state **state_list)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_ENTRY();

	state =
	    acpi_ut_create_pkg_state(internal_object, external_object, index);
	if (!state) {
		return (AE_NO_MEMORY);
	}

	acpi_ut_push_generic_state(state_list, state);
	return (AE_OK);
}


void
acpi_ut_push_generic_state(union acpi_generic_state **list_head,
			   union acpi_generic_state *state)
{
	ACPI_FUNCTION_TRACE(ut_push_generic_state);

	/* Push the state object onto the front of the list (stack) */

	state->common.next = *list_head;
	*list_head = state;

	return_VOID;
}


union acpi_generic_state *acpi_ut_pop_generic_state(union acpi_generic_state
						    **list_head)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_TRACE(ut_pop_generic_state);

	/* Remove the state object at the head of the list (stack) */

	state = *list_head;
	if (state) {

		/* Update the list head */

		*list_head = state->common.next;
	}

	return_PTR(state);
}


union acpi_generic_state *acpi_ut_create_generic_state(void)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_ENTRY();

	state = acpi_os_acquire_object(acpi_gbl_state_cache);
	if (state) {

		/* Initialize */
		memset(state, 0, sizeof(union acpi_generic_state));
		state->common.descriptor_type = ACPI_DESC_TYPE_STATE;
	}

	return (state);
}


struct acpi_thread_state *acpi_ut_create_thread_state(void)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_TRACE(ut_create_thread_state);

	/* Create the generic state object */

	state = acpi_ut_create_generic_state();
	if (!state) {
		return_PTR(NULL);
	}

	/* Init fields specific to the update struct */

	state->common.descriptor_type = ACPI_DESC_TYPE_STATE_THREAD;
	state->thread.thread_id = acpi_os_get_thread_id();

	/* Check for invalid thread ID - zero is very bad, it will break things */

	if (!state->thread.thread_id) {
		ACPI_ERROR((AE_INFO, "Invalid zero ID from AcpiOsGetThreadId"));
		state->thread.thread_id = (acpi_thread_id) 1;
	}

	return_PTR((struct acpi_thread_state *)state);
}


union acpi_generic_state *acpi_ut_create_update_state(union acpi_operand_object
						      *object, u16 action)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_TRACE_PTR(ut_create_update_state, object);

	/* Create the generic state object */

	state = acpi_ut_create_generic_state();
	if (!state) {
		return_PTR(NULL);
	}

	/* Init fields specific to the update struct */

	state->common.descriptor_type = ACPI_DESC_TYPE_STATE_UPDATE;
	state->update.object = object;
	state->update.value = action;

	return_PTR(state);
}


union acpi_generic_state *acpi_ut_create_pkg_state(void *internal_object,
						   void *external_object,
						   u16 index)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_TRACE_PTR(ut_create_pkg_state, internal_object);

	/* Create the generic state object */

	state = acpi_ut_create_generic_state();
	if (!state) {
		return_PTR(NULL);
	}

	/* Init fields specific to the update struct */

	state->common.descriptor_type = ACPI_DESC_TYPE_STATE_PACKAGE;
	state->pkg.source_object = (union acpi_operand_object *)internal_object;
	state->pkg.dest_object = external_object;
	state->pkg.index = index;
	state->pkg.num_packages = 1;

	return_PTR(state);
}


union acpi_generic_state *acpi_ut_create_control_state(void)
{
	union acpi_generic_state *state;

	ACPI_FUNCTION_TRACE(ut_create_control_state);

	/* Create the generic state object */

	state = acpi_ut_create_generic_state();
	if (!state) {
		return_PTR(NULL);
	}

	/* Init fields specific to the control struct */

	state->common.descriptor_type = ACPI_DESC_TYPE_STATE_CONTROL;
	state->common.state = ACPI_CONTROL_CONDITIONAL_EXECUTING;

	return_PTR(state);
}


void acpi_ut_delete_generic_state(union acpi_generic_state *state)
{
	ACPI_FUNCTION_TRACE(ut_delete_generic_state);

	/* Ignore null state */

	if (state) {
		(void)acpi_os_release_object(acpi_gbl_state_cache, state);
	}
	return_VOID;
}
