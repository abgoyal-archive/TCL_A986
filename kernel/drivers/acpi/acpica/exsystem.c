



#include <acpi/acpi.h>
#include "accommon.h"
#include "acinterp.h"

#define _COMPONENT          ACPI_EXECUTER
ACPI_MODULE_NAME("exsystem")

acpi_status acpi_ex_system_wait_semaphore(acpi_semaphore semaphore, u16 timeout)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ex_system_wait_semaphore);

	status = acpi_os_wait_semaphore(semaphore, 1, ACPI_DO_NOT_WAIT);
	if (ACPI_SUCCESS(status)) {
		return_ACPI_STATUS(status);
	}

	if (status == AE_TIME) {

		/* We must wait, so unlock the interpreter */

		acpi_ex_relinquish_interpreter();

		status = acpi_os_wait_semaphore(semaphore, 1, timeout);

		ACPI_DEBUG_PRINT((ACPI_DB_EXEC,
				  "*** Thread awake after blocking, %s\n",
				  acpi_format_exception(status)));

		/* Reacquire the interpreter */

		acpi_ex_reacquire_interpreter();
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_ex_system_wait_mutex(acpi_mutex mutex, u16 timeout)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ex_system_wait_mutex);

	status = acpi_os_acquire_mutex(mutex, ACPI_DO_NOT_WAIT);
	if (ACPI_SUCCESS(status)) {
		return_ACPI_STATUS(status);
	}

	if (status == AE_TIME) {

		/* We must wait, so unlock the interpreter */

		acpi_ex_relinquish_interpreter();

		status = acpi_os_acquire_mutex(mutex, timeout);

		ACPI_DEBUG_PRINT((ACPI_DB_EXEC,
				  "*** Thread awake after blocking, %s\n",
				  acpi_format_exception(status)));

		/* Reacquire the interpreter */

		acpi_ex_reacquire_interpreter();
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_ex_system_do_stall(u32 how_long)
{
	acpi_status status = AE_OK;

	ACPI_FUNCTION_ENTRY();

	if (how_long > 255) {	/* 255 microseconds */
		/*
		 * Longer than 255 usec, this is an error
		 *
		 * (ACPI specifies 100 usec as max, but this gives some slack in
		 * order to support existing BIOSs)
		 */
		ACPI_ERROR((AE_INFO, "Time parameter is too large (%u)",
			    how_long));
		status = AE_AML_OPERAND_VALUE;
	} else {
		acpi_os_stall(how_long);
	}

	return (status);
}


acpi_status acpi_ex_system_do_sleep(u64 how_long)
{
	ACPI_FUNCTION_ENTRY();

	/* Since this thread will sleep, we must release the interpreter */

	acpi_ex_relinquish_interpreter();

	/*
	 * For compatibility with other ACPI implementations and to prevent
	 * accidental deep sleeps, limit the sleep time to something reasonable.
	 */
	if (how_long > ACPI_MAX_SLEEP) {
		how_long = ACPI_MAX_SLEEP;
	}

	acpi_os_sleep(how_long);

	/* And now we must get the interpreter again */

	acpi_ex_reacquire_interpreter();
	return (AE_OK);
}


acpi_status acpi_ex_system_signal_event(union acpi_operand_object * obj_desc)
{
	acpi_status status = AE_OK;

	ACPI_FUNCTION_TRACE(ex_system_signal_event);

	if (obj_desc) {
		status =
		    acpi_os_signal_semaphore(obj_desc->event.os_semaphore, 1);
	}

	return_ACPI_STATUS(status);
}


acpi_status
acpi_ex_system_wait_event(union acpi_operand_object *time_desc,
			  union acpi_operand_object *obj_desc)
{
	acpi_status status = AE_OK;

	ACPI_FUNCTION_TRACE(ex_system_wait_event);

	if (obj_desc) {
		status =
		    acpi_ex_system_wait_semaphore(obj_desc->event.os_semaphore,
						  (u16) time_desc->integer.
						  value);
	}

	return_ACPI_STATUS(status);
}


acpi_status acpi_ex_system_reset_event(union acpi_operand_object *obj_desc)
{
	acpi_status status = AE_OK;
	acpi_semaphore temp_semaphore;

	ACPI_FUNCTION_ENTRY();

	/*
	 * We are going to simply delete the existing semaphore and
	 * create a new one!
	 */
	status =
	    acpi_os_create_semaphore(ACPI_NO_UNIT_LIMIT, 0, &temp_semaphore);
	if (ACPI_SUCCESS(status)) {
		(void)acpi_os_delete_semaphore(obj_desc->event.os_semaphore);
		obj_desc->event.os_semaphore = temp_semaphore;
	}

	return (status);
}
