


#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"
#include "acevents.h"
#include "actables.h"

#define _COMPONENT          ACPI_UTILITIES
ACPI_MODULE_NAME("utinit")

/* Local prototypes */
static void acpi_ut_terminate(void);


static void acpi_ut_terminate(void)
{
	struct acpi_gpe_block_info *gpe_block;
	struct acpi_gpe_block_info *next_gpe_block;
	struct acpi_gpe_xrupt_info *gpe_xrupt_info;
	struct acpi_gpe_xrupt_info *next_gpe_xrupt_info;

	ACPI_FUNCTION_TRACE(ut_terminate);

	/* Free global GPE blocks and related info structures */

	gpe_xrupt_info = acpi_gbl_gpe_xrupt_list_head;
	while (gpe_xrupt_info) {
		gpe_block = gpe_xrupt_info->gpe_block_list_head;
		while (gpe_block) {
			next_gpe_block = gpe_block->next;
			ACPI_FREE(gpe_block->event_info);
			ACPI_FREE(gpe_block->register_info);
			ACPI_FREE(gpe_block);

			gpe_block = next_gpe_block;
		}
		next_gpe_xrupt_info = gpe_xrupt_info->next;
		ACPI_FREE(gpe_xrupt_info);
		gpe_xrupt_info = next_gpe_xrupt_info;
	}

	return_VOID;
}


void acpi_ut_subsystem_shutdown(void)
{
	ACPI_FUNCTION_TRACE(ut_subsystem_shutdown);

#ifndef ACPI_ASL_COMPILER

	/* Close the acpi_event Handling */

	acpi_ev_terminate();
#endif

	/* Close the Namespace */

	acpi_ns_terminate();

	/* Delete the ACPI tables */

	acpi_tb_terminate();

	/* Close the globals */

	acpi_ut_terminate();

	/* Purge the local caches */

	(void)acpi_ut_delete_caches();
	return_VOID;
}
