


#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"

#define _COMPONENT          ACPI_NAMESPACE
ACPI_MODULE_NAME("nsrepair2")

typedef
acpi_status(*acpi_repair_function) (struct acpi_predefined_data *data,
				    union acpi_operand_object **return_object_ptr);

typedef struct acpi_repair_info {
	char name[ACPI_NAME_SIZE];
	acpi_repair_function repair_function;

} acpi_repair_info;

/* Local prototypes */

static const struct acpi_repair_info *acpi_ns_match_repairable_name(struct
								    acpi_namespace_node
								    *node);

static acpi_status
acpi_ns_repair_ALR(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr);

static acpi_status
acpi_ns_repair_FDE(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr);

static acpi_status
acpi_ns_repair_PSS(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr);

static acpi_status
acpi_ns_repair_TSS(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr);

static acpi_status
acpi_ns_check_sorted_list(struct acpi_predefined_data *data,
			  union acpi_operand_object *return_object,
			  u32 expected_count,
			  u32 sort_index,
			  u8 sort_direction, char *sort_key_name);

static void
acpi_ns_sort_list(union acpi_operand_object **elements,
		  u32 count, u32 index, u8 sort_direction);

/* Values for sort_direction above */

#define ACPI_SORT_ASCENDING     0
#define ACPI_SORT_DESCENDING    1

static const struct acpi_repair_info acpi_ns_repairable_names[] = {
	{"_ALR", acpi_ns_repair_ALR},
	{"_FDE", acpi_ns_repair_FDE},
	{"_GTM", acpi_ns_repair_FDE},	/* _GTM has same repair as _FDE */
	{"_PSS", acpi_ns_repair_PSS},
	{"_TSS", acpi_ns_repair_TSS},
	{{0, 0, 0, 0}, NULL}	/* Table terminator */
};

#define ACPI_FDE_FIELD_COUNT        5
#define ACPI_FDE_BYTE_BUFFER_SIZE   5
#define ACPI_FDE_DWORD_BUFFER_SIZE  (ACPI_FDE_FIELD_COUNT * sizeof (u32))


acpi_status
acpi_ns_complex_repairs(struct acpi_predefined_data *data,
			struct acpi_namespace_node *node,
			acpi_status validate_status,
			union acpi_operand_object **return_object_ptr)
{
	const struct acpi_repair_info *predefined;
	acpi_status status;

	/* Check if this name is in the list of repairable names */

	predefined = acpi_ns_match_repairable_name(node);
	if (!predefined) {
		return (validate_status);
	}

	status = predefined->repair_function(data, return_object_ptr);
	return (status);
}


static const struct acpi_repair_info *acpi_ns_match_repairable_name(struct
								    acpi_namespace_node
								    *node)
{
	const struct acpi_repair_info *this_name;

	/* Search info table for a repairable predefined method/object name */

	this_name = acpi_ns_repairable_names;
	while (this_name->repair_function) {
		if (ACPI_COMPARE_NAME(node->name.ascii, this_name->name)) {
			return (this_name);
		}
		this_name++;
	}

	return (NULL);		/* Not found */
}


static acpi_status
acpi_ns_repair_ALR(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr)
{
	union acpi_operand_object *return_object = *return_object_ptr;
	acpi_status status;

	status = acpi_ns_check_sorted_list(data, return_object, 2, 1,
					   ACPI_SORT_ASCENDING,
					   "AmbientIlluminance");

	return (status);
}


static acpi_status
acpi_ns_repair_FDE(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr)
{
	union acpi_operand_object *return_object = *return_object_ptr;
	union acpi_operand_object *buffer_object;
	u8 *byte_buffer;
	u32 *dword_buffer;
	u32 i;

	ACPI_FUNCTION_NAME(ns_repair_FDE);

	switch (return_object->common.type) {
	case ACPI_TYPE_BUFFER:

		/* This is the expected type. Length should be (at least) 5 DWORDs */

		if (return_object->buffer.length >= ACPI_FDE_DWORD_BUFFER_SIZE) {
			return (AE_OK);
		}

		/* We can only repair if we have exactly 5 BYTEs */

		if (return_object->buffer.length != ACPI_FDE_BYTE_BUFFER_SIZE) {
			ACPI_WARN_PREDEFINED((AE_INFO, data->pathname,
					      data->node_flags,
					      "Incorrect return buffer length %u, expected %u",
					      return_object->buffer.length,
					      ACPI_FDE_DWORD_BUFFER_SIZE));

			return (AE_AML_OPERAND_TYPE);
		}

		/* Create the new (larger) buffer object */

		buffer_object =
		    acpi_ut_create_buffer_object(ACPI_FDE_DWORD_BUFFER_SIZE);
		if (!buffer_object) {
			return (AE_NO_MEMORY);
		}

		/* Expand each byte to a DWORD */

		byte_buffer = return_object->buffer.pointer;
		dword_buffer =
		    ACPI_CAST_PTR(u32, buffer_object->buffer.pointer);

		for (i = 0; i < ACPI_FDE_FIELD_COUNT; i++) {
			*dword_buffer = (u32) *byte_buffer;
			dword_buffer++;
			byte_buffer++;
		}

		ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
				  "%s Expanded Byte Buffer to expected DWord Buffer\n",
				  data->pathname));
		break;

	default:
		return (AE_AML_OPERAND_TYPE);
	}

	/* Delete the original return object, return the new buffer object */

	acpi_ut_remove_reference(return_object);
	*return_object_ptr = buffer_object;

	data->flags |= ACPI_OBJECT_REPAIRED;
	return (AE_OK);
}


static acpi_status
acpi_ns_repair_TSS(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr)
{
	union acpi_operand_object *return_object = *return_object_ptr;
	acpi_status status;

	status = acpi_ns_check_sorted_list(data, return_object, 5, 1,
					   ACPI_SORT_DESCENDING,
					   "PowerDissipation");

	return (status);
}


static acpi_status
acpi_ns_repair_PSS(struct acpi_predefined_data *data,
		   union acpi_operand_object **return_object_ptr)
{
	union acpi_operand_object *return_object = *return_object_ptr;
	union acpi_operand_object **outer_elements;
	u32 outer_element_count;
	union acpi_operand_object **elements;
	union acpi_operand_object *obj_desc;
	u32 previous_value;
	acpi_status status;
	u32 i;

	/*
	 * Entries (sub-packages) in the _PSS Package must be sorted by power
	 * dissipation, in descending order. If it appears that the list is
	 * incorrectly sorted, sort it. We sort by cpu_frequency, since this
	 * should be proportional to the power.
	 */
	status = acpi_ns_check_sorted_list(data, return_object, 6, 0,
					   ACPI_SORT_DESCENDING,
					   "CpuFrequency");
	if (ACPI_FAILURE(status)) {
		return (status);
	}

	/*
	 * We now know the list is correctly sorted by CPU frequency. Check if
	 * the power dissipation values are proportional.
	 */
	previous_value = ACPI_UINT32_MAX;
	outer_elements = return_object->package.elements;
	outer_element_count = return_object->package.count;

	for (i = 0; i < outer_element_count; i++) {
		elements = (*outer_elements)->package.elements;
		obj_desc = elements[1];	/* Index1 = power_dissipation */

		if ((u32) obj_desc->integer.value > previous_value) {
			ACPI_WARN_PREDEFINED((AE_INFO, data->pathname,
					      data->node_flags,
					      "SubPackage[%u,%u] - suspicious power dissipation values",
					      i - 1, i));
		}

		previous_value = (u32) obj_desc->integer.value;
		outer_elements++;
	}

	return (AE_OK);
}


static acpi_status
acpi_ns_check_sorted_list(struct acpi_predefined_data *data,
			  union acpi_operand_object *return_object,
			  u32 expected_count,
			  u32 sort_index,
			  u8 sort_direction, char *sort_key_name)
{
	u32 outer_element_count;
	union acpi_operand_object **outer_elements;
	union acpi_operand_object **elements;
	union acpi_operand_object *obj_desc;
	u32 i;
	u32 previous_value;

	ACPI_FUNCTION_NAME(ns_check_sorted_list);

	/* The top-level object must be a package */

	if (return_object->common.type != ACPI_TYPE_PACKAGE) {
		return (AE_AML_OPERAND_TYPE);
	}

	/*
	 * NOTE: assumes list of sub-packages contains no NULL elements.
	 * Any NULL elements should have been removed by earlier call
	 * to acpi_ns_remove_null_elements.
	 */
	outer_elements = return_object->package.elements;
	outer_element_count = return_object->package.count;
	if (!outer_element_count) {
		return (AE_AML_PACKAGE_LIMIT);
	}

	previous_value = 0;
	if (sort_direction == ACPI_SORT_DESCENDING) {
		previous_value = ACPI_UINT32_MAX;
	}

	/* Examine each subpackage */

	for (i = 0; i < outer_element_count; i++) {

		/* Each element of the top-level package must also be a package */

		if ((*outer_elements)->common.type != ACPI_TYPE_PACKAGE) {
			return (AE_AML_OPERAND_TYPE);
		}

		/* Each sub-package must have the minimum length */

		if ((*outer_elements)->package.count < expected_count) {
			return (AE_AML_PACKAGE_LIMIT);
		}

		elements = (*outer_elements)->package.elements;
		obj_desc = elements[sort_index];

		if (obj_desc->common.type != ACPI_TYPE_INTEGER) {
			return (AE_AML_OPERAND_TYPE);
		}

		/*
		 * The list must be sorted in the specified order. If we detect a
		 * discrepancy, sort the entire list.
		 */
		if (((sort_direction == ACPI_SORT_ASCENDING) &&
		     (obj_desc->integer.value < previous_value)) ||
		    ((sort_direction == ACPI_SORT_DESCENDING) &&
		     (obj_desc->integer.value > previous_value))) {
			acpi_ns_sort_list(return_object->package.elements,
					  outer_element_count, sort_index,
					  sort_direction);

			data->flags |= ACPI_OBJECT_REPAIRED;

			ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
					  "%s: Repaired unsorted list - now sorted by %s\n",
					  data->pathname, sort_key_name));
			return (AE_OK);
		}

		previous_value = (u32) obj_desc->integer.value;
		outer_elements++;
	}

	return (AE_OK);
}


static void
acpi_ns_sort_list(union acpi_operand_object **elements,
		  u32 count, u32 index, u8 sort_direction)
{
	union acpi_operand_object *obj_desc1;
	union acpi_operand_object *obj_desc2;
	union acpi_operand_object *temp_obj;
	u32 i;
	u32 j;

	/* Simple bubble sort */

	for (i = 1; i < count; i++) {
		for (j = (count - 1); j >= i; j--) {
			obj_desc1 = elements[j - 1]->package.elements[index];
			obj_desc2 = elements[j]->package.elements[index];

			if (((sort_direction == ACPI_SORT_ASCENDING) &&
			     (obj_desc1->integer.value >
			      obj_desc2->integer.value))
			    || ((sort_direction == ACPI_SORT_DESCENDING)
				&& (obj_desc1->integer.value <
				    obj_desc2->integer.value))) {
				temp_obj = elements[j - 1];
				elements[j - 1] = elements[j];
				elements[j] = temp_obj;
			}
		}
	}
}
