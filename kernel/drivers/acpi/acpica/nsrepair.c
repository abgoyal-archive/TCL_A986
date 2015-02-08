


#include <acpi/acpi.h>
#include "accommon.h"
#include "acnamesp.h"
#include "acinterp.h"
#include "acpredef.h"

#define _COMPONENT          ACPI_NAMESPACE
ACPI_MODULE_NAME("nsrepair")

/* Local prototypes */
static acpi_status
acpi_ns_convert_to_integer(union acpi_operand_object *original_object,
			   union acpi_operand_object **return_object);

static acpi_status
acpi_ns_convert_to_string(union acpi_operand_object *original_object,
			  union acpi_operand_object **return_object);

static acpi_status
acpi_ns_convert_to_buffer(union acpi_operand_object *original_object,
			  union acpi_operand_object **return_object);

static acpi_status
acpi_ns_convert_to_package(union acpi_operand_object *original_object,
			   union acpi_operand_object **return_object);


acpi_status
acpi_ns_repair_object(struct acpi_predefined_data *data,
		      u32 expected_btypes,
		      u32 package_index,
		      union acpi_operand_object **return_object_ptr)
{
	union acpi_operand_object *return_object = *return_object_ptr;
	union acpi_operand_object *new_object;
	acpi_status status;

	ACPI_FUNCTION_NAME(ns_repair_object);

	/*
	 * At this point, we know that the type of the returned object was not
	 * one of the expected types for this predefined name. Attempt to
	 * repair the object by converting it to one of the expected object
	 * types for this predefined name.
	 */
	if (expected_btypes & ACPI_RTYPE_INTEGER) {
		status = acpi_ns_convert_to_integer(return_object, &new_object);
		if (ACPI_SUCCESS(status)) {
			goto object_repaired;
		}
	}
	if (expected_btypes & ACPI_RTYPE_STRING) {
		status = acpi_ns_convert_to_string(return_object, &new_object);
		if (ACPI_SUCCESS(status)) {
			goto object_repaired;
		}
	}
	if (expected_btypes & ACPI_RTYPE_BUFFER) {
		status = acpi_ns_convert_to_buffer(return_object, &new_object);
		if (ACPI_SUCCESS(status)) {
			goto object_repaired;
		}
	}
	if (expected_btypes & ACPI_RTYPE_PACKAGE) {
		status = acpi_ns_convert_to_package(return_object, &new_object);
		if (ACPI_SUCCESS(status)) {
			goto object_repaired;
		}
	}

	/* We cannot repair this object */

	return (AE_AML_OPERAND_TYPE);

      object_repaired:

	/* Object was successfully repaired */

	/*
	 * If the original object is a package element, we need to:
	 * 1. Set the reference count of the new object to match the
	 *    reference count of the old object.
	 * 2. Decrement the reference count of the original object.
	 */
	if (package_index != ACPI_NOT_PACKAGE_ELEMENT) {
		new_object->common.reference_count =
		    return_object->common.reference_count;

		if (return_object->common.reference_count > 1) {
			return_object->common.reference_count--;
		}

		ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
				  "%s: Converted %s to expected %s at index %u\n",
				  data->pathname,
				  acpi_ut_get_object_type_name(return_object),
				  acpi_ut_get_object_type_name(new_object),
				  package_index));
	} else {
		ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
				  "%s: Converted %s to expected %s\n",
				  data->pathname,
				  acpi_ut_get_object_type_name(return_object),
				  acpi_ut_get_object_type_name(new_object)));
	}

	/* Delete old object, install the new return object */

	acpi_ut_remove_reference(return_object);
	*return_object_ptr = new_object;
	data->flags |= ACPI_OBJECT_REPAIRED;
	return (AE_OK);
}


static acpi_status
acpi_ns_convert_to_integer(union acpi_operand_object *original_object,
			   union acpi_operand_object **return_object)
{
	union acpi_operand_object *new_object;
	acpi_status status;
	u64 value = 0;
	u32 i;

	switch (original_object->common.type) {
	case ACPI_TYPE_STRING:

		/* String-to-Integer conversion */

		status = acpi_ut_strtoul64(original_object->string.pointer,
					   ACPI_ANY_BASE, &value);
		if (ACPI_FAILURE(status)) {
			return (status);
		}
		break;

	case ACPI_TYPE_BUFFER:

		/* Buffer-to-Integer conversion. Max buffer size is 64 bits. */

		if (original_object->buffer.length > 8) {
			return (AE_AML_OPERAND_TYPE);
		}

		/* Extract each buffer byte to create the integer */

		for (i = 0; i < original_object->buffer.length; i++) {
			value |=
			    ((u64) original_object->buffer.
			     pointer[i] << (i * 8));
		}
		break;

	default:
		return (AE_AML_OPERAND_TYPE);
	}

	new_object = acpi_ut_create_integer_object(value);
	if (!new_object) {
		return (AE_NO_MEMORY);
	}

	*return_object = new_object;
	return (AE_OK);
}


static acpi_status
acpi_ns_convert_to_string(union acpi_operand_object *original_object,
			  union acpi_operand_object **return_object)
{
	union acpi_operand_object *new_object;
	acpi_size length;
	acpi_status status;

	switch (original_object->common.type) {
	case ACPI_TYPE_INTEGER:
		/*
		 * Integer-to-String conversion. Commonly, convert
		 * an integer of value 0 to a NULL string. The last element of
		 * _BIF and _BIX packages occasionally need this fix.
		 */
		if (original_object->integer.value == 0) {

			/* Allocate a new NULL string object */

			new_object = acpi_ut_create_string_object(0);
			if (!new_object) {
				return (AE_NO_MEMORY);
			}
		} else {
			status =
			    acpi_ex_convert_to_string(original_object,
						      &new_object,
						      ACPI_IMPLICIT_CONVERT_HEX);
			if (ACPI_FAILURE(status)) {
				return (status);
			}
		}
		break;

	case ACPI_TYPE_BUFFER:
		/*
		 * Buffer-to-String conversion. Use a to_string
		 * conversion, no transform performed on the buffer data. The best
		 * example of this is the _BIF method, where the string data from
		 * the battery is often (incorrectly) returned as buffer object(s).
		 */
		length = 0;
		while ((length < original_object->buffer.length) &&
		       (original_object->buffer.pointer[length])) {
			length++;
		}

		/* Allocate a new string object */

		new_object = acpi_ut_create_string_object(length);
		if (!new_object) {
			return (AE_NO_MEMORY);
		}

		/*
		 * Copy the raw buffer data with no transform. String is already NULL
		 * terminated at Length+1.
		 */
		ACPI_MEMCPY(new_object->string.pointer,
			    original_object->buffer.pointer, length);
		break;

	default:
		return (AE_AML_OPERAND_TYPE);
	}

	*return_object = new_object;
	return (AE_OK);
}


static acpi_status
acpi_ns_convert_to_buffer(union acpi_operand_object *original_object,
			  union acpi_operand_object **return_object)
{
	union acpi_operand_object *new_object;
	acpi_status status;
	union acpi_operand_object **elements;
	u32 *dword_buffer;
	u32 count;
	u32 i;

	switch (original_object->common.type) {
	case ACPI_TYPE_INTEGER:
		/*
		 * Integer-to-Buffer conversion.
		 * Convert the Integer to a packed-byte buffer. _MAT and other
		 * objects need this sometimes, if a read has been performed on a
		 * Field object that is less than or equal to the global integer
		 * size (32 or 64 bits).
		 */
		status =
		    acpi_ex_convert_to_buffer(original_object, &new_object);
		if (ACPI_FAILURE(status)) {
			return (status);
		}
		break;

	case ACPI_TYPE_STRING:

		/* String-to-Buffer conversion. Simple data copy */

		new_object =
		    acpi_ut_create_buffer_object(original_object->string.
						 length);
		if (!new_object) {
			return (AE_NO_MEMORY);
		}

		ACPI_MEMCPY(new_object->buffer.pointer,
			    original_object->string.pointer,
			    original_object->string.length);
		break;

	case ACPI_TYPE_PACKAGE:
		/*
		 * This case is often seen for predefined names that must return a
		 * Buffer object with multiple DWORD integers within. For example,
		 * _FDE and _GTM. The Package can be converted to a Buffer.
		 */

		/* All elements of the Package must be integers */

		elements = original_object->package.elements;
		count = original_object->package.count;

		for (i = 0; i < count; i++) {
			if ((!*elements) ||
			    ((*elements)->common.type != ACPI_TYPE_INTEGER)) {
				return (AE_AML_OPERAND_TYPE);
			}
			elements++;
		}

		/* Create the new buffer object to replace the Package */

		new_object = acpi_ut_create_buffer_object(ACPI_MUL_4(count));
		if (!new_object) {
			return (AE_NO_MEMORY);
		}

		/* Copy the package elements (integers) to the buffer as DWORDs */

		elements = original_object->package.elements;
		dword_buffer = ACPI_CAST_PTR(u32, new_object->buffer.pointer);

		for (i = 0; i < count; i++) {
			*dword_buffer = (u32) (*elements)->integer.value;
			dword_buffer++;
			elements++;
		}
		break;

	default:
		return (AE_AML_OPERAND_TYPE);
	}

	*return_object = new_object;
	return (AE_OK);
}


static acpi_status
acpi_ns_convert_to_package(union acpi_operand_object *original_object,
			   union acpi_operand_object **return_object)
{
	union acpi_operand_object *new_object;
	union acpi_operand_object **elements;
	u32 length;
	u8 *buffer;

	switch (original_object->common.type) {
	case ACPI_TYPE_BUFFER:

		/* Buffer-to-Package conversion */

		length = original_object->buffer.length;
		new_object = acpi_ut_create_package_object(length);
		if (!new_object) {
			return (AE_NO_MEMORY);
		}

		/* Convert each buffer byte to an integer package element */

		elements = new_object->package.elements;
		buffer = original_object->buffer.pointer;

		while (length--) {
			*elements =
			    acpi_ut_create_integer_object((u64) *buffer);
			if (!*elements) {
				acpi_ut_remove_reference(new_object);
				return (AE_NO_MEMORY);
			}
			elements++;
			buffer++;
		}
		break;

	default:
		return (AE_AML_OPERAND_TYPE);
	}

	*return_object = new_object;
	return (AE_OK);
}


acpi_status
acpi_ns_repair_null_element(struct acpi_predefined_data *data,
			    u32 expected_btypes,
			    u32 package_index,
			    union acpi_operand_object **return_object_ptr)
{
	union acpi_operand_object *return_object = *return_object_ptr;
	union acpi_operand_object *new_object;

	ACPI_FUNCTION_NAME(ns_repair_null_element);

	/* No repair needed if return object is non-NULL */

	if (return_object) {
		return (AE_OK);
	}

	/*
	 * Attempt to repair a NULL element of a Package object. This applies to
	 * predefined names that return a fixed-length package and each element
	 * is required. It does not apply to variable-length packages where NULL
	 * elements are allowed, especially at the end of the package.
	 */
	if (expected_btypes & ACPI_RTYPE_INTEGER) {

		/* Need an Integer - create a zero-value integer */

		new_object = acpi_ut_create_integer_object(0);
	} else if (expected_btypes & ACPI_RTYPE_STRING) {

		/* Need a String - create a NULL string */

		new_object = acpi_ut_create_string_object(0);
	} else if (expected_btypes & ACPI_RTYPE_BUFFER) {

		/* Need a Buffer - create a zero-length buffer */

		new_object = acpi_ut_create_buffer_object(0);
	} else {
		/* Error for all other expected types */

		return (AE_AML_OPERAND_TYPE);
	}

	if (!new_object) {
		return (AE_NO_MEMORY);
	}

	/* Set the reference count according to the parent Package object */

	new_object->common.reference_count =
	    data->parent_package->common.reference_count;

	ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
			  "%s: Converted NULL package element to expected %s at index %u\n",
			  data->pathname,
			  acpi_ut_get_object_type_name(new_object),
			  package_index));

	*return_object_ptr = new_object;
	data->flags |= ACPI_OBJECT_REPAIRED;
	return (AE_OK);
}


void
acpi_ns_remove_null_elements(struct acpi_predefined_data *data,
			     u8 package_type,
			     union acpi_operand_object *obj_desc)
{
	union acpi_operand_object **source;
	union acpi_operand_object **dest;
	u32 count;
	u32 new_count;
	u32 i;

	ACPI_FUNCTION_NAME(ns_remove_null_elements);

	/*
	 * PTYPE1 packages contain no subpackages.
	 * PTYPE2 packages contain a variable number of sub-packages. We can
	 * safely remove all NULL elements from the PTYPE2 packages.
	 */
	switch (package_type) {
	case ACPI_PTYPE1_FIXED:
	case ACPI_PTYPE1_VAR:
	case ACPI_PTYPE1_OPTION:
		return;

	case ACPI_PTYPE2:
	case ACPI_PTYPE2_COUNT:
	case ACPI_PTYPE2_PKG_COUNT:
	case ACPI_PTYPE2_FIXED:
	case ACPI_PTYPE2_MIN:
	case ACPI_PTYPE2_REV_FIXED:
		break;

	default:
		return;
	}

	count = obj_desc->package.count;
	new_count = count;

	source = obj_desc->package.elements;
	dest = source;

	/* Examine all elements of the package object, remove nulls */

	for (i = 0; i < count; i++) {
		if (!*source) {
			new_count--;
		} else {
			*dest = *source;
			dest++;
		}
		source++;
	}

	/* Update parent package if any null elements were removed */

	if (new_count < count) {
		ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
				  "%s: Found and removed %u NULL elements\n",
				  data->pathname, (count - new_count)));

		/* NULL terminate list and update the package count */

		*dest = NULL;
		obj_desc->package.count = new_count;
	}
}


acpi_status
acpi_ns_repair_package_list(struct acpi_predefined_data *data,
			    union acpi_operand_object **obj_desc_ptr)
{
	union acpi_operand_object *pkg_obj_desc;

	ACPI_FUNCTION_NAME(ns_repair_package_list);

	/*
	 * Create the new outer package and populate it. The new package will
	 * have a single element, the lone subpackage.
	 */
	pkg_obj_desc = acpi_ut_create_package_object(1);
	if (!pkg_obj_desc) {
		return (AE_NO_MEMORY);
	}

	pkg_obj_desc->package.elements[0] = *obj_desc_ptr;

	/* Return the new object in the object pointer */

	*obj_desc_ptr = pkg_obj_desc;
	data->flags |= ACPI_OBJECT_REPAIRED;

	ACPI_DEBUG_PRINT((ACPI_DB_REPAIR,
			  "%s: Repaired incorrectly formed Package\n",
			  data->pathname));

	return (AE_OK);
}
