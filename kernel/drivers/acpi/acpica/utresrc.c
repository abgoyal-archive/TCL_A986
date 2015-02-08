


#include <acpi/acpi.h>
#include "accommon.h"
#include "amlresrc.h"

#define _COMPONENT          ACPI_UTILITIES
ACPI_MODULE_NAME("utresrc")
#if defined(ACPI_DISASSEMBLER) || defined (ACPI_DEBUGGER)
const char *acpi_gbl_bm_decode[] = {
	"NotBusMaster",
	"BusMaster"
};

const char *acpi_gbl_config_decode[] = {
	"0 - Good Configuration",
	"1 - Acceptable Configuration",
	"2 - Suboptimal Configuration",
	"3 - ***Invalid Configuration***",
};

const char *acpi_gbl_consume_decode[] = {
	"ResourceProducer",
	"ResourceConsumer"
};

const char *acpi_gbl_dec_decode[] = {
	"PosDecode",
	"SubDecode"
};

const char *acpi_gbl_he_decode[] = {
	"Level",
	"Edge"
};

const char *acpi_gbl_io_decode[] = {
	"Decode10",
	"Decode16"
};

const char *acpi_gbl_ll_decode[] = {
	"ActiveHigh",
	"ActiveLow"
};

const char *acpi_gbl_max_decode[] = {
	"MaxNotFixed",
	"MaxFixed"
};

const char *acpi_gbl_mem_decode[] = {
	"NonCacheable",
	"Cacheable",
	"WriteCombining",
	"Prefetchable"
};

const char *acpi_gbl_min_decode[] = {
	"MinNotFixed",
	"MinFixed"
};

const char *acpi_gbl_mtp_decode[] = {
	"AddressRangeMemory",
	"AddressRangeReserved",
	"AddressRangeACPI",
	"AddressRangeNVS"
};

const char *acpi_gbl_rng_decode[] = {
	"InvalidRanges",
	"NonISAOnlyRanges",
	"ISAOnlyRanges",
	"EntireRange"
};

const char *acpi_gbl_rw_decode[] = {
	"ReadOnly",
	"ReadWrite"
};

const char *acpi_gbl_shr_decode[] = {
	"Exclusive",
	"Shared"
};

const char *acpi_gbl_siz_decode[] = {
	"Transfer8",
	"Transfer8_16",
	"Transfer16",
	"InvalidSize"
};

const char *acpi_gbl_trs_decode[] = {
	"DenseTranslation",
	"SparseTranslation"
};

const char *acpi_gbl_ttp_decode[] = {
	"TypeStatic",
	"TypeTranslation"
};

const char *acpi_gbl_typ_decode[] = {
	"Compatibility",
	"TypeA",
	"TypeB",
	"TypeF"
};

#endif

const u8 acpi_gbl_resource_aml_sizes[] = {
	/* Small descriptors */

	0,
	0,
	0,
	0,
	ACPI_AML_SIZE_SMALL(struct aml_resource_irq),
	ACPI_AML_SIZE_SMALL(struct aml_resource_dma),
	ACPI_AML_SIZE_SMALL(struct aml_resource_start_dependent),
	ACPI_AML_SIZE_SMALL(struct aml_resource_end_dependent),
	ACPI_AML_SIZE_SMALL(struct aml_resource_io),
	ACPI_AML_SIZE_SMALL(struct aml_resource_fixed_io),
	0,
	0,
	0,
	0,
	ACPI_AML_SIZE_SMALL(struct aml_resource_vendor_small),
	ACPI_AML_SIZE_SMALL(struct aml_resource_end_tag),

	/* Large descriptors */

	0,
	ACPI_AML_SIZE_LARGE(struct aml_resource_memory24),
	ACPI_AML_SIZE_LARGE(struct aml_resource_generic_register),
	0,
	ACPI_AML_SIZE_LARGE(struct aml_resource_vendor_large),
	ACPI_AML_SIZE_LARGE(struct aml_resource_memory32),
	ACPI_AML_SIZE_LARGE(struct aml_resource_fixed_memory32),
	ACPI_AML_SIZE_LARGE(struct aml_resource_address32),
	ACPI_AML_SIZE_LARGE(struct aml_resource_address16),
	ACPI_AML_SIZE_LARGE(struct aml_resource_extended_irq),
	ACPI_AML_SIZE_LARGE(struct aml_resource_address64),
	ACPI_AML_SIZE_LARGE(struct aml_resource_extended_address64)
};

static const u8 acpi_gbl_resource_types[] = {
	/* Small descriptors */

	0,
	0,
	0,
	0,
	ACPI_SMALL_VARIABLE_LENGTH,
	ACPI_FIXED_LENGTH,
	ACPI_SMALL_VARIABLE_LENGTH,
	ACPI_FIXED_LENGTH,
	ACPI_FIXED_LENGTH,
	ACPI_FIXED_LENGTH,
	0,
	0,
	0,
	0,
	ACPI_VARIABLE_LENGTH,
	ACPI_FIXED_LENGTH,

	/* Large descriptors */

	0,
	ACPI_FIXED_LENGTH,
	ACPI_FIXED_LENGTH,
	0,
	ACPI_VARIABLE_LENGTH,
	ACPI_FIXED_LENGTH,
	ACPI_FIXED_LENGTH,
	ACPI_VARIABLE_LENGTH,
	ACPI_VARIABLE_LENGTH,
	ACPI_VARIABLE_LENGTH,
	ACPI_VARIABLE_LENGTH,
	ACPI_FIXED_LENGTH
};


acpi_status
acpi_ut_walk_aml_resources(u8 * aml,
			   acpi_size aml_length,
			   acpi_walk_aml_callback user_function, void **context)
{
	acpi_status status;
	u8 *end_aml;
	u8 resource_index;
	u32 length;
	u32 offset = 0;

	ACPI_FUNCTION_TRACE(ut_walk_aml_resources);

	/* The absolute minimum resource template is one end_tag descriptor */

	if (aml_length < sizeof(struct aml_resource_end_tag)) {
		return_ACPI_STATUS(AE_AML_NO_RESOURCE_END_TAG);
	}

	/* Point to the end of the resource template buffer */

	end_aml = aml + aml_length;

	/* Walk the byte list, abort on any invalid descriptor type or length */

	while (aml < end_aml) {

		/* Validate the Resource Type and Resource Length */

		status = acpi_ut_validate_resource(aml, &resource_index);
		if (ACPI_FAILURE(status)) {
			return_ACPI_STATUS(status);
		}

		/* Get the length of this descriptor */

		length = acpi_ut_get_descriptor_length(aml);

		/* Invoke the user function */

		if (user_function) {
			status =
			    user_function(aml, length, offset, resource_index,
					  context);
			if (ACPI_FAILURE(status)) {
				return (status);
			}
		}

		/* An end_tag descriptor terminates this resource template */

		if (acpi_ut_get_resource_type(aml) ==
		    ACPI_RESOURCE_NAME_END_TAG) {
			/*
			 * There must be at least one more byte in the buffer for
			 * the 2nd byte of the end_tag
			 */
			if ((aml + 1) >= end_aml) {
				return_ACPI_STATUS(AE_AML_NO_RESOURCE_END_TAG);
			}

			/* Return the pointer to the end_tag if requested */

			if (!user_function) {
				*context = aml;
			}

			/* Normal exit */

			return_ACPI_STATUS(AE_OK);
		}

		aml += length;
		offset += length;
	}

	/* Did not find an end_tag descriptor */

	return (AE_AML_NO_RESOURCE_END_TAG);
}


acpi_status acpi_ut_validate_resource(void *aml, u8 * return_index)
{
	u8 resource_type;
	u8 resource_index;
	acpi_rs_length resource_length;
	acpi_rs_length minimum_resource_length;

	ACPI_FUNCTION_ENTRY();

	/*
	 * 1) Validate the resource_type field (Byte 0)
	 */
	resource_type = ACPI_GET8(aml);

	/*
	 * Byte 0 contains the descriptor name (Resource Type)
	 * Examine the large/small bit in the resource header
	 */
	if (resource_type & ACPI_RESOURCE_NAME_LARGE) {

		/* Verify the large resource type (name) against the max */

		if (resource_type > ACPI_RESOURCE_NAME_LARGE_MAX) {
			return (AE_AML_INVALID_RESOURCE_TYPE);
		}

		/*
		 * Large Resource Type -- bits 6:0 contain the name
		 * Translate range 0x80-0x8B to index range 0x10-0x1B
		 */
		resource_index = (u8) (resource_type - 0x70);
	} else {
		/*
		 * Small Resource Type -- bits 6:3 contain the name
		 * Shift range to index range 0x00-0x0F
		 */
		resource_index = (u8)
		    ((resource_type & ACPI_RESOURCE_NAME_SMALL_MASK) >> 3);
	}

	/* Check validity of the resource type, zero indicates name is invalid */

	if (!acpi_gbl_resource_types[resource_index]) {
		return (AE_AML_INVALID_RESOURCE_TYPE);
	}

	/*
	 * 2) Validate the resource_length field. This ensures that the length
	 *    is at least reasonable, and guarantees that it is non-zero.
	 */
	resource_length = acpi_ut_get_resource_length(aml);
	minimum_resource_length = acpi_gbl_resource_aml_sizes[resource_index];

	/* Validate based upon the type of resource - fixed length or variable */

	switch (acpi_gbl_resource_types[resource_index]) {
	case ACPI_FIXED_LENGTH:

		/* Fixed length resource, length must match exactly */

		if (resource_length != minimum_resource_length) {
			return (AE_AML_BAD_RESOURCE_LENGTH);
		}
		break;

	case ACPI_VARIABLE_LENGTH:

		/* Variable length resource, length must be at least the minimum */

		if (resource_length < minimum_resource_length) {
			return (AE_AML_BAD_RESOURCE_LENGTH);
		}
		break;

	case ACPI_SMALL_VARIABLE_LENGTH:

		/* Small variable length resource, length can be (Min) or (Min-1) */

		if ((resource_length > minimum_resource_length) ||
		    (resource_length < (minimum_resource_length - 1))) {
			return (AE_AML_BAD_RESOURCE_LENGTH);
		}
		break;

	default:

		/* Shouldn't happen (because of validation earlier), but be sure */

		return (AE_AML_INVALID_RESOURCE_TYPE);
	}

	/* Optionally return the resource table index */

	if (return_index) {
		*return_index = resource_index;
	}

	return (AE_OK);
}


u8 acpi_ut_get_resource_type(void *aml)
{
	ACPI_FUNCTION_ENTRY();

	/*
	 * Byte 0 contains the descriptor name (Resource Type)
	 * Examine the large/small bit in the resource header
	 */
	if (ACPI_GET8(aml) & ACPI_RESOURCE_NAME_LARGE) {

		/* Large Resource Type -- bits 6:0 contain the name */

		return (ACPI_GET8(aml));
	} else {
		/* Small Resource Type -- bits 6:3 contain the name */

		return ((u8) (ACPI_GET8(aml) & ACPI_RESOURCE_NAME_SMALL_MASK));
	}
}


u16 acpi_ut_get_resource_length(void *aml)
{
	acpi_rs_length resource_length;

	ACPI_FUNCTION_ENTRY();

	/*
	 * Byte 0 contains the descriptor name (Resource Type)
	 * Examine the large/small bit in the resource header
	 */
	if (ACPI_GET8(aml) & ACPI_RESOURCE_NAME_LARGE) {

		/* Large Resource type -- bytes 1-2 contain the 16-bit length */

		ACPI_MOVE_16_TO_16(&resource_length, ACPI_ADD_PTR(u8, aml, 1));

	} else {
		/* Small Resource type -- bits 2:0 of byte 0 contain the length */

		resource_length = (u16) (ACPI_GET8(aml) &
					 ACPI_RESOURCE_NAME_SMALL_LENGTH_MASK);
	}

	return (resource_length);
}


u8 acpi_ut_get_resource_header_length(void *aml)
{
	ACPI_FUNCTION_ENTRY();

	/* Examine the large/small bit in the resource header */

	if (ACPI_GET8(aml) & ACPI_RESOURCE_NAME_LARGE) {
		return (sizeof(struct aml_resource_large_header));
	} else {
		return (sizeof(struct aml_resource_small_header));
	}
}


u32 acpi_ut_get_descriptor_length(void *aml)
{
	ACPI_FUNCTION_ENTRY();

	/*
	 * Get the Resource Length (does not include header length) and add
	 * the header length (depends on if this is a small or large resource)
	 */
	return (acpi_ut_get_resource_length(aml) +
		acpi_ut_get_resource_header_length(aml));
}


acpi_status
acpi_ut_get_resource_end_tag(union acpi_operand_object * obj_desc,
			     u8 ** end_tag)
{
	acpi_status status;

	ACPI_FUNCTION_TRACE(ut_get_resource_end_tag);

	/* Allow a buffer length of zero */

	if (!obj_desc->buffer.length) {
		*end_tag = obj_desc->buffer.pointer;
		return_ACPI_STATUS(AE_OK);
	}

	/* Validate the template and get a pointer to the end_tag */

	status = acpi_ut_walk_aml_resources(obj_desc->buffer.pointer,
					    obj_desc->buffer.length, NULL,
					    (void **)end_tag);

	return_ACPI_STATUS(status);
}
