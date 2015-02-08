


#ifndef _ACCONFIG_H
#define _ACCONFIG_H



#define ACPI_OS_NAME                    "Microsoft Windows NT"

/* Maximum objects in the various object caches */

#define ACPI_MAX_STATE_CACHE_DEPTH      96	/* State objects */
#define ACPI_MAX_PARSE_CACHE_DEPTH      96	/* Parse tree objects */
#define ACPI_MAX_EXTPARSE_CACHE_DEPTH   96	/* Parse tree objects */
#define ACPI_MAX_OBJECT_CACHE_DEPTH     96	/* Interpreter operand objects */
#define ACPI_MAX_NAMESPACE_CACHE_DEPTH  96	/* Namespace objects */

#define ACPI_CHECKSUM_ABORT             FALSE


/* Version of ACPI supported */

#define ACPI_CA_SUPPORT_LEVEL           3

/* Maximum count for a semaphore object */

#define ACPI_MAX_SEMAPHORE_COUNT        256

/* Maximum object reference count (detects object deletion issues) */

#define ACPI_MAX_REFERENCE_COUNT        0x1000

/* Default page size for use in mapping memory for operation regions */

#define ACPI_DEFAULT_PAGE_SIZE          4096	/* Must be power of 2 */

/* owner_id tracking. 8 entries allows for 255 owner_ids */

#define ACPI_NUM_OWNERID_MASKS          8

/* Size of the root table array is increased by this increment */

#define ACPI_ROOT_TABLE_SIZE_INCREMENT  4

/* Maximum number of While() loop iterations before forced abort */

#define ACPI_MAX_LOOP_ITERATIONS        0xFFFF

/* Maximum sleep allowed via Sleep() operator */

#define ACPI_MAX_SLEEP                  20000	/* Two seconds */


/* Number of distinct GPE register blocks and register width */

#define ACPI_MAX_GPE_BLOCKS             2
#define ACPI_GPE_REGISTER_WIDTH         8

/* Method info (in WALK_STATE), containing local variables and argumetns */

#define ACPI_METHOD_NUM_LOCALS          8
#define ACPI_METHOD_MAX_LOCAL           7

#define ACPI_METHOD_NUM_ARGS            7
#define ACPI_METHOD_MAX_ARG             6

/* Length of _HID, _UID, _CID, and UUID values */

#define ACPI_DEVICE_ID_LENGTH           0x09
#define ACPI_MAX_CID_LENGTH             48
#define ACPI_UUID_LENGTH                16

#define ACPI_OBJ_NUM_OPERANDS           8
#define ACPI_OBJ_MAX_OPERAND            7

/* Number of elements in the Result Stack frame, can be an arbitrary value */

#define ACPI_RESULTS_FRAME_OBJ_NUM      8

#define ACPI_RESULTS_OBJ_NUM_MAX        255

/* Names within the namespace are 4 bytes long */

#define ACPI_NAME_SIZE                  4
#define ACPI_PATH_SEGMENT_LENGTH        5	/* 4 chars for name + 1 char for separator */
#define ACPI_PATH_SEPARATOR             '.'

/* Sizes for ACPI table headers */

#define ACPI_OEM_ID_SIZE                6
#define ACPI_OEM_TABLE_ID_SIZE          8

/* Constants used in searching for the RSDP in low memory */

#define ACPI_EBDA_PTR_LOCATION          0x0000040E	/* Physical Address */
#define ACPI_EBDA_PTR_LENGTH            2
#define ACPI_EBDA_WINDOW_SIZE           1024
#define ACPI_HI_RSDP_WINDOW_BASE        0x000E0000	/* Physical Address */
#define ACPI_HI_RSDP_WINDOW_SIZE        0x00020000
#define ACPI_RSDP_SCAN_STEP             16

/* Operation regions */

#define ACPI_NUM_PREDEFINED_REGIONS     9
#define ACPI_USER_REGION_BEGIN          0x80

/* Maximum space_ids for Operation Regions */

#define ACPI_MAX_ADDRESS_SPACE          255

/* Array sizes.  Used for range checking also */

#define ACPI_MAX_MATCH_OPCODE           5

/* RSDP checksums */

#define ACPI_RSDP_CHECKSUM_LENGTH       20
#define ACPI_RSDP_XCHECKSUM_LENGTH      36

/* SMBus and IPMI bidirectional buffer size */

#define ACPI_SMBUS_BUFFER_SIZE          34
#define ACPI_IPMI_BUFFER_SIZE           66

/* _sx_d and _sx_w control methods */

#define ACPI_NUM_sx_d_METHODS           4
#define ACPI_NUM_sx_w_METHODS           5


#define ACPI_DEBUGGER_MAX_ARGS          8	/* Must be max method args + 1 */

#define ACPI_DEBUGGER_COMMAND_PROMPT    '-'
#define ACPI_DEBUGGER_EXECUTE_PROMPT    '%'

#endif				/* _ACCONFIG_H */
