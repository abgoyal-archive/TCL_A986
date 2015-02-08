


#ifndef __ACSTRUCT_H__
#define __ACSTRUCT_H__

/* acpisrc:struct_defs -- for acpisrc conversion */


#define ACPI_NEXT_OP_DOWNWARD       1
#define ACPI_NEXT_OP_UPWARD         2

#define ACPI_WALK_NON_METHOD        0
#define ACPI_WALK_METHOD            0x01
#define ACPI_WALK_METHOD_RESTART    0x02

/* Flags for i_aSL compiler only */

#define ACPI_WALK_CONST_REQUIRED    0x10
#define ACPI_WALK_CONST_OPTIONAL    0x20

struct acpi_walk_state {
	struct acpi_walk_state *next;	/* Next walk_state in list */
	u8 descriptor_type;	/* To differentiate various internal objs */
	u8 walk_type;
	u16 opcode;		/* Current AML opcode */
	u8 next_op_info;	/* Info about next_op */
	u8 num_operands;	/* Stack pointer for Operands[] array */
	u8 operand_index;	/* Index into operand stack, to be used by acpi_ds_obj_stack_push */
	acpi_owner_id owner_id;	/* Owner of objects created during the walk */
	u8 last_predicate;	/* Result of last predicate */
	u8 current_result;
	u8 return_used;
	u8 scope_depth;
	u8 pass_number;		/* Parse pass during table load */
	u8 result_size;		/* Total elements for the result stack */
	u8 result_count;	/* Current number of occupied elements of result stack */
	u32 aml_offset;
	u32 arg_types;
	u32 method_breakpoint;	/* For single stepping */
	u32 user_breakpoint;	/* User AML breakpoint */
	u32 parse_flags;

	struct acpi_parse_state parser_state;	/* Current state of parser */
	u32 prev_arg_types;
	u32 arg_count;		/* push for fixed or var args */

	struct acpi_namespace_node arguments[ACPI_METHOD_NUM_ARGS];	/* Control method arguments */
	struct acpi_namespace_node local_variables[ACPI_METHOD_NUM_LOCALS];	/* Control method locals */
	union acpi_operand_object *operands[ACPI_OBJ_NUM_OPERANDS + 1];	/* Operands passed to the interpreter (+1 for NULL terminator) */
	union acpi_operand_object **params;

	u8 *aml_last_while;
	union acpi_operand_object **caller_return_desc;
	union acpi_generic_state *control_state;	/* List of control states (nested IFs) */
	struct acpi_namespace_node *deferred_node;	/* Used when executing deferred opcodes */
	union acpi_operand_object *implicit_return_obj;
	struct acpi_namespace_node *method_call_node;	/* Called method Node */
	union acpi_parse_object *method_call_op;	/* method_call Op if running a method */
	union acpi_operand_object *method_desc;	/* Method descriptor if running a method */
	struct acpi_namespace_node *method_node;	/* Method node if running a method. */
	union acpi_parse_object *op;	/* Current parser op */
	const struct acpi_opcode_info *op_info;	/* Info on current opcode */
	union acpi_parse_object *origin;	/* Start of walk [Obsolete] */
	union acpi_operand_object *result_obj;
	union acpi_generic_state *results;	/* Stack of accumulated results */
	union acpi_operand_object *return_desc;	/* Return object, if any */
	union acpi_generic_state *scope_info;	/* Stack of nested scopes */
	union acpi_parse_object *prev_op;	/* Last op that was processed */
	union acpi_parse_object *next_op;	/* next op to be processed */
	struct acpi_thread_state *thread;
	acpi_parse_downwards descending_callback;
	acpi_parse_upwards ascending_callback;
};

/* Info used by acpi_ps_init_objects */

struct acpi_init_walk_info {
	u16 method_count;
	u16 device_count;
	u16 op_region_count;
	u16 field_count;
	u16 buffer_count;
	u16 package_count;
	u16 op_region_init;
	u16 field_init;
	u16 buffer_init;
	u16 package_init;
	u16 object_count;
	acpi_owner_id owner_id;
	u32 table_index;
};

struct acpi_get_devices_info {
	acpi_walk_callback user_function;
	void *context;
	const char *hid;
};

union acpi_aml_operands {
	union acpi_operand_object *operands[7];

	struct {
		struct acpi_object_integer *type;
		struct acpi_object_integer *code;
		struct acpi_object_integer *argument;

	} fatal;

	struct {
		union acpi_operand_object *source;
		struct acpi_object_integer *index;
		union acpi_operand_object *target;

	} index;

	struct {
		union acpi_operand_object *source;
		struct acpi_object_integer *index;
		struct acpi_object_integer *length;
		union acpi_operand_object *target;

	} mid;
};

struct acpi_evaluate_info {
	struct acpi_namespace_node *prefix_node;
	char *pathname;
	union acpi_operand_object *obj_desc;
	union acpi_operand_object **parameters;
	struct acpi_namespace_node *resolved_node;
	union acpi_operand_object *return_object;
	u8 param_count;
	u8 pass_number;
	u8 return_object_type;
	u8 flags;
};

/* Values for Flags above */

#define ACPI_IGNORE_RETURN_VALUE        1

/* Info used by acpi_ns_initialize_devices */

struct acpi_device_walk_info {
	u16 device_count;
	u16 num_STA;
	u16 num_INI;
	struct acpi_table_desc *table_desc;
	struct acpi_evaluate_info *evaluate_info;
};

/* TBD: [Restructure] Merge with struct above */

struct acpi_walk_info {
	u32 debug_level;
	u32 count;
	acpi_owner_id owner_id;
	u8 display_type;
};

/* Display Types */

#define ACPI_DISPLAY_SUMMARY        (u8) 0
#define ACPI_DISPLAY_OBJECTS        (u8) 1
#define ACPI_DISPLAY_MASK           (u8) 1

#define ACPI_DISPLAY_SHORT          (u8) 2

#endif
