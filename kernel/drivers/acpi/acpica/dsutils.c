


#include <acpi/acpi.h>
#include "accommon.h"
#include "acparser.h"
#include "amlcode.h"
#include "acdispat.h"
#include "acinterp.h"
#include "acnamesp.h"
#include "acdebug.h"

#define _COMPONENT          ACPI_DISPATCHER
ACPI_MODULE_NAME("dsutils")

void acpi_ds_clear_implicit_return(struct acpi_walk_state *walk_state)
{
	ACPI_FUNCTION_NAME(ds_clear_implicit_return);

	/*
	 * Slack must be enabled for this feature
	 */
	if (!acpi_gbl_enable_interpreter_slack) {
		return;
	}

	if (walk_state->implicit_return_obj) {
		/*
		 * Delete any "stale" implicit return. However, in
		 * complex statements, the implicit return value can be
		 * bubbled up several levels.
		 */
		ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
				  "Removing reference on stale implicit return obj %p\n",
				  walk_state->implicit_return_obj));

		acpi_ut_remove_reference(walk_state->implicit_return_obj);
		walk_state->implicit_return_obj = NULL;
	}
}

#ifndef ACPI_NO_METHOD_EXECUTION

u8
acpi_ds_do_implicit_return(union acpi_operand_object *return_desc,
			   struct acpi_walk_state *walk_state, u8 add_reference)
{
	ACPI_FUNCTION_NAME(ds_do_implicit_return);

	/*
	 * Slack must be enabled for this feature, and we must
	 * have a valid return object
	 */
	if ((!acpi_gbl_enable_interpreter_slack) || (!return_desc)) {
		return (FALSE);
	}

	ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
			  "Result %p will be implicitly returned; Prev=%p\n",
			  return_desc, walk_state->implicit_return_obj));

	/*
	 * Delete any "stale" implicit return value first. However, in
	 * complex statements, the implicit return value can be
	 * bubbled up several levels, so we don't clear the value if it
	 * is the same as the return_desc.
	 */
	if (walk_state->implicit_return_obj) {
		if (walk_state->implicit_return_obj == return_desc) {
			return (TRUE);
		}
		acpi_ds_clear_implicit_return(walk_state);
	}

	/* Save the implicit return value, add a reference if requested */

	walk_state->implicit_return_obj = return_desc;
	if (add_reference) {
		acpi_ut_add_reference(return_desc);
	}

	return (TRUE);
}


u8
acpi_ds_is_result_used(union acpi_parse_object * op,
		       struct acpi_walk_state * walk_state)
{
	const struct acpi_opcode_info *parent_info;

	ACPI_FUNCTION_TRACE_PTR(ds_is_result_used, op);

	/* Must have both an Op and a Result Object */

	if (!op) {
		ACPI_ERROR((AE_INFO, "Null Op"));
		return_UINT8(TRUE);
	}

	/*
	 * We know that this operator is not a
	 * Return() operator (would not come here.) The following code is the
	 * optional support for a so-called "implicit return". Some AML code
	 * assumes that the last value of the method is "implicitly" returned
	 * to the caller. Just save the last result as the return value.
	 * NOTE: this is optional because the ASL language does not actually
	 * support this behavior.
	 */
	(void)acpi_ds_do_implicit_return(walk_state->result_obj, walk_state,
					 TRUE);

	/*
	 * Now determine if the parent will use the result
	 *
	 * If there is no parent, or the parent is a scope_op, we are executing
	 * at the method level. An executing method typically has no parent,
	 * since each method is parsed separately.  A method invoked externally
	 * via execute_control_method has a scope_op as the parent.
	 */
	if ((!op->common.parent) ||
	    (op->common.parent->common.aml_opcode == AML_SCOPE_OP)) {

		/* No parent, the return value cannot possibly be used */

		ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
				  "At Method level, result of [%s] not used\n",
				  acpi_ps_get_opcode_name(op->common.
							  aml_opcode)));
		return_UINT8(FALSE);
	}

	/* Get info on the parent. The root_op is AML_SCOPE */

	parent_info =
	    acpi_ps_get_opcode_info(op->common.parent->common.aml_opcode);
	if (parent_info->class == AML_CLASS_UNKNOWN) {
		ACPI_ERROR((AE_INFO, "Unknown parent opcode Op=%p", op));
		return_UINT8(FALSE);
	}

	/*
	 * Decide what to do with the result based on the parent.  If
	 * the parent opcode will not use the result, delete the object.
	 * Otherwise leave it as is, it will be deleted when it is used
	 * as an operand later.
	 */
	switch (parent_info->class) {
	case AML_CLASS_CONTROL:

		switch (op->common.parent->common.aml_opcode) {
		case AML_RETURN_OP:

			/* Never delete the return value associated with a return opcode */

			goto result_used;

		case AML_IF_OP:
		case AML_WHILE_OP:

			/*
			 * If we are executing the predicate AND this is the predicate op,
			 * we will use the return value
			 */
			if ((walk_state->control_state->common.state ==
			     ACPI_CONTROL_PREDICATE_EXECUTING)
			    && (walk_state->control_state->control.
				predicate_op == op)) {
				goto result_used;
			}
			break;

		default:
			/* Ignore other control opcodes */
			break;
		}

		/* The general control opcode returns no result */

		goto result_not_used;

	case AML_CLASS_CREATE:

		/*
		 * These opcodes allow term_arg(s) as operands and therefore
		 * the operands can be method calls.  The result is used.
		 */
		goto result_used;

	case AML_CLASS_NAMED_OBJECT:

		if ((op->common.parent->common.aml_opcode == AML_REGION_OP) ||
		    (op->common.parent->common.aml_opcode == AML_DATA_REGION_OP)
		    || (op->common.parent->common.aml_opcode == AML_PACKAGE_OP)
		    || (op->common.parent->common.aml_opcode ==
			AML_VAR_PACKAGE_OP)
		    || (op->common.parent->common.aml_opcode == AML_BUFFER_OP)
		    || (op->common.parent->common.aml_opcode ==
			AML_INT_EVAL_SUBTREE_OP)
		    || (op->common.parent->common.aml_opcode ==
			AML_BANK_FIELD_OP)) {
			/*
			 * These opcodes allow term_arg(s) as operands and therefore
			 * the operands can be method calls.  The result is used.
			 */
			goto result_used;
		}

		goto result_not_used;

	default:

		/*
		 * In all other cases. the parent will actually use the return
		 * object, so keep it.
		 */
		goto result_used;
	}

      result_used:
	ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
			  "Result of [%s] used by Parent [%s] Op=%p\n",
			  acpi_ps_get_opcode_name(op->common.aml_opcode),
			  acpi_ps_get_opcode_name(op->common.parent->common.
						  aml_opcode), op));

	return_UINT8(TRUE);

      result_not_used:
	ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
			  "Result of [%s] not used by Parent [%s] Op=%p\n",
			  acpi_ps_get_opcode_name(op->common.aml_opcode),
			  acpi_ps_get_opcode_name(op->common.parent->common.
						  aml_opcode), op));

	return_UINT8(FALSE);
}


void
acpi_ds_delete_result_if_not_used(union acpi_parse_object *op,
				  union acpi_operand_object *result_obj,
				  struct acpi_walk_state *walk_state)
{
	union acpi_operand_object *obj_desc;
	acpi_status status;

	ACPI_FUNCTION_TRACE_PTR(ds_delete_result_if_not_used, result_obj);

	if (!op) {
		ACPI_ERROR((AE_INFO, "Null Op"));
		return_VOID;
	}

	if (!result_obj) {
		return_VOID;
	}

	if (!acpi_ds_is_result_used(op, walk_state)) {

		/* Must pop the result stack (obj_desc should be equal to result_obj) */

		status = acpi_ds_result_pop(&obj_desc, walk_state);
		if (ACPI_SUCCESS(status)) {
			acpi_ut_remove_reference(result_obj);
		}
	}

	return_VOID;
}


acpi_status acpi_ds_resolve_operands(struct acpi_walk_state *walk_state)
{
	u32 i;
	acpi_status status = AE_OK;

	ACPI_FUNCTION_TRACE_PTR(ds_resolve_operands, walk_state);

	/*
	 * Attempt to resolve each of the valid operands
	 * Method arguments are passed by reference, not by value.  This means
	 * that the actual objects are passed, not copies of the objects.
	 */
	for (i = 0; i < walk_state->num_operands; i++) {
		status =
		    acpi_ex_resolve_to_value(&walk_state->operands[i],
					     walk_state);
		if (ACPI_FAILURE(status)) {
			break;
		}
	}

	return_ACPI_STATUS(status);
}


void acpi_ds_clear_operands(struct acpi_walk_state *walk_state)
{
	u32 i;

	ACPI_FUNCTION_TRACE_PTR(ds_clear_operands, walk_state);

	/* Remove a reference on each operand on the stack */

	for (i = 0; i < walk_state->num_operands; i++) {
		/*
		 * Remove a reference to all operands, including both
		 * "Arguments" and "Targets".
		 */
		acpi_ut_remove_reference(walk_state->operands[i]);
		walk_state->operands[i] = NULL;
	}

	walk_state->num_operands = 0;
	return_VOID;
}
#endif


acpi_status
acpi_ds_create_operand(struct acpi_walk_state *walk_state,
		       union acpi_parse_object *arg, u32 arg_index)
{
	acpi_status status = AE_OK;
	char *name_string;
	u32 name_length;
	union acpi_operand_object *obj_desc;
	union acpi_parse_object *parent_op;
	u16 opcode;
	acpi_interpreter_mode interpreter_mode;
	const struct acpi_opcode_info *op_info;

	ACPI_FUNCTION_TRACE_PTR(ds_create_operand, arg);

	/* A valid name must be looked up in the namespace */

	if ((arg->common.aml_opcode == AML_INT_NAMEPATH_OP) &&
	    (arg->common.value.string) &&
	    !(arg->common.flags & ACPI_PARSEOP_IN_STACK)) {
		ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH, "Getting a name: Arg=%p\n",
				  arg));

		/* Get the entire name string from the AML stream */

		status =
		    acpi_ex_get_name_string(ACPI_TYPE_ANY,
					    arg->common.value.buffer,
					    &name_string, &name_length);

		if (ACPI_FAILURE(status)) {
			return_ACPI_STATUS(status);
		}

		/* All prefixes have been handled, and the name is in name_string */

		/*
		 * Special handling for buffer_field declarations. This is a deferred
		 * opcode that unfortunately defines the field name as the last
		 * parameter instead of the first.  We get here when we are performing
		 * the deferred execution, so the actual name of the field is already
		 * in the namespace.  We don't want to attempt to look it up again
		 * because we may be executing in a different scope than where the
		 * actual opcode exists.
		 */
		if ((walk_state->deferred_node) &&
		    (walk_state->deferred_node->type == ACPI_TYPE_BUFFER_FIELD)
		    && (arg_index ==
			(u32) ((walk_state->opcode ==
				AML_CREATE_FIELD_OP) ? 3 : 2))) {
			obj_desc =
			    ACPI_CAST_PTR(union acpi_operand_object,
					  walk_state->deferred_node);
			status = AE_OK;
		} else {	/* All other opcodes */

			/*
			 * Differentiate between a namespace "create" operation
			 * versus a "lookup" operation (IMODE_LOAD_PASS2 vs.
			 * IMODE_EXECUTE) in order to support the creation of
			 * namespace objects during the execution of control methods.
			 */
			parent_op = arg->common.parent;
			op_info =
			    acpi_ps_get_opcode_info(parent_op->common.
						    aml_opcode);
			if ((op_info->flags & AML_NSNODE)
			    && (parent_op->common.aml_opcode !=
				AML_INT_METHODCALL_OP)
			    && (parent_op->common.aml_opcode != AML_REGION_OP)
			    && (parent_op->common.aml_opcode !=
				AML_INT_NAMEPATH_OP)) {

				/* Enter name into namespace if not found */

				interpreter_mode = ACPI_IMODE_LOAD_PASS2;
			} else {
				/* Return a failure if name not found */

				interpreter_mode = ACPI_IMODE_EXECUTE;
			}

			status =
			    acpi_ns_lookup(walk_state->scope_info, name_string,
					   ACPI_TYPE_ANY, interpreter_mode,
					   ACPI_NS_SEARCH_PARENT |
					   ACPI_NS_DONT_OPEN_SCOPE, walk_state,
					   ACPI_CAST_INDIRECT_PTR(struct
								  acpi_namespace_node,
								  &obj_desc));
			/*
			 * The only case where we pass through (ignore) a NOT_FOUND
			 * error is for the cond_ref_of opcode.
			 */
			if (status == AE_NOT_FOUND) {
				if (parent_op->common.aml_opcode ==
				    AML_COND_REF_OF_OP) {
					/*
					 * For the Conditional Reference op, it's OK if
					 * the name is not found;  We just need a way to
					 * indicate this to the interpreter, set the
					 * object to the root
					 */
					obj_desc = ACPI_CAST_PTR(union
								 acpi_operand_object,
								 acpi_gbl_root_node);
					status = AE_OK;
				} else {
					/*
					 * We just plain didn't find it -- which is a
					 * very serious error at this point
					 */
					status = AE_AML_NAME_NOT_FOUND;
				}
			}

			if (ACPI_FAILURE(status)) {
				ACPI_ERROR_NAMESPACE(name_string, status);
			}
		}

		/* Free the namestring created above */

		ACPI_FREE(name_string);

		/* Check status from the lookup */

		if (ACPI_FAILURE(status)) {
			return_ACPI_STATUS(status);
		}

		/* Put the resulting object onto the current object stack */

		status = acpi_ds_obj_stack_push(obj_desc, walk_state);
		if (ACPI_FAILURE(status)) {
			return_ACPI_STATUS(status);
		}
		ACPI_DEBUGGER_EXEC(acpi_db_display_argument_object
				   (obj_desc, walk_state));
	} else {
		/* Check for null name case */

		if ((arg->common.aml_opcode == AML_INT_NAMEPATH_OP) &&
		    !(arg->common.flags & ACPI_PARSEOP_IN_STACK)) {
			/*
			 * If the name is null, this means that this is an
			 * optional result parameter that was not specified
			 * in the original ASL.  Create a Zero Constant for a
			 * placeholder.  (Store to a constant is a Noop.)
			 */
			opcode = AML_ZERO_OP;	/* Has no arguments! */

			ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
					  "Null namepath: Arg=%p\n", arg));
		} else {
			opcode = arg->common.aml_opcode;
		}

		/* Get the object type of the argument */

		op_info = acpi_ps_get_opcode_info(opcode);
		if (op_info->object_type == ACPI_TYPE_INVALID) {
			return_ACPI_STATUS(AE_NOT_IMPLEMENTED);
		}

		if ((op_info->flags & AML_HAS_RETVAL)
		    || (arg->common.flags & ACPI_PARSEOP_IN_STACK)) {
			ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
					  "Argument previously created, already stacked\n"));

			ACPI_DEBUGGER_EXEC(acpi_db_display_argument_object
					   (walk_state->
					    operands[walk_state->num_operands -
						     1], walk_state));

			/*
			 * Use value that was already previously returned
			 * by the evaluation of this argument
			 */
			status = acpi_ds_result_pop(&obj_desc, walk_state);
			if (ACPI_FAILURE(status)) {
				/*
				 * Only error is underflow, and this indicates
				 * a missing or null operand!
				 */
				ACPI_EXCEPTION((AE_INFO, status,
						"Missing or null operand"));
				return_ACPI_STATUS(status);
			}
		} else {
			/* Create an ACPI_INTERNAL_OBJECT for the argument */

			obj_desc =
			    acpi_ut_create_internal_object(op_info->
							   object_type);
			if (!obj_desc) {
				return_ACPI_STATUS(AE_NO_MEMORY);
			}

			/* Initialize the new object */

			status =
			    acpi_ds_init_object_from_op(walk_state, arg, opcode,
							&obj_desc);
			if (ACPI_FAILURE(status)) {
				acpi_ut_delete_object_desc(obj_desc);
				return_ACPI_STATUS(status);
			}
		}

		/* Put the operand object on the object stack */

		status = acpi_ds_obj_stack_push(obj_desc, walk_state);
		if (ACPI_FAILURE(status)) {
			return_ACPI_STATUS(status);
		}

		ACPI_DEBUGGER_EXEC(acpi_db_display_argument_object
				   (obj_desc, walk_state));
	}

	return_ACPI_STATUS(AE_OK);
}


acpi_status
acpi_ds_create_operands(struct acpi_walk_state *walk_state,
			union acpi_parse_object *first_arg)
{
	acpi_status status = AE_OK;
	union acpi_parse_object *arg;
	union acpi_parse_object *arguments[ACPI_OBJ_NUM_OPERANDS];
	u32 arg_count = 0;
	u32 index = walk_state->num_operands;
	u32 i;

	ACPI_FUNCTION_TRACE_PTR(ds_create_operands, first_arg);

	/* Get all arguments in the list */

	arg = first_arg;
	while (arg) {
		if (index >= ACPI_OBJ_NUM_OPERANDS) {
			return_ACPI_STATUS(AE_BAD_DATA);
		}

		arguments[index] = arg;
		walk_state->operands[index] = NULL;

		/* Move on to next argument, if any */

		arg = arg->common.next;
		arg_count++;
		index++;
	}

	index--;

	/* It is the appropriate order to get objects from the Result stack */

	for (i = 0; i < arg_count; i++) {
		arg = arguments[index];

		/* Force the filling of the operand stack in inverse order */

		walk_state->operand_index = (u8) index;

		status = acpi_ds_create_operand(walk_state, arg, index);
		if (ACPI_FAILURE(status)) {
			goto cleanup;
		}

		index--;

		ACPI_DEBUG_PRINT((ACPI_DB_DISPATCH,
				  "Arg #%d (%p) done, Arg1=%p\n", index, arg,
				  first_arg));
	}

	return_ACPI_STATUS(status);

      cleanup:
	/*
	 * We must undo everything done above; meaning that we must
	 * pop everything off of the operand stack and delete those
	 * objects
	 */
	acpi_ds_obj_stack_pop_and_delete(arg_count, walk_state);

	ACPI_EXCEPTION((AE_INFO, status, "While creating Arg %d", index));
	return_ACPI_STATUS(status);
}


acpi_status acpi_ds_evaluate_name_path(struct acpi_walk_state *walk_state)
{
	acpi_status status = AE_OK;
	union acpi_parse_object *op = walk_state->op;
	union acpi_operand_object **operand = &walk_state->operands[0];
	union acpi_operand_object *new_obj_desc;
	u8 type;

	ACPI_FUNCTION_TRACE_PTR(ds_evaluate_name_path, walk_state);

	if (!op->common.parent) {

		/* This happens after certain exception processing */

		goto exit;
	}

	if ((op->common.parent->common.aml_opcode == AML_PACKAGE_OP) ||
	    (op->common.parent->common.aml_opcode == AML_VAR_PACKAGE_OP) ||
	    (op->common.parent->common.aml_opcode == AML_REF_OF_OP)) {

		/* TBD: Should we specify this feature as a bit of op_info->Flags of these opcodes? */

		goto exit;
	}

	status = acpi_ds_create_operand(walk_state, op, 0);
	if (ACPI_FAILURE(status)) {
		goto exit;
	}

	if (op->common.flags & ACPI_PARSEOP_TARGET) {
		new_obj_desc = *operand;
		goto push_result;
	}

	type = (*operand)->common.type;

	status = acpi_ex_resolve_to_value(operand, walk_state);
	if (ACPI_FAILURE(status)) {
		goto exit;
	}

	if (type == ACPI_TYPE_INTEGER) {

		/* It was incremented by acpi_ex_resolve_to_value */

		acpi_ut_remove_reference(*operand);

		status =
		    acpi_ut_copy_iobject_to_iobject(*operand, &new_obj_desc,
						    walk_state);
		if (ACPI_FAILURE(status)) {
			goto exit;
		}
	} else {
		/*
		 * The object either was anew created or is
		 * a Namespace node - don't decrement it.
		 */
		new_obj_desc = *operand;
	}

	/* Cleanup for name-path operand */

	status = acpi_ds_obj_stack_pop(1, walk_state);
	if (ACPI_FAILURE(status)) {
		walk_state->result_obj = new_obj_desc;
		goto exit;
	}

      push_result:

	walk_state->result_obj = new_obj_desc;

	status = acpi_ds_result_push(walk_state->result_obj, walk_state);
	if (ACPI_SUCCESS(status)) {

		/* Force to take it from stack */

		op->common.flags |= ACPI_PARSEOP_IN_STACK;
	}

      exit:

	return_ACPI_STATUS(status);
}
