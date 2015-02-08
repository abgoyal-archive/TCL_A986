
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "op_cpu_type.h"
#include "op_file.h"

op_interface op_get_interface(void)
{
	static op_interface current_interface = OP_INTERFACE_NO_GOOD;

	if (current_interface != OP_INTERFACE_NO_GOOD)
		return current_interface;

	if (op_file_readable("/proc/sys/dev/oprofile/cpu_type")) {
		current_interface = OP_INTERFACE_24;
	} else if (op_file_readable("/dev/oprofile/cpu_type")) {
		current_interface = OP_INTERFACE_26;
	}

	return current_interface;
}
