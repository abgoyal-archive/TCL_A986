
#include "odb.h"

odb_node_t * odb_get_iterator(odb_t const * odb, odb_node_nr_t * nr)
{
	/* node zero is unused */
	*nr = odb->data->descr->current_size - 1;
	return odb->data->node_base + 1;
}
