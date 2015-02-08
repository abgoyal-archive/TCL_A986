
#include "op_get_time.h"

#include <time.h>

char * op_get_time(void)
{
	time_t t = time(NULL);

	if (t == -1)
		return "";

	return ctime(&t);
}
