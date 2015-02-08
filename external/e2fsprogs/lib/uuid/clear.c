
#include "string.h"

#include "uuidP.h"

void uuid_clear(uuid_t uu)
{
	memset(uu, 0, 16);
}

