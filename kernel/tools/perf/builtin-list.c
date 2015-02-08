
#include "builtin.h"

#include "perf.h"

#include "util/parse-events.h"
#include "util/cache.h"

int cmd_list(int argc __used, const char **argv __used, const char *prefix __used)
{
	setup_pager();
	print_events();
	return 0;
}
