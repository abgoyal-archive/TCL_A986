
#include <stddef.h>
#include <time.h>
#include "pppd.h"

char pppd_version[] = VERSION;

static int minconnect = 0;

static option_t my_options[] = {
	{ "minconnect", o_int, &minconnect,
	  "Set minimum connect time before idle timeout applies" },
	{ NULL }
};

static int my_get_idle(struct ppp_idle *idle)
{
	time_t t;

	if (idle == NULL)
		return minconnect? minconnect: idle_time_limit;
	t = idle->xmit_idle;
	if (idle->recv_idle < t)
		t = idle->recv_idle;
	return idle_time_limit - t;
}

void plugin_init(void)
{
	info("plugin_init");
	add_options(my_options);
	idle_time_hook = my_get_idle;
}
