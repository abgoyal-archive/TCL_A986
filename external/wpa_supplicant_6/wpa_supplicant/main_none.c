

#include "includes.h"

#include "common.h"
#include "wpa_supplicant_i.h"

int main(int argc, char *argv[])
{
	struct wpa_interface iface;
	int exitcode = 0;
	struct wpa_params params;
	struct wpa_global *global;

	memset(&params, 0, sizeof(params));
	params.wpa_debug_level = MSG_INFO;

	global = wpa_supplicant_init(&params);
	if (global == NULL)
		return -1;

	memset(&iface, 0, sizeof(iface));
	/* TODO: set interface parameters */

	if (wpa_supplicant_add_iface(global, &iface) == NULL)
		exitcode = -1;

	if (exitcode == 0)
		exitcode = wpa_supplicant_run(global);

	wpa_supplicant_deinit(global);

	return exitcode;
}
