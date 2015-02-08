

#include "includes.h"

#include "common.h"
#include "config.h"
#include "base64.h"


struct wpa_config * wpa_config_read(const char *name)
{
	struct wpa_config *config;

	config = wpa_config_alloc_empty(NULL, NULL);
	if (config == NULL)
		return NULL;
	/* TODO: fill in configuration data */
	return config;
}


int wpa_config_write(const char *name, struct wpa_config *config)
{
	struct wpa_ssid *ssid;
	struct wpa_config_blob *blob;

	wpa_printf(MSG_DEBUG, "Writing configuration file '%s'", name);

	/* TODO: write global config parameters */


	for (ssid = config->ssid; ssid; ssid = ssid->next) {
		/* TODO: write networks */
	}

	for (blob = config->blobs; blob; blob = blob->next) {
		/* TODO: write blobs */
	}

	return 0;
}
