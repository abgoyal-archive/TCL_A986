
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <errno.h>
#include <dlfcn.h>
#include <string.h>
#include <sys/stat.h>

#include <bluetooth/bluetooth.h>

#include <glib.h>

#include "plugin.h"
#include "log.h"
#include "hcid.h"
#include "btio.h"

static GSList *plugins = NULL;

struct bluetooth_plugin {
	void *handle;
	gboolean active;
	struct bluetooth_plugin_desc *desc;
};

static gint compare_priority(gconstpointer a, gconstpointer b)
{
	const struct bluetooth_plugin *plugin1 = a;
	const struct bluetooth_plugin *plugin2 = b;

	return plugin2->desc->priority - plugin1->desc->priority;
}

static gboolean add_plugin(void *handle, struct bluetooth_plugin_desc *desc)
{
	struct bluetooth_plugin *plugin;

	if (desc->init == NULL)
		return FALSE;

	if (g_str_equal(desc->version, VERSION) == FALSE) {
		error("Version mismatch for %s", desc->name);
		return FALSE;
	}

	DBG("Loading %s plugin", desc->name);

	plugin = g_try_new0(struct bluetooth_plugin, 1);
	if (plugin == NULL)
		return FALSE;

	plugin->handle = handle;
	plugin->active = FALSE;
	plugin->desc = desc;

	plugins = g_slist_insert_sorted(plugins, plugin, compare_priority);

	return TRUE;
}

static gboolean is_disabled(const char *name, char **list)
{
	int i;

	if (list == NULL)
		return FALSE;

	for (i = 0; list[i] != NULL; i++) {
		char *str;
		gboolean equal;

		if (g_str_equal(name, list[i]))
			return TRUE;

		str = g_strdup_printf("%s.so", list[i]);

		equal = g_str_equal(str, name);

		g_free(str);

		if (equal)
			return TRUE;
	}

	return FALSE;
}

#include "builtin.h"

gboolean plugin_init(GKeyFile *config)
{
	GSList *list;
	GDir *dir;
	const gchar *file;
	gchar **disabled;
	unsigned int i;

	/* Make a call to BtIO API so its symbols got resolved before the
	 * plugins are loaded. */
	bt_io_error_quark();

	if (config)
		disabled = g_key_file_get_string_list(config, "General",
							"DisablePlugins",
							NULL, NULL);
	else
		disabled = NULL;

	DBG("Loading builtin plugins");

	for (i = 0; __bluetooth_builtin[i]; i++) {
		if (is_disabled(__bluetooth_builtin[i]->name, disabled))
			continue;

		add_plugin(NULL,  __bluetooth_builtin[i]);
	}

	if (strlen(PLUGINDIR) == 0) {
		g_strfreev(disabled);
		goto start;
	}

	DBG("Loading plugins %s", PLUGINDIR);

	dir = g_dir_open(PLUGINDIR, 0, NULL);
	if (!dir) {
		g_strfreev(disabled);
		goto start;
	}

	while ((file = g_dir_read_name(dir)) != NULL) {
		struct bluetooth_plugin_desc *desc;
		void *handle;
		gchar *filename;

		if (g_str_has_prefix(file, "lib") == TRUE ||
				g_str_has_suffix(file, ".so") == FALSE)
			continue;

		if (is_disabled(file, disabled))
			continue;

		filename = g_build_filename(PLUGINDIR, file, NULL);

		handle = dlopen(filename, RTLD_NOW);
		if (handle == NULL) {
			error("Can't load plugin %s: %s", filename,
								dlerror());
			g_free(filename);
			continue;
		}

		g_free(filename);

		desc = dlsym(handle, "bluetooth_plugin_desc");
		if (desc == NULL) {
			error("Can't load plugin description: %s", dlerror());
			dlclose(handle);
			continue;
		}

		if (add_plugin(handle, desc) == FALSE)
			dlclose(handle);
	}

	g_dir_close(dir);

	g_strfreev(disabled);

start:
	for (list = plugins; list; list = list->next) {
		struct bluetooth_plugin *plugin = list->data;

		if (plugin->desc->init() < 0) {
			error("Failed to init %s plugin", plugin->desc->name);
			continue;
		}

		plugin->active = TRUE;
	}

	return TRUE;
}

void plugin_cleanup(void)
{
	GSList *list;

	DBG("Cleanup plugins");

	for (list = plugins; list; list = list->next) {
		struct bluetooth_plugin *plugin = list->data;

		if (plugin->active == TRUE && plugin->desc->exit)
			plugin->desc->exit();

		if (plugin->handle != NULL)
			dlclose(plugin->handle);

		g_free(plugin);
	}

	g_slist_free(plugins);
}
