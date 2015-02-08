
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdarg.h>
#include <syslog.h>

#include <glib.h>

#include "log.h"

void info(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);

	vsyslog(LOG_INFO, format, ap);

	va_end(ap);
}

void error(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);

	vsyslog(LOG_ERR, format, ap);

	va_end(ap);
}

void btd_debug(const char *format, ...)
{
	va_list ap;

	va_start(ap, format);

	vsyslog(LOG_DEBUG, format, ap);

	va_end(ap);
}

extern struct btd_debug_desc __start___debug[];
extern struct btd_debug_desc __stop___debug[];

static gchar **enabled = NULL;

static gboolean is_enabled(struct btd_debug_desc *desc)
{
	int i;

	if (enabled == NULL)
		return 0;

	for (i = 0; enabled[i] != NULL; i++) {
		if (desc->name != NULL && g_pattern_match_simple(enabled[i],
							desc->name) == TRUE)
			return 1;
		if (desc->file != NULL && g_pattern_match_simple(enabled[i],
							desc->file) == TRUE)
			return 1;
	}

	return 0;
}

void __btd_toggle_debug()
{
	struct btd_debug_desc *desc;

	for (desc = __start___debug; desc < __stop___debug; desc++)
		desc->flags |= BTD_DEBUG_FLAG_PRINT;
}

void __btd_log_init(const char *debug, int detach)
{
	int option = LOG_NDELAY | LOG_PID;
	struct btd_debug_desc *desc;
	const char *name = NULL, *file = NULL;

	if (debug != NULL)
		enabled = g_strsplit_set(debug, ":, ", 0);

	for (desc = __start___debug; desc < __stop___debug; desc++) {
		if (file != NULL || name != NULL) {
			if (g_strcmp0(desc->file, file) == 0) {
				if (desc->name == NULL)
					desc->name = name;
			} else
				file = NULL;
		}

		if (is_enabled(desc))
			desc->flags |= BTD_DEBUG_FLAG_PRINT;
	}

	if (!detach)
		option |= LOG_PERROR;

	openlog("bluetoothd", option, LOG_DAEMON);

	syslog(LOG_INFO, "Bluetooth deamon %s", VERSION);
}

void __btd_log_cleanup(void)
{
	closelog();

	g_strfreev(enabled);
}
