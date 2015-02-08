
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "qemu-queue.h"
#include "envlist.h"

struct envlist_entry {
	const char *ev_var;			/* actual env value */
	QLIST_ENTRY(envlist_entry) ev_link;
};

struct envlist {
	QLIST_HEAD(, envlist_entry) el_entries;	/* actual entries */
	size_t el_count;			/* number of entries */
};

static int envlist_parse(envlist_t *envlist,
    const char *env, int (*)(envlist_t *, const char *));

envlist_t *
envlist_create(void)
{
	envlist_t *envlist;

	if ((envlist = malloc(sizeof (*envlist))) == NULL)
		return (NULL);

	QLIST_INIT(&envlist->el_entries);
	envlist->el_count = 0;

	return (envlist);
}

void
envlist_free(envlist_t *envlist)
{
	struct envlist_entry *entry;

	assert(envlist != NULL);

	while (envlist->el_entries.lh_first != NULL) {
		entry = envlist->el_entries.lh_first;
		QLIST_REMOVE(entry, ev_link);

		free((char *)entry->ev_var);
		free(entry);
	}
	free(envlist);
}

int
envlist_parse_set(envlist_t *envlist, const char *env)
{
	return (envlist_parse(envlist, env, &envlist_setenv));
}

int
envlist_parse_unset(envlist_t *envlist, const char *env)
{
	return (envlist_parse(envlist, env, &envlist_unsetenv));
}

static int
envlist_parse(envlist_t *envlist, const char *env,
    int (*callback)(envlist_t *, const char *))
{
	char *tmpenv, *envvar;
	char *envsave = NULL;

	assert(callback != NULL);

	if ((envlist == NULL) || (env == NULL))
		return (EINVAL);

	/*
	 * We need to make temporary copy of the env string
	 * as strtok_r(3) modifies it while it tokenizes.
	 */
	if ((tmpenv = strdup(env)) == NULL)
		return (errno);

	envvar = strtok_r(tmpenv, ",", &envsave);
	while (envvar != NULL) {
		if ((*callback)(envlist, envvar) != 0) {
			free(tmpenv);
			return (errno);
		}
		envvar = strtok_r(NULL, ",", &envsave);
	}

	free(tmpenv);
	return (0);
}

int
envlist_setenv(envlist_t *envlist, const char *env)
{
	struct envlist_entry *entry = NULL;
	const char *eq_sign;
	size_t envname_len;

	if ((envlist == NULL) || (env == NULL))
		return (EINVAL);

	/* find out first equals sign in given env */
	if ((eq_sign = strchr(env, '=')) == NULL)
		return (EINVAL);
	envname_len = eq_sign - env + 1;

	/*
	 * If there already exists variable with given name
	 * we remove and release it before allocating a whole
	 * new entry.
	 */
	for (entry = envlist->el_entries.lh_first; entry != NULL;
	    entry = entry->ev_link.le_next) {
		if (strncmp(entry->ev_var, env, envname_len) == 0)
			break;
	}

	if (entry != NULL) {
		QLIST_REMOVE(entry, ev_link);
		free((char *)entry->ev_var);
		free(entry);
	} else {
		envlist->el_count++;
	}

	if ((entry = malloc(sizeof (*entry))) == NULL)
		return (errno);
	if ((entry->ev_var = strdup(env)) == NULL) {
		free(entry);
		return (errno);
	}
	QLIST_INSERT_HEAD(&envlist->el_entries, entry, ev_link);

	return (0);
}

int
envlist_unsetenv(envlist_t *envlist, const char *env)
{
	struct envlist_entry *entry;
	size_t envname_len;

	if ((envlist == NULL) || (env == NULL))
		return (EINVAL);

	/* env is not allowed to contain '=' */
	if (strchr(env, '=') != NULL)
		return (EINVAL);

	/*
	 * Find out the requested entry and remove
	 * it from the list.
	 */
	envname_len = strlen(env);
	for (entry = envlist->el_entries.lh_first; entry != NULL;
	    entry = entry->ev_link.le_next) {
		if (strncmp(entry->ev_var, env, envname_len) == 0)
			break;
	}
	if (entry != NULL) {
		QLIST_REMOVE(entry, ev_link);
		free((char *)entry->ev_var);
		free(entry);

		envlist->el_count--;
	}
	return (0);
}

char **
envlist_to_environ(const envlist_t *envlist, size_t *count)
{
	struct envlist_entry *entry;
	char **env, **penv;

	penv = env = malloc((envlist->el_count + 1) * sizeof (char *));
	if (env == NULL)
		return (NULL);

	for (entry = envlist->el_entries.lh_first; entry != NULL;
	    entry = entry->ev_link.le_next) {
		*(penv++) = strdup(entry->ev_var);
	}
	*penv = NULL; /* NULL terminate the list */

	if (count != NULL)
		*count = envlist->el_count;

	return (env);
}
