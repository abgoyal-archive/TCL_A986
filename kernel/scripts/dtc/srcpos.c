

#include "dtc.h"
#include "srcpos.h"


struct dtc_file *srcpos_file;

static int dtc_open_one(struct dtc_file *file,
                        const char *search,
                        const char *fname)
{
	char *fullname;

	if (search) {
		fullname = xmalloc(strlen(search) + strlen(fname) + 2);

		strcpy(fullname, search);
		strcat(fullname, "/");
		strcat(fullname, fname);
	} else {
		fullname = strdup(fname);
	}

	file->file = fopen(fullname, "r");
	if (!file->file) {
		free(fullname);
		return 0;
	}

	file->name = fullname;
	return 1;
}


struct dtc_file *dtc_open_file(const char *fname,
                               const struct search_path *search)
{
	static const struct search_path default_search = { NULL, NULL, NULL };

	struct dtc_file *file;
	const char *slash;

	file = xmalloc(sizeof(struct dtc_file));

	slash = strrchr(fname, '/');
	if (slash) {
		char *dir = xmalloc(slash - fname + 1);

		memcpy(dir, fname, slash - fname);
		dir[slash - fname] = 0;
		file->dir = dir;
	} else {
		file->dir = NULL;
	}

	if (streq(fname, "-")) {
		file->name = "stdin";
		file->file = stdin;
		return file;
	}

	if (fname[0] == '/') {
		file->file = fopen(fname, "r");
		if (!file->file)
			goto fail;

		file->name = strdup(fname);
		return file;
	}

	if (!search)
		search = &default_search;

	while (search) {
		if (dtc_open_one(file, search->dir, fname))
			return file;

		if (errno != ENOENT)
			goto fail;

		search = search->next;
	}

fail:
	die("Couldn't open \"%s\": %s\n", fname, strerror(errno));
}

void dtc_close_file(struct dtc_file *file)
{
	if (fclose(file->file))
		die("Error closing \"%s\": %s\n", file->name, strerror(errno));

	free(file->dir);
	free(file);
}
