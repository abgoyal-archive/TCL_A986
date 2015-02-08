#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <fcntl.h>

#define INLINE_DECLARE
#include "globals.h"

struct file_info {
	struct list_head head;
	FILE *ofp;
	char *oname;
};

struct buf_info {
	struct list_head head;
	void *buf;
};

LIST_HEAD(files_to_clean);
LIST_HEAD(all_bufs);

static void clean_files(void)
{
	struct list_head *p, *q;

	list_for_each_safe(p, q, &files_to_clean) {
		struct stat buf;
		struct file_info *fip = list_entry(p, struct file_info, head);

		fclose(fip->ofp);
		if (!stat(fip->oname, &buf) && (buf.st_size == 0))
			unlink(fip->oname);

		list_del(&fip->head);
		free(fip->oname);
		free(fip);
	}
}

static void clean_bufs(void)
{
	struct list_head *p, *q;

	list_for_each_safe(p, q, &all_bufs) {
		struct buf_info *bip = list_entry(p, struct buf_info, head);

		list_del(&bip->head);
		free(bip->buf);
		free(bip);
	}
}

#ifndef _ANDROID_
static int increase_limit(int resource, rlim_t increase)
{
	struct rlimit rlim;
	int save_errno = errno;

	if (!getrlimit(resource, &rlim)) {
		rlim.rlim_cur += increase;
		if (rlim.rlim_cur >= rlim.rlim_max)
			rlim.rlim_max = rlim.rlim_cur + increase;

		if (!setrlimit(resource, &rlim))
			return 1;
	}

	errno = save_errno;
	return 0;
}
#endif

static int handle_open_failure(void)
{
	if (errno == ENFILE || errno == EMFILE)
#ifndef _ANDROID_
		return increase_limit(RLIMIT_NOFILE, 16);
#else
		return -ENOSYS;
#endif

	return 0;
}

void add_file(FILE *fp, char *oname)
{
	struct file_info *fip = malloc(sizeof(*fip));

	fip->ofp = fp;
	fip->oname = oname;
	list_add_tail(&fip->head, &files_to_clean);
}

void add_buf(void *buf)
{
	struct buf_info *bip = malloc(sizeof(*bip));

	bip->buf = buf;
	list_add_tail(&bip->head, &all_bufs);
}

void clean_allocs(void)
{
	clean_files();
	clean_bufs();
}

char *make_dev_hdr(char *pad, size_t len, struct d_info *dip, int add_parens)
{
	if (dip->devmap)
		snprintf(pad, len, "%s", dip->devmap);
	else if (add_parens)
		snprintf(pad, len, "(%3d,%3d)",
			 MAJOR(dip->device), MINOR(dip->device));
	else
		snprintf(pad, len, "%d,%d",
			 MAJOR(dip->device), MINOR(dip->device));

	return pad;
}

FILE *my_fopen(const char *path, const char *mode)
{
	FILE *fp;

	do {
		fp = fopen(path, mode);
	} while (fp == NULL && handle_open_failure());

	return fp;
}

int my_open(const char *path, int flags)
{
	int fd;

	do {
		fd = open(path, flags);
	} while (fd < 0 && handle_open_failure());

	return fd;
}

void dbg_ping(void) {}
