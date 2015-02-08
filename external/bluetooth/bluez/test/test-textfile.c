
#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#include "textfile.h"

static void print_entry(char *key, char *value, void *data)
{
	printf("%s %s\n", key, value);
}

int main(int argc, char *argv[])
{
	char filename[] = "/tmp/textfile";
	char key[18], value[512], *str;
	unsigned int i, j, size, max = 10;
	int fd, err;

	size = getpagesize();
	printf("System uses a page size of %d bytes\n\n", size);

	fd = creat(filename, 0644);
	err = ftruncate(fd, 0);

	memset(value, 0, sizeof(value));
	for (i = 0; i < (size / sizeof(value)); i++)
		err = write(fd, value, sizeof(value));

	close(fd);

	sprintf(key, "11:11:11:11:11:11");
	str = textfile_get(filename, key);

	err = truncate(filename, 0);


	sprintf(key, "00:00:00:00:00:00");
	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	memset(value, 0, sizeof(value));
	if (textfile_put(filename, key, value) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	str = textfile_get(filename, key);
	if (!str)
		fprintf(stderr, "No value for %s\n", key);
	else
		free(str);

	snprintf(value, sizeof(value), "Test");
	if (textfile_put(filename, key, value) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	if (textfile_put(filename, key, value) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	if (textfile_put(filename, key, value) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	str = textfile_get(filename, key);
	if (str) {
		fprintf(stderr, "Found value for %s\n", key);
		free(str);
	}

	for (i = 1; i < max + 1; i++) {
		sprintf(key, "00:00:00:00:00:%02X", i);

		memset(value, 0, sizeof(value));
		for (j = 0; j < i; j++)
			value[j] = 'x';

		printf("%s %s\n", key, value);

		if (textfile_put(filename, key, value) < 0) {
			fprintf(stderr, "%s (%d)\n", strerror(errno), errno);
			break;
		}

		str = textfile_get(filename, key);
		if (!str)
			fprintf(stderr, "No value for %s\n", key);
		else
			free(str);
	}


	sprintf(key, "00:00:00:00:00:%02X", max);

	memset(value, 0, sizeof(value));
	for (j = 0; j < max; j++)
		value[j] = 'y';

	if (textfile_put(filename, key, value) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	sprintf(key, "00:00:00:00:00:%02X", 1);

	memset(value, 0, sizeof(value));
	for (j = 0; j < max; j++)
		value[j] = 'z';

	if (textfile_put(filename, key, value) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	printf("\n");

	for (i = 1; i < max + 1; i++) {
		sprintf(key, "00:00:00:00:00:%02X", i);

		str = textfile_get(filename, key);
		if (str) {
			printf("%s %s\n", key, str);
			free(str);
		}
	}


	sprintf(key, "00:00:00:00:00:%02X", 2);

	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	sprintf(key, "00:00:00:00:00:%02X", max - 3);

	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	printf("\n");

	textfile_foreach(filename, print_entry, NULL);


	sprintf(key, "00:00:00:00:00:%02X", 1);

	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	sprintf(key, "00:00:00:00:00:%02X", max);

	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	sprintf(key, "00:00:00:00:00:%02X", max + 1);

	if (textfile_del(filename, key) < 0)
		fprintf(stderr, "%s (%d)\n", strerror(errno), errno);

	printf("\n");

	textfile_foreach(filename, print_entry, NULL);

	return 0;
}
