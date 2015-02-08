
#include <unistd.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


main(int argc, char **argv)
{
	struct stat	statbuf;
	char *filename;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s dir\n", argv[0]);
		exit(1);
	}
	filename = argv[1];
	if (stat(filename, &statbuf) < 0) {
		perror(filename);
		exit(1);
	}
	if (S_ISDIR(statbuf.st_mode)) {
		if (!opendir(filename)) {
			perror(filename);
			exit(1);
		}
	} else {
		if (open(filename, O_RDONLY) < 0) {
			perror(filename);
			exit(1);
		}
	}
	sleep(30);
}
