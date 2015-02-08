

/* Made up value to limit allocation sizes */
#include <string.h>
#include <stdlib.h>

#define IIO_MAX_NAME_LENGTH 30

#define IIO_EVENT_CODE_RING_50_FULL 200
#define IIO_EVENT_CODE_RING_75_FULL 201
#define IIO_EVENT_CODE_RING_100_FULL 202

const char *iio_dir = "/sys/bus/iio/devices/";

struct iio_event_data {
	int id;
	__s64 timestamp;
};

inline int find_type_by_name(const char *name, const char *type)
{
	const struct dirent *ent;
	int number, numstrlen;

	FILE *nameFile;
	DIR *dp;
	char thisname[IIO_MAX_NAME_LENGTH];
	char *filename;
	struct stat Stat;

	dp = opendir(iio_dir);
	if (dp == NULL) {
		printf("No industrialio devices available");
		return -ENODEV;
	}

	while (ent = readdir(dp), ent != NULL) {
		if (strcmp(ent->d_name, ".") != 0 &&
			strcmp(ent->d_name, "..") != 0 &&
			strlen(ent->d_name) > strlen(type) &&
			strncmp(ent->d_name, type, strlen(type)) == 0) {
			numstrlen = sscanf(ent->d_name + strlen(type),
					   "%d",
					   &number);
			/* verify the next character is not a colon */
			if (strncmp(ent->d_name + strlen(type) + numstrlen,
					":",
					1) != 0) {
				filename = malloc(strlen(iio_dir)
						+ strlen(type)
						+ numstrlen
						+ 6);
				if (filename == NULL)
					return -ENOMEM;
				sprintf(filename, "%s%s%d/name",
					iio_dir,
					type,
					number);
				nameFile = fopen(filename, "r");
				if (!nameFile)
					continue;
				free(filename);
				fscanf(nameFile, "%s", thisname);
				if (strcmp(name, thisname) == 0)
					return number;
				fclose(nameFile);
			}
		}
	}
	return -ENODEV;
}

inline int _write_sysfs_int(char *filename, char *basedir, int val, int verify)
{
	int ret;
	FILE *sysfsfp;
	int test;
	char *temp = malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;
	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		printf("failed to open %s\n", temp);
		ret = -errno;
		goto error_free;
	}
	fprintf(sysfsfp, "%d", val);
	fclose(sysfsfp);
	if (verify) {
		sysfsfp = fopen(temp, "r");
		if (sysfsfp == NULL) {
			printf("failed to open %s\n", temp);
			ret = -errno;
			goto error_free;
		}
		fscanf(sysfsfp, "%d", &test);
		if (test != val) {
			printf("Possible failure in int write %d to %s%s\n",
				val,
				basedir,
				filename);
			ret = -1;
		}
	}
error_free:
	free(temp);
	return ret;
}

int write_sysfs_int(char *filename, char *basedir, int val)
{
	return _write_sysfs_int(filename, basedir, val, 0);
}

int write_sysfs_int_and_verify(char *filename, char *basedir, int val)
{
	return _write_sysfs_int(filename, basedir, val, 1);
}

int _write_sysfs_string(char *filename, char *basedir, char *val, int verify)
{
	int ret;
	FILE  *sysfsfp;
	char *temp = malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL) {
		printf("Memory allocation failed\n");
		return -ENOMEM;
	}
	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		printf("Could not open %s\n", temp);
		ret = -errno;
		goto error_free;
	}
	fprintf(sysfsfp, "%s", val);
	fclose(sysfsfp);
	if (verify) {
		sysfsfp = fopen(temp, "r");
		if (sysfsfp == NULL) {
			ret = -errno;
			goto error_free;
		}
		fscanf(sysfsfp, "%s", temp);
		if (strcmp(temp, val) != 0) {
			printf("Possible failure in string write of %s "
				"Should be %s "
				"writen to %s\%s\n",
				temp,
				val,
				basedir,
				filename);
			ret = -1;
		}
	}
error_free:
	free(temp);

	return ret;
}
int write_sysfs_string_and_verify(char *filename, char *basedir, char *val)
{
	return _write_sysfs_string(filename, basedir, val, 1);
}

int write_sysfs_string(char *filename, char *basedir, char *val)
{
	return _write_sysfs_string(filename, basedir, val, 0);
}

int read_sysfs_posint(char *filename, char *basedir)
{
	int ret;
	FILE  *sysfsfp;
	char *temp = malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL) {
		printf("Memory allocation failed");
		return -ENOMEM;
	}
	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "r");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}
	fscanf(sysfsfp, "%d\n", &ret);
	fclose(sysfsfp);
error_free:
	free(temp);
	return ret;
}

int read_sysfs_float(char *filename, char *basedir, float *val)
{
	float ret = 0;
	FILE  *sysfsfp;
	char *temp = malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL) {
		printf("Memory allocation failed");
		return -ENOMEM;
	}
	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "r");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}
	fscanf(sysfsfp, "%f\n", val);
	fclose(sysfsfp);
error_free:
	free(temp);
	return ret;
}
