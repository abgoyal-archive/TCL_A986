
#ifndef __TEXTFILE_H
#define __TEXTFILE_H

int create_dirs(const char *filename, const mode_t mode);
int create_file(const char *filename, const mode_t mode);
int create_name(char *buf, size_t size, const char *path,
				const char *address, const char *name);

int textfile_put(const char *pathname, const char *key, const char *value);
int textfile_caseput(const char *pathname, const char *key, const char *value);
int textfile_del(const char *pathname, const char *key);
int textfile_casedel(const char *pathname, const char *key);
char *textfile_get(const char *pathname, const char *key);
char *textfile_caseget(const char *pathname, const char *key);

int textfile_foreach(const char *pathname,
		void (*func)(char *key, char *value, void *data), void *data);

#endif /* __TEXTFILE_H */
