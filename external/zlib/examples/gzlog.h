



#ifndef GZLOG_H
#define GZLOG_H

/* gzlog object type */
typedef void gzlog;

gzlog *gzlog_open(char *path);

int gzlog_write(gzlog *log, void *data, size_t len);

int gzlog_compress(gzlog *log);

int gzlog_close(gzlog *log);

#endif
