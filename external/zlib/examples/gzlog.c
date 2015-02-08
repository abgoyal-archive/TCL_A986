



#include <sys/types.h>
#include <stdio.h>      /* rename, fopen, fprintf, fclose */
#include <stdlib.h>     /* malloc, free */
#include <string.h>     /* strlen, strrchr, strcpy, strncpy, strcmp */
#include <fcntl.h>      /* open */
#include <unistd.h>     /* lseek, read, write, close, unlink, sleep, */
                        /* ftruncate, fsync */
#include <errno.h>      /* errno */
#include <time.h>       /* time, ctime */
#include <sys/stat.h>   /* stat */
#include <sys/time.h>   /* utimes */
#include "zlib.h"       /* crc32 */

#include "gzlog.h"      /* header for external access */

#define local static
typedef unsigned int uint;
typedef unsigned long ulong;

/* Macro for debugging to deterministically force recovery operations */
#ifdef DEBUG
    #include <setjmp.h>         /* longjmp */
    jmp_buf gzlog_jump;         /* where to go back to */
    int gzlog_bail = 0;         /* which point to bail at (1..8) */
    int gzlog_count = -1;       /* number of times through to wait */
#   define BAIL(n) do { if (n == gzlog_bail && gzlog_count-- == 0) \
                            longjmp(gzlog_jump, gzlog_bail); } while (0)
#else
#   define BAIL(n)
#endif

/* how old the lock file can be in seconds before considering it stale */
#define PATIENCE 300

/* maximum stored block size in Kbytes -- must be in 1..63 */
#define MAX_STORE 16

#define TRIGGER 1024

/* size of a deflate dictionary (this cannot be changed) */
#define DICT 32768U

/* values for the operation (2 bits) */
#define NO_OP 0
#define APPEND_OP 1
#define COMPRESS_OP 2
#define REPLACE_OP 3

/* macros to extract little-endian integers from an unsigned byte buffer */
#define PULL2(p) ((p)[0]+((uint)((p)[1])<<8))
#define PULL4(p) (PULL2(p)+((ulong)PULL2(p+2)<<16))
#define PULL8(p) (PULL4(p)+((off_t)PULL4(p+4)<<32))

/* macros to store integers into a byte buffer in little-endian order */
#define PUT2(p,a) do {(p)[0]=a;(p)[1]=(a)>>8;} while(0)
#define PUT4(p,a) do {PUT2(p,a);PUT2(p+2,a>>16);} while(0)
#define PUT8(p,a) do {PUT4(p,a);PUT4(p+4,a>>32);} while(0)

/* internal structure for log information */
#define LOGID "\106\035\172"    /* should be three non-zero characters */
struct log {
    char id[4];     /* contains LOGID to detect inadvertent overwrites */
    int fd;         /* file descriptor for .gz file, opened read/write */
    char *path;     /* allocated path, e.g. "/var/log/foo" or "foo" */
    char *end;      /* end of path, for appending suffices such as ".gz" */
    off_t first;    /* offset of first stored block first length byte */
    int back;       /* location of first block id in bits back from first */
    uint stored;    /* bytes currently in last stored block */
    off_t last;     /* offset of last stored block first length byte */
    ulong ccrc;     /* crc of compressed data */
    ulong clen;     /* length (modulo 2^32) of compressed data */
    ulong tcrc;     /* crc of total data */
    ulong tlen;     /* length (modulo 2^32) of total data */
    time_t lock;    /* last modify time of our lock file */
};

/* gzip header for gzlog */
local unsigned char log_gzhead[] = {
    0x1f, 0x8b,                 /* magic gzip id */
    8,                          /* compression method is deflate */
    4,                          /* there is an extra field (no file name) */
    0, 0, 0, 0,                 /* no modification time provided */
    0, 0xff,                    /* no extra flags, no OS specified */
    39, 0, 'a', 'p', 35, 0      /* extra field with "ap" subfield */
                                /* 35 is EXTRA, 39 is EXTRA + 4 */
};

#define HEAD sizeof(log_gzhead)     /* should be 16 */

/* initial gzip extra field content (52 == HEAD + EXTRA + 1) */
local unsigned char log_gzext[] = {
    52, 0, 0, 0, 0, 0, 0, 0,    /* offset of first stored block length */
    52, 0, 0, 0, 0, 0, 0, 0,    /* offset of last stored block length */
    0, 0, 0, 0, 0, 0, 0, 0,     /* compressed data crc and length */
    0, 0, 0, 0, 0, 0, 0, 0,     /* total data crc and length */
    0, 0,                       /* final stored block data length */
    5                           /* op is NO_OP, last bit 8 bits back */
};

#define EXTRA sizeof(log_gzext)     /* should be 35 */

/* initial gzip data and trailer */
local unsigned char log_gzbody[] = {
    1, 0, 0, 0xff, 0xff,        /* empty stored block (last) */
    0, 0, 0, 0,                 /* crc */
    0, 0, 0, 0                  /* uncompressed length */
};

#define BODY sizeof(log_gzbody)

local int log_lock(struct log *log)
{
    int fd;
    struct stat st;

    strcpy(log->end, ".lock");
    while ((fd = open(log->path, O_CREAT | O_EXCL, 0644)) < 0) {
        if (errno != EEXIST)
            return -1;
        if (stat(log->path, &st) == 0 && time(NULL) - st.st_mtime > PATIENCE) {
            unlink(log->path);
            continue;
        }
        sleep(2);       /* relinquish the CPU for two seconds while waiting */
    }
    close(fd);
    if (stat(log->path, &st) == 0)
        log->lock = st.st_mtime;
    return 0;
}

local void log_touch(struct log *log)
{
    struct stat st;

    strcpy(log->end, ".lock");
    utimes(log->path, NULL);
    if (stat(log->path, &st) == 0)
        log->lock = st.st_mtime;
}

local int log_check(struct log *log)
{
    struct stat st;

    strcpy(log->end, ".lock");
    if (stat(log->path, &st) || st.st_mtime != log->lock)
        return 1;
    log_touch(log);
    return 0;
}

/* Unlock a previously acquired lock, but only if it's ours. */
local void log_unlock(struct log *log)
{
    if (log_check(log))
        return;
    strcpy(log->end, ".lock");
    unlink(log->path);
    log->lock = 0;
}

local int log_head(struct log *log)
{
    int op;
    unsigned char buf[HEAD + EXTRA];

    if (lseek(log->fd, 0, SEEK_SET) < 0 ||
        read(log->fd, buf, HEAD + EXTRA) != HEAD + EXTRA ||
        memcmp(buf, log_gzhead, HEAD)) {
        return -1;
    }
    log->first = PULL8(buf + HEAD);
    log->last = PULL8(buf + HEAD + 8);
    log->ccrc = PULL4(buf + HEAD + 16);
    log->clen = PULL4(buf + HEAD + 20);
    log->tcrc = PULL4(buf + HEAD + 24);
    log->tlen = PULL4(buf + HEAD + 28);
    log->stored = PULL2(buf + HEAD + 32);
    log->back = 3 + (buf[HEAD + 34] & 7);
    op = (buf[HEAD + 34] >> 3) & 3;
    return op;
}

local int log_mark(struct log *log, int op)
{
    int ret;
    unsigned char ext[EXTRA];

    PUT8(ext, log->first);
    PUT8(ext + 8, log->last);
    PUT4(ext + 16, log->ccrc);
    PUT4(ext + 20, log->clen);
    PUT4(ext + 24, log->tcrc);
    PUT4(ext + 28, log->tlen);
    PUT2(ext + 32, log->stored);
    ext[34] = log->back - 3 + (op << 3);
    fsync(log->fd);
    ret = lseek(log->fd, HEAD, SEEK_SET) < 0 ||
          write(log->fd, ext, EXTRA) != EXTRA ? -1 : 0;
    fsync(log->fd);
    return ret;
}

local int log_last(struct log *log, int last)
{
    int back, len, mask;
    unsigned char buf[6];

    /* determine the locations of the bytes and bits to modify */
    back = log->last == log->first ? log->back : 8;
    len = back > 8 ? 2 : 1;                 /* bytes back from log->last */
    mask = 0x80 >> ((back - 1) & 7);        /* mask for block last-bit */

    /* get the byte to modify (one or two back) into buf[0] -- don't need to
       read the byte if the last-bit is eight bits back, since in that case
       the entire byte will be modified */
    buf[0] = 0;
    if (back != 8 && (lseek(log->fd, log->last - len, SEEK_SET) < 0 ||
                      read(log->fd, buf, 1) != 1))
        return -1;

    /* change the last-bit of the last stored block as requested -- note
       that all bits above the last-bit are set to zero, per the type bits
       of a stored block being 00 and per the convention that the bits to
       bring the stream to a byte boundary are also zeros */
    buf[1] = 0;
    buf[2 - len] = (*buf & (mask - 1)) + (last ? mask : 0);

    /* write the modified stored block header and lengths, move the file
       pointer to after the last stored block data */
    PUT2(buf + 2, log->stored);
    PUT2(buf + 4, log->stored ^ 0xffff);
    return lseek(log->fd, log->last - len, SEEK_SET) < 0 ||
           write(log->fd, buf + 2 - len, len + 4) != len + 4 ||
           lseek(log->fd, log->stored, SEEK_CUR) < 0 ? -1 : 0;
}

local int log_append(struct log *log, unsigned char *data, size_t len)
{
    uint put;
    off_t end;
    unsigned char buf[8];

    /* set the last block last-bit and length, in case recovering an
       interrupted append, then position the file pointer to append to the
       block */
    if (log_last(log, 1))
        return -1;

    /* append, adding stored blocks and updating the offset of the last stored
       block as needed, and update the total crc and length */
    while (len) {
        /* append as much as we can to the last block */
        put = (MAX_STORE << 10) - log->stored;
        if (put > len)
            put = (uint)len;
        if (put) {
            if (write(log->fd, data, put) != put)
                return -1;
            BAIL(1);
            log->tcrc = crc32(log->tcrc, data, put);
            log->tlen += put;
            log->stored += put;
            data += put;
            len -= put;
        }

        /* if we need to, add a new empty stored block */
        if (len) {
            /* mark current block as not last */
            if (log_last(log, 0))
                return -1;

            /* point to new, empty stored block */
            log->last += 4 + log->stored + 1;
            log->stored = 0;
        }

        /* mark last block as last, update its length */
        if (log_last(log, 1))
            return -1;
        BAIL(2);
    }

    /* write the new crc and length trailer, and truncate just in case (could
       be recovering from partial append with a missing foo.add file) */
    PUT4(buf, log->tcrc);
    PUT4(buf + 4, log->tlen);
    if (write(log->fd, buf, 8) != 8 ||
        (end = lseek(log->fd, 0, SEEK_CUR)) < 0 || ftruncate(log->fd, end))
        return -1;

    /* write the extra field, marking the log file as done, delete .add file */
    if (log_mark(log, NO_OP))
        return -1;
    strcpy(log->end, ".add");
    unlink(log->path);          /* ignore error, since may not exist */
    return 0;
}

local int log_replace(struct log *log)
{
    int ret;
    char *dest;

    /* delete foo.add file */
    strcpy(log->end, ".add");
    unlink(log->path);         /* ignore error, since may not exist */
    BAIL(3);

    /* rename foo.name to foo.dict, replacing foo.dict if it exists */
    strcpy(log->end, ".dict");
    dest = malloc(strlen(log->path) + 1);
    if (dest == NULL)
        return -2;
    strcpy(dest, log->path);
    strcpy(log->end, ".temp");
    ret = rename(log->path, dest);
    free(dest);
    if (ret && errno != ENOENT)
        return -1;
    BAIL(4);

    /* mark the foo.gz file as done */
    return log_mark(log, NO_OP);
}

local int log_compress(struct log *log, unsigned char *data, size_t len)
{
    int fd;
    uint got, max;
    ssize_t dict;
    off_t end;
    z_stream strm;
    unsigned char buf[DICT];

    /* compress and append compressed data */
    if (len) {
        /* set up for deflate, allocating memory */
        strm.zalloc = Z_NULL;
        strm.zfree = Z_NULL;
        strm.opaque = Z_NULL;
        if (deflateInit2(&strm, Z_DEFAULT_COMPRESSION, Z_DEFLATED, -15, 8,
                         Z_DEFAULT_STRATEGY) != Z_OK)
            return -2;

        /* read in dictionary (last 32K of data that was compressed) */
        strcpy(log->end, ".dict");
        fd = open(log->path, O_RDONLY, 0);
        if (fd >= 0) {
            dict = read(fd, buf, DICT);
            close(fd);
            if (dict < 0) {
                deflateEnd(&strm);
                return -1;
            }
            if (dict)
                deflateSetDictionary(&strm, buf, (uint)dict);
        }
        log_touch(log);

        /* prime deflate with last bits of previous block, position write
           pointer to write those bits and overwrite what follows */
        if (lseek(log->fd, log->first - (log->back > 8 ? 2 : 1),
                SEEK_SET) < 0 ||
            read(log->fd, buf, 1) != 1 || lseek(log->fd, -1, SEEK_CUR) < 0) {
            deflateEnd(&strm);
            return -1;
        }
        deflatePrime(&strm, (8 - log->back) & 7, *buf);

        /* compress, finishing with a partial non-last empty static block */
        strm.next_in = data;
        max = (((uint)0 - 1) >> 1) + 1; /* in case int smaller than size_t */
        do {
            strm.avail_in = len > max ? max : (uint)len;
            len -= strm.avail_in;
            do {
                strm.avail_out = DICT;
                strm.next_out = buf;
                deflate(&strm, len ? Z_NO_FLUSH : Z_PARTIAL_FLUSH);
                got = DICT - strm.avail_out;
                if (got && write(log->fd, buf, got) != got) {
                    deflateEnd(&strm);
                    return -1;
                }
                log_touch(log);
            } while (strm.avail_out == 0);
        } while (len);
        deflateEnd(&strm);
        BAIL(5);

        /* find start of empty static block -- scanning backwards the first one
           bit is the second bit of the block, if the last byte is zero, then
           we know the byte before that has a one in the top bit, since an
           empty static block is ten bits long */
        if ((log->first = lseek(log->fd, -1, SEEK_CUR)) < 0 ||
            read(log->fd, buf, 1) != 1)
            return -1;
        log->first++;
        if (*buf) {
            log->back = 1;
            while ((*buf & ((uint)1 << (8 - log->back++))) == 0)
                ;       /* guaranteed to terminate, since *buf != 0 */
        }
        else
            log->back = 10;

        /* update compressed crc and length */
        log->ccrc = log->tcrc;
        log->clen = log->tlen;
    }
    else {
        /* no data to compress -- fix up existing gzip stream */
        log->tcrc = log->ccrc;
        log->tlen = log->clen;
    }

    /* complete and truncate gzip stream */
    log->last = log->first;
    log->stored = 0;
    PUT4(buf, log->tcrc);
    PUT4(buf + 4, log->tlen);
    if (log_last(log, 1) || write(log->fd, buf, 8) != 8 ||
        (end = lseek(log->fd, 0, SEEK_CUR)) < 0 || ftruncate(log->fd, end))
        return -1;
    BAIL(6);

    /* mark as being in the replace operation */
    if (log_mark(log, REPLACE_OP))
        return -1;

    /* execute the replace operation and mark the file as done */
    return log_replace(log);
}

/* log a repair record to the .repairs file */
local void log_log(struct log *log, int op, char *record)
{
    time_t now;
    FILE *rec;

    now = time(NULL);
    strcpy(log->end, ".repairs");
    rec = fopen(log->path, "a");
    if (rec == NULL)
        return;
    fprintf(rec, "%.24s %s recovery: %s\n", ctime(&now), op == APPEND_OP ?
            "append" : (op == COMPRESS_OP ? "compress" : "replace"), record);
    fclose(rec);
    return;
}

local int log_recover(struct log *log, int op)
{
    int fd, ret = 0;
    unsigned char *data = NULL;
    size_t len = 0;
    struct stat st;

    /* log recovery */
    log_log(log, op, "start");

    /* load foo.add file if expected and present */
    if (op == APPEND_OP || op == COMPRESS_OP) {
        strcpy(log->end, ".add");
        if (stat(log->path, &st) == 0 && st.st_size) {
            len = (size_t)(st.st_size);
            if (len != st.st_size || (data = malloc(st.st_size)) == NULL) {
                log_log(log, op, "allocation failure");
                return -2;
            }
            if ((fd = open(log->path, O_RDONLY, 0)) < 0) {
                log_log(log, op, ".add file read failure");
                return -1;
            }
            ret = read(fd, data, len) != len;
            close(fd);
            if (ret) {
                log_log(log, op, ".add file read failure");
                return -1;
            }
            log_log(log, op, "loaded .add file");
        }
        else
            log_log(log, op, "missing .add file!");
    }

    /* recover the interrupted operation */
    switch (op) {
    case APPEND_OP:
        ret = log_append(log, data, len);
        break;
    case COMPRESS_OP:
        ret = log_compress(log, data, len);
        break;
    case REPLACE_OP:
        ret = log_replace(log);
    }

    /* log status */
    log_log(log, op, ret ? "failure" : "complete");

    /* clean up */
    if (data != NULL)
        free(data);
    return ret;
}

/* Close the foo.gz file (if open) and release the lock. */
local void log_close(struct log *log)
{
    if (log->fd >= 0)
        close(log->fd);
    log->fd = -1;
    log_unlock(log);
}

local int log_open(struct log *log)
{
    int op;

    /* release open file resource if left over -- can occur if lock lost
       between gzlog_open() and gzlog_write() */
    if (log->fd >= 0)
        close(log->fd);
    log->fd = -1;

    /* negotiate exclusive access */
    if (log_lock(log) < 0)
        return -1;

    /* open the log file, foo.gz */
    strcpy(log->end, ".gz");
    log->fd = open(log->path, O_RDWR | O_CREAT, 0644);
    if (log->fd < 0) {
        log_close(log);
        return -1;
    }

    /* if new, initialize foo.gz with an empty log, delete old dictionary */
    if (lseek(log->fd, 0, SEEK_END) == 0) {
        if (write(log->fd, log_gzhead, HEAD) != HEAD ||
            write(log->fd, log_gzext, EXTRA) != EXTRA ||
            write(log->fd, log_gzbody, BODY) != BODY) {
            log_close(log);
            return -1;
        }
        strcpy(log->end, ".dict");
        unlink(log->path);
    }

    /* verify log file and load extra field information */
    if ((op = log_head(log)) < 0) {
        log_close(log);
        return -1;
    }

    /* check for interrupted process and if so, recover */
    if (op != NO_OP && log_recover(log, op)) {
        log_close(log);
        return -1;
    }

    /* touch the lock file to prevent another process from grabbing it */
    log_touch(log);
    return 0;
}

/* See gzlog.h for the description of the external methods below */
gzlog *gzlog_open(char *path)
{
    size_t n;
    struct log *log;

    /* check arguments */
    if (path == NULL || *path == 0)
        return NULL;

    /* allocate and initialize log structure */
    log = malloc(sizeof(struct log));
    if (log == NULL)
        return NULL;
    strcpy(log->id, LOGID);
    log->fd = -1;

    /* save path and end of path for name construction */
    n = strlen(path);
    log->path = malloc(n + 9);              /* allow for ".repairs" */
    if (log->path == NULL) {
        free(log);
        return NULL;
    }
    strcpy(log->path, path);
    log->end = log->path + n;

    /* gain exclusive access and verify log file -- may perform a
       recovery operation if needed */
    if (log_open(log)) {
        free(log->path);
        free(log);
        return NULL;
    }

    /* return pointer to log structure */
    return log;
}

int gzlog_compress(gzlog *logd)
{
    int fd, ret;
    uint block;
    size_t len, next;
    unsigned char *data, buf[5];
    struct log *log = logd;

    /* check arguments */
    if (log == NULL || strcmp(log->id, LOGID) || len < 0)
        return -3;

    /* see if we lost the lock -- if so get it again and reload the extra
       field information (it probably changed), recover last operation if
       necessary */
    if (log_check(log) && log_open(log))
        return -1;

    /* create space for uncompressed data */
    len = ((size_t)(log->last - log->first) & ~(((size_t)1 << 10) - 1)) +
          log->stored;
    if ((data = malloc(len)) == NULL)
        return -2;

    /* do statement here is just a cheap trick for error handling */
    do {
        /* read in the uncompressed data */
        if (lseek(log->fd, log->first - 1, SEEK_SET) < 0)
            break;
        next = 0;
        while (next < len) {
            if (read(log->fd, buf, 5) != 5)
                break;
            block = PULL2(buf + 1);
            if (next + block > len ||
                read(log->fd, (char *)data + next, block) != block)
                break;
            next += block;
        }
        if (lseek(log->fd, 0, SEEK_CUR) != log->last + 4 + log->stored)
            break;
        log_touch(log);

        /* write the uncompressed data to the .add file */
        strcpy(log->end, ".add");
        fd = open(log->path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0)
            break;
        ret = write(fd, data, len) != len;
        if (ret | close(fd))
            break;
        log_touch(log);

        /* write the dictionary for the next compress to the .temp file */
        strcpy(log->end, ".temp");
        fd = open(log->path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (fd < 0)
            break;
        next = DICT > len ? len : DICT;
        ret = write(fd, (char *)data + len - next, next) != next;
        if (ret | close(fd))
            break;
        log_touch(log);

        /* roll back to compressed data, mark the compress in progress */
        log->last = log->first;
        log->stored = 0;
        if (log_mark(log, COMPRESS_OP))
            break;
        BAIL(7);

        /* compress and append the data (clears mark) */
        ret = log_compress(log, data, len);
        free(data);
        return ret;
    } while (0);

    /* broke out of do above on i/o error */
    free(data);
    return -1;
}

int gzlog_write(gzlog *logd, void *data, size_t len)
{
    int fd, ret;
    struct log *log = logd;

    /* check arguments */
    if (log == NULL || strcmp(log->id, LOGID) || len < 0)
        return -3;
    if (data == NULL || len == 0)
        return 0;

    /* see if we lost the lock -- if so get it again and reload the extra
       field information (it probably changed), recover last operation if
       necessary */
    if (log_check(log) && log_open(log))
        return -1;

    /* create and write .add file */
    strcpy(log->end, ".add");
    fd = open(log->path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0)
        return -1;
    ret = write(fd, data, len) != len;
    if (ret | close(fd))
        return -1;
    log_touch(log);

    /* mark log file with append in progress */
    if (log_mark(log, APPEND_OP))
        return -1;
    BAIL(8);

    /* append data (clears mark) */
    if (log_append(log, data, len))
        return -1;

    /* check to see if it's time to compress -- if not, then done */
    if (((log->last - log->first) >> 10) + (log->stored >> 10) < TRIGGER)
        return 0;

    /* time to compress */
    return gzlog_compress(log);
}

int gzlog_close(gzlog *logd)
{
    struct log *log = logd;

    /* check arguments */
    if (log == NULL || strcmp(log->id, LOGID))
        return -3;

    /* close the log file and release the lock */
    log_close(log);

    /* free structure and return */
    if (log->path != NULL)
        free(log->path);
    strcpy(log->id, "bad");
    free(log);
    return 0;
}
