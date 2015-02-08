



/* external functions and related types and constants */
#include <stdio.h>          /* fprintf() */
#include <stdlib.h>         /* malloc(), free() */
#include <string.h>         /* strerror(), strcmp(), strlen(), memcpy() */
#include <errno.h>          /* errno */
#include <fcntl.h>          /* open() */
#include <unistd.h>         /* read(), write(), close(), chown(), unlink() */
#include <sys/types.h>
#include <sys/stat.h>       /* stat(), chmod() */
#include <utime.h>          /* utime() */
#include "zlib.h"           /* inflateBackInit(), inflateBack(), */
                            /* inflateBackEnd(), crc32() */

/* function declaration */
#define local static

/* buffer constants */
#define SIZE 32768U         /* input and output buffer sizes */
#define PIECE 16384         /* limits i/o chunks for 16-bit int case */

struct ind {
    int infile;
    unsigned char *inbuf;
};

local unsigned in(void *in_desc, unsigned char **buf)
{
    int ret;
    unsigned len;
    unsigned char *next;
    struct ind *me = (struct ind *)in_desc;

    next = me->inbuf;
    *buf = next;
    len = 0;
    do {
        ret = PIECE;
        if ((unsigned)ret > SIZE - len)
            ret = (int)(SIZE - len);
        ret = (int)read(me->infile, next, ret);
        if (ret == -1) {
            len = 0;
            break;
        }
        next += ret;
        len += ret;
    } while (ret != 0 && len < SIZE);
    return len;
}

struct outd {
    int outfile;
    int check;                  /* true if checking crc and total */
    unsigned long crc;
    unsigned long total;
};

local int out(void *out_desc, unsigned char *buf, unsigned len)
{
    int ret;
    struct outd *me = (struct outd *)out_desc;

    if (me->check) {
        me->crc = crc32(me->crc, buf, len);
        me->total += len;
    }
    if (me->outfile != -1)
        do {
            ret = PIECE;
            if ((unsigned)ret > len)
                ret = (int)len;
            ret = (int)write(me->outfile, buf, ret);
            if (ret == -1)
                return 1;
            buf += ret;
            len -= ret;
        } while (len != 0);
    return 0;
}

/* next input byte macro for use inside lunpipe() and gunpipe() */
#define NEXT() (have ? 0 : (have = in(indp, &next)), \
                last = have ? (have--, (int)(*next++)) : -1)

unsigned char inbuf[SIZE];              /* input buffer */
unsigned char outbuf[SIZE];             /* output buffer */
unsigned short prefix[65536];           /* index to LZW prefix string */
unsigned char suffix[65536];            /* one-character LZW suffix */
unsigned char match[65280 + 2];         /* buffer for reversed match or gzip
                                           32K sliding window */

#define FLUSHCODE() \
    do { \
        left = 0; \
        rem = 0; \
        if (chunk > have) { \
            chunk -= have; \
            have = 0; \
            if (NEXT() == -1) \
                break; \
            chunk--; \
            if (chunk > have) { \
                chunk = have = 0; \
                break; \
            } \
        } \
        have -= chunk; \
        next += chunk; \
        chunk = 0; \
    } while (0)

local int lunpipe(unsigned have, unsigned char *next, struct ind *indp,
                  int outfile, z_stream *strm)
{
    int last;                   /* last byte read by NEXT(), or -1 if EOF */
    unsigned chunk;             /* bytes left in current chunk */
    int left;                   /* bits left in rem */
    unsigned rem;               /* unused bits from input */
    int bits;                   /* current bits per code */
    unsigned code;              /* code, table traversal index */
    unsigned mask;              /* mask for current bits codes */
    int max;                    /* maximum bits per code for this stream */
    unsigned flags;             /* compress flags, then block compress flag */
    unsigned end;               /* last valid entry in prefix/suffix tables */
    unsigned temp;              /* current code */
    unsigned prev;              /* previous code */
    unsigned final;             /* last character written for previous code */
    unsigned stack;             /* next position for reversed string */
    unsigned outcnt;            /* bytes in output buffer */
    struct outd outd;           /* output structure */
    unsigned char *p;

    /* set up output */
    outd.outfile = outfile;
    outd.check = 0;

    /* process remainder of compress header -- a flags byte */
    flags = NEXT();
    if (last == -1)
        return Z_BUF_ERROR;
    if (flags & 0x60) {
        strm->msg = (char *)"unknown lzw flags set";
        return Z_DATA_ERROR;
    }
    max = flags & 0x1f;
    if (max < 9 || max > 16) {
        strm->msg = (char *)"lzw bits out of range";
        return Z_DATA_ERROR;
    }
    if (max == 9)                           /* 9 doesn't really mean 9 */
        max = 10;
    flags &= 0x80;                          /* true if block compress */

    /* clear table */
    bits = 9;
    mask = 0x1ff;
    end = flags ? 256 : 255;

    /* set up: get first 9-bit code, which is the first decompressed byte, but
       don't create a table entry until the next code */
    if (NEXT() == -1)                       /* no compressed data is ok */
        return Z_OK;
    final = prev = (unsigned)last;          /* low 8 bits of code */
    if (NEXT() == -1)                       /* missing a bit */
        return Z_BUF_ERROR;
    if (last & 1) {                         /* code must be < 256 */
        strm->msg = (char *)"invalid lzw code";
        return Z_DATA_ERROR;
    }
    rem = (unsigned)last >> 1;              /* remaining 7 bits */
    left = 7;
    chunk = bits - 2;                       /* 7 bytes left in this chunk */
    outbuf[0] = (unsigned char)final;       /* write first decompressed byte */
    outcnt = 1;

    /* decode codes */
    stack = 0;
    for (;;) {
        /* if the table will be full after this, increment the code size */
        if (end >= mask && bits < max) {
            FLUSHCODE();
            bits++;
            mask <<= 1;
            mask++;
        }

        /* get a code of length bits */
        if (chunk == 0)                     /* decrement chunk modulo bits */
            chunk = bits;
        code = rem;                         /* low bits of code */
        if (NEXT() == -1) {                 /* EOF is end of compressed data */
            /* write remaining buffered output */
            if (outcnt && out(&outd, outbuf, outcnt)) {
                strm->next_in = outbuf;     /* signal write error */
                return Z_BUF_ERROR;
            }
            return Z_OK;
        }
        code += (unsigned)last << left;     /* middle (or high) bits of code */
        left += 8;
        chunk--;
        if (bits > left) {                  /* need more bits */
            if (NEXT() == -1)               /* can't end in middle of code */
                return Z_BUF_ERROR;
            code += (unsigned)last << left; /* high bits of code */
            left += 8;
            chunk--;
        }
        code &= mask;                       /* mask to current code length */
        left -= bits;                       /* number of unused bits */
        rem = (unsigned)last >> (8 - left); /* unused bits from last byte */

        /* process clear code (256) */
        if (code == 256 && flags) {
            FLUSHCODE();
            bits = 9;                       /* initialize bits and mask */
            mask = 0x1ff;
            end = 255;                      /* empty table */
            continue;                       /* get next code */
        }

        /* special code to reuse last match */
        temp = code;                        /* save the current code */
        if (code > end) {
            /* Be picky on the allowed code here, and make sure that the code
               we drop through (prev) will be a valid index so that random
               input does not cause an exception.  The code != end + 1 check is
               empirically derived, and not checked in the original uncompress
               code.  If this ever causes a problem, that check could be safely
               removed.  Leaving this check in greatly improves gun's ability
               to detect random or corrupted input after a compress header.
               In any case, the prev > end check must be retained. */
            if (code != end + 1 || prev > end) {
                strm->msg = (char *)"invalid lzw code";
                return Z_DATA_ERROR;
            }
            match[stack++] = (unsigned char)final;
            code = prev;
        }

        /* walk through linked list to generate output in reverse order */
        p = match + stack;
        while (code >= 256) {
            *p++ = suffix[code];
            code = prefix[code];
        }
        stack = p - match;
        match[stack++] = (unsigned char)code;
        final = code;

        /* link new table entry */
        if (end < mask) {
            end++;
            prefix[end] = (unsigned short)prev;
            suffix[end] = (unsigned char)final;
        }

        /* set previous code for next iteration */
        prev = temp;

        /* write output in forward order */
        while (stack > SIZE - outcnt) {
            while (outcnt < SIZE)
                outbuf[outcnt++] = match[--stack];
            if (out(&outd, outbuf, outcnt)) {
                strm->next_in = outbuf; /* signal write error */
                return Z_BUF_ERROR;
            }
            outcnt = 0;
        }
        p = match + stack;
        do {
            outbuf[outcnt++] = *--p;
        } while (p > match);
        stack = 0;

        /* loop for next code with final and prev as the last match, rem and
           left provide the first 0..7 bits of the next code, end is the last
           valid table entry */
    }
}

local int gunpipe(z_stream *strm, int infile, int outfile)
{
    int ret, first, last;
    unsigned have, flags, len;
    unsigned char *next = NULL;
    struct ind ind, *indp;
    struct outd outd;

    /* setup input buffer */
    ind.infile = infile;
    ind.inbuf = inbuf;
    indp = &ind;

    /* decompress concatenated gzip streams */
    have = 0;                               /* no input data read in yet */
    first = 1;                              /* looking for first gzip header */
    strm->next_in = Z_NULL;                 /* so Z_BUF_ERROR means EOF */
    for (;;) {
        /* look for the two magic header bytes for a gzip stream */
        if (NEXT() == -1) {
            ret = Z_OK;
            break;                          /* empty gzip stream is ok */
        }
        if (last != 31 || (NEXT() != 139 && last != 157)) {
            strm->msg = (char *)"incorrect header check";
            ret = first ? Z_DATA_ERROR : Z_ERRNO;
            break;                          /* not a gzip or compress header */
        }
        first = 0;                          /* next non-header is junk */

        /* process a compress (LZW) file -- can't be concatenated after this */
        if (last == 157) {
            ret = lunpipe(have, next, indp, outfile, strm);
            break;
        }

        /* process remainder of gzip header */
        ret = Z_BUF_ERROR;
        if (NEXT() != 8) {                  /* only deflate method allowed */
            if (last == -1) break;
            strm->msg = (char *)"unknown compression method";
            ret = Z_DATA_ERROR;
            break;
        }
        flags = NEXT();                     /* header flags */
        NEXT();                             /* discard mod time, xflgs, os */
        NEXT();
        NEXT();
        NEXT();
        NEXT();
        NEXT();
        if (last == -1) break;
        if (flags & 0xe0) {
            strm->msg = (char *)"unknown header flags set";
            ret = Z_DATA_ERROR;
            break;
        }
        if (flags & 4) {                    /* extra field */
            len = NEXT();
            len += (unsigned)(NEXT()) << 8;
            if (last == -1) break;
            while (len > have) {
                len -= have;
                have = 0;
                if (NEXT() == -1) break;
                len--;
            }
            if (last == -1) break;
            have -= len;
            next += len;
        }
        if (flags & 8)                      /* file name */
            while (NEXT() != 0 && last != -1)
                ;
        if (flags & 16)                     /* comment */
            while (NEXT() != 0 && last != -1)
                ;
        if (flags & 2) {                    /* header crc */
            NEXT();
            NEXT();
        }
        if (last == -1) break;

        /* set up output */
        outd.outfile = outfile;
        outd.check = 1;
        outd.crc = crc32(0L, Z_NULL, 0);
        outd.total = 0;

        /* decompress data to output */
        strm->next_in = next;
        strm->avail_in = have;
        ret = inflateBack(strm, in, indp, out, &outd);
        if (ret != Z_STREAM_END) break;
        next = strm->next_in;
        have = strm->avail_in;
        strm->next_in = Z_NULL;             /* so Z_BUF_ERROR means EOF */

        /* check trailer */
        ret = Z_BUF_ERROR;
        if (NEXT() != (int)(outd.crc & 0xff) ||
            NEXT() != (int)((outd.crc >> 8) & 0xff) ||
            NEXT() != (int)((outd.crc >> 16) & 0xff) ||
            NEXT() != (int)((outd.crc >> 24) & 0xff)) {
            /* crc error */
            if (last != -1) {
                strm->msg = (char *)"incorrect data check";
                ret = Z_DATA_ERROR;
            }
            break;
        }
        if (NEXT() != (int)(outd.total & 0xff) ||
            NEXT() != (int)((outd.total >> 8) & 0xff) ||
            NEXT() != (int)((outd.total >> 16) & 0xff) ||
            NEXT() != (int)((outd.total >> 24) & 0xff)) {
            /* length error */
            if (last != -1) {
                strm->msg = (char *)"incorrect length check";
                ret = Z_DATA_ERROR;
            }
            break;
        }

        /* go back and look for another gzip stream */
    }

    /* clean up and return */
    return ret;
}

local void copymeta(char *from, char *to)
{
    struct stat was;
    struct utimbuf when;

    /* get all of from's Unix meta data, return if not a regular file */
    if (stat(from, &was) != 0 || (was.st_mode & S_IFMT) != S_IFREG)
        return;

    /* set to's mode bits, ignore errors */
    (void)chmod(to, was.st_mode & 07777);

    /* copy owner's user and group, ignore errors */
    (void)chown(to, was.st_uid, was.st_gid);

    /* copy access and modify times, ignore errors */
    when.actime = was.st_atime;
    when.modtime = was.st_mtime;
    (void)utime(to, &when);
}

local int gunzip(z_stream *strm, char *inname, char *outname, int test)
{
    int ret;
    int infile, outfile;

    /* open files */
    if (inname == NULL || *inname == 0) {
        inname = "-";
        infile = 0;     /* stdin */
    }
    else {
        infile = open(inname, O_RDONLY, 0);
        if (infile == -1) {
            fprintf(stderr, "gun cannot open %s\n", inname);
            return 0;
        }
    }
    if (test)
        outfile = -1;
    else if (outname == NULL || *outname == 0) {
        outname = "-";
        outfile = 1;    /* stdout */
    }
    else {
        outfile = open(outname, O_CREAT | O_TRUNC | O_WRONLY, 0666);
        if (outfile == -1) {
            close(infile);
            fprintf(stderr, "gun cannot create %s\n", outname);
            return 0;
        }
    }
    errno = 0;

    /* decompress */
    ret = gunpipe(strm, infile, outfile);
    if (outfile > 2) close(outfile);
    if (infile > 2) close(infile);

    /* interpret result */
    switch (ret) {
    case Z_OK:
    case Z_ERRNO:
        if (infile > 2 && outfile > 2) {
            copymeta(inname, outname);          /* copy attributes */
            unlink(inname);
        }
        if (ret == Z_ERRNO)
            fprintf(stderr, "gun warning: trailing garbage ignored in %s\n",
                    inname);
        break;
    case Z_DATA_ERROR:
        if (outfile > 2) unlink(outname);
        fprintf(stderr, "gun data error on %s: %s\n", inname, strm->msg);
        break;
    case Z_MEM_ERROR:
        if (outfile > 2) unlink(outname);
        fprintf(stderr, "gun out of memory error--aborting\n");
        return 1;
    case Z_BUF_ERROR:
        if (outfile > 2) unlink(outname);
        if (strm->next_in != Z_NULL) {
            fprintf(stderr, "gun write error on %s: %s\n",
                    outname, strerror(errno));
        }
        else if (errno) {
            fprintf(stderr, "gun read error on %s: %s\n",
                    inname, strerror(errno));
        }
        else {
            fprintf(stderr, "gun unexpected end of file on %s\n",
                    inname);
        }
        break;
    default:
        if (outfile > 2) unlink(outname);
        fprintf(stderr, "gun internal error--aborting\n");
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    int ret, len, test;
    char *outname;
    unsigned char *window;
    z_stream strm;

    /* initialize inflateBack state for repeated use */
    window = match;                         /* reuse LZW match buffer */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    ret = inflateBackInit(&strm, 15, window);
    if (ret != Z_OK) {
        fprintf(stderr, "gun out of memory error--aborting\n");
        return 1;
    }

    /* decompress each file to the same name with the suffix removed */
    argc--;
    argv++;
    test = 0;
    if (argc && strcmp(*argv, "-h") == 0) {
        fprintf(stderr, "gun 1.6 (17 Jan 2010)\n");
        fprintf(stderr, "Copyright (C) 2003-2010 Mark Adler\n");
        fprintf(stderr, "usage: gun [-t] [file1.gz [file2.Z ...]]\n");
        return 0;
    }
    if (argc && strcmp(*argv, "-t") == 0) {
        test = 1;
        argc--;
        argv++;
    }
    if (argc)
        do {
            if (test)
                outname = NULL;
            else {
                len = (int)strlen(*argv);
                if (strcmp(*argv + len - 3, ".gz") == 0 ||
                    strcmp(*argv + len - 3, "-gz") == 0)
                    len -= 3;
                else if (strcmp(*argv + len - 2, ".z") == 0 ||
                    strcmp(*argv + len - 2, "-z") == 0 ||
                    strcmp(*argv + len - 2, "_z") == 0 ||
                    strcmp(*argv + len - 2, ".Z") == 0)
                    len -= 2;
                else {
                    fprintf(stderr, "gun error: no gz type on %s--skipping\n",
                            *argv);
                    continue;
                }
                outname = malloc(len + 1);
                if (outname == NULL) {
                    fprintf(stderr, "gun out of memory error--aborting\n");
                    ret = 1;
                    break;
                }
                memcpy(outname, *argv, len);
                outname[len] = 0;
            }
            ret = gunzip(&strm, *argv, outname, test);
            if (outname != NULL) free(outname);
            if (ret) break;
        } while (argv++, --argc);
    else
        ret = gunzip(&strm, NULL, NULL, test);

    /* clean up */
    inflateBackEnd(&strm);
    return ret;
}
