



#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "zlib.h"

#define local static

/* print nastygram and leave */
local void quit(char *why)
{
    fprintf(stderr, "fitblk abort: %s\n", why);
    exit(1);
}

#define RAWLEN 4096    /* intermediate uncompressed buffer size */

local int partcompress(FILE *in, z_streamp def)
{
    int ret, flush;
    unsigned char raw[RAWLEN];

    flush = Z_NO_FLUSH;
    do {
        def->avail_in = fread(raw, 1, RAWLEN, in);
        if (ferror(in))
            return Z_ERRNO;
        def->next_in = raw;
        if (feof(in))
            flush = Z_FINISH;
        ret = deflate(def, flush);
        assert(ret != Z_STREAM_ERROR);
    } while (def->avail_out != 0 && flush == Z_NO_FLUSH);
    return ret;
}

local int recompress(z_streamp inf, z_streamp def)
{
    int ret, flush;
    unsigned char raw[RAWLEN];

    flush = Z_NO_FLUSH;
    do {
        /* decompress */
        inf->avail_out = RAWLEN;
        inf->next_out = raw;
        ret = inflate(inf, Z_NO_FLUSH);
        assert(ret != Z_STREAM_ERROR && ret != Z_DATA_ERROR &&
               ret != Z_NEED_DICT);
        if (ret == Z_MEM_ERROR)
            return ret;

        /* compress what was decompresed until done or no room */
        def->avail_in = RAWLEN - inf->avail_out;
        def->next_in = raw;
        if (inf->avail_out != 0)
            flush = Z_FINISH;
        ret = deflate(def, flush);
        assert(ret != Z_STREAM_ERROR);
    } while (ret != Z_STREAM_END && def->avail_out != 0);
    return ret;
}

#define EXCESS 256      /* empirically determined stream overage */
#define MARGIN 8        /* amount to back off for completion */

/* compress from stdin to fixed-size block on stdout */
int main(int argc, char **argv)
{
    int ret;                /* return code */
    unsigned size;          /* requested fixed output block size */
    unsigned have;          /* bytes written by deflate() call */
    unsigned char *blk;     /* intermediate and final stream */
    unsigned char *tmp;     /* close to desired size stream */
    z_stream def, inf;      /* zlib deflate and inflate states */

    /* get requested output size */
    if (argc != 2)
        quit("need one argument: size of output block");
    ret = strtol(argv[1], argv + 1, 10);
    if (argv[1][0] != 0)
        quit("argument must be a number");
    if (ret < 8)            /* 8 is minimum zlib stream size */
        quit("need positive size of 8 or greater");
    size = (unsigned)ret;

    /* allocate memory for buffers and compression engine */
    blk = malloc(size + EXCESS);
    def.zalloc = Z_NULL;
    def.zfree = Z_NULL;
    def.opaque = Z_NULL;
    ret = deflateInit(&def, Z_DEFAULT_COMPRESSION);
    if (ret != Z_OK || blk == NULL)
        quit("out of memory");

    /* compress from stdin until output full, or no more input */
    def.avail_out = size + EXCESS;
    def.next_out = blk;
    ret = partcompress(stdin, &def);
    if (ret == Z_ERRNO)
        quit("error reading input");

    /* if it all fit, then size was undersubscribed -- done! */
    if (ret == Z_STREAM_END && def.avail_out >= EXCESS) {
        /* write block to stdout */
        have = size + EXCESS - def.avail_out;
        if (fwrite(blk, 1, have, stdout) != have || ferror(stdout))
            quit("error writing output");

        /* clean up and print results to stderr */
        ret = deflateEnd(&def);
        assert(ret != Z_STREAM_ERROR);
        free(blk);
        fprintf(stderr,
                "%u bytes unused out of %u requested (all input)\n",
                size - have, size);
        return 0;
    }

    /* it didn't all fit -- set up for recompression */
    inf.zalloc = Z_NULL;
    inf.zfree = Z_NULL;
    inf.opaque = Z_NULL;
    inf.avail_in = 0;
    inf.next_in = Z_NULL;
    ret = inflateInit(&inf);
    tmp = malloc(size + EXCESS);
    if (ret != Z_OK || tmp == NULL)
        quit("out of memory");
    ret = deflateReset(&def);
    assert(ret != Z_STREAM_ERROR);

    /* do first recompression close to the right amount */
    inf.avail_in = size + EXCESS;
    inf.next_in = blk;
    def.avail_out = size + EXCESS;
    def.next_out = tmp;
    ret = recompress(&inf, &def);
    if (ret == Z_MEM_ERROR)
        quit("out of memory");

    /* set up for next reocmpression */
    ret = inflateReset(&inf);
    assert(ret != Z_STREAM_ERROR);
    ret = deflateReset(&def);
    assert(ret != Z_STREAM_ERROR);

    /* do second and final recompression (third compression) */
    inf.avail_in = size - MARGIN;   /* assure stream will complete */
    inf.next_in = tmp;
    def.avail_out = size;
    def.next_out = blk;
    ret = recompress(&inf, &def);
    if (ret == Z_MEM_ERROR)
        quit("out of memory");
    assert(ret == Z_STREAM_END);    /* otherwise MARGIN too small */

    /* done -- write block to stdout */
    have = size - def.avail_out;
    if (fwrite(blk, 1, have, stdout) != have || ferror(stdout))
        quit("error writing output");

    /* clean up and print results to stderr */
    free(tmp);
    ret = inflateEnd(&inf);
    assert(ret != Z_STREAM_ERROR);
    ret = deflateEnd(&def);
    assert(ret != Z_STREAM_ERROR);
    free(blk);
    fprintf(stderr,
            "%u bytes unused out of %u requested (%lu input)\n",
            size - have, size, def.total_in);
    return 0;
}
