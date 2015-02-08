


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "zlib.h"

#define local static

#define SPAN 1048576L       /* desired distance between access points */
#define WINSIZE 32768U      /* sliding window size */
#define CHUNK 16384         /* file input buffer size */

/* access point entry */
struct point {
    off_t out;          /* corresponding offset in uncompressed data */
    off_t in;           /* offset in input file of first full byte */
    int bits;           /* number of bits (1-7) from byte at in - 1, or 0 */
    unsigned char window[WINSIZE];  /* preceding 32K of uncompressed data */
};

/* access point list */
struct access {
    int have;           /* number of list entries filled in */
    int size;           /* number of list entries allocated */
    struct point *list; /* allocated list */
};

/* Deallocate an index built by build_index() */
local void free_index(struct access *index)
{
    if (index != NULL) {
        free(index->list);
        free(index);
    }
}

local struct access *addpoint(struct access *index, int bits,
    off_t in, off_t out, unsigned left, unsigned char *window)
{
    struct point *next;

    /* if list is empty, create it (start with eight points) */
    if (index == NULL) {
        index = malloc(sizeof(struct access));
        if (index == NULL) return NULL;
        index->list = malloc(sizeof(struct point) << 3);
        if (index->list == NULL) {
            free(index);
            return NULL;
        }
        index->size = 8;
        index->have = 0;
    }

    /* if list is full, make it bigger */
    else if (index->have == index->size) {
        index->size <<= 1;
        next = realloc(index->list, sizeof(struct point) * index->size);
        if (next == NULL) {
            free_index(index);
            return NULL;
        }
        index->list = next;
    }

    /* fill in entry and increment how many we have */
    next = index->list + index->have;
    next->bits = bits;
    next->in = in;
    next->out = out;
    if (left)
        memcpy(next->window, window + WINSIZE - left, left);
    if (left < WINSIZE)
        memcpy(next->window + left, window, WINSIZE - left);
    index->have++;

    /* return list, possibly reallocated */
    return index;
}

local int build_index(FILE *in, off_t span, struct access **built)
{
    int ret;
    off_t totin, totout;        /* our own total counters to avoid 4GB limit */
    off_t last;                 /* totout value of last access point */
    struct access *index;       /* access points being generated */
    z_stream strm;
    unsigned char input[CHUNK];
    unsigned char window[WINSIZE];

    /* initialize inflate */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit2(&strm, 47);      /* automatic zlib or gzip decoding */
    if (ret != Z_OK)
        return ret;

    /* inflate the input, maintain a sliding window, and build an index -- this
       also validates the integrity of the compressed data using the check
       information at the end of the gzip or zlib stream */
    totin = totout = last = 0;
    index = NULL;               /* will be allocated by first addpoint() */
    strm.avail_out = 0;
    do {
        /* get some compressed data from input file */
        strm.avail_in = fread(input, 1, CHUNK, in);
        if (ferror(in)) {
            ret = Z_ERRNO;
            goto build_index_error;
        }
        if (strm.avail_in == 0) {
            ret = Z_DATA_ERROR;
            goto build_index_error;
        }
        strm.next_in = input;

        /* process all of that, or until end of stream */
        do {
            /* reset sliding window if necessary */
            if (strm.avail_out == 0) {
                strm.avail_out = WINSIZE;
                strm.next_out = window;
            }

            /* inflate until out of input, output, or at end of block --
               update the total input and output counters */
            totin += strm.avail_in;
            totout += strm.avail_out;
            ret = inflate(&strm, Z_BLOCK);      /* return at end of block */
            totin -= strm.avail_in;
            totout -= strm.avail_out;
            if (ret == Z_NEED_DICT)
                ret = Z_DATA_ERROR;
            if (ret == Z_MEM_ERROR || ret == Z_DATA_ERROR)
                goto build_index_error;
            if (ret == Z_STREAM_END)
                break;

            /* if at end of block, consider adding an index entry (note that if
               data_type indicates an end-of-block, then all of the
               uncompressed data from that block has been delivered, and none
               of the compressed data after that block has been consumed,
               except for up to seven bits) -- the totout == 0 provides an
               entry point after the zlib or gzip header, and assures that the
               index always has at least one access point; we avoid creating an
               access point after the last block by checking bit 6 of data_type
             */
            if ((strm.data_type & 128) && !(strm.data_type & 64) &&
                (totout == 0 || totout - last > span)) {
                index = addpoint(index, strm.data_type & 7, totin,
                                 totout, strm.avail_out, window);
                if (index == NULL) {
                    ret = Z_MEM_ERROR;
                    goto build_index_error;
                }
                last = totout;
            }
        } while (strm.avail_in != 0);
    } while (ret != Z_STREAM_END);

    /* clean up and return index (release unused entries in list) */
    (void)inflateEnd(&strm);
    index = realloc(index, sizeof(struct point) * index->have);
    index->size = index->have;
    *built = index;
    return index->size;

    /* return error */
  build_index_error:
    (void)inflateEnd(&strm);
    if (index != NULL)
        free_index(index);
    return ret;
}

local int extract(FILE *in, struct access *index, off_t offset,
                  unsigned char *buf, int len)
{
    int ret, skip;
    z_stream strm;
    struct point *here;
    unsigned char input[CHUNK];
    unsigned char discard[WINSIZE];

    /* proceed only if something reasonable to do */
    if (len < 0)
        return 0;

    /* find where in stream to start */
    here = index->list;
    ret = index->have;
    while (--ret && here[1].out <= offset)
        here++;

    /* initialize file and inflate state to start there */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit2(&strm, -15);         /* raw inflate */
    if (ret != Z_OK)
        return ret;
    ret = fseeko(in, here->in - (here->bits ? 1 : 0), SEEK_SET);
    if (ret == -1)
        goto extract_ret;
    if (here->bits) {
        ret = getc(in);
        if (ret == -1) {
            ret = ferror(in) ? Z_ERRNO : Z_DATA_ERROR;
            goto extract_ret;
        }
        (void)inflatePrime(&strm, here->bits, ret >> (8 - here->bits));
    }
    (void)inflateSetDictionary(&strm, here->window, WINSIZE);

    /* skip uncompressed bytes until offset reached, then satisfy request */
    offset -= here->out;
    strm.avail_in = 0;
    skip = 1;                               /* while skipping to offset */
    do {
        /* define where to put uncompressed data, and how much */
        if (offset == 0 && skip) {          /* at offset now */
            strm.avail_out = len;
            strm.next_out = buf;
            skip = 0;                       /* only do this once */
        }
        if (offset > WINSIZE) {             /* skip WINSIZE bytes */
            strm.avail_out = WINSIZE;
            strm.next_out = discard;
            offset -= WINSIZE;
        }
        else if (offset != 0) {             /* last skip */
            strm.avail_out = (unsigned)offset;
            strm.next_out = discard;
            offset = 0;
        }

        /* uncompress until avail_out filled, or end of stream */
        do {
            if (strm.avail_in == 0) {
                strm.avail_in = fread(input, 1, CHUNK, in);
                if (ferror(in)) {
                    ret = Z_ERRNO;
                    goto extract_ret;
                }
                if (strm.avail_in == 0) {
                    ret = Z_DATA_ERROR;
                    goto extract_ret;
                }
                strm.next_in = input;
            }
            ret = inflate(&strm, Z_NO_FLUSH);       /* normal inflate */
            if (ret == Z_NEED_DICT)
                ret = Z_DATA_ERROR;
            if (ret == Z_MEM_ERROR || ret == Z_DATA_ERROR)
                goto extract_ret;
            if (ret == Z_STREAM_END)
                break;
        } while (strm.avail_out != 0);

        /* if reach end of stream, then don't keep trying to get more */
        if (ret == Z_STREAM_END)
            break;

        /* do until offset reached and requested data read, or stream ends */
    } while (skip);

    /* compute number of uncompressed bytes read after offset */
    ret = skip ? 0 : len - strm.avail_out;

    /* clean up and return bytes read or error */
  extract_ret:
    (void)inflateEnd(&strm);
    return ret;
}

int main(int argc, char **argv)
{
    int len;
    off_t offset;
    FILE *in;
    struct access *index = NULL;
    unsigned char buf[CHUNK];

    /* open input file */
    if (argc != 2) {
        fprintf(stderr, "usage: zran file.gz\n");
        return 1;
    }
    in = fopen(argv[1], "rb");
    if (in == NULL) {
        fprintf(stderr, "zran: could not open %s for reading\n", argv[1]);
        return 1;
    }

    /* build index */
    len = build_index(in, SPAN, &index);
    if (len < 0) {
        fclose(in);
        switch (len) {
        case Z_MEM_ERROR:
            fprintf(stderr, "zran: out of memory\n");
            break;
        case Z_DATA_ERROR:
            fprintf(stderr, "zran: compressed data error in %s\n", argv[1]);
            break;
        case Z_ERRNO:
            fprintf(stderr, "zran: read error on %s\n", argv[1]);
            break;
        default:
            fprintf(stderr, "zran: error %d while building index\n", len);
        }
        return 1;
    }
    fprintf(stderr, "zran: built index with %d access points\n", len);

    /* use index by reading some bytes from an arbitrary offset */
    offset = (index->list[index->have - 1].out << 1) / 3;
    len = extract(in, index, offset, buf, CHUNK);
    if (len < 0)
        fprintf(stderr, "zran: extraction failed: %s error\n",
                len == Z_MEM_ERROR ? "out of memory" : "input corrupted");
    else {
        fwrite(buf, 1, len, stdout);
        fprintf(stderr, "zran: extracted %d bytes at %llu\n", len, offset);
    }

    /* clean up and exit */
    free_index(index);
    fclose(in);
    return 0;
}
