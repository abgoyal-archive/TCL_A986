


#include <setjmp.h>             /* for setjmp(), longjmp(), and jmp_buf */
#include "blast.h"              /* prototype for blast() */

#define local static            /* for local function definitions */
#define MAXBITS 13              /* maximum code length */
#define MAXWIN 4096             /* maximum window size */

/* input and output state */
struct state {
    /* input state */
    blast_in infun;             /* input function provided by user */
    void *inhow;                /* opaque information passed to infun() */
    unsigned char *in;          /* next input location */
    unsigned left;              /* available input at in */
    int bitbuf;                 /* bit buffer */
    int bitcnt;                 /* number of bits in bit buffer */

    /* input limit error return state for bits() and decode() */
    jmp_buf env;

    /* output state */
    blast_out outfun;           /* output function provided by user */
    void *outhow;               /* opaque information passed to outfun() */
    unsigned next;              /* index of next write location in out[] */
    int first;                  /* true to check distances (for first 4K) */
    unsigned char out[MAXWIN];  /* output buffer and sliding window */
};

local int bits(struct state *s, int need)
{
    int val;            /* bit accumulator */

    /* load at least need bits into val */
    val = s->bitbuf;
    while (s->bitcnt < need) {
        if (s->left == 0) {
            s->left = s->infun(s->inhow, &(s->in));
            if (s->left == 0) longjmp(s->env, 1);       /* out of input */
        }
        val |= (int)(*(s->in)++) << s->bitcnt;          /* load eight bits */
        s->left--;
        s->bitcnt += 8;
    }

    /* drop need bits and update buffer, always zero to seven bits left */
    s->bitbuf = val >> need;
    s->bitcnt -= need;

    /* return need bits, zeroing the bits above that */
    return val & ((1 << need) - 1);
}

struct huffman {
    short *count;       /* number of symbols of each length */
    short *symbol;      /* canonically ordered symbols */
};

local int decode(struct state *s, struct huffman *h)
{
    int len;            /* current number of bits in code */
    int code;           /* len bits being decoded */
    int first;          /* first code of length len */
    int count;          /* number of codes of length len */
    int index;          /* index of first code of length len in symbol table */
    int bitbuf;         /* bits from stream */
    int left;           /* bits left in next or left to process */
    short *next;        /* next number of codes */

    bitbuf = s->bitbuf;
    left = s->bitcnt;
    code = first = index = 0;
    len = 1;
    next = h->count + 1;
    while (1) {
        while (left--) {
            code |= (bitbuf & 1) ^ 1;   /* invert code */
            bitbuf >>= 1;
            count = *next++;
            if (code < first + count) { /* if length len, return symbol */
                s->bitbuf = bitbuf;
                s->bitcnt = (s->bitcnt - len) & 7;
                return h->symbol[index + (code - first)];
            }
            index += count;             /* else update for next length */
            first += count;
            first <<= 1;
            code <<= 1;
            len++;
        }
        left = (MAXBITS+1) - len;
        if (left == 0) break;
        if (s->left == 0) {
            s->left = s->infun(s->inhow, &(s->in));
            if (s->left == 0) longjmp(s->env, 1);       /* out of input */
        }
        bitbuf = *(s->in)++;
        s->left--;
        if (left > 8) left = 8;
    }
    return -9;                          /* ran out of codes */
}

local int construct(struct huffman *h, const unsigned char *rep, int n)
{
    int symbol;         /* current symbol when stepping through length[] */
    int len;            /* current length when stepping through h->count[] */
    int left;           /* number of possible codes left of current length */
    short offs[MAXBITS+1];      /* offsets in symbol table for each length */
    short length[256];  /* code lengths */

    /* convert compact repeat counts into symbol bit length list */
    symbol = 0;
    do {
        len = *rep++;
        left = (len >> 4) + 1;
        len &= 15;
        do {
            length[symbol++] = len;
        } while (--left);
    } while (--n);
    n = symbol;

    /* count number of codes of each length */
    for (len = 0; len <= MAXBITS; len++)
        h->count[len] = 0;
    for (symbol = 0; symbol < n; symbol++)
        (h->count[length[symbol]])++;   /* assumes lengths are within bounds */
    if (h->count[0] == n)               /* no codes! */
        return 0;                       /* complete, but decode() will fail */

    /* check for an over-subscribed or incomplete set of lengths */
    left = 1;                           /* one possible code of zero length */
    for (len = 1; len <= MAXBITS; len++) {
        left <<= 1;                     /* one more bit, double codes left */
        left -= h->count[len];          /* deduct count from possible codes */
        if (left < 0) return left;      /* over-subscribed--return negative */
    }                                   /* left > 0 means incomplete */

    /* generate offsets into symbol table for each length for sorting */
    offs[1] = 0;
    for (len = 1; len < MAXBITS; len++)
        offs[len + 1] = offs[len] + h->count[len];

    /*
     * put symbols in table sorted by length, by symbol order within each
     * length
     */
    for (symbol = 0; symbol < n; symbol++)
        if (length[symbol] != 0)
            h->symbol[offs[length[symbol]]++] = symbol;

    /* return zero for complete set, positive for incomplete set */
    return left;
}

local int decomp(struct state *s)
{
    int lit;            /* true if literals are coded */
    int dict;           /* log2(dictionary size) - 6 */
    int symbol;         /* decoded symbol, extra bits for distance */
    int len;            /* length for copy */
    int dist;           /* distance for copy */
    int copy;           /* copy counter */
    unsigned char *from, *to;   /* copy pointers */
    static int virgin = 1;                              /* build tables once */
    static short litcnt[MAXBITS+1], litsym[256];        /* litcode memory */
    static short lencnt[MAXBITS+1], lensym[16];         /* lencode memory */
    static short distcnt[MAXBITS+1], distsym[64];       /* distcode memory */
    static struct huffman litcode = {litcnt, litsym};   /* length code */
    static struct huffman lencode = {lencnt, lensym};   /* length code */
    static struct huffman distcode = {distcnt, distsym};/* distance code */
        /* bit lengths of literal codes */
    static const unsigned char litlen[] = {
        11, 124, 8, 7, 28, 7, 188, 13, 76, 4, 10, 8, 12, 10, 12, 10, 8, 23, 8,
        9, 7, 6, 7, 8, 7, 6, 55, 8, 23, 24, 12, 11, 7, 9, 11, 12, 6, 7, 22, 5,
        7, 24, 6, 11, 9, 6, 7, 22, 7, 11, 38, 7, 9, 8, 25, 11, 8, 11, 9, 12,
        8, 12, 5, 38, 5, 38, 5, 11, 7, 5, 6, 21, 6, 10, 53, 8, 7, 24, 10, 27,
        44, 253, 253, 253, 252, 252, 252, 13, 12, 45, 12, 45, 12, 61, 12, 45,
        44, 173};
        /* bit lengths of length codes 0..15 */
    static const unsigned char lenlen[] = {2, 35, 36, 53, 38, 23};
        /* bit lengths of distance codes 0..63 */
    static const unsigned char distlen[] = {2, 20, 53, 230, 247, 151, 248};
    static const short base[16] = {     /* base for length codes */
        3, 2, 4, 5, 6, 7, 8, 9, 10, 12, 16, 24, 40, 72, 136, 264};
    static const char extra[16] = {     /* extra bits for length codes */
        0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8};

    /* set up decoding tables (once--might not be thread-safe) */
    if (virgin) {
        construct(&litcode, litlen, sizeof(litlen));
        construct(&lencode, lenlen, sizeof(lenlen));
        construct(&distcode, distlen, sizeof(distlen));
        virgin = 0;
    }

    /* read header */
    lit = bits(s, 8);
    if (lit > 1) return -1;
    dict = bits(s, 8);
    if (dict < 4 || dict > 6) return -2;

    /* decode literals and length/distance pairs */
    do {
        if (bits(s, 1)) {
            /* get length */
            symbol = decode(s, &lencode);
            len = base[symbol] + bits(s, extra[symbol]);
            if (len == 519) break;              /* end code */

            /* get distance */
            symbol = len == 2 ? 2 : dict;
            dist = decode(s, &distcode) << symbol;
            dist += bits(s, symbol);
            dist++;
            if (s->first && dist > s->next)
                return -3;              /* distance too far back */

            /* copy length bytes from distance bytes back */
            do {
                to = s->out + s->next;
                from = to - dist;
                copy = MAXWIN;
                if (s->next < dist) {
                    from += copy;
                    copy = dist;
                }
                copy -= s->next;
                if (copy > len) copy = len;
                len -= copy;
                s->next += copy;
                do {
                    *to++ = *from++;
                } while (--copy);
                if (s->next == MAXWIN) {
                    if (s->outfun(s->outhow, s->out, s->next)) return 1;
                    s->next = 0;
                    s->first = 0;
                }
            } while (len != 0);
        }
        else {
            /* get literal and write it */
            symbol = lit ? decode(s, &litcode) : bits(s, 8);
            s->out[s->next++] = symbol;
            if (s->next == MAXWIN) {
                if (s->outfun(s->outhow, s->out, s->next)) return 1;
                s->next = 0;
                s->first = 0;
            }
        }
    } while (1);
    return 0;
}

/* See comments in blast.h */
int blast(blast_in infun, void *inhow, blast_out outfun, void *outhow)
{
    struct state s;             /* input/output state */
    int err;                    /* return value */

    /* initialize input state */
    s.infun = infun;
    s.inhow = inhow;
    s.left = 0;
    s.bitbuf = 0;
    s.bitcnt = 0;

    /* initialize output state */
    s.outfun = outfun;
    s.outhow = outhow;
    s.next = 0;
    s.first = 1;

    /* return if bits() or decode() tries to read past available input */
    if (setjmp(s.env) != 0)             /* if came back here via longjmp(), */
        err = 2;                        /*  then skip decomp(), return error */
    else
        err = decomp(&s);               /* decompress */

    /* write any leftover output and update the error code if needed */
    if (err != 1 && s.next && s.outfun(s.outhow, s.out, s.next) && err == 0)
        err = 1;
    return err;
}

#ifdef TEST
/* Example of how to use blast() */
#include <stdio.h>
#include <stdlib.h>

#define CHUNK 16384

local unsigned inf(void *how, unsigned char **buf)
{
    static unsigned char hold[CHUNK];

    *buf = hold;
    return fread(hold, 1, CHUNK, (FILE *)how);
}

local int outf(void *how, unsigned char *buf, unsigned len)
{
    return fwrite(buf, 1, len, (FILE *)how) != len;
}

/* Decompress a PKWare Compression Library stream from stdin to stdout */
int main(void)
{
    int ret, n;

    /* decompress to stdout */
    ret = blast(inf, stdin, outf, stdout);
    if (ret != 0) fprintf(stderr, "blast error: %d\n", ret);

    /* see if there are any leftover bytes */
    n = 0;
    while (getchar() != EOF) n++;
    if (n) fprintf(stderr, "blast warning: %d unused bytes of input\n", n);

    /* return blast() error code */
    return ret;
}
#endif
