



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define local static

/* special data types */
typedef unsigned long long big_t;   /* type for code counting */
typedef unsigned long long code_t;  /* type for bit pattern counting */
struct tab {                        /* type for been here check */
    size_t len;         /* length of bit vector in char's */
    char *vec;          /* allocated bit vector */
};



/* Globals to avoid propagating constants or constant pointers recursively */
local int max;          /* maximum allowed bit length for the codes */
local int root;         /* size of base code table in bits */
local int large;        /* largest code table so far */
local size_t size;      /* number of elements in num and done */
local int *code;        /* number of symbols assigned to each bit length */
local big_t *num;       /* saved results array for code counting */
local struct tab *done; /* states already evaluated array */

/* Index function for num[] and done[] */
#define INDEX(i,j,k) (((size_t)((i-1)>>1)*((i-2)>>1)+(j>>1)-1)*(max-1)+k-1)

/* Free allocated space.  Uses globals code, num, and done. */
local void cleanup(void)
{
    size_t n;

    if (done != NULL) {
        for (n = 0; n < size; n++)
            if (done[n].len)
                free(done[n].vec);
        free(done);
    }
    if (num != NULL)
        free(num);
    if (code != NULL)
        free(code);
}

local big_t count(int syms, int len, int left)
{
    big_t sum;          /* number of possible codes from this juncture */
    big_t got;          /* value returned from count() */
    int least;          /* least number of syms to use at this juncture */
    int most;           /* most number of syms to use at this juncture */
    int use;            /* number of bit patterns to use in next call */
    size_t index;       /* index of this case in *num */

    /* see if only one possible code */
    if (syms == left)
        return 1;

    /* note and verify the expected state */
    assert(syms > left && left > 0 && len < max);

    /* see if we've done this one already */
    index = INDEX(syms, left, len);
    got = num[index];
    if (got)
        return got;         /* we have -- return the saved result */

    /* we need to use at least this many bit patterns so that the code won't be
       incomplete at the next length (more bit patterns than symbols) */
    least = (left << 1) - syms;
    if (least < 0)
        least = 0;

    /* we can use at most this many bit patterns, lest there not be enough
       available for the remaining symbols at the maximum length (if there were
       no limit to the code length, this would become: most = left - 1) */
    most = (((code_t)left << (max - len)) - syms) /
            (((code_t)1 << (max - len)) - 1);

    /* count all possible codes from this juncture and add them up */
    sum = 0;
    for (use = least; use <= most; use++) {
        got = count(syms - use, len + 1, (left - use) << 1);
        sum += got;
        if (got == -1 || sum < got)         /* overflow */
            return -1;
    }

    /* verify that all recursive calls are productive */
    assert(sum != 0);

    /* save the result and return it */
    num[index] = sum;
    return sum;
}

local int beenhere(int syms, int len, int left, int mem, int rem)
{
    size_t index;       /* index for this state's bit vector */
    size_t offset;      /* offset in this state's bit vector */
    int bit;            /* mask for this state's bit */
    size_t length;      /* length of the bit vector in bytes */
    char *vector;       /* new or enlarged bit vector */

    /* point to vector for (syms,left,len), bit in vector for (mem,rem) */
    index = INDEX(syms, left, len);
    mem -= 1 << root;
    offset = (mem >> 3) + rem;
    offset = ((offset * (offset + 1)) >> 1) + rem;
    bit = 1 << (mem & 7);

    /* see if we've been here */
    length = done[index].len;
    if (offset < length && (done[index].vec[offset] & bit) != 0)
        return 1;       /* done this! */

    /* we haven't been here before -- set the bit to show we have now */

    /* see if we need to lengthen the vector in order to set the bit */
    if (length <= offset) {
        /* if we have one already, enlarge it, zero out the appended space */
        if (length) {
            do {
                length <<= 1;
            } while (length <= offset);
            vector = realloc(done[index].vec, length);
            if (vector != NULL)
                memset(vector + done[index].len, 0, length - done[index].len);
        }

        /* otherwise we need to make a new vector and zero it out */
        else {
            length = 1 << (len - root);
            while (length <= offset)
                length <<= 1;
            vector = calloc(length, sizeof(char));
        }

        /* in either case, bail if we can't get the memory */
        if (vector == NULL) {
            fputs("abort: unable to allocate enough memory\n", stderr);
            cleanup();
            exit(1);
        }

        /* install the new vector */
        done[index].len = length;
        done[index].vec = vector;
    }

    /* set the bit */
    done[index].vec[offset] |= bit;
    return 0;
}

local void examine(int syms, int len, int left, int mem, int rem)
{
    int least;          /* least number of syms to use at this juncture */
    int most;           /* most number of syms to use at this juncture */
    int use;            /* number of bit patterns to use in next call */

    /* see if we have a complete code */
    if (syms == left) {
        /* set the last code entry */
        code[len] = left;

        /* complete computation of memory used by this code */
        while (rem < left) {
            left -= rem;
            rem = 1 << (len - root);
            mem += rem;
        }
        assert(rem == left);

        /* if this is a new maximum, show the entries used and the sub-code */
        if (mem > large) {
            large = mem;
            printf("max %d: ", mem);
            for (use = root + 1; use <= max; use++)
                if (code[use])
                    printf("%d[%d] ", code[use], use);
            putchar('\n');
            fflush(stdout);
        }

        /* remove entries as we drop back down in the recursion */
        code[len] = 0;
        return;
    }

    /* prune the tree if we can */
    if (beenhere(syms, len, left, mem, rem))
        return;

    /* we need to use at least this many bit patterns so that the code won't be
       incomplete at the next length (more bit patterns than symbols) */
    least = (left << 1) - syms;
    if (least < 0)
        least = 0;

    /* we can use at most this many bit patterns, lest there not be enough
       available for the remaining symbols at the maximum length (if there were
       no limit to the code length, this would become: most = left - 1) */
    most = (((code_t)left << (max - len)) - syms) /
            (((code_t)1 << (max - len)) - 1);

    /* occupy least table spaces, creating new sub-tables as needed */
    use = least;
    while (rem < use) {
        use -= rem;
        rem = 1 << (len - root);
        mem += rem;
    }
    rem -= use;

    /* examine codes from here, updating table space as we go */
    for (use = least; use <= most; use++) {
        code[len] = use;
        examine(syms - use, len + 1, (left - use) << 1,
                mem + (rem ? 1 << (len - root) : 0), rem << 1);
        if (rem == 0) {
            rem = 1 << (len - root);
            mem += rem;
        }
        rem--;
    }

    /* remove entries as we drop back down in the recursion */
    code[len] = 0;
}

local void enough(int syms)
{
    int n;              /* number of remaing symbols for this node */
    int left;           /* number of unused bit patterns at this length */
    size_t index;       /* index of this case in *num */

    /* clear code */
    for (n = 0; n <= max; n++)
        code[n] = 0;

    /* look at all (root + 1) bit and longer codes */
    large = 1 << root;              /* base table */
    if (root < max)                 /* otherwise, there's only a base table */
        for (n = 3; n <= syms; n++)
            for (left = 2; left < n; left += 2)
            {
                /* look at all reachable (root + 1) bit nodes, and the
                   resulting codes (complete at root + 2 or more) */
                index = INDEX(n, left, root + 1);
                if (root + 1 < max && num[index])       /* reachable node */
                    examine(n, root + 1, left, 1 << root, 0);

                /* also look at root bit codes with completions at root + 1
                   bits (not saved in num, since complete), just in case */
                if (num[index - 1] && n <= left << 1)
                    examine((n - left) << 1, root + 1, (n - left) << 1,
                            1 << root, 0);
            }

    /* done */
    printf("done: maximum of %d table entries\n", large);
}

int main(int argc, char **argv)
{
    int syms;           /* total number of symbols to code */
    int n;              /* number of symbols to code for this run */
    big_t got;          /* return value of count() */
    big_t sum;          /* accumulated number of codes over n */

    /* set up globals for cleanup() */
    code = NULL;
    num = NULL;
    done = NULL;

    /* get arguments -- default to the deflate literal/length code */
    syms = 286;
        root = 9;
    max = 15;
    if (argc > 1) {
        syms = atoi(argv[1]);
        if (argc > 2) {
            root = atoi(argv[2]);
                        if (argc > 3)
                                max = atoi(argv[3]);
                }
    }
    if (argc > 4 || syms < 2 || root < 1 || max < 1) {
        fputs("invalid arguments, need: [sym >= 2 [root >= 1 [max >= 1]]]\n",
                          stderr);
        return 1;
    }

    /* if not restricting the code length, the longest is syms - 1 */
    if (max > syms - 1)
        max = syms - 1;

    /* determine the number of bits in a code_t */
    n = 0;
    while (((code_t)1 << n) != 0)
        n++;

    /* make sure that the calculation of most will not overflow */
    if (max > n || syms - 2 >= (((code_t)0 - 1) >> (max - 1))) {
        fputs("abort: code length too long for internal types\n", stderr);
        return 1;
    }

    /* reject impossible code requests */
    if (syms - 1 > ((code_t)1 << max) - 1) {
        fprintf(stderr, "%d symbols cannot be coded in %d bits\n",
                syms, max);
        return 1;
    }

    /* allocate code vector */
    code = calloc(max + 1, sizeof(int));
    if (code == NULL) {
        fputs("abort: unable to allocate enough memory\n", stderr);
        return 1;
    }

    /* determine size of saved results array, checking for overflows,
       allocate and clear the array (set all to zero with calloc()) */
    if (syms == 2)              /* iff max == 1 */
        num = NULL;             /* won't be saving any results */
    else {
        size = syms >> 1;
        if (size > ((size_t)0 - 1) / (n = (syms - 1) >> 1) ||
                (size *= n, size > ((size_t)0 - 1) / (n = max - 1)) ||
                (size *= n, size > ((size_t)0 - 1) / sizeof(big_t)) ||
                (num = calloc(size, sizeof(big_t))) == NULL) {
            fputs("abort: unable to allocate enough memory\n", stderr);
            cleanup();
            return 1;
        }
    }

    /* count possible codes for all numbers of symbols, add up counts */
    sum = 0;
    for (n = 2; n <= syms; n++) {
        got = count(n, 1, 2);
        sum += got;
        if (got == -1 || sum < got) {       /* overflow */
            fputs("abort: can't count that high!\n", stderr);
            cleanup();
            return 1;
        }
        printf("%llu %d-codes\n", got, n);
    }
    printf("%llu total codes for 2 to %d symbols", sum, syms);
    if (max < syms - 1)
        printf(" (%d-bit length limit)\n", max);
    else
        puts(" (no length limit)");

    /* allocate and clear done array for beenhere() */
    if (syms == 2)
        done = NULL;
    else if (size > ((size_t)0 - 1) / sizeof(struct tab) ||
             (done = calloc(size, sizeof(struct tab))) == NULL) {
        fputs("abort: unable to allocate enough memory\n", stderr);
        cleanup();
        return 1;
    }

    /* find and show maximum inflate table usage */
        if (root > max)                 /* reduce root to max length */
                root = max;
    if (syms < ((code_t)1 << (root + 1)))
        enough(syms);
    else
        puts("cannot handle minimum code lengths > root");

    /* done */
    cleanup();
    return 0;
}
