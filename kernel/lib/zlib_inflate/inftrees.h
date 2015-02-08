
#ifndef INFTREES_H
#define INFTREES_H



typedef struct {
    unsigned char op;           /* operation, extra bits, table bits */
    unsigned char bits;         /* bits in this part of the code */
    unsigned short val;         /* offset in table or code value */
} code;


#define ENOUGH 2048
#define MAXD 592

/* Type of code to build for inftable() */
typedef enum {
    CODES,
    LENS,
    DISTS
} codetype;

extern int zlib_inflate_table (codetype type, unsigned short *lens,
                             unsigned codes, code **table,
                             unsigned *bits, unsigned short *work);
#endif
