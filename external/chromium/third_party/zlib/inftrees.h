

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

extern int inflate_table OF((codetype type, unsigned short FAR *lens,
                             unsigned codes, code FAR * FAR *table,
                             unsigned FAR *bits, unsigned short FAR *work));
