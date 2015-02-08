


typedef struct {
    unsigned char op;           /* operation, extra bits, table bits */
    unsigned char bits;         /* bits in this part of the code */
    unsigned short val;         /* offset in table or code value */
} code;


#define ENOUGH_LENS 852
#define ENOUGH_DISTS 594
#define ENOUGH (ENOUGH_LENS+ENOUGH_DISTS)

/* Type of code to build for inflate_table9() */
typedef enum {
    CODES,
    LENS,
    DISTS
} codetype;

extern int inflate_table9 OF((codetype type, unsigned short FAR *lens,
                             unsigned codes, code FAR * FAR *table,
                             unsigned FAR *bits, unsigned short FAR *work));
