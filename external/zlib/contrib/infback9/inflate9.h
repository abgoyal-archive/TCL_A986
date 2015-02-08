


/* Possible inflate modes between inflate() calls */
typedef enum {
        TYPE,       /* i: waiting for type bits, including last-flag bit */
        STORED,     /* i: waiting for stored size (length and complement) */
        TABLE,      /* i: waiting for dynamic block table lengths */
            LEN,        /* i: waiting for length/lit code */
    DONE,       /* finished check, done -- remain here until reset */
    BAD         /* got a data error -- remain here until reset */
} inflate_mode;


/* state maintained between inflate() calls.  Approximately 7K bytes. */
struct inflate_state {
        /* sliding window */
    unsigned char FAR *window;  /* allocated sliding window, if needed */
        /* dynamic table building */
    unsigned ncode;             /* number of code length code lengths */
    unsigned nlen;              /* number of length code lengths */
    unsigned ndist;             /* number of distance code lengths */
    unsigned have;              /* number of code lengths in lens[] */
    code FAR *next;             /* next available space in codes[] */
    unsigned short lens[320];   /* temporary storage for code lengths */
    unsigned short work[288];   /* work area for code table building */
    code codes[ENOUGH];         /* space for code tables */
};
