


#define EN0	0	/* MODE == encrypt */
#define DE1	1	/* MODE == decrypt */

extern void deskey(unsigned char *, int);

extern void usekey(unsigned long *);

extern void cpkey(unsigned long *);

extern void des(unsigned char *, unsigned char *);

