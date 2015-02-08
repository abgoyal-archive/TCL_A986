


#ifndef __P
# if defined(__STDC__) || defined(__GNUC__)
#  define __P(x) x
# else
#  define __P(x) ()
# endif
#endif


typedef struct {
	unsigned int buffer[4]; /* Holds 4-word result of MD computation */
	unsigned char count[8]; /* Number of bits processed so far */
	unsigned int done;      /* Nonzero means MD computation finished */
} MD4_CTX;

extern void MD4Init __P((MD4_CTX *MD));

extern void MD4Update __P((MD4_CTX *MD, unsigned char *X, unsigned int count));

extern void MD4Print __P((MD4_CTX *));

extern void MD4Final __P((unsigned char *, MD4_CTX *));

