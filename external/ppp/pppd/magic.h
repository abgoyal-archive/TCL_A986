

void magic_init __P((void));	/* Initialize the magic number generator */
u_int32_t magic __P((void));	/* Returns the next magic number */

/* Fill buffer with random bytes */
void random_bytes __P((unsigned char *buf, int len));
