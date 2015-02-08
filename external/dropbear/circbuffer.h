
#ifndef _CIRCBUFFER_H_
#define _CIRCBUFFER_H_
struct circbuf {

	unsigned int size;
	unsigned int readpos;
	unsigned int writepos;
	unsigned int used;
	unsigned char* data;
};

typedef struct circbuf circbuffer;

circbuffer * cbuf_new(unsigned int size);
void cbuf_free(circbuffer * cbuf);

unsigned int cbuf_getused(circbuffer * cbuf); /* how much data stored */
unsigned int cbuf_getavail(circbuffer * cbuf); /* how much we can write */
unsigned int cbuf_readlen(circbuffer *cbuf); /* max linear read len */
unsigned int cbuf_writelen(circbuffer *cbuf); /* max linear write len */

unsigned char* cbuf_readptr(circbuffer *cbuf, unsigned int len);
unsigned char* cbuf_writeptr(circbuffer *cbuf, unsigned int len);
void cbuf_incrwrite(circbuffer *cbuf, unsigned int len);
void cbuf_incrread(circbuffer *cbuf, unsigned int len);
#endif
