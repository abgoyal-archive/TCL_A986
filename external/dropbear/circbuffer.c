
#include "includes.h"
#include "dbutil.h"
#include "circbuffer.h"

#define MAX_CBUF_SIZE 100000000

circbuffer * cbuf_new(unsigned int size) {

	circbuffer *cbuf = NULL;

	if (size > MAX_CBUF_SIZE) {
		dropbear_exit("bad cbuf size");
	}

	cbuf = (circbuffer*)m_malloc(sizeof(circbuffer));
	cbuf->data = (unsigned char*)m_malloc(size);
	cbuf->used = 0;
	cbuf->readpos = 0;
	cbuf->writepos = 0;
	cbuf->size = size;

	return cbuf;
}

void cbuf_free(circbuffer * cbuf) {

	m_free(cbuf->data);
	m_free(cbuf);
}

unsigned int cbuf_getused(circbuffer * cbuf) {

	return cbuf->used;

}

unsigned int cbuf_getavail(circbuffer * cbuf) {

	return cbuf->size - cbuf->used;

}

unsigned int cbuf_readlen(circbuffer *cbuf) {

	dropbear_assert(((2*cbuf->size)+cbuf->writepos-cbuf->readpos)%cbuf->size == cbuf->used%cbuf->size);
	dropbear_assert(((2*cbuf->size)+cbuf->readpos-cbuf->writepos)%cbuf->size == (cbuf->size-cbuf->used)%cbuf->size);

	if (cbuf->used == 0) {
		TRACE(("cbuf_readlen: unused buffer"))
		return 0;
	}

	if (cbuf->readpos < cbuf->writepos) {
		return cbuf->writepos - cbuf->readpos;
	}

	return cbuf->size - cbuf->readpos;
}

unsigned int cbuf_writelen(circbuffer *cbuf) {

	dropbear_assert(cbuf->used <= cbuf->size);
	dropbear_assert(((2*cbuf->size)+cbuf->writepos-cbuf->readpos)%cbuf->size == cbuf->used%cbuf->size);
	dropbear_assert(((2*cbuf->size)+cbuf->readpos-cbuf->writepos)%cbuf->size == (cbuf->size-cbuf->used)%cbuf->size);

	if (cbuf->used == cbuf->size) {
		TRACE(("cbuf_writelen: full buffer"))
		return 0; /* full */
	}
	
	if (cbuf->writepos < cbuf->readpos) {
		return cbuf->readpos - cbuf->writepos;
	}

	return cbuf->size - cbuf->writepos;
}

unsigned char* cbuf_readptr(circbuffer *cbuf, unsigned int len) {
	if (len > cbuf_readlen(cbuf)) {
		dropbear_exit("bad cbuf read");
	}

	return &cbuf->data[cbuf->readpos];
}

unsigned char* cbuf_writeptr(circbuffer *cbuf, unsigned int len) {

	if (len > cbuf_writelen(cbuf)) {
		dropbear_exit("bad cbuf write");
	}

	return &cbuf->data[cbuf->writepos];
}

void cbuf_incrwrite(circbuffer *cbuf, unsigned int len) {
	if (len > cbuf_writelen(cbuf)) {
		dropbear_exit("bad cbuf write");
	}

	cbuf->used += len;
	dropbear_assert(cbuf->used <= cbuf->size);
	cbuf->writepos = (cbuf->writepos + len) % cbuf->size;
}


void cbuf_incrread(circbuffer *cbuf, unsigned int len) {
	if (len > cbuf_readlen(cbuf)) {
		dropbear_exit("bad cbuf read");
	}

	dropbear_assert(cbuf->used >= len);
	cbuf->used -= len;
	cbuf->readpos = (cbuf->readpos + len) % cbuf->size;
}
