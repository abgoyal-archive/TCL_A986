

#include <linux/kernel.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

#define memzero(s,n)	memset ((s),0,(n))
#define puts		srm_printk
extern long srm_printk(const char *, ...)
     __attribute__ ((format (printf, 1, 2)));

#define OF(args)  args
#define STATIC static

typedef unsigned char  uch;
typedef unsigned short ush;
typedef unsigned long  ulg;

#define WSIZE 0x8000		/* Window size must be at least 32k, */
				/* and a power of two */

static uch *inbuf;		/* input buffer */
static uch *window;		/* Sliding window buffer */

static unsigned insize;		/* valid bytes in inbuf */
static unsigned inptr;		/* index of next byte to be processed in inbuf */
static unsigned outcnt;		/* bytes in output buffer */

/* gzip flag byte */
#define ASCII_FLAG   0x01 /* bit 0 set: file probably ascii text */
#define CONTINUATION 0x02 /* bit 1 set: continuation of multi-part gzip file */
#define EXTRA_FIELD  0x04 /* bit 2 set: extra field present */
#define ORIG_NAME    0x08 /* bit 3 set: original file name present */
#define COMMENT      0x10 /* bit 4 set: file comment present */
#define ENCRYPTED    0x20 /* bit 5 set: file is encrypted */
#define RESERVED     0xC0 /* bit 6,7:   reserved */

#define get_byte()  (inptr < insize ? inbuf[inptr++] : fill_inbuf())

/* Diagnostic functions */
#ifdef DEBUG
#  define Assert(cond,msg) {if(!(cond)) error(msg);}
#  define Trace(x) fprintf x
#  define Tracev(x) {if (verbose) fprintf x ;}
#  define Tracevv(x) {if (verbose>1) fprintf x ;}
#  define Tracec(c,x) {if (verbose && (c)) fprintf x ;}
#  define Tracecv(c,x) {if (verbose>1 && (c)) fprintf x ;}
#else
#  define Assert(cond,msg)
#  define Trace(x)
#  define Tracev(x)
#  define Tracevv(x)
#  define Tracec(c,x)
#  define Tracecv(c,x)
#endif

static int  fill_inbuf(void);
static void flush_window(void);
static void error(char *m);

static char *input_data;
static int  input_data_size;

static uch *output_data;
static ulg output_ptr;
static ulg bytes_out;

static void error(char *m);
static void gzip_mark(void **);
static void gzip_release(void **);

extern int end;
static ulg free_mem_ptr;
static ulg free_mem_end_ptr;

#define HEAP_SIZE 0x3000

#include "../../../lib/inflate.c"

int fill_inbuf(void)
{
	if (insize != 0)
		error("ran out of input data");

	inbuf = input_data;
	insize = input_data_size;

	inptr = 1;
	return inbuf[0];
}

void flush_window(void)
{
	ulg c = crc;
	unsigned n;
	uch *in, *out, ch;

	in = window;
	out = &output_data[output_ptr];
	for (n = 0; n < outcnt; n++) {
		ch = *out++ = *in++;
		c = crc_32_tab[((int)c ^ ch) & 0xff] ^ (c >> 8);
	}
	crc = c;
	bytes_out += (ulg)outcnt;
	output_ptr += (ulg)outcnt;
	outcnt = 0;
/*	puts("."); */
}

static void error(char *x)
{
	puts("\n\n");
	puts(x);
	puts("\n\n -- System halted");

	while(1);	/* Halt */
}

unsigned int
decompress_kernel(void *output_start,
		  void *input_start,
		  size_t ksize,
		  size_t kzsize)
{
	output_data		= (uch *)output_start;
	input_data		= (uch *)input_start;
	input_data_size		= kzsize; /* use compressed size */

	/* FIXME FIXME FIXME */
	free_mem_ptr		= (ulg)output_start + ksize;
	free_mem_end_ptr	= (ulg)output_start + ksize + 0x200000;
	/* FIXME FIXME FIXME */

	/* put in temp area to reduce initial footprint */
	window = malloc(WSIZE);

	makecrc();
/*	puts("Uncompressing Linux..."); */
	gunzip();
/*	puts(" done, booting the kernel.\n"); */
	return output_ptr;
}
