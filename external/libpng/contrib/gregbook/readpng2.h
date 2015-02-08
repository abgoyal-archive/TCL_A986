

#ifndef TRUE
#  define TRUE 1
#  define FALSE 0
#endif

#ifndef MAX
#  define MAX(a,b)  ((a) > (b)? (a) : (b))
#  define MIN(a,b)  ((a) < (b)? (a) : (b))
#endif

#ifdef DEBUG
#  define Trace(x)  {fprintf x ; fflush(stderr); fflush(stdout);}
#else
#  define Trace(x)  ;
#endif

enum rpng2_states {
    kPreInit = 0,
    kWindowInit,
    kDone
};

typedef unsigned char   uch;
typedef unsigned short  ush;
typedef unsigned long   ulg;

typedef struct _mainprog_info {
    double display_exponent;
    ulg width;
    ulg height;
    void *png_ptr;
    void *info_ptr;
    void (*mainprog_init)(void);
    void (*mainprog_display_row)(ulg row_num);
    void (*mainprog_finish_display)(void);
    uch *image_data;
    uch **row_pointers;
    jmp_buf jmpbuf;
    int passes;              /* not used */
    int pass;
    int rowbytes;
    int channels;
    int need_bgcolor;
#if (defined(__i386__) || defined(_M_IX86) || defined(__x86_64__))
    int nommxfilters;
    int nommxcombine;
    int nommxinterlace;
#endif
    int state;
    uch bg_red;
    uch bg_green;
    uch bg_blue;
} mainprog_info;


/* prototypes for public functions in readpng2.c */

void readpng2_version_info(void);

int readpng2_check_sig(uch *sig, int num);

int readpng2_init(mainprog_info *mainprog_ptr);

int readpng2_decode_data(mainprog_info *mainprog_ptr, uch *rawbuf, ulg length);

void readpng2_cleanup(mainprog_info *mainprog_ptr);
