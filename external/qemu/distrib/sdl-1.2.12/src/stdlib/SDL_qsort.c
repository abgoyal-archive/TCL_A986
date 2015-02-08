
#include "SDL_config.h"

#include "SDL_stdinc.h"

#define assert(X)
#define malloc	SDL_malloc
#define free	SDL_free
#define memcpy	SDL_memcpy
#define memmove	SDL_memmove
#define qsort	SDL_qsort


#ifndef HAVE_QSORT

static char _ID[]="<qsort.c gjm 1.12 1998-03-19>";

#define WORD_BYTES sizeof(int)

#define STACK_SIZE (8*sizeof(size_t))

#define TRUNC_nonaligned	12
#define TRUNC_aligned		12
#define TRUNC_words		12*WORD_BYTES	/* nb different meaning */

#define PIVOT_THRESHOLD 40

typedef struct { char * first; char * last; } stack_entry;
#define pushLeft {stack[stacktop].first=ffirst;stack[stacktop++].last=last;}
#define pushRight {stack[stacktop].first=first;stack[stacktop++].last=llast;}
#define doLeft {first=ffirst;llast=last;continue;}
#define doRight {ffirst=first;last=llast;continue;}
#define pop {if (--stacktop<0) break;\
  first=ffirst=stack[stacktop].first;\
  last=llast=stack[stacktop].last;\
  continue;}


/* The recursion logic is the same in each case: */
#define Recurse(Trunc)				\
      { size_t l=last-ffirst,r=llast-first;	\
        if (l<Trunc) {				\
          if (r>=Trunc) doRight			\
          else pop				\
        }					\
        else if (l<=r) { pushLeft; doRight }	\
        else if (r>=Trunc) { pushRight; doLeft }\
        else doLeft				\
      }

/* and so is the pivoting logic: */
#define Pivot(swapper,sz)			\
  if ((size_t)(last-first)>PIVOT_THRESHOLD*sz) mid=pivot_big(first,mid,last,sz,compare);\
  else {	\
    if (compare(first,mid)<0) {			\
      if (compare(mid,last)>0) {		\
        swapper(mid,last);			\
        if (compare(first,mid)>0) swapper(first,mid);\
      }						\
    }						\
    else {					\
      if (compare(mid,last)>0) swapper(first,last)\
      else {					\
        swapper(first,mid);			\
        if (compare(mid,last)>0) swapper(mid,last);\
      }						\
    }						\
    first+=sz; last-=sz;			\
  }

#ifdef DEBUG_QSORT
#include <stdio.h>
#endif

/* and so is the partitioning logic: */
#define Partition(swapper,sz) {			\
  int swapped=0;				\
  do {						\
    while (compare(first,pivot)<0) first+=sz;	\
    while (compare(pivot,last)<0) last-=sz;	\
    if (first<last) {				\
      swapper(first,last); swapped=1;		\
      first+=sz; last-=sz; }			\
    else if (first==last) { first+=sz; last-=sz; break; }\
  } while (first<=last);			\
  if (!swapped) pop				\
}

#define PreInsertion(swapper,limit,sz)		\
  first=base;					\
  last=first + (nmemb>limit ? limit : nmemb-1)*sz;\
  while (last!=base) {				\
    if (compare(first,last)>0) first=last;	\
    last-=sz; }					\
  if (first!=base) swapper(first,(char*)base);

/* and so is the insertion sort, in the first two cases: */
#define Insertion(swapper)			\
  last=((char*)base)+nmemb*size;		\
  for (first=((char*)base)+size;first!=last;first+=size) {	\
    char *test;					\
    /* Find the right place for |first|.	\
     * My apologies for var reuse. */		\
    for (test=first-size;compare(test,first)>0;test-=size) ;	\
    test+=size;					\
    if (test!=first) {				\
      /* Shift everything in [test,first)	\
       * up by one, and place |first|		\
       * where |test| is. */			\
      memcpy(pivot,first,size);			\
      memmove(test+size,test,first-test);	\
      memcpy(test,pivot,size);			\
    }						\
  }

#define SWAP_nonaligned(a,b) { \
  register char *aa=(a),*bb=(b); \
  register size_t sz=size; \
  do { register char t=*aa; *aa++=*bb; *bb++=t; } while (--sz); }

#define SWAP_aligned(a,b) { \
  register int *aa=(int*)(a),*bb=(int*)(b); \
  register size_t sz=size; \
  do { register int t=*aa;*aa++=*bb; *bb++=t; } while (sz-=WORD_BYTES); }

#define SWAP_words(a,b) { \
  register int t=*((int*)a); *((int*)a)=*((int*)b); *((int*)b)=t; }

/* ---------------------------------------------------------------------- */

static char * pivot_big(char *first, char *mid, char *last, size_t size,
                        int compare(const void *, const void *)) {
  size_t d=(((last-first)/size)>>3)*size;
  char *m1,*m2,*m3;
  { char *a=first, *b=first+d, *c=first+2*d;
#ifdef DEBUG_QSORT
fprintf(stderr,"< %d %d %d\n",*(int*)a,*(int*)b,*(int*)c);
#endif
    m1 = compare(a,b)<0 ?
           (compare(b,c)<0 ? b : (compare(a,c)<0 ? c : a))
         : (compare(a,c)<0 ? a : (compare(b,c)<0 ? c : b));
  }
  { char *a=mid-d, *b=mid, *c=mid+d;
#ifdef DEBUG_QSORT
fprintf(stderr,". %d %d %d\n",*(int*)a,*(int*)b,*(int*)c);
#endif
    m2 = compare(a,b)<0 ?
           (compare(b,c)<0 ? b : (compare(a,c)<0 ? c : a))
         : (compare(a,c)<0 ? a : (compare(b,c)<0 ? c : b));
  }
  { char *a=last-2*d, *b=last-d, *c=last;
#ifdef DEBUG_QSORT
fprintf(stderr,"> %d %d %d\n",*(int*)a,*(int*)b,*(int*)c);
#endif
    m3 = compare(a,b)<0 ?
           (compare(b,c)<0 ? b : (compare(a,c)<0 ? c : a))
         : (compare(a,c)<0 ? a : (compare(b,c)<0 ? c : b));
  }
#ifdef DEBUG_QSORT
fprintf(stderr,"-> %d %d %d\n",*(int*)m1,*(int*)m2,*(int*)m3);
#endif
  return compare(m1,m2)<0 ?
           (compare(m2,m3)<0 ? m2 : (compare(m1,m3)<0 ? m3 : m1))
         : (compare(m1,m3)<0 ? m1 : (compare(m2,m3)<0 ? m3 : m2));
}

/* ---------------------------------------------------------------------- */

static void qsort_nonaligned(void *base, size_t nmemb, size_t size,
           int (*compare)(const void *, const void *)) {

  stack_entry stack[STACK_SIZE];
  int stacktop=0;
  char *first,*last;
  char *pivot=malloc(size);
  size_t trunc=TRUNC_nonaligned*size;
  assert(pivot!=0);

  first=(char*)base; last=first+(nmemb-1)*size;

  if ((size_t)(last-first)>trunc) {
    char *ffirst=first, *llast=last;
    while (1) {
      /* Select pivot */
      { char * mid=first+size*((last-first)/size >> 1);
        Pivot(SWAP_nonaligned,size);
        memcpy(pivot,mid,size);
      }
      /* Partition. */
      Partition(SWAP_nonaligned,size);
      /* Prepare to recurse/iterate. */
      Recurse(trunc)
    }
  }
  PreInsertion(SWAP_nonaligned,TRUNC_nonaligned,size);
  Insertion(SWAP_nonaligned);
  free(pivot);
}

static void qsort_aligned(void *base, size_t nmemb, size_t size,
           int (*compare)(const void *, const void *)) {

  stack_entry stack[STACK_SIZE];
  int stacktop=0;
  char *first,*last;
  char *pivot=malloc(size);
  size_t trunc=TRUNC_aligned*size;
  assert(pivot!=0);

  first=(char*)base; last=first+(nmemb-1)*size;

  if ((size_t)(last-first)>trunc) {
    char *ffirst=first,*llast=last;
    while (1) {
      /* Select pivot */
      { char * mid=first+size*((last-first)/size >> 1);
        Pivot(SWAP_aligned,size);
        memcpy(pivot,mid,size);
      }
      /* Partition. */
      Partition(SWAP_aligned,size);
      /* Prepare to recurse/iterate. */
      Recurse(trunc)
    }
  }
  PreInsertion(SWAP_aligned,TRUNC_aligned,size);
  Insertion(SWAP_aligned);
  free(pivot);
}

static void qsort_words(void *base, size_t nmemb,
           int (*compare)(const void *, const void *)) {

  stack_entry stack[STACK_SIZE];
  int stacktop=0;
  char *first,*last;
  char *pivot=malloc(WORD_BYTES);
  assert(pivot!=0);

  first=(char*)base; last=first+(nmemb-1)*WORD_BYTES;

  if (last-first>TRUNC_words) {
    char *ffirst=first, *llast=last;
    while (1) {
#ifdef DEBUG_QSORT
fprintf(stderr,"Doing %d:%d: ",
        (first-(char*)base)/WORD_BYTES,
        (last-(char*)base)/WORD_BYTES);
#endif
      /* Select pivot */
      { char * mid=first+WORD_BYTES*((last-first) / (2*WORD_BYTES));
        Pivot(SWAP_words,WORD_BYTES);
        *(int*)pivot=*(int*)mid;
      }
#ifdef DEBUG_QSORT
fprintf(stderr,"pivot=%d\n",*(int*)pivot);
#endif
      /* Partition. */
      Partition(SWAP_words,WORD_BYTES);
      /* Prepare to recurse/iterate. */
      Recurse(TRUNC_words)
    }
  }
  PreInsertion(SWAP_words,(TRUNC_words/WORD_BYTES),WORD_BYTES);
  /* Now do insertion sort. */
  last=((char*)base)+nmemb*WORD_BYTES;
  for (first=((char*)base)+WORD_BYTES;first!=last;first+=WORD_BYTES) {
    /* Find the right place for |first|. My apologies for var reuse */
    int *pl=(int*)(first-WORD_BYTES),*pr=(int*)first;
    *(int*)pivot=*(int*)first;
    for (;compare(pl,pivot)>0;pr=pl,--pl) {
      *pr=*pl; }
    if (pr!=(int*)first) *pr=*(int*)pivot;
  }
  free(pivot);
}

/* ---------------------------------------------------------------------- */

void qsort(void *base, size_t nmemb, size_t size,
           int (*compare)(const void *, const void *)) {

  if (nmemb<=1) return;
  if (((uintptr_t)base|size)&(WORD_BYTES-1))
    qsort_nonaligned(base,nmemb,size,compare);
  else if (size!=WORD_BYTES)
    qsort_aligned(base,nmemb,size,compare);
  else
    qsort_words(base,nmemb,compare);
}

#endif /* !HAVE_QSORT */
