

#ifndef MPI_TYPE_H
#define MPI_TYPE_H


#ifndef MPI_POINTER
#define MPI_POINTER     *
#endif



typedef signed   char   S8;
typedef unsigned char   U8;
typedef signed   short  S16;
typedef unsigned short  U16;


typedef int32_t   S32;
typedef u_int32_t U32;

typedef struct _S64
{
    U32          Low;
    S32          High;
} S64;

typedef struct _U64
{
    U32          Low;
    U32          High;
} U64;


/****************************************************************************/
/*  Pointers                                                                */
/****************************************************************************/

typedef S8      *PS8;
typedef U8      *PU8;
typedef S16     *PS16;
typedef U16     *PU16;
typedef S32     *PS32;
typedef U32     *PU32;
typedef S64     *PS64;
typedef U64     *PU64;


#endif

