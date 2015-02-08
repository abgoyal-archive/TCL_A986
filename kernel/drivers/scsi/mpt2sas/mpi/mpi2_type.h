

#ifndef MPI2_TYPE_H
#define MPI2_TYPE_H


#ifndef MPI2_POINTER
#define MPI2_POINTER     *
#endif

/* the basic types may have already been included by mpi_type.h */
#ifndef MPI_TYPE_H

typedef u8 U8;
typedef __le16 U16;
typedef __le32 U32;
typedef __le64 U64 __attribute__((aligned(4)));


typedef U8      *PU8;
typedef U16     *PU16;
typedef U32     *PU32;
typedef U64     *PU64;

#endif

#endif

