#ifndef _ASM_X86_UNALIGNED_H
#define _ASM_X86_UNALIGNED_H


#define get_unaligned(ptr) (*(ptr))

#define put_unaligned(val, ptr) ((void)( *(ptr) = (val) ))

#endif /* _ASM_X86_UNALIGNED_H */
