
/* Written by Jim Meyering.  */

#ifndef UNLOCKED_IO_H
# define UNLOCKED_IO_H 1


# include <stdio.h>

# if HAVE_DECL_CLEARERR_UNLOCKED
#  undef clearerr
#  define clearerr(x) clearerr_unlocked (x)
# else
#  define clearerr_unlocked(x) clearerr (x)
# endif

# if HAVE_DECL_FEOF_UNLOCKED
#  undef feof
#  define feof(x) feof_unlocked (x)
# else
#  define feof_unlocked(x) feof (x)
# endif

# if HAVE_DECL_FERROR_UNLOCKED
#  undef ferror
#  define ferror(x) ferror_unlocked (x)
# else
#  define ferror_unlocked(x) ferror (x)
# endif

# if HAVE_DECL_FFLUSH_UNLOCKED
#  undef fflush
#  define fflush(x) fflush_unlocked (x)
# else
#  define fflush_unlocked(x) fflush (x)
# endif

# if HAVE_DECL_FGETS_UNLOCKED
#  undef fgets
#  define fgets(x,y,z) fgets_unlocked (x,y,z)
# else
#  define fgets_unlocked(x,y,z) fgets (x,y,z)
# endif

# if HAVE_DECL_FPUTC_UNLOCKED
#  undef fputc
#  define fputc(x,y) fputc_unlocked (x,y)
# else
#  define fputc_unlocked(x,y) fputc (x,y)
# endif

# if HAVE_DECL_FPUTS_UNLOCKED
#  undef fputs
#  define fputs(x,y) fputs_unlocked (x,y)
# else
#  define fputs_unlocked(x,y) fputs (x,y)
# endif

# if HAVE_DECL_FREAD_UNLOCKED
#  undef fread
#  define fread(w,x,y,z) fread_unlocked (w,x,y,z)
# else
#  define fread_unlocked(w,x,y,z) fread (w,x,y,z)
# endif

# if HAVE_DECL_FWRITE_UNLOCKED
#  undef fwrite
#  define fwrite(w,x,y,z) fwrite_unlocked (w,x,y,z)
# else
#  define fwrite_unlocked(w,x,y,z) fwrite (w,x,y,z)
# endif

# if HAVE_DECL_GETC_UNLOCKED
#  undef getc
#  define getc(x) getc_unlocked (x)
# else
#  define getc_unlocked(x) getc (x)
# endif

# if HAVE_DECL_GETCHAR_UNLOCKED
#  undef getchar
#  define getchar() getchar_unlocked ()
# else
#  define getchar_unlocked() getchar ()
# endif

# if HAVE_DECL_PUTC_UNLOCKED
#  undef putc
#  define putc(x,y) putc_unlocked (x,y)
# else
#  define putc_unlocked(x,y) putc (x,y)
# endif

# if HAVE_DECL_PUTCHAR_UNLOCKED
#  undef putchar
#  define putchar(x) putchar_unlocked (x)
# else
#  define putchar_unlocked(x) putchar (x)
# endif

# undef flockfile
# define flockfile(x) ((void) 0)

# undef ftrylockfile
# define ftrylockfile(x) 0

# undef funlockfile
# define funlockfile(x) ((void) 0)

#endif /* UNLOCKED_IO_H */
