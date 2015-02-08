
#ifndef DIRNAME_H_
# define DIRNAME_H_ 1

# include <stdbool.h>
# include <stddef.h>

# ifndef DIRECTORY_SEPARATOR
#  define DIRECTORY_SEPARATOR '/'
# endif

# ifndef ISSLASH
#  define ISSLASH(C) ((C) == DIRECTORY_SEPARATOR)
# endif

# ifndef FILE_SYSTEM_PREFIX_LEN
#  define FILE_SYSTEM_PREFIX_LEN(File_name) 0
# endif

# define IS_ABSOLUTE_FILE_NAME(F) ISSLASH ((F)[FILE_SYSTEM_PREFIX_LEN (F)])
# define IS_RELATIVE_FILE_NAME(F) (! IS_ABSOLUTE_FILE_NAME (F))

char *base_name (char const *file);
char *dir_name (char const *file);
size_t base_len (char const *file);
size_t dir_len (char const *file);

bool strip_trailing_slashes (char *file);

#endif /* not DIRNAME_H_ */
