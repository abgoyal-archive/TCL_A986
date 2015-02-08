
#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include "dirname.h"

#include <string.h>
#include "xalloc.h"

size_t
dir_len (char const *file)
{
  size_t prefix_length = FILE_SYSTEM_PREFIX_LEN (file);
  size_t length;

  /* Strip the basename and any redundant slashes before it.  */
  for (length = base_name (file) - file;  prefix_length < length;  length--)
    if (! ISSLASH (file[length - 1]))
      return length;

  /* But don't strip the only slash from "/".  */
  return prefix_length + ISSLASH (file[prefix_length]);
}


char *
dir_name (char const *file)
{
  size_t length = dir_len (file);
  bool append_dot = (length == FILE_SYSTEM_PREFIX_LEN (file));
  char *dir = xmalloc (length + append_dot + 1);
  memcpy (dir, file, length);
  if (append_dot)
    dir[length++] = '.';
  dir[length] = 0;
  return dir;
}

#ifdef TEST_DIRNAME

# define MAX_BUFF_LEN 1024
# include <stdio.h>

char *program_name;

int
main (int argc, char *argv[])
{
  char buff[MAX_BUFF_LEN + 1];

  program_name = argv[0];

  buff[MAX_BUFF_LEN] = 0;
  while (fgets (buff, MAX_BUFF_LEN, stdin) && buff[0])
    {
      char file[MAX_BUFF_LEN];
      char expected_result[MAX_BUFF_LEN];
      char const *result;
      sscanf (buff, "%s %s", file, expected_result);
      result = dir_name (file);
      if (strcmp (result, expected_result))
	printf ("%s: got %s, expected %s\n", file, result, expected_result);
    }
  return 0;
}
#endif
