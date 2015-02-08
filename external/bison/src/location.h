
#ifndef LOCATION_H_
# define LOCATION_H_

# include "uniqstr.h"

/* A boundary between two characters.  */
typedef struct
{
  /* The name of the file that contains the boundary.  */
  uniqstr file;

  /* The (origin-1) line that contains the boundary.
     If this is INT_MAX, the line number has overflowed.  */
  int line;

  /* The (origin-1) column just after the boundary.  This is neither a
     byte count, nor a character count; it is a column count.
     If this is INT_MAX, the column number has overflowed.  */
  int column;

} boundary;

/* Return nonzero if A and B are equal boundaries.  */
static inline bool
equal_boundaries (boundary a, boundary b)
{
  return (a.column == b.column
	  && a.line == b.line
	  && UNIQSTR_EQ (a.file, b.file));
}

/* A location, that is, a region of source code.  */
typedef struct
{
  /* Boundary just before the location starts.  */
  boundary start;

  /* Boundary just after the location ends.  */
  boundary end;

} location;

#define YYLTYPE location

extern location const empty_location;

void location_print (FILE *out, location loc);

#endif /* ! defined LOCATION_H_ */
