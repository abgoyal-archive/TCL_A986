
#ifndef MUSCLE_TAB_H_
# define MUSCLE_TAB_H_

# include "location.h"

void muscle_init (void);
void muscle_insert (const char *key, char *value);
char *muscle_find (const char *key);
void muscle_free (void);


/* An obstack dedicated to receive muscle keys and values.  */
extern struct obstack muscle_obstack;

#define MUSCLE_INSERT_BOOL(Key, Value)				\
{								\
  int v = Value;						\
  MUSCLE_INSERT_INT (Key, v);					\
}

#define MUSCLE_INSERT_INT(Key, Value)				\
{								\
  obstack_fgrow1 (&muscle_obstack, "%d", Value);		\
  obstack_1grow (&muscle_obstack, 0);				\
  muscle_insert (Key, obstack_finish (&muscle_obstack));	\
}

#define MUSCLE_INSERT_LONG_INT(Key, Value)			\
{								\
  obstack_fgrow1 (&muscle_obstack, "%ld", Value);		\
  obstack_1grow (&muscle_obstack, 0);				\
  muscle_insert (Key, obstack_finish (&muscle_obstack));	\
}

#define MUSCLE_INSERT_STRING(Key, Value)			\
{								\
  obstack_sgrow (&muscle_obstack, Value);			\
  obstack_1grow (&muscle_obstack, 0);				\
  muscle_insert (Key, obstack_finish (&muscle_obstack));	\
}

#define MUSCLE_OBSTACK_SGROW(Obstack, Value)			\
{								\
  char const *p;						\
  for (p = Value; *p; p++)					\
    switch (*p)							\
      {								\
      case '$':	obstack_sgrow (Obstack, "$]["); break;		\
      case '@':	obstack_sgrow (Obstack, "@@" ); break;		\
      case '[':	obstack_sgrow (Obstack, "@{" ); break;		\
      case ']':	obstack_sgrow (Obstack, "@}" ); break;		\
      default: obstack_1grow (Obstack, *p); break;		\
      }								\
}

#define MUSCLE_INSERT_C_STRING(Key, Value)			\
{								\
  MUSCLE_OBSTACK_SGROW (&muscle_obstack,			\
			quotearg_style (c_quoting_style,	\
					Value));		\
  obstack_1grow (&muscle_obstack, 0);				\
  muscle_insert (Key, obstack_finish (&muscle_obstack));	\
}


void muscle_grow (const char *key, const char *value, const char *separator);



void muscle_code_grow (const char *key, const char *value, location loc);



void muscle_pair_list_grow (const char *muscle,
			    const char *a1, const char *a2);

void muscles_m4_output (FILE *out);

#endif /* not MUSCLE_TAB_H_ */
