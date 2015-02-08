
#include <config.h>
#include "system.h"

#include <hash.h>
#include <quotearg.h>

#include "files.h"
#include "muscle_tab.h"
#include "getargs.h"

typedef struct
{
  const char *key;
  char *value;
} muscle_entry;

/* An obstack used to create some entries.  */
struct obstack muscle_obstack;

/* Initial capacity of muscles hash table.  */
#define HT_INITIAL_CAPACITY 257

static struct hash_table *muscle_table = NULL;

static bool
hash_compare_muscles (void const *x, void const *y)
{
  muscle_entry const *m1 = x;
  muscle_entry const *m2 = y;
  return strcmp (m1->key, m2->key) == 0;
}

static size_t
hash_muscle (const void *x, size_t tablesize)
{
  muscle_entry const *m = x;
  return hash_string (m->key, tablesize);
}


void
muscle_init (void)
{
  /* Initialize the muscle obstack.  */
  obstack_init (&muscle_obstack);

  muscle_table = hash_initialize (HT_INITIAL_CAPACITY, NULL, hash_muscle,
				  hash_compare_muscles, free);

  /* Version and input file.  */
  MUSCLE_INSERT_STRING ("version", VERSION);
  MUSCLE_INSERT_C_STRING ("file_name", grammar_file);
}



void
muscle_free (void)
{
  hash_free (muscle_table);
  obstack_free (&muscle_obstack, NULL);
}




void
muscle_insert (const char *key, char *value)
{
  muscle_entry probe;
  muscle_entry *entry;

  probe.key = key;
  entry = hash_lookup (muscle_table, &probe);

  if (!entry)
    {
      /* First insertion in the hash. */
      entry = xmalloc (sizeof *entry);
      entry->key = key;
      hash_insert (muscle_table, entry);
    }
  entry->value = value;
}



void
muscle_grow (const char *key, const char *val, const char *separator)
{
  muscle_entry probe;
  muscle_entry *entry = NULL;

  probe.key = key;
  entry = hash_lookup (muscle_table, &probe);

  if (!entry)
    {
      /* First insertion in the hash. */
      entry = xmalloc (sizeof *entry);
      entry->key = key;
      hash_insert (muscle_table, entry);
      entry->value = xstrdup (val);
    }
  else
    {
      /* Grow the current value. */
      char *new_val;
      obstack_sgrow (&muscle_obstack, entry->value);
      free (entry->value);
      obstack_sgrow (&muscle_obstack, separator);
      obstack_sgrow (&muscle_obstack, val);
      obstack_1grow (&muscle_obstack, 0);
      new_val = obstack_finish (&muscle_obstack);
      entry->value = xstrdup (new_val);
      obstack_free (&muscle_obstack, new_val);
    }
}



void
muscle_code_grow (const char *key, const char *val, location loc)
{
  char *extension = NULL;
  obstack_fgrow1 (&muscle_obstack, "]b4_syncline(%d, [[", loc.start.line);
  MUSCLE_OBSTACK_SGROW (&muscle_obstack,
			quotearg_style (c_quoting_style, loc.start.file));
  obstack_sgrow (&muscle_obstack, "]])[\n");
  obstack_sgrow (&muscle_obstack, val);
  obstack_1grow (&muscle_obstack, 0);
  extension = obstack_finish (&muscle_obstack);
  muscle_grow (key, extension, "");
}



void muscle_pair_list_grow (const char *muscle,
			    const char *a1, const char *a2)
{
  char *pair;
  obstack_fgrow2 (&muscle_obstack, "[[[%s]], [[%s]]]", a1, a2);
  obstack_1grow (&muscle_obstack, 0);
  pair = obstack_finish (&muscle_obstack);
  muscle_grow (muscle, pair, ",\n");
  obstack_free (&muscle_obstack, pair);
}


char *
muscle_find (const char *key)
{
  muscle_entry probe;
  muscle_entry *result = NULL;

  probe.key = key;
  result = hash_lookup (muscle_table, &probe);
  return result ? result->value : NULL;
}



static inline bool
muscle_m4_output (muscle_entry *entry, FILE *out)
{
  fprintf (out, "m4_define([b4_%s],\n", entry->key);
  fprintf (out, "[[%s]])\n\n\n", entry->value);
  return true;
}

static bool
muscle_m4_output_processor (void *entry, void *out)
{
  return muscle_m4_output (entry, out);
}



void
muscles_m4_output (FILE *out)
{
  hash_do_for_each (muscle_table, muscle_m4_output_processor, out);
}
