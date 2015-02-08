
#include <config.h>
#include "system.h"

#include <error.h>
#include <hash.h>
#include <quotearg.h>

#include "uniqstr.h"


/* Initial capacity of uniqstr hash table.  */
#define HT_INITIAL_CAPACITY 257

static struct hash_table *uniqstrs_table = NULL;


uniqstr
uniqstr_new (char const *str)
{
  uniqstr res = hash_lookup (uniqstrs_table, str);
  if (!res)
    {
      /* First insertion in the hash. */
      res = xstrdup (str);
      hash_insert (uniqstrs_table, res);
    }
  return res;
}



void
uniqstr_assert (char const *str)
{
  if (!hash_lookup (uniqstrs_table, str))
    {
      error (0, 0,
	     "not a uniqstr: %s", quotearg (str));
      abort ();
    }
}



static inline bool
uniqstr_print (uniqstr ustr)
{
  fprintf (stderr, "%s\n", ustr);
  return true;
}

static bool
uniqstr_print_processor (void *ustr, void *null ATTRIBUTE_UNUSED)
{
  return uniqstr_print (ustr);
}



static bool
hash_compare_uniqstr (void const *m1, void const *m2)
{
  return strcmp (m1, m2) == 0;
}

static size_t
hash_uniqstr (void const *m, size_t tablesize)
{
  return hash_string (m, tablesize);
}


void
uniqstrs_new (void)
{
  uniqstrs_table = hash_initialize (HT_INITIAL_CAPACITY,
				    NULL,
				    hash_uniqstr,
				    hash_compare_uniqstr,
				    free);
}



static void
uniqstrs_do (Hash_processor processor, void *processor_data)
{
  hash_do_for_each (uniqstrs_table, processor, processor_data);
}



void
uniqstrs_print (void)
{
  uniqstrs_do (uniqstr_print_processor, NULL);
}



void
uniqstrs_free (void)
{
  hash_free (uniqstrs_table);
}
