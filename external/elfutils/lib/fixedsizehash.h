
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <sys/cdefs.h>
#include <sys/param.h>

#include <system.h>

#define CONCAT(t1,t2) __CONCAT (t1,t2)



/* Defined separately.  */
extern size_t next_prime (size_t seed);


/* Set default values.  */
#ifndef HASHTYPE
# define HASHTYPE size_t
#endif

#ifndef CLASS
# define CLASS
#endif

#ifndef PREFIX
# define PREFIX
#endif


/* The data structure.  */
struct CONCAT(PREFIX,fshash)
{
  size_t nslots;
  struct CONCAT(PREFIX,fshashent)
  {
    HASHTYPE hval;
#ifdef STORE_POINTER
# define ENTRYP(el) (el).entry
    TYPE *entry;
#else
# define ENTRYP(el) &(el).entry
    TYPE entry;
#endif
  } table[0];
};


/* Constructor for the hashing table.  */
CLASS struct CONCAT(PREFIX,fshash) *
CONCAT(PREFIX,fshash_init) (size_t nelems)
{
  struct CONCAT(PREFIX,fshash) *result;
  /* We choose a size for the hashing table 150% over the number of
     entries.  This will guarantee short medium search lengths.  */
  const size_t max_size_t = ~((size_t) 0);

  if (nelems >= (max_size_t / 3) * 2)
    {
      errno = EINVAL;
      return NULL;
    }

  /* Adjust the size to be used for the hashing table.  */
  nelems = next_prime (MAX ((nelems * 3) / 2, 10));

  /* Allocate the data structure for the result.  */
  result = (struct CONCAT(PREFIX,fshash) *)
    xcalloc (sizeof (struct CONCAT(PREFIX,fshash))
	     + (nelems + 1) * sizeof (struct CONCAT(PREFIX,fshashent)), 1);
  if (result == NULL)
    return NULL;

  result->nslots = nelems;

  return result;
}


#ifndef NO_FINI_FCT
CLASS void
CONCAT(PREFIX,fshash_fini) (struct CONCAT(PREFIX,fshash) *htab)
{
  free (htab);
}
#endif


static struct CONCAT(PREFIX,fshashent) *
CONCAT(PREFIX,fshash_lookup) (struct CONCAT(PREFIX,fshash) *htab,
			      HASHTYPE hval, TYPE *data)
{
  size_t idx = 1 + hval % htab->nslots;

  if (htab->table[idx].hval != 0)
    {
      HASHTYPE hash;

      /* See whether this is the same entry.  */
      if (htab->table[idx].hval == hval
	  && COMPARE (data, ENTRYP (htab->table[idx])) == 0)
	return &htab->table[idx];

      /* Second hash function as suggested in [Knuth].  */
      hash = 1 + hval % (htab->nslots - 2);

      do
	{
	  if (idx <= hash)
	    idx = htab->nslots + idx - hash;
	  else
	    idx -= hash;

	  if (htab->table[idx].hval == hval
	      && COMPARE (data, ENTRYP(htab->table[idx])) == 0)
	    return &htab->table[idx];
	}
      while (htab->table[idx].hval != 0);
    }

  return &htab->table[idx];
}


CLASS int
__attribute__ ((unused))
CONCAT(PREFIX,fshash_insert) (struct CONCAT(PREFIX,fshash) *htab,
			      const char *str, size_t len, TYPE *data)
{
  HASHTYPE hval = HASHFCT (str, len ?: strlen (str));
  struct CONCAT(PREFIX,fshashent) *slot;

  slot = CONCAT(PREFIX,fshash_lookup) (htab, hval, data);
 if (slot->hval != 0)
    /* We don't want to overwrite the old value.  */
    return -1;

  slot->hval = hval;
#ifdef STORE_POINTER
  slot->entry = data;
#else
  slot->entry = *data;
#endif

  return 0;
}


#ifdef INSERT_HASH
CLASS int
__attribute__ ((unused))
CONCAT(PREFIX,fshash_insert_hash) (struct CONCAT(PREFIX,fshash) *htab,
				   HASHTYPE hval, TYPE *data)
{
  struct CONCAT(PREFIX,fshashent) *slot;

  slot = CONCAT(PREFIX,fshash_lookup) (htab, hval, data);
  if (slot->hval != 0)
    /* We don't want to overwrite the old value.  */
    return -1;

  slot->hval = hval;
#ifdef STORE_POINTER
  slot->entry = data;
#else
  slot->entry = *data;
#endif

  return 0;
}
#endif


CLASS int
__attribute__ ((unused))
CONCAT(PREFIX,fshash_overwrite) (struct CONCAT(PREFIX,fshash) *htab,
				 const char *str, size_t len, TYPE *data)
{
  HASHTYPE hval = HASHFCT (str, len ?: strlen (str));
  struct CONCAT(PREFIX,fshashent) *slot;

  slot = CONCAT(PREFIX,fshash_lookup) (htab, hval, data);
  slot->hval = hval;
#ifdef STORE_POINTER
  slot->entry = data;
#else
  slot->entry = *data;
#endif

  return 0;
}


const CLASS TYPE *
CONCAT(PREFIX,fshash_find) (const struct CONCAT(PREFIX,fshash) *htab,
			    const char *str, size_t len, TYPE *data)
{
  HASHTYPE hval = HASHFCT (str, len ?: strlen (str));
  struct CONCAT(PREFIX,fshashent) *slot;

  slot = CONCAT(PREFIX,fshash_lookup) ((struct CONCAT(PREFIX,fshash) *) htab,
				       hval, data);
  if (slot->hval == 0)
    /* Not found.  */
    return NULL;

  return ENTRYP(*slot);
}


/* Unset the macros we expect.  */
#undef TYPE
#undef HASHFCT
#undef HASHTYPE
#undef COMPARE
#undef CLASS
#undef PREFIX
#undef INSERT_HASH
#undef STORE_POINTER
#undef NO_FINI_FCT
