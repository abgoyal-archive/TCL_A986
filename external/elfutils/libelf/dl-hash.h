
#ifndef _DL_HASH_H
#define _DL_HASH_H	1


static inline unsigned int
__attribute__ ((__pure__))
_dl_elf_hash (const char *name)
{
  const unsigned char *iname = (const unsigned char *) name;
  unsigned int hash = (unsigned int) *iname++;
  if (*iname != '\0')
    {
      hash = (hash << 4) + (unsigned int) *iname++;
      if (*iname != '\0')
	{
	  hash = (hash << 4) + (unsigned int) *iname++;
	  if (*iname != '\0')
	    {
	      hash = (hash << 4) + (unsigned int) *iname++;
	      if (*iname != '\0')
		{
		  hash = (hash << 4) + (unsigned int) *iname++;
		  while (*iname != '\0')
		    {
		      unsigned int hi;
		      hash = (hash << 4) + (unsigned int) *iname++;
		      hi = hash & 0xf0000000;

		      /* The algorithm specified in the ELF ABI is as
			 follows:

			 if (hi != 0)
			 hash ^= hi >> 24;

			 hash &= ~hi;

			 But the following is equivalent and a lot
			 faster, especially on modern processors.  */

		      hash ^= hi;
		      hash ^= hi >> 24;
		    }
		}
	    }
	}
    }
  return hash;
}

#endif /* dl-hash.h */
