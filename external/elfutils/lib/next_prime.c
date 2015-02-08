
#include <stddef.h>


/* Test whether CANDIDATE is a prime.  */
static int
is_prime (size_t candidate)
{
  /* No even number and none less than 10 will be passed here.  */
  size_t divn = 3;
  size_t sq = divn * divn;

  while (sq < candidate && candidate % divn != 0)
    {
      size_t old_sq = sq;
      ++divn;
      sq += 4 * divn;
      if (sq < old_sq)
	return 1;
      ++divn;
    }

  return candidate % divn != 0;
}


/* We need primes for the table size.  */
size_t
next_prime (size_t seed)
{
  /* Make it definitely odd.  */
  seed |= 1;

  while (!is_prime (seed))
    seed += 2;

  return seed;
}
