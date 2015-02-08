
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-mempool.h"
#include "dbus-internals.h"



typedef struct DBusFreedElement DBusFreedElement;

struct DBusFreedElement
{
  DBusFreedElement *next; /**< next element of the free list */
};

#define ELEMENT_PADDING 4

typedef struct DBusMemBlock DBusMemBlock;

struct DBusMemBlock
{
  DBusMemBlock *next;  /**< next block in the list, which is already used up;
                        *   only saved so we can free all the blocks
                        *   when we free the mem pool.
                        */

  /* this is a long so that "elements" is aligned */
  long used_so_far;     /**< bytes of this block already allocated as elements. */
  
  unsigned char elements[ELEMENT_PADDING]; /**< the block data, actually allocated to required size */
};

struct DBusMemPool
{
  int element_size;                /**< size of a single object in the pool */
  int block_size;                  /**< size of most recently allocated block */
  unsigned int zero_elements : 1;  /**< whether to zero-init allocated elements */

  DBusFreedElement *free_elements; /**< a free list of elements to recycle */
  DBusMemBlock *blocks;            /**< blocks of memory from malloc() */
  int allocated_elements;          /**< Count of outstanding allocated elements */
};

/** @} */



DBusMemPool*
_dbus_mem_pool_new (int element_size,
                    dbus_bool_t zero_elements)
{
  DBusMemPool *pool;

  pool = dbus_new0 (DBusMemPool, 1);
  if (pool == NULL)
    return NULL;

  /* Make the element size at least 8 bytes. */
  if (element_size < 8)
    element_size = 8;
  
  /* these assertions are equivalent but the first is more clear
   * to programmers that see it fail.
   */
  _dbus_assert (element_size >= (int) sizeof (void*));
  _dbus_assert (element_size >= (int) sizeof (DBusFreedElement));

  /* align the element size to a pointer boundary so we won't get bus
   * errors under other architectures.  
   */
  pool->element_size = _DBUS_ALIGN_VALUE (element_size, sizeof (void *));

  pool->zero_elements = zero_elements != FALSE;

  pool->allocated_elements = 0;
  
  /* pick a size for the first block; it increases
   * for each block we need to allocate. This is
   * actually half the initial block size
   * since _dbus_mem_pool_alloc() unconditionally
   * doubles it prior to creating a new block.  */
  pool->block_size = pool->element_size * 8;

  _dbus_assert ((pool->block_size %
                 pool->element_size) == 0);
  
  return pool;
}

void
_dbus_mem_pool_free (DBusMemPool *pool)
{
  DBusMemBlock *block;

  block = pool->blocks;
  while (block != NULL)
    {
      DBusMemBlock *next = block->next;

      dbus_free (block);

      block = next;
    }

  dbus_free (pool);
}

void*
_dbus_mem_pool_alloc (DBusMemPool *pool)
{
#ifdef DBUS_BUILD_TESTS
  if (_dbus_disable_mem_pools ())
    {
      DBusMemBlock *block;
      int alloc_size;
      
      /* This is obviously really silly, but it's
       * debug-mode-only code that is compiled out
       * when tests are disabled (_dbus_disable_mem_pools()
       * is a constant expression FALSE so this block
       * should vanish)
       */
      
      alloc_size = sizeof (DBusMemBlock) - ELEMENT_PADDING +
        pool->element_size;
      
      if (pool->zero_elements)
        block = dbus_malloc0 (alloc_size);
      else
        block = dbus_malloc (alloc_size);

      if (block != NULL)
        {
          block->next = pool->blocks;
          pool->blocks = block;
          pool->allocated_elements += 1;

          return (void*) &block->elements[0];
        }
      else
        return NULL;
    }
  else
#endif
    {
      if (_dbus_decrement_fail_alloc_counter ())
        {
          _dbus_verbose (" FAILING mempool alloc\n");
          return NULL;
        }
      else if (pool->free_elements)
        {
          DBusFreedElement *element = pool->free_elements;

          pool->free_elements = pool->free_elements->next;

          if (pool->zero_elements)
            memset (element, '\0', pool->element_size);

          pool->allocated_elements += 1;
          
          return element;
        }
      else
        {
          void *element;
      
          if (pool->blocks == NULL ||
              pool->blocks->used_so_far == pool->block_size)
            {
              /* Need a new block */
              DBusMemBlock *block;
              int alloc_size;
#ifdef DBUS_BUILD_TESTS
              int saved_counter;
#endif
          
              if (pool->block_size <= _DBUS_INT_MAX / 4) /* avoid overflow */
                {
                  /* use a larger block size for our next block */
                  pool->block_size *= 2;
                  _dbus_assert ((pool->block_size %
                                 pool->element_size) == 0);
                }

              alloc_size = sizeof (DBusMemBlock) - ELEMENT_PADDING + pool->block_size;

#ifdef DBUS_BUILD_TESTS
              /* We save/restore the counter, so that memory pools won't
               * cause a given function to have different number of
               * allocations on different invocations. i.e.  when testing
               * we want consistent alloc patterns. So we skip our
               * malloc here for purposes of failed alloc simulation.
               */
              saved_counter = _dbus_get_fail_alloc_counter ();
              _dbus_set_fail_alloc_counter (_DBUS_INT_MAX);
#endif
          
              if (pool->zero_elements)
                block = dbus_malloc0 (alloc_size);
              else
                block = dbus_malloc (alloc_size);

#ifdef DBUS_BUILD_TESTS
              _dbus_set_fail_alloc_counter (saved_counter);
              _dbus_assert (saved_counter == _dbus_get_fail_alloc_counter ());
#endif
          
              if (block == NULL)
                return NULL;

              block->used_so_far = 0;
              block->next = pool->blocks;
              pool->blocks = block;          
            }
      
          element = &pool->blocks->elements[pool->blocks->used_so_far];
          
          pool->blocks->used_so_far += pool->element_size;

          pool->allocated_elements += 1;
          
          return element;
        }
    }
}

dbus_bool_t
_dbus_mem_pool_dealloc (DBusMemPool *pool,
                        void        *element)
{
#ifdef DBUS_BUILD_TESTS
  if (_dbus_disable_mem_pools ())
    {
      DBusMemBlock *block;
      DBusMemBlock *prev;

      /* mmm, fast. ;-) debug-only code, so doesn't matter. */
      
      prev = NULL;
      block = pool->blocks;

      while (block != NULL)
        {
          if (block->elements == (unsigned char*) element)
            {
              if (prev)
                prev->next = block->next;
              else
                pool->blocks = block->next;
              
              dbus_free (block);

              _dbus_assert (pool->allocated_elements > 0);
              pool->allocated_elements -= 1;
              
              if (pool->allocated_elements == 0)
                _dbus_assert (pool->blocks == NULL);
              
              return pool->blocks == NULL;
            }
          prev = block;
          block = block->next;
        }
      
      _dbus_assert_not_reached ("freed nonexistent block");
      return FALSE;
    }
  else
#endif
    {
      DBusFreedElement *freed;
      
      freed = element;
      freed->next = pool->free_elements;
      pool->free_elements = freed;
      
      _dbus_assert (pool->allocated_elements > 0);
      pool->allocated_elements -= 1;
      
      return pool->allocated_elements == 0;
    }
}

/** @} */

#ifdef DBUS_BUILD_TESTS
#include "dbus-test.h"
#include <stdio.h>
#include <time.h>

static void
time_for_size (int size)
{
  int i;
  int j;
  clock_t start;
  clock_t end;
#define FREE_ARRAY_SIZE 512
#define N_ITERATIONS FREE_ARRAY_SIZE * 512
  void *to_free[FREE_ARRAY_SIZE];
  DBusMemPool *pool;

  _dbus_verbose ("Timings for size %d\n", size);
  
  _dbus_verbose (" malloc\n");
  
  start = clock ();
  
  i = 0;
  j = 0;
  while (i < N_ITERATIONS)
    {
      to_free[j] = dbus_malloc (size);
      _dbus_assert (to_free[j] != NULL); /* in a real app of course this is wrong */

      ++j;

      if (j == FREE_ARRAY_SIZE)
        {
          j = 0;
          while (j < FREE_ARRAY_SIZE)
            {
              dbus_free (to_free[j]);
              ++j;
            }

          j = 0;
        }
      
      ++i;
    }

  end = clock ();

  _dbus_verbose ("  created/destroyed %d elements in %g seconds\n",
                 N_ITERATIONS, (end - start) / (double) CLOCKS_PER_SEC);



  _dbus_verbose (" mempools\n");
  
  start = clock ();

  pool = _dbus_mem_pool_new (size, FALSE);
  
  i = 0;
  j = 0;
  while (i < N_ITERATIONS)
    {
      to_free[j] = _dbus_mem_pool_alloc (pool); 
      _dbus_assert (to_free[j] != NULL);  /* in a real app of course this is wrong */

      ++j;

      if (j == FREE_ARRAY_SIZE)
        {
          j = 0;
          while (j < FREE_ARRAY_SIZE)
            {
              _dbus_mem_pool_dealloc (pool, to_free[j]);
              ++j;
            }

          j = 0;
        }
      
      ++i;
    }

  _dbus_mem_pool_free (pool);
  
  end = clock ();

  _dbus_verbose ("  created/destroyed %d elements in %g seconds\n",
                 N_ITERATIONS, (end - start) / (double) CLOCKS_PER_SEC);

  _dbus_verbose (" zeroed malloc\n");
    
  start = clock ();
  
  i = 0;
  j = 0;
  while (i < N_ITERATIONS)
    {
      to_free[j] = dbus_malloc0 (size);
      _dbus_assert (to_free[j] != NULL); /* in a real app of course this is wrong */

      ++j;

      if (j == FREE_ARRAY_SIZE)
        {
          j = 0;
          while (j < FREE_ARRAY_SIZE)
            {
              dbus_free (to_free[j]);
              ++j;
            }

          j = 0;
        }
      
      ++i;
    }

  end = clock ();

  _dbus_verbose ("  created/destroyed %d elements in %g seconds\n",
                 N_ITERATIONS, (end - start) / (double) CLOCKS_PER_SEC);
  
  _dbus_verbose (" zeroed mempools\n");
  
  start = clock ();

  pool = _dbus_mem_pool_new (size, TRUE);
  
  i = 0;
  j = 0;
  while (i < N_ITERATIONS)
    {
      to_free[j] = _dbus_mem_pool_alloc (pool); 
      _dbus_assert (to_free[j] != NULL);  /* in a real app of course this is wrong */

      ++j;

      if (j == FREE_ARRAY_SIZE)
        {
          j = 0;
          while (j < FREE_ARRAY_SIZE)
            {
              _dbus_mem_pool_dealloc (pool, to_free[j]);
              ++j;
            }

          j = 0;
        }
      
      ++i;
    }

  _dbus_mem_pool_free (pool);
  
  end = clock ();

  _dbus_verbose ("  created/destroyed %d elements in %g seconds\n",
                 N_ITERATIONS, (end - start) / (double) CLOCKS_PER_SEC);
}

dbus_bool_t
_dbus_mem_pool_test (void)
{
  int i;
  int element_sizes[] = { 4, 8, 16, 50, 124 };
  
  i = 0;
  while (i < _DBUS_N_ELEMENTS (element_sizes))
    {
      time_for_size (element_sizes[i]);
      ++i;
    }
  
  return TRUE;
}

#endif /* DBUS_BUILD_TESTS */
