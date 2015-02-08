
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-list.h"
#include "dbus-mempool.h"
#include "dbus-threads-internal.h"


static DBusMemPool *list_pool;
_DBUS_DEFINE_GLOBAL_LOCK (list);


static DBusList*
alloc_link (void *data)
{
  DBusList *link;

  _DBUS_LOCK (list);

  if (list_pool == NULL)
    {      
      list_pool = _dbus_mem_pool_new (sizeof (DBusList), TRUE);

      if (list_pool == NULL)
        {
          _DBUS_UNLOCK (list);
          return NULL;
        }

      link = _dbus_mem_pool_alloc (list_pool);
      if (link == NULL)
        {
          _dbus_mem_pool_free (list_pool);
          list_pool = NULL;
          _DBUS_UNLOCK (list);
          return NULL;
        }
    }
  else
    {
      link = _dbus_mem_pool_alloc (list_pool);
    }

  if (link)
    link->data = data;
  
  _DBUS_UNLOCK (list);

  return link;
}

static void
free_link (DBusList *link)
{  
  _DBUS_LOCK (list);
  if (_dbus_mem_pool_dealloc (list_pool, link))
    {
      _dbus_mem_pool_free (list_pool);
      list_pool = NULL;
    }
  
  _DBUS_UNLOCK (list);
}

static void
link_before (DBusList **list,
             DBusList  *before_this_link,
             DBusList  *link)
{
  if (*list == NULL)
    {
      link->prev = link;
      link->next = link;
      *list = link;
    }
  else
    {      
      link->next = before_this_link;
      link->prev = before_this_link->prev;
      before_this_link->prev = link;
      link->prev->next = link;
      
      if (before_this_link == *list)
        *list = link;
    }
}

static void
link_after (DBusList **list,
            DBusList  *after_this_link,
            DBusList  *link)
{
  if (*list == NULL)
    {
      link->prev = link;
      link->next = link;
      *list = link;
    }
  else
    {
      link->prev = after_this_link;
      link->next = after_this_link->next;
      after_this_link->next = link;
      link->next->prev = link;
    }
}

/** @} */





DBusList*
_dbus_list_alloc_link (void *data)
{
  return alloc_link (data);
}

void
_dbus_list_free_link (DBusList *link)
{
  free_link (link);
}


dbus_bool_t
_dbus_list_append (DBusList **list,
                   void      *data)
{
  if (!_dbus_list_prepend (list, data))
    return FALSE;

  /* Now cycle the list forward one so the prepended node is the tail */
  *list = (*list)->next;

  return TRUE;
}

dbus_bool_t
_dbus_list_prepend (DBusList **list,
                    void      *data)
{
  DBusList *link;

  link = alloc_link (data);
  if (link == NULL)
    return FALSE;

  link_before (list, *list, link);

  return TRUE;
}

void
_dbus_list_append_link (DBusList **list,
			DBusList *link)
{
  _dbus_list_prepend_link (list, link);

  /* Now cycle the list forward one so the prepended node is the tail */
  *list = (*list)->next;
}

void
_dbus_list_prepend_link (DBusList **list,
			 DBusList *link)
{
  link_before (list, *list, link);
}

#ifdef DBUS_BUILD_TESTS
dbus_bool_t
_dbus_list_insert_before (DBusList **list,
                          DBusList  *before_this_link,
                          void      *data)
{
  DBusList *link;
  
  if (before_this_link == NULL)
    return _dbus_list_append (list, data);
  else
    {
      link = alloc_link (data);
      if (link == NULL)
        return FALSE;
  
      link_before (list, before_this_link, link);
    }
  
  return TRUE;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_list_insert_after (DBusList **list,
                         DBusList  *after_this_link,
                         void      *data)
{
  DBusList *link;  

  if (after_this_link == NULL)
    return _dbus_list_prepend (list, data);
  else
    {
      link = alloc_link (data);
      if (link == NULL)
        return FALSE;
  
      link_after (list, after_this_link, link);
    }
  
  return TRUE;
}

void
_dbus_list_insert_before_link (DBusList **list,
                               DBusList  *before_this_link,
                               DBusList  *link)
{
  if (before_this_link == NULL)
    _dbus_list_append_link (list, link);
  else
    link_before (list, before_this_link, link);
}

void
_dbus_list_insert_after_link (DBusList **list,
                              DBusList  *after_this_link,
                              DBusList  *link)
{
  if (after_this_link == NULL)
    _dbus_list_prepend_link (list, link);
  else  
    link_after (list, after_this_link, link);
}

dbus_bool_t
_dbus_list_remove (DBusList **list,
                   void      *data)
{
  DBusList *link;

  link = *list;
  while (link != NULL)
    {
      if (link->data == data)
        {
          _dbus_list_remove_link (list, link);
          return TRUE;
        }
      
      link = _dbus_list_get_next_link (list, link);
    }

  return FALSE;
}

dbus_bool_t
_dbus_list_remove_last (DBusList **list,
                        void      *data)
{
  DBusList *link;

  link = _dbus_list_find_last (list, data);
  if (link)
    {
      _dbus_list_remove_link (list, link);
      return TRUE;
    }
  else
    return FALSE;
}

DBusList*
_dbus_list_find_last (DBusList **list,
                      void      *data)
{
  DBusList *link;

  link = _dbus_list_get_last_link (list);

  while (link != NULL)
    {
      if (link->data == data)
        return link;
      
      link = _dbus_list_get_prev_link (list, link);
    }

  return NULL;
}

void
_dbus_list_unlink (DBusList **list,
                   DBusList  *link)
{
  if (link->next == link)
    {
      /* one-element list */
      *list = NULL;
    }
  else
    {      
      link->prev->next = link->next;
      link->next->prev = link->prev;
      
      if (*list == link)
        *list = link->next;
    }

  link->next = NULL;
  link->prev = NULL;
}

void
_dbus_list_remove_link (DBusList **list,
                        DBusList  *link)
{
  _dbus_list_unlink (list, link);
  free_link (link);
}

void
_dbus_list_clear (DBusList **list)
{
  DBusList *link;

  link = *list;
  while (link != NULL)
    {
      DBusList *next = _dbus_list_get_next_link (list, link);
      
      free_link (link);
      
      link = next;
    }

  *list = NULL;
}

DBusList*
_dbus_list_get_first_link (DBusList **list)
{
  return *list;
}

DBusList*
_dbus_list_get_last_link (DBusList **list)
{
  if (*list == NULL)
    return NULL;
  else
    return (*list)->prev;
}

void*
_dbus_list_get_last (DBusList **list)
{
  if (*list == NULL)
    return NULL;
  else
    return (*list)->prev->data;
}

void*
_dbus_list_get_first (DBusList **list)
{
  if (*list == NULL)
    return NULL;
  else
    return (*list)->data;
}

DBusList*
_dbus_list_pop_first_link (DBusList **list)
{
  DBusList *link;
  
  link = _dbus_list_get_first_link (list);
  if (link == NULL)
    return NULL;

  _dbus_list_unlink (list, link);

  return link;
}

void*
_dbus_list_pop_first (DBusList **list)
{
  DBusList *link;
  void *data;
  
  link = _dbus_list_get_first_link (list);
  if (link == NULL)
    return NULL;
  
  data = link->data;
  _dbus_list_remove_link (list, link);

  return data;
}

void*
_dbus_list_pop_last (DBusList **list)
{
  DBusList *link;
  void *data;
  
  link = _dbus_list_get_last_link (list);
  if (link == NULL)
    return NULL;
  
  data = link->data;
  _dbus_list_remove_link (list, link);

  return data;
}

#ifdef DBUS_BUILD_TESTS
DBusList*
_dbus_list_pop_last_link (DBusList **list)
{
  DBusList *link;
  
  link = _dbus_list_get_last_link (list);
  if (link == NULL)
    return NULL;

  _dbus_list_unlink (list, link);

  return link;
}
#endif /* DBUS_BUILD_TESTS */

dbus_bool_t
_dbus_list_copy (DBusList **list,
                 DBusList **dest)
{
  DBusList *link;

  _dbus_assert (list != dest);

  *dest = NULL;
  
  link = *list;
  while (link != NULL)
    {
      if (!_dbus_list_append (dest, link->data))
        {
          /* free what we have so far */
          _dbus_list_clear (dest);
          return FALSE;
        }
      
      link = _dbus_list_get_next_link (list, link);
    }

  return TRUE;
}

int
_dbus_list_get_length (DBusList **list)
{
  DBusList *link;
  int length;

  length = 0;
  
  link = *list;
  while (link != NULL)
    {
      ++length;
      
      link = _dbus_list_get_next_link (list, link);
    }

  return length;
}

void
_dbus_list_foreach (DBusList          **list,
                    DBusForeachFunction function,
                    void               *data)
{
  DBusList *link;

  link = *list;
  while (link != NULL)
    {
      DBusList *next = _dbus_list_get_next_link (list, link);
      
      (* function) (link->data, data);
      
      link = next;
    }
}

dbus_bool_t
_dbus_list_length_is_one (DBusList **list)
{
  return (*list != NULL &&
          (*list)->next == *list);
}

/** @} */

#ifdef DBUS_BUILD_TESTS
#include "dbus-test.h"
#include <stdio.h>

static void
verify_list (DBusList **list)
{
  DBusList *link;
  int length;
  
  link = *list;

  if (link == NULL)
    return;

  if (link->next == link)
    {
      _dbus_assert (link->prev == link);
      _dbus_assert (*list == link);
      return;
    }

  length = 0;
  do
    {
      length += 1;
      _dbus_assert (link->prev->next == link);
      _dbus_assert (link->next->prev == link);
      link = link->next;
    }
  while (link != *list);

  _dbus_assert (length == _dbus_list_get_length (list));

  if (length == 1)
    _dbus_assert (_dbus_list_length_is_one (list));
  else
    _dbus_assert (!_dbus_list_length_is_one (list));
}

static dbus_bool_t
is_ascending_sequence (DBusList **list)
{
  DBusList *link;
  int prev;

  prev = _DBUS_INT_MIN;
  
  link = _dbus_list_get_first_link (list);
  while (link != NULL)
    {
      int v = _DBUS_POINTER_TO_INT (link->data);
      
      if (v <= prev)
        return FALSE;

      prev = v;
      
      link = _dbus_list_get_next_link (list, link);
    }

  return TRUE;
}

static dbus_bool_t
is_descending_sequence (DBusList **list)
{
  DBusList *link;
  int prev;

  prev = _DBUS_INT_MAX;
  
  link = _dbus_list_get_first_link (list);
  while (link != NULL)
    {
      int v = _DBUS_POINTER_TO_INT (link->data);

      if (v >= prev)
        return FALSE;

      prev = v;
      
      link = _dbus_list_get_next_link (list, link);
    }

  return TRUE;
}

static dbus_bool_t
all_even_values (DBusList **list)
{
  DBusList *link;
  
  link = _dbus_list_get_first_link (list);
  while (link != NULL)
    {
      int v = _DBUS_POINTER_TO_INT (link->data);

      if ((v % 2) != 0)
        return FALSE;
      
      link = _dbus_list_get_next_link (list, link);
    }

  return TRUE;
}

static dbus_bool_t
all_odd_values (DBusList **list)
{
  DBusList *link;
  
  link = _dbus_list_get_first_link (list);
  while (link != NULL)
    {
      int v = _DBUS_POINTER_TO_INT (link->data);

      if ((v % 2) == 0)
        return FALSE;
      
      link = _dbus_list_get_next_link (list, link);
    }

  return TRUE;
}

static dbus_bool_t
lists_equal (DBusList **list1,
             DBusList **list2)
{
  DBusList *link1;
  DBusList *link2;
  
  link1 = _dbus_list_get_first_link (list1);
  link2 = _dbus_list_get_first_link (list2);
  while (link1 && link2)
    {
      if (link1->data != link2->data)
        return FALSE;
      
      link1 = _dbus_list_get_next_link (list1, link1);
      link2 = _dbus_list_get_next_link (list2, link2);
    }

  if (link1 || link2)
    return FALSE;

  return TRUE;
}

dbus_bool_t
_dbus_list_test (void)
{
  DBusList *list1;
  DBusList *list2;
  DBusList *link1;
  DBusList *link2;
  DBusList *copy1;
  DBusList *copy2;
  int i;
  
  list1 = NULL;
  list2 = NULL;

  /* Test append and prepend */
  
  i = 0;
  while (i < 10)
    {
      if (!_dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i)))
        _dbus_assert_not_reached ("could not allocate for append");
      
      if (!_dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i)))
        _dbus_assert_not_reached ("count not allocate for prepend");
      ++i;

      verify_list (&list1);
      verify_list (&list2);
      
      _dbus_assert (_dbus_list_get_length (&list1) == i);
      _dbus_assert (_dbus_list_get_length (&list2) == i);
    }

  _dbus_assert (is_ascending_sequence (&list1));
  _dbus_assert (is_descending_sequence (&list2));

  /* Test list clear */
  _dbus_list_clear (&list1);
  _dbus_list_clear (&list2);

  verify_list (&list1);
  verify_list (&list2);

  /* Test get_first, get_last, pop_first, pop_last */
  
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;
    }

  --i;
  while (i >= 0)
    {
      void *got_data1;
      void *got_data2;
      
      void *data1;
      void *data2;

      got_data1 = _dbus_list_get_last (&list1);
      got_data2 = _dbus_list_get_first (&list2);
      
      data1 = _dbus_list_pop_last (&list1);
      data2 = _dbus_list_pop_first (&list2);

      _dbus_assert (got_data1 == data1);
      _dbus_assert (got_data2 == data2);
      
      _dbus_assert (_DBUS_POINTER_TO_INT (data1) == i);
      _dbus_assert (_DBUS_POINTER_TO_INT (data2) == i);

      verify_list (&list1);
      verify_list (&list2);

      _dbus_assert (is_ascending_sequence (&list1));
      _dbus_assert (is_descending_sequence (&list2));
      
      --i;
    }

  _dbus_assert (list1 == NULL);
  _dbus_assert (list2 == NULL);

  /* Test get_first_link, get_last_link, pop_first_link, pop_last_link */
  
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;
    }

  --i;
  while (i >= 0)
    {
      DBusList *got_link1;
      DBusList *got_link2;

      DBusList *link1;
      DBusList *link2;
      
      void *data1;
      void *data2;
      
      got_link1 = _dbus_list_get_last_link (&list1);
      got_link2 = _dbus_list_get_first_link (&list2);
      
      link1 = _dbus_list_pop_last_link (&list1);
      link2 = _dbus_list_pop_first_link (&list2);

      _dbus_assert (got_link1 == link1);
      _dbus_assert (got_link2 == link2);

      data1 = link1->data;
      data2 = link2->data;

      _dbus_list_free_link (link1);
      _dbus_list_free_link (link2);
      
      _dbus_assert (_DBUS_POINTER_TO_INT (data1) == i);
      _dbus_assert (_DBUS_POINTER_TO_INT (data2) == i);

      verify_list (&list1);
      verify_list (&list2);

      _dbus_assert (is_ascending_sequence (&list1));
      _dbus_assert (is_descending_sequence (&list2));
      
      --i;
    }

  _dbus_assert (list1 == NULL);
  _dbus_assert (list2 == NULL);
  
  /* Test iteration */
  
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;

      verify_list (&list1);
      verify_list (&list2);
      
      _dbus_assert (_dbus_list_get_length (&list1) == i);
      _dbus_assert (_dbus_list_get_length (&list2) == i);
    }

  _dbus_assert (is_ascending_sequence (&list1));
  _dbus_assert (is_descending_sequence (&list2));

  --i;
  link2 = _dbus_list_get_first_link (&list2);
  while (link2 != NULL)
    {
      verify_list (&link2); /* pretend this link is the head */
      
      _dbus_assert (_DBUS_POINTER_TO_INT (link2->data) == i);
      
      link2 = _dbus_list_get_next_link (&list2, link2);
      --i;
    }

  i = 0;
  link1 = _dbus_list_get_first_link (&list1);
  while (link1 != NULL)
    {
      verify_list (&link1); /* pretend this link is the head */
      
      _dbus_assert (_DBUS_POINTER_TO_INT (link1->data) == i);
      
      link1 = _dbus_list_get_next_link (&list1, link1);
      ++i;
    }

  --i;
  link1 = _dbus_list_get_last_link (&list1);
  while (link1 != NULL)
    {
      verify_list (&link1); /* pretend this link is the head */

      _dbus_assert (_DBUS_POINTER_TO_INT (link1->data) == i);
      
      link1 = _dbus_list_get_prev_link (&list1, link1);
      --i;
    }

  _dbus_list_clear (&list1);
  _dbus_list_clear (&list2);

  /* Test remove */
  
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;
    }

  --i;
  while (i >= 0)
    {
      if ((i % 2) == 0)
        {
          if (!_dbus_list_remove (&list1, _DBUS_INT_TO_POINTER (i)))
            _dbus_assert_not_reached ("element should have been in list");
          if (!_dbus_list_remove (&list2, _DBUS_INT_TO_POINTER (i)))
            _dbus_assert_not_reached ("element should have been in list");

          verify_list (&list1);
          verify_list (&list2);
        }
      --i;
    }

  _dbus_assert (all_odd_values (&list1));
  _dbus_assert (all_odd_values (&list2));

  _dbus_list_clear (&list1);
  _dbus_list_clear (&list2);

  /* test removing the other half of the elements */
  
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;
    }

  --i;
  while (i >= 0)
    {
      if ((i % 2) != 0)
        {
          if (!_dbus_list_remove (&list1, _DBUS_INT_TO_POINTER (i)))
            _dbus_assert_not_reached ("element should have been in list");
          if (!_dbus_list_remove (&list2, _DBUS_INT_TO_POINTER (i)))
            _dbus_assert_not_reached ("element should have been in list");

          verify_list (&list1);
          verify_list (&list2);
        }
      --i;
    }

  _dbus_assert (all_even_values (&list1));
  _dbus_assert (all_even_values (&list2));

  /* clear list using remove_link */
  while (list1 != NULL)
    {
      _dbus_list_remove_link (&list1, list1);
      verify_list (&list1);
    }
  while (list2 != NULL)
    {
      _dbus_list_remove_link (&list2, list2);
      verify_list (&list2);
    }

  /* Test remove link more generally */
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;
    }

  --i;
  link2 = _dbus_list_get_first_link (&list2);
  while (link2 != NULL)
    {
      DBusList *next = _dbus_list_get_next_link (&list2, link2);
      
      _dbus_assert (_DBUS_POINTER_TO_INT (link2->data) == i);

      if ((i % 2) == 0)
        _dbus_list_remove_link (&list2, link2);

      verify_list (&list2);
      
      link2 = next;
      --i;
    }

  _dbus_assert (all_odd_values (&list2));  
  _dbus_list_clear (&list2);
  
  i = 0;
  link1 = _dbus_list_get_first_link (&list1);
  while (link1 != NULL)
    {
      DBusList *next = _dbus_list_get_next_link (&list1, link1);

      _dbus_assert (_DBUS_POINTER_TO_INT (link1->data) == i);

      if ((i % 2) != 0)
        _dbus_list_remove_link (&list1, link1);

      verify_list (&list1);
      
      link1 = next;
      ++i;
    }

  _dbus_assert (all_even_values (&list1));
  _dbus_list_clear (&list1);

  /* Test copying a list */
  i = 0;
  while (i < 10)
    {
      _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (i));
      _dbus_list_prepend (&list2, _DBUS_INT_TO_POINTER (i));
      ++i;
    }

  /* bad pointers, because they are allowed in the copy dest */
  copy1 = _DBUS_INT_TO_POINTER (0x342234);
  copy2 = _DBUS_INT_TO_POINTER (23);
  
  _dbus_list_copy (&list1, &copy1);
  verify_list (&list1);
  verify_list (&copy1);
  _dbus_assert (lists_equal (&list1, &copy1));
  
  _dbus_list_copy (&list2, &copy2);
  verify_list (&list2);
  verify_list (&copy2);
  _dbus_assert (lists_equal (&list2, &copy2));

  /* Now test copying empty lists */
  _dbus_list_clear (&list1);
  _dbus_list_clear (&list2);
  _dbus_list_clear (&copy1);
  _dbus_list_clear (&copy2);
  
  /* bad pointers, because they are allowed in the copy dest */
  copy1 = _DBUS_INT_TO_POINTER (0x342234);
  copy2 = _DBUS_INT_TO_POINTER (23);
  
  _dbus_list_copy (&list1, &copy1);
  verify_list (&list1);
  verify_list (&copy1);
  _dbus_assert (lists_equal (&list1, &copy1));
  
  _dbus_list_copy (&list2, &copy2);
  verify_list (&list2);
  verify_list (&copy2);
  _dbus_assert (lists_equal (&list2, &copy2));

  _dbus_list_clear (&list1);
  _dbus_list_clear (&list2);
  
  /* insert_before on empty list */
  _dbus_list_insert_before (&list1, NULL,
                            _DBUS_INT_TO_POINTER (0));
  verify_list (&list1);

  /* inserting before first element */
  _dbus_list_insert_before (&list1, list1,
                            _DBUS_INT_TO_POINTER (2));
  verify_list (&list1);
  _dbus_assert (is_descending_sequence (&list1));

  /* inserting in the middle */
  _dbus_list_insert_before (&list1, list1->next,
                            _DBUS_INT_TO_POINTER (1));
  verify_list (&list1);
  _dbus_assert (is_descending_sequence (&list1));  

  /* using insert_before to append */
  _dbus_list_insert_before (&list1, NULL,
                            _DBUS_INT_TO_POINTER (-1));
  verify_list (&list1);
  _dbus_assert (is_descending_sequence (&list1));
  
  _dbus_list_clear (&list1);

  /* insert_after on empty list */
  _dbus_list_insert_after (&list1, NULL,
                           _DBUS_INT_TO_POINTER (0));
  verify_list (&list1);

  /* inserting after first element */
  _dbus_list_insert_after (&list1, list1,
                           _DBUS_INT_TO_POINTER (1));
  verify_list (&list1);
  _dbus_assert (is_ascending_sequence (&list1));

  /* inserting at the end */
  _dbus_list_insert_after (&list1, list1->next,
                           _DBUS_INT_TO_POINTER (2));
  verify_list (&list1);
  _dbus_assert (is_ascending_sequence (&list1));

  /* using insert_after to prepend */
  _dbus_list_insert_after (&list1, NULL,
                           _DBUS_INT_TO_POINTER (-1));
  verify_list (&list1);
  _dbus_assert (is_ascending_sequence (&list1));
  
  _dbus_list_clear (&list1);

  /* using remove_last */
  _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (2));
  _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (1));
  _dbus_list_append (&list1, _DBUS_INT_TO_POINTER (3));

  _dbus_list_remove_last (&list1, _DBUS_INT_TO_POINTER (2));
  
  verify_list (&list1);
  _dbus_assert (is_ascending_sequence (&list1));
  
  _dbus_list_clear (&list1);
  
  return TRUE;
}

#endif
