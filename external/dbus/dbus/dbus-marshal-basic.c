
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-internals.h"
#include "dbus-marshal-basic.h"
#include "dbus-signature.h"

#include <string.h>


static void
pack_2_octets (dbus_uint16_t   value,
               int             byte_order,
               unsigned char  *data)
{
  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, 2) == data);

  if ((byte_order) == DBUS_LITTLE_ENDIAN)
    *((dbus_uint16_t*)(data)) = DBUS_UINT16_TO_LE (value);
  else
    *((dbus_uint16_t*)(data)) = DBUS_UINT16_TO_BE (value);
}

static void
pack_4_octets (dbus_uint32_t   value,
               int             byte_order,
               unsigned char  *data)
{
  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, 4) == data);

  if ((byte_order) == DBUS_LITTLE_ENDIAN)
    *((dbus_uint32_t*)(data)) = DBUS_UINT32_TO_LE (value);
  else
    *((dbus_uint32_t*)(data)) = DBUS_UINT32_TO_BE (value);
}

static void
pack_8_octets (DBusBasicValue     value,
               int                byte_order,
               unsigned char     *data)
{
  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, 8) == data);

#ifdef DBUS_HAVE_INT64
  if ((byte_order) == DBUS_LITTLE_ENDIAN)
    *((dbus_uint64_t*)(data)) = DBUS_UINT64_TO_LE (value.u64);
  else
    *((dbus_uint64_t*)(data)) = DBUS_UINT64_TO_BE (value.u64);
#else
  *(DBus8ByteStruct*)data = value.u64;
  swap_8_octets ((DBusBasicValue*)data, byte_order);
#endif
}

void
_dbus_pack_uint32 (dbus_uint32_t   value,
                   int             byte_order,
                   unsigned char  *data)
{
  pack_4_octets (value, byte_order, data);
}

#ifndef DBUS_HAVE_INT64
/* from ORBit */
static void
swap_bytes (unsigned char *data,
            unsigned int   len)
{
  unsigned char *p1 = data;
  unsigned char *p2 = data + len - 1;

  while (p1 < p2)
    {
      unsigned char tmp = *p1;
      *p1 = *p2;
      *p2 = tmp;

      --p2;
      ++p1;
    }
}
#endif /* !DBUS_HAVE_INT64 */

static void
swap_8_octets (DBusBasicValue    *value,
               int                byte_order)
{
  if (byte_order != DBUS_COMPILER_BYTE_ORDER)
    {
#ifdef DBUS_HAVE_INT64
      value->u64 = DBUS_UINT64_SWAP_LE_BE (value->u64);
#else
      swap_bytes ((unsigned char *)value, 8);
#endif
    }
}

#if 0
static DBusBasicValue
unpack_8_octets (int                  byte_order,
                 const unsigned char *data)
{
  DBusBasicValue r;

  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, 8) == data);
  _dbus_assert (sizeof (r) == 8);

#ifdef DBUS_HAVE_INT64
  if (byte_order == DBUS_LITTLE_ENDIAN)
    r.u64 = DBUS_UINT64_FROM_LE (*(dbus_uint64_t*)data);
  else
    r.u64 = DBUS_UINT64_FROM_BE (*(dbus_uint64_t*)data);
#else
  r.u64 = *(DBus8ByteStruct*)data;
  swap_8_octets (&r, byte_order);
#endif

  return r;
}
#endif

#ifndef _dbus_unpack_uint16
dbus_uint16_t
_dbus_unpack_uint16 (int                  byte_order,
                     const unsigned char *data)
{
  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, 2) == data);

  if (byte_order == DBUS_LITTLE_ENDIAN)
    return DBUS_UINT16_FROM_LE (*(dbus_uint16_t*)data);
  else
    return DBUS_UINT16_FROM_BE (*(dbus_uint16_t*)data);
}
#endif /* _dbus_unpack_uint16 */

#ifndef _dbus_unpack_uint32
dbus_uint32_t
_dbus_unpack_uint32 (int                  byte_order,
                     const unsigned char *data)
{
  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, 4) == data);

  if (byte_order == DBUS_LITTLE_ENDIAN)
    return DBUS_UINT32_FROM_LE (*(dbus_uint32_t*)data);
  else
    return DBUS_UINT32_FROM_BE (*(dbus_uint32_t*)data);
}
#endif /* _dbus_unpack_uint32 */

static void
set_2_octets (DBusString          *str,
              int                  offset,
              dbus_uint16_t        value,
              int                  byte_order)
{
  char *data;

  _dbus_assert (byte_order == DBUS_LITTLE_ENDIAN ||
                byte_order == DBUS_BIG_ENDIAN);

  data = _dbus_string_get_data_len (str, offset, 2);

  pack_2_octets (value, byte_order, data);
}

static void
set_4_octets (DBusString          *str,
              int                  offset,
              dbus_uint32_t        value,
              int                  byte_order)
{
  char *data;

  _dbus_assert (byte_order == DBUS_LITTLE_ENDIAN ||
                byte_order == DBUS_BIG_ENDIAN);

  data = _dbus_string_get_data_len (str, offset, 4);

  pack_4_octets (value, byte_order, data);
}

static void
set_8_octets (DBusString          *str,
              int                  offset,
              DBusBasicValue       value,
              int                  byte_order)
{
  char *data;

  _dbus_assert (byte_order == DBUS_LITTLE_ENDIAN ||
                byte_order == DBUS_BIG_ENDIAN);

  data = _dbus_string_get_data_len (str, offset, 8);

  pack_8_octets (value, byte_order, data);
}

void
_dbus_marshal_set_uint32 (DBusString          *str,
                          int                  pos,
                          dbus_uint32_t        value,
                          int                  byte_order)
{
  set_4_octets (str, pos, value, byte_order);
}

static dbus_bool_t
set_string (DBusString          *str,
            int                  pos,
            const char          *value,
            int                  byte_order,
            int                 *old_end_pos,
            int                 *new_end_pos)
{
  int old_len, new_len;
  DBusString dstr;

  _dbus_string_init_const (&dstr, value);

  _dbus_assert (_DBUS_ALIGN_VALUE (pos, 4) == (unsigned) pos);
  old_len = _dbus_unpack_uint32 (byte_order,
                                 _dbus_string_get_const_data_len (str, pos, 4));

  new_len = _dbus_string_get_length (&dstr);

  if (!_dbus_string_replace_len (&dstr, 0, new_len,
                                 str, pos + 4, old_len))
    return FALSE;

  _dbus_marshal_set_uint32 (str, pos, new_len, byte_order);

  if (old_end_pos)
    *old_end_pos = pos + 4 + old_len + 1;
  if (new_end_pos)
    *new_end_pos = pos + 4 + new_len + 1;

  return TRUE;
}

static dbus_bool_t
set_signature (DBusString          *str,
               int                  pos,
               const char          *value,
               int                  byte_order,
               int                 *old_end_pos,
               int                 *new_end_pos)
{
  int old_len, new_len;
  DBusString dstr;

  _dbus_string_init_const (&dstr, value);

  old_len = _dbus_string_get_byte (str, pos);
  new_len = _dbus_string_get_length (&dstr);

  if (!_dbus_string_replace_len (&dstr, 0, new_len,
                                 str, pos + 1, old_len))
    return FALSE;

  _dbus_string_set_byte (str, pos, new_len);

  if (old_end_pos)
    *old_end_pos = pos + 1 + old_len + 1;
  if (new_end_pos)
    *new_end_pos = pos + 1 + new_len + 1;

  return TRUE;
}

dbus_bool_t
_dbus_marshal_set_basic (DBusString       *str,
                         int               pos,
                         int               type,
                         const void       *value,
                         int               byte_order,
                         int              *old_end_pos,
                         int              *new_end_pos)
{
  const DBusBasicValue *vp;

  vp = value;

  switch (type)
    {
    case DBUS_TYPE_BYTE:
      _dbus_string_set_byte (str, pos, vp->byt);
      if (old_end_pos)
        *old_end_pos = pos + 1;
      if (new_end_pos)
        *new_end_pos = pos + 1;
      return TRUE;
      break;
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
      pos = _DBUS_ALIGN_VALUE (pos, 2);
      set_2_octets (str, pos, vp->u16, byte_order);
      if (old_end_pos)
        *old_end_pos = pos + 2;
      if (new_end_pos)
        *new_end_pos = pos + 2;
      return TRUE;
      break;
    case DBUS_TYPE_BOOLEAN:
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
      pos = _DBUS_ALIGN_VALUE (pos, 4);
      set_4_octets (str, pos, vp->u32, byte_order);
      if (old_end_pos)
        *old_end_pos = pos + 4;
      if (new_end_pos)
        *new_end_pos = pos + 4;
      return TRUE;
      break;
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
      pos = _DBUS_ALIGN_VALUE (pos, 8);
      set_8_octets (str, pos, *vp, byte_order);
      if (old_end_pos)
        *old_end_pos = pos + 8;
      if (new_end_pos)
        *new_end_pos = pos + 8;
      return TRUE;
      break;
    case DBUS_TYPE_STRING:
    case DBUS_TYPE_OBJECT_PATH:
      pos = _DBUS_ALIGN_VALUE (pos, 4);
      _dbus_assert (vp->str != NULL);
      return set_string (str, pos, vp->str, byte_order,
                         old_end_pos, new_end_pos);
      break;
    case DBUS_TYPE_SIGNATURE:
      _dbus_assert (vp->str != NULL);
      return set_signature (str, pos, vp->str, byte_order,
                            old_end_pos, new_end_pos);
      break;
    default:
      _dbus_assert_not_reached ("not a basic type");
      return FALSE;
      break;
    }
}

dbus_uint32_t
_dbus_marshal_read_uint32  (const DBusString *str,
                            int               pos,
                            int               byte_order,
                            int              *new_pos)
{
  pos = _DBUS_ALIGN_VALUE (pos, 4);

  if (new_pos)
    *new_pos = pos + 4;

  _dbus_assert (pos + 4 <= _dbus_string_get_length (str));
  
  return _dbus_unpack_uint32 (byte_order,
                              _dbus_string_get_const_data (str) + pos);
}

void
_dbus_marshal_read_basic (const DBusString      *str,
                          int                    pos,
                          int                    type,
                          void                  *value,
                          int                    byte_order,
                          int                   *new_pos)
{
  const char *str_data;
  DBusBasicValue *vp;

  _dbus_assert (dbus_type_is_basic (type));

  str_data = _dbus_string_get_const_data (str);
  vp = value;

  switch (type)
    {
    case DBUS_TYPE_BYTE:
      vp->byt = _dbus_string_get_byte (str, pos);
      (pos)++;
      break;
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
      pos = _DBUS_ALIGN_VALUE (pos, 2);
      vp->u16 = *(dbus_uint16_t *)(str_data + pos);
      if (byte_order != DBUS_COMPILER_BYTE_ORDER)
	vp->u16 = DBUS_UINT16_SWAP_LE_BE (vp->u16);
      pos += 2;
      break;
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
    case DBUS_TYPE_BOOLEAN:
      pos = _DBUS_ALIGN_VALUE (pos, 4);
      vp->u32 = *(dbus_uint32_t *)(str_data + pos);
      if (byte_order != DBUS_COMPILER_BYTE_ORDER)
	vp->u32 = DBUS_UINT32_SWAP_LE_BE (vp->u32);
      pos += 4;
      break;
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
      pos = _DBUS_ALIGN_VALUE (pos, 8);
#ifdef DBUS_HAVE_INT64
      if (byte_order != DBUS_COMPILER_BYTE_ORDER)
        vp->u64 = DBUS_UINT64_SWAP_LE_BE (*(dbus_uint64_t*)(str_data + pos));
      else
        vp->u64 = *(dbus_uint64_t*)(str_data + pos);
#else
      vp->u64 = *(DBus8ByteStruct*) (str_data + pos);
      swap_8_octets (vp, byte_order);
#endif
      pos += 8;
      break;
    case DBUS_TYPE_STRING:
    case DBUS_TYPE_OBJECT_PATH:
      {
        int len;

        len = _dbus_marshal_read_uint32 (str, pos, byte_order, &pos);

        vp->str = (char*) str_data + pos;

        pos += len + 1; /* length plus nul */
      }
      break;
    case DBUS_TYPE_SIGNATURE:
      {
        int len;

        len = _dbus_string_get_byte (str, pos);
        pos += 1;

        vp->str = (char*) str_data + pos;

        pos += len + 1; /* length plus nul */
      }
      break;
    default:
      _dbus_warn_check_failed ("type %s %d not a basic type\n",
                               _dbus_type_to_string (type), type);
      _dbus_assert_not_reached ("not a basic type");
      break;
    }

  if (new_pos)
    *new_pos = pos;
}

static dbus_bool_t
marshal_2_octets (DBusString   *str,
                  int           insert_at,
                  dbus_uint16_t value,
                  int           byte_order,
                  int          *pos_after)
{
  dbus_bool_t retval;
  int orig_len;

  _dbus_assert (sizeof (value) == 2);

  if (byte_order != DBUS_COMPILER_BYTE_ORDER)
    value = DBUS_UINT16_SWAP_LE_BE (value);

  orig_len = _dbus_string_get_length (str);

  retval = _dbus_string_insert_2_aligned (str, insert_at,
                                          (const unsigned char *)&value);

  if (pos_after)
    {
      *pos_after = insert_at + (_dbus_string_get_length (str) - orig_len);
      _dbus_assert (*pos_after <= _dbus_string_get_length (str));
    }

  return retval;
}

static dbus_bool_t
marshal_4_octets (DBusString   *str,
                  int           insert_at,
                  dbus_uint32_t value,
                  int           byte_order,
                  int          *pos_after)
{
  dbus_bool_t retval;
  int orig_len;

  _dbus_assert (sizeof (value) == 4);

  if (byte_order != DBUS_COMPILER_BYTE_ORDER)
    value = DBUS_UINT32_SWAP_LE_BE (value);

  orig_len = _dbus_string_get_length (str);

  retval = _dbus_string_insert_4_aligned (str, insert_at,
                                          (const unsigned char *)&value);

  if (pos_after)
    {
      *pos_after = insert_at + (_dbus_string_get_length (str) - orig_len);
      _dbus_assert (*pos_after <= _dbus_string_get_length (str));
    }

  return retval;
}

static dbus_bool_t
marshal_8_octets (DBusString    *str,
                  int            insert_at,
                  DBusBasicValue value,
                  int            byte_order,
                  int           *pos_after)
{
  dbus_bool_t retval;
  int orig_len;

  _dbus_assert (sizeof (value) == 8);

  swap_8_octets (&value, byte_order);

  orig_len = _dbus_string_get_length (str);

  retval = _dbus_string_insert_8_aligned (str, insert_at,
                                          (const unsigned char *)&value);

  if (pos_after)
    *pos_after = insert_at + _dbus_string_get_length (str) - orig_len;

  return retval;
}

enum
  {
    MARSHAL_AS_STRING,
    MARSHAL_AS_SIGNATURE,
    MARSHAL_AS_BYTE_ARRAY
  };

static dbus_bool_t
marshal_len_followed_by_bytes (int                  marshal_as,
                               DBusString          *str,
                               int                  insert_at,
                               const unsigned char *value,
                               int                  data_len, /* doesn't include nul if any */
                               int                  byte_order,
                               int                 *pos_after)
{
  int pos;
  DBusString value_str;
  int value_len;

  _dbus_assert (byte_order == DBUS_LITTLE_ENDIAN || byte_order == DBUS_BIG_ENDIAN);
  if (insert_at > _dbus_string_get_length (str))
    _dbus_warn ("insert_at = %d string len = %d data_len = %d\n",
                insert_at, _dbus_string_get_length (str), data_len);
  
  if (marshal_as == MARSHAL_AS_BYTE_ARRAY)
    value_len = data_len;
  else
    value_len = data_len + 1; /* value has a nul */

  _dbus_string_init_const_len (&value_str, value, value_len);

  pos = insert_at;

  if (marshal_as == MARSHAL_AS_SIGNATURE)
    {
      _dbus_assert (data_len <= DBUS_MAXIMUM_SIGNATURE_LENGTH);
      _dbus_assert (data_len <= 255); /* same as max sig len right now */
      
      if (!_dbus_string_insert_byte (str, pos, data_len))
        goto oom;

      pos += 1;
    }
  else
    {
      if (!marshal_4_octets (str, pos, data_len,
                             byte_order, &pos))
        goto oom;
    }

  if (!_dbus_string_copy_len (&value_str, 0, value_len,
                              str, pos))
    goto oom;

#if 0
  /* too expensive */
  _dbus_assert (_dbus_string_equal_substring (&value_str, 0, value_len,
                                              str, pos));
  _dbus_verbose_bytes_of_string (str, pos, value_len);
#endif

  pos += value_len;

  if (pos_after)
    *pos_after = pos;

  return TRUE;

 oom:
  /* Delete what we've inserted */
  _dbus_string_delete (str, insert_at, pos - insert_at);

  return FALSE;
}

static dbus_bool_t
marshal_string (DBusString    *str,
                int            insert_at,
                const char    *value,
                int            byte_order,
                int           *pos_after)
{
  return marshal_len_followed_by_bytes (MARSHAL_AS_STRING,
                                        str, insert_at, value,
                                        strlen (value),
                                        byte_order, pos_after);
}

static dbus_bool_t
marshal_signature (DBusString    *str,
                   int            insert_at,
                   const char    *value,
                   int           *pos_after)
{
  return marshal_len_followed_by_bytes (MARSHAL_AS_SIGNATURE,
                                        str, insert_at, value,
                                        strlen (value),
                                        DBUS_COMPILER_BYTE_ORDER, /* irrelevant */
                                        pos_after);
}

dbus_bool_t
_dbus_marshal_write_basic (DBusString *str,
                           int         insert_at,
                           int         type,
                           const void *value,
                           int         byte_order,
                           int        *pos_after)
{
  const DBusBasicValue *vp;

  _dbus_assert (dbus_type_is_basic (type));

  vp = value;

  switch (type)
    {
    case DBUS_TYPE_BYTE:
      if (!_dbus_string_insert_byte (str, insert_at, vp->byt))
        return FALSE;
      if (pos_after)
        *pos_after = insert_at + 1;
      return TRUE;
      break;
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
      return marshal_2_octets (str, insert_at, vp->u16,
                               byte_order, pos_after);
      break;
    case DBUS_TYPE_BOOLEAN:
      return marshal_4_octets (str, insert_at, vp->u32 != FALSE,
                               byte_order, pos_after);
      break;
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
      return marshal_4_octets (str, insert_at, vp->u32,
                               byte_order, pos_after);
      break;
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
      return marshal_8_octets (str, insert_at, *vp, byte_order, pos_after);
      break;

    case DBUS_TYPE_STRING:
    case DBUS_TYPE_OBJECT_PATH:
      _dbus_assert (vp->str != NULL);
      return marshal_string (str, insert_at, vp->str, byte_order, pos_after);
      break;
    case DBUS_TYPE_SIGNATURE:
      _dbus_assert (vp->str != NULL);
      return marshal_signature (str, insert_at, vp->str, pos_after);
      break;
    default:
      _dbus_assert_not_reached ("not a basic type");
      return FALSE;
      break;
    }
}

static dbus_bool_t
marshal_1_octets_array (DBusString          *str,
                        int                  insert_at,
                        const unsigned char *value,
                        int                  n_elements,
                        int                  byte_order,
                        int                 *pos_after)
{
  int pos;
  DBusString value_str;

  _dbus_string_init_const_len (&value_str, value, n_elements);

  pos = insert_at;

  if (!_dbus_string_copy_len (&value_str, 0, n_elements,
                              str, pos))
    return FALSE;

  pos += n_elements;

  if (pos_after)
    *pos_after = pos;

  return TRUE;
}

void
_dbus_swap_array (unsigned char *data,
                  int            n_elements,
                  int            alignment)
{
  unsigned char *d;
  unsigned char *end;

  _dbus_assert (_DBUS_ALIGN_ADDRESS (data, alignment) == data);

  /* we use const_data and cast it off so DBusString can be a const string
   * for the unit tests. don't ask.
   */
  d = data;
  end = d + (n_elements * alignment);
  
  if (alignment == 8)
    {
      while (d != end)
        {
#ifdef DBUS_HAVE_INT64
          *((dbus_uint64_t*)d) = DBUS_UINT64_SWAP_LE_BE (*((dbus_uint64_t*)d));
#else
          swap_8_bytes ((DBusBasicValue*) d);
#endif
          d += 8;
        }
    }
  else if (alignment == 4)
    {
      while (d != end)
        {
          *((dbus_uint32_t*)d) = DBUS_UINT32_SWAP_LE_BE (*((dbus_uint32_t*)d));
          d += 4;
        }
    }
  else
    {
      _dbus_assert (alignment == 2);
      
      while (d != end)
        {
          *((dbus_uint16_t*)d) = DBUS_UINT16_SWAP_LE_BE (*((dbus_uint16_t*)d));
          d += 2;
        }
    }
}

static void
swap_array (DBusString *str,
            int         array_start,
            int         n_elements,
            int         byte_order,
            int         alignment)
{
  _dbus_assert (_DBUS_ALIGN_VALUE (array_start, alignment) == (unsigned) array_start);

  if (byte_order != DBUS_COMPILER_BYTE_ORDER)
    {
      /* we use const_data and cast it off so DBusString can be a const string
       * for the unit tests. don't ask.
       */
      _dbus_swap_array ((unsigned char*) (_dbus_string_get_const_data (str) + array_start),
                        n_elements, alignment);
    }
}

static dbus_bool_t
marshal_fixed_multi (DBusString           *str,
                     int                   insert_at,
                     const DBusBasicValue *value,
                     int                   n_elements,
                     int                   byte_order,
                     int                   alignment,
                     int                  *pos_after)
{
  int old_string_len;
  int array_start;
  DBusString t;
  int len_in_bytes;

  _dbus_assert (n_elements <= DBUS_MAXIMUM_ARRAY_LENGTH / alignment);
  
  old_string_len = _dbus_string_get_length (str);

  len_in_bytes = n_elements * alignment;
  array_start = insert_at;
  
  /* Note that we do alignment padding unconditionally
   * even if the array is empty; this means that
   * padding + len is always equal to the number of bytes
   * in the array.
   */

  if (!_dbus_string_insert_alignment (str, &array_start, alignment))
    goto error;

  _dbus_string_init_const_len (&t,
                               (const unsigned char*) value,
                               len_in_bytes);

  if (!_dbus_string_copy (&t, 0,
                          str, array_start))
    goto error;

  swap_array (str, array_start, n_elements, byte_order, alignment);

  if (pos_after)
    *pos_after = array_start + len_in_bytes;
  
  return TRUE;

 error:
  _dbus_string_delete (str, insert_at,
                       _dbus_string_get_length (str) - old_string_len);

  return FALSE;
}

dbus_bool_t
_dbus_marshal_write_fixed_multi (DBusString *str,
                                 int         insert_at,
                                 int         element_type,
                                 const void *value,
                                 int         n_elements,
                                 int         byte_order,
                                 int        *pos_after)
{
  const void* vp = *(const DBusBasicValue**)value;
  
  _dbus_assert (dbus_type_is_fixed (element_type));
  _dbus_assert (n_elements >= 0);

#if 0
  _dbus_verbose ("writing %d elements of %s\n",
                 n_elements, _dbus_type_to_string (element_type));
#endif
  
  switch (element_type)
    {
    case DBUS_TYPE_BYTE:
      return marshal_1_octets_array (str, insert_at, vp, n_elements, byte_order, pos_after);
      break;
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
      return marshal_fixed_multi (str, insert_at, vp, n_elements, byte_order, 2, pos_after);
      /* FIXME: we canonicalize to 0 or 1 for the single boolean case
       * should we here too ? */
    case DBUS_TYPE_BOOLEAN:
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
      return marshal_fixed_multi (str, insert_at, vp, n_elements, byte_order, 4, pos_after);
      break;
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
      return marshal_fixed_multi (str, insert_at, vp, n_elements, byte_order, 8, pos_after);
      break;

    default:
      _dbus_assert_not_reached ("non fixed type in array write");
      break;
    }

  return FALSE;
}


void
_dbus_marshal_skip_basic (const DBusString      *str,
                          int                    type,
                          int                    byte_order,
                          int                   *pos)
{
  _dbus_assert (byte_order == DBUS_LITTLE_ENDIAN ||
                byte_order == DBUS_BIG_ENDIAN);
  
  switch (type)
    {
    case DBUS_TYPE_BYTE:
      (*pos)++;
      break;
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
      *pos = _DBUS_ALIGN_VALUE (*pos, 2);
      *pos += 2;
      break;
    case DBUS_TYPE_BOOLEAN:
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
      *pos = _DBUS_ALIGN_VALUE (*pos, 4);
      *pos += 4;
      break;
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
      *pos = _DBUS_ALIGN_VALUE (*pos, 8);
      *pos += 8;
      break;
    case DBUS_TYPE_STRING:
    case DBUS_TYPE_OBJECT_PATH:
      {
        int len;

        len = _dbus_marshal_read_uint32 (str, *pos, byte_order, pos);
        
        *pos += len + 1; /* length plus nul */
      }
      break;
    case DBUS_TYPE_SIGNATURE:
      {
        int len;

        len = _dbus_string_get_byte (str, *pos);

        *pos += len + 2; /* length byte plus length plus nul */
      }
      break;
    default:
      _dbus_warn ("type %s not a basic type\n",
                  _dbus_type_to_string (type));
      _dbus_assert_not_reached ("not a basic type");
      break;
    }
}

void
_dbus_marshal_skip_array (const DBusString  *str,
                          int                element_type,
                          int                byte_order,
                          int               *pos)
{
  dbus_uint32_t array_len;
  int i;
  int alignment;

  i = _DBUS_ALIGN_VALUE (*pos, 4);

  array_len = _dbus_marshal_read_uint32 (str, i, byte_order, &i);

  alignment = _dbus_type_get_alignment (element_type);

  i = _DBUS_ALIGN_VALUE (i, alignment);

  *pos = i + array_len;
}

int
_dbus_type_get_alignment (int typecode)
{
  switch (typecode)
    {
    case DBUS_TYPE_BYTE:
    case DBUS_TYPE_VARIANT:
    case DBUS_TYPE_SIGNATURE:
      return 1;
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
      return 2;
    case DBUS_TYPE_BOOLEAN:
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
      /* this stuff is 4 since it starts with a length */
    case DBUS_TYPE_STRING:
    case DBUS_TYPE_OBJECT_PATH:
    case DBUS_TYPE_ARRAY:
      return 4;
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
      /* struct is 8 since it could contain an 8-aligned item
       * and it's simpler to just always align structs to 8;
       * we want the amount of padding in a struct of a given
       * type to be predictable, not location-dependent.
       * DICT_ENTRY is always the same as struct.
       */
    case DBUS_TYPE_STRUCT:
    case DBUS_TYPE_DICT_ENTRY:
      return 8;

    default:
      _dbus_assert_not_reached ("unknown typecode in _dbus_type_get_alignment()");
      return 0;
    }
}


dbus_bool_t
_dbus_type_is_valid (int typecode)
{
  switch (typecode)
    {
    case DBUS_TYPE_BYTE:
    case DBUS_TYPE_BOOLEAN:
    case DBUS_TYPE_INT16:
    case DBUS_TYPE_UINT16:
    case DBUS_TYPE_INT32:
    case DBUS_TYPE_UINT32:
    case DBUS_TYPE_INT64:
    case DBUS_TYPE_UINT64:
    case DBUS_TYPE_DOUBLE:
    case DBUS_TYPE_STRING:
    case DBUS_TYPE_OBJECT_PATH:
    case DBUS_TYPE_SIGNATURE:
    case DBUS_TYPE_ARRAY:
    case DBUS_TYPE_STRUCT:
    case DBUS_TYPE_DICT_ENTRY:
    case DBUS_TYPE_VARIANT:
      return TRUE;

    default:
      return FALSE;
    }
}

const char *
_dbus_type_to_string (int typecode)
{
  switch (typecode)
    {
    case DBUS_TYPE_INVALID:
      return "invalid";
    case DBUS_TYPE_BOOLEAN:
      return "boolean";
    case DBUS_TYPE_BYTE:
      return "byte";
    case DBUS_TYPE_INT16:
      return "int16";
    case DBUS_TYPE_UINT16:
      return "uint16";
    case DBUS_TYPE_INT32:
      return "int32";
    case DBUS_TYPE_UINT32:
      return "uint32";
    case DBUS_TYPE_INT64:
      return "int64";
    case DBUS_TYPE_UINT64:
      return "uint64";      
    case DBUS_TYPE_DOUBLE:
      return "double";
    case DBUS_TYPE_STRING:
      return "string";
    case DBUS_TYPE_OBJECT_PATH:
      return "object_path";
    case DBUS_TYPE_SIGNATURE:
      return "signature";
    case DBUS_TYPE_STRUCT:
      return "struct";
    case DBUS_TYPE_DICT_ENTRY:
      return "dict_entry";
    case DBUS_TYPE_ARRAY:
      return "array";
    case DBUS_TYPE_VARIANT:
      return "variant";
    case DBUS_STRUCT_BEGIN_CHAR:
      return "begin_struct";
    case DBUS_STRUCT_END_CHAR:
      return "end_struct";
    case DBUS_DICT_ENTRY_BEGIN_CHAR:
      return "begin_dict_entry";
    case DBUS_DICT_ENTRY_END_CHAR:
      return "end_dict_entry";
    default:
      return "unknown";
    }
}

void
_dbus_verbose_bytes (const unsigned char *data,
                     int                  len,
                     int                  offset)
{
  int i;
  const unsigned char *aligned;

  _dbus_assert (len >= 0);

  if (!_dbus_is_verbose())
    return;

  /* Print blanks on first row if appropriate */
  aligned = _DBUS_ALIGN_ADDRESS (data, 4);
  if (aligned > data)
    aligned -= 4;
  _dbus_assert (aligned <= data);

  if (aligned != data)
    {
      _dbus_verbose ("%4d\t%p: ", - (data - aligned), aligned);
      while (aligned != data)
        {
          _dbus_verbose ("    ");
          ++aligned;
        }
    }

  /* now print the bytes */
  i = 0;
  while (i < len)
    {
      if (_DBUS_ALIGN_ADDRESS (&data[i], 4) == &data[i])
        {
          _dbus_verbose ("%4d\t%p: ",
                         offset + i, &data[i]);
        }

      if (data[i] >= 32 &&
          data[i] <= 126)
        _dbus_verbose (" '%c' ", data[i]);
      else
        _dbus_verbose ("0x%s%x ",
                       data[i] <= 0xf ? "0" : "", data[i]);

      ++i;

      if (_DBUS_ALIGN_ADDRESS (&data[i], 4) == &data[i])
        {
          if (i > 3)
            _dbus_verbose ("BE: %d LE: %d",
                           _dbus_unpack_uint32 (DBUS_BIG_ENDIAN, &data[i-4]),
                           _dbus_unpack_uint32 (DBUS_LITTLE_ENDIAN, &data[i-4]));

          if (i > 7 &&
              _DBUS_ALIGN_ADDRESS (&data[i], 8) == &data[i])
            {
#ifdef DBUS_HAVE_INT64
              /* I think I probably mean "GNU libc printf" and not "GNUC"
               * but we'll wait until someone complains. If you hit this,
               * just turn off verbose mode as a workaround.
               */
#if __GNUC__
              _dbus_verbose (" u64: 0x%llx",
                             *(dbus_uint64_t*)&data[i-8]);
#endif
#endif
              _dbus_verbose (" dbl: %g",
                             *(double*)&data[i-8]);
            }

          _dbus_verbose ("\n");
        }
    }

  _dbus_verbose ("\n");
}

void
_dbus_verbose_bytes_of_string (const DBusString    *str,
                               int                  start,
                               int                  len)
{
  const char *d;
  int real_len;

  real_len = _dbus_string_get_length (str);

  _dbus_assert (start >= 0);

  if (start > real_len)
    {
      _dbus_verbose ("  [%d,%d) is not inside string of length %d\n",
                     start, len, real_len);
      return;
    }

  if ((start + len) > real_len)
    {
      _dbus_verbose ("  [%d,%d) extends outside string of length %d\n",
                     start, len, real_len);
      len = real_len - start;
    }

  d = _dbus_string_get_const_data_len (str, start, len);

  _dbus_verbose_bytes (d, len, start);
}

static int
map_type_char_to_type (int t)
{
  if (t == DBUS_STRUCT_BEGIN_CHAR)
    return DBUS_TYPE_STRUCT;
  else if (t == DBUS_DICT_ENTRY_BEGIN_CHAR)
    return DBUS_TYPE_DICT_ENTRY;
  else
    {
      _dbus_assert (t != DBUS_STRUCT_END_CHAR);
      _dbus_assert (t != DBUS_DICT_ENTRY_END_CHAR);
      return t;
    }
}

int
_dbus_first_type_in_signature (const DBusString *str,
                               int               pos)
{
  return map_type_char_to_type (_dbus_string_get_byte (str, pos));
}

int
_dbus_first_type_in_signature_c_str (const char       *str,
				     int               pos)
{
  return map_type_char_to_type (str[pos]);
}

/** @} */

#ifdef DBUS_BUILD_TESTS
#include "dbus-test.h"
#include <stdio.h>

void
_dbus_marshal_read_fixed_multi  (const DBusString *str,
                                 int               pos,
                                 int               element_type,
                                 void             *value,
                                 int               n_elements,
                                 int               byte_order,
                                 int              *new_pos)
{
  int array_len;
  int alignment;

  _dbus_assert (dbus_type_is_fixed (element_type));
  _dbus_assert (dbus_type_is_basic (element_type));

#if 0
  _dbus_verbose ("reading %d elements of %s\n",
                 n_elements, _dbus_type_to_string (element_type));
#endif
  
  alignment = _dbus_type_get_alignment (element_type);

  pos = _DBUS_ALIGN_VALUE (pos, alignment);
  
  array_len = n_elements * alignment;

  *(const DBusBasicValue**) value = (void*) _dbus_string_get_const_data_len (str, pos, array_len);
  if (new_pos)
    *new_pos = pos + array_len;
}

static void
swap_test_array (void *array,
                 int   len_bytes,
                 int   byte_order,
                 int   alignment)
{
  DBusString t;

  if (alignment == 1)
    return;
  
  _dbus_string_init_const_len (&t, array, len_bytes);
  swap_array (&t, 0, len_bytes / alignment, byte_order, alignment);
}

#define MARSHAL_BASIC(typename, byte_order, literal)                    \
  do {                                                                  \
     v_##typename = literal;                                            \
     if (!_dbus_marshal_write_basic (&str, pos, DBUS_TYPE_##typename,   \
                                    &v_##typename,                      \
                                    byte_order, NULL))                  \
       _dbus_assert_not_reached ("no memory");                          \
   } while (0)

#define DEMARSHAL_BASIC(typename, byte_order)                                   \
  do {                                                                          \
    _dbus_marshal_read_basic (&str, pos, DBUS_TYPE_##typename, &v_##typename,   \
                              byte_order, &pos);                                \
  } while (0)

#define DEMARSHAL_BASIC_AND_CHECK(typename, byte_order, literal)                        \
  do {                                                                                  \
    DEMARSHAL_BASIC (typename, byte_order);                                             \
    if (literal != v_##typename)                                                        \
      {                                                                                 \
        _dbus_verbose_bytes_of_string (&str, dump_pos,                                  \
                                     _dbus_string_get_length (&str) - dump_pos);        \
        _dbus_assert_not_reached ("demarshaled wrong value");                           \
      }                                                                                 \
  } while (0)

#define MARSHAL_TEST(typename, byte_order, literal)             \
  do {                                                          \
    MARSHAL_BASIC (typename, byte_order, literal);              \
    dump_pos = pos;                                             \
    DEMARSHAL_BASIC_AND_CHECK (typename, byte_order, literal);  \
  } while (0)

#define MARSHAL_TEST_STRCMP(typename, byte_order, literal)                              \
  do {                                                                                  \
    MARSHAL_BASIC (typename, byte_order, literal);                                      \
    dump_pos = pos;                                                                     \
    DEMARSHAL_BASIC (typename, byte_order);                                             \
    if (strcmp (literal, v_##typename) != 0)                                            \
      {                                                                                 \
        _dbus_verbose_bytes_of_string (&str, dump_pos,                                  \
                                       _dbus_string_get_length (&str) - dump_pos);      \
        _dbus_warn ("literal '%s'\nvalue  '%s'\n", literal, v_##typename);              \
        _dbus_assert_not_reached ("demarshaled wrong value");                           \
      }                                                                                 \
  } while (0)

#define MARSHAL_FIXED_ARRAY(typename, byte_order, literal)                                      \
  do {                                                                                          \
     int next;                                                                                  \
     v_UINT32 = sizeof(literal);                                                                \
     if (!_dbus_marshal_write_basic (&str, pos, DBUS_TYPE_UINT32, &v_UINT32,                    \
                                     byte_order, &next))                                        \
       _dbus_assert_not_reached ("no memory");                                                  \
     v_ARRAY_##typename = literal;                                                              \
     if (!_dbus_marshal_write_fixed_multi (&str, next, DBUS_TYPE_##typename,                    \
                                           &v_ARRAY_##typename, _DBUS_N_ELEMENTS(literal),      \
                                           byte_order, NULL))                                   \
       _dbus_assert_not_reached ("no memory");                                                  \
   } while (0)

#define DEMARSHAL_FIXED_ARRAY(typename, byte_order)                                             \
  do {                                                                                          \
    int next;                                                                                   \
    alignment = _dbus_type_get_alignment (DBUS_TYPE_##typename);                                \
    v_UINT32 = _dbus_marshal_read_uint32 (&str, dump_pos, byte_order, &next);                   \
    _dbus_marshal_read_fixed_multi (&str, next, DBUS_TYPE_##typename, &v_ARRAY_##typename,      \
                                    v_UINT32/alignment,                                         \
                                    byte_order, NULL);                                          \
    swap_test_array (v_ARRAY_##typename, v_UINT32,                                              \
                     byte_order, alignment);                                                    \
  } while (0)

#define DEMARSHAL_FIXED_ARRAY_AND_CHECK(typename, byte_order, literal)                  \
  do {                                                                                  \
    DEMARSHAL_FIXED_ARRAY (typename, byte_order);                                       \
    if (memcmp (literal, v_ARRAY_##typename, sizeof (literal) != 0))                    \
      {                                                                                 \
        _dbus_verbose ("MARSHALED DATA\n");                                             \
        _dbus_verbose_bytes_of_string (&str, dump_pos,                                  \
                                      _dbus_string_get_length (&str) - dump_pos);       \
        _dbus_verbose ("LITERAL DATA\n");                                               \
        _dbus_verbose_bytes ((char*)literal, sizeof (literal), 0);                      \
        _dbus_verbose ("READ DATA\n");                                                  \
        _dbus_verbose_bytes ((char*)v_ARRAY_##typename, sizeof (literal), 0);           \
        _dbus_assert_not_reached ("demarshaled wrong fixed array value");               \
      }                                                                                 \
  } while (0)

#define MARSHAL_TEST_FIXED_ARRAY(typename, byte_order, literal)         \
  do {                                                                  \
    MARSHAL_FIXED_ARRAY (typename, byte_order, literal);                \
    dump_pos = pos;                                                     \
    DEMARSHAL_FIXED_ARRAY_AND_CHECK (typename, byte_order, literal);    \
  } while (0)

dbus_bool_t
_dbus_marshal_test (void)
{
  int alignment;
  DBusString str;
  int pos, dump_pos;
  unsigned char array1[5] = { 3, 4, 0, 1, 9 };
  dbus_int16_t array2[3] = { 124, 457, 780 };
  dbus_int32_t array4[3] = { 123, 456, 789 };
#ifdef DBUS_HAVE_INT64
  dbus_int64_t array8[3] = { DBUS_INT64_CONSTANT (0x123ffffffff),
                             DBUS_INT64_CONSTANT (0x456ffffffff),
                             DBUS_INT64_CONSTANT (0x789ffffffff) };
  dbus_int64_t *v_ARRAY_INT64;
#endif
  unsigned char *v_ARRAY_BYTE;
  dbus_int16_t *v_ARRAY_INT16;
  dbus_uint16_t *v_ARRAY_UINT16;
  dbus_int32_t *v_ARRAY_INT32;
  dbus_uint32_t *v_ARRAY_UINT32;
  DBusString t;
  double v_DOUBLE;
  double t_DOUBLE;
  dbus_int16_t v_INT16;
  dbus_uint16_t v_UINT16;
  dbus_int32_t v_INT32;
  dbus_uint32_t v_UINT32;
  dbus_int64_t v_INT64;
  dbus_uint64_t v_UINT64;
  unsigned char v_BYTE;
  dbus_bool_t v_BOOLEAN;
  const char *v_STRING;
  const char *v_SIGNATURE;
  const char *v_OBJECT_PATH;
  int byte_order;

  if (!_dbus_string_init (&str))
    _dbus_assert_not_reached ("failed to init string");

  pos = 0;

  /* Marshal doubles */
  MARSHAL_BASIC (DOUBLE, DBUS_BIG_ENDIAN, 3.14);
  DEMARSHAL_BASIC (DOUBLE, DBUS_BIG_ENDIAN);
  t_DOUBLE = 3.14;
  if (!_DBUS_DOUBLES_BITWISE_EQUAL (t_DOUBLE, v_DOUBLE))
    _dbus_assert_not_reached ("got wrong double value");

  MARSHAL_BASIC (DOUBLE, DBUS_LITTLE_ENDIAN, 3.14);
  DEMARSHAL_BASIC (DOUBLE, DBUS_LITTLE_ENDIAN);
  t_DOUBLE = 3.14;
  if (!_DBUS_DOUBLES_BITWISE_EQUAL (t_DOUBLE, v_DOUBLE))
    _dbus_assert_not_reached ("got wrong double value");

  /* Marshal signed 16 integers */
  MARSHAL_TEST (INT16, DBUS_BIG_ENDIAN, -12345);
  MARSHAL_TEST (INT16, DBUS_LITTLE_ENDIAN, -12345);

  /* Marshal unsigned 16 integers */
  MARSHAL_TEST (UINT16, DBUS_BIG_ENDIAN, 0x1234);
  MARSHAL_TEST (UINT16, DBUS_LITTLE_ENDIAN, 0x1234);
  
  /* Marshal signed integers */
  MARSHAL_TEST (INT32, DBUS_BIG_ENDIAN, -12345678);
  MARSHAL_TEST (INT32, DBUS_LITTLE_ENDIAN, -12345678);

  /* Marshal unsigned integers */
  MARSHAL_TEST (UINT32, DBUS_BIG_ENDIAN, 0x12345678);
  MARSHAL_TEST (UINT32, DBUS_LITTLE_ENDIAN, 0x12345678);

#ifdef DBUS_HAVE_INT64
  /* Marshal signed integers */
  MARSHAL_TEST (INT64, DBUS_BIG_ENDIAN, DBUS_INT64_CONSTANT (-0x123456789abc7));
  MARSHAL_TEST (INT64, DBUS_LITTLE_ENDIAN, DBUS_INT64_CONSTANT (-0x123456789abc7));

  /* Marshal unsigned integers */
  MARSHAL_TEST (UINT64, DBUS_BIG_ENDIAN, DBUS_UINT64_CONSTANT (0x123456789abc7));
  MARSHAL_TEST (UINT64, DBUS_LITTLE_ENDIAN, DBUS_UINT64_CONSTANT (0x123456789abc7));
#endif /* DBUS_HAVE_INT64 */

  /* Marshal byte */
  MARSHAL_TEST (BYTE, DBUS_BIG_ENDIAN, 5);
  MARSHAL_TEST (BYTE, DBUS_LITTLE_ENDIAN, 5);

  /* Marshal all possible bools! */
  MARSHAL_TEST (BOOLEAN, DBUS_BIG_ENDIAN, FALSE);
  MARSHAL_TEST (BOOLEAN, DBUS_LITTLE_ENDIAN, FALSE);
  MARSHAL_TEST (BOOLEAN, DBUS_BIG_ENDIAN, TRUE);
  MARSHAL_TEST (BOOLEAN, DBUS_LITTLE_ENDIAN, TRUE);

  /* Marshal strings */
  MARSHAL_TEST_STRCMP (STRING, DBUS_BIG_ENDIAN, "");
  MARSHAL_TEST_STRCMP (STRING, DBUS_LITTLE_ENDIAN, "");
  MARSHAL_TEST_STRCMP (STRING, DBUS_BIG_ENDIAN, "This is the dbus test string");
  MARSHAL_TEST_STRCMP (STRING, DBUS_LITTLE_ENDIAN, "This is the dbus test string");

  /* object paths */
  MARSHAL_TEST_STRCMP (OBJECT_PATH, DBUS_BIG_ENDIAN, "/a/b/c");
  MARSHAL_TEST_STRCMP (OBJECT_PATH, DBUS_LITTLE_ENDIAN, "/a/b/c");

  /* signatures */
  MARSHAL_TEST_STRCMP (SIGNATURE, DBUS_BIG_ENDIAN, "");
  MARSHAL_TEST_STRCMP (SIGNATURE, DBUS_LITTLE_ENDIAN, "");
  MARSHAL_TEST_STRCMP (SIGNATURE, DBUS_BIG_ENDIAN, "a(ii)");
  MARSHAL_TEST_STRCMP (SIGNATURE, DBUS_LITTLE_ENDIAN, "a(ii)");

  /* Arrays */
  MARSHAL_TEST_FIXED_ARRAY (INT16, DBUS_BIG_ENDIAN, array2);
  MARSHAL_TEST_FIXED_ARRAY (INT16, DBUS_LITTLE_ENDIAN, array2);
  MARSHAL_TEST_FIXED_ARRAY (UINT16, DBUS_BIG_ENDIAN, array2);
  MARSHAL_TEST_FIXED_ARRAY (UINT16, DBUS_LITTLE_ENDIAN, array2);
  
  MARSHAL_TEST_FIXED_ARRAY (INT32, DBUS_BIG_ENDIAN, array4);
  MARSHAL_TEST_FIXED_ARRAY (INT32, DBUS_LITTLE_ENDIAN, array4);
  MARSHAL_TEST_FIXED_ARRAY (UINT32, DBUS_BIG_ENDIAN, array4);
  MARSHAL_TEST_FIXED_ARRAY (UINT32, DBUS_LITTLE_ENDIAN, array4);

  MARSHAL_TEST_FIXED_ARRAY (BYTE, DBUS_BIG_ENDIAN, array1);
  MARSHAL_TEST_FIXED_ARRAY (BYTE, DBUS_LITTLE_ENDIAN, array1);
  
#ifdef DBUS_HAVE_INT64
  MARSHAL_TEST_FIXED_ARRAY (INT64, DBUS_BIG_ENDIAN, array8);
  MARSHAL_TEST_FIXED_ARRAY (INT64, DBUS_LITTLE_ENDIAN, array8);
#endif

#if 0

  /*
   * FIXME restore the set/pack tests
   */

#ifdef DBUS_HAVE_INT64
  /* set/pack 64-bit integers */
  _dbus_string_set_length (&str, 8);

  /* signed little */
  _dbus_marshal_set_int64 (&str, DBUS_LITTLE_ENDIAN,
                           0, DBUS_INT64_CONSTANT (-0x123456789abc7));

  _dbus_assert (DBUS_INT64_CONSTANT (-0x123456789abc7) ==
                _dbus_unpack_int64 (DBUS_LITTLE_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* signed big */
  _dbus_marshal_set_int64 (&str, DBUS_BIG_ENDIAN,
                           0, DBUS_INT64_CONSTANT (-0x123456789abc7));

  _dbus_assert (DBUS_INT64_CONSTANT (-0x123456789abc7) ==
                _dbus_unpack_int64 (DBUS_BIG_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* signed little pack */
  _dbus_pack_int64 (DBUS_INT64_CONSTANT (-0x123456789abc7),
                    DBUS_LITTLE_ENDIAN,
                    _dbus_string_get_data (&str));

  _dbus_assert (DBUS_INT64_CONSTANT (-0x123456789abc7) ==
                _dbus_unpack_int64 (DBUS_LITTLE_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* signed big pack */
  _dbus_pack_int64 (DBUS_INT64_CONSTANT (-0x123456789abc7),
                    DBUS_BIG_ENDIAN,
                    _dbus_string_get_data (&str));

  _dbus_assert (DBUS_INT64_CONSTANT (-0x123456789abc7) ==
                _dbus_unpack_int64 (DBUS_BIG_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* unsigned little */
  _dbus_marshal_set_uint64 (&str, DBUS_LITTLE_ENDIAN,
                            0, DBUS_UINT64_CONSTANT (0x123456789abc7));

  _dbus_assert (DBUS_UINT64_CONSTANT (0x123456789abc7) ==
                _dbus_unpack_uint64 (DBUS_LITTLE_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

  /* unsigned big */
  _dbus_marshal_set_uint64 (&str, DBUS_BIG_ENDIAN,
                            0, DBUS_UINT64_CONSTANT (0x123456789abc7));

  _dbus_assert (DBUS_UINT64_CONSTANT (0x123456789abc7) ==
                _dbus_unpack_uint64 (DBUS_BIG_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

  /* unsigned little pack */
  _dbus_pack_uint64 (DBUS_UINT64_CONSTANT (0x123456789abc7),
                     DBUS_LITTLE_ENDIAN,
                     _dbus_string_get_data (&str));

  _dbus_assert (DBUS_UINT64_CONSTANT (0x123456789abc7) ==
                _dbus_unpack_uint64 (DBUS_LITTLE_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

  /* unsigned big pack */
  _dbus_pack_uint64 (DBUS_UINT64_CONSTANT (0x123456789abc7),
                     DBUS_BIG_ENDIAN,
                     _dbus_string_get_data (&str));

  _dbus_assert (DBUS_UINT64_CONSTANT (0x123456789abc7) ==
                _dbus_unpack_uint64 (DBUS_BIG_ENDIAN,
                                     _dbus_string_get_const_data (&str)));
#endif /* DBUS_HAVE_INT64 */

  /* set/pack 32-bit integers */
  _dbus_string_set_length (&str, 4);

  /* signed little */
  _dbus_marshal_set_int32 (&str, DBUS_LITTLE_ENDIAN,
                           0, -0x123456);

  _dbus_assert (-0x123456 ==
                _dbus_unpack_int32 (DBUS_LITTLE_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* signed big */
  _dbus_marshal_set_int32 (&str, DBUS_BIG_ENDIAN,
                           0, -0x123456);

  _dbus_assert (-0x123456 ==
                _dbus_unpack_int32 (DBUS_BIG_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* signed little pack */
  _dbus_pack_int32 (-0x123456,
                    DBUS_LITTLE_ENDIAN,
                    _dbus_string_get_data (&str));

  _dbus_assert (-0x123456 ==
                _dbus_unpack_int32 (DBUS_LITTLE_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* signed big pack */
  _dbus_pack_int32 (-0x123456,
                    DBUS_BIG_ENDIAN,
                    _dbus_string_get_data (&str));

  _dbus_assert (-0x123456 ==
                _dbus_unpack_int32 (DBUS_BIG_ENDIAN,
                                    _dbus_string_get_const_data (&str)));

  /* unsigned little */
  _dbus_marshal_set_uint32 (&str,
                            0, 0x123456,
                            DBUS_LITTLE_ENDIAN);

  _dbus_assert (0x123456 ==
                _dbus_unpack_uint32 (DBUS_LITTLE_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

  /* unsigned big */
  _dbus_marshal_set_uint32 (&str,
                            0, 0x123456,
                            DBUS_BIG_ENDIAN);

  _dbus_assert (0x123456 ==
                _dbus_unpack_uint32 (DBUS_BIG_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

  /* unsigned little pack */
  _dbus_pack_uint32 (0x123456,
                     DBUS_LITTLE_ENDIAN,
                     _dbus_string_get_data (&str));

  _dbus_assert (0x123456 ==
                _dbus_unpack_uint32 (DBUS_LITTLE_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

  /* unsigned big pack */
  _dbus_pack_uint32 (0x123456,
                     DBUS_BIG_ENDIAN,
                     _dbus_string_get_data (&str));

  _dbus_assert (0x123456 ==
                _dbus_unpack_uint32 (DBUS_BIG_ENDIAN,
                                     _dbus_string_get_const_data (&str)));

#endif /* set/pack tests for integers */

  /* Strings in-place set */
  byte_order = DBUS_LITTLE_ENDIAN;
  while (TRUE)
    {
      /* Init a string */
      _dbus_string_set_length (&str, 0);

      /* reset pos for the macros */
      pos = 0;

      MARSHAL_TEST_STRCMP (STRING, byte_order, "Hello world");

      /* Set it to something longer */
      _dbus_string_init_const (&t, "Hello world foo");

      v_STRING = _dbus_string_get_const_data (&t);
      _dbus_marshal_set_basic (&str, 0, DBUS_TYPE_STRING,
                               &v_STRING, byte_order, NULL, NULL);

      _dbus_marshal_read_basic (&str, 0, DBUS_TYPE_STRING,
                                &v_STRING, byte_order,
                                NULL);
      _dbus_assert (strcmp (v_STRING, "Hello world foo") == 0);

      /* Set it to something shorter */
      _dbus_string_init_const (&t, "Hello");

      v_STRING = _dbus_string_get_const_data (&t);
      _dbus_marshal_set_basic (&str, 0, DBUS_TYPE_STRING,
                               &v_STRING, byte_order, NULL, NULL);
      _dbus_marshal_read_basic (&str, 0, DBUS_TYPE_STRING,
                                &v_STRING, byte_order,
                                NULL);
      _dbus_assert (strcmp (v_STRING, "Hello") == 0);

      /* Do the other byte order */
      if (byte_order == DBUS_LITTLE_ENDIAN)
        byte_order = DBUS_BIG_ENDIAN;
      else
        break;
    }

  /* Clean up */
  _dbus_string_free (&str);

  return TRUE;
}

#endif /* DBUS_BUILD_TESTS */
