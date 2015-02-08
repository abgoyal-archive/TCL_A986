
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-marshal-recursive.h"
#include "dbus-marshal-basic.h"
#include "dbus-signature.h"
#include "dbus-internals.h"


/** turn this on to get deluged in TypeReader verbose spam */
#define RECURSIVE_MARSHAL_READ_TRACE  0

/** turn this on to get deluged in TypeWriter verbose spam */
#define RECURSIVE_MARSHAL_WRITE_TRACE 0

static void
free_fixups (DBusList **fixups)
{
  DBusList *link;

  link = _dbus_list_get_first_link (fixups);
  while (link != NULL)
    {
      DBusList *next;

      next = _dbus_list_get_next_link (fixups, link);

      dbus_free (link->data);
      _dbus_list_free_link (link);

      link = next;
    }

  *fixups = NULL;
}

static void
apply_and_free_fixups (DBusList      **fixups,
                       DBusTypeReader *reader)
{
  DBusList *link;

#if RECURSIVE_MARSHAL_WRITE_TRACE
  if (*fixups)
    _dbus_verbose (" %d FIXUPS to apply\n",
                   _dbus_list_get_length (fixups));
#endif

  link = _dbus_list_get_first_link (fixups);
  while (link != NULL)
    {
      DBusList *next;

      next = _dbus_list_get_next_link (fixups, link);

      if (reader)
        {
          DBusArrayLenFixup *f;

          f = link->data;

#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose (" applying FIXUP to reader %p at pos %d new_len = %d old len %d\n",
                         reader, f->len_pos_in_reader, f->new_len,
                         _dbus_marshal_read_uint32 (reader->value_str,
                                                    f->len_pos_in_reader,
                                                    reader->byte_order, NULL));
#endif

          _dbus_marshal_set_uint32 ((DBusString*) reader->value_str,
                                    f->len_pos_in_reader,
                                    f->new_len,
                                    reader->byte_order);
        }

      dbus_free (link->data);
      _dbus_list_free_link (link);

      link = next;
    }

  *fixups = NULL;
}

struct DBusTypeReaderClass
{
  const char *name;       /**< name for debugging */
  int         id;         /**< index in all_reader_classes */
  dbus_bool_t types_only; /**< only iterates over types, not values */
  void        (* recurse)          (DBusTypeReader        *sub,
                                    DBusTypeReader        *parent); /**< recurse with this reader as sub */
  dbus_bool_t (* check_finished)   (const DBusTypeReader  *reader); /**< check whether reader is at the end */
  void        (* next)             (DBusTypeReader        *reader,
                                    int                    current_type); /**< go to the next value */
};

static int
element_type_get_alignment (const DBusString *str,
                            int               pos)
{
  return _dbus_type_get_alignment (_dbus_first_type_in_signature (str, pos));
}

static void
reader_init (DBusTypeReader    *reader,
             int                byte_order,
             const DBusString  *type_str,
             int                type_pos,
             const DBusString  *value_str,
             int                value_pos)
{
  reader->byte_order = byte_order;
  reader->finished = FALSE;
  reader->type_str = type_str;
  reader->type_pos = type_pos;
  reader->value_str = value_str;
  reader->value_pos = value_pos;
}

static void
base_reader_recurse (DBusTypeReader *sub,
                     DBusTypeReader *parent)
{
  /* point subreader at the same place as parent */
  reader_init (sub,
               parent->byte_order,
               parent->type_str,
               parent->type_pos,
               parent->value_str,
               parent->value_pos);
}

static void
struct_or_dict_entry_types_only_reader_recurse (DBusTypeReader *sub,
                                                DBusTypeReader *parent)
{
  base_reader_recurse (sub, parent);
  
  _dbus_assert (_dbus_string_get_byte (sub->type_str,
                                       sub->type_pos) == DBUS_STRUCT_BEGIN_CHAR ||
                _dbus_string_get_byte (sub->type_str,
                                       sub->type_pos) == DBUS_DICT_ENTRY_BEGIN_CHAR);

  sub->type_pos += 1;
}

static void
struct_or_dict_entry_reader_recurse (DBusTypeReader *sub,
                                     DBusTypeReader *parent)
{
  struct_or_dict_entry_types_only_reader_recurse (sub, parent);

  /* struct and dict entry have 8 byte alignment */
  sub->value_pos = _DBUS_ALIGN_VALUE (sub->value_pos, 8);
}

static void
array_types_only_reader_recurse (DBusTypeReader *sub,
                                 DBusTypeReader *parent)
{
  base_reader_recurse (sub, parent);

  /* point type_pos at the array element type */
  sub->type_pos += 1;

  /* Init with values likely to crash things if misused */
  sub->u.array.start_pos = _DBUS_INT_MAX;
  sub->array_len_offset = 7;
}

#define ARRAY_READER_LEN_POS(reader) \
  ((reader)->u.array.start_pos - ((int)(reader)->array_len_offset) - 4)

static int
array_reader_get_array_len (const DBusTypeReader *reader)
{
  dbus_uint32_t array_len;
  int len_pos;

  len_pos = ARRAY_READER_LEN_POS (reader);

  _dbus_assert (_DBUS_ALIGN_VALUE (len_pos, 4) == (unsigned) len_pos);
  array_len = _dbus_unpack_uint32 (reader->byte_order,
                                   _dbus_string_get_const_data_len (reader->value_str, len_pos, 4));

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("   reader %p len_pos %d array len %u len_offset %d\n",
                 reader, len_pos, array_len, reader->array_len_offset);
#endif

  _dbus_assert (reader->u.array.start_pos - len_pos - 4 < 8);

  return array_len;
}

static void
array_reader_recurse (DBusTypeReader *sub,
                      DBusTypeReader *parent)
{
  int alignment;
  int len_pos;

  array_types_only_reader_recurse (sub, parent);

  sub->value_pos = _DBUS_ALIGN_VALUE (sub->value_pos, 4);

  len_pos = sub->value_pos;

  sub->value_pos += 4; /* for the length */

  alignment = element_type_get_alignment (sub->type_str,
                                          sub->type_pos);

  sub->value_pos = _DBUS_ALIGN_VALUE (sub->value_pos, alignment);

  sub->u.array.start_pos = sub->value_pos;
  _dbus_assert ((sub->u.array.start_pos - (len_pos + 4)) < 8); /* only 3 bits in array_len_offset */
  sub->array_len_offset = sub->u.array.start_pos - (len_pos + 4);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("    type reader %p array start = %d len_offset = %d array len = %d array element type = %s\n",
                 sub,
                 sub->u.array.start_pos,
                 sub->array_len_offset,
                 array_reader_get_array_len (sub),
                 _dbus_type_to_string (_dbus_first_type_in_signature (sub->type_str,
                                                                sub->type_pos)));
#endif
}

static void
variant_reader_recurse (DBusTypeReader *sub,
                        DBusTypeReader *parent)
{
  int sig_len;
  int contained_alignment;

  base_reader_recurse (sub, parent);

  /* Variant is 1 byte sig length (without nul), signature with nul,
   * padding to 8-boundary, then values
   */

  sig_len = _dbus_string_get_byte (sub->value_str, sub->value_pos);

  sub->type_str = sub->value_str;
  sub->type_pos = sub->value_pos + 1;

  sub->value_pos = sub->type_pos + sig_len + 1;

  contained_alignment = _dbus_type_get_alignment (_dbus_first_type_in_signature (sub->type_str,
                                                                           sub->type_pos));
  
  sub->value_pos = _DBUS_ALIGN_VALUE (sub->value_pos, contained_alignment);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("    type reader %p variant containing '%s'\n",
                 sub,
                 _dbus_string_get_const_data_len (sub->type_str,
                                                  sub->type_pos, 0));
#endif
}

static dbus_bool_t
array_reader_check_finished (const DBusTypeReader *reader)
{
  int end_pos;

  /* return the array element type if elements remain, and
   * TYPE_INVALID otherwise
   */

  end_pos = reader->u.array.start_pos + array_reader_get_array_len (reader);

  _dbus_assert (reader->value_pos <= end_pos);
  _dbus_assert (reader->value_pos >= reader->u.array.start_pos);

  return reader->value_pos == end_pos;
}

static void
skip_one_complete_type (const DBusString *type_str,
                        int              *type_pos)
{
  _dbus_type_signature_next (_dbus_string_get_const_data (type_str),
			     type_pos);
}

void
_dbus_type_signature_next (const char       *type_str,
			   int              *type_pos)
{
  const unsigned char *p;
  const unsigned char *start;

  _dbus_assert (type_str != NULL);
  _dbus_assert (type_pos != NULL);
  
  start = type_str;
  p = start + *type_pos;

  _dbus_assert (*p != DBUS_STRUCT_END_CHAR);
  _dbus_assert (*p != DBUS_DICT_ENTRY_END_CHAR);
  
  while (*p == DBUS_TYPE_ARRAY)
    ++p;

  _dbus_assert (*p != DBUS_STRUCT_END_CHAR);
  _dbus_assert (*p != DBUS_DICT_ENTRY_END_CHAR);
  
  if (*p == DBUS_STRUCT_BEGIN_CHAR)
    {
      int depth;

      depth = 1;

      while (TRUE)
        {
          _dbus_assert (*p != DBUS_TYPE_INVALID);

          ++p;

          _dbus_assert (*p != DBUS_TYPE_INVALID);

          if (*p == DBUS_STRUCT_BEGIN_CHAR)
            depth += 1;
          else if (*p == DBUS_STRUCT_END_CHAR)
            {
              depth -= 1;
              if (depth == 0)
                {
                  ++p;
                  break;
                }
            }
        }
    }
  else if (*p == DBUS_DICT_ENTRY_BEGIN_CHAR)
    {
      int depth;

      depth = 1;

      while (TRUE)
        {
          _dbus_assert (*p != DBUS_TYPE_INVALID);

          ++p;

          _dbus_assert (*p != DBUS_TYPE_INVALID);

          if (*p == DBUS_DICT_ENTRY_BEGIN_CHAR)
            depth += 1;
          else if (*p == DBUS_DICT_ENTRY_END_CHAR)
            {
              depth -= 1;
              if (depth == 0)
                {
                  ++p;
                  break;
                }
            }
        }
    }
  else
    {
      ++p;
    }

  *type_pos = (int) (p - start);
}

static int
find_len_of_complete_type (const DBusString *type_str,
                           int               type_pos)
{
  int end;

  end = type_pos;

  skip_one_complete_type (type_str, &end);

  return end - type_pos;
}

static void
base_reader_next (DBusTypeReader *reader,
                  int             current_type)
{
  switch (current_type)
    {
    case DBUS_TYPE_DICT_ENTRY:
    case DBUS_TYPE_STRUCT:
    case DBUS_TYPE_VARIANT:
      /* Scan forward over the entire container contents */
      {
        DBusTypeReader sub;

        if (reader->klass->types_only && current_type == DBUS_TYPE_VARIANT)
          ;
        else
          {
            /* Recurse into the struct or variant */
            _dbus_type_reader_recurse (reader, &sub);

            /* Skip everything in this subreader */
            while (_dbus_type_reader_next (&sub))
              {
                /* nothing */;
              }
          }
        if (!reader->klass->types_only)
          reader->value_pos = sub.value_pos;

        /* Now we are at the end of this container; for variants, the
         * subreader's type_pos is totally inapplicable (it's in the
         * value string) but we know that we increment by one past the
         * DBUS_TYPE_VARIANT
         */
        if (current_type == DBUS_TYPE_VARIANT)
          reader->type_pos += 1;
        else
          reader->type_pos = sub.type_pos;
      }
      break;

    case DBUS_TYPE_ARRAY:
      {
        if (!reader->klass->types_only)
          _dbus_marshal_skip_array (reader->value_str,
                                    _dbus_first_type_in_signature (reader->type_str,
                                                                   reader->type_pos + 1),
                                    reader->byte_order,
                                    &reader->value_pos);

        skip_one_complete_type (reader->type_str, &reader->type_pos);
      }
      break;

    default:
      if (!reader->klass->types_only)
        _dbus_marshal_skip_basic (reader->value_str,
                                  current_type, reader->byte_order,
                                  &reader->value_pos);

      reader->type_pos += 1;
      break;
    }
}

static void
struct_reader_next (DBusTypeReader *reader,
                    int             current_type)
{
  int t;

  base_reader_next (reader, current_type);

  /* for STRUCT containers we return FALSE at the end of the struct,
   * for INVALID we return FALSE at the end of the signature.
   * In both cases we arrange for get_current_type() to return INVALID
   * which is defined to happen iff we're at the end (no more next())
   */
  t = _dbus_string_get_byte (reader->type_str, reader->type_pos);
  if (t == DBUS_STRUCT_END_CHAR)
    {
      reader->type_pos += 1;
      reader->finished = TRUE;
    }
}

static void
dict_entry_reader_next (DBusTypeReader *reader,
                        int             current_type)
{
  int t;

  base_reader_next (reader, current_type);

  /* for STRUCT containers we return FALSE at the end of the struct,
   * for INVALID we return FALSE at the end of the signature.
   * In both cases we arrange for get_current_type() to return INVALID
   * which is defined to happen iff we're at the end (no more next())
   */
  t = _dbus_string_get_byte (reader->type_str, reader->type_pos);
  if (t == DBUS_DICT_ENTRY_END_CHAR)
    {
      reader->type_pos += 1;
      reader->finished = TRUE;
    }
}

static void
array_types_only_reader_next (DBusTypeReader *reader,
                              int             current_type)
{
  /* We have one "element" to be iterated over
   * in each array, which is its element type.
   * So the finished flag indicates whether we've
   * iterated over it yet or not.
   */
  reader->finished = TRUE;
}

static void
array_reader_next (DBusTypeReader *reader,
                   int             current_type)
{
  /* Skip one array element */
  int end_pos;

  end_pos = reader->u.array.start_pos + array_reader_get_array_len (reader);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  reader %p array next START start_pos = %d end_pos = %d value_pos = %d current_type = %s\n",
                 reader,
                 reader->u.array.start_pos,
                 end_pos, reader->value_pos,
                 _dbus_type_to_string (current_type));
#endif

  _dbus_assert (reader->value_pos < end_pos);
  _dbus_assert (reader->value_pos >= reader->u.array.start_pos);

  switch (_dbus_first_type_in_signature (reader->type_str,
                                         reader->type_pos))
    {
    case DBUS_TYPE_DICT_ENTRY:
    case DBUS_TYPE_STRUCT:
    case DBUS_TYPE_VARIANT:
      {
        DBusTypeReader sub;

        /* Recurse into the struct or variant */
        _dbus_type_reader_recurse (reader, &sub);

        /* Skip everything in this element */
        while (_dbus_type_reader_next (&sub))
          {
            /* nothing */;
          }

        /* Now we are at the end of this element */
        reader->value_pos = sub.value_pos;
      }
      break;

    case DBUS_TYPE_ARRAY:
      {
        _dbus_marshal_skip_array (reader->value_str,
                                  _dbus_first_type_in_signature (reader->type_str,
                                                           reader->type_pos + 1),
                                  reader->byte_order,
                                  &reader->value_pos);
      }
      break;

    default:
      {
        _dbus_marshal_skip_basic (reader->value_str,
                                  current_type, reader->byte_order,
                                  &reader->value_pos);
      }
      break;
    }

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  reader %p array next END start_pos = %d end_pos = %d value_pos = %d current_type = %s\n",
                 reader,
                 reader->u.array.start_pos,
                 end_pos, reader->value_pos,
                 _dbus_type_to_string (current_type));
#endif

  _dbus_assert (reader->value_pos <= end_pos);

  if (reader->value_pos == end_pos)
    {
      skip_one_complete_type (reader->type_str,
                              &reader->type_pos);
    }
}

static const DBusTypeReaderClass body_reader_class = {
  "body", 0,
  FALSE,
  NULL, /* body is always toplevel, so doesn't get recursed into */
  NULL,
  base_reader_next
};

static const DBusTypeReaderClass body_types_only_reader_class = {
  "body types", 1,
  TRUE,
  NULL, /* body is always toplevel, so doesn't get recursed into */
  NULL,
  base_reader_next
};

static const DBusTypeReaderClass struct_reader_class = {
  "struct", 2,
  FALSE,
  struct_or_dict_entry_reader_recurse,
  NULL,
  struct_reader_next
};

static const DBusTypeReaderClass struct_types_only_reader_class = {
  "struct types", 3,
  TRUE,
  struct_or_dict_entry_types_only_reader_recurse,
  NULL,
  struct_reader_next
};

static const DBusTypeReaderClass dict_entry_reader_class = {
  "dict_entry", 4,
  FALSE,
  struct_or_dict_entry_reader_recurse,
  NULL,
  dict_entry_reader_next
};

static const DBusTypeReaderClass dict_entry_types_only_reader_class = {
  "dict_entry types", 5,
  TRUE,
  struct_or_dict_entry_types_only_reader_recurse,
  NULL,
  dict_entry_reader_next
};

static const DBusTypeReaderClass array_reader_class = {
  "array", 6,
  FALSE,
  array_reader_recurse,
  array_reader_check_finished,
  array_reader_next
};

static const DBusTypeReaderClass array_types_only_reader_class = {
  "array types", 7,
  TRUE,
  array_types_only_reader_recurse,
  NULL,
  array_types_only_reader_next
};

static const DBusTypeReaderClass variant_reader_class = {
  "variant", 8,
  FALSE,
  variant_reader_recurse,
  NULL,
  base_reader_next
};

static const DBusTypeReaderClass const *
all_reader_classes[] = {
  &body_reader_class,
  &body_types_only_reader_class,
  &struct_reader_class,
  &struct_types_only_reader_class,
  &dict_entry_reader_class,
  &dict_entry_types_only_reader_class,
  &array_reader_class,
  &array_types_only_reader_class,
  &variant_reader_class
};

void
_dbus_type_reader_init (DBusTypeReader    *reader,
                        int                byte_order,
                        const DBusString  *type_str,
                        int                type_pos,
                        const DBusString  *value_str,
                        int                value_pos)
{
  reader->klass = &body_reader_class;

  reader_init (reader, byte_order, type_str, type_pos,
               value_str, value_pos);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p init type_pos = %d value_pos = %d remaining sig '%s'\n",
                 reader, reader->type_pos, reader->value_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0));
#endif
}

void
_dbus_type_reader_init_types_only (DBusTypeReader    *reader,
                                   const DBusString  *type_str,
                                   int                type_pos)
{
  reader->klass = &body_types_only_reader_class;

  reader_init (reader, DBUS_COMPILER_BYTE_ORDER /* irrelevant */,
               type_str, type_pos, NULL, _DBUS_INT_MAX /* crashes if we screw up */);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p init types only type_pos = %d remaining sig '%s'\n",
                 reader, reader->type_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0));
#endif
}

int
_dbus_type_reader_get_current_type (const DBusTypeReader *reader)
{
  int t;

  if (reader->finished ||
      (reader->klass->check_finished &&
       (* reader->klass->check_finished) (reader)))
    t = DBUS_TYPE_INVALID;
  else
    t = _dbus_first_type_in_signature (reader->type_str,
                                       reader->type_pos);

  _dbus_assert (t != DBUS_STRUCT_END_CHAR);
  _dbus_assert (t != DBUS_STRUCT_BEGIN_CHAR);
  _dbus_assert (t != DBUS_DICT_ENTRY_END_CHAR);
  _dbus_assert (t != DBUS_DICT_ENTRY_BEGIN_CHAR);
  
#if 0
  _dbus_verbose ("  type reader %p current type_pos = %d type = %s\n",
                 reader, reader->type_pos,
                 _dbus_type_to_string (t));
#endif

  return t;
}

int
_dbus_type_reader_get_element_type (const DBusTypeReader  *reader)
{
  int element_type;

  _dbus_assert (_dbus_type_reader_get_current_type (reader) == DBUS_TYPE_ARRAY);

  element_type = _dbus_first_type_in_signature (reader->type_str,
                                          reader->type_pos + 1);

  return element_type;
}

int
_dbus_type_reader_get_value_pos (const DBusTypeReader  *reader)
{
  return reader->value_pos;
}

void
_dbus_type_reader_read_raw (const DBusTypeReader  *reader,
                            const unsigned char  **value_location)
{
  _dbus_assert (!reader->klass->types_only);

  *value_location = _dbus_string_get_const_data_len (reader->value_str,
                                                     reader->value_pos,
                                                     0);
}

void
_dbus_type_reader_read_basic (const DBusTypeReader    *reader,
                              void                    *value)
{
  int t;

  _dbus_assert (!reader->klass->types_only);

  t = _dbus_type_reader_get_current_type (reader);

  _dbus_marshal_read_basic (reader->value_str,
                            reader->value_pos,
                            t, value,
                            reader->byte_order,
                            NULL);


#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p read basic type_pos = %d value_pos = %d remaining sig '%s'\n",
                 reader, reader->type_pos, reader->value_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0));
#endif
}

int
_dbus_type_reader_get_array_length (const DBusTypeReader  *reader)
{
  _dbus_assert (!reader->klass->types_only);
  _dbus_assert (reader->klass == &array_reader_class);

  return array_reader_get_array_len (reader);
}

void
_dbus_type_reader_read_fixed_multi (const DBusTypeReader  *reader,
                                    void                  *value,
                                    int                   *n_elements)
{
  int element_type;
  int end_pos;
  int remaining_len;
  int alignment;
  int total_len;

  _dbus_assert (!reader->klass->types_only);
  _dbus_assert (reader->klass == &array_reader_class);

  element_type = _dbus_first_type_in_signature (reader->type_str,
                                                reader->type_pos);

  _dbus_assert (element_type != DBUS_TYPE_INVALID); /* why we don't use get_current_type() */
  _dbus_assert (dbus_type_is_fixed (element_type));

  alignment = _dbus_type_get_alignment (element_type);

  _dbus_assert (reader->value_pos >= reader->u.array.start_pos);

  total_len = array_reader_get_array_len (reader);
  end_pos = reader->u.array.start_pos + total_len;
  remaining_len = end_pos - reader->value_pos;

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("end_pos %d total_len %d remaining_len %d value_pos %d\n",
                 end_pos, total_len, remaining_len, reader->value_pos);
#endif

  _dbus_assert (remaining_len <= total_len);

  if (remaining_len == 0)
    *(const DBusBasicValue**) value = NULL;
  else
    *(const DBusBasicValue**) value =
      (void*) _dbus_string_get_const_data_len (reader->value_str,
                                               reader->value_pos,
                                               remaining_len);

  *n_elements = remaining_len / alignment;
  _dbus_assert ((remaining_len % alignment) == 0);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p read fixed array type_pos = %d value_pos = %d remaining sig '%s'\n",
                 reader, reader->type_pos, reader->value_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0));
#endif
}

void
_dbus_type_reader_recurse (DBusTypeReader *reader,
                           DBusTypeReader *sub)
{
  int t;

  t = _dbus_first_type_in_signature (reader->type_str, reader->type_pos);

  switch (t)
    {
    case DBUS_TYPE_STRUCT:
      if (reader->klass->types_only)
        sub->klass = &struct_types_only_reader_class;
      else
        sub->klass = &struct_reader_class;
      break;
    case DBUS_TYPE_DICT_ENTRY:
      if (reader->klass->types_only)
        sub->klass = &dict_entry_types_only_reader_class;
      else
        sub->klass = &dict_entry_reader_class;
      break;
    case DBUS_TYPE_ARRAY:
      if (reader->klass->types_only)
        sub->klass = &array_types_only_reader_class;
      else
        sub->klass = &array_reader_class;
      break;
    case DBUS_TYPE_VARIANT:
      if (reader->klass->types_only)
        _dbus_assert_not_reached ("can't recurse into variant typecode");
      else
        sub->klass = &variant_reader_class;
      break;
    default:
      _dbus_verbose ("recursing into type %s\n", _dbus_type_to_string (t));
#ifndef DBUS_DISABLE_CHECKS
      if (t == DBUS_TYPE_INVALID)
        _dbus_warn_check_failed ("You can't recurse into an empty array or off the end of a message body\n");
#endif /* DBUS_DISABLE_CHECKS */

      _dbus_assert_not_reached ("don't yet handle recursing into this type");
    }

  _dbus_assert (sub->klass == all_reader_classes[sub->klass->id]);

  (* sub->klass->recurse) (sub, reader);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p RECURSED type_pos = %d value_pos = %d remaining sig '%s'\n",
                 sub, sub->type_pos, sub->value_pos,
                 _dbus_string_get_const_data_len (sub->type_str, sub->type_pos, 0));
#endif
}

dbus_bool_t
_dbus_type_reader_next (DBusTypeReader *reader)
{
  int t;

  t = _dbus_type_reader_get_current_type (reader);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p START next() { type_pos = %d value_pos = %d remaining sig '%s' current_type = %s\n",
                 reader, reader->type_pos, reader->value_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0),
                 _dbus_type_to_string (t));
#endif

  if (t == DBUS_TYPE_INVALID)
    return FALSE;

  (* reader->klass->next) (reader, t);

#if RECURSIVE_MARSHAL_READ_TRACE
  _dbus_verbose ("  type reader %p END next() type_pos = %d value_pos = %d remaining sig '%s' current_type = %s\n",
                 reader, reader->type_pos, reader->value_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0),
                 _dbus_type_to_string (_dbus_type_reader_get_current_type (reader)));
#endif

  return _dbus_type_reader_get_current_type (reader) != DBUS_TYPE_INVALID;
}

dbus_bool_t
_dbus_type_reader_has_next (const DBusTypeReader *reader)
{
  /* Not efficient but works for now. */
  DBusTypeReader copy;

  copy = *reader;
  return _dbus_type_reader_next (&copy);
}

void
_dbus_type_reader_get_signature (const DBusTypeReader  *reader,
                                 const DBusString     **str_p,
                                 int                   *start_p,
                                 int                   *len_p)
{
  *str_p = reader->type_str;
  *start_p = reader->type_pos;
  *len_p = find_len_of_complete_type (reader->type_str, reader->type_pos);
}

typedef struct
{
  DBusString replacement; /**< Marshaled value including alignment padding */
  int padding;            /**< How much of the replacement block is padding */
} ReplacementBlock;

static dbus_bool_t
replacement_block_init (ReplacementBlock *block,
                        DBusTypeReader   *reader)
{
  if (!_dbus_string_init (&block->replacement))
    return FALSE;

  /* % 8 is the padding to have the same align properties in
   * our replacement string as we do at the position being replaced
   */
  block->padding = reader->value_pos % 8;

  if (!_dbus_string_lengthen (&block->replacement, block->padding))
    goto oom;

  return TRUE;

 oom:
  _dbus_string_free (&block->replacement);
  return FALSE;
}

static dbus_bool_t
replacement_block_replace (ReplacementBlock     *block,
                           DBusTypeReader       *reader,
                           const DBusTypeReader *realign_root)
{
  DBusTypeWriter writer;
  DBusTypeReader realign_reader;
  DBusList *fixups;
  int orig_len;

  _dbus_assert (realign_root != NULL);

  orig_len = _dbus_string_get_length (&block->replacement);

  realign_reader = *realign_root;

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("INITIALIZING replacement block writer %p at value_pos %d\n",
                 &writer, _dbus_string_get_length (&block->replacement));
#endif
  _dbus_type_writer_init_values_only (&writer,
                                      realign_reader.byte_order,
                                      realign_reader.type_str,
                                      realign_reader.type_pos,
                                      &block->replacement,
                                      _dbus_string_get_length (&block->replacement));

  _dbus_assert (realign_reader.value_pos <= reader->value_pos);

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("COPYING from reader at value_pos %d to writer %p starting after value_pos %d\n",
                 realign_reader.value_pos, &writer, reader->value_pos);
#endif
  fixups = NULL;
  if (!_dbus_type_writer_write_reader_partial (&writer,
                                               &realign_reader,
                                               reader,
                                               block->padding,
                                               _dbus_string_get_length (&block->replacement) - block->padding,
                                               &fixups))
    goto oom;

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("REPLACEMENT at padding %d len %d\n", block->padding,
                 _dbus_string_get_length (&block->replacement) - block->padding);
  _dbus_verbose_bytes_of_string (&block->replacement, block->padding,
                                 _dbus_string_get_length (&block->replacement) - block->padding);
  _dbus_verbose ("TO BE REPLACED at value_pos = %d (align pad %d) len %d realign_reader.value_pos %d\n",
                 reader->value_pos, reader->value_pos % 8,
                 realign_reader.value_pos - reader->value_pos,
                 realign_reader.value_pos);
  _dbus_verbose_bytes_of_string (reader->value_str,
                                 reader->value_pos,
                                 realign_reader.value_pos - reader->value_pos);
#endif

  /* Move the replacement into position
   * (realign_reader should now be at the end of the block to be replaced)
   */
  if (!_dbus_string_replace_len (&block->replacement, block->padding,
                                 _dbus_string_get_length (&block->replacement) - block->padding,
                                 (DBusString*) reader->value_str,
                                 reader->value_pos,
                                 realign_reader.value_pos - reader->value_pos))
    goto oom;

  /* Process our fixups now that we can't have an OOM error */
  apply_and_free_fixups (&fixups, reader);

  return TRUE;

 oom:
  _dbus_string_set_length (&block->replacement, orig_len);
  free_fixups (&fixups);
  return FALSE;
}

static void
replacement_block_free (ReplacementBlock *block)
{
  _dbus_string_free (&block->replacement);
}

static dbus_bool_t
reader_set_basic_variable_length (DBusTypeReader       *reader,
                                  int                   current_type,
                                  const void           *value,
                                  const DBusTypeReader *realign_root)
{
  dbus_bool_t retval;
  ReplacementBlock block;
  DBusTypeWriter writer;

  _dbus_assert (realign_root != NULL);

  retval = FALSE;

  if (!replacement_block_init (&block, reader))
    return FALSE;

  /* Write the new basic value */
#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("INITIALIZING writer %p to write basic value at value_pos %d of replacement string\n",
                 &writer, _dbus_string_get_length (&block.replacement));
#endif
  _dbus_type_writer_init_values_only (&writer,
                                      reader->byte_order,
                                      reader->type_str,
                                      reader->type_pos,
                                      &block.replacement,
                                      _dbus_string_get_length (&block.replacement));
#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("WRITING basic value to writer %p (replacement string)\n", &writer);
#endif
  if (!_dbus_type_writer_write_basic (&writer, current_type, value))
    goto out;

  if (!replacement_block_replace (&block,
                                  reader,
                                  realign_root))
    goto out;

  retval = TRUE;

 out:
  replacement_block_free (&block);
  return retval;
}

static void
reader_set_basic_fixed_length (DBusTypeReader *reader,
                               int             current_type,
                               const void     *value)
{
  _dbus_marshal_set_basic ((DBusString*) reader->value_str,
                           reader->value_pos,
                           current_type,
                           value,
                           reader->byte_order,
                           NULL, NULL);
}

dbus_bool_t
_dbus_type_reader_set_basic (DBusTypeReader       *reader,
                             const void           *value,
                             const DBusTypeReader *realign_root)
{
  int current_type;

  _dbus_assert (!reader->klass->types_only);
  _dbus_assert (reader->value_str == realign_root->value_str);
  _dbus_assert (reader->value_pos >= realign_root->value_pos);

  current_type = _dbus_type_reader_get_current_type (reader);

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  SET BASIC type reader %p type_pos = %d value_pos = %d remaining sig '%s' realign_root = %p with value_pos %d current_type = %s\n",
                 reader, reader->type_pos, reader->value_pos,
                 _dbus_string_get_const_data_len (reader->type_str, reader->type_pos, 0),
                 realign_root,
                 realign_root ? realign_root->value_pos : -1,
                 _dbus_type_to_string (current_type));
  _dbus_verbose_bytes_of_string (realign_root->value_str, realign_root->value_pos,
                                 _dbus_string_get_length (realign_root->value_str) -
                                 realign_root->value_pos);
#endif

  _dbus_assert (dbus_type_is_basic (current_type));

  if (dbus_type_is_fixed (current_type))
    {
      reader_set_basic_fixed_length (reader, current_type, value);
      return TRUE;
    }
  else
    {
      _dbus_assert (realign_root != NULL);
      return reader_set_basic_variable_length (reader, current_type,
                                               value, realign_root);
    }
}

dbus_bool_t
_dbus_type_reader_delete (DBusTypeReader        *reader,
                          const DBusTypeReader  *realign_root)
{
  dbus_bool_t retval;
  ReplacementBlock block;

  _dbus_assert (realign_root != NULL);
  _dbus_assert (reader->klass == &array_reader_class);

  retval = FALSE;

  if (!replacement_block_init (&block, reader))
    return FALSE;

  if (!replacement_block_replace (&block,
                                  reader,
                                  realign_root))
    goto out;

  retval = TRUE;

 out:
  replacement_block_free (&block);
  return retval;
}

dbus_bool_t
_dbus_type_reader_greater_than (const DBusTypeReader  *lhs,
                                const DBusTypeReader  *rhs)
{
  _dbus_assert (lhs->value_str == rhs->value_str);

  return lhs->value_pos > rhs->value_pos;
}


void
_dbus_type_writer_init (DBusTypeWriter *writer,
                        int             byte_order,
                        DBusString     *type_str,
                        int             type_pos,
                        DBusString     *value_str,
                        int             value_pos)
{
  writer->byte_order = byte_order;
  writer->type_str = type_str;
  writer->type_pos = type_pos;
  writer->value_str = value_str;
  writer->value_pos = value_pos;
  writer->container_type = DBUS_TYPE_INVALID;
  writer->type_pos_is_expectation = FALSE;
  writer->enabled = TRUE;

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("writer %p init remaining sig '%s'\n", writer,
                 writer->type_str ?
                 _dbus_string_get_const_data_len (writer->type_str, writer->type_pos, 0) :
                 "unknown");
#endif
}

void
_dbus_type_writer_init_types_delayed (DBusTypeWriter *writer,
                                      int             byte_order,
                                      DBusString     *value_str,
                                      int             value_pos)
{
  _dbus_type_writer_init (writer, byte_order,
                          NULL, 0, value_str, value_pos);
}

void
_dbus_type_writer_add_types (DBusTypeWriter *writer,
                             DBusString     *type_str,
                             int             type_pos)
{
  if (writer->type_str == NULL) /* keeps us from using this as setter */
    {
      writer->type_str = type_str;
      writer->type_pos = type_pos;
    }
}

void
_dbus_type_writer_remove_types (DBusTypeWriter *writer)
{
  writer->type_str = NULL;
  writer->type_pos = -1;
}

void
_dbus_type_writer_init_values_only (DBusTypeWriter   *writer,
                                    int               byte_order,
                                    const DBusString *type_str,
                                    int               type_pos,
                                    DBusString       *value_str,
                                    int               value_pos)
{
  _dbus_type_writer_init (writer, byte_order,
                          (DBusString*)type_str, type_pos,
                          value_str, value_pos);

  writer->type_pos_is_expectation = TRUE;
}

static dbus_bool_t
_dbus_type_writer_write_basic_no_typecode (DBusTypeWriter *writer,
                                           int             type,
                                           const void     *value)
{
  if (writer->enabled)
    return _dbus_marshal_write_basic (writer->value_str,
                                      writer->value_pos,
                                      type,
                                      value,
                                      writer->byte_order,
                                      &writer->value_pos);
  else
    return TRUE;
}

static void
writer_recurse_init_and_check (DBusTypeWriter *writer,
                               int             container_type,
                               DBusTypeWriter *sub)
{
  _dbus_type_writer_init (sub,
                          writer->byte_order,
                          writer->type_str,
                          writer->type_pos,
                          writer->value_str,
                          writer->value_pos);

  sub->container_type = container_type;

  if (writer->type_pos_is_expectation ||
      (sub->container_type == DBUS_TYPE_ARRAY || sub->container_type == DBUS_TYPE_VARIANT))
    sub->type_pos_is_expectation = TRUE;
  else
    sub->type_pos_is_expectation = FALSE;

  sub->enabled = writer->enabled;

#ifndef DBUS_DISABLE_CHECKS
  if (writer->type_pos_is_expectation && writer->type_str)
    {
      int expected;

      expected = _dbus_first_type_in_signature (writer->type_str, writer->type_pos);

      if (expected != sub->container_type)
        {
          _dbus_warn_check_failed ("Writing an element of type %s, but the expected type here is %s\n",
                                   _dbus_type_to_string (sub->container_type),
                                   _dbus_type_to_string (expected));
          _dbus_assert_not_reached ("bad array element or variant content written");
        }
    }
#endif /* DBUS_DISABLE_CHECKS */

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p recurse parent %s type_pos = %d value_pos = %d is_expectation = %d remaining sig '%s' enabled = %d\n",
                 writer,
                 _dbus_type_to_string (writer->container_type),
                 writer->type_pos, writer->value_pos, writer->type_pos_is_expectation,
                 writer->type_str ?
                 _dbus_string_get_const_data_len (writer->type_str, writer->type_pos, 0) :
                 "unknown",
                 writer->enabled);
  _dbus_verbose ("  type writer %p recurse sub %s   type_pos = %d value_pos = %d is_expectation = %d enabled = %d\n",
                 sub,
                 _dbus_type_to_string (sub->container_type),
                 sub->type_pos, sub->value_pos,
                 sub->type_pos_is_expectation,
                 sub->enabled);
#endif
}

static dbus_bool_t
write_or_verify_typecode (DBusTypeWriter *writer,
                          int             typecode)
{
  /* A subwriter inside an array or variant will have type_pos
   * pointing to the expected typecode; a writer not inside an array
   * or variant has type_pos pointing to the next place to insert a
   * typecode.
   */
#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p write_or_verify start type_pos = %d remaining sig '%s' enabled = %d\n",
                 writer, writer->type_pos,
                 writer->type_str ?
                 _dbus_string_get_const_data_len (writer->type_str, writer->type_pos, 0) :
                 "unknown",
                 writer->enabled);
#endif

  if (writer->type_str == NULL)
    return TRUE;

  if (writer->type_pos_is_expectation)
    {
#ifndef DBUS_DISABLE_CHECKS
      {
        int expected;

        expected = _dbus_string_get_byte (writer->type_str, writer->type_pos);

        if (expected != typecode)
          {
            _dbus_warn_check_failed ("Array or variant type requires that type %s be written, but %s was written\n",
                                     _dbus_type_to_string (expected), _dbus_type_to_string (typecode));
            _dbus_assert_not_reached ("bad type inserted somewhere inside an array or variant");
          }
      }
#endif /* DBUS_DISABLE_CHECKS */

      /* if immediately inside an array we'd always be appending an element,
       * so the expected type doesn't change; if inside a struct or something
       * below an array, we need to move through said struct or something.
       */
      if (writer->container_type != DBUS_TYPE_ARRAY)
        writer->type_pos += 1;
    }
  else
    {
      if (!_dbus_string_insert_byte (writer->type_str,
                                     writer->type_pos,
                                     typecode))
        return FALSE;

      writer->type_pos += 1;
    }

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p write_or_verify end type_pos = %d remaining sig '%s'\n",
                 writer, writer->type_pos,
                 _dbus_string_get_const_data_len (writer->type_str, writer->type_pos, 0));
#endif

  return TRUE;
}

static dbus_bool_t
writer_recurse_struct_or_dict_entry (DBusTypeWriter   *writer,
                                     int               begin_char,
                                     const DBusString *contained_type,
                                     int               contained_type_start,
                                     int               contained_type_len,
                                     DBusTypeWriter   *sub)
{
  /* FIXME right now contained_type is ignored; we could probably
   * almost trivially fix the code so if it's present we
   * write it out and then set type_pos_is_expectation
   */

  /* Ensure that we'll be able to add alignment padding and the typecode */
  if (writer->enabled)
    {
      if (!_dbus_string_alloc_space (sub->value_str, 8))
        return FALSE;
    }

  if (!write_or_verify_typecode (sub, begin_char))
    _dbus_assert_not_reached ("failed to insert struct typecode after prealloc");

  if (writer->enabled)
    {
      if (!_dbus_string_insert_bytes (sub->value_str,
                                      sub->value_pos,
                                      _DBUS_ALIGN_VALUE (sub->value_pos, 8) - sub->value_pos,
                                      '\0'))
        _dbus_assert_not_reached ("should not have failed to insert alignment padding for struct");
      sub->value_pos = _DBUS_ALIGN_VALUE (sub->value_pos, 8);
    }

  return TRUE;
}


static dbus_bool_t
writer_recurse_array (DBusTypeWriter   *writer,
                      const DBusString *contained_type,
                      int               contained_type_start,
                      int               contained_type_len,
                      DBusTypeWriter   *sub,
                      dbus_bool_t       is_array_append)
{
  dbus_uint32_t value = 0;
  int alignment;
  int aligned;

#ifndef DBUS_DISABLE_CHECKS
  if (writer->container_type == DBUS_TYPE_ARRAY &&
      writer->type_str)
    {
      if (!_dbus_string_equal_substring (contained_type,
                                         contained_type_start,
                                         contained_type_len,
                                         writer->type_str,
                                         writer->u.array.element_type_pos + 1))
        {
          _dbus_warn_check_failed ("Writing an array of '%s' but this is incompatible with the expected type of elements in the parent array\n",
                                   _dbus_string_get_const_data_len (contained_type,
                                                                    contained_type_start,
                                                                    contained_type_len));
          _dbus_assert_not_reached ("incompatible type for child array");
        }
    }
#endif /* DBUS_DISABLE_CHECKS */

  if (writer->enabled && !is_array_append)
    {
      /* 3 pad + 4 bytes for the array length, and 4 bytes possible padding
       * before array values
       */
      if (!_dbus_string_alloc_space (sub->value_str, 3 + 4 + 4))
        return FALSE;
    }

  if (writer->type_str != NULL)
    {
      sub->type_pos += 1; /* move to point to the element type, since type_pos
                           * should be the expected type for further writes
                           */
      sub->u.array.element_type_pos = sub->type_pos;
    }

  if (!writer->type_pos_is_expectation)
    {
      /* sub is a toplevel/outermost array so we need to write the type data */

      /* alloc space for array typecode, element signature */
      if (!_dbus_string_alloc_space (writer->type_str, 1 + contained_type_len))
        return FALSE;

      if (!_dbus_string_insert_byte (writer->type_str,
                                     writer->type_pos,
                                     DBUS_TYPE_ARRAY))
        _dbus_assert_not_reached ("failed to insert array typecode after prealloc");

      if (!_dbus_string_copy_len (contained_type,
                                  contained_type_start, contained_type_len,
                                  sub->type_str,
                                  sub->u.array.element_type_pos))
        _dbus_assert_not_reached ("should not have failed to insert array element typecodes");
    }

  if (writer->type_str != NULL)
    {
      /* If the parent is an array, we hold type_pos pointing at the array element type;
       * otherwise advance it to reflect the array value we just recursed into
       */
      if (writer->container_type != DBUS_TYPE_ARRAY)
        writer->type_pos += 1 + contained_type_len;
      else
        _dbus_assert (writer->type_pos_is_expectation); /* because it's an array */
    }

  if (writer->enabled)
    {
      /* Write (or jump over, if is_array_append) the length */
      sub->u.array.len_pos = _DBUS_ALIGN_VALUE (sub->value_pos, 4);

      if (is_array_append)
        {
          sub->value_pos += 4;
        }
      else
        {
          if (!_dbus_type_writer_write_basic_no_typecode (sub, DBUS_TYPE_UINT32,
                                                          &value))
            _dbus_assert_not_reached ("should not have failed to insert array len");
        }

      _dbus_assert (sub->u.array.len_pos == sub->value_pos - 4);

      /* Write alignment padding for array elements
       * Note that we write the padding *even for empty arrays*
       * to avoid wonky special cases
       */
      alignment = element_type_get_alignment (contained_type, contained_type_start);

      aligned = _DBUS_ALIGN_VALUE (sub->value_pos, alignment);
      if (aligned != sub->value_pos)
        {
          if (!is_array_append)
            {
              if (!_dbus_string_insert_bytes (sub->value_str,
                                              sub->value_pos,
                                              aligned - sub->value_pos,
                                              '\0'))
                _dbus_assert_not_reached ("should not have failed to insert alignment padding");
            }

          sub->value_pos = aligned;
        }

      sub->u.array.start_pos = sub->value_pos;

      if (is_array_append)
        {
          dbus_uint32_t len;

          _dbus_assert (_DBUS_ALIGN_VALUE (sub->u.array.len_pos, 4) ==
                        (unsigned) sub->u.array.len_pos);
          len = _dbus_unpack_uint32 (sub->byte_order,
                                     _dbus_string_get_const_data_len (sub->value_str,
                                                                      sub->u.array.len_pos,
                                                                      4));

          sub->value_pos += len;
        }
    }
  else
    {
      /* not enabled, so we won't write the len_pos; set it to -1 to so indicate */
      sub->u.array.len_pos = -1;
      sub->u.array.start_pos = sub->value_pos;
    }

  _dbus_assert (sub->u.array.len_pos < sub->u.array.start_pos);
  _dbus_assert (is_array_append || sub->u.array.start_pos == sub->value_pos);

#if RECURSIVE_MARSHAL_WRITE_TRACE
      _dbus_verbose ("  type writer %p recurse array done remaining sig '%s' array start_pos = %d len_pos = %d value_pos = %d\n", sub,
                     sub->type_str ?
                     _dbus_string_get_const_data_len (sub->type_str, sub->type_pos, 0) :
                     "unknown",
                     sub->u.array.start_pos, sub->u.array.len_pos, sub->value_pos);
#endif

  return TRUE;
}

static dbus_bool_t
writer_recurse_variant (DBusTypeWriter   *writer,
                        const DBusString *contained_type,
                        int               contained_type_start,
                        int               contained_type_len,
                        DBusTypeWriter   *sub)
{
  int contained_alignment;
  
  if (writer->enabled)
    {
      /* Allocate space for the worst case, which is 1 byte sig
       * length, nul byte at end of sig, and 7 bytes padding to
       * 8-boundary.
       */
      if (!_dbus_string_alloc_space (sub->value_str, contained_type_len + 9))
        return FALSE;
    }

  /* write VARIANT typecode to the parent's type string */
  if (!write_or_verify_typecode (writer, DBUS_TYPE_VARIANT))
    return FALSE;

  /* If not enabled, mark that we have no type_str anymore ... */

  if (!writer->enabled)
    {
      sub->type_str = NULL;
      sub->type_pos = -1;

      return TRUE;
    }

  /* If we're enabled then continue ... */

  if (!_dbus_string_insert_byte (sub->value_str,
                                 sub->value_pos,
                                 contained_type_len))
    _dbus_assert_not_reached ("should not have failed to insert variant type sig len");

  sub->value_pos += 1;

  /* Here we switch over to the expected type sig we're about to write */
  sub->type_str = sub->value_str;
  sub->type_pos = sub->value_pos;

  if (!_dbus_string_copy_len (contained_type, contained_type_start, contained_type_len,
                              sub->value_str, sub->value_pos))
    _dbus_assert_not_reached ("should not have failed to insert variant type sig");

  sub->value_pos += contained_type_len;

  if (!_dbus_string_insert_byte (sub->value_str,
                                 sub->value_pos,
                                 DBUS_TYPE_INVALID))
    _dbus_assert_not_reached ("should not have failed to insert variant type nul termination");

  sub->value_pos += 1;

  contained_alignment = _dbus_type_get_alignment (_dbus_first_type_in_signature (contained_type, contained_type_start));
  
  if (!_dbus_string_insert_bytes (sub->value_str,
                                  sub->value_pos,
                                  _DBUS_ALIGN_VALUE (sub->value_pos, contained_alignment) - sub->value_pos,
                                  '\0'))
    _dbus_assert_not_reached ("should not have failed to insert alignment padding for variant body");
  sub->value_pos = _DBUS_ALIGN_VALUE (sub->value_pos, contained_alignment);

  return TRUE;
}

static dbus_bool_t
_dbus_type_writer_recurse_contained_len (DBusTypeWriter   *writer,
                                         int               container_type,
                                         const DBusString *contained_type,
                                         int               contained_type_start,
                                         int               contained_type_len,
                                         DBusTypeWriter   *sub,
                                         dbus_bool_t       is_array_append)
{
  writer_recurse_init_and_check (writer, container_type, sub);

  switch (container_type)
    {
    case DBUS_TYPE_STRUCT:
      return writer_recurse_struct_or_dict_entry (writer,
                                                  DBUS_STRUCT_BEGIN_CHAR,
                                                  contained_type,
                                                  contained_type_start, contained_type_len,
                                                  sub);
      break;
    case DBUS_TYPE_DICT_ENTRY:
      return writer_recurse_struct_or_dict_entry (writer,
                                                  DBUS_DICT_ENTRY_BEGIN_CHAR,
                                                  contained_type,
                                                  contained_type_start, contained_type_len,
                                                  sub);
      break;
    case DBUS_TYPE_ARRAY:
      return writer_recurse_array (writer,
                                   contained_type, contained_type_start, contained_type_len,
                                   sub, is_array_append);
      break;
    case DBUS_TYPE_VARIANT:
      return writer_recurse_variant (writer,
                                     contained_type, contained_type_start, contained_type_len,
                                     sub);
      break;
    default:
      _dbus_assert_not_reached ("tried to recurse into type that doesn't support that");
      return FALSE;
      break;
    }
}

dbus_bool_t
_dbus_type_writer_recurse (DBusTypeWriter   *writer,
                           int               container_type,
                           const DBusString *contained_type,
                           int               contained_type_start,
                           DBusTypeWriter   *sub)
{
  int contained_type_len;

  if (contained_type)
    contained_type_len = find_len_of_complete_type (contained_type, contained_type_start);
  else
    contained_type_len = 0;

  return _dbus_type_writer_recurse_contained_len (writer, container_type,
                                                  contained_type,
                                                  contained_type_start,
                                                  contained_type_len,
                                                  sub,
                                                  FALSE);
}

dbus_bool_t
_dbus_type_writer_append_array (DBusTypeWriter   *writer,
                                const DBusString *contained_type,
                                int               contained_type_start,
                                DBusTypeWriter   *sub)
{
  int contained_type_len;

  if (contained_type)
    contained_type_len = find_len_of_complete_type (contained_type, contained_type_start);
  else
    contained_type_len = 0;

  return _dbus_type_writer_recurse_contained_len (writer, DBUS_TYPE_ARRAY,
                                                  contained_type,
                                                  contained_type_start,
                                                  contained_type_len,
                                                  sub,
                                                  TRUE);
}

static int
writer_get_array_len (DBusTypeWriter *writer)
{
  _dbus_assert (writer->container_type == DBUS_TYPE_ARRAY);
  return writer->value_pos - writer->u.array.start_pos;
}

dbus_bool_t
_dbus_type_writer_unrecurse (DBusTypeWriter *writer,
                             DBusTypeWriter *sub)
{
  /* type_pos_is_expectation never gets unset once set, or we'd get all hosed */
  _dbus_assert (!writer->type_pos_is_expectation ||
                (writer->type_pos_is_expectation && sub->type_pos_is_expectation));

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p unrecurse type_pos = %d value_pos = %d is_expectation = %d container_type = %s\n",
                 writer, writer->type_pos, writer->value_pos, writer->type_pos_is_expectation,
                 _dbus_type_to_string (writer->container_type));
  _dbus_verbose ("  type writer %p unrecurse sub type_pos = %d value_pos = %d is_expectation = %d container_type = %s\n",
                 sub, sub->type_pos, sub->value_pos,
                 sub->type_pos_is_expectation,
                 _dbus_type_to_string (sub->container_type));
#endif

  if (sub->container_type == DBUS_TYPE_STRUCT)
    {
      if (!write_or_verify_typecode (sub, DBUS_STRUCT_END_CHAR))
        return FALSE;
    }
  else if (sub->container_type == DBUS_TYPE_DICT_ENTRY)
    {
      if (!write_or_verify_typecode (sub, DBUS_DICT_ENTRY_END_CHAR))
        return FALSE;
    }
  else if (sub->container_type == DBUS_TYPE_ARRAY)
    {
      if (sub->u.array.len_pos >= 0) /* len_pos == -1 if we weren't enabled when we passed it */
        {
          dbus_uint32_t len;

          /* Set the array length */
          len = writer_get_array_len (sub);
          _dbus_marshal_set_uint32 (sub->value_str,
                                    sub->u.array.len_pos,
                                    len,
                                    sub->byte_order);
#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("    filled in sub array len to %u at len_pos %d\n",
                         len, sub->u.array.len_pos);
#endif
        }
#if RECURSIVE_MARSHAL_WRITE_TRACE
      else
        {
          _dbus_verbose ("    not filling in sub array len because we were disabled when we passed the len\n");
        }
#endif
    }

  /* Now get type_pos right for the parent writer. Here are the cases:
   *
   * Cases !writer->type_pos_is_expectation:
   *   (in these cases we want to update to the new insertion point)
   *
   * - if we recursed into a STRUCT then we didn't know in advance
   *   what the types in the struct would be; so we have to fill in
   *   that information now.
   *       writer->type_pos = sub->type_pos
   *
   * - if we recursed into anything else, we knew the full array
   *   type, or knew the single typecode marking VARIANT, so
   *   writer->type_pos is already correct.
   *       writer->type_pos should remain as-is
   *
   * - note that the parent is never an ARRAY or VARIANT, if it were
   *   then type_pos_is_expectation would be TRUE. The parent
   *   is thus known to be a toplevel or STRUCT.
   *
   * Cases where writer->type_pos_is_expectation:
   *   (in these cases we want to update to next expected type to write)
   *
   * - we recursed from STRUCT into STRUCT and we didn't increment
   *   type_pos in the parent just to stay consistent with the
   *   !writer->type_pos_is_expectation case (though we could
   *   special-case this in recurse_struct instead if we wanted)
   *       writer->type_pos = sub->type_pos
   *
   * - we recursed from STRUCT into ARRAY or VARIANT and type_pos
   *   for parent should have been incremented already
   *       writer->type_pos should remain as-is
   *
   * - we recursed from ARRAY into a sub-element, so type_pos in the
   *   parent is the element type and should remain the element type
   *   for the benefit of the next child element
   *       writer->type_pos should remain as-is
   *
   * - we recursed from VARIANT into its value, so type_pos in the
   *   parent makes no difference since there's only one value
   *   and we just finished writing it and won't use type_pos again
   *       writer->type_pos should remain as-is
   *
   *
   * For all these, DICT_ENTRY is the same as STRUCT
   */
  if (writer->type_str != NULL)
    {
      if ((sub->container_type == DBUS_TYPE_STRUCT ||
           sub->container_type == DBUS_TYPE_DICT_ENTRY) &&
          (writer->container_type == DBUS_TYPE_STRUCT ||
           writer->container_type == DBUS_TYPE_DICT_ENTRY ||
           writer->container_type == DBUS_TYPE_INVALID))
        {
          /* Advance the parent to the next struct field */
          writer->type_pos = sub->type_pos;
        }
    }

  writer->value_pos = sub->value_pos;

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p unrecursed type_pos = %d value_pos = %d remaining sig '%s'\n",
                 writer, writer->type_pos, writer->value_pos,
                 writer->type_str ?
                 _dbus_string_get_const_data_len (writer->type_str, writer->type_pos, 0) :
                 "unknown");
#endif

  return TRUE;
}

dbus_bool_t
_dbus_type_writer_write_basic (DBusTypeWriter *writer,
                               int             type,
                               const void     *value)
{
  dbus_bool_t retval;

  /* First ensure that our type realloc will succeed */
  if (!writer->type_pos_is_expectation && writer->type_str != NULL)
    {
      if (!_dbus_string_alloc_space (writer->type_str, 1))
        return FALSE;
    }

  retval = FALSE;

  if (!_dbus_type_writer_write_basic_no_typecode (writer, type, value))
    goto out;

  if (!write_or_verify_typecode (writer, type))
    _dbus_assert_not_reached ("failed to write typecode after prealloc");

  retval = TRUE;

 out:
#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p basic type_pos = %d value_pos = %d is_expectation = %d enabled = %d\n",
                 writer, writer->type_pos, writer->value_pos, writer->type_pos_is_expectation,
                 writer->enabled);
#endif

  return retval;
}

dbus_bool_t
_dbus_type_writer_write_fixed_multi (DBusTypeWriter        *writer,
                                     int                    element_type,
                                     const void            *value,
                                     int                    n_elements)
{
  _dbus_assert (writer->container_type == DBUS_TYPE_ARRAY);
  _dbus_assert (dbus_type_is_fixed (element_type));
  _dbus_assert (writer->type_pos_is_expectation);
  _dbus_assert (n_elements >= 0);

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p entering fixed multi type_pos = %d value_pos = %d n_elements %d\n",
                 writer, writer->type_pos, writer->value_pos, n_elements);
#endif

  if (!write_or_verify_typecode (writer, element_type))
    _dbus_assert_not_reached ("OOM should not happen if only verifying typecode");

  if (writer->enabled)
    {
      if (!_dbus_marshal_write_fixed_multi (writer->value_str,
                                            writer->value_pos,
                                            element_type,
                                            value,
                                            n_elements,
                                            writer->byte_order,
                                            &writer->value_pos))
        return FALSE;
    }

#if RECURSIVE_MARSHAL_WRITE_TRACE
  _dbus_verbose ("  type writer %p fixed multi written new type_pos = %d new value_pos = %d n_elements %d\n",
                 writer, writer->type_pos, writer->value_pos, n_elements);
#endif

  return TRUE;
}

static void
enable_if_after (DBusTypeWriter       *writer,
                 DBusTypeReader       *reader,
                 const DBusTypeReader *start_after)
{
  if (start_after)
    {
      if (!writer->enabled && _dbus_type_reader_greater_than (reader, start_after))
        {
          _dbus_type_writer_set_enabled (writer, TRUE);
#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("ENABLING writer %p at %d because reader at value_pos %d is after reader at value_pos %d\n",
                         writer, writer->value_pos, reader->value_pos, start_after->value_pos);
#endif
        }

      _dbus_assert ((!writer->enabled && !_dbus_type_reader_greater_than (reader, start_after)) ||
                    (writer->enabled && _dbus_type_reader_greater_than (reader, start_after)));
    }
}

static dbus_bool_t
append_fixup (DBusList               **fixups,
              const DBusArrayLenFixup *fixup)
{
  DBusArrayLenFixup *f;

  f = dbus_new (DBusArrayLenFixup, 1);
  if (f == NULL)
    return FALSE;

  *f = *fixup;

  if (!_dbus_list_append (fixups, f))
    {
      dbus_free (f);
      return FALSE;
    }

  _dbus_assert (f->len_pos_in_reader == fixup->len_pos_in_reader);
  _dbus_assert (f->new_len == fixup->new_len);

  return TRUE;
}

static dbus_bool_t
writer_write_reader_helper (DBusTypeWriter       *writer,
                            DBusTypeReader       *reader,
                            const DBusTypeReader *start_after,
                            int                   start_after_new_pos,
                            int                   start_after_new_len,
                            DBusList            **fixups,
                            dbus_bool_t           inside_start_after)
{
  int current_type;

  while ((current_type = _dbus_type_reader_get_current_type (reader)) != DBUS_TYPE_INVALID)
    {
      if (dbus_type_is_container (current_type))
        {
          DBusTypeReader subreader;
          DBusTypeWriter subwriter;
          const DBusString *sig_str;
          int sig_start;
          int sig_len;
          dbus_bool_t enabled_at_recurse;
          dbus_bool_t past_start_after;
          int reader_array_len_pos;
          int reader_array_start_pos;
          dbus_bool_t this_is_start_after;

          /* type_pos is checked since e.g. in a struct the struct
           * and its first field have the same value_pos.
           * type_str will differ in reader/start_after for variants
           * where type_str is inside the value_str
           */
          if (!inside_start_after && start_after &&
              reader->value_pos == start_after->value_pos &&
              reader->type_str == start_after->type_str &&
              reader->type_pos == start_after->type_pos)
            this_is_start_after = TRUE;
          else
            this_is_start_after = FALSE;

          _dbus_type_reader_recurse (reader, &subreader);

          if (current_type == DBUS_TYPE_ARRAY)
            {
              reader_array_len_pos = ARRAY_READER_LEN_POS (&subreader);
              reader_array_start_pos = subreader.u.array.start_pos;
            }
          else
            {
              /* quiet gcc */
              reader_array_len_pos = -1;
              reader_array_start_pos = -1;
            }

          _dbus_type_reader_get_signature (&subreader, &sig_str,
                                           &sig_start, &sig_len);

#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("about to recurse into %s reader at %d subreader at %d writer at %d start_after reader at %d write target len %d inside_start_after = %d this_is_start_after = %d\n",
                         _dbus_type_to_string (current_type),
                         reader->value_pos,
                         subreader.value_pos,
                         writer->value_pos,
                         start_after ? start_after->value_pos : -1,
                         _dbus_string_get_length (writer->value_str),
                         inside_start_after, this_is_start_after);
#endif

          if (!inside_start_after && !this_is_start_after)
            enable_if_after (writer, &subreader, start_after);
          enabled_at_recurse = writer->enabled;
          if (!_dbus_type_writer_recurse_contained_len (writer, current_type,
                                                        sig_str, sig_start, sig_len,
                                                        &subwriter, FALSE))
            goto oom;

#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("recursed into subwriter at %d write target len %d\n",
                         subwriter.value_pos,
                         _dbus_string_get_length (subwriter.value_str));
#endif

          if (!writer_write_reader_helper (&subwriter, &subreader, start_after,
                                           start_after_new_pos, start_after_new_len,
                                           fixups,
                                           inside_start_after ||
                                           this_is_start_after))
            goto oom;

#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("about to unrecurse from %s subreader at %d writer at %d subwriter at %d  write target len %d\n",
                         _dbus_type_to_string (current_type),
                         subreader.value_pos,
                         writer->value_pos,
                         subwriter.value_pos,
                         _dbus_string_get_length (writer->value_str));
#endif

          if (!inside_start_after && !this_is_start_after)
            enable_if_after (writer, &subreader, start_after);
          past_start_after = writer->enabled;
          if (!_dbus_type_writer_unrecurse (writer, &subwriter))
            goto oom;

          /* If we weren't enabled when we recursed, we didn't
           * write an array len; if we passed start_after
           * somewhere inside the array, then we need to generate
           * a fixup.
           */
          if (start_after != NULL &&
              !enabled_at_recurse && past_start_after &&
              current_type == DBUS_TYPE_ARRAY &&
              fixups != NULL)
            {
              DBusArrayLenFixup fixup;
              int bytes_written_after_start_after;
              int bytes_before_start_after;
              int old_len;

              /* this subwriter access is moderately unkosher since we
               * already unrecursed, but it works as long as unrecurse
               * doesn't break us on purpose
               */
              bytes_written_after_start_after = writer_get_array_len (&subwriter);

              bytes_before_start_after =
                start_after->value_pos - reader_array_start_pos;

              fixup.len_pos_in_reader = reader_array_len_pos;
              fixup.new_len =
                bytes_before_start_after +
                start_after_new_len +
                bytes_written_after_start_after;

              _dbus_assert (_DBUS_ALIGN_VALUE (fixup.len_pos_in_reader, 4) ==
                            (unsigned) fixup.len_pos_in_reader);

              old_len = _dbus_unpack_uint32 (reader->byte_order,
                                             _dbus_string_get_const_data_len (reader->value_str,
                                                                              fixup.len_pos_in_reader, 4));

              if (old_len != fixup.new_len && !append_fixup (fixups, &fixup))
                goto oom;

#if RECURSIVE_MARSHAL_WRITE_TRACE
              _dbus_verbose ("Generated fixup len_pos_in_reader = %d new_len = %d reader_array_start_pos = %d start_after->value_pos = %d bytes_before_start_after = %d start_after_new_len = %d bytes_written_after_start_after = %d\n",
                             fixup.len_pos_in_reader,
                             fixup.new_len,
                             reader_array_start_pos,
                             start_after->value_pos,
                             bytes_before_start_after,
                             start_after_new_len,
                             bytes_written_after_start_after);
#endif
            }
        }
      else
        {
          DBusBasicValue val;

          _dbus_assert (dbus_type_is_basic (current_type));

#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("Reading basic value %s at %d\n",
                         _dbus_type_to_string (current_type),
                         reader->value_pos);
#endif

          _dbus_type_reader_read_basic (reader, &val);

#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("Writing basic value %s at %d write target len %d inside_start_after = %d\n",
                         _dbus_type_to_string (current_type),
                         writer->value_pos,
                         _dbus_string_get_length (writer->value_str),
                         inside_start_after);
#endif
          if (!inside_start_after)
            enable_if_after (writer, reader, start_after);
          if (!_dbus_type_writer_write_basic (writer, current_type, &val))
            goto oom;
#if RECURSIVE_MARSHAL_WRITE_TRACE
          _dbus_verbose ("Wrote basic value %s, new value_pos %d write target len %d\n",
                         _dbus_type_to_string (current_type),
                         writer->value_pos,
                         _dbus_string_get_length (writer->value_str));
#endif
        }

      _dbus_type_reader_next (reader);
    }

  return TRUE;

 oom:
  if (fixups)
    apply_and_free_fixups (fixups, NULL); /* NULL for reader to apply to */

  return FALSE;
}

dbus_bool_t
_dbus_type_writer_write_reader_partial (DBusTypeWriter       *writer,
                                        DBusTypeReader       *reader,
                                        const DBusTypeReader *start_after,
                                        int                   start_after_new_pos,
                                        int                   start_after_new_len,
                                        DBusList            **fixups)
{
  DBusTypeWriter orig;
  int orig_type_len;
  int orig_value_len;
  int new_bytes;
  int orig_enabled;

  orig = *writer;
  orig_type_len = _dbus_string_get_length (writer->type_str);
  orig_value_len = _dbus_string_get_length (writer->value_str);
  orig_enabled = writer->enabled;

  if (start_after)
    _dbus_type_writer_set_enabled (writer, FALSE);

  if (!writer_write_reader_helper (writer, reader, start_after,
                                   start_after_new_pos,
                                   start_after_new_len,
                                   fixups, FALSE))
    goto oom;

  _dbus_type_writer_set_enabled (writer, orig_enabled);
  return TRUE;

 oom:
  if (!writer->type_pos_is_expectation)
    {
      new_bytes = _dbus_string_get_length (writer->type_str) - orig_type_len;
      _dbus_string_delete (writer->type_str, orig.type_pos, new_bytes);
    }
  new_bytes = _dbus_string_get_length (writer->value_str) - orig_value_len;
  _dbus_string_delete (writer->value_str, orig.value_pos, new_bytes);

  *writer = orig;

  return FALSE;
}

dbus_bool_t
_dbus_type_writer_write_reader (DBusTypeWriter       *writer,
                                DBusTypeReader       *reader)
{
  return _dbus_type_writer_write_reader_partial (writer, reader, NULL, 0, 0, NULL);
}

void
_dbus_type_writer_set_enabled (DBusTypeWriter   *writer,
                               dbus_bool_t       enabled)
{
  writer->enabled = enabled != FALSE;
}

/** @} */ /* end of DBusMarshal group */

/* tests in dbus-marshal-recursive-util.c */
