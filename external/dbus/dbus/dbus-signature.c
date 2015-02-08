
/* -*- mode: C; c-file-style: "gnu" -*- */

#include "dbus-signature.h"
#include "dbus-marshal-recursive.h"
#include "dbus-marshal-basic.h"
#include "dbus-internals.h"
#include "dbus-test.h"

typedef struct
{ 
  const char *pos;           /**< current position in the signature string */
  unsigned int finished : 1; /**< true if we are at the end iter */
  unsigned int in_array : 1; /**< true if we are a subiterator pointing to an array's element type */
} DBusSignatureRealIter;

/** macro that checks whether a typecode is a container type */
#define TYPE_IS_CONTAINER(typecode)             \
    ((typecode) == DBUS_TYPE_STRUCT ||          \
     (typecode) == DBUS_TYPE_DICT_ENTRY ||      \
     (typecode) == DBUS_TYPE_VARIANT ||         \
     (typecode) == DBUS_TYPE_ARRAY)



void
dbus_signature_iter_init (DBusSignatureIter *iter,
			  const char        *signature)
{
  DBusSignatureRealIter *real_iter = (DBusSignatureRealIter *) iter;

  real_iter->pos = signature;
  real_iter->finished = FALSE;
  real_iter->in_array = FALSE;
}

int
dbus_signature_iter_get_current_type (const DBusSignatureIter *iter)
{
  DBusSignatureRealIter *real_iter = (DBusSignatureRealIter *) iter;

  return _dbus_first_type_in_signature_c_str (real_iter->pos, 0);
}

char *
dbus_signature_iter_get_signature (const DBusSignatureIter *iter)
{
  DBusSignatureRealIter *real_iter = (DBusSignatureRealIter *) iter;
  DBusString str;
  char *ret;
  int pos;
  
  if (!_dbus_string_init (&str))
    return NULL;

  pos = 0;
  _dbus_type_signature_next (real_iter->pos, &pos);

  if (!_dbus_string_append_len (&str, real_iter->pos, pos))
    return NULL;
  if (!_dbus_string_steal_data (&str, &ret))
    ret = NULL;
  _dbus_string_free (&str);

  return ret; 
}

int
dbus_signature_iter_get_element_type (const DBusSignatureIter *iter)
{
  DBusSignatureRealIter *real_iter = (DBusSignatureRealIter *) iter;

  _dbus_return_val_if_fail (dbus_signature_iter_get_current_type (iter) == DBUS_TYPE_ARRAY, DBUS_TYPE_INVALID);

  return _dbus_first_type_in_signature_c_str (real_iter->pos, 1);
}

dbus_bool_t
dbus_signature_iter_next (DBusSignatureIter *iter)
{
  DBusSignatureRealIter *real_iter = (DBusSignatureRealIter *) iter;

  if (real_iter->finished)
    return FALSE;
  else
    {
      int pos;

      if (real_iter->in_array)
	{
	  real_iter->finished = TRUE;
	  return FALSE;
	}

      pos = 0;
      _dbus_type_signature_next (real_iter->pos, &pos);
      real_iter->pos += pos;

      if (*real_iter->pos == DBUS_STRUCT_END_CHAR
	  || *real_iter->pos == DBUS_DICT_ENTRY_END_CHAR)
	{
	  real_iter->finished = TRUE;
	  return FALSE;
	}

      return *real_iter->pos != DBUS_TYPE_INVALID;
    }
}

void
dbus_signature_iter_recurse (const DBusSignatureIter *iter,
			     DBusSignatureIter       *subiter)
{
  DBusSignatureRealIter *real_iter = (DBusSignatureRealIter *) iter;
  DBusSignatureRealIter *real_sub_iter = (DBusSignatureRealIter *) subiter;

  _dbus_return_if_fail (dbus_type_is_container (dbus_signature_iter_get_current_type (iter)));

  *real_sub_iter = *real_iter;
  real_sub_iter->in_array = FALSE;
  real_sub_iter->pos++;

  if (dbus_signature_iter_get_current_type (iter) == DBUS_TYPE_ARRAY)
    real_sub_iter->in_array = TRUE;
}

dbus_bool_t
dbus_signature_validate (const char       *signature,
			 DBusError        *error)
			 
{
  DBusString str;

  _dbus_string_init_const (&str, signature);
  if (_dbus_validate_signature (&str, 0, _dbus_string_get_length (&str)))
    return TRUE;
  dbus_set_error (error, DBUS_ERROR_INVALID_SIGNATURE, "Corrupt type signature");
  return FALSE;
}

dbus_bool_t
dbus_signature_validate_single (const char       *signature,
				DBusError        *error)
{
  DBusSignatureIter iter;

  if (!dbus_signature_validate (signature, error))
    return FALSE;

  dbus_signature_iter_init (&iter, signature);
  if (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_INVALID)
    goto lose;
  if (!dbus_signature_iter_next (&iter))
    return TRUE;
 lose:
  dbus_set_error (error, DBUS_ERROR_INVALID_SIGNATURE, "Exactly one complete type required in signature");
  return FALSE;
}

dbus_bool_t
dbus_type_is_container (int typecode)
{
  /* only reasonable (non-line-noise) typecodes are allowed */
  _dbus_return_val_if_fail (_dbus_type_is_valid (typecode) || typecode == DBUS_TYPE_INVALID,
			    FALSE);
  return TYPE_IS_CONTAINER (typecode);
}

dbus_bool_t
dbus_type_is_basic (int typecode)
{
  /* only reasonable (non-line-noise) typecodes are allowed */
  _dbus_return_val_if_fail (_dbus_type_is_valid (typecode) || typecode == DBUS_TYPE_INVALID,
			    FALSE);

  /* everything that isn't invalid or a container */
  return !(typecode == DBUS_TYPE_INVALID || TYPE_IS_CONTAINER (typecode));
}

dbus_bool_t
dbus_type_is_fixed (int typecode)
{
  /* only reasonable (non-line-noise) typecodes are allowed */
  _dbus_return_val_if_fail (_dbus_type_is_valid (typecode) || typecode == DBUS_TYPE_INVALID,
			    FALSE);
  
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
      return TRUE;
    default:
      return FALSE;
    }
}

/** @} */ /* end of DBusSignature group */

#ifdef DBUS_BUILD_TESTS

dbus_bool_t
_dbus_signature_test (void)
{
  DBusSignatureIter iter;
  DBusSignatureIter subiter;
  DBusSignatureIter subsubiter;
  DBusSignatureIter subsubsubiter;
  const char *sig;
  dbus_bool_t boolres;

  _dbus_assert (sizeof (DBusSignatureIter) >= sizeof (DBusSignatureRealIter));

  sig = "";
  _dbus_assert (dbus_signature_validate (sig, NULL));
  _dbus_assert (!dbus_signature_validate_single (sig, NULL));
  dbus_signature_iter_init (&iter, sig);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_INVALID);

  sig = DBUS_TYPE_STRING_AS_STRING;
  _dbus_assert (dbus_signature_validate (sig, NULL));
  _dbus_assert (dbus_signature_validate_single (sig, NULL));
  dbus_signature_iter_init (&iter, sig);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_STRING);

  sig = DBUS_TYPE_STRING_AS_STRING DBUS_TYPE_BYTE_AS_STRING;
  _dbus_assert (dbus_signature_validate (sig, NULL));
  dbus_signature_iter_init (&iter, sig);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_STRING);
  boolres = dbus_signature_iter_next (&iter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_BYTE);

  sig = DBUS_TYPE_UINT16_AS_STRING
    DBUS_STRUCT_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_STRING_AS_STRING
    DBUS_TYPE_UINT32_AS_STRING
    DBUS_TYPE_VARIANT_AS_STRING
    DBUS_TYPE_DOUBLE_AS_STRING
    DBUS_STRUCT_END_CHAR_AS_STRING;
  _dbus_assert (dbus_signature_validate (sig, NULL));
  dbus_signature_iter_init (&iter, sig);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_UINT16);
  boolres = dbus_signature_iter_next (&iter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_STRUCT);
  dbus_signature_iter_recurse (&iter, &subiter);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_STRING);
  boolres = dbus_signature_iter_next (&subiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_UINT32);
  boolres = dbus_signature_iter_next (&subiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_VARIANT);
  boolres = dbus_signature_iter_next (&subiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_DOUBLE);

  sig = DBUS_TYPE_UINT16_AS_STRING
    DBUS_STRUCT_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_UINT32_AS_STRING
    DBUS_TYPE_BYTE_AS_STRING
    DBUS_TYPE_ARRAY_AS_STRING
    DBUS_TYPE_ARRAY_AS_STRING
    DBUS_TYPE_DOUBLE_AS_STRING
    DBUS_STRUCT_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_BYTE_AS_STRING
    DBUS_STRUCT_END_CHAR_AS_STRING
    DBUS_STRUCT_END_CHAR_AS_STRING;
  _dbus_assert (dbus_signature_validate (sig, NULL));
  dbus_signature_iter_init (&iter, sig);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_UINT16);
  boolres = dbus_signature_iter_next (&iter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_STRUCT);
  dbus_signature_iter_recurse (&iter, &subiter);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_UINT32);
  boolres = dbus_signature_iter_next (&subiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_BYTE);
  boolres = dbus_signature_iter_next (&subiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_ARRAY);
  _dbus_assert (dbus_signature_iter_get_element_type (&subiter) == DBUS_TYPE_ARRAY);

  dbus_signature_iter_recurse (&subiter, &subsubiter);
  _dbus_assert (dbus_signature_iter_get_current_type (&subsubiter) == DBUS_TYPE_ARRAY);
  _dbus_assert (dbus_signature_iter_get_element_type (&subsubiter) == DBUS_TYPE_DOUBLE);

  dbus_signature_iter_recurse (&subsubiter, &subsubsubiter);
  _dbus_assert (dbus_signature_iter_get_current_type (&subsubsubiter) == DBUS_TYPE_DOUBLE);
  boolres = dbus_signature_iter_next (&subiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subiter) == DBUS_TYPE_STRUCT);
  dbus_signature_iter_recurse (&subiter, &subsubiter);
  _dbus_assert (dbus_signature_iter_get_current_type (&subsubiter) == DBUS_TYPE_BYTE);

  sig = DBUS_TYPE_ARRAY_AS_STRING
    DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_INT16_AS_STRING
    DBUS_TYPE_STRING_AS_STRING
    DBUS_DICT_ENTRY_END_CHAR_AS_STRING
    DBUS_TYPE_VARIANT_AS_STRING;
  _dbus_assert (dbus_signature_validate (sig, NULL));
  _dbus_assert (!dbus_signature_validate_single (sig, NULL));
  dbus_signature_iter_init (&iter, sig);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_ARRAY);
  _dbus_assert (dbus_signature_iter_get_element_type (&iter) == DBUS_TYPE_DICT_ENTRY);

  dbus_signature_iter_recurse (&iter, &subiter);
  dbus_signature_iter_recurse (&subiter, &subsubiter);
  _dbus_assert (dbus_signature_iter_get_current_type (&subsubiter) == DBUS_TYPE_INT16);
  boolres = dbus_signature_iter_next (&subsubiter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&subsubiter) == DBUS_TYPE_STRING);
  boolres = dbus_signature_iter_next (&subsubiter);
  _dbus_assert (!boolres);

  boolres = dbus_signature_iter_next (&iter);
  _dbus_assert (boolres);
  _dbus_assert (dbus_signature_iter_get_current_type (&iter) == DBUS_TYPE_VARIANT);
  boolres = dbus_signature_iter_next (&iter);
  _dbus_assert (!boolres);

  sig = DBUS_TYPE_DICT_ENTRY_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_TYPE_ARRAY_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_TYPE_UINT32_AS_STRING
    DBUS_TYPE_ARRAY_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_TYPE_ARRAY_AS_STRING
    DBUS_TYPE_DICT_ENTRY_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_DICT_ENTRY_END_CHAR_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_INT32_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_DICT_ENTRY_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_INT32_AS_STRING
    DBUS_TYPE_STRING_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_STRUCT_END_CHAR_AS_STRING
    DBUS_STRUCT_BEGIN_CHAR_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));

  sig = DBUS_STRUCT_BEGIN_CHAR_AS_STRING
    DBUS_TYPE_BOOLEAN_AS_STRING;
  _dbus_assert (!dbus_signature_validate (sig, NULL));
  return TRUE;
#if 0
 oom:
  _dbus_assert_not_reached ("out of memory");
  return FALSE;
#endif
}

#endif

