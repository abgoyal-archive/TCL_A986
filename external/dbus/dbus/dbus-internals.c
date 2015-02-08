
/* -*- mode: C; c-file-style: "gnu" -*- */
#include "dbus-internals.h"
#include "dbus-protocol.h"
#include "dbus-marshal-basic.h"
#include "dbus-test.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#ifdef DBUS_ANDROID_LOG
#define LOG_TAG "libdbus"
#include <cutils/log.h>
#endif /* DBUS_ANDROID_LOG */















const char _dbus_no_memory_message[] = "Not enough memory";

static dbus_bool_t warn_initted = FALSE;
static dbus_bool_t fatal_warnings = FALSE;
static dbus_bool_t fatal_warnings_on_check_failed = TRUE;

static void
init_warnings(void)
{
  if (!warn_initted)
    {
      const char *s;
      s = _dbus_getenv ("DBUS_FATAL_WARNINGS");
      if (s && *s)
        {
          if (*s == '0')
            {
              fatal_warnings = FALSE;
              fatal_warnings_on_check_failed = FALSE;
            }
          else if (*s == '1')
            {
              fatal_warnings = TRUE;
              fatal_warnings_on_check_failed = TRUE;
            }
          else
            {
              fprintf(stderr, "DBUS_FATAL_WARNINGS should be set to 0 or 1 if set, not '%s'",
                      s);
            }
        }

      warn_initted = TRUE;
    }
}

void
_dbus_warn (const char *format,
            ...)
{
  va_list args;

  if (!warn_initted)
    init_warnings ();
  
  va_start (args, format);
#ifdef DBUS_ANDROID_LOG
  LOG_PRI_VA(ANDROID_LOG_WARN, LOG_TAG, format, args);
#else
  vfprintf (stderr, format, args);
#endif /* DBUS_ANDROID_LOG */
  va_end (args);

  if (fatal_warnings)
    {
      fflush (stderr);
      _dbus_abort ();
    }
}

void
_dbus_warn_check_failed(const char *format,
                        ...)
{
  va_list args;
  
  if (!warn_initted)
    init_warnings ();

  fprintf (stderr, "process %lu: ", _dbus_getpid ());
  
  va_start (args, format);
#ifdef DBUS_ANDROID_LOG
  LOG_PRI_VA(ANDROID_LOG_ERROR, LOG_TAG, format, args);
#else
  vfprintf (stderr, format, args);
#endif /* DBUS_ANDROID_LOG */
  va_end (args);

  if (fatal_warnings_on_check_failed)
    {
      fflush (stderr);
      _dbus_abort ();
    }
}

#ifdef DBUS_ENABLE_VERBOSE_MODE

static dbus_bool_t verbose_initted = FALSE;
static dbus_bool_t verbose = TRUE;

/** Whether to show the current thread in verbose messages */
#define PTHREAD_IN_VERBOSE 0
#if PTHREAD_IN_VERBOSE
#include <pthread.h>
#endif

static inline void
_dbus_verbose_init (void)
{
  if (!verbose_initted)
    {
#ifdef DBUS_ANDROID_LOG
      /* Don't bother checking environment variable - just print the
         verbose logs (can still be disabled with DBUS_ENABLE_VERBOSE_MODE) */
      verbose = TRUE;
#else
      const char *p = _dbus_getenv ("DBUS_VERBOSE"); 
      verbose = p != NULL && *p == '1';
#endif
      verbose_initted = TRUE;
    }
}

dbus_bool_t
_dbus_is_verbose_real (void)
{
  _dbus_verbose_init ();
  return verbose;
}

void
_dbus_verbose_real (const char *format,
                    ...)
{
  va_list args;
  static dbus_bool_t need_pid = TRUE;
  int len;

  /* things are written a bit oddly here so that
   * in the non-verbose case we just have the one
   * conditional and return immediately.
   */
  if (!_dbus_is_verbose_real())
    return;

  /* Print out pid before the line */
  if (need_pid)
    {
#if PTHREAD_IN_VERBOSE
      fprintf (stderr, "%lu: 0x%lx: ", _dbus_getpid (), pthread_self ());
#else
      fprintf (stderr, "%lu: ", _dbus_getpid ());
#endif
    }
      

  /* Only print pid again if the next line is a new line */
  len = strlen (format);
  if (format[len-1] == '\n')
    need_pid = TRUE;
  else
    need_pid = FALSE;
  
  va_start (args, format);
#ifdef DBUS_ANDROID_LOG
  LOG_PRI_VA(ANDROID_LOG_DEBUG, LOG_TAG, format, args);
#else
  vfprintf (stderr, format, args);
#endif /* DBUS_ANDROID_LOG */
  va_end (args);

  fflush (stderr);
}

void
_dbus_verbose_reset_real (void)
{
  verbose_initted = FALSE;
}

#endif /* DBUS_ENABLE_VERBOSE_MODE */

char*
_dbus_strdup (const char *str)
{
  size_t len;
  char *copy;
  
  if (str == NULL)
    return NULL;
  
  len = strlen (str);

  copy = dbus_malloc (len + 1);
  if (copy == NULL)
    return NULL;

  memcpy (copy, str, len + 1);
  
  return copy;
}

void*
_dbus_memdup (const void  *mem,
              size_t       n_bytes)
{
  void *copy;

  copy = dbus_malloc (n_bytes);
  if (copy == NULL)
    return NULL;

  memcpy (copy, mem, n_bytes);
  
  return copy;
}

char**
_dbus_dup_string_array (const char **array)
{
  int len;
  int i;
  char **copy;
  
  if (array == NULL)
    return NULL;

  for (len = 0; array[len] != NULL; ++len)
    ;

  copy = dbus_new0 (char*, len + 1);
  if (copy == NULL)
    return NULL;

  i = 0;
  while (i < len)
    {
      copy[i] = _dbus_strdup (array[i]);
      if (copy[i] == NULL)
        {
          dbus_free_string_array (copy);
          return NULL;
        }

      ++i;
    }

  return copy;
}

dbus_bool_t
_dbus_string_array_contains (const char **array,
                             const char  *str)
{
  int i;

  i = 0;
  while (array[i] != NULL)
    {
      if (strcmp (array[i], str) == 0)
        return TRUE;
      ++i;
    }

  return FALSE;
}

void
_dbus_generate_uuid (DBusGUID *uuid)
{
  long now;

  _dbus_get_current_time (&now, NULL);

  uuid->as_uint32s[DBUS_UUID_LENGTH_WORDS - 1] = DBUS_UINT32_TO_BE (now);
  
  _dbus_generate_random_bytes_buffer (uuid->as_bytes, DBUS_UUID_LENGTH_BYTES - 4);
}

dbus_bool_t
_dbus_uuid_encode (const DBusGUID *uuid,
                   DBusString     *encoded)
{
  DBusString binary;
  _dbus_string_init_const_len (&binary, uuid->as_bytes, DBUS_UUID_LENGTH_BYTES);
  return _dbus_string_hex_encode (&binary, 0, encoded, _dbus_string_get_length (encoded));
}

static dbus_bool_t
_dbus_read_uuid_file_without_creating (const DBusString *filename,
                                       DBusGUID         *uuid,
                                       DBusError        *error)
{
  DBusString contents;
  DBusString decoded;
  int end;
  
  _dbus_string_init (&contents);
  _dbus_string_init (&decoded);
  
  if (!_dbus_file_get_contents (&contents, filename, error))
    goto error;

  _dbus_string_chop_white (&contents);

  if (_dbus_string_get_length (&contents) != DBUS_UUID_LENGTH_HEX)
    {
      dbus_set_error (error, DBUS_ERROR_INVALID_FILE_CONTENT,
                      "UUID file '%s' should contain a hex string of length %d, not length %d, with no other text",
                      _dbus_string_get_const_data (filename),
                      DBUS_UUID_LENGTH_HEX,
                      _dbus_string_get_length (&contents));
      goto error;
    }

  if (!_dbus_string_hex_decode (&contents, 0, &end, &decoded, 0))
    {
      _DBUS_SET_OOM (error);
      goto error;
    }

  if (end == 0)
    {
      dbus_set_error (error, DBUS_ERROR_INVALID_FILE_CONTENT,
                      "UUID file '%s' contains invalid hex data",
                      _dbus_string_get_const_data (filename));
      goto error;
    }

  if (_dbus_string_get_length (&decoded) != DBUS_UUID_LENGTH_BYTES)
    {
      dbus_set_error (error, DBUS_ERROR_INVALID_FILE_CONTENT,
                      "UUID file '%s' contains %d bytes of hex-encoded data instead of %d",
                      _dbus_string_get_const_data (filename),
                      _dbus_string_get_length (&decoded),
                      DBUS_UUID_LENGTH_BYTES);
      goto error;
    }

  _dbus_string_copy_to_buffer (&decoded, uuid->as_bytes, DBUS_UUID_LENGTH_BYTES);

  _dbus_string_free (&decoded);
  _dbus_string_free (&contents);

  _DBUS_ASSERT_ERROR_IS_CLEAR (error);

  return TRUE;
  
 error:
  _DBUS_ASSERT_ERROR_IS_SET (error);
  _dbus_string_free (&contents);
  _dbus_string_free (&decoded);
  return FALSE;
}

static dbus_bool_t
_dbus_create_uuid_file_exclusively (const DBusString *filename,
                                    DBusGUID         *uuid,
                                    DBusError        *error)
{
  DBusString encoded;

  _dbus_string_init (&encoded);

  _dbus_generate_uuid (uuid);
  
  if (!_dbus_uuid_encode (uuid, &encoded))
    {
      _DBUS_SET_OOM (error);
      goto error;
    }
  
  /* FIXME this is racy; we need a save_file_exclusively
   * function. But in practice this should be fine for now.
   *
   * - first be sure we can create the file and it
   *   doesn't exist by creating it empty with O_EXCL
   * - then create it by creating a temporary file and
   *   overwriting atomically with rename()
   */
  if (!_dbus_create_file_exclusively (filename, error))
    goto error;

  if (!_dbus_string_append_byte (&encoded, '\n'))
    {
      _DBUS_SET_OOM (error);
      goto error;
    }
  
  if (!_dbus_string_save_to_file (&encoded, filename, error))
    goto error;

  if (!_dbus_make_file_world_readable (filename, error))
    goto error;

  _dbus_string_free (&encoded);

  _DBUS_ASSERT_ERROR_IS_CLEAR (error);
  return TRUE;
  
 error:
  _DBUS_ASSERT_ERROR_IS_SET (error);
  _dbus_string_free (&encoded);
  return FALSE;        
}

dbus_bool_t
_dbus_read_uuid_file (const DBusString *filename,
                      DBusGUID         *uuid,
                      dbus_bool_t       create_if_not_found,
                      DBusError        *error)
{
  DBusError read_error;

  dbus_error_init (&read_error);

  if (_dbus_read_uuid_file_without_creating (filename, uuid, &read_error))
    return TRUE;

  if (!create_if_not_found)
    {
      dbus_move_error (&read_error, error);
      return FALSE;
    }

  /* If the file exists and contains junk, we want to keep that error
   * message instead of overwriting it with a "file exists" error
   * message when we try to write
   */
  if (dbus_error_has_name (&read_error, DBUS_ERROR_INVALID_FILE_CONTENT))
    {
      dbus_move_error (&read_error, error);
      return FALSE;
    }
  else
    {
      dbus_error_free (&read_error);
      return _dbus_create_uuid_file_exclusively (filename, uuid, error);
    }
}

_DBUS_DEFINE_GLOBAL_LOCK (machine_uuid);
static int machine_uuid_initialized_generation = 0;
static DBusGUID machine_uuid;

dbus_bool_t
_dbus_get_local_machine_uuid_encoded (DBusString *uuid_str)
{
  dbus_bool_t ok;
  
  _DBUS_LOCK (machine_uuid);
  if (machine_uuid_initialized_generation != _dbus_current_generation)
    {
      DBusError error;
      dbus_error_init (&error);
      if (!_dbus_read_local_machine_uuid (&machine_uuid, FALSE,
                                          &error))
        {          
#ifndef DBUS_BUILD_TESTS
          /* For the test suite, we may not be installed so just continue silently
           * here. But in a production build, we want to be nice and loud about
           * this.
           */
          _dbus_warn_check_failed ("D-Bus library appears to be incorrectly set up; failed to read machine uuid: %s\n"
                                   "See the manual page for dbus-uuidgen to correct this issue.\n",
                                   error.message);
#endif
          
          dbus_error_free (&error);
          
          _dbus_generate_uuid (&machine_uuid);
        }
    }

  ok = _dbus_uuid_encode (&machine_uuid, uuid_str);

  _DBUS_UNLOCK (machine_uuid);

  return ok;
}

#ifdef DBUS_BUILD_TESTS
const char *
_dbus_header_field_to_string (int header_field)
{
  switch (header_field)
    {
    case DBUS_HEADER_FIELD_INVALID:
      return "invalid";
    case DBUS_HEADER_FIELD_PATH:
      return "path";
    case DBUS_HEADER_FIELD_INTERFACE:
      return "interface";
    case DBUS_HEADER_FIELD_MEMBER:
      return "member";
    case DBUS_HEADER_FIELD_ERROR_NAME:
      return "error-name";
    case DBUS_HEADER_FIELD_REPLY_SERIAL:
      return "reply-serial";
    case DBUS_HEADER_FIELD_DESTINATION:
      return "destination";
    case DBUS_HEADER_FIELD_SENDER:
      return "sender";
    case DBUS_HEADER_FIELD_SIGNATURE:
      return "signature";
    default:
      return "unknown";
    }
}
#endif /* DBUS_BUILD_TESTS */

#ifndef DBUS_DISABLE_CHECKS
/** String used in _dbus_return_if_fail macro */
const char _dbus_return_if_fail_warning_format[] =
"arguments to %s() were incorrect, assertion \"%s\" failed in file %s line %d.\n"
"This is normally a bug in some application using the D-Bus library.\n";
#endif

#ifndef DBUS_DISABLE_ASSERT
void
_dbus_real_assert (dbus_bool_t  condition,
                   const char  *condition_text,
                   const char  *file,
                   int          line,
                   const char  *func)
{
  if (_DBUS_UNLIKELY (!condition))
    {
      _dbus_warn ("%lu: assertion failed \"%s\" file \"%s\" line %d function %s\n",
                  _dbus_getpid (), condition_text, file, line, func);
      _dbus_abort ();
    }
}

void
_dbus_real_assert_not_reached (const char *explanation,
                               const char *file,
                               int         line)
{
  _dbus_warn ("File \"%s\" line %d process %lu should not have been reached: %s\n",
              file, line, _dbus_getpid (), explanation);
  _dbus_abort ();
}
#endif /* DBUS_DISABLE_ASSERT */
  
#ifdef DBUS_BUILD_TESTS
static dbus_bool_t
run_failing_each_malloc (int                    n_mallocs,
                         const char            *description,
                         DBusTestMemoryFunction func,
                         void                  *data)
{
  n_mallocs += 10; /* fudge factor to ensure reallocs etc. are covered */
  
  while (n_mallocs >= 0)
    {      
      _dbus_set_fail_alloc_counter (n_mallocs);

      _dbus_verbose ("\n===\n%s: (will fail malloc %d with %d failures)\n===\n",
                     description, n_mallocs,
                     _dbus_get_fail_alloc_failures ());

      if (!(* func) (data))
        return FALSE;
      
      n_mallocs -= 1;
    }

  _dbus_set_fail_alloc_counter (_DBUS_INT_MAX);

  return TRUE;
}                        

dbus_bool_t
_dbus_test_oom_handling (const char             *description,
                         DBusTestMemoryFunction  func,
                         void                   *data)
{
  int approx_mallocs;
  const char *setting;
  int max_failures_to_try;
  int i;

  /* Run once to see about how many mallocs are involved */
  
  _dbus_set_fail_alloc_counter (_DBUS_INT_MAX);

  _dbus_verbose ("Running once to count mallocs\n");
  
  if (!(* func) (data))
    return FALSE;
  
  approx_mallocs = _DBUS_INT_MAX - _dbus_get_fail_alloc_counter ();

  _dbus_verbose ("\n=================\n%s: about %d mallocs total\n=================\n",
                 description, approx_mallocs);

  setting = _dbus_getenv ("DBUS_TEST_MALLOC_FAILURES");
  if (setting != NULL)
    {
      DBusString str;
      long v;
      _dbus_string_init_const (&str, setting);
      v = 4;
      if (!_dbus_string_parse_int (&str, 0, &v, NULL))
        _dbus_warn ("couldn't parse '%s' as integer\n", setting);
      max_failures_to_try = v;
    }
  else
    {
      max_failures_to_try = 4;
    }

  i = setting ? max_failures_to_try - 1 : 1;
  while (i < max_failures_to_try)
    {
      _dbus_set_fail_alloc_failures (i);
      if (!run_failing_each_malloc (approx_mallocs, description, func, data))
        return FALSE;
      ++i;
    }
  
  _dbus_verbose ("\n=================\n%s: all iterations passed\n=================\n",
                 description);

  return TRUE;
}
#endif /* DBUS_BUILD_TESTS */

/** @} */
