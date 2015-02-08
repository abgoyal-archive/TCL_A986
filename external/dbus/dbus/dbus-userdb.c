
/* -*- mode: C; c-file-style: "gnu" -*- */
#define DBUS_USERDB_INCLUDES_PRIVATE 1
#include "dbus-userdb.h"
#include "dbus-hash.h"
#include "dbus-test.h"
#include "dbus-internals.h"
#include "dbus-protocol.h"
#include <string.h>


void
_dbus_user_info_free_allocated (DBusUserInfo *info)
{
  if (info == NULL) /* hash table will pass NULL */
    return;

  _dbus_user_info_free (info);
  dbus_free (info);
}

void
_dbus_group_info_free_allocated (DBusGroupInfo *info)
{
  if (info == NULL) /* hash table will pass NULL */
    return;

  _dbus_group_info_free (info);
  dbus_free (info);
}

dbus_bool_t
_dbus_is_a_number (const DBusString *str,
                   unsigned long    *num)
{
  int end;

  if (_dbus_string_parse_int (str, 0, num, &end) &&
      end == _dbus_string_get_length (str))
    return TRUE;
  else
    return FALSE;
}

DBusUserInfo*
_dbus_user_database_lookup (DBusUserDatabase *db,
                            dbus_uid_t        uid,
                            const DBusString *username,
                            DBusError        *error)
{
  DBusUserInfo *info;

  _DBUS_ASSERT_ERROR_IS_CLEAR (error);
  _dbus_assert (uid != DBUS_UID_UNSET || username != NULL);

  /* See if the username is really a number */
  if (uid == DBUS_UID_UNSET)
    {
      unsigned long n;

      if (_dbus_is_a_number (username, &n))
        uid = n;
    }

  if (uid != DBUS_UID_UNSET)
    info = _dbus_hash_table_lookup_ulong (db->users, uid);
  else
    info = _dbus_hash_table_lookup_string (db->users_by_name, _dbus_string_get_const_data (username));
  
  if (info)
    {
      _dbus_verbose ("Using cache for UID "DBUS_UID_FORMAT" information\n",
                     info->uid);
      return info;
    }
  else
    {
      if (uid != DBUS_UID_UNSET)
	_dbus_verbose ("No cache for UID "DBUS_UID_FORMAT"\n",
		       uid);
      else
	_dbus_verbose ("No cache for user \"%s\"\n",
		       _dbus_string_get_const_data (username));
      
      info = dbus_new0 (DBusUserInfo, 1);
      if (info == NULL)
        {
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          return NULL;
        }

      if (uid != DBUS_UID_UNSET)
        {
          if (!_dbus_user_info_fill_uid (info, uid, error))
            {
              _DBUS_ASSERT_ERROR_IS_SET (error);
              _dbus_user_info_free_allocated (info);
              return NULL;
            }
        }
      else
        {
          if (!_dbus_user_info_fill (info, username, error))
            {
              _DBUS_ASSERT_ERROR_IS_SET (error);
              _dbus_user_info_free_allocated (info);
              return NULL;
            }
        }

      /* be sure we don't use these after here */
      uid = DBUS_UID_UNSET;
      username = NULL;

      /* insert into hash */
      if (!_dbus_hash_table_insert_ulong (db->users, info->uid, info))
        {
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          _dbus_user_info_free_allocated (info);
          return NULL;
        }

      if (!_dbus_hash_table_insert_string (db->users_by_name,
                                           info->username,
                                           info))
        {
          _dbus_hash_table_remove_ulong (db->users, info->uid);
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          return NULL;
        }
      
      return info;
    }
}

_DBUS_DEFINE_GLOBAL_LOCK(system_users);
static dbus_bool_t database_locked = FALSE;
static DBusUserDatabase *system_db = NULL;
static DBusString process_username;
static DBusString process_homedir;
      
static void
shutdown_system_db (void *data)
{
  _dbus_user_database_unref (system_db);
  system_db = NULL;
  _dbus_string_free (&process_username);
  _dbus_string_free (&process_homedir);
}

static dbus_bool_t
init_system_db (void)
{
  _dbus_assert (database_locked);
    
  if (system_db == NULL)
    {
      DBusError error;
      const DBusUserInfo *info;
      
      system_db = _dbus_user_database_new ();
      if (system_db == NULL)
        return FALSE;

      dbus_error_init (&error);

      if (!_dbus_user_database_get_uid (system_db,
                                        _dbus_getuid (),
                                        &info,
                                        &error))
        {
          _dbus_user_database_unref (system_db);
          system_db = NULL;
          
          if (dbus_error_has_name (&error, DBUS_ERROR_NO_MEMORY))
            {
              dbus_error_free (&error);
              return FALSE;
            }
          else
            {
              /* This really should not happen. */
              _dbus_warn ("Could not get password database information for UID of current process: %s\n",
                          error.message);
              dbus_error_free (&error);
              return FALSE;
            }
        }

      if (!_dbus_string_init (&process_username))
        {
          _dbus_user_database_unref (system_db);
          system_db = NULL;
          return FALSE;
        }

      if (!_dbus_string_init (&process_homedir))
        {
          _dbus_string_free (&process_username);
          _dbus_user_database_unref (system_db);
          system_db = NULL;
          return FALSE;
        }

      if (!_dbus_string_append (&process_username,
                                info->username) ||
          !_dbus_string_append (&process_homedir,
                                info->homedir) ||
          !_dbus_register_shutdown_func (shutdown_system_db, NULL))
        {
          _dbus_string_free (&process_username);
          _dbus_string_free (&process_homedir);
          _dbus_user_database_unref (system_db);
          system_db = NULL;
          return FALSE;
        }
    }

  return TRUE;
}

void
_dbus_user_database_lock_system (void)
{
  _DBUS_LOCK (system_users);
  database_locked = TRUE;
}

void
_dbus_user_database_unlock_system (void)
{
  database_locked = FALSE;
  _DBUS_UNLOCK (system_users);
}

DBusUserDatabase*
_dbus_user_database_get_system (void)
{
  _dbus_assert (database_locked);

  init_system_db ();
  
  return system_db;
}

dbus_bool_t
_dbus_username_from_current_process (const DBusString **username)
{
  _dbus_user_database_lock_system ();
  if (!init_system_db ())
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }
  *username = &process_username;
  _dbus_user_database_unlock_system ();  

  return TRUE;
}

dbus_bool_t
_dbus_homedir_from_current_process (const DBusString  **homedir)
{
  _dbus_user_database_lock_system ();
  if (!init_system_db ())
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }
  *homedir = &process_homedir;
  _dbus_user_database_unlock_system ();

  return TRUE;
}

dbus_bool_t
_dbus_homedir_from_username (const DBusString *username,
                             DBusString       *homedir)
{
  DBusUserDatabase *db;
  const DBusUserInfo *info;
  _dbus_user_database_lock_system ();

  db = _dbus_user_database_get_system ();
  if (db == NULL)
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  if (!_dbus_user_database_get_username (db, username,
                                         &info, NULL))
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  if (!_dbus_string_append (homedir, info->homedir))
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }
  
  _dbus_user_database_unlock_system ();
  return TRUE;
}

dbus_bool_t
_dbus_credentials_from_username (const DBusString *username,
                                 DBusCredentials  *credentials)
{
  DBusUserDatabase *db;
  const DBusUserInfo *info;
  _dbus_user_database_lock_system ();

  db = _dbus_user_database_get_system ();
  if (db == NULL)
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  if (!_dbus_user_database_get_username (db, username,
                                         &info, NULL))
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  credentials->pid = DBUS_PID_UNSET;
  credentials->uid = info->uid;
  credentials->gid = info->primary_gid;
  
  _dbus_user_database_unlock_system ();
  return TRUE;
}

DBusUserDatabase*
_dbus_user_database_new (void)
{
  DBusUserDatabase *db;
  
  db = dbus_new0 (DBusUserDatabase, 1);
  if (db == NULL)
    return NULL;

  db->refcount = 1;

  db->users = _dbus_hash_table_new (DBUS_HASH_ULONG,
                                    NULL, (DBusFreeFunction) _dbus_user_info_free_allocated);
  
  if (db->users == NULL)
    goto failed;

  db->groups = _dbus_hash_table_new (DBUS_HASH_ULONG,
                                     NULL, (DBusFreeFunction) _dbus_group_info_free_allocated);
  
  if (db->groups == NULL)
    goto failed;

  db->users_by_name = _dbus_hash_table_new (DBUS_HASH_STRING,
                                            NULL, NULL);
  if (db->users_by_name == NULL)
    goto failed;
  
  db->groups_by_name = _dbus_hash_table_new (DBUS_HASH_STRING,
                                             NULL, NULL);
  if (db->groups_by_name == NULL)
    goto failed;
  
  return db;
  
 failed:
  _dbus_user_database_unref (db);
  return NULL;
}

void
_dbus_user_database_flush (DBusUserDatabase *db) 
{
  _dbus_hash_table_remove_all(db->users_by_name);
  _dbus_hash_table_remove_all(db->groups_by_name);
  _dbus_hash_table_remove_all(db->users);
  _dbus_hash_table_remove_all(db->groups);
}

#ifdef DBUS_BUILD_TESTS
DBusUserDatabase *
_dbus_user_database_ref (DBusUserDatabase  *db)
{
  _dbus_assert (db->refcount > 0);

  db->refcount += 1;

  return db;
}
#endif /* DBUS_BUILD_TESTS */

void
_dbus_user_database_unref (DBusUserDatabase  *db)
{
  _dbus_assert (db->refcount > 0);

  db->refcount -= 1;
  if (db->refcount == 0)
    {
      if (db->users)
        _dbus_hash_table_unref (db->users);

      if (db->groups)
        _dbus_hash_table_unref (db->groups);

      if (db->users_by_name)
        _dbus_hash_table_unref (db->users_by_name);

      if (db->groups_by_name)
        _dbus_hash_table_unref (db->groups_by_name);
      
      dbus_free (db);
    }
}

dbus_bool_t
_dbus_user_database_get_uid (DBusUserDatabase    *db,
                             dbus_uid_t           uid,
                             const DBusUserInfo **info,
                             DBusError           *error)
{
  *info = _dbus_user_database_lookup (db, uid, NULL, error);
  return *info != NULL;
}

dbus_bool_t
_dbus_user_database_get_username  (DBusUserDatabase     *db,
                                   const DBusString     *username,
                                   const DBusUserInfo  **info,
                                   DBusError            *error)
{
  *info = _dbus_user_database_lookup (db, DBUS_UID_UNSET, username, error);
  return *info != NULL;
}

/** @} */

/* Tests in dbus-userdb-util.c */
