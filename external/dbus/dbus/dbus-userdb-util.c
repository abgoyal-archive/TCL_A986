
/* -*- mode: C; c-file-style: "gnu" -*- */
#define DBUS_USERDB_INCLUDES_PRIVATE 1
#include "dbus-userdb.h"
#include "dbus-test.h"
#include "dbus-internals.h"
#include "dbus-protocol.h"
#include <string.h>


dbus_bool_t
_dbus_is_console_user (dbus_uid_t uid,
		       DBusError *error)
{

  DBusUserDatabase *db;
  const DBusUserInfo *info;
  dbus_bool_t result = FALSE; 

#ifdef HAVE_CONSOLE_OWNER_FILE

  DBusString f;
  DBusStat st;

  if (!_dbus_string_init (&f))
    {
      _DBUS_SET_OOM (error);
      return FALSE;
    }

  if (!_dbus_string_append(&f, DBUS_CONSOLE_OWNER_FILE))
    {
      _dbus_string_free(&f);
      _DBUS_SET_OOM (error);
      return FALSE;
    }

  if (_dbus_stat(&f, &st, NULL) && (st.uid == uid))
    {
      _dbus_string_free(&f);
      return TRUE;
    }

  _dbus_string_free(&f);

#endif /* HAVE_CONSOLE_OWNER_FILE */

  _dbus_user_database_lock_system ();

  db = _dbus_user_database_get_system ();
  if (db == NULL)
    {
      dbus_set_error (error, DBUS_ERROR_FAILED, "Could not get system database.");
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  info = _dbus_user_database_lookup (db, uid, NULL, error);

  if (info == NULL)
    {
      _dbus_user_database_unlock_system ();
       return FALSE;
    }

  result = _dbus_user_at_console (info->username, error);

  _dbus_user_database_unlock_system ();

  return result;
}


dbus_bool_t
_dbus_credentials_from_uid (dbus_uid_t        uid,
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

  if (!_dbus_user_database_get_uid (db, uid,
                                    &info, NULL))
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  _dbus_assert (info->uid == uid);
  
  credentials->pid = DBUS_PID_UNSET;
  credentials->uid = info->uid;
  credentials->gid = info->primary_gid;
  
  _dbus_user_database_unlock_system ();
  return TRUE;
}


dbus_bool_t
_dbus_get_user_id (const DBusString  *username,
                   dbus_uid_t        *uid)
{
  DBusCredentials creds;

  if (!_dbus_credentials_from_username (username, &creds))
    return FALSE;

  if (creds.uid == DBUS_UID_UNSET)
    return FALSE;

  *uid = creds.uid;

  return TRUE;
}

dbus_bool_t
_dbus_get_group_id (const DBusString  *groupname,
                    dbus_gid_t        *gid)
{
  DBusUserDatabase *db;
  const DBusGroupInfo *info;
  _dbus_user_database_lock_system ();

  db = _dbus_user_database_get_system ();
  if (db == NULL)
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  if (!_dbus_user_database_get_groupname (db, groupname,
                                          &info, NULL))
    {
      _dbus_user_database_unlock_system ();
      return FALSE;
    }

  *gid = info->gid;
  
  _dbus_user_database_unlock_system ();
  return TRUE;
}

DBusGroupInfo*
_dbus_user_database_lookup_group (DBusUserDatabase *db,
                                  dbus_gid_t        gid,
                                  const DBusString *groupname,
                                  DBusError        *error)
{
  DBusGroupInfo *info;

  _DBUS_ASSERT_ERROR_IS_CLEAR (error);

   /* See if the group is really a number */
   if (gid == DBUS_UID_UNSET)
    {
      unsigned long n;

      if (_dbus_is_a_number (groupname, &n))
        gid = n;
    }


  if (gid != DBUS_GID_UNSET)
    info = _dbus_hash_table_lookup_ulong (db->groups, gid);
  else
    info = _dbus_hash_table_lookup_string (db->groups_by_name,
                                           _dbus_string_get_const_data (groupname));
  if (info)
    {
      _dbus_verbose ("Using cache for GID "DBUS_GID_FORMAT" information\n",
                     info->gid);
      return info;
    }
  else
    {
      if (gid != DBUS_GID_UNSET)
	_dbus_verbose ("No cache for GID "DBUS_GID_FORMAT"\n",
		       gid);
      else
	_dbus_verbose ("No cache for groupname \"%s\"\n",
		       _dbus_string_get_const_data (groupname));
      
      info = dbus_new0 (DBusGroupInfo, 1);
      if (info == NULL)
        {
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          return NULL;
        }

      if (gid != DBUS_GID_UNSET)
        {
          if (!_dbus_group_info_fill_gid (info, gid, error))
            {
              _DBUS_ASSERT_ERROR_IS_SET (error);
              _dbus_group_info_free_allocated (info);
              return NULL;
            }
        }
      else
        {
          if (!_dbus_group_info_fill (info, groupname, error))
            {
              _DBUS_ASSERT_ERROR_IS_SET (error);
              _dbus_group_info_free_allocated (info);
              return NULL;
            }
        }

      /* don't use these past here */
      gid = DBUS_GID_UNSET;
      groupname = NULL;

      if (!_dbus_hash_table_insert_ulong (db->groups, info->gid, info))
        {
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          _dbus_group_info_free_allocated (info);
          return NULL;
        }


      if (!_dbus_hash_table_insert_string (db->groups_by_name,
                                           info->groupname,
                                           info))
        {
          _dbus_hash_table_remove_ulong (db->groups, info->gid);
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          return NULL;
        }
      
      return info;
    }
}


dbus_bool_t
_dbus_user_database_get_groupname (DBusUserDatabase     *db,
                                   const DBusString     *groupname,
                                   const DBusGroupInfo **info,
                                   DBusError            *error)
{
  *info = _dbus_user_database_lookup_group (db, DBUS_GID_UNSET, groupname, error);
  return *info != NULL;
}

dbus_bool_t
_dbus_user_database_get_gid (DBusUserDatabase     *db,
                             dbus_gid_t            gid,
                             const DBusGroupInfo **info,
                             DBusError            *error)
{
  *info = _dbus_user_database_lookup_group (db, gid, NULL, error);
  return *info != NULL;
}


dbus_bool_t
_dbus_user_database_get_groups (DBusUserDatabase  *db,
                                dbus_uid_t         uid,
                                dbus_gid_t       **group_ids,
                                int               *n_group_ids,
                                DBusError         *error)
{
  DBusUserInfo *info;
  
  _DBUS_ASSERT_ERROR_IS_CLEAR (error);

  *group_ids = NULL;
  *n_group_ids = 0;
  
  info = _dbus_user_database_lookup (db, uid, NULL, error);
  if (info == NULL)
    {
      _DBUS_ASSERT_ERROR_IS_SET (error);
      return FALSE;
    }

  if (info->n_group_ids > 0)
    {
      *group_ids = dbus_new (dbus_gid_t, info->n_group_ids);
      if (*group_ids == NULL)
        {
          dbus_set_error (error, DBUS_ERROR_NO_MEMORY, NULL);
          return FALSE;
        }

      *n_group_ids = info->n_group_ids;

      memcpy (*group_ids, info->group_ids, info->n_group_ids * sizeof (dbus_gid_t));
    }

  return TRUE;
}

/** @} */

#ifdef DBUS_BUILD_TESTS
#include <stdio.h>

dbus_bool_t
_dbus_userdb_test (const char *test_data_dir)
{
  const DBusString *username;
  const DBusString *homedir;

  if (!_dbus_username_from_current_process (&username))
    _dbus_assert_not_reached ("didn't get username");

  if (!_dbus_homedir_from_current_process (&homedir))
    _dbus_assert_not_reached ("didn't get homedir");  

  printf ("    Current user: %s homedir: %s\n",
          _dbus_string_get_const_data (username),
          _dbus_string_get_const_data (homedir));
  
  return TRUE;
}
#endif /* DBUS_BUILD_TESTS */
