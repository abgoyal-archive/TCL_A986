
/* -*- mode: C; c-file-style: "gnu" -*- */

#include <string.h>
#include "dbus-internals.h"
#include "dbus-list.h"
#include "dbus-memory.h"
#include "dbus-protocol.h"
#include "dbus-shell.h"
#include "dbus-string.h"


static dbus_bool_t
unquote_string_inplace (char* str, char** end)
{
  char* dest;
  char* s;
  char quote_char;
  
  dest = s = str;

  quote_char = *s;
  
  if (!(*s == '"' || *s == '\''))
    {
      *end = str;
      return FALSE;
    }

  /* Skip the initial quote mark */
  ++s;

  if (quote_char == '"')
    {
      while (*s)
        {
          _dbus_assert(s > dest); /* loop invariant */
      
          switch (*s)
            {
            case '"':
              /* End of the string, return now */
              *dest = '\0';
              ++s;
              *end = s;
              return TRUE;

            case '\\':
              /* Possible escaped quote or \ */
              ++s;
              switch (*s)
                {
                case '"':
                case '\\':
                case '`':
                case '$':
                case '\n':
                  *dest = *s;
                  ++s;
                  ++dest;
                  break;

                default:
                  /* not an escaped char */
                  *dest = '\\';
                  ++dest;
                  /* ++s already done. */
                  break;
                }
              break;

            default:
              *dest = *s;
              ++dest;
              ++s;
              break;
            }

          _dbus_assert(s > dest); /* loop invariant */
        }
    }
  else
    {
      while (*s)
        {
          _dbus_assert(s > dest); /* loop invariant */
          
          if (*s == '\'')
            {
              /* End of the string, return now */
              *dest = '\0';
              ++s;
              *end = s;
              return TRUE;
            }
          else
            {
              *dest = *s;
              ++dest;
              ++s;
            }

          _dbus_assert(s > dest); /* loop invariant */
        }
    }
  
  /* If we reach here this means the close quote was never encountered */

  *dest = '\0';
  
  *end = s;
  return FALSE;
}

char*
_dbus_shell_quote (const char *unquoted_string)
{
  /* We always use single quotes, because the algorithm is cheesier.
   * We could use double if we felt like it, that might be more
   * human-readable.
   */

  const char *p;
  char *ret;
  DBusString dest;
  
  _dbus_string_init (&dest);

  p = unquoted_string;

  /* could speed this up a lot by appending chunks of text at a
   * time.
   */
  while (*p)
    {
      /* Replace literal ' with a close ', a \', and a open ' */
      if (*p == '\'')
	{
	  if (!_dbus_string_append (&dest, "'\\''"))
	    {
	      _dbus_string_free (&dest);
	      return NULL;
	    }
	}
      else
	{
	  if (!_dbus_string_append_byte (&dest, *p))
	    {
	      _dbus_string_free (&dest);
	      return NULL;
	    }
	}

      ++p;
    }

  /* close the quote */
  if (_dbus_string_append_byte (&dest, '\''))
    {
      ret = _dbus_strdup (_dbus_string_get_data (&dest));
      _dbus_string_free (&dest);

      return ret;
    }

  _dbus_string_free (&dest);

  return NULL;
}

char*
_dbus_shell_unquote (const char *quoted_string)
{
  char *unquoted;
  char *end;
  char *start;
  char *ret;
  DBusString retval;

  unquoted = _dbus_strdup (quoted_string);
  if (unquoted == NULL)
    return NULL;

  start = unquoted;
  end = unquoted;
  if (!_dbus_string_init (&retval))
    {
      dbus_free (unquoted);
      return NULL;
    }

  /* The loop allows cases such as
   * "foo"blah blah'bar'woo foo"baz"la la la\'\''foo'
   */
  while (*start)
    {
      /* Append all non-quoted chars, honoring backslash escape
       */
      
      while (*start && !(*start == '"' || *start == '\''))
        {
          if (*start == '\\')
            {
              /* all characters can get escaped by backslash,
               * except newline, which is removed if it follows
               * a backslash outside of quotes
               */
              
              ++start;
              if (*start)
                {
                  if (*start != '\n')
		    {
		      if (!_dbus_string_append_byte (&retval, *start))
			goto error;
		    }
                  ++start;
                }
            }
          else
            {
              if (!_dbus_string_append_byte (&retval, *start))
		goto error;
              ++start;
            }
        }

      if (*start)
        {
          if (!unquote_string_inplace (start, &end))
	    goto error;
          else
            {
              if (!_dbus_string_append (&retval, start))
		goto error;
              start = end;
            }
        }
    }

  ret = _dbus_strdup (_dbus_string_get_data (&retval));
  if (!ret)
    goto error;

  dbus_free (unquoted);
  _dbus_string_free (&retval);
  
  return ret;
  
 error:
  dbus_free (unquoted);
  _dbus_string_free (&retval);
  return NULL;
}


static dbus_bool_t
delimit_token (DBusString *token,
               DBusList **retval,
	       DBusError *error)
{
  char *str;

  str = _dbus_strdup (_dbus_string_get_data (token));
  if (!str)
    {
      _DBUS_SET_OOM (error);
      return FALSE;
    }

  if (!_dbus_list_append (retval, str))
    {
      dbus_free (str);
      _DBUS_SET_OOM (error);
      return FALSE;
    }

  return TRUE;
}

static DBusList*
tokenize_command_line (const char *command_line, DBusError *error)
{
  char current_quote;
  const char *p;
  DBusString current_token;
  DBusList *retval = NULL;
  dbus_bool_t quoted;;

  current_quote = '\0';
  quoted = FALSE;
  p = command_line;

  if (!_dbus_string_init (&current_token))
    {
      _DBUS_SET_OOM (error);
      return NULL;
    }

  while (*p)
    {
      if (current_quote == '\\')
        {
          if (*p == '\n')
            {
              /* we append nothing; backslash-newline become nothing */
            }
          else
            {
	      if (!_dbus_string_append_byte (&current_token, '\\') || 
	          !_dbus_string_append_byte (&current_token, *p))
		{
		  _DBUS_SET_OOM (error);
		  goto error;
		}
            }

          current_quote = '\0';
        }
      else if (current_quote == '#')
        {
          /* Discard up to and including next newline */
          while (*p && *p != '\n')
            ++p;

          current_quote = '\0';
          
          if (*p == '\0')
            break;
        }
      else if (current_quote)
        {
          if (*p == current_quote &&
              /* check that it isn't an escaped double quote */
              !(current_quote == '"' && quoted))
            {
              /* close the quote */
              current_quote = '\0';
            }

          /* Everything inside quotes, and the close quote,
           * gets appended literally.
           */

          if (!_dbus_string_append_byte (&current_token, *p))
	    {
	      _DBUS_SET_OOM (error);
	      goto error;
	    }
        }
      else
        {
          switch (*p)
            {
            case '\n':
              if (!delimit_token (&current_token, &retval, error))
		goto error;

		_dbus_string_free (&current_token);

		if (!_dbus_string_init (&current_token))
		  {
		    _DBUS_SET_OOM (error);
		    goto init_error;
		  }

              break;

            case ' ':
            case '\t':
              /* If the current token contains the previous char, delimit
               * the current token. A nonzero length
               * token should always contain the previous char.
               */
              if (_dbus_string_get_length (&current_token) > 0)
                {
                  if (!delimit_token (&current_token, &retval, error))
		    goto error;

		  _dbus_string_free (&current_token);

		  if (!_dbus_string_init (&current_token))
		    {
		      _DBUS_SET_OOM (error);
		      goto init_error;
		    }

                }
              
              /* discard all unquoted blanks (don't add them to a token) */
              break;


              /* single/double quotes are appended to the token,
               * escapes are maybe appended next time through the loop,
               * comment chars are never appended.
               */
              
            case '\'':
            case '"':
              if (!_dbus_string_append_byte (&current_token, *p))
		{
		  _DBUS_SET_OOM (error);
		  goto error;
		}

              /* FALL THRU */
              
            case '#':
            case '\\':
              current_quote = *p;
              break;

            default:
              /* Combines rules 4) and 6) - if we have a token, append to it,
               * otherwise create a new token.
               */
              if (!_dbus_string_append_byte (&current_token, *p))
		{
		  _DBUS_SET_OOM (error);
		  goto error;
		}
              break;
            }
        }

      /* We need to count consecutive backslashes mod 2, 
       * to detect escaped doublequotes.
       */
      if (*p != '\\')
	quoted = FALSE;
      else
	quoted = !quoted;

      ++p;
    }

  if (!delimit_token (&current_token, &retval, error))
    goto error;

  if (current_quote)
    {
      dbus_set_error_const (error, DBUS_ERROR_INVALID_ARGS, "Unclosed quotes in command line");
      goto error;
    }

  if (retval == NULL)
    {
      dbus_set_error_const (error, DBUS_ERROR_INVALID_ARGS, "No tokens found in command line");
      goto error;
    }
 
  _dbus_string_free (&current_token);
 
  return retval;

 error:
  _dbus_string_free (&current_token);

 init_error:
  if (retval)
    {
      _dbus_list_foreach (&retval, (DBusForeachFunction) dbus_free, NULL);
      _dbus_list_clear (&retval);
    }

  return NULL;
}

dbus_bool_t
_dbus_shell_parse_argv (const char *command_line,
			int        *argcp,
			char     ***argvp,
			DBusError  *error)
{
  /* Code based on poptParseArgvString() from libpopt */
  int argc = 0;
  char **argv = NULL;
  DBusList *tokens = NULL;
  int i;
  DBusList *tmp_list;
  
  if (!command_line)
    {
      _dbus_verbose ("Command line is NULL\n");
      return FALSE;
    }

  tokens = tokenize_command_line (command_line, error);
  if (tokens == NULL)
    {
      _dbus_verbose ("No tokens for command line '%s'\n", command_line);
      return FALSE;
    }

  /* Because we can't have introduced any new blank space into the
   * tokens (we didn't do any new expansions), we don't need to
   * perform field splitting. If we were going to honor IFS or do any
   * expansions, we would have to do field splitting on each word
   * here. Also, if we were going to do any expansion we would need to
   * remove any zero-length words that didn't contain quotes
   * originally; but since there's no expansion we know all words have
   * nonzero length, unless they contain quotes.
   * 
   * So, we simply remove quotes, and don't do any field splitting or
   * empty word removal, since we know there was no way to introduce
   * such things.
   */

  argc = _dbus_list_get_length (&tokens);
  argv = dbus_new (char *, argc + 1);
  if (!argv)
    {
      _DBUS_SET_OOM (error);
      goto error;
    }

  i = 0;
  tmp_list = tokens;
  while (tmp_list)
    {
      argv[i] = _dbus_shell_unquote (tmp_list->data);

      if (!argv[i])
        {
          int j;
	  for (j = 0; j < i; j++)
	    dbus_free(argv[j]);

          dbus_free (argv);
	  _DBUS_SET_OOM (error);
	  goto error;
        }

      tmp_list = _dbus_list_get_next_link (&tokens, tmp_list);
      ++i;
    }
  argv[argc] = NULL;
  
  _dbus_list_foreach (&tokens, (DBusForeachFunction) dbus_free, NULL);
  _dbus_list_clear (&tokens);
  
  if (argcp)
    *argcp = argc;

  if (argvp)
    *argvp = argv;
  else
    dbus_free_string_array (argv);

  return TRUE;

 error:
  _dbus_list_foreach (&tokens, (DBusForeachFunction) dbus_free, NULL);
  _dbus_list_clear (&tokens);

  return FALSE;

}
