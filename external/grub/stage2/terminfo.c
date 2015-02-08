/* terminfo.c - read a terminfo entry from the command line */

#include <shared.h>
#include "terminfo.h"
#include "tparm.h"
#include "serial.h"

/* Current terminal capabilities. Default is "vt100".  */
struct terminfo term =
  {
    .name                = "vt100",
    .cursor_address      = "\e[%i%p1%d;%p2%dH",
    .clear_screen        = "\e[H\e[J",
    .enter_standout_mode = "\e[7m",
    .exit_standout_mode  = "\e[m"
  };


char *
ti_unescape_memory (const char *in, const char *end) 
{
  static char out_buffer[256];
  char c;
  char *out;

  out = out_buffer;
  do
    {
      c = *(in++);
      switch (c) 
	{
	case '^':
	  if (*in >= 'A' && *in <= 'Z')
	    {
	      *out = (*in) - 'A';
	      in++;
	    }
	  else
	    {
	      *out = '^';
	    }
	  break;
	case '\\':
	  c = *(in++);
	  if (c >= '0' && c <= '9')
	    {
	      // octal number
	      int n = 0;
	      do
		{
		  n = (n << 4) | (c - '0');
		  c = *(in++);
		}
	      while (c >= '0' && c <= '9');
	      
	      *out++ = (char)(n & 0xff);
	      
	      // redo last character
	      in--;
	      
	      break;
	    } 

	  switch (c) 
	    {
	    case 'e':
	    case 'E':
	      *out++ = '\e';
	      break;
	    case 'n':
	      *out++ = '\n';
	      break;
	    case 'r':
	      *out++ = '\r';
	      break;
	    case 't':
	      *out++ = '\t';
	      break;
	    case 'b':
	      *out++ = '\b';
	      break;
	    case 'f':
	      *out++ = '\f';
	      break;
	    case 's':
	      *out++ = ' ';
	      break;
	    case '\\':
	      *out++ = '\\';
	      break;
	    case '^':
	      *out++ = '^';
	      break;
	    case ',':
	      *out++ = ',';
	      break;
	    case ':':
	      *out++ = ':';
	      break;
	    case '0':
	      *out++ = '\200';
	      break;
	    }
	  break;
	default:
	  *out++ = c;
	  break;
	}
    }
  while (in <= end);
  
  return out_buffer;
}

char *
ti_unescape_string (const char *in) 
{
  return ti_unescape_memory (in, in + grub_strlen (in));
}

char *
ti_escape_memory (const char *in, const char *end) 
{
  static char out_buffer[256];
  char c;
  char *out;

  out = out_buffer;
  do
    {
      c = *(in++);
      switch (c)
	{
	case '\e':
	  *out++ = '\\'; *out++ = 'e'; break;
	case ' ':
	  *out++ = '\\'; *out++ = 's'; break;
	case '\\':
	  *out++ = '\\'; *out++ = '\\'; break;
	case '0' ... '9':
	case 'a' ... 'z':
	case 'A' ... 'Z':
	case '%':
	case '+':
	case '-':
	case '*':
	case '/':
	case ';':
	case ':':
	case '{':
	case '}':
	case '[':
	case ']':
	  *out++ = c; break;
	case 0 ... 25:
	  *out++ = '^'; *out++ = 'A' + c; break;
	default:
	  *out++ = '\\'; 
	  *out++ = ((c >> 8) & 7) + '0';
	  *out++ = ((c >> 4) & 7) + '0';
	  *out++ = ((c >> 0) & 7) + '0';
	  break;
	}
    }
  while (in < end);
  
  *out++ = 0;
  
  return out_buffer;
}

char *
ti_escape_string (const char *in) 
{
  return ti_escape_memory (in, in + grub_strlen (in));
}

/* move the cursor to the given position starting with "0". */
void
ti_cursor_address (int x, int y)
{
  grub_putstr (grub_tparm (term.cursor_address, y, x));
}

/* clear the screen. */
void 
ti_clear_screen (void)
{
  grub_putstr (grub_tparm (term.clear_screen));
}

/* enter reverse video */
void 
ti_enter_standout_mode (void)
{
  grub_putstr (grub_tparm (term.enter_standout_mode));
}

/* exit reverse video */
void 
ti_exit_standout_mode (void)
{
  grub_putstr (grub_tparm (term.exit_standout_mode));
}

/* set the current terminal emulation to use */
void 
ti_set_term (const struct terminfo *new)
{
  grub_memmove (&term, new, sizeof (struct terminfo));
}

/* get the current terminal emulation */
void
ti_get_term(struct terminfo *copy)
{
  grub_memmove (copy, &term, sizeof (struct terminfo));
}
