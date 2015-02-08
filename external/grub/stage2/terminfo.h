/* terminfo.h - read a terminfo entry from the command line */

#ifndef GRUB_TERMCAP_HEADER
#define GRUB_TERMCAP_HEADER	1

#define TERMINFO_LEN 40

typedef struct terminfo
{
  char name[TERMINFO_LEN];
  char cursor_address[TERMINFO_LEN];
  char clear_screen[TERMINFO_LEN];
  char enter_standout_mode[TERMINFO_LEN];
  char exit_standout_mode[TERMINFO_LEN];
}
terminfo;


/* Function prototypes.  */
char *ti_escape_memory (const char *in, const char *end);
char *ti_escape_string (const char *in);
char *ti_unescape_memory (const char *in, const char *end);
char *ti_unescape_string (const char *in);

void ti_set_term (const struct terminfo *new);
void ti_get_term (struct terminfo *copy);

void ti_cursor_address (int x, int y);
void ti_clear_screen (void);
void ti_enter_standout_mode (void);
void ti_exit_standout_mode (void);

#endif /* ! GRUB_TERMCAP_HEADER */
