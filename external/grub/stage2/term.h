/* term.h - definitions for terminal handling */

#ifndef GRUB_TERM_HEADER
#define GRUB_TERM_HEADER	1

/* These are used to represent the various color states we use */
typedef enum
{
  /* represents the color used to display all text that does not use the user
   * defined colors below
   */
  COLOR_STATE_STANDARD,
  /* represents the user defined colors for normal text */
  COLOR_STATE_NORMAL,
  /* represents the user defined colors for highlighted text */
  COLOR_STATE_HIGHLIGHT
} color_state;

#ifndef STAGE1_5

/* Flags for representing the capabilities of a terminal.  */

/* Set when input characters shouldn't be echoed back.  */
#define TERM_NO_ECHO		(1 << 0)
/* Set when the editing feature should be disabled.  */
#define TERM_NO_EDIT		(1 << 1)
/* Set when the terminal cannot do fancy things.  */
#define TERM_DUMB		(1 << 2)
/* Set when the terminal needs to be initialized.  */
#define TERM_NEED_INIT		(1 << 16)

struct term_entry
{
  /* The name of a terminal.  */
  const char *name;
  /* The feature flags defined above.  */
  unsigned long flags;
  /* Put a character.  */
  void (*putchar) (int c);
  /* Check if any input character is available.  */
  int (*checkkey) (void);
  /* Get a character.  */
  int (*getkey) (void);
  /* Get the cursor position. The return value is ((X << 8) | Y).  */
  int (*getxy) (void);
  /* Go to the position (X, Y).  */
  void (*gotoxy) (int x, int y);
  /* Clear the screen.  */
  void (*cls) (void);
  /* Set the current color to be used */
  void (*setcolorstate) (color_state state);
  /* Set the normal color and the highlight color. The format of each
     color is VGA's.  */
  void (*setcolor) (int normal_color, int highlight_color);
  /* Turn on/off the cursor.  */
  int (*setcursor) (int on);
};

/* This lists up available terminals.  */
extern struct term_entry term_table[];
extern struct term_entry *current_term;

#endif /* ! STAGE1_5 */

/* The console stuff.  */
extern int console_current_color;
void console_putchar (int c);

#ifndef STAGE1_5
int console_checkkey (void);
int console_getkey (void);
int console_getxy (void);
void console_gotoxy (int x, int y);
void console_cls (void);
void console_setcolorstate (color_state state);
void console_setcolor (int normal_color, int highlight_color);
int console_setcursor (int on);
#endif

#ifdef SUPPORT_SERIAL
void serial_putchar (int c);
int serial_checkkey (void);
int serial_getkey (void);
int serial_getxy (void);
void serial_gotoxy (int x, int y);
void serial_cls (void);
void serial_setcolorstate (color_state state);
#endif

#ifdef SUPPORT_HERCULES
void hercules_putchar (int c);
int hercules_getxy (void);
void hercules_gotoxy (int x, int y);
void hercules_cls (void);
void hercules_setcolorstate (color_state state);
void hercules_setcolor (int normal_color, int highlight_color);
int hercules_setcursor (int on);
#endif

#endif /* ! GRUB_TERM_HEADER */
