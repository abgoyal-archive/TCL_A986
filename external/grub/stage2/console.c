/* term_console.c - console input and output */

#include <shared.h>
#include <term.h>


int console_current_color = A_NORMAL;
static int console_standard_color = A_NORMAL;
static int console_normal_color = A_NORMAL;
static int console_highlight_color = A_REVERSE;
static color_state console_color_state = COLOR_STATE_STANDARD;

void
console_setcolorstate (color_state state)
{
  switch (state) {
    case COLOR_STATE_STANDARD:
      console_current_color = console_standard_color;
      break;
    case COLOR_STATE_NORMAL:
      console_current_color = console_normal_color;
      break;
    case COLOR_STATE_HIGHLIGHT:
      console_current_color = console_highlight_color;
      break;
    default:
      console_current_color = console_standard_color;
      break;
  }

  console_color_state = state;
}

void
console_setcolor (int normal_color, int highlight_color)
{
  console_normal_color = normal_color;
  console_highlight_color = highlight_color;

  console_setcolorstate (console_color_state);
}
