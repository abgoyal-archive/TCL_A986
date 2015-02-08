
#if 1
#include <gnome.h>
#else
#include <stdio.h>
#endif

#include "GnomeGUISupport.h"

void GnomeGUISupport::postErrorMessage(const char *message, const char *title)
{
#if 1
  gchar *s;
  GtkWidget *error;

  s = g_strconcat(title, ":\n", message, NULL);
  error = gnome_error_dialog(s);
  gtk_widget_show(error);
  g_free(s);
#else
   fprintf(stderr, "%s: %s\n", title, message);
#endif
}


