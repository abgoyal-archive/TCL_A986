
/* -*- Mode: C; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */


#ifndef _GTK_DRAWING_H_
#define _GTK_DRAWING_H_

#include <gtk/gtk.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*** type definitions ***/
typedef struct {
  guint8 active;
  guint8 focused;
  guint8 inHover;
  guint8 disabled;
  guint8 isDefault;
  guint8 canDefault;
  /* The depressed state is for buttons which remain active for a longer period:
   * activated toggle buttons or buttons showing a popup menu. */
  guint8 depressed;
  gint32 curpos; /* curpos and maxpos are used for scrollbars */
  gint32 maxpos;
} GtkWidgetState;

typedef struct {
  gint slider_width;
  gint trough_border;
  gint stepper_size;
  gint stepper_spacing;
  gint min_slider_size;
} MozGtkScrollbarMetrics;

typedef struct _GtkThemeParts {
    GdkColormap* colormap;
    GtkWidget* protoWindow;
    GtkWidget* protoLayout;
    GtkWidget* buttonWidget;
    GtkWidget* toggleButtonWidget;
    GtkWidget* buttonArrowWidget;
    GtkWidget* checkboxWidget;
    GtkWidget* radiobuttonWidget;
    GtkWidget* horizScrollbarWidget;
    GtkWidget* vertScrollbarWidget;
    GtkWidget* spinWidget;
    GtkWidget* hScaleWidget;
    GtkWidget* vScaleWidget;
    GtkWidget* entryWidget;
    GtkWidget* comboBoxWidget;
    GtkWidget* comboBoxButtonWidget;
    GtkWidget* comboBoxArrowWidget;
    GtkWidget* comboBoxSeparatorWidget;
    GtkWidget* comboBoxEntryWidget;
    GtkWidget* comboBoxEntryTextareaWidget;
    GtkWidget* comboBoxEntryButtonWidget;
    GtkWidget* comboBoxEntryArrowWidget;
    GtkWidget* handleBoxWidget;
    GtkWidget* toolbarWidget;
    GtkWidget* frameWidget;
    GtkWidget* statusbarWidget;
    GtkWidget* progresWidget;
    GtkWidget* tabWidget;
    GtkWidget* tooltipWidget;
    GtkWidget* menuBarWidget;
    GtkWidget* menuBarItemWidget;
    GtkWidget* menuPopupWidget;
    GtkWidget* menuItemWidget;
    GtkWidget* imageMenuItemWidget;
    GtkWidget* checkMenuItemWidget;
    GtkWidget* treeViewWidget;
    GtkTreeViewColumn* middleTreeViewColumn;
    GtkWidget* treeHeaderCellWidget;
    GtkWidget* treeHeaderSortArrowWidget;
    GtkWidget* expanderWidget;
    GtkWidget* toolbarSeparatorWidget;
    GtkWidget* menuSeparatorWidget;
    GtkWidget* hpanedWidget;
    GtkWidget* vpanedWidget;
    GtkWidget* scrolledWindowWidget;
} GtkThemeParts;

typedef enum {
  MOZ_GTK_STEPPER_DOWN        = 1 << 0,
  MOZ_GTK_STEPPER_BOTTOM      = 1 << 1,
  MOZ_GTK_STEPPER_VERTICAL    = 1 << 2
} GtkScrollbarButtonFlags;

/** flags for tab state **/
typedef enum {
  /* first eight bits are used to pass a margin */
  MOZ_GTK_TAB_MARGIN_MASK     = 0xFF,
  /* bottom tabs */
  MOZ_GTK_TAB_BOTTOM          = 1 << 8,
  /* the first tab in the group */
  MOZ_GTK_TAB_FIRST           = 1 << 9,
  /* the selected tab */
  MOZ_GTK_TAB_SELECTED        = 1 << 10
} GtkTabFlags;

/** flags for menuitems **/
typedef enum {
  /* menuitem is part of the menubar */
  MOZ_TOPLEVEL_MENU_ITEM      = 1 << 0
} GtkMenuItemFlags;

/* function type for moz_gtk_enable_style_props */
typedef gint (*style_prop_t)(GtkStyle*, const gchar*, gint);

/*** result/error codes ***/
#define MOZ_GTK_SUCCESS 0
#define MOZ_GTK_UNKNOWN_WIDGET -1
#define MOZ_GTK_UNSAFE_THEME -2

/*** checkbox/radio flags ***/
#define MOZ_GTK_WIDGET_CHECKED 1
#define MOZ_GTK_WIDGET_INCONSISTENT (1 << 1)

/*** widget type constants ***/
typedef enum {
  /* Paints a GtkButton. flags is a GtkReliefStyle. */
  MOZ_GTK_BUTTON,
  /* Paints a GtkCheckButton. flags is a boolean, 1=checked, 0=not checked. */
  MOZ_GTK_CHECKBUTTON,
  /* Paints a GtkRadioButton. flags is a boolean, 1=checked, 0=not checked. */
  MOZ_GTK_RADIOBUTTON,
  /**
   * Paints the button of a GtkScrollbar. flags is a GtkArrowType giving
   * the arrow direction.
   */
  MOZ_GTK_SCROLLBAR_BUTTON,
  /* Paints the trough (track) of a GtkScrollbar. */
  MOZ_GTK_SCROLLBAR_TRACK_HORIZONTAL,
  MOZ_GTK_SCROLLBAR_TRACK_VERTICAL,
  /* Paints the slider (thumb) of a GtkScrollbar. */
  MOZ_GTK_SCROLLBAR_THUMB_HORIZONTAL,
  MOZ_GTK_SCROLLBAR_THUMB_VERTICAL,
  /* Paints a GtkScale. */
  MOZ_GTK_SCALE_HORIZONTAL,
  MOZ_GTK_SCALE_VERTICAL,
  /* Paints a GtkScale thumb. */
  MOZ_GTK_SCALE_THUMB_HORIZONTAL,
  MOZ_GTK_SCALE_THUMB_VERTICAL,
  /* Paints a GtkSpinButton */
  MOZ_GTK_SPINBUTTON,
  MOZ_GTK_SPINBUTTON_UP,
  MOZ_GTK_SPINBUTTON_DOWN,
  MOZ_GTK_SPINBUTTON_ENTRY,
  /* Paints the gripper of a GtkHandleBox. */
  MOZ_GTK_GRIPPER,
  /* Paints a GtkEntry. */
  MOZ_GTK_ENTRY,
  /* Paints the native caret (or in GTK-speak: insertion cursor) */
  MOZ_GTK_ENTRY_CARET,
  /* Paints a GtkOptionMenu. */
  MOZ_GTK_DROPDOWN,
  /* Paints a dropdown arrow (a GtkButton containing a down GtkArrow). */
  MOZ_GTK_DROPDOWN_ARROW,
  /* Paints an entry in an editable option menu */
  MOZ_GTK_DROPDOWN_ENTRY,
  /* Paints the container part of a GtkCheckButton. */
  MOZ_GTK_CHECKBUTTON_CONTAINER,
  /* Paints the container part of a GtkRadioButton. */
  MOZ_GTK_RADIOBUTTON_CONTAINER,
  /* Paints the label of a GtkCheckButton (focus outline) */
  MOZ_GTK_CHECKBUTTON_LABEL,
  /* Paints the label of a GtkRadioButton (focus outline) */
  MOZ_GTK_RADIOBUTTON_LABEL,
  /* Paints the background of a GtkHandleBox. */
  MOZ_GTK_TOOLBAR,
  /* Paints a toolbar separator */
  MOZ_GTK_TOOLBAR_SEPARATOR,
  /* Paints a GtkToolTip */
  MOZ_GTK_TOOLTIP,
  /* Paints a GtkFrame (e.g. a status bar panel). */
  MOZ_GTK_FRAME,
  /* Paints a resize grip for a GtkWindow */
  MOZ_GTK_RESIZER,
  /* Paints a GtkProgressBar. */
  MOZ_GTK_PROGRESSBAR,
  /* Paints a progress chunk of a GtkProgressBar. */
  MOZ_GTK_PROGRESS_CHUNK,
  /* Paints a tab of a GtkNotebook. flags is a GtkTabFlags, defined above. */
  MOZ_GTK_TAB,
  /* Paints the background and border of a GtkNotebook. */
  MOZ_GTK_TABPANELS,
  /* Paints a GtkArrow for a GtkNotebook. flags is a GtkArrowType. */
  MOZ_GTK_TAB_SCROLLARROW,
  /* Paints the background and border of a GtkTreeView */
  MOZ_GTK_TREEVIEW,
  /* Paints treeheader cells */
  MOZ_GTK_TREE_HEADER_CELL,
  /* Paints sort arrows in treeheader cells */
  MOZ_GTK_TREE_HEADER_SORTARROW,
  /* Paints an expander for a GtkTreeView */
  MOZ_GTK_TREEVIEW_EXPANDER,
  /* Paints a GtkExpander */
  MOZ_GTK_EXPANDER,
  /* Paints the background of the menu bar. */
  MOZ_GTK_MENUBAR,
  /* Paints the background of menus, context menus. */
  MOZ_GTK_MENUPOPUP,
  /* Paints the arrow of menuitems that contain submenus */
  MOZ_GTK_MENUARROW,
  /* Paints an arrow that points down */
  MOZ_GTK_TOOLBARBUTTON_ARROW,
  /* Paints items of menubar and popups. */
  MOZ_GTK_MENUITEM,
  MOZ_GTK_CHECKMENUITEM,
  MOZ_GTK_RADIOMENUITEM,
  MOZ_GTK_MENUSEPARATOR,
  /* Paints a GtkVPaned separator */
  MOZ_GTK_SPLITTER_HORIZONTAL,
  /* Paints a GtkHPaned separator */
  MOZ_GTK_SPLITTER_VERTICAL,
  /* Paints the background of a window, dialog or page. */
  MOZ_GTK_WINDOW
} GtkThemeWidgetType;

/*** General library functions ***/
gint moz_gtk_init();

void
moz_gtk_use_theme_parts(GtkThemeParts* parts);

gint moz_gtk_enable_style_props(style_prop_t styleGetProp);

gint moz_gtk_shutdown();

void moz_gtk_destroy_theme_parts_widgets(GtkThemeParts* parts);

/*** Widget drawing ***/
gint
moz_gtk_widget_paint(GtkThemeWidgetType widget, GdkDrawable* drawable,
                     GdkRectangle* rect, GdkRectangle* cliprect,
                     GtkWidgetState* state, gint flags,
                     GtkTextDirection direction);


/*** Widget metrics ***/
gint moz_gtk_get_widget_border(GtkThemeWidgetType widget, gint* left, gint* top, 
                               gint* right, gint* bottom, GtkTextDirection direction,
                               gboolean inhtml);

gint
moz_gtk_checkbox_get_metrics(gint* indicator_size, gint* indicator_spacing);

gint
moz_gtk_radio_get_metrics(gint* indicator_size, gint* indicator_spacing);

gint
moz_gtk_button_get_inner_border(GtkWidget* widget, GtkBorder* inner_border);

gint
moz_gtk_widget_get_focus(GtkWidget* widget, gboolean* interior_focus,
                         gint* focus_width, gint* focus_pad);

gint
moz_gtk_get_scalethumb_metrics(GtkOrientation orient, gint* thumb_length, gint* thumb_height);

gint
moz_gtk_get_scrollbar_metrics(MozGtkScrollbarMetrics* metrics);

gint moz_gtk_get_combo_box_entry_button_size(gint* width, gint* height);

gint moz_gtk_get_tab_scroll_arrow_size(gint* width, gint* height);

gint moz_gtk_get_downarrow_size(gint* width, gint* height);

gint moz_gtk_get_toolbar_separator_width(gint* size);

gint moz_gtk_get_expander_size(gint* size);

gint moz_gtk_get_treeview_expander_size(gint* size);

gint moz_gtk_get_menu_separator_height(gint* size);

gint moz_gtk_splitter_get_metrics(gint orientation, gint* size);

GtkWidget* moz_gtk_get_scrollbar_widget(void);

gint moz_gtk_get_tab_thickness(void);

gboolean moz_gtk_images_in_menus(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
