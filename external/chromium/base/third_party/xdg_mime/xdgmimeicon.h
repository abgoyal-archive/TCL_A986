/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef __XDG_MIME_ICON_H__
#define __XDG_MIME_ICON_H__

#include "xdgmime.h"

typedef struct XdgIconList XdgIconList;

#ifdef XDG_PREFIX
#define _xdg_mime_icon_read_from_file        XDG_ENTRY(icon_read_from_file)
#define _xdg_mime_icon_list_new              XDG_ENTRY(icon_list_new)
#define _xdg_mime_icon_list_free             XDG_ENTRY(icon_list_free)
#define _xdg_mime_icon_list_lookup           XDG_ENTRY(icon_list_lookup)
#define _xdg_mime_icon_list_dump             XDG_ENTRY(icon_list_dump)
#endif

void          _xdg_mime_icon_read_from_file (XdgIconList *list,
					    const char   *file_name);
XdgIconList  *_xdg_mime_icon_list_new       (void);
void          _xdg_mime_icon_list_free      (XdgIconList *list);
const char   *_xdg_mime_icon_list_lookup    (XdgIconList *list,
					     const char  *mime);
void          _xdg_mime_icon_list_dump      (XdgIconList *list);

#endif /* __XDG_MIME_ICON_H__ */
