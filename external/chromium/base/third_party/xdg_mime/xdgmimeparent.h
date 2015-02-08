/* -*- mode: C; c-file-style: "gnu" -*- */

#ifndef __XDG_MIME_PARENT_H__
#define __XDG_MIME_PARENT_H__

#include "xdgmime.h"

typedef struct XdgParentList XdgParentList;

#ifdef XDG_PREFIX
#define _xdg_mime_parent_read_from_file        XDG_RESERVED_ENTRY(parent_read_from_file)
#define _xdg_mime_parent_list_new              XDG_RESERVED_ENTRY(parent_list_new)
#define _xdg_mime_parent_list_free             XDG_RESERVED_ENTRY(parent_list_free)
#define _xdg_mime_parent_list_lookup           XDG_RESERVED_ENTRY(parent_list_lookup)
#define _xdg_mime_parent_list_dump             XDG_RESERVED_ENTRY(parent_list_dump)
#endif

void          _xdg_mime_parent_read_from_file (XdgParentList *list,
					       const char    *file_name);
XdgParentList *_xdg_mime_parent_list_new       (void);
void           _xdg_mime_parent_list_free      (XdgParentList *list);
const char   **_xdg_mime_parent_list_lookup    (XdgParentList *list,
						const char    *mime);
void           _xdg_mime_parent_list_dump      (XdgParentList *list);

#endif /* __XDG_MIME_PARENT_H__ */
