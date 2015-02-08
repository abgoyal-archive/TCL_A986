
#ifndef _android_charmap_h
#define _android_charmap_h

#include "android/keycode.h"
#include "android/keycode-array.h"

/* this defines a structure used to describe an Android keyboard charmap */
typedef struct AKeyEntry {
    unsigned short  code;
    unsigned short  base;
    unsigned short  caps;
    unsigned short  fn;
    unsigned short  caps_fn;
    unsigned short  number;
} AKeyEntry;

/* Defines size of name buffer in AKeyCharmap entry. */
#define AKEYCHARMAP_NAME_SIZE   32

typedef struct AKeyCharmap {
    const AKeyEntry*  entries;
    int               num_entries;
    char              name[ AKEYCHARMAP_NAME_SIZE ];
} AKeyCharmap;

void kcm_extract_charmap_name(const char* kcm_file_path,
                              char* charmap_name,
                              int max_len);

int android_charmap_setup(const char* kcm_file_path);

/* Cleanups initialization performed in android_charmap_setup routine. */
void android_charmap_done(void);

const AKeyCharmap* android_get_charmap_by_name(const char* name);

const AKeyCharmap* android_get_charmap_by_index(unsigned int index);

int
android_charmap_reverse_map_unicode(const AKeyCharmap* cmap,
                                    unsigned int unicode,
                                    int  down,
                                    AKeycodeBuffer* keycodes);

#endif /* _android_charmap_h */
