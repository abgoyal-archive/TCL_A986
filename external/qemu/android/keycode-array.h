
#ifndef QEMU_ANDROID_KEYCODE_ARRAY_H
#define QEMU_ANDROID_KEYCODE_ARRAY_H


/* Maximum number of keycodes kept in the array. */
#define  MAX_KEYCODES   256*2

/* Describes array of keycodes collected for transferring to the core. */
typedef struct AKeycodeBuffer {
    /* Number of keycodes collected in the array. */
    int                 keycode_count;

    /* Array of collected keycodes. */
    int                 keycodes[ MAX_KEYCODES ];
} AKeycodeBuffer;

/* Adds a key event to the array of keycodes. */
void
android_keycodes_add_key_event( AKeycodeBuffer* keycodes,
                                unsigned       code,
                                unsigned       down );

/* Flushes (transfers) collected keycodes to the core. */
void
android_keycodes_flush(AKeycodeBuffer* keycodes);

#endif /* QEMU_ANDROID_KEYCODE_ARRAY_H */
