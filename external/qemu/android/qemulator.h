

#ifndef QEMU_ANDROID_QEMULATOR_H
#define QEMU_ANDROID_QEMULATOR_H

#include "android/config.h"
#include "android/skin/file.h"
#include "android/skin/keyboard.h"
#include "android/skin/window.h"
#include "android/cmdline-option.h"

typedef struct {
    AConfig*       aconfig;
    SkinFile*      layout_file;
    SkinLayout*    layout;
    SkinKeyboard*  keyboard;
    SkinWindow*    window;
    int            win_x;
    int            win_y;
    int            show_trackball;
    SkinTrackBall* trackball;
    int            lcd_brightness;
    SkinImage*     onion;
    SkinRotation   onion_rotation;
    int            onion_alpha;

    AndroidOptions  opts[1];  /* copy of options */
} QEmulator;

/* Gets a pointer to a QEmulator structure instance. */
QEmulator*
qemulator_get(void);

/* Initializes QEmulator structure instance. */
int
qemulator_init( QEmulator*       emulator,
                AConfig*         aconfig,
                const char*      basepath,
                int              x,
                int              y,
                AndroidOptions*  opts );

/* Uninitializes QEmulator structure instance on exit. */
void
qemulator_done( QEmulator* emulator );

/* Sets title on the emulator's window. */
void
qemulator_set_title( QEmulator* emulator );

/* Gets emulator's layout. */
SkinLayout*
qemulator_get_layout( QEmulator* emulator );

/* A helper routine for getting device DPI. */
int
get_device_dpi( AndroidOptions*  opts );

#endif  // QEMU_ANDROID_QEMULATOR_H
