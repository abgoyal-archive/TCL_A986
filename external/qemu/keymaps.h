

#ifndef __QEMU_KEYMAPS_H__
#define __QEMU_KEYMAPS_H__

#include "qemu-common.h"

typedef struct {
	const char* name;
	int keysym;
} name2keysym_t;

struct key_range {
    int start;
    int end;
    struct key_range *next;
};

#define MAX_NORMAL_KEYCODE 512
#define MAX_EXTRA_COUNT 256
typedef struct {
    uint16_t keysym2keycode[MAX_NORMAL_KEYCODE];
    struct {
	int keysym;
	uint16_t keycode;
    } keysym2keycode_extra[MAX_EXTRA_COUNT];
    int extra_count;
    struct key_range *keypad_range;
    struct key_range *numlock_range;
} kbd_layout_t;


void *init_keyboard_layout(const name2keysym_t *table, const char *language);
int keysym2scancode(void *kbd_layout, int keysym);
int keycode_is_keypad(void *kbd_layout, int keycode);
int keysym_is_numlock(void *kbd_layout, int keysym);

#endif /* __QEMU_KEYMAPS_H__ */
