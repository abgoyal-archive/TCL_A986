
#ifndef _SPARSE_KEYMAP_H
#define _SPARSE_KEYMAP_H


#define KE_END		0	/* Indicates end of keymap */
#define KE_KEY		1	/* Ordinary key/button */
#define KE_SW		2	/* Switch (predetermined value) */
#define KE_VSW		3	/* Switch (value supplied at runtime) */
#define KE_IGNORE	4	/* Known entry that should be ignored */
#define KE_LAST		KE_IGNORE

struct key_entry {
	int type;		/* See KE_* above */
	u32 code;
	union {
		u16 keycode;		/* For KE_KEY */
		struct {		/* For KE_SW, KE_VSW */
			u8 code;
			u8 value;	/* For KE_SW, ignored by KE_VSW */
		} sw;
	};
};

struct key_entry *sparse_keymap_entry_from_scancode(struct input_dev *dev,
						    unsigned int code);
struct key_entry *sparse_keymap_entry_from_keycode(struct input_dev *dev,
						   unsigned int code);
int sparse_keymap_setup(struct input_dev *dev,
			const struct key_entry *keymap,
			int (*setup)(struct input_dev *, struct key_entry *));
void sparse_keymap_free(struct input_dev *dev);

void sparse_keymap_report_entry(struct input_dev *dev, const struct key_entry *ke,
				unsigned int value, bool autorelease);

bool sparse_keymap_report_event(struct input_dev *dev, unsigned int code,
				unsigned int value, bool autorelease);

#endif /* _SPARSE_KEYMAP_H */
