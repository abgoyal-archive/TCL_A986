

#include <linux/input.h>
#include <linux/input/sparse-keymap.h>
#include <linux/slab.h>

MODULE_AUTHOR("Dmitry Torokhov <dtor@mail.ru>");
MODULE_DESCRIPTION("Generic support for sparse keymaps");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");

struct key_entry *sparse_keymap_entry_from_scancode(struct input_dev *dev,
						    unsigned int code)
{
	struct key_entry *key;

	for (key = dev->keycode; key->type != KE_END; key++)
		if (code == key->code)
			return key;

	return NULL;
}
EXPORT_SYMBOL(sparse_keymap_entry_from_scancode);

struct key_entry *sparse_keymap_entry_from_keycode(struct input_dev *dev,
						   unsigned int keycode)
{
	struct key_entry *key;

	for (key = dev->keycode; key->type != KE_END; key++)
		if (key->type == KE_KEY && keycode == key->keycode)
			return key;

	return NULL;
}
EXPORT_SYMBOL(sparse_keymap_entry_from_keycode);

static int sparse_keymap_getkeycode(struct input_dev *dev,
				    unsigned int scancode,
				    unsigned int *keycode)
{
	const struct key_entry *key;

	if (dev->keycode) {
		key = sparse_keymap_entry_from_scancode(dev, scancode);
		if (key && key->type == KE_KEY) {
			*keycode = key->keycode;
			return 0;
		}
	}

	return -EINVAL;
}

static int sparse_keymap_setkeycode(struct input_dev *dev,
				    unsigned int scancode,
				    unsigned int keycode)
{
	struct key_entry *key;
	int old_keycode;

	if (dev->keycode) {
		key = sparse_keymap_entry_from_scancode(dev, scancode);
		if (key && key->type == KE_KEY) {
			old_keycode = key->keycode;
			key->keycode = keycode;
			set_bit(keycode, dev->keybit);
			if (!sparse_keymap_entry_from_keycode(dev, old_keycode))
				clear_bit(old_keycode, dev->keybit);
			return 0;
		}
	}

	return -EINVAL;
}

int sparse_keymap_setup(struct input_dev *dev,
			const struct key_entry *keymap,
			int (*setup)(struct input_dev *, struct key_entry *))
{
	size_t map_size = 1; /* to account for the last KE_END entry */
	const struct key_entry *e;
	struct key_entry *map, *entry;
	int i;
	int error;

	for (e = keymap; e->type != KE_END; e++)
		map_size++;

	map = kcalloc(map_size, sizeof (struct key_entry), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	memcpy(map, keymap, map_size * sizeof (struct key_entry));

	for (i = 0; i < map_size; i++) {
		entry = &map[i];

		if (setup) {
			error = setup(dev, entry);
			if (error)
				goto err_out;
		}

		switch (entry->type) {
		case KE_KEY:
			__set_bit(EV_KEY, dev->evbit);
			__set_bit(entry->keycode, dev->keybit);
			break;

		case KE_SW:
			__set_bit(EV_SW, dev->evbit);
			__set_bit(entry->sw.code, dev->swbit);
			break;
		}
	}

	dev->keycode = map;
	dev->keycodemax = map_size;
	dev->getkeycode = sparse_keymap_getkeycode;
	dev->setkeycode = sparse_keymap_setkeycode;

	return 0;

 err_out:
	kfree(map);
	return error;

}
EXPORT_SYMBOL(sparse_keymap_setup);

void sparse_keymap_free(struct input_dev *dev)
{
	unsigned long flags;

	/*
	 * Take event lock to prevent racing with input_get_keycode()
	 * and input_set_keycode() if we are called while input device
	 * is still registered.
	 */
	spin_lock_irqsave(&dev->event_lock, flags);

	kfree(dev->keycode);
	dev->keycode = NULL;
	dev->keycodemax = 0;

	spin_unlock_irqrestore(&dev->event_lock, flags);
}
EXPORT_SYMBOL(sparse_keymap_free);

void sparse_keymap_report_entry(struct input_dev *dev, const struct key_entry *ke,
				unsigned int value, bool autorelease)
{
	switch (ke->type) {
	case KE_KEY:
		input_report_key(dev, ke->keycode, value);
		input_sync(dev);
		if (value && autorelease) {
			input_report_key(dev, ke->keycode, 0);
			input_sync(dev);
		}
		break;

	case KE_SW:
		value = ke->sw.value;
		/* fall through */

	case KE_VSW:
		input_report_switch(dev, ke->sw.code, value);
		break;
	}
}
EXPORT_SYMBOL(sparse_keymap_report_entry);

bool sparse_keymap_report_event(struct input_dev *dev, unsigned int code,
				unsigned int value, bool autorelease)
{
	const struct key_entry *ke =
		sparse_keymap_entry_from_scancode(dev, code);

	if (ke) {
		sparse_keymap_report_entry(dev, ke, value, autorelease);
		return true;
	}

	return false;
}
EXPORT_SYMBOL(sparse_keymap_report_event);

