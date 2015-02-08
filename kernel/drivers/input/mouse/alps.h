

#ifndef _ALPS_H
#define _ALPS_H

struct alps_model_info {
        unsigned char signature[3];
        unsigned char byte0, mask0;
        unsigned char flags;
};

struct alps_data {
	struct input_dev *dev2;		/* Relative device */
	char phys[32];			/* Phys */
	const struct alps_model_info *i;/* Info */
	int prev_fin;			/* Finger bit from previous packet */
	struct timer_list timer;
};

#ifdef CONFIG_MOUSE_PS2_ALPS
int alps_detect(struct psmouse *psmouse, bool set_properties);
int alps_init(struct psmouse *psmouse);
#else
inline int alps_detect(struct psmouse *psmouse, bool set_properties)
{
	return -ENOSYS;
}
inline int alps_init(struct psmouse *psmouse)
{
	return -ENOSYS;
}
#endif /* CONFIG_MOUSE_PS2_ALPS */

#endif
