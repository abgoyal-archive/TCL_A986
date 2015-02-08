
#ifndef __ASM_ARCH_PXA27x_KEYPAD_H
#define __ASM_ARCH_PXA27x_KEYPAD_H

#include <linux/input.h>
#include <linux/input/matrix_keypad.h>

#define MAX_MATRIX_KEY_ROWS	(8)
#define MAX_MATRIX_KEY_COLS	(8)
#define MATRIX_ROW_SHIFT	(3)
#define MAX_DIRECT_KEY_NUM	(8)

struct pxa27x_keypad_platform_data {

	/* code map for the matrix keys */
	unsigned int	matrix_key_rows;
	unsigned int	matrix_key_cols;
	unsigned int	*matrix_key_map;
	int		matrix_key_map_size;

	/* direct keys */
	int		direct_key_num;
	unsigned int	direct_key_map[MAX_DIRECT_KEY_NUM];

	/* rotary encoders 0 */
	int		enable_rotary0;
	int		rotary0_rel_code;
	int		rotary0_up_key;
	int		rotary0_down_key;

	/* rotary encoders 1 */
	int		enable_rotary1;
	int		rotary1_rel_code;
	int		rotary1_up_key;
	int		rotary1_down_key;

	/* key debounce interval */
	unsigned int	debounce_interval;
};

extern void pxa_set_keypad_info(struct pxa27x_keypad_platform_data *info);

#endif /* __ASM_ARCH_PXA27x_KEYPAD_H */
