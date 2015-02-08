

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uuid.h>
#include <linux/random.h>

static void __uuid_gen_common(__u8 b[16])
{
	int i;
	u32 r;

	for (i = 0; i < 4; i++) {
		r = random32();
		memcpy(b + i * 4, &r, 4);
	}
	/* reversion 0b10 */
	b[8] = (b[8] & 0x3F) | 0x80;
}

void uuid_le_gen(uuid_le *lu)
{
	__uuid_gen_common(lu->b);
	/* version 4 : random generation */
	lu->b[7] = (lu->b[7] & 0x0F) | 0x40;
}
EXPORT_SYMBOL_GPL(uuid_le_gen);

void uuid_be_gen(uuid_be *bu)
{
	__uuid_gen_common(bu->b);
	/* version 4 : random generation */
	bu->b[6] = (bu->b[6] & 0x0F) | 0x40;
}
EXPORT_SYMBOL_GPL(uuid_be_gen);
