

#include <crypto/twofish.h>
#include <linux/crypto.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>

asmlinkage void twofish_enc_blk(struct crypto_tfm *tfm, u8 *dst, const u8 *src);
asmlinkage void twofish_dec_blk(struct crypto_tfm *tfm, u8 *dst, const u8 *src);

static void twofish_encrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	twofish_enc_blk(tfm, dst, src);
}

static void twofish_decrypt(struct crypto_tfm *tfm, u8 *dst, const u8 *src)
{
	twofish_dec_blk(tfm, dst, src);
}

static struct crypto_alg alg = {
	.cra_name		=	"twofish",
	.cra_driver_name	=	"twofish-asm",
	.cra_priority		=	200,
	.cra_flags		=	CRYPTO_ALG_TYPE_CIPHER,
	.cra_blocksize		=	TF_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct twofish_ctx),
	.cra_alignmask		=	3,
	.cra_module		=	THIS_MODULE,
	.cra_list		=	LIST_HEAD_INIT(alg.cra_list),
	.cra_u			=	{
		.cipher = {
			.cia_min_keysize	=	TF_MIN_KEY_SIZE,
			.cia_max_keysize	=	TF_MAX_KEY_SIZE,
			.cia_setkey		=	twofish_setkey,
			.cia_encrypt		=	twofish_encrypt,
			.cia_decrypt		=	twofish_decrypt
		}
	}
};

static int __init init(void)
{
	return crypto_register_alg(&alg);
}

static void __exit fini(void)
{
	crypto_unregister_alg(&alg);
}

module_init(init);
module_exit(fini);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION ("Twofish Cipher Algorithm, asm optimized");
MODULE_ALIAS("twofish");
MODULE_ALIAS("twofish-asm");
