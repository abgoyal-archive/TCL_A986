

#ifndef __BFA_FWIMG_PRIV_H__
#define __BFA_FWIMG_PRIV_H__

#define	BFI_FLASH_CHUNK_SZ		256	/*  Flash chunk size */
#define	BFI_FLASH_CHUNK_SZ_WORDS	(BFI_FLASH_CHUNK_SZ/sizeof(u32))

extern u32 *bfi_image_ct_get_chunk(u32 off);
extern u32 bfi_image_ct_size;
extern u32 *bfi_image_cb_get_chunk(u32 off);
extern u32 bfi_image_cb_size;
extern u32 *bfi_image_cb;
extern u32 *bfi_image_ct;

#endif /* __BFA_FWIMG_PRIV_H__ */
