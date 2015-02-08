

#include <linux/errno.h>

#include <asm/page.h>

#include "mlx4.h"
#include "icm.h"

int mlx4_pd_alloc(struct mlx4_dev *dev, u32 *pdn)
{
	struct mlx4_priv *priv = mlx4_priv(dev);

	*pdn = mlx4_bitmap_alloc(&priv->pd_bitmap);
	if (*pdn == -1)
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL_GPL(mlx4_pd_alloc);

void mlx4_pd_free(struct mlx4_dev *dev, u32 pdn)
{
	mlx4_bitmap_free(&mlx4_priv(dev)->pd_bitmap, pdn);
}
EXPORT_SYMBOL_GPL(mlx4_pd_free);

int mlx4_init_pd_table(struct mlx4_dev *dev)
{
	struct mlx4_priv *priv = mlx4_priv(dev);

	return mlx4_bitmap_init(&priv->pd_bitmap, dev->caps.num_pds,
				(1 << 24) - 1, dev->caps.reserved_pds, 0);
}

void mlx4_cleanup_pd_table(struct mlx4_dev *dev)
{
	mlx4_bitmap_cleanup(&mlx4_priv(dev)->pd_bitmap);
}


int mlx4_uar_alloc(struct mlx4_dev *dev, struct mlx4_uar *uar)
{
	uar->index = mlx4_bitmap_alloc(&mlx4_priv(dev)->uar_table.bitmap);
	if (uar->index == -1)
		return -ENOMEM;

	uar->pfn = (pci_resource_start(dev->pdev, 2) >> PAGE_SHIFT) + uar->index;

	return 0;
}
EXPORT_SYMBOL_GPL(mlx4_uar_alloc);

void mlx4_uar_free(struct mlx4_dev *dev, struct mlx4_uar *uar)
{
	mlx4_bitmap_free(&mlx4_priv(dev)->uar_table.bitmap, uar->index);
}
EXPORT_SYMBOL_GPL(mlx4_uar_free);

int mlx4_init_uar_table(struct mlx4_dev *dev)
{
	if (dev->caps.num_uars <= 128) {
		mlx4_err(dev, "Only %d UAR pages (need more than 128)\n",
			 dev->caps.num_uars);
		mlx4_err(dev, "Increase firmware log2_uar_bar_megabytes?\n");
		return -ENODEV;
	}

	return mlx4_bitmap_init(&mlx4_priv(dev)->uar_table.bitmap,
				dev->caps.num_uars, dev->caps.num_uars - 1,
				max(128, dev->caps.reserved_uars), 0);
}

void mlx4_cleanup_uar_table(struct mlx4_dev *dev)
{
	mlx4_bitmap_cleanup(&mlx4_priv(dev)->uar_table.bitmap);
}
