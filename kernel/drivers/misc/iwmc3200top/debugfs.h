

#ifndef __DEBUGFS_H__
#define __DEBUGFS_H__


#ifdef CONFIG_IWMC3200TOP_DEBUGFS

struct iwmct_debugfs {
	const char *name;
	struct dentry *dir_drv;
	struct dir_drv_files {
	} dbgfs_drv_files;
};

void iwmct_dbgfs_register(struct iwmct_priv *priv, const char *name);
void iwmct_dbgfs_unregister(struct iwmct_debugfs *dbgfs);

#else /* CONFIG_IWMC3200TOP_DEBUGFS */

struct iwmct_debugfs;

static inline void
iwmct_dbgfs_register(struct iwmct_priv *priv, const char *name)
{}

static inline void
iwmct_dbgfs_unregister(struct iwmct_debugfs *dbgfs)
{}

#endif /* CONFIG_IWMC3200TOP_DEBUGFS */

#endif /* __DEBUGFS_H__ */

