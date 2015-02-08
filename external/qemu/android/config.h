
#ifndef ANDROID_CONFIG_H
#define ANDROID_CONFIG_H

typedef struct AConfig  AConfig;

struct AConfig
{
    AConfig*     next;
    AConfig*     first_child;
    AConfig*     last_child;
    const char*  name;
    const char*  value;
};

/* parse a text string into a config node tree */
extern void   aconfig_load(AConfig*  root, char*  data);

/* parse a file into a config node tree, return 0 in case of success, -1 otherwise */
extern int    aconfig_load_file(AConfig*  root, const char*  path);

/* save a config node tree into a file, return 0 in case of success, -1 otherwise */
extern int    aconfig_save_file(AConfig*  root, const char* path);

/* create a single config node */
extern AConfig*  aconfig_node(const char *name, const char *value);

/* locate a named child of a config node */
extern AConfig*  aconfig_find(AConfig *root, const char *name);

/* add a named child to a config node (or modify it if it already exists) */
extern void      aconfig_set(AConfig *root, const char *name, const char *value);


extern int          aconfig_bool    (AConfig *root, const char *name, int _default);
extern unsigned     aconfig_unsigned(AConfig *root, const char *name, unsigned _default);
extern int          aconfig_int     (AConfig *root, const char *name, int _default);
extern const char*  aconfig_str     (AConfig *root, const char *name, const char *_default);

#endif /* ANDROID_CONFIG_H */
