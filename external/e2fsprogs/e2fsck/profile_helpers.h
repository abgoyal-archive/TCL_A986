
long profile_get_values
	(profile_t profile, const char *const *names, char ***ret_values);

void profile_free_list
	(char **list);

long profile_get_relation_names
	(profile_t profile, const char **names, char ***ret_names);

long profile_get_subsection_names
	(profile_t profile, const char **names, char ***ret_names);

void profile_release_string (char *str);

long profile_init_path
	(const char * filelist, profile_t *ret_profile);

