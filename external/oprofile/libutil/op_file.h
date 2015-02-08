
#ifndef OP_FILE_H
#define OP_FILE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>

int op_file_readable(char const * file);

time_t op_get_mtime(char const * file);

int create_dir(char const * dir);


int create_path(char const * path);

typedef void (*get_pathname_callback)(char const * pathname, void * name_list);

enum recursion_type {
	NO_RECURSION = 2,
	MATCH_ANY_ENTRY_RECURSION = 4,
	MATCH_DIR_ONLY_RECURSION = 8,
};
int get_matching_pathnames(void * name_list, get_pathname_callback,
			   char const * base_dir, char const * filter,
			   enum recursion_type recursion);


#ifdef __cplusplus
}
#endif

#endif /* OP_FILE_H */
