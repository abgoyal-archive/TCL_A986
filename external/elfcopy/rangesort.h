#ifndef RANGESORT_H
#define RANGESORT_H

/* This implements a simple sorted list of non-overlapping ranges. */

#include <debug.h>
#include <common.h>
#include <gelf.h>

typedef enum range_error_t {
    ERROR_CONTAINS,
    ERROR_OVERLAPS
} range_error_t;

typedef struct range_t range_t;
struct range_t {
    GElf_Off start;
    GElf_Off length;
    void *user;
    void (*err_fn)(range_error_t, range_t *, range_t *);
    void (*user_dtor)(void *);
};

typedef struct range_list_t range_list_t;

range_list_t* init_range_list();
void destroy_range_list(range_list_t *);

void add_unique_range_nosort(range_list_t *ranges, 
                             GElf_Off start, GElf_Off length, 
                             void *user,
                             void (*err_fn)(range_error_t, range_t *, range_t *),
                             void (*user_dtor)(void * ));

range_list_t* sort_ranges(range_list_t *ranges);
range_t *find_range(range_list_t *ranges, GElf_Off value);
int get_num_ranges(const range_list_t *ranges);
range_t *get_sorted_ranges(const range_list_t *ranges, int *num_ranges);
GElf_Off get_last_address(const range_list_t *ranges);


typedef struct {
    int num_ranges;
    range_t *ranges;
} contiguous_range_info_t;

range_list_t* get_contiguous_ranges(const range_list_t *);


range_list_t* subtract_ranges(const range_list_t *r, const range_list_t *s);

#endif/*RANGESORT_H*/
