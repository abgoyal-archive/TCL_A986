

#include <linux/flex_array.h>
#include <linux/slab.h>
#include <linux/stddef.h>

struct flex_array_part {
	char elements[FLEX_ARRAY_PART_SIZE];
};

static inline int elements_fit_in_base(struct flex_array *fa)
{
	int data_size = fa->element_size * fa->total_nr_elements;
	if (data_size <= FLEX_ARRAY_BASE_BYTES_LEFT)
		return 1;
	return 0;
}

struct flex_array *flex_array_alloc(int element_size, unsigned int total,
					gfp_t flags)
{
	struct flex_array *ret;
	int max_size = FLEX_ARRAY_NR_BASE_PTRS *
				FLEX_ARRAY_ELEMENTS_PER_PART(element_size);

	/* max_size will end up 0 if element_size > PAGE_SIZE */
	if (total > max_size)
		return NULL;
	ret = kzalloc(sizeof(struct flex_array), flags);
	if (!ret)
		return NULL;
	ret->element_size = element_size;
	ret->total_nr_elements = total;
	if (elements_fit_in_base(ret) && !(flags & __GFP_ZERO))
		memset(&ret->parts[0], FLEX_ARRAY_FREE,
						FLEX_ARRAY_BASE_BYTES_LEFT);
	return ret;
}

static int fa_element_to_part_nr(struct flex_array *fa,
					unsigned int element_nr)
{
	return element_nr / FLEX_ARRAY_ELEMENTS_PER_PART(fa->element_size);
}

void flex_array_free_parts(struct flex_array *fa)
{
	int part_nr;

	if (elements_fit_in_base(fa))
		return;
	for (part_nr = 0; part_nr < FLEX_ARRAY_NR_BASE_PTRS; part_nr++)
		kfree(fa->parts[part_nr]);
}

void flex_array_free(struct flex_array *fa)
{
	flex_array_free_parts(fa);
	kfree(fa);
}

static unsigned int index_inside_part(struct flex_array *fa,
					unsigned int element_nr)
{
	unsigned int part_offset;

	part_offset = element_nr %
				FLEX_ARRAY_ELEMENTS_PER_PART(fa->element_size);
	return part_offset * fa->element_size;
}

static struct flex_array_part *
__fa_get_part(struct flex_array *fa, int part_nr, gfp_t flags)
{
	struct flex_array_part *part = fa->parts[part_nr];
	if (!part) {
		part = kmalloc(sizeof(struct flex_array_part), flags);
		if (!part)
			return NULL;
		if (!(flags & __GFP_ZERO))
			memset(part, FLEX_ARRAY_FREE,
				sizeof(struct flex_array_part));
		fa->parts[part_nr] = part;
	}
	return part;
}

int flex_array_put(struct flex_array *fa, unsigned int element_nr, void *src,
			gfp_t flags)
{
	int part_nr = fa_element_to_part_nr(fa, element_nr);
	struct flex_array_part *part;
	void *dst;

	if (element_nr >= fa->total_nr_elements)
		return -ENOSPC;
	if (elements_fit_in_base(fa))
		part = (struct flex_array_part *)&fa->parts[0];
	else {
		part = __fa_get_part(fa, part_nr, flags);
		if (!part)
			return -ENOMEM;
	}
	dst = &part->elements[index_inside_part(fa, element_nr)];
	memcpy(dst, src, fa->element_size);
	return 0;
}

int flex_array_clear(struct flex_array *fa, unsigned int element_nr)
{
	int part_nr = fa_element_to_part_nr(fa, element_nr);
	struct flex_array_part *part;
	void *dst;

	if (element_nr >= fa->total_nr_elements)
		return -ENOSPC;
	if (elements_fit_in_base(fa))
		part = (struct flex_array_part *)&fa->parts[0];
	else {
		part = fa->parts[part_nr];
		if (!part)
			return -EINVAL;
	}
	dst = &part->elements[index_inside_part(fa, element_nr)];
	memset(dst, FLEX_ARRAY_FREE, fa->element_size);
	return 0;
}

int flex_array_prealloc(struct flex_array *fa, unsigned int start,
			unsigned int end, gfp_t flags)
{
	int start_part;
	int end_part;
	int part_nr;
	struct flex_array_part *part;

	if (start >= fa->total_nr_elements || end >= fa->total_nr_elements)
		return -ENOSPC;
	if (elements_fit_in_base(fa))
		return 0;
	start_part = fa_element_to_part_nr(fa, start);
	end_part = fa_element_to_part_nr(fa, end);
	for (part_nr = start_part; part_nr <= end_part; part_nr++) {
		part = __fa_get_part(fa, part_nr, flags);
		if (!part)
			return -ENOMEM;
	}
	return 0;
}

void *flex_array_get(struct flex_array *fa, unsigned int element_nr)
{
	int part_nr = fa_element_to_part_nr(fa, element_nr);
	struct flex_array_part *part;

	if (element_nr >= fa->total_nr_elements)
		return NULL;
	if (elements_fit_in_base(fa))
		part = (struct flex_array_part *)&fa->parts[0];
	else {
		part = fa->parts[part_nr];
		if (!part)
			return NULL;
	}
	return &part->elements[index_inside_part(fa, element_nr)];
}

static int part_is_free(struct flex_array_part *part)
{
	int i;

	for (i = 0; i < sizeof(struct flex_array_part); i++)
		if (part->elements[i] != FLEX_ARRAY_FREE)
			return 0;
	return 1;
}

int flex_array_shrink(struct flex_array *fa)
{
	struct flex_array_part *part;
	int part_nr;
	int ret = 0;

	if (elements_fit_in_base(fa))
		return ret;
	for (part_nr = 0; part_nr < FLEX_ARRAY_NR_BASE_PTRS; part_nr++) {
		part = fa->parts[part_nr];
		if (!part)
			continue;
		if (part_is_free(part)) {
			fa->parts[part_nr] = NULL;
			kfree(part);
			ret++;
		}
	}
	return ret;
}
