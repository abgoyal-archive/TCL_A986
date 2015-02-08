

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>


static int get_range(char **str, int *pint)
{
	int x, inc_counter, upper_range;

	(*str)++;
	upper_range = simple_strtol((*str), NULL, 0);
	inc_counter = upper_range - *pint;
	for (x = *pint; x < upper_range; x++)
		*pint++ = x;
	return inc_counter;
}


int get_option (char **str, int *pint)
{
	char *cur = *str;

	if (!cur || !(*cur))
		return 0;
	*pint = simple_strtol (cur, str, 0);
	if (cur == *str)
		return 0;
	if (**str == ',') {
		(*str)++;
		return 2;
	}
	if (**str == '-')
		return 3;

	return 1;
}

 
char *get_options(const char *str, int nints, int *ints)
{
	int res, i = 1;

	while (i < nints) {
		res = get_option ((char **)&str, ints + i);
		if (res == 0)
			break;
		if (res == 3) {
			int range_nums;
			range_nums = get_range((char **)&str, ints + i);
			if (range_nums < 0)
				break;
			/*
			 * Decrement the result by one to leave out the
			 * last number in the range.  The next iteration
			 * will handle the upper number in the range
			 */
			i += (range_nums - 1);
		}
		i++;
		if (res == 1)
			break;
	}
	ints[0] = i - 1;
	return (char *)str;
}


unsigned long long memparse(const char *ptr, char **retptr)
{
	char *endptr;	/* local pointer to end of parsed string */

	unsigned long long ret = simple_strtoull(ptr, &endptr, 0);

	switch (*endptr) {
	case 'G':
	case 'g':
		ret <<= 10;
	case 'M':
	case 'm':
		ret <<= 10;
	case 'K':
	case 'k':
		ret <<= 10;
		endptr++;
	default:
		break;
	}

	if (retptr)
		*retptr = endptr;

	return ret;
}


EXPORT_SYMBOL(memparse);
EXPORT_SYMBOL(get_option);
EXPORT_SYMBOL(get_options);
