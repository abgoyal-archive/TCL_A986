

#include "yaffs_tagsvalidity.h"

void yaffs_InitialiseTags(yaffs_ExtendedTags * tags)
{
	memset(tags, 0, sizeof(yaffs_ExtendedTags));
	tags->validMarker0 = 0xAAAAAAAA;
	tags->validMarker1 = 0x55555555;
}

int yaffs_ValidateTags(yaffs_ExtendedTags * tags)
{
	return (tags->validMarker0 == 0xAAAAAAAA &&
		tags->validMarker1 == 0x55555555);

}
