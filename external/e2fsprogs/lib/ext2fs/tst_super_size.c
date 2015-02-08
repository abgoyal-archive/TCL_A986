
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "ext2_fs.h"

int main(int argc, char **argv)
{
	int s = sizeof(struct ext2_super_block);

	printf("Size of struct ext2_super_block is %d\n", s);
	if (s != 1024) {
		exit(1);
	}
	exit(0);
}
		
