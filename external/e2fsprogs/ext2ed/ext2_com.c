
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ext2ed.h"

void type_ext2___super (char *command_line)


{
	char buffer [80];
	
	super_info.copy_num=0;
	sprintf (buffer,"setoffset %ld",file_system_info.super_block_offset);dispatch (buffer);
	sprintf (buffer,"settype ext2_super_block");dispatch (buffer);
}

void type_ext2___cd (char *command_line)


{
	char temp [80],buffer [80],*ptr;
	
	ptr=parse_word (command_line,buffer);
	if (*ptr==0) {
		wprintw (command_win,"Error - No argument specified\n");refresh_command_win ();return;
	}
	ptr=parse_word (ptr,buffer);
	
	if (buffer [0] != '/') {
		wprintw (command_win,"Error - Use a full pathname (begin with '/')\n");refresh_command_win ();return;
	}

	/* Note the various dispatches below - They should be intuitive if you know the ext2 filesystem structure */

	dispatch ("super");dispatch ("group");dispatch ("inode");dispatch ("next");dispatch ("dir");
	if (buffer [1] != 0) {
		sprintf (temp,"cd %s",buffer+1);dispatch (temp);
	}
}

void type_ext2___group (char *command_line)


{
	long group_num=0;
	char *ptr,buffer [80];
	
	ptr=parse_word (command_line,buffer);
	if (*ptr!=0) {
		ptr=parse_word (ptr,buffer);
		group_num=atol (buffer);
	}

	group_info.copy_num=0;group_info.group_num=0;
	sprintf (buffer,"setoffset %ld",file_system_info.first_group_desc_offset);dispatch (buffer);
	sprintf (buffer,"settype ext2_group_desc");dispatch (buffer);
	sprintf (buffer,"entry %ld",group_num);dispatch (buffer);
}
