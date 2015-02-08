
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "ext2ed.h"

int write_access;

int low_read (unsigned char *buffer,unsigned long length,unsigned long offset)


{

#ifdef DEBUG

	char temp [80];

	if (device_handle==NULL) {					/* Check that a device is indeed open */
		internal_error ("No device opened yet read requested","disk","low_read");
		return (0);
	}
	if (offset > file_system_info.file_system_size) {		/* Check that the offset is within limits */
		sprintf (temp,"Seek offset %ld is out of range",offset);
		internal_error (temp,"disk","low_read");
		return (0);
	}

#endif	

	if ( (fseek (device_handle,offset,SEEK_SET))==-1) {		/* Seek to the required offset */
		wprintw (command_win,"Error - Failed to seek to offset %ld in device %s\n",offset,device_name);
		refresh_command_win ();
		return (0);
	};

	if ( (fread (buffer,1,length,device_handle))==-1) {		/* And do the actual reading */
		wprintw (command_win,"Error - Failed to read from offset %ld in device %s\n",offset,device_name);
		refresh_command_win ();return (0);
	};
	
	return (1);
}

int low_write (unsigned char *buffer,unsigned long length,unsigned long offset)

{
	char temp [80];
	
	if (!write_access) {
		wprintw (command_win,"Error - Write access not aviable (use enablewrite)\n");
		return (0);
	}

#ifdef DEBUG

	if (!AllowChanges) {
		internal_error ("AllowChanges=0 yet enablewrite succeeded","disk","low_write");
		return (0);
	}
	
	if (device_handle==NULL) {
		internal_error ("No device opened yet read requested","disk","low_write");
		return (0);
	}

	if (offset > file_system_info.file_system_size) {
		sprintf (temp,"Seek offset %ld is out of range",offset);
		internal_error (temp,"disk","low_write");
		return (0);
	}

#endif	

	if (LogChanges)
		if (!log_changes (buffer,length,offset))
			return (0);

	if ( (fseek (device_handle,offset,SEEK_SET))==-1) {
		wprintw (command_win,"Error - Failed to seek to offset %ld in device %s\n",offset,device_name);
		refresh_command_win ();return (0);
	};


	if ( (fwrite (buffer,1,length,device_handle))==-1) {
		wprintw (command_win,"Error - Failed to write to offset %ld in device %s\n",offset,device_name);
		refresh_command_win ();return (0);
	};

	wprintw (command_win,"Data written");refresh_command_win ();	
	return (1);
}

int log_changes (unsigned char *buffer,unsigned long length,unsigned long offset)


{
	unsigned char *original;
	
	int i;
	time_t current_time;
	FILE *fp;
	
	if ((fp=fopen (LogFile,"a+"))==NULL) {
		wprintw (command_win,"Error - Unable to open log file %s\n",LogFile);
		refresh_command_win ();return (0);
	};

	current_time=time (NULL);
	
	fprintf (fp,"\n----- EXT2ED log begin -----\n\n");
	fprintf (fp,"Time: %s\nDevice: %s\n",ctime ((time_t *) &current_time),device_name);
	fprintf (fp,"Offset: %lu\nLength: %lu\n",offset,length);
	
	original=(unsigned char *) malloc (length*sizeof (unsigned char));

	if (original==NULL) {
		wprintw (command_win,"Fatal error - Can\'t allocate %lu bytes!");
		refresh_command_win ();fclose (fp);return (0);
	}
	
	if (!low_read (original,length,offset)) {
		fclose (fp);return (0);
	}

	fprintf (fp,"\nOriginal data:\n\n");

	for (i=0;i<length;i++) {
		if (i%16==0 && i!=0) fprintf (fp,"\n");
		fprintf (fp,"%02x ",original [i]);
	}
	
	fprintf (fp,"\n\nNew data:\n\n");	
	
	for (i=0;i<length;i++) {
		if (i%16==0 && i!=0) fprintf (fp,"\n");
		fprintf (fp,"%02x ",buffer [i]);
	}
	
	fprintf (fp,"\n----- EXT2ED log end  -----\n");

	fclose (fp);	
	return (1);
}

int load_type_data (void)


{
	if (device_handle==NULL) {
		printf ("Error - No device opened\n");
		return (0);		
	}

	if (device_offset==-1) {
		printf ("Error - No offset set\n");
		return (0);
	}
	
	if (low_read (type_data.u.buffer,EXT2_MAX_BLOCK_SIZE,device_offset)==0)
		return (0);
		
	if (current_type!=NULL)
		if (strcmp (current_type->name,"ext2_dir_entry")==0)
			current_type->length=type_data.u.t_ext2_dir_entry.rec_len;

	return (1);
}

int write_type_data (void)

{
	if (device_handle==NULL) {
		wprintw (command_win,"Error - No device opened\n");
		refresh_command_win ();
		return (0);		
	}

	if (device_offset==-1) {
		wprintw (command_win,"Error - No offset set\n");
		refresh_command_win ();
		return (0);
	}
	
	if (low_write (type_data.u.buffer,file_system_info.block_size,device_offset)==0)
		return (0);
		
	return (1);
}

