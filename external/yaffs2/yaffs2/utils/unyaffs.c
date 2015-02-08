/*
 * unyaffs: extract files from yaffs2 file system image to current directory
 *
 * Created by Kai Wei <kai.wei.cn@gmail.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <getopt.h>
#include <libgen.h>
#include <stdint.h>

#include "unyaffs.h"

#define CHUNK_SIZE 2048
#define SPARE_SIZE 64
#define MAX_OBJECTS 10000
#define YAFFS_OBJECTID_ROOT     1

typedef struct {
  uint8_t data_first[464];
  uint8_t bbf[2];
  uint8_t data_last[36];
  uint8_t spare[16];
  uint8_t ecc[10];
} msm7227_codeword_struct;

msm7227_codeword_struct page_raw_data[4];


unsigned char data[CHUNK_SIZE + SPARE_SIZE];
unsigned char *chunk_data = data;
unsigned char *spare_data = data + CHUNK_SIZE;
int img_file;
int verbose=0;
int is_raw_dump=0;
FILE *fs_config=NULL;
char root_name[50];

char line[200];
int line_count=0;
char *lines;

#define READ_OK (1)
#define END_OF_IMAGE (0)
#define READ_KO (-1)

int read_chunk(void);
int process_chunk(void);

char *obj_list[MAX_OBJECTS];
int process_chunk(void)
{
	int out_file, remain, s;
	char *full_path_name;

	yaffs_PackedTags2 *pt = (yaffs_PackedTags2 *)spare_data;
	if (pt->t.byteCount == 0xffff)  {	//a new object 

		yaffs_ObjectHeader *oh = (yaffs_ObjectHeader *)malloc(sizeof(yaffs_ObjectHeader));
		memcpy(oh, chunk_data, sizeof(yaffs_ObjectHeader));

		full_path_name = (char *)malloc(strlen(oh->name) + strlen(obj_list[oh->parentObjectId]) + 2);
		if (full_path_name == NULL) {
			perror("malloc full path name\n");
		}
		strcpy(full_path_name, obj_list[oh->parentObjectId]);
		strcat(full_path_name, "/");
		strcat(full_path_name, oh->name);
		obj_list[pt->t.objectId] = full_path_name;
        
        if ( fs_config ) {
              char *path = full_path_name, *newlines;
              int i=0;
              while ( (path[i]=='.' || path[i]=='/') &&  i++ < strlen(full_path_name) );
              
              newlines=(char *)malloc(YAFFS_MAX_NAME_LENGTH*(line_count+1));
              if(line_count) {
                memcpy(newlines,lines,YAFFS_MAX_NAME_LENGTH*line_count);
                free(lines);
              }
              lines=newlines;

              if(strlen(&path[i])) {
                /* NOTE : the mode can be 4 digits , ex : 2644 */
                sprintf(&lines[YAFFS_MAX_NAME_LENGTH*line_count++], "%s/%s %d %d %03o\n", root_name, &path[i], oh->yst_uid, oh->yst_gid, oh->yst_mode & 0xFFF);
              }else{
                sprintf(&lines[YAFFS_MAX_NAME_LENGTH*line_count++], "%s %d %d %03o\n", root_name, oh->yst_uid, oh->yst_gid, oh->yst_mode & 0x1FF);
              }
            return 0;
        }
        
        if ( verbose ) {
              char *path_start = full_path_name;
              printf("%s size:%d type:%d uid:%d gid:%d perm:%o\n", full_path_name, oh->fileSize, oh->type, oh->yst_uid, oh->yst_gid, oh->yst_mode);
        }
        
		switch(oh->type) {
			case YAFFS_OBJECT_TYPE_FILE:
				remain = oh->fileSize;
				out_file = creat(full_path_name, oh->yst_mode);
				while(remain > 0) {
					if (read_chunk()!=1) {
            printf("still %d bytes to read from file %s\n", remain, full_path_name);
            //close(out_file);
						return -1;
          }
          /* check that the chunk spare is valid */
          if(pt->t.byteCount > CHUNK_SIZE ) {
            fprintf(stderr,"failed to read file %s. %d bytes missing\n", 
                    full_path_name, remain);
            return -1;
          }
          
          
					s = (remain < pt->t.byteCount) ? remain : pt->t.byteCount;	
					if (write(out_file, chunk_data, s) == -1) {
            perror(full_path_name);
            //close(out_file);
						return -1;
          }
					remain -= s;
				}
        
				close(out_file);
				break;
			case YAFFS_OBJECT_TYPE_SYMLINK:
				symlink(oh->alias, full_path_name);
				break;
			case YAFFS_OBJECT_TYPE_DIRECTORY:
				mkdir(full_path_name, 0777);
				break;
			case YAFFS_OBJECT_TYPE_HARDLINK:
				link(obj_list[oh->equivalentObjectId], full_path_name);
				break;
		}
    }
    
    return 0;
        
}

static unsigned long chunk_number = 0;

/*
 *  read_chunk() in img file. chunk is actually a page.
 *  
 *  ret status :
 *   1 read chunk OK     : 1 chunk read
 *   0 end of image      : 0 chunk read
 *  -1 read chunk failed : read size was less than a chunk
 *  
 */

int read_chunk(void)
{
	ssize_t s;
	int ret = READ_KO;
  int i;
  int offset=0;
	memset(chunk_data, 0xff, sizeof(chunk_data));
  if ( is_raw_dump ) {
	  s = read(img_file, &page_raw_data[0], CHUNK_SIZE + SPARE_SIZE);
      /* this will copy all the first codewords data and spare in the chunk, the last codeword spare in SPARE */
      /* the used spare size is only 16 bytes */
    for ( i=0;i<4;i++) {
      memcpy(chunk_data+offset, &page_raw_data[i].data_first,
             sizeof(page_raw_data[i].data_first));
      offset+=sizeof(page_raw_data[i].data_first);
      
      memcpy(chunk_data+offset, &page_raw_data[i].data_last,
             sizeof(page_raw_data[i].data_last));
      offset+=sizeof(page_raw_data[i].data_last);
      
      memcpy(chunk_data+offset, &page_raw_data[i].spare,         
             sizeof(page_raw_data[i].spare));
      offset+=sizeof(page_raw_data[i].spare);
    }
  }else {
	  s = read(img_file, data, CHUNK_SIZE + SPARE_SIZE);
  }
    
	if (s == -1) {
		perror("read() of image file failed\n");
    ret=READ_KO;
	} else if (s == 0) {
    if ( verbose ) printf("end of image\n");
    ret=END_OF_IMAGE;
	} else if ((s == (CHUNK_SIZE + SPARE_SIZE))) {
		ret = READ_OK;
	} else {
		fprintf(stderr, "broken image file\n");
    ret = READ_KO;
	}
    chunk_number++;
    //printf("chunk %d\n", chunk_number);
	return ret;
}

int compare_fn (const char *s1, const char *s2 )
{
  /* TODO compare lowcase strings ! */
  return strcmp (s1, s2);
}


int main(int argc, char **argv)
{
  int i;
  int status; // return status 0 on success 1 on error
	if (argc < 2 || argc > 4 ) {
		printf("Usage: unyaffs [options] image_file_name\n\
\n\
            options :\n\
\n\
            --fs_config /path/to/outputile.txt\n\
                output the files permissions to a file\n\
            --raw \n\
                indicates that the input image is a raw dump \n\
            -v/--verbose\n\
                ouput the files info to stdout\n");
		exit(1);
	}
    
  while (1) {
    int this_option_optind = optind ? optind : 1;
    int option_index = 0;
    static struct option long_options[] = {
      {"fs_config", 1, 0, 'f'},
      {"raw", 0, 0, 'r'},
      {"verbose", 0, 0, 'v'},
      {0, 0, 0, 0}
    };

    int c = getopt_long (argc, argv, "fv", long_options, &option_index);
    if (c == -1)
      break;
  
    switch (c) {
      case 'f':
        if (optarg) {
            printf (" output file info to : %s\n", optarg);
            fs_config = fopen(optarg, "w" );
            if (!fs_config) {
              fprintf(stderr,"can't open fs_config file %s\n",optarg);
              exit(1);
            }
        }else { /* no argument, output to stdout */
            fs_config = stdout;
        }
              
        break;
      case 'r':
        is_raw_dump =1;
        break;
      case 'v':
      printf("verbose mode :\n");
        verbose =1;
        break;
      case '?':
        fprintf(stderr,"unknown option %s: \n",long_options[option_index].name);
      break;
    
    }
  }
    
	img_file = open(argv[argc-1], O_RDONLY);
	if (img_file == -1) {
		fprintf(stderr,"open image file <%s> failed\n",argv[argc-1]);
		exit(1);
	}
    
    strcpy(root_name, basename(argv[argc-1]));
    i=0;
    do{
      if ( root_name[i] == '.' ) { 
        root_name[i]=0;
      }
    }while (root_name[i++]!=0);
    
	obj_list[YAFFS_OBJECTID_ROOT] = ".";
	while(1) {
    int ret = read_chunk();
		if (ret == READ_KO) {
      status=1;
			break;
    }else if ( ret == END_OF_IMAGE ) {
      status=0;
      break;
    }
		if(process_chunk()) {
      fprintf(stderr,"chunk error\n");
      status=1;
      break;
    }
	}
	close(img_file);
    
    if(fs_config) {
      /* sort output for clarity */
      qsort (lines, line_count, YAFFS_MAX_NAME_LENGTH, compare_fn);
      for (i = 0; i < line_count; i++)
        fprintf(fs_config,&lines[i*YAFFS_MAX_NAME_LENGTH]);
        fclose(fs_config);
    }
	return status;
}
