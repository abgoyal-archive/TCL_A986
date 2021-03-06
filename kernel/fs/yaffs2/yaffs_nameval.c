



#include "yaffs_nameval.h"

#include "yportenv.h"
 
static int nval_find(const char *xb, int xb_size, const YCHAR *name,
		int *exist_size)
{
	int pos=0;
	int size;

	memcpy(&size,xb,sizeof(int));
	while(size > 0 && (size < xb_size) && (pos + size < xb_size)){
		if(yaffs_strncmp((YCHAR *)(xb+pos+sizeof(int)),name,size) == 0){
			if(exist_size)
				*exist_size = size;
			return pos;
		}
		pos += size;
		if(pos < xb_size -sizeof(int))
			memcpy(&size,xb + pos,sizeof(int));
		else
			size = 0;
	}
	if(exist_size)
		*exist_size = 0;
	return -1;
}

static int nval_used(const char *xb, int xb_size)
{
	int pos=0;
	int size;

	memcpy(&size,xb + pos,sizeof(int));
	while(size > 0 && (size < xb_size) && (pos + size < xb_size)){
		pos += size;
		if(pos < xb_size -sizeof(int))
			memcpy(&size,xb + pos,sizeof(int));
		else
			size = 0;
	}
	return pos;
}

int nval_del(char *xb, int xb_size, const YCHAR *name)
{
	int pos  = nval_find(xb, xb_size, name, NULL);
	int size;
	
	if(pos >= 0 && pos < xb_size){
		/* Find size, shift rest over this record, then zero out the rest of buffer */
		memcpy(&size,xb+pos,sizeof(int));
		memcpy(xb + pos, xb + pos + size, xb_size - (pos + size));
		memset(xb + (xb_size - size),0,size);
		return 0;
	} else
		return -ENODATA;
}

int nval_set(char *xb, int xb_size, const YCHAR *name, const char *buf, int bsize, int flags)
{
	int pos;
	int namelen = yaffs_strnlen(name,xb_size);
	int reclen;
	int size_exist = 0;
	int space;
	int start;

	pos = nval_find(xb,xb_size,name, &size_exist);

	if(flags & XATTR_CREATE && pos >= 0)
		return -EEXIST;
	if(flags & XATTR_REPLACE && pos < 0)
		return -ENODATA;

	start = nval_used(xb,xb_size);
	space = xb_size - start + size_exist;

	reclen = (sizeof(int) + namelen + 1 + bsize);

	if(reclen > space)
		return -ENOSPC;

	if(pos >= 0){
		nval_del(xb,xb_size,name);
		start = nval_used(xb, xb_size);
	}

	pos = start;

	memcpy(xb + pos,&reclen,sizeof(int));
	pos +=sizeof(int);
	yaffs_strncpy((YCHAR *)(xb + pos), name, reclen);
	pos+= (namelen+1);
	memcpy(xb + pos,buf,bsize);
	return 0;
}

int nval_get(const char *xb, int xb_size, const YCHAR *name, char *buf, int bsize)
{
	int pos = nval_find(xb,xb_size,name,NULL);
	int size;
	
	if(pos >= 0 && pos< xb_size){
		
		memcpy(&size,xb +pos,sizeof(int));
		pos+=sizeof(int); /* advance past record length */
		size -= sizeof(int);

		/* Advance over name string */
		while(xb[pos] && size > 0 && pos < xb_size){
			pos++;
			size--;
		}
		/*Advance over NUL */
		pos++;
		size--;

		if(size <= bsize){
			memcpy(buf,xb + pos,size);
			return size;
		}
		
	}
	if(pos >= 0)
		return -ERANGE;
	else
		return -ENODATA;
}

int nval_list(const char *xb, int xb_size, char *buf, int bsize)
{
	int pos = 0;
	int size;
	int name_len;
	int ncopied = 0;
	int filled = 0;

	memcpy(&size,xb + pos,sizeof(int));
	while(size > sizeof(int) && size <= xb_size && (pos + size) < xb_size && !filled){
		pos+= sizeof(int);
		size-=sizeof(int);
		name_len = yaffs_strnlen((YCHAR *)(xb + pos), size);
		if(ncopied + name_len + 1 < bsize){
			memcpy(buf,xb+pos,name_len * sizeof(YCHAR));
			buf+= name_len;
			*buf = '\0';
			buf++;
			if(sizeof(YCHAR) > 1){
				*buf = '\0';
				buf++;
			}
			ncopied += (name_len+1);
		} else
			filled = 1;
		pos+=size;
		if(pos < xb_size -sizeof(int))
			memcpy(&size,xb + pos,sizeof(int));
		else
			size = 0;
	}
	return ncopied;
}


int nval_hasvalues(const char *xb, int xb_size)
{
	return nval_used(xb, xb_size) > 0;
}
