


#define PNG_INTERNAL
#define PNG_NO_PEDANTIC_WARNINGS
#include "png.h"
#ifdef PNG_READ_SUPPORTED

void /* PRIVATE */
png_read_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
   png_debug1(4, "reading %d bytes", (int)length);
 
   if (png_ptr->read_data_fn != NULL)
      (*(png_ptr->read_data_fn))(png_ptr, data, length);
   else
      png_error(png_ptr, "Call to NULL read function");

#ifdef PNG_INDEX_SUPPORTED
   png_ptr->total_data_read += length;
#endif
}

#ifdef PNG_INDEX_SUPPORTED
void /* PRIVATE */
png_seek_data(png_structp png_ptr, png_uint_32 offset)
{
   if (png_ptr->seek_data_fn != NULL)
      (*(png_ptr->seek_data_fn))(png_ptr, offset);
   else
      png_error(png_ptr, "Call to NULL seek function");
}
#endif

#ifdef PNG_STDIO_SUPPORTED
#ifndef USE_FAR_KEYWORD
void PNGAPI
png_default_read_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
   png_size_t check;

   if (png_ptr == NULL)
      return;
   /* fread() returns 0 on error, so it is OK to store this in a png_size_t
    * instead of an int, which is what fread() actually returns.
    */
#ifdef _WIN32_WCE
   if ( !ReadFile((HANDLE)(png_ptr->io_ptr), data, length, &check, NULL) )
      check = 0;
#else
   check = (png_size_t)fread(data, (png_size_t)1, length,
      (png_FILE_p)png_ptr->io_ptr);
#endif

   if (check != length)
      png_error(png_ptr, "Read Error");
}
#else

#define NEAR_BUF_SIZE 1024
#define MIN(a,b) (a <= b ? a : b)

static void PNGAPI
png_default_read_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
   int check;
   png_byte *n_data;
   png_FILE_p io_ptr;

   if (png_ptr == NULL)
      return;
   /* Check if data really is near. If so, use usual code. */
   n_data = (png_byte *)CVT_PTR_NOCHECK(data);
   io_ptr = (png_FILE_p)CVT_PTR(png_ptr->io_ptr);
   if ((png_bytep)n_data == data)
   {
#ifdef _WIN32_WCE
      if ( !ReadFile((HANDLE)(png_ptr->io_ptr), data, length, &check,
          NULL) )
         check = 0;
#else
      check = fread(n_data, 1, length, io_ptr);
#endif
   }
   else
   {
      png_byte buf[NEAR_BUF_SIZE];
      png_size_t read, remaining, err;
      check = 0;
      remaining = length;
      do
      {
         read = MIN(NEAR_BUF_SIZE, remaining);
#ifdef _WIN32_WCE
         if ( !ReadFile((HANDLE)(io_ptr), buf, read, &err, NULL) )
            err = 0;
#else
         err = fread(buf, (png_size_t)1, read, io_ptr);
#endif
         png_memcpy(data, buf, read); /* copy far buffer to near buffer */
         if (err != read)
            break;
         else
            check += err;
         data += read;
         remaining -= read;
      }
      while (remaining != 0);
   }
   if ((png_uint_32)check != (png_uint_32)length)
      png_error(png_ptr, "read Error");
}
#endif
#endif

void PNGAPI
png_set_read_fn(png_structp png_ptr, png_voidp io_ptr,
   png_rw_ptr read_data_fn)
{
   if (png_ptr == NULL)
      return;
   png_ptr->io_ptr = io_ptr;

#ifdef PNG_STDIO_SUPPORTED
   if (read_data_fn != NULL)
      png_ptr->read_data_fn = read_data_fn;
   else
      png_ptr->read_data_fn = png_default_read_data;
#else
   png_ptr->read_data_fn = read_data_fn;
#endif

   /* It is an error to write to a read device */
   if (png_ptr->write_data_fn != NULL)
   {
      png_ptr->write_data_fn = NULL;
      png_warning(png_ptr,
         "It's an error to set both read_data_fn and write_data_fn in the ");
      png_warning(png_ptr,
         "same structure.  Resetting write_data_fn to NULL.");
   }

#ifdef PNG_WRITE_FLUSH_SUPPORTED
   png_ptr->output_flush_fn = NULL;
#endif
}

#ifdef PNG_INDEX_SUPPORTED
void PNGAPI
png_set_seek_fn(png_structp png_ptr, png_seek_ptr seek_data_fn)
{
   if (png_ptr == NULL)
      return;
   png_ptr->seek_data_fn = seek_data_fn;
}
#endif
#endif /* PNG_READ_SUPPORTED */
