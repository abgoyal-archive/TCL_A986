


#define PNG_INTERNAL
#define PNG_NO_PEDANTIC_WARNINGS
#include "png.h"
#ifdef PNG_WRITE_SUPPORTED


void /* PRIVATE */
png_write_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
   if (png_ptr->write_data_fn != NULL )
      (*(png_ptr->write_data_fn))(png_ptr, data, length);
   else
      png_error(png_ptr, "Call to NULL write function");
}

#ifdef PNG_STDIO_SUPPORTED
#ifndef USE_FAR_KEYWORD
void PNGAPI
png_default_write_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
   png_uint_32 check;

   if (png_ptr == NULL)
      return;
#ifdef _WIN32_WCE
   if ( !WriteFile((HANDLE)(png_ptr->io_ptr), data, length, &check, NULL) )
      check = 0;
#else
   check = fwrite(data, 1, length, (png_FILE_p)(png_ptr->io_ptr));
#endif
   if (check != length)
      png_error(png_ptr, "Write Error");
}
#else

#define NEAR_BUF_SIZE 1024
#define MIN(a,b) (a <= b ? a : b)

void PNGAPI
png_default_write_data(png_structp png_ptr, png_bytep data, png_size_t length)
{
   png_uint_32 check;
   png_byte *near_data;  /* Needs to be "png_byte *" instead of "png_bytep" */
   png_FILE_p io_ptr;

   if (png_ptr == NULL)
      return;
   /* Check if data really is near. If so, use usual code. */
   near_data = (png_byte *)CVT_PTR_NOCHECK(data);
   io_ptr = (png_FILE_p)CVT_PTR(png_ptr->io_ptr);
   if ((png_bytep)near_data == data)
   {
#ifdef _WIN32_WCE
      if ( !WriteFile(io_ptr, near_data, length, &check, NULL) )
         check = 0;
#else
      check = fwrite(near_data, 1, length, io_ptr);
#endif
   }
   else
   {
      png_byte buf[NEAR_BUF_SIZE];
      png_size_t written, remaining, err;
      check = 0;
      remaining = length;
      do
      {
         written = MIN(NEAR_BUF_SIZE, remaining);
         png_memcpy(buf, data, written); /* Copy far buffer to near buffer */
#ifdef _WIN32_WCE
         if ( !WriteFile(io_ptr, buf, written, &err, NULL) )
            err = 0;
#else
         err = fwrite(buf, 1, written, io_ptr);
#endif
         if (err != written)
            break;

         else
            check += err;

         data += written;
         remaining -= written;
      }
      while (remaining != 0);
   }
   if (check != length)
      png_error(png_ptr, "Write Error");
}

#endif
#endif

#ifdef PNG_WRITE_FLUSH_SUPPORTED
void /* PRIVATE */
png_flush(png_structp png_ptr)
{
   if (png_ptr->output_flush_fn != NULL)
      (*(png_ptr->output_flush_fn))(png_ptr);
}

#ifdef PNG_STDIO_SUPPORTED
void PNGAPI
png_default_flush(png_structp png_ptr)
{
#ifndef _WIN32_WCE
   png_FILE_p io_ptr;
#endif
   if (png_ptr == NULL)
      return;
#ifndef _WIN32_WCE
   io_ptr = (png_FILE_p)CVT_PTR((png_ptr->io_ptr));
   fflush(io_ptr);
#endif
}
#endif
#endif

void PNGAPI
png_set_write_fn(png_structp png_ptr, png_voidp io_ptr,
   png_rw_ptr write_data_fn, png_flush_ptr output_flush_fn)
{
   if (png_ptr == NULL)
      return;

   png_ptr->io_ptr = io_ptr;

#ifdef PNG_STDIO_SUPPORTED
   if (write_data_fn != NULL)
      png_ptr->write_data_fn = write_data_fn;

   else
      png_ptr->write_data_fn = png_default_write_data;
#else
   png_ptr->write_data_fn = write_data_fn;
#endif

#ifdef PNG_WRITE_FLUSH_SUPPORTED
#ifdef PNG_STDIO_SUPPORTED
   if (output_flush_fn != NULL)
      png_ptr->output_flush_fn = output_flush_fn;

   else
      png_ptr->output_flush_fn = png_default_flush;
#else
   png_ptr->output_flush_fn = output_flush_fn;
#endif
#endif /* PNG_WRITE_FLUSH_SUPPORTED */

   /* It is an error to read while writing a png file */
   if (png_ptr->read_data_fn != NULL)
   {
      png_ptr->read_data_fn = NULL;
      png_warning(png_ptr,
         "Attempted to set both read_data_fn and write_data_fn in");
      png_warning(png_ptr,
         "the same structure.  Resetting read_data_fn to NULL.");
   }
}

#ifdef USE_FAR_KEYWORD
#ifdef _MSC_VER
void *png_far_to_near(png_structp png_ptr, png_voidp ptr, int check)
{
   void *near_ptr;
   void FAR *far_ptr;
   FP_OFF(near_ptr) = FP_OFF(ptr);
   far_ptr = (void FAR *)near_ptr;

   if (check != 0)
      if (FP_SEG(ptr) != FP_SEG(far_ptr))
         png_error(png_ptr, "segment lost in conversion");

   return(near_ptr);
}
#  else
void *png_far_to_near(png_structp png_ptr, png_voidp ptr, int check)
{
   void *near_ptr;
   void FAR *far_ptr;
   near_ptr = (void FAR *)ptr;
   far_ptr = (void FAR *)near_ptr;

   if (check != 0)
      if (far_ptr != ptr)
         png_error(png_ptr, "segment lost in conversion");

   return(near_ptr);
}
#   endif
#   endif
#endif /* PNG_WRITE_SUPPORTED */
