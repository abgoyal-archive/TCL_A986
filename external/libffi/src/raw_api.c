
/* This file defines generic functions for use with the raw api. */

#include <ffi.h>
#include <ffi_common.h>

#if !FFI_NO_RAW_API

size_t
ffi_raw_size (ffi_cif *cif)
{
  size_t result = 0;
  int i;

  ffi_type **at = cif->arg_types;

  for (i = cif->nargs-1; i >= 0; i--, at++)
    {
#if !FFI_NO_STRUCTS
      if ((*at)->type == FFI_TYPE_STRUCT)
	result += ALIGN (sizeof (void*), FFI_SIZEOF_ARG);
      else
#endif
	result += ALIGN ((*at)->size, FFI_SIZEOF_ARG);
    }

  return result;
}


void
ffi_raw_to_ptrarray (ffi_cif *cif, ffi_raw *raw, void **args)
{
  unsigned i;
  ffi_type **tp = cif->arg_types;

#if WORDS_BIGENDIAN

  for (i = 0; i < cif->nargs; i++, tp++, args++)
    {	  
      switch ((*tp)->type)
	{
	case FFI_TYPE_UINT8:
	case FFI_TYPE_SINT8:
	  *args = (void*) ((char*)(raw++) + FFI_SIZEOF_ARG - 1);
	  break;
	  
	case FFI_TYPE_UINT16:
	case FFI_TYPE_SINT16:
	  *args = (void*) ((char*)(raw++) + FFI_SIZEOF_ARG - 2);
	  break;

#if FFI_SIZEOF_ARG >= 4	  
	case FFI_TYPE_UINT32:
	case FFI_TYPE_SINT32:
	  *args = (void*) ((char*)(raw++) + FFI_SIZEOF_ARG - 4);
	  break;
#endif
	
#if !FFI_NO_STRUCTS  
	case FFI_TYPE_STRUCT:
	  *args = (raw++)->ptr;
	  break;
#endif

	case FFI_TYPE_POINTER:
	  *args = (void*) &(raw++)->ptr;
	  break;
	  
	default:
	  *args = raw;
	  raw += ALIGN ((*tp)->size, FFI_SIZEOF_ARG) / FFI_SIZEOF_ARG;
	}
    }

#else /* WORDS_BIGENDIAN */

#if !PDP

  /* then assume little endian */
  for (i = 0; i < cif->nargs; i++, tp++, args++)
    {	  
#if !FFI_NO_STRUCTS
      if ((*tp)->type == FFI_TYPE_STRUCT)
	{
	  *args = (raw++)->ptr;
	}
      else
#endif
	{
	  *args = (void*) raw;
	  raw += ALIGN ((*tp)->size, sizeof (void*)) / sizeof (void*);
	}
    }

#else
#error "pdp endian not supported"
#endif /* ! PDP */

#endif /* WORDS_BIGENDIAN */
}

void
ffi_ptrarray_to_raw (ffi_cif *cif, void **args, ffi_raw *raw)
{
  unsigned i;
  ffi_type **tp = cif->arg_types;

  for (i = 0; i < cif->nargs; i++, tp++, args++)
    {	  
      switch ((*tp)->type)
	{
	case FFI_TYPE_UINT8:
	  (raw++)->uint = *(UINT8*) (*args);
	  break;

	case FFI_TYPE_SINT8:
	  (raw++)->sint = *(SINT8*) (*args);
	  break;

	case FFI_TYPE_UINT16:
	  (raw++)->uint = *(UINT16*) (*args);
	  break;

	case FFI_TYPE_SINT16:
	  (raw++)->sint = *(SINT16*) (*args);
	  break;

#if FFI_SIZEOF_ARG >= 4
	case FFI_TYPE_UINT32:
	  (raw++)->uint = *(UINT32*) (*args);
	  break;

	case FFI_TYPE_SINT32:
	  (raw++)->sint = *(SINT32*) (*args);
	  break;
#endif

#if !FFI_NO_STRUCTS
	case FFI_TYPE_STRUCT:
	  (raw++)->ptr = *args;
	  break;
#endif

	case FFI_TYPE_POINTER:
	  (raw++)->ptr = **(void***) args;
	  break;

	default:
	  memcpy ((void*) raw->data, (void*)*args, (*tp)->size);
	  raw += ALIGN ((*tp)->size, FFI_SIZEOF_ARG) / FFI_SIZEOF_ARG;
	}
    }
}

#if !FFI_NATIVE_RAW_API



void ffi_raw_call (ffi_cif *cif, void (*fn)(void), void *rvalue, ffi_raw *raw)
{
  void **avalue = (void**) alloca (cif->nargs * sizeof (void*));
  ffi_raw_to_ptrarray (cif, raw, avalue);
  ffi_call (cif, fn, rvalue, avalue);
}

#if FFI_CLOSURES		/* base system provides closures */

static void
ffi_translate_args (ffi_cif *cif, void *rvalue,
		    void **avalue, void *user_data)
{
  ffi_raw *raw = (ffi_raw*)alloca (ffi_raw_size (cif));
  ffi_raw_closure *cl = (ffi_raw_closure*)user_data;

  ffi_ptrarray_to_raw (cif, avalue, raw);
  (*cl->fun) (cif, rvalue, raw, cl->user_data);
}

ffi_status
ffi_prep_raw_closure_loc (ffi_raw_closure* cl,
			  ffi_cif *cif,
			  void (*fun)(ffi_cif*,void*,ffi_raw*,void*),
			  void *user_data,
			  void *codeloc)
{
  ffi_status status;

  status = ffi_prep_closure_loc ((ffi_closure*) cl,
				 cif,
				 &ffi_translate_args,
				 codeloc,
				 codeloc);
  if (status == FFI_OK)
    {
      cl->fun       = fun;
      cl->user_data = user_data;
    }

  return status;
}

#endif /* FFI_CLOSURES */
#endif /* !FFI_NATIVE_RAW_API */

#if FFI_CLOSURES


ffi_status
ffi_prep_raw_closure (ffi_raw_closure* cl,
		      ffi_cif *cif,
		      void (*fun)(ffi_cif*,void*,ffi_raw*,void*),
		      void *user_data)
{
  return ffi_prep_raw_closure_loc (cl, cif, fun, user_data, cl);
}

#endif /* FFI_CLOSURES */

#endif /* !FFI_NO_RAW_API */
