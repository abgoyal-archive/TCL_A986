#include "tomcrypt.h"


#ifdef LTC_DER

void der_sequence_free(ltc_asn1_list *in)
{
   ltc_asn1_list *l;
   
   /* walk to the start of the chain */
   while (in->prev != NULL || in->parent != NULL) {
      if (in->parent != NULL) {
          in = in->parent;
      } else {
          in = in->prev;
      }
   }
   
   /* now walk the list and free stuff */
   while (in != NULL) {
      /* is there a child? */
      if (in->child) {
         /* disconnect */
         in->child->parent = NULL;
         der_sequence_free(in->child);
      }
      
      switch (in->type) { 
         case LTC_ASN1_SET:
         case LTC_ASN1_SETOF:
         case LTC_ASN1_SEQUENCE: break;
         case LTC_ASN1_INTEGER : if (in->data != NULL) { mp_clear(in->data); } break;
         default               : if (in->data != NULL) { XFREE(in->data);    }
      }
      
      /* move to next and free current */
      l = in->next;
      free(in);
      in = l;
   }     
}

#endif

/* $Source: /cvs/libtom/libtomcrypt/src/pk/asn1/der/sequence/der_sequence_free.c,v $ */
/* $Revision: 1.3 $ */
/* $Date: 2006/03/31 14:15:35 $ */
