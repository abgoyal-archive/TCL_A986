// Copyright (c) 2008 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
// Written in NSPR style to also be suitable for adding to the NSS demo suite

#ifndef __MEMIO_H
#define __MEMIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "prio.h"

/* Opaque structure.  Really just a more typesafe alias for PRFilePrivate. */
struct memio_Private;
typedef struct memio_Private memio_Private;


/* Create the I/O layer and its two circular buffers. */
PRFileDesc *memio_CreateIOLayer(int bufsize);

/* Must call before trying to make an ssl connection */
void memio_SetPeerName(PRFileDesc *fd, const PRNetAddr *peername);

memio_Private *memio_GetSecret(PRFileDesc *fd);

int memio_GetReadParams(memio_Private *secret, char **buf);

void memio_PutReadResult(memio_Private *secret, int bytes_read);

int memio_GetWriteParams(memio_Private *secret, const char **buf);

void memio_PutWriteResult(memio_Private *secret, int bytes_written);


#ifdef __cplusplus
}
#endif

#endif
