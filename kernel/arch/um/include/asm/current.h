

#ifndef __UM_CURRENT_H
#define __UM_CURRENT_H

#include "linux/thread_info.h"

#define current (current_thread_info()->task)

#endif
