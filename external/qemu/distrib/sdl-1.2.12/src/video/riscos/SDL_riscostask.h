
#include "SDL_config.h"


/* Task initialisation/Clean up */

extern int RISCOS_InitTask();
extern void RISCOS_ExitTask();
extern int RISCOS_GetWimpVersion();
extern int RISCOS_GetTaskHandle();


/* Wimp mode saveing/restoring */
extern void RISCOS_StoreWimpMode();
extern void RISCOS_RestoreWimpMode();
