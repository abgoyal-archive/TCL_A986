

#ifndef ANDROIDLOG_H_
#define ANDROIDLOG_H_

#ifdef ANDROID_DOM_LOGGING
#include <stdio.h>
extern FILE* gDomTreeFile;
#define DOM_TREE_LOG_FILE "/sdcard/domTree.txt"
#define DUMP_DOM_LOGD(...) { if (gDomTreeFile) \
    fprintf(gDomTreeFile, __VA_ARGS__); else LOGD(__VA_ARGS__); }

extern FILE* gRenderTreeFile;
#define RENDER_TREE_LOG_FILE "/sdcard/renderTree.txt"
#define DUMP_RENDER_LOGD(...) { if (gRenderTreeFile) \
    fprintf(gRenderTreeFile, __VA_ARGS__); else LOGD(__VA_ARGS__); }
#else
#define DUMP_DOM_LOGD(...) ((void)0)
#define DUMP_RENDER_LOGD(...) ((void)0)
#endif /* ANDROID_DOM_LOGGING */

#define DISPLAY_TREE_LOG_FILE "/sdcard/displayTree.txt"
#define LAYERS_TREE_LOG_FILE "/sdcard/layersTree.plist"

#endif /* ANDROIDLOG_H_ */
