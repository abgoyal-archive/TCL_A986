

#define LOG_TAG "webcore_test"

#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <utils/Log.h>

namespace android {
extern void benchmark(const char*, int, int ,int);
}

int main(int argc, char** argv) {
    int width = 800;
    int height = 600;
    int reloadCount = 0;
    while (true) {
        int c = getopt(argc, argv, "d:r:");
        if (c == -1)
            break;
        else if (c == 'd') {
            char* x = strchr(optarg, 'x');
            if (x) {
                width = atoi(optarg);
                height = atoi(x + 1);
                LOGD("Rendering page at %dx%d", width, height);
            }
        } else if (c == 'r') {
            reloadCount = atoi(optarg);
            if (reloadCount < 0)
                reloadCount = 0;
            LOGD("Reloading %d times", reloadCount);
        }
    }
    if (optind >= argc) {
        LOGE("Please supply a file to read\n");
        return 1;
    }

    android::benchmark(argv[optind], reloadCount, width, height);
}
