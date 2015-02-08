
#ifdef __cplusplus
extern "C" {
#endif

typedef void* a2dpData;

int a2dp_init(int rate, int channels, a2dpData* dataPtr);
void a2dp_set_sink(a2dpData data, const char* address);
int a2dp_write(a2dpData data, const void* buffer, int count);
int a2dp_stop(a2dpData data);
void a2dp_cleanup(a2dpData data);

#ifdef __cplusplus
}
#endif
