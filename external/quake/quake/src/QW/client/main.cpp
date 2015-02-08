
#include <nativehelper/jni.h>
#include <stdio.h>
#include <assert.h>

#include <private/opengles/gl_context.h>
#include <GLES/gl.h>

// Delegate to "C" routines to do the rest of the processing.
// (We do this because it's difficult to mix the C++ headers
// for JNI and gl_context with the C headers for quake.)

extern "C"
{
void AndroidInit(int width, int height);
int AndroidEvent(int type, int value);
void AndroidStep();
int AndroidQuiting();
}

void
qinit(JNIEnv *env, jobject thiz, /* jobjectArray config, */
       jint width, jint height) {
	AndroidInit(width, height);
}

jboolean
qevent(JNIEnv *env, jobject thiz, jint type, jint value) {
	return AndroidEvent(type, value) ? JNI_TRUE : JNI_FALSE;
}

void
qstep(JNIEnv *env, jobject thiz) {
	AndroidStep();
}

jboolean
qquitting(JNIEnv *env, jobject thiz) {
  return AndroidQuiting() ? JNI_TRUE : JNI_FALSE;
}

static const char *classPathName = "com/android/quake/QuakeApplication";

static JNINativeMethod methods[] = {
  {"init", "(II)V", (void*)qinit },
  {"event", "(II)Z", (void*)qevent },
  {"step", "()V", (void*)qstep },
  {"quitting", "()Z", (void*)qquitting },
};

static int registerNativeMethods(JNIEnv* env, const char* className,
    JNINativeMethod* gMethods, int numMethods)
{
    jclass clazz;

    clazz = env->FindClass(className);
    if (clazz == NULL) {
        fprintf(stderr,
            "Native registration unable to find class '%s'\n", className);
        return JNI_FALSE;
    }
    if (env->RegisterNatives(clazz, gMethods, numMethods) < 0) {
        fprintf(stderr, "RegisterNatives failed for '%s'\n", className);
        return JNI_FALSE;
    }

    return JNI_TRUE;
}

static int registerNatives(JNIEnv* env)
{
  if (!registerNativeMethods(env, classPathName,
			     methods, sizeof(methods) / sizeof(methods[0]))) {
    return JNI_FALSE;
  }

  return JNI_TRUE;
}

jint JNI_OnLoad(JavaVM* vm, void* reserved)
{
    JNIEnv* env = NULL;
    jint result = -1;

    if (vm->GetEnv((void**) &env, JNI_VERSION_1_4) != JNI_OK) {
        fprintf(stderr, "ERROR: GetEnv failed\n");
        goto bail;
    }
    assert(env != NULL);

    printf("In mgmain JNI_OnLoad\n");

    if (!registerNatives(env)) {
        fprintf(stderr, "ERROR: miniglobe native registration failed\n");
        goto bail;
    }

    /* success -- return valid version number */
    result = JNI_VERSION_1_4;

bail:
    return result;
}
