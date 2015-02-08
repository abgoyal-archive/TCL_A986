

#ifndef MY_JAVA_VM_H
#define MY_JAVA_VM_H

// Make it 1 just to appease any assertions or checks for valid objects
#define MY_JOBJECT ((jobject) 1)

void InitializeJavaVM();

#endif
