

@interface WebWorkersPrivate : NSObject {
}

// Returns the total number of currently executing worker threads (shared + dedicated).
+ (unsigned) workerThreadCount;

@end
