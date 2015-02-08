NSECS_PER_SEC    = 1000000000

def avg(total, n):
    return total / n

def nsecs(secs, nsecs):
    return secs * NSECS_PER_SEC + nsecs

def nsecs_secs(nsecs):
    return nsecs / NSECS_PER_SEC

def nsecs_nsecs(nsecs):
    return nsecs % NSECS_PER_SEC

def nsecs_str(nsecs):
    str = "%5u.%09u" % (nsecs_secs(nsecs), nsecs_nsecs(nsecs)),
    return str

def clear_term():
    print("\x1b[H\x1b[2J")
