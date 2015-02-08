
#include <linux/module.h>

asmlinkage long long __ashldi3 (long long, int);
asmlinkage long long __ashrdi3 (long long, int);
asmlinkage long long __lshrdi3 (long long, int);
asmlinkage long long __muldi3 (long long, long long);

EXPORT_SYMBOL(__ashldi3);
EXPORT_SYMBOL(__ashrdi3);
EXPORT_SYMBOL(__lshrdi3);
EXPORT_SYMBOL(__muldi3);

