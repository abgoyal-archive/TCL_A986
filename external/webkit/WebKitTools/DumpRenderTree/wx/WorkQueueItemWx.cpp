

#include "config.h"
#include "WorkQueueItem.h"

#include "DumpRenderTree.h"

bool LoadItem::invoke() const
{
    return false;
}

bool ReloadItem::invoke() const
{
    return false;
}

bool ScriptItem::invoke() const
{
    return false;
}

bool BackForwardItem::invoke() const
{
    return false;
}
