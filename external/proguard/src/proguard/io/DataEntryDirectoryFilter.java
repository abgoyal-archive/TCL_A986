
package proguard.io;

import proguard.util.StringMatcher;

public class DataEntryDirectoryFilter
implements   DataEntryFilter
{
    // Implementations for DataEntryFilter.

    public boolean accepts(DataEntry dataEntry)
    {
        return dataEntry != null && dataEntry.isDirectory();
    }
}