
package proguard.io;

import proguard.util.StringMatcher;

public class DataEntryNameFilter
implements   DataEntryFilter
{
    private final StringMatcher stringMatcher;


    /**
     * Creates a new DataEntryNameFilter.
     * @param stringMatcher the string matcher that will be applied to the names
     *                      of the filtered data entries.
     */
    public DataEntryNameFilter(StringMatcher stringMatcher)
    {
        this.stringMatcher = stringMatcher;
    }


    // Implementations for DataEntryFilter.

    public boolean accepts(DataEntry dataEntry)
    {
        return dataEntry != null && stringMatcher.matches(dataEntry.getName());
    }
}
