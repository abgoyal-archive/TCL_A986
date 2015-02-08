
package proguard.io;

import java.io.IOException;


public interface DataEntryReader
{
    /**
     * Reads the given data entry.
     */
    public void read(DataEntry dataEntry) throws IOException;
}
