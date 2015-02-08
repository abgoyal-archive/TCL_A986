
package proguard.io;

import java.io.IOException;


public interface DataEntryPump
{
    /**
     * Applies the given DataEntryReader to all data entries that the
     * implementation can provide.
     */
    public void pumpDataEntries(DataEntryReader dataEntryReader)
    throws IOException;
}
