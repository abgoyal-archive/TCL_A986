
package proguard.io;

import java.io.IOException;
import java.util.zip.*;

public class JarReader implements DataEntryReader
{
    private final DataEntryReader dataEntryReader;


    /**
     * Creates a new JarReader.
     */
    public JarReader(DataEntryReader dataEntryReader)
    {
        this.dataEntryReader = dataEntryReader;
    }


    // Implementation for DataEntryReader.

    public void read(DataEntry dataEntry) throws IOException
    {
        ZipInputStream zipInputStream = new ZipInputStream(dataEntry.getInputStream());

        try
        {
            // Get all entries from the input jar.
            while (true)
            {
                // Can we get another entry?
                ZipEntry zipEntry = zipInputStream.getNextEntry();
                if (zipEntry == null)
                {
                    break;
                }

                // Delegate the actual reading to the data entry reader.
                dataEntryReader.read(new ZipDataEntry(dataEntry,
                                                      zipEntry,
                                                      zipInputStream));
            }
        }
        finally
        {
            dataEntry.closeInputStream();
        }
    }
}
