
package proguard.io;

import java.io.*;

public interface DataEntry
{
    /**
     * Returns the name of this data entry.
     */
    public String getName();

    /**
     * Returns whether the data entry represents a directory.
     */
    public boolean isDirectory();


    /**
     * Returns an input stream for reading the content of this data entry.
     * The data entry may not represent a directory.
     */
    public InputStream getInputStream() throws IOException;


    /**
     * Closes the previously retrieved InputStream.
     */
    public void closeInputStream() throws IOException;


    /**
     * Returns the parent of this data entry, or <code>null</null> if it doesn't
     * have one.
     */
    public DataEntry getParent();
}
