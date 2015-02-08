
package proguard.io;

import java.io.*;


public interface DataEntryWriter
{
    /**
     * Creates a directory.
     * @param dataEntry the data entry for which the directory is to be created.
     * @return whether the directory has been created.
     */
    public boolean createDirectory(DataEntry dataEntry) throws IOException;


    /**
     * Returns an output stream for writing data. The caller must not close
     * the output stream; closing the output stream is the responsibility of
     * the implementation of this interface.
     * @param dataEntry the data entry for which the output stream is to be created.
     * @return the output stream. The stream may be <code>null</code> to indicate
     *         that the data entry should not be written.
     */
    public OutputStream getOutputStream(DataEntry dataEntry) throws IOException;


    /**
     * Returns an output stream for writing data. The caller must not close
     * the output stream; closing the output stream is the responsibility of
     * the implementation of this interface.
     * @param dataEntry the data entry for which the output stream is to be created.
     * @param finisher  the optional finisher that will be called before this
     *                  class closes the output stream (at some later point in
     *                  time) that will be returned (now).
     * @return the output stream. The stream may be <code>null</code> to indicate
     *         that the data entry should not be written.
     */
    public OutputStream getOutputStream(DataEntry dataEntry,
                                        Finisher  finisher) throws IOException;


    /**
     * Finishes writing all data entries.
     */
    public void close() throws IOException;
}
