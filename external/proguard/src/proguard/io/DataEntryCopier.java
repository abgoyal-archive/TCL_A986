
package proguard.io;

import proguard.util.ExtensionMatcher;

import java.io.*;


public class DataEntryCopier implements DataEntryReader
{
    private static final int BUFFER_SIZE = 1024;

    private final DataEntryWriter dataEntryWriter;
    private final byte[]          buffer = new byte[BUFFER_SIZE];



    public DataEntryCopier(DataEntryWriter dataEntryWriter)
    {
        this.dataEntryWriter = dataEntryWriter;
    }


    // Implementations for DataEntryReader.

    public void read(DataEntry dataEntry) throws IOException
    {
        try
        {
            if (dataEntry.isDirectory())
            {
                dataEntryWriter.createDirectory(dataEntry);
            }
            else
            {
                // Get the output entry corresponding to this input entry.
                OutputStream outputStream = dataEntryWriter.getOutputStream(dataEntry);
                if (outputStream != null)
                {
                    InputStream inputStream = dataEntry.getInputStream();

                    // Copy the data from the input entry to the output entry.
                    copyData(inputStream, outputStream);

                    // Close the data entries.
                    dataEntry.closeInputStream();
                }
            }
        }
        catch (IOException ex)
        {
            System.err.println("Warning: can't write resource [" + dataEntry.getName() + "] (" + ex.getMessage() + ")");
        }
    }


    /**
     * Copies all data that it can read from the given input stream to the
     * given output stream.
     */
    protected void copyData(InputStream  inputStream,
                            OutputStream outputStream)
    throws IOException
    {
        while (true)
        {
            int count = inputStream.read(buffer);
            if (count < 0)
            {
                break;
            }
            outputStream.write(buffer, 0, count);
        }

        outputStream.flush();
    }


    /**
     * A main method for testing file/jar/war/directory copying.
     */
    public static void main(String[] args)
    {
        try
        {
            String input  = args[0];
            String output = args[1];

            boolean outputIsJar = output.endsWith(".jar");
            boolean outputIsWar = output.endsWith(".war");
            boolean outputIsEar = output.endsWith(".ear");
            boolean outputIsZip = output.endsWith(".zip");

            DataEntryWriter writer = new DirectoryWriter(new File(output),
                                                         outputIsJar ||
                                                         outputIsWar ||
                                                         outputIsEar ||
                                                         outputIsZip);

        if (!outputIsJar)
        {
            // Zip up any zips, if necessary.
            DataEntryWriter zipWriter = new JarWriter(writer);
            if (outputIsZip)
            {
                // Always zip.
                writer = zipWriter;
            }
            else
            {
                // Only zip up zips.
                writer = new FilteredDataEntryWriter(new DataEntryParentFilter(
                                                     new DataEntryNameFilter(
                                                     new ExtensionMatcher(".zip"))),
                                                     zipWriter,
                                                     writer);
            }

            // Zip up any wars, if necessary.
            DataEntryWriter warWriter = new JarWriter(writer);
            if (outputIsWar)
            {
                // Always zip.
                writer = warWriter;
            }
            else
            {
                // Only zip up wars.
                writer = new FilteredDataEntryWriter(new DataEntryParentFilter(
                                                     new DataEntryNameFilter(
                                                     new ExtensionMatcher(".war"))),
                                                     warWriter,
                                                     writer);
            }
        }

        // Zip up any jars, if necessary.
        DataEntryWriter jarWriter = new JarWriter(writer);
        if (outputIsJar)
        {
            // Always zip.
            writer = jarWriter;
        }
        else
        {
            // Only zip up jars.
            writer = new FilteredDataEntryWriter(new DataEntryParentFilter(
                                                 new DataEntryNameFilter(
                                                 new ExtensionMatcher(".jar"))),
                                                 jarWriter,
                                                 writer);
        }


            // Create the copying DataEntryReader.
            DataEntryReader reader = new DataEntryCopier(writer);


            boolean inputIsJar = input.endsWith(".jar");
            boolean inputIsWar = input.endsWith(".war");
            boolean inputIsZip = input.endsWith(".zip");

            // Unzip any jars, if necessary.
            DataEntryReader jarReader = new JarReader(reader);
            if (inputIsJar)
            {
                // Always unzip.
                reader = jarReader;
            }
            else
            {
                // Only unzip jar entries.
                reader = new FilteredDataEntryReader(new DataEntryNameFilter(
                                                     new ExtensionMatcher(".jar")),
                                                     jarReader,
                                                     reader);

                // Unzip any wars, if necessary.
                DataEntryReader warReader = new JarReader(reader);
                if (inputIsWar)
                {
                    // Always unzip.
                    reader = warReader;
                }
                else
                {
                    // Only unzip war entries.
                    reader = new FilteredDataEntryReader(new DataEntryNameFilter(
                                                         new ExtensionMatcher(".war")),
                                                         warReader,
                                                         reader);
                }

                // Unzip any zips, if necessary.
                DataEntryReader zipReader = new JarReader(reader);
                if (inputIsZip)
                {
                    // Always unzip.
                    reader = zipReader;
                }
                else
                {
                    // Only unzip zip entries.
                    reader = new FilteredDataEntryReader(new DataEntryNameFilter(
                                                         new ExtensionMatcher(".zip")),
                                                         zipReader,
                                                         reader);
                }
            }

            DirectoryPump directoryReader = new DirectoryPump(new File(input));

            directoryReader.pumpDataEntries(reader);

            writer.close();
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }
}
