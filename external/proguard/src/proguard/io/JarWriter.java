
package proguard.io;

import proguard.classfile.ClassConstants;

import java.io.*;
import java.util.*;
import java.util.jar.*;
import java.util.zip.*;


public class JarWriter implements DataEntryWriter, Finisher
{
    private final DataEntryWriter dataEntryWriter;
    private final Manifest        manifest;
    private final String          comment;

    private OutputStream    currentParentOutputStream;
    private ZipOutputStream currentJarOutputStream;
    private Finisher        currentFinisher;
    private DataEntry       currentDataEntry;

    // The names of the jar entries that are already in the jar.
    private final Set jarEntryNames = new HashSet();


    /**
     * Creates a new JarWriter without manifest or comment.
     */
    public JarWriter(DataEntryWriter dataEntryWriter)
    {
        this(dataEntryWriter, null, null);
    }


    /**
     * Creates a new JarWriter.
     */
    public JarWriter(DataEntryWriter dataEntryWriter,
                     Manifest        manifest,
                     String          comment)
    {
        this.dataEntryWriter = dataEntryWriter;
        this.manifest        = manifest;
        this.comment         = comment;
    }


    // Implementations for DataEntryWriter.

    public boolean createDirectory(DataEntry dataEntry) throws IOException
    {
        //Make sure we can start with a new entry.
        if (!prepareEntry(dataEntry))
        {
            return false;
        }

        // Close the previous ZIP entry, if any.
        closeEntry();

        // Get the directory entry name.
        String name = dataEntry.getName() + ClassConstants.INTERNAL_PACKAGE_SEPARATOR;

        // We have to check if the name is already used, because
        // ZipOutputStream doesn't handle this case properly (it throws
        // an exception which can be caught, but the ZipDataEntry is
        // remembered anyway).
        if (jarEntryNames.add(name))
        {
            // Create a new directory entry.
            currentJarOutputStream.putNextEntry(new ZipEntry(name));
            currentJarOutputStream.closeEntry();
        }

        // Clear the finisher.
        currentFinisher  = null;
        currentDataEntry = null;

        return true;
    }


    public OutputStream getOutputStream(DataEntry dataEntry) throws IOException
    {
        return getOutputStream(dataEntry,  null);
    }


    public OutputStream getOutputStream(DataEntry dataEntry,
                                        Finisher  finisher) throws IOException
    {
        //Make sure we can start with a new entry.
        if (!prepareEntry(dataEntry))
        {
            return null;
        }

        // Do we need a new entry?
        if (!dataEntry.equals(currentDataEntry))
        {
            // Close the previous ZIP entry, if any.
            closeEntry();

            // Get the entry name.
            String name = dataEntry.getName();

            // We have to check if the name is already used, because
            // ZipOutputStream doesn't handle this case properly (it throws
            // an exception which can be caught, but the ZipDataEntry is
            // remembered anyway).
            if (!jarEntryNames.add(name))
            {
                throw new IOException("Duplicate zip entry ["+dataEntry+"]");
            }

            // Create a new entry.
            currentJarOutputStream.putNextEntry(new ZipEntry(name));

            // Set up the finisher for the entry.
            currentFinisher  = finisher;
            currentDataEntry = dataEntry;
        }

        return currentJarOutputStream;
    }


    public void finish() throws IOException
    {
        // Finish the entire ZIP stream, if any.
        if (currentJarOutputStream != null)
        {
            // Close the previous ZIP entry, if any.
            closeEntry();

            // Finish the entire ZIP stream.
            currentJarOutputStream.finish();
            currentJarOutputStream    = null;
            currentParentOutputStream = null;
            jarEntryNames.clear();
        }
    }


    public void close() throws IOException
    {
        // Close the parent stream.
        dataEntryWriter.close();
    }


    // Small utility methods.

    /**
     * Makes sure the current output stream is set up for the given entry.
     */
    private boolean prepareEntry(DataEntry dataEntry) throws IOException
    {
        // Get the parent stream, new or exisiting.
        // This may finish our own jar output stream.
        OutputStream parentOutputStream =
            dataEntryWriter.getOutputStream(dataEntry.getParent(), this);

        // Did we get a stream?
        if (parentOutputStream == null)
        {
            return false;
        }

        // Do we need a new stream?
        if (currentParentOutputStream == null)
        {
            currentParentOutputStream = parentOutputStream;

            // Create a new jar stream, with a manifest, if set.
            currentJarOutputStream = manifest != null ?
                new JarOutputStream(parentOutputStream, manifest) :
                new ZipOutputStream(parentOutputStream);

            // Add a comment, if set.
            if (comment != null)
            {
                currentJarOutputStream.setComment(comment);
            }
        }

        return true;
    }


    /**
     * Closes the previous ZIP entry, if any.
     */
    private void closeEntry() throws IOException
    {
        if (currentDataEntry != null)
        {
            // Let any finisher finish up first.
            if (currentFinisher != null)
            {
                currentFinisher.finish();
                currentFinisher = null;
            }

            currentJarOutputStream.closeEntry();
            currentDataEntry = null;
        }
    }
}
