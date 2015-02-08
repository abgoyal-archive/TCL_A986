
package proguard.io;

import proguard.classfile.*;

import java.io.*;

public class DataEntryRewriter extends DataEntryCopier
{
    private final ClassPool classPool;


    /**
     * Creates a new DataEntryRewriter.
     */
    public DataEntryRewriter(ClassPool       classPool,
                             DataEntryWriter dataEntryWriter)
    {
        super(dataEntryWriter);

        this.classPool = classPool;
    }


    // Implementations for DataEntryCopier.

    protected void copyData(InputStream  inputStream,
                            OutputStream outputStream)
    throws IOException
    {
        Reader reader = new BufferedReader(new InputStreamReader(inputStream));
        Writer writer = new BufferedWriter(new OutputStreamWriter(outputStream));

        copyData(reader, writer);

        writer.flush();
        outputStream.flush();
    }


    /**
     * Copies all data that it can read from the given reader to the given
     * writer.
     */
    protected void copyData(Reader reader,
                            Writer writer)
    throws IOException
    {
        StringBuffer word = new StringBuffer();

        while (true)
        {
            int i = reader.read();
            if (i < 0)
            {
                break;
            }

            // Is the character part of a word?
            char c = (char)i;
            if (Character.isJavaIdentifierPart(c) ||
                c == '.' ||
                c == '-')
            {
                // Collect the characters in this word.
                word.append(c);
            }
            else
            {
                // Write out the updated word, if any.
                writeUpdatedWord(writer, word.toString());
                word.setLength(0);

                // Write out the character that terminated it.
                writer.write(c);
            }
        }

        // Write out the final word.
        writeUpdatedWord(writer, word.toString());
    }


    // Small utility methods.

    /**
     * Writes the given word to the given writer, after having adapted it,
     * based on the renamed class names.
     */
    private void writeUpdatedWord(Writer writer, String word)
    throws IOException
    {
        if (word.length() > 0)
        {
            String newWord = word;

            boolean containsDots = word.indexOf('.') >= 0;

            // Replace dots by forward slashes.
            String className = containsDots ?
                word.replace('.', ClassConstants.INTERNAL_PACKAGE_SEPARATOR) :
                word;

            // Find the class corrsponding to the word.
            Clazz clazz = classPool.getClass(className);
            if (clazz != null)
            {
                // Update the word if necessary.
                String newClassName = clazz.getName();
                if (!className.equals(newClassName))
                {
                    // Replace forward slashes by dots.
                    newWord = containsDots ?
                        newClassName.replace(ClassConstants.INTERNAL_PACKAGE_SEPARATOR, '.') :
                        newClassName;
                }
            }

            writer.write(newWord);
        }
    }
}
