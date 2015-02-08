
package proguard.io;

import proguard.classfile.*;

import java.io.*;

public class ManifestRewriter extends DataEntryRewriter
{
    /**
     * Creates a new ManifestRewriter.
     */
    public ManifestRewriter(ClassPool       classPool,
                            DataEntryWriter dataEntryWriter)
    {
        super(classPool, dataEntryWriter);
    }


    // Implementations for DataEntryRewriter.

    protected void copyData(Reader reader,
                            Writer writer)
    throws IOException
    {
        super.copyData(new SplitLineReader(reader),
                       new SplitLineWriter(writer));
    }


    /**
     * This Reader reads manifest files, joining any split lines.
     */
    private static class SplitLineReader extends FilterReader
    {
        private char[] buffer      = new char[2];
        private int    bufferIndex = 0;
        private int    bufferSize  = 0;


        public SplitLineReader(Reader reader)
        {
            super(reader);
        }


        // Implementations for Reader.

        public int read() throws IOException
        {
            while (true)
            {
                if (bufferIndex < bufferSize)
                {
                    return buffer[bufferIndex++];
                }

                // Read the first character.
                int c1 = super.read();
                if (c1 != '\n' && c1 != '\r')
                {
                    return c1;
                }

                bufferIndex = 0;
                bufferSize  = 0;
                buffer[bufferSize++] = '\n';

                // Read the second character.
                int c2 = super.read();
                if (c2 == ' ')
                {
                    bufferSize = 0;
                    continue;
                }

                if (c1 != '\r' || c2 != '\n')
                {
                    buffer[bufferSize++] = (char)c2;
                    continue;
                }

                // Read the third character.
                int c3 = super.read();
                if (c3 == ' ')
                {
                    bufferSize = 0;
                    continue;
                }

                buffer[bufferSize++] = (char)c3;
            }
        }


        public int read(char[] cbuf, int off, int len) throws IOException
        {
            // Delegate to reading a single character at a time.
            int count = 0;
            while (count < len)
            {
                int c = read();
                if (c == -1)
                {
                    break;
                }

                cbuf[off + count++] = (char)c;
            }

            return count;
        }


        public long skip(long n) throws IOException
        {
            // Delegate to reading a single character at a time.
            int count = 0;
            while (count < n)
            {
                int c = read();
                if (c == -1)
                {
                    break;
                }

                count++;
            }

            return count;
        }
    }


    /**
     * This Writer writes manifest files, splitting any long lines.
     */
    private static class SplitLineWriter extends FilterWriter
    {
        private int counter = 0;


        public SplitLineWriter(Writer writer)
        {
            super(writer);
        }


        // Implementations for Reader.

        public void write(int c) throws IOException
        {
            // TODO: We should actually count the Utf-8 bytes, not the characters.
            if (c == '\n')
            {
                // Reset the character count.
                counter = 0;
            }
            else if (counter == 70)
            {
                // Insert are newline and space.
                super.write('\n');
                super.write(' ');

                counter = 2;
            }
            else
            {
                counter++;
            }

            super.write(c);
        }


        public void write(char[] cbuf, int off, int len) throws IOException
        {
            for (int count = 0; count < len; count++)
            {
                write(cbuf[off + count]);
            }
        }


        public void write(String str, int off, int len) throws IOException
        {
            write(str.toCharArray(), off, len);
        }
    }
}