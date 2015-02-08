
package proguard.obfuscate;

import java.io.*;
import java.util.*;

public class DictionaryNameFactory implements NameFactory
{
    private static final char COMMENT_CHARACTER = '#';


    private final List        names;
    private final NameFactory nameFactory;

    private int index = 0;


    /**
     * Creates a new <code>DictionaryNameFactory</code>.
     * @param file        the file from which the names can be read.
     * @param nameFactory the name factory from which names will be retrieved
     *                    if the list of read names has been exhausted.
     */
    public DictionaryNameFactory(File        file,
                                 NameFactory nameFactory) throws IOException
    {
        this.names       = new ArrayList();
        this.nameFactory = nameFactory;

        Reader reader = new FileReader(file);

        try
        {
            StringBuffer buffer = new StringBuffer();

            while (true)
            {
                // Read the next character.
                int c = reader.read();

                // Is it a valid identifier character?
                if (c != -1 &&
                    (buffer.length() == 0 ?
                         Character.isJavaIdentifierStart((char)c) :
                         Character.isJavaIdentifierPart((char)c)))
                {
                    // Append it to the current identifier.
                    buffer.append((char)c);
                }
                else
                {
                    // Did we collect a new identifier?
                    if (buffer.length() > 0)
                    {
                        // Add the completed name to the list of names, if it's
                        // not in it yet.
                        String name = buffer.toString();
                        if (!names.contains(name))
                        {
                            names.add(name);
                        }

                        // Clear the buffer.
                        buffer.setLength(0);
                    }

                    // Is this the beginning of a comment line?
                    if (c == COMMENT_CHARACTER)
                    {
                        // Skip all characters till the end of the line.
                        do
                        {
                            c = reader.read();
                        }
                        while (c != -1 &&
                               c != '\n' &&
                               c != '\r');
                    }

                    // Is this the end of the file?
                    if (c == -1)
                    {
                        // Just return.
                        return;
                    }
                }
            }
        }
        finally
        {
            reader.close();
        }
    }


    /**
     * Creates a new <code>DictionaryNameFactory</code>.
     * @param dictionaryNameFactory the dictionary name factory whose dictionary
     *                              will be used.
     * @param nameFactory           the name factory from which names will be
     *                              retrieved if the list of read names has been
     *                              exhausted.
     */
    public DictionaryNameFactory(DictionaryNameFactory dictionaryNameFactory,
                                 NameFactory           nameFactory)
    {
        this.names       = dictionaryNameFactory.names;
        this.nameFactory = nameFactory;
    }


    // Implementations for NameFactory.

    public void reset()
    {
        index = 0;

        nameFactory.reset();
    }


    public String nextName()
    {
        String name;

        // Do we still have names?
        if (index < names.size())
        {
            // Return the next name.
            name = (String)names.get(index++);
        }
        else
        {
            // Return the next different name from the other name factory.
            do
            {
                name = nameFactory.nextName();
            }
            while (names.contains(name));
        }

        return name;
    }


    public static void main(String[] args)
    {
        try
        {
            DictionaryNameFactory factory =
                new DictionaryNameFactory(new File(args[0]), new SimpleNameFactory());

            for (int counter = 0; counter < 50; counter++)
            {
                System.out.println("["+factory.nextName()+"]");
            }
        }
        catch (IOException ex)
        {
            ex.printStackTrace();
        }
    }
}
