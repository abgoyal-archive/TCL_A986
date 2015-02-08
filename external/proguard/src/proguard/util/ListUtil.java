
package proguard.util;

import java.util.*;


public class ListUtil
{
    /**
     * Creates a comma-separated String from the given List of String objects.
     */
    public static String commaSeparatedString(List list)
    {
        if (list == null)
        {
            return null;
        }

        StringBuffer buffer = new StringBuffer();

        for (int index = 0; index < list.size(); index++)
        {
            if (index > 0)
            {
                buffer.append(',');
            }

            buffer.append(quotedString((String)list.get(index)));
        }

        return buffer.toString();
    }


    /**
     * Creates a List of String objects from the given comma-separated String.
     */
    public static List commaSeparatedList(String string)
    {
        if (string == null)
        {
            return null;
        }

        List list = new ArrayList();
        int index = 0;
        while ((index = skipWhitespace(string, index)) < string.length())
        {
            int nextIndex;

            // Do we have an opening quote?
            if (string.charAt(index) == '\'')
            {
                // Parse a quoted string.
                nextIndex = string.indexOf('\'', index + 1);
                if (nextIndex < 0)
                {
                    nextIndex = string.length();
                }

                list.add(string.substring(index + 1, nextIndex));
            }
            else
            {
                // Parse a non-quoted string.
                nextIndex = string.indexOf(',', index);
                if (nextIndex < 0)
                {
                    nextIndex = string.length();
                }

                String substring = string.substring(index, nextIndex).trim();
                if (substring.length() > 0)
                {
                    list.add(substring);
                }
            }

            index = nextIndex + 1;
        }

        return list;
    }


    /**
     * Skips any whitespace characters.
     */
    private static int skipWhitespace(String string, int index)
    {
        while (index < string.length() &&
               Character.isWhitespace(string.charAt(index)))
        {
            index++;
        }
        return index;
    }


    /**
     * Returns a quoted version of the given string, if necessary.
     */
    private static String quotedString(String string)
    {
        return string.length()     == 0 ||
               string.indexOf(' ') >= 0 ||
               string.indexOf('@') >= 0 ||
               string.indexOf('{') >= 0 ||
               string.indexOf('}') >= 0 ||
               string.indexOf('(') >= 0 ||
               string.indexOf(')') >= 0 ||
               string.indexOf(':') >= 0 ||
               string.indexOf(';') >= 0 ||
               string.indexOf(',') >= 0  ? ("'" + string + "'") :
                                           (      string      );
    }


    public static void main(String[] args)
    {
        if (args.length == 1)
        {
            System.out.println("Input string: ["+args[0]+"]");

            List list = commaSeparatedList(args[0]);

            System.out.println("Resulting list:");
            for (int index = 0; index < list.size(); index++)
            {
                System.out.println("["+list.get(index)+"]");
            }
        }
        else
        {
            List list = Arrays.asList(args);

            System.out.println("Input list:");
            for (int index = 0; index < list.size(); index++)
            {
                System.out.println("["+list.get(index)+"]");
            }

            String string = commaSeparatedString(list);

            System.out.println("Resulting string: ["+string+"]");
        }
    }
}
