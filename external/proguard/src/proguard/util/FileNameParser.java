
package proguard.util;

import java.io.File;

public class FileNameParser implements StringParser
{
    // Implementations for StringParser.

    public StringMatcher parse(String regularExpression)
    {
        int           index;
        StringMatcher nextMatcher = new EmptyStringMatcher();

        // Look for wildcards.
        for (index = 0; index < regularExpression.length(); index++)
        {
            // Is there a '**' wildcard?
            if (regularExpression.regionMatches(index, "**", 0, 2))
            {
                // Create a matcher for the wildcard and, recursively, for the
                // remainder of the string.
                nextMatcher =
                    new VariableStringMatcher(null,
                                              null,
                                              0,
                                              Integer.MAX_VALUE,
                                              parse(regularExpression.substring(index + 2)));
                break;
            }

            // Is there a '*' wildcard?
            else if (regularExpression.charAt(index) == '*')
            {
                // Create a matcher for the wildcard and, recursively, for the
                // remainder of the string.
                nextMatcher =
                    new VariableStringMatcher(null,
                                              new char[] { File.pathSeparatorChar, '/' },
                                              0,
                                              Integer.MAX_VALUE,
                                              parse(regularExpression.substring(index + 1)));
                break;
            }

            // Is there a '?' wildcard?
            else if (regularExpression.charAt(index) == '?')
            {
                // Create a matcher for the wildcard and, recursively, for the
                // remainder of the string.
                nextMatcher =
                    new VariableStringMatcher(null,
                                              new char[] { File.pathSeparatorChar, '/' },
                                              1,
                                              1,
                                              parse(regularExpression.substring(index + 1)));
                break;
            }
        }

        // Return a matcher for the fixed first part of the regular expression,
        // if any, and the remainder.
        return index != 0 ?
            (StringMatcher)new FixedStringMatcher(regularExpression.substring(0, index), nextMatcher) :
            (StringMatcher)nextMatcher;
    }


    /**
     * A main method for testing file name matching.
     */
    public static void main(String[] args)
    {
        try
        {
            System.out.println("Regular expression ["+args[0]+"]");
            FileNameParser parser  = new FileNameParser();
            StringMatcher  matcher = parser.parse(args[0]);
            for (int index = 1; index < args.length; index++)
            {
                String string = args[index];
                System.out.print("String             ["+string+"]");
                System.out.println(" -> match = "+matcher.matches(args[index]));
            }
        }
        catch (Exception ex)
        {
            ex.printStackTrace();
        }
    }
}
