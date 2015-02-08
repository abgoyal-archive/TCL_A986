
package proguard.util;

import proguard.classfile.ClassConstants;

public class ClassNameParser implements StringParser
{
    private static final char[] INTERNAL_PRIMITIVE_TYPES = new char[]
    {
        ClassConstants.INTERNAL_TYPE_VOID,
        ClassConstants.INTERNAL_TYPE_BOOLEAN,
        ClassConstants.INTERNAL_TYPE_BYTE,
        ClassConstants.INTERNAL_TYPE_CHAR,
        ClassConstants.INTERNAL_TYPE_SHORT,
        ClassConstants.INTERNAL_TYPE_INT,
        ClassConstants.INTERNAL_TYPE_LONG,
        ClassConstants.INTERNAL_TYPE_FLOAT,
        ClassConstants.INTERNAL_TYPE_DOUBLE,
    };


    // Implementations for StringParser.

    public StringMatcher parse(String regularExpression)
    {
        int           index;
        StringMatcher nextMatcher = new EmptyStringMatcher();

        // Look for wildcards.
        for (index = 0; index < regularExpression.length(); index++)
        {
            // Is there an 'L///;' wildcard?
            if (regularExpression.regionMatches(index, "L///;", 0, 5))
            {
                SettableMatcher settableMatcher = new SettableMatcher();

                // Create a matcher, recursively, for the remainder of the
                // string, optionally preceded by any type.
                nextMatcher =
                    new OrMatcher(parse(regularExpression.substring(index + 5)),
                                  createAnyTypeMatcher(settableMatcher));

                settableMatcher.setMatcher(nextMatcher);

                break;
            }

            // Is there an 'L***;' wildcard?
            if (regularExpression.regionMatches(index, "L***;", 0, 5))
            {
                // Create a matcher for the wildcard and, recursively, for the
                // remainder of the string.
                nextMatcher =
                    createAnyTypeMatcher(parse(regularExpression.substring(index + 5)));
                break;
            }

            // Is there a '**' wildcard?
            if (regularExpression.regionMatches(index, "**", 0, 2))
            {
                // Create a matcher for the wildcard and, recursively, for the
                // remainder of the string.
                nextMatcher =
                    new VariableStringMatcher(null,
                                              new char[] { ClassConstants.INTERNAL_TYPE_CLASS_END },
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
                                              new char[] { ClassConstants.INTERNAL_TYPE_CLASS_END, ClassConstants.INTERNAL_PACKAGE_SEPARATOR },
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
                                              new char[] { ClassConstants.INTERNAL_TYPE_CLASS_END, ClassConstants.INTERNAL_PACKAGE_SEPARATOR },
                                              1,
                                              1,
                                              parse(regularExpression.substring(index + 1)));
                break;
            }

            // Is there a '%' wildcard?
            else if (regularExpression.charAt(index) == '%')
            {
                // Create a matcher for the wildcard and, recursively, for the
                // remainder of the string.
                nextMatcher =
                    new VariableStringMatcher(INTERNAL_PRIMITIVE_TYPES,
                                              null,
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


    // Small utility methods.


    /**
     * Creates a StringMatcher that matches any type (class or primitive type,
     * array or non-array) and then the given matcher.
     */
    private VariableStringMatcher createAnyTypeMatcher(StringMatcher nextMatcher)
    {
        return new VariableStringMatcher(new char[] { ClassConstants.INTERNAL_TYPE_ARRAY },
                                  null,
                                  0,
                                  255,
        new OrMatcher(
        new VariableStringMatcher(INTERNAL_PRIMITIVE_TYPES,
                                  null,
                                  1,
                                  1,
                                  nextMatcher),
        new VariableStringMatcher(new char[] { ClassConstants.INTERNAL_TYPE_CLASS_START },
                                  null,
                                  1,
                                  1,
        new VariableStringMatcher(null,
                                  new char[] { ClassConstants.INTERNAL_TYPE_CLASS_END },
                                  0,
                                  Integer.MAX_VALUE,
        new VariableStringMatcher(new char[] { ClassConstants.INTERNAL_TYPE_CLASS_END },
                                  null,
                                  1,
                                  1,
                                  nextMatcher)))));
    }


    /**
     * A main method for testing class name matching.
     */
    public static void main(String[] args)
    {
        try
        {
            System.out.println("Regular expression ["+args[0]+"]");
            ClassNameParser parser  = new ClassNameParser();
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
