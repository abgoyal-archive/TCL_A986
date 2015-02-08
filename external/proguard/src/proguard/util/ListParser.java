
package proguard.util;

import java.util.List;

public class ListParser implements StringParser
{
    private final StringParser stringParser;


    /**
     * Creates a new ListParser that parses individual elements in the
     * comma-separated list with the given StringParser.
     */
    public ListParser(StringParser stringParser)
    {
        this.stringParser = stringParser;
    }


    // Implementations for StringParser.

    public StringMatcher parse(String regularExpression)
    {
        // Does the regular expression contain a ',' list separator?
        return parse(ListUtil.commaSeparatedList(regularExpression));
    }


    /**
     * Creates a StringMatcher for the given regular expression, which can
     * be a list of optionally negated simple entries.
     * <p>
     * An empty list results in a StringMatcher that matches any string.
     */
    public StringMatcher parse(List regularExpressions)
    {
        StringMatcher listMatcher = null;

        // Loop over all simple regular expressions, backward, creating a
        // linked list of matchers.
        for (int index = regularExpressions.size()-1; index >=0; index--)
        {
            String regularExpression = (String)regularExpressions.get(index);

            StringMatcher entryMatcher = parseEntry(regularExpression);

            // Prepend the entry matcher.
            listMatcher =
                listMatcher == null ?
                    (StringMatcher)entryMatcher :
                isNegated(regularExpression) ?
                    (StringMatcher)new AndMatcher(entryMatcher, listMatcher) :
                    (StringMatcher)new OrMatcher(entryMatcher, listMatcher);
        }

        return listMatcher != null ? listMatcher : new ConstantMatcher(true);
    }


    // Small utility methods.

    /**
     * Creates a StringMatcher for the given regular expression, which is a
     * an optionally negated simple expression.
     */
    private StringMatcher parseEntry(String regularExpression)
    {
        // Wrap the matcher if the regular expression starts with a '!' negator.
        return isNegated(regularExpression) ?
          new NotMatcher(stringParser.parse(regularExpression.substring(1))) :
          stringParser.parse(regularExpression);
    }


    /**
     * Returns whether the given simple regular expression is negated.
     */
    private boolean isNegated(String regularExpression)
    {
        return regularExpression.length() > 0 &&
               regularExpression.charAt(0) == '!';
    }


    /**
     * A main method for testing name matching.
     */
    public static void main(String[] args)
    {
        try
        {
            System.out.println("Regular expression ["+args[0]+"]");
            ListParser parser  = new ListParser(new NameParser());
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
