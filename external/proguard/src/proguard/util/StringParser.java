
package proguard.util;

public interface StringParser
{
    /**
     * Creates a StringMatcher for the given regular expression.
     */
    public StringMatcher parse(String regularExpression);
}
