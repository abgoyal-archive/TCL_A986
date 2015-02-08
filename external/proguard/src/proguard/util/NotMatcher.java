
package proguard.util;

public class NotMatcher implements StringMatcher
{
    private final StringMatcher matcher;


    public NotMatcher(StringMatcher matcher)
    {
        this.matcher = matcher;
    }


    // Implementations for StringMatcher.

    public boolean matches(String string)
    {
        return !matcher.matches(string);
    }
}
