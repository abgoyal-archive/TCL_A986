
package proguard.util;

public class SettableMatcher implements StringMatcher
{
    private StringMatcher matcher;


    public void setMatcher(StringMatcher matcher)
    {
        this.matcher = matcher;
    }


    // Implementations for StringMatcher.

    public boolean matches(String string)
    {
        return matcher.matches(string);
    }
}
