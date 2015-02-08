
package proguard.util;

public class EmptyStringMatcher implements StringMatcher
{
    // Implementations for StringMatcher.

    public boolean matches(String string)
    {
        return string.length() == 0;
    }
}
