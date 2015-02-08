
package proguard.util;

public class ExtensionMatcher implements StringMatcher
{
    private final String extension;


    /**
     * Creates a new StringMatcher.
     * @param extension the extension against which strings will be matched.
     */
    public ExtensionMatcher(String extension)
    {
        this.extension = extension;
    }


    // Implementations for StringMatcher.

    public boolean matches(String string)
    {
        return endsWithIgnoreCase(string, extension);
    }


    /**
     * Returns whether the given string ends with the given suffix, ignoring its
     * case.
     */
    private static boolean endsWithIgnoreCase(String string, String suffix)
    {
        int stringLength = string.length();
        int suffixLength = suffix.length();

        return string.regionMatches(true, stringLength - suffixLength, suffix, 0, suffixLength);
    }
}
