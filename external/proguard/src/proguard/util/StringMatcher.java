
package proguard.util;


public interface StringMatcher
{
    /**
     * Checks whether the given string matches.
     * @param string the string to match.
     * @return a boolean indicating whether the string matches the criterion.
     */
    public boolean matches(String string);
}
