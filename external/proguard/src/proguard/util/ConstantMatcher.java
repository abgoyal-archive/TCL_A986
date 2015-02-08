
package proguard.util;

public class ConstantMatcher implements StringMatcher
{
    private boolean matches;


    /**
     * Creates a new ConstantMatcher that always returns the given result.
     */
    public ConstantMatcher(boolean matches)
    {
        this.matches = matches;
    }


    // Implementations for StringMatcher.

    public boolean matches(String string)
    {
        return matches;
    }
}