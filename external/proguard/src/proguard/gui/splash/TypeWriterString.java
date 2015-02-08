
package proguard.gui.splash;

public class TypeWriterString implements VariableString
{
    private final String string;
    private final Timing timing;

    private int    cachedLength = -1;
    private String cachedString;


    /**
     * Creates a new TypeWriterString.
     * @param string the basic String.
     * @param timing the applied timing.
     */
    public TypeWriterString(String string, Timing timing)
    {
        this.string = string;
        this.timing = timing;
    }


    // Implementation for VariableString.

    public String getString(long time)
    {
        double t = timing.getTiming(time);

        int stringLength = string.length();
        int length = (int)(stringLength * t + 0.5);
        if (length != cachedLength)
        {
            cachedLength = length;
            cachedString = string.substring(0, length);
            if (t > 0.0 && length < stringLength)
            {
                cachedString += "_";
            }
        }

        return cachedString;
    }
}
