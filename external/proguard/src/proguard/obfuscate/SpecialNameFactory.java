
package proguard.obfuscate;

public class SpecialNameFactory implements NameFactory
{
    private static final char SPECIAL_SUFFIX = '_';


    private final NameFactory nameFactory;


    /**
     * Creates a new <code>SpecialNameFactory</code>.
     * @param nameFactory the name factory from which original names will be
     *                    retrieved.
     */
    public SpecialNameFactory(NameFactory nameFactory)
    {
        this.nameFactory = nameFactory;
    }


    // Implementations for NameFactory.

    public void reset()
    {
        nameFactory.reset();
    }


    public String nextName()
    {
        return nameFactory.nextName() + SPECIAL_SUFFIX;
    }


    // Small utility methods.

    /**
     * Returns whether the given name is special.
     */
    static boolean isSpecialName(String name)
    {
        return name != null &&
               name.charAt(name.length()-1) == SPECIAL_SUFFIX;
    }


    public static void main(String[] args)
    {
        SpecialNameFactory factory = new SpecialNameFactory(new SimpleNameFactory());

        for (int counter = 0; counter < 50; counter++)
        {
            System.out.println("["+factory.nextName()+"]");
        }
    }
}
