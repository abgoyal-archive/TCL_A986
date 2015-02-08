
package proguard;

public class KeepClassSpecification extends ClassSpecification
{
    public final boolean markClasses;
    public final boolean markConditionally;
    public final boolean allowShrinking;
    public final boolean allowOptimization;
    public final boolean allowObfuscation;


    /**
     * Creates a new KeepClassSpecification for all possible classes.
     * @param markClasses        specifies whether to mark the classes.
     *                           If false, only class members are marked.
     *                           If true, the classes are marked as well.
     * @param markConditionally  specifies whether to mark the classes and
     *                           class members conditionally. If true, classes
     *                           and class members are marked, on the condition
     *                           that all specified class members are present.
     * @param allowShrinking     specifies whether shrinking is allowed.
     * @param allowOptimization  specifies whether optimization is allowed.
     * @param allowObfuscation   specifies whether obfuscation is allowed.
     */
    public KeepClassSpecification(boolean markClasses,
                                  boolean markConditionally,
                                  boolean allowShrinking,
                                  boolean allowOptimization,
                                  boolean allowObfuscation)
    {
        this.markClasses       = markClasses;
        this.markConditionally = markConditionally;
        this.allowShrinking    = allowShrinking;
        this.allowOptimization = allowOptimization;
        this.allowObfuscation  = allowObfuscation;
    }


    /**
     * Creates a new KeepClassSpecification.
     * @param markClasses        specifies whether to mark the classes.
     *                           If false, only class members are marked.
     *                           If true, the classes are marked as well.
     * @param markConditionally  specifies whether to mark the classes and
     *                           class members conditionally. If true, classes
     *                           and class members are marked, on the condition
     *                           that all specified class members are present.
     * @param allowShrinking     specifies whether shrinking is allowed.
     * @param allowOptimization  specifies whether optimization is allowed.
     * @param allowObfuscation   specifies whether obfuscation is allowed.
     * @param classSpecification the specification of classes and class members.
     */
    public KeepClassSpecification(boolean            markClasses,
                                  boolean            markConditionally,
                                  boolean            allowShrinking,
                                  boolean            allowOptimization,
                                  boolean            allowObfuscation,
                                  ClassSpecification classSpecification)
    {
        super(classSpecification);

        this.markClasses       = markClasses;
        this.markConditionally = markConditionally;
        this.allowShrinking    = allowShrinking;
        this.allowOptimization = allowOptimization;
        this.allowObfuscation  = allowObfuscation;
    }


    // Implementations for Object.

    public boolean equals(Object object)
    {
        if (object == null ||
            this.getClass() != object.getClass())
        {
            return false;
        }

        KeepClassSpecification other = (KeepClassSpecification)object;
        return
            this.markClasses       == other.markClasses       &&
            this.markConditionally == other.markConditionally &&
            this.allowShrinking    == other.allowShrinking    &&
            this.allowOptimization == other.allowOptimization &&
            this.allowObfuscation  == other.allowObfuscation  &&
            super.equals(other);
    }

    public int hashCode()
    {
        return
            (markClasses       ? 0 :  1) ^
            (markConditionally ? 0 :  2) ^
            (allowShrinking    ? 0 :  4) ^
            (allowOptimization ? 0 :  8) ^
            (allowObfuscation  ? 0 : 16) ^
            super.hashCode();
    }

    public Object clone()
    {
//        try
//        {
            return super.clone();
//        }
//        catch (CloneNotSupportedException e)
//        {
//            return null;
//        }
    }
}
