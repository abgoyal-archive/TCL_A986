
package proguard.ant;

import proguard.KeepClassSpecification;

import java.util.List;

public class KeepSpecificationElement extends ClassSpecificationElement
{
    private boolean allowShrinking;
    private boolean allowOptimization;
    private boolean allowObfuscation;


    /**
     * Adds the contents of this class specification element to the given list.
     * @param keepSpecifications the class specifications to be extended.
     * @param markClasses         specifies whether to mark the classes.
     * @param markConditionally   specifies whether to mark the classes
     *                            and class members conditionally.
     */
    public void appendTo(List    keepSpecifications,
                         boolean markClasses,
                         boolean markConditionally)
    {
        // Get the referenced file set, or else this one.
        KeepSpecificationElement keepSpecificationElement = isReference() ?
            (KeepSpecificationElement)getCheckedRef(this.getClass(),
                                                     this.getClass().getName()) :
            this;

        KeepClassSpecification keepClassSpecification =
            new KeepClassSpecification(markClasses,
                                  markConditionally,
                                  allowShrinking,
                                  allowOptimization,
                                  allowObfuscation,
                                  createClassSpecification(keepSpecificationElement));

        // Add it to the list.
        keepSpecifications.add(keepClassSpecification);
    }


    // Ant task attributes.

    public void setAllowshrinking(boolean allowShrinking)
    {
        this.allowShrinking = allowShrinking;
    }


    public void setAllowoptimization(boolean allowOptimization)
    {
        this.allowOptimization = allowOptimization;
    }


    public void setAllowobfuscation(boolean allowObfuscation)
    {
        this.allowObfuscation = allowObfuscation;
    }
}
