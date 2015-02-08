
package proguard;


public class MemberSpecification
{
    public int    requiredSetAccessFlags;
    public int    requiredUnsetAccessFlags;
    public final String annotationType;
    public final String name;
    public final String descriptor;


    /**
     * Creates a new option to keep all possible class members.
     */
    public MemberSpecification()
    {
        this(0,
             0,
             null,
             null,
             null);
    }


    /**
     * Creates a new option to keep the specified class member(s).
     *
     * @param requiredSetAccessFlags   the class access flags that must be set
     *                                 in order for the class to apply.
     * @param requiredUnsetAccessFlags the class access flags that must be unset
     *                                 in order for the class to apply.
     * @param annotationType           the name of the class that must be an
     *                                 annotation in order for the class member
     *                                 to apply. The name may be null to specify
     *                                 that no annotation is required.
     * @param name                     the class member name. The name may be
     *                                 null to specify any class member or it
     *                                 may contain "*" or "?" wildcards.
     * @param descriptor               the class member descriptor. The
     *                                 descriptor may be null to specify any
     *                                 class member or it may contain
     *                                 "**", "*", or "?" wildcards.
     */
    public MemberSpecification(int    requiredSetAccessFlags,
                               int    requiredUnsetAccessFlags,
                               String annotationType,
                               String name,
                               String descriptor)
    {
        this.requiredSetAccessFlags   = requiredSetAccessFlags;
        this.requiredUnsetAccessFlags = requiredUnsetAccessFlags;
        this.annotationType           = annotationType;
        this.name                     = name;
        this.descriptor               = descriptor;
    }



    // Implementations for Object.

    public boolean equals(Object object)
    {
        if (object == null ||
            this.getClass() != object.getClass())
        {
            return false;
        }

        MemberSpecification other = (MemberSpecification)object;
        return
            (this.requiredSetAccessFlags   == other.requiredSetAccessFlags                                                                      ) &&
            (this.requiredUnsetAccessFlags == other.requiredUnsetAccessFlags                                                                    ) &&
            (this.annotationType == null ? other.annotationType == null : this.annotationType.equals(other.annotationType)) &&
            (this.name           == null ? other.name           == null : this.name.equals(other.name)                    ) &&
            (this.descriptor     == null ? other.descriptor     == null : this.descriptor.equals(other.descriptor)        );
    }

    public int hashCode()
    {
        return
            (requiredSetAccessFlags                                ) ^
            (requiredUnsetAccessFlags                              ) ^
            (annotationType == null ? 0 : annotationType.hashCode()) ^
            (name           == null ? 0 : name.hashCode()          ) ^
            (descriptor     == null ? 0 : descriptor.hashCode()    );
    }
}
