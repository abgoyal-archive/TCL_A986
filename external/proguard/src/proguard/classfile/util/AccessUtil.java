
package proguard.classfile.util;

import proguard.classfile.ClassConstants;


public class AccessUtil
{
    public static final int PRIVATE         = 0;
    public static final int PACKAGE_VISIBLE = 1;
    public static final int PROTECTED       = 2;
    public static final int PUBLIC          = 3;


    // The mask of access flags.
    private static final int ACCESS_MASK =
        ClassConstants.INTERNAL_ACC_PUBLIC  |
        ClassConstants.INTERNAL_ACC_PRIVATE |
        ClassConstants.INTERNAL_ACC_PROTECTED;


    /**
     * Returns the corresponding access level of the given access flags.
     * @param accessFlags the internal access flags.
     * @return the corresponding access level: <code>PRIVATE</code>,
     *         <code>PACKAGE_VISIBLE</code>, <code>PROTECTED</code>, or
     *         <code>PUBLIC</code>.
     */
    public static int accessLevel(int accessFlags)
    {
        switch (accessFlags & ACCESS_MASK)
        {
            case ClassConstants.INTERNAL_ACC_PRIVATE:   return PRIVATE;
            default:                                    return PACKAGE_VISIBLE;
            case ClassConstants.INTERNAL_ACC_PROTECTED: return PROTECTED;
            case ClassConstants.INTERNAL_ACC_PUBLIC:    return PUBLIC;
        }
    }


    /**
     * Returns the corresponding access flags of the given access level.
     * @param accessLevel the access level: <code>PRIVATE</code>,
     *                    <code>PACKAGE_VISIBLE</code>, <code>PROTECTED</code>,
     *                    or <code>PUBLIC</code>.
     * @return the corresponding internal access flags,  the internal access
     *         flags as a logical bit mask of <code>INTERNAL_ACC_PRIVATE</code>,
     *         <code>INTERNAL_ACC_PROTECTED</code>, and
     *         <code>INTERNAL_ACC_PUBLIC</code>.
     */
    public static int accessFlags(int accessLevel)
    {
        switch (accessLevel)
        {
            case PRIVATE:   return ClassConstants.INTERNAL_ACC_PRIVATE;
            default:        return 0;
            case PROTECTED: return ClassConstants.INTERNAL_ACC_PROTECTED;
            case PUBLIC:    return ClassConstants.INTERNAL_ACC_PUBLIC;
        }
    }


    /**
     * Replaces the access part of the given access flags.
     * @param accessFlags the internal access flags.
     * @param accessFlags the new internal access flags.
     */
    public static int replaceAccessFlags(int accessFlags, int newAccessFlags)
    {
        // A private class member should not be explicitly final.
        if (newAccessFlags == ClassConstants.INTERNAL_ACC_PRIVATE)
        {
            accessFlags &= ~ClassConstants.INTERNAL_ACC_FINAL;
        }

        return (accessFlags    & ~ACCESS_MASK) |
               (newAccessFlags &  ACCESS_MASK);
    }
}
