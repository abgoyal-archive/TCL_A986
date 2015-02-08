
package proguard.classfile.visitor;

import proguard.classfile.*;

import java.util.Set;

public class ClassVersionSetter implements ClassVisitor
{
    private final int classVersion;

    private final Set newerClassVersions;


    /**
     * Creates a new ClassVersionSetter.
     * @param classVersion the class version number.
     */
    public ClassVersionSetter(int classVersion)
    {
        this(classVersion, null);
    }


    /**
     * Creates a new ClassVersionSetter that also stores any newer class version
     * numbers that it encounters while visiting program classes.
     * @param classVersion       the class version number.
     * @param newerClassVersions the <code>Set</code> in which newer class
     *                           version numbers can be collected.
     */
    public ClassVersionSetter(int classVersion,
                              Set newerClassVersions)
    {
        this.classVersion       = classVersion;
        this.newerClassVersions = newerClassVersions;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (programClass.u4version > classVersion &&
            newerClassVersions != null)
        {
            newerClassVersions.add(new Integer(programClass.u4version));
        }

        programClass.u4version = classVersion;
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        // Library classes don't have version numbers.
    }
}
