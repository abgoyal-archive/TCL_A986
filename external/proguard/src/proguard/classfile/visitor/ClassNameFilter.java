
package proguard.classfile.visitor;

import proguard.classfile.*;
import proguard.util.*;

import java.util.List;

public class ClassNameFilter implements ClassVisitor
{
    private final StringMatcher regularExpressionMatcher;
    private final ClassVisitor  classVisitor;


    /**
     * Creates a new ClassNameFilter.
     * @param regularExpression the regular expression against which class names
     *                          will be matched.
     * @param classVisitor      the <code>ClassVisitor</code> to which visits
     *                          will be delegated.
     */
    public ClassNameFilter(String       regularExpression,
                           ClassVisitor classVisitor)
    {
        this(new ListParser(new ClassNameParser()).parse(regularExpression),
             classVisitor);
    }


    /**
     * Creates a new ClassNameFilter.
     * @param regularExpression the regular expression against which class names
     *                          will be matched.
     * @param classVisitor      the <code>ClassVisitor</code> to which visits
     *                          will be delegated.
     */
    public ClassNameFilter(List         regularExpression,
                           ClassVisitor classVisitor)
    {
        this(new ListParser(new ClassNameParser()).parse(regularExpression),
             classVisitor);
    }


    /**
     * Creates a new ClassNameFilter.
     * @param regularExpressionMatcher the regular expression against which
     *                                 class names will be matched.
     * @param classVisitor             the <code>ClassVisitor</code> to which
     *                                 visits will be delegated.
     */
    public ClassNameFilter(StringMatcher regularExpressionMatcher,
                           ClassVisitor  classVisitor)
    {
        this.regularExpressionMatcher = regularExpressionMatcher;
        this.classVisitor             = classVisitor;
    }


    // Implementations for ClassVisitor.

    public void visitProgramClass(ProgramClass programClass)
    {
        if (accepted(programClass.getName()))
        {
            classVisitor.visitProgramClass(programClass);
        }
    }


    public void visitLibraryClass(LibraryClass libraryClass)
    {
        if (accepted(libraryClass.getName()))
        {
            classVisitor.visitLibraryClass(libraryClass);
        }
    }


    // Small utility methods.

    private boolean accepted(String name)
    {
        return regularExpressionMatcher.matches(name);
    }
}
