
package proguard;

import java.util.*;


public class ClassPath
{
    private final List classPathEntries = new ArrayList();


    /**
     * Returns whether the class path contains any output entries.
     */
    public boolean hasOutput()
    {
        for (int index = 0; index < classPathEntries.size(); index++)
        {
            if (((ClassPathEntry)classPathEntries.get(index)).isOutput())
            {
                return true;
            }
        }

        return false;
    }


    // Delegates to List.

    public void clear()
    {
        classPathEntries.clear();
    }

    public void add(int index, ClassPathEntry classPathEntry)
    {
        classPathEntries.add(index, classPathEntry);
    }

    public boolean add(ClassPathEntry classPathEntry)
    {
        return classPathEntries.add(classPathEntry);
    }

    public boolean addAll(ClassPath classPath)
    {
        return classPathEntries.addAll(classPath.classPathEntries);
    }

    public ClassPathEntry get(int index)
    {
        return (ClassPathEntry)classPathEntries.get(index);
    }

    public ClassPathEntry remove(int index)
    {
        return (ClassPathEntry)classPathEntries.remove(index);
    }

    public boolean isEmpty()
    {
        return classPathEntries.isEmpty();
    }

    public int size()
    {
        return classPathEntries.size();
    }
}
