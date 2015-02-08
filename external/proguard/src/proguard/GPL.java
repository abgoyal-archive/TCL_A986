
package proguard;

import java.io.*;
import java.util.*;

public class GPL
{
    /**
     * Prints out a note about the GPL if ProGuard is linked against unknown
     * code.
     */
    public static void check()
    {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        new Exception().printStackTrace(new PrintStream(out));
        LineNumberReader reader = new LineNumberReader(
                                  new InputStreamReader(
                                  new ByteArrayInputStream(out.toByteArray())));

        Set unknownPackageNames = unknownPackageNames(reader);

        if (unknownPackageNames.size() > 0)
        {
            String uniquePackageNames = uniquePackageNames(unknownPackageNames);

            System.out.println("ProGuard is released under the GNU General Public License. The authors of all");
            System.out.println("programs or plugins that link to it ("+uniquePackageNames+"...) therefore");
            System.out.println("must ensure that these programs carry the GNU General Public License as well.");
        }
    }


    /**
     * Returns a set of package names from the given stack trace.
     */
    private static Set unknownPackageNames(LineNumberReader reader)
    {
        Set packageNames = new HashSet();

        try
        {
            while (true)
            {
                String line = reader.readLine();
                if (line == null)
                {
                    break;
                }

                line = line.trim();
                if (line.startsWith("at "))
                {
                    line = line.substring(2).trim();
                    line = trimSuffix(line, '(');
                    line = trimSuffix(line, '.');
                    line = trimSuffix(line, '.');

                    if (line.length() > 0 && !isKnown(line))
                    {
                        packageNames.add(line);
                    }
                }
            }
        }
        catch (IOException ex)
        {
            // We'll just stop looking for more names.
        }

        return packageNames;
    }


    /**
     * Returns a comma-separated list of package names from the set, excluding
     * any subpackages of packages in the set.
     */
    private static String uniquePackageNames(Set packageNames)
    {
        StringBuffer buffer = new StringBuffer();

        Iterator iterator = packageNames.iterator();
        while (iterator.hasNext())
        {
            String packageName = (String)iterator.next();
            if (!containsPrefix(packageNames, packageName))
            {
                buffer.append(packageName).append(", ");
            }
        }

        return buffer.toString();
    }


    /**
     * Returns a given string without the suffix, as defined by the given
     * separator.
     */
    private static String trimSuffix(String string, char separator)
    {
        int index = string.lastIndexOf(separator);
        return index < 0 ? "" : string.substring(0, index);
    }


    /**
     * Returns whether the given set contains a prefix of the given name.
     */
    private static boolean containsPrefix(Set set, String name)
    {
        int index = 0;

        while (!set.contains(name.substring(0, index)))
        {
            index = name.indexOf('.', index + 1);
            if (index < 0)
            {
                return false;
            }
        }

        return true;
    }


    /**
     * Returns whether the given package name has been granted an exception
     * against the GPL linking clause, by the copyright holder of ProGuard.
     * This method is not legally binding, but of course the actual license is.
     * Please contact the copyright holder if you would like an exception for
     * your code as well.
     */
    private static boolean isKnown(String packageName)
    {
        return packageName.startsWith("java")                   ||
               packageName.startsWith("sun.reflect")            ||
               packageName.startsWith("proguard")               ||
               packageName.startsWith("org.apache.tools.ant")   ||
               packageName.startsWith("org.apache.tools.maven") ||
               packageName.startsWith("org.eclipse")            ||
               packageName.startsWith("org.netbeans")           ||
               packageName.startsWith("com.sun.kvem")           ||
               packageName.startsWith("net.certiv.proguarddt")  ||
               packageName.startsWith("eclipseme")              ||
               packageName.startsWith("jg.j2me")                ||
               packageName.startsWith("jg.common")              ||
               packageName.startsWith("jg.buildengine");
    }


    public static void main(String[] args)
    {
        LineNumberReader reader = new LineNumberReader(
                                  new InputStreamReader(System.in));

        Set unknownPackageNames = unknownPackageNames(reader);

        if (unknownPackageNames.size() > 0)
        {
            String uniquePackageNames = uniquePackageNames(unknownPackageNames);

            System.out.println(uniquePackageNames);
        }
    }
}
