
package proguard.ant;

import org.apache.tools.ant.types.DataType;
import proguard.classfile.util.ClassUtil;
import proguard.util.ListUtil;

import java.util.List;

public class FilterElement extends DataType
{
    private String filter;


    /**
     * Adds the contents of this element to the given name filter.
     * @param filter   the list of attributes to be extended.
     * @param internal specifies whether the filter string should be converted
     *                 to internal types.
     */
    public void appendTo(List filter, boolean internal)
    {
        // Get the referenced element, or else this one.
        FilterElement filterElement = isReference() ?
            (FilterElement)getCheckedRef(this.getClass(),
                                         this.getClass().getName()) :
            this;

        String filterString = filterElement.filter;

        if (filterString == null)
        {
            // Clear the filter to keep all names.
            filter.clear();
        }
        else
        {
            if (internal)
            {
                filterString = ClassUtil.internalClassName(filterString);
            }

            // Append the filter.
            filter.addAll(ListUtil.commaSeparatedList(filterString));
        }
    }


    // Ant task attributes.

    public void setName(String name)
    {
        this.filter = name;
    }


    public void setFilter(String filter)
    {
        this.filter = filter;
    }
}
