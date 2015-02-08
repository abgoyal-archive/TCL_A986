
package proguard.classfile;




public interface VisitorAccepter
{
    /**
     * Gets the visitor information of the visitor accepter.
     */
    public Object getVisitorInfo();


    /**
     * Sets the visitor information of the visitor accepter.
     */
    public void setVisitorInfo(Object visitorInfo);
}
