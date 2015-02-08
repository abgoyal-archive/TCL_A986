package jdiff;

import java.io.*;
import java.util.*;

class ParamAPI implements Comparable {
    /** Name of the (name, type) pair. */
    public String name_;

    /** Type of the (name, type) pair. */
    public String type_;

    public ParamAPI(String name, String type) {
        name_ = name;
        type_ = type;
    }

    /** Compare two ParamAPI objects using both name and type. */
    public int compareTo(Object o) {
        ParamAPI oParamAPI = (ParamAPI)o;
        int comp = name_.compareTo(oParamAPI.name_);
        if (comp != 0)
            return comp;
        comp = type_.compareTo(oParamAPI.type_);
        if (comp != 0)
            return comp;
        return 0;
    }
  
    /** 
     * Tests two ParamAPI objects using just the name, used by indexOf().
     */
    public boolean equals(Object o) {
        if (name_.compareTo(((ParamAPI)o).name_) == 0)
            return true;
        return false;
    }
    
    /** Used to create signatures. */
    public String toString() {
        if (type_.compareTo("void") == 0)
            return "";
        return type_;
    }
}  
