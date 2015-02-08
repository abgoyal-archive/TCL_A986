

#ifndef WebKitPluginHostTypes_h
#define WebKitPluginHostTypes_h

typedef uint8_t* plist_bytes_t;
typedef uint8_t* application_name_t;

typedef char* data_t;

#ifndef __MigTypeCheck
#define __MigTypeCheck 1
#endif

enum LoadURLFlags {
    IsPost = 1 << 0,
    PostDataIsFile = 1 << 1, 
    AllowHeadersInPostData = 1 << 2,
    AllowPopups = 1 << 3,
};
 
enum InvokeType {
    Invoke,
    InvokeDefault,
    Construct
};

enum ValueType {
    VoidValueType,
    NullValueType,
    BoolValueType,
    DoubleValueType,
    StringValueType,
    JSObjectValueType,
    NPObjectValueType
};

#endif // WebKitPluginHostTypes_h
