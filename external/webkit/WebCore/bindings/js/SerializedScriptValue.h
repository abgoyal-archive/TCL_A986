

#ifndef SerializedScriptValue_h
#define SerializedScriptValue_h

#include "ScriptValue.h"

typedef const struct OpaqueJSContext* JSContextRef;
typedef const struct OpaqueJSValue* JSValueRef;

namespace WebCore {
    class File;
    class FileList;
    class ImageData;
    class SerializedArray;
    class SerializedFileList;
    class SerializedImageData;
    class SerializedObject;

    class SharedSerializedData : public RefCounted<SharedSerializedData> {
    public:
        virtual ~SharedSerializedData() { }
        SerializedArray* asArray();
        SerializedObject* asObject();
        SerializedFileList* asFileList();
        SerializedImageData* asImageData();
    };

    class SerializedScriptValue;

    class SerializedScriptValueData {
    public:
        enum SerializedType {
            EmptyType,
            DateType,
            NumberType,
            ImmediateType,
            ObjectType,
            ArrayType,
            StringType,
            FileType,
            FileListType,
            ImageDataType
        };

        SerializedType type() const { return m_type; }
        static SerializedScriptValueData serialize(JSC::ExecState*, JSC::JSValue);
        JSC::JSValue deserialize(JSC::ExecState*, JSC::JSGlobalObject*, bool mustCopy) const;

        ~SerializedScriptValueData()
        {
            if (m_sharedData)
                tearDownSerializedData();
        }

        SerializedScriptValueData()
            : m_type(EmptyType)
        {
        }

        explicit SerializedScriptValueData(const String& string)
            : m_type(StringType)
            , m_string(string.crossThreadString()) // FIXME: Should be able to just share the Rep
        {
        }
        
        explicit SerializedScriptValueData(const File*);
        explicit SerializedScriptValueData(const FileList*);
        explicit SerializedScriptValueData(const ImageData*);

        explicit SerializedScriptValueData(JSC::JSValue value)
            : m_type(ImmediateType)
        {
            ASSERT(!value.isCell());
            m_data.m_immediate = JSC::JSValue::encode(value);
        }

        SerializedScriptValueData(SerializedType type, double value)
            : m_type(type)
        {
            m_data.m_double = value;
        }

        SerializedScriptValueData(RefPtr<SerializedObject>);
        SerializedScriptValueData(RefPtr<SerializedArray>);

        JSC::JSValue asImmediate() const
        {
            ASSERT(m_type == ImmediateType);
            return JSC::JSValue::decode(m_data.m_immediate);
        }

        double asDouble() const
        {
            ASSERT(m_type == NumberType || m_type == DateType);
            return m_data.m_double;
        }

        String asString() const
        {
            ASSERT(m_type == StringType || m_type == FileType);
            return m_string;
        }

        SerializedObject* asObject() const
        {
            ASSERT(m_type == ObjectType);
            ASSERT(m_sharedData);
            return m_sharedData->asObject();
        }

        SerializedArray* asArray() const
        {
            ASSERT(m_type == ArrayType);
            ASSERT(m_sharedData);
            return m_sharedData->asArray();
        }

        SerializedFileList* asFileList() const
        {
            ASSERT(m_type == FileListType);
            ASSERT(m_sharedData);
            return m_sharedData->asFileList();
        }
        
        SerializedImageData* asImageData() const
        {
            ASSERT(m_type == ImageDataType);
            ASSERT(m_sharedData);
            return m_sharedData->asImageData();
        }

        operator bool() const { return m_type != EmptyType; }

        SerializedScriptValueData release()
        {
            SerializedScriptValueData result = *this;
            *this = SerializedScriptValueData();
            return result;
        }

    private:
        void tearDownSerializedData();
        SerializedType m_type;
        RefPtr<SharedSerializedData> m_sharedData;
        String m_string;
        union {
            double m_double;
            JSC::EncodedJSValue m_immediate;
        } m_data;
    };

    class SerializedScriptValue : public RefCounted<SerializedScriptValue> {
    public:
        static PassRefPtr<SerializedScriptValue> create(JSC::ExecState* exec, JSC::JSValue value)
        {
            return adoptRef(new SerializedScriptValue(SerializedScriptValueData::serialize(exec, value)));
        }

        static PassRefPtr<SerializedScriptValue> create(JSContextRef, JSValueRef value, JSValueRef* exception);

        static PassRefPtr<SerializedScriptValue> create(String string)
        {
            return adoptRef(new SerializedScriptValue(SerializedScriptValueData(string)));
        }

        static PassRefPtr<SerializedScriptValue> create()
        {
            return adoptRef(new SerializedScriptValue(SerializedScriptValueData()));
        }

        PassRefPtr<SerializedScriptValue> release()
        {
            PassRefPtr<SerializedScriptValue> result = adoptRef(new SerializedScriptValue(m_value));
            m_value = SerializedScriptValueData();
            result->m_mustCopy = true;
            return result;
        }

        String toString()
        {
            if (m_value.type() != SerializedScriptValueData::StringType)
                return "";
            return m_value.asString();
        }

        JSC::JSValue deserialize(JSC::ExecState* exec, JSC::JSGlobalObject* globalObject)
        {
            if (!m_value)
                return JSC::jsNull();
            return m_value.deserialize(exec, globalObject, m_mustCopy);
        }

        JSValueRef deserialize(JSContextRef, JSValueRef* exception);
        ~SerializedScriptValue();

    private:
        SerializedScriptValue(SerializedScriptValueData value)
            : m_value(value)
            , m_mustCopy(false)
        {
        }

        SerializedScriptValueData m_value;
        bool m_mustCopy;
    };
}

#endif // SerializedScriptValue_h
