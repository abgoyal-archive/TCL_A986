

#ifndef CharacterData_h
#define CharacterData_h

#include "Node.h"

namespace WebCore {

class CharacterData : public Node {
public:
    String data() const { return m_data; }
    void setData(const String&, ExceptionCode&);
    unsigned length() const { return m_data->length(); }
    String substringData(unsigned offset, unsigned count, ExceptionCode&);
    void appendData(const String&, ExceptionCode&);
    void insertData(unsigned offset, const String&, ExceptionCode&);
    void deleteData(unsigned offset, unsigned count, ExceptionCode&);
    void replaceData(unsigned offset, unsigned count, const String&, ExceptionCode&);

    bool containsOnlyWhitespace() const;

    StringImpl* dataImpl() { return m_data.get(); }

protected:
    CharacterData(Document*, const String&, ConstructionType);

    virtual bool rendererIsNeeded(RenderStyle*);

    void setDataImpl(PassRefPtr<StringImpl> impl) { m_data = impl; }
    void dispatchModifiedEvent(StringImpl* oldValue);

private:
    virtual String nodeValue() const;
    virtual void setNodeValue(const String&, ExceptionCode&);
    virtual bool isCharacterDataNode() const { return true; }
    virtual int maxCharacterOffset() const;
    virtual bool offsetInCharacters() const;

    void checkCharDataOperation(unsigned offset, ExceptionCode&);

    RefPtr<StringImpl> m_data;
};

} // namespace WebCore

#endif // CharacterData_h

