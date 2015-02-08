

#ifndef WebHTMLRepresentation_H
#define WebHTMLRepresentation_H

#include "WebKit.h"

class WebFrame;

class WebHTMLRepresentation : public IWebHTMLRepresentation, IWebDocumentRepresentation
{
public:
    static WebHTMLRepresentation* createInstance(WebFrame* frame);
protected:
    WebHTMLRepresentation();
    ~WebHTMLRepresentation();
public:

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef();
    virtual ULONG STDMETHODCALLTYPE Release();

    // IWebHTMLRepresentation
    virtual HRESULT STDMETHODCALLTYPE supportedMIMETypes(
        /* [out][in] */ BSTR* types,
        /* [out][in] */ int* cTypes);
    
    virtual HRESULT STDMETHODCALLTYPE supportedNonImageMIMETypes(
        /* [out][in] */ BSTR* types,
        /* [out][in] */ int* cTypes);
    
    virtual HRESULT STDMETHODCALLTYPE supportedImageMIMETypes(
        /* [out][in] */ BSTR* types,
        /* [out][in] */ int* cTypes);
    
    virtual HRESULT STDMETHODCALLTYPE attributedStringFromDOMNodes(
        /* [in] */ IDOMNode* startNode,
        /* [in] */ int startOffset,
        /* [in] */ IDOMNode* endNode,
        /* [in] */ int endOffset,
        /* [retval][out] */ IDataObject** attributedString);
    
    virtual HRESULT STDMETHODCALLTYPE elementWithName(
        /* [in] */ BSTR name,
        /* [in] */ IDOMElement* form,
        /* [retval][out] */ IDOMElement** element);
    
    virtual HRESULT STDMETHODCALLTYPE elementDoesAutoComplete(
        /* [in] */ IDOMElement* element,
        /* [retval][out] */ BOOL* result);
    
    virtual HRESULT STDMETHODCALLTYPE elementIsPassword(
        /* [in] */ IDOMElement* element,
        /* [retval][out] */ BOOL* result);
    
    virtual HRESULT STDMETHODCALLTYPE formForElement(
        /* [in] */ IDOMElement* element,
        /* [retval][out] */ IDOMElement** form);
    
    virtual HRESULT STDMETHODCALLTYPE currentForm(
        /* [retval][out] */ IDOMElement** form);
    
    virtual HRESULT STDMETHODCALLTYPE controlsInForm(
        /* [in] */ IDOMElement* form,
        /* [out][in] */ IDOMElement** controls,
        /* [out][in] */ int* cControls);
    
    /* Deprecated. Use the variant that includes resultDistance and resultIsInCellAbove instead. */
    virtual HRESULT STDMETHODCALLTYPE deprecatedSearchForLabels(
        /* [size_is][in] */ BSTR *labels,
        /* [in] */ int cLabels,
        /* [in] */ IDOMElement *beforeElement,
        /* [retval][out] */ BSTR *result);
    
    virtual HRESULT STDMETHODCALLTYPE matchLabels(
        /* [size_is][in] */ BSTR *labels,
        /* [in] */ int cLabels,
        /* [in] */ IDOMElement *againstElement,
        /* [retval][out] */ BSTR *result);

    virtual HRESULT STDMETHODCALLTYPE searchForLabels(BSTR* labels, unsigned cLabels, IDOMElement* beforeElement, unsigned* resultDistance, BOOL* resultIsInCellAbove, BSTR* result);
    
    // IWebDocumentRepresentation
    virtual HRESULT STDMETHODCALLTYPE setDataSource(
        /* [in] */ IWebDataSource* dataSource);
    
    virtual HRESULT STDMETHODCALLTYPE receivedData(
        /* [in] */ IStream* data,
        /* [in] */ IWebDataSource* dataSource);
    
    virtual HRESULT STDMETHODCALLTYPE receivedError(
        /* [in] */ IWebError* error,
        /* [in] */ IWebDataSource* dataSource);
    
    virtual HRESULT STDMETHODCALLTYPE finishedLoadingWithDataSource(
        /* [in] */ IWebDataSource* dataSource);
    
    virtual HRESULT STDMETHODCALLTYPE canProvideDocumentSource(
        /* [retval][out] */ BOOL* result);
    
    virtual HRESULT STDMETHODCALLTYPE documentSource(
        /* [retval][out] */ BSTR* source);
    
    virtual HRESULT STDMETHODCALLTYPE title(
        /* [retval][out] */ BSTR* docTitle);

protected:
    ULONG m_refCount;
    WebFrame* m_frame;
};

#endif
