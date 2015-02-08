

#ifndef MemoryStream_H
#define MemoryStream_H

#include <objidl.h>

#include <WebCore/COMPtr.h>
#include <WebCore/SharedBuffer.h>
#include <WTF/PassRefPtr.h>
#include <WTF/RefPtr.h>

class MemoryStream : public IStream
{
public:
    static COMPtr<MemoryStream> createInstance(PassRefPtr<WebCore::SharedBuffer> buffer);

protected:
    MemoryStream(PassRefPtr<WebCore::SharedBuffer> buffer);
    ~MemoryStream();
public:

    // IUnknown
    virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppvObject);
    virtual ULONG STDMETHODCALLTYPE AddRef(void);
    virtual ULONG STDMETHODCALLTYPE Release(void);

    // ISequentialStream
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE Read( 
        /* [length_is][size_is][out] */ void* pv,
        /* [in] */ ULONG cb,
        /* [out] */ ULONG* pcbRead);
    
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE Write( 
        /* [size_is][in] */ const void* pv,
        /* [in] */ ULONG cb,
        /* [out] */ ULONG* pcbWritten);

    // IStream
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE Seek( 
        /* [in] */ LARGE_INTEGER dlibMove,
        /* [in] */ DWORD dwOrigin,
        /* [out] */ ULARGE_INTEGER* plibNewPosition);
    
    virtual HRESULT STDMETHODCALLTYPE SetSize( 
        /* [in] */ ULARGE_INTEGER libNewSize);
    
    virtual /* [local] */ HRESULT STDMETHODCALLTYPE CopyTo( 
        /* [unique][in] */ IStream* pstm,
        /* [in] */ ULARGE_INTEGER cb,
        /* [out] */ ULARGE_INTEGER* pcbRead,
        /* [out] */ ULARGE_INTEGER* pcbWritten);
    
    virtual HRESULT STDMETHODCALLTYPE Commit( 
        /* [in] */ DWORD grfCommitFlags);
    
    virtual HRESULT STDMETHODCALLTYPE Revert( void);
    
    virtual HRESULT STDMETHODCALLTYPE LockRegion( 
        /* [in] */ ULARGE_INTEGER libOffset,
        /* [in] */ ULARGE_INTEGER cb,
        /* [in] */ DWORD dwLockType);
    
    virtual HRESULT STDMETHODCALLTYPE UnlockRegion( 
        /* [in] */ ULARGE_INTEGER libOffset,
        /* [in] */ ULARGE_INTEGER cb,
        /* [in] */ DWORD dwLockType);
    
    virtual HRESULT STDMETHODCALLTYPE Stat( 
        /* [out] */ STATSTG* pstatstg,
        /* [in] */ DWORD grfStatFlag);
    
    virtual HRESULT STDMETHODCALLTYPE Clone( 
        /* [out] */ IStream** ppstm);

protected:
    ULONG m_refCount;
    RefPtr<WebCore::SharedBuffer> m_buffer;
    size_t m_pos;
};

#endif
