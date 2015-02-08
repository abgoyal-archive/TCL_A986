

#include "config.h"
#include "WebKitDLL.h"
#include "MemoryStream.h"

using std::min;
using namespace WebCore;

// MemoryStream ---------------------------------------------------------------

MemoryStream::MemoryStream(PassRefPtr<SharedBuffer> buffer)
: m_refCount(0)
, m_buffer(buffer)
, m_pos(0)
{
    gClassCount++;
    gClassNameCount.add("MemoryStream");
}

MemoryStream::~MemoryStream()
{
    gClassCount--;
    gClassNameCount.remove("MemoryStream");
}

COMPtr<MemoryStream> MemoryStream::createInstance(PassRefPtr<SharedBuffer> buffer)
{
    return new MemoryStream(buffer);
}

// IUnknown -------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE MemoryStream::QueryInterface(REFIID riid, void** ppvObject)
{
    *ppvObject = 0;
    if (IsEqualGUID(riid, IID_IUnknown))
        *ppvObject = static_cast<IUnknown*>(this);
    else if (IsEqualGUID(riid, IID_ISequentialStream))
        *ppvObject = static_cast<ISequentialStream*>(this);
    else if (IsEqualGUID(riid, IID_IStream))
        *ppvObject = static_cast<IStream*>(this);
    else
        return E_NOINTERFACE;

    AddRef();
    return S_OK;
}

ULONG STDMETHODCALLTYPE MemoryStream::AddRef(void)
{
    return ++m_refCount;
}

ULONG STDMETHODCALLTYPE MemoryStream::Release(void)
{
    ULONG newRef = --m_refCount;
    if (!newRef)
        delete(this);

    return newRef;
}

// ISequentialStream ----------------------------------------------------------

HRESULT STDMETHODCALLTYPE MemoryStream::Read( 
    /* [length_is][size_is][out] */ void* pv,
    /* [in] */ ULONG cb,
    /* [out] */ ULONG* pcbRead)
{
    *pcbRead = 0;

    if (!m_buffer)
        return E_FAIL;

    if (m_pos >= m_buffer->size())
        return S_FALSE;

    if (m_pos + cb < m_buffer->size())
        *pcbRead = cb;
    else
        *pcbRead = (ULONG) (m_buffer->size() - m_pos);

    memcpy(pv, m_buffer->data() + m_pos, *pcbRead);
    m_pos += *pcbRead;

    return S_OK;
}

HRESULT STDMETHODCALLTYPE MemoryStream::Write( 
    /* [size_is][in] */ const void* /*pv*/,
    /* [in] */ ULONG /*cb*/,
    /* [out] */ ULONG* /*pcbWritten*/)
{
    // we use this for read-only streams
    return STG_E_ACCESSDENIED;
}

// IStream --------------------------------------------------------------------

HRESULT STDMETHODCALLTYPE MemoryStream::Seek( 
    /* [in] */ LARGE_INTEGER dlibMove,
    /* [in] */ DWORD dwOrigin,
    /* [out] */ ULARGE_INTEGER* plibNewPosition)
{
    if (!m_buffer)
        return E_FAIL;

    size_t proposedPos = m_pos;
    size_t lowPartNeg;
    switch (dwOrigin) {
        case STREAM_SEEK_SET:
            proposedPos = dlibMove.LowPart;
            if (dlibMove.HighPart)
                return E_FAIL;
            break;
        case STREAM_SEEK_CUR:
            if (!dlibMove.HighPart) {
                if (dlibMove.LowPart > (m_buffer->size() - m_pos))
                    return E_FAIL;
                proposedPos += dlibMove.LowPart;
            } else if (dlibMove.HighPart == -1) {
                lowPartNeg = (~dlibMove.LowPart)+1;
                if (lowPartNeg > m_pos)
                    return E_FAIL;
                proposedPos = m_pos - lowPartNeg;
            } else
                return E_FAIL;
            break;
        case STREAM_SEEK_END:
            if (dlibMove.HighPart != -1)
                return E_FAIL;
            lowPartNeg = (~dlibMove.LowPart)+1;
            if (lowPartNeg > m_buffer->size())
                return E_FAIL;
            proposedPos = m_buffer->size() - lowPartNeg;
            break;
        default:
            return E_FAIL;
    }

    if (proposedPos >= m_buffer->size())
        return E_FAIL;

    if (plibNewPosition) {
        plibNewPosition->HighPart = 0;
        plibNewPosition->LowPart = (DWORD) m_pos;
    }

    return S_OK;
}

HRESULT STDMETHODCALLTYPE MemoryStream::SetSize( 
    /* [in] */ ULARGE_INTEGER /*libNewSize*/)
{
    return STG_E_INVALIDFUNCTION;
}

HRESULT STDMETHODCALLTYPE MemoryStream::CopyTo( 
    /* [unique][in] */ IStream* pstm,
    /* [in] */ ULARGE_INTEGER cb,
    /* [out] */ ULARGE_INTEGER* pcbRead,
    /* [out] */ ULARGE_INTEGER* pcbWritten)
{
    if (!m_buffer)
        return E_FAIL;
    if (cb.HighPart) {
        cb.HighPart = 0;
        cb.LowPart = (DWORD)-1;
    }

    ULONG written;
    ULONG read = min(cb.LowPart, (ULONG)(m_buffer->size()-m_pos));
    HRESULT hr = pstm->Write(m_buffer->data()+m_pos, read, &written);
    if (pcbWritten) {
        pcbWritten->HighPart = 0;
        pcbWritten->LowPart = written;
    }
    if (pcbRead) {
        pcbRead->HighPart = 0;
        pcbRead->LowPart = read;
    }

    return hr;
}

HRESULT STDMETHODCALLTYPE MemoryStream::Commit( 
    /* [in] */ DWORD /*grfCommitFlags*/)
{
    return S_OK;
}

HRESULT STDMETHODCALLTYPE MemoryStream::Revert( void)
{
    return S_OK;
}

HRESULT STDMETHODCALLTYPE MemoryStream::LockRegion( 
    /* [in] */ ULARGE_INTEGER /*libOffset*/,
    /* [in] */ ULARGE_INTEGER /*cb*/,
    /* [in] */ DWORD /*dwLockType*/)
{
    return STG_E_INVALIDFUNCTION;
}

HRESULT STDMETHODCALLTYPE MemoryStream::UnlockRegion( 
    /* [in] */ ULARGE_INTEGER /*libOffset*/,
    /* [in] */ ULARGE_INTEGER /*cb*/,
    /* [in] */ DWORD /*dwLockType*/)
{
    return STG_E_INVALIDFUNCTION;
}

HRESULT STDMETHODCALLTYPE MemoryStream::Stat( 
    /* [out] */ STATSTG* pstatstg,
    /* [in] */ DWORD /*grfStatFlag*/)
{
    if (!pstatstg)
        return E_POINTER;

    if (!m_buffer)
        return E_FAIL;

    memset(pstatstg, 0, sizeof(STATSTG));
    pstatstg->type = STGTY_STREAM;
    pstatstg->cbSize.LowPart = (DWORD) m_buffer->size();
    return S_OK;
}

HRESULT STDMETHODCALLTYPE MemoryStream::Clone( 
    /* [out] */ IStream** ppstm)
{
    MemoryStream::createInstance(m_buffer).copyRefTo(ppstm);
    // FIXME: MSDN says we should be returning STG_E_INSUFFICIENT_MEMORY instead of E_OUTOFMEMORY here.
    return (*ppstm) ? S_OK : E_OUTOFMEMORY;
}
