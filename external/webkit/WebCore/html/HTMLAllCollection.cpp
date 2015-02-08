

#include "config.h"
#include "HTMLAllCollection.h"

#include "Node.h"

namespace WebCore {

PassRefPtr<HTMLAllCollection> HTMLAllCollection::create(PassRefPtr<Node> base)
{
    return adoptRef(new HTMLAllCollection(base));
}

HTMLAllCollection::HTMLAllCollection(PassRefPtr<Node> base)
    : HTMLCollection(base, DocAll)
{
}

HTMLAllCollection::~HTMLAllCollection()
{
}

} // namespace WebCore
