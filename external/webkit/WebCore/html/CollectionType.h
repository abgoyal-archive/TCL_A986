

#ifndef CollectionType_h
#define CollectionType_h

namespace WebCore {

enum CollectionType {
    // unnamed collection types cached in the document

    DocImages,    // all <img> elements in the document
    DocApplets,   // all <object> and <applet> elements
    DocEmbeds,    // all <embed> elements
    DocObjects,   // all <object> elements
    DocForms,     // all <form> elements
    DocLinks,     // all <a> _and_ <area> elements with a value for href
    DocAnchors,   // all <a> elements with a value for name
    DocScripts,   // all <script> elements

    DocAll,       // "all" elements (IE)
    NodeChildren, // first-level children (IE)

    // named collection types cached in the document

    WindowNamedItems,
    DocumentNamedItems,

    // types not cached in the document; these are types that can't be used on a document

    TableTBodies, // all <tbody> elements in this table
    TSectionRows, // all row elements in this table section
    TRCells,      // all cells in this row
    SelectOptions,
    DataListOptions,
    MapAreas,

    OtherCollection
};

static const CollectionType FirstUnnamedDocumentCachedType = DocImages;
static const unsigned NumUnnamedDocumentCachedTypes = NodeChildren - DocImages + 1;

static const CollectionType FirstNamedDocumentCachedType = WindowNamedItems;
static const unsigned NumNamedDocumentCachedTypes = DocumentNamedItems - WindowNamedItems + 1;

} // namespace

#endif
