

#ifndef GenericBinding_h
#define GenericBinding_h

namespace WebCore {

// Used to instantiate binding templates for any methods shared among all
// language bindings.
class GenericBinding {};

// Class to represent execution state for each language binding.
template <class T>
class State {};

// Common notion of execution state for language bindings.
template <>
class State<GenericBinding> {
    // Any methods shared across bindings can go here.
};

}

#endif // GenericBinding_h
