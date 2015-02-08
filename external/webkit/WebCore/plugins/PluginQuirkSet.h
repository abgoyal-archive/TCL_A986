

#ifndef PluginQuirkSet_h
#define PluginQuirkSet_h


namespace WebCore {

    enum PluginQuirk {
        PluginQuirkWantsMozillaUserAgent = 1 << 0,
        PluginQuirkDeferFirstSetWindowCall = 1 << 1,
        PluginQuirkThrottleInvalidate = 1 << 2, 
        PluginQuirkRemoveWindowlessVideoParam = 1 << 3,
        PluginQuirkThrottleWMUserPlusOneMessages = 1 << 4,
        PluginQuirkDontUnloadPlugin = 1 << 5,
        PluginQuirkDontCallWndProcForSameMessageRecursively = 1 << 6,
        PluginQuirkHasModalMessageLoop = 1 << 7,
        PluginQuirkFlashURLNotifyBug = 1 << 8,
        PluginQuirkDontClipToZeroRectWhenScrolling = 1 << 9,
        PluginQuirkDontSetNullWindowHandleOnDestroy = 1 << 10,
        PluginQuirkDontAllowMultipleInstances = 1 << 11,
        PluginQuirkRequiresGtkToolKit = 1 << 12,
        PluginQuirkRequiresDefaultScreenDepth = 1 << 13
    };

    class PluginQuirkSet {
        public:
            PluginQuirkSet() : m_quirks(0) { }
            void add(PluginQuirk quirk) { m_quirks |= quirk; }
            bool contains(PluginQuirk quirk) const { return m_quirks & quirk; }
        private:
            unsigned m_quirks;
    };

} // namespace WebCore

#endif // PluginQuirkSet_h
