

#ifndef WebAccessibilityRole_h
#define WebAccessibilityRole_h

#include "WebCommon.h"

namespace WebKit {

// These values must match WebCore::AccessibilityRole values
enum WebAccessibilityRole {
    WebAccessibilityRoleUnknown = 1,
    WebAccessibilityRoleButton,
    WebAccessibilityRoleRadioButton,
    WebAccessibilityRoleCheckBox,
    WebAccessibilityRoleSlider,
    WebAccessibilityRoleTabGroup,
    WebAccessibilityRoleTextField,
    WebAccessibilityRoleStaticText,
    WebAccessibilityRoleTextArea,
    WebAccessibilityRoleScrollArea,
    WebAccessibilityRolePopUpButton,
    WebAccessibilityRoleMenuButton,
    WebAccessibilityRoleTable,
    WebAccessibilityRoleApplication,
    WebAccessibilityRoleGroup,
    WebAccessibilityRoleRadioGroup,
    WebAccessibilityRoleList,
    WebAccessibilityRoleScrollBar,
    WebAccessibilityRoleValueIndicator,
    WebAccessibilityRoleImage,
    WebAccessibilityRoleMenuBar,
    WebAccessibilityRoleMenu,
    WebAccessibilityRoleMenuItem,
    WebAccessibilityRoleColumn,
    WebAccessibilityRoleRow,
    WebAccessibilityRoleToolbar,
    WebAccessibilityRoleBusyIndicator,
    WebAccessibilityRoleProgressIndicator,
    WebAccessibilityRoleWindow,
    WebAccessibilityRoleDrawer,
    WebAccessibilityRoleSystemWide,
    WebAccessibilityRoleOutline,
    WebAccessibilityRoleIncrementor,
    WebAccessibilityRoleBrowser,
    WebAccessibilityRoleComboBox,
    WebAccessibilityRoleSplitGroup,
    WebAccessibilityRoleSplitter,
    WebAccessibilityRoleColorWell,
    WebAccessibilityRoleGrowArea,
    WebAccessibilityRoleSheet,
    WebAccessibilityRoleHelpTag,
    WebAccessibilityRoleMatte,
    WebAccessibilityRoleRuler,
    WebAccessibilityRoleRulerMarker,
    WebAccessibilityRoleLink,
    WebAccessibilityRoleDisclosureTriangle,
    WebAccessibilityRoleGrid,
    WebAccessibilityRoleCell,
    WebAccessibilityRoleColumnHeader,
    WebAccessibilityRoleRowHeader,

    WebAccessibilityRoleWebCoreLink,
    WebAccessibilityRoleImageMapLink,
    WebAccessibilityRoleImageMap,
    WebAccessibilityRoleListMarker,
    WebAccessibilityRoleWebArea,
    WebAccessibilityRoleHeading,
    WebAccessibilityRoleListBox,
    WebAccessibilityRoleListBoxOption,
    WebAccessibilityRoleTableHeaderContainer,
    WebAccessibilityRoleDefinitionListTerm,
    WebAccessibilityRoleDefinitionListDefinition,
    WebAccessibilityRoleAnnotation,
    WebAccessibilityRoleSliderThumb,
    WebAccessibilityRoleIgnored,
    WebAccessibilityRoleTab,
    WebAccessibilityRoleTabList,
    WebAccessibilityRoleTabPanel,
    WebAccessibilityRoleTreeRole,
    WebAccessibilityRoleTreeGrid,
    WebAccessibilityRoleTreeItemRole,
    WebAccessibilityRoleDirectory,
    WebAccessibilityRoleEditableText,

    WebAccessibilityRoleListItem,
    WebAccessibilityRoleMenuListPopup,
    WebAccessibilityRoleMenuListOption,

    WebAccessibilityRoleLandmarkApplication,
    WebAccessibilityRoleLandmarkBanner,
    WebAccessibilityRoleLandmarkComplementary,
    WebAccessibilityRoleLandmarkContentInfo,
    WebAccessibilityRoleLandmarkMain,
    WebAccessibilityRoleLandmarkNavigation,
    WebAccessibilityRoleLandmarkSearch,

    WebAccessibilityRoleApplicationAlert,
    WebAccessibilityRoleApplicationAlertDialog,
    WebAccessibilityRoleApplicationDialog,
    WebAccessibilityRoleApplicationLog,
    WebAccessibilityRoleApplicationMarquee,
    WebAccessibilityRoleApplicationStatus,
    WebAccessibilityRoleApplicationTimer,

    WebAccessibilityRoleDocument,
    WebAccessibilityRoleDocumentArticle,
    WebAccessibilityRoleDocumentMath,
    WebAccessibilityRoleDocumentNote,
    WebAccessibilityRoleDocumentRegion,

    WebAccessibilityRoleUserInterfaceTooltip
};

} // namespace WebKit

#endif
