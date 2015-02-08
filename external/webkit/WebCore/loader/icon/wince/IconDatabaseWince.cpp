

#include "config.h"
#include "IconDatabase.h"

#include "AutodrainedPool.h"
#include "CString.h"
#include "DocumentLoader.h"
#include "FileSystem.h"
#include "IconDatabaseClient.h"
#include "IconRecord.h"
#include "Image.h"

namespace WebCore {

// Function to obtain the global icon database.
IconDatabase* iconDatabase() { return 0; }

IconDatabase::IconDatabase() {}
IconDatabase::~IconDatabase() {}

void IconDatabase::setClient(IconDatabaseClient*) {}

bool IconDatabase::open(const String& path) { return false; }
void IconDatabase::close() {}

void IconDatabase::removeAllIcons() {}

Image* IconDatabase::iconForPageURL(const String&, const IntSize&) { return 0; }
void IconDatabase::readIconForPageURLFromDisk(const String&) {}
String IconDatabase::iconURLForPageURL(const String&) { return String(); }
Image* IconDatabase::defaultIcon(const IntSize&) { return 0;}

void IconDatabase::retainIconForPageURL(const String&) {}
void IconDatabase::releaseIconForPageURL(const String&) {}

void IconDatabase::setIconDataForIconURL(PassRefPtr<SharedBuffer> data, const String&) {}
void IconDatabase::setIconURLForPageURL(const String& iconURL, const String& pageURL) {}

IconLoadDecision IconDatabase::loadDecisionForIconURL(const String&, DocumentLoader*) { return IconLoadNo; }
bool IconDatabase::iconDataKnownForIconURL(const String&) { return false; }

void IconDatabase::setEnabled(bool enabled) {}
bool IconDatabase::isEnabled() const { return false; }

void IconDatabase::setPrivateBrowsingEnabled(bool flag) {}
bool IconDatabase::isPrivateBrowsingEnabled() const { return false; }

void IconDatabase::delayDatabaseCleanup() {}
void IconDatabase::allowDatabaseCleanup() {}
void IconDatabase::checkIntegrityBeforeOpening() {}

// Support for WebCoreStatistics in WebKit
size_t IconDatabase::pageURLMappingCount() { return 0; }
size_t IconDatabase::retainedPageURLCount() {return 0; }
size_t IconDatabase::iconRecordCount() { return 0; }
size_t IconDatabase::iconRecordCountWithData() { return 0; }

bool IconDatabase::isOpen() const { return false; }
String IconDatabase::databasePath() const { return String(); }
String IconDatabase::defaultDatabaseFilename() { return String(); }

} // namespace WebCore
