

#include "config.h"
#include "qwebsettings.h"

#include "qwebpage.h"
#include "qwebpage_p.h"
#include "qwebplugindatabase_p.h"

#include "Cache.h"
#include "CrossOriginPreflightResultCache.h"
#include "FontCache.h"
#include "Page.h"
#include "PageCache.h"
#include "Settings.h"
#include "KURL.h"
#include "PlatformString.h"
#include "IconDatabase.h"
#include "Image.h"
#include "IntSize.h"
#include "ApplicationCacheStorage.h"
#include "DatabaseTracker.h"
#include "FileSystem.h"

#include <QApplication>
#include <QDesktopServices>
#include <QDir>
#include <QHash>
#include <QSharedData>
#include <QUrl>
#include <QFileInfo>

#if ENABLE(QT_BEARER)
#include "NetworkStateNotifier.h"
#endif

void QWEBKIT_EXPORT qt_networkAccessAllowed(bool isAllowed)
{
#if ENABLE(QT_BEARER)
    WebCore::networkStateNotifier().setNetworkAccessAllowed(isAllowed);
#endif
}

class QWebSettingsPrivate {
public:
    QWebSettingsPrivate(WebCore::Settings* wcSettings = 0)
        : settings(wcSettings)
    {
    }

    QHash<int, QString> fontFamilies;
    QHash<int, int> fontSizes;
    QHash<int, bool> attributes;
    QUrl userStyleSheetLocation;
    QString defaultTextEncoding;
    QString localStoragePath;
    QString offlineWebApplicationCachePath;
    qint64 offlineStorageDefaultQuota;

    void apply();
    WebCore::Settings* settings;
};

typedef QHash<int, QPixmap> WebGraphicHash;
Q_GLOBAL_STATIC(WebGraphicHash, _graphics)

static WebGraphicHash* graphics()
{
    WebGraphicHash* hash = _graphics();

    if (hash->isEmpty()) {
        hash->insert(QWebSettings::MissingImageGraphic, QPixmap(QLatin1String(":webkit/resources/missingImage.png")));
        hash->insert(QWebSettings::MissingPluginGraphic, QPixmap(QLatin1String(":webkit/resources/nullPlugin.png")));
        hash->insert(QWebSettings::DefaultFrameIconGraphic, QPixmap(QLatin1String(":webkit/resources/urlIcon.png")));
        hash->insert(QWebSettings::TextAreaSizeGripCornerGraphic, QPixmap(QLatin1String(":webkit/resources/textAreaResizeCorner.png")));
        hash->insert(QWebSettings::DeleteButtonGraphic, QPixmap(QLatin1String(":webkit/resources/deleteButton.png")));
    }

    return hash;
}

Q_GLOBAL_STATIC(QList<QWebSettingsPrivate*>, allSettings);

void QWebSettingsPrivate::apply()
{
    if (settings) {
        settings->setTextAreasAreResizable(true);

        QWebSettingsPrivate* global = QWebSettings::globalSettings()->d;

        QString family = fontFamilies.value(QWebSettings::StandardFont,
                                            global->fontFamilies.value(QWebSettings::StandardFont));
        settings->setStandardFontFamily(family);

        family = fontFamilies.value(QWebSettings::FixedFont,
                                    global->fontFamilies.value(QWebSettings::FixedFont));
        settings->setFixedFontFamily(family);

        family = fontFamilies.value(QWebSettings::SerifFont,
                                    global->fontFamilies.value(QWebSettings::SerifFont));
        settings->setSerifFontFamily(family);

        family = fontFamilies.value(QWebSettings::SansSerifFont,
                                    global->fontFamilies.value(QWebSettings::SansSerifFont));
        settings->setSansSerifFontFamily(family);

        family = fontFamilies.value(QWebSettings::CursiveFont,
                                    global->fontFamilies.value(QWebSettings::CursiveFont));
        settings->setCursiveFontFamily(family);

        family = fontFamilies.value(QWebSettings::FantasyFont,
                                    global->fontFamilies.value(QWebSettings::FantasyFont));
        settings->setFantasyFontFamily(family);

        int size = fontSizes.value(QWebSettings::MinimumFontSize,
                                   global->fontSizes.value(QWebSettings::MinimumFontSize));
        settings->setMinimumFontSize(size);

        size = fontSizes.value(QWebSettings::MinimumLogicalFontSize,
                                   global->fontSizes.value(QWebSettings::MinimumLogicalFontSize));
        settings->setMinimumLogicalFontSize(size);

        size = fontSizes.value(QWebSettings::DefaultFontSize,
                                   global->fontSizes.value(QWebSettings::DefaultFontSize));
        settings->setDefaultFontSize(size);

        size = fontSizes.value(QWebSettings::DefaultFixedFontSize,
                                   global->fontSizes.value(QWebSettings::DefaultFixedFontSize));
        settings->setDefaultFixedFontSize(size);

        bool value = attributes.value(QWebSettings::AutoLoadImages,
                                      global->attributes.value(QWebSettings::AutoLoadImages));
        settings->setLoadsImagesAutomatically(value);

        value = attributes.value(QWebSettings::JavascriptEnabled,
                                      global->attributes.value(QWebSettings::JavascriptEnabled));
        settings->setJavaScriptEnabled(value);
#if USE(ACCELERATED_COMPOSITING)
        value = attributes.value(QWebSettings::AcceleratedCompositingEnabled,
                                      global->attributes.value(QWebSettings::AcceleratedCompositingEnabled));

        settings->setAcceleratedCompositingEnabled(value);
#endif
        value = attributes.value(QWebSettings::JavascriptCanOpenWindows,
                                      global->attributes.value(QWebSettings::JavascriptCanOpenWindows));
        settings->setJavaScriptCanOpenWindowsAutomatically(value);

        value = attributes.value(QWebSettings::JavaEnabled,
                                      global->attributes.value(QWebSettings::JavaEnabled));
        settings->setJavaEnabled(value);

        value = attributes.value(QWebSettings::PluginsEnabled,
                                      global->attributes.value(QWebSettings::PluginsEnabled));
        settings->setPluginsEnabled(value);

        value = attributes.value(QWebSettings::PrivateBrowsingEnabled,
                                      global->attributes.value(QWebSettings::PrivateBrowsingEnabled));
        settings->setPrivateBrowsingEnabled(value);

        value = attributes.value(QWebSettings::JavascriptCanAccessClipboard,
                                      global->attributes.value(QWebSettings::JavascriptCanAccessClipboard));
        settings->setDOMPasteAllowed(value);

        value = attributes.value(QWebSettings::DeveloperExtrasEnabled,
                                      global->attributes.value(QWebSettings::DeveloperExtrasEnabled));
        settings->setDeveloperExtrasEnabled(value);

        QUrl location = !userStyleSheetLocation.isEmpty() ? userStyleSheetLocation : global->userStyleSheetLocation;
        settings->setUserStyleSheetLocation(WebCore::KURL(location));

        QString encoding = !defaultTextEncoding.isEmpty() ? defaultTextEncoding: global->defaultTextEncoding;
        settings->setDefaultTextEncodingName(encoding);

        QString storagePath = !localStoragePath.isEmpty() ? localStoragePath : global->localStoragePath;
        settings->setLocalStorageDatabasePath(storagePath);

        value = attributes.value(QWebSettings::ZoomTextOnly,
                                 global->attributes.value(QWebSettings::ZoomTextOnly));
        settings->setZoomsTextOnly(value);

        value = attributes.value(QWebSettings::PrintElementBackgrounds,
                                      global->attributes.value(QWebSettings::PrintElementBackgrounds));
        settings->setShouldPrintBackgrounds(value);

        value = attributes.value(QWebSettings::OfflineStorageDatabaseEnabled,
                                      global->attributes.value(QWebSettings::OfflineStorageDatabaseEnabled));
        settings->setDatabasesEnabled(value);

        value = attributes.value(QWebSettings::OfflineWebApplicationCacheEnabled,
                                      global->attributes.value(QWebSettings::OfflineWebApplicationCacheEnabled));
        settings->setOfflineWebApplicationCacheEnabled(value);

        value = attributes.value(QWebSettings::LocalStorageEnabled,
                                      global->attributes.value(QWebSettings::LocalStorageEnabled));
        settings->setLocalStorageEnabled(value);

        value = attributes.value(QWebSettings::LocalContentCanAccessRemoteUrls,
                                      global->attributes.value(QWebSettings::LocalContentCanAccessRemoteUrls));
        settings->setAllowUniversalAccessFromFileURLs(value);

        value = attributes.value(QWebSettings::XSSAuditorEnabled,
                                      global->attributes.value(QWebSettings::XSSAuditorEnabled));
        settings->setXSSAuditorEnabled(value);

        settings->setUsesPageCache(WebCore::pageCache()->capacity());
    } else {
        QList<QWebSettingsPrivate*> settings = *::allSettings();
        for (int i = 0; i < settings.count(); ++i)
            settings[i]->apply();
    }
}

QWebSettings* QWebSettings::globalSettings()
{
    static QWebSettings* global = 0;
    if (!global)
        global = new QWebSettings;
    return global;
}






QWebSettings::QWebSettings()
    : d(new QWebSettingsPrivate)
{
    // Initialize our global defaults
    d->fontSizes.insert(QWebSettings::MinimumFontSize, 0);
    d->fontSizes.insert(QWebSettings::MinimumLogicalFontSize, 0);
    d->fontSizes.insert(QWebSettings::DefaultFontSize, 16);
    d->fontSizes.insert(QWebSettings::DefaultFixedFontSize, 13);
    d->fontFamilies.insert(QWebSettings::StandardFont, QLatin1String("Arial"));
    d->fontFamilies.insert(QWebSettings::FixedFont, QLatin1String("Courier New"));
    d->fontFamilies.insert(QWebSettings::SerifFont, QLatin1String("Times New Roman"));
    d->fontFamilies.insert(QWebSettings::SansSerifFont, QLatin1String("Arial"));
    d->fontFamilies.insert(QWebSettings::CursiveFont, QLatin1String("Arial"));
    d->fontFamilies.insert(QWebSettings::FantasyFont, QLatin1String("Arial"));

    d->attributes.insert(QWebSettings::AutoLoadImages, true);
    d->attributes.insert(QWebSettings::DnsPrefetchEnabled, false);
    d->attributes.insert(QWebSettings::JavascriptEnabled, true);
    d->attributes.insert(QWebSettings::LinksIncludedInFocusChain, true);
    d->attributes.insert(QWebSettings::ZoomTextOnly, false);
    d->attributes.insert(QWebSettings::PrintElementBackgrounds, true);
    d->attributes.insert(QWebSettings::OfflineStorageDatabaseEnabled, false);
    d->attributes.insert(QWebSettings::OfflineWebApplicationCacheEnabled, false);
    d->attributes.insert(QWebSettings::LocalStorageEnabled, false);
    d->attributes.insert(QWebSettings::LocalContentCanAccessRemoteUrls, false);
    d->attributes.insert(QWebSettings::AcceleratedCompositingEnabled, false);
    d->offlineStorageDefaultQuota = 5 * 1024 * 1024;
    d->defaultTextEncoding = QLatin1String("iso-8859-1");
}

QWebSettings::QWebSettings(WebCore::Settings* settings)
    : d(new QWebSettingsPrivate(settings))
{
    d->settings = settings;
    d->apply();
    allSettings()->append(d);
}

QWebSettings::~QWebSettings()
{
    if (d->settings)
        allSettings()->removeAll(d);

    delete d;
}

void QWebSettings::setFontSize(FontSize type, int size)
{
    d->fontSizes.insert(type, size);
    d->apply();
}

int QWebSettings::fontSize(FontSize type) const
{
    int defaultValue = 0;
    if (d->settings) {
        QWebSettingsPrivate* global = QWebSettings::globalSettings()->d;
        defaultValue = global->fontSizes.value(type);
    }
    return d->fontSizes.value(type, defaultValue);
}

void QWebSettings::resetFontSize(FontSize type)
{
    if (d->settings) {
        d->fontSizes.remove(type);
        d->apply();
    }
}

void QWebSettings::setUserStyleSheetUrl(const QUrl& location)
{
    d->userStyleSheetLocation = location;
    d->apply();
}

QUrl QWebSettings::userStyleSheetUrl() const
{
    return d->userStyleSheetLocation;
}

void QWebSettings::setDefaultTextEncoding(const QString& encoding)
{
    d->defaultTextEncoding = encoding;
    d->apply();
}

QString QWebSettings::defaultTextEncoding() const
{
    return d->defaultTextEncoding;
}

void QWebSettings::setIconDatabasePath(const QString& path)
{
    WebCore::iconDatabase()->delayDatabaseCleanup();

    if (!path.isEmpty()) {
        WebCore::iconDatabase()->setEnabled(true);
        QFileInfo info(path);
        if (info.isDir() && info.isWritable())
            WebCore::iconDatabase()->open(path);
    } else {
        WebCore::iconDatabase()->setEnabled(false);
        WebCore::iconDatabase()->close();
    }
}

QString QWebSettings::iconDatabasePath()
{
    if (WebCore::iconDatabase()->isEnabled() && WebCore::iconDatabase()->isOpen())
        return WebCore::iconDatabase()->databasePath();
    else
        return QString();
}

void QWebSettings::clearIconDatabase()
{
    if (WebCore::iconDatabase()->isEnabled() && WebCore::iconDatabase()->isOpen())
        WebCore::iconDatabase()->removeAllIcons();
}

QIcon QWebSettings::iconForUrl(const QUrl& url)
{
    WebCore::Image* image = WebCore::iconDatabase()->iconForPageURL(WebCore::KURL(url).string(),
                                WebCore::IntSize(16, 16));
    if (!image)
        return QPixmap();

    QPixmap* icon = image->nativeImageForCurrentFrame();
    if (!icon)
        return QPixmap();

    return* icon;
}


void QWebSettings::setWebGraphic(WebGraphic type, const QPixmap& graphic)
{
    WebGraphicHash* h = graphics();
    if (graphic.isNull())
        h->remove(type);
    else
        h->insert(type, graphic);
}

QPixmap QWebSettings::webGraphic(WebGraphic type)
{
    return graphics()->value(type);
}

void QWebSettings::clearMemoryCaches()
{
    // Turn the cache on and off.  Disabling the object cache will remove all
    // resources from the cache.  They may still live on if they are referenced
    // by some Web page though.
    if (!WebCore::cache()->disabled()) {
        WebCore::cache()->setDisabled(true);
        WebCore::cache()->setDisabled(false);
    }

    int pageCapacity = WebCore::pageCache()->capacity();
    // Setting size to 0, makes all pages be released.
    WebCore::pageCache()->setCapacity(0);
    WebCore::pageCache()->releaseAutoreleasedPagesNow();
    WebCore::pageCache()->setCapacity(pageCapacity);

    // Invalidating the font cache and freeing all inactive font data.
    WebCore::fontCache()->invalidate();

    // Empty the Cross-Origin Preflight cache
    WebCore::CrossOriginPreflightResultCache::shared().empty();
}

void QWebSettings::setMaximumPagesInCache(int pages)
{
    QWebSettingsPrivate* global = QWebSettings::globalSettings()->d;
    WebCore::pageCache()->setCapacity(qMax(0, pages));
    global->apply();
}

int QWebSettings::maximumPagesInCache()
{
    return WebCore::pageCache()->capacity();
}

void QWebSettings::setObjectCacheCapacities(int cacheMinDeadCapacity, int cacheMaxDead, int totalCapacity)
{
    bool disableCache = !cacheMinDeadCapacity && !cacheMaxDead && !totalCapacity;
    WebCore::cache()->setDisabled(disableCache);

    WebCore::cache()->setCapacities(qMax(0, cacheMinDeadCapacity),
                                    qMax(0, cacheMaxDead),
                                    qMax(0, totalCapacity));
}

void QWebSettings::setFontFamily(FontFamily which, const QString& family)
{
    d->fontFamilies.insert(which, family);
    d->apply();
}

QString QWebSettings::fontFamily(FontFamily which) const
{
    QString defaultValue;
    if (d->settings) {
        QWebSettingsPrivate* global = QWebSettings::globalSettings()->d;
        defaultValue = global->fontFamilies.value(which);
    }
    return d->fontFamilies.value(which, defaultValue);
}

void QWebSettings::resetFontFamily(FontFamily which)
{
    if (d->settings) {
        d->fontFamilies.remove(which);
        d->apply();
    }
}

void QWebSettings::setAttribute(WebAttribute attr, bool on)
{
    d->attributes.insert(attr, on);
    d->apply();
}

bool QWebSettings::testAttribute(WebAttribute attr) const
{
    bool defaultValue = false;
    if (d->settings) {
        QWebSettingsPrivate* global = QWebSettings::globalSettings()->d;
        defaultValue = global->attributes.value(attr);
    }
    return d->attributes.value(attr, defaultValue);
}

void QWebSettings::resetAttribute(WebAttribute attr)
{
    if (d->settings) {
        d->attributes.remove(attr);
        d->apply();
    }
}

void QWebSettings::setOfflineStoragePath(const QString& path)
{
#if ENABLE(DATABASE)
    WebCore::DatabaseTracker::tracker().setDatabaseDirectoryPath(path);
#endif
}

QString QWebSettings::offlineStoragePath()
{
#if ENABLE(DATABASE)
    return WebCore::DatabaseTracker::tracker().databaseDirectoryPath();
#else
    return QString();
#endif
}

void QWebSettings::setOfflineStorageDefaultQuota(qint64 maximumSize)
{
    QWebSettings::globalSettings()->d->offlineStorageDefaultQuota = maximumSize;
}

qint64 QWebSettings::offlineStorageDefaultQuota()
{
    return QWebSettings::globalSettings()->d->offlineStorageDefaultQuota;
}

void QWebSettings::setOfflineWebApplicationCachePath(const QString& path)
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    WebCore::cacheStorage().setCacheDirectory(path);
#endif
}

QString QWebSettings::offlineWebApplicationCachePath()
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    return WebCore::cacheStorage().cacheDirectory();
#else
    return QString();
#endif
}

void QWebSettings::setOfflineWebApplicationCacheQuota(qint64 maximumSize)
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    WebCore::cacheStorage().setMaximumSize(maximumSize);
#endif
}

qint64 QWebSettings::offlineWebApplicationCacheQuota()
{
#if ENABLE(OFFLINE_WEB_APPLICATIONS)
    return WebCore::cacheStorage().maximumSize();
#else
    return 0;
#endif
}

void QWebSettings::setLocalStoragePath(const QString& path)
{
    d->localStoragePath = path;
    d->apply();
}

QString QWebSettings::localStoragePath() const
{
    return d->localStoragePath;
}

void QWebSettings::enablePersistentStorage(const QString& path)
{
    QString storagePath;

    if (path.isEmpty()) {
        storagePath = QDesktopServices::storageLocation(QDesktopServices::DataLocation);

        if (storagePath.isEmpty())
            storagePath = WebCore::pathByAppendingComponent(QDir::homePath(), QCoreApplication::applicationName());
    } else
        storagePath = path;

    WebCore::makeAllDirectories(storagePath);

    QWebSettings::setIconDatabasePath(storagePath);
    QWebSettings::setOfflineWebApplicationCachePath(storagePath);
    QWebSettings::setOfflineStoragePath(WebCore::pathByAppendingComponent(storagePath, "Databases"));
    QWebSettings::globalSettings()->setLocalStoragePath(WebCore::pathByAppendingComponent(storagePath, "LocalStorage"));
    QWebSettings::globalSettings()->setAttribute(QWebSettings::LocalStorageEnabled, true);
    QWebSettings::globalSettings()->setAttribute(QWebSettings::OfflineStorageDatabaseEnabled, true);
    QWebSettings::globalSettings()->setAttribute(QWebSettings::OfflineWebApplicationCacheEnabled, true);
}

