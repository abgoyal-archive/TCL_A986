

#include "qemu-common.h"
#include "qemu-queue.h"
#include "sysemu.h"
#include "hw/watchdog.h"

static QLIST_HEAD(watchdog_list, WatchdogTimerModel) watchdog_list;

void watchdog_add_model(WatchdogTimerModel *model)
{
    QLIST_INSERT_HEAD(&watchdog_list, model, entry);
}

int select_watchdog(const char *p)
{
    WatchdogTimerModel *model;

    if (watchdog) {
        fprintf(stderr,
                 "qemu: only one watchdog option may be given\n");
        return 1;
    }

    /* -watchdog ? lists available devices and exits cleanly. */
    if (strcmp(p, "?") == 0) {
        QLIST_FOREACH(model, &watchdog_list, entry) {
            fprintf(stderr, "\t%s\t%s\n",
                     model->wdt_name, model->wdt_description);
        }
        return 2;
    }

    QLIST_FOREACH(model, &watchdog_list, entry) {
        if (strcasecmp(model->wdt_name, p) == 0) {
            watchdog = model;
            return 0;
        }
    }

    fprintf(stderr, "Unknown -watchdog device. Supported devices are:\n");
    QLIST_FOREACH(model, &watchdog_list, entry) {
        fprintf(stderr, "\t%s\t%s\n",
                 model->wdt_name, model->wdt_description);
    }
    return 1;
}

int select_watchdog_action(const char *p)
{
    if (strcasecmp(p, "reset") == 0)
        watchdog_action = WDT_RESET;
    else if (strcasecmp(p, "shutdown") == 0)
        watchdog_action = WDT_SHUTDOWN;
    else if (strcasecmp(p, "poweroff") == 0)
        watchdog_action = WDT_POWEROFF;
    else if (strcasecmp(p, "pause") == 0)
        watchdog_action = WDT_PAUSE;
    else if (strcasecmp(p, "debug") == 0)
        watchdog_action = WDT_DEBUG;
    else if (strcasecmp(p, "none") == 0)
        watchdog_action = WDT_NONE;
    else
        return -1;

    return 0;
}

void watchdog_perform_action(void)
{
    switch(watchdog_action) {
    case WDT_RESET:             /* same as 'system_reset' in monitor */
        qemu_system_reset_request();
        break;

    case WDT_SHUTDOWN:          /* same as 'system_powerdown' in monitor */
        qemu_system_powerdown_request();
        break;

    case WDT_POWEROFF:          /* same as 'quit' command in monitor */
        exit(0);
        break;

    case WDT_PAUSE:             /* same as 'stop' command in monitor */
        vm_stop(0);
        break;

    case WDT_DEBUG:
        fprintf(stderr, "watchdog: timer fired\n");
        break;

    case WDT_NONE:
        break;
    }
}

void watchdog_pc_init(PCIBus *pci_bus)
{
    if (watchdog)
        watchdog->wdt_pc_init(pci_bus);
}

void register_watchdogs(void)
{
#if 0
    wdt_ib700_init();
    wdt_i6300esb_init();
#endif
}
