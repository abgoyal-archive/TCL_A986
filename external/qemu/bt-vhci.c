

#include "qemu-common.h"
#include "qemu-char.h"
#include "sysemu.h"
#include "net.h"
#include "hw/bt.h"

#define VHCI_DEV	"/dev/vhci"
#define VHCI_UDEV	"/dev/hci_vhci"

struct bt_vhci_s {
    int fd;
    struct HCIInfo *info;

    uint8_t hdr[4096];
    int len;
};

static void vhci_read(void *opaque)
{
    struct bt_vhci_s *s = (struct bt_vhci_s *) opaque;
    uint8_t *pkt;
    int pktlen;

    /* Seems that we can't read only the header first and then the amount
     * of data indicated in the header because Linux will discard everything
     * that's not been read in one go.  */
    s->len = read(s->fd, s->hdr, sizeof(s->hdr));

    if (s->len < 0) {
        fprintf(stderr, "qemu: error %i reading the PDU\n", errno);
        return;
    }

    pkt = s->hdr;
    while (s->len --)
        switch (*pkt ++) {
        case HCI_COMMAND_PKT:
            if (s->len < 3)
                goto bad_pkt;

            pktlen = MIN(pkt[2] + 3, s->len);
            s->info->cmd_send(s->info, pkt, pktlen);
            s->len -= pktlen;
            pkt += pktlen;
            break;

        case HCI_ACLDATA_PKT:
            if (s->len < 4)
                goto bad_pkt;

            pktlen = MIN(((pkt[3] << 8) | pkt[2]) + 4, s->len);
            s->info->acl_send(s->info, pkt, pktlen);
            s->len -= pktlen;
            pkt += pktlen;
            break;

        case HCI_SCODATA_PKT:
            if (s->len < 3)
                goto bad_pkt;

            pktlen = MIN(pkt[2] + 3, s->len);
            s->info->sco_send(s->info, pkt, pktlen);
            s->len -= pktlen;
            pkt += pktlen;
            break;

        default:
        bad_pkt:
            fprintf(stderr, "qemu: bad HCI packet type %02x\n", pkt[-1]);
        }
}

static void vhci_host_send(void *opaque,
                int type, const uint8_t *data, int len)
{
    struct bt_vhci_s *s = (struct bt_vhci_s *) opaque;
#if 0
    uint8_t pkt = type;
    struct iovec iv[2];

    iv[0].iov_base = &pkt;
    iv[0].iov_len  = 1;
    iv[1].iov_base = (void *) data;
    iv[1].iov_len  = len;

    while (writev(s->fd, iv, 2) < 0)
        if (errno != EAGAIN && errno != EINTR) {
            fprintf(stderr, "qemu: error %i writing bluetooth packet.\n",
                            errno);
            return;
        }
#else
    /* Apparently VHCI wants us to write everything in one chunk :-(  */
    static uint8_t buf[4096];

    buf[0] = type;
    memcpy(buf + 1, data, len);

    while (write(s->fd, buf, len + 1) < 0)
        if (errno != EAGAIN && errno != EINTR) {
            fprintf(stderr, "qemu: error %i writing bluetooth packet.\n",
                            errno);
            return;
        }
#endif
}

static void vhci_out_hci_packet_event(void *opaque,
                const uint8_t *data, int len)
{
    vhci_host_send(opaque, HCI_EVENT_PKT, data, len);
}

static void vhci_out_hci_packet_acl(void *opaque,
                const uint8_t *data, int len)
{
    vhci_host_send(opaque, HCI_ACLDATA_PKT, data, len);
}

void bt_vhci_init(struct HCIInfo *info)
{
    struct bt_vhci_s *s;
    int err[2];
    int fd;

    fd = open(VHCI_DEV, O_RDWR);
    err[0] = errno;
    if (fd < 0) {
        fd = open(VHCI_UDEV, O_RDWR);
        err[1] = errno;
    }

    if (fd < 0) {
        fprintf(stderr, "qemu: Can't open `%s': %s (%i)\n",
                        VHCI_DEV, strerror(err[0]), err[0]);
        fprintf(stderr, "qemu: Can't open `%s': %s (%i)\n",
                        VHCI_UDEV, strerror(err[1]), err[1]);
        exit(-1);
    }

    s = qemu_mallocz(sizeof(struct bt_vhci_s));
    s->fd = fd;
    s->info = info ?: qemu_next_hci();
    s->info->opaque = s;
    s->info->evt_recv = vhci_out_hci_packet_event;
    s->info->acl_recv = vhci_out_hci_packet_acl;

    qemu_set_fd_handler(s->fd, vhci_read, NULL, s);
}
