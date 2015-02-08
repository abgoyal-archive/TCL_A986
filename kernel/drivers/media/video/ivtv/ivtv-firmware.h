

#ifndef IVTV_FIRMWARE_H
#define IVTV_FIRMWARE_H

int ivtv_firmware_init(struct ivtv *itv);
void ivtv_firmware_versions(struct ivtv *itv);
void ivtv_halt_firmware(struct ivtv *itv);
void ivtv_init_mpeg_decoder(struct ivtv *itv);

#endif
