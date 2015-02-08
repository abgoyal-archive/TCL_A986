
#ifndef __SOUND_PRODIGY_HIFI_H
#define __SOUND_PRODIGY_HIFI_H


#define PRODIGY_HIFI_DEVICE_DESC 	       "{Audiotrak,Prodigy 7.1 HIFI},"\
                                           "{Audiotrak Prodigy HD2},"\
                                           "{Hercules Fortissimo IV},"

#define VT1724_SUBDEVICE_PRODIGY_HIFI	0x38315441	/* PRODIGY 7.1 HIFI */
#define VT1724_SUBDEVICE_PRODIGY_HD2	0x37315441	/* PRODIGY HD2 */
#define VT1724_SUBDEVICE_FORTISSIMO4	0x81160100	/* Fortissimo IV */


extern struct snd_ice1712_card_info  snd_vt1724_prodigy_hifi_cards[];

#endif /* __SOUND_PRODIGY_HIFI_H */
