

#ifndef UUID_H
#define UUID_H

#define UUID_LEN 16

int uuid_str2bin(const char *str, u8 *bin);
int uuid_bin2str(const u8 *bin, char *str, size_t max_len);
int is_nil_uuid(const u8 *uuid);
void uuid_gen_mac_addr(const u8 *mac_addr, u8 *uuid);

#endif /* UUID_H */
