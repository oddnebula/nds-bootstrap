#ifndef TWL_WIFI_WPA_H
#define TWL_WIFI_WPA_H

#ifdef WPA_HOST_TEST
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#else
#include <nds/ndstypes.h>
#endif

#define WPA_SHA1_LEN 20

void wpaHmacSha1(u8* out, const u8* key, u32 keyLen, const u8* data, u32 dataLen);

// IEEE 802.11i PRF. Data is label, a zero byte, input, and a final counter byte.
void wpaPrf(u8* out, u32 outLen, const u8* key, u32 keyLen, u8* data, u32 dataLen);

// RFC 3394 AES-128 key unwrap. In is (blocks+1)*8 bytes, out is blocks*8 bytes.
bool wpaAesUnwrap(const u8* kek, const u8* in, u8* out, u32 blocks);

#endif // TWL_WIFI_WPA_H
