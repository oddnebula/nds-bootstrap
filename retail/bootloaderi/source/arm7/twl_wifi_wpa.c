/*
 * WPA2 crypto for DSi-mode WiFi: SHA-1, HMAC-SHA1, the IEEE 802.11i PRF and AES
 * key unwrap. Written for small size, not speed; only the handshake uses it.
 */

#include "twl_wifi_wpa.h"

#define SHA1_BLOCK 64

typedef struct {
	u32 h[5];
	u32 total;
	u8 block[SHA1_BLOCK];
	u32 fill;
} sha1Ctx;

static inline u32 rol(u32 x, int n) {
	return (x << n) | (x >> (32 - n));
}

static void sha1Block(sha1Ctx* ctx) {
	u32 w[16];
	for (int i = 0; i < 16; i++) {
		const u8* p = ctx->block + i*4;
		w[i] = ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
	}

	u32 a = ctx->h[0], b = ctx->h[1], c = ctx->h[2], d = ctx->h[3], e = ctx->h[4];
	for (int i = 0; i < 80; i++) {
		u32 word;
		if (i < 16) {
			word = w[i];
		} else {
			word = rol(w[(i+13)&15] ^ w[(i+8)&15] ^ w[(i+2)&15] ^ w[i&15], 1);
			w[i&15] = word;
		}

		u32 f, k;
		if (i < 20) {
			f = (b & c) | (~b & d);
			k = 0x5A827999;
		} else if (i < 40) {
			f = b ^ c ^ d;
			k = 0x6ED9EBA1;
		} else if (i < 60) {
			f = (b & c) | (b & d) | (c & d);
			k = 0x8F1BBCDC;
		} else {
			f = b ^ c ^ d;
			k = 0xCA62C1D6;
		}

		const u32 t = rol(a, 5) + f + e + k + word;
		e = d;
		d = c;
		c = rol(b, 30);
		b = a;
		a = t;
	}

	ctx->h[0] += a;
	ctx->h[1] += b;
	ctx->h[2] += c;
	ctx->h[3] += d;
	ctx->h[4] += e;
}

static void sha1Init(sha1Ctx* ctx) {
	ctx->h[0] = 0x67452301;
	ctx->h[1] = 0xEFCDAB89;
	ctx->h[2] = 0x98BADCFE;
	ctx->h[3] = 0x10325476;
	ctx->h[4] = 0xC3D2E1F0;
	ctx->total = 0;
	ctx->fill = 0;
}

static void sha1Update(sha1Ctx* ctx, const u8* data, u32 len) {
	ctx->total += len;
	while (len--) {
		ctx->block[ctx->fill++] = *data++;
		if (ctx->fill == SHA1_BLOCK) {
			sha1Block(ctx);
			ctx->fill = 0;
		}
	}
}

static void sha1Final(sha1Ctx* ctx, u8* out) {
	const u32 bits = ctx->total * 8;
	const u32 high = ctx->total >> 29;

	const u8 pad = 0x80;
	const u8 zero = 0;
	sha1Update(ctx, &pad, 1);
	while (ctx->fill != SHA1_BLOCK - 8) {
		sha1Update(ctx, &zero, 1);
	}

	const u8 length[8] = {
		0, 0, 0, high,
		bits >> 24, (bits >> 16) & 0xFF, (bits >> 8) & 0xFF, bits & 0xFF
	};
	sha1Update(ctx, length, 8);

	for (int i = 0; i < 5; i++) {
		out[i*4+0] = ctx->h[i] >> 24;
		out[i*4+1] = (ctx->h[i] >> 16) & 0xFF;
		out[i*4+2] = (ctx->h[i] >> 8) & 0xFF;
		out[i*4+3] = ctx->h[i] & 0xFF;
	}
}

void wpaHmacSha1(u8* out, const u8* key, u32 keyLen, const u8* data, u32 dataLen) {
	u8 pad[SHA1_BLOCK];
	u8 inner[WPA_SHA1_LEN];
	sha1Ctx ctx;

	// Keys here are never longer than a block
	for (u32 i = 0; i < SHA1_BLOCK; i++) {
		pad[i] = (i < keyLen ? key[i] : 0) ^ 0x36;
	}
	sha1Init(&ctx);
	sha1Update(&ctx, pad, SHA1_BLOCK);
	sha1Update(&ctx, data, dataLen);
	sha1Final(&ctx, inner);

	for (u32 i = 0; i < SHA1_BLOCK; i++) {
		pad[i] ^= 0x36 ^ 0x5C;
	}
	sha1Init(&ctx);
	sha1Update(&ctx, pad, SHA1_BLOCK);
	sha1Update(&ctx, inner, WPA_SHA1_LEN);
	sha1Final(&ctx, out);
}

void wpaPrf(u8* out, u32 outLen, const u8* key, u32 keyLen, u8* data, u32 dataLen) {
	u8 temp[WPA_SHA1_LEN];
	u8* counter = data + dataLen - 1;

	*counter = 0;
	while (outLen) {
		const u32 len = outLen > WPA_SHA1_LEN ? WPA_SHA1_LEN : outLen;
		wpaHmacSha1(temp, key, keyLen, data, dataLen);
		(*counter)++;
		for (u32 i = 0; i < len; i++) {
			*out++ = temp[i];
		}
		outLen -= len;
	}
}

// AES-128, decryption only

static u8 sbox[256];
static u8 invSbox[256];
static bool sboxReady = false;

static inline u8 xtime(u8 x) {
	return (x << 1) ^ ((x & 0x80) ? 0x1B : 0);
}

static void aesInitTables(void) {
	if (sboxReady) {
		return;
	}

	// Walk the field with generator 3 and its inverse
	u8 p = 1, q = 1;
	do {
		p = p ^ xtime(p);

		q ^= q << 1;
		q ^= q << 2;
		q ^= q << 4;
		if (q & 0x80) {
			q ^= 0x09;
		}

		const u8 x = q ^ ((q << 1) | (q >> 7)) ^ ((q << 2) | (q >> 6)) ^ ((q << 3) | (q >> 5)) ^ ((q << 4) | (q >> 4));
		sbox[p] = x ^ 0x63;
	} while (p != 1);
	sbox[0] = 0x63;

	for (u32 i = 0; i < 256; i++) {
		invSbox[sbox[i]] = i;
	}
	sboxReady = true;
}

static void aesExpandKey(const u8* key, u8* roundKeys) {
	u8 rcon = 1;
	for (u32 i = 0; i < 16; i++) {
		roundKeys[i] = key[i];
	}
	for (u32 i = 16; i < 176; i += 4) {
		u8 t0 = roundKeys[i-4], t1 = roundKeys[i-3], t2 = roundKeys[i-2], t3 = roundKeys[i-1];
		if ((i & 15) == 0) {
			const u8 temp = t0;
			t0 = sbox[t1] ^ rcon;
			t1 = sbox[t2];
			t2 = sbox[t3];
			t3 = sbox[temp];
			rcon = xtime(rcon);
		}
		roundKeys[i+0] = roundKeys[i-16] ^ t0;
		roundKeys[i+1] = roundKeys[i-15] ^ t1;
		roundKeys[i+2] = roundKeys[i-14] ^ t2;
		roundKeys[i+3] = roundKeys[i-13] ^ t3;
	}
}

static inline u8 aesMul(u8 x, u8 factor) {
	u8 result = 0;
	while (factor) {
		if (factor & 1) {
			result ^= x;
		}
		x = xtime(x);
		factor >>= 1;
	}
	return result;
}

static void aesDecrypt(const u8* roundKeys, u8* s) {
	for (u32 i = 0; i < 16; i++) {
		s[i] ^= roundKeys[160+i];
	}

	for (int round = 9; round >= 0; round--) {
		// Inverse ShiftRows and SubBytes
		u8 t[16];
		for (u32 i = 0; i < 16; i++) {
			t[(i + 4*(i & 3)) & 15] = invSbox[s[i]];
		}

		for (u32 i = 0; i < 16; i++) {
			s[i] = t[i] ^ roundKeys[round*16+i];
		}

		if (round == 0) {
			break;
		}

		// Inverse MixColumns
		for (u32 c = 0; c < 16; c += 4) {
			const u8 a = s[c], b = s[c+1], d = s[c+2], e = s[c+3];
			s[c+0] = aesMul(a, 14) ^ aesMul(b, 11) ^ aesMul(d, 13) ^ aesMul(e, 9);
			s[c+1] = aesMul(a, 9) ^ aesMul(b, 14) ^ aesMul(d, 11) ^ aesMul(e, 13);
			s[c+2] = aesMul(a, 13) ^ aesMul(b, 9) ^ aesMul(d, 14) ^ aesMul(e, 11);
			s[c+3] = aesMul(a, 11) ^ aesMul(b, 13) ^ aesMul(d, 9) ^ aesMul(e, 14);
		}
	}
}

bool wpaAesUnwrap(const u8* kek, const u8* in, u8* out, u32 blocks) {
	u8 roundKeys[176];
	u8 a[8];
	u8 b[16];

	aesInitTables();
	aesExpandKey(kek, roundKeys);

	for (u32 i = 0; i < 8; i++) {
		a[i] = in[i];
	}
	for (u32 i = 0; i < blocks*8; i++) {
		out[i] = in[8+i];
	}

	for (int j = 5; j >= 0; j--) {
		for (u32 i = blocks; i >= 1; i--) {
			u8* r = out + (i-1)*8;
			for (u32 k = 0; k < 8; k++) {
				b[k] = a[k];
				b[8+k] = r[k];
			}
			b[7] ^= (blocks*j + i) & 0xFF;
			b[6] ^= ((blocks*j + i) >> 8) & 0xFF;
			aesDecrypt(roundKeys, b);
			for (u32 k = 0; k < 8; k++) {
				a[k] = b[k];
				r[k] = b[8+k];
			}
		}
	}

	for (u32 i = 0; i < 8; i++) {
		if (a[i] != 0xA6) {
			return false;
		}
	}
	return true;
}
