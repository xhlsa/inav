/*
 * This file is part of INAV.
 *
 * INAV is free software. You can redistribute this software
 * and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * INAV is distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * ExpressLRS binding phrase to UID.
 *
 * The ExpressLRS build and web UI compute the UID as the first six bytes of
 * MD5("-DMY_BINDING_PHRASE=\"<phrase>\""). A minimal RFC 1321 MD5 follows,
 * only used once at receiver initialisation.
 */

#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_EXPRESSLRS

#include "rx/expresslrs_common.h"

typedef struct {
    uint32_t state[4];
    uint64_t length;
    uint8_t buffer[64];
    uint8_t bufferLen;
} md5Context_t;

static const uint32_t md5K[64] = {
    0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
    0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
    0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
    0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
    0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
    0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
    0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
    0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
};

static const uint8_t md5R[64] = {
    7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
    5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20, 5,  9, 14, 20,
    4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
    6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
};

static uint32_t md5Rotl(uint32_t x, uint8_t c)
{
    return (x << c) | (x >> (32 - c));
}

static void md5Transform(md5Context_t *ctx, const uint8_t *block)
{
    uint32_t w[16];
    for (int i = 0; i < 16; i++) {
        w[i] = (uint32_t)block[i * 4] | ((uint32_t)block[i * 4 + 1] << 8) |
               ((uint32_t)block[i * 4 + 2] << 16) | ((uint32_t)block[i * 4 + 3] << 24);
    }

    uint32_t a = ctx->state[0];
    uint32_t b = ctx->state[1];
    uint32_t c = ctx->state[2];
    uint32_t d = ctx->state[3];

    for (int i = 0; i < 64; i++) {
        uint32_t f;
        int g;

        if (i < 16) {
            f = (b & c) | (~b & d);
            g = i;
        } else if (i < 32) {
            f = (d & b) | (~d & c);
            g = (5 * i + 1) % 16;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3 * i + 5) % 16;
        } else {
            f = c ^ (b | ~d);
            g = (7 * i) % 16;
        }

        const uint32_t temp = d;
        d = c;
        c = b;
        b = b + md5Rotl(a + f + md5K[i] + w[g], md5R[i]);
        a = temp;
    }

    ctx->state[0] += a;
    ctx->state[1] += b;
    ctx->state[2] += c;
    ctx->state[3] += d;
}

static void md5Init(md5Context_t *ctx)
{
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xefcdab89;
    ctx->state[2] = 0x98badcfe;
    ctx->state[3] = 0x10325476;
    ctx->length = 0;
    ctx->bufferLen = 0;
}

static void md5Update(md5Context_t *ctx, const uint8_t *data, size_t len)
{
    ctx->length += len;

    while (len--) {
        ctx->buffer[ctx->bufferLen++] = *data++;
        if (ctx->bufferLen == 64) {
            md5Transform(ctx, ctx->buffer);
            ctx->bufferLen = 0;
        }
    }
}

static void md5Final(md5Context_t *ctx, uint8_t digest[16])
{
    const uint64_t bitLength = ctx->length * 8;
    const uint8_t pad = 0x80;
    const uint8_t zero = 0;

    md5Update(ctx, &pad, 1);
    while (ctx->bufferLen != 56) {
        md5Update(ctx, &zero, 1);
    }

    uint8_t lengthBytes[8];
    for (int i = 0; i < 8; i++) {
        lengthBytes[i] = (uint8_t)(bitLength >> (8 * i));
    }
    md5Update(ctx, lengthBytes, 8);

    for (int i = 0; i < 4; i++) {
        digest[i * 4] = (uint8_t)ctx->state[i];
        digest[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 8);
        digest[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 16);
        digest[i * 4 + 3] = (uint8_t)(ctx->state[i] >> 24);
    }
}

void elrsUidFromBindPhrase(const char *phrase, uint8_t uid[6])
{
    static const char prefix[] = "-DMY_BINDING_PHRASE=\"";
    static const char suffix[] = "\"";

    md5Context_t ctx;
    uint8_t digest[16];

    md5Init(&ctx);
    md5Update(&ctx, (const uint8_t *)prefix, strlen(prefix));
    md5Update(&ctx, (const uint8_t *)phrase, strlen(phrase));
    md5Update(&ctx, (const uint8_t *)suffix, strlen(suffix));
    md5Final(&ctx, digest);

    memcpy(uid, digest, 6);
}

#endif // USE_RX_EXPRESSLRS
