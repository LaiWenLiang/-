#include "bsp_token.h"
#include <string.h>
#include <stdio.h>

/***************************************************************************************************
 * 本文件包含三个小工具：SHA1、HMAC-SHA1、base64 编解码
 * 都是标准算法的直白实现，没有依赖任何库
 ***************************************************************************************************/

/* ==================== SHA1 ==================== */
typedef struct
{
    uint32_t h[5];        /* 结果寄存器 */
    uint8_t  buf[64];     /* 当前块缓冲 */
    uint32_t buf_len;     /* 缓冲里已有字节数 */
    uint32_t total_len;   /* 总字节数 */
} SHA1_Ctx;

static uint32_t sha1_rol(uint32_t v, uint8_t n)
{
    return (v << n) | (v >> (32 - n));
}

/* 处理一个 64 字节块 */
static void sha1_block(SHA1_Ctx *ctx, const uint8_t *block)
{
    uint32_t w[80];
    uint32_t a, b, c, d, e, f, k, tmp;
    uint8_t  i;

    for (i = 0; i < 16; i++)
    {
        w[i] = ((uint32_t)block[i * 4] << 24) | ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] << 8) | block[i * 4 + 3];
    }
    for (i = 16; i < 80; i++)
    {
        w[i] = sha1_rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    a = ctx->h[0]; b = ctx->h[1]; c = ctx->h[2]; d = ctx->h[3]; e = ctx->h[4];

    for (i = 0; i < 80; i++)
    {
        if (i < 20)      { f = (b & c) | ((~b) & d);       k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d;                  k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else             { f = b ^ c ^ d;                  k = 0xCA62C1D6; }
        tmp = sha1_rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = sha1_rol(b, 30); b = a; a = tmp;
    }

    ctx->h[0] += a; ctx->h[1] += b; ctx->h[2] += c; ctx->h[3] += d; ctx->h[4] += e;
}

static void sha1_init(SHA1_Ctx *ctx)
{
    ctx->h[0] = 0x67452301; ctx->h[1] = 0xEFCDAB89;
    ctx->h[2] = 0x98BADCFE; ctx->h[3] = 0x10325476;
    ctx->h[4] = 0xC3D2E1F0;
    ctx->buf_len = 0;
    ctx->total_len = 0;
}

static void sha1_update(SHA1_Ctx *ctx, const uint8_t *data, uint32_t len)
{
    while (len--)
    {
        ctx->buf[ctx->buf_len++] = *data++;
        ctx->total_len++;
        if (ctx->buf_len == 64)
        {
            sha1_block(ctx, ctx->buf);
            ctx->buf_len = 0;
        }
    }
}

static void sha1_final(SHA1_Ctx *ctx, uint8_t out[20])
{
    uint64_t bit_len = (uint64_t)ctx->total_len * 8;
    uint8_t  i;

    /* 补 0x80，再补 0，直到剩 8 字节放长度 */
    i = 0x80;
    sha1_update(ctx, &i, 1);
    i = 0;
    while (ctx->buf_len != 56)
    {
        sha1_update(ctx, &i, 1);
    }
    for (i = 0; i < 8; i++)
    {
        uint8_t b = (uint8_t)(bit_len >> (56 - i * 8));
        sha1_update(ctx, &b, 1);
    }
    for (i = 0; i < 5; i++)
    {
        out[i * 4]     = (uint8_t)(ctx->h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(ctx->h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(ctx->h[i] >> 8);
        out[i * 4 + 3] = (uint8_t)(ctx->h[i]);
    }
}

/* ==================== HMAC-SHA1 ==================== */
static void hmac_sha1(const uint8_t *key, uint16_t key_len,
                      const uint8_t *msg, uint16_t msg_len,
                      uint8_t out[20])
{
    uint8_t  k_pad[64];
    uint8_t  tmp[20];
    uint16_t i;
    SHA1_Ctx ctx;

    /* 密钥超过 64 字节先算一次 SHA1；密钥不足补 0 */
    memset(k_pad, 0, 64);
    if (key_len > 64)
    {
        sha1_init(&ctx);
        sha1_update(&ctx, key, key_len);
        sha1_final(&ctx, k_pad);
    }
    else
    {
        memcpy(k_pad, key, key_len);
    }

    /* 内层：SHA1((key ^ ipad) || msg) */
    sha1_init(&ctx);
    for (i = 0; i < 64; i++) { k_pad[i] ^= 0x36; }
    sha1_update(&ctx, k_pad, 64);
    for (i = 0; i < 64; i++) { k_pad[i] ^= 0x36; }   /* 还原 */
    sha1_update(&ctx, msg, msg_len);
    sha1_final(&ctx, tmp);

    /* 外层：SHA1((key ^ opad) || 内层结果) */
    sha1_init(&ctx);
    for (i = 0; i < 64; i++) { k_pad[i] ^= 0x5C; }
    sha1_update(&ctx, k_pad, 64);
    sha1_update(&ctx, tmp, 20);
    sha1_final(&ctx, out);
}

/* ==================== base64 ==================== */
static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static uint16_t base64_encode(const uint8_t *in, uint16_t len, char *out)
{
    uint16_t i = 0, o = 0;

    while (i < len)
    {
        uint32_t v = 0;
        uint8_t  n = 0;
        uint8_t  j;

        for (j = 0; j < 3; j++)
        {
            v <<= 8;
            if (i < len) { v |= in[i++]; n++; }
        }
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = (n > 1) ? b64_table[(v >> 6) & 0x3F] : '=';
        out[o++] = (n > 2) ? b64_table[v & 0x3F] : '=';
    }
    out[o] = '\0';
    return o;
}

static int8_t b64_val(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

static uint16_t base64_decode(const char *in, uint8_t *out)
{
    uint16_t o = 0;
    uint32_t v = 0;
    uint8_t  n = 0;
    int8_t   d;

    while (*in && *in != '=')
    {
        d = b64_val(*in++);
        if (d < 0) continue;
        v = (v << 6) | (uint8_t)d;
        n++;
        if (n == 4)
        {
            out[o++] = (uint8_t)(v >> 16);
            out[o++] = (uint8_t)(v >> 8);
            out[o++] = (uint8_t)v;
            v = 0; n = 0;
        }
    }
    if (n == 3)      { v <<= 6;  out[o++] = (uint8_t)(v >> 16); out[o++] = (uint8_t)(v >> 8); }
    else if (n == 2) { v <<= 12; out[o++] = (uint8_t)(v >> 16); }
    return o;
}

/* ==================== OneNET token ==================== */
uint16_t OneNET_Token(char *out, const char *product_id, const char *dev_key_base64, uint32_t expire_time)
{
    char     content[160];
    char     sign_b64[40];
    uint8_t  key[64];
    uint8_t  sign[20];
    uint16_t key_len;

    /* 1. 设备密钥 base64 解码后才是 HMAC 的 key */
    key_len = base64_decode(dev_key_base64, key);

    /* 2. 拼接签名内容：et\nmethod\nres\nversion */
    snprintf(content, sizeof(content), "%lu\nsha1\nproducts/%s\n2018-10-31",
             (unsigned long)expire_time, product_id);

    /* 3. HMAC-SHA1 签名后 base64 */
    hmac_sha1(key, key_len, (const uint8_t *)content, strlen(content), sign);
    base64_encode(sign, 20, sign_b64);

    /* 4. 拼出完整 token */
    return (uint16_t)snprintf(out, 160,
                              "version=2018-10-31&res=products/%s&et=%lu&method=sha1&sign=%s",
                              product_id, (unsigned long)expire_time, sign_b64);
}
