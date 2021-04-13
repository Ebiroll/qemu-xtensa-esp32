
#include "qemu/osdep.h"
#include "crypto/cipher.h"
#include "nrf51_aes_ccm.h"

/*
 * Counter with CBC-MAC (CCM) with AES
 *
 * Copyright (c) 2010-2012, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

/*
 * AES CCM implementation is taken from wpa_supplicant.
 * It is modified to be compatible with QEMU calls.
 */

#define AES_BLOCK_SIZE (16u)

#define PUT_BE16(a, val)            \
    do {                    \
        (a)[0] = ((uint16_t) (val)) >> 8;    \
        (a)[1] = ((uint16_t) (val)) & 0xff;  \
    } while (0)


static void xor_aes_block(uint8_t *dst, const uint8_t *src)
{
    uint32_t *d = (uint32_t *) dst;
    uint32_t *s = (uint32_t *) src;
    *d++ ^= *s++;
    *d++ ^= *s++;
    *d++ ^= *s++;
    *d++ ^= *s++;
}

static int aes_encrypt(void * aes, const uint8_t * block, uint8_t * out)
{
    Error * err;
    if (qcrypto_cipher_encrypt(aes, block, out, AES_BLOCK_SIZE, &err) < 0)
    {
        printf("[aes-ccm] encrypt returned error\n");
        return -1;
    }

    return 0;
}

static void aes_ccm_auth_start(void *aes, size_t M, size_t L, const uint8_t *nonce,
                               const uint8_t *aad, size_t aad_len, size_t plain_len,
                               uint8_t *x)
{
    uint8_t aad_buf[2 * AES_BLOCK_SIZE];
    uint8_t b[AES_BLOCK_SIZE];

    /* Authentication */
    /* B_0: Flags | Nonce N | l(m) */
    b[0] = aad_len ? 0x40 : 0 /* Adata */;
    b[0] |= (((M - 2) / 2) /* M' */ << 3);
    b[0] |= (L - 1) /* L' */;
    memcpy(&b[1], nonce, 15 - L);
    PUT_BE16(&b[AES_BLOCK_SIZE - L], plain_len);

    aes_encrypt(aes, b, x); /* X_1 = E(K, B_0) */

    if (!aad_len)
        return;

    PUT_BE16(aad_buf, aad_len);
    memcpy(aad_buf + 2, aad, aad_len);
    memset(aad_buf + 2 + aad_len, 0, sizeof(aad_buf) - 2 - aad_len);

    xor_aes_block(aad_buf, x);
    aes_encrypt(aes, aad_buf, x); /* X_2 = E(K, X_1 XOR B_1) */

    if (aad_len > AES_BLOCK_SIZE - 2) {
        xor_aes_block(&aad_buf[AES_BLOCK_SIZE], x);
        /* X_3 = E(K, X_2 XOR B_2) */
        aes_encrypt(aes, &aad_buf[AES_BLOCK_SIZE], x);
    }
}


static void aes_ccm_auth(void *aes, const uint8_t *data, size_t len, uint8_t *x)
{
    size_t last = len % AES_BLOCK_SIZE;
    size_t i;

    for (i = 0; i < len / AES_BLOCK_SIZE; i++) {
        /* X_i+1 = E(K, X_i XOR B_i) */
        xor_aes_block(x, data);
        data += AES_BLOCK_SIZE;
        aes_encrypt(aes, x, x);
    }
    if (last) {
        /* XOR zero-padded last block */
        for (i = 0; i < last; i++)
            x[i] ^= *data++;
        aes_encrypt(aes, x, x);
    }
}


static void aes_ccm_encr_start(size_t L, const uint8_t *nonce, uint8_t *a)
{
    /* A_i = Flags | Nonce N | Counter i */
    a[0] = L - 1; /* Flags = L' */
    memcpy(&a[1], nonce, 15 - L);
}


static void aes_ccm_encr(void *aes, size_t L, const uint8_t *in, size_t len, uint8_t *out,
                         uint8_t *a)
{
    size_t last = len % AES_BLOCK_SIZE;
    size_t i;

    /* crypt = msg XOR (S_1 | S_2 | ... | S_n) */
    for (i = 1; i <= len / AES_BLOCK_SIZE; i++) {
        PUT_BE16(&a[AES_BLOCK_SIZE - 2], i);
        /* S_i = E(K, A_i) */
        aes_encrypt(aes, a, out);
        xor_aes_block(out, in);
        out += AES_BLOCK_SIZE;
        in += AES_BLOCK_SIZE;
    }
    if (last) {
        PUT_BE16(&a[AES_BLOCK_SIZE - 2], i);
        aes_encrypt(aes, a, out);
        /* XOR zero-padded last block */
        for (i = 0; i < last; i++)
            *out++ ^= *in++;
    }
}


static void aes_ccm_encr_auth(void *aes, size_t M, uint8_t *x, uint8_t *a, uint8_t *auth)
{
    size_t i;
    uint8_t tmp[AES_BLOCK_SIZE];

    /* U = T XOR S_0; S_0 = E(K, A_0) */
    PUT_BE16(&a[AES_BLOCK_SIZE - 2], 0);
    aes_encrypt(aes, a, tmp);
    for (i = 0; i < M; i++)
        auth[i] = x[i] ^ tmp[i];
}


static void aes_ccm_decr_auth(void *aes, size_t M, uint8_t *a, const uint8_t *auth, uint8_t *t)
{
    size_t i;
    uint8_t tmp[AES_BLOCK_SIZE];

    /* U = T XOR S_0; S_0 = E(K, A_0) */
    PUT_BE16(&a[AES_BLOCK_SIZE - 2], 0);
    aes_encrypt(aes, a, tmp);
    for (i = 0; i < M; i++)
        t[i] = auth[i] ^ tmp[i];
}


/* AES-CCM with fixed L=2 and aad_len <= 30 assumption */
int aes_ccm_ae(const uint8_t *key, size_t key_len, const uint8_t *nonce,
               size_t M, const uint8_t *plain, size_t plain_len,
               const uint8_t *aad, size_t aad_len, uint8_t *crypt, uint8_t *auth)
{
    const size_t L = 2; //Specificed as 2 in BT spec. as well.
    QCryptoCipher *aes;
    Error * err;
    uint8_t x[AES_BLOCK_SIZE], a[AES_BLOCK_SIZE];

    if (aad_len > 30 || M > AES_BLOCK_SIZE)
        return -1;

    aes = qcrypto_cipher_new(QCRYPTO_CIPHER_ALG_AES_128,
                             QCRYPTO_CIPHER_MODE_ECB,
                             key, key_len, &err);
    if (aes == NULL)
        return -1;

    aes_ccm_auth_start(aes, M, L, nonce, aad, aad_len, plain_len, x);
    aes_ccm_auth(aes, plain, plain_len, x);

    /* Encryption */
    aes_ccm_encr_start(L, nonce, a);
    aes_ccm_encr(aes, L, plain, plain_len, crypt, a);
    aes_ccm_encr_auth(aes, M, x, a, auth);

    qcrypto_cipher_free(aes);

    return 0;
}


/* AES-CCM with fixed L=2 and aad_len <= 30 assumption */
int aes_ccm_ad(const uint8_t *key, size_t key_len, const uint8_t *nonce,
               size_t M, const uint8_t *crypt, size_t crypt_len,
               const uint8_t *aad, size_t aad_len, const uint8_t *auth, uint8_t *plain)
{
    const size_t L = 2;
    QCryptoCipher *aes;
    Error * err;
    uint8_t x[AES_BLOCK_SIZE], a[AES_BLOCK_SIZE];
    uint8_t t[AES_BLOCK_SIZE];

    if (aad_len > 30 || M > AES_BLOCK_SIZE)
        return -1;

    aes = qcrypto_cipher_new(QCRYPTO_CIPHER_ALG_AES_128,
                             QCRYPTO_CIPHER_MODE_ECB,
                             key, key_len, &err);
    if (aes == NULL)
        return -1;

    /* Decryption */
    aes_ccm_encr_start(L, nonce, a);
    aes_ccm_decr_auth(aes, M, a, auth, t);

    /* plaintext = msg XOR (S_1 | S_2 | ... | S_n) */
    aes_ccm_encr(aes, L, crypt, crypt_len, plain, a);

    aes_ccm_auth_start(aes, M, L, nonce, aad, aad_len, crypt_len, x);
    aes_ccm_auth(aes, plain, crypt_len, x);

    qcrypto_cipher_free(aes);

    if (memcmp(x, t, M) != 0) {
        return -1;
    }

    return 0;
}

static void hexdump(const char * label, const uint8_t * data, const int size)
{
    printf("%s:", label);
    for (int i = 0; i < size; i++)
    {
        if (i % 4 == 0)
        {
            puts("");
        }
        printf("%02x ", data[i]);
    }
    puts("");
}

/* 
 * Test vectors taken from RFC 3610.
 * https://www.ietf.org/rfc/rfc3610.txt
 */
void nrf51_aes_ccm_test(void)
{
    const uint8_t key[] = { 
        0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
        0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF
    };
    const uint8_t nonce[] = {
        0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xA0,
        0xA1, 0xA2, 0xA3, 0xA4, 0xA5
    };
    const uint8_t packet[] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E
    };
    const uint8_t cipher_expected[] = {
        0x58, 0x8C, 0x97, 0x9A, 0x61, 0xC6, 0x63, 0xD2,
        0xF0, 0x66, 0xD0, 0xC2, 0xC0, 0xF9, 0x89, 0x80,
        0x6D, 0x5F, 0x6B, 0x61, 0xDA, 0xC3, 0x84
    };
    const uint8_t auth_expected[] = {
        0x17, 0xE8, 0xD1, 0x2C, 0xFD, 0xF9, 0x26, 0xE0
    };
    const uint8_t aad_len = 8;
    const uint8_t M = 8; //size of tag (MAC) field.
    const uint8_t * data_begin = packet + aad_len;
    const size_t data_len = sizeof(packet) - aad_len;
    uint8_t cipher[64];
    uint8_t auth[M];
    /*int aes_ccm_ae(const uint8_t *key, size_t key_len, const uint8_t *nonce,
               size_t M, const uint8_t *plain, size_t plain_len,
               const uint8_t *aad, size_t aad_len, uint8_t *crypt, uint8_t *auth) */
    int ret = aes_ccm_ae(key, sizeof(key), nonce, M, data_begin, data_len,
                packet /* aad */, aad_len, cipher, auth);

    printf("Result of AES: %d\n", ret);

    hexdump("AAD", packet, aad_len);
    hexdump("Cipher", cipher, sizeof(packet) - aad_len);
    hexdump("MAC", auth, M);

    if ( !(memcmp(cipher, cipher_expected, data_len) || memcmp(auth, auth_expected, M)) )
    {
        puts("Test PASSED.");
    }
    else
    {
        puts("Test FAILED.");
    }

    exit(0);
}
