#pragma once


typedef struct {
   unsigned char data[64];
   unsigned int datalen;
   unsigned int bitlen[2];
   unsigned int state[8];
} SHA256_CTX;


#define MAX_SHA_BUFF 64 

typedef struct Esp32SHAState {
    MemoryRegion iomem;
    int         max_buff_pos;
    SHA256_CTX  ctx;
    char buffer_data[MAX_SHA_BUFF];
    char hash_result[MAX_SHA_BUFF];
} Esp32SHAState;

extern const MemoryRegionOps esp_sha_ops;