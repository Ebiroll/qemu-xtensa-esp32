#pragma once


typedef struct {
   unsigned char data[64];
   unsigned int datalen;
   unsigned int bitlen[2];
   unsigned int state[8];
} SHA256_CTX;


#define MAX_SHA_BUFF 32*4
// Bootloader only uses first 15 text registers..??
#define SHA_BUFF_LEN 60


typedef struct Esp32SHAState {
    MemoryRegion iomem;
    int         max_buff_pos;
    SHA256_CTX  ctx;
    int initiated;
    unsigned char buffer_data[MAX_SHA_BUFF];
    // MAx 4 MB, total buffer
    int total_buff_pos;
    int num_bits;
    unsigned char total_buffer[1024*1024*4];
    unsigned char hash_result[MAX_SHA_BUFF];
} Esp32SHAState;

extern const MemoryRegionOps esp_sha_ops;