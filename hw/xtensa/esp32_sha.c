
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "elf.h"
#include "cpu.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/sysbus.h"
#include "hw/block/flash.h"
#include "sysemu/block-backend.h"
#include "sysemu/char.h"
#include "sysemu/device_tree.h"
#include "qemu/error-report.h"
#include "bootparam.h"
#include "qemu/timer.h"
#include "inttypes.h"
#include "hw/i2c/i2c.h"
#include "esp32_sha.h"

#define uchar unsigned char // 8-bit byte
#define uint unsigned int // 32-bit word



/* 
sha256_start
io read 1c 
io write 1c,2 
io read 20 
io write 20,0 

-----------
while(REG_READ(SHA_256_BUSY_REG) != 0) 
io read 309c

  sha_text_reg[block_count + i] = __builtin_bswap32(w[i]); 
  io write 3000,61626364 


io write 3034,0 
io write 3038,0 

  --------------------------
  qemu_sha256_finish(handle,hash); 


REG_WRITE(SHA_256_LOAD_REG, 1);  
io write 3098,1 

      uint32_t *sha_text_reg = (uint32_t *)(SHA_TEXT_BASE);                                                           │
   │160         for (int i = 0; i < DIGEST_WORDS; i++) {                                                                        │
   │161             digest_words[i] = __builtin_bswap32(sha_text_reg[i]);                                                       │
   │162         }                                         

Hash results,
io read 3000 
io read 3004 
io read 3008 
io read 300c 
io read 3010
io read 3014 
io read 3018 
io read 301c 



       REG_WRITE(SHA_256_CONTINUE_REG, 1);     
       io write 3094,1

*/

#include <stdio.h> 
#include <string.h>


// DBL_INT_ADD treats two unsigned ints a and b as one 64-bit integer and adds c to it
#define DBL_INT_ADD(a,b,c) if (a > 0xffffffff - (c)) ++b; a += c;
#define ROTLEFT(a,b) (((a) << (b)) | ((a) >> (32-(b))))
#define ROTRIGHT(a,b) (((a) >> (b)) | ((a) << (32-(b))))

#define CH(x,y,z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTRIGHT(x,2) ^ ROTRIGHT(x,13) ^ ROTRIGHT(x,22))
#define EP1(x) (ROTRIGHT(x,6) ^ ROTRIGHT(x,11) ^ ROTRIGHT(x,25))
#define SIG0(x) (ROTRIGHT(x,7) ^ ROTRIGHT(x,18) ^ ((x) >> 3))
#define SIG1(x) (ROTRIGHT(x,17) ^ ROTRIGHT(x,19) ^ ((x) >> 10))


uint k[64] = {
   0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
   0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
   0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
   0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
   0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
   0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
   0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
   0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};


void sha256_transform(SHA256_CTX *ctx, uchar data[])
{  
   uint a,b,c,d,e,f,g,h,i,j,t1,t2,m[64];
      
   for (i=0,j=0; i < 16; ++i, j += 4)
      m[i] = (data[j] << 24) | (data[j+1] << 16) | (data[j+2] << 8) | (data[j+3]);
   for ( ; i < 64; ++i)
      m[i] = SIG1(m[i-2]) + m[i-7] + SIG0(m[i-15]) + m[i-16];

   a = ctx->state[0];
   b = ctx->state[1];
   c = ctx->state[2];
   d = ctx->state[3];
   e = ctx->state[4];
   f = ctx->state[5];
   g = ctx->state[6];
   h = ctx->state[7];
   
   for (i = 0; i < 64; ++i) {
      t1 = h + EP1(e) + CH(e,f,g) + k[i] + m[i];
      t2 = EP0(a) + MAJ(a,b,c);
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
   }
   
   ctx->state[0] += a;
   ctx->state[1] += b;
   ctx->state[2] += c;
   ctx->state[3] += d;
   ctx->state[4] += e;
   ctx->state[5] += f;
   ctx->state[6] += g;
   ctx->state[7] += h;
}  

void sha256_init(SHA256_CTX *ctx)
{  
   ctx->datalen = 0; 
   ctx->bitlen[0] = 0; 
   ctx->bitlen[1] = 0; 
   ctx->state[0] = 0x6a09e667;
   ctx->state[1] = 0xbb67ae85;
   ctx->state[2] = 0x3c6ef372;
   ctx->state[3] = 0xa54ff53a;
   ctx->state[4] = 0x510e527f;
   ctx->state[5] = 0x9b05688c;
   ctx->state[6] = 0x1f83d9ab;
   ctx->state[7] = 0x5be0cd19;
}

void sha256_update(SHA256_CTX *ctx, const uchar *data, uint len)
{  
   uint t,i;
   
   for (i=0; i < len; ++i) { 
      ctx->data[ctx->datalen] = data[i]; 
      ctx->datalen++; 
      if (ctx->datalen == 64) { 
         sha256_transform(ctx,ctx->data);
         DBL_INT_ADD(ctx->bitlen[0],ctx->bitlen[1],512); 
         ctx->datalen = 0; 
      }  
   }  
}  

void sha256_final(SHA256_CTX *ctx, uchar hash[])
{  
   uint i; 
#if 0
   i = ctx->datalen; 
   // Pad whatever data is left in the buffer. 
   if (ctx->datalen < 56) { 
      ctx->data[i++] = 0x80; 
      while (i < 56) 
         ctx->data[i++] = 0x00; 
   }  
   else { 
      ctx->data[i++] = 0x80; 
      while (i < 64) 
         ctx->data[i++] = 0x00; 
      sha256_transform(ctx,ctx->data);
      memset(ctx->data,0,56); 
   }  
   
   // Append to the padding the total message's length in bits and transform. 
   DBL_INT_ADD(ctx->bitlen[0],ctx->bitlen[1],ctx->datalen * 8);
   ctx->data[63] = ctx->bitlen[0]; 
   ctx->data[62] = ctx->bitlen[0] >> 8; 
   ctx->data[61] = ctx->bitlen[0] >> 16; 
   ctx->data[60] = ctx->bitlen[0] >> 24; 
   ctx->data[59] = ctx->bitlen[1]; 
   ctx->data[58] = ctx->bitlen[1] >> 8; 
   ctx->data[57] = ctx->bitlen[1] >> 16;  
   ctx->data[56] = ctx->bitlen[1] >> 24; 

   sha256_transform(ctx,ctx->data);

#endif



   // Since this implementation uses little endian byte ordering and SHA uses big endian,
   // reverse all the bytes when copying the final state to the output hash. 
   for (i=0; i < 4; ++i) { 
      hash[i]    = (ctx->state[0] >> (24-i*8)) & 0x000000ff; 
      hash[i+4]  = (ctx->state[1] >> (24-i*8)) & 0x000000ff; 
      hash[i+8]  = (ctx->state[2] >> (24-i*8)) & 0x000000ff;
      hash[i+12] = (ctx->state[3] >> (24-i*8)) & 0x000000ff;
      hash[i+16] = (ctx->state[4] >> (24-i*8)) & 0x000000ff;
      hash[i+20] = (ctx->state[5] >> (24-i*8)) & 0x000000ff;
      hash[i+24] = (ctx->state[6] >> (24-i*8)) & 0x000000ff;
      hash[i+28] = (ctx->state[7] >> (24-i*8)) & 0x000000ff;
   }  
}  


void print_dump(unsigned char hash[])
{
   int idx;
   for (idx=0; idx < 64*2; idx++) {
      printf("%02x",hash[idx]);
      if (idx%10==9) {
          printf("\n");
      }

      if (idx%60==59) {
          printf("\n");
      }

   }
   printf("\n");
}

uint64_t g_lastVal=0;

static void esp_sha_write(void *opaque, hwaddr addr,
        uint64_t val, unsigned size)
{
    Esp32SHAState *s=opaque;

    if (s->initiated!=0x47) {
        //printf("sha init 0\n");
        sha256_init(&s->ctx);
        s->initiated=0x47;
        s->max_buff_pos=0;
        s->total_buff_pos=0;
    }

    //printf("sha write 0x%" PRIx64 " \n",addr);
    if (addr<MAX_SHA_BUFF) {
        if (s->max_buff_pos<addr) {
            s->max_buff_pos=addr;
        }
        // Last value written is num bits
        int b_pos=addr;
        s->num_bits=val & 0xffffffff;
        if (addr>=4) {
            //s->num_bits=s->num_bits + (g_lastVal << 32);
        }
        g_lastVal=val  & 0xffffffff;
        s->buffer_data[b_pos+3]=0xff & (val & 0x000000ff);
        s->buffer_data[b_pos+2]=0xff & ((val & 0x0000ff00) >> 8);
        s->buffer_data[b_pos+1]=0xff & ((val & 0x00ff0000) >> 16);
        s->buffer_data[b_pos+0]=0xff & ((val & 0xff000000) >> 24);
        s->total_buffer[s->total_buff_pos+0]=s->buffer_data[b_pos+0];
        s->total_buffer[s->total_buff_pos+1]=s->buffer_data[b_pos+1];
        s->total_buffer[s->total_buff_pos+2]=s->buffer_data[b_pos+2];
        s->total_buffer[s->total_buff_pos+3]=s->buffer_data[b_pos+3];
        s->total_buff_pos+=4;

        //printf("sha val 0x%x\n",s->buffer_data[b_pos]);
        //printf("sha val 0x%x\n",s->buffer_data[b_pos+1]);
        //printf("sha val 0x%x\n",s->buffer_data[b_pos+2]);
        //printf("sha val 0x%x\n",s->buffer_data[b_pos+3]);
    }

    switch (addr)
    {
        case 0x90:  // Start
            //printf("sha start\n"); 
            sha256_update(&s->ctx,s->buffer_data,SHA_BUFF_LEN);
            break;

        case 0x94:  // continue
            //printf("sha update 1 %2X\n",SHA_BUFF_LEN); 
            sha256_update(&s->ctx,s->buffer_data,SHA_BUFF_LEN);
            //sha256_update(&s->ctx,s->total_buffer,s->num_bits/8);
            //sha256_init(&s->ctx);
            break;

        case 0x98:  // load
            //printf("sha final %d %d %d\n",s->ctx.datalen,s->num_bits/8, s->max_buff_pos);
            //print_dump(s->total_buffer);

            // Saved buffer of all data 
            //sha256_init(&s->ctx);
            //sha256_update(&s->ctx,s->total_buffer,s->num_bits/8);
            //sha256_final(&s->ctx,s->hash_result);

            s->max_buff_pos=0;
            s->total_buff_pos=0;
            
            // This would have been better, but does not work so well :-P
            //s->num_bits 
            //print_dump(s->total_buffer);
            //sha256_update(&s->ctx,s->buffer_data,SHA_BUFF_LEN);
            sha256_final(&s->ctx,s->hash_result);

            sha256_init(&s->ctx);
            //memset(ctx->data,0,56); 
            //printf("sha init 1\n"); 
 

            break;


        case 0x9c:  // 
            return ;
            break;
        default:
         return ;
            break;
    }
    
}

static uint64_t esp_sha_read(void *opaque, hwaddr addr,
        unsigned size)
{

    Esp32SHAState *s=opaque;

    //printf("sha read 0x%" PRIx64 " \n",addr);

    if (addr<=MAX_SHA_BUFF) {
        uint64_t b_pos=addr;
        uint64_t ret= (s->hash_result[b_pos+3]) + ((s->hash_result[b_pos+2]) << 8) + ((s->hash_result[b_pos+1]) << 16) + ((s->hash_result[b_pos+0]) << 24);
        //printf("sha val 0x%" PRIx64 " \n",ret);

        return ret;
    }

    switch (addr) {
        case 0x90:
          return 0;
        break;
        case 0x9c:
          return 0;
        break;

        default:
         return 0x1111111111;
        break;
    }


}

const MemoryRegionOps esp_sha_ops = {
    .read = esp_sha_read,
    .write = esp_sha_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};
