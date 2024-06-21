#include <stdio.h>
#include "common.h"
#include "crc.h"

#define CRC5_MASK 0x1F


typedef struct
{
    float frequency;
} bm1397Module;

typedef enum
{
    JOB_PACKET = 0,
    CMD_PACKET = 1,
} packet_type_t;

typedef enum
{
    JOB_RESP = 0,
    CMD_RESP = 1,
} response_type_t;

typedef struct __attribute__((__packed__))
{
    uint8_t job_id;
    uint8_t num_midstates;
    uint8_t starting_nonce[4];
    uint8_t nbits[4];
    uint8_t ntime[4];
    uint8_t merkle4[4];
    uint8_t midstate[32];
    uint8_t midstate1[32];
    uint8_t midstate2[32];
    uint8_t midstate3[32];
} job_packet;

void BM1397_init(uint64_t frequency);
int BM1397_set_default_baud(void);
void BM1397_send_hash_frequency(float frequency);
task_result * BM1397_proccess_work();
