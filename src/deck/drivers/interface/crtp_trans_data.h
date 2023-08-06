#ifndef __CRTP_TRANS_DATA_H__
#define __CRTP_TRANS_DATA_H__

#define CRTP_TRANS_DATA_TASK_NAME "CRTP_TRANS_DATA_TASK"
#define CRTP_PORT_TRANSFER_DATA 9

#include <stdint.h>

typedef union olsrPacket_u
{
    uint8_t raw[9];
    struct
    {
        uint8_t test1;
        uint16_t test2;
        uint16_t test3;
        float test4;
    } __attribute__((packed));
} __attribute__((packed)) olsrPacket;

typedef union
{
    uint8_t raw[17];
    struct
    {
        uint32_t magic;     // 4 bytes 4
        int16_t jitter;     // 2 bytes 6
        uint16_t period;    // 2 bytes 8
        uint8_t type;       // 1 bytes 用于区分是第几个数组 9
        uint16_t block;     // 2 bytes 因为要分块传输数据，所以要确定是第几块 11
        uint16_t msgLength; // 2 bytes 传输的数据块的长度 13
        uint16_t firstDim; // 2 bytes 测距表的存储数量 15
        uint16_t secondDim; // 2bytes, 最大的连续丢包的数量 17
    } __attribute__((packed));
} __attribute__((packed)) CrtpSendData_Meta_t;

void crtpTransDataInit();
void crtpSendDataWithArray(CrtpSendData_Meta_t sendData_Meta, uint16_t msgSize, uint8_t *pointer);

#endif