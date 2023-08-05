#ifndef _ADHOCDECK_SENDDATA_H_
#define _ADHOCDECK_SENDDATA_H_

#include "adhocdeck.h"
#include "swarm_ranging.h"
#include "ranging_struct.h"
typedef union
{
  uint8_t raw[11];
  struct
  {
    uint32_t magic;     // 4 bytes
    int16_t jitter;     // 2 bytes
    uint16_t period;    // 2bytes;
    uint16_t msgLength; // 2bytes
    uint8_t type;       // 1bytes
  } __attribute__((packed));
} __attribute__((packed)) SendData_Meta_t;

#endif