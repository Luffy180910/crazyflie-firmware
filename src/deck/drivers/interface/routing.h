#ifndef __ROUTING_H__
#define __ROUTING_H__
#include "stdint.h"
#include "adhocdeck.h"

#define ROUTING_RX_QUEUE_SIZE 10
#define ROUTING_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)

#define ROUTING_TX_INTERVAL 100

typedef struct {
  uint32_t seqNumber;
  uint16_t srcAddress;
} __attribute__((packed)) MockData_t;

void routingInit();
#endif