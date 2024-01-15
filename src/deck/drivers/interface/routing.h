#ifndef __ROUTING_H__
#define __ROUTING_H__
#include "stdint.h"
#include "adhocdeck.h"

#define ROUTING_RX_QUEUE_SIZE 10
#define ROUTING_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)

typedef struct {
  UWB_Address_t srcAddress;
  UWB_Address_t destAddress;
  uint32_t seqNumber;
} __attribute__((packed)) DataPacket_t;

void routingInit();
#endif