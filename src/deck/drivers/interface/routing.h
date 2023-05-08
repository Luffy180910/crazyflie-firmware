#ifndef __ROUTING_H__
#define __ROUTING_H__
#include "stdint.h"
#include "adhocdeck.h"

#define ROUTING_RX_QUEUE_SIZE 10
#define ROUTING_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)
#define DATA_INTERVAL 5000
#define DATA_BODY_MAX_SIZE 10

enum ROUTING_TYPE {
  ROUTING_TO_LOCAL = 0,
  ROUTING_TO_NEXT = 1,
  ROUTING_TO_OTHER = 2
};

typedef struct {
  uint16_t srcAddress; // 2 bytes
  uint16_t dstAddress; // 2 bytes
  uint16_t nextAddress; // 2 bytes
  uint16_t msgSequence; // 2 bytes
  uint16_t msgLength; // 2 bytes
  uint8_t timeToLive; // 1 bytes
} __attribute__((packed)) MockData_Header_t; // 11 bytes

typedef struct {
  uint8_t size; // 1 bytes
  uint16_t path[DATA_BODY_MAX_SIZE]; // 20 bytes
} __attribute__((packed)) MockData_Body_t;

typedef struct {
  MockData_Header_t header; // 11 bytes
  MockData_Body_t body; // 20 bytes
} __attribute__((packed)) MockData_t; // 31 bytes


void routingInit();
void computeRouting();

#endif