#ifndef __ROUTING_H__
#define __ROUTING_H__
#include "stdint.h"
#include "semphr.h"
#include "adhocdeck.h"

#define ROUTING_RX_QUEUE_SIZE 5
#define ROUTING_RX_QUEUE_ITEM_SIZE sizeof (UWB_Packet_t)

#define ROUTING_TABLE_SIZE 15

/* Data Packet */
#define ROUTING_DATA_PAYLOAD_SIZE_MAX sizeof (UWB_Packet_t) - sizeof(Data_Packet_Header_t)

typedef struct {
  UWB_Address_t srcAddress;
  UWB_Address_t destAddress;
  uint32_t seqNumber;
  uint16_t length;
} __attribute__((packed)) Data_Packet_Header_t;

typedef struct {
  Data_Packet_Header_t header;
  uint8_t payload[ROUTING_DATA_PAYLOAD_SIZE_MAX];
} __attribute__((packed)) Data_Packet_t;

typedef struct {
  // TODO add metrics
} Route_Metric_t;

typedef struct {
  UWB_Address_t destAddress;
  UWB_Address_t nextHop;
  uint8_t hopCount;
  Time_t expirationTime;
  /* AODV properties */
  uint32_t destSeqNumber;
  bool validDestSeqFlag;
  uint64_t precursors; // bit set
  Route_Metric_t metrics;
} Route_Entry_t;

typedef struct {
  Route_Entry_t entries[ROUTING_TABLE_SIZE];
  SemaphoreHandle_t mu; // mutex
} Routing_Table_t; // TODO: implement as a heap

void routingInit();

/* Routing Table Operations */
void routingTableInit(Routing_Table_t *table);
void routingTableAddEntry(Routing_Table_t *table, Route_Entry_t entry);
void routingTableUpdateEntry(Routing_Table_t *table, Route_Entry_t entry);
void routingTableRemoveEntry(Routing_Table_t *table, UWB_Address_t destAddress);
Route_Entry_t routingTableFindEntry(Routing_Table_t *table, UWB_Address_t destAddress);

/* Debug Operations */
void printRoutingTable(Routing_Table_t *table);
#endif