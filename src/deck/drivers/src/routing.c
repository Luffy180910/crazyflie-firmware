// #define DEBUG_MODULE "ROUTING"

#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "deck.h"
#include "estimator.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "routing.h"
#include "routing_struct.h"
#include "tc.h"
#include <stdlib.h>

// Experiment
#define SRC_ADDR 1
#define DST_ADDR 5

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static int seqNumber = 1;
static uint16_t routingShortestTree[ROUTING_NODE_MAX_SIZE] = {0};
static uint16_t MY_UWB_ADDRESS;

void routingRxCallback(void *parameters) {
//   DEBUG_PRINT("routingRxCallback \n");
}

void routingTxCallback(void *parameters) {
//   DEBUG_PRINT("routingTxCallback \n");
}

int generateRoutingDataMessage(MockData_t *message, uint16_t dstAddress) {
  /* Generate body */
  message->body.size = 0;
  message->body.path[0] = MY_UWB_ADDRESS;
  message->body.size++;
  /* Generate header */
  message->header.msgSequence = seqNumber++;
  message->header.srcAddress = MY_UWB_ADDRESS;
  message->header.dstAddress = dstAddress;
  message->header.nextAddress = routingShortestTreeFindRoute(routingShortestTree, dstAddress, MY_UWB_ADDRESS);
  message->header.timeToLive = 10;
  message->header.msgLength = sizeof(MockData_Header_t) + sizeof(MockData_Body_t);

   /* Error prevention mechanism */
   if (message->header.nextAddress == ROUTING_INF) return -1;

  return message->header.msgLength;
}

static void processRoutingDataMessage(MockData_t *mockData) {
  // DEBUG_PRINT("received routing data, seq number = %d \n", mockData->header.msgSequence);
  MockData_Body_t *mockDataBody = (MockData_Body_t *) &mockData->body;
  if (MY_UWB_ADDRESS == DST_ADDR) {
    DEBUG_PRINT("PATH: ");
    for (int i = 0; i < mockDataBody->size; i++) {
      DEBUG_PRINT("%d:%u ", i, mockDataBody->path[i]);
    }
    DEBUG_PRINT("%u\n", MY_UWB_ADDRESS);
  }
}
static void updateRoutingDataMessage(MockData_t *mockData) {
  /* Update Header */
  MockData_Header_t *mockDataHeader = (MockData_Header_t *) &mockData->header;
  mockDataHeader->nextAddress = routingShortestTreeFindRoute(routingShortestTree, mockDataHeader->dstAddress, MY_UWB_ADDRESS);
  mockDataHeader->timeToLive--;
  /* Update Body */
  MockData_Body_t *mockDataBody = (MockData_Body_t *) &mockData->body;
  mockDataBody->path[mockDataBody->size] = MY_UWB_ADDRESS;
  mockDataBody->size++;
}

static void dispatchRoutingDataMessage(UWB_Packet_t *rxPacketCache) {
  MockData_t *mockData = (MockData_t *) &rxPacketCache->payload;
  MockData_Header_t *mockDataHeader = (MockData_Header_t *) &mockData->header;
  enum ROUTING_TYPE type;
  /* Judge mock data message type */
  if (mockDataHeader->nextAddress == MY_UWB_ADDRESS) {
    type = ROUTING_TO_NEXT;
  } 
  if (mockDataHeader->dstAddress == MY_UWB_ADDRESS) {
    type = ROUTING_TO_LOCAL;
  } 
  // ADD: Fix a logic judge
  // if (mockDataHeader->nextAddress != MY_UWB_ADDRESS && mockDataHeader->dstAddress != MY_UWB_ADDRESS) {
  //   type = ROUTING_TO_OTHER;
  // }
  if (mockDataHeader->nextAddress != MY_UWB_ADDRESS) {
    type = ROUTING_TO_OTHER;
  }
  /* Solve case */
  switch (type) {
    case ROUTING_TO_LOCAL:
      DEBUG_PRINT("LOCAL!\n");
      processRoutingDataMessage(mockData);
      return;
    case ROUTING_TO_NEXT:
      DEBUG_PRINT("NEXT!\n");
      updateRoutingDataMessage(mockData);
      uwbSendPacketBlock(rxPacketCache);
      return;
    case ROUTING_TO_OTHER:
      DEBUG_PRINT("OTHER!\n");
      return;
    default:
      return;
  }
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = DATA;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    // TODO: Desigh data
    computeRouting();
    if (MY_UWB_ADDRESS == SRC_ADDR) {
      int msgLen = generateRoutingDataMessage((MockData_t *) &txPacketCache.payload, DST_ADDR);
      MockData_Header_t *mockDataHeader = (MockData_Header_t *) &txPacketCache.payload;
      DEBUG_PRINT("DataLen: %d\n", msgLen);
      DEBUG_PRINT("NextAddr: %u\n", mockDataHeader->nextAddress);
      if (msgLen != -1) {
        txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
        uwbSendPacketBlock(&txPacketCache);
      }
    }
    int jitter = (int) (rand() / (float) RAND_MAX * 9) - 4;
    vTaskDelay(DATA_INTERVAL + M2T(jitter));
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(DATA, &rxPacketCache)) {
      computeRouting();
      dispatchRoutingDataMessage(&rxPacketCache);
    }
  }
}

void routingInit() {
  MY_UWB_ADDRESS = getUWBAddress();
  rxQueue = xQueueCreate(ROUTING_RX_QUEUE_SIZE, ROUTING_RX_QUEUE_ITEM_SIZE);

  UWB_Message_Listener_t listener;
  listener.type = DATA;
  listener.rxQueue = rxQueue;
  listener.rxCb = routingRxCallback;
  listener.txCb = routingTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(uwbRoutingTxTask, ADHOC_DECK_ROUTING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRoutingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRoutingRxTask, ADHOC_DECK_ROUTING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRoutingRxTaskHandle); // TODO optimize STACK SIZE
}

void computeRouting() {
  /* Error prevention mechanism */
  if (getTcFloodingTopologyTableSize() == 0) return;
  /* Routing Structures Init */
  routingShortestTreeClear(routingShortestTree);

  Routing_Node_State_Table_Set_t routingNodeStateTableSet;
  routingNodeStateTableSetInit(&routingNodeStateTableSet);

  Routing_Priority_Queue_t routingPriorityQueue;
  routingPriorityQueueInit(&routingPriorityQueue);
  /* Base Case Init */
  set_index_t baseIndex = routingNodeStateTableSetInsert(&routingNodeStateTableSet, MY_UWB_ADDRESS, 0);
  routingPriorityQueuePush(&routingPriorityQueue, &routingNodeStateTableSet.setData[baseIndex].data);
  /* Compute Routing */
  Routing_Node_State_Table_t nodeItem;
  set_index_t nodeIndex;

  while (!isRoutingPriorityQueueEmpty(&routingPriorityQueue)) {
    routingPriorityQueueGetFront(&routingPriorityQueue, &nodeItem);
    routingPriorityQueuePopFront(&routingPriorityQueue);

    if (nodeItem.visited) continue;
    else {
      nodeIndex = findInRoutingNodeStateTableSet(&routingNodeStateTableSet, nodeItem.address);
      routingNodeStateTableSet.setData[nodeIndex].data.visited = true;
    }

    for(uint16_t addressIndex = 0; addressIndex < FLOODING_TOPOLOGY_TABLE_SIZE; addressIndex++) {
      Flooding_Topology_Table_t topologyTable = getTcFloodingTopologyTable(nodeItem.address, addressIndex);
      if (topologyTable.distance != -1) {
        uint16_t srcAddress = nodeItem.address;
        uint16_t dstAddress = addressIndex;
        uint16_t edgeDistance = topologyTable.distance;
        set_index_t dstNodeIndex = findInRoutingNodeStateTableSet(&routingNodeStateTableSet, dstAddress);
        if (dstNodeIndex != -1) {
          if (routingNodeStateTableSet.setData[dstAddress].data.distance > (nodeItem.distance + edgeDistance)) {
            routingNodeStateTableSet.setData[dstAddress].data.distance = nodeItem.distance + edgeDistance;
            routingPriorityQueuePush(&routingPriorityQueue, &routingNodeStateTableSet.setData[dstAddress].data);
            routingShortestTree[dstAddress] = srcAddress;
          }
        } else {
          dstNodeIndex = routingNodeStateTableSetInsert(&routingNodeStateTableSet, dstAddress, nodeItem.distance + edgeDistance);
          routingPriorityQueuePush(&routingPriorityQueue, &routingNodeStateTableSet.setData[dstNodeIndex].data);
          routingShortestTree[dstAddress] = srcAddress;
        }
      }
    }
  }
}

