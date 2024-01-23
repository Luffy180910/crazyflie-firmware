#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "autoconf.h"
#include "debug.h"
#include "system.h"
#include "routing.h"
#include "aodv.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct {
  UWB_Address_t origAddress;
  uint32_t requestId;
} RREQ_Buffer_Item_t;

typedef struct {
  uint8_t index;
  RREQ_Buffer_Item_t items[AODV_RREQ_BUFFER_SIZE_MAX];
} RREQ_Buffer_t;

static TaskHandle_t aodvRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static uint32_t aodvMsgSeqNumber = 0;
static uint32_t aodvRequestId = 0;
static RREQ_Buffer_t rreqBuffer;
static Routing_Table_t *routingTable;

static void rreqBufferInit(RREQ_Buffer_t *buffer) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    buffer->items[i].requestId = 0;
    buffer->items[i].origAddress = UWB_DEST_EMPTY;
  }
}

static void rreqBufferAdd(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  RREQ_Buffer_Item_t item = {.origAddress = neighborAddress, .requestId = requestId};
  buffer->items[buffer->index] = item;
  buffer->index = (buffer->index + 1) % AODV_RREQ_BUFFER_SIZE_MAX;
}

static int rreqBufferFind(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    if (neighborAddress == buffer->items[i].origAddress && requestId == buffer->items[i].requestId) {
      return i;
    }
  }
  return -1;
}

static bool rreqBufferIsDuplicate(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    if (neighborAddress == buffer->items[i].origAddress && requestId == buffer->items[i].requestId) {
      return true;
    }
  }
  return false;
}

static uint64_t precursorListAdd(uint64_t precursors, UWB_Address_t address) {
  return precursors | (1ULL << address);
}

static uint64_t precursorListRemove(uint64_t precursors, UWB_Address_t address) {
  return precursors & ~(1ULL << address);
}

static void aodvProcessRREQ(AODV_RREQ_Message_t *message) {
  if (rreqBufferIsDuplicate(&rreqBuffer, message->origAddress, message->requestId)) {
    DEBUG_PRINT("aodvProcessRREQ: Discard duplicate rreq message originator = %u, dest = %u, reqId = %lu.\n",
                message->origAddress,
                message->destAddress,
                message->requestId);
    return;
  }
//  rreqBufferAdd(&rreqBuffer, message->origAddress, message->requestId);
}

static void aodvProcessRREP(AODV_RREP_Message_t *message) {
// TODO
}

static void aodvProcessRERR(AODV_RERR_Message_t *message) {
// TODO
}

static void aodvProcessRREPACK(AODV_RREP_ACK_Message_t *message) {
// TODO
}

// TODO: test
void aodvDiscoveryRoute(UWB_Address_t destAddress) {
  DEBUG_PRINT("aodvDiscoveryRoute: Try to discovery route to %u.\n", destAddress);
  UWB_Packet_t packet;
  packet.header.type = UWB_AODV_MESSAGE;
  packet.header.srcAddress = uwbGetAddress();
  packet.header.destAddress = UWB_DEST_ANY;
  packet.header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREQ_Message_t);

  AODV_RREQ_Message_t *rreqMsg = (AODV_RREQ_Message_t *) &packet.payload;
  rreqMsg->type = AODV_RREQ;
  rreqMsg->hopCount = 1;
  rreqMsg->requestId = aodvRequestId++;
  rreqMsg->origAddress = uwbGetAddress();
  rreqMsg->origSeqNumber = aodvMsgSeqNumber++;
  rreqMsg->destAddress = destAddress;

  Route_Entry_t routeEntry = routingTableFindEntry(routingTable, destAddress);
  /* Find corresponding route entry for destAddress. */
  if (routeEntry.destAddress != UWB_DEST_EMPTY) {
    if (routeEntry.validDestSeqFlag) {
      rreqMsg->destSeqNumber = routeEntry.destSeqNumber;
    } else {
      rreqMsg->destSeqNumber = 0;
      rreqMsg->flags.U = true;
    }
    routeEntry.expirationTime = xTaskGetTickCount() + M2T(AODV_ROUTE_DISCOVERY_TIME);
    routingTableUpdateEntry(routingTable, routeEntry);
  } else {
    rreqMsg->flags.U = true;
    rreqMsg->destSeqNumber = 0;
    /* Add new route entry for destAddress. */
    Route_Entry_t newRouteEntry = emptyRouteEntry();
    newRouteEntry.destAddress = destAddress;
    newRouteEntry.expirationTime = xTaskGetTickCount() + M2T(AODV_ROUTE_DISCOVERY_TIME);
    routingTableAddEntry(routingTable, newRouteEntry);
  }

  if (AODV_GRATUITOUS_REPLY) {
    rreqMsg->flags.G = true;
  }

  if (AODV_DESTINATION_ONLY) {
    rreqMsg->flags.D = true;
  }

  DEBUG_PRINT("aodvDiscoveryRoute: Send aodv rreq, reqId = %lu, orig = %u, origSeq = %lu, dest = %u, destSeq = %lu.\n",
              rreqMsg->requestId,
              rreqMsg->origAddress,
              rreqMsg->origSeqNumber,
              rreqMsg->destAddress,
              rreqMsg->destSeqNumber
  );

  uwbSendPacketBlock(&packet);
}

void aodvRxCallback(void *parameters) {
  DEBUG_PRINT("aodvRxCallback\n");
}

void aodvTxCallback(void *parameters) {
  DEBUG_PRINT("aodvTxCallback\n");
}

static void aodvRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(UWB_AODV_MESSAGE, &rxPacketCache)) {
      xSemaphoreTake(routingTable->mu, portMAX_DELAY);
      DEBUG_PRINT("aodvRxTask: receive aodv message from neighbor %u.\n", rxPacketCache.header.srcAddress);
      /* Update route to neighbor (reverse route) */
      Route_Entry_t routeEntry = routingTableFindEntry(routingTable, rxPacketCache.header.srcAddress);
      if (routeEntry.destAddress != UWB_DEST_EMPTY && routeEntry.validDestSeqFlag && routeEntry.hopCount == 1) {
        routeEntry.expirationTime = MAX(routeEntry.expirationTime, xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME));
        routingTableUpdateEntry(routingTable, routeEntry);
      } else {
        routeEntry = emptyRouteEntry();
        routeEntry.destAddress = rxPacketCache.header.srcAddress;
        routeEntry.nextHop = rxPacketCache.header.srcAddress;
        routeEntry.hopCount = 1;
        routeEntry.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
        routingTableAddEntry(routingTable, routeEntry);
      }

      uint8_t msgType = rxPacketCache.payload[0];
      switch (msgType) {
        case AODV_RREQ:aodvProcessRREQ((AODV_RREQ_Message_t *) rxPacketCache.payload);
          break;
        case AODV_RREP:aodvProcessRREP((AODV_RREP_Message_t *) rxPacketCache.payload);
          break;
        case AODV_RERR:aodvProcessRERR((AODV_RERR_Message_t *) rxPacketCache.payload);
          break;
        case AODV_RREP_ACK:aodvProcessRREPACK((AODV_RREP_ACK_Message_t *) rxPacketCache.payload);
          break;
        default:DEBUG_PRINT("aodvRxTask: Receive unknown aodv message type %u from %u, discard.\n", msgType,
                            rxPacketCache.header.srcAddress);
      }
      xSemaphoreGive(routingTable->mu);
    }
    vTaskDelay(M2T(1));
  }

}

void aodvInit() {
  rxQueue = xQueueCreate(AODV_RX_QUEUE_SIZE, AODV_RX_QUEUE_ITEM_SIZE);
  rreqBufferInit(&rreqBuffer);
  routingTable = getGlobalRoutingTable();

  UWB_Message_Listener_t listener;
  listener.type = UWB_AODV_MESSAGE;
  listener.rxQueue = rxQueue;
  listener.rxCb = aodvRxCallback;
  listener.txCb = aodvTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(aodvRxTask,
              ADHOC_DECK_AODV_RX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &aodvRxTaskHandle);
}
