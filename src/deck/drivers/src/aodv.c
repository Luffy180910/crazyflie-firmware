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
static uint32_t aodvSeqNumber = 0;
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

static int aodvCompareSeqNumber(uint32_t first, uint32_t second) {
  int32_t firstV = (int32_t) first;
  int32_t secondV = (int32_t) second;
  if (firstV == secondV) {
    return 0;
  }
  if (firstV - secondV > 0) {
    return 1;
  }
  return -1;
}

static void sendRREP(AODV_RREQ_Message_t *rreq, Route_Entry_t toOrigin) {
  /*
   * Destination node MUST increment its own sequence number by one if the sequence number in the
   * RREQ packet is equal to that incremented value. Otherwise, the destination does not change
   * its sequence number before generating the  RREP message.
   */
  if (!rreq->flags.U && rreq->destSeqNumber == aodvSeqNumber + 1) {
    aodvSeqNumber++;
  }
  // TODO: check
  UWB_Packet_t packet = {
      .header.type = UWB_AODV_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = toOrigin.nextHop,
      .header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREP_Message_t)
  };
  AODV_RREP_Message_t *rrep = (AODV_RREP_Message_t *) &packet.payload;
  rrep->type = AODV_RREP;
  rrep->hopCount = 0;
  rrep->destAddress = rreq->destAddress;
  rrep->destSeqNumber = aodvSeqNumber;
  rrep->origAddress = toOrigin.destAddress;
  rrep->lifetime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
  uwbSendPacketBlock(&packet);
}

static void sendRREPByIntermediateNode(Route_Entry_t toDest, Route_Entry_t toOrigin) {
  /* Have not implement gratuitous rrep here. */

  // TODO: check
  UWB_Packet_t packet = {
      .header.type = UWB_AODV_MESSAGE,
      .header.srcAddress = uwbGetAddress(),
      .header.destAddress = toOrigin.nextHop,
      .header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREP_Message_t)
  };

  AODV_RREP_Message_t *rrep = (AODV_RREP_Message_t *) &packet.payload;
  rrep->type = AODV_RREP;
  rrep->flags.prefixSize = 0;
  rrep->hopCount = toDest.hopCount;
  rrep->destAddress = toDest.destAddress;
  rrep->destSeqNumber = toDest.destSeqNumber;
  rrep->origAddress = toOrigin.destAddress;
  rrep->lifetime = toDest.expirationTime;

  /* If the node we received a RREQ for is a neighbor we are
   * probably facing a unidirectional link... Better request a RREP-ack
   */
  if (toDest.hopCount == 1) {
    rrep->flags.A = true;
  }

  toDest.precursors = precursorListAdd(toDest.precursors, toOrigin.nextHop);
  toOrigin.precursors = precursorListAdd(toOrigin.precursors, toDest.nextHop);
  routingTableUpdateEntry(routingTable, toDest);
  routingTableUpdateEntry(routingTable, toOrigin);

  uwbSendPacketBlock(&packet);
}

static void aodvProcessRREQ(UWB_Packet_t *packet) {
  AODV_RREQ_Message_t *rreq = (AODV_RREQ_Message_t *) &packet->payload;
  DEBUG_PRINT("aodvProcessRREQ: %u forward RREQ from origin = %u, reqId = %lu, src = %u, dest = %u, destSeq = %lu.\n",
              uwbGetAddress(),
              rreq->origAddress,
              rreq->requestId,
              packet->header.srcAddress,
              rreq->destAddress,
              rreq->destSeqNumber
  );
  if (rreqBufferIsDuplicate(&rreqBuffer, rreq->origAddress, rreq->requestId)) {
    DEBUG_PRINT("aodvProcessRREQ: Discard duplicate rreq origin = %u, reqId = %lu, dest = %u.\n",
                rreq->origAddress,
                rreq->requestId,
                rreq->destAddress
    );
    return;
  } else {
    rreqBufferAdd(&rreqBuffer, rreq->origAddress, rreq->requestId);
  }
  rreq->hopCount++;

  /*  Origin -> Neighbor -> Me -> Dest
   *  When the reverse route is created or updated, the following actions on the route are also
   * carried out:
   *  1. the Originator Sequence Number from the RREQ is compared to the corresponding destination
   * sequence number in the route table entry and copied if greater than the existing value there;
   *  2. the valid sequence number field is set to true;
   *  3. the next hop in the routing table becomes the node from which the RREQ was received
   *  4. the hop count is copied from the Hop Count in the rreq;
   *  5. the Lifetime is updated.
   */

  /* Update route for Me to Origin through Neighbor */
  Route_Entry_t toOrigin = routingTableFindEntry(routingTable, rreq->origAddress);
  if (toOrigin.destAddress == UWB_DEST_EMPTY) {
    toOrigin.destAddress = rreq->origAddress;
    toOrigin.validDestSeqFlag = true;
    toOrigin.destSeqNumber = rreq->origSeqNumber;
    toOrigin.hopCount = rreq->hopCount;
    toOrigin.nextHop = packet->header.srcAddress;
    toOrigin.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableAddEntry(routingTable, toOrigin);
  } else {
    if (toOrigin.validDestSeqFlag) {
      if (aodvCompareSeqNumber(rreq->origSeqNumber, toOrigin.destSeqNumber) > 0) {
        toOrigin.destSeqNumber = rreq->origSeqNumber;
      }
    } else {
      toOrigin.destSeqNumber = rreq->origSeqNumber;
    }
    toOrigin.validDestSeqFlag = true;
    toOrigin.nextHop = packet->header.srcAddress;
    toOrigin.hopCount = rreq->hopCount;
    toOrigin.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableUpdateEntry(routingTable, toOrigin);
  }

  /* Update route for Me to Neighbor */
  Route_Entry_t toNeighbor = routingTableFindEntry(routingTable, packet->header.srcAddress);
  if (toNeighbor.destAddress == UWB_DEST_EMPTY) {
    toNeighbor.destAddress = packet->header.srcAddress;
    toNeighbor.validDestSeqFlag = false;
    toNeighbor.destSeqNumber = rreq->origSeqNumber;
    toNeighbor.hopCount = 1;
    toNeighbor.nextHop = packet->header.srcAddress;
    toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableAddEntry(routingTable, toNeighbor);
  } else {
    toNeighbor.flags.aodvValidRoute = true;
    toNeighbor.validDestSeqFlag = false;
    toNeighbor.destAddress = rreq->origSeqNumber;
    toNeighbor.hopCount = 1;
    toNeighbor.nextHop = packet->header.srcAddress;
    toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
    routingTableUpdateEntry(routingTable, toNeighbor);
  }

  /* A node generates a RREP if either:
   * (i) it is itself the destination,
   */
  if (rreq->destAddress == uwbGetAddress()) {
    sendRREP(rreq, toOrigin);
    return;
  }
  /*
   * (ii) or it has an active route to the destination, the destination sequence number in the
   * node's existing route table entry for the destination is valid and greater than or equal to
   * the Destination Sequence Number of the RREQ, and the "destination only" flag is NOT set.
   */
  Route_Entry_t toDest = routingTableFindEntry(routingTable, rreq->destAddress);
  if (toDest.destAddress != UWB_DEST_EMPTY) {
    /* Drop RREQ that will make a loop. (i.e. A->D but get A->B->C->B->C->B->C) */
    if (toDest.nextHop == packet->header.srcAddress) {
      DEBUG_PRINT(
          "aodvProcessRREQ: %u drop RREQ from origin = %u, reqId = %lu, src = %u, dest = %u, destSeq = %lu, next hop = %u.\n",
          uwbGetAddress(),
          rreq->origAddress,
          rreq->requestId,
          packet->header.srcAddress,
          rreq->destAddress,
          rreq->destSeqNumber,
          toDest.nextHop);
      return;
    }
    /*
     * The Destination Sequence number for the requested destination is set to the maximum of
     * the corresponding value received in the RREQ message, and the destination sequence value
     * currently maintained by the node for the requested destination. However, the forwarding
     * node MUST NOT modify its maintained value for the destination sequence number, even if
     * the value received in the incoming RREQ is larger than the value currently maintained by
     * the forwarding node.
     */
    if (rreq->flags.U || (toDest.validDestSeqFlag && aodvCompareSeqNumber(toDest.destSeqNumber, rreq->destSeqNumber))) {
      if (!rreq->flags.D && toDest.flags.aodvValidRoute) {
        toOrigin = routingTableFindEntry(routingTable, rreq->origAddress);
        sendRREPByIntermediateNode(toDest, toOrigin);
        return;
      }
      rreq->destSeqNumber = toDest.destSeqNumber;
      rreq->flags.U = false;
    }

    DEBUG_PRINT("aodvProcessRREQ: %u forward RREQ from origin = %u, reqId = %lu, src = %u, dest = %u, destSeq = %lu.\n",
                uwbGetAddress(),
                rreq->origAddress,
                rreq->requestId,
                packet->header.srcAddress,
                rreq->destAddress,
                rreq->destSeqNumber
    );
    /* Forward this RREQ message. */
    packet->header.srcAddress = uwbGetAddress();
    packet->header.destAddress = UWB_DEST_ANY;
    uwbSendPacketBlock(packet);
  }
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
  rreqMsg->origSeqNumber = aodvSeqNumber++;
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
      /* Update route for Me to Neighbor (reverse route) */
      Route_Entry_t toNeighbor = routingTableFindEntry(routingTable, rxPacketCache.header.srcAddress);
      if (toNeighbor.destAddress != UWB_DEST_EMPTY && toNeighbor.validDestSeqFlag && toNeighbor.hopCount == 1) {
        toNeighbor.expirationTime = MAX(toNeighbor.expirationTime, xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME));
        routingTableUpdateEntry(routingTable, toNeighbor);
      } else {
        toNeighbor.destAddress = rxPacketCache.header.srcAddress;
        toNeighbor.nextHop = rxPacketCache.header.srcAddress;
        toNeighbor.destSeqNumber = 0;
        toNeighbor.hopCount = 1;
        toNeighbor.validDestSeqFlag = false;
        toNeighbor.expirationTime = xTaskGetTickCount() + M2T(ROUTING_TABLE_HOLD_TIME);
        routingTableAddEntry(routingTable, toNeighbor);
      }

      uint8_t msgType = rxPacketCache.payload[0];
      switch (msgType) {
        case AODV_RREQ:aodvProcessRREQ(&rxPacketCache);
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
