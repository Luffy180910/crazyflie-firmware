#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "autoconf.h"
#include "debug.h"
#include "system.h"
#include "routing.h"
#include "aodv.h"

typedef struct {
  UWB_Address_t neighborAddress;
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

static void rreqBufferInit(RREQ_Buffer_t *buffer) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    buffer->items[i].requestId = 0;
    buffer->items[i].neighborAddress = UWB_DEST_EMPTY;
  }
}

static void rreqBufferAdd(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  RREQ_Buffer_Item_t item = {.neighborAddress = neighborAddress, .requestId = requestId};
  buffer->items[buffer->index] = item;
  buffer->index = (buffer->index + 1) % AODV_RREQ_BUFFER_SIZE_MAX;
}

static int rreqBufferFind(RREQ_Buffer_t *buffer, UWB_Address_t neighborAddress, uint32_t requestId) {
  for (int i = 0; i < AODV_RREQ_BUFFER_SIZE_MAX; i++) {
    if (neighborAddress == buffer->items[i].neighborAddress && requestId == buffer->items[i].requestId) {
      return i;
    }
  }
  return -1;
}

static uint64_t precursorListAdd(uint64_t precursors, UWB_Address_t address) {
  return precursors | (1ULL << address);
}

static uint64_t precursorListRemove(uint64_t precursors, UWB_Address_t address) {
  return precursors & ~(1ULL << address);
}

static void aodvProcessRREQ(AODV_RREQ_Message_t *message) {
// TODO
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

void aodvDiscoveryRoute(UWB_Address_t destAddress) {
  DEBUG_PRINT("aodvDiscoveryRoute: Try to discovery route to %u.\n", destAddress);
  UWB_Packet_t packet;
  packet.header.type = UWB_AODV_MESSAGE;
  packet.header.srcAddress = uwbGetAddress();
  packet.header.destAddress = UWB_DEST_ANY;
  packet.header.length = sizeof(UWB_Packet_Header_t) + sizeof(AODV_RREQ_Message_t);
  AODV_RREQ_Message_t *rreqMsg = (AODV_RREQ_Message_t *) &packet.payload;
  // TODO: check
  rreqMsg->flags.U = true;
  rreqMsg->type = AODV_RREQ;
  rreqMsg->hopCount = 0;
  rreqMsg->requestId = ++aodvRequestId;
  rreqMsg->destAddress = destAddress;
  rreqMsg->destSeqNumber = 0;
  rreqMsg->origAddress = uwbGetAddress();
  rreqMsg->origSeqNumber = ++aodvMsgSeqNumber;
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
      DEBUG_PRINT("aodvRxTask: receive aodv message from %u.\n", rxPacketCache.header.srcAddress);
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
        default:DEBUG_PRINT("aodvRxTask: Receive unknown aodv message type %u.\n", msgType);
      }
    }
    vTaskDelay(M2T(1));
  }

}

void aodvInit() {
  rxQueue = xQueueCreate(AODV_RX_QUEUE_SIZE, AODV_RX_QUEUE_ITEM_SIZE);
  rreqBufferInit(&rreqBuffer);

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
