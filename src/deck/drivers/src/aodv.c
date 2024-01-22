#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "autoconf.h"
#include "debug.h"
#include "system.h"
#include "routing.h"
#include "aodv.h"

static TaskHandle_t aodvRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static uint32_t aodvMsgSeqNumber = 0;
static uint32_t aodvRequestId = 0;

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
  packet.header.length = sizeof(UWB_Packet_t) + sizeof(AODV_RREQ_Message_t);
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
