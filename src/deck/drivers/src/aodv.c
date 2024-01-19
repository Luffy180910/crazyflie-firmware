#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "system.h"
#include "routing.h"
#include "aodv.h"

static TaskHandle_t aodvTxTaskHandle = 0;
static TaskHandle_t aodvRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static uint32_t seqNumber = 0;

void aodvRxCallback(void *parameters) {
  DEBUG_PRINT("aodvRxCallback\n");
}

void aodvTxCallback(void *parameters) {
  DEBUG_PRINT("aodvTxCallback\n");
}

static void aodvTxTask(void *parameters) {
  systemWaitStart();

  UWB_Data_Packet_t txDataPacketCache;
  txDataPacketCache.header.type = UWB_DATA_MESSAGE_AODV;
  txDataPacketCache.header.srcAddress = uwbGetAddress();

  while (true) {
    // TODO: modify txDataPacketCache.header.destAddress
    txDataPacketCache.header.seqNumber = seqNumber++;
    txDataPacketCache.header.length = sizeof(UWB_Data_Packet_Header_t);
    uwbSendDataPacketBlock(&txDataPacketCache);
    vTaskDelay(M2T(1000));
  }
}

static void aodvRxTask(void *parameters) {
  systemWaitStart();

  UWB_Data_Packet_t rxDataPacketCache;

  while (true) {
    if (uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_AODV, &rxDataPacketCache)) {
      DEBUG_PRINT("aodvRxTask: receive aodv message from %u, seq = %lu.\n",
                  rxDataPacketCache.header.srcAddress,
                  rxDataPacketCache.header.seqNumber);
    }
    vTaskDelay(M2T(1));
  }

}

void aodvInit() {
  rxQueue = xQueueCreate(AODV_RX_QUEUE_SIZE, AODV_RX_QUEUE_ITEM_SIZE);
  UWB_Data_Packet_Listener_t listener;
  listener.type = UWB_DATA_MESSAGE_AODV;
  listener.rxQueue = rxQueue;
  listener.rxCb = aodvRxCallback;
  listener.txCb = aodvTxCallback;
  uwbRegisterDataPacketListener(&listener);

  xTaskCreate(aodvTxTask,
              ADHOC_DECK_AODV_TX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &aodvTxTaskHandle);
  xTaskCreate(aodvRxTask,
              ADHOC_DECK_AODV_RX_TASK_NAME,
              UWB_TASK_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &aodvRxTaskHandle);
}
