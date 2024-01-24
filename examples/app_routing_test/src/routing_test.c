#include "FreeRTOS.h"
#include "app.h"
#include "debug.h"
#include "task.h"

#include "routing.h"

/* Queue Constants */
#define APP_RX_QUEUE_SIZE 5
#define APP_RX_QUEUE_ITEM_SIZE sizeof(UWB_Data_Packet_t)

#define APP_TX_INTERVAL 1000

static TaskHandle_t appTxTaskHandle;
static TaskHandle_t appRxTaskHandle;
static QueueHandle_t rxQueue;

static void appTxTask() {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_COMMAND;
  dataTxPacket.header.srcAddress = uwbGetAddress();
  dataTxPacket.header.destAddress = 5;
  dataTxPacket.header.ttl = 15;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
  while (1) {
    vTaskDelay(M2T(APP_TX_INTERVAL));
    uwbSendDataPacketBlock(&dataTxPacket);
  }
}

static void appRxTask() {
  UWB_Data_Packet_t dataRxPacket;
  while (1) {
    if (uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_COMMAND, &dataRxPacket)) {
      DEBUG_PRINT("appRxTask: %u Received data packet from %u.\n",
                  uwbGetAddress(),
                  dataRxPacket.header.srcAddress
      );
    }
    vTaskDelay(M2T(1));
  }
}

void appMain() {
  rxQueue = xQueueCreate(APP_RX_QUEUE_SIZE, APP_RX_QUEUE_ITEM_SIZE);
  UWB_Data_Packet_Listener_t listener = {
    .type = UWB_DATA_MESSAGE_COMMAND,
    .rxQueue = rxQueue
  };
  uwbRegisterDataPacketListener(&listener);
  xTaskCreate(appTxTask, "ADHOC_ROUTING_TEST_TX", UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &appTxTaskHandle);
  xTaskCreate(appRxTask, "ADHOC_ROUTING_TEST_RX", UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &appRxTaskHandle);
}
