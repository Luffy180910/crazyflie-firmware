#include "FreeRTOS.h"
#include "app.h"
#include "debug.h"
#include "task.h"

#include "routing.h"


static TaskHandle_t appTxTaskHandle;
static TaskHandle_t appRxTaskHandle;

static void appTxTask() {
  UWB_Data_Packet_t dataTxPacket;
  dataTxPacket.header.type = UWB_DATA_MESSAGE_COMMAND;
  dataTxPacket.header.srcAddress = uwbGetAddress();
  dataTxPacket.header.destAddress = uwbGetAddress();
  dataTxPacket.header.ttl = 2;
  dataTxPacket.header.length = sizeof(UWB_Data_Packet_Header_t);
  while (1) {
    DEBUG_PRINT("appTxTask\n");
    vTaskDelay(M2T(1000));
  }
}

static void appRxTask() {
  while (1) {
    DEBUG_PRINT("appRxTask\n");
    vTaskDelay(M2T(1000));
  }
}

void appMain() {
  xTaskCreate(appTxTask, "ADHOC_ROUTING_TEST_TX", UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &appTxTaskHandle);
  xTaskCreate(appRxTask, "ADHOC_ROUTING_TEST_RX", UWB_TASK_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &appRxTaskHandle);
}
