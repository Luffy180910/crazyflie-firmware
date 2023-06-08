#define DEBUG_MODULE "SNIFFER"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "adhocdeck.h"
#include "usb.h"
#include "debug.h"
#include "sniffer.h"

static TaskHandle_t snifferTaskHandle = 0;
static QueueHandle_t rxQueue;

static void snifferTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;
  rxPacketCache.header.length = 0;

  while (1) {
    // TODO 添加分隔符
    if (uwbReceivePacketWait(SNIFFER, &rxPacketCache, 1000)) {
      uint16_t msgSize = rxPacketCache.header.length - sizeof(Packet_Header_t);
      DEBUG_PRINT("type %d, len = %u \n", rxPacketCache.header.type, msgSize);
      uint8_t *pointer = rxPacketCache.payload;
      int remain = msgSize;
      while (remain > 0) {
        int sizeToSend = remain > USB_RX_TX_PACKET_SIZE ? USB_RX_TX_PACKET_SIZE : remain;
        usbSendData(sizeToSend, pointer);
        pointer += sizeToSend;
        remain -= sizeToSend;
        DEBUG_PRINT("send data, size = %d, remain = %d \n", sizeToSend, remain);
      }
    }
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    vTaskDelay(1); // TODO pick proper timespan
  }
}

void snifferInit() {
  rxQueue = xQueueCreate(SNIFFER_RX_QUEUE_SIZE, SNIFFER_RX_QUEUE_ITEM_SIZE);

  UWB_Message_Listener_t listener;
  listener.type = SNIFFER;
  listener.rxQueue = rxQueue;
  listener.rxCb = NULL;
  listener.txCb = NULL;
  uwbRegisterListener(&listener);

  xTaskCreate(snifferTask, ADHOC_DECK_SNIFFER_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &snifferTaskHandle);
}

