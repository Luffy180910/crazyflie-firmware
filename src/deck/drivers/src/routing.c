#define DEBUG_MODULE "ROUTING"

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
#include "math.h"

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "routing.h"

#define DGC_DBG_DECISION_ID 0x30060
#define DGC_DBG_DECISION_MASK 0x7FFFFFFF

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static int seqNumber = 1;
static double A = 120.7;

void routingRxCallback(void *parameters) {
  DEBUG_PRINT("routingRxCallback \n");
}

void routingTxCallback(void *parameters) {
  DEBUG_PRINT("routingTxCallback \n");
}

int generateRoutingDataMessage(MockData_t *message) {
  int msgLen = sizeof(MockData_t);
  message->seqNumber = seqNumber++;
  return msgLen;
}

static void processRoutingDataMessage(UWB_Packet_t *packet) {
  MockData_t *mockData = (MockData_t *) packet->payload;

  // TEST: Power estimation
  uint32_t F1 = dwt_read32bitreg(IP_DIAG_2_ID) & IP_DIAG_2_MASK; // F1
  uint32_t F2 = dwt_read32bitreg(IP_DIAG_3_ID) & IP_DIAG_3_MASK; // F2
  uint32_t F3 = dwt_read32bitreg(IP_DIAG_4_ID) & IP_DIAG_4_MASK; // F3
  uint32_t N = dwt_read32bitreg(IP_DIAG_12_ID) & IP_DIAG_12_MASK; // N
  uint32_t D = (dwt_read32bitreg(DGC_DBG_DECISION_ID) & DGC_DBG_DECISION_MASK) >> 28; // D
  uint32_t C = dwt_read32bitreg(IP_DIAG_1_ID) & IP_DIAG_1_MASK; // C

  uint16_t rx_tune_en = dwt_read16bitoffsetreg(DGC_CFG_ID, DGC_CFG_RX_TUNE_EN_BIT_OFFSET) & DGC_CFG_RX_TUNE_EN_BIT_MASK; // the DGC_DECISION

  double FPP = 0;
  double RX_Level = 0;
  if (rx_tune_en) {
    FPP = 10 * log10((F1*F1 + F2*F2 + F3*F3) / N*N) + (6 * D) - A;
    RX_Level = 10 * log10((C * pow(2, 21)) / N*N) + (6 * D) - A;
  } else {
    FPP = 10 * log10((F1*F1 + F2*F2 + F3*F3) / N*N) - A;
    RX_Level = 10 * log10((C * pow(2, 21)) / N*N) - A;
  }

  DEBUG_PRINT("FPP: %lld, RX_Level: %lld \n", FPP, RX_Level);
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = DATA;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int msgLen = generateRoutingDataMessage((MockData_t *) &txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);

    // TEST: Power adjustment
    while (!dwt_checkidlerc()) {};
    setTxConfigPower(0xfefefefe);

    vTaskDelay(M2T(2000));
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(DATA, &rxPacketCache)) {
      processRoutingDataMessage(&rxPacketCache);
    }
  }
}

void routingInit() {
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

