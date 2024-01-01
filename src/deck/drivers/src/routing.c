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

#include "adhocdeck.h"
#include "dwTypes.h"
#include "libdw3000.h"
#include "dw3000.h"
#include "routing.h"

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static QueueHandle_t rxQueue;
static int seqNumber = 1;

/* log block */
#define MAX_NEIGHBOR_ADDRESS 20
#define DEBUG_PRINT_INTERVAL 1000

double PACKET_LOSS_RATE[MAX_NEIGHBOR_ADDRESS + 1] = {0};
uint32_t RECEIVE_COUNT[MAX_NEIGHBOR_ADDRESS + 1] = {0};
uint32_t LOSS_COUNT[MAX_NEIGHBOR_ADDRESS + 1] = {0};
uint32_t LAST_RECEIVED_SEQ[MAX_NEIGHBOR_ADDRESS + 1] = {0};

void routingRxCallback(void *parameters) {
//  DEBUG_PRINT("routingRxCallback \n");
}

void routingTxCallback(void *parameters) {
//  DEBUG_PRINT("routingTxCallback \n");
}

int generateRoutingDataMessage(MockData_t *message) {
  int msgLen = sizeof(MockData_t);
  message->seqNumber = seqNumber++;
  message->srcAddress = getUWBAddress();
  return msgLen;
}

static void processRoutingDataMessage(UWB_Packet_t *packet) {
  MockData_t *mockData = (MockData_t *) packet->payload;
  ASSERT(mockData->srcAddress <= MAX_NEIGHBOR_ADDRESS);

  uint16_t neighborAddress = mockData->srcAddress;
  uint32_t curSeqNumber = mockData->seqNumber;
  uint32_t lastSeqNumber = LAST_RECEIVED_SEQ[neighborAddress];

  // if neighbor has rebooted, reset corresponding log data
  if (curSeqNumber < lastSeqNumber) {
    // reset log data
    PACKET_LOSS_RATE[neighborAddress] = 0.0;
    RECEIVE_COUNT[neighborAddress] = 0;
    LOSS_COUNT[neighborAddress] = 0;
    LAST_RECEIVED_SEQ[neighborAddress] = 0;
    lastSeqNumber = 0;
  }

  if (lastSeqNumber != 0) {
      LOSS_COUNT[neighborAddress] += curSeqNumber - lastSeqNumber - 1;
  }

  RECEIVE_COUNT[neighborAddress]++;
  LAST_RECEIVED_SEQ[neighborAddress] = curSeqNumber;
  PACKET_LOSS_RATE[neighborAddress] = (double) LOSS_COUNT[neighborAddress] / (RECEIVE_COUNT[neighborAddress] + LOSS_COUNT[neighborAddress]);

//  DEBUG_PRINT("received routing data from %d, seq number = %d \n", mockData->srcAddress, mockData->seqNumber);
}

static void printDebugInfo() {
  DEBUG_PRINT("------------------------\n");
  double minPacketLossRate = 0.0;
  double maxPacketLossRate = 0.0;
  double averagePacketLossRate = 0.0;
  int neighborCount = 0;
  uint32_t receiveCount = 0;
  uint32_t lossCount = 0;

  for (int i = 0; i <= MAX_NEIGHBOR_ADDRESS; i++) {
    if (RECEIVE_COUNT[i] == 0) {
      continue;
    }
    neighborCount++;
    if (PACKET_LOSS_RATE[i] < minPacketLossRate) {
      minPacketLossRate = PACKET_LOSS_RATE[i];
    }
    if (PACKET_LOSS_RATE[i] > maxPacketLossRate) {
      maxPacketLossRate = PACKET_LOSS_RATE[i];
    }
    averagePacketLossRate += PACKET_LOSS_RATE[i];
    receiveCount += RECEIVE_COUNT[i];
    lossCount += LOSS_COUNT[i];

    DEBUG_PRINT("neighbor: %d, loss count: %lu, receive count: %lu, total count : %lu, packet loss rate: %.2f \n",
                i, LOSS_COUNT[i], RECEIVE_COUNT[i], LOSS_COUNT[i] + RECEIVE_COUNT[i], PACKET_LOSS_RATE[i] * 100);
  }
  DEBUG_PRINT("min: %.2f, max: %.2f, average: %.2f, total loss: %lu, total received: %lu, neighbor count: %d \n",
              minPacketLossRate * 100, maxPacketLossRate * 100, averagePacketLossRate * 100 / neighborCount, lossCount, receiveCount, neighborCount);
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  TickType_t LAST_PRINT_TIME = xTaskGetTickCount();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = DATA;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int msgLen = generateRoutingDataMessage((MockData_t *) &txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);

    if (LAST_PRINT_TIME + M2T(DEBUG_PRINT_INTERVAL) < xTaskGetTickCount()) {
      printDebugInfo();
      LAST_PRINT_TIME = xTaskGetTickCount();
    }

    vTaskDelay(M2T(ROUTING_TX_INTERVAL));
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(DATA, &rxPacketCache)) {
      processRoutingDataMessage(&rxPacketCache);
//      vTaskDelay(M2T(1));
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

