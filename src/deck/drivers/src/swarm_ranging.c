#include <math.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "adhocdeck.h"
#include "swarm_ranging.h"
#include "ranging_struct.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static int rangingSeqNumber = 1;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};

void rangingRxCallback(void *parameters) {
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *) parameters;

  dwTime_t rxTime;
  dwt_readrxtimestamp((uint8_t *) &rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *) packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters) {
  dwTime_t txTime;
  dwt_readtxtimestamp((uint8_t *) &txTime.raw);
  Timestamp_Tuple_t timestamp = {.timestamp = txTime, .seqNumber = rangingSeqNumber};
  updateTfBuffer(timestamp);
}

int16_t getDistance(uint16_t neighborAddress) {
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

void setDistance(uint16_t neighborAddress, int16_t distance) {
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  distanceTowards[neighborAddress] = distance;
}

static void uwbRangingTxTask(void *parameters) {
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = RANGING;
  Ranging_Message_t *rangingMessage = &txPacketCache.payload;
//  txPacketCache.header.mac = ? TODO init mac header
  while (true) {
    int taskDelay = generateRangingMessage(rangingMessage);
    txPacketCache.header.length = sizeof(Packet_Header_t) + rangingMessage->header.msgLength;
    uwbSendPacketBlock(&txPacketCache);
    vTaskDelay(taskDelay);
  }
}

static void uwbRangingRxTask(void *parameters) {
  systemWaitStart();

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true) {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY)) {
//      DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
    }
  }
}

void rangingInit() {
  MY_UWB_ADDRESS = getUWBAddress();
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  rxQueue = xQueueCreate(RANGING_RX_QUEUE_SIZE, RANGING_RX_QUEUE_ITEM_SIZE);
  rangingTableSetInit(&rangingTableSet);

  listener.type = RANGING;
  listener.rxQueue = NULL; // handle rxQueue in swarm_ranging.c instead of adhocdeck.c
  listener.rxCb = rangingRxCallback;
  listener.txCb = rangingTxCallback;
  uwbRegisterListener(&listener);

  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  xTaskCreate(uwbRangingTxTask, ADHOC_DECK_RANGING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRangingRxTask, ADHOC_DECK_RANGING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRangingRxTaskHandle); // TODO optimize STACK SIZE
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp) {
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  /* Handle new neighbor */
  if (neighborIndex == -1) {
    if (rangingTableSet.freeQueueEntry == -1) {
      /* Ranging table set is full, ignore this ranging message. */
      DEBUG_PRINT("Ranging table is full, cannot handle new neighbor %d\n", neighborAddress);
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
  /* Update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;
  /* Update latest received timestamp of this neighbor */
  neighborRangingTable->latestReceived = neighborRangingTable->Re;
  /* Update expiration time of this neighbor */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;
  /* Update Tr and Rr if possible. */
  Timestamp_Tuple_t neighborTr = rangingMessage->header.lastTxTimestamp;
  if (neighborTr.timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full
      && neighborTr.seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber) {
    rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                             neighborTr,
                             neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
  }

  /* Try to find corresponding Rf for MY_UWB_ADDRESS. */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0, .seqNumber = 0};
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16))) {
    /* Retrieve body unit from received ranging message. */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++) {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress()) {
        neighborRf = rangingMessage->bodyUnits[i].timestamp;
        break;
      }
    }
  }

  /* Trigger event handler according to Rf */
  if (neighborRf.timestamp.full) {
    neighborRangingTable->Rf = neighborRf;
    rangingTableOnEvent(neighborRangingTable, RX_Rf);
  } else {
    rangingTableOnEvent(neighborRangingTable, RX_NO_Rf);
  }

  #ifdef ENABLE_DYNAMIC_RANGING_PERIOD
  /* update period according to distance and velocity */
  neighborRangingTable->period = M2T(DYNAMIC_RANGING_COEFFICIENT * (neighborRangingTable->distance / rangingMessage->header.velocity));
  /* bound ranging period between RANGING_PERIOD_MIN and RANGING_PERIOD_MAX */
  neighborRangingTable->period = MAX(neighborRangingTable->period, M2T(RANGING_PERIOD_MIN));
  neighborRangingTable->period = MIN(neighborRangingTable->period, M2T(RANGING_PERIOD_MAX));
  #endif
}

Time_t generateRangingMessage(Ranging_Message_t *rangingMessage) {
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  rangingTableSetClearExpire(&rangingTableSet);
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  Time_t curTime = xTaskGetTickCount();
  /* Using the default RANGING_PERIOD when DYNAMIC_RANGING_PERIOD is not enabled. */
  Time_t taskDelay = M2T(RANGING_PERIOD);
  /* Generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next) {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER) {
      break;
    }
    if (table->state == S1 || table->latestReceived.timestamp.full) {
      #ifdef ENABLE_DYNAMIC_RANGING_PERIOD
      /* Only include timestamps with expected delivery time less or equal than current time. */
      if (curTime < table->nextExpectedDeliveryTime) {
        continue;
      }
      table->nextExpectedDeliveryTime = curTime + table->period;
      /* Change task delay dynamically, may increase packet loss rate since ranging period now is determined
       * by the minimum expected delivery time.
       */
      taskDelay = MIN(taskDelay, table->nextExpectedDeliveryTime - curTime);
      /* Bound the dynamic task delay between RANGING_PERIOD_MIN and RANGING_PERIOD */
      taskDelay = MAX(RANGING_PERIOD_MIN, taskDelay);
      #endif
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that latestReceived is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->latestReceived;
      bodyUnitNumber++;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
      rangingTableOnEvent(table, TX_Tf);
    }
  }
  /* Generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  rangingMessage->header.lastTxTimestamp = getLatestTxTimestamp();
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short) (velocity * 100);
  return taskDelay;
}

LOG_GROUP_START(Ranging)
        LOG_ADD(LOG_INT16, distTo1, distanceTowards + 1)
        LOG_ADD(LOG_INT16, distTo2, distanceTowards + 2)
        LOG_ADD(LOG_INT16, distTo3, distanceTowards + 3)
        LOG_ADD(LOG_INT16, distTo4, distanceTowards + 4)
        LOG_ADD(LOG_INT16, distTo5, distanceTowards + 5)
        LOG_ADD(LOG_INT16, distTo6, distanceTowards + 6)
        LOG_ADD(LOG_INT16, distTo7, distanceTowards + 7)
        LOG_ADD(LOG_INT16, distTo8, distanceTowards + 8)
        LOG_ADD(LOG_INT16, distTo9, distanceTowards + 9)
        LOG_ADD(LOG_INT16, distTo10, distanceTowards + 10)
LOG_GROUP_STOP(Ranging)
