#define DEBUG_MODULE "swarmranging"
#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "assert.h"
#include "adhocdeck.h"
#include "ranging_struct.h"
#include "swarm_ranging.h"
#include <stdlib.h>

static uint16_t MY_UWB_ADDRESS;

static QueueHandle_t rxQueue;
static Ranging_Table_Set_t rangingTableSet;
static UWB_Message_Listener_t listener;
static TaskHandle_t uwbRangingTxTaskHandle = 0;
static TaskHandle_t uwbRangingRxTaskHandle = 0;

static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};
static int TfBufferIndex = 0;
static int rangingSeqNumber = 1;

static logVarId_t idVelocityX, idVelocityY, idVelocityZ;
static float velocity;

int16_t distanceTowards[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = -1};

/*-------------------------------------------------*/
int16_t startStatistic = 0; // 等于1开始统计，等于2结束统计
int16_t jitter = 0;
uint16_t TX_PERIOD_IN_MS = 20;
static bool firstStatisticSuccRx[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = true};
static bool firstStatisticSuccRanging[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = true};

uint16_t lastSuccRangingSeq[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE]=0};  // 上次邻居成功测距的序号，辅助
uint16_t lastSuccRxPacketSeq[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE]=0}; // 上次邻居成功测距的序号，辅助
uint16_t continuousLossPacketCount[RANGING_TABLE_SIZE + 1][MAX_STATISTIC_LOSS_NUM + 1]={0};  // [i][j],两次成功收包j代表间隔的次数，值就是事件发生的次数
uint16_t continuousRangingFailCount[RANGING_TABLE_SIZE + 1][MAX_STATISTIC_LOSS_NUM + 1]={0}; // [i][j],两次成功测距j代表间隔的次数，值就是事件发生的次数
uint16_t rxPacketCount[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE]=0};                                    // 收到其他无人机数据包的次数
uint16_t rangingSuccCount[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE]=0};                                 // 与其他无人机成功测距的次数


int16_t getStartStatistic(){
  return startStatistic;
}

int16_t getJitter(){
  return jitter;
}

uint16_t getPeriod(){
  return TX_PERIOD_IN_MS;
}

/*-----------------------------------------------*/


void rangingRxCallback(void *parameters)
{
  // DEBUG_PRINT("rangingRxCallback \n");

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  UWB_Packet_t *packet = (UWB_Packet_t *)parameters;

  dwTime_t rxTime = {0};
  dwt_readrxtimestamp((uint8_t *)&rxTime.raw);
  Ranging_Message_With_Timestamp_t rxMessageWithTimestamp;
  rxMessageWithTimestamp.rxTime = rxTime;
  Ranging_Message_t *rangingMessage = (Ranging_Message_t *)packet->payload;
  rxMessageWithTimestamp.rangingMessage = *rangingMessage;

  xQueueSendFromISR(rxQueue, &rxMessageWithTimestamp, &xHigherPriorityTaskWoken);
}

void rangingTxCallback(void *parameters)
{
  dwTime_t txTime = {0};
  dwt_readtxtimestamp((uint8_t *)&txTime.raw);
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex].seqNumber = rangingSeqNumber;
  TfBuffer[TfBufferIndex].timestamp = txTime;
}

int16_t getDistance(uint16_t neighborAddress)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  return distanceTowards[neighborAddress];
}

void setDistance(uint16_t neighborAddress, int16_t distance)
{
  ASSERT(neighborAddress <= RANGING_TABLE_SIZE);
  distanceTowards[neighborAddress] = distance;
}

static void uwbRangingTxTask(void *parameters)
{
  systemWaitStart();

  /* velocity log variable id */
  idVelocityX = logGetVarId("stateEstimate", "vx");
  idVelocityY = logGetVarId("stateEstimate", "vy");
  idVelocityZ = logGetVarId("stateEstimate", "vz");

  UWB_Packet_t txPacketCache;
  txPacketCache.header.type = RANGING;
  //  txPacketCache.header.mac = ? TODO init mac header
  while (true)
  {
    int msgLen = generateRangingMessage((Ranging_Message_t *)&txPacketCache.payload);
    txPacketCache.header.length = sizeof(Packet_Header_t) + msgLen;
    uwbSendPacketBlock(&txPacketCache);
    if (jitter != 0)
    {
      vTaskDelay(TX_PERIOD_IN_MS + rand() % (jitter + 1));
    }
    else
    {
      vTaskDelay(TX_PERIOD_IN_MS);
    }
  }
}

static void uwbRangingRxTask(void *parameters)
{
  systemWaitStart();
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  Ranging_Message_With_Timestamp_t rxPacketCache;

  while (true)
  {
    if (xQueueReceive(rxQueue, &rxPacketCache, portMAX_DELAY))
    {
      //      DEBUG_PRINT("uwbRangingRxTask: received ranging message \n");
      processRangingMessage(&rxPacketCache);
    }
  }
}

void rangingInit()
{
  MY_UWB_ADDRESS = getUWBAddress();
  DEBUG_PRINT("MY_UWB_ADDRESS = %d \n", MY_UWB_ADDRESS);
  srand(MY_UWB_ADDRESS);
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

int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf)
{

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (Rr.timestamp.full - Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (Tr.timestamp.full - Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (Rf.timestamp.full - Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (Tf.timestamp.full - Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);
  int16_t distance = (int16_t)tprop_ctn * 0.4691763978616;

  bool isErrorOccurred = false;
  if (distance > 1000 || distance < 0)
  {
    DEBUG_PRINT("isErrorOccurred\n");
    isErrorOccurred = true;
  }

  if (tRound2 < 0 || tReply2 < 0)
  {
    DEBUG_PRINT("tRound2 < 0 || tReply2 < 0\n");
    isErrorOccurred = true;
  }

  if (isErrorOccurred)
  {
    return 0;
  }
  // DEBUG_PRINT("d=%d\n",distance);
  return distance;
}

void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp)
{
  Ranging_Message_t *rangingMessage = &rangingMessageWithTimestamp->rangingMessage;
  uint16_t neighborAddress = rangingMessage->header.srcAddress;
  set_index_t neighborIndex = findInRangingTableSet(&rangingTableSet, neighborAddress);

  /* handle new neighbor */
  if (neighborIndex == -1)
  {
    if (rangingTableSet.freeQueueEntry == -1)
    {
      /* ranging table set is full, ignore this ranging message */
      return;
    }
    Ranging_Table_t table;
    rangingTableInit(&table, neighborAddress,TX_PERIOD_IN_MS);
    neighborIndex = rangingTableSetInsert(&rangingTableSet, &table);
  }

  Ranging_Table_t *neighborRangingTable = &rangingTableSet.setData[neighborIndex].data;
  Ranging_Table_Tr_Rr_Buffer_t *neighborTrRrBuffer = &neighborRangingTable->TrRrBuffer;

  /* update Re */
  neighborRangingTable->Re.timestamp = rangingMessageWithTimestamp->rxTime;
  neighborRangingTable->Re.seqNumber = rangingMessage->header.msgSequence;

  /* update Tr and Rr */
  Timestamp_Tuple_t neighborTr = rangingMessage->header.lastTxTimestamp;
  if (neighborTr.timestamp.full && neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.timestamp.full && neighborTr.seqNumber == neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr.seqNumber)
  {
    rangingTableBufferUpdate(&neighborRangingTable->TrRrBuffer,
                             neighborTr,
                             neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr);
  }

  /* update Rf */
  Timestamp_Tuple_t neighborRf = {.timestamp.full = 0};
  if (rangingMessage->header.filter & (1 << (getUWBAddress() % 16)))
  {
    /* retrieve body unit */
    uint8_t bodyUnitCount = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < bodyUnitCount; i++)
    {
      if (rangingMessage->bodyUnits[i].address == getUWBAddress())
      {
        // neighborRf = rangingMessage->bodyUnits[i].timestamp;
        neighborRf.timestamp = rangingMessage->bodyUnits[i].timestamp;
        neighborRf.seqNumber = rangingMessage->bodyUnits[i].seqNumber;
        break;
      }
    }
  }
  // printRangingTable(neighborRangingTable);
  if (neighborRf.timestamp.full)
  {
    neighborRangingTable->Rf = neighborRf;
    // TODO it is possible that can not find corresponding Tf
    /* find corresponding Tf in TfBuffer */
    for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++)
    {
      if (TfBuffer[i].seqNumber == neighborRf.seqNumber)
      {
        neighborRangingTable->Tf = TfBuffer[i];
      }
    }

    Ranging_Table_Tr_Rr_Candidate_t Tr_Rr_Candidate = rangingTableBufferGetCandidate(&neighborRangingTable->TrRrBuffer,
                                                                                     neighborRangingTable->Tf);
    /* try to compute distance */
    if (Tr_Rr_Candidate.Tr.timestamp.full && Tr_Rr_Candidate.Rr.timestamp.full &&
        neighborRangingTable->Tp.timestamp.full && neighborRangingTable->Rp.timestamp.full &&
        neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full)
    {
      int16_t distance = computeDistance(neighborRangingTable->Tp, neighborRangingTable->Rp,
                                         Tr_Rr_Candidate.Tr, Tr_Rr_Candidate.Rr,
                                         neighborRangingTable->Tf, neighborRangingTable->Rf);
      if (distance > 0)
      {
        neighborRangingTable->distance = distance;
        setDistance(neighborRangingTable->neighborAddress, distance);
        /*--------------------------------------------------*/
        if (startStatistic==1)
        {
          uint16_t lastSeqNumber = lastSuccRangingSeq[neighborAddress];
          uint16_t curSeqNumber = rangingMessage->header.msgSequence;
          if (firstStatisticSuccRanging[neighborAddress]) // 第一次统计
          {
            firstStatisticSuccRanging[neighborAddress] = false;
          }
          else
          {
            // 计算距离上次成功测距的数量
            uint16_t loss_num = curSeqNumber - lastSeqNumber - 1;
            // 判断距离上次成功测距的数量是否超出了最大值
            loss_num = loss_num > MAX_STATISTIC_LOSS_NUM ? MAX_STATISTIC_LOSS_NUM : loss_num;
            continuousRangingFailCount[neighborAddress][loss_num]++;
          }
          lastSuccRangingSeq[neighborAddress] = rangingMessage->header.msgSequence;
          rangingSuccCount[neighborAddress]++;
        }
        /*--------------------------------------------------*/
      }
      else
      {
        // DEBUG_PRINT("distance is not updated since some error occurs\n");
      }
    }
  }

  /* Tp <- Tf, Rp <- Rf */
  if (neighborRangingTable->Tf.timestamp.full && neighborRangingTable->Rf.timestamp.full)
  {
    rangingTableShift(neighborRangingTable);
  }

  /* update Rr */
  neighborTrRrBuffer->candidates[neighborTrRrBuffer->cur].Rr = neighborRangingTable->Re;

  /* update expiration time */
  neighborRangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);

  neighborRangingTable->state = RECEIVED;
  /*------------------------------------------------------*/
  if (startStatistic == 1)
  {
    uint16_t lastSeqNumber = lastSuccRxPacketSeq[neighborAddress];
    uint16_t curSeqNumber = rangingMessage->header.msgSequence;
    if (firstStatisticSuccRx[neighborAddress])
    {
      firstStatisticSuccRx[neighborAddress] = false;
      firstStatisticSuccRanging[neighborAddress] = false; // 这里添加处理是为了解决开始统计即丢包，导致这段即使丢包了也不算测距失败的问题
      lastSuccRangingSeq[neighborAddress] = curSeqNumber;
    }
    else
    {
      // 计算丢包数量
      uint16_t loss_num = curSeqNumber - lastSeqNumber - 1;
      // 判断丢包数量是否超出了最大值
      loss_num = loss_num > MAX_STATISTIC_LOSS_NUM ? MAX_STATISTIC_LOSS_NUM : loss_num;
      continuousLossPacketCount[neighborAddress][loss_num]++; // 主要为了更新这个
    }
    // DEBUG_PRINT("%d:%d\n",neighborAddress,rxPacketCount[neighborAddress]);
    lastSuccRxPacketSeq[neighborAddress] = curSeqNumber;
    rxPacketCount[neighborAddress]++; // 主要为了更新这个
  }else{
    // DEBUG_PRINT("%d:%d\n",neighborAddress,rxPacketCount[neighborAddress]);
  }
  /*------------------------------------------------------*/
}

int generateRangingMessage(Ranging_Message_t *rangingMessage)
{
#ifdef ENABLE_BUS_BOARDING_SCHEME
  sortRangingTableSet(&rangingTableSet);
#endif
  rangingTableSetClearExpire(&rangingTableSet);
  int8_t bodyUnitNumber = 0;
  rangingSeqNumber++;
  int curSeqNumber = rangingSeqNumber;
  rangingMessage->header.filter = 0;
  /* generate message body */
  for (set_index_t index = rangingTableSet.fullQueueEntry; index != -1;
       index = rangingTableSet.setData[index].next)
  {
    Ranging_Table_t *table = &rangingTableSet.setData[index].data;
    if (bodyUnitNumber >= MAX_BODY_UNIT_NUMBER)
    {
      break;
    }
    if (table->state == RECEIVED)
    {
      rangingMessage->bodyUnits[bodyUnitNumber].timestamp = table->Re.timestamp;
      rangingMessage->bodyUnits[bodyUnitNumber].seqNumber = table->Re.seqNumber;
      rangingMessage->bodyUnits[bodyUnitNumber].address = table->neighborAddress;
      /* It is possible that Re is not the newest timestamp, because the newest may be in rxQueue
       * waiting to be handled.
       */
      bodyUnitNumber++;
      table->state = TRANSMITTED;
      rangingMessage->header.filter |= 1 << (table->neighborAddress % 16);
    }
  }
  /* generate message header */
  rangingMessage->header.srcAddress = MY_UWB_ADDRESS;
  rangingMessage->header.msgLength = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * bodyUnitNumber;
  rangingMessage->header.msgSequence = curSeqNumber;
  rangingMessage->header.lastTxTimestamp = TfBuffer[TfBufferIndex];
  float velocityX = logGetFloat(idVelocityX);
  float velocityY = logGetFloat(idVelocityY);
  float velocityZ = logGetFloat(idVelocityZ);
  velocity = sqrt(pow(velocityX, 2) + pow(velocityY, 2) + pow(velocityZ, 2));
  /* velocity in cm/s */
  rangingMessage->header.velocity = (short)(velocity * 100);
  return rangingMessage->header.msgLength;
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

PARAM_GROUP_START(Statistic)
PARAM_ADD(PARAM_INT16, jitter, &jitter)
PARAM_ADD(PARAM_INT16, statistic, &startStatistic)
PARAM_ADD(PARAM_UINT16, period, &TX_PERIOD_IN_MS)
PARAM_GROUP_STOP(Statistic)