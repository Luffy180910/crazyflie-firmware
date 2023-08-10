
#include "crtp_trans_data.h"
#include "crtp.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "config.h"
#include "debug.h"
#include "string.h"
#include "adhocdeck.h"
#include "swarm_ranging.h"
#include "ranging_struct.h"

typedef enum
{
  START_TOTAL_TRANS = 0,
  END_TOTAL_TRANS = 1,
  TOTAL_TRANS = 2,
  ADD_OLSR_PACKET = 3,
  DELETE_OLSR_PACKET = 4,
  MODIFY_OLSR_PACKET = 5
} transOlsrChannels_t;

static TaskHandle_t crtpTxHandle = 0;

extern uint16_t continuousLossPacketCount[RANGING_TABLE_SIZE + 1][MAX_STATISTIC_LOSS_NUM + 1];  // [i][j],两次成功收包j代表间隔的次数，值就是事件发生的次数
extern uint16_t continuousRangingFailCount[RANGING_TABLE_SIZE + 1][MAX_STATISTIC_LOSS_NUM + 1]; // [i][j],两次成功测距j代表间隔的次数，值就是事件发生的次数
extern uint16_t rxPacketCount[RANGING_TABLE_SIZE + 1];                                          // 收到其他无人机数据包的次数
extern uint16_t rangingSuccCount[RANGING_TABLE_SIZE + 1];
extern uint16_t lossAndinvalidPacketCount[RANGING_TABLE_SIZE + 1];
extern uint16_t allSendPacketNum[RANGING_TABLE_SIZE + 1];
extern uint32_t tx_rx[RX_TX_MAX_NUM];
static void crtpTxOlsrTask(void *parameters)
{
  CRTPPacket packet;

  CrtpSendData_Meta_t crtpSendData_Meta = {
      .magic = 0xBB88,
      .jitter = 0,
      .period = 0,
      .type = 0,
      .firstDim = 0,
      .secondDim = 0};
  while (true)
  {
    vTaskDelay(1000);
    int16_t startStatistic = getStartStatistic();
    while (getStartStatistic() == 2)
    {
      vTaskDelay(1000);
      // 300KB每秒
      crtpSendData_Meta.jitter = getJitter();
      crtpSendData_Meta.period = getPeriod();
      // 1. 传送continuousLossPacketCount
      uint16_t msgSize = sizeof(continuousLossPacketCount);
      msgSize = sizeof(continuousLossPacketCount);
      crtpSendData_Meta.type = 1;
      crtpSendData_Meta.firstDim = RANGING_TABLE_SIZE + 1;
      crtpSendData_Meta.secondDim = MAX_STATISTIC_LOSS_NUM + 1;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, (uint8_t *)&continuousLossPacketCount[0][0]);
      // 2. 传送continuousRangingFailCount
      msgSize = sizeof(continuousRangingFailCount);
      crtpSendData_Meta.type = 2;
      crtpSendData_Meta.firstDim = RANGING_TABLE_SIZE + 1;
      crtpSendData_Meta.secondDim = MAX_STATISTIC_LOSS_NUM + 1;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, &continuousRangingFailCount[0][0]);
      // 3. 传送rxPacketCount
      msgSize = sizeof(rxPacketCount);
      crtpSendData_Meta.type = 3;
      crtpSendData_Meta.firstDim = RANGING_TABLE_SIZE + 1;
      crtpSendData_Meta.secondDim = 0;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, rxPacketCount);
      // 4. 传送rangingSuccCount
      msgSize = sizeof(rangingSuccCount);
      crtpSendData_Meta.type = 4;
      crtpSendData_Meta.firstDim = RANGING_TABLE_SIZE + 1;
      crtpSendData_Meta.secondDim = 0;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, rangingSuccCount);
      // 5. 传送rangingSuccCount
      msgSize = sizeof(lossAndinvalidPacketCount);
      crtpSendData_Meta.type = 5;
      crtpSendData_Meta.firstDim = RANGING_TABLE_SIZE + 1;
      crtpSendData_Meta.secondDim = 0;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, lossAndinvalidPacketCount);
      // 6. 传送rangingSuccCount
      msgSize = sizeof(allSendPacketNum);
      crtpSendData_Meta.type = 6;
      crtpSendData_Meta.firstDim = RANGING_TABLE_SIZE + 1;
      crtpSendData_Meta.secondDim = 0;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, allSendPacketNum);
      // 7. 
      msgSize = sizeof(tx_rx);
      crtpSendData_Meta.type = 7;
      crtpSendData_Meta.firstDim = RX_TX_MAX_NUM;
      crtpSendData_Meta.secondDim = 0;
      crtpSendDataWithArray(crtpSendData_Meta, msgSize, tx_rx);

    }
  }
}

void crtpSendDataWithArray(CrtpSendData_Meta_t sendData_Meta, uint16_t msgSize, uint8_t *pointer)
{
  CRTPPacket crtpPacket;
  int block = 0; // 传输该数据的第几块
  // 先传头部数据
  sendData_Meta.block = block;
  sendData_Meta.msgLength = msgSize;
  uint16_t length = sizeof(CrtpSendData_Meta_t);
  crtpPacket.size = length;
  crtpPacket.header = CRTP_HEADER(CRTP_PORT_TRANSFER_DATA, TOTAL_TRANS);
  memcpy(crtpPacket.data, sendData_Meta.raw, length);
  crtpSendPacket(&crtpPacket);
  vTaskDelay(20);
  uint8_t *pointer_send = pointer;
  int remain = msgSize;
  while (remain > 0)
  {
    block++;
    sendData_Meta.block = block;

    int sizeToSend = remain > CRTP_MAX_DATA_SIZE ? CRTP_MAX_DATA_SIZE : remain;
    crtpPacket.size = sizeToSend;
    memcpy(crtpPacket.data, pointer_send, sizeToSend);
    crtpSendPacket(&crtpPacket);
    vTaskDelay(20);
    pointer_send += sizeToSend;
    remain -= sizeToSend;
  }
}

void crtpTransDataInit()
{
  xTaskCreate(crtpTxOlsrTask, CRTP_TRANS_DATA_TASK_NAME, 1 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &crtpTxHandle);
}