#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_
#include "adhocdeck.h"
#include "ranging_struct.h"

/* Function Switch */
#define ENABLE_BUS_BOARDING_SCHEME

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 10
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Constants */
#define RANGING_INTERVAL_MIN 20  // default 20
#define RANGING_INTERVAL_MAX 500 // default 500
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_INTERVAL_MAX / RANGING_INTERVAL_MIN)
// #define TX_PERIOD_IN_MS 20
/*---------------------------------------------*/
#define MAX_STATISTIC_LOSS_NUM 149
// 设置parameter: period，jitter，spispeed
// 设置parameter,如果为真，则开始
static bool firstStatisticSuccRx[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = true};
static bool firstStatisticSuccRanging[RANGING_TABLE_SIZE + 1] = {[0 ... RANGING_TABLE_SIZE] = true};

static uint16_t lastSuccRangingSeq[RANGING_TABLE_SIZE + 1] = {0};  // 上次邻居成功测距的序号，辅助
static uint16_t lastSuccRxPacketSeq[RANGING_TABLE_SIZE + 1] = {0}; // 上次邻居成功测距的序号，辅助

static uint16_t continuousLossPacketCount[RANGING_TABLE_SIZE + 1][MAX_STATISTIC_LOSS_NUM + 1];  // [i][j],两次成功收包j代表间隔的次数，值就是事件发生的次数
static uint16_t continuousRangingFailCount[RANGING_TABLE_SIZE + 1][MAX_STATISTIC_LOSS_NUM + 1]; // [i][j],两次成功测距j代表间隔的次数，值就是事件发生的次数
static uint16_t rxPacketCount[RANGING_TABLE_SIZE + 1] = {0};                                    // 收到其他无人机数据包的次数
static uint16_t rangingSuccCount[RANGING_TABLE_SIZE + 1] = {0};                                 // 与其他无人机成功测距的次数
int16_t getStartStatistic();
int16_t getJitter();
uint16_t getPeriod();
/*---------------------------------------------*/

/* Ranging Operations */
void rangingInit();
int16_t computeDistance(Timestamp_Tuple_t Tp, Timestamp_Tuple_t Rp,
                        Timestamp_Tuple_t Tr, Timestamp_Tuple_t Rr,
                        Timestamp_Tuple_t Tf, Timestamp_Tuple_t Rf);
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp);
int generateRangingMessage(Ranging_Message_t *rangingMessage);
int16_t getDistance(uint16_t neighborAddress);
void setDistance(uint16_t neighborAddress, int16_t distance);

#endif