#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_

#include "ranging_struct.h"

/* Function Switch */
//#define ENABLE_BUS_BOARDING_SCHEME
//#define ENABLE_DYNAMIC_RANGING_PERIOD
#ifdef ENABLE_DYNAMIC_RANGING_PERIOD
  #define DYNAMIC_RANGING_COEFFICIENT 1
#endif

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 10
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Constants */
#define RANGING_PERIOD 100 // in ms
#define RANGING_PERIOD_MIN 20 // default 20ms
#define RANGING_PERIOD_MAX 500 // default 500ms

/* Ranging Operations */
void rangingInit();
void processRangingMessage(Ranging_Message_With_Timestamp_t *rangingMessageWithTimestamp);
Time_t generateRangingMessage(Ranging_Message_t *rangingMessage);
int16_t getDistance(uint16_t neighborAddress);
void setDistance(uint16_t neighborAddress, int16_t distance);

#endif