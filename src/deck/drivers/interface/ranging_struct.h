#ifndef __RANGING_STRUCT_H__
#define __RANGING_STRUCT_H__

#include <stdbool.h>
#include "FreeRTOS.h"
#include "dwTypes.h"

#define MAX_BODY_UNIT_NUMBER 7
//#define MAX_BODY_UNIT_NUMBER (FRAME_LEN_MAX - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t) // 1 ~ 83
#define RANGING_TABLE_SIZE 10
#define RANGING_TABLE_HOLD_TIME 10000
#define Tr_Rr_BUFFER_POOL_SIZE 5
#define Tf_BUFFER_POOL_SIZE 100

typedef portTickType Time_t;
typedef short set_index_t;

/* Timestamp Tuple */
typedef struct {
  dwTime_t timestamp; // 8 byte
  uint16_t seqNumber; // 2 byte
} __attribute__((packed)) Timestamp_Tuple_t; // 10 byte

/* Body Unit */
typedef struct {
  uint16_t address; // 2 byte
  Timestamp_Tuple_t timestamp; // 10 byte
} __attribute__((packed)) Body_Unit_t; // 12 byte

/* Ranging Message Header*/
typedef struct {
  uint16_t srcAddress; // 2 byte
  uint16_t msgSequence; // 2 byte
  Timestamp_Tuple_t lastTxTimestamp; // 10 byte
  short velocity; // 2 byte cm/s
  uint16_t msgLength; // 2 byte
  uint16_t filter; // 16 bits bloom filter
} __attribute__((packed)) Ranging_Message_Header_t; // 20 byte

/* Ranging Message */
typedef struct {
  Ranging_Message_Header_t header; // 18 byte
  Body_Unit_t bodyUnits[MAX_BODY_UNIT_NUMBER]; // 12 byte * MAX_NEIGHBOR_SIZE
} __attribute__((packed)) Ranging_Message_t; // 20 + 12 byte * MAX_NEIGHBOR_SIZE

/* Ranging Message With RX Timestamp, used in RX Queue */
typedef struct {
  Ranging_Message_t rangingMessage;
  dwTime_t rxTime;
} __attribute__((packed)) Ranging_Message_With_Timestamp_t;

typedef struct {
  Timestamp_Tuple_t Tr;
  Timestamp_Tuple_t Rr;
} __attribute__((packed)) Ranging_Table_Tr_Rr_Candidate_t;

/* Tr and Rr candidate buffer for each Ranging Table */
typedef struct {
  set_index_t latest;
  set_index_t cur;
  Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_POOL_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

/* Tr_Rr Buffer Operations */
void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t Tr,
                              Timestamp_Tuple_t Rr);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                                                               Timestamp_Tuple_t Tf);

typedef enum {
  RESERVED,
  S1,
  S2,
  S3,
  S4,
  S5,
  RANGING_TABLE_STATE_COUNT
//  S1 = 0b00000000, /* bit string of (Rp Tf Rf Tp Rr Tf Re 0) */
//  S2 = 0b00000100,
//  S3 = 0b10011000,
//  S4 = 0b10011100,
//  S5 = 0b11111110,
} RANGING_TABLE_STATE;

typedef enum {
  TX_Tf,
  RX_NO_Rf,
  RX_Rf,
  RANGING_TABLE_EVENT_COUNT
} RANGING_TABLE_EVENT;

/* Ranging Table
  +------+------+------+------+------+
  |  Rp  |  Tr  |  Rf  |  P   |  tn  |
  +------+------+------+------+------+
  |  Tp  |  Rr  |  Tf  |  Re  |  ts  |
  +------+------+------+------+------+
*/
typedef struct {
  uint16_t neighborAddress;

  Timestamp_Tuple_t Rp;
  Timestamp_Tuple_t Tp;
  Ranging_Table_Tr_Rr_Buffer_t TrRrBuffer;
  Timestamp_Tuple_t Rf;
  Timestamp_Tuple_t Tf;
  Timestamp_Tuple_t Re;
  Timestamp_Tuple_t latestReceived;

  Time_t period;
  Time_t nextExpectedDeliveryTime;
  Time_t expirationTime;
  int16_t distance;

  RANGING_TABLE_STATE state;
} __attribute__((packed)) Ranging_Table_t;

typedef void (*RangingTableEventHandler)(Ranging_Table_t *);

void rangingTableInit(Ranging_Table_t *rangingTable, uint16_t address);
void rangingTableOnEvent(Ranging_Table_t *rangingTable, RANGING_TABLE_EVENT event);

typedef struct {
  set_index_t next;
  Ranging_Table_t data;
} __attribute__((packed)) Ranging_Table_Set_Item_t;

/* Ranging Table Set */
typedef struct {
  Ranging_Table_Set_Item_t setData[RANGING_TABLE_SIZE];
  set_index_t freeQueueEntry;
  set_index_t fullQueueEntry;
  int size;
} Ranging_Table_Set_t;

/* Ranging Table Set Operations */
void rangingTableSetInit(Ranging_Table_Set_t *rangingTableSet);

set_index_t rangingTableSetInsert(Ranging_Table_Set_t *rangingTableSet,
                                  Ranging_Table_t *rangingTable);

set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet,
                                  uint16_t address);

bool deleteRangingTableByIndex(Ranging_Table_Set_t *rangingTableSet,
                               set_index_t index);

bool rangingTableSetClearExpire(Ranging_Table_Set_t *rangingTableSet);

void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet);

/* Tf Buffer Operations */

void updateTfBuffer(Timestamp_Tuple_t timestamp);

Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber);

Timestamp_Tuple_t getLatestTxTimestamp();

/* Debug Operations */

void printRangingTable(Ranging_Table_t *rangingTable);

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet);

void printRangingMessage(Ranging_Message_t *rangingMessage);

#endif