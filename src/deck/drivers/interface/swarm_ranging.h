#ifndef _SWARM_RANGING_H_
#define _SWARM_RANGING_H_

#include "dwTypes.h"

/* Function Switch */
//#define ENABLE_BUS_BOARDING_SCHEME
//#define ENABLE_DYNAMIC_RANGING_PERIOD
#ifdef ENABLE_DYNAMIC_RANGING_PERIOD
#define DYNAMIC_RANGING_COEFFICIENT 1
#endif

/* Ranging Constants */
#define RANGING_PERIOD 100 // in ms
#define RANGING_PERIOD_MIN 20 // default 20ms
#define RANGING_PERIOD_MAX 500 // default 500ms
#define MAX_NEIGHBOR 10 // default up to 10 neighbors

/* Queue Constants */
#define RANGING_RX_QUEUE_SIZE 10
#define RANGING_RX_QUEUE_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)

/* Ranging Struct Constants */
#define MAX_Tr_UNIT 3
#define MAX_BODY_UNIT 7
//#define MAX_BODY_UNIT (FRAME_LEN_MAX - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t) // 1 ~ 83
#define RANGING_TABLE_SIZE MAX_NEIGHBOR
#define RANGING_TABLE_HOLD_TIME 10000
#define Tr_Rr_BUFFER_POOL_SIZE 3
#define Tf_BUFFER_POOL_SIZE (4 * RANGING_PERIOD_MAX / RANGING_PERIOD_MIN)

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
  Timestamp_Tuple_t lastTxTimestamps[MAX_Tr_UNIT]; // 10 byte *
  short velocity; // 2 byte cm/s
  uint16_t msgLength; // 2 byte
  uint16_t filter; // 16 bits bloom filter
} __attribute__((packed)) Ranging_Message_Header_t; // 20 byte

/* Ranging Message */
typedef struct {
  Ranging_Message_Header_t header; // 18 byte
  Body_Unit_t bodyUnits[MAX_BODY_UNIT]; // 12 byte * MAX_NEIGHBOR_SIZE
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
  set_index_t latest; /* Index of latest valid (Tr,Rr) pair */
  set_index_t cur; /* Index of current empty slot for next valid (Tr,Rr) pair */
  Ranging_Table_Tr_Rr_Candidate_t candidates[Tr_Rr_BUFFER_POOL_SIZE];
} __attribute__((packed)) Ranging_Table_Tr_Rr_Buffer_t;

typedef enum {
  RESERVED,
  S1,
  S2,
  S3,
  S4,
  S5, /* S5 is effectively a temporary state for distance calculation. */
  RANGING_TABLE_STATE_COUNT
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

/* Ranging Operations */
void rangingInit();
int16_t getDistance(uint16_t neighborAddress);
void setDistance(uint16_t neighborAddress, int16_t distance);

/* Tr_Rr Buffer Operations */
void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer);
void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t Tr,
                              Timestamp_Tuple_t Rr);
Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                                                               Timestamp_Tuple_t Tf);

/* Tf Buffer Operations */
void updateTfBuffer(Timestamp_Tuple_t timestamp);
Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber);
Timestamp_Tuple_t getLatestTxTimestamp();
void getLastestNTxTimestamps(Timestamp_Tuple_t* timestamps, int n);

/* Ranging Table Operations */
void rangingTableInit(Ranging_Table_t *rangingTable, uint16_t address);
void rangingTableOnEvent(Ranging_Table_t *rangingTable, RANGING_TABLE_EVENT event);
void rangingTableSetInit(Ranging_Table_Set_t *rangingTableSet);
set_index_t rangingTableSetInsert(Ranging_Table_Set_t *rangingTableSet, Ranging_Table_t *rangingTable);
set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet, uint16_t address);
bool deleteRangingTableByIndex(Ranging_Table_Set_t *rangingTableSet, set_index_t index);
bool rangingTableSetClearExpire(Ranging_Table_Set_t *rangingTableSet);
void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet);

/* Debug Operations */
void printRangingTable(Ranging_Table_t *rangingTable);
void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet);
void printRangingMessage(Ranging_Message_t *rangingMessage);

#endif