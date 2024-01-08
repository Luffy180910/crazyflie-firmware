#include "ranging_struct.h"
#include "adhocdeck.h"
#include "swarm_ranging.h"
#include <stdio.h>
#include <string.h>
#include "task.h"
#include "debug.h"

void rangingTableBufferInit(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer) {
  rangingTableBuffer->cur = 0;
  rangingTableBuffer->latest = 0;
  Timestamp_Tuple_t empty = {.seqNumber = 0, .timestamp.full = 0};
  for (set_index_t i = 0; i < Tr_Rr_BUFFER_POOL_SIZE; i++) {
    rangingTableBuffer->candidates[i].Tr = empty;
    rangingTableBuffer->candidates[i].Rr = empty;
  }
}

void rangingTableBufferUpdate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                              Timestamp_Tuple_t Tr,
                              Timestamp_Tuple_t Rr) {
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Tr = Tr;
  rangingTableBuffer->candidates[rangingTableBuffer->cur].Rr = Rr;
  // shift
  rangingTableBuffer->latest = rangingTableBuffer->cur;
  rangingTableBuffer->cur = (rangingTableBuffer->cur + 1) % Tr_Rr_BUFFER_POOL_SIZE;
}

Ranging_Table_Tr_Rr_Candidate_t rangingTableBufferGetCandidate(Ranging_Table_Tr_Rr_Buffer_t *rangingTableBuffer,
                                                               Timestamp_Tuple_t Tf) {
  set_index_t index = rangingTableBuffer->latest;
  uint64_t rightBound = Tf.timestamp.full % MAX_TIMESTAMP;
  Ranging_Table_Tr_Rr_Candidate_t candidate = {.Rr.timestamp.full = 0, .Tr.timestamp.full = 0};

  for (int count = 0; count < Tr_Rr_BUFFER_POOL_SIZE; count++) {
    if (rangingTableBuffer->candidates[index].Rr.timestamp.full &&
        rangingTableBuffer->candidates[index].Rr.timestamp.full % MAX_TIMESTAMP < rightBound) {
      candidate.Tr = rangingTableBuffer->candidates[index].Tr;
      candidate.Rr = rangingTableBuffer->candidates[index].Rr;
      break;
    }
    index = (index - 1 + Tr_Rr_BUFFER_POOL_SIZE) % Tr_Rr_BUFFER_POOL_SIZE;
  }

  return candidate;
}

void rangingTableInit(Ranging_Table_t *rangingTable, address_t address) {
  memset(rangingTable, 0, sizeof(Ranging_Table_t));
  // TODO: check state init
  rangingTable->state = S1;
  rangingTable->neighborAddress = address;
  rangingTable->period = RANGING_PERIOD;
  rangingTable->nextExpectedDeliveryTime = xTaskGetTickCount() + rangingTable->period;
  rangingTable->expirationTime = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
  rangingTableBufferInit(&rangingTable->TrRrBuffer); // TODO remove this since memset() is called
}

// TODO: merge state handlers

static void S1_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;
  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = S2;
  RANGING_TABLE_STATE curState = rangingTable->state;
  DEBUG_PRINT("S1_Tf: %d -> %d\n", prevState, curState);
}

static void S1_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;
  rangingTable->state = S1;
  RANGING_TABLE_STATE curState = rangingTable->state;
  DEBUG_PRINT("S1_RX_NO_Rf: %d -> %d\n", prevState, curState);
}

static void S1_RX_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;
  rangingTable->state = S1;
  RANGING_TABLE_STATE curState = rangingTable->state;
  DEBUG_PRINT("S1_RX_Rf: %d -> %d\n", prevState, curState);
}

static void S2_Tf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;
  /* Don't update Tf here since sending message is an async action, we put all Tf in TfBuffer. */
  rangingTable->state = S2;
  RANGING_TABLE_STATE curState = rangingTable->state;
  DEBUG_PRINT("S2_Tf: %d -> %d\n", prevState, curState);
}

static void S2_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  RANGING_TABLE_STATE prevState = rangingTable->state;
  rangingTable->state = S2;
  RANGING_TABLE_STATE curState = rangingTable->state;
  DEBUG_PRINT("S2_RX_NO_Rf: %d -> %d\n", prevState, curState);
}

static void S2_RX_Rf(Ranging_Table_t *rangingTable) {
  // TODO
  RANGING_TABLE_STATE prevState = rangingTable->state;
  rangingTable->state = S3;
  RANGING_TABLE_STATE curState = rangingTable->state;
  DEBUG_PRINT("S2_RX_Rf: %d -> %d\n", prevState, curState);
}

static void S3_Tf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S3_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S3_RX_Rf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S4_Tf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S4_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S4_RX_Rf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S5_Tf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S5_RX_NO_Rf(Ranging_Table_t *rangingTable) {
  // TODO
}

static void S5_RX_Rf(Ranging_Table_t *rangingTable) {
  // TODO
}

static RangingTableEventHandler EVENT_HANDLER[RANGING_TABLE_STATE_COUNT][RANGING_TABLE_EVENT_COUNT] = {
    {S1_Tf, S1_RX_NO_Rf, S1_RX_Rf},
    {S2_Tf, S2_RX_NO_Rf, S2_RX_Rf},
    {S3_Tf, S3_RX_NO_Rf, S3_RX_Rf},
    {S4_Tf, S4_RX_NO_Rf, S4_RX_Rf},
    {S5_Tf, S5_RX_NO_Rf, S5_RX_Rf},
};

void rangingTableOnEvent(Ranging_Table_t *rangingTable, RANGING_TABLE_EVENT event) {
  // TODO: check if mutex is needed.
  ASSERT(rangingTable->state < RANGING_TABLE_STATE_COUNT);
  ASSERT(event < RANGING_TABLE_EVENT_COUNT);
  EVENT_HANDLER[rangingTable->state][event](rangingTable);
//  switch (event) {
//    case TX_Tf:
//      DEBUG_PRINT("Event: TX_Tf, neighbor: %d\n", rangingTable->neighborAddress);
//      EVENT_HANDLER[curState][event](rangingTable);
//      break;
//    case RX_NO_Rf:
//      DEBUG_PRINT("Event: RX_NO_Rf, neighbor: %d\n", rangingTable->neighborAddress);
//      EVENT_HANDLER[curState][event](rangingTable);
//      break;
//    case RX_Rf:
//      DEBUG_PRINT("Event: RX_Rf, neighbor: %d\n", rangingTable->neighborAddress);
//      EVENT_HANDLER[curState][event](rangingTable);
//      break;
//    default:
//      DEBUG_PRINT("Error: invalid ranging table event\n");
//  }
}
// TODO merge swarm_ranging.h and ranging_struct.h
static void handleEvent() {

}

//TODO add semaphore to protect ranging table structure.
static set_index_t rangingTableSetMalloc(
    Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->freeQueueEntry == -1) {
    DEBUG_PRINT("Ranging Table Set is FULL, malloc failed.\n");
    return -1;
  } else {
    set_index_t candidate = rangingTableSet->freeQueueEntry;
    rangingTableSet->freeQueueEntry =
        rangingTableSet->setData[candidate].next;
    // insert to full queue
    set_index_t temp = rangingTableSet->fullQueueEntry;
    rangingTableSet->fullQueueEntry = candidate;
    rangingTableSet->setData[candidate].next = temp;
    return candidate;
  }
}

static bool rangingTableSetFree(Ranging_Table_Set_t *rangingTableSet,
                                set_index_t item_index) {
  if (-1 == item_index) {
    return true;
  }
  // delete from full queue
  set_index_t pre = rangingTableSet->fullQueueEntry;
  if (item_index == pre) {
    rangingTableSet->fullQueueEntry = rangingTableSet->setData[pre].next;
    // insert into empty queue
    rangingTableSet->setData[item_index].next =
        rangingTableSet->freeQueueEntry;
    rangingTableSet->freeQueueEntry = item_index;
    rangingTableSet->size = rangingTableSet->size - 1;
    return true;
  } else {
    while (pre != -1) {
      if (rangingTableSet->setData[pre].next == item_index) {
        rangingTableSet->setData[pre].next =
            rangingTableSet->setData[item_index].next;
        // insert into empty queue
        rangingTableSet->setData[item_index].next =
            rangingTableSet->freeQueueEntry;
        rangingTableSet->freeQueueEntry = item_index;
        rangingTableSet->size = rangingTableSet->size - 1;
        return true;
      }
      pre = rangingTableSet->setData[pre].next;
    }
  }
  return false;
}

void rangingTableSetInit(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t i;
  for (i = 0; i < RANGING_TABLE_SIZE - 1; i++) {
    rangingTableSet->setData[i].next = i + 1;
  }
  rangingTableSet->setData[i].next = -1;
  rangingTableSet->freeQueueEntry = 0;
  rangingTableSet->fullQueueEntry = -1;
  rangingTableSet->size = 0;
}

set_index_t rangingTableSetInsert(Ranging_Table_Set_t *rangingTableSet,
                                  Ranging_Table_t *table) {
  set_index_t candidate = rangingTableSetMalloc(rangingTableSet);
  if (candidate != -1) {
    memcpy(&rangingTableSet->setData[candidate].data, table,
           sizeof(Ranging_Table_t));
    rangingTableSet->size++;
  }
  return candidate;
}

set_index_t findInRangingTableSet(Ranging_Table_Set_t *rangingTableSet,
                                  address_t addr) {
  set_index_t iter = rangingTableSet->fullQueueEntry;
  while (iter != -1) {
    Ranging_Table_Set_Item_t cur = rangingTableSet->setData[iter];
    if (cur.data.neighborAddress == addr) {
      break;
    }
    iter = cur.next;
  }
  return iter;
}

bool deleteRangingTableByIndex(Ranging_Table_Set_t *rangingTableSet,
                               set_index_t index) {
  return rangingTableSetFree(rangingTableSet, index);
}

void printRangingTable(Ranging_Table_t *table) {
  DEBUG_PRINT("Rp = %u, Tr = %u, Rf = %u, \n",
              table->Rp.seqNumber,
              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.seqNumber,
              table->Rf.seqNumber);
  DEBUG_PRINT("Tp = %u, Rr = %u, Tf = %u, Re = %u, \n",
              table->Tp.seqNumber,
              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.seqNumber,
              table->Tf.seqNumber,
              table->Re.seqNumber);
  DEBUG_PRINT("====\n");
//  DEBUG_PRINT("Rp = %2x%8lx, Tr = %2x%8lx, Rf = %2x%8lx, \n",
//              table->Rp.timestamp.high8,
//              table->Rp.timestamp.low32,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.timestamp.high8,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.timestamp.low32,
//              table->Rf.timestamp.high8,
//              table->Rf.timestamp.low32);
//  DEBUG_PRINT("Tp = %2x%8lx, Rr = %2x%8lx, Tf = %2x%8lx, Re = %2x%8lx, \n",
//              table->Tp.timestamp.high8,
//              table->Tp.timestamp.low32,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.timestamp.high8,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.timestamp.low32,
//              table->Tf.timestamp.high8,
//              table->Tf.timestamp.low32,
//              table->Re.timestamp.high8,
//              table->Re.timestamp.low32);
//  DEBUG_PRINT("====\n");
//  DEBUG_PRINT("Rp = %llu, Tr = %llu, Rf = %llu, \n",
//              table->Rp.timestamp.full,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Tr.seqNumber,
//              table->Rf.timestamp.full);
//  DEBUG_PRINT("Tp = %llu, Rr = %llu, Tf = %llu, Re = %llu, \n",
//              table->Tp.timestamp.full,
//              table->TrRrBuffer.candidates[table->TrRrBuffer.latest].Rr.seqNumber,
//              table->Tf.timestamp.full,
//              table->Re.timestamp.full);
//  DEBUG_PRINT("====\n");
}

void printRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
  for (set_index_t index = rangingTableSet->fullQueueEntry; index != -1;
       index = rangingTableSet->setData[index].next) {
    printRangingTable(&rangingTableSet->setData[index].data);
  }
}

bool rangingTableSetClearExpire(Ranging_Table_Set_t *rangingTableSet) {
  set_index_t candidate = rangingTableSet->fullQueueEntry;
  Time_t now = xTaskGetTickCount();
  bool has_changed = false;
  while (candidate != -1) {
    Ranging_Table_Set_Item_t temp = rangingTableSet->setData[candidate];
    if (temp.data.expirationTime < now) {
      set_index_t next_index = temp.next;
      rangingTableSetFree(rangingTableSet, candidate);
      setDistance(temp.data.neighborAddress, -1);
      candidate = next_index;
      has_changed = true;
      continue;
    }
    candidate = temp.next;
  }
  return has_changed;
}

void sortRangingTableSet(Ranging_Table_Set_t *rangingTableSet) {
  if (rangingTableSet->fullQueueEntry == -1) {
    return;
  }
  set_index_t new_head = rangingTableSet->fullQueueEntry;
  set_index_t cur = rangingTableSet->setData[new_head].next;
  rangingTableSet->setData[new_head].next = -1;
  set_index_t next = -1;
  while (cur != -1) {
    next = rangingTableSet->setData[cur].next;
    if (rangingTableSet->setData[cur].data.nextExpectedDeliveryTime <=
        rangingTableSet->setData[new_head].data.nextExpectedDeliveryTime) {
      rangingTableSet->setData[cur].next = new_head;
      new_head = cur;
    } else {
      set_index_t start = rangingTableSet->setData[new_head].next;
      set_index_t pre = new_head;
      while (start != -1 &&
          rangingTableSet->setData[cur].data.nextExpectedDeliveryTime >
              rangingTableSet->setData[start].data.nextExpectedDeliveryTime) {
        pre = start;
        start = rangingTableSet->setData[start].next;
      }
      rangingTableSet->setData[cur].next = start;
      rangingTableSet->setData[pre].next = cur;
    }
    cur = next;
  }
  rangingTableSet->fullQueueEntry = new_head;
}

static int TfBufferIndex = 0;
static Timestamp_Tuple_t TfBuffer[Tf_BUFFER_POOL_SIZE] = {0};

void updateTfBuffer(Timestamp_Tuple_t timestamp) {
  TfBufferIndex++;
  TfBufferIndex %= Tf_BUFFER_POOL_SIZE;
  TfBuffer[TfBufferIndex] = timestamp;
}

Timestamp_Tuple_t findTfBySeqNumber(uint16_t seqNumber) {
  Timestamp_Tuple_t Tf = {.timestamp.full = 0, .seqNumber = 0};
  for (int i = 0; i < Tf_BUFFER_POOL_SIZE; i++) {
    if (TfBuffer[i].seqNumber == seqNumber) {
      Tf = TfBuffer[i];
      break;
    }
  }
  return Tf;
}

Timestamp_Tuple_t getLatestTxTimestamp() {
  return TfBuffer[TfBufferIndex];
}

void printRangingMessage(Ranging_Message_t *rangingMessage) {
  DEBUG_PRINT(
      "msgLength=%u, msgSequence=%d, srcAddress=%u, velocity=%d\n, last_tx_timestamp_seq=%u, lastTxTimestamp=%2x%8lx\n",
      rangingMessage->header.msgLength,
      rangingMessage->header.msgSequence,
      rangingMessage->header.srcAddress,
      rangingMessage->header.velocity,
      rangingMessage->header.lastTxTimestamp.seqNumber,
      rangingMessage->header.lastTxTimestamp.timestamp.high8,
      rangingMessage->header.lastTxTimestamp.timestamp.low32);

  if (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t) == 0) {
    return;
  }
  int body_unit_number = (rangingMessage->header.msgLength - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  if (body_unit_number >= MAX_BODY_UNIT_NUMBER) {
    DEBUG_PRINT("===printRangingMessage: wrong body unit number occurs===\n");
    return;
  }
  for (int i = 0; i < body_unit_number; i++) {
    DEBUG_PRINT("body_unit_address=%u, body_unit_seq=%u\n",
                rangingMessage->bodyUnits[i].address,
                rangingMessage->bodyUnits[i].timestamp.seqNumber);
    DEBUG_PRINT("body_unit_timestamp=%2x%8lx\n",
                rangingMessage->bodyUnits[i].timestamp.timestamp.high8,
                rangingMessage->bodyUnits[i].timestamp.timestamp.low32);
  }
}