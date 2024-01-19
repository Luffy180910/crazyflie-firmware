#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "autoconf.h"
#include "debug.h"
#include "log.h"
#include "system.h"

#include "routing.h"

static TaskHandle_t uwbRoutingTxTaskHandle = 0;
static TaskHandle_t uwbRoutingRxTaskHandle = 0;
static QueueHandle_t txQueue;
static QueueHandle_t rxQueue;
static xQueueHandle queues[UWB_DATA_MESSAGE_TYPE_COUNT];
static UWB_Data_Packet_Listener_t listeners[UWB_DATA_MESSAGE_TYPE_COUNT];
static Routing_Table_t routingTable;
static int seqNumber = 1;

void routingRxCallback(void *parameters) {
  UWB_Packet_t *uwbPacket = (UWB_Packet_t *) parameters;
  UWB_Data_Packet_t *uwbDataPacket = (UWB_Data_Packet_t *) &uwbPacket->payload;
  ASSERT(uwbDataPacket->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
  if (listeners[uwbDataPacket->header.type].rxCb) {
    listeners[uwbDataPacket->header.type].rxCb(uwbDataPacket);
  }
}

void routingTxCallback(void *parameters) {
  UWB_Packet_t *uwbPacket = (UWB_Packet_t *) parameters;
  UWB_Data_Packet_t *uwbDataPacket = (UWB_Data_Packet_t *) &uwbPacket->payload;
  ASSERT(uwbDataPacket->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
  if (listeners[uwbDataPacket->header.type].txCb) {
    listeners[uwbDataPacket->header.type].txCb(uwbDataPacket);
  }
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.srcAddress = uwbGetAddress();
  txPacketCache.header.destAddress = UWB_DEST_EMPTY;
  txPacketCache.header.type = UWB_DATA_MESSAGE;
  txPacketCache.header.length = 0;

  UWB_Data_Packet_t *txDataPacketCache = (UWB_Data_Packet_t *) &txPacketCache.payload;

  while (true) {
    if (xQueueReceive(txQueue, txDataPacketCache, portMAX_DELAY)) {
      ASSERT(txDataPacketCache->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
      ASSERT(txDataPacketCache->header.length < ROUTING_DATA_PACKET_SIZE_MAX);
      // TODO: modify txPacketCache.header.destAddress according to routing table.
      // TODO: check data packet cache.
      uwbSendPacketBlock(&txPacketCache);
      vTaskDelay(M2T(1));
    }
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;
  UWB_Data_Packet_t *rxDataPacketCache = (UWB_Data_Packet_t *) &rxPacketCache.payload;

  while (true) {
    if (uwbReceivePacketBlock(UWB_DATA_MESSAGE, &rxPacketCache)) {
      ASSERT(rxDataPacketCache->header.type < UWB_DATA_MESSAGE_TYPE_COUNT);
      ASSERT(rxDataPacketCache->header.length < ROUTING_DATA_PACKET_SIZE_MAX);
      /* Dispatch Data Message */
      if (listeners[rxDataPacketCache->header.type].rxQueue) {
        if (xQueueSend(listeners[rxDataPacketCache->header.type].rxQueue, rxDataPacketCache, M2T(100)) != pdPASS) {
          DEBUG_PRINT("uwbRoutingRxTask: Timeout when dispatch data message type %d\n", rxDataPacketCache->header.type);
        }
      }
    }
  }
}

void routingInit() {
  txQueue = xQueueCreate(ROUTING_TX_QUEUE_SIZE, ROUTING_TX_QUEUE_ITEM_SIZE);
  rxQueue = xQueueCreate(ROUTING_RX_QUEUE_SIZE, ROUTING_RX_QUEUE_ITEM_SIZE);
  routingTableInit(&routingTable);

  UWB_Message_Listener_t listener;
  listener.type = UWB_DATA_MESSAGE;
  listener.rxQueue = rxQueue;
  listener.rxCb = routingRxCallback;
  listener.txCb = routingTxCallback;
  uwbRegisterListener(&listener);

  xTaskCreate(uwbRoutingTxTask,
              ADHOC_DECK_ROUTING_TX_TASK_NAME,
              4 * configMINIMAL_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &uwbRoutingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRoutingRxTask,
              ADHOC_DECK_ROUTING_RX_TASK_NAME,
              4 * configMINIMAL_STACK_SIZE,
              NULL,
              ADHOC_DECK_TASK_PRI,
              &uwbRoutingRxTaskHandle); // TODO optimize STACK SIZE
}

/* Messaging Operations */
int uwbSendDataPacket(UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, 0);
}

int uwbSendDataPacketBlock(UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, portMAX_DELAY);
}

int uwbSendDataPacketWait(UWB_Data_Packet_t *packet, int wait) {
  ASSERT(packet);
  return xQueueSend(txQueue, packet, M2T(wait));
}

int uwbReceiveDataPacket(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  ASSERT(type < UWB_DATA_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, 0);
}

int uwbReceiveDataPacketBlock(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet) {
  ASSERT(packet);
  ASSERT(type < UWB_DATA_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, portMAX_DELAY);
}

int uwbReceiveDataPacketWait(UWB_DATA_MESSAGE_TYPE type, UWB_Data_Packet_t *packet, int wait) {
  ASSERT(packet);
  ASSERT(type < UWB_DATA_MESSAGE_TYPE_COUNT);
  return xQueueReceive(queues[type], packet, M2T(wait));
}

void uwbRegisterDataPacketListener(UWB_Data_Packet_Listener_t *listener) {
  ASSERT(listener->type < UWB_DATA_MESSAGE_TYPE_COUNT);
  queues[listener->type] = listener->rxQueue;
  listeners[listener->type] = *listener;
}

/* Routing Table Operations */
static Route_Entry_t EMPTY_ROUTE_ENTRY = {
    .destAddress = UWB_DEST_EMPTY,
    .nextHop = UWB_DEST_EMPTY,
    .hopCount = 0,
    .expirationTime = 0,
    .destSeqNumber = 0,
    .validDestSeqFlag = false,
    .precursors = 0,
    // TODO: init metrics
};

Routing_Table_t *getGlobalRoutingTable() {
  return &routingTable;
}

void routingTableInit(Routing_Table_t *table) {
  table->size = 0;
  table->mu = xSemaphoreCreateMutex();
  for (int i = 0; i < ROUTING_TABLE_SIZE_MAX; i++) {
    table->entries[i] = EMPTY_ROUTE_ENTRY;
  }
}

static void routingTableSwapRouteEntry(Routing_Table_t *table, int first, int second) {
  Route_Entry_t temp = table->entries[first];
  table->entries[first] = table->entries[second];
  table->entries[second] = temp;
}

typedef int (*routeEntryCompareFunc)(Route_Entry_t *, Route_Entry_t *);

static int COMPARE_BY_DEST_ADDRESS(Route_Entry_t *first, Route_Entry_t *second) {
  if (first->destAddress == second->destAddress) {
    return 0;
  }
  if (first->destAddress > second->destAddress) {
    return 1;
  }
  return -1;
}

static int COMPARE_BY_EXPIRATION_TIME(Route_Entry_t *first, Route_Entry_t *second) {
  if (first->expirationTime == second->expirationTime) {
    return 0;
  }
  if (first->expirationTime > second->expirationTime) {
    return -1;
  }
  return 1;
}

static int routingTableGetStalestEntry(Routing_Table_t *table) {
  if (table->size == 0) {
    DEBUG_PRINT("routingTableEvictStalestEntry: Routing table is empty, ignore.\n");
    return -1;
  }
  int stalestIndex = 0;
  for (int i = 0; i < table->size; i++) {
    if (COMPARE_BY_EXPIRATION_TIME(&table->entries[i], &table->entries[stalestIndex]) > 0) {
      stalestIndex = i;
    }
  }
  return stalestIndex;
}

/* Build the heap */
static void routingTableArrange(Routing_Table_t *table, int index, int len, routeEntryCompareFunc compare) {
  int leftChild = 2 * index + 1;
  int rightChild = 2 * index + 2;
  int maxIndex = index;
  if (leftChild < len && compare(&table->entries[maxIndex], &table->entries[leftChild]) < 0) {
    maxIndex = leftChild;
  }
  if (rightChild < len && compare(&table->entries[maxIndex], &table->entries[rightChild]) < 0) {
    maxIndex = rightChild;
  }
  if (maxIndex != index) {
    routingTableSwapRouteEntry(table, index, maxIndex);
    routingTableArrange(table, maxIndex, len, compare);
  }
}

/* Sort the routing table */
static void routingTableRearrange(Routing_Table_t *table, routeEntryCompareFunc compare) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  /* Build max heap */
  for (int i = table->size / 2 - 1; i >= 0; i--) {
    routingTableArrange(table, i, table->size, compare);
  }
  for (int i = table->size - 1; i >= 0; i--) {
    routingTableSwapRouteEntry(table, 0, i);
    routingTableArrange(table, 0, i, compare);
  }
  xSemaphoreGive(table->mu);
}

void routingTableAddEntry(Routing_Table_t *table, Route_Entry_t entry) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  int index = routingTableFindEntry(table, entry.destAddress);
  if (index != -1) {
    DEBUG_PRINT("routingTableAddEntry: Try to add an already added route entry for dest %u, update it instead.\n",
                entry.destAddress);
    table->entries[index] = entry;
  } else {
    ASSERT(ROUTING_TABLE_SIZE_MAX > 0);
    if (table->size == ROUTING_TABLE_SIZE_MAX) {
      int evictedIndex = -1;
      #ifdef ROUTING_TABLE_EVICT_POLICY_STALEST
      evictedIndex = routingTableGetStalestEntry(table);
      #else
      /* Randomly drop a route entry */
      evictedIndex = rand() % table->size;
      #endif
      DEBUG_PRINT("routingTableEvictStalestEntry: Routing table is full, evict stalest route entry for dest %u.\n",
                  table->entries[evictedIndex].destAddress);
      /* Swap it to last */
      int lastEntryIndex = ROUTING_TABLE_SIZE_MAX - 1;
      routingTableSwapRouteEntry(table, evictedIndex, lastEntryIndex);
      /* Clear the last entry */
      table->entries[lastEntryIndex] = EMPTY_ROUTE_ENTRY;
      table->size--;
    }
    /* Add the new entry to the last */
    uint8_t curIndex = table->size;
    table->entries[curIndex] = entry;
    table->size++;
    /* Sort the routing table */
    routingTableRearrange(table, COMPARE_BY_DEST_ADDRESS);
  }
  xSemaphoreGive(table->mu);
}

void routingTableUpdateEntry(Routing_Table_t *table, Route_Entry_t entry) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  int index = routingTableFindEntry(table, entry.destAddress);
  if (index == -1) {
    DEBUG_PRINT("routingTableUpdateEntry: Cannot find correspond route entry for dest %u, add it instead.\n",
                entry.destAddress);
    routingTableAddEntry(table, entry);
  } else {
    table->entries[index] = entry;
    DEBUG_PRINT("routingTableUpdateEntry: Update route entry for dest %u.\n", entry.destAddress);
  }
  xSemaphoreGive(table->mu);
}

void routingTableRemoveEntry(Routing_Table_t *table, UWB_Address_t destAddress) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  if (table->size == 0) {
    DEBUG_PRINT("routingTableRemoveEntry: Routing table is empty, ignore.\n");
    return;
  }
  int index = routingTableFindEntry(table, destAddress);
  if (index == -1) {
    DEBUG_PRINT("routingTableRemoveEntry: Cannot find correspond route entry for dest %u, ignore.\n", destAddress);
    return;
  }
  routingTableSwapRouteEntry(table, index, table->size - 1);
  table->entries[table->size - 1] = EMPTY_ROUTE_ENTRY;
  table->size--;
  routingTableRearrange(table, COMPARE_BY_DEST_ADDRESS);
  xSemaphoreGive(table->mu);
}

int routingTableFindEntry(Routing_Table_t *table, UWB_Address_t targetAddress) {
  /* Binary Search */
  int left = -1, right = table->size;
  while (left + 1 != right) {
    int mid = left + (right - left) / 2;
    if (table->entries[mid].destAddress == targetAddress) {
      return mid;
    } else if (table->entries[mid].destAddress > targetAddress) {
      right = mid;
    } else {
      left = mid;
    }
  }
  return -1;
}

void printRouteEntry(Route_Entry_t entry) {
  DEBUG_PRINT("dest\t next\t hop\t destSeq\t expire\t \n");
  DEBUG_PRINT("%u\t %u\t %u\t %lu\t %lu\t \n",
              entry.destAddress,
              entry.nextHop,
              entry.hopCount,
              entry.destSeqNumber,
              entry.expirationTime);
}

void printRoutingTable(Routing_Table_t *table) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  DEBUG_PRINT("dest\t next\t hop\t destSeq\t expire\t \n");
  for (int i = 0; i < table->size; i++) {
    if (table->entries[i].destAddress == UWB_DEST_EMPTY) {
      continue;
    }
    DEBUG_PRINT("%u\t %u\t %u\t %lu\t %lu\t \n",
                table->entries[i].destAddress,
                table->entries[i].nextHop,
                table->entries[i].hopCount,
                table->entries[i].destSeqNumber,
                table->entries[i].expirationTime);
  }
  DEBUG_PRINT("---\n");
  xSemaphoreGive(table->mu);
}

