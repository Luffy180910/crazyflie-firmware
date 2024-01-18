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
static QueueHandle_t rxQueue;
static Routing_Table_t routingTable;
static int seqNumber = 1;

void routingRxCallback(void *parameters) {
//  DEBUG_PRINT("routingRxCallback \n");
}

void routingTxCallback(void *parameters) {
//  DEBUG_PRINT("routingTxCallback \n");
}

static void processRoutingDataMessage(UWB_Packet_t *packet) {
  // TODO
}

static void uwbRoutingTxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t txPacketCache;
  txPacketCache.header.srcAddress = uwbGetAddress();
  txPacketCache.header.destAddress = UWB_DEST_EMPTY;
  txPacketCache.header.type = UWB_DATA_MESSAGE;
  txPacketCache.header.length = 0;

  while (true) {
    // TODO
    printRoutingTable(&routingTable);
    vTaskDelay(M2T(2000));
  }
}

static void uwbRoutingRxTask(void *parameters) {
  systemWaitStart();

  UWB_Packet_t rxPacketCache;

  while (true) {
    if (uwbReceivePacketBlock(UWB_DATA_MESSAGE, &rxPacketCache)) {
      processRoutingDataMessage(&rxPacketCache);
    }
  }
}

void routingInit() {
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

/* Routing Table Operations */
static Route_Entry_t EMPTY_ROUTE_ENTRY = {
    .destAddress = UWB_DEST_EMPTY,
    .nextHop = UWB_DEST_EMPTY,
    .hopCount = 0,
    .expirationTime = 0,
    .destSeqNumber = 0,
    .validDestSeqFlag =false,
    .precursors = 0,
    // TODO: init metrics
};

Routing_Table_t getGlobalRoutingTable() {
  return routingTable;
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

// TODO: check
static int COMPARE_BY_EXPIRATION_TIME(Route_Entry_t *first, Route_Entry_t *second) {
  if (first->expirationTime == second->expirationTime) {
    return 0;
  }
  if (first->expirationTime > second->expirationTime) {
    return -1;
  }
  return 1;
}

// TODO: test
static void routingTableEvictStalestEntry(Routing_Table_t *table) {
  if (table->size == 0) {
    DEBUG_PRINT("routingTableEvictStalestEntry: Routing table is empty, ignore.\n");
    return;
  }
  int stalestIndex = 0;
  for (int i = 0; i < table->size; i++) {
    if (COMPARE_BY_EXPIRATION_TIME(&routingTable.entries[i], &routingTable.entries[stalestIndex]) > 0) {
      stalestIndex = i;
    }
  }
  DEBUG_PRINT("routingTableEvictStalestEntry: Routing table is full, evict stalest route entry for dest %u.\n",
              table->entries[stalestIndex].destAddress);
  routingTableRemoveEntry(table, table->entries[stalestIndex].destAddress);
}

/* Rebuild the heap (Sort the routing table) */
static void routingTableRearrange(Routing_Table_t *table, int start, int end, routeEntryCompareFunc compare) {
  ASSERT(start >= 0);
  ASSERT(end < table->size);
  ASSERT(start <= end);

  xSemaphoreTake(table->mu, portMAX_DELAY);
  // TODO
  xSemaphoreGive(table->mu);
}

// TODO: test
void routingTableAddEntry(Routing_Table_t *table, Route_Entry_t entry) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  int index = routingTableFindEntry(table, entry.destAddress);
  if (index != -1) {
    DEBUG_PRINT("routingTableAddEntry: Try to add an already added route entry for dest %u, update it instead.\n",
                entry.destAddress);
    table->entries[index] = entry;
  } else {
    if (table->size == ROUTING_TABLE_SIZE_MAX) {
      #ifdef ROUTING_TABLE_EVICT_POLICY_STALEST
      routingTableEvictStalestEntry(table);
      #else
      /* Randomly drop a route entry */
      int randomIndex = rand() % table->size;
      routingTableRemoveEntry(table, table->entries[randomIndex].destAddress);
      #endif
    }
    /* Add the new entry to the last */
    uint8_t curIndex = table->size;
    table->entries[curIndex] = entry;
    table->size++;
    // TODO: heap operation
  }
  xSemaphoreGive(table->mu);
}

// TODO: test
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
    // TODO: heap operation
  }
  xSemaphoreGive(table->mu);
}

// TODO: test & check
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
  // TODO: heap operation
  routingTableRearrange(table, index, table->size - 1, COMPARE_BY_DEST_ADDRESS);
  xSemaphoreGive(table->mu);
}

// TODO: test
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
  DEBUG_PRINT("%u\t %u\t %u\n", entry.destAddress, entry.nextHop, entry.hopCount);
}

void printRoutingTable(Routing_Table_t *table) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  DEBUG_PRINT("dest\t next\t hop\t \n");
  for (int i = 0; i < table->size; i++) {
    if (table->entries[i].destAddress == UWB_DEST_EMPTY) {
      continue;
    }
    DEBUG_PRINT("%u\t %u\t %u\n", table->entries[i].destAddress, table->entries[i].nextHop, table->entries[i].hopCount);
  }
  DEBUG_PRINT("---\n");
  xSemaphoreGive(table->mu);
}

