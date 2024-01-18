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
  txPacketCache.header.destAddress = UWB_EMPTY_DEST_ADDRESS;
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

  xTaskCreate(uwbRoutingTxTask, ADHOC_DECK_ROUTING_TX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRoutingTxTaskHandle); // TODO optimize STACK SIZE
  xTaskCreate(uwbRoutingRxTask, ADHOC_DECK_ROUTING_RX_TASK_NAME, 4 * configMINIMAL_STACK_SIZE, NULL,
              ADHOC_DECK_TASK_PRI, &uwbRoutingRxTaskHandle); // TODO optimize STACK SIZE
}

/* Routing Table Operations */
void routingTableInit(Routing_Table_t *table) {
  table->mu = xSemaphoreCreateMutex();
  Route_Entry_t empty = {
      .destAddress = UWB_EMPTY_DEST_ADDRESS,
      .nextHop = UWB_EMPTY_DEST_ADDRESS,
      .hopCount = 0,
      .expirationTime = 0,
      .destSeqNumber = 0,
      .validDestSeqFlag =false,
      .precursors = 0,
      // TODO: init metrics
  };
  for (int i = 0; i < ROUTING_TABLE_SIZE; i++) {
    table->entries[i] = empty;
  }
}

void routingTableAddEntry(Routing_Table_t *table, Route_Entry_t entry) {
  // TODO
}

void routingTableUpdateEntry(Routing_Table_t *table, Route_Entry_t entry) {
  // TODO
}

void routingTableRemoveEntry(Routing_Table_t *table, UWB_Address_t destAddress) {
  // TODO
}

//Route_Entry_t routingTableFindEntry(Routing_Table_t *table, UWB_Address_t destAddress) {
//  // TODO
//}

void printRoutingTable(Routing_Table_t *table) {
  xSemaphoreTake(table->mu, portMAX_DELAY);
  DEBUG_PRINT("dest\t next\t hop\t \n");
  for (int i = 0; i < ROUTING_TABLE_SIZE; i++) {
    DEBUG_PRINT("%u\t %u\t %u\n",
                table->entries[i].destAddress,
                table->entries[i].nextHop,
                table->entries[i].hopCount);
  }
  DEBUG_PRINT("---------------\n");
  xSemaphoreGive(table->mu);
}

