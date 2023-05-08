#ifndef __ROUTING_STRUCT_H__
#define __ROUTING_STRUCT_H__

#include "flooding_struct.h"

#define ROUTING_NODE_MAX_SIZE 20
#define ROUTING_INF 34463

/* Routing Node State Table */
typedef struct {
    uint16_t address;
    bool visited;
    uint16_t distance;
} __attribute__((packed)) Routing_Node_State_Table_t;

typedef struct {
    set_index_t next;
    Routing_Node_State_Table_t data;
} __attribute__((packed)) Routing_Node_State_Table_Set_Item_t;

typedef struct {
    Routing_Node_State_Table_Set_Item_t setData[ROUTING_NODE_MAX_SIZE];
    set_index_t freeQueueEntry;
    set_index_t fullQueueEntry;
    int size;
} Routing_Node_State_Table_Set_t;

/* Routing Node State Table Set Operations */
void routingNodeStateTableSetInit(Routing_Node_State_Table_Set_t *routingNodeStateTableSet);
set_index_t routingNodeStateTableSetInsert(Routing_Node_State_Table_Set_t *routingNodeStateTableSet,
                                           uint16_t dstAddress, uint16_t distance);
set_index_t findInRoutingNodeStateTableSet(Routing_Node_State_Table_Set_t *routingNodeStateTableSet,
                                           uint16_t dstAddress);

/* Routing Priority Queue */
typedef struct {
    uint8_t size;
    uint8_t maxSize;
    Routing_Node_State_Table_t queueData[ROUTING_NODE_MAX_SIZE];
} __attribute__((packed)) Routing_Priority_Queue_t;

/* Routing Priority Queue Operations */
void routingPriorityQueueInit(Routing_Priority_Queue_t *routingPriorityQueue);
bool isRoutingPriorityQueueEmpty(Routing_Priority_Queue_t *routingPriorityQueue);
bool isRoutingPriorityQueueFull(Routing_Priority_Queue_t *routingPriorityQueue);
bool routingPriorityQueuePush(Routing_Priority_Queue_t *routingPriorityQueue, Routing_Node_State_Table_t *item);
// TODO: Check
bool routingPriorityQueueGetFront(Routing_Priority_Queue_t *routingPriorityQueue, Routing_Node_State_Table_t *item);
bool routingPriorityQueuePopFront(Routing_Priority_Queue_t *routingPriorityQueue);

/* Routing Shortest Tree Operations */
void routingShortestTreeClear(uint16_t *routingShortestTree);
uint16_t routingShortestTreeFindRoute(uint16_t *routingShortestTree, uint16_t dstAddress, uint16_t myAddress);

#endif