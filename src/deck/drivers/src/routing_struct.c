#include "routing_struct.h"
#include "debug.h"
#include <string.h>

/* Routing Node State Table Set Operations */
static set_index_t routingNodeStateTableSetMalloc(Routing_Node_State_Table_Set_t *routingNodeStateTableSet) {
    if (routingNodeStateTableSet->freeQueueEntry == -1) {
        DEBUG_PRINT("Routing Node State Table Set is FULL, malloc failed.\n");
        return -1;
    } else {
        set_index_t candidate = routingNodeStateTableSet->freeQueueEntry;
        routingNodeStateTableSet->freeQueueEntry = routingNodeStateTableSet->setData[candidate].next;
        set_index_t temp = routingNodeStateTableSet->fullQueueEntry;
        routingNodeStateTableSet->fullQueueEntry = candidate;
        routingNodeStateTableSet->setData[candidate].next = temp;
        return candidate;
    }
}

void routingNodeStateTableSetInit(Routing_Node_State_Table_Set_t *routingNodeStateTableSet){
    set_index_t i;
    for(i = 0; i < ROUTING_NODE_MAX_SIZE - 1; i++) {
        routingNodeStateTableSet->setData[i].data.distance = ROUTING_INF;
        routingNodeStateTableSet->setData[i].data.visited = false;
        routingNodeStateTableSet->setData[i].next = i + 1;
    }
    routingNodeStateTableSet->setData[i].data.distance = ROUTING_INF;
    routingNodeStateTableSet->setData[i].next = -1;
    routingNodeStateTableSet->setData[i].data.visited = false;

    routingNodeStateTableSet->freeQueueEntry = 0;
    routingNodeStateTableSet->fullQueueEntry = -1;
    routingNodeStateTableSet->size = 0;
}

set_index_t routingNodeStateTableSetInsert(Routing_Node_State_Table_Set_t *routingNodeStateTableSet,
                                           uint16_t dstAddress, uint16_t distance){
    set_index_t candidate = routingNodeStateTableSetMalloc(routingNodeStateTableSet);
    if (candidate != -1) {
        routingNodeStateTableSet->setData[candidate].data.address = dstAddress;
        routingNodeStateTableSet->setData[candidate].data.distance = distance;
        routingNodeStateTableSet->size++;
    }
    return candidate;
}

set_index_t findInRoutingNodeStateTableSet(Routing_Node_State_Table_Set_t *routingNodeStateTableSet,
                                           uint16_t dstAddress) {
    set_index_t candidate = routingNodeStateTableSet->fullQueueEntry;
    while (candidate != -1) {
        if (routingNodeStateTableSet->setData[candidate].data.address == dstAddress) {
            break;
        }
        candidate = routingNodeStateTableSet->setData[candidate].next;
    }
    return candidate;
}

/* Routing Priority Queue Operations */
void routingPriorityQueueInit(Routing_Priority_Queue_t *routingPriorityQueue) {
    memset(routingPriorityQueue, 0, sizeof(Routing_Priority_Queue_t));
    routingPriorityQueue->maxSize = ROUTING_NODE_MAX_SIZE;
    routingPriorityQueue->size = 0;
}

bool isRoutingPriorityQueueEmpty(Routing_Priority_Queue_t *routingPriorityQueue) {
    if (routingPriorityQueue == NULL) return false;
    else if (routingPriorityQueue->size == 0) return true;
    else return false;
}

bool isRoutingPriorityQueueFull(Routing_Priority_Queue_t *routingPriorityQueue) {
    if (routingPriorityQueue == NULL) return false;
    else if (routingPriorityQueue->size == routingPriorityQueue->maxSize) return true;
    else return false;
}

bool routingPriorityQueuePush(Routing_Priority_Queue_t *routingPriorityQueue, Routing_Node_State_Table_t *item) {
    if (isRoutingPriorityQueueFull(routingPriorityQueue)) return false;
    set_index_t i;
    for (i = routingPriorityQueue->size + 1; item->distance < routingPriorityQueue->queueData[i / 2].distance && i > 1; i /= 2) {
        routingPriorityQueue->queueData[i] = routingPriorityQueue->queueData[i / 2];
    }
    memcpy(&routingPriorityQueue->queueData[i], item, sizeof(Routing_Node_State_Table_t));
    routingPriorityQueue->size++;
    return true;
}

bool routingPriorityQueueGetFront(Routing_Priority_Queue_t *routingPriorityQueue, Routing_Node_State_Table_t *item) {
    if (isRoutingPriorityQueueEmpty(routingPriorityQueue)) return false;
    memcpy(item, &routingPriorityQueue->queueData[1], sizeof(Routing_Node_State_Table_t));
    return true;
}

bool routingPriorityQueuePopFront(Routing_Priority_Queue_t *routingPriorityQueue) {
    if (isRoutingPriorityQueueEmpty(routingPriorityQueue)) return false;
    Routing_Node_State_Table_t *lastItem = &routingPriorityQueue->queueData[routingPriorityQueue->size];
    routingPriorityQueue->size--;
    if (routingPriorityQueue->size == 0) {
        memset(routingPriorityQueue->queueData, 0, (routingPriorityQueue->maxSize) * sizeof(Routing_Node_State_Table_t));
        return true;
    }

    int i, minChild = 0;
    for (i = 1; i * 2 <= routingPriorityQueue->size; i = minChild) {
        minChild = i * 2;
        if (routingPriorityQueue->queueData[minChild].distance > routingPriorityQueue->queueData[minChild + 1].distance && 
            minChild != routingPriorityQueue->size) {
            minChild += 1;
        }
        if (lastItem->distance > routingPriorityQueue->queueData[minChild].distance) {
            memcpy(&routingPriorityQueue->queueData[i], &routingPriorityQueue->queueData[minChild], sizeof(Routing_Node_State_Table_t));
        } else {
            break;
        }
    }
    memcpy(&routingPriorityQueue->queueData[i], lastItem, sizeof(Routing_Node_State_Table_t));

    return true;
}

/* Routing Shortest Tree Operations */
void routingShortestTreeClear(uint16_t *routingShortestTree) {
    memset(routingShortestTree, 0, ROUTING_NODE_MAX_SIZE * sizeof(uint16_t));
}

uint16_t routingShortestTreeFindRoute(uint16_t *routingShortestTree, uint16_t dstAddress, uint16_t myAddress) {
    uint16_t nextAddress = dstAddress;
    uint16_t parentAddress = routingShortestTree[dstAddress];
    while (parentAddress != 0) {
        if (parentAddress == myAddress) {
            return nextAddress;
        }
        nextAddress = parentAddress;
        parentAddress = routingShortestTree[parentAddress];
    }
    return ROUTING_INF;
}