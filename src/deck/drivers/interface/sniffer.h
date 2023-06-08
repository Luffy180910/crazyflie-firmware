#ifndef _ADHOCDECK_SNIFFER_H_
#define _ADHOCDECK_SNIFFER_H_

#include "adhocdeck.h"

/* Queue Constants */
#define SNIFFER_RX_QUEUE_SIZE 10
#define SNIFFER_RX_QUEUE_ITEM_SIZE sizeof(UWB_Packet_t)

void snifferInit();

#endif
