#ifndef __ADHOCDECK_H__
#define __ADHOCDECK_H__

#include "libdw3000.h"
#include "mac_802_15_4.h"
#include "queue.h"

/* Function Switch */
 #define UWB_ENABLE_PHR_EXT_MODE

#define UWB_SPEED_OF_LIGHT 299702547
#define UWB_MAX_TIMESTAMP 1099511627776  // 2**40
#define UWB_TX_ANT_DLY 16385
#define UWB_RX_ANT_DLY 16385

#define UWB_FRAME_LEN_STD 127
#define UWB_FRAME_LEN_EXT 1023
#ifdef UWB_ENABLE_PHR_EXT_MODE
  #define UWB_FRAME_LEN_MAX UWB_FRAME_LEN_EXT
#else
  #define UWB_FRAME_LEN_MAX UWB_FRAME_LEN_STD
#endif

/* Queue Constants */
#define UWB_TX_QUEUE_SIZE 10
#define UWB_TX_QUEUE_ITEM_SIZE sizeof(UWB_Packet_t)

/* UWB Packet */
#define UWB_PACKET_SIZE UWB_FRAME_LEN_MAX
#define UWB_PAYLOAD_SIZE (UWB_PACKET_SIZE - sizeof(Packet_Header_t))
#define UWB_DEST_ANY 65535
#define UWB_DEST_ONE_HOP 65534

/* TX options */
static dwt_txconfig_t txconfig_options = {
    .PGcount = 0x0,
    .PGdly = 0x34,
    .power = 0xfdfdfdfd
};

/* PHR configuration */
static dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for
          non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
#ifdef UWB_ENABLE_PHR_EXT_MODE
    DWT_PHRMODE_EXT, /* Extended PHY header mode. */
#else
    DWT_PHRMODE_STD, /* Standard PHY header mode. */
#endif
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e
                     */
    DWT_PDOA_M0     /* PDOA mode off */
};

typedef uint16_t UWB_Address_t;

/* UWB packet definition */
typedef enum {
  UWB_RANGING_MESSAGE = 0,
  UWB_FLOODING_MESSAGE = 1,
  UWB_DATA_MESSAGE = 2,
  UWB_MESSAGE_TYPE_COUNT, /* only used for counting message types. */
} UWB_MESSAGE_TYPE;

typedef struct {
  UWB_Address_t srcAddress;
  UWB_Address_t destAddress;
  uint32_t seqNumber;
  UWB_MESSAGE_TYPE type: 6;
  uint16_t length: 10;
} __attribute__((packed)) Packet_Header_t;

typedef struct {
  Packet_Header_t header; // Packet header
  uint8_t payload[UWB_PAYLOAD_SIZE];
} __attribute__((packed)) UWB_Packet_t;

typedef void (*UWBCallback)(void *);

typedef struct {
  UWB_MESSAGE_TYPE type;
  QueueHandle_t rxQueue;
  UWBCallback rxCb;
  UWBCallback txCb;
} UWB_Message_Listener_t;

/* UWB operations */
uint16_t uwbGetAddress();
int uwbSendPacket(UWB_Packet_t *packet);
int uwbSendPacketBlock(UWB_Packet_t *packet);
int uwbReceivePacket(UWB_MESSAGE_TYPE type, UWB_Packet_t *packet);
int uwbReceivePacketBlock(UWB_MESSAGE_TYPE type, UWB_Packet_t *packet);
int uwbReceivePacketWait(UWB_MESSAGE_TYPE type, UWB_Packet_t *packet, int wait);
void uwbRegisterListener(UWB_Message_Listener_t *listener);

#endif