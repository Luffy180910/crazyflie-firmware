#define DEBUG_MODULE "sendData"

#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "system.h"
#include "adhocdeck.h"
#include "usb.h"
#include "debug.h"
#include "usb_tran_data.h"


static void sendDataTask(void *parameters)
{
    systemWaitStart();
    SendData_Meta_t sendData_Meta = {
        .magic = 0xBB88,
        .jitter = 0,
        .period = 0,
        .msgLength = 0};

    while (1)
    {
        vTaskDelay(1000);
        while (startStatistic == 2)
        {
            vTaskDelay(1);
            // 300KB每秒
            sendData_Meta.jitter = jitter;
            sendData_Meta.period = TX_PERIOD_IN_MS;
            // 1. 传送lastSuccRangingSeq
            uint16_t msgSize = sizeof(lastSuccRangingSeq);
            sendData_Meta.msgLength = msgSize;
            sendData_Meta.type = 1;
            sendDataWithArray(sendData_Meta, msgSize, lastSuccRangingSeq);

            // 2. 传送lastSuccRxPacketSeq
            msgSize = sizeof(lastSuccRxPacketSeq);
            sendData_Meta.msgLength = msgSize;
            sendData_Meta.type = 2;
            sendDataWithArray(sendData_Meta, msgSize, lastSuccRxPacketSeq);

            // 3. 传送continuousLossPacketCount
            msgSize = sizeof(continuousLossPacketCount);
            sendData_Meta.msgLength = msgSize;
            sendData_Meta.type = 3;
            sendDataWithArray(sendData_Meta, msgSize, continuousLossPacketCount);

            // 4. 传送continuousRangingFailCount
            msgSize = sizeof(continuousRangingFailCount);
            sendData_Meta.msgLength = msgSize;
            sendData_Meta.type = 4;
            sendDataWithArray(sendData_Meta, msgSize, continuousRangingFailCount);
            // 5. 传送rxPacketCount
            msgSize = sizeof(rxPacketCount);
            sendData_Meta.msgLength = msgSize;
            sendData_Meta.type = 5;
            sendDataWithArray(sendData_Meta, msgSize, rxPacketCount);
            // 6. 传送rangingSuccCount
            msgSize = sizeof(rangingSuccCount);
            sendData_Meta.msgLength = msgSize;
            sendData_Meta.type = 6;
            sendDataWithArray(sendData_Meta, msgSize, rangingSuccCount);
        }
    }
}
void sendDataWithArray(SendData_Meta_t sendData_Meta, uint16_t msgSize, uint8_t *pointer)
{
    usbSendData(sizeof(SendData_Meta_t), sendData_Meta.raw);

    uint8_t *pointer_send = pointer;
    int remain = msgSize;
    while (remain > 0)
    {
        int sizeToSend = remain > USB_RX_TX_PACKET_SIZE ? USB_RX_TX_PACKET_SIZE : remain;
        usbSendData(sizeToSend, pointer_send);
        pointer_send += sizeToSend;
        remain -= sizeToSend;
    }
}
