/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_direct.h"

#define BASE_PORT 5760

static const struct serialPortVTable directVTable; // Forward
static directPort_t directSerialPorts[SERIAL_PORT_COUNT];
static bool directPortInitialized[SERIAL_PORT_COUNT];

static uartCreatedCallback uartCreatedCall = NULL;
static uartDataCallback uartDataCall = NULL;

PLUGIN_EXPORT void setUartCreatedCallback(uartCreatedCallback callback) {
    uartCreatedCall = callback;
}
PLUGIN_EXPORT void setUartDataCallback(uartDataCallback callback) {
    uartDataCall = callback;
}

static directPort_t* directReconfigure(directPort_t *s, int id)
{
    if (directPortInitialized[id]) {
        fprintf(stderr, "port is already initialized!\n");
        return s;
    }

    directPortInitialized[id] = true;
    if (uartCreatedCall) {
        uartCreatedCall(id);
    }
    s->id = id;
    return s;
}

serialPort_t *serDirectOpen(serialPortIdentifier_e identifier, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    directPort_t *s = NULL;

    int id = findSerialPortIndexByIdentifier(identifier);

    if (id >= 0 && id < (int)ARRAYLEN(directSerialPorts)) {
        s = directReconfigure(&directSerialPorts[id], id);
    }

    if (!s) {
        return NULL;
    }

    s->port.vTable = &directVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.txBufferHead = s->port.txBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.txBufferSize = TX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;
    s->port.txBuffer = s->txBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

static uint32_t directTotalRxBytesWaiting(const serialPort_t *instance)
{
    directPort_t *s = (directPort_t*)instance;
    uint32_t count;
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }

    return count;
}

static uint32_t directTotalTxBytesFree(const serialPort_t *instance)
{
    directPort_t *s = (directPort_t*)instance;
    uint32_t bytesUsed;

    if (s->port.txBufferHead >= s->port.txBufferTail) {
        bytesUsed = s->port.txBufferHead - s->port.txBufferTail;
    } else {
        bytesUsed = s->port.txBufferSize + s->port.txBufferHead - s->port.txBufferTail;
    }
    uint32_t bytesFree = (s->port.txBufferSize - 1) - bytesUsed;

    return bytesFree;
}

static bool isdirectTransmitBufferEmpty(const serialPort_t *instance)
{
    directPort_t *s = (directPort_t *)instance;
    bool isEmpty = s->port.txBufferTail == s->port.txBufferHead;
    return isEmpty;
}

static uint8_t directRead(serialPort_t *instance)
{
    uint8_t ch;
    directPort_t *s = (directPort_t *)instance;

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }

    return ch;
}

static void uartWriteInternal(int id, const void* data, const int size) {
    if (uartDataCall) {
        uartDataCall(id, data, size);
    }
}

void uartDataOut(directPort_t *instance)
{
    directPort_t *s = instance;

    if (s->port.txBufferHead < s->port.txBufferTail) {
        // send data till end of buffer
        int chunk = s->port.txBufferSize - s->port.txBufferTail;
        uartWriteInternal(s->id, (const void *)&s->port.txBuffer[s->port.txBufferTail], chunk);
        s->port.txBufferTail = 0;
    }
    int chunk = s->port.txBufferHead - s->port.txBufferTail;
    if (chunk)
        uartWriteInternal(s->id, (const void*)&s->port.txBuffer[s->port.txBufferTail], chunk);
    s->port.txBufferTail = s->port.txBufferHead;
}

static void uartWrite(serialPort_t *instance, uint8_t ch)
{
    directPort_t *s = (directPort_t *)instance;

    s->port.txBuffer[s->port.txBufferHead] = ch;
    if (s->port.txBufferHead + 1 >= s->port.txBufferSize) {
        s->port.txBufferHead = 0;
    } else {
        s->port.txBufferHead++;
    }

    uartDataOut(s);
}

void directDataIn(directPort_t *s, uint8_t* ch, int size)
{
    while (size--) {
        s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
        if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferHead = 0;
        } else {
            s->port.rxBufferHead++;
        }
    }
}

void uartDataIn(int id, const void* data, const int size) {
    if (id < 0 || id >= SERIAL_PORT_COUNT || !directPortInitialized[id]) {
        abort();
    }
    directDataIn(directSerialPorts + id, (uint8_t*)data, (int)size);
}

static const struct serialPortVTable directVTable = {
        .serialWrite = uartWrite,
        .serialTotalRxWaiting = directTotalRxBytesWaiting,
        .serialTotalTxFree = directTotalTxBytesFree,
        .serialRead = directRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isdirectTransmitBufferEmpty,
        .setMode = NULL,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = NULL,
        .beginWrite = NULL,
        .endWrite = NULL,
};
