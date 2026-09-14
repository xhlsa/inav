/*
 * This file is part of INAV.
 *
 * INAV is free software. You can redistribute this software
 * and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * INAV is distributed in the hope that it
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
 * Based on https://github.com/ExpressLRS/ExpressLRS
 * Thanks to AlessandroAU, original creator of the ExpressLRS project.
 *
 * Ported from Betaflight drivers/rx/rx_sx1280.c.
 *
 * Betaflight drives the packet path as a chain of non-blocking DMA SPI
 * sequences: each step's completion callback waits for the radio BUSY line to
 * fall (via EXTI) and then starts the next step. INAV's SPI layer is blocking,
 * so each step's transfer here is performed synchronously inside the callback
 * that starts it (a few bytes, well under 100us), while the BUSY waits between
 * steps remain interrupt driven exactly as in Betaflight. The radio is the only
 * device on its SPI bus.
 *
 * Mutual exclusion:
 *  - A packet chain (DIO1 EXTI) or FHSS chain (tock timer) only starts if no
 *    other chain is in progress (sx1280Processing).
 *  - Blocking accessors used from task context (radio configuration) wait for
 *    the chain to finish and then transfer with the radio IRQs masked, so a
 *    transfer can never be interleaved with a chain step on the wire.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_SX1280

#include "build/atomic.h"
#include "build/debug.h"

#include "common/utils.h"

#include "drivers/bus.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "drivers/rx/rx_spi.h"
#include "drivers/rx/rx_sx1280.h"

#include "rx/expresslrs.h"
#include "rx/expresslrs_common.h"
#include "rx/expresslrs_impl.h"

// Radio IRQ sources that can start or advance a chain
#define NVIC_PRIO_SX1280_ALL NVIC_PRIO_RX_ELRS_TIMER

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;

typedef busStatus_e sx1280SegmentCallback(void);

typedef struct sx1280Segment_s {
    const uint8_t *txData;
    uint8_t *rxData;
    uint8_t length;
    bool negateCS;                      // End of this CS-asserted transfer
    sx1280SegmentCallback *callback;    // Called after a transfer ending with negateCS
} sx1280Segment_t;

// The following global variables are accessed from interrupt context to process the sequence of steps in packet processing
// As there is only ever one device, no need to add a device context; globals will do
static dioReasonFlags_e irqReason; // Used to pass irq status from sx1280IrqStatusRead() to sx1280ProcessIrq()
static volatile uint8_t packetStats[2];
static uint8_t FIFOaddr; // Used to pass data from sx1280GotFIFOAddr() to sx1280DoReadBuffer()

static IO_t busy;

static extiCallbackRec_t busyExti;

static volatile timeUs_t sx1280Processing;

static volatile bool pendingDoFHSS = false;
static sx1280PacketTypes_e sx1280PacketMode;

#define SX1280_BUSY_TIMEOUT_US 1000
// Time a task-level access waits for an in-progress chain before forcing through
#define SX1280_TASK_WAIT_TIMEOUT_US 5000

static const sx1280Segment_t *currentSegment;

// Perform all transfers of a segment list synchronously. Segments up to and
// including one with negateCS set are sent with CS held low.
static void sx1280RunSegments(const sx1280Segment_t *segments)
{
    busDevice_t *dev = rxSpiGetDevice();
    busTransferDescriptor_t transfers[4];
    int count = 0;

    for (const sx1280Segment_t *seg = segments; seg->length != 0; seg++) {
        transfers[count].rxBuf = seg->rxData;
        transfers[count].txBuf = seg->txData;
        transfers[count].length = seg->length;
        count++;

        if (seg->negateCS || count == ARRAYLEN(transfers)) {
            // The tick timer may preempt the EXTI chain and force a new chain
            // (sx1280HandleFromTick); never let that start mid-transfer.
            ATOMIC_BLOCK(NVIC_PRIO_SX1280_ALL) {
                busTransferMultiple(dev, transfers, count);
            }
            count = 0;

            if (seg->callback) {
                currentSegment = seg;
                if (seg->callback() == BUS_ABORT) {
                    return;
                }
            }
        }
    }
}

bool sx1280IsBusy(void)
{
    return IORead(busy);
}

static bool sx1280PollBusy(void)
{
    uint32_t startTime = micros();
    while (IORead(busy)) {
        if ((micros() - startTime) > SX1280_BUSY_TIMEOUT_US) {
            return false;
        } else {
            // Ensure a service window exists for interrupts
            __NOP();
        }
    }
    return true;
}

static bool sx1280MarkBusy(void)
{
    // Check that there isn't already a sequence of accesses to the SX1280 in progress
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (sx1280Processing) {
            return false;
        }

        sx1280Processing = micros() | 1;
    }

    return true;
}

static void sx1280ClearBusyFn(void)
{
    EXTIEnable(busy, false);
}

static void sx1280BusyExtiHandler(extiCallbackRec_t *cb);

static extiHandlerCallback *busyWaitingFn;

static void sx1280BusyExtiHandler(extiCallbackRec_t *cb)
{
    if (busyWaitingFn) {
        busyWaitingFn(cb);
    }
}

// Run waitingFn as soon as the radio is no longer busy: now if it is idle,
// else from the BUSY falling edge interrupt.
// waitingFn() must call sx1280ClearBusyFn() to prevent repeated calls
static void sx1280SetBusyFn(extiHandlerCallback *waitingFn)
{
    bool sx1280Busy;

    ATOMIC_BLOCK(NVIC_PRIO_RX_BUSY_EXTI) {
        // Drop a falling edge latched while nobody was waiting for it
        EXTIClearPending(busy);
        sx1280Busy = IORead(busy);
        if (sx1280Busy) {
            busyWaitingFn = waitingFn;
            EXTIEnable(busy, true);
        } else {
            EXTIEnable(busy, false);
        }
    }

    if (!sx1280Busy) {
        waitingFn(&busyExti);
    }
}

static void sx1280MarkFree(void)
{
    // Mark that current sequence of accesses is concluded
    sx1280Processing = (timeUs_t)0;
}

// Switch to waiting for EXTI interrupt
static void sx1280EnableExti(void)
{
    sx1280MarkFree();
    rxSpiEnableExti();
}

// Unlikely as it is for the code to lock up waiting on a busy SX1280, we can't afford the risk
// If this routine is called twice in succession whilst waiting on the same busy, force the code to advance
// Called from the Tick timer
bool sx1280HandleFromTick(void)
{
    // Grab a copy to prevent a race condition
    timeUs_t startTime = sx1280Processing;

    if (startTime) {
        // No operation should take SX1280_BUSY_TIMEOUT_US us
        if (cmpTimeUs(micros(), startTime) > SX1280_BUSY_TIMEOUT_US) {
            // Brute force abandon the current sequence of operations
            sx1280ClearBusyFn();
            // Renable EXTI
            sx1280EnableExti();

            return true;
        }
    }

    return false;
}

// Blocking full-duplex transfer for use outside the interrupt chain
static void sx1280TransferBlocking(uint8_t *data, uint8_t length)
{
    const timeUs_t startTime = micros();

    while (true) {
        bool done = false;
        const bool timedOut = cmpTimeUs(micros(), startTime) > SX1280_TASK_WAIT_TIMEOUT_US;

        ATOMIC_BLOCK(NVIC_PRIO_SX1280_ALL) {
            if (timedOut && sx1280Processing) {
                // A chain has stalled (e.g. with the tick timer stopped nothing
                // else would recover it). Abandon it; the caller reconfigures the
                // radio and sx1280StartReceiving() re-arms the DIO interrupt.
                sx1280ClearBusyFn();
                sx1280MarkFree();
            }
            if ((!sx1280Processing && !IORead(busy)) || timedOut) {
                rxSpiTransferCommandMulti(data, length);
                done = true;
            }
        }

        if (done) {
            return;
        }

        __NOP();
    }
}

bool sx1280Init(IO_t resetPin, IO_t busyPin)
{
    if (!rxSpiExtiConfigured()) {
        return false;
    }

    if (resetPin) {
        IOInit(resetPin, OWNER_RX, RESOURCE_OUTPUT, 0);
        IOConfigGPIO(resetPin, IOCFG_OUT_PP);
    }

    if (!busyPin) {
        return false;
    }

    IOInit(busyPin, OWNER_RX, RESOURCE_INPUT, 0);
    IOConfigGPIO(busyPin, IOCFG_IN_FLOATING);

    busy = busyPin;

    if (resetPin) {
        IOLo(resetPin);
        delay(50);
        IOConfigGPIO(resetPin, IOCFG_IN_FLOATING); // leave floating, internal pullup on sx1280 side
        delay(20);
    }

    uint16_t firmwareRev = (((sx1280ReadRegister(SX1280_REG_FIRMWARE_VERSION_MSB)) << 8) | (sx1280ReadRegister(SX1280_REG_FIRMWARE_VERSION_MSB + 1)));
    if ((firmwareRev == 0) || (firmwareRev == 65535)) {
        return false;
    }

    // BUSY falling edge advances the packet chain
    EXTIHandlerInit(&busyExti, sx1280BusyExtiHandler);
#if defined(AT32F43x)
    EXTIConfig(busy, &busyExti, NVIC_PRIO_RX_BUSY_EXTI, EXINT_TRIGGER_FALLING_EDGE);
#elif defined(STM32F7) || defined(STM32H7)
    EXTIConfig(busy, &busyExti, NVIC_PRIO_RX_BUSY_EXTI, IO_CONFIG(GPIO_MODE_IT_FALLING, GPIO_SPEED_FREQ_LOW, GPIO_NOPULL));
#else
    EXTIConfig(busy, &busyExti, NVIC_PRIO_RX_BUSY_EXTI, EXTI_Trigger_Falling);
#endif
    EXTIEnable(busy, false);

    sx1280SetMode(SX1280_MODE_STDBY_RC);
    sx1280WriteCommand(SX1280_RADIO_SET_AUTOFS, 0x01);
    sx1280WriteRegister(SX1280_REG_RX_GAIN_REGIME, sx1280ReadRegister(SX1280_REG_RX_GAIN_REGIME) | 0xC0); //default is low power mode, switch to high sensitivity instead

    return true;
}

void sx1280WriteCommand(const uint8_t address, const uint8_t data)
{
    uint8_t outBuffer[2] = { address, data };

    sx1280PollBusy();
    sx1280TransferBlocking(outBuffer, sizeof(outBuffer));
}

void sx1280WriteCommandBurst(const uint8_t address, const uint8_t *data, const uint8_t length)
{
    uint8_t outBuffer[length + 1];

    outBuffer[0] = address;

    memcpy(outBuffer + 1, data, length);

    sx1280PollBusy();
    sx1280TransferBlocking(&outBuffer[0], length + 1);
}

void sx1280ReadCommandBurst(const uint8_t address, uint8_t *data, const uint8_t length)
{
    uint8_t outBuffer[length + 2];

    outBuffer[0] = address;
    outBuffer[1] = 0x00;

    memcpy(outBuffer + 2, data, length);

    sx1280PollBusy();
    sx1280TransferBlocking(&outBuffer[0], length + 2);
    memcpy(data, outBuffer + 2, length);
}

void sx1280WriteRegisterBurst(const uint16_t address, const uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 3];

    outBuffer[0] = (uint8_t) SX1280_RADIO_WRITE_REGISTER;
    outBuffer[1] = ((address & 0xFF00) >> 8);
    outBuffer[2] = (address & 0x00FF);

    memcpy(outBuffer + 3, buffer, size);

    sx1280PollBusy();
    sx1280TransferBlocking(&outBuffer[0], size + 3);
}

void sx1280WriteRegister(const uint16_t address, const uint8_t value)
{
    sx1280WriteRegisterBurst(address, &value, 1);
}

void sx1280ReadRegisterBurst(const uint16_t address, uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 4];

    outBuffer[0] = (uint8_t) SX1280_RADIO_READ_REGISTER;
    outBuffer[1] = ((address & 0xFF00) >> 8);
    outBuffer[2] = (address & 0x00FF);
    outBuffer[3] = 0x00;

    sx1280PollBusy();
    sx1280TransferBlocking(&outBuffer[0], size + 4);
    memcpy(buffer, outBuffer + 4, size);
}

uint8_t sx1280ReadRegister(const uint16_t address)
{
    uint8_t data;
    sx1280ReadRegisterBurst(address, &data, 1);
    return data;
}

void sx1280WriteBuffer(const uint8_t offset, const uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 2];

    outBuffer[0] = (uint8_t) SX1280_RADIO_WRITE_BUFFER;
    outBuffer[1] = offset;

    memcpy(outBuffer + 2, buffer, size);

    sx1280PollBusy();
    sx1280TransferBlocking(&outBuffer[0], size + 2);
}

void sx1280ReadBuffer(const uint8_t offset, uint8_t *buffer, const uint8_t size)
{
    uint8_t outBuffer[size + 3];

    outBuffer[0] = (uint8_t) SX1280_RADIO_READ_BUFFER;
    outBuffer[1] = offset;
    outBuffer[2] = 0x00;

    sx1280PollBusy();
    sx1280TransferBlocking(&outBuffer[0], size + 3);
    memcpy(buffer, outBuffer + 3, size);
}

uint8_t sx1280GetStatus(void)
{
    uint8_t buffer[3] = {(uint8_t) SX1280_RADIO_GET_STATUS, 0, 0};
    sx1280PollBusy();
    sx1280TransferBlocking(&buffer[0], 3);
    return buffer[0];
}

static void sx1280ConfigModParamsLora(const sx1280LoraBandwidths_e bw, const sx1280LoraSpreadingFactors_e sf, const sx1280LoraCodingRates_e cr)
{
    uint8_t rfparams[3];
    rfparams[0] = sf;
    rfparams[1] = bw;
    rfparams[2] = cr;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, 3);

    switch (sf) {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        sx1280WriteRegister(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x1E); // SF5 or SF6
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        sx1280WriteRegister(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x37); // SF7 or SF8
        break;
    default:
        sx1280WriteRegister(SX1280_REG_SF_ADDITIONAL_CONFIG, 0x32); // SF9, SF10, SF11, SF12
    }
}

static void sx1280SetPacketParamsLora(const uint8_t preambleLength, const sx1280LoraPacketLengthsModes_e headerType, const uint8_t payloadLength,
    const sx1280LoraCrcModes_e crc, const bool invertIQ)
{
    uint8_t buf[7];
    buf[0] = preambleLength;
    buf[1] = headerType;
    buf[2] = payloadLength;
    buf[3] = crc;
    buf[4] = invertIQ ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL;
    buf[5] = 0x00;
    buf[6] = 0x00;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_PACKETPARAMS, buf, 7);
}

static void sx1280ConfigModParamsFlrc(const SX1280_RadioFlrcBandwidths_t bw, const SX1280_RadioFlrcCodingRates_t cr, const SX1280_RadioFlrcGaussianFilter_t bt)
{
    uint8_t rfparams[3];
    rfparams[0] = bw;
    rfparams[1] = cr;
    rfparams[2] = bt;

    sx1280WriteCommandBurst(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, 3);
}

static void sx1280SetPacketParamsFlrc(uint8_t PreambleLength, uint8_t HeaderType,
    uint8_t PayloadLength, uint32_t syncWord, uint16_t crcSeed, uint8_t cr)
{
    if (PreambleLength < 8)
        PreambleLength = 8;

    uint8_t buf[7];
    buf[0] = ((PreambleLength / 4) - 1) << 4;   // AGCPreambleLength
    buf[1] = SX1280_FLRC_SYNC_WORD_LEN_P32S;    // SyncWordLength
    buf[2] = SX1280_FLRC_RX_MATCH_SYNC_WORD_1;  // SyncWordMatch
    buf[3] = HeaderType;                        // PacketType
    buf[4] = PayloadLength;                     // PayloadLength
    buf[5] = SX1280_FLRC_CRC_3_BYTE;            // CrcLength
    buf[6] = SX1280_FLRC_WHITENING_DISABLE;     // Must be whitening disabled
    sx1280WriteCommandBurst(SX1280_RADIO_SET_PACKETPARAMS, buf, 7);

    // CRC seed (use dedicated cipher)
    buf[0] = (uint8_t)(crcSeed >> 8);
    buf[1] = (uint8_t)crcSeed;
    sx1280WriteRegisterBurst(SX1280_REG_FLRC_CRC_SEED, buf, 2);

    // Set SyncWord1
    buf[0] = (uint8_t)(syncWord >> 24);
    buf[1] = (uint8_t)(syncWord >> 16);
    buf[2] = (uint8_t)(syncWord >> 8);
    buf[3] = (uint8_t)syncWord;

    // DS_SX1280-1_V3.2.pdf - 16.4 FLRC Modem: Increased PER in FLRC Packets with Synch Word
    if (((cr == SX1280_FLRC_CR_1_2) || (cr == SX1280_FLRC_CR_3_4)) &&
        ((buf[0] == 0x8C && buf[1] == 0x38) || (buf[0] == 0x63 && buf[1] == 0x0E))) {
        uint8_t temp = buf[0];
        buf[0] = buf[1];
        buf[1] = temp;
        // For SX1280_FLRC_CR_3_4 the datasheet also says
        // "In addition to this the two LSB values XX XX must not be in the range 0x0000 to 0x3EFF"
        if (cr == SX1280_FLRC_CR_3_4 && buf[3] <= 0x3e)
            buf[3] |= 0x80; // 0x80 or 0x40 would work
    }

    sx1280WriteRegisterBurst(SX1280_REG_FLRC_SYNC_WORD, buf, 4);

    // Set permissible sync errors = 0
    sx1280WriteRegister(SX1280_REG_FLRC_SYNC_ADDR_CTRL, sx1280ReadRegister(SX1280_REG_FLRC_SYNC_ADDR_CTRL) & 0xf0);
}

void sx1280Config(const uint8_t bw, const uint8_t sfbt, const uint8_t cr,
    const uint32_t freq, const uint8_t preambleLength, const bool iqInverted,
    const uint32_t flrcSyncWord, const uint16_t flrcCrcSeed, const bool isFlrc)
{
    sx1280SetMode(SX1280_MODE_STDBY_RC);

    sx1280PacketMode = (isFlrc) ? SX1280_PACKET_TYPE_FLRC : SX1280_PACKET_TYPE_LORA;
    sx1280WriteCommand(SX1280_RADIO_SET_PACKETTYPE, sx1280PacketMode);

    if (isFlrc) {
        sx1280ConfigModParamsFlrc(bw, cr, sfbt);
        sx1280SetPacketParamsFlrc(preambleLength, SX1280_FLRC_PACKET_FIXED_LENGTH, 8, flrcSyncWord, flrcCrcSeed, cr);
    } else {
        sx1280ConfigModParamsLora(bw, sfbt, cr);
        sx1280SetPacketParamsLora(preambleLength, SX1280_LORA_PACKET_FIXED_LENGTH, 8, SX1280_LORA_CRC_OFF, iqInverted);
    }

    sx1280SetDioIrqParams(
        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_SYNCWORD_ERROR | SX1280_IRQ_CRC_ERROR, // irqMask
        SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE, // dio1Mask
        SX1280_IRQ_RADIO_NONE,
        SX1280_IRQ_RADIO_NONE
    );

    sx1280SetOutputPower(13); //default is max power (12.5dBm for SX1280 RX)
    sx1280SetFrequencyReg(freq);
}

void sx1280SetOutputPower(const int8_t power)
{
    uint8_t buf[2];
    buf[0] = power + 18;
    buf[1] = (uint8_t) SX1280_RADIO_RAMP_04_US;
    sx1280WriteCommandBurst(SX1280_RADIO_SET_TXPARAMS, buf, 2);
}

void sx1280SetMode(const sx1280OperatingModes_e opMode)
{
    uint8_t buf[3];

    switch (opMode) {
    case SX1280_MODE_SLEEP:
        sx1280WriteCommand(SX1280_RADIO_SET_SLEEP, 0x01);
        break;
    case SX1280_MODE_CALIBRATION:
        break;
    case SX1280_MODE_STDBY_RC:
        sx1280WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_RC);
        break;
    case SX1280_MODE_STDBY_XOSC:
        sx1280WriteCommand(SX1280_RADIO_SET_STANDBY, SX1280_STDBY_XOSC);
        break;
    case SX1280_MODE_FS:
        sx1280WriteCommand(SX1280_RADIO_SET_FS, 0x00);
        break;
    case SX1280_MODE_RX:
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        sx1280WriteCommandBurst(SX1280_RADIO_SET_RX, buf, 3);
        break;
    case SX1280_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        sx1280WriteCommandBurst(SX1280_RADIO_SET_TX, buf, 3);
        break;
    case SX1280_MODE_CAD: // not implemented yet
    default:
        break;
    }
}

void sx1280SetFrequencyReg(const uint32_t freqReg)
{
    uint8_t buf[3] = {0};

    buf[0] = (uint8_t)((freqReg >> 16) & 0xFF);
    buf[1] = (uint8_t)((freqReg >> 8) & 0xFF);
    buf[2] = (uint8_t)(freqReg & 0xFF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_RFFREQUENCY, buf, 3);
}

void sx1280AdjustFrequency(int32_t *offset, const uint32_t freq)
{
    UNUSED(offset);
    UNUSED(freq);
}

void sx1280SetFifoAddr(const uint8_t txBaseAddr, const uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    sx1280WriteCommandBurst(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx1280SetDioIrqParams(const uint16_t irqMask, const uint16_t dio1Mask, const uint16_t dio2Mask, const uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    sx1280WriteCommandBurst(SX1280_RADIO_SET_DIOIRQPARAMS, buf, 8);
}

void sx1280TransmitData(const uint8_t *data, const uint8_t length)
{
    sx1280WriteBuffer(0x00, data, length);
    sx1280SetMode(SX1280_MODE_TX);
}

static uint8_t sx1280GetRxBufferAddr(void)
{
    uint8_t status[2] = {0};
    sx1280ReadCommandBurst(SX1280_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void sx1280ReceiveData(uint8_t *data, const uint8_t length)
{
    uint8_t FIFOaddr = sx1280GetRxBufferAddr();
    sx1280ReadBuffer(FIFOaddr, data, length);
}

void sx1280StartReceiving(void)
{
    const timeUs_t startTime = micros();

    // Wait for any chain to complete rather than silently skipping the mode change
    while (!sx1280MarkBusy()) {
        if (cmpTimeUs(micros(), startTime) > SX1280_TASK_WAIT_TIMEOUT_US) {
            return;
        }
    }

    // sx1280TransferBlocking() would wait for sx1280Processing, which we now own
    uint8_t buf[4] = { SX1280_RADIO_SET_RX, 0x00, 0xFF, 0xFF };
    sx1280PollBusy();
    ATOMIC_BLOCK(NVIC_PRIO_SX1280_ALL) {
        rxSpiTransferCommandMulti(buf, sizeof(buf));
    }

    sx1280EnableExti();
}

void sx1280GetLastPacketStats(int8_t *rssi, int8_t *snr)
{
    if (sx1280PacketMode == SX1280_PACKET_TYPE_FLRC) {
        // No SNR in FLRC mode
        *rssi = -(int8_t)(packetStats[1] / 2);
        *snr = 0;
    } else {
        *rssi = -(int8_t)(packetStats[0] / 2);
        *snr = (int8_t)packetStats[1];
        int8_t negOffset = (*snr < 0) ? (*snr / 4) : 0;
        *rssi += negOffset;
    }
}

void sx1280ClearIrqStatus(const uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    sx1280WriteCommandBurst(SX1280_RADIO_CLR_IRQSTATUS, buf, 2);
}

// Forward Definitions for the interrupt chain //
static void sx1280IrqGetStatus(extiCallbackRec_t *cb);
static busStatus_e sx1280IrqStatusRead(void);
static void sx1280IrqClearStatus(extiCallbackRec_t *cb);
static busStatus_e sx1280IrqCmdComplete(void);
static void sx1280ProcessIrq(extiCallbackRec_t *cb);
static busStatus_e sx1280GotFIFOAddr(void);
static void sx1280DoReadBuffer(extiCallbackRec_t *cb);
static busStatus_e sx1280ReadBufferComplete(void);
static void sx1280GetPacketStats(extiCallbackRec_t *cb);
static busStatus_e sx1280GetStatsCmdComplete(void);
static busStatus_e sx1280IsFhssReq(void);
static void sx1280SetFrequency(extiCallbackRec_t *cb);
static busStatus_e sx1280SetFreqComplete(void);
static void sx1280StartReceivingChain(extiCallbackRec_t *cb);
static busStatus_e sx1280EnableIRQs(void);
static void sx1280SendTelemetryBuffer(extiCallbackRec_t *cb);
static busStatus_e sx1280TelemetryComplete(void);
static void sx1280StartTransmittingChain(extiCallbackRec_t *cb);

// DIO1 rising edge
void sx1280ISR(void)
{
    // Only attempt to access the SX1280 if it is currently idle to avoid any race condition
    if (sx1280MarkBusy()) {
        sx1280SetBusyFn(sx1280IrqGetStatus);
    }
}

// Next, the reason for the IRQ must be read

static void sx1280IrqGetStatus(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    static const uint8_t irqStatusCmd[] = {SX1280_RADIO_GET_IRQSTATUS, 0, 0, 0};
    static uint8_t irqStatus[sizeof(irqStatusCmd)];

    static const sx1280Segment_t segments[] = {
            {irqStatusCmd, irqStatus, sizeof(irqStatusCmd), true, sx1280IrqStatusRead},
            {NULL, NULL, 0, false, NULL},
    };

    sx1280RunSegments(segments);
}

// Read the IRQ status, and save it to irqStatus variable
static busStatus_e sx1280IrqStatusRead(void)
{
    uint16_t irqStatus = (currentSegment->rxData[2] << 8) | currentSegment->rxData[3];

    if (irqStatus & SX1280_IRQ_TX_DONE) {
        irqReason = ELRS_DIO_TX_DONE;
    } else if (irqStatus & SX1280_IRQ_RX_DONE) {
        irqReason = ELRS_DIO_RX_DONE;

        if (sx1280PacketMode == SX1280_PACKET_TYPE_FLRC) {
            // Reject the packet early if CRC/Syncword error or syncword valid not set
            if ((irqStatus & (SX1280_IRQ_CRC_ERROR | SX1280_IRQ_SYNCWORD_ERROR)) ||
                !(irqStatus & SX1280_IRQ_SYNCWORD_VALID)) {
                irqReason |= ELRS_DIO_HWERROR;
            }
        }
    } else {
        irqReason = ELRS_DIO_UNKNOWN;
    }

    sx1280SetBusyFn(sx1280IrqClearStatus);
    return BUS_READY;
}

// Clear the IRQ bit in the Radio registers

static void sx1280IrqClearStatus(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    static const uint8_t irqCmd[] = {
        SX1280_RADIO_CLR_IRQSTATUS,
        (uint8_t)(((uint16_t)SX1280_IRQ_RADIO_ALL >> 8) & 0x00FF),
        (uint8_t)((uint16_t)SX1280_IRQ_RADIO_ALL & 0x00FF)
    };

    static const sx1280Segment_t segments[] = {
            {irqCmd, NULL, sizeof(irqCmd), true, sx1280IrqCmdComplete},
            {NULL, NULL, 0, false, NULL},
    };

    sx1280RunSegments(segments);
}

// Callback follow clear of IRQ status
static busStatus_e sx1280IrqCmdComplete(void)
{
    // If HWERROR reported on RX, just do nothing and wait for the timer to expire
    if (!(irqReason & ELRS_DIO_HWERROR)) {
        sx1280SetBusyFn(sx1280ProcessIrq);
    }

    return BUS_READY;
}

// Process IRQ status
static void sx1280ProcessIrq(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    if (irqReason & ELRS_DIO_RX_DONE) {
        // Fire off the chain to read and decode the packet from the radio
        // Get the buffer status to determine the FIFO address
        static const uint8_t cmdBufStatusCmd[] = {SX1280_RADIO_GET_RXBUFFERSTATUS, 0, 0, 0};
        static uint8_t bufStatus[sizeof(cmdBufStatusCmd)];

        static const sx1280Segment_t segments[] = {
            {cmdBufStatusCmd, bufStatus, sizeof(cmdBufStatusCmd), true, sx1280GotFIFOAddr},
            {NULL, NULL, 0, false, NULL},
        };

        sx1280RunSegments(segments);

    } else {
        // return to RX mode immediately, the next packet will be an RX and we won't need to FHSS
        static const uint8_t irqSetRxCmd[] = {SX1280_RADIO_SET_RX, 0, 0xff, 0xff};

        static const sx1280Segment_t segments[] = {
            {irqSetRxCmd, NULL, sizeof(irqSetRxCmd), true, sx1280EnableIRQs},
            {NULL, NULL, 0, false, NULL},
        };

        sx1280RunSegments(segments);
    }
}

// First we read from the FIFO address register to determine the FIFO address
static busStatus_e sx1280GotFIFOAddr(void)
{
    FIFOaddr = currentSegment->rxData[3];

    // Wait until no longer busy and read the buffer
    sx1280SetBusyFn(sx1280DoReadBuffer);

    return BUS_READY;
}

// Using the addr val stored to the global varable FIFOaddr, read the buffer
static void sx1280DoReadBuffer(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    static uint8_t cmdReadBuf[] = {SX1280_RADIO_READ_BUFFER, 0, 0};

    cmdReadBuf[1] = FIFOaddr;

    static sx1280Segment_t segments[] = {
            {cmdReadBuf, NULL, sizeof(cmdReadBuf), false, NULL},
            {NULL, NULL, ELRS_RX_TX_BUFF_SIZE, true, sx1280ReadBufferComplete},
            {NULL, NULL, 0, false, NULL},
    };

    segments[1].rxData = (uint8_t *)expressLrsGetRxBuffer();

    sx1280RunSegments(segments);
}

// Get the Packet Status and RSSI
static busStatus_e sx1280ReadBufferComplete(void)
{
    sx1280SetBusyFn(sx1280GetPacketStats);

    return BUS_READY;
}

// Save the Packet Stats to the global variables
static void sx1280GetPacketStats(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    static const uint8_t getStatsCmd[] = {SX1280_RADIO_GET_PACKETSTATUS, 0, 0, 0};
    static uint8_t stats[sizeof(getStatsCmd)];

    static const sx1280Segment_t segments[] = {
            {getStatsCmd, stats, sizeof(getStatsCmd), true, sx1280GetStatsCmdComplete},
            {NULL, NULL, 0, false, NULL},
    };

    sx1280RunSegments(segments);
}

// Process and decode the RF packet
static busStatus_e sx1280GetStatsCmdComplete(void)
{
    volatile uint8_t *payload = expressLrsGetPayloadBuffer();

    packetStats[0] = currentSegment->rxData[2];
    packetStats[1] = currentSegment->rxData[3];

    expressLrsSetRfPacketStatus(processRFPacket(payload, rxSpiGetLastExtiTimeUs()));

    return sx1280IsFhssReq();
}

void sx1280HandleFromTock(void)
{
    bool startFhss = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (expressLrsIsFhssReq()) {
            if (sx1280MarkBusy()) {
                pendingDoFHSS = false;
                startFhss = true;
            } else {
                pendingDoFHSS = true;
            }
        }
    }

    // Run the (synchronous) SPI chain outside the fully masked section
    if (startFhss) {
        sx1280SetBusyFn(sx1280SetFrequency);
    }
}

// Next we need to check if we need to FHSS and then do so if needed
static busStatus_e sx1280IsFhssReq(void)
{
    if (expressLrsIsFhssReq()) {
        sx1280SetBusyFn(sx1280SetFrequency);
    } else {
        sx1280SetFreqComplete();
    }

    return BUS_READY;
}

// Set the frequency
static void sx1280SetFrequency(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    uint32_t currentFreq = expressLrsGetCurrentFreq();

    sx1280ClearBusyFn();

    static uint8_t setFreqCmd[] = {SX1280_RADIO_SET_RFFREQUENCY, 0, 0, 0};
    setFreqCmd[1] = (uint8_t)((currentFreq >> 16) & 0xFF);
    setFreqCmd[2] = (uint8_t)((currentFreq >> 8) & 0xFF);
    setFreqCmd[3] = (uint8_t)(currentFreq & 0xFF);

    static const sx1280Segment_t segments[] = {
            {setFreqCmd, NULL, sizeof(setFreqCmd), true, sx1280SetFreqComplete},
            {NULL, NULL, 0, false, NULL},
    };

    sx1280RunSegments(segments);
}

// Determine if we need to go back to RX or if we need to send TLM data
static busStatus_e sx1280SetFreqComplete(void)
{
    pendingDoFHSS = false;

    if (expressLrsTelemRespReq()) {
        expressLrsDoTelem();
        // if it's time to do TLM and we have enough to do so
        sx1280SetBusyFn(sx1280SendTelemetryBuffer);
    } else {
        // we don't need to send TLM and we've already FHSS so just hop back into RX mode
        sx1280SetBusyFn(sx1280StartReceivingChain);
    }

    return BUS_READY;
}

// Go back into RX mode
static void sx1280StartReceivingChain(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    // Issue command to start receiving
    // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
    static const uint8_t irqSetRxCmd[] = {SX1280_RADIO_SET_RX, 0, 0xff, 0xff};

    static const sx1280Segment_t segments[] = {
            {irqSetRxCmd, NULL, sizeof(irqSetRxCmd), true, sx1280EnableIRQs},
            {NULL, NULL, 0, false, NULL},
    };

    sx1280RunSegments(segments);
}

static busStatus_e sx1280EnableIRQs(void)
{
    if (pendingDoFHSS) {
        pendingDoFHSS = false;
        sx1280SetBusyFn(sx1280SetFrequency);
    } else {
        // Switch back to waiting for EXTI interrupt
        sx1280EnableExti();
    }

    return BUS_READY;
}

// Send telemetry response
static void sx1280SendTelemetryBuffer(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    static const uint8_t writeBufferCmd[] = {SX1280_RADIO_WRITE_BUFFER, 0};

    static sx1280Segment_t segments[] = {
            {writeBufferCmd, NULL, sizeof(writeBufferCmd), false, NULL},
            {NULL, NULL, ELRS_RX_TX_BUFF_SIZE, true, sx1280TelemetryComplete},
            {NULL, NULL, 0, false, NULL},
    };

    segments[1].txData = (uint8_t *)expressLrsGetTelemetryBuffer();

    sx1280RunSegments(segments);
}

static busStatus_e sx1280TelemetryComplete(void)
{
    sx1280SetBusyFn(sx1280StartTransmittingChain);

    return BUS_READY;
}

static void sx1280StartTransmittingChain(extiCallbackRec_t *cb)
{
    UNUSED(cb);

    sx1280ClearBusyFn();

    //uses timeout Time-out duration = periodBase * periodBaseCount
    // periodBase = 1ms, page 71 datasheet
    // no timeout set for now
    // TODO dynamic timeout based on expected onairtime
    static const uint8_t irqSetTxCmd[] = {SX1280_RADIO_SET_TX, 0, 0xff, 0xff};

    static const sx1280Segment_t segments[] = {
            {irqSetTxCmd, NULL, sizeof(irqSetTxCmd), true, sx1280EnableIRQs},
            {NULL, NULL, 0, false, NULL},
    };

    sx1280RunSegments(segments);
}

#endif /* USE_RX_SX1280 */
