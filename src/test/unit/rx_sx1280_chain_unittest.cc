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
 * Host simulation of the SX1280 interrupt chain (drivers/rx/rx_sx1280.c and
 * drivers/rx/rx_spi.c, compiled unmodified).
 *
 * A fake SX1280 answers the SPI commands and drives DIO1 and BUSY; BUSY stays
 * high for a random time after every command, including not at all. EXTI is
 * modelled with latched pending bits, enable masks, software trigger and the
 * dispatcher's clear-before-dispatch order. Interrupts are taken between
 * simulation steps (no preemption), so this checks the chain's sequencing and
 * edge handling, not priority races.
 *
 * Invariants checked: every received packet is processed exactly once, no SPI
 * transfer ever happens while BUSY is high, telemetry and FHSS requests are
 * serviced, and the chain always returns to waiting on DIO1.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>
#include <random>
#include <vector>

extern "C" {
    #include "platform.h"

    #include "common/utils.h"

    #include "drivers/bus.h"
    #include "drivers/exti.h"
    #include "drivers/io.h"
    #include "drivers/rx/rx_spi.h"
    #include "drivers/rx/rx_sx1280.h"

    #include "rx/expresslrs.h"
    #include "rx/expresslrs_common.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// ---------------------------------------------------------------------------
// Simulation state

enum { LINE_DIO1 = 3, LINE_BUSY = 15, LINE_BIND = 2, LINE_LED = 9, LINE_RESET = 1 };

static uint8_t pinStorage[16];
static IO_t lineIo(int line) { return (IO_t)&pinStorage[line]; }
static int ioLine(IO_t io) { return (int)((uint8_t *)io - pinStorage); }

typedef struct {
    bool enabled;
    bool pending;
    bool level;
    extiCallbackRec_t *cb;
} simExti_t;

static simExti_t exti[16];
static uint32_t simUs;
static bool clearBeforeDispatch = true;

// Fake radio
static uint16_t radioIrq;
static uint32_t busyUntilUs;
static uint32_t txDoneAtUs;
static std::mt19937 rng;
static uint32_t maxBusyUs = 600;
static uint8_t rxPacket[ELRS_RX_TX_BUFF_SIZE];
static uint8_t telemetryWritten[ELRS_RX_TX_BUFF_SIZE];

static int transfersWhileBusy;
static int setRxCount;
static int setTxCount;
static int setFreqCount;
static int writeBufferCount;

// ELRS hooks
static uint8_t rxBuffer[ELRS_RX_TX_BUFF_SIZE];
static uint8_t payloadBuffer[ELRS_RX_TX_BUFF_SIZE];
static uint8_t telemetryBuffer[ELRS_RX_TX_BUFF_SIZE];
static int packetsProcessed;
static uint8_t lastProcessedSeq;
static int outOfOrder;       // a packet older than one already processed
static int duplicates;       // same buffer read twice (radio overwrote it mid-chain)
static bool fhssRequested;
static int telemEvery;
static int processedSinceTelem;

static void setLevel(int line, bool level)
{
    simExti_t *e = &exti[line];
    bool rising = !e->level && level;
    bool falling = e->level && !level;
    e->level = level;
    // Edge detection latches the pending bit even while the line is masked
    if ((line == LINE_DIO1 && rising) || (line == LINE_BUSY && falling)) {
        e->pending = true;
    }
}

static void updateRadioLines(void)
{
    setLevel(LINE_BUSY, simUs < busyUntilUs);
    if (txDoneAtUs && simUs >= txDoneAtUs) {
        txDoneAtUs = 0;
        radioIrq |= SX1280_IRQ_TX_DONE;
    }
    setLevel(LINE_DIO1, (radioIrq & (SX1280_IRQ_RX_DONE | SX1280_IRQ_TX_DONE)) != 0);
}

// Take all pending, enabled interrupts
static void dispatchInterrupts(void)
{
    for (int guard = 0; guard < 1000; guard++) {
        uint32_t active = 0;
        for (int line = 0; line < 16; line++) {
            if (exti[line].enabled && exti[line].pending && exti[line].cb) {
                active |= 1U << line;
            }
        }
        if (!active) {
            return;
        }
        if (clearBeforeDispatch) {
            for (int line = 0; line < 16; line++) {
                if (active & (1U << line)) exti[line].pending = false;
            }
        }
        for (int line = 15; line >= 0; line--) {
            if (active & (1U << line)) {
                exti[line].cb->fn(exti[line].cb);
                if (!clearBeforeDispatch) exti[line].pending = false;
            }
        }
    }
    ADD_FAILURE() << "interrupt storm";
}

static void step(uint32_t us)
{
    for (uint32_t i = 0; i < us; i++) {
        simUs++;
        updateRadioLines();
        dispatchInterrupts();
    }
}

static void radioCommand(const uint8_t *tx, uint8_t *rx, int length, const uint8_t *tx2, uint8_t *rx2, int length2)
{
    if (simUs < busyUntilUs) {
        transfersWhileBusy++;
    }

    uint8_t cmd = tx ? tx[0] : 0;
    switch (cmd) {
    case SX1280_RADIO_GET_IRQSTATUS:
        if (rx && length >= 4) { rx[2] = radioIrq >> 8; rx[3] = radioIrq & 0xFF; }
        break;
    case SX1280_RADIO_CLR_IRQSTATUS:
        radioIrq &= ~(((uint16_t)tx[1] << 8) | tx[2]);
        break;
    case SX1280_RADIO_GET_RXBUFFERSTATUS:
        if (rx && length >= 4) { rx[2] = 8; rx[3] = 0x10; }
        break;
    case SX1280_RADIO_READ_BUFFER:
        EXPECT_EQ(0x10, tx[1]);
        if (rx2) memcpy(rx2, rxPacket, std::min(length2, (int)sizeof(rxPacket)));
        break;
    case SX1280_RADIO_GET_PACKETSTATUS:
        if (rx && length >= 4) { rx[2] = 100; rx[3] = 20; }
        break;
    case SX1280_RADIO_SET_RX:
        setRxCount++;
        break;
    case SX1280_RADIO_SET_TX:
        setTxCount++;
        txDoneAtUs = simUs + 400;
        break;
    case SX1280_RADIO_SET_RFFREQUENCY:
        setFreqCount++;
        break;
    case SX1280_RADIO_WRITE_BUFFER:
        writeBufferCount++;
        if (tx2) memcpy(telemetryWritten, tx2, std::min(length2, (int)sizeof(telemetryWritten)));
        break;
    case SX1280_RADIO_READ_REGISTER:
        if (rx && length >= 5) rx[4] = 0xA5;   // firmware version, non-zero
        break;
    default:
        break;
    }

    // BUSY asserts after each command for a random time, sometimes not at all
    uint32_t busyFor = (rng() % 4 == 0) ? 0 : (rng() % (maxBusyUs + 1));
    busyUntilUs = simUs + busyFor;
    updateRadioLines();
}

static void receivePacket(uint8_t seq)
{
    memset(rxPacket, seq, sizeof(rxPacket));
    radioIrq |= SX1280_IRQ_RX_DONE;
    updateRadioLines();
}

static void resetSim(uint32_t seed)
{
    memset(exti, 0, sizeof(exti));
    simUs = 1000;
    radioIrq = 0;
    busyUntilUs = 0;
    txDoneAtUs = 0;
    rng.seed(seed);
    transfersWhileBusy = setRxCount = setTxCount = setFreqCount = writeBufferCount = 0;
    packetsProcessed = 0;
    lastProcessedSeq = 0;
    outOfOrder = 0;
    duplicates = 0;
    fhssRequested = false;
    telemEvery = 0;
    processedSinceTelem = 0;
    clearBeforeDispatch = true;
    maxBusyUs = 600;
}

static void startRadio(void)
{
    ASSERT_TRUE(rxSpiDeviceInit());
    ASSERT_TRUE(sx1280Init(lineIo(LINE_RESET), lineIo(LINE_BUSY)));
    rxSpiExtiInit(sx1280ISR);
    busyUntilUs = 0;
    updateRadioLines();
    sx1280StartReceiving();
    step(10);
}

// ---------------------------------------------------------------------------

TEST(RxSx1280ChainUnitTest, EveryPacketProcessedOnceWithRandomBusyTiming)
{
    for (uint32_t seed = 1; seed <= 20; seed++) {
        resetSim(seed);
        startRadio();

        int sent = 0;
        for (int n = 1; n <= 200; n++) {
            receivePacket((uint8_t)n);
            sent++;
            // Packet interval 4ms (250Hz), well above the chain duration
            step(4000);
        }

        EXPECT_EQ(sent, packetsProcessed) << "seed " << seed;
        EXPECT_EQ(0, duplicates) << "seed " << seed;
        EXPECT_EQ(0, outOfOrder) << "seed " << seed;
        EXPECT_EQ(0, transfersWhileBusy) << "seed " << seed;
        EXPECT_TRUE(exti[LINE_DIO1].enabled) << "seed " << seed;
    }
}

TEST(RxSx1280ChainUnitTest, PacketArrivingDuringChainIsNotLost)
{
    for (uint32_t seed = 1; seed <= 20; seed++) {
        resetSim(seed);
        maxBusyUs = 900;
        startRadio();

        int sent = 0;
        for (int n = 1; n <= 200; n++) {
            receivePacket((uint8_t)n);
            sent++;
            // Next packet lands while the previous chain is still running
            step(1500 + rng() % 1500);
        }
        step(20000);

        // The radio has one buffer: a packet overwritten before it is read is
        // lost, and one arriving after its IRQ was cleared can be read twice
        // (as in Betaflight). What matters is that nothing stalls or reorders.
        EXPECT_GE(packetsProcessed - duplicates, sent * 3 / 4) << "seed " << seed;
        EXPECT_EQ(0, outOfOrder) << "seed " << seed;
        EXPECT_EQ(0, transfersWhileBusy) << "seed " << seed;
        EXPECT_EQ(0, radioIrq & SX1280_IRQ_RX_DONE) << "seed " << seed << ": RX_DONE left unserviced";
        EXPECT_TRUE(exti[LINE_DIO1].enabled) << "seed " << seed;
    }
}

TEST(RxSx1280ChainUnitTest, TelemetryTransmitAndTxDoneHandled)
{
    resetSim(42);
    startRadio();
    telemEvery = 4;

    for (int n = 1; n <= 100; n++) {
        receivePacket((uint8_t)n);
        step(4000);
    }

    EXPECT_EQ(100, packetsProcessed);
    EXPECT_EQ(25, setTxCount);
    EXPECT_EQ(25, writeBufferCount);
    EXPECT_EQ(0, memcmp(telemetryWritten, telemetryBuffer, sizeof(telemetryBuffer)));
    EXPECT_EQ(0, radioIrq);
    EXPECT_EQ(0, transfersWhileBusy);
    EXPECT_TRUE(exti[LINE_DIO1].enabled);
}

TEST(RxSx1280ChainUnitTest, FhssFromTockWhenIdleAndWhenBusy)
{
    resetSim(7);
    startRadio();

    // Idle: tock hops immediately
    fhssRequested = true;
    sx1280HandleFromTock();
    step(2000);
    EXPECT_EQ(1, setFreqCount);

    // Busy: hop is deferred to the end of the packet chain
    maxBusyUs = 800;
    int hopsExpected = 1;
    for (int n = 1; n <= 50; n++) {
        receivePacket((uint8_t)n);
        step(1 + rng() % 200);          // chain now in progress
        fhssRequested = true;
        sx1280HandleFromTock();
        hopsExpected++;
        step(4000);
    }

    EXPECT_EQ(50, packetsProcessed);
    EXPECT_EQ(hopsExpected, setFreqCount);
    EXPECT_EQ(0, transfersWhileBusy);
    EXPECT_TRUE(exti[LINE_DIO1].enabled);
}

TEST(RxSx1280ChainUnitTest, OldClearAfterDispatchLosesEdges)
{
    // Documents why drivers/exti.c now clears pending bits before dispatch:
    // with the previous order the chain stalls on the first lost edge.
    resetSim(3);
    clearBeforeDispatch = false;
    startRadio();

    for (int n = 1; n <= 200; n++) {
        receivePacket((uint8_t)n);
        step(4000);
    }

    EXPECT_LT(packetsProcessed, 200);
}

// ---------------------------------------------------------------------------
// STUBS

extern "C" {
    // Time passes while the driver polls (busy-wait loops); interrupts are
    // still only taken between simulation steps
    uint32_t micros(void) { simUs++; updateRadioLines(); return simUs; }
    uint32_t microsISR(void) { return simUs; }
    uint32_t millis(void) { return simUs / 1000; }
    void delay(uint32_t ms) { simUs += ms * 1000; updateRadioLines(); }

    void IOInit(IO_t, resourceOwner_e, resourceType_e, uint8_t) {}
    void IOConfigGPIO(IO_t, ioConfig_t) {}
    bool IORead(IO_t io) { return exti[ioLine(io)].level; }
    void IOHi(IO_t) {}
    void IOLo(IO_t) {}
    void IOToggle(IO_t) {}
    IO_t IOGetByTag(ioTag_t tag)
    {
        // Target pins used by drivers/rx/rx_spi.c
        if (tag == IO_TAG(RX_SPI_EXTI_PIN)) return lineIo(LINE_DIO1);
        if (tag == IO_TAG(RX_SPI_BIND_PIN)) return lineIo(LINE_BIND);
        if (tag == IO_TAG(RX_SPI_LED_PIN)) return lineIo(LINE_LED);
        return IO_NONE;
    }

    void EXTIHandlerInit(extiCallbackRec_t *cb, extiHandlerCallback *fn) { cb->fn = fn; }
    void EXTIConfig(IO_t io, extiCallbackRec_t *cb, int, EXTITrigger_TypeDef) { exti[ioLine(io)].cb = cb; }
    void EXTIEnable(IO_t io, bool enable) { exti[ioLine(io)].enabled = enable; }
    void EXTIClearPending(IO_t io) { exti[ioLine(io)].pending = false; }
    void EXTITriggerSoftware(IO_t io) { exti[ioLine(io)].pending = true; }

    static busDevice_t fakeDevice;
    busDevice_t *busDeviceInit(busType_e, devHardwareType_e, uint8_t, resourceOwner_e) { return &fakeDevice; }
    void busSetSpeed(const busDevice_t *, busSpeed_e) {}
    bool busTransfer(const busDevice_t *, uint8_t *rxBuf, const uint8_t *txBuf, int length)
    {
        uint8_t tmp[64];
        memcpy(tmp, txBuf, length);
        radioCommand(tmp, rxBuf, length, NULL, NULL, 0);
        return true;
    }
    bool busTransferMultiple(const busDevice_t *, busTransferDescriptor_t *dsc, int count)
    {
        if (count == 1) {
            radioCommand(dsc[0].txBuf, dsc[0].rxBuf, dsc[0].length, NULL, NULL, 0);
        } else {
            radioCommand(dsc[0].txBuf, dsc[0].rxBuf, dsc[0].length, dsc[1].txBuf, dsc[1].rxBuf, dsc[1].length);
        }
        return true;
    }

    rx_spi_received_e processRFPacket(volatile uint8_t *, uint32_t)
    {
        packetsProcessed++;
        uint8_t seq = rxBuffer[0];
        if (seq == lastProcessedSeq) {
            duplicates++;
        } else if (seq < lastProcessedSeq) {
            outOfOrder++;
        }
        lastProcessedSeq = seq;
        processedSinceTelem++;
        return RX_SPI_RECEIVED_DATA;
    }
    void expressLrsSetRfPacketStatus(rx_spi_received_e) {}
    volatile uint8_t *expressLrsGetRxBuffer(void) { return rxBuffer; }
    volatile uint8_t *expressLrsGetPayloadBuffer(void) { return payloadBuffer; }
    volatile uint8_t *expressLrsGetTelemetryBuffer(void) { return telemetryBuffer; }
    uint32_t expressLrsGetCurrentFreq(void) { return 0x123456; }
    bool expressLrsIsFhssReq(void)
    {
        bool req = fhssRequested;
        fhssRequested = false;
        return req;
    }
    bool expressLrsTelemRespReq(void)
    {
        if (telemEvery && processedSinceTelem >= telemEvery) {
            return true;
        }
        return false;
    }
    void expressLrsDoTelem(void)
    {
        processedSinceTelem = 0;
        for (int i = 0; i < ELRS_RX_TX_BUFF_SIZE; i++) telemetryBuffer[i] = 0xC0 + i;
    }
}
