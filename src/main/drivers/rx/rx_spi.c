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
 * Adapted from Betaflight drivers/rx/rx_spi.c and rx/rx_spi_common.c.
 *
 * Unlike Betaflight, the radio bus device, EXTI, LED and bind pins come from
 * target defines rather than a parameter group.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_SPI

#include "build/atomic.h"

#include "drivers/bus.h"
#include "drivers/exti.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/nvic.h"
#include "drivers/time.h"

#include "drivers/rx/rx_spi.h"

#define INTERVAL_RX_LOSS_MS 1000
#define INTERVAL_RX_BIND_MS 250

static busDevice_t *rxSpiDevice;

static IO_t extiPin = IO_NONE;
static extiCallbackRec_t rxSpiExtiCallbackRec;
static rxSpiExtiHandlerFn *extiHandler;
static volatile timeUs_t lastExtiTimeUs = 0;

static IO_t ledPin = IO_NONE;
static bool ledInversion = false;

static IO_t bindPin = IO_NONE;
static bool bindRequested;
static bool lastBindPinStatus;

bool rxSpiDeviceInit(void)
{
    rxSpiDevice = busDeviceInit(BUSTYPE_SPI, DEVHW_SX1280, 0, OWNER_RX);
    if (rxSpiDevice == NULL) {
        return false;
    }

    // SX1280 accepts up to 18MHz. STANDARD is ~9MHz on the AT32F435 APB1 SPI buses.
    busSetSpeed(rxSpiDevice, BUS_SPEED_STANDARD);

#ifdef RX_SPI_EXTI_PIN
    extiPin = IOGetByTag(IO_TAG(RX_SPI_EXTI_PIN));
    if (extiPin) {
        IOInit(extiPin, OWNER_RX, RESOURCE_EXTI, 0);
    }
#endif

#ifdef RX_SPI_LED_PIN
    ledPin = IOGetByTag(IO_TAG(RX_SPI_LED_PIN));
    IOInit(ledPin, OWNER_LED, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(ledPin, IOCFG_OUT_PP);
#ifdef RX_SPI_LED_INVERTED
    ledInversion = true;
#endif
    rxSpiLedOff();
#endif

#ifdef RX_SPI_BIND_PIN
    bindPin = IOGetByTag(IO_TAG(RX_SPI_BIND_PIN));
    IOInit(bindPin, OWNER_RX, RESOURCE_INPUT, 0);
    IOConfigGPIO(bindPin, IOCFG_IPU);
    lastBindPinStatus = IORead(bindPin);
#endif

    return true;
}

busDevice_t *rxSpiGetDevice(void)
{
    return rxSpiDevice;
}

void rxSpiTransferCommandMulti(uint8_t *data, uint8_t length)
{
    busTransfer(rxSpiDevice, data, data, length);
}

static void rxSpiExtiHandler(extiCallbackRec_t *callback)
{
    UNUSED(callback);

    lastExtiTimeUs = microsISR();

    if (extiHandler) {
        extiHandler();
    }
}

bool rxSpiExtiConfigured(void)
{
    return extiPin != IO_NONE;
}

void rxSpiExtiInit(rxSpiExtiHandlerFn *handler)
{
    if (!extiPin) {
        return;
    }

    extiHandler = handler;

    IOConfigGPIO(extiPin, IOCFG_IPD);
    EXTIHandlerInit(&rxSpiExtiCallbackRec, rxSpiExtiHandler);
#if defined(AT32F43x)
    EXTIConfig(extiPin, &rxSpiExtiCallbackRec, NVIC_PRIO_RX_INT_EXTI, EXINT_TRIGGER_RISING_EDGE);
#elif defined(STM32F7) || defined(STM32H7)
    EXTIConfig(extiPin, &rxSpiExtiCallbackRec, NVIC_PRIO_RX_INT_EXTI, IO_CONFIG(GPIO_MODE_IT_RISING, GPIO_SPEED_FREQ_LOW, GPIO_PULLDOWN));
#else
    EXTIConfig(extiPin, &rxSpiExtiCallbackRec, NVIC_PRIO_RX_INT_EXTI, EXTI_Trigger_Rising);
#endif
    EXTIEnable(extiPin, false);
}

// (Re-)arm the DIO interrupt. Called from interrupt and task context once the
// radio is idle. The DIO line is level-high while an IRQ is pending in the radio,
// so a rising edge that happened while the previous sequence ran would otherwise
// be lost: check the level and service it immediately.
void rxSpiEnableExti(void)
{
    if (!extiPin) {
        return;
    }

    bool pending;

    ATOMIC_BLOCK(NVIC_PRIO_RX_INT_EXTI) {
        EXTIClearPending(extiPin);
        EXTIEnable(extiPin, true);
        pending = IORead(extiPin);
        if (pending) {
            // Serviced directly below; drop any edge latched meanwhile
            EXTIClearPending(extiPin);
        }
    }

    if (pending) {
        rxSpiExtiHandler(NULL);
    }
}

bool rxSpiGetExtiState(void)
{
    return IORead(extiPin);
}

timeUs_t rxSpiGetLastExtiTimeUs(void)
{
    return lastExtiTimeUs;
}

void rxSpiLedOn(void)
{
    if (ledPin) {
        ledInversion ? IOLo(ledPin) : IOHi(ledPin);
    }
}

void rxSpiLedOff(void)
{
    if (ledPin) {
        ledInversion ? IOHi(ledPin) : IOLo(ledPin);
    }
}

void rxSpiLedToggle(void)
{
    if (ledPin) {
        IOToggle(ledPin);
    }
}

void rxSpiLedBlink(timeMs_t blinkMs)
{
    static timeMs_t ledBlinkMs = 0;

    if ((ledBlinkMs + blinkMs) > millis()) {
        return;
    }
    ledBlinkMs = millis();

    rxSpiLedToggle();
}

void rxSpiLedBlinkRxLoss(bool packetReceived)
{
    static timeMs_t rxLossMs = 0;

    if (ledPin) {
        if (packetReceived) {
            rxSpiLedOn();
        } else {
            if ((rxLossMs + INTERVAL_RX_LOSS_MS) > millis()) {
                return;
            }
            rxSpiLedToggle();
        }
        rxLossMs = millis();
    }
}

void rxSpiLedBlinkBind(void)
{
    rxSpiLedBlink(INTERVAL_RX_BIND_MS);
}

void rxSpiBind(void)
{
    bindRequested = true;
}

bool rxSpiCheckBindRequested(bool reset)
{
    if (bindPin) {
        bool bindPinStatus = IORead(bindPin);
        if (lastBindPinStatus && !bindPinStatus) {
            bindRequested = true;
        }
        lastBindPinStatus = bindPinStatus;
    }

    if (!bindRequested) {
        return false;
    }

    if (reset) {
        bindRequested = false;
    }

    return true;
}

#endif // USE_RX_SPI
