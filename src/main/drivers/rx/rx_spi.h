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
 * Low level access to an SPI receiver radio: bus device, DIO/EXTI line,
 * status LED and bind button. Adapted from Betaflight drivers/rx/rx_spi.c.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

#include "drivers/bus.h"
#include "drivers/io_types.h"

typedef void rxSpiExtiHandlerFn(void);

bool rxSpiDeviceInit(void);
busDevice_t *rxSpiGetDevice(void);

// Transfer a full-duplex buffer in place with CS asserted for the whole transfer
void rxSpiTransferCommandMulti(uint8_t *data, uint8_t length);

// DIO line. The handler is called from interrupt context on a rising edge.
bool rxSpiExtiConfigured(void);
void rxSpiExtiInit(rxSpiExtiHandlerFn *handler);
void rxSpiEnableExti(void);
bool rxSpiGetExtiState(void);
timeUs_t rxSpiGetLastExtiTimeUs(void);

void rxSpiLedOn(void);
void rxSpiLedOff(void);
void rxSpiLedToggle(void);
void rxSpiLedBlink(timeMs_t blinkMs);
void rxSpiLedBlinkRxLoss(bool packetReceived);
void rxSpiLedBlinkBind(void);

void rxSpiBind(void);
bool rxSpiCheckBindRequested(bool reset);
