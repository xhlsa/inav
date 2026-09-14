/*
 * This file is part of INAV, ported from Betaflight (Cleanflight and Betaflight).
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
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"

#include "rx/rx.h"
#include "rx/expresslrs_common.h"

#define EXPRESSLRS_BIND_PHRASE_MAX_LENGTH 32

typedef enum {
    RX_SPI_RECEIVED_NONE = 0,
    RX_SPI_RECEIVED_BIND = (1 << 0),
    RX_SPI_RECEIVED_DATA = (1 << 1),        // New RC channel data
    RX_SPI_PROCESSING_REQUIRED = (1 << 2),
    RX_SPI_RECEIVED_LINK = (1 << 3),        // Valid packet without RC data (sync, MSP)
} rx_spi_received_e;

typedef struct rxExpressLrsSpiConfig_s {
    uint8_t UID[6];
    uint8_t domain;
    uint8_t rateIndex;
    uint8_t modelId;
    char bindPhrase[EXPRESSLRS_BIND_PHRASE_MAX_LENGTH + 1];
} rxExpressLrsSpiConfig_t;

PG_DECLARE(rxExpressLrsSpiConfig_t, rxExpressLrsSpiConfig);

bool expressLrsSpiInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig);
void expressLrsSetRcDataFromPayload(uint16_t *rcData, const uint8_t *payload);
rx_spi_received_e expressLrsDataReceived(uint8_t *payload);
rx_spi_received_e processRFPacket(volatile uint8_t *payload, uint32_t timeStampUs);
bool expressLrsIsFhssReq(void);
void expressLrsDoTelem(void);
bool expressLrsTelemRespReq(void);
void expressLrsSetRfPacketStatus(rx_spi_received_e status);
uint32_t expressLrsGetCurrentFreq(void);
volatile uint8_t *expressLrsGetRxBuffer(void);
volatile uint8_t *expressLrsGetTelemetryBuffer(void);
volatile uint8_t *expressLrsGetPayloadBuffer(void);
void expressLrsHandleTelemetryUpdate(void);
void expressLrsStop(void);
void expressLrsISR(bool runAlways);
