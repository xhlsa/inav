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
 * SPI receiver (onboard radio) RX provider. Adapted from Betaflight rx/rx_spi.c;
 * only ExpressLRS is supported.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_SPI

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"

#ifdef USE_RX_EXPRESSLRS
#include "rx/expresslrs.h"
#include "rx/expresslrs_common.h"
#endif

static uint16_t rxSpiRcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
// Written from the radio interrupt chain
static volatile uint8_t rxSpiPayload[ELRS_RX_TX_BUFF_SIZE];

static uint16_t rxSpiReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    if (channel >= rxRuntimeConfig->channelCount) {
        return 0;
    }

    return rxSpiRcData[channel];
}

// Called by the RX task check function
static uint8_t rxSpiFrameStatus(rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    uint8_t status = RX_FRAME_PENDING;

    const rx_spi_received_e result = expressLrsDataReceived((uint8_t *)rxSpiPayload);

    if (result & RX_SPI_RECEIVED_DATA) {
        uint8_t payloadCopy[ELRS_RX_TX_BUFF_SIZE];

        // The radio ISR chain may be writing the next packet into the payload
        ATOMIC_BLOCK(NVIC_PRIO_RX_ELRS_TIMER) {
            memcpy(payloadCopy, (uint8_t *)rxSpiPayload, sizeof(payloadCopy));
        }

        expressLrsSetRcDataFromPayload(rxSpiRcData, payloadCopy);
        status = RX_FRAME_COMPLETE;
    }

    return status;
}

static bool rxSpiProcessFrame(const rxRuntimeConfig_t *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);
    return true;
}

bool rxSpiInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    // Not reported to INAV until the first RC packet arrives
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxSpiRcData[i] = PWM_RANGE_MIDDLE;
    }

#ifdef USE_RX_EXPRESSLRS
    if (!expressLrsSpiInit(rxConfig, rxRuntimeConfig)) {
        return false;
    }
#else
    return false;
#endif

    rxRuntimeConfig->rcReadRawFn = rxSpiReadRawRC;
    rxRuntimeConfig->rcFrameStatusFn = rxSpiFrameStatus;
    rxRuntimeConfig->rcProcessFrameFn = rxSpiProcessFrame;

    return true;
}

#endif // USE_RX_SPI
