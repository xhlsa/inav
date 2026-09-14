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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/io_impl.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
#include "drivers/sensor.h"

timerHardware_t timerHardware[] = {
    // Motor order, pins and timers as Betaflight 2025.12 LIONBEE_V1.
    // TMR5 is left free for a future ExpressLRS SPI port.
    DEF_TIM(TMR2, CH2, PA1,  TIM_USE_OUTPUT_AUTO, 0, 0),   // M1, DMA1 CH1
    DEF_TIM(TMR2, CH1, PA0,  TIM_USE_OUTPUT_AUTO, 0, 1),   // M2, DMA1 CH2
    DEF_TIM(TMR1, CH2, PA9,  TIM_USE_OUTPUT_AUTO, 0, 2),   // M3, DMA1 CH3
    DEF_TIM(TMR1, CH3, PA10, TIM_USE_OUTPUT_AUTO, 0, 3),   // M4, DMA1 CH4

    DEF_TIM(TMR3, CH4, PB1,  TIM_USE_LED, 0, 4),           // LED strip, DMA1 CH5
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);

#ifdef USE_HARDWARE_PREBOOT_SETUP
void initialisePreBootHardware(void)
{
    // Keep the unsupported RTC6705 deselected on the shared SPI3 bus.
    IOInit(DEFIO_IO(RTC6705_CS_PIN), OWNER_SYSTEM, RESOURCE_OUTPUT, 0);
    IOHi(DEFIO_IO(RTC6705_CS_PIN));
    IOConfigGPIO(DEFIO_IO(RTC6705_CS_PIN), IOCFG_OUT_PP);
}
#endif
