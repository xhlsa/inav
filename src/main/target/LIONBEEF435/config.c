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

#include <stdint.h>

#include "platform.h"

#include "fc/fc_msp_box.h"
#include "fc/config.h"

#include "drivers/pwm_mapping.h"

#include "flight/mixer.h"
#include "flight/mixer_profile.h"

#include "io/piniobox.h"
#include "io/serial.h"

#include "sensors/barometer.h"
#include "sensors/compass.h"

void targetConfiguration(void)
{
    // GPS (M10Q) on the only hardware UART.
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART5)].functionMask = FUNCTION_GPS;

    // Settings carried over from Betaflight 2025.12 LIONBEE_V1 config.c.
    motorConfigMutable()->motorPwmProtocol = PWM_TYPE_DSHOT600;
    motorConfigMutable()->motorPoleCount = 12;

    // NewBeeDrone firmware drives this baro with the DPS310 driver. INAV's
    // autodetect tries SPL06 first and both chips report ID 0x10.
    barometerConfigMutable()->baro_hardware = BARO_DPS310;

    // Betaflight yaw_motors_reversed = true (props out).
    for (int i = 0; i < MAX_MIXER_PROFILE_COUNT; i++) {
        mixerProfilesMutable(i)->mixer_config.motorDirectionInverted = true;
    }

    // Betaflight: QMC5883 at CW180_DEG, then board pitch 180 => CW180_DEG_FLIP.
    // The compass is in the GPS module, so this does not change with the
    // gyro variant. Betaflight's extra board yaw 90 on BMI270 boards corrects
    // the gyro chip and is not carried into the compass here. Verify on bench.
    compassConfigMutable()->mag_hardware = MAG_QMC5883;
    compassConfigMutable()->mag_align = CW180_DEG_FLIP;

    pinioBoxConfigMutable()->permanentId[0] = BOX_PERMANENT_ID_USER1;
    pinioBoxConfigMutable()->permanentId[1] = BOX_PERMANENT_ID_USER2;
}
