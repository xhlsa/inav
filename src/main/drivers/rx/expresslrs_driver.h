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
 * Author: Dominic Clifton / Seriously Pro Racing (Betaflight)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/timer.h"

bool expressLrsInitialiseTimer(HAL_Timer_t *timer);
void expressLrsTimerEnableIRQs(void);
void expressLrsUpdateTimerInterval(uint16_t intervalUs);
void expressLrsUpdatePhaseShift(int32_t newPhaseShift);
void expressLrsOnTimerTickISR(void);
void expressLrsOnTimerTockISR(void);

void expressLrsTimerIncreaseFrequencyOffset(void);
void expressLrsTimerDecreaseFrequencyOffset(void);
void expressLrsTimerResetFrequencyOffset(void);

void expressLrsTimerStop(void);
void expressLrsTimerResume(void);

bool expressLrsTimerIsRunning(void);

void expressLrsTimerDebug(void);
