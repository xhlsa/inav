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
 * Ported from Betaflight drivers/rx/expresslrs_driver.c.
 *
 * Authors:
 * Dominic Clifton/Hydra - Timer-based timeout implementation.
 * AlessandroAU - stdperiph Timer-based timeout implementation.
 *
 * The timer runs in 1MHz ticks and fires twice per packet interval (tick and
 * tock). Its reload value is adjusted every half period to phase lock to the
 * transmitter. INAV has no free-standing timer API, so a pinless channel-0
 * TCH is created on the dedicated timer and only its overflow callback is used.
 */

#include <string.h>

#include "platform.h"

#if defined(USE_RX_EXPRESSLRS) && !defined(AT32F43x)
#error "SPI ExpressLRS: the phase-lock timer driver is only implemented for AT32F43x"
#endif

#if defined(USE_RX_EXPRESSLRS) && defined(AT32F43x)

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

#include "drivers/rx/expresslrs_driver.h"

#define MHZ_TO_HZ(x) ((x) * 1000000)

#define TIMER_INTERVAL_US_DEFAULT 20000
#define TICK_TOCK_COUNT 2

static HAL_Timer_t *timer;
static TCH_t *timerTch;
static timerCallbacks_t timerCallbacks;
static timerHardware_t timerHw;

typedef enum {
    TICK,
    TOCK
} tickTock_e;

typedef struct elrsTimerState_s {
    bool running;
    volatile tickTock_e tickTock;
    uint32_t intervalUs;
    int32_t frequencyOffsetTicks;
    int32_t phaseShiftUs;
} elrsTimerState_t;

// Use a little ram to keep the amount of CPU cycles used in the ISR lower.
typedef struct elrsPhaseShiftLimits_s {
    int32_t min;
    int32_t max;
} elrsPhaseShiftLimits_t;

static elrsPhaseShiftLimits_t phaseShiftLimits;

static elrsTimerState_t timerState = {
    false,
    TOCK, // Start on TOCK (in ELRS isTick is initialised to false)
    TIMER_INTERVAL_US_DEFAULT,
    0,
    0
};

void expressLrsTimerDebug(void)
{
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 2, timerState.frequencyOffsetTicks);
    DEBUG_SET(DEBUG_RX_EXPRESSLRS_PHASELOCK, 3, timerState.phaseShiftUs);
}

static void expressLrsRecalculatePhaseShiftLimits(void)
{
    phaseShiftLimits.max = (timerState.intervalUs / TICK_TOCK_COUNT);
    phaseShiftLimits.min = -phaseShiftLimits.max;
}

static uint16_t expressLrsCalculateMaximumExpectedPeriod(uint16_t intervalUs)
{
    // The timer reload register must not overflow when frequencyOffsetTicks is added to it.
    // frequencyOffsetTicks is not expected to be higher than 1/4 of the interval.
    // also, timer resolution must be as high as possible.
    const uint16_t maximumExpectedPeriod = (intervalUs / TICK_TOCK_COUNT) + (timerState.intervalUs / 4);
    return maximumExpectedPeriod;
}

static void expressLrsSetReload(uint32_t reload)
{
    tmr_period_value_set(timer, reload);
}

void expressLrsUpdateTimerInterval(uint16_t intervalUs)
{
    timerState.intervalUs = intervalUs;
    expressLrsRecalculatePhaseShiftLimits();

    timerConfigBase(timerTch, expressLrsCalculateMaximumExpectedPeriod(timerState.intervalUs), MHZ_TO_HZ(1));
    expressLrsSetReload((timerState.intervalUs / TICK_TOCK_COUNT) - 1);
}

void expressLrsUpdatePhaseShift(int32_t newPhaseShift)
{
    timerState.phaseShiftUs = constrain(newPhaseShift, phaseShiftLimits.min, phaseShiftLimits.max);
}

void expressLrsTimerIncreaseFrequencyOffset(void)
{
    timerState.frequencyOffsetTicks++;
}

void expressLrsTimerDecreaseFrequencyOffset(void)
{
    timerState.frequencyOffsetTicks--;
}

void expressLrsTimerResetFrequencyOffset(void)
{
    timerState.frequencyOffsetTicks = 0;
}

static void expressLrsOnTimerUpdate(TCH_t *tch, uint32_t value)
{
    UNUSED(tch);
    UNUSED(value);

    if (timerState.tickTock == TICK) {
        uint32_t adjustedPeriod = (timerState.intervalUs / TICK_TOCK_COUNT) + timerState.frequencyOffsetTicks;

        expressLrsSetReload(adjustedPeriod - 1);

        expressLrsOnTimerTickISR();

        timerState.tickTock = TOCK;
    } else {
        uint32_t adjustedPeriod = (timerState.intervalUs / TICK_TOCK_COUNT) + timerState.phaseShiftUs + timerState.frequencyOffsetTicks;

        expressLrsSetReload(adjustedPeriod - 1);

        timerState.phaseShiftUs = 0;

        expressLrsOnTimerTockISR();

        timerState.tickTock = TICK;
    }
}

bool expressLrsTimerIsRunning(void)
{
    return timerState.running;
}

void expressLrsTimerStop(void)
{
    tmr_interrupt_enable(timer, TMR_OVF_INT, FALSE);
    tmr_counter_enable(timer, FALSE);
    tmr_counter_value_set(timer, 0);

    timerState.running = false;
}

void expressLrsTimerResume(void)
{
    timerState.tickTock = TOCK;

    expressLrsSetReload(timerState.intervalUs / TICK_TOCK_COUNT);
    tmr_counter_value_set(timer, 0);

    tmr_flag_clear(timer, TMR_OVF_FLAG);
    tmr_interrupt_enable(timer, TMR_OVF_INT, TRUE);

    timerState.running = true;

    tmr_counter_enable(timer, TRUE);
    tmr_event_sw_trigger(timer, TMR_OVERFLOW_SWTRIG);
}

bool expressLrsInitialiseTimer(void *timerInstance)
{
    HAL_Timer_t *t = (HAL_Timer_t *)timerInstance;

    int timerIndex = -1;
    for (int i = 0; i < HARDWARE_TIMER_DEFINITION_COUNT; i++) {
        if (timerDefinitions[i].tim == t) {
            timerIndex = i;
            break;
        }
    }

    if (t == NULL || timerIndex < 0 || timerDefinitions[timerIndex].irq == 0) {
        return false;
    }

    timer = t;

    // Not in the target timerHardware[] table, so timerInit() did not clock it
    RCC_ClockCmd(timerDefinitions[timerIndex].rcc, ENABLE);

    memset(&timerHw, 0, sizeof(timerHw));
    timerHw.tim = timer;
    timerHw.tag = IO_TAG(NONE);
    timerHw.channelIndex = 0;

    timerTch = timerGetTCH(&timerHw);
    if (timerTch == NULL) {
        return false;
    }

    expressLrsUpdateTimerInterval(timerState.intervalUs);

    timerChInitCallbacks(&timerCallbacks, NULL, NULL, expressLrsOnTimerUpdate);
    // Enables the update interrupt; immediately stopped again until the radio is configured
    timerChConfigCallbacks(timerTch, &timerCallbacks);
    expressLrsTimerStop();

    return true;
}

void expressLrsTimerEnableIRQs(void)
{
    impl_timerNVICConfigure(timerTch, NVIC_PRIO_RX_ELRS_TIMER);
}

#endif
