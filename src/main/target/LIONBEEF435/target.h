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
 * NewBeeDrone LionBee V1 (AT32F435CGU7, 1S 18650 AIO, onboard SPI ELRS).
 * Betaflight 2025.12 target LIONBEE_V1; the same board was called
 * LIONBEE_V2_REVB in Betaflight 4.5. NOT the later LIONBEE_V2 (April 2026),
 * which has a UART receiver and a different pin map.
 *
 * Pin map ported from newbeedrone/2025.12.x-config configs/LIONBEE_V1 and
 * newbeedrone/nbd-betaflight 4.5-release target LIONBEE_V2_REVB.
 *
 * Onboard ExpressLRS receiver: SX1280 on SPI2, driven by the ported
 * Betaflight SPI ExpressLRS stack (receiver_type = SPI). ELRS 3.x and 4.x
 * are separate builds (LIONBEEF435_ELRS3 / LIONBEEF435_ELRS4) and must match
 * the transmitter's major version.
 *
 * NOT YET SUPPORTED BY THIS TARGET:
 *  - Onboard RTC6705 VTX (SPI3, CS PB2). INAV has no RTC6705 driver, so
 *    the VTX frequency is not programmed. The external PA power-select
 *    lines are exposed as PINIO1/PINIO2 for bench experiments only.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER         "LNB1"
#define USBD_PRODUCT_STRING             "LionBee V1"

// *************** LED / BEEPER ********************
#define LED0                            PB8

// No beeper pin on this board; use DShot beacon.

// *************** Gyro & ACC **********************
#define USE_SPI
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                    PA5
#define SPI1_MISO_PIN                   PA6
#define SPI1_MOSI_PIN                   PA7
#define SPI1_NSS_PIN                    PA4

// Boards ship with either ICM42688P or (from March 2026) BMI270 on the same
// footprint. Betaflight applies gyro CW270 plus board pitch 180 (ICM42688P)
// or board pitch 180 + yaw 90 (BMI270). Composed through Betaflight's
// rotation matrix these are exactly INAV CW90_DEG_FLIP and CW180_DEG_FLIP.
#define USE_IMU_ICM42605
#define IMU_ICM42605_ALIGN              CW90_DEG_FLIP
#define ICM42605_SPI_BUS                BUS_SPI1
#define ICM42605_CS_PIN                 SPI1_NSS_PIN

#define USE_IMU_BMI270
#define IMU_BMI270_ALIGN                CW180_DEG_FLIP
#define BMI270_SPI_BUS                  BUS_SPI1
#define BMI270_CS_PIN                   SPI1_NSS_PIN

// *************** SPI3: OSD / FLASH / VTX *********
#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN                    PB12
#define SPI3_MISO_PIN                   PB4
#define SPI3_MOSI_PIN                   PB0
// PB12 SCK and PB0 MOSI are MUX7 on AT32F435; INAV's SPI3 default is MUX6.
#define SPI3_SCK_AF                     GPIO_MUX_7
#define SPI3_MISO_AF                    GPIO_MUX_6
#define SPI3_MOSI_AF                    GPIO_MUX_7

#define USE_MAX7456
#define MAX7456_SPI_BUS                 BUS_SPI3
#define MAX7456_CS_PIN                  PC14

#define USE_FLASHFS
#define USE_FLASH_M25P16
#define M25P16_SPI_BUS                  BUS_SPI3
#define M25P16_CS_PIN                   PC15
#define USE_FLASH_W25N01G
#define W25N01G_SPI_BUS                 BUS_SPI3
#define W25N01G_CS_PIN                  PC15
#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

// RTC6705 VTX, driver not available in INAV:
//   CS PB2 on SPI3, PA power select PA14 (LSB) / PB7 (MSB).
// PB2 is driven high in initialisePreBootHardware() so the VTX stays
// deselected while the OSD and flash use the shared bus.
#define RTC6705_CS_PIN                  PB2
#define USE_HARDWARE_PREBOOT_SETUP

// *************** Onboard ELRS receiver ***********
#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN                    PB13
#define SPI2_MISO_PIN                   PB14
#define SPI2_MOSI_PIN                   PB15

#define USE_RX_SPI
#define USE_RX_EXPRESSLRS
#define USE_RX_SX1280
#if !defined(USE_ELRSV3) && !defined(USE_ELRSV4)
#define USE_ELRSV3
#endif

#define SX1280_SPI_BUS                  BUS_SPI2
#define SX1280_CS_PIN                   PA8
#define RX_SPI_EXTI_PIN                 PB3     // DIO1
#define RX_EXPRESSLRS_SPI_BUSY_PIN      PA15
#define RX_EXPRESSLRS_SPI_RESET_PIN     PH3
#define RX_SPI_BIND_PIN                 PH2
#define RX_SPI_LED_PIN                  PB9
#define RX_SPI_LED_INVERTED
// Betaflight 2025.12 uses TMR5 for ELRS timing, so motors avoid TMR5.
#define RX_EXPRESSLRS_TIMER_INSTANCE    TMR5
// Gyro EXTI is PC13 (unused by INAV's ICM42605/BMI270 drivers).

// *************** I2C: Baro / Mag *****************
#define USE_I2C
#define USE_I2C_DEVICE_2
#define I2C2_SCL                        PB10
#define I2C2_SDA                        PB11
#define DEFAULT_I2C_BUS                 BUS_I2C2

#define USE_BARO
#define BARO_I2C_BUS                    BUS_I2C2
#define USE_BARO_BMP280
#define USE_BARO_BMP388
#define USE_BARO_DPS310
#define USE_BARO_SPL06
#define USE_BARO_MS5611

#define USE_MAG
#define MAG_I2C_BUS                     BUS_I2C2
#define USE_MAG_ALL

// *************** UART ****************************
#define USE_VCP

#define USE_UART5
#define UART5_RX_PIN                    PB5
#define UART5_TX_PIN                    PB6

#define SERIAL_PORT_COUNT               2   // VCP, UART5

#define DEFAULT_RX_TYPE                 RX_TYPE_SPI

// *************** ADC *****************************
#define USE_ADC
#define ADC_INSTANCE                    ADC1
#define ADC1_DMA_STREAM                 DMA2_CHANNEL5
#define ADC_CHANNEL_1_PIN               PA2
#define ADC_CHANNEL_2_PIN               PA3
#define VBAT_ADC_CHANNEL                ADC_CHN_1
#define CURRENT_METER_ADC_CHANNEL       ADC_CHN_2

// Betaflight 2025.12 LIONBEE_V1: vbat_scale 110, ibata_scale 447
// (4.5 REV_B had 58 / 513). INAV vbat scale is x10. Calibrate with a meter.
#define VBAT_SCALE_DEFAULT              1100
#define CURRENT_METER_SCALE             447

// *************** PINIO (VTX PA power select) *****
#define USE_PINIO
#define USE_PINIOBOX
#define PINIO1_PIN                      PA14
#define PINIO2_PIN                      PB7

// *************** LED STRIP ***********************
#define USE_LED_STRIP
#define WS2811_PIN                      PB1

#define DEFAULT_FEATURES                (FEATURE_TX_PROF_SEL | FEATURE_VBAT | FEATURE_CURRENT_METER | \
                                         FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_GPS | FEATURE_LED_STRIP)

// *************** Motors **************************
#define USE_SERIAL_4WAY_BLHELI_INTERFACE
#define ENABLE_DSHOT
#define USE_DSHOT
#define USE_ESC_SENSOR

#define MAX_PWM_OUTPUT_PORTS            4

#define TARGET_IO_PORTA                 0xffff
#define TARGET_IO_PORTB                 0xffff
#define TARGET_IO_PORTC                 0xffff
#define TARGET_IO_PORTD                 0xffff
#define TARGET_IO_PORTH                 (BIT(2) | BIT(3))
