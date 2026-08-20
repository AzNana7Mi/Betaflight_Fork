/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
/// 适用于公司aifpv的特定版本
/// 还没有改好
#pragma once

#define FC_TARGET_MCU APM32F407
#define BOARD_NAME PR_F405_BT
#define MANUFACTURER_ID ABCD

#define TARGET_BOARD_IDENTIFIER "PRFB"
#define USBD_PRODUCT_STRING     "PR F405 BT"
// 使用私有实现MSP V2的补丁, 默认MSP Displayport使用v1, 这里加了一行补丁
// 对应文件src/main/io/displayport_msp.c
#define PATCH_20260818_ALLOW_DISPLAYPORT_MSP_V2
// 扩展补丁, OSD和关键数据传输共用一个串口
#define PATCH_20260818_EXT_OSD

#define USE_ACC
#define USE_BARO
#define USE_FLASH
#define USE_GYRO
#define USE_GYRO_SPI_ICM42688P
#define USE_ACC_SPI_ICM42688P
#define USE_BARO_DPS310
#define USE_FLASH_W25Q128FV
#define USE_MAX7456

#define BEEPER_PIN PA4 //蜂鸣器(预留)
#define MOTOR1_PIN PC8 //电机1
#define MOTOR2_PIN PC9 //电机2
#define MOTOR3_PIN PA9 //电机3
#define MOTOR4_PIN PA10 // 电机4

#define RX_PPM_PIN PA3 //接收机RX UART2
#define UART1_TX_PIN PB6
#define UART2_TX_PIN PA2
#define UART3_TX_PIN PC10
#define UART4_TX_PIN PA0
#define UART5_TX_PIN PC12
#define UART6_TX_PIN PC6
#define UART1_RX_PIN PB7
#define UART2_RX_PIN PA3
#define UART3_RX_PIN PC11
#define UART4_RX_PIN PA1
#define UART5_RX_PIN PD2
#define UART6_RX_PIN PC7
#define I2C1_SCL_PIN PB8
#define I2C1_SDA_PIN PB9
#define LED0_PIN PC5

#define SPI2_SCK_PIN PB13
#define SPI3_SCK_PIN PB3
#define SPI2_SDI_PIN PC2
#define SPI3_SDI_PIN PB4
#define SPI2_SDO_PIN PC3
#define SPI3_SDO_PIN PB5
//#define ESCSERIAL_PIN PC7
#define ADC_VBAT_PIN PC1
#define ADC_CURR_PIN PC0
#define PINIO1_PIN PC4
#define PINIO2_PIN PC13
#define FLASH_CS_PIN PA15
#define GYRO_1_EXTI_PIN PB12
#define GYRO_1_CS_PIN PB2   // 从 PA5 移开，把 PA5 让给 SPI1_SCK
// --- 定义 AT7456E (MAX7456) 的 SPI1 及引脚 ---
#define USE_SPI_DEVICE_1
#define SPI1_SCK_PIN                PA5
#define SPI1_SDI_PIN                PA6
#define SPI1_SDO_PIN                PA7
#define OSD_CS_PIN                  PB1

#define TIMER_PIN_MAPPING   TIMER_PIN_MAP( 0, MOTOR1_PIN, 1, -1 ) \
                            TIMER_PIN_MAP( 1, MOTOR2_PIN, 1, -1 ) \
                            TIMER_PIN_MAP( 2, MOTOR3_PIN, 1, 0 ) \
                            TIMER_PIN_MAP( 3, MOTOR4_PIN, 1, 0 ) \
                            TIMER_PIN_MAP( 4, RX_PPM_PIN, 2, 0 ) \

#define ADC1_DMA_OPT 0

#define SERIALRX_UART SERIAL_PORT_USART2
#define SERIALRX_PROVIDER SERIALRX_CRSF

#define MAG_I2C_INSTANCE I2CDEV_1
#define BARO_I2C_INSTANCE I2CDEV_1

#define DEFAULT_DSHOT_BITBANG DSHOT_BITBANG_ON
#define DEFAULT_DSHOT_BURST DSHOT_DMAR_OFF
#define DEFAULT_DSHOT_TELEMETRY DSHOT_TELEMETRY_ON
#define DEFAULT_MOTOR_DSHOT_SPEED PWM_TYPE_DSHOT300
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_VOLTAGE_METER_SCALE 110
#define DEFAULT_CURRENT_METER_SCALE 250
#define BEEPER_INVERTED
#define SYSTEM_HSE_MHZ 8
#define DEFAULT_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define FLASH_SPI_INSTANCE SPI3
#define GYRO_1_SPI_INSTANCE SPI2
#define OSD_SPI_INSTANCE SPI1
//#define ESC_SENSOR_UART SERIAL_PORT_UART4
#define PINIO1_BOX 40
#define PINIO2_BOX 41

#define DEFAULT_FEATURES (FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP)

