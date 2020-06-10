use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strncpy(_: *mut libc::c_char, _: *const libc::c_char, _: libc::c_ulong)
     -> *mut libc::c_char;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    static shortGitRevision: *const libc::c_char;
    #[no_mangle]
    static targetName: *const libc::c_char;
    /*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
    #[no_mangle]
    static mut debug: [int16_t; 4];
    #[no_mangle]
    fn displayIsGrabbed(instance: *const displayPort_t) -> bool;
    #[no_mangle]
    fn ug2864hsweg01InitI2C(bus_0: *mut busDevice_t) -> bool;
    #[no_mangle]
    fn i2c_OLED_set_xy(bus_0: *mut busDevice_t, col: uint8_t, row: uint8_t);
    #[no_mangle]
    fn i2c_OLED_set_line(bus_0: *mut busDevice_t, row: uint8_t);
    #[no_mangle]
    fn i2c_OLED_send_char(bus_0: *mut busDevice_t, ascii: libc::c_uchar);
    #[no_mangle]
    fn i2c_OLED_send_string(bus_0: *mut busDevice_t,
                            string: *const libc::c_char);
    #[no_mangle]
    fn i2c_OLED_clear_display_quick(bus_0: *mut busDevice_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    // Device management
    #[no_mangle]
    fn cmsDisplayPortRegister(pDisplay: *mut displayPort_t) -> bool;
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut dashboardConfig_System: dashboardConfig_t;
    #[no_mangle]
    static mut currentPidProfile: *mut pidProfile_s;
    #[no_mangle]
    fn getCurrentPidProfileIndex() -> uint8_t;
    #[no_mangle]
    fn getCurrentControlRateProfileIndex() -> uint8_t;
    #[no_mangle]
    static mut controlRateProfiles_SystemArray: [controlRateConfig_t; 6];
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn failsafePhase() -> failsafePhase_e;
    // To make this useful we should log as many packets as we can fit characters a single line of a OLED display.
    #[no_mangle]
    static mut gpsPacketLog: [libc::c_char; 21];
    // Navigation mode
    #[no_mangle]
    static mut gpsData: gpsData_t;
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    // it's a binary toogle to distinct a GPS position update
    #[no_mangle]
    static mut GPS_packetCount: uint32_t;
    #[no_mangle]
    static mut GPS_svInfoReceivedCount: uint32_t;
    // Bitfield Qualtity
    #[no_mangle]
    static mut GPS_svinfo_cno: [uint8_t; 16];
    /*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
    #[no_mangle]
    fn displayPortOledInit(device: *mut libc::c_void) -> *mut displayPort_t;
    #[no_mangle]
    static rcChannelLetters: [libc::c_char; 0];
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    static mut rxRuntimeConfig: rxRuntimeConfig_t;
    #[no_mangle]
    fn rxIsReceivingSignal() -> bool;
    #[no_mangle]
    fn rxAreFlightChannelsValid() -> bool;
    #[no_mangle]
    fn getTaskInfo(taskId: cfTaskId_e, taskInfo: *mut cfTaskInfo_t);
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    static mut gyro: gyro_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    static mut mag: mag_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SR: uint32_t,
    pub DR: uint32_t,
    pub CRCPR: uint32_t,
    pub RXCRCR: uint32_t,
    pub TXCRCR: uint32_t,
    pub I2SCFGR: uint32_t,
    pub I2SPR: uint32_t,
}
/* * 
  * @brief  HAL Lock structures definition  
  */
pub type HAL_LockTypeDef = libc::c_uint;
pub const HAL_LOCKED: HAL_LockTypeDef = 1;
pub const HAL_UNLOCKED: HAL_LockTypeDef = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dma.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup DMA
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMA_Exported_Types DMA Exported Types
  * @brief    DMA Exported Types 
  * @{
  */
/* * 
  * @brief  DMA Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub Channel: uint32_t,
    pub Direction: uint32_t,
    pub PeriphInc: uint32_t,
    pub MemInc: uint32_t,
    pub PeriphDataAlignment: uint32_t,
    pub MemDataAlignment: uint32_t,
    pub Mode: uint32_t,
    pub Priority: uint32_t,
    pub FIFOMode: uint32_t,
    pub FIFOThreshold: uint32_t,
    pub MemBurst: uint32_t,
    pub PeriphBurst: uint32_t,
}
/* * 
  * @brief  HAL DMA State structures definition
  */
pub type HAL_DMA_StateTypeDef = libc::c_uint;
/* !< DMA Abort state                     */
/* !< DMA error state                     */
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
/* !< DMA timeout state                   */
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
/* !< DMA process is ongoing              */
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
/* !< DMA initialized and ready for use   */
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
/* !< DMA not yet initialized or disabled */
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
/* * 
  * @brief  DMA handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __DMA_HandleTypeDef {
    pub Instance: *mut DMA_Stream_TypeDef,
    pub Init: DMA_InitTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_DMA_StateTypeDef,
    pub Parent: *mut libc::c_void,
    pub XferCpltCallback: Option<unsafe extern "C" fn(_:
                                                          *mut __DMA_HandleTypeDef)
                                     -> ()>,
    pub XferHalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                              *mut __DMA_HandleTypeDef)
                                         -> ()>,
    pub XferM1CpltCallback: Option<unsafe extern "C" fn(_:
                                                            *mut __DMA_HandleTypeDef)
                                       -> ()>,
    pub XferM1HalfCpltCallback: Option<unsafe extern "C" fn(_:
                                                                *mut __DMA_HandleTypeDef)
                                           -> ()>,
    pub XferErrorCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub XferAbortCallback: Option<unsafe extern "C" fn(_:
                                                           *mut __DMA_HandleTypeDef)
                                      -> ()>,
    pub ErrorCode: uint32_t,
    pub StreamBaseAddress: uint32_t,
    pub StreamIndex: uint32_t,
}
pub type DMA_HandleTypeDef = __DMA_HandleTypeDef;
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_spi.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of SPI HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @addtogroup SPI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */
/* *
  * @brief  SPI Configuration Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_InitTypeDef {
    pub Mode: uint32_t,
    pub Direction: uint32_t,
    pub DataSize: uint32_t,
    pub CLKPolarity: uint32_t,
    pub CLKPhase: uint32_t,
    pub NSS: uint32_t,
    pub BaudRatePrescaler: uint32_t,
    pub FirstBit: uint32_t,
    pub TIMode: uint32_t,
    pub CRCCalculation: uint32_t,
    pub CRCPolynomial: uint32_t,
    pub CRCLength: uint32_t,
    pub NSSPMode: uint32_t,
}
/* *
  * @brief  HAL SPI State structure definition
  */
pub type HAL_SPI_StateTypeDef = libc::c_uint;
/* !< SPI abort is ongoing                               */
/* !< SPI error state                                    */
pub const HAL_SPI_STATE_ABORT: HAL_SPI_StateTypeDef = 7;
/* !< Data Transmission and Reception process is ongoing */
pub const HAL_SPI_STATE_ERROR: HAL_SPI_StateTypeDef = 6;
/* !< Data Reception process is ongoing                  */
pub const HAL_SPI_STATE_BUSY_TX_RX: HAL_SPI_StateTypeDef = 5;
/* !< Data Transmission process is ongoing               */
pub const HAL_SPI_STATE_BUSY_RX: HAL_SPI_StateTypeDef = 4;
/* !< an internal process is ongoing                     */
pub const HAL_SPI_STATE_BUSY_TX: HAL_SPI_StateTypeDef = 3;
/* !< Peripheral Initialized and ready for use           */
pub const HAL_SPI_STATE_BUSY: HAL_SPI_StateTypeDef = 2;
/* !< Peripheral not Initialized                         */
pub const HAL_SPI_STATE_READY: HAL_SPI_StateTypeDef = 1;
pub const HAL_SPI_STATE_RESET: HAL_SPI_StateTypeDef = 0;
/* *
  * @brief  SPI handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __SPI_HandleTypeDef {
    pub Instance: *mut SPI_TypeDef,
    pub Init: SPI_InitTypeDef,
    pub pTxBuffPtr: *mut uint8_t,
    pub TxXferSize: uint16_t,
    pub TxXferCount: uint16_t,
    pub pRxBuffPtr: *mut uint8_t,
    pub RxXferSize: uint16_t,
    pub RxXferCount: uint16_t,
    pub CRCSize: uint32_t,
    pub RxISR: Option<unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                          -> ()>,
    pub TxISR: Option<unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                          -> ()>,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_SPI_StateTypeDef,
    pub ErrorCode: uint32_t,
}
pub type SPI_HandleTypeDef = __SPI_HandleTypeDef;
/* !< SPI Error code                           */
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub handle: *mut SPI_HandleTypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
// cached here for efficiency
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type displayPort_t = displayPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
// CMS state
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const Z: C2RustUnnamed_0 = 2;
pub const Y: C2RustUnnamed_0 = 1;
pub const X: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FD_YAW: C2RustUnnamed_1 = 2;
pub const FD_PITCH: C2RustUnnamed_1 = 1;
pub const FD_ROLL: C2RustUnnamed_1 = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_2 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_2 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_2 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_2 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_2 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_2 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_2 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_2 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_2 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_2 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_2 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_2 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_2 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_2 = 8192;
pub const FEATURE_3D: C2RustUnnamed_2 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_2 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_2 = 512;
pub const FEATURE_GPS: C2RustUnnamed_2 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_2 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_2 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_2 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_2 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_2 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_2 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dashboardConfig_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type dashboardConfig_t = dashboardConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidProfile_s {
    pub yaw_lowpass_hz: uint16_t,
    pub dterm_lowpass_hz: uint16_t,
    pub dterm_notch_hz: uint16_t,
    pub dterm_notch_cutoff: uint16_t,
    pub pid: [pidf_t; 5],
    pub dterm_filter_type: uint8_t,
    pub itermWindupPointPercent: uint8_t,
    pub pidSumLimit: uint16_t,
    pub pidSumLimitYaw: uint16_t,
    pub pidAtMinThrottle: uint8_t,
    pub levelAngleLimit: uint8_t,
    pub horizon_tilt_effect: uint8_t,
    pub horizon_tilt_expert_mode: uint8_t,
    pub antiGravityMode: uint8_t,
    pub itermThrottleThreshold: uint16_t,
    pub itermAcceleratorGain: uint16_t,
    pub yawRateAccelLimit: uint16_t,
    pub rateAccelLimit: uint16_t,
    pub crash_dthreshold: uint16_t,
    pub crash_gthreshold: uint16_t,
    pub crash_setpoint_threshold: uint16_t,
    pub crash_time: uint16_t,
    pub crash_delay: uint16_t,
    pub crash_recovery_angle: uint8_t,
    pub crash_recovery_rate: uint8_t,
    pub vbatPidCompensation: uint8_t,
    pub feedForwardTransition: uint8_t,
    pub crash_limit_yaw: uint16_t,
    pub itermLimit: uint16_t,
    pub dterm_lowpass2_hz: uint16_t,
    pub crash_recovery: uint8_t,
    pub throttle_boost: uint8_t,
    pub throttle_boost_cutoff: uint8_t,
    pub iterm_rotation: uint8_t,
    pub smart_feedforward: uint8_t,
    pub iterm_relax_type: uint8_t,
    pub iterm_relax_cutoff: uint8_t,
    pub iterm_relax: uint8_t,
    pub acro_trainer_angle_limit: uint8_t,
    pub acro_trainer_debug_axis: uint8_t,
    pub acro_trainer_gain: uint8_t,
    pub acro_trainer_lookahead_ms: uint16_t,
    pub abs_control_gain: uint8_t,
    pub abs_control_limit: uint8_t,
    pub abs_control_error_limit: uint8_t,
}
pub type pidf_t = pidf_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidf_s {
    pub P: uint8_t,
    pub I: uint8_t,
    pub D: uint8_t,
    pub F: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct controlRateConfig_s {
    pub thrMid8: uint8_t,
    pub thrExpo8: uint8_t,
    pub rates_type: uint8_t,
    pub rcRates: [uint8_t; 3],
    pub rcExpo: [uint8_t; 3],
    pub rates: [uint8_t; 3],
    pub dynThrPID: uint8_t,
    pub tpa_breakpoint: uint16_t,
    pub throttle_limit_type: uint8_t,
    pub throttle_limit_percent: uint8_t,
}
pub type controlRateConfig_t = controlRateConfig_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_3 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_3 = 2;
pub const ARMED: C2RustUnnamed_3 = 1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_4 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_4 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_4 = 4;
pub const GPS_FIX: C2RustUnnamed_4 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_4 = 1;
pub type pidProfile_t = pidProfile_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_5,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_5 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type failsafePhase_e = libc::c_uint;
pub const FAILSAFE_GPS_RESCUE: failsafePhase_e = 6;
pub const FAILSAFE_RX_LOSS_RECOVERED: failsafePhase_e = 5;
pub const FAILSAFE_RX_LOSS_MONITORING: failsafePhase_e = 4;
pub const FAILSAFE_LANDED: failsafePhase_e = 3;
pub const FAILSAFE_LANDING: failsafePhase_e = 2;
pub const FAILSAFE_RX_LOSS_DETECTED: failsafePhase_e = 1;
pub const FAILSAFE_IDLE: failsafePhase_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
// Breakpoint where TPA is activated
// Sets the throttle limiting type - off, scale or clip
// Sets the maximum pilot commanded throttle limit
// Additional yaw filter when yaw axis too noisy
// Delta Filter in hz
// Biquad dterm notch hz
// Biquad dterm notch low cutoff
// Filter selection for dterm
// Experimental ITerm windup threshold, percent motor saturation
// Disable/Enable pids on zero throttle. Normally even without airmode P and D would be active.
// Max angle in degrees in level mode
// inclination factor for Horizon mode
// OFF or ON
// Betaflight PID controller parameters
// type of anti gravity method
// max allowed throttle delta before iterm accelerated in ms
// Iterm Accelerator Gain when itermThrottlethreshold is hit
// yaw accel limiter for deg/sec/ms
// accel limiter roll/pitch deg/sec/ms
// dterm crash value
// gyro crash value
// setpoint must be below this value to detect crash, so flips and rolls are not interpreted as crashes
// ms
// ms
// degrees
// degree/second
// Scale PIDsum to battery voltage
// Feed forward weight transition
// limits yaw errorRate, so crashes don't cause huge throttle increase
// Extra PT1 Filter on D in hz
// off, on, on and beeps when it is in crash recovery mode
// how much should throttle be boosted during transient changes 0-100, 100 adds 10x hpf filtered throttle
// Which cutoff frequency to use for throttle boost. higher cutoffs keep the boost on for shorter. Specified in hz.
// rotates iterm to translate world errors to local coordinate system
// takes only the larger of P and the D weight feed forward term if they have the same sign.
// Specifies type of relax algorithm
// This cutoff frequency specifies a low pass filter which predicts average response of the quad to setpoint
// Enable iterm suppression during stick input
// Acro trainer roll/pitch angle limit in degrees
// The axis for which record debugging values are captured 0=roll, 1=pitch
// The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
// The lookahead window in milliseconds used to reduce overshoot
// How strongly should the absolute accumulated error be corrected for
// Limit to the correction
// Limit to the accumulated error
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
pub type gpsSolutionData_t = gpsSolutionData_s;
pub type gpsMessageState_e = libc::c_uint;
pub const GPS_MESSAGE_STATE_ENTRY_COUNT: gpsMessageState_e = 4;
pub const GPS_MESSAGE_STATE_GALILEO: gpsMessageState_e = 3;
pub const GPS_MESSAGE_STATE_SBAS: gpsMessageState_e = 2;
pub const GPS_MESSAGE_STATE_INIT: gpsMessageState_e = 1;
pub const GPS_MESSAGE_STATE_IDLE: gpsMessageState_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsData_s {
    pub errors: uint32_t,
    pub timeouts: uint32_t,
    pub lastMessage: uint32_t,
    pub lastLastMessage: uint32_t,
    pub state_position: uint32_t,
    pub state_ts: uint32_t,
    pub state: uint8_t,
    pub baudrateIndex: uint8_t,
    pub messageState: gpsMessageState_e,
}
pub type gpsData_t = gpsData_s;
pub type pageId_e = libc::c_uint;
pub const PAGE_COUNT: pageId_e = 9;
pub const PAGE_DEBUG: pageId_e = 8;
pub const PAGE_GPS: pageId_e = 7;
pub const PAGE_TASKS: pageId_e = 6;
pub const PAGE_PROFILE: pageId_e = 5;
pub const PAGE_RX: pageId_e = 4;
pub const PAGE_SENSORS: pageId_e = 3;
pub const PAGE_BATTERY: pageId_e = 2;
pub const PAGE_ARMED: pageId_e = 1;
pub const PAGE_WELCOME: pageId_e = 0;
pub type pageState_t = pageState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pageState_s {
    pub pageChanging: bool,
    pub page: *const pageEntry_t,
    pub pageFlags: uint8_t,
    pub cycleIndex: uint8_t,
    pub nextPageAt: uint32_t,
}
pub type pageEntry_t = pageEntry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pageEntry_s {
    pub id: pageId_e,
    pub title: *mut libc::c_char,
    pub drawFn: pageFnPtr,
    pub flags: uint8_t,
}
pub type pageFnPtr = Option<unsafe extern "C" fn() -> ()>;
pub const PAGE_STATE_FLAG_FORCE_PAGE_CHANGE: C2RustUnnamed_7 = 2;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfTaskInfo_t {
    pub taskName: *const libc::c_char,
    pub subTaskName: *const libc::c_char,
    pub isEnabled: bool,
    pub staticPriority: uint8_t,
    pub desiredPeriod: timeDelta_t,
    pub latestDeltaTime: timeDelta_t,
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
    pub averageExecutionTime: timeUs_t,
}
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 30;
pub const TASK_NONE: cfTaskId_e = 29;
pub const TASK_COUNT: cfTaskId_e = 29;
pub const TASK_PINIOBOX: cfTaskId_e = 28;
pub const TASK_ADC_INTERNAL: cfTaskId_e = 27;
pub const TASK_RCDEVICE: cfTaskId_e = 26;
pub const TASK_CAMCTRL: cfTaskId_e = 25;
pub const TASK_VTXCTRL: cfTaskId_e = 24;
pub const TASK_CMS: cfTaskId_e = 23;
pub const TASK_ESC_SENSOR: cfTaskId_e = 22;
pub const TASK_OSD: cfTaskId_e = 21;
pub const TASK_TRANSPONDER: cfTaskId_e = 20;
pub const TASK_LEDSTRIP: cfTaskId_e = 19;
pub const TASK_TELEMETRY: cfTaskId_e = 18;
pub const TASK_DASHBOARD: cfTaskId_e = 17;
pub const TASK_ALTITUDE: cfTaskId_e = 16;
pub const TASK_RANGEFINDER: cfTaskId_e = 15;
pub const TASK_BARO: cfTaskId_e = 14;
pub const TASK_COMPASS: cfTaskId_e = 13;
pub const TASK_GPS: cfTaskId_e = 12;
pub const TASK_BEEPER: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
pub type mag_t = mag_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mag_s {
    pub magADC: [libc::c_float; 3],
    pub magneticDeclination: libc::c_float,
}
pub const SENSOR_MAG: C2RustUnnamed_6 = 8;
pub type gyro_t = gyro_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub const SENSOR_GYRO: C2RustUnnamed_6 = 1;
pub type acc_t = acc_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type accDev_t = accDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDev_s {
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
}
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
// gps error counter - crc error/lost of data/sync etc..
// last time valid GPS data was received (millis)
// last-last valid GPS message. Used to calculate delta.
// incremental variable for loops
// timestamp for last state_position increment
// GPS thread state. Used for detecting cable disconnects and configuring attached devices
// index into auto-detecting or current baudrate
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
// driver-provided alignment
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
pub const SENSOR_ACC: C2RustUnnamed_6 = 2;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub type batteryConfig_t = batteryConfig_s;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
// number of RC channels as reported by current input driver
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub const PAGE_STATE_FLAG_CYCLE_ENABLED: C2RustUnnamed_7 = 1;
pub type C2RustUnnamed_6 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_6 = 64;
pub const SENSOR_GPS: C2RustUnnamed_6 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_6 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_6 = 16;
pub const SENSOR_BARO: C2RustUnnamed_6 = 4;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const PAGE_STATE_FLAG_NONE: C2RustUnnamed_7 = 0;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn dashboardConfig() -> *const dashboardConfig_t {
    return &mut dashboardConfig_System;
}
#[inline]
unsafe extern "C" fn controlRateProfiles(mut _index: libc::c_int)
 -> *const controlRateConfig_t {
    return &mut *controlRateProfiles_SystemArray.as_mut_ptr().offset(_index as
                                                                         isize)
               as *mut controlRateConfig_t;
}
// Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
static mut bus: *mut busDevice_t =
    0 as *const busDevice_t as *mut busDevice_t;
static mut nextDisplayUpdateAt: uint32_t = 0 as libc::c_int as uint32_t;
static mut dashboardPresent: bool = 0 as libc::c_int != 0;
static mut displayPort: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
static mut lineBuffer: [libc::c_char; 22] = [0; 22];
static mut tickerCharacters: *const libc::c_char =
    b"|/-\\\x00" as *const u8 as *const libc::c_char;
static mut pageState: pageState_t =
    pageState_t{pageChanging: false,
                page: 0 as *const pageEntry_t,
                pageFlags: 0,
                cycleIndex: 0,
                nextPageAt: 0,};
unsafe extern "C" fn resetDisplay() {
    dashboardPresent = ug2864hsweg01InitI2C(bus);
}
#[no_mangle]
pub unsafe extern "C" fn LCDprint(mut i: uint8_t) {
    i2c_OLED_send_char(bus, i);
}
unsafe extern "C" fn padLineBuffer() {
    let mut length: uint8_t = strlen(lineBuffer.as_mut_ptr()) as uint8_t;
    while (length as libc::c_ulong) <
              (::core::mem::size_of::<[libc::c_char; 22]>() as
                   libc::c_ulong).wrapping_sub(1 as libc::c_int as
                                                   libc::c_ulong) {
        let fresh0 = length;
        length = length.wrapping_add(1);
        lineBuffer[fresh0 as usize] = ' ' as i32 as libc::c_char
    }
    lineBuffer[length as usize] = 0 as libc::c_int as libc::c_char;
}
unsafe extern "C" fn padHalfLineBuffer() {
    let mut halfLineIndex: uint8_t =
        (::core::mem::size_of::<[libc::c_char; 22]>() as
             libc::c_ulong).wrapping_div(2 as libc::c_int as libc::c_ulong) as
            uint8_t;
    let mut length: uint8_t = strlen(lineBuffer.as_mut_ptr()) as uint8_t;
    while (length as libc::c_int) <
              halfLineIndex as libc::c_int - 1 as libc::c_int {
        let fresh1 = length;
        length = length.wrapping_add(1);
        lineBuffer[fresh1 as usize] = ' ' as i32 as libc::c_char
    }
    lineBuffer[length as usize] = 0 as libc::c_int as libc::c_char;
}
// LCDbar(n,v) : draw a bar graph - n number of chars for width, v value in % to display
unsafe extern "C" fn drawHorizonalPercentageBar(mut width: uint8_t,
                                                mut percent: uint8_t) {
    let mut i: uint8_t = 0; // full
    let mut j: uint8_t = 0; // partial fill
    if percent as libc::c_int > 100 as libc::c_int {
        percent = 100 as libc::c_int as uint8_t
    }
    j =
        (width as libc::c_int * percent as libc::c_int / 100 as libc::c_int)
            as uint8_t;
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < j as libc::c_int {
        LCDprint(159 as libc::c_int as uint8_t);
        i = i.wrapping_add(1)
    }
    if (j as libc::c_int) < width as libc::c_int {
        LCDprint((154 as libc::c_int +
                      (percent as libc::c_int * width as libc::c_int *
                           5 as libc::c_int / 100 as libc::c_int -
                           5 as libc::c_int * j as libc::c_int)) as uint8_t);
    }
    i = (j as libc::c_int + 1 as libc::c_int) as uint8_t;
    while (i as libc::c_int) < width as libc::c_int {
        LCDprint(154 as libc::c_int as uint8_t);
        i = i.wrapping_add(1)
    };
    // empty
}
unsafe extern "C" fn updateTicker() {
    static mut tickerIndex: uint8_t = 0 as libc::c_int as uint8_t;
    i2c_OLED_set_xy(bus,
                    (128 as libc::c_int /
                         (5 as libc::c_int + 1 as libc::c_int) -
                         1 as libc::c_int) as uint8_t,
                    0 as libc::c_int as uint8_t);
    i2c_OLED_send_char(bus,
                       *tickerCharacters.offset(tickerIndex as isize) as
                           libc::c_uchar);
    tickerIndex = tickerIndex.wrapping_add(1);
    tickerIndex =
        (tickerIndex as
             libc::c_ulong).wrapping_rem((::core::mem::size_of::<*const libc::c_char>()
                                              as
                                              libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_char>()
                                                                              as
                                                                              libc::c_ulong))
            as uint8_t;
}
unsafe extern "C" fn updateRxStatus() {
    i2c_OLED_set_xy(bus,
                    (128 as libc::c_int /
                         (5 as libc::c_int + 1 as libc::c_int) -
                         2 as libc::c_int) as uint8_t,
                    0 as libc::c_int as uint8_t);
    let mut rxStatus: libc::c_char = '!' as i32 as libc::c_char;
    if rxIsReceivingSignal() { rxStatus = 'r' as i32 as libc::c_char }
    if rxAreFlightChannelsValid() { rxStatus = 'R' as i32 as libc::c_char }
    i2c_OLED_send_char(bus, rxStatus as libc::c_uchar);
}
unsafe extern "C" fn updateFailsafeStatus() {
    let mut failsafeIndicator: libc::c_char = '?' as i32 as libc::c_char;
    match failsafePhase() as libc::c_uint {
        0 => { failsafeIndicator = '-' as i32 as libc::c_char }
        1 => { failsafeIndicator = 'R' as i32 as libc::c_char }
        2 => { failsafeIndicator = 'l' as i32 as libc::c_char }
        3 => { failsafeIndicator = 'L' as i32 as libc::c_char }
        4 => { failsafeIndicator = 'M' as i32 as libc::c_char }
        5 => { failsafeIndicator = 'r' as i32 as libc::c_char }
        6 => { failsafeIndicator = 'G' as i32 as libc::c_char }
        _ => { }
    }
    i2c_OLED_set_xy(bus,
                    (128 as libc::c_int /
                         (5 as libc::c_int + 1 as libc::c_int) -
                         3 as libc::c_int) as uint8_t,
                    0 as libc::c_int as uint8_t);
    i2c_OLED_send_char(bus, failsafeIndicator as libc::c_uchar);
}
unsafe extern "C" fn showTitle() {
    i2c_OLED_set_line(bus, 0 as libc::c_int as uint8_t);
    i2c_OLED_send_string(bus, (*pageState.page).title);
}
unsafe extern "C" fn handlePageChange() {
    i2c_OLED_clear_display_quick(bus);
    showTitle();
}
unsafe extern "C" fn drawRxChannel(mut channelIndex: uint8_t,
                                   mut width: uint8_t) {
    LCDprint(*rcChannelLetters.as_ptr().offset(channelIndex as isize) as
                 uint8_t);
    let percentage: uint32_t =
        ((constrain(rcData[channelIndex as usize] as libc::c_int,
                    1000 as libc::c_int, 2000 as libc::c_int) -
              1000 as libc::c_int) * 100 as libc::c_int /
             (2000 as libc::c_int - 1000 as libc::c_int)) as uint32_t;
    drawHorizonalPercentageBar((width as libc::c_int - 1 as libc::c_int) as
                                   uint8_t, percentage as uint8_t);
}
unsafe extern "C" fn showRxPage() {
    let mut channelIndex: libc::c_int = 0 as libc::c_int;
    while channelIndex < rxRuntimeConfig.channelCount as libc::c_int &&
              channelIndex < 14 as libc::c_int {
        i2c_OLED_set_line(bus,
                          (channelIndex / 2 as libc::c_int + 1 as libc::c_int)
                              as uint8_t);
        drawRxChannel(channelIndex as uint8_t,
                      (128 as libc::c_int /
                           (5 as libc::c_int + 1 as libc::c_int) /
                           2 as libc::c_int) as uint8_t);
        if !(channelIndex >= rxRuntimeConfig.channelCount as libc::c_int) {
            if 128 as libc::c_int / (5 as libc::c_int + 1 as libc::c_int) &
                   1 as libc::c_int != 0 {
                LCDprint(' ' as i32 as uint8_t);
            }
            drawRxChannel((channelIndex + 1 as libc::c_int) as uint8_t,
                          (128 as libc::c_int /
                               (5 as libc::c_int + 1 as libc::c_int) /
                               2 as libc::c_int) as uint8_t);
        }
        channelIndex += 2 as libc::c_int
    };
}
unsafe extern "C" fn showWelcomePage() {
    let mut rowIndex: uint8_t = 1 as libc::c_int as uint8_t;
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"v%s (%s)\x00" as *const u8 as *const libc::c_char,
                b"2.5.0\x00" as *const u8 as *const libc::c_char,
                shortGitRevision);
    let fresh2 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh2);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    let fresh3 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh3);
    i2c_OLED_send_string(bus, targetName);
}
unsafe extern "C" fn showArmedPage() { }
unsafe extern "C" fn showProfilePage() {
    let mut rowIndex: uint8_t = 1 as libc::c_int as uint8_t;
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"Profile: %d\x00" as *const u8 as *const libc::c_char,
                getCurrentPidProfileIndex() as libc::c_int);
    let fresh4 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh4);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    static mut axisTitles: [*const libc::c_char; 3] =
        [b"ROL\x00" as *const u8 as *const libc::c_char,
         b"PIT\x00" as *const u8 as *const libc::c_char,
         b"YAW\x00" as *const u8 as *const libc::c_char];
    let mut pidProfile: *const pidProfile_t = currentPidProfile;
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        tfp_sprintf(lineBuffer.as_mut_ptr(),
                    b"%s P:%3d I:%3d D:%3d\x00" as *const u8 as
                        *const libc::c_char, axisTitles[axis as usize],
                    (*pidProfile).pid[axis as usize].P as libc::c_int,
                    (*pidProfile).pid[axis as usize].I as libc::c_int,
                    (*pidProfile).pid[axis as usize].D as libc::c_int);
        padLineBuffer();
        let fresh5 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh5);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
        axis += 1
    }
    let currentRateProfileIndex: uint8_t =
        getCurrentControlRateProfileIndex();
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"Rate profile: %d\x00" as *const u8 as *const libc::c_char,
                currentRateProfileIndex as libc::c_int);
    let fresh6 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh6);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    let mut controlRateConfig: *const controlRateConfig_t =
        controlRateProfiles(currentRateProfileIndex as libc::c_int);
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"RRr:%d PRR:%d YRR:%d\x00" as *const u8 as
                    *const libc::c_char,
                (*controlRateConfig).rcRates[FD_ROLL as libc::c_int as usize]
                    as libc::c_int,
                (*controlRateConfig).rcRates[FD_PITCH as libc::c_int as usize]
                    as libc::c_int,
                (*controlRateConfig).rcRates[FD_YAW as libc::c_int as usize]
                    as libc::c_int);
    padLineBuffer();
    let fresh7 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh7);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"RE:%d PE:%d YE:%d\x00" as *const u8 as *const libc::c_char,
                (*controlRateConfig).rcExpo[FD_ROLL as libc::c_int as usize]
                    as libc::c_int,
                (*controlRateConfig).rcExpo[FD_PITCH as libc::c_int as usize]
                    as libc::c_int,
                (*controlRateConfig).rcExpo[FD_YAW as libc::c_int as usize] as
                    libc::c_int);
    padLineBuffer();
    let fresh8 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh8);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"RR:%d PR:%d YR:%d\x00" as *const u8 as *const libc::c_char,
                (*controlRateConfig).rates[FD_ROLL as libc::c_int as usize] as
                    libc::c_int,
                (*controlRateConfig).rates[FD_PITCH as libc::c_int as usize]
                    as libc::c_int,
                (*controlRateConfig).rates[FD_YAW as libc::c_int as usize] as
                    libc::c_int);
    padLineBuffer();
    let fresh9 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh9);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
}
unsafe extern "C" fn showGpsPage() {
    if !feature(FEATURE_GPS as libc::c_int as uint32_t) {
        pageState.pageFlags =
            (pageState.pageFlags as libc::c_int |
                 PAGE_STATE_FLAG_FORCE_PAGE_CHANGE as libc::c_int) as uint8_t;
        return
    }
    let mut rowIndex: uint8_t = 1 as libc::c_int as uint8_t;
    static mut gpsTicker: uint8_t = 0 as libc::c_int as uint8_t;
    static mut lastGPSSvInfoReceivedCount: uint32_t =
        0 as libc::c_int as uint32_t;
    if GPS_svInfoReceivedCount != lastGPSSvInfoReceivedCount {
        lastGPSSvInfoReceivedCount = GPS_svInfoReceivedCount;
        gpsTicker = gpsTicker.wrapping_add(1);
        gpsTicker =
            (gpsTicker as
                 libc::c_ulong).wrapping_rem((::core::mem::size_of::<*const libc::c_char>()
                                                  as
                                                  libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_char>()
                                                                                  as
                                                                                  libc::c_ulong))
                as uint8_t
    }
    i2c_OLED_set_xy(bus, 0 as libc::c_int as uint8_t, rowIndex);
    i2c_OLED_send_char(bus,
                       *tickerCharacters.offset(gpsTicker as isize) as
                           libc::c_uchar);
    let fresh10 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_xy(bus,
                    ({
                         let mut _a: libc::c_int = 0 as libc::c_int;
                         let mut _b: uint8_t =
                             ((128 as libc::c_int /
                                   (5 as libc::c_int + 1 as libc::c_int)) as
                                  libc::c_ulong).wrapping_sub((::core::mem::size_of::<[uint8_t; 16]>()
                                                                   as
                                                                   libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                                   as
                                                                                                   libc::c_ulong)).wrapping_div(2
                                                                                                                                    as
                                                                                                                                    libc::c_int
                                                                                                                                    as
                                                                                                                                    libc::c_ulong)
                                 as uint8_t;
                         if _a > _b as libc::c_int {
                             _a
                         } else { _b as libc::c_int }
                     }) as uint8_t, fresh10);
    let mut index: uint32_t = 0;
    index = 0 as libc::c_int as uint32_t;
    while (index as libc::c_ulong) <
              (::core::mem::size_of::<[uint8_t; 16]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                   as libc::c_ulong) &&
              index <
                  (128 as libc::c_int / (5 as libc::c_int + 1 as libc::c_int))
                      as libc::c_uint {
        let mut bargraphOffset: uint8_t =
            (GPS_svinfo_cno[index as usize] as uint16_t as libc::c_int *
                 7 as libc::c_int / (55 as libc::c_int - 1 as libc::c_int)) as
                uint8_t;
        bargraphOffset =
            ({
                 let mut _a: uint8_t = bargraphOffset;
                 let mut _b: libc::c_int =
                     7 as libc::c_int - 1 as libc::c_int;
                 if (_a as libc::c_int) < _b { _a as libc::c_int } else { _b }
             }) as uint8_t;
        i2c_OLED_send_char(bus,
                           (128 as libc::c_int + 32 as libc::c_int +
                                bargraphOffset as libc::c_int) as
                               libc::c_uchar);
        index = index.wrapping_add(1)
    }
    let mut fixChar: libc::c_char =
        if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
            'Y' as i32
        } else { 'N' as i32 } as libc::c_char;
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"Sats: %d Fix: %c\x00" as *const u8 as *const libc::c_char,
                gpsSol.numSat as libc::c_int, fixChar as libc::c_int);
    padLineBuffer();
    let fresh11 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh11);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"La/Lo: %d/%d\x00" as *const u8 as *const libc::c_char,
                gpsSol.llh.lat as libc::c_long / 10000000 as libc::c_long,
                gpsSol.llh.lon as libc::c_long / 10000000 as libc::c_long);
    padLineBuffer();
    let fresh12 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh12);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"Spd: %d\x00" as *const u8 as *const libc::c_char,
                gpsSol.groundSpeed as libc::c_int);
    padHalfLineBuffer();
    i2c_OLED_set_line(bus, rowIndex);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"GC: %d\x00" as *const u8 as *const libc::c_char,
                gpsSol.groundCourse as libc::c_int);
    padHalfLineBuffer();
    let fresh13 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_xy(bus,
                    (128 as libc::c_int /
                         (5 as libc::c_int + 1 as libc::c_int) /
                         2 as libc::c_int) as uint8_t, fresh13);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"RX: %d\x00" as *const u8 as *const libc::c_char,
                GPS_packetCount);
    padHalfLineBuffer();
    i2c_OLED_set_line(bus, rowIndex);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"ERRs: %d\x00" as *const u8 as *const libc::c_char,
                gpsData.errors, gpsData.timeouts);
    padHalfLineBuffer();
    let fresh14 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_xy(bus,
                    (128 as libc::c_int /
                         (5 as libc::c_int + 1 as libc::c_int) /
                         2 as libc::c_int) as uint8_t, fresh14);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"Dt: %d\x00" as *const u8 as *const libc::c_char,
                gpsData.lastMessage.wrapping_sub(gpsData.lastLastMessage));
    padHalfLineBuffer();
    i2c_OLED_set_line(bus, rowIndex);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    tfp_sprintf(lineBuffer.as_mut_ptr(),
                b"TOs: %d\x00" as *const u8 as *const libc::c_char,
                gpsData.timeouts);
    padHalfLineBuffer();
    let fresh15 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_xy(bus,
                    (128 as libc::c_int /
                         (5 as libc::c_int + 1 as libc::c_int) /
                         2 as libc::c_int) as uint8_t, fresh15);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    strncpy(lineBuffer.as_mut_ptr(), gpsPacketLog.as_mut_ptr(),
            21 as libc::c_int as libc::c_ulong);
    padHalfLineBuffer();
    let fresh16 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh16);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
}
unsafe extern "C" fn showBatteryPage() {
    let mut rowIndex: uint8_t = 1 as libc::c_int as uint8_t;
    if (*batteryConfig()).voltageMeterSource as libc::c_uint !=
           VOLTAGE_METER_NONE as libc::c_int as libc::c_uint {
        tfp_sprintf(lineBuffer.as_mut_ptr(),
                    b"Volts: %d.%1d Cells: %d\x00" as *const u8 as
                        *const libc::c_char,
                    getBatteryVoltage() as libc::c_int / 10 as libc::c_int,
                    getBatteryVoltage() as libc::c_int % 10 as libc::c_int,
                    getBatteryCellCount() as libc::c_int);
        padLineBuffer();
        let fresh17 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh17);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
        let mut batteryPercentage: uint8_t =
            calculateBatteryPercentageRemaining();
        let fresh18 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh18);
        drawHorizonalPercentageBar((128 as libc::c_int /
                                        (5 as libc::c_int + 1 as libc::c_int))
                                       as uint8_t, batteryPercentage);
    }
    if (*batteryConfig()).currentMeterSource as libc::c_uint !=
           CURRENT_METER_NONE as libc::c_int as libc::c_uint {
        let mut amperage: int32_t = getAmperage();
        tfp_sprintf(lineBuffer.as_mut_ptr(),
                    b"Amps: %d.%2d mAh: %d\x00" as *const u8 as
                        *const libc::c_char, amperage / 100 as libc::c_int,
                    amperage % 100 as libc::c_int, getMAhDrawn());
        padLineBuffer();
        let fresh19 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh19);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
        let mut capacityPercentage: uint8_t =
            calculateBatteryPercentageRemaining();
        let fresh20 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh20);
        drawHorizonalPercentageBar((128 as libc::c_int /
                                        (5 as libc::c_int + 1 as libc::c_int))
                                       as uint8_t, capacityPercentage);
    };
}
unsafe extern "C" fn showSensorsPage() {
    let mut rowIndex: uint8_t = 1 as libc::c_int as uint8_t;
    static mut format: *const libc::c_char =
        b"%s %5d %5d %5d\x00" as *const u8 as *const libc::c_char;
    let fresh21 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh21);
    i2c_OLED_send_string(bus,
                         b"        X     Y     Z\x00" as *const u8 as
                             *const libc::c_char);
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        tfp_sprintf(lineBuffer.as_mut_ptr(), format,
                    b"ACC\x00" as *const u8 as *const libc::c_char,
                    lrintf(acc.accADC[X as libc::c_int as usize]),
                    lrintf(acc.accADC[Y as libc::c_int as usize]),
                    lrintf(acc.accADC[Z as libc::c_int as usize]));
        padLineBuffer();
        let fresh22 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh22);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    }
    if sensors(SENSOR_GYRO as libc::c_int as uint32_t) {
        tfp_sprintf(lineBuffer.as_mut_ptr(), format,
                    b"GYR\x00" as *const u8 as *const libc::c_char,
                    lrintf(gyro.gyroADCf[X as libc::c_int as usize]),
                    lrintf(gyro.gyroADCf[Y as libc::c_int as usize]),
                    lrintf(gyro.gyroADCf[Z as libc::c_int as usize]));
        padLineBuffer();
        let fresh23 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh23);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    }
    if sensors(SENSOR_MAG as libc::c_int as uint32_t) {
        tfp_sprintf(lineBuffer.as_mut_ptr(), format,
                    b"MAG\x00" as *const u8 as *const libc::c_char,
                    lrintf(mag.magADC[X as libc::c_int as usize]),
                    lrintf(mag.magADC[Y as libc::c_int as usize]),
                    lrintf(mag.magADC[Z as libc::c_int as usize]));
        padLineBuffer();
        let fresh24 = rowIndex;
        rowIndex = rowIndex.wrapping_add(1);
        i2c_OLED_set_line(bus, fresh24);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    }
    tfp_sprintf(lineBuffer.as_mut_ptr(), format,
                b"I&H\x00" as *const u8 as *const libc::c_char,
                attitude.values.roll as libc::c_int,
                attitude.values.pitch as libc::c_int,
                attitude.values.yaw as libc::c_int / 10 as libc::c_int);
    padLineBuffer();
    let fresh25 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh25);
    i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
    /*
    uint8_t length;

    ftoa(EstG.A[X], lineBuffer);
    length = strlen(lineBuffer);
    while (length < HALF_SCREEN_CHARACTER_COLUMN_COUNT) {
        lineBuffer[length++] = ' ';
        lineBuffer[length+1] = 0;
    }
    ftoa(EstG.A[Y], lineBuffer + length);
    padLineBuffer();
    i2c_OLED_set_line(bus, rowIndex++);
    i2c_OLED_send_string(bus, lineBuffer);

    ftoa(EstG.A[Z], lineBuffer);
    length = strlen(lineBuffer);
    while (length < HALF_SCREEN_CHARACTER_COLUMN_COUNT) {
        lineBuffer[length++] = ' ';
        lineBuffer[length+1] = 0;
    }
    ftoa(smallAngle, lineBuffer + length);
    padLineBuffer();
    i2c_OLED_set_line(bus, rowIndex++);
    i2c_OLED_send_string(bus, lineBuffer);
    */
}
unsafe extern "C" fn showTasksPage() {
    let mut rowIndex: uint8_t = 1 as libc::c_int as uint8_t;
    static mut format: *const libc::c_char =
        b"%2d%6d%5d%4d%4d\x00" as *const u8 as *const libc::c_char;
    let fresh26 = rowIndex;
    rowIndex = rowIndex.wrapping_add(1);
    i2c_OLED_set_line(bus, fresh26);
    i2c_OLED_send_string(bus,
                         b"Task max  avg mx% av%\x00" as *const u8 as
                             *const libc::c_char);
    let mut taskInfo: cfTaskInfo_t =
        cfTaskInfo_t{taskName: 0 as *const libc::c_char,
                     subTaskName: 0 as *const libc::c_char,
                     isEnabled: false,
                     staticPriority: 0,
                     desiredPeriod: 0,
                     latestDeltaTime: 0,
                     maxExecutionTime: 0,
                     totalExecutionTime: 0,
                     averageExecutionTime: 0,};
    let mut taskId: cfTaskId_e = TASK_SYSTEM;
    while (taskId as libc::c_uint) < TASK_COUNT as libc::c_int as libc::c_uint
          {
        getTaskInfo(taskId, &mut taskInfo);
        if taskInfo.isEnabled as libc::c_int != 0 &&
               taskId as libc::c_uint !=
                   TASK_SERIAL as libc::c_int as libc::c_uint {
            // don't waste a line of the display showing serial taskInfo
            let taskFrequency: libc::c_int =
                (1000000.0f32 / taskInfo.latestDeltaTime as libc::c_float) as
                    libc::c_int;
            let maxLoad: libc::c_int =
                taskInfo.maxExecutionTime.wrapping_mul(taskFrequency as
                                                           libc::c_uint).wrapping_add(5000
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint).wrapping_div(10000
                                                                                                                         as
                                                                                                                         libc::c_int
                                                                                                                         as
                                                                                                                         libc::c_uint)
                    as libc::c_int;
            let averageLoad: libc::c_int =
                taskInfo.averageExecutionTime.wrapping_mul(taskFrequency as
                                                               libc::c_uint).wrapping_add(5000
                                                                                              as
                                                                                              libc::c_int
                                                                                              as
                                                                                              libc::c_uint).wrapping_div(10000
                                                                                                                             as
                                                                                                                             libc::c_int
                                                                                                                             as
                                                                                                                             libc::c_uint)
                    as libc::c_int;
            tfp_sprintf(lineBuffer.as_mut_ptr(), format,
                        taskId as libc::c_uint, taskInfo.maxExecutionTime,
                        taskInfo.averageExecutionTime, maxLoad, averageLoad);
            padLineBuffer();
            let fresh27 = rowIndex;
            rowIndex = rowIndex.wrapping_add(1);
            i2c_OLED_set_line(bus, fresh27);
            i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
            if rowIndex as libc::c_int >
                   64 as libc::c_int / (7 as libc::c_int + 1 as libc::c_int) {
                break ;
            }
        }
        taskId += 1
    };
}
unsafe extern "C" fn showDebugPage() {
    let mut rowIndex: libc::c_int = 0 as libc::c_int;
    while rowIndex < 4 as libc::c_int {
        tfp_sprintf(lineBuffer.as_mut_ptr(),
                    b"%d = %5d\x00" as *const u8 as *const libc::c_char,
                    rowIndex, debug[rowIndex as usize] as libc::c_int);
        padLineBuffer();
        i2c_OLED_set_line(bus, (rowIndex + 1 as libc::c_int) as uint8_t);
        i2c_OLED_send_string(bus, lineBuffer.as_mut_ptr());
        rowIndex += 1
    };
}
static mut pages: [pageEntry_t; 9] =
    unsafe {
        [{
             let mut init =
                 pageEntry_s{id: PAGE_WELCOME,
                             title:
                                 b"Cleanflight\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showWelcomePage as
                                          unsafe extern "C" fn() -> ()),
                             flags:
                                 ((1 as libc::c_int) << 0 as libc::c_int) as
                                     uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_ARMED,
                             title:
                                 b"ARMED\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showArmedPage as
                                          unsafe extern "C" fn() -> ()),
                             flags:
                                 ((1 as libc::c_int) << 0 as libc::c_int) as
                                     uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_PROFILE,
                             title:
                                 b"PROFILE\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showProfilePage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_GPS,
                             title:
                                 b"GPS\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showGpsPage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_RX,
                             title:
                                 b"RX\x00" as *const u8 as *const libc::c_char
                                     as *mut libc::c_char,
                             drawFn:
                                 Some(showRxPage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_BATTERY,
                             title:
                                 b"BATTERY\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showBatteryPage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_SENSORS,
                             title:
                                 b"SENSORS\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showSensorsPage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_TASKS,
                             title:
                                 b"TASKS\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showTasksPage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         },
         {
             let mut init =
                 pageEntry_s{id: PAGE_DEBUG,
                             title:
                                 b"DEBUG\x00" as *const u8 as
                                     *const libc::c_char as *mut libc::c_char,
                             drawFn:
                                 Some(showDebugPage as
                                          unsafe extern "C" fn() -> ()),
                             flags: 0 as libc::c_int as uint8_t,};
             init
         }]
    };
#[no_mangle]
pub unsafe extern "C" fn dashboardSetPage(mut pageId: pageId_e) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < PAGE_COUNT as libc::c_int {
        let mut candidatePage: *const pageEntry_t =
            &*pages.as_ptr().offset(i as isize) as *const pageEntry_t;
        if (*candidatePage).id as libc::c_uint == pageId as libc::c_uint {
            pageState.page = candidatePage
        }
        i += 1
    }
    pageState.pageFlags =
        (pageState.pageFlags as libc::c_int |
             PAGE_STATE_FLAG_FORCE_PAGE_CHANGE as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn dashboardUpdate(mut currentTimeUs: timeUs_t) {
    static mut previousArmedState: uint8_t = 0 as libc::c_int as uint8_t;
    if displayIsGrabbed(displayPort) { return }
    let updateNow: bool =
        currentTimeUs.wrapping_sub(nextDisplayUpdateAt) as int32_t as
            libc::c_long >= 0 as libc::c_long;
    if !updateNow { return }
    nextDisplayUpdateAt =
        currentTimeUs.wrapping_add((1000 as libc::c_int * 1000 as libc::c_int
                                        / 5 as libc::c_int) as libc::c_uint);
    let mut armedState: bool =
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            1 as libc::c_int
        } else { 0 as libc::c_int } != 0;
    let mut armedStateChanged: bool =
        armedState as libc::c_int != previousArmedState as libc::c_int;
    previousArmedState = armedState as uint8_t;
    if armedState {
        if !armedStateChanged { return }
        dashboardSetPage(PAGE_ARMED);
        pageState.pageChanging = 1 as libc::c_int != 0
    } else {
        if armedStateChanged {
            pageState.pageFlags =
                (pageState.pageFlags as libc::c_int |
                     PAGE_STATE_FLAG_FORCE_PAGE_CHANGE as libc::c_int) as
                    uint8_t
        }
        pageState.pageChanging =
            pageState.pageFlags as libc::c_int &
                PAGE_STATE_FLAG_FORCE_PAGE_CHANGE as libc::c_int != 0 ||
                currentTimeUs.wrapping_sub(pageState.nextPageAt) as int32_t as
                    libc::c_long >= 0 as libc::c_long &&
                    pageState.pageFlags as libc::c_int &
                        PAGE_STATE_FLAG_CYCLE_ENABLED as libc::c_int != 0;
        if pageState.pageChanging as libc::c_int != 0 &&
               pageState.pageFlags as libc::c_int &
                   PAGE_STATE_FLAG_CYCLE_ENABLED as libc::c_int != 0 {
            loop  {
                pageState.cycleIndex = pageState.cycleIndex.wrapping_add(1);
                pageState.cycleIndex =
                    (pageState.cycleIndex as libc::c_int %
                         PAGE_COUNT as libc::c_int) as uint8_t;
                pageState.page =
                    &*pages.as_ptr().offset(pageState.cycleIndex as isize) as
                        *const pageEntry_t;
                if !((*pageState.page).flags as libc::c_int &
                         (1 as libc::c_int) << 0 as libc::c_int != 0) {
                    break ;
                }
            }
        }
    }
    if pageState.pageChanging {
        pageState.pageFlags =
            (pageState.pageFlags as libc::c_int &
                 !(PAGE_STATE_FLAG_FORCE_PAGE_CHANGE as libc::c_int)) as
                uint8_t;
        pageState.nextPageAt =
            currentTimeUs.wrapping_add((1000 as libc::c_int *
                                            1000 as libc::c_int *
                                            5 as libc::c_int) as
                                           libc::c_uint);
        // Some OLED displays do not respond on the first initialisation so refresh the display
        // when the page changes in the hopes the hardware responds.  This also allows the
        // user to power off/on the display or connect it while powered.
        resetDisplay();
        if !dashboardPresent { return }
        handlePageChange();
    }
    if !dashboardPresent { return }
    (*pageState.page).drawFn.expect("non-null function pointer")();
    if !armedState {
        updateFailsafeStatus();
        updateRxStatus();
        updateTicker();
    };
}
#[no_mangle]
pub unsafe extern "C" fn dashboardInit() {
    static mut dashBoardBus: busDevice_t =
        busDevice_t{bustype: BUSTYPE_NONE,
                    busdev_u:
                        C2RustUnnamed{spi:
                                          deviceSpi_s{instance:
                                                          0 as
                                                              *const SPI_TypeDef
                                                              as
                                                              *mut SPI_TypeDef,
                                                      handle:
                                                          0 as
                                                              *const SPI_HandleTypeDef
                                                              as
                                                              *mut SPI_HandleTypeDef,
                                                      csnPin:
                                                          0 as
                                                              *const libc::c_void
                                                              as
                                                              *mut libc::c_void,},},};
    dashBoardBus.busdev_u.i2c.device =
        ((*dashboardConfig()).device as libc::c_int - 1 as libc::c_int) as
            I2CDevice;
    dashBoardBus.busdev_u.i2c.address = (*dashboardConfig()).address;
    bus = &mut dashBoardBus;
    delay(200 as libc::c_int as timeMs_t);
    resetDisplay();
    delay(200 as libc::c_int as timeMs_t);
    displayPort = displayPortOledInit(bus as *mut libc::c_void);
    if dashboardPresent { cmsDisplayPortRegister(displayPort); }
    memset(&mut pageState as *mut pageState_t as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<pageState_t>() as libc::c_ulong);
    dashboardSetPage(PAGE_WELCOME);
    let mut now: uint32_t = micros();
    dashboardUpdate(now);
    dashboardSetNextPageChangeAt(now.wrapping_add((1000 as libc::c_int *
                                                       1000 as libc::c_int *
                                                       5 as libc::c_int) as
                                                      libc::c_uint));
}
#[no_mangle]
pub unsafe extern "C" fn dashboardShowFixedPage(mut pageId: pageId_e) {
    dashboardSetPage(pageId);
    dashboardDisablePageCycling();
}
#[no_mangle]
pub unsafe extern "C" fn dashboardSetNextPageChangeAt(mut futureMicros:
                                                          timeUs_t) {
    pageState.nextPageAt = futureMicros;
}
#[no_mangle]
pub unsafe extern "C" fn dashboardEnablePageCycling() {
    pageState.pageFlags =
        (pageState.pageFlags as libc::c_int |
             PAGE_STATE_FLAG_CYCLE_ENABLED as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn dashboardResetPageCycling() {
    pageState.cycleIndex =
        (PAGE_COUNT as libc::c_int - 1 as libc::c_int) as uint8_t;
    // start at first page
}
#[no_mangle]
pub unsafe extern "C" fn dashboardDisablePageCycling() {
    pageState.pageFlags =
        (pageState.pageFlags as libc::c_int &
             !(PAGE_STATE_FLAG_CYCLE_ENABLED as libc::c_int)) as uint8_t;
}
// USE_DASHBOARD
