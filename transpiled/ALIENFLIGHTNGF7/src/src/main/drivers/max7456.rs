use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
    #[no_mangle]
    fn spiBusSetInstance(bus: *mut busDevice_t, instance: *mut SPI_TypeDef);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOIsFreeOrPreinit(io: IO_t) -> bool;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed = 38;
pub const DEBUG_RTH: C2RustUnnamed = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed = 36;
pub const DEBUG_USB: C2RustUnnamed = 35;
pub const DEBUG_CURRENT: C2RustUnnamed = 34;
pub const DEBUG_SDIO: C2RustUnnamed = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed = 28;
pub const DEBUG_FPORT: C2RustUnnamed = 27;
pub const DEBUG_SBUS: C2RustUnnamed = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed = 16;
pub const DEBUG_FFT: C2RustUnnamed = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed = 12;
pub const DEBUG_STACK: C2RustUnnamed = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed = 3;
pub const DEBUG_BATTERY: C2RustUnnamed = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed = 1;
pub const DEBUG_NONE: C2RustUnnamed = 0;
/* !< SPI Error code                           */
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct max7456Config_s {
    pub clockConfig: uint8_t,
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
}
pub type max7456Config_t = max7456Config_s;
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
// Video Character Display parameters
pub type VIDEO_SYSTEMS = libc::c_uint;
pub const VIDEO_SYSTEM_NTSC: VIDEO_SYSTEMS = 2;
pub const VIDEO_SYSTEM_PAL: VIDEO_SYSTEMS = 1;
pub const VIDEO_SYSTEM_AUTO: VIDEO_SYSTEMS = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcdProfile_s {
    pub video_system: uint8_t,
    pub h_offset: int8_t,
    pub v_offset: int8_t,
}
pub type vcdProfile_t = vcdProfile_s;
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
    pub busdev_u: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
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
pub type C2RustUnnamed_1 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_1 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_1 = 8;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_1 = 16;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_1 = 256;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_1 = 256;
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
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
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
// millisecond time
pub type timeMs_t = uint32_t;
// XXX Should be related to VIDEO_BUFFER_CHARS_*?
// On shared SPI buss we want to change clock for OSD chip and restore for other devices.
#[no_mangle]
pub static mut max7456BusDevice: busDevice_t =
    busDevice_t{bustype: BUSTYPE_NONE,
                busdev_u:
                    C2RustUnnamed_0{spi:
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
#[no_mangle]
pub static mut busdev: *mut busDevice_t =
    unsafe { &max7456BusDevice as *const busDevice_t as *mut busDevice_t };
static mut max7456SpiClock: uint16_t =
    SPI_CLOCK_STANDARD as libc::c_int as uint16_t;
#[no_mangle]
pub static mut maxScreenSize: uint16_t = 480 as libc::c_int as uint16_t;
// We write everything in screenBuffer and then compare
// screenBuffer with shadowBuffer to upgrade only changed chars.
// This solution is faster then redrawing entire screen.
static mut screenBuffer: [uint8_t; 520] = [0; 520];
// For faster writes we use memcpy so we need some space to don't overwrite buffer
static mut shadowBuffer: [uint8_t; 480] = [0; 480];
static mut spiBuff: [uint8_t; 600] = [0; 600];
static mut videoSignalCfg: uint8_t = 0;
static mut videoSignalReg: uint8_t = 0x8 as libc::c_int as uint8_t;
// OSD_ENABLE required to trigger first ReInit
static mut displayMemoryModeReg: uint8_t = 0 as libc::c_int as uint8_t;
static mut hosRegValue: uint8_t = 0;
// HOS (Horizontal offset register) value
static mut vosRegValue: uint8_t = 0;
// VOS (Vertical offset register) value
static mut max7456Lock: bool = 0 as libc::c_int != 0;
static mut fontIsLoading: bool = 0 as libc::c_int != 0;
static mut max7456DeviceType: uint8_t = 0;
unsafe extern "C" fn max7456Send(mut add: uint8_t, mut data: uint8_t)
 -> uint8_t {
    spiTransferByte((*busdev).busdev_u.spi.instance, add);
    return spiTransferByte((*busdev).busdev_u.spi.instance, data);
}
#[no_mangle]
pub unsafe extern "C" fn max7456GetRowsCount() -> uint8_t {
    return if videoSignalReg as libc::c_int & 0x40 as libc::c_int != 0 {
               16 as libc::c_int
           } else { 13 as libc::c_int } as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn max7456ReInit() {
    let mut srdata: uint8_t = 0 as libc::c_int as uint8_t;
    static mut firstInit: bool = 1 as libc::c_int != 0;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    match videoSignalCfg as libc::c_int {
        1 => {
            videoSignalReg =
                (0x40 as libc::c_int | 0x8 as libc::c_int) as uint8_t
        }
        2 => {
            videoSignalReg =
                (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
        }
        0 => {
            srdata =
                max7456Send(0xa0 as libc::c_int as uint8_t,
                            0 as libc::c_int as uint8_t);
            if srdata as libc::c_int & 0x4 as libc::c_int == 0 &&
                   srdata as libc::c_int & 0x2 as libc::c_int != 0 {
                videoSignalReg =
                    (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            } else if srdata as libc::c_int & 0x4 as libc::c_int == 0 &&
                          srdata as libc::c_int & 0x1 as libc::c_int != 0 {
                videoSignalReg =
                    (0x40 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            } else {
                // No valid input signal, fallback to default (XXX NTSC for now)
                videoSignalReg =
                    (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            }
        }
        _ => { }
    } // NTSC
    if videoSignalReg as libc::c_int & 0x40 as libc::c_int != 0 {
        //PAL
        maxScreenSize = 480 as libc::c_int as uint16_t
    } else { maxScreenSize = 390 as libc::c_int as uint16_t }
    // Set all rows to same charactor black/white level
    max7456Brightness(0 as libc::c_int as uint8_t,
                      2 as libc::c_int as uint8_t);
    // Re-enable MAX7456 (last function call disables it)
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    // Make sure the Max7456 is enabled
    max7456Send(0 as libc::c_int as uint8_t, videoSignalReg);
    max7456Send(0x2 as libc::c_int as uint8_t, hosRegValue);
    max7456Send(0x3 as libc::c_int as uint8_t, vosRegValue);
    max7456Send(0x4 as libc::c_int as uint8_t,
                (displayMemoryModeReg as libc::c_int | 0x4 as libc::c_int) as
                    uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    // Clear shadow to force redraw all screen in non-dma mode.
    memset(shadowBuffer.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           maxScreenSize as libc::c_ulong);
    if firstInit {
        max7456DrawScreenSlow();
        firstInit = 0 as libc::c_int != 0
    };
}
// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)
#[no_mangle]
pub unsafe extern "C" fn max7456Init(mut max7456Config:
                                         *const max7456Config_t,
                                     mut pVcdProfile: *const vcdProfile_t,
                                     mut cpuOverclock: bool) -> bool {
    max7456HardwareReset();
    if (*max7456Config).csTag == 0 { return 0 as libc::c_int != 0 }
    (*busdev).busdev_u.spi.csnPin = IOGetByTag((*max7456Config).csTag);
    if !IOIsFreeOrPreinit((*busdev).busdev_u.spi.csnPin) {
        return 0 as libc::c_int != 0
    }
    IOInit((*busdev).busdev_u.spi.csnPin, OWNER_OSD_CS,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                 (0x1 as libc::c_uint |
                      (0x3 as libc::c_uint) << 2 as libc::c_int |
                      (0 as libc::c_uint) << 5 as libc::c_int) as ioConfig_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiBusSetInstance(busdev,
                      spiInstanceByDevice(((*max7456Config).spiDevice as
                                               libc::c_int - 1 as libc::c_int)
                                              as SPIDevice));
    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // Do this at half the speed for safety.
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  (SPI_CLOCK_STANDARD as libc::c_int * 2 as libc::c_int) as
                      uint16_t); // CA[8] bit
    max7456Send(0xa as libc::c_int as uint8_t,
                ((1 as libc::c_int) << 6 as libc::c_int) as uint8_t);
    if max7456Send((0xa as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                   0xff as libc::c_int as uint8_t) as libc::c_int &
           (1 as libc::c_int) << 6 as libc::c_int != 0 {
        max7456DeviceType = 1 as libc::c_int as uint8_t
    } else { max7456DeviceType = 0 as libc::c_int as uint8_t }
    // Determine SPI clock divisor based on config and the device type.
    match (*max7456Config).clockConfig as libc::c_int {
        0 => {
            max7456SpiClock =
                (SPI_CLOCK_STANDARD as libc::c_int * 2 as libc::c_int) as
                    uint16_t
        }
        1 => {
            max7456SpiClock =
                if cpuOverclock as libc::c_int != 0 &&
                       max7456DeviceType as libc::c_int == 0 as libc::c_int {
                    (SPI_CLOCK_STANDARD as libc::c_int) * 2 as libc::c_int
                } else { SPI_CLOCK_STANDARD as libc::c_int } as uint16_t
        }
        2 => {
            max7456SpiClock = SPI_CLOCK_STANDARD as libc::c_int as uint16_t
        }
        _ => { }
    }
    if debugMode as libc::c_int == DEBUG_MAX7456_SPICLOCK as libc::c_int {
        debug[0 as libc::c_int as usize] = cpuOverclock as int16_t
    }
    if debugMode as libc::c_int == DEBUG_MAX7456_SPICLOCK as libc::c_int {
        debug[1 as libc::c_int as usize] = max7456DeviceType as int16_t
    }
    if debugMode as libc::c_int == DEBUG_MAX7456_SPICLOCK as libc::c_int {
        debug[2 as libc::c_int as usize] = max7456SpiClock as int16_t
    }
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    // force soft reset on Max7456
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    max7456Send(0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    // Setup values to write to registers
    videoSignalCfg = (*pVcdProfile).video_system;
    hosRegValue =
        (32 as libc::c_int - (*pVcdProfile).h_offset as libc::c_int) as
            uint8_t;
    vosRegValue =
        (16 as libc::c_int - (*pVcdProfile).v_offset as libc::c_int) as
            uint8_t;
    // Real init will be made later when driver detect idle.
    return 1 as libc::c_int != 0;
}
/* *
 * Sets inversion of black and white pixels.
 */
#[no_mangle]
pub unsafe extern "C" fn max7456Invert(mut invert: bool) {
    if invert {
        displayMemoryModeReg =
            (displayMemoryModeReg as libc::c_int | 0x8 as libc::c_int) as
                uint8_t
    } else {
        displayMemoryModeReg =
            (displayMemoryModeReg as libc::c_int & !(0x8 as libc::c_int)) as
                uint8_t
    }
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    max7456Send(0x4 as libc::c_int as uint8_t, displayMemoryModeReg);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
}
/* *
 * Sets the brighness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
#[no_mangle]
pub unsafe extern "C" fn max7456Brightness(mut black: uint8_t,
                                           mut white: uint8_t) {
    let reg: uint8_t =
        ((black as libc::c_int) << 2 as libc::c_int |
             3 as libc::c_int - white as libc::c_int) as uint8_t;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    let mut i: libc::c_int = 0x10 as libc::c_int;
    while i <= 0x1f as libc::c_int { max7456Send(i as uint8_t, reg); i += 1 }
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
}
//just fill with spaces with some tricks
#[no_mangle]
pub unsafe extern "C" fn max7456ClearScreen() {
    memset(screenBuffer.as_mut_ptr() as *mut libc::c_void,
           0x20 as libc::c_int, 480 as libc::c_int as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn max7456GetScreenBuffer() -> *mut uint8_t {
    return screenBuffer.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn max7456WriteChar(mut x: uint8_t, mut y: uint8_t,
                                          mut c: uint8_t) {
    screenBuffer[(y as libc::c_int * 30 as libc::c_int + x as libc::c_int) as
                     usize] = c;
}
#[no_mangle]
pub unsafe extern "C" fn max7456Write(mut x: uint8_t, mut y: uint8_t,
                                      mut buff: *const libc::c_char) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while *buff.offset(i as isize) != 0 {
        if x as libc::c_int + i < 30 as libc::c_int {
            // Do not write over screen
            screenBuffer[(y as libc::c_int * 30 as libc::c_int +
                              x as libc::c_int + i) as usize] =
                *buff.offset(i as isize) as uint8_t
        }
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456DmaInProgress() -> bool {
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456BuffersSynced() -> bool {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < maxScreenSize as libc::c_int {
        if screenBuffer[i as usize] as libc::c_int !=
               shadowBuffer[i as usize] as libc::c_int {
            return 0 as libc::c_int != 0
        }
        i += 1
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456ReInitIfRequired() {
    static mut lastSigCheckMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut videoDetectTimeMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut reInitCount: uint16_t = 0 as libc::c_int as uint16_t;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    let stallCheck: uint8_t =
        max7456Send((0 as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                    0 as libc::c_int as uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    let nowMs: timeMs_t = millis();
    if stallCheck as libc::c_int != videoSignalReg as libc::c_int {
        max7456ReInit();
    } else if videoSignalCfg as libc::c_int ==
                  VIDEO_SYSTEM_AUTO as libc::c_int &&
                  nowMs.wrapping_sub(lastSigCheckMs) >
                      1000 as libc::c_int as libc::c_uint {
        // Adjust output format based on the current input format.
        spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
        IOLo((*busdev).busdev_u.spi.csnPin);
        let videoSense: uint8_t =
            max7456Send(0xa0 as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
        IOHi((*busdev).busdev_u.spi.csnPin);
        spiSetDivisor((*busdev).busdev_u.spi.instance,
                      SPI_CLOCK_FAST as libc::c_int as uint16_t);
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[0 as libc::c_int as usize] =
                (videoSignalReg as libc::c_int & 0x40 as libc::c_int) as
                    int16_t
        }
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[1 as libc::c_int as usize] =
                (videoSense as libc::c_int & 0x7 as libc::c_int) as int16_t
        }
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[3 as libc::c_int as usize] =
                max7456GetRowsCount() as int16_t
        }
        if videoSense as libc::c_int & 0x4 as libc::c_int != 0 {
            videoDetectTimeMs = 0 as libc::c_int as uint32_t
        } else if videoSense as libc::c_int & 0x4 as libc::c_int == 0 &&
                      videoSense as libc::c_int & 0x1 as libc::c_int != 0 &&
                      videoSignalReg as libc::c_int & 0x40 as libc::c_int ==
                          0 as libc::c_int ||
                      videoSense as libc::c_int & 0x4 as libc::c_int == 0 &&
                          videoSense as libc::c_int & 0x1 as libc::c_int == 0
                          &&
                          videoSignalReg as libc::c_int & 0x40 as libc::c_int
                              == 0x40 as libc::c_int {
            if videoDetectTimeMs != 0 {
                if millis().wrapping_sub(videoDetectTimeMs) >
                       100 as libc::c_int as libc::c_uint {
                    max7456ReInit();
                    if debugMode as libc::c_int ==
                           DEBUG_MAX7456_SIGNAL as libc::c_int {
                        reInitCount = reInitCount.wrapping_add(1);
                        debug[2 as libc::c_int as usize] =
                            reInitCount as int16_t
                    }
                }
            } else {
                // Wait for signal to stabilize
                videoDetectTimeMs = millis()
            }
        }
        lastSigCheckMs = nowMs
    };
    //------------   end of (re)init-------------------------------------
}
#[no_mangle]
pub unsafe extern "C" fn max7456DrawScreen() {
    static mut pos: uint16_t = 0 as libc::c_int as uint16_t;
    if !max7456Lock && !fontIsLoading {
        // (Re)Initialize MAX7456 at startup or stall is detected.
        max7456Lock = 1 as libc::c_int != 0;
        max7456ReInitIfRequired();
        let mut buff_len: libc::c_int = 0 as libc::c_int;
        let mut k: libc::c_int = 0 as libc::c_int;
        while k < 100 as libc::c_int {
            if screenBuffer[pos as usize] as libc::c_int !=
                   shadowBuffer[pos as usize] as libc::c_int {
                let fresh0 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh0 as usize] = 0x5 as libc::c_int as uint8_t;
                let fresh1 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh1 as usize] =
                    (pos as libc::c_int >> 8 as libc::c_int) as uint8_t;
                let fresh2 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh2 as usize] = 0x6 as libc::c_int as uint8_t;
                let fresh3 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh3 as usize] =
                    (pos as libc::c_int & 0xff as libc::c_int) as uint8_t;
                let fresh4 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh4 as usize] = 0x7 as libc::c_int as uint8_t;
                let fresh5 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh5 as usize] = screenBuffer[pos as usize];
                shadowBuffer[pos as usize] = screenBuffer[pos as usize]
            }
            pos = pos.wrapping_add(1);
            if pos as libc::c_int >= maxScreenSize as libc::c_int {
                pos = 0 as libc::c_int as uint16_t;
                break ;
            } else { k += 1 }
        }
        if buff_len != 0 {
            spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
            IOLo((*busdev).busdev_u.spi.csnPin);
            spiTransfer((*busdev).busdev_u.spi.instance, spiBuff.as_mut_ptr(),
                        0 as *mut uint8_t, buff_len);
            IOHi((*busdev).busdev_u.spi.csnPin);
            spiSetDivisor((*busdev).busdev_u.spi.instance,
                          SPI_CLOCK_FAST as libc::c_int as uint16_t);
            // MAX7456_DMA_CHANNEL_TX
        }
        max7456Lock = 0 as libc::c_int != 0
    };
}
unsafe extern "C" fn max7456DrawScreenSlow() {
    let mut escapeCharFound: bool = false;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    // Enable auto-increment mode and update every character in the screenBuffer.
    // The "escape" character 0xFF must be skipped as it causes the MAX7456 to exit auto-increment mode.
    max7456Send(0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    max7456Send(0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    max7456Send(0x4 as libc::c_int as uint8_t,
                (displayMemoryModeReg as libc::c_int | 1 as libc::c_int) as
                    uint8_t);
    let mut xx: libc::c_int = 0 as libc::c_int;
    while xx < maxScreenSize as libc::c_int {
        if screenBuffer[xx as usize] as libc::c_int == 0xff as libc::c_int {
            escapeCharFound = 1 as libc::c_int != 0;
            max7456Send(0x7 as libc::c_int as uint8_t, ' ' as i32 as uint8_t);
            // replace the 0xFF character with a blank in the first pass to avoid terminating auto-increment
        } else {
            max7456Send(0x7 as libc::c_int as uint8_t,
                        screenBuffer[xx as usize]);
        }
        shadowBuffer[xx as usize] = screenBuffer[xx as usize];
        xx += 1
    }
    max7456Send(0x7 as libc::c_int as uint8_t,
                0xff as libc::c_int as uint8_t);
    max7456Send(0x4 as libc::c_int as uint8_t, displayMemoryModeReg);
    // If we found any of the "escape" character 0xFF, then make a second pass
    // to update them with direct addressing
    if escapeCharFound {
        let mut xx_0: libc::c_int = 0 as libc::c_int;
        while xx_0 < maxScreenSize as libc::c_int {
            if screenBuffer[xx_0 as usize] as libc::c_int ==
                   0xff as libc::c_int {
                max7456Send(0x5 as libc::c_int as uint8_t,
                            (xx_0 >> 8 as libc::c_int) as uint8_t);
                max7456Send(0x6 as libc::c_int as uint8_t,
                            (xx_0 & 0xff as libc::c_int) as uint8_t);
                max7456Send(0x7 as libc::c_int as uint8_t,
                            0xff as libc::c_int as uint8_t);
            }
            xx_0 += 1
        }
    }
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
}
// should not be used when armed
#[no_mangle]
pub unsafe extern "C" fn max7456RefreshAll() {
    if !max7456Lock {
        max7456Lock = 1 as libc::c_int != 0;
        max7456ReInitIfRequired();
        max7456DrawScreenSlow();
        max7456Lock = 0 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456WriteNvm(mut char_address: uint8_t,
                                         mut font_data: *const uint8_t) {
    while max7456Lock { }
    max7456Lock = 1 as libc::c_int != 0;
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    IOLo((*busdev).busdev_u.spi.csnPin);
    // disable display
    fontIsLoading = 1 as libc::c_int != 0; // set start address high
    max7456Send(0 as libc::c_int as uint8_t,
                0 as libc::c_int as uint8_t); //set start address low
    max7456Send(0x9 as libc::c_int as uint8_t, char_address);
    let mut x: libc::c_int = 0 as libc::c_int;
    while x < 54 as libc::c_int {
        max7456Send(0xa as libc::c_int as uint8_t, x as uint8_t);
        max7456Send(0xb as libc::c_int as uint8_t,
                    *font_data.offset(x as isize));
        ledToggle(0 as libc::c_int);
        x += 1
    }
    // Transfer 54 bytes from shadow ram to NVM
    max7456Send(0x8 as libc::c_int as uint8_t,
                0xa0 as libc::c_int as uint8_t);
    // Wait until bit 5 in the status register returns to 0 (12ms)
    while max7456Send(0xa0 as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t) as libc::c_int &
              0x20 as libc::c_int != 0 as libc::c_int {
    }
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  SPI_CLOCK_FAST as libc::c_int as uint16_t);
    max7456Lock = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456HardwareReset() { }
// USE_MAX7456
