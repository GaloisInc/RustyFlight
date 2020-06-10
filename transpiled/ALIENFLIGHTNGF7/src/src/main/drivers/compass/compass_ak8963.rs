use ::libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn spiPreinitCsByIO(io: IO_t);
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiBusWriteRegister(bus: *const busDevice_t, reg: uint8_t,
                           data: uint8_t) -> bool;
    #[no_mangle]
    fn spiBusReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                                data: *mut uint8_t, length: uint8_t) -> bool;
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
    // preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
    // expand pinid to to ioTag_t
    //speed is packed inside modebits 5 and 2,
    // declare available IO pins. Available pins are specified per target
    // unimplemented
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOHi(io: IO_t);
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
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
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
pub type C2RustUnnamed = libc::c_uint;
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
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
// Slave I2C on SPI master
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct magDev_s {
    pub init: sensorMagInitFuncPtr,
    pub read: sensorMagReadFuncPtr,
    pub exti: extiCallbackRec_t,
    pub busdev: busDevice_t,
    pub magAlign: sensor_align_e,
    pub magIntExtiTag: ioTag_t,
    pub magGain: [int16_t; 3],
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
pub type extiCallbackRec_t = extiCallbackRec_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
pub type sensorMagReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut magDev_s, _: *mut int16_t) -> bool>;
pub type sensorMagInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut magDev_s) -> bool>;
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
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
pub type magDev_t = magDev_s;
pub type ak8963ReadState_e = libc::c_uint;
pub const WAITING_FOR_DATA: ak8963ReadState_e = 2;
pub const WAITING_FOR_STATUS: ak8963ReadState_e = 1;
pub const CHECK_STATUS: ak8963ReadState_e = 0;
pub type queuedReadState_t = queuedReadState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct queuedReadState_s {
    pub waiting: bool,
    pub len: uint8_t,
    pub readStartedAt: uint32_t,
}
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 28;
pub const TASK_NONE: cfTaskId_e = 27;
pub const TASK_COUNT: cfTaskId_e = 27;
pub const TASK_PINIOBOX: cfTaskId_e = 26;
pub const TASK_ADC_INTERNAL: cfTaskId_e = 25;
pub const TASK_RCDEVICE: cfTaskId_e = 24;
pub const TASK_CAMCTRL: cfTaskId_e = 23;
pub const TASK_VTXCTRL: cfTaskId_e = 22;
pub const TASK_CMS: cfTaskId_e = 21;
pub const TASK_ESC_SENSOR: cfTaskId_e = 20;
pub const TASK_OSD: cfTaskId_e = 19;
pub const TASK_LEDSTRIP: cfTaskId_e = 18;
pub const TASK_TELEMETRY: cfTaskId_e = 17;
pub const TASK_DASHBOARD: cfTaskId_e = 16;
pub const TASK_ALTITUDE: cfTaskId_e = 15;
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
#[inline(always)]
unsafe extern "C" fn __disable_irq() {
    asm!("cpsid i" : : : "memory" : "volatile");
}
#[inline(always)]
unsafe extern "C" fn __enable_irq() {
    asm!("cpsie i" : : : "memory" : "volatile");
}
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
unsafe extern "C" fn ak8963SpiWriteRegisterDelay(mut bus: *const busDevice_t,
                                                 mut reg: uint8_t,
                                                 mut data: uint8_t) -> bool {
    spiBusWriteRegister(bus, reg, data);
    delayMicroseconds(10 as libc::c_int as timeUs_t);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963SlaveReadRegisterBuffer(mut slavedev:
                                                       *const busDevice_t,
                                                   mut reg: uint8_t,
                                                   mut buf: *mut uint8_t,
                                                   mut len: uint8_t) -> bool {
    let mut bus: *const busDevice_t = (*slavedev).busdev_u.mpuSlave.master;
    // initialize function
    // read 3 axis data function
    // time read was queued in micros.
    ak8963SpiWriteRegisterDelay(bus, 0x25 as libc::c_int as uint8_t,
                                ((*slavedev).busdev_u.mpuSlave.address as
                                     libc::c_int | 0x80 as libc::c_int) as
                                    uint8_t); // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, 0x26 as libc::c_int as uint8_t,
                                reg); // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, 0x27 as libc::c_int as uint8_t,
                                (len as libc::c_int & 0xf as libc::c_int |
                                     0x80 as libc::c_int) as
                                    uint8_t); // read number of bytes
    delay(4 as libc::c_int as timeMs_t); // read I2C
    __disable_irq(); // set I2C slave address for write
    let mut ack: bool =
        spiBusReadRegisterBuffer(bus, 0x49 as libc::c_int as uint8_t, buf,
                                 len); // set I2C slave register
    __enable_irq(); // set I2C sLave value
    return ack; // write 1 byte
}
unsafe extern "C" fn ak8963SlaveWriteRegister(mut slavedev:
                                                  *const busDevice_t,
                                              mut reg: uint8_t,
                                              mut data: uint8_t) -> bool {
    let mut bus: *const busDevice_t =
        (*slavedev).busdev_u.mpuSlave.master; // set I2C slave address for read
    ak8963SpiWriteRegisterDelay(bus, 0x25 as libc::c_int as uint8_t,
                                (*slavedev).busdev_u.mpuSlave.address); // set I2C slave register
    ak8963SpiWriteRegisterDelay(bus, 0x26 as libc::c_int as uint8_t,
                                reg); // read number of bytes
    ak8963SpiWriteRegisterDelay(bus, 0x63 as libc::c_int as uint8_t,
                                data); // read I2C buffer
    ak8963SpiWriteRegisterDelay(bus, 0x27 as libc::c_int as uint8_t,
                                (1 as libc::c_int & 0xf as libc::c_int |
                                     0x80 as libc::c_int) as uint8_t);
    return 1 as libc::c_int != 0;
}
static mut queuedRead: queuedReadState_t =
    {
        let mut init =
            queuedReadState_s{waiting: 0 as libc::c_int != 0,
                              len: 0 as libc::c_int as uint8_t,
                              readStartedAt: 0 as libc::c_int as uint32_t,};
        init
    };
unsafe extern "C" fn ak8963SlaveStartRead(mut slavedev: *const busDevice_t,
                                          mut reg: uint8_t, mut len: uint8_t)
 -> bool {
    if queuedRead.waiting { return 0 as libc::c_int != 0 }
    let mut bus: *const busDevice_t = (*slavedev).busdev_u.mpuSlave.master;
    queuedRead.len = len;
    ak8963SpiWriteRegisterDelay(bus, 0x25 as libc::c_int as uint8_t,
                                ((*slavedev).busdev_u.mpuSlave.address as
                                     libc::c_int | 0x80 as libc::c_int) as
                                    uint8_t);
    ak8963SpiWriteRegisterDelay(bus, 0x26 as libc::c_int as uint8_t, reg);
    ak8963SpiWriteRegisterDelay(bus, 0x27 as libc::c_int as uint8_t,
                                (len as libc::c_int & 0xf as libc::c_int |
                                     0x80 as libc::c_int) as uint8_t);
    queuedRead.readStartedAt = micros();
    queuedRead.waiting = 1 as libc::c_int != 0;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963SlaveQueuedReadTimeRemaining() -> uint32_t {
    if !queuedRead.waiting { return 0 as libc::c_int as uint32_t }
    let mut timeSinceStarted: int32_t =
        micros().wrapping_sub(queuedRead.readStartedAt) as int32_t;
    let mut timeRemaining: int32_t = 8000 as libc::c_int - timeSinceStarted;
    if timeRemaining < 0 as libc::c_int {
        return 0 as libc::c_int as uint32_t
    }
    return timeRemaining as uint32_t;
}
unsafe extern "C" fn ak8963SlaveCompleteRead(mut slavedev: *const busDevice_t,
                                             mut buf: *mut uint8_t) -> bool {
    let mut timeRemaining: uint32_t = ak8963SlaveQueuedReadTimeRemaining();
    let mut bus: *const busDevice_t = (*slavedev).busdev_u.mpuSlave.master;
    if timeRemaining > 0 as libc::c_int as libc::c_uint {
        delayMicroseconds(timeRemaining);
    }
    queuedRead.waiting = 0 as libc::c_int != 0;
    spiBusReadRegisterBuffer(bus, 0x49 as libc::c_int as uint8_t, buf,
                             queuedRead.len);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963SlaveReadData(mut busdev: *const busDevice_t,
                                         mut buf: *mut uint8_t) -> bool {
    static mut state: ak8963ReadState_e = CHECK_STATUS;
    let mut ack: bool = 0 as libc::c_int != 0;
    // we currently need a different approach for the MPU9250 connected via SPI.
    // we cannot use the ak8963SlaveReadRegisterBuffer() method for SPI, it is to slow and blocks for far too long.
    let mut retry: bool = 1 as libc::c_int != 0;
    loop  {
        match state as libc::c_uint {
            0 => {
                ak8963SlaveStartRead(busdev, 0x2 as libc::c_int as uint8_t,
                                     1 as libc::c_int as uint8_t);
                state = WAITING_FOR_STATUS;
                return 0 as libc::c_int != 0
            }
            1 => {
                let mut timeRemaining: uint32_t =
                    ak8963SlaveQueuedReadTimeRemaining();
                if timeRemaining != 0 { return 0 as libc::c_int != 0 }
                ack =
                    ak8963SlaveCompleteRead(busdev,
                                            &mut *buf.offset(0 as libc::c_int
                                                                 as isize));
                let mut status: uint8_t =
                    *buf.offset(0 as libc::c_int as isize);
                if !ack ||
                       status as libc::c_int & 0x1 as libc::c_int ==
                           0 as libc::c_int {
                    // too early. queue the status read again
                    state = CHECK_STATUS;
                    if retry {
                        retry = 0 as libc::c_int != 0
                    } else { return 0 as libc::c_int != 0 }
                } else {
                    // read the 6 bytes of data and the status2 register
                    ak8963SlaveStartRead(busdev,
                                         0x3 as libc::c_int as uint8_t,
                                         7 as libc::c_int as
                                             uint8_t); // start reading again    uint8_t status2 = buf[6];
                    state =
                        WAITING_FOR_DATA; // power down before entering fuse mode
                    return 0 as libc::c_int != 0
                }
            }
            2 => {
                let mut timeRemaining_0: uint32_t =
                    ak8963SlaveQueuedReadTimeRemaining(); // Enter Fuse ROM access mode
                if timeRemaining_0 != 0 {
                    return 0 as libc::c_int != 0
                } // Read the x-, y-, and z-axis calibration values
                ack =
                    ak8963SlaveCompleteRead(busdev,
                                            &mut *buf.offset(0 as libc::c_int
                                                                 as
                                                                 isize)); // power down after reading.
                state = CHECK_STATUS;
                break ;
            }
            _ => { break ; }
        }
    }
    return ack;
}
unsafe extern "C" fn ak8963ReadRegisterBuffer(mut busdev: *const busDevice_t,
                                              mut reg: uint8_t,
                                              mut buf: *mut uint8_t,
                                              mut len: uint8_t) -> bool {
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_MPU_SLAVE as libc::c_int as libc::c_uint {
        return ak8963SlaveReadRegisterBuffer(busdev, reg, buf, len)
    }
    return busReadRegisterBuffer(busdev, reg, buf, len);
}
unsafe extern "C" fn ak8963WriteRegister(mut busdev: *const busDevice_t,
                                         mut reg: uint8_t, mut data: uint8_t)
 -> bool {
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_MPU_SLAVE as libc::c_int as libc::c_uint {
        return ak8963SlaveWriteRegister(busdev, reg, data)
    }
    return busWriteRegister(busdev, reg, data);
}
unsafe extern "C" fn ak8963DirectReadData(mut busdev: *const busDevice_t,
                                          mut buf: *mut uint8_t) -> bool {
    let mut status: uint8_t = 0;
    let mut ack: bool =
        ak8963ReadRegisterBuffer(busdev, 0x2 as libc::c_int as uint8_t,
                                 &mut status, 1 as libc::c_int as uint8_t);
    if !ack || status as libc::c_int & 0x1 as libc::c_int == 0 as libc::c_int
       {
        return 0 as libc::c_int != 0
    }
    return ak8963ReadRegisterBuffer(busdev, 0x3 as libc::c_int as uint8_t,
                                    buf, 7 as libc::c_int as uint8_t);
}
unsafe extern "C" fn parseMag(mut raw: *mut uint8_t, mut gain: int16_t)
 -> int16_t {
    let mut ret: libc::c_int =
        ((*raw.offset(1 as libc::c_int as isize) as libc::c_int) <<
             8 as libc::c_int |
             *raw.offset(0 as libc::c_int as isize) as libc::c_int) as int16_t
            as libc::c_int * gain as libc::c_int / 256 as libc::c_int;
    return constrain(ret, -(32767 as libc::c_int) - 1 as libc::c_int,
                     32767 as libc::c_int) as int16_t;
}
unsafe extern "C" fn ak8963Read(mut mag: *mut magDev_t,
                                mut magData: *mut int16_t) -> bool {
    let mut ack: bool = 0 as libc::c_int != 0;
    let mut buf: [uint8_t; 7] = [0; 7];
    let mut busdev: *const busDevice_t = &mut (*mag).busdev;
    match (*busdev).bustype as libc::c_uint {
        1 | 2 => { ack = ak8963DirectReadData(busdev, buf.as_mut_ptr()) }
        3 => { ack = ak8963SlaveReadData(busdev, buf.as_mut_ptr()) }
        _ => { }
    }
    let mut status2: uint8_t = buf[6 as libc::c_int as usize];
    if !ack { return 0 as libc::c_int != 0 }
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        (0x10 as libc::c_int | 0x1 as libc::c_int) as
                            uint8_t);
    if status2 as libc::c_int & 0x8 as libc::c_int != 0 {
        return 0 as libc::c_int != 0
    }
    *magData.offset(X as libc::c_int as isize) =
        parseMag(buf.as_mut_ptr().offset(0 as libc::c_int as isize),
                 (*mag).magGain[X as libc::c_int as usize]);
    *magData.offset(Y as libc::c_int as isize) =
        parseMag(buf.as_mut_ptr().offset(2 as libc::c_int as isize),
                 (*mag).magGain[Y as libc::c_int as usize]);
    *magData.offset(Z as libc::c_int as isize) =
        parseMag(buf.as_mut_ptr().offset(4 as libc::c_int as isize),
                 (*mag).magGain[Z as libc::c_int as usize]);
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn ak8963Init(mut mag: *mut magDev_t) -> bool {
    let mut asa: [uint8_t; 3] = [0; 3];
    let mut status: uint8_t = 0;
    let mut busdev: *const busDevice_t = &mut (*mag).busdev;
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        0xf as libc::c_int as uint8_t);
    ak8963ReadRegisterBuffer(busdev, 0x10 as libc::c_int as uint8_t,
                             asa.as_mut_ptr(),
                             ::core::mem::size_of::<[uint8_t; 3]>() as
                                 libc::c_ulong as uint8_t);
    (*mag).magGain[X as libc::c_int as usize] =
        (asa[X as libc::c_int as usize] as libc::c_int + 128 as libc::c_int)
            as int16_t;
    (*mag).magGain[Y as libc::c_int as usize] =
        (asa[Y as libc::c_int as usize] as libc::c_int + 128 as libc::c_int)
            as int16_t;
    (*mag).magGain[Z as libc::c_int as usize] =
        (asa[Z as libc::c_int as usize] as libc::c_int + 128 as libc::c_int)
            as int16_t;
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
    // Clear status registers
    ak8963ReadRegisterBuffer(busdev, 0x2 as libc::c_int as uint8_t,
                             &mut status, 1 as libc::c_int as uint8_t);
    ak8963ReadRegisterBuffer(busdev, 0x9 as libc::c_int as uint8_t,
                             &mut status, 1 as libc::c_int as uint8_t);
    // Trigger first measurement
    ak8963WriteRegister(busdev, 0xa as libc::c_int as uint8_t,
                        (0x10 as libc::c_int | 0x1 as libc::c_int) as
                            uint8_t); // Disable
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn ak8963BusInit(mut busdev: *const busDevice_t) {
    match (*busdev).bustype as libc::c_uint {
        2 => {
            IOHi((*busdev).busdev_u.spi.csnPin);
            IOInit((*busdev).busdev_u.spi.csnPin, OWNER_COMPASS_CS,
                   0 as libc::c_int as uint8_t);
            IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                         (0x1 as libc::c_uint |
                              (0 as libc::c_uint) << 2 as libc::c_int |
                              (0 as libc::c_uint) << 5 as libc::c_int) as
                             ioConfig_t);
            spiSetDivisor((*busdev).busdev_u.spi.instance,
                          SPI_CLOCK_STANDARD as libc::c_int as uint16_t);
        }
        3 => {
            rescheduleTask(TASK_COMPASS,
                           (1000000 as libc::c_int / 40 as libc::c_int) as
                               uint32_t);
            // initialze I2C master via SPI bus
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x37 as libc::c_int as uint8_t,
                                        ((1 as libc::c_int) <<
                                             4 as libc::c_int |
                                             (1 as libc::c_int) <<
                                                 1 as libc::c_int) as
                                            uint8_t); // I2C multi-master / 400kHz
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x24 as libc::c_int as uint8_t,
                                        0xd as libc::c_int as
                                            uint8_t); // I2C master mode, SPI mode only
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x6a as libc::c_int as uint8_t,
                                        0x30 as libc::c_int as uint8_t);
        }
        1 | _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn ak8963BusDeInit(mut busdev: *const busDevice_t) {
    match (*busdev).bustype as libc::c_uint {
        2 => { spiPreinitCsByIO((*busdev).busdev_u.spi.csnPin); }
        3 => {
            ak8963SpiWriteRegisterDelay((*busdev).busdev_u.mpuSlave.master,
                                        0x37 as libc::c_int as uint8_t,
                                        ((1 as libc::c_int) <<
                                             4 as libc::c_int) as uint8_t);
        }
        1 | _ => { }
    };
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
#[no_mangle]
pub unsafe extern "C" fn ak8963Detect(mut mag: *mut magDev_t) -> bool {
    let mut sig: uint8_t = 0 as libc::c_int as uint8_t; // reset MAG
    let mut busdev: *mut busDevice_t = &mut (*mag).busdev; // check for AK8963
    if ((*busdev).bustype as libc::c_uint ==
            BUSTYPE_I2C as libc::c_int as libc::c_uint ||
            (*busdev).bustype as libc::c_uint ==
                BUSTYPE_MPU_SLAVE as libc::c_int as libc::c_uint) &&
           (*busdev).busdev_u.mpuSlave.address as libc::c_int ==
               0 as libc::c_int {
        (*busdev).busdev_u.mpuSlave.address = 0xc as libc::c_int as uint8_t
    }
    ak8963BusInit(busdev);
    ak8963WriteRegister(busdev, 0xb as libc::c_int as uint8_t,
                        0x1 as libc::c_int as uint8_t);
    delay(4 as libc::c_int as timeMs_t);
    let mut ack: bool =
        ak8963ReadRegisterBuffer(busdev, 0 as libc::c_int as uint8_t,
                                 &mut sig, 1 as libc::c_int as uint8_t);
    if ack as libc::c_int != 0 && sig as libc::c_int == 0x48 as libc::c_int {
        // 0x48 / 01001000 / 'H'
        (*mag).init =
            Some(ak8963Init as
                     unsafe extern "C" fn(_: *mut magDev_t) -> bool);
        (*mag).read =
            Some(ak8963Read as
                     unsafe extern "C" fn(_: *mut magDev_t, _: *mut int16_t)
                         -> bool);
        return 1 as libc::c_int != 0
    }
    ak8963BusDeInit(busdev);
    return 0 as libc::c_int != 0;
}
