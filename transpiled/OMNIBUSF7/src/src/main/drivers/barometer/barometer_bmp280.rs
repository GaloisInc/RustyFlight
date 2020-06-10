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
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
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
    fn delay(ms: timeMs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __int64_t = libc::c_long;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief DMA Controller
  */
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
/* *
  * @brief Serial Peripheral Interface
  */
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
// baro start operation
// baro calculation (filled params are pressure and temperature)
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type baroDev_t = baroDev_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_0 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_0 = 8;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_0 = 16;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_0 = 256;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_0 = 256;
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
pub type bmp280_calib_param_t = bmp280_calib_param_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bmp280_calib_param_s {
    pub dig_T1: uint16_t,
    pub dig_T2: int16_t,
    pub dig_T3: int16_t,
    pub dig_P1: uint16_t,
    pub dig_P2: int16_t,
    pub dig_P3: int16_t,
    pub dig_P4: int16_t,
    pub dig_P5: int16_t,
    pub dig_P6: int16_t,
    pub dig_P7: int16_t,
    pub dig_P8: int16_t,
    pub dig_P9: int16_t,
    pub t_fine: int32_t,
}
static mut bmp280_chip_id: uint8_t = 0 as libc::c_int as uint8_t;
static mut bmp280_cal: bmp280_calib_param_t =
    bmp280_calib_param_t{dig_T1: 0,
                         dig_T2: 0,
                         dig_T3: 0,
                         dig_P1: 0,
                         dig_P2: 0,
                         dig_P3: 0,
                         dig_P4: 0,
                         dig_P5: 0,
                         dig_P6: 0,
                         dig_P7: 0,
                         dig_P8: 0,
                         dig_P9: 0,
                         t_fine: 0,};
/* calibration T1 data */
/* calibration T2 data */
/* calibration T3 data */
/* calibration P1 data */
/* calibration P2 data */
/* calibration P3 data */
/* calibration P4 data */
/* calibration P5 data */
/* calibration P6 data */
/* calibration P7 data */
/* calibration P8 data */
/* calibration P9 data */
/* calibration t_fine data */
// uncompensated pressure and temperature
#[no_mangle]
pub static mut bmp280_up: int32_t = 0 as libc::c_int;
#[no_mangle]
pub static mut bmp280_ut: int32_t = 0 as libc::c_int;
#[no_mangle]
pub unsafe extern "C" fn bmp280BusInit(mut busdev: *mut busDevice_t) {
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_SPI as libc::c_int as libc::c_uint {
        IOHi((*busdev).busdev_u.spi.csnPin);
        IOInit((*busdev).busdev_u.spi.csnPin, OWNER_BARO_CS,
               0 as libc::c_int as uint8_t);
        IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                     (0x1 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
        spiSetDivisor((*busdev).busdev_u.spi.instance,
                      SPI_CLOCK_STANDARD as libc::c_int as
                          uint16_t); // Disable
        // XXX
    };
}
#[no_mangle]
pub unsafe extern "C" fn bmp280BusDeinit(mut busdev: *mut busDevice_t) {
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_SPI as libc::c_int as libc::c_uint {
        spiPreinitCsByIO((*busdev).busdev_u.spi.csnPin);
    };
}
// 10/16 = 0.625 ms
#[no_mangle]
pub unsafe extern "C" fn bmp280Detect(mut baro: *mut baroDev_t) -> bool {
    delay(20 as libc::c_int as timeMs_t);
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev;
    let mut defaultAddressApplied: bool = 0 as libc::c_int != 0;
    bmp280BusInit(busdev);
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0 as libc::c_int {
        // Default address for BMP280
        (*busdev).busdev_u.i2c.address =
            0x76 as libc::c_int as uint8_t; /* read Chip Id */
        defaultAddressApplied = 1 as libc::c_int != 0
    }
    busReadRegisterBuffer(busdev, 0xd0 as libc::c_int as uint8_t,
                          &mut bmp280_chip_id, 1 as libc::c_int as uint8_t);
    if bmp280_chip_id as libc::c_int != 0x58 as libc::c_int {
        bmp280BusDeinit(busdev);
        if defaultAddressApplied {
            (*busdev).busdev_u.i2c.address = 0 as libc::c_int as uint8_t
        }
        return 0 as libc::c_int != 0
    }
    // read calibration
    busReadRegisterBuffer(busdev, 0x88 as libc::c_int as uint8_t,
                          &mut bmp280_cal as *mut bmp280_calib_param_t as
                              *mut uint8_t, 24 as libc::c_int as uint8_t);
    // set oversampling + power mode (forced), and start sampling
    busWriteRegister(busdev, 0xf4 as libc::c_int as uint8_t,
                     ((0x4 as libc::c_int) << 2 as libc::c_int |
                          (0x1 as libc::c_int) << 5 as libc::c_int |
                          0x1 as libc::c_int) as uint8_t);
    // these are dummy as temperature is measured as part of pressure
    (*baro).ut_delay = 0 as libc::c_int as uint16_t;
    (*baro).get_ut =
        Some(bmp280_get_ut as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).start_ut =
        Some(bmp280_start_ut as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    // only _up part is executed, and gets both temperature and pressure
    (*baro).start_up =
        Some(bmp280_start_up as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).get_up =
        Some(bmp280_get_up as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).up_delay =
        ((20 as libc::c_int +
              37 as libc::c_int *
                  (((1 as libc::c_int) << 0x1 as libc::c_int >>
                        1 as libc::c_int) +
                       ((1 as libc::c_int) << 0x4 as libc::c_int >>
                            1 as libc::c_int)) +
              (if 0x4 as libc::c_int != 0 {
                   10 as libc::c_int
               } else { 0 as libc::c_int }) + 15 as libc::c_int) /
             16 as libc::c_int * 1000 as libc::c_int) as uint16_t;
    (*baro).calculate =
        Some(bmp280_calculate as
                 unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t)
                     -> ());
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn bmp280_start_ut(mut baro: *mut baroDev_t) {
    // dummy
}
unsafe extern "C" fn bmp280_get_ut(mut baro: *mut baroDev_t) {
    // dummy
}
unsafe extern "C" fn bmp280_start_up(mut baro: *mut baroDev_t) {
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    busWriteRegister(&mut (*baro).busdev, 0xf4 as libc::c_int as uint8_t,
                     ((0x4 as libc::c_int) << 2 as libc::c_int |
                          (0x1 as libc::c_int) << 5 as libc::c_int |
                          0x1 as libc::c_int) as uint8_t);
}
unsafe extern "C" fn bmp280_get_up(mut baro: *mut baroDev_t) {
    let mut data: [uint8_t; 6] = [0; 6];
    // read data from sensor
    busReadRegisterBuffer(&mut (*baro).busdev, 0xf7 as libc::c_int as uint8_t,
                          data.as_mut_ptr(), 6 as libc::c_int as uint8_t);
    bmp280_up =
        ((data[0 as libc::c_int as usize] as uint32_t) << 12 as libc::c_int |
             (data[1 as libc::c_int as usize] as uint32_t) << 4 as libc::c_int
             |
             data[2 as libc::c_int as usize] as uint32_t >> 4 as libc::c_int)
            as int32_t;
    bmp280_ut =
        ((data[3 as libc::c_int as usize] as uint32_t) << 12 as libc::c_int |
             (data[4 as libc::c_int as usize] as uint32_t) << 4 as libc::c_int
             |
             data[5 as libc::c_int as usize] as uint32_t >> 4 as libc::c_int)
            as int32_t;
}
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
unsafe extern "C" fn bmp280_compensate_T(mut adc_T: int32_t) -> int32_t {
    let mut var1: int32_t = 0;
    let mut var2: int32_t = 0;
    let mut T: int32_t = 0;
    var1 =
        ((adc_T >> 3 as libc::c_int) -
             ((bmp280_cal.dig_T1 as int32_t) << 1 as libc::c_int)) *
            bmp280_cal.dig_T2 as int32_t >> 11 as libc::c_int;
    var2 =
        (((adc_T >> 4 as libc::c_int) - bmp280_cal.dig_T1 as int32_t) *
             ((adc_T >> 4 as libc::c_int) - bmp280_cal.dig_T1 as int32_t) >>
             12 as libc::c_int) * bmp280_cal.dig_T3 as int32_t >>
            14 as libc::c_int;
    bmp280_cal.t_fine = var1 + var2;
    T =
        bmp280_cal.t_fine * 5 as libc::c_int + 128 as libc::c_int >>
            8 as libc::c_int;
    return T;
}
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
unsafe extern "C" fn bmp280_compensate_P(mut adc_P: int32_t) -> uint32_t {
    let mut var1: int64_t = 0;
    let mut var2: int64_t = 0;
    let mut p: int64_t = 0;
    var1 =
        bmp280_cal.t_fine as int64_t - 128000 as libc::c_int as libc::c_long;
    var2 = var1 * var1 * bmp280_cal.dig_P6 as int64_t;
    var2 =
        var2 + ((var1 * bmp280_cal.dig_P5 as int64_t) << 17 as libc::c_int);
    var2 = var2 + ((bmp280_cal.dig_P4 as int64_t) << 35 as libc::c_int);
    var1 =
        (var1 * var1 * bmp280_cal.dig_P3 as int64_t >> 8 as libc::c_int) +
            ((var1 * bmp280_cal.dig_P2 as int64_t) << 12 as libc::c_int);
    var1 =
        (((1 as libc::c_int as int64_t) << 47 as libc::c_int) + var1) *
            bmp280_cal.dig_P1 as int64_t >> 33 as libc::c_int;
    if var1 == 0 as libc::c_int as libc::c_long {
        return 0 as libc::c_int as uint32_t
    }
    p = (1048576 as libc::c_int - adc_P) as int64_t;
    p =
        ((p << 31 as libc::c_int) - var2) *
            3125 as libc::c_int as libc::c_long / var1;
    var1 =
        bmp280_cal.dig_P9 as int64_t * (p >> 13 as libc::c_int) *
            (p >> 13 as libc::c_int) >> 25 as libc::c_int;
    var2 = bmp280_cal.dig_P8 as int64_t * p >> 19 as libc::c_int;
    p =
        (p + var1 + var2 >> 8 as libc::c_int) +
            ((bmp280_cal.dig_P7 as int64_t) << 4 as libc::c_int);
    return p as uint32_t;
}
unsafe extern "C" fn bmp280_calculate(mut pressure: *mut int32_t,
                                      mut temperature: *mut int32_t) {
    // calculate
    let mut t: int32_t = 0;
    let mut p: uint32_t = 0;
    t = bmp280_compensate_T(bmp280_ut);
    p = bmp280_compensate_P(bmp280_up);
    if !pressure.is_null() {
        *pressure =
            p.wrapping_div(256 as libc::c_int as libc::c_uint) as int32_t
    }
    if !temperature.is_null() { *temperature = t };
}
