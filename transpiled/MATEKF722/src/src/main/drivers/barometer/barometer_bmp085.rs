use ::libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
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
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
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
// packet tag to specify IO pin
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bmp085Config_s {
    pub xclrIO: ioTag_t,
    pub eocIO: ioTag_t,
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
pub type bmp085Config_t = bmp085Config_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bmp085_t {
    pub cal_param: bmp085_smd500_calibration_param_t,
    pub mode: uint8_t,
    pub chip_id: uint8_t,
    pub ml_version: uint8_t,
    pub al_version: uint8_t,
    pub dev_addr: uint8_t,
    pub param_b5: int32_t,
    pub oversampling_setting: int16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bmp085_smd500_calibration_param_t {
    pub ac1: int16_t,
    pub ac2: int16_t,
    pub ac3: int16_t,
    pub ac4: uint16_t,
    pub ac5: uint16_t,
    pub ac6: uint16_t,
    pub b1: int16_t,
    pub b2: int16_t,
    pub mb: int16_t,
    pub mc: int16_t,
    pub md: int16_t,
}
//calibration parameter
static mut bmp085: bmp085_t =
    bmp085_t{cal_param:
                 bmp085_smd500_calibration_param_t{ac1: 0,
                                                   ac2: 0,
                                                   ac3: 0,
                                                   ac4: 0,
                                                   ac5: 0,
                                                   ac6: 0,
                                                   b1: 0,
                                                   b2: 0,
                                                   mb: 0,
                                                   mc: 0,
                                                   md: 0,},
             mode: 0,
             chip_id: 0,
             ml_version: 0,
             al_version: 0,
             dev_addr: 0,
             param_b5: 0,
             oversampling_setting: 0,};
// 6000+21000=27000 1.5ms margin according to the spec (25.5ms P conversion time with OSS=3)
static mut bmp085InitDone: bool = 0 as libc::c_int != 0;
static mut bmp085_ut: uint16_t = 0;
// static result of temperature measurement
static mut bmp085_up: uint32_t = 0;
static mut xclrIO: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
#[no_mangle]
pub unsafe extern "C" fn bmp085InitXclrIO(mut config: *const bmp085Config_t) {
    if xclrIO.is_null() && !config.is_null() &&
           (*config).xclrIO as libc::c_int != 0 {
        xclrIO = IOGetByTag((*config).xclrIO);
        IOInit(xclrIO, OWNER_BARO_CS, 0 as libc::c_int as uint8_t);
        IOConfigGPIO(xclrIO,
                     (0x1 as libc::c_uint |
                          (0 as libc::c_uint) << 2 as libc::c_int |
                          (0 as libc::c_uint) << 5 as libc::c_int) as
                         ioConfig_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn bmp085Disable(mut config: *const bmp085Config_t) {
    bmp085InitXclrIO(config);
}
#[no_mangle]
pub unsafe extern "C" fn bmp085Detect(mut config: *const bmp085Config_t,
                                      mut baro: *mut baroDev_t) -> bool {
    let mut data: uint8_t = 0;
    let mut ack: bool = false;
    let mut defaultAddressApplied: bool = 0 as libc::c_int != 0;
    if bmp085InitDone { return 1 as libc::c_int != 0 }
    bmp085InitXclrIO(config);
    // enable baro
    delay(20 as libc::c_int as
              timeMs_t); // datasheet says 10ms, we'll be careful and do 20.
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev;
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0 as libc::c_int {
        // Default address for BMP085
        (*busdev).busdev_u.i2c.address =
            0x77 as libc::c_int as uint8_t; /* read Chip Id */
        defaultAddressApplied = 1 as libc::c_int != 0
    }
    ack =
        busReadRegisterBuffer(busdev, 0xd0 as libc::c_int as uint8_t,
                              &mut data, 1 as libc::c_int as uint8_t);
    if ack {
        bmp085.chip_id =
            ((data as libc::c_int & 0xff as libc::c_int) >> 0 as libc::c_int)
                as uint8_t;
        bmp085.oversampling_setting = 3 as libc::c_int as int16_t;
        if bmp085.chip_id as libc::c_int == 0x55 as libc::c_int {
            /* get bitslice */
            busReadRegisterBuffer(busdev, 0xd1 as libc::c_int as uint8_t,
                                  &mut data,
                                  1 as libc::c_int as
                                      uint8_t); /* read Version reg */
            bmp085.ml_version =
                ((data as libc::c_int & 0xf as libc::c_int) >>
                     0 as libc::c_int) as uint8_t; /* get ML Version */
            bmp085.al_version =
                ((data as libc::c_int & 0xf0 as libc::c_int) >>
                     4 as libc::c_int) as uint8_t; /* get AL Version */
            bmp085_get_cal_param(busdev); /* readout bmp085 calibparam structure */
            (*baro).ut_delay =
                6000 as libc::c_int as
                    uint16_t; // temperature in 0.01 C (make same as MS5611)
            (*baro).up_delay = 27000 as libc::c_int as uint16_t;
            (*baro).start_ut =
                Some(bmp085_start_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_ut =
                Some(bmp085_get_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).start_up =
                Some(bmp085_start_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_up =
                Some(bmp085_get_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).calculate =
                Some(bmp085_calculate as
                         unsafe extern "C" fn(_: *mut int32_t,
                                              _: *mut int32_t) -> ());
            bmp085InitDone = 1 as libc::c_int != 0;
            return 1 as libc::c_int != 0
        }
    }
    if defaultAddressApplied {
        (*busdev).busdev_u.i2c.address = 0 as libc::c_int as uint8_t
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn bmp085_get_temperature(mut ut: uint32_t) -> int32_t {
    let mut temperature: int32_t = 0;
    let mut x1: int32_t = 0;
    let mut x2: int32_t = 0;
    x1 =
        (ut as int32_t - bmp085.cal_param.ac6 as int32_t) *
            bmp085.cal_param.ac5 as int32_t >> 15 as libc::c_int;
    x2 =
        ((bmp085.cal_param.mc as int32_t) << 11 as libc::c_int) /
            (x1 + bmp085.cal_param.md as libc::c_int);
    bmp085.param_b5 = x1 + x2;
    temperature =
        bmp085.param_b5 * 10 as libc::c_int + 8 as libc::c_int >>
            4 as libc::c_int;
    return temperature;
}
unsafe extern "C" fn bmp085_get_pressure(mut up: uint32_t) -> int32_t {
    let mut pressure: int32_t = 0;
    let mut x1: int32_t = 0;
    let mut x2: int32_t = 0;
    let mut x3: int32_t = 0;
    let mut b3: int32_t = 0;
    let mut b6: int32_t = 0;
    let mut b4: uint32_t = 0;
    let mut b7: uint32_t = 0;
    b6 = bmp085.param_b5 - 4000 as libc::c_int;
    // *****calculate B3************
    x1 = b6 * b6 >> 12 as libc::c_int;
    x1 *= bmp085.cal_param.b2 as libc::c_int;
    x1 >>= 11 as libc::c_int;
    x2 = bmp085.cal_param.ac2 as libc::c_int * b6;
    x2 >>= 11 as libc::c_int;
    x3 = x1 + x2;
    b3 =
        (bmp085.cal_param.ac1 as int32_t * 4 as libc::c_int + x3 <<
             bmp085.oversampling_setting as libc::c_int) + 2 as libc::c_int >>
            2 as libc::c_int;
    // *****calculate B4************
    x1 =
        bmp085.cal_param.ac3 as libc::c_int * b6 >>
            13 as libc::c_int; // pressure in Pa
    x2 =
        bmp085.cal_param.b1 as libc::c_int * (b6 * b6 >> 12 as libc::c_int) >>
            16 as libc::c_int;
    x3 = x1 + x2 + 2 as libc::c_int >> 2 as libc::c_int;
    b4 =
        (bmp085.cal_param.ac4 as
             libc::c_uint).wrapping_mul((x3 + 32768 as libc::c_int) as
                                            uint32_t) >> 15 as libc::c_int;
    b7 =
        up.wrapping_sub(b3 as
                            libc::c_uint).wrapping_mul((50000 as libc::c_int
                                                            >>
                                                            bmp085.oversampling_setting
                                                                as
                                                                libc::c_int)
                                                           as libc::c_uint);
    if b7 < 0x80000000 as libc::c_uint {
        pressure = (b7 << 1 as libc::c_int).wrapping_div(b4) as int32_t
    } else { pressure = (b7.wrapping_div(b4) << 1 as libc::c_int) as int32_t }
    x1 = pressure >> 8 as libc::c_int;
    x1 *= x1;
    x1 = x1 * 3038 as libc::c_int >> 16 as libc::c_int;
    x2 = pressure * -(7357 as libc::c_int) >> 16 as libc::c_int;
    pressure += x1 + x2 + 3791 as libc::c_int >> 4 as libc::c_int;
    return pressure;
}
unsafe extern "C" fn bmp085_start_ut(mut baro: *mut baroDev_t) {
    busWriteRegister(&mut (*baro).busdev, 0xf4 as libc::c_int as uint8_t,
                     0x2e as libc::c_int as uint8_t);
}
unsafe extern "C" fn bmp085_get_ut(mut baro: *mut baroDev_t) {
    let mut data: [uint8_t; 2] = [0; 2];
    busReadRegisterBuffer(&mut (*baro).busdev, 0xf6 as libc::c_int as uint8_t,
                          data.as_mut_ptr(), 2 as libc::c_int as uint8_t);
    bmp085_ut =
        ((data[0 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[1 as libc::c_int as usize] as libc::c_int) as uint16_t;
}
unsafe extern "C" fn bmp085_start_up(mut baro: *mut baroDev_t) {
    let mut ctrl_reg_data: uint8_t = 0;
    ctrl_reg_data =
        (0x34 as libc::c_int +
             ((bmp085.oversampling_setting as libc::c_int) <<
                  6 as libc::c_int)) as uint8_t;
    busWriteRegister(&mut (*baro).busdev, 0xf4 as libc::c_int as uint8_t,
                     ctrl_reg_data);
}
/* * read out up for pressure conversion
 depending on the oversampling ratio setting up can be 16 to 19 bit
 \return up parameter that represents the uncompensated pressure value
 */
unsafe extern "C" fn bmp085_get_up(mut baro: *mut baroDev_t) {
    let mut data: [uint8_t; 3] = [0; 3];
    busReadRegisterBuffer(&mut (*baro).busdev, 0xf6 as libc::c_int as uint8_t,
                          data.as_mut_ptr(), 3 as libc::c_int as uint8_t);
    bmp085_up =
        ((data[0 as libc::c_int as usize] as uint32_t) << 16 as libc::c_int |
             (data[1 as libc::c_int as usize] as uint32_t) << 8 as libc::c_int
             | data[2 as libc::c_int as usize] as uint32_t) >>
            8 as libc::c_int - bmp085.oversampling_setting as libc::c_int;
}
unsafe extern "C" fn bmp085_calculate(mut pressure: *mut int32_t,
                                      mut temperature: *mut int32_t) {
    let mut temp: int32_t = 0;
    let mut press: int32_t = 0;
    temp = bmp085_get_temperature(bmp085_ut as uint32_t);
    press = bmp085_get_pressure(bmp085_up);
    if !pressure.is_null() { *pressure = press }
    if !temperature.is_null() { *temperature = temp };
}
// static result of pressure measurement
unsafe extern "C" fn bmp085_get_cal_param(mut busdev: *mut busDevice_t) {
    let mut data: [uint8_t; 22] = [0; 22];
    busReadRegisterBuffer(busdev, 0xaa as libc::c_int as uint8_t,
                          data.as_mut_ptr(), 22 as libc::c_int as uint8_t);
    /*parameters AC1-AC6*/
    bmp085.cal_param.ac1 =
        ((data[0 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[1 as libc::c_int as usize] as libc::c_int) as int16_t;
    bmp085.cal_param.ac2 =
        ((data[2 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[3 as libc::c_int as usize] as libc::c_int) as int16_t;
    bmp085.cal_param.ac3 =
        ((data[4 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[5 as libc::c_int as usize] as libc::c_int) as int16_t;
    bmp085.cal_param.ac4 =
        ((data[6 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[7 as libc::c_int as usize] as libc::c_int) as uint16_t;
    bmp085.cal_param.ac5 =
        ((data[8 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[9 as libc::c_int as usize] as libc::c_int) as uint16_t;
    bmp085.cal_param.ac6 =
        ((data[10 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[11 as libc::c_int as usize] as libc::c_int) as uint16_t;
    /*parameters B1,B2*/
    bmp085.cal_param.b1 =
        ((data[12 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[13 as libc::c_int as usize] as libc::c_int) as int16_t;
    bmp085.cal_param.b2 =
        ((data[14 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[15 as libc::c_int as usize] as libc::c_int) as int16_t;
    /*parameters MB,MC,MD*/
    bmp085.cal_param.mb =
        ((data[16 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[17 as libc::c_int as usize] as libc::c_int) as int16_t;
    bmp085.cal_param.mc =
        ((data[18 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[19 as libc::c_int as usize] as libc::c_int) as int16_t;
    bmp085.cal_param.md =
        ((data[20 as libc::c_int as usize] as libc::c_int) << 8 as libc::c_int
             | data[21 as libc::c_int as usize] as libc::c_int) as int16_t;
}
/* BARO */
