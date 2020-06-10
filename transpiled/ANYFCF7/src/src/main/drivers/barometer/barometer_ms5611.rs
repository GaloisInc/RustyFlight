use ::libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
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
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __int64_t = libc::c_long;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
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
// microsecond time
pub type timeUs_t = uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
static mut ms5611_ut: uint32_t = 0;
// static result of temperature measurement
static mut ms5611_up: uint32_t = 0;
// static result of pressure measurement
static mut ms5611_c: [uint16_t; 8] = [0; 8];
// on-chip ROM
static mut ms5611_osr: uint8_t = 0x8 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn ms5611BusInit(mut busdev: *mut busDevice_t) { }
#[no_mangle]
pub unsafe extern "C" fn ms5611BusDeinit(mut busdev: *mut busDevice_t) { }
#[no_mangle]
pub unsafe extern "C" fn ms5611Detect(mut baro: *mut baroDev_t) -> bool {
    let mut sig: uint8_t =
        0; // No idea how long the chip takes to power-up, but let's make it 10ms
    let mut i: libc::c_int = 0;
    let mut defaultAddressApplied: bool = 0 as libc::c_int != 0;
    delay(10 as libc::c_int as timeMs_t);
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev;
    ms5611BusInit(busdev);
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0 as libc::c_int {
        // Default address for MS5611
        (*busdev).busdev_u.i2c.address = 0x77 as libc::c_int as uint8_t;
        defaultAddressApplied = 1 as libc::c_int != 0
    }
    if !(!busReadRegisterBuffer(busdev, 0xa0 as libc::c_int as uint8_t,
                                &mut sig, 1 as libc::c_int as uint8_t) ||
             sig as libc::c_int == 0xff as libc::c_int) {
        ms5611_reset(busdev);
        // read all coefficients
        i = 0 as libc::c_int;
        while i < 8 as libc::c_int {
            ms5611_c[i as usize] = ms5611_prom(busdev, i as int8_t);
            i += 1
        }
        // check crc, bail out if wrong - we are probably talking to BMP085 w/o XCLR line!
        if !(ms5611_crc(ms5611_c.as_mut_ptr()) as libc::c_int !=
                 0 as libc::c_int) {
            // TODO prom + CRC
            (*baro).ut_delay =
                10000 as libc::c_int as uint16_t; // send PROM READ command
            (*baro).up_delay = 10000 as libc::c_int as uint16_t; // read ADC
            (*baro).start_ut =
                Some(ms5611_start_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_ut =
                Some(ms5611_get_ut as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).start_up =
                Some(ms5611_start_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).get_up =
                Some(ms5611_get_up as
                         unsafe extern "C" fn(_: *mut baroDev_t) -> ());
            (*baro).calculate =
                Some(ms5611_calculate as
                         unsafe extern "C" fn(_: *mut int32_t,
                                              _: *mut int32_t) -> ());
            return 1 as libc::c_int != 0
        }
    }
    ms5611BusDeinit(busdev);
    if defaultAddressApplied {
        (*busdev).busdev_u.i2c.address = 0 as libc::c_int as uint8_t
    }
    return 0 as libc::c_int != 0;
}
unsafe extern "C" fn ms5611_reset(mut busdev: *mut busDevice_t) {
    busWriteRegister(busdev, 0x1e as libc::c_int as uint8_t,
                     1 as libc::c_int as uint8_t);
    delayMicroseconds(2800 as libc::c_int as timeUs_t);
}
unsafe extern "C" fn ms5611_prom(mut busdev: *mut busDevice_t,
                                 mut coef_num: int8_t) -> uint16_t {
    let mut rxbuf: [uint8_t; 2] =
        [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    busReadRegisterBuffer(busdev,
                          (0xa0 as libc::c_int +
                               coef_num as libc::c_int * 2 as libc::c_int) as
                              uint8_t, rxbuf.as_mut_ptr(),
                          2 as libc::c_int as uint8_t);
    return ((rxbuf[0 as libc::c_int as usize] as libc::c_int) <<
                8 as libc::c_int |
                rxbuf[1 as libc::c_int as usize] as libc::c_int) as uint16_t;
}
unsafe extern "C" fn ms5611_crc(mut prom: *mut uint16_t) -> int8_t {
    let mut i: int32_t = 0;
    let mut j: int32_t = 0;
    let mut res: uint32_t = 0 as libc::c_int as uint32_t;
    let mut crc: uint8_t =
        (*prom.offset(7 as libc::c_int as isize) as libc::c_int &
             0xf as libc::c_int) as uint8_t;
    let ref mut fresh0 = *prom.offset(7 as libc::c_int as isize);
    *fresh0 = (*fresh0 as libc::c_int & 0xff00 as libc::c_int) as uint16_t;
    let mut blankEeprom: bool = 1 as libc::c_int != 0;
    i = 0 as libc::c_int;
    while i < 16 as libc::c_int {
        if *prom.offset((i >> 1 as libc::c_int) as isize) != 0 {
            blankEeprom = 0 as libc::c_int != 0
        }
        if i & 1 as libc::c_int != 0 {
            res ^=
                (*prom.offset((i >> 1 as libc::c_int) as isize) as libc::c_int
                     & 0xff as libc::c_int) as libc::c_uint
        } else {
            res ^=
                (*prom.offset((i >> 1 as libc::c_int) as isize) as libc::c_int
                     >> 8 as libc::c_int) as libc::c_uint
        }
        j = 8 as libc::c_int;
        while j > 0 as libc::c_int {
            if res & 0x8000 as libc::c_int as libc::c_uint != 0 {
                res ^= 0x1800 as libc::c_int as libc::c_uint
            }
            res <<= 1 as libc::c_int;
            j -= 1
        }
        i += 1
    }
    let ref mut fresh1 = *prom.offset(7 as libc::c_int as isize);
    *fresh1 = (*fresh1 as libc::c_int | crc as libc::c_int) as uint16_t;
    if !blankEeprom &&
           crc as libc::c_uint ==
               res >> 12 as libc::c_int & 0xf as libc::c_int as libc::c_uint {
        return 0 as libc::c_int as int8_t
    }
    return -(1 as libc::c_int) as int8_t;
}
unsafe extern "C" fn ms5611_read_adc(mut busdev: *mut busDevice_t)
 -> uint32_t {
    let mut rxbuf: [uint8_t; 3] = [0; 3];
    busReadRegisterBuffer(busdev, 0 as libc::c_int as uint8_t,
                          rxbuf.as_mut_ptr(), 3 as libc::c_int as uint8_t);
    return ((rxbuf[0 as libc::c_int as usize] as libc::c_int) <<
                16 as libc::c_int |
                (rxbuf[1 as libc::c_int as usize] as libc::c_int) <<
                    8 as libc::c_int |
                rxbuf[2 as libc::c_int as usize] as libc::c_int) as uint32_t;
}
unsafe extern "C" fn ms5611_start_ut(mut baro: *mut baroDev_t) {
    busWriteRegister(&mut (*baro).busdev,
                     (0x40 as libc::c_int + 0x10 as libc::c_int +
                          ms5611_osr as libc::c_int) as uint8_t,
                     1 as libc::c_int as uint8_t);
    // D2 (temperature) conversion start!
}
unsafe extern "C" fn ms5611_get_ut(mut baro: *mut baroDev_t) {
    ms5611_ut = ms5611_read_adc(&mut (*baro).busdev);
}
unsafe extern "C" fn ms5611_start_up(mut baro: *mut baroDev_t) {
    busWriteRegister(&mut (*baro).busdev,
                     (0x40 as libc::c_int + 0 as libc::c_int +
                          ms5611_osr as libc::c_int) as uint8_t,
                     1 as libc::c_int as uint8_t);
    // D1 (pressure) conversion start!
}
unsafe extern "C" fn ms5611_get_up(mut baro: *mut baroDev_t) {
    ms5611_up = ms5611_read_adc(&mut (*baro).busdev);
}
unsafe extern "C" fn ms5611_calculate(mut pressure: *mut int32_t,
                                      mut temperature: *mut int32_t) {
    let mut press: uint32_t = 0;
    let mut temp: int64_t = 0;
    let mut delt: int64_t = 0;
    let mut dT: int64_t =
        (ms5611_ut as int64_t as
             libc::c_ulong).wrapping_sub((ms5611_c[5 as libc::c_int as usize]
                                              as
                                              uint64_t).wrapping_mul(256 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_ulong))
            as int64_t;
    let mut off: int64_t =
        ((ms5611_c[2 as libc::c_int as usize] as int64_t) <<
             16 as libc::c_int) +
            (ms5611_c[4 as libc::c_int as usize] as int64_t * dT >>
                 7 as libc::c_int);
    let mut sens: int64_t =
        ((ms5611_c[1 as libc::c_int as usize] as int64_t) <<
             15 as libc::c_int) +
            (ms5611_c[3 as libc::c_int as usize] as int64_t * dT >>
                 8 as libc::c_int);
    temp =
        2000 as libc::c_int as libc::c_long +
            (dT * ms5611_c[6 as libc::c_int as usize] as int64_t >>
                 23 as libc::c_int);
    if temp < 2000 as libc::c_int as libc::c_long {
        // temperature lower than 20degC
        delt = temp - 2000 as libc::c_int as libc::c_long;
        delt = 5 as libc::c_int as libc::c_long * delt * delt;
        off -= delt >> 1 as libc::c_int;
        sens -= delt >> 2 as libc::c_int;
        if temp < -(1500 as libc::c_int) as libc::c_long {
            // temperature lower than -15degC
            delt = temp + 1500 as libc::c_int as libc::c_long;
            delt = delt * delt;
            off -= 7 as libc::c_int as libc::c_long * delt;
            sens -=
                11 as libc::c_int as libc::c_long * delt >> 1 as libc::c_int
        }
        temp -= dT * dT >> 31 as libc::c_int
    }
    press =
        ((ms5611_up as int64_t * sens >> 21 as libc::c_int) - off >>
             15 as libc::c_int) as uint32_t;
    if !pressure.is_null() { *pressure = press as int32_t }
    if !temperature.is_null() { *temperature = temp as int32_t };
}
