use ::libc;
extern "C" {
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn spiInitDevice(device: SPIDevice);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
pub type rccPeriphTag_t = uint8_t;
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
pub type spiDevice_t = SPIDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPIDevice_s {
    pub dev: *mut SPI_TypeDef,
    pub sck: ioTag_t,
    pub miso: ioTag_t,
    pub mosi: ioTag_t,
    pub sckAF: uint8_t,
    pub misoAF: uint8_t,
    pub mosiAF: uint8_t,
    pub rcc: rccPeriphTag_t,
    pub errorCount: uint16_t,
    pub leadingEdge: bool,
    pub hspi: SPI_HandleTypeDef,
    pub hdma: DMA_HandleTypeDef,
    pub dmaIrqHandler: uint8_t,
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
#[no_mangle]
pub static mut spiDevice: [spiDevice_t; 4] =
    [spiDevice_t{dev: 0 as *const SPI_TypeDef as *mut SPI_TypeDef,
                 sck: 0,
                 miso: 0,
                 mosi: 0,
                 sckAF: 0,
                 misoAF: 0,
                 mosiAF: 0,
                 rcc: 0,
                 errorCount: 0,
                 leadingEdge: false,
                 hspi:
                     SPI_HandleTypeDef{Instance:
                                           0 as *const SPI_TypeDef as
                                               *mut SPI_TypeDef,
                                       Init:
                                           SPI_InitTypeDef{Mode: 0,
                                                           Direction: 0,
                                                           DataSize: 0,
                                                           CLKPolarity: 0,
                                                           CLKPhase: 0,
                                                           NSS: 0,
                                                           BaudRatePrescaler:
                                                               0,
                                                           FirstBit: 0,
                                                           TIMode: 0,
                                                           CRCCalculation: 0,
                                                           CRCPolynomial: 0,
                                                           CRCLength: 0,
                                                           NSSPMode: 0,},
                                       pTxBuffPtr:
                                           0 as *const uint8_t as
                                               *mut uint8_t,
                                       TxXferSize: 0,
                                       TxXferCount: 0,
                                       pRxBuffPtr:
                                           0 as *const uint8_t as
                                               *mut uint8_t,
                                       RxXferSize: 0,
                                       RxXferCount: 0,
                                       CRCSize: 0,
                                       RxISR: None,
                                       TxISR: None,
                                       hdmatx:
                                           0 as *const DMA_HandleTypeDef as
                                               *mut DMA_HandleTypeDef,
                                       hdmarx:
                                           0 as *const DMA_HandleTypeDef as
                                               *mut DMA_HandleTypeDef,
                                       Lock: HAL_UNLOCKED,
                                       State: HAL_SPI_STATE_RESET,
                                       ErrorCode: 0,},
                 hdma:
                     DMA_HandleTypeDef{Instance:
                                           0 as *const DMA_Stream_TypeDef as
                                               *mut DMA_Stream_TypeDef,
                                       Init:
                                           DMA_InitTypeDef{Channel: 0,
                                                           Direction: 0,
                                                           PeriphInc: 0,
                                                           MemInc: 0,
                                                           PeriphDataAlignment:
                                                               0,
                                                           MemDataAlignment:
                                                               0,
                                                           Mode: 0,
                                                           Priority: 0,
                                                           FIFOMode: 0,
                                                           FIFOThreshold: 0,
                                                           MemBurst: 0,
                                                           PeriphBurst: 0,},
                                       Lock: HAL_UNLOCKED,
                                       State: HAL_DMA_STATE_RESET,
                                       Parent:
                                           0 as *const libc::c_void as
                                               *mut libc::c_void,
                                       XferCpltCallback: None,
                                       XferHalfCpltCallback: None,
                                       XferM1CpltCallback: None,
                                       XferM1HalfCpltCallback: None,
                                       XferErrorCallback: None,
                                       XferAbortCallback: None,
                                       ErrorCode: 0,
                                       StreamBaseAddress: 0,
                                       StreamIndex: 0,},
                 dmaIrqHandler: 0,}; 4];
#[no_mangle]
pub unsafe extern "C" fn spiDeviceByInstance(mut instance: *mut SPI_TypeDef)
 -> SPIDevice {
    if instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3000
                                                                              as
                                                                              libc::c_uint)
               as *mut SPI_TypeDef {
        return SPIDEV_1
    } // read transaction
    if instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3400
                                                                              as
                                                                              libc::c_uint)
               as *mut SPI_TypeDef {
        return SPIDEV_4
    } // read transaction
    return SPIINVALID;
}
#[no_mangle]
pub unsafe extern "C" fn spiInstanceByDevice(mut device: SPIDevice)
 -> *mut SPI_TypeDef {
    if device as libc::c_int >= 4 as libc::c_int {
        return 0 as *mut SPI_TypeDef
    }
    return spiDevice[device as usize].dev;
}
#[no_mangle]
pub unsafe extern "C" fn spiInit(mut device: SPIDevice) -> bool {
    match device as libc::c_int {
        -1 => { return 0 as libc::c_int != 0 }
        0 => { spiInitDevice(device); return 1 as libc::c_int != 0 }
        3 => { spiInitDevice(device); return 1 as libc::c_int != 0 }
        1 | 2 | _ => { }
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiTimeoutUserCallback(mut instance:
                                                    *mut SPI_TypeDef)
 -> uint32_t {
    let mut device: SPIDevice = spiDeviceByInstance(instance);
    if device as libc::c_int == SPIINVALID as libc::c_int {
        return -(1 as libc::c_int) as uint32_t
    }
    ::core::ptr::write_volatile(&mut spiDevice[device as usize].errorCount as
                                    *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&spiDevice[device
                                                                                      as
                                                                                      usize].errorCount
                                                                           as
                                                                           *const uint16_t).wrapping_add(1));
    return spiDevice[device as usize].errorCount as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusTransfer(mut bus: *const busDevice_t,
                                        mut txData: *const uint8_t,
                                        mut rxData: *mut uint8_t,
                                        mut length: libc::c_int) -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransfer((*bus).busdev_u.spi.instance, txData, rxData, length);
    IOHi((*bus).busdev_u.spi.csnPin);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiGetErrorCounter(mut instance: *mut SPI_TypeDef)
 -> uint16_t {
    let mut device: SPIDevice = spiDeviceByInstance(instance);
    if device as libc::c_int == SPIINVALID as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    return spiDevice[device as usize].errorCount;
}
#[no_mangle]
pub unsafe extern "C" fn spiResetErrorCounter(mut instance:
                                                  *mut SPI_TypeDef) {
    let mut device: SPIDevice = spiDeviceByInstance(instance);
    if device as libc::c_int != SPIINVALID as libc::c_int {
        ::core::ptr::write_volatile(&mut spiDevice[device as usize].errorCount
                                        as *mut uint16_t,
                                    0 as libc::c_int as uint16_t)
    };
}
#[no_mangle]
pub unsafe extern "C" fn spiBusWriteRegister(mut bus: *const busDevice_t,
                                             mut reg: uint8_t,
                                             mut data: uint8_t) -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransferByte((*bus).busdev_u.spi.instance, reg);
    spiTransferByte((*bus).busdev_u.spi.instance, data);
    IOHi((*bus).busdev_u.spi.csnPin);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusReadRegisterBuffer(mut bus: *const busDevice_t,
                                                  mut reg: uint8_t,
                                                  mut data: *mut uint8_t,
                                                  mut length: uint8_t)
 -> bool {
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransferByte((*bus).busdev_u.spi.instance,
                    (reg as libc::c_int | 0x80 as libc::c_int) as uint8_t);
    spiTransfer((*bus).busdev_u.spi.instance, 0 as *const uint8_t, data,
                length as libc::c_int);
    IOHi((*bus).busdev_u.spi.csnPin);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusReadRegister(mut bus: *const busDevice_t,
                                            mut reg: uint8_t) -> uint8_t {
    let mut data: uint8_t = 0;
    IOLo((*bus).busdev_u.spi.csnPin);
    spiTransferByte((*bus).busdev_u.spi.instance,
                    (reg as libc::c_int | 0x80 as libc::c_int) as uint8_t);
    spiTransfer((*bus).busdev_u.spi.instance, 0 as *const uint8_t, &mut data,
                1 as libc::c_int);
    IOHi((*bus).busdev_u.spi.csnPin);
    return data;
}
#[no_mangle]
pub unsafe extern "C" fn spiBusSetInstance(mut bus: *mut busDevice_t,
                                           mut instance: *mut SPI_TypeDef) {
    (*bus).bustype = BUSTYPE_SPI;
    (*bus).busdev_u.spi.instance = instance;
}
