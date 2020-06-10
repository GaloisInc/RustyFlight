use ::libc;
extern "C" {
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiTransfer(instance: *mut SPI_TypeDef, txData: *const uint8_t,
                   rxData: *mut uint8_t, len: libc::c_int) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
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
pub type flashType_e = libc::c_uint;
pub const FLASH_TYPE_NAND: flashType_e = 1;
pub const FLASH_TYPE_NOR: flashType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashGeometry_s {
    pub sectors: uint16_t,
    pub pageSize: uint16_t,
    pub sectorSize: uint32_t,
    pub totalSize: uint32_t,
    pub pagesPerSector: uint16_t,
    pub flashType: flashType_e,
}
pub type flashGeometry_t = flashGeometry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashVTable_s {
    pub isReady: Option<unsafe extern "C" fn(_: *mut flashDevice_t) -> bool>,
    pub waitForReady: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                  _: uint32_t) -> bool>,
    pub eraseSector: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                 _: uint32_t) -> ()>,
    pub eraseCompletely: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                    -> ()>,
    pub pageProgramBegin: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                      _: uint32_t) -> ()>,
    pub pageProgramContinue: Option<unsafe extern "C" fn(_:
                                                             *mut flashDevice_t,
                                                         _: *const uint8_t,
                                                         _: libc::c_int)
                                        -> ()>,
    pub pageProgramFinish: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                      -> ()>,
    pub pageProgram: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                                 _: uint32_t,
                                                 _: *const uint8_t,
                                                 _: libc::c_int) -> ()>,
    pub flush: Option<unsafe extern "C" fn(_: *mut flashDevice_t) -> ()>,
    pub readBytes: Option<unsafe extern "C" fn(_: *mut flashDevice_t,
                                               _: uint32_t, _: *mut uint8_t,
                                               _: libc::c_int)
                              -> libc::c_int>,
    pub getGeometry: Option<unsafe extern "C" fn(_: *mut flashDevice_t)
                                -> *const flashGeometry_t>,
}
// Slave I2C on SPI master
// Count of the number of erasable blocks on the device
// In bytes
// This is just pagesPerSector * pageSize
// This is just sectorSize * sectors
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
/*
 * Author: jflyper
 */
pub type flashDevice_t = flashDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct flashDevice_s {
    pub busdev: *mut busDevice_t,
    pub vTable: *const flashVTable_s,
    pub geometry: flashGeometry_t,
    pub currentWriteAddress: uint32_t,
    pub isLargeFlash: bool,
    pub couldBeBusy: bool,
}
pub type flashVTable_t = flashVTable_s;
// Whether we've performed an action that could have made the device busy
    // for writes. This allows us to avoid polling for writable status
    // when it is definitely ready already.
// millisecond time
pub type timeMs_t = uint32_t;
unsafe extern "C" fn m25p16_disable(mut bus: *mut busDevice_t) {
    IOHi((*bus).busdev_u.spi.csnPin);
    asm!("nop" : : : : "volatile");
}
unsafe extern "C" fn m25p16_enable(mut bus: *mut busDevice_t) {
    asm!("nop" : : : : "volatile");
    IOLo((*bus).busdev_u.spi.csnPin);
}
unsafe extern "C" fn m25p16_transfer(mut bus: *mut busDevice_t,
                                     mut txData: *const uint8_t,
                                     mut rxData: *mut uint8_t,
                                     mut len: libc::c_int) {
    m25p16_enable(bus);
    spiTransfer((*bus).busdev_u.spi.instance, txData, rxData, len);
    m25p16_disable(bus);
}
/* *
 * Send the given command byte to the device.
 */
unsafe extern "C" fn m25p16_performOneByteCommand(mut bus: *mut busDevice_t,
                                                  mut command: uint8_t) {
    m25p16_enable(bus);
    spiTransferByte((*bus).busdev_u.spi.instance, command);
    m25p16_disable(bus);
}
/* *
 * The flash requires this write enable command to be sent before commands that would cause
 * a write like program and erase.
 */
unsafe extern "C" fn m25p16_writeEnable(mut fdevice: *mut flashDevice_t) {
    m25p16_performOneByteCommand((*fdevice).busdev,
                                 0x6 as libc::c_int as uint8_t);
    // Assume that we're about to do some writing, so the device is just about to become busy
    (*fdevice).couldBeBusy = 1 as libc::c_int != 0;
}
unsafe extern "C" fn m25p16_readStatus(mut bus: *mut busDevice_t) -> uint8_t {
    let command: [uint8_t; 2] =
        [0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    let mut in_0: [uint8_t; 2] = [0; 2];
    m25p16_transfer(bus, command.as_ptr(), in_0.as_mut_ptr(),
                    ::core::mem::size_of::<[uint8_t; 2]>() as libc::c_ulong as
                        libc::c_int);
    return in_0[1 as libc::c_int as usize];
}
unsafe extern "C" fn m25p16_isReady(mut fdevice: *mut flashDevice_t) -> bool {
    // If couldBeBusy is false, don't bother to poll the flash chip for its status
    (*fdevice).couldBeBusy =
        (*fdevice).couldBeBusy as libc::c_int != 0 &&
            m25p16_readStatus((*fdevice).busdev) as libc::c_int &
                0x1 as libc::c_int != 0 as libc::c_int;
    return !(*fdevice).couldBeBusy;
}
unsafe extern "C" fn m25p16_waitForReady(mut fdevice: *mut flashDevice_t,
                                         mut timeoutMillis: uint32_t)
 -> bool {
    let mut time: uint32_t = millis();
    while !m25p16_isReady(fdevice) {
        if millis().wrapping_sub(time) > timeoutMillis {
            return 0 as libc::c_int != 0
        }
    }
    return 1 as libc::c_int != 0;
}
/* *
 * Read chip identification and geometry information (into global `geometry`).
 *
 * Returns true if we get valid ident, false if something bad happened like there is no M25P16.
 */
#[no_mangle]
pub unsafe extern "C" fn m25p16_detect(mut fdevice: *mut flashDevice_t,
                                       mut chipID: uint32_t) -> bool {
    match chipID {
        15679509 | 2105365 => {
            (*fdevice).geometry.sectors = 32 as libc::c_int as uint16_t;
            (*fdevice).geometry.pagesPerSector =
                256 as libc::c_int as uint16_t
        }
        15679510 | 12722198 => {
            (*fdevice).geometry.sectors = 64 as libc::c_int as uint16_t;
            (*fdevice).geometry.pagesPerSector =
                256 as libc::c_int as uint16_t
        }
        2144791 | 15679511 | 12722199 => {
            (*fdevice).geometry.sectors = 128 as libc::c_int as uint16_t;
            (*fdevice).geometry.pagesPerSector =
                256 as libc::c_int as uint16_t
        }
        2144792 | 15679512 | 90136 => {
            (*fdevice).geometry.sectors = 256 as libc::c_int as uint16_t;
            (*fdevice).geometry.pagesPerSector =
                256 as libc::c_int as uint16_t
        }
        15679513 | 12722201 => {
            (*fdevice).geometry.sectors = 512 as libc::c_int as uint16_t;
            (*fdevice).geometry.pagesPerSector =
                256 as libc::c_int as uint16_t
        }
        _ => {
            // Unsupported chip or not an SPI NOR flash
            (*fdevice).geometry.sectors =
                0 as libc::c_int as
                    uint16_t; // Just for luck we'll assume the chip could be busy even though it isn't specced to be
            (*fdevice).geometry.pagesPerSector = 0 as libc::c_int as uint16_t;
            (*fdevice).geometry.sectorSize = 0 as libc::c_int as uint32_t;
            (*fdevice).geometry.totalSize = 0 as libc::c_int as uint32_t;
            return 0 as libc::c_int != 0
        }
    }
    (*fdevice).geometry.flashType = FLASH_TYPE_NOR;
    (*fdevice).geometry.pageSize = 256 as libc::c_int as uint16_t;
    (*fdevice).geometry.sectorSize =
        ((*fdevice).geometry.pagesPerSector as libc::c_int *
             (*fdevice).geometry.pageSize as libc::c_int) as uint32_t;
    (*fdevice).geometry.totalSize =
        (*fdevice).geometry.sectorSize.wrapping_mul((*fdevice).geometry.sectors
                                                        as libc::c_uint);
    if (*fdevice).geometry.totalSize >
           (16 as libc::c_int * 1024 as libc::c_int * 1024 as libc::c_int) as
               libc::c_uint {
        (*fdevice).isLargeFlash = 1 as libc::c_int != 0;
        m25p16_performOneByteCommand((*fdevice).busdev,
                                     0xb7 as libc::c_int as uint8_t);
    }
    (*fdevice).couldBeBusy = 1 as libc::c_int != 0;
    (*fdevice).vTable = &m25p16_vTable;
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn m25p16_setCommandAddress(mut buf: *mut uint8_t,
                                              mut address: uint32_t,
                                              mut useLongAddress: bool) {
    if useLongAddress {
        let fresh0 = buf;
        buf = buf.offset(1);
        *fresh0 =
            (address >> 24 as libc::c_int &
                 0xff as libc::c_int as libc::c_uint) as uint8_t
    }
    let fresh1 = buf;
    buf = buf.offset(1);
    *fresh1 =
        (address >> 16 as libc::c_int & 0xff as libc::c_int as libc::c_uint)
            as uint8_t;
    let fresh2 = buf;
    buf = buf.offset(1);
    *fresh2 =
        (address >> 8 as libc::c_int & 0xff as libc::c_int as libc::c_uint) as
            uint8_t;
    *buf = (address & 0xff as libc::c_int as libc::c_uint) as uint8_t;
}
/* *
 * Erase a sector full of bytes to all 1's at the given byte offset in the flash chip.
 */
unsafe extern "C" fn m25p16_eraseSector(mut fdevice: *mut flashDevice_t,
                                        mut address: uint32_t) {
    let mut out: [uint8_t; 5] = [0xd8 as libc::c_int as uint8_t, 0, 0, 0, 0];
    m25p16_setCommandAddress(&mut *out.as_mut_ptr().offset(1 as libc::c_int as
                                                               isize),
                             address, (*fdevice).isLargeFlash);
    m25p16_waitForReady(fdevice, 5000 as libc::c_int as uint32_t);
    m25p16_writeEnable(fdevice);
    m25p16_transfer((*fdevice).busdev, out.as_mut_ptr(), 0 as *mut uint8_t,
                    ::core::mem::size_of::<[uint8_t; 5]>() as libc::c_ulong as
                        libc::c_int);
}
unsafe extern "C" fn m25p16_eraseCompletely(mut fdevice: *mut flashDevice_t) {
    m25p16_waitForReady(fdevice, 21000 as libc::c_int as uint32_t);
    m25p16_writeEnable(fdevice);
    m25p16_performOneByteCommand((*fdevice).busdev,
                                 0xc7 as libc::c_int as uint8_t);
}
unsafe extern "C" fn m25p16_pageProgramBegin(mut fdevice: *mut flashDevice_t,
                                             mut address: uint32_t) {
    (*fdevice).currentWriteAddress = address;
}
unsafe extern "C" fn m25p16_pageProgramContinue(mut fdevice:
                                                    *mut flashDevice_t,
                                                mut data: *const uint8_t,
                                                mut length: libc::c_int) {
    let mut command: [uint8_t; 5] =
        [0x2 as libc::c_int as uint8_t, 0, 0, 0, 0];
    m25p16_setCommandAddress(&mut *command.as_mut_ptr().offset(1 as
                                                                   libc::c_int
                                                                   as isize),
                             (*fdevice).currentWriteAddress,
                             (*fdevice).isLargeFlash);
    m25p16_waitForReady(fdevice, 6 as libc::c_int as uint32_t);
    m25p16_writeEnable(fdevice);
    m25p16_enable((*fdevice).busdev);
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance,
                command.as_mut_ptr(), 0 as *mut uint8_t,
                if (*fdevice).isLargeFlash as libc::c_int != 0 {
                    5 as libc::c_int
                } else { 4 as libc::c_int });
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance, data,
                0 as *mut uint8_t, length);
    m25p16_disable((*fdevice).busdev);
    (*fdevice).currentWriteAddress =
        ((*fdevice).currentWriteAddress as
             libc::c_uint).wrapping_add(length as libc::c_uint) as uint32_t as
            uint32_t;
}
unsafe extern "C" fn m25p16_pageProgramFinish(mut fdevice:
                                                  *mut flashDevice_t) {
}
/* *
 * Write bytes to a flash page. Address must not cross a page boundary.
 *
 * Bits can only be set to zero, not from zero back to one again. In order to set bits to 1, use the erase command.
 *
 * Length must be smaller than the page size.
 *
 * This will wait for the flash to become ready before writing begins.
 *
 * Datasheet indicates typical programming time is 0.8ms for 256 bytes, 0.2ms for 64 bytes, 0.05ms for 16 bytes.
 * (Although the maximum possible write time is noted as 5ms).
 *
 * If you want to write multiple buffers (whose sum of sizes is still not more than the page size) then you can
 * break this operation up into one beginProgram call, one or more continueProgram calls, and one finishProgram call.
 */
unsafe extern "C" fn m25p16_pageProgram(mut fdevice: *mut flashDevice_t,
                                        mut address: uint32_t,
                                        mut data: *const uint8_t,
                                        mut length: libc::c_int) {
    m25p16_pageProgramBegin(fdevice, address);
    m25p16_pageProgramContinue(fdevice, data, length);
    m25p16_pageProgramFinish(fdevice);
}
/* *
 * Read `length` bytes into the provided `buffer` from the flash starting from the given `address` (which need not lie
 * on a page boundary).
 *
 * Waits up to DEFAULT_TIMEOUT_MILLIS milliseconds for the flash to become ready before reading.
 *
 * The number of bytes actually read is returned, which can be zero if an error or timeout occurred.
 */
unsafe extern "C" fn m25p16_readBytes(mut fdevice: *mut flashDevice_t,
                                      mut address: uint32_t,
                                      mut buffer: *mut uint8_t,
                                      mut length: libc::c_int)
 -> libc::c_int {
    let mut command: [uint8_t; 5] =
        [0x3 as libc::c_int as uint8_t, 0, 0, 0, 0];
    m25p16_setCommandAddress(&mut *command.as_mut_ptr().offset(1 as
                                                                   libc::c_int
                                                                   as isize),
                             address, (*fdevice).isLargeFlash);
    if !m25p16_waitForReady(fdevice, 6 as libc::c_int as uint32_t) {
        return 0 as libc::c_int
    }
    m25p16_enable((*fdevice).busdev);
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance,
                command.as_mut_ptr(), 0 as *mut uint8_t,
                if (*fdevice).isLargeFlash as libc::c_int != 0 {
                    5 as libc::c_int
                } else { 4 as libc::c_int });
    spiTransfer((*(*fdevice).busdev).busdev_u.spi.instance,
                0 as *const uint8_t, buffer, length);
    m25p16_disable((*fdevice).busdev);
    return length;
}
/* *
 * Fetch information about the detected flash chip layout.
 *
 * Can be called before calling m25p16_init() (the result would have totalSize = 0).
 */
unsafe extern "C" fn m25p16_getGeometry(mut fdevice: *mut flashDevice_t)
 -> *const flashGeometry_t {
    return &mut (*fdevice).geometry;
}
#[no_mangle]
pub static mut m25p16_vTable: flashVTable_t =
    unsafe {
        {
            let mut init =
                flashVTable_s{isReady:
                                  Some(m25p16_isReady as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> bool),
                              waitForReady:
                                  Some(m25p16_waitForReady as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t)
                                               -> bool),
                              eraseSector:
                                  Some(m25p16_eraseSector as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t)
                                               -> ()),
                              eraseCompletely:
                                  Some(m25p16_eraseCompletely as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> ()),
                              pageProgramBegin:
                                  Some(m25p16_pageProgramBegin as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t)
                                               -> ()),
                              pageProgramContinue:
                                  Some(m25p16_pageProgramContinue as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _:
                                                                    *const uint8_t,
                                                                _:
                                                                    libc::c_int)
                                               -> ()),
                              pageProgramFinish:
                                  Some(m25p16_pageProgramFinish as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> ()),
                              pageProgram:
                                  Some(m25p16_pageProgram as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t,
                                                                _:
                                                                    *const uint8_t,
                                                                _:
                                                                    libc::c_int)
                                               -> ()),
                              flush: None,
                              readBytes:
                                  Some(m25p16_readBytes as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t,
                                                                _: uint32_t,
                                                                _:
                                                                    *mut uint8_t,
                                                                _:
                                                                    libc::c_int)
                                               -> libc::c_int),
                              getGeometry:
                                  Some(m25p16_getGeometry as
                                           unsafe extern "C" fn(_:
                                                                    *mut flashDevice_t)
                                               -> *const flashGeometry_t),};
            init
        }
    };
