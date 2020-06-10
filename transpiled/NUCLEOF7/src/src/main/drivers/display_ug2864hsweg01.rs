use ::libc;
extern "C" {
    #[no_mangle]
    fn i2cWrite(device: I2CDevice, addr_: uint8_t, reg: uint8_t,
                data: uint8_t) -> bool;
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
pub type size_t = libc::c_ulong;
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
pub type busDevice_t = busDevice_s;
// cached here for efficiency
// 0b00000000
#[no_mangle]
pub static mut CHAR_FORMAT: libc::c_uchar = 0 as libc::c_int as libc::c_uchar;
static mut multiWiiFont: [[uint8_t; 5]; 135] =
    [[0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x4f as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x14 as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t],
     [0x24 as libc::c_int as uint8_t, 0x2a as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0x2a as libc::c_int as uint8_t,
      0x12 as libc::c_int as uint8_t],
     [0x23 as libc::c_int as uint8_t, 0x13 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t,
      0x62 as libc::c_int as uint8_t],
     [0x36 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x55 as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
      0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x1c as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t, 0x1c as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x14 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x3e as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t],
     [0x8 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x3e as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x30 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x8 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x60 as libc::c_int as uint8_t,
      0x60 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x20 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x2 as libc::c_int as uint8_t],
     [0x3e as libc::c_int as uint8_t, 0x51 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x45 as libc::c_int as uint8_t,
      0x3e as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x42 as libc::c_int as uint8_t, 0x61 as libc::c_int as uint8_t,
      0x51 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t],
     [0x21 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x45 as libc::c_int as uint8_t, 0x4b as libc::c_int as uint8_t,
      0x31 as libc::c_int as uint8_t],
     [0x18 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x12 as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
      0x10 as libc::c_int as uint8_t],
     [0x27 as libc::c_int as uint8_t, 0x45 as libc::c_int as uint8_t,
      0x45 as libc::c_int as uint8_t, 0x45 as libc::c_int as uint8_t,
      0x39 as libc::c_int as uint8_t],
     [0x3c as libc::c_int as uint8_t, 0x4a as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x30 as libc::c_int as uint8_t],
     [0x1 as libc::c_int as uint8_t, 0x71 as libc::c_int as uint8_t,
      0x9 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
      0x3 as libc::c_int as uint8_t],
     [0x36 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t],
     [0x6 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x29 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x36 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x56 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x8 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x14 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t],
     [0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x51 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
      0x6 as libc::c_int as uint8_t],
     [0x32 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x79 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x3e as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x11 as libc::c_int as uint8_t,
      0x11 as libc::c_int as uint8_t, 0x11 as libc::c_int as uint8_t,
      0x7e as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t],
     [0x3e as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
      0x1c as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
      0x9 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t],
     [0x3e as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x7a as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x20 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t, 0x3f as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0xc as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t],
     [0x3e as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x3e as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
      0x9 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
      0x6 as libc::c_int as uint8_t],
     [0x3e as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x51 as libc::c_int as uint8_t, 0x21 as libc::c_int as uint8_t,
      0x5e as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
      0x19 as libc::c_int as uint8_t, 0x29 as libc::c_int as uint8_t,
      0x46 as libc::c_int as uint8_t],
     [0x46 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x31 as libc::c_int as uint8_t],
     [0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t],
     [0x3f as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x3f as libc::c_int as uint8_t],
     [0x1f as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0x1f as libc::c_int as uint8_t],
     [0x3f as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x38 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x3f as libc::c_int as uint8_t],
     [0x63 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x63 as libc::c_int as uint8_t],
     [0x7 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x70 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x7 as libc::c_int as uint8_t],
     [0x61 as libc::c_int as uint8_t, 0x51 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x45 as libc::c_int as uint8_t,
      0x43 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x2 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x41 as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x4 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t],
     [0x40 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t],
     [0x1 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x20 as libc::c_int as uint8_t, 0x54 as libc::c_int as uint8_t,
      0x54 as libc::c_int as uint8_t, 0x54 as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x48 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x38 as libc::c_int as uint8_t],
     [0x38 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t],
     [0x38 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t, 0x48 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t],
     [0x38 as libc::c_int as uint8_t, 0x54 as libc::c_int as uint8_t,
      0x54 as libc::c_int as uint8_t, 0x54 as libc::c_int as uint8_t,
      0x18 as libc::c_int as uint8_t],
     [0x8 as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x9 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x2 as libc::c_int as uint8_t],
     [0x6 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x49 as libc::c_int as uint8_t,
      0x3f as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x7d as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x20 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t, 0x3d as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x7f as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x7c as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x18 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x7c as libc::c_int as uint8_t],
     [0x7c as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t],
     [0x38 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x38 as libc::c_int as uint8_t],
     [0x7c as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t],
     [0x8 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x18 as libc::c_int as uint8_t,
      0x7c as libc::c_int as uint8_t],
     [0x7c as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t],
     [0x48 as libc::c_int as uint8_t, 0x54 as libc::c_int as uint8_t,
      0x54 as libc::c_int as uint8_t, 0x54 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t],
     [0x4 as libc::c_int as uint8_t, 0x3f as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t],
     [0x3c as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0x7c as libc::c_int as uint8_t],
     [0x1c as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0x1c as libc::c_int as uint8_t],
     [0x3c as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x30 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t],
     [0x44 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x10 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t],
     [0xc as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0x50 as libc::c_int as uint8_t,
      0x3c as libc::c_int as uint8_t],
     [0x44 as libc::c_int as uint8_t, 0x64 as libc::c_int as uint8_t,
      0x54 as libc::c_int as uint8_t, 0x4c as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x2 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x2 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x2 as libc::c_int as uint8_t],
     [0x3e as libc::c_int as uint8_t, 0x55 as libc::c_int as uint8_t,
      0x55 as libc::c_int as uint8_t, 0x41 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x79 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x18 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
      0x74 as libc::c_int as uint8_t, 0x2e as libc::c_int as uint8_t,
      0x24 as libc::c_int as uint8_t],
     [0x48 as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x49 as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t],
     [0x5d as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
      0x5d as libc::c_int as uint8_t],
     [0x15 as libc::c_int as uint8_t, 0x16 as libc::c_int as uint8_t,
      0x7c as libc::c_int as uint8_t, 0x16 as libc::c_int as uint8_t,
      0x15 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x77 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0xa as libc::c_int as uint8_t, 0x55 as libc::c_int as uint8_t,
      0x55 as libc::c_int as uint8_t, 0x55 as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0xd as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t],
     [0x8 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x2a as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x22 as libc::c_int as uint8_t],
     [0x4 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
      0x1c as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0x5 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x44 as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x5f as libc::c_int as uint8_t, 0x44 as libc::c_int as uint8_t,
      0x44 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
      0x1 as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t,
      0x3e as libc::c_int as uint8_t],
     [0x6 as libc::c_int as uint8_t, 0xf as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x7f as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x18 as libc::c_int as uint8_t,
      0x18 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x50 as libc::c_int as uint8_t, 0x20 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0xd as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x22 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x2a as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x8 as libc::c_int as uint8_t],
     [0x17 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x34 as libc::c_int as uint8_t, 0x2a as libc::c_int as uint8_t,
      0x7d as libc::c_int as uint8_t],
     [0x17 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
      0x4 as libc::c_int as uint8_t, 0x6a as libc::c_int as uint8_t,
      0x59 as libc::c_int as uint8_t],
     [0x30 as libc::c_int as uint8_t, 0x48 as libc::c_int as uint8_t,
      0x45 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t],
     [0x42 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x42 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x42 as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x7e as libc::c_int as uint8_t, 0x42 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t],
     [0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x7e as libc::c_int as uint8_t],
     [0x5a as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t],
     [0x5a as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x40 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
      0x5a as libc::c_int as uint8_t],
     [0x7a as libc::c_int as uint8_t, 0x60 as libc::c_int as uint8_t,
      0x60 as libc::c_int as uint8_t, 0x60 as libc::c_int as uint8_t,
      0x7a as libc::c_int as uint8_t],
     [0x7a as libc::c_int as uint8_t, 0x70 as libc::c_int as uint8_t,
      0x70 as libc::c_int as uint8_t, 0x70 as libc::c_int as uint8_t,
      0x7a as libc::c_int as uint8_t],
     [0x7a as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0x78 as libc::c_int as uint8_t, 0x78 as libc::c_int as uint8_t,
      0x7a as libc::c_int as uint8_t],
     [0x7a as libc::c_int as uint8_t, 0x7c as libc::c_int as uint8_t,
      0x7c as libc::c_int as uint8_t, 0x7c as libc::c_int as uint8_t,
      0x7a as libc::c_int as uint8_t],
     [0x7a as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x7e as libc::c_int as uint8_t, 0x7e as libc::c_int as uint8_t,
      0x7a as libc::c_int as uint8_t]];
unsafe extern "C" fn i2c_OLED_send_cmd(mut bus: *mut busDevice_t,
                                       mut command: uint8_t) -> bool {
    return i2cWrite((*bus).busdev_u.i2c.device, (*bus).busdev_u.i2c.address,
                    0x80 as libc::c_int as uint8_t, command);
}
unsafe extern "C" fn i2c_OLED_send_cmdarray(mut bus: *mut busDevice_t,
                                            mut commands: *const uint8_t,
                                            mut len: size_t) -> bool {
    let mut i: size_t = 0 as libc::c_int as size_t;
    while i < len {
        if !i2c_OLED_send_cmd(bus, *commands.offset(i as isize)) {
            return 0 as libc::c_int != 0
        }
        i = i.wrapping_add(1)
    }
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn i2c_OLED_send_byte(mut bus: *mut busDevice_t,
                                        mut val: uint8_t) -> bool {
    return i2cWrite((*bus).busdev_u.i2c.device, (*bus).busdev_u.i2c.address,
                    0x40 as libc::c_int as uint8_t, val);
}
#[no_mangle]
pub unsafe extern "C" fn i2c_OLED_clear_display_quick(mut bus:
                                                          *mut busDevice_t) {
    static mut i2c_OLED_cmd_clear_display_quick: [uint8_t; 4] =
        [0xb0 as libc::c_int as uint8_t, 0x40 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t];
    i2c_OLED_send_cmdarray(bus, i2c_OLED_cmd_clear_display_quick.as_ptr(),
                           (::core::mem::size_of::<[uint8_t; 4]>() as
                                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong));
    let mut i: uint16_t = 0 as libc::c_int as uint16_t;
    while (i as libc::c_int) < 1024 as libc::c_int {
        // fill the display's RAM with graphic... 128*64 pixel picture
        i2c_OLED_send_byte(bus, 0 as libc::c_int as uint8_t);
        i = i.wrapping_add(1)
        // clear
    };
}
#[no_mangle]
pub unsafe extern "C" fn i2c_OLED_clear_display(mut bus: *mut busDevice_t) {
    static mut i2c_OLED_cmd_clear_display_pre: [uint8_t; 4] =
        [0xa6 as libc::c_int as uint8_t, 0xae as libc::c_int as uint8_t,
         0x20 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    i2c_OLED_send_cmdarray(bus, i2c_OLED_cmd_clear_display_pre.as_ptr(),
                           (::core::mem::size_of::<[uint8_t; 4]>() as
                                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong));
    i2c_OLED_clear_display_quick(bus);
    static mut i2c_OLED_cmd_clear_display_post: [uint8_t; 3] =
        [0x81 as libc::c_int as uint8_t, 200 as libc::c_int as uint8_t,
         0xaf as libc::c_int as uint8_t];
    i2c_OLED_send_cmdarray(bus, i2c_OLED_cmd_clear_display_post.as_ptr(),
                           (::core::mem::size_of::<[uint8_t; 3]>() as
                                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong));
}
#[no_mangle]
pub unsafe extern "C" fn i2c_OLED_set_xy(mut bus: *mut busDevice_t,
                                         mut col: uint8_t, mut row: uint8_t) {
    let mut i2c_OLED_cmd_set_xy: [uint8_t; 3] =
        [(0xb0 as libc::c_int + row as libc::c_int) as uint8_t,
         (0 as libc::c_int +
              ((5 as libc::c_int + 1 as libc::c_int) * col as libc::c_int &
                   0xf as libc::c_int)) as uint8_t,
         (0x10 as libc::c_int +
              ((5 as libc::c_int + 1 as libc::c_int) * col as libc::c_int >>
                   4 as libc::c_int & 0xf as libc::c_int)) as uint8_t];
    i2c_OLED_send_cmdarray(bus, i2c_OLED_cmd_set_xy.as_mut_ptr(),
                           (::core::mem::size_of::<[uint8_t; 3]>() as
                                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong));
}
#[no_mangle]
pub unsafe extern "C" fn i2c_OLED_set_line(mut bus: *mut busDevice_t,
                                           mut row: uint8_t) {
    i2c_OLED_set_xy(bus, 0 as libc::c_int as uint8_t, row);
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
pub unsafe extern "C" fn i2c_OLED_send_char(mut bus: *mut busDevice_t,
                                            mut ascii: libc::c_uchar) {
    let mut i: libc::c_uchar = 0; // apply
    let mut buffer: uint8_t = 0;
    i = 0 as libc::c_int as libc::c_uchar;
    while (i as libc::c_int) < 5 as libc::c_int {
        buffer =
            multiWiiFont[(ascii as libc::c_int - 32 as libc::c_int) as
                             usize][i as usize];
        buffer =
            (buffer as libc::c_int ^ CHAR_FORMAT as libc::c_int) as uint8_t;
        i2c_OLED_send_byte(bus, buffer);
        i = i.wrapping_add(1)
    }
    i2c_OLED_send_byte(bus, CHAR_FORMAT);
    // the gap
}
#[no_mangle]
pub unsafe extern "C" fn i2c_OLED_send_string(mut bus: *mut busDevice_t,
                                              mut string:
                                                  *const libc::c_char) {
    // Sends a string of chars until null terminator
    while *string != 0 {
        i2c_OLED_send_char(bus, *string as libc::c_uchar);
        string = string.offset(1)
    };
}
/* *
* according to http://www.adafruit.com/datasheets/UG-2864HSWEG01.pdf Chapter 4.4 Page 15
*/
#[no_mangle]
pub unsafe extern "C" fn ug2864hsweg01InitI2C(mut bus: *mut busDevice_t)
 -> bool {
    // Set display OFF
    if !i2c_OLED_send_cmd(bus, 0xae as libc::c_int as uint8_t) {
        return 0 as libc::c_int != 0
    }
    static mut i2c_OLED_cmd_init: [uint8_t; 22] =
        [0xd4 as libc::c_int as uint8_t, 0x80 as libc::c_int as uint8_t,
         0xa8 as libc::c_int as uint8_t, 0x3f as libc::c_int as uint8_t,
         0xd3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x40 as libc::c_int as uint8_t, 0x8d as libc::c_int as uint8_t,
         0x14 as libc::c_int as uint8_t, 0xa1 as libc::c_int as uint8_t,
         0xc8 as libc::c_int as uint8_t, 0xda as libc::c_int as uint8_t,
         0x12 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
         0xcf as libc::c_int as uint8_t, 0xd9 as libc::c_int as uint8_t,
         0xf1 as libc::c_int as uint8_t, 0xdb as libc::c_int as uint8_t,
         0x40 as libc::c_int as uint8_t, 0xa4 as libc::c_int as uint8_t,
         0xa6 as libc::c_int as uint8_t, 0xaf as libc::c_int as uint8_t];
    i2c_OLED_send_cmdarray(bus, i2c_OLED_cmd_init.as_ptr(),
                           (::core::mem::size_of::<[uint8_t; 22]>() as
                                libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                as
                                                                libc::c_ulong));
    i2c_OLED_clear_display(bus);
    return 1 as libc::c_int != 0;
}
// USE_I2C_OLED_DISPLAY
