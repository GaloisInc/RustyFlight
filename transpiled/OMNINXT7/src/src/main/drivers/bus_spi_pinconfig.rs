use ::libc;
extern "C" {
    #[no_mangle]
    static mut spiDevice: [spiDevice_t; 4];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type size_t = libc::c_ulong;
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
pub type ioTag_t = uint8_t;
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
pub type rccPeriphTag_t = uint8_t;
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiPinConfig_s {
    pub ioTagSck: ioTag_t,
    pub ioTagMiso: ioTag_t,
    pub ioTagMosi: ioTag_t,
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
pub type spiPinConfig_t = spiPinConfig_s;
pub type spiHardware_t = spiHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiHardware_s {
    pub device: SPIDevice,
    pub reg: *mut SPI_TypeDef,
    pub sckPins: [spiPinDef_t; 4],
    pub misoPins: [spiPinDef_t; 4],
    pub mosiPins: [spiPinDef_t; 4],
    pub rcc: rccPeriphTag_t,
    pub dmaIrqHandler: uint8_t,
}
pub type spiPinDef_t = spiPinDef_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct spiPinDef_s {
    pub pin: ioTag_t,
    pub af: uint8_t,
}
pub const DMA2_ST1_HANDLER: C2RustUnnamed = 10;
pub const RCC_APB2: rcc_reg = 2;
pub const DMA1_ST7_HANDLER: C2RustUnnamed = 8;
pub const RCC_APB1: rcc_reg = 3;
pub const DMA1_ST4_HANDLER: C2RustUnnamed = 5;
pub const DMA2_ST3_HANDLER: C2RustUnnamed = 12;
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
pub type C2RustUnnamed = libc::c_uint;
pub const DMA_LAST_HANDLER: C2RustUnnamed = 16;
pub const DMA2_ST7_HANDLER: C2RustUnnamed = 16;
pub const DMA2_ST6_HANDLER: C2RustUnnamed = 15;
pub const DMA2_ST5_HANDLER: C2RustUnnamed = 14;
pub const DMA2_ST4_HANDLER: C2RustUnnamed = 13;
pub const DMA2_ST2_HANDLER: C2RustUnnamed = 11;
pub const DMA2_ST0_HANDLER: C2RustUnnamed = 9;
pub const DMA1_ST6_HANDLER: C2RustUnnamed = 7;
pub const DMA1_ST5_HANDLER: C2RustUnnamed = 6;
pub const DMA1_ST3_HANDLER: C2RustUnnamed = 4;
pub const DMA1_ST2_HANDLER: C2RustUnnamed = 3;
pub const DMA1_ST1_HANDLER: C2RustUnnamed = 2;
pub const DMA1_ST0_HANDLER: C2RustUnnamed = 1;
pub const DMA_NONE: C2RustUnnamed = 0;
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
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
// Initialized in run_static_initializers
#[no_mangle]
pub static mut spiHardware: [spiHardware_t; 4] =
    [spiHardware_t{device: SPIDEV_1,
                   reg: 0 as *mut SPI_TypeDef,
                   sckPins: [spiPinDef_t{pin: 0, af: 0,}; 4],
                   misoPins: [spiPinDef_t{pin: 0, af: 0,}; 4],
                   mosiPins: [spiPinDef_t{pin: 0, af: 0,}; 4],
                   rcc: 0,
                   dmaIrqHandler: 0,}; 4];
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
  Flash M25p16 tolerates 20mhz, SPI_CLOCK_FAST should sit around 20 or less.
*/
//00.42188 MHz
//06.57500 MHz
//13.50000 MHz
//54.00000 MHz
// Macros to convert between CLI bus number and SPIDevice.
// Size of SPI CS pre-initialization tag arrays
#[no_mangle]
pub unsafe extern "C" fn spiPinConfigure(mut pConfig: *const spiPinConfig_t) {
    let mut hwindex: size_t =
        0 as libc::c_int as size_t; // XXX Should be part of transfer context
    while hwindex <
              (::core::mem::size_of::<[spiHardware_t; 4]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<spiHardware_t>()
                                                   as libc::c_ulong) {
        let mut hw: *const spiHardware_t =
            &*spiHardware.as_ptr().offset(hwindex as isize) as
                *const spiHardware_t;
        if !(*hw).reg.is_null() {
            let mut device: SPIDevice = (*hw).device;
            let mut pDev: *mut spiDevice_t =
                &mut *spiDevice.as_mut_ptr().offset(device as isize) as
                    *mut spiDevice_t;
            let mut pindex: libc::c_int = 0 as libc::c_int;
            while pindex < 4 as libc::c_int {
                if (*pConfig.offset(device as isize)).ioTagSck as libc::c_int
                       == (*hw).sckPins[pindex as usize].pin as libc::c_int {
                    (*pDev).sck = (*hw).sckPins[pindex as usize].pin;
                    (*pDev).sckAF = (*hw).sckPins[pindex as usize].af
                }
                if (*pConfig.offset(device as isize)).ioTagMiso as libc::c_int
                       == (*hw).misoPins[pindex as usize].pin as libc::c_int {
                    (*pDev).miso = (*hw).misoPins[pindex as usize].pin;
                    (*pDev).misoAF = (*hw).misoPins[pindex as usize].af
                }
                if (*pConfig.offset(device as isize)).ioTagMosi as libc::c_int
                       == (*hw).mosiPins[pindex as usize].pin as libc::c_int {
                    (*pDev).mosi = (*hw).mosiPins[pindex as usize].pin;
                    (*pDev).mosiAF = (*hw).mosiPins[pindex as usize].af
                }
                pindex += 1
            }
            if (*pDev).sck as libc::c_int != 0 &&
                   (*pDev).miso as libc::c_int != 0 &&
                   (*pDev).mosi as libc::c_int != 0 {
                (*pDev).dev = (*hw).reg;
                (*pDev).rcc = (*hw).rcc;
                (*pDev).leadingEdge = 0 as libc::c_int != 0;
                (*pDev).dmaIrqHandler = (*hw).dmaIrqHandler
            }
        }
        hwindex = hwindex.wrapping_add(1)
    };
}
unsafe extern "C" fn run_static_initializers() {
    spiHardware =
        [{
             let mut init =
                 spiHardware_s{device: SPIDEV_1,
                               reg:
                                   (0x40000000 as
                                        libc::c_uint).wrapping_add(0x10000 as
                                                                       libc::c_uint).wrapping_add(0x3000
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((0 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 5 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 3 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((0 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 6 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 4 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((0 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 7 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 5 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               rcc:
                                   (((RCC_APB2 as libc::c_int) <<
                                         5 as libc::c_int) as libc::c_long |
                                        (16 as libc::c_int *
                                             (((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint) as
                                                  libc::c_long >
                                                  65535 as libc::c_long) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8 as libc::c_int *
                                                  (((0x1 as libc::c_uint) <<
                                                        12 as libc::c_uint) as
                                                       libc::c_long *
                                                       1 as libc::c_long >>
                                                       16 as libc::c_int *
                                                           (((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 12 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long >
                                                                65535 as
                                                                    libc::c_long)
                                                               as libc::c_int
                                                       >
                                                       255 as libc::c_int as
                                                           libc::c_long) as
                                                      libc::c_int) as
                                                 libc::c_long +
                                                 (8 as libc::c_int as
                                                      libc::c_long -
                                                      90 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 12 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          12
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          12
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   12
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               14 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               |
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long)
                                                      -
                                                      2 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 12 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          12
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          12
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   12
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,
                               dmaIrqHandler:
                                   DMA2_ST3_HANDLER as libc::c_int as
                                       uint8_t,};
             init
         },
         {
             let mut init =
                 spiHardware_s{device: SPIDEV_2,
                               reg:
                                   (0x40000000 as
                                        libc::c_uint).wrapping_add(0x3800 as
                                                                       libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((0 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 9 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 10 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 13 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 14 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((2 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 2 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 15 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((2 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 1 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((2 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 3 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,}],
                               rcc:
                                   (((RCC_APB1 as libc::c_int) <<
                                         5 as libc::c_int) as libc::c_long |
                                        (16 as libc::c_int *
                                             (((0x1 as libc::c_uint) <<
                                                   14 as libc::c_uint) as
                                                  libc::c_long >
                                                  65535 as libc::c_long) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8 as libc::c_int *
                                                  (((0x1 as libc::c_uint) <<
                                                        14 as libc::c_uint) as
                                                       libc::c_long *
                                                       1 as libc::c_long >>
                                                       16 as libc::c_int *
                                                           (((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 14 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long >
                                                                65535 as
                                                                    libc::c_long)
                                                               as libc::c_int
                                                       >
                                                       255 as libc::c_int as
                                                           libc::c_long) as
                                                      libc::c_int) as
                                                 libc::c_long +
                                                 (8 as libc::c_int as
                                                      libc::c_long -
                                                      90 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 14 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          14
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          14
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   14
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               14 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               |
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long)
                                                      -
                                                      2 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 14 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          14
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          14
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   14
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,
                               dmaIrqHandler:
                                   DMA1_ST4_HANDLER as libc::c_int as
                                       uint8_t,};
             init
         },
         {
             let mut init =
                 spiHardware_s{device: SPIDEV_3,
                               reg:
                                   (0x40000000 as
                                        libc::c_uint).wrapping_add(0x3c00 as
                                                                       libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 3 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x6 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((2 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 10 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x6 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 4 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x6 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((2 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 11 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x6 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 2 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x7 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 5 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x6 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            ((2 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 12 as
                                                                     libc::c_int)
                                                                as ioTag_t,
                                                        af:
                                                            0x6 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }],
                               rcc:
                                   (((RCC_APB1 as libc::c_int) <<
                                         5 as libc::c_int) as libc::c_long |
                                        (16 as libc::c_int *
                                             (((0x1 as libc::c_uint) <<
                                                   15 as libc::c_uint) as
                                                  libc::c_long >
                                                  65535 as libc::c_long) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8 as libc::c_int *
                                                  (((0x1 as libc::c_uint) <<
                                                        15 as libc::c_uint) as
                                                       libc::c_long *
                                                       1 as libc::c_long >>
                                                       16 as libc::c_int *
                                                           (((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 15 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long >
                                                                65535 as
                                                                    libc::c_long)
                                                               as libc::c_int
                                                       >
                                                       255 as libc::c_int as
                                                           libc::c_long) as
                                                      libc::c_int) as
                                                 libc::c_long +
                                                 (8 as libc::c_int as
                                                      libc::c_long -
                                                      90 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 15 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   15
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               14 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               |
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long)
                                                      -
                                                      2 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 15 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   15
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,
                               dmaIrqHandler:
                                   DMA1_ST7_HANDLER as libc::c_int as
                                       uint8_t,};
             init
         },
         {
             let mut init =
                 spiHardware_s{device: SPIDEV_4,
                               reg:
                                   (0x40000000 as
                                        libc::c_uint).wrapping_add(0x10000 as
                                                                       libc::c_uint).wrapping_add(0x3400
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut SPI_TypeDef,
                               sckPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               misoPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               mosiPins:
                                   [{
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            spiPinDef_s{pin:
                                                            0 as libc::c_int
                                                                as ioTag_t,
                                                        af:
                                                            0x5 as
                                                                libc::c_uint
                                                                as uint8_t,};
                                        init
                                    }, spiPinDef_t{pin: 0, af: 0,},
                                    spiPinDef_t{pin: 0, af: 0,}],
                               rcc:
                                   (((RCC_APB2 as libc::c_int) <<
                                         5 as libc::c_int) as libc::c_long |
                                        (16 as libc::c_int *
                                             (((0x1 as libc::c_uint) <<
                                                   13 as libc::c_uint) as
                                                  libc::c_long >
                                                  65535 as libc::c_long) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8 as libc::c_int *
                                                  (((0x1 as libc::c_uint) <<
                                                        13 as libc::c_uint) as
                                                       libc::c_long *
                                                       1 as libc::c_long >>
                                                       16 as libc::c_int *
                                                           (((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 13 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long >
                                                                65535 as
                                                                    libc::c_long)
                                                               as libc::c_int
                                                       >
                                                       255 as libc::c_int as
                                                           libc::c_long) as
                                                      libc::c_int) as
                                                 libc::c_long +
                                                 (8 as libc::c_int as
                                                      libc::c_long -
                                                      90 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 13 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          13
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          13
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   13
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               4 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               14 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               |
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long)
                                                      -
                                                      2 as libc::c_int as
                                                          libc::c_long /
                                                          ((((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 13 as
                                                                     libc::c_uint)
                                                                as
                                                                libc::c_long *
                                                                1 as
                                                                    libc::c_long
                                                                >>
                                                                16 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          13
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         >
                                                                         65535
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int
                                                                >>
                                                                8 as
                                                                    libc::c_int
                                                                    *
                                                                    (((0x1 as
                                                                           libc::c_uint)
                                                                          <<
                                                                          13
                                                                              as
                                                                              libc::c_uint)
                                                                         as
                                                                         libc::c_long
                                                                         *
                                                                         1 as
                                                                             libc::c_long
                                                                         >>
                                                                         16 as
                                                                             libc::c_int
                                                                             *
                                                                             (((0x1
                                                                                    as
                                                                                    libc::c_uint)
                                                                                   <<
                                                                                   13
                                                                                       as
                                                                                       libc::c_uint)
                                                                                  as
                                                                                  libc::c_long
                                                                                  >
                                                                                  65535
                                                                                      as
                                                                                      libc::c_long)
                                                                                 as
                                                                                 libc::c_int
                                                                         >
                                                                         255
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_long)
                                                                        as
                                                                        libc::c_int)
                                                               /
                                                               2 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long
                                                               +
                                                               1 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_long))))
                                       as rccPeriphTag_t,
                               dmaIrqHandler:
                                   DMA2_ST1_HANDLER as libc::c_int as
                                       uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
