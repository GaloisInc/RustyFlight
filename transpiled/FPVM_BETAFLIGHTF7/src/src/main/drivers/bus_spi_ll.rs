use ::libc;
extern "C" {
    #[no_mangle]
    fn ffs(__i: libc::c_int) -> libc::c_int;
    /* *
  * @}
  */
    /* * @defgroup SPI_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
    #[no_mangle]
    fn LL_SPI_DeInit(SPIx: *mut SPI_TypeDef) -> ErrorStatus;
    #[no_mangle]
    fn LL_SPI_Init(SPIx: *mut SPI_TypeDef,
                   SPI_InitStruct: *mut LL_SPI_InitTypeDef) -> ErrorStatus;
    #[no_mangle]
    fn spiTimeoutUserCallback(instance: *mut SPI_TypeDef) -> uint32_t;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    fn RCC_ResetCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
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
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_SPI_InitTypeDef {
    pub TransferDirection: uint32_t,
    pub Mode: uint32_t,
    pub DataWidth: uint32_t,
    pub ClockPolarity: uint32_t,
    pub ClockPhase: uint32_t,
    pub NSS: uint32_t,
    pub BaudRate: uint32_t,
    pub BitOrder: uint32_t,
    pub CRCCalculation: uint32_t,
    pub CRCPoly: uint32_t,
}
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
pub type spiDevice_t = SPIDevice_s;
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
/* !< FIFO transmission empty */
/* !< FIFO transmission 1/4   */
/* !< FIFO transmission 1/2   */
/* !< FIFO transmission full  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_DMA_PARITY DMA Parity
  * @{
  */
/* !< Select DMA parity Even */
/* !< Select DMA parity Odd  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup SPI_LL_Exported_Macros SPI Exported Macros
  * @{
  */
/* * @defgroup SPI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in SPI register
  * @param  __INSTANCE__ SPI Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in SPI register
  * @param  __INSTANCE__ SPI Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup SPI_LL_Exported_Functions SPI Exported Functions
  * @{
  */
/* * @defgroup SPI_LL_EF_Configuration Configuration
  * @{
  */
/* *
  * @brief  Enable SPI peripheral
  * @rmtoll CR1          SPE           LL_SPI_Enable
  * @param  SPIx SPI Instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_Enable(mut SPIx: *mut SPI_TypeDef) {
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         6 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Disable SPI peripheral
  * @note   When disabling the SPI, follow the procedure described in the Reference Manual.
  * @rmtoll CR1          SPE           LL_SPI_Disable
  * @param  SPIx SPI Instance
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_Disable(mut SPIx: *mut SPI_TypeDef) {
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Set baud rate prescaler
  * @note   These bits should not be changed when communication is ongoing. SPI BaudRate = fPCLK/Prescaler.
  * @rmtoll CR1          BR            LL_SPI_SetBaudRatePrescaler
  * @param  SPIx SPI Instance
  * @param  BaudRate This parameter can be one of the following values:
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV2
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV4
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV8
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV16
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV32
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV64
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV128
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV256
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_SetBaudRatePrescaler(mut SPIx: *mut SPI_TypeDef,
                                                 mut BaudRate: uint32_t) {
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint32_t,
                                (*SPIx).CR1 &
                                    !((0x7 as libc::c_uint) <<
                                          3 as libc::c_uint) | BaudRate);
}
/* *
  * @brief  Set threshold of RXFIFO that triggers an RXNE event
  * @rmtoll CR2          FRXTH         LL_SPI_SetRxFIFOThreshold
  * @param  SPIx SPI Instance
  * @param  Threshold This parameter can be one of the following values:
  *         @arg @ref LL_SPI_RX_FIFO_TH_HALF
  *         @arg @ref LL_SPI_RX_FIFO_TH_QUARTER
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_SetRxFIFOThreshold(mut SPIx: *mut SPI_TypeDef,
                                               mut Threshold: uint32_t) {
    ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint32_t,
                                (*SPIx).CR2 &
                                    !((0x1 as libc::c_uint) <<
                                          12 as libc::c_uint) | Threshold);
}
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_FLAG_Management FLAG Management
  * @{
  */
/* *
  * @brief  Check if Rx buffer is not empty
  * @rmtoll SR           RXNE          LL_SPI_IsActiveFlag_RXNE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_SPI_IsActiveFlag_RXNE(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return ((*SPIx).SR & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                (0x1 as libc::c_uint) << 0 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Check if Tx buffer is empty
  * @rmtoll SR           TXE           LL_SPI_IsActiveFlag_TXE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_SPI_IsActiveFlag_TXE(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return ((*SPIx).SR & (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Get busy flag
  * @note   The BSY flag is cleared under any one of the following conditions:
  * -When the SPI is correctly disabled
  * -When a fault is detected in Master mode (MODF bit set to 1)
  * -In Master mode, when it finishes a data transmission and no new data is ready to be
  * sent
  * -In Slave mode, when the BSY flag is set to '0' for at least one SPI clock cycle between
  * each data transfer.
  * @rmtoll SR           BSY           LL_SPI_IsActiveFlag_BSY
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
#[inline]
unsafe extern "C" fn LL_SPI_IsActiveFlag_BSY(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return ((*SPIx).SR & (0x1 as libc::c_uint) << 7 as libc::c_uint ==
                (0x1 as libc::c_uint) << 7 as libc::c_uint) as libc::c_int as
               uint32_t;
}
/* *
  * @brief  Get FIFO Transmission Level
  * @rmtoll SR           FTLVL         LL_SPI_GetTxFIFOLevel
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_TX_FIFO_EMPTY
  *         @arg @ref LL_SPI_TX_FIFO_QUARTER_FULL
  *         @arg @ref LL_SPI_TX_FIFO_HALF_FULL
  *         @arg @ref LL_SPI_TX_FIFO_FULL
  */
#[inline]
unsafe extern "C" fn LL_SPI_GetTxFIFOLevel(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return (*SPIx).SR & (0x3 as libc::c_uint) << 11 as libc::c_uint;
}
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_DATA_Management DATA Management
  * @{
  */
/* *
  * @brief  Read 8-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_ReceiveData8
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x00 and Max_Data=0xFF
  */
#[inline]
unsafe extern "C" fn LL_SPI_ReceiveData8(mut SPIx: *mut SPI_TypeDef)
 -> uint8_t {
    return (*SPIx).DR as uint8_t;
}
/* *
  * @brief  Read 16-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_ReceiveData16
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
#[inline]
unsafe extern "C" fn LL_SPI_ReceiveData16(mut SPIx: *mut SPI_TypeDef)
 -> uint16_t {
    return (*SPIx).DR as uint16_t;
}
/* *
  * @brief  Write 8-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_TransmitData8
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_TransmitData8(mut SPIx: *mut SPI_TypeDef,
                                          mut TxData: uint8_t) {
    ::core::ptr::write_volatile(&mut (*SPIx).DR as *mut uint32_t as
                                    *mut uint8_t, TxData);
}
/* *
  * @brief  Write 16-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_TransmitData16
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_SPI_TransmitData16(mut SPIx: *mut SPI_TypeDef,
                                           mut TxData: uint16_t) {
    ::core::ptr::write_volatile(&mut (*SPIx).DR as *mut uint32_t as
                                    *mut uint16_t, TxData);
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
pub unsafe extern "C" fn spiInitDevice(mut device: SPIDevice) {
    let mut spi: *mut spiDevice_t =
        &mut *spiDevice.as_mut_ptr().offset(device as isize) as
            *mut spiDevice_t;
    // Enable SPI clock
    RCC_ClockCmd((*spi).rcc, ENABLE);
    RCC_ResetCmd((*spi).rcc, ENABLE);
    IOInit(IOGetByTag((*spi).sck), OWNER_SPI_SCK,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOInit(IOGetByTag((*spi).miso), OWNER_SPI_MISO,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOInit(IOGetByTag((*spi).mosi), OWNER_SPI_MOSI,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    if (*spi).leadingEdge as libc::c_int == 1 as libc::c_int {
        IOConfigGPIOAF(IOGetByTag((*spi).sck),
                       (0x2 as libc::c_uint |
                            (0x3 as libc::c_uint) << 2 as libc::c_int |
                            (0x2 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t, (*spi).sckAF);
    } else {
        IOConfigGPIOAF(IOGetByTag((*spi).sck),
                       (0x2 as libc::c_uint |
                            (0x3 as libc::c_uint) << 2 as libc::c_int |
                            (0x1 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t, (*spi).sckAF);
    }
    IOConfigGPIOAF(IOGetByTag((*spi).miso),
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0x1 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, (*spi).misoAF);
    IOConfigGPIOAF(IOGetByTag((*spi).mosi),
                   (0x2 as libc::c_uint |
                        (0x3 as libc::c_uint) << 2 as libc::c_int |
                        (0 as libc::c_uint) << 5 as libc::c_int) as
                       ioConfig_t, (*spi).mosiAF);
    LL_SPI_Disable((*spi).dev);
    LL_SPI_DeInit((*spi).dev);
    let mut init: LL_SPI_InitTypeDef =
        {
            let mut init =
                LL_SPI_InitTypeDef{TransferDirection: 0 as libc::c_uint,
                                   Mode:
                                       (0x1 as libc::c_uint) <<
                                           2 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               8 as libc::c_uint,
                                   DataWidth: 0x700 as libc::c_uint,
                                   ClockPolarity:
                                       if (*spi).leadingEdge as libc::c_int !=
                                              0 {
                                           0 as libc::c_uint
                                       } else {
                                           ((0x1 as libc::c_uint)) <<
                                               1 as libc::c_uint
                                       },
                                   ClockPhase:
                                       if (*spi).leadingEdge as libc::c_int !=
                                              0 {
                                           0 as libc::c_uint
                                       } else {
                                           ((0x1 as libc::c_uint)) <<
                                               0 as libc::c_uint
                                       },
                                   NSS:
                                       (0x1 as libc::c_uint) <<
                                           9 as libc::c_uint,
                                   BaudRate: 0x10 as libc::c_uint,
                                   BitOrder: 0 as libc::c_uint,
                                   CRCCalculation: 0 as libc::c_uint,
                                   CRCPoly: 7 as libc::c_int as uint32_t,};
            init
        };
    LL_SPI_SetRxFIFOThreshold((*spi).dev,
                              (0x1 as libc::c_uint) << 12 as libc::c_uint);
    LL_SPI_Init((*spi).dev, &mut init);
    LL_SPI_Enable((*spi).dev);
}
#[no_mangle]
pub unsafe extern "C" fn spiTransferByte(mut instance: *mut SPI_TypeDef,
                                         mut txByte: uint8_t) -> uint8_t {
    let mut spiTimeout: uint16_t = 1000 as libc::c_int as uint16_t;
    while LL_SPI_IsActiveFlag_TXE(instance) == 0 {
        let fresh0 = spiTimeout;
        spiTimeout = spiTimeout.wrapping_sub(1);
        if fresh0 as libc::c_int == 0 as libc::c_int {
            return spiTimeoutUserCallback(instance) as uint8_t
        }
    }
    LL_SPI_TransmitData8(instance, txByte);
    spiTimeout = 1000 as libc::c_int as uint16_t;
    while LL_SPI_IsActiveFlag_RXNE(instance) == 0 {
        let fresh1 = spiTimeout;
        spiTimeout = spiTimeout.wrapping_sub(1);
        if fresh1 as libc::c_int == 0 as libc::c_int {
            return spiTimeoutUserCallback(instance) as uint8_t
        }
    }
    return LL_SPI_ReceiveData8(instance);
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
/*
  Flash M25p16 tolerates 20mhz, SPI_CLOCK_FAST should sit around 20 or less.
*/
//00.42188 MHz
//06.57500 MHz
//13.50000 MHz
//54.00000 MHz
// Macros to convert between CLI bus number and SPIDevice.
// Size of SPI CS pre-initialization tag arrays
/* *
 * Return true if the bus is currently in the middle of a transmission.
 */
#[no_mangle]
pub unsafe extern "C" fn spiIsBusBusy(mut instance: *mut SPI_TypeDef)
 -> bool {
    return LL_SPI_GetTxFIFOLevel(instance) != 0 as libc::c_uint ||
               LL_SPI_IsActiveFlag_BSY(instance) != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiTransfer(mut instance: *mut SPI_TypeDef,
                                     mut txData: *const uint8_t,
                                     mut rxData: *mut uint8_t,
                                     mut len: libc::c_int) -> bool {
    // set 16-bit transfer
    ::core::ptr::write_volatile(&mut (*instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           12 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    while len > 1 as libc::c_int {
        let mut spiTimeout: libc::c_int = 1000 as libc::c_int;
        while LL_SPI_IsActiveFlag_TXE(instance) == 0 {
            let fresh2 = spiTimeout;
            spiTimeout = spiTimeout - 1;
            if fresh2 == 0 as libc::c_int {
                return spiTimeoutUserCallback(instance) != 0
            }
        }
        let mut w: uint16_t = 0;
        if !txData.is_null() {
            w = *(txData as *mut uint16_t);
            txData = txData.offset(2 as libc::c_int as isize)
        } else { w = 0xffff as libc::c_int as uint16_t }
        LL_SPI_TransmitData16(instance, w);
        spiTimeout = 1000 as libc::c_int;
        while LL_SPI_IsActiveFlag_RXNE(instance) == 0 {
            let fresh3 = spiTimeout;
            spiTimeout = spiTimeout - 1;
            if fresh3 == 0 as libc::c_int {
                return spiTimeoutUserCallback(instance) != 0
            }
        }
        w = LL_SPI_ReceiveData16(instance);
        if !rxData.is_null() {
            *(rxData as *mut uint16_t) = w;
            rxData = rxData.offset(2 as libc::c_int as isize)
        }
        len -= 2 as libc::c_int
    }
    // set 8-bit transfer
    ::core::ptr::write_volatile(&mut (*instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         12 as libc::c_uint) as uint32_t as
                                    uint32_t);
    if len != 0 {
        let mut spiTimeout_0: libc::c_int = 1000 as libc::c_int;
        while LL_SPI_IsActiveFlag_TXE(instance) == 0 {
            let fresh4 = spiTimeout_0;
            spiTimeout_0 = spiTimeout_0 - 1;
            if fresh4 == 0 as libc::c_int {
                return spiTimeoutUserCallback(instance) != 0
            }
        }
        let mut b: uint8_t =
            if !txData.is_null() {
                let fresh5 = txData;
                txData = txData.offset(1);
                *fresh5 as libc::c_int
            } else { 0xff as libc::c_int } as uint8_t;
        LL_SPI_TransmitData8(instance, b);
        spiTimeout_0 = 1000 as libc::c_int;
        while LL_SPI_IsActiveFlag_RXNE(instance) == 0 {
            let fresh6 = spiTimeout_0;
            spiTimeout_0 = spiTimeout_0 - 1;
            if fresh6 == 0 as libc::c_int {
                return spiTimeoutUserCallback(instance) != 0
            }
        }
        b = LL_SPI_ReceiveData8(instance);
        if !rxData.is_null() {
            let fresh7 = rxData;
            rxData = rxData.offset(1);
            *fresh7 = b
        }
        len -= 1
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn spiSetDivisor(mut instance: *mut SPI_TypeDef,
                                       mut divisor: uint16_t) {
    // SPI2 and SPI3 are on APB1/AHB1 which PCLK is half that of APB2/AHB2.
    if instance ==
           (0x40000000 as libc::c_uint).wrapping_add(0x3800 as libc::c_uint)
               as *mut SPI_TypeDef ||
           instance ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x3c00 as libc::c_uint) as
                   *mut SPI_TypeDef {
        divisor = (divisor as libc::c_int / 2 as libc::c_int) as uint16_t
        // Safe for divisor == 0 or 1
    }
    LL_SPI_Disable(instance);
    LL_SPI_SetBaudRatePrescaler(instance,
                                if divisor as libc::c_int != 0 {
                                    ((ffs(divisor as libc::c_int |
                                              0x100 as libc::c_int) -
                                          2 as libc::c_int)) <<
                                        3 as libc::c_uint
                                } else { 0 as libc::c_int } as uint32_t);
    LL_SPI_Enable(instance);
}
