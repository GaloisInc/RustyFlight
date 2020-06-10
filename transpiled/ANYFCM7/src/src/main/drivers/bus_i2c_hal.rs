use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    /* *
  ******************************************************************************
  * @file    stm32f7xx_hal_i2c_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of I2C HAL Extended module.
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
    /* * @addtogroup I2CEx
  * @{
  */
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
    /* * @defgroup I2CEx_Exported_Constants I2C Extended Exported Constants
  * @{
  */
    /* * @defgroup I2CEx_Analog_Filter I2C Extended Analog Filter
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2CEx_FastModePlus I2C Extended Fast Mode Plus
  * @{
  */
    /* !< Fast Mode Plus not supported       */
    /* !< Enable Fast Mode Plus on PB6       */
    /* !< Enable Fast Mode Plus on PB7       */
    /* !< Enable Fast Mode Plus on PB8       */
    /* !< Enable Fast Mode Plus on PB9       */
    /* !< Enable Fast Mode Plus on I2C1 pins */
    /* !< Enable Fast Mode Plus on I2C2 pins */
    /* !< Enable Fast Mode Plus on I2C3 pins */
    /* !< Fast Mode Plus I2C4 not supported  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
    /* * @addtogroup I2CEx_Exported_Functions I2C Extended Exported Functions
  * @{
  */
    /* * @addtogroup I2CEx_Exported_Functions_Group1 Extended features functions
  * @brief    Extended features functions
  * @{
  */
    /* Peripheral Control functions  ************************************************/
    #[no_mangle]
    fn HAL_I2CEx_ConfigAnalogFilter(hi2c: *mut I2C_HandleTypeDef,
                                    AnalogFilter: uint32_t)
     -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
    /* * @defgroup I2C_Exported_Constants I2C Exported Constants
  * @{
  */
    /* * @defgroup I2C_XFEROPTIONS  I2C Sequential Transfer Options
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_ADDRESSING_MODE I2C Addressing Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_DUAL_ADDRESSING_MODE I2C Dual Addressing Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_OWN_ADDRESS2_MASKS I2C Own Address2 Masks
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_GENERAL_CALL_ADDRESSING_MODE I2C General Call Addressing Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_NOSTRETCH_MODE I2C No-Stretch Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_MEMORY_ADDRESS_SIZE I2C Memory Address Size
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_XFERDIRECTION I2C Transfer Direction Master Point of View
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_RELOAD_END_MODE I2C Reload End Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_START_STOP_MODE I2C Start or Stop Mode
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_Interrupt_configuration_definition I2C Interrupt configuration definition
  * @brief I2C Interrupt definition
  *        Elements values convention: 0xXXXXXXXX
  *           - XXXXXXXX  : Interrupt control mask
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup I2C_Flag_definition I2C Flag definition
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macros -----------------------------------------------------------*/
    /* * @defgroup I2C_Exported_Macros I2C Exported Macros
  * @{
  */
    /* * @brief Reset I2C handle state.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
    /* * @brief  Enable the specified I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *        This parameter can be one of the following values:
  *            @arg @ref I2C_IT_ERRI  Errors interrupt enable
  *            @arg @ref I2C_IT_TCI   Transfer complete interrupt enable
  *            @arg @ref I2C_IT_STOPI STOP detection interrupt enable
  *            @arg @ref I2C_IT_NACKI NACK received interrupt enable
  *            @arg @ref I2C_IT_ADDRI Address match interrupt enable
  *            @arg @ref I2C_IT_RXI   RX interrupt enable
  *            @arg @ref I2C_IT_TXI   TX interrupt enable
  *
  * @retval None
  */
    /* * @brief  Disable the specified I2C interrupt.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *        This parameter can be one of the following values:
  *            @arg @ref I2C_IT_ERRI  Errors interrupt enable
  *            @arg @ref I2C_IT_TCI   Transfer complete interrupt enable
  *            @arg @ref I2C_IT_STOPI STOP detection interrupt enable
  *            @arg @ref I2C_IT_NACKI NACK received interrupt enable
  *            @arg @ref I2C_IT_ADDRI Address match interrupt enable
  *            @arg @ref I2C_IT_RXI   RX interrupt enable
  *            @arg @ref I2C_IT_TXI   TX interrupt enable
  *
  * @retval None
  */
    /* * @brief  Check whether the specified I2C interrupt source is enabled or not.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __INTERRUPT__ specifies the I2C interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref I2C_IT_ERRI  Errors interrupt enable
  *            @arg @ref I2C_IT_TCI   Transfer complete interrupt enable
  *            @arg @ref I2C_IT_STOPI STOP detection interrupt enable
  *            @arg @ref I2C_IT_NACKI NACK received interrupt enable
  *            @arg @ref I2C_IT_ADDRI Address match interrupt enable
  *            @arg @ref I2C_IT_RXI   RX interrupt enable
  *            @arg @ref I2C_IT_TXI   TX interrupt enable
  *
  * @retval The new state of __INTERRUPT__ (SET or RESET).
  */
    /* * @brief  Check whether the specified I2C flag is set or not.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg @ref I2C_FLAG_TXE     Transmit data register empty
  *            @arg @ref I2C_FLAG_TXIS    Transmit interrupt status
  *            @arg @ref I2C_FLAG_RXNE    Receive data register not empty
  *            @arg @ref I2C_FLAG_ADDR    Address matched (slave mode)
  *            @arg @ref I2C_FLAG_AF      Acknowledge failure received flag
  *            @arg @ref I2C_FLAG_STOPF   STOP detection flag
  *            @arg @ref I2C_FLAG_TC      Transfer complete (master mode)
  *            @arg @ref I2C_FLAG_TCR     Transfer complete reload
  *            @arg @ref I2C_FLAG_BERR    Bus error
  *            @arg @ref I2C_FLAG_ARLO    Arbitration lost
  *            @arg @ref I2C_FLAG_OVR     Overrun/Underrun
  *            @arg @ref I2C_FLAG_PECERR  PEC error in reception
  *            @arg @ref I2C_FLAG_TIMEOUT Timeout or Tlow detection flag
  *            @arg @ref I2C_FLAG_ALERT   SMBus alert
  *            @arg @ref I2C_FLAG_BUSY    Bus busy
  *            @arg @ref I2C_FLAG_DIR     Transfer direction (slave mode)
  *
  * @retval The new state of __FLAG__ (SET or RESET).
  */
    /* * @brief  Clear the I2C pending flags which are cleared by writing 1 in a specific bit.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @param  __FLAG__ specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg @ref I2C_FLAG_TXE     Transmit data register empty
  *            @arg @ref I2C_FLAG_ADDR    Address matched (slave mode)
  *            @arg @ref I2C_FLAG_AF      Acknowledge failure received flag
  *            @arg @ref I2C_FLAG_STOPF   STOP detection flag
  *            @arg @ref I2C_FLAG_BERR    Bus error
  *            @arg @ref I2C_FLAG_ARLO    Arbitration lost
  *            @arg @ref I2C_FLAG_OVR     Overrun/Underrun
  *            @arg @ref I2C_FLAG_PECERR  PEC error in reception
  *            @arg @ref I2C_FLAG_TIMEOUT Timeout or Tlow detection flag
  *            @arg @ref I2C_FLAG_ALERT   SMBus alert
  *
  * @retval None
  */
    /* * @brief  Enable the specified I2C peripheral.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
    /* * @brief  Disable the specified I2C peripheral.
  * @param  __HANDLE__ specifies the I2C Handle.
  * @retval None
  */
    /* * @brief  Generate a Non-Acknowledge I2C peripheral in Slave mode.
  * @param  __HANDLE__: specifies the I2C Handle. 
  * @retval None
  */
    /* *
  * @}
  */
    /* Include I2C HAL Extended module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup I2C_Exported_Functions
  * @{
  */
    /* * @addtogroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
/* Initialization and de-initialization functions******************************/
    #[no_mangle]
    fn HAL_I2C_Init(hi2c: *mut I2C_HandleTypeDef) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* IO operation functions  ****************************************************/
 /* ****** Blocking mode: Polling */
    #[no_mangle]
    fn HAL_I2C_Master_Transmit(hi2c: *mut I2C_HandleTypeDef,
                               DevAddress: uint16_t, pData: *mut uint8_t,
                               Size: uint16_t, Timeout: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_I2C_Master_Receive(hi2c: *mut I2C_HandleTypeDef,
                              DevAddress: uint16_t, pData: *mut uint8_t,
                              Size: uint16_t, Timeout: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_I2C_Mem_Write(hi2c: *mut I2C_HandleTypeDef, DevAddress: uint16_t,
                         MemAddress: uint16_t, MemAddSize: uint16_t,
                         pData: *mut uint8_t, Size: uint16_t,
                         Timeout: uint32_t) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_I2C_Mem_Read(hi2c: *mut I2C_HandleTypeDef, DevAddress: uint16_t,
                        MemAddress: uint16_t, MemAddSize: uint16_t,
                        pData: *mut uint8_t, Size: uint16_t,
                        Timeout: uint32_t) -> HAL_StatusTypeDef;
    /* *
  * @}
  */
    /* * @addtogroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */
/* ****** I2C IRQHandler and Callbacks used in non blocking modes (Interrupt and DMA) */
    #[no_mangle]
    fn HAL_I2C_EV_IRQHandler(hi2c: *mut I2C_HandleTypeDef);
    #[no_mangle]
    fn HAL_I2C_ER_IRQHandler(hi2c: *mut I2C_HandleTypeDef);
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
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
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn_Type = libc::c_int;
pub const SDMMC2_IRQn: IRQn_Type = 103;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const OTG_HS_IRQn: IRQn_Type = 77;
pub const OTG_HS_WKUP_IRQn: IRQn_Type = 76;
pub const OTG_HS_EP1_IN_IRQn: IRQn_Type = 75;
pub const OTG_HS_EP1_OUT_IRQn: IRQn_Type = 74;
pub const I2C3_ER_IRQn: IRQn_Type = 73;
pub const I2C3_EV_IRQn: IRQn_Type = 72;
pub const USART6_IRQn: IRQn_Type = 71;
pub const DMA2_Stream7_IRQn: IRQn_Type = 70;
pub const DMA2_Stream6_IRQn: IRQn_Type = 69;
pub const DMA2_Stream5_IRQn: IRQn_Type = 68;
pub const OTG_FS_IRQn: IRQn_Type = 67;
pub const ETH_WKUP_IRQn: IRQn_Type = 62;
pub const ETH_IRQn: IRQn_Type = 61;
pub const DMA2_Stream4_IRQn: IRQn_Type = 60;
pub const DMA2_Stream3_IRQn: IRQn_Type = 59;
pub const DMA2_Stream2_IRQn: IRQn_Type = 58;
pub const DMA2_Stream1_IRQn: IRQn_Type = 57;
pub const DMA2_Stream0_IRQn: IRQn_Type = 56;
pub const TIM7_IRQn: IRQn_Type = 55;
pub const TIM6_DAC_IRQn: IRQn_Type = 54;
pub const UART5_IRQn: IRQn_Type = 53;
pub const UART4_IRQn: IRQn_Type = 52;
pub const SPI3_IRQn: IRQn_Type = 51;
pub const TIM5_IRQn: IRQn_Type = 50;
pub const SDMMC1_IRQn: IRQn_Type = 49;
pub const FMC_IRQn: IRQn_Type = 48;
pub const DMA1_Stream7_IRQn: IRQn_Type = 47;
pub const TIM8_CC_IRQn: IRQn_Type = 46;
pub const TIM8_TRG_COM_TIM14_IRQn: IRQn_Type = 45;
pub const TIM8_UP_TIM13_IRQn: IRQn_Type = 44;
pub const TIM8_BRK_TIM12_IRQn: IRQn_Type = 43;
pub const OTG_FS_WKUP_IRQn: IRQn_Type = 42;
pub const RTC_Alarm_IRQn: IRQn_Type = 41;
pub const EXTI15_10_IRQn: IRQn_Type = 40;
pub const USART3_IRQn: IRQn_Type = 39;
pub const USART2_IRQn: IRQn_Type = 38;
pub const USART1_IRQn: IRQn_Type = 37;
pub const SPI2_IRQn: IRQn_Type = 36;
pub const SPI1_IRQn: IRQn_Type = 35;
pub const I2C2_ER_IRQn: IRQn_Type = 34;
pub const I2C2_EV_IRQn: IRQn_Type = 33;
pub const I2C1_ER_IRQn: IRQn_Type = 32;
pub const I2C1_EV_IRQn: IRQn_Type = 31;
pub const TIM4_IRQn: IRQn_Type = 30;
pub const TIM3_IRQn: IRQn_Type = 29;
pub const TIM2_IRQn: IRQn_Type = 28;
pub const TIM1_CC_IRQn: IRQn_Type = 27;
pub const TIM1_TRG_COM_TIM11_IRQn: IRQn_Type = 26;
pub const TIM1_UP_TIM10_IRQn: IRQn_Type = 25;
pub const TIM1_BRK_TIM9_IRQn: IRQn_Type = 24;
pub const EXTI9_5_IRQn: IRQn_Type = 23;
pub const CAN1_SCE_IRQn: IRQn_Type = 22;
pub const CAN1_RX1_IRQn: IRQn_Type = 21;
pub const CAN1_RX0_IRQn: IRQn_Type = 20;
pub const CAN1_TX_IRQn: IRQn_Type = 19;
pub const ADC_IRQn: IRQn_Type = 18;
pub const DMA1_Stream6_IRQn: IRQn_Type = 17;
pub const DMA1_Stream5_IRQn: IRQn_Type = 16;
pub const DMA1_Stream4_IRQn: IRQn_Type = 15;
pub const DMA1_Stream3_IRQn: IRQn_Type = 14;
pub const DMA1_Stream2_IRQn: IRQn_Type = 13;
pub const DMA1_Stream1_IRQn: IRQn_Type = 12;
pub const DMA1_Stream0_IRQn: IRQn_Type = 11;
pub const EXTI4_IRQn: IRQn_Type = 10;
pub const EXTI3_IRQn: IRQn_Type = 9;
pub const EXTI2_IRQn: IRQn_Type = 8;
pub const EXTI1_IRQn: IRQn_Type = 7;
pub const EXTI0_IRQn: IRQn_Type = 6;
pub const RCC_IRQn: IRQn_Type = 5;
pub const FLASH_IRQn: IRQn_Type = 4;
pub const RTC_WKUP_IRQn: IRQn_Type = 3;
pub const TAMP_STAMP_IRQn: IRQn_Type = 2;
pub const PVD_IRQn: IRQn_Type = 1;
pub const WWDG_IRQn: IRQn_Type = 0;
pub const SysTick_IRQn: IRQn_Type = -1;
pub const PendSV_IRQn: IRQn_Type = -2;
pub const DebugMonitor_IRQn: IRQn_Type = -4;
pub const SVCall_IRQn: IRQn_Type = -5;
pub const UsageFault_IRQn: IRQn_Type = -10;
pub const BusFault_IRQn: IRQn_Type = -11;
pub const MemoryManagement_IRQn: IRQn_Type = -12;
pub const NonMaskableInt_IRQn: IRQn_Type = -14;
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
  * @brief Inter-integrated Circuit Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub OAR1: uint32_t,
    pub OAR2: uint32_t,
    pub TIMINGR: uint32_t,
    pub TIMEOUTR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub PECR: uint32_t,
    pub RXDR: uint32_t,
    pub TXDR: uint32_t,
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_def.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains HAL common defines, enumeration, macros and 
  *          structures definitions. 
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
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  HAL Status structures definition  
  */
pub type HAL_StatusTypeDef = libc::c_uint;
pub const HAL_TIMEOUT: HAL_StatusTypeDef = 3;
pub const HAL_BUSY: HAL_StatusTypeDef = 2;
pub const HAL_ERROR: HAL_StatusTypeDef = 1;
pub const HAL_OK: HAL_StatusTypeDef = 0;
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
  * @file    stm32f7xx_hal_i2c.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of I2C HAL module.
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
/* * @addtogroup I2C
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup I2C_Exported_Types I2C Exported Types
  * @{
  */
/* * @defgroup I2C_Configuration_Structure_definition I2C Configuration Structure definition
  * @brief  I2C Configuration Structure definition  
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub Timing: uint32_t,
    pub OwnAddress1: uint32_t,
    pub AddressingMode: uint32_t,
    pub DualAddressMode: uint32_t,
    pub OwnAddress2: uint32_t,
    pub OwnAddress2Masks: uint32_t,
    pub GeneralCallMode: uint32_t,
    pub NoStretchMode: uint32_t,
}
/* * 
  * @}
  */
/* * @defgroup HAL_state_structure_definition HAL state structure definition
  * @brief  HAL State structure definition
  * @note  HAL I2C State value coding follow below described bitmap :\n
  *          b7-b6  Error information\n
  *             00 : No Error\n
  *             01 : Abort (Abort user request on going)\n
  *             10 : Timeout\n
  *             11 : Error\n
  *          b5     IP initilisation status\n
  *             0  : Reset (IP not initialized)\n
  *             1  : Init done (IP initialized and ready to use. HAL I2C Init function called)\n
  *          b4     (not used)\n
  *             x  : Should be set to 0\n
  *          b3\n
  *             0  : Ready or Busy (No Listen mode ongoing)\n
  *             1  : Listen (IP in Address Listen Mode)\n
  *          b2     Intrinsic process state\n
  *             0  : Ready\n
  *             1  : Busy (IP busy with some configuration or internal operations)\n
  *          b1     Rx state\n
  *             0  : Ready (no Rx operation ongoing)\n
  *             1  : Busy (Rx operation ongoing)\n
  *          b0     Tx state\n
  *             0  : Ready (no Tx operation ongoing)\n
  *             1  : Busy (Tx operation ongoing)
  * @{
  */
pub type HAL_I2C_StateTypeDef = libc::c_uint;
/* !< Error                                     */
/* !< Timeout state                             */
pub const HAL_I2C_STATE_ERROR: HAL_I2C_StateTypeDef = 224;
/* !< Abort user request ongoing                */
pub const HAL_I2C_STATE_TIMEOUT: HAL_I2C_StateTypeDef = 160;
/* !< Address Listen Mode and Data Reception
                                                 process is ongoing                         */
pub const HAL_I2C_STATE_ABORT: HAL_I2C_StateTypeDef = 96;
/* !< Address Listen Mode and Data Transmission
                                                 process is ongoing                         */
pub const HAL_I2C_STATE_BUSY_RX_LISTEN: HAL_I2C_StateTypeDef = 42;
/* !< Address Listen Mode is ongoing            */
pub const HAL_I2C_STATE_BUSY_TX_LISTEN: HAL_I2C_StateTypeDef = 41;
/* !< Data Reception process is ongoing         */
pub const HAL_I2C_STATE_LISTEN: HAL_I2C_StateTypeDef = 40;
/* !< Data Transmission process is ongoing      */
pub const HAL_I2C_STATE_BUSY_RX: HAL_I2C_StateTypeDef = 34;
/* !< An internal process is ongoing            */
pub const HAL_I2C_STATE_BUSY_TX: HAL_I2C_StateTypeDef = 33;
/* !< Peripheral Initialized and ready for use  */
pub const HAL_I2C_STATE_BUSY: HAL_I2C_StateTypeDef = 36;
/* !< Peripheral is not yet Initialized         */
pub const HAL_I2C_STATE_READY: HAL_I2C_StateTypeDef = 32;
pub const HAL_I2C_STATE_RESET: HAL_I2C_StateTypeDef = 0;
/* *
  * @}
  */
/* * @defgroup HAL_mode_structure_definition HAL mode structure definition
  * @brief  HAL Mode structure definition
  * @note  HAL I2C Mode value coding follow below described bitmap :\n
  *          b7     (not used)\n
  *             x  : Should be set to 0\n
  *          b6\n
  *             0  : None\n
  *             1  : Memory (HAL I2C communication is in Memory Mode)\n
  *          b5\n
  *             0  : None\n
  *             1  : Slave (HAL I2C communication is in Slave Mode)\n
  *          b4\n
  *             0  : None\n
  *             1  : Master (HAL I2C communication is in Master Mode)\n
  *          b3-b2-b1-b0  (not used)\n
  *             xxxx : Should be set to 0000
  * @{
  */
pub type HAL_I2C_ModeTypeDef = libc::c_uint;
/* !< I2C communication is in Memory Mode       */
/* !< I2C communication is in Slave Mode        */
pub const HAL_I2C_MODE_MEM: HAL_I2C_ModeTypeDef = 64;
/* !< I2C communication is in Master Mode       */
pub const HAL_I2C_MODE_SLAVE: HAL_I2C_ModeTypeDef = 32;
/* !< No I2C communication on going             */
pub const HAL_I2C_MODE_MASTER: HAL_I2C_ModeTypeDef = 16;
pub const HAL_I2C_MODE_NONE: HAL_I2C_ModeTypeDef = 0;
/* * 
  * @}
  */
/* * @defgroup I2C_Error_Code_definition I2C Error Code definition
  * @brief  I2C Error Code definition
  * @{
  */
/* !< No error              */
/* !< BERR error            */
/* !< ARLO error            */
/* !< ACKF error            */
/* !< OVR error             */
/* !< DMA transfer error    */
/* !< Timeout error         */
/* !< Size Management error */
/* *
  * @}
  */
/* * @defgroup I2C_handle_Structure_definition I2C handle Structure definition
  * @brief  I2C handle Structure definition
  * @{
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __I2C_HandleTypeDef {
    pub Instance: *mut I2C_TypeDef,
    pub Init: I2C_InitTypeDef,
    pub pBuffPtr: *mut uint8_t,
    pub XferSize: uint16_t,
    pub XferCount: uint16_t,
    pub XferOptions: uint32_t,
    pub PreviousState: uint32_t,
    pub XferISR: Option<unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                             _: uint32_t, _: uint32_t)
                            -> HAL_StatusTypeDef>,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: HAL_I2C_StateTypeDef,
    pub Mode: HAL_I2C_ModeTypeDef,
    pub ErrorCode: uint32_t,
    pub AddrEventCount: uint32_t,
}
pub type I2C_HandleTypeDef = __I2C_HandleTypeDef;
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
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
/* !< I2C Address Event counter                 */
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
// microsecond time
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
pub type rccPeriphTag_t = uint8_t;
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
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type i2cHardware_t = i2cHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cHardware_s {
    pub device: I2CDevice,
    pub reg: *mut I2C_TypeDef,
    pub sclPins: [i2cPinDef_t; 4],
    pub sdaPins: [i2cPinDef_t; 4],
    pub rcc: rccPeriphTag_t,
    pub ev_irq: uint8_t,
    pub er_irq: uint8_t,
}
pub type i2cPinDef_t = i2cPinDef_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cPinDef_s {
    pub ioTag: ioTag_t,
}
pub type i2cDevice_t = i2cDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct i2cDevice_s {
    pub hardware: *const i2cHardware_t,
    pub reg: *mut I2C_TypeDef,
    pub scl: IO_t,
    pub sda: IO_t,
    pub overClock: bool,
    pub pullUp: bool,
    pub handle: I2C_HandleTypeDef,
}
// MCU/Driver dependent member follows
// Initialized in run_static_initializers
#[no_mangle]
pub static mut i2cHardware: [i2cHardware_t; 4] =
    [i2cHardware_t{device: I2CDEV_1,
                   reg: 0 as *mut I2C_TypeDef,
                   sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                   sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                   rcc: 0,
                   ev_irq: 0,
                   er_irq: 0,}; 4];
#[no_mangle]
pub static mut i2cDevice: [i2cDevice_t; 4] =
    [i2cDevice_t{hardware: 0 as *const i2cHardware_t,
                 reg: 0 as *const I2C_TypeDef as *mut I2C_TypeDef,
                 scl: 0 as *const libc::c_void as *mut libc::c_void,
                 sda: 0 as *const libc::c_void as *mut libc::c_void,
                 overClock: false,
                 pullUp: false,
                 handle:
                     I2C_HandleTypeDef{Instance:
                                           0 as *const I2C_TypeDef as
                                               *mut I2C_TypeDef,
                                       Init:
                                           I2C_InitTypeDef{Timing: 0,
                                                           OwnAddress1: 0,
                                                           AddressingMode: 0,
                                                           DualAddressMode: 0,
                                                           OwnAddress2: 0,
                                                           OwnAddress2Masks:
                                                               0,
                                                           GeneralCallMode: 0,
                                                           NoStretchMode: 0,},
                                       pBuffPtr:
                                           0 as *const uint8_t as
                                               *mut uint8_t,
                                       XferSize: 0,
                                       XferCount: 0,
                                       XferOptions: 0,
                                       PreviousState: 0,
                                       XferISR: None,
                                       hdmatx:
                                           0 as *const DMA_HandleTypeDef as
                                               *mut DMA_HandleTypeDef,
                                       hdmarx:
                                           0 as *const DMA_HandleTypeDef as
                                               *mut DMA_HandleTypeDef,
                                       Lock: HAL_UNLOCKED,
                                       State: HAL_I2C_STATE_RESET,
                                       Mode: HAL_I2C_MODE_NONE,
                                       ErrorCode: 0,
                                       AddrEventCount: 0,},}; 4];
#[no_mangle]
pub unsafe extern "C" fn I2C1_ER_IRQHandler() {
    HAL_I2C_ER_IRQHandler(&mut (*i2cDevice.as_mut_ptr().offset(I2CDEV_1 as
                                                                   libc::c_int
                                                                   as
                                                                   isize)).handle);
}
#[no_mangle]
pub unsafe extern "C" fn I2C1_EV_IRQHandler() {
    HAL_I2C_EV_IRQHandler(&mut (*i2cDevice.as_mut_ptr().offset(I2CDEV_1 as
                                                                   libc::c_int
                                                                   as
                                                                   isize)).handle);
}
#[no_mangle]
pub unsafe extern "C" fn I2C2_ER_IRQHandler() {
    HAL_I2C_ER_IRQHandler(&mut (*i2cDevice.as_mut_ptr().offset(I2CDEV_2 as
                                                                   libc::c_int
                                                                   as
                                                                   isize)).handle);
}
#[no_mangle]
pub unsafe extern "C" fn I2C2_EV_IRQHandler() {
    HAL_I2C_EV_IRQHandler(&mut (*i2cDevice.as_mut_ptr().offset(I2CDEV_2 as
                                                                   libc::c_int
                                                                   as
                                                                   isize)).handle);
}
#[no_mangle]
pub unsafe extern "C" fn I2C3_ER_IRQHandler() {
    HAL_I2C_ER_IRQHandler(&mut (*i2cDevice.as_mut_ptr().offset(I2CDEV_3 as
                                                                   libc::c_int
                                                                   as
                                                                   isize)).handle);
}
#[no_mangle]
pub unsafe extern "C" fn I2C3_EV_IRQHandler() {
    HAL_I2C_EV_IRQHandler(&mut (*i2cDevice.as_mut_ptr().offset(I2CDEV_3 as
                                                                   libc::c_int
                                                                   as
                                                                   isize)).handle);
}
static mut i2cErrorCount: uint16_t = 0 as libc::c_int as uint16_t;
static mut i2cOverClock: bool = false;
#[no_mangle]
pub unsafe extern "C" fn i2cSetOverclock(mut OverClock: uint8_t) {
    i2cOverClock =
        if OverClock as libc::c_int != 0 {
            1 as libc::c_int
        } else { 0 as libc::c_int } != 0;
}
unsafe extern "C" fn i2cHandleHardwareFailure(mut device: I2CDevice) -> bool {
    ::core::ptr::write_volatile(&mut i2cErrorCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&i2cErrorCount
                                                                           as
                                                                           *const uint16_t).wrapping_add(1));
    // reinit peripheral + clock out garbage
    //i2cInit(device);
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn i2cWriteBuffer(mut device: I2CDevice,
                                        mut addr_: uint8_t, mut reg_: uint8_t,
                                        mut len_: uint8_t,
                                        mut data: *mut uint8_t) -> bool {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 4 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    let mut pHandle: *mut I2C_HandleTypeDef =
        &mut (*i2cDevice.as_mut_ptr().offset(device as isize)).handle;
    if (*pHandle).Instance.is_null() { return 0 as libc::c_int != 0 }
    let mut status: HAL_StatusTypeDef = HAL_OK;
    if reg_ as libc::c_int == 0xff as libc::c_int {
        status =
            HAL_I2C_Master_Transmit(pHandle,
                                    ((addr_ as libc::c_int) <<
                                         1 as libc::c_int) as uint16_t, data,
                                    len_ as uint16_t,
                                    0x1000 as libc::c_int as uint32_t)
    } else {
        status =
            HAL_I2C_Mem_Write(pHandle,
                              ((addr_ as libc::c_int) << 1 as libc::c_int) as
                                  uint16_t, reg_ as uint16_t,
                              0x1 as libc::c_uint as uint16_t, data,
                              len_ as uint16_t,
                              0x1000 as libc::c_int as uint32_t)
    }
    if status as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        return i2cHandleHardwareFailure(device)
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn i2cWrite(mut device: I2CDevice, mut addr_: uint8_t,
                                  mut reg_: uint8_t, mut data: uint8_t)
 -> bool {
    return i2cWriteBuffer(device, addr_, reg_, 1 as libc::c_int as uint8_t,
                          &mut data);
}
#[no_mangle]
pub unsafe extern "C" fn i2cRead(mut device: I2CDevice, mut addr_: uint8_t,
                                 mut reg_: uint8_t, mut len: uint8_t,
                                 mut buf: *mut uint8_t) -> bool {
    if device as libc::c_int == I2CINVALID as libc::c_int ||
           device as libc::c_int > 4 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    let mut pHandle: *mut I2C_HandleTypeDef =
        &mut (*i2cDevice.as_mut_ptr().offset(device as isize)).handle;
    if (*pHandle).Instance.is_null() { return 0 as libc::c_int != 0 }
    let mut status: HAL_StatusTypeDef = HAL_OK;
    if reg_ as libc::c_int == 0xff as libc::c_int {
        status =
            HAL_I2C_Master_Receive(pHandle,
                                   ((addr_ as libc::c_int) <<
                                        1 as libc::c_int) as uint16_t, buf,
                                   len as uint16_t,
                                   0x1000 as libc::c_int as uint32_t)
    } else {
        status =
            HAL_I2C_Mem_Read(pHandle,
                             ((addr_ as libc::c_int) << 1 as libc::c_int) as
                                 uint16_t, reg_ as uint16_t,
                             0x1 as libc::c_uint as uint16_t, buf,
                             len as uint16_t,
                             0x1000 as libc::c_int as uint32_t)
    }
    if status as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        return i2cHandleHardwareFailure(device)
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn i2cInit(mut device: I2CDevice) {
    if device as libc::c_int == I2CINVALID as libc::c_int { return }
    let mut pDev: *mut i2cDevice_t =
        &mut *i2cDevice.as_mut_ptr().offset(device as isize) as
            *mut i2cDevice_t;
    let mut hardware: *const i2cHardware_t = (*pDev).hardware;
    if hardware.is_null() { return }
    let mut scl: IO_t = (*pDev).scl;
    let mut sda: IO_t = (*pDev).sda;
    IOInit(scl, OWNER_I2C_SCL,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    IOInit(sda, OWNER_I2C_SDA,
           (device as libc::c_int + 1 as libc::c_int) as uint8_t);
    // Enable RCC
    RCC_ClockCmd((*hardware).rcc, ENABLE);
    i2cUnstick(scl, sda);
    // Init pins
    IOConfigGPIOAF(scl,
                   if (*pDev).pullUp as libc::c_int != 0 {
                       (0x12 as libc::c_uint |
                            (0x3 as libc::c_uint) << 2 as libc::c_int) |
                           (0x1 as libc::c_uint) << 5 as libc::c_int
                   } else {
                       (0x12 as libc::c_uint |
                            (0x3 as libc::c_uint) << 2 as libc::c_int) |
                           (0 as libc::c_uint) << 5 as libc::c_int
                   } as ioConfig_t, 0x4 as libc::c_uint as uint8_t);
    IOConfigGPIOAF(sda,
                   if (*pDev).pullUp as libc::c_int != 0 {
                       (0x12 as libc::c_uint |
                            (0x3 as libc::c_uint) << 2 as libc::c_int) |
                           (0x1 as libc::c_uint) << 5 as libc::c_int
                   } else {
                       (0x12 as libc::c_uint |
                            (0x3 as libc::c_uint) << 2 as libc::c_int) |
                           (0 as libc::c_uint) << 5 as libc::c_int
                   } as ioConfig_t, 0x4 as libc::c_uint as uint8_t);
    // Init I2C peripheral
    let mut pHandle: *mut I2C_HandleTypeDef = &mut (*pDev).handle;
    memset(pHandle as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<I2C_HandleTypeDef>() as libc::c_ulong);
    (*pHandle).Instance = (*(*pDev).hardware).reg;
    // / TODO: HAL check if I2C timing is correct
    if (*pDev).overClock {
        // 800khz Maximum speed tested on various boards without issues
        (*pHandle).Init.Timing = 0x500d1d as libc::c_int as uint32_t
    } else { (*pHandle).Init.Timing = 0x500c6f as libc::c_int as uint32_t }
    (*pHandle).Init.OwnAddress1 = 0 as libc::c_int as uint32_t;
    (*pHandle).Init.AddressingMode = 0x1 as libc::c_uint;
    (*pHandle).Init.DualAddressMode = 0 as libc::c_uint;
    (*pHandle).Init.OwnAddress2 = 0 as libc::c_int as uint32_t;
    (*pHandle).Init.GeneralCallMode = 0 as libc::c_uint;
    (*pHandle).Init.NoStretchMode = 0 as libc::c_uint;
    HAL_I2C_Init(pHandle);
    // Enable the Analog I2C Filter
    HAL_I2CEx_ConfigAnalogFilter(pHandle, 0 as libc::c_uint);
    // Setup interrupt handlers
    HAL_NVIC_SetPriority((*hardware).er_irq as IRQn_Type,
                         ((((0 as libc::c_int) <<
                                (4 as libc::c_int as
                                     libc::c_uint).wrapping_sub((7 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_sub(0x5
                                                                                                    as
                                                                                                    libc::c_uint))
                                |
                                0 as libc::c_int &
                                    0xf as libc::c_int >>
                                        (7 as libc::c_int as
                                             libc::c_uint).wrapping_sub(0x5 as
                                                                            libc::c_uint))
                               << 4 as libc::c_int & 0xf0 as libc::c_int) >>
                              (4 as libc::c_int as
                                   libc::c_uint).wrapping_sub((7 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint).wrapping_sub(0x5
                                                                                                  as
                                                                                                  libc::c_uint))
                              >> 4 as libc::c_int) as uint32_t,
                         ((((0 as libc::c_int) <<
                                (4 as libc::c_int as
                                     libc::c_uint).wrapping_sub((7 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_sub(0x5
                                                                                                    as
                                                                                                    libc::c_uint))
                                |
                                0 as libc::c_int &
                                    0xf as libc::c_int >>
                                        (7 as libc::c_int as
                                             libc::c_uint).wrapping_sub(0x5 as
                                                                            libc::c_uint))
                               << 4 as libc::c_int & 0xf0 as libc::c_int &
                               0xf as libc::c_int >>
                                   (7 as libc::c_int as
                                        libc::c_uint).wrapping_sub(0x5 as
                                                                       libc::c_uint))
                              >> 4 as libc::c_int) as uint32_t);
    HAL_NVIC_EnableIRQ((*hardware).er_irq as IRQn_Type);
    HAL_NVIC_SetPriority((*hardware).ev_irq as IRQn_Type,
                         ((((0 as libc::c_int) <<
                                (4 as libc::c_int as
                                     libc::c_uint).wrapping_sub((7 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_sub(0x5
                                                                                                    as
                                                                                                    libc::c_uint))
                                |
                                0 as libc::c_int &
                                    0xf as libc::c_int >>
                                        (7 as libc::c_int as
                                             libc::c_uint).wrapping_sub(0x5 as
                                                                            libc::c_uint))
                               << 4 as libc::c_int & 0xf0 as libc::c_int) >>
                              (4 as libc::c_int as
                                   libc::c_uint).wrapping_sub((7 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint).wrapping_sub(0x5
                                                                                                  as
                                                                                                  libc::c_uint))
                              >> 4 as libc::c_int) as uint32_t,
                         ((((0 as libc::c_int) <<
                                (4 as libc::c_int as
                                     libc::c_uint).wrapping_sub((7 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_sub(0x5
                                                                                                    as
                                                                                                    libc::c_uint))
                                |
                                0 as libc::c_int &
                                    0xf as libc::c_int >>
                                        (7 as libc::c_int as
                                             libc::c_uint).wrapping_sub(0x5 as
                                                                            libc::c_uint))
                               << 4 as libc::c_int & 0xf0 as libc::c_int &
                               0xf as libc::c_int >>
                                   (7 as libc::c_int as
                                        libc::c_uint).wrapping_sub(0x5 as
                                                                       libc::c_uint))
                              >> 4 as libc::c_int) as uint32_t);
    HAL_NVIC_EnableIRQ((*hardware).ev_irq as IRQn_Type);
}
#[no_mangle]
pub unsafe extern "C" fn i2cGetErrorCounter() -> uint16_t {
    return i2cErrorCount;
}
unsafe extern "C" fn i2cUnstick(mut scl: IO_t, mut sda: IO_t) {
    let mut i: libc::c_int = 0;
    IOHi(scl);
    IOHi(sda);
    IOConfigGPIO(scl,
                 (0x11 as libc::c_uint |
                      (0 as libc::c_uint) << 2 as libc::c_int |
                      (0 as libc::c_uint) << 5 as libc::c_int) as ioConfig_t);
    IOConfigGPIO(sda,
                 (0x11 as libc::c_uint |
                      (0 as libc::c_uint) << 2 as libc::c_int |
                      (0 as libc::c_uint) << 5 as libc::c_int) as ioConfig_t);
    // Clock out, with SDA high:
    //   7 data bits
    //   1 READ bit
    //   1 cycle for the ACK
    i = 0 as libc::c_int;
    while i < 7 as libc::c_int + 1 as libc::c_int + 1 as libc::c_int {
        // Wait for any clock stretching to finish
        let mut timeout: libc::c_int = 500 as libc::c_int / 10 as libc::c_int;
        while !IORead(scl) && timeout != 0 {
            delayMicroseconds(10 as libc::c_int as timeUs_t);
            timeout -= 1
        }
        // Pull low
        IOLo(scl); // Set bus low
        delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as
                              timeUs_t); // Set bus high
        IOHi(scl);
        delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
        i += 1
    }
    // Generate a stop condition in case there was none
    IOLo(scl); // Set bus scl high
    delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
    IOLo(sda);
    delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
    IOHi(scl);
    delayMicroseconds((10 as libc::c_int / 2 as libc::c_int) as timeUs_t);
    IOHi(sda);
    // Set bus sda high
}
unsafe extern "C" fn run_static_initializers() {
    i2cHardware =
        [{
             let mut init =
                 i2cHardware_s{device: I2CDEV_2,
                               reg:
                                   (0x40000000 as
                                        libc::c_uint).wrapping_add(0x5800 as
                                                                       libc::c_uint)
                                       as *mut I2C_TypeDef,
                               sclPins:
                                   [{
                                        let mut init =
                                            i2cPinDef_s{ioTag:
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
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            0 as libc::c_int
                                                                as ioTag_t,};
                                        init
                                    }, i2cPinDef_t{ioTag: 0,},
                                    i2cPinDef_t{ioTag: 0,}],
                               sdaPins:
                                   [{
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            ((1 as libc::c_int
                                                                  +
                                                                  1 as
                                                                      libc::c_int)
                                                                 <<
                                                                 4 as
                                                                     libc::c_int
                                                                 |
                                                                 11 as
                                                                     libc::c_int)
                                                                as ioTag_t,};
                                        init
                                    },
                                    {
                                        let mut init =
                                            i2cPinDef_s{ioTag:
                                                            0 as libc::c_int
                                                                as ioTag_t,};
                                        init
                                    }, i2cPinDef_t{ioTag: 0,},
                                    i2cPinDef_t{ioTag: 0,}],
                               rcc:
                                   (((RCC_APB1 as libc::c_int) <<
                                         5 as libc::c_int) as libc::c_long |
                                        (16 as libc::c_int *
                                             (((0x1 as libc::c_uint) <<
                                                   22 as libc::c_uint) as
                                                  libc::c_long >
                                                  65535 as libc::c_long) as
                                                 libc::c_int) as libc::c_long
                                            +
                                            ((8 as libc::c_int *
                                                  (((0x1 as libc::c_uint) <<
                                                        22 as libc::c_uint) as
                                                       libc::c_long *
                                                       1 as libc::c_long >>
                                                       16 as libc::c_int *
                                                           (((0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 22 as
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
                                                                 22 as
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
                                                                          22
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
                                                                          22
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
                                                                                   22
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
                                                                 22 as
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
                                                                          22
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
                                                                          22
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
                                                                                   22
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
                               ev_irq: I2C2_EV_IRQn as libc::c_int as uint8_t,
                               er_irq:
                                   I2C2_ER_IRQn as libc::c_int as uint8_t,};
             init
         },
         i2cHardware_t{device: I2CDEV_1,
                       reg: 0 as *mut I2C_TypeDef,
                       sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       rcc: 0,
                       ev_irq: 0,
                       er_irq: 0,},
         i2cHardware_t{device: I2CDEV_1,
                       reg: 0 as *mut I2C_TypeDef,
                       sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       rcc: 0,
                       ev_irq: 0,
                       er_irq: 0,},
         i2cHardware_t{device: I2CDEV_1,
                       reg: 0 as *mut I2C_TypeDef,
                       sclPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       sdaPins: [i2cPinDef_t{ioTag: 0,}; 4],
                       rcc: 0,
                       ev_irq: 0,
                       er_irq: 0,}]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
