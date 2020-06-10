use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_DMA_IRQHandler(hdma: *mut DMA_HandleTypeDef);
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    #[no_mangle]
    fn HAL_UART_IRQHandler(huart: *mut UART_HandleTypeDef);
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
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
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    static mut uartDevmap: [*mut uartDevice_t; 0];
    #[no_mangle]
    static uartVTable: [serialPortVTable; 0];
    #[no_mangle]
    fn uartStartTxDMA(s: *mut uartPort_t);
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_TypeDef {
    pub LISR: uint32_t,
    pub HISR: uint32_t,
    pub LIFCR: uint32_t,
    pub HIFCR: uint32_t,
}
/* *
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub CR3: uint32_t,
    pub BRR: uint32_t,
    pub GTPR: uint32_t,
    pub RTOR: uint32_t,
    pub RQR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub RDR: uint32_t,
    pub TDR: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS STM32F7xx Device Peripheral Access Layer Header File.           
  *            
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The STM32F7xx device used in the target application
  *              - To use or not the peripheral�s drivers in application code(i.e. 
  *                code will be based on direct access to peripheral�s registers 
  *                rather than drivers API), this option is controlled by 
  *                "#define USE_HAL_DRIVER"
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
/* * @addtogroup CMSIS
  * @{
  */
/* * @addtogroup stm32f7xx
  * @{
  */
/* __cplusplus */
/* * @addtogroup Library_configuration_section
  * @{
  */
/* *
  * @brief STM32 Family
  */
/* STM32F7 */
/* Uncomment the line below according to the target STM32 device used in your
   application 
  */
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
/* USE_HAL_DRIVER */
/* *
  * @brief CMSIS Device version number V1.2.0
  */
/* !< [31:24] main version */
/* !< [23:16] sub1 version */
/* !< [15:8]  sub2 version */
/* !< [7:0]  release candidate */
/* *
  * @}
  */
/* * @addtogroup Device_Included
  * @{
  */
/* *
  * @}
  */
/* * @addtogroup Exported_types
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
pub const SET: C2RustUnnamed = 1;
pub const RESET: C2RustUnnamed = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_InitTypeDef {
    pub BaudRate: uint32_t,
    pub WordLength: uint32_t,
    pub StopBits: uint32_t,
    pub Parity: uint32_t,
    pub Mode: uint32_t,
    pub HwFlowCtl: uint32_t,
    pub OverSampling: uint32_t,
    pub OneBitSampling: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_AdvFeatureInitTypeDef {
    pub AdvFeatureInit: uint32_t,
    pub TxPinLevelInvert: uint32_t,
    pub RxPinLevelInvert: uint32_t,
    pub DataInvert: uint32_t,
    pub Swap: uint32_t,
    pub OverrunDisable: uint32_t,
    pub DMADisableonRxError: uint32_t,
    pub AutoBaudRateEnable: uint32_t,
    pub AutoBaudRateMode: uint32_t,
    pub MSBFirst: uint32_t,
}
pub type HAL_UART_StateTypeDef = libc::c_uint;
pub const HAL_UART_STATE_ERROR: HAL_UART_StateTypeDef = 224;
pub const HAL_UART_STATE_TIMEOUT: HAL_UART_StateTypeDef = 160;
pub const HAL_UART_STATE_BUSY_TX_RX: HAL_UART_StateTypeDef = 35;
pub const HAL_UART_STATE_BUSY_RX: HAL_UART_StateTypeDef = 34;
pub const HAL_UART_STATE_BUSY_TX: HAL_UART_StateTypeDef = 33;
pub const HAL_UART_STATE_BUSY: HAL_UART_StateTypeDef = 36;
pub const HAL_UART_STATE_READY: HAL_UART_StateTypeDef = 32;
pub const HAL_UART_STATE_RESET: HAL_UART_StateTypeDef = 0;
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_uart.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of UART HAL module.
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
/* * @addtogroup UART
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup UART_Exported_Types UART Exported Types
  * @{
  */
/* *
  * @brief UART Init Structure definition
  */
/* !< This member configures the UART communication baud rate.
                                           The baud rate register is computed using the following formula:
                                           - If oversampling is 16 or in LIN mode,
                                              Baud Rate Register = ((PCLKx) / ((huart->Init.BaudRate)))
                                           - If oversampling is 8,
                                              Baud Rate Register[15:4] = ((2 * PCLKx) / ((huart->Init.BaudRate)))[15:4]
                                              Baud Rate Register[3] =  0
                                              Baud Rate Register[2:0] =  (((2 * PCLKx) / ((huart->Init.BaudRate)))[3:0]) >> 1      */
/* !< Specifies the number of data bits transmitted or received in a frame.
                                           This parameter can be a value of @ref UARTEx_Word_Length */
/* !< Specifies the number of stop bits transmitted.
                                           This parameter can be a value of @ref UART_Stop_Bits */
/* !< Specifies the parity mode.
                                           This parameter can be a value of @ref UART_Parity
                                           @note When parity is enabled, the computed parity is inserted
                                                 at the MSB position of the transmitted data (9th bit when
                                                 the word length is set to 9 data bits; 8th bit when the
                                                 word length is set to 8 data bits). */
/* !< Specifies whether the Receive or Transmit mode is enabled or disabled.
                                           This parameter can be a value of @ref UART_Mode */
/* !< Specifies whether the hardware flow control mode is enabled
                                           or disabled.
                                           This parameter can be a value of @ref UART_Hardware_Flow_Control */
/* !< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to fPCLK/8).
                                           This parameter can be a value of @ref UART_Over_Sampling */
/* !< Specifies whether a single sample or three samples' majority vote is selected.
                                           Selecting the single sample method increases the receiver tolerance to clock
                                           deviations. This parameter can be a value of @ref UART_OneBit_Sampling */
/* *
  * @brief  UART Advanced Features initialization structure definition
  */
/* !< Specifies which advanced UART features is initialized. Several
                                       Advanced Features may be initialized at the same time .
                                       This parameter can be a value of @ref UART_Advanced_Features_Initialization_Type */
/* !< Specifies whether the TX pin active level is inverted.
                                       This parameter can be a value of @ref UART_Tx_Inv  */
/* !< Specifies whether the RX pin active level is inverted.
                                       This parameter can be a value of @ref UART_Rx_Inv  */
/* !< Specifies whether data are inverted (positive/direct logic
                                       vs negative/inverted logic).
                                       This parameter can be a value of @ref UART_Data_Inv */
/* !< Specifies whether TX and RX pins are swapped.
                                       This parameter can be a value of @ref UART_Rx_Tx_Swap */
/* !< Specifies whether the reception overrun detection is disabled.
                                       This parameter can be a value of @ref UART_Overrun_Disable */
/* !< Specifies whether the DMA is disabled in case of reception error.
                                       This parameter can be a value of @ref UART_DMA_Disable_on_Rx_Error */
/* !< Specifies whether auto Baud rate detection is enabled.
                                       This parameter can be a value of @ref UART_AutoBaudRate_Enable */
/* !< If auto Baud rate detection is enabled, specifies how the rate
                                       detection is carried out.
                                       This parameter can be a value of @ref UART_AutoBaud_Rate_Mode */
/* !< Specifies whether MSB is sent first on UART line.
                                       This parameter can be a value of @ref UART_MSB_First */
/* *
  * @brief HAL UART State structures definition
  * @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
  *        - gState contains UART state information related to global Handle management 
  *          and also information related to Tx operations.
  *          gState value coding follow below described bitmap :
  *          b7-b6  Error information 
  *             00 : No Error
  *             01 : (Not Used)
  *             10 : Timeout
  *             11 : Error
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP not initialized. HAL UART Init function already called)
  *          b4-b3  (not used)
  *             xx : Should be set to 00
  *          b2     Intrinsic process state
  *             0  : Ready
  *             1  : Busy (IP busy with some configuration or internal operations)
  *          b1     (not used)
  *             x  : Should be set to 0
  *          b0     Tx state
  *             0  : Ready (no Tx operation ongoing)
  *             1  : Busy (Tx operation ongoing)
  *        - RxState contains information related to Rx operations.
  *          RxState value coding follow below described bitmap :
  *          b7-b6  (not used)
  *             xx : Should be set to 00
  *          b5     IP initilisation status
  *             0  : Reset (IP not initialized)
  *             1  : Init done (IP not initialized)
  *          b4-b2  (not used)
  *            xxx : Should be set to 000
  *          b1     Rx state
  *             0  : Ready (no Rx operation ongoing)
  *             1  : Busy (Rx operation ongoing)
  *          b0     (not used)
  *             x  : Should be set to 0.
  */
/* !< Peripheral is not initialized
                                                   Value is allowed for gState and RxState */
/* !< Peripheral Initialized and ready for use
                                                   Value is allowed for gState and RxState */
/* !< an internal process is ongoing 
                                                   Value is allowed for gState only */
/* !< Data Transmission process is ongoing
                                                   Value is allowed for gState only */
/* !< Data Reception process is ongoing
                                                   Value is allowed for RxState only */
/* !< Data Transmission and Reception process is ongoing
                                                   Not to be used for neither gState nor RxState.
                                                   Value is result of combination (Or) between gState and RxState values */
/* !< Timeout state
                                                   Value is allowed for gState only */
/* !< Error
                                                   Value is allowed for gState only */
/* *
  * @brief UART clock sources definition
  */
/* !< PCLK1 clock source  */
/* !< PCLK2 clock source  */
/* !< HSI clock source    */
/* !< SYSCLK clock source */
/* !< LSE clock source       */
/* !< Undefined clock source */
/* *
  * @brief  UART handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct UART_HandleTypeDef {
    pub Instance: *mut USART_TypeDef,
    pub Init: UART_InitTypeDef,
    pub AdvancedInit: UART_AdvFeatureInitTypeDef,
    pub pTxBuffPtr: *mut uint8_t,
    pub TxXferSize: uint16_t,
    pub TxXferCount: uint16_t,
    pub pRxBuffPtr: *mut uint8_t,
    pub RxXferSize: uint16_t,
    pub RxXferCount: uint16_t,
    pub Mask: uint16_t,
    pub hdmatx: *mut DMA_HandleTypeDef,
    pub hdmarx: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub gState: HAL_UART_StateTypeDef,
    pub RxState: HAL_UART_StateTypeDef,
    pub ErrorCode: uint32_t,
}
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dmaChannelDescriptor_s {
    pub dma: *mut DMA_TypeDef,
    pub ref_0: *mut DMA_Stream_TypeDef,
    pub stream: uint8_t,
    pub irqHandlerCallback: dmaCallbackHandlerFuncPtr,
    pub flagsShift: uint8_t,
    pub irqN: IRQn_Type,
    pub userParam: uint32_t,
    pub owner: resourceOwner_e,
    pub resourceIndex: uint8_t,
    pub completeFlag: uint32_t,
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
pub type dmaCallbackHandlerFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut dmaChannelDescriptor_s) -> ()>;
pub type dmaChannelDescriptor_t = dmaChannelDescriptor_s;
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST7_HANDLER: dmaIdentifier_e = 16;
pub const DMA2_ST6_HANDLER: dmaIdentifier_e = 15;
pub const DMA2_ST5_HANDLER: dmaIdentifier_e = 14;
pub const DMA2_ST4_HANDLER: dmaIdentifier_e = 13;
pub const DMA2_ST3_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_ST2_HANDLER: dmaIdentifier_e = 11;
pub const DMA2_ST1_HANDLER: dmaIdentifier_e = 10;
pub const DMA2_ST0_HANDLER: dmaIdentifier_e = 9;
pub const DMA1_ST7_HANDLER: dmaIdentifier_e = 8;
pub const DMA1_ST6_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_ST5_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_ST4_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_ST3_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_ST2_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_ST1_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_ST0_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
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
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
pub type serialPort_t = serialPort_s;
pub type UARTDevice_e = libc::c_uint;
pub const UARTDEV_8: UARTDevice_e = 7;
pub const UARTDEV_7: UARTDevice_e = 6;
pub const UARTDEV_6: UARTDevice_e = 5;
pub const UARTDEV_5: UARTDevice_e = 4;
pub const UARTDEV_4: UARTDevice_e = 3;
pub const UARTDEV_3: UARTDevice_e = 2;
pub const UARTDEV_2: UARTDevice_e = 1;
pub const UARTDEV_1: UARTDevice_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartPort_s {
    pub port: serialPort_t,
    pub rxDMAHandle: DMA_HandleTypeDef,
    pub txDMAHandle: DMA_HandleTypeDef,
    pub rxDMAStream: *mut DMA_Stream_TypeDef,
    pub txDMAStream: *mut DMA_Stream_TypeDef,
    pub rxDMAChannel: uint32_t,
    pub txDMAChannel: uint32_t,
    pub rxDMAIrq: uint32_t,
    pub txDMAIrq: uint32_t,
    pub rxDMAPos: uint32_t,
    pub txDMAPeripheralBaseAddr: uint32_t,
    pub rxDMAPeripheralBaseAddr: uint32_t,
    pub Handle: UART_HandleTypeDef,
    pub USARTx: *mut USART_TypeDef,
    pub txDMAEmpty: bool,
}
pub type uartPort_t = uartPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartHardware_s {
    pub device: UARTDevice_e,
    pub reg: *mut USART_TypeDef,
    pub DMAChannel: uint32_t,
    pub txDMAStream: *mut DMA_Stream_TypeDef,
    pub rxDMAStream: *mut DMA_Stream_TypeDef,
    pub rxPins: [ioTag_t; 3],
    pub txPins: [ioTag_t; 3],
    pub rcc_ahb1: uint32_t,
    pub rcc_apb2: rccPeriphTag_t,
    pub rcc_apb1: rccPeriphTag_t,
    pub af: uint8_t,
    pub txIrq: uint8_t,
    pub rxIrq: uint8_t,
    pub txPriority: uint8_t,
    pub rxPriority: uint8_t,
}
// All USARTs can also be used as UART, and we use them only as UART.
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
// Configuration constants
// Count number of configured UARTs
pub type uartHardware_t = uartHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartDevice_s {
    pub port: uartPort_t,
    pub hardware: *const uartHardware_t,
    pub rx: ioTag_t,
    pub tx: ioTag_t,
    pub rxBuffer: [uint8_t; 128],
    pub txBuffer: [uint8_t; 256],
}
// XXX Not required for full allocation
// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.
pub type uartDevice_t = uartDevice_s;
// Initialized in run_static_initializers
#[no_mangle]
pub static mut uartHardware: [uartHardware_t; 6] =
    [uartHardware_t{device: UARTDEV_1,
                    reg: 0 as *mut USART_TypeDef,
                    DMAChannel: 0,
                    txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                    rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                    rxPins: [0; 3],
                    txPins: [0; 3],
                    rcc_ahb1: 0,
                    rcc_apb2: 0,
                    rcc_apb1: 0,
                    af: 0,
                    txIrq: 0,
                    rxIrq: 0,
                    txPriority: 0,
                    rxPriority: 0,}; 6];
#[no_mangle]
pub unsafe extern "C" fn uartIrqHandler(mut s: *mut uartPort_t) {
    let mut huart: *mut UART_HandleTypeDef = &mut (*s).Handle;
    /* UART in mode Receiver ---------------------------------------------------*/
    if (*(*huart).Instance).ISR &
           (1 as libc::c_int as uint32_t) <<
               (0x525 as libc::c_uint >> 0x8 as libc::c_int) !=
           RESET as libc::c_int as libc::c_uint {
        let mut rbyte: uint8_t =
            ((*(*huart).Instance).RDR &
                 0xff as libc::c_int as uint8_t as libc::c_uint) as uint8_t;
        if (*s).port.rxCallback.is_some() {
            (*s).port.rxCallback.expect("non-null function pointer")(rbyte as
                                                                         uint16_t,
                                                                     (*s).port.rxCallbackData);
        } else {
            ::core::ptr::write_volatile((*s).port.rxBuffer.offset((*s).port.rxBufferHead
                                                                      as
                                                                      isize),
                                        rbyte);
            (*s).port.rxBufferHead =
                (*s).port.rxBufferHead.wrapping_add(1 as libc::c_int as
                                                        libc::c_uint).wrapping_rem((*s).port.rxBufferSize)
        }
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               8 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).RQR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).RQR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* UART parity error interrupt occurred -------------------------------------*/
    if (*(*huart).Instance).ISR &
           (1 as libc::c_int as uint32_t) <<
               (0x28 as libc::c_uint >> 0x8 as libc::c_int) !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint)
    }
    /* UART frame error interrupt occurred --------------------------------------*/
    if (*(*huart).Instance).ISR &
           (1 as libc::c_int as uint32_t) <<
               (0x100 as libc::c_uint >> 0x8 as libc::c_int) !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        1 as libc::c_uint)
    }
    /* UART noise error interrupt occurred --------------------------------------*/
    if (*(*huart).Instance).ISR &
           (1 as libc::c_int as uint32_t) <<
               (0x200 as libc::c_uint >> 0x8 as libc::c_int) !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        2 as libc::c_uint)
    }
    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if (*(*huart).Instance).ISR &
           (1 as libc::c_int as uint32_t) <<
               (0x300 as libc::c_uint >> 0x8 as libc::c_int) !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint)
    }
    /* UART in mode Transmitter ------------------------------------------------*/
    if (*s).txDMAStream.is_null() &&
           (*(*huart).Instance).ISR &
               (1 as libc::c_int as uint32_t) <<
                   (0x727 as libc::c_uint >> 0x8 as libc::c_int) !=
               RESET as libc::c_int as libc::c_uint {
        /* Check that a Tx process is ongoing */
        if (*huart).gState as libc::c_uint !=
               HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
            if (*s).port.txBufferTail == (*s).port.txBufferHead {
                ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                                *mut uint16_t,
                                            0 as libc::c_int as uint16_t);
                /* Disable the UART Transmit Data Register Empty Interrupt */
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            } else {
                if (*huart).Init.WordLength ==
                       (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                       (*huart).Init.Parity == 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*huart).Instance).TDR
                                                    as *mut uint32_t,
                                                (*(*s).port.txBuffer.offset((*s).port.txBufferTail
                                                                                as
                                                                                isize)
                                                     as uint16_t as
                                                     libc::c_int &
                                                     0x1ff as libc::c_uint as
                                                         uint16_t as
                                                         libc::c_int) as
                                                    uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*huart).Instance).TDR
                                                    as *mut uint32_t,
                                                *(*s).port.txBuffer.offset((*s).port.txBufferTail
                                                                               as
                                                                               isize)
                                                    as uint32_t)
                }
                (*s).port.txBufferTail =
                    (*s).port.txBufferTail.wrapping_add(1 as libc::c_int as
                                                            libc::c_uint).wrapping_rem((*s).port.txBufferSize)
            }
        }
    }
    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if (*(*huart).Instance).ISR &
           (1 as libc::c_int as uint32_t) <<
               (0x626 as libc::c_uint >> 0x8 as libc::c_int) !=
           RESET as libc::c_int as libc::c_uint {
        HAL_UART_IRQHandler(huart);
        if !(*s).txDMAStream.is_null() { handleUsartTxDma(s); }
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
/*
 * jflyper - Refactoring, cleanup and made pin-configurable
 */
unsafe extern "C" fn handleUsartTxDma(mut s: *mut uartPort_t) {
    if (*s).port.txBufferHead != (*s).port.txBufferTail {
        uartStartTxDMA(s);
    } else { (*s).txDMAEmpty = 1 as libc::c_int != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn dmaIRQHandler(mut descriptor:
                                           *mut dmaChannelDescriptor_t) {
    let mut s: *mut uartPort_t =
        &mut (*((*descriptor).userParam as *mut uartDevice_t)).port;
    HAL_DMA_IRQHandler(&mut (*s).txDMAHandle);
}
// XXX Should serialUART be consolidated?
#[no_mangle]
pub unsafe extern "C" fn serialUART(mut device: UARTDevice_e,
                                    mut baudRate: uint32_t,
                                    mut mode: portMode_e,
                                    mut options: portOptions_e)
 -> *mut uartPort_t {
    let mut uartdev: *mut uartDevice_t =
        *uartDevmap.as_mut_ptr().offset(device as isize);
    if uartdev.is_null() { return 0 as *mut uartPort_t }
    let mut s: *mut uartPort_t = &mut (*uartdev).port;
    (*s).port.vTable = uartVTable.as_ptr();
    (*s).port.baudRate = baudRate;
    (*s).port.rxBuffer = (*uartdev).rxBuffer.as_mut_ptr();
    (*s).port.txBuffer = (*uartdev).txBuffer.as_mut_ptr();
    (*s).port.rxBufferSize =
        (::core::mem::size_of::<[uint8_t; 128]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>() as
                                             libc::c_ulong) as uint32_t;
    (*s).port.txBufferSize =
        (::core::mem::size_of::<[uint8_t; 256]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>() as
                                             libc::c_ulong) as uint32_t;
    let mut hardware: *const uartHardware_t = (*uartdev).hardware;
    (*s).USARTx = (*hardware).reg;
    if !(*hardware).rxDMAStream.is_null() {
        (*s).rxDMAChannel = (*hardware).DMAChannel;
        (*s).rxDMAStream = (*hardware).rxDMAStream
    }
    if !(*hardware).txDMAStream.is_null() {
        (*s).txDMAChannel = (*hardware).DMAChannel;
        (*s).txDMAStream = (*hardware).txDMAStream;
        // DMA TX Interrupt
        dmaInit((*hardware).txIrq as dmaIdentifier_e, OWNER_SERIAL_TX,
                (device as
                     libc::c_uint).wrapping_add(1 as libc::c_int as
                                                    libc::c_uint) as uint8_t);
        dmaSetHandler((*hardware).txIrq as dmaIdentifier_e,
                      Some(dmaIRQHandler as
                               unsafe extern "C" fn(_:
                                                        *mut dmaChannelDescriptor_t)
                                   -> ()), (*hardware).txPriority as uint32_t,
                      uartdev as uint32_t);
    }
    (*s).txDMAPeripheralBaseAddr =
        &mut (*(*s).USARTx).TDR as *mut uint32_t as uint32_t;
    (*s).rxDMAPeripheralBaseAddr =
        &mut (*(*s).USARTx).RDR as *mut uint32_t as uint32_t;
    (*s).Handle.Instance = (*hardware).reg;
    let mut txIO: IO_t = IOGetByTag((*uartdev).tx);
    let mut rxIO: IO_t = IOGetByTag((*uartdev).rx);
    if options as libc::c_uint & SERIAL_BIDIR as libc::c_int as libc::c_uint
           != 0 && !txIO.is_null() {
        let mut ioCfg: ioConfig_t =
            ((if options as libc::c_uint &
                     SERIAL_INVERTED as libc::c_int as libc::c_uint != 0 ||
                     options as libc::c_uint &
                         SERIAL_BIDIR_PP as libc::c_int as libc::c_uint != 0 {
                  0x2 as libc::c_uint
              } else { 0x12 as libc::c_uint }) |
                 (0x2 as libc::c_uint) << 2 as libc::c_int |
                 (if options as libc::c_uint &
                         SERIAL_INVERTED as libc::c_int as libc::c_uint != 0
                         ||
                         options as libc::c_uint &
                             SERIAL_BIDIR_PP as libc::c_int as libc::c_uint !=
                             0 {
                      0x2 as libc::c_uint
                  } else { 0x1 as libc::c_uint }) << 5 as libc::c_int) as
                ioConfig_t;
        IOInit(txIO, OWNER_SERIAL_TX,
               (device as
                    libc::c_uint).wrapping_add(1 as libc::c_int as
                                                   libc::c_uint) as uint8_t);
        IOConfigGPIOAF(txIO, ioCfg, (*hardware).af);
    } else {
        if mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint != 0
               && !txIO.is_null() {
            IOInit(txIO, OWNER_SERIAL_TX,
                   (device as
                        libc::c_uint).wrapping_add(1 as libc::c_int as
                                                       libc::c_uint) as
                       uint8_t);
            IOConfigGPIOAF(txIO,
                           (0x2 as libc::c_uint |
                                (0 as libc::c_uint) << 2 as libc::c_int |
                                (0 as libc::c_uint) << 5 as libc::c_int) as
                               ioConfig_t, (*hardware).af);
        }
        if mode as libc::c_uint & MODE_RX as libc::c_int as libc::c_uint != 0
               && !rxIO.is_null() {
            IOInit(rxIO, OWNER_SERIAL_RX,
                   (device as
                        libc::c_uint).wrapping_add(1 as libc::c_int as
                                                       libc::c_uint) as
                       uint8_t);
            IOConfigGPIOAF(rxIO,
                           (0x2 as libc::c_uint |
                                (0 as libc::c_uint) << 2 as libc::c_int |
                                (0 as libc::c_uint) << 5 as libc::c_int) as
                               ioConfig_t, (*hardware).af);
        }
    }
    if (*s).rxDMAChannel == 0 {
        HAL_NVIC_SetPriority((*hardware).rxIrq as IRQn_Type,
                             ((*hardware).rxPriority as libc::c_int >>
                                  (4 as libc::c_int as
                                       libc::c_uint).wrapping_sub((7 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint).wrapping_sub(0x5
                                                                                                      as
                                                                                                      libc::c_uint))
                                  >> 4 as libc::c_int) as uint32_t,
                             (((*hardware).rxPriority as libc::c_int &
                                   0xf as libc::c_int >>
                                       (7 as libc::c_int as
                                            libc::c_uint).wrapping_sub(0x5 as
                                                                           libc::c_uint))
                                  >> 4 as libc::c_int) as uint32_t);
        HAL_NVIC_EnableIRQ((*hardware).rxIrq as IRQn_Type);
    }
    return s;
}
unsafe extern "C" fn run_static_initializers() {
    uartHardware =
        [{
             let mut init =
                 uartHardware_s{device: UARTDEV_1,
                                reg:
                                    (0x40000000 as
                                         libc::c_uint).wrapping_add(0x10000 as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_uint)
                                        as *mut USART_TypeDef,
                                DMAChannel: 0x8000000 as libc::c_uint,
                                txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxPins:
                                    [((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          10 as libc::c_int) as ioTag_t,
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 7 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 9 as libc::c_int)
                                         as ioTag_t,
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 6 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc_ahb1: 0,
                                rcc_apb2:
                                    (((RCC_APB2 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (((0x1 as libc::c_uint) <<
                                                    4 as libc::c_uint) as
                                                   libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (((0x1 as libc::c_uint) <<
                                                         4 as libc::c_uint) as
                                                        libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (((0x1 as
                                                                   libc::c_uint)
                                                                  <<
                                                                  4 as
                                                                      libc::c_uint)
                                                                 as
                                                                 libc::c_long
                                                                 >
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
                                                                  4 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           4
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
                                                                           4
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    4
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
                                                                  4 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           4
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
                                                                           4
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    4
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
                                rcc_apb1: 0,
                                af: 0x7 as libc::c_uint as uint8_t,
                                txIrq:
                                    DMA2_ST7_HANDLER as libc::c_int as
                                        uint8_t,
                                rxIrq: USART1_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          1 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          1 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         },
         {
             let mut init =
                 uartHardware_s{device: UARTDEV_2,
                                reg:
                                    (0x40000000 as
                                         libc::c_uint).wrapping_add(0x4400 as
                                                                        libc::c_uint)
                                        as *mut USART_TypeDef,
                                DMAChannel: 0x8000000 as libc::c_uint,
                                txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxPins:
                                    [((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 3 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 2 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc_ahb1: 0,
                                rcc_apb2: 0,
                                rcc_apb1:
                                    (((RCC_APB1 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (((0x1 as libc::c_uint) <<
                                                    17 as libc::c_uint) as
                                                   libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (((0x1 as libc::c_uint) <<
                                                         17 as libc::c_uint)
                                                        as libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (((0x1 as
                                                                   libc::c_uint)
                                                                  <<
                                                                  17 as
                                                                      libc::c_uint)
                                                                 as
                                                                 libc::c_long
                                                                 >
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
                                                                  17 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           17
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
                                                                           17
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    17
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
                                                                  17 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           17
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
                                                                           17
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    17
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
                                af: 0x7 as libc::c_uint as uint8_t,
                                txIrq:
                                    DMA1_ST6_HANDLER as libc::c_int as
                                        uint8_t,
                                rxIrq: USART2_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          0 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          2 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         },
         {
             let mut init =
                 uartHardware_s{device: UARTDEV_3,
                                reg:
                                    (0x40000000 as
                                         libc::c_uint).wrapping_add(0x4800 as
                                                                        libc::c_uint)
                                        as *mut USART_TypeDef,
                                DMAChannel: 0x8000000 as libc::c_uint,
                                txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxPins:
                                    [((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          10 as libc::c_int) as ioTag_t,
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          10 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc_ahb1: 0,
                                rcc_apb2: 0,
                                rcc_apb1:
                                    (((RCC_APB1 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (((0x1 as libc::c_uint) <<
                                                    18 as libc::c_uint) as
                                                   libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (((0x1 as libc::c_uint) <<
                                                         18 as libc::c_uint)
                                                        as libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (((0x1 as
                                                                   libc::c_uint)
                                                                  <<
                                                                  18 as
                                                                      libc::c_uint)
                                                                 as
                                                                 libc::c_long
                                                                 >
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
                                                                  18 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           18
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
                                                                           18
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    18
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
                                                                  18 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           18
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
                                                                           18
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    18
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
                                af: 0x7 as libc::c_uint as uint8_t,
                                txIrq:
                                    DMA1_ST3_HANDLER as libc::c_int as
                                        uint8_t,
                                rxIrq: USART3_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          0 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          2 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         },
         {
             let mut init =
                 uartHardware_s{device: UARTDEV_4,
                                reg:
                                    (0x40000000 as
                                         libc::c_uint).wrapping_add(0x4c00 as
                                                                        libc::c_uint)
                                        as *mut USART_TypeDef,
                                DMAChannel: 0x8000000 as libc::c_uint,
                                txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxPins:
                                    [((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 1 as libc::c_int)
                                         as ioTag_t,
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 0 as libc::c_int)
                                         as ioTag_t,
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          10 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc_ahb1: 0,
                                rcc_apb2: 0,
                                rcc_apb1:
                                    (((RCC_APB1 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (((0x1 as libc::c_uint) <<
                                                    19 as libc::c_uint) as
                                                   libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (((0x1 as libc::c_uint) <<
                                                         19 as libc::c_uint)
                                                        as libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (((0x1 as
                                                                   libc::c_uint)
                                                                  <<
                                                                  19 as
                                                                      libc::c_uint)
                                                                 as
                                                                 libc::c_long
                                                                 >
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
                                                                  19 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           19
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
                                                                           19
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    19
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
                                                                  19 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           19
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
                                                                           19
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    19
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
                                af: 0x8 as libc::c_uint as uint8_t,
                                txIrq:
                                    DMA1_ST4_HANDLER as libc::c_int as
                                        uint8_t,
                                rxIrq: UART4_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          0 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          2 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         },
         {
             let mut init =
                 uartHardware_s{device: UARTDEV_5,
                                reg:
                                    (0x40000000 as
                                         libc::c_uint).wrapping_add(0x5000 as
                                                                        libc::c_uint)
                                        as *mut USART_TypeDef,
                                DMAChannel: 0x8000000 as libc::c_uint,
                                txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxPins:
                                    [((3 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 2 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          12 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc_ahb1: 0,
                                rcc_apb2: 0,
                                rcc_apb1:
                                    (((RCC_APB1 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (((0x1 as libc::c_uint) <<
                                                    20 as libc::c_uint) as
                                                   libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (((0x1 as libc::c_uint) <<
                                                         20 as libc::c_uint)
                                                        as libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (((0x1 as
                                                                   libc::c_uint)
                                                                  <<
                                                                  20 as
                                                                      libc::c_uint)
                                                                 as
                                                                 libc::c_long
                                                                 >
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
                                                                  20 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           20
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
                                                                           20
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    20
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
                                                                  20 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           20
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
                                                                           20
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    20
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
                                af: 0x8 as libc::c_uint as uint8_t,
                                txIrq:
                                    DMA1_ST7_HANDLER as libc::c_int as
                                        uint8_t,
                                rxIrq: UART5_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          0 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          2 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         },
         {
             let mut init =
                 uartHardware_s{device: UARTDEV_6,
                                reg:
                                    (0x40000000 as
                                         libc::c_uint).wrapping_add(0x10000 as
                                                                        libc::c_uint).wrapping_add(0x1400
                                                                                                       as
                                                                                                       libc::c_uint)
                                        as *mut USART_TypeDef,
                                DMAChannel: 0xa000000 as libc::c_uint,
                                txDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxDMAStream: 0 as *mut DMA_Stream_TypeDef,
                                rxPins:
                                    [((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 7 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 6 as libc::c_int)
                                         as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc_ahb1: 0,
                                rcc_apb2:
                                    (((RCC_APB2 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (((0x1 as libc::c_uint) <<
                                                    5 as libc::c_uint) as
                                                   libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (((0x1 as libc::c_uint) <<
                                                         5 as libc::c_uint) as
                                                        libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (((0x1 as
                                                                   libc::c_uint)
                                                                  <<
                                                                  5 as
                                                                      libc::c_uint)
                                                                 as
                                                                 libc::c_long
                                                                 >
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
                                                                  5 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           5
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
                                                                           5
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    5
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
                                                                  5 as
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
                                                                     (((0x1 as
                                                                            libc::c_uint)
                                                                           <<
                                                                           5
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
                                                                           5
                                                                               as
                                                                               libc::c_uint)
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (((0x1
                                                                                     as
                                                                                     libc::c_uint)
                                                                                    <<
                                                                                    5
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
                                rcc_apb1: 0,
                                af: 0x8 as libc::c_uint as uint8_t,
                                txIrq:
                                    DMA2_ST6_HANDLER as libc::c_int as
                                        uint8_t,
                                rxIrq: USART6_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          0 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x5
                                                                                                              as
                                                                                                              libc::c_uint))
                                          |
                                          2 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x5
                                                                                      as
                                                                                      libc::c_uint))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// USE_UART
