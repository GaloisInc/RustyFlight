use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_RCC_GetSysClockFreq() -> uint32_t;
    #[no_mangle]
    fn HAL_RCC_GetPCLK1Freq() -> uint32_t;
    #[no_mangle]
    fn HAL_RCC_GetPCLK2Freq() -> uint32_t;
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort_IT(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    /* *
  ******************************************************************************
  * @file    stm32f7xx_hal.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file contains all the functions prototypes for the HAL 
  *          module driver.
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
    /* * @addtogroup HAL
  * @{
  */
    /* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup HAL_Exported_Constants HAL Exported Constants
  * @{
  */
    /* * @defgroup SYSCFG_BootMode Boot Mode
  * @{
  */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup HAL_Exported_Macros HAL Exported Macros
  * @{
  */
    /* * @brief  Freeze/Unfreeze Peripherals in Debug mode 
  */
    /* * @brief  FMC (NOR/RAM) mapped at 0x60000000 and SDRAM mapped at 0xC0000000
  */
    /* * @brief  FMC/SDRAM  mapped at 0x60000000 (NOR/RAM) mapped at 0xC0000000
  */
    /* *
  * @brief  Return the memory boot mapping as configured by user.
  * @retval The boot mode as configured by user. The returned value can be one
  *         of the following values:
  *           @arg @ref SYSCFG_MEM_BOOT_ADD0
  *           @arg @ref SYSCFG_MEM_BOOT_ADD1
  */
    /* STM32F765xx || STM32F767xx || STM32F769xx || STM32F777xx || STM32F779xx */
    /* *
  * @}
  */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup HAL_Exported_Functions
  * @{
  */
/* * @addtogroup HAL_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ******************************/
    /* *
  * @}
  */
    /* * @addtogroup HAL_Exported_Functions_Group2
  * @{
  */ 
/* Peripheral Control functions  ************************************************/
    #[no_mangle]
    fn HAL_GetTick() -> uint32_t;
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
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub PLLCFGR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub AHB1RSTR: uint32_t,
    pub AHB2RSTR: uint32_t,
    pub AHB3RSTR: uint32_t,
    pub RESERVED0: uint32_t,
    pub APB1RSTR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub RESERVED1: [uint32_t; 2],
    pub AHB1ENR: uint32_t,
    pub AHB2ENR: uint32_t,
    pub AHB3ENR: uint32_t,
    pub RESERVED2: uint32_t,
    pub APB1ENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub RESERVED3: [uint32_t; 2],
    pub AHB1LPENR: uint32_t,
    pub AHB2LPENR: uint32_t,
    pub AHB3LPENR: uint32_t,
    pub RESERVED4: uint32_t,
    pub APB1LPENR: uint32_t,
    pub APB2LPENR: uint32_t,
    pub RESERVED5: [uint32_t; 2],
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub RESERVED6: [uint32_t; 2],
    pub SSCGR: uint32_t,
    pub PLLI2SCFGR: uint32_t,
    pub PLLSAICFGR: uint32_t,
    pub DCKCFGR1: uint32_t,
    pub DCKCFGR2: uint32_t,
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
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
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
pub type UART_ClockSourceTypeDef = libc::c_uint;
pub const UART_CLOCKSOURCE_UNDEFINED: UART_ClockSourceTypeDef = 16;
pub const UART_CLOCKSOURCE_LSE: UART_ClockSourceTypeDef = 8;
pub const UART_CLOCKSOURCE_SYSCLK: UART_ClockSourceTypeDef = 4;
pub const UART_CLOCKSOURCE_HSI: UART_ClockSourceTypeDef = 2;
pub const UART_CLOCKSOURCE_PCLK2: UART_ClockSourceTypeDef = 1;
pub const UART_CLOCKSOURCE_PCLK1: UART_ClockSourceTypeDef = 0;
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
/* !< DMA Stream Index                       */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup UART_Exported_Functions UART Exported Functions
  * @{
  */
/* * @defgroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the USARTx or the UARTy
    in asynchronous mode.
      (+) For the asynchronous mode only these parameters can be configured:
        (++) Baud Rate
        (++) Word Length
        (++) Stop Bit
        (++) Parity: If the parity is enabled, then the MSB bit of the data written
             in the data register is transmitted but is changed by the parity bit.
             Depending on the frame length defined by the M bit (8-bits or 9-bits),
             please refer to Reference manual for possible UART frame formats.
        (++) Hardware flow control
        (++) Receiver/transmitter modes
        (++) Over Sampling Method
    [..]
    The HAL_UART_Init(), HAL_HalfDuplex_Init(), HAL_LIN_Init() and HAL_MultiProcessor_Init() APIs
    follow respectively the UART asynchronous, UART Half duplex, LIN and Multi-Processor
    configuration procedures (details for the procedures are available in reference manual (RM0329)).

@endverbatim
  * @{
  */
/* *
  * @brief Initializes the UART mode according to the specified
  *         parameters in the UART_InitTypeDef and creates the associated handle .
  * @param huart: uart handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Init(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    ((*huart).Init.HwFlowCtl) != 0 as libc::c_uint;
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*huart).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_UART_MspInit(huart);
    }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the UART Communication parameters */
    if UART_SetConfig(huart) as libc::c_uint ==
           HAL_ERROR as libc::c_int as libc::c_uint {
        return HAL_ERROR
    }
    if (*huart).AdvancedInit.AdvFeatureInit != 0 as libc::c_uint {
        UART_AdvFeatureConfig(huart);
    }
    /* In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           14 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               11 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               1 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
    return UART_CheckIdleState(huart);
}
/* *
  * @brief Initializes the half-duplex mode according to the specified
  *         parameters in the UART_InitTypeDef and creates the associated handle .
  * @param huart: UART handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_HalfDuplex_Init(mut huart:
                                                 *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*huart).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_UART_MspInit(huart);
    }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the UART Communication parameters */
    if UART_SetConfig(huart) as libc::c_uint ==
           HAL_ERROR as libc::c_int as libc::c_uint {
        return HAL_ERROR
    }
    if (*huart).AdvancedInit.AdvFeatureInit != 0 as libc::c_uint {
        UART_AdvFeatureConfig(huart);
    }
    /* In half-duplex mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN and IREN bits in the USART_CR3 register.*/
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           14 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               11 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Enable the Half-Duplex mode by setting the HDSEL bit in the CR3 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         3 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
    return UART_CheckIdleState(huart);
}
/* *
  * @brief Initialize the LIN mode according to the specified
  *        parameters in the UART_InitTypeDef and creates the associated handle .
  * @param huart: UART handle.
  * @param BreakDetectLength: specifies the LIN break detection length.
  *        This parameter can be one of the following values:
  *          @arg @ref UART_LINBREAKDETECTLENGTH_10B 10-bit break detection
  *          @arg @ref UART_LINBREAKDETECTLENGTH_11B 11-bit break detection
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_LIN_Init(mut huart: *mut UART_HandleTypeDef,
                                      mut BreakDetectLength: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*huart).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_UART_MspInit(huart);
    }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the UART Communication parameters */
    if UART_SetConfig(huart) as libc::c_uint ==
           HAL_ERROR as libc::c_int as libc::c_uint {
        return HAL_ERROR
    }
    if (*huart).AdvancedInit.AdvFeatureInit != 0 as libc::c_uint {
        UART_AdvFeatureConfig(huart);
    }
    /* In LIN mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN and IREN bits in the USART_CR3 register.*/
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           11 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Enable the LIN mode by setting the LINEN bit in the CR2 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         14 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Set the USART LIN Break detection length. */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR2 &
                                    !((0x1 as libc::c_uint) <<
                                          5 as libc::c_uint) |
                                    BreakDetectLength);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
    return UART_CheckIdleState(huart);
}
/* *
  * @brief Initialize the multiprocessor mode according to the specified
  *        parameters in the UART_InitTypeDef and initialize the associated handle.
  * @param huart: UART handle.
  * @param Address: UART node address (4-, 6-, 7- or 8-bit long).
  * @param WakeUpMethod: specifies the UART wakeup method.
  *        This parameter can be one of the following values:
  *          @arg @ref UART_WAKEUPMETHOD_IDLELINE WakeUp by an idle line detection
  *          @arg @ref UART_WAKEUPMETHOD_ADDRESSMARK WakeUp by an address mark
  * @note  If the user resorts to idle line detection wake up, the Address parameter
  *        is useless and ignored by the initialization function.
  * @note  If the user resorts to address mark wake up, the address length detection
  *        is configured by default to 4 bits only. For the UART to be able to
  *        manage 6-, 7- or 8-bit long addresses detection
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MultiProcessor_Init(mut huart:
                                                     *mut UART_HandleTypeDef,
                                                 mut Address: uint8_t,
                                                 mut WakeUpMethod: uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    /* Check the wake up method parameter */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*huart).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_UART_MspInit(huart);
    }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the UART Communication parameters */
    if UART_SetConfig(huart) as libc::c_uint ==
           HAL_ERROR as libc::c_int as libc::c_uint {
        return HAL_ERROR
    }
    if (*huart).AdvancedInit.AdvFeatureInit != 0 as libc::c_uint {
        UART_AdvFeatureConfig(huart);
    }
    /* In multiprocessor mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register. */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           14 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               11 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               1 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    if WakeUpMethod == (0x1 as libc::c_uint) << 11 as libc::c_uint {
        /* If address mark wake up method is chosen, set the USART address node */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0xff as libc::c_uint) <<
                                              24 as libc::c_uint) |
                                        (Address as uint32_t) <<
                                            24 as libc::c_uint)
    }
    /* Set the wake up method by setting the WAKE bit in the CR1 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR1 &
                                    !((0x1 as libc::c_uint) <<
                                          11 as libc::c_uint) | WakeUpMethod);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
    return UART_CheckIdleState(huart);
}
/* *
  * @brief Initialize the RS485 Driver enable feature according to the specified
  *         parameters in the UART_InitTypeDef and creates the associated handle.
  * @param huart: UART handle.
  * @param Polarity: select the driver enable polarity.
  *        This parameter can be one of the following values:
  *          @arg @ref UART_DE_POLARITY_HIGH DE signal is active high
  *          @arg @ref UART_DE_POLARITY_LOW  DE signal is active low
  * @param AssertionTime: Driver Enable assertion time:
  *                         5-bit value defining the time between the activation of the DE (Driver Enable)
  *                         signal and the beginning of the start bit. It is expressed in sample time
  *                         units (1/8 or 1/16 bit time, depending on the oversampling rate)
  * @param DeassertionTime: Driver Enable deassertion time:
  *                         5-bit value defining the time between the end of the last stop bit, in a
  *                         transmitted message, and the de-activation of the DE (Driver Enable) signal.
  *                         It is expressed in sample time units (1/8 or 1/16 bit time, depending on the
  *                         oversampling rate).
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_RS485Ex_Init(mut huart: *mut UART_HandleTypeDef,
                                          mut Polarity: uint32_t,
                                          mut AssertionTime: uint32_t,
                                          mut DeassertionTime: uint32_t)
 -> HAL_StatusTypeDef {
    let mut temp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    /* Check the Driver Enable UART instance */
    /* Check the Driver Enable polarity */
    /* Check the Driver Enable assertion time */
    /* Check the Driver Enable deassertion time */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*huart).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, CORTEX */
        HAL_UART_MspInit(huart);
    }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the UART Communication parameters */
    if UART_SetConfig(huart) as libc::c_uint ==
           HAL_ERROR as libc::c_int as libc::c_uint {
        return HAL_ERROR
    }
    if (*huart).AdvancedInit.AdvFeatureInit != 0 as libc::c_uint {
        UART_AdvFeatureConfig(huart);
    }
    /* Enable the Driver Enable mode by setting the DEM bit in the CR3 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         14 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Set the Driver Enable polarity */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR3 &
                                    !((0x1 as libc::c_uint) <<
                                          15 as libc::c_uint) | Polarity);
    /* Set the Driver Enable assertion and deassertion times */
    temp = AssertionTime << 21 as libc::c_uint;
    temp |= DeassertionTime << 16 as libc::c_uint;
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR1 &
                                    !((0x1f as libc::c_uint) <<
                                          16 as libc::c_uint |
                                          (0x1f as libc::c_uint) <<
                                              21 as libc::c_uint) | temp);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
    return UART_CheckIdleState(huart);
}
/* *
  * @brief DeInitializes the UART peripheral
  * @param huart: uart handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_DeInit(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t, 0 as libc::c_uint);
    /* DeInit the low level hardware */
    HAL_UART_MspDeInit(huart);
    ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_RESET);
    ::core::ptr::write_volatile(&mut (*huart).RxState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_RESET);
    /* Process Unlock */
    (*huart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief UART MSP Init
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_MspInit(mut huart:
                                              *mut UART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_MspInit can be implemented in the user file
   */
}
/* *
  * @brief UART MSP DeInit
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_MspDeInit(mut huart:
                                                *mut UART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_MspDeInit can be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup UART_Exported_Functions_Group2 IO operation functions
  * @brief UART Transmit/Receive functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the UART asynchronous
    and Half duplex data transfers.

    (#) There are two mode of transfer:
       (+) Blocking mode: The communication is performed in polling mode.
           The HAL status of all data processing is returned by the same function
           after finishing transfer.
       (+) Non-Blocking mode: The communication is performed using Interrupts
           or DMA, These API's return the HAL status.
           The end of the data processing will be indicated through the
           dedicated UART IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The HAL_UART_TxCpltCallback(), HAL_UART_RxCpltCallback() user callbacks
           will be executed respectively at the end of the transmit or Receive process
           The HAL_UART_ErrorCallback()user callback will be executed when a communication error is detected

    (#) Blocking mode API's are :
        (+) HAL_UART_Transmit()
        (+) HAL_UART_Receive()

    (#) Non-Blocking mode API's with Interrupt are :
        (+) HAL_UART_Transmit_IT()
        (+) HAL_UART_Receive_IT()
        (+) HAL_UART_IRQHandler()
        (+) UART_Transmit_IT()
        (+) UART_Receive_IT()

    (#) Non-Blocking mode API's with DMA are :
        (+) HAL_UART_Transmit_DMA()
        (+) HAL_UART_Receive_DMA()
        (+) HAL_UART_DMAPause()
        (+) HAL_UART_DMAResume()
        (+) HAL_UART_DMAStop()

    (#) A set of Transfer Complete Callbacks are provided in Non_Blocking mode:
        (+) HAL_UART_TxHalfCpltCallback()
        (+) HAL_UART_TxCpltCallback()
        (+) HAL_UART_RxHalfCpltCallback()
        (+) HAL_UART_RxCpltCallback()
        (+) HAL_UART_ErrorCallback()


    -@- In the Half duplex communication, it is forbidden to run the transmit
        and receive process in parallel, the UART state HAL_UART_STATE_BUSY_TX_RX can't be useful.

@endverbatim
  * @{
  */
/* *
  * @brief Send an amount of data in blocking mode.
  * @param huart: UART handle.
  * @param pData: Pointer to data buffer.
  * @param Size: Amount of data to be sent.
  * @param Timeout: Timeout duration.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Transmit(mut huart: *mut UART_HandleTypeDef,
                                           mut pData: *mut uint8_t,
                                           mut Size: uint16_t,
                                           mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Check that a Tx process is not already ongoing */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*huart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*huart).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*huart).gState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_BUSY_TX);
        /* Init tickstart for timeout managment*/
        tickstart = HAL_GetTick();
        (*huart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                        *mut uint16_t, Size);
        while (*huart).TxXferCount as libc::c_uint > 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*huart).TxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            if UART_WaitOnFlagUntilTimeout(huart, 0x80 as libc::c_uint, RESET,
                                           tickstart, Timeout) as libc::c_uint
                   != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            if (*huart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*huart).Init.Parity == 0 as libc::c_uint {
                tmp = pData as *mut uint16_t;
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).TDR as
                                                *mut uint32_t,
                                            (*tmp as libc::c_int &
                                                 0x1ff as libc::c_uint as
                                                     uint16_t as libc::c_int)
                                                as uint32_t);
                pData = pData.offset(2 as libc::c_int as isize)
            } else {
                let fresh0 = pData;
                pData = pData.offset(1);
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).TDR as
                                                *mut uint32_t,
                                            (*fresh0 as libc::c_int &
                                                 0xff as libc::c_uint as
                                                     uint8_t as libc::c_int)
                                                as uint32_t)
            }
        }
        if UART_WaitOnFlagUntilTimeout(huart, 0x40 as libc::c_uint, RESET,
                                       tickstart, Timeout) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        /* At end of Tx process, restore huart->gState to Ready */
        ::core::ptr::write_volatile(&mut (*huart).gState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_READY);
        /* Process Unlocked */
        (*huart).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Receive an amount of data in blocking mode.
  * @param huart: UART handle.
  * @param pData: pointer to data buffer.
  * @param Size: amount of data to be received.
  * @param Timeout: Timeout duration.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Receive(mut huart: *mut UART_HandleTypeDef,
                                          mut pData: *mut uint8_t,
                                          mut Size: uint16_t,
                                          mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut uhMask: uint16_t = 0;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Check that a Rx process is not already ongoing */
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*huart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*huart).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*huart).RxState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_BUSY_RX);
        /* Init tickstart for timeout managment*/
        tickstart = HAL_GetTick();
        (*huart).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*huart).RxXferCount as
                                        *mut uint16_t, Size);
        /* Computation of UART mask to apply to RDR register */
        if (*huart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            if (*huart).Init.Parity == 0 as libc::c_uint {
                (*huart).Mask = 0x1ff as libc::c_int as uint16_t
            } else { (*huart).Mask = 0xff as libc::c_int as uint16_t }
        } else if (*huart).Init.WordLength == 0 as libc::c_uint {
            if (*huart).Init.Parity == 0 as libc::c_uint {
                (*huart).Mask = 0xff as libc::c_int as uint16_t
            } else { (*huart).Mask = 0x7f as libc::c_int as uint16_t }
        } else if (*huart).Init.WordLength == 0x10000000 as libc::c_uint {
            if (*huart).Init.Parity == 0 as libc::c_uint {
                (*huart).Mask = 0x7f as libc::c_int as uint16_t
            } else { (*huart).Mask = 0x3f as libc::c_int as uint16_t }
        }
        uhMask = (*huart).Mask;
        /* as long as data have to be received */
        while (*huart).RxXferCount as libc::c_uint > 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*huart).RxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*huart).RxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            if UART_WaitOnFlagUntilTimeout(huart, 0x20 as libc::c_uint, RESET,
                                           tickstart, Timeout) as libc::c_uint
                   != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            if (*huart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*huart).Init.Parity == 0 as libc::c_uint {
                tmp = pData as *mut uint16_t;
                *tmp =
                    ((*(*huart).Instance).RDR & uhMask as libc::c_uint) as
                        uint16_t;
                pData = pData.offset(2 as libc::c_uint as isize)
            } else {
                let fresh1 = pData;
                pData = pData.offset(1);
                *fresh1 =
                    ((*(*huart).Instance).RDR &
                         uhMask as uint8_t as libc::c_uint) as uint8_t
            }
        }
        /* At end of Rx process, restore huart->RxState to Ready */
        ::core::ptr::write_volatile(&mut (*huart).RxState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_READY);
        /* Process Unlocked */
        (*huart).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Send an amount of data in interrupt mode.
  * @param huart: UART handle.
  * @param pData: pointer to data buffer.
  * @param Size: amount of data to be sent.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Transmit_IT(mut huart:
                                                  *mut UART_HandleTypeDef,
                                              mut pData: *mut uint8_t,
                                              mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check that a Tx process is not already ongoing */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*huart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*huart).Lock = HAL_LOCKED }
        (*huart).pTxBuffPtr = pData;
        (*huart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                        *mut uint16_t, Size);
        ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*huart).gState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_BUSY_TX);
        /* Process Unlocked */
        (*huart).Lock = HAL_UNLOCKED;
        /* Enable the UART Transmit Data Register Empty Interrupt */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Receive an amount of data in interrupt mode.
  * @param huart: UART handle.
  * @param pData: pointer to data buffer.
  * @param Size: amount of data to be received.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Receive_IT(mut huart:
                                                 *mut UART_HandleTypeDef,
                                             mut pData: *mut uint8_t,
                                             mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    /* Check that a Rx process is not already ongoing */
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*huart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*huart).Lock = HAL_LOCKED }
        (*huart).pRxBuffPtr = pData;
        (*huart).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*huart).RxXferCount as
                                        *mut uint16_t, Size);
        /* Computation of UART mask to apply to RDR register */
        if (*huart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            if (*huart).Init.Parity == 0 as libc::c_uint {
                (*huart).Mask = 0x1ff as libc::c_int as uint16_t
            } else { (*huart).Mask = 0xff as libc::c_int as uint16_t }
        } else if (*huart).Init.WordLength == 0 as libc::c_uint {
            if (*huart).Init.Parity == 0 as libc::c_uint {
                (*huart).Mask = 0xff as libc::c_int as uint16_t
            } else { (*huart).Mask = 0x7f as libc::c_int as uint16_t }
        } else if (*huart).Init.WordLength == 0x10000000 as libc::c_uint {
            if (*huart).Init.Parity == 0 as libc::c_uint {
                (*huart).Mask = 0x7f as libc::c_int as uint16_t
            } else { (*huart).Mask = 0x3f as libc::c_int as uint16_t }
        }
        ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*huart).RxState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_BUSY_RX);
        /* Process Unlocked */
        (*huart).Lock = HAL_UNLOCKED;
        /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the UART Parity Error and Data Register not empty Interrupts */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Send an amount of data in DMA mode.
  * @param huart: UART handle.
  * @param pData: pointer to data buffer.
  * @param Size: amount of data to be sent.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Transmit_DMA(mut huart:
                                                   *mut UART_HandleTypeDef,
                                               mut pData: *mut uint8_t,
                                               mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint32_t = 0 as *mut uint32_t;
    /* Check that a Tx process is not already ongoing */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*huart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*huart).Lock = HAL_LOCKED }
        (*huart).pTxBuffPtr = pData;
        (*huart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                        *mut uint16_t, Size);
        ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*huart).gState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_BUSY_TX);
        /* Set the UART DMA transfer complete callback */
        (*(*huart).hdmatx).XferCpltCallback =
            Some(UART_DMATransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the UART DMA Half transfer complete callback */
        (*(*huart).hdmatx).XferHalfCpltCallback =
            Some(UART_DMATxHalfCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*huart).hdmatx).XferErrorCallback =
            Some(UART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA abort callback */
        (*(*huart).hdmatx).XferAbortCallback = None;
        /* Enable the UART transmit DMA channel */
        tmp = &mut pData as *mut *mut uint8_t as *mut uint32_t;
        HAL_DMA_Start_IT((*huart).hdmatx, *tmp,
                         &mut (*(*huart).Instance).TDR as *mut uint32_t as
                             uint32_t, Size as uint32_t);
        /* Clear the TC flag in the SR register by writing 0 to it */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                        *mut uint32_t, 0x40 as libc::c_uint);
        /* Process Unlocked */
        (*huart).Lock = HAL_UNLOCKED;
        /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the UART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Receive an amount of data in DMA mode.
  * @param huart: UART handle.
  * @param pData: pointer to data buffer.
  * @param Size: amount of data to be received.
  * @note   When the UART parity is enabled (PCE = 1), the received data contain
  *         the parity bit (MSB position).
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_Receive_DMA(mut huart:
                                                  *mut UART_HandleTypeDef,
                                              mut pData: *mut uint8_t,
                                              mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint32_t = 0 as *mut uint32_t;
    /* Check that a Rx process is not already ongoing */
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*huart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*huart).Lock = HAL_LOCKED }
        (*huart).pRxBuffPtr = pData;
        (*huart).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*huart).RxState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_BUSY_RX);
        /* Set the UART DMA transfer complete callback */
        (*(*huart).hdmarx).XferCpltCallback =
            Some(UART_DMAReceiveCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the UART DMA Half transfer complete callback */
        (*(*huart).hdmarx).XferHalfCpltCallback =
            Some(UART_DMARxHalfCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*huart).hdmarx).XferErrorCallback =
            Some(UART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA abort callback */
        (*(*huart).hdmarx).XferAbortCallback = None;
        /* Enable the DMA channel */
        tmp = &mut pData as *mut *mut uint8_t as *mut uint32_t;
        HAL_DMA_Start_IT((*huart).hdmarx,
                         &mut (*(*huart).Instance).RDR as *mut uint32_t as
                             uint32_t, *tmp, Size as uint32_t);
        /* Process Unlocked */
        (*huart).Lock = HAL_UNLOCKED;
        /* Enable the UART Parity Error Interrupt */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Pause the DMA Transfer.
  * @param huart: UART handle.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_DMAPause(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint &&
           (*(*huart).Instance).CR3 &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Disable the UART DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_BUSY_RX as libc::c_int as libc::c_uint &&
           (*(*huart).Instance).CR3 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               8 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the UART DMA Rx request */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Process Unlocked */
    (*huart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief Resume the DMA Transfer.
  * @param huart: UART handle.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_DMAResume(mut huart:
                                                *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        /* Enable the UART DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        /* Clear the Overrun flag before resuming the Rx transfer*/
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint);
        /* Reenable PE and ERR (Frame error, noise error, overrun error) interrupts */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the UART DMA Rx request */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* If the UART peripheral is still not enabled, enable it */
    if (*(*huart).Instance).CR1 & (0x1 as libc::c_uint) << 0 as libc::c_uint
           == 0 as libc::c_uint {
        /* Enable UART peripheral */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief Stop the DMA Transfer.
  * @param huart: UART handle.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_DMAStop(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* The Lock is not implemented on this API to allow the user application
     to call the HAL UART API under callbacks HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback() /
     HAL_UART_TxHalfCpltCallback / HAL_UART_RxHalfCpltCallback:
     indeed, when HAL_DMA_Abort() API is called, the DMA TX/RX Transfer or Half Transfer complete
     interrupt is generated if the DMA transfer interruption occurs at the middle or at the end of
     the stream and the corresponding call back is executed. */
    /* Stop UART DMA Tx request if ongoing */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint &&
           (*(*huart).Instance).CR3 &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Abort the UART DMA Tx channel */
        if !(*huart).hdmatx.is_null() { HAL_DMA_Abort((*huart).hdmatx); }
        UART_EndTxTransfer(huart);
    }
    /* Stop UART DMA Rx request if ongoing */
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_BUSY_RX as libc::c_int as libc::c_uint &&
           (*(*huart).Instance).CR3 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Abort the UART DMA Rx channel */
        if !(*huart).hdmarx.is_null() { HAL_DMA_Abort((*huart).hdmarx); }
        UART_EndRxTransfer(huart);
    }
    return HAL_OK;
}
/* *
  * @brief This function handles UART interrupt request.
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_IRQHandler(mut huart:
                                                 *mut UART_HandleTypeDef) {
    let mut isrflags: uint32_t = (*(*huart).Instance).ISR;
    let mut cr1its: uint32_t = (*(*huart).Instance).CR1;
    let mut cr3its: uint32_t = (*(*huart).Instance).CR3;
    let mut errorflags: uint32_t = 0;
    /* If no error occurs */
    errorflags =
        isrflags &
            ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                 (0x1 as libc::c_uint) << 1 as libc::c_uint |
                 (0x1 as libc::c_uint) << 3 as libc::c_uint |
                 (0x1 as libc::c_uint) << 2 as libc::c_uint);
    if errorflags == RESET as libc::c_int as libc::c_uint {
        /* UART in mode Receiver ---------------------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr1its & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            UART_Receive_IT(huart);
            return
        }
    }
    /* If some errors occur */
    if errorflags != RESET as libc::c_int as libc::c_uint &&
           (cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                RESET as libc::c_int as libc::c_uint ||
                cr1its &
                    ((0x1 as libc::c_uint) << 5 as libc::c_uint |
                         (0x1 as libc::c_uint) << 8 as libc::c_uint) !=
                    RESET as libc::c_int as libc::c_uint)
       { /* End if some error occurs */
        /* UART parity error interrupt occurred -------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr1its & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*huart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*huart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* UART frame error interrupt occurred --------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            1 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*huart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*huart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* UART noise error interrupt occurred --------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            2 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*huart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*huart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x2 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* UART Over-Run interrupt occurred -----------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 3 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               (cr1its & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                    RESET as libc::c_int as libc::c_uint ||
                    cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                        RESET as libc::c_int as libc::c_uint) {
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*huart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*huart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x8 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* Call UART Error Call back function if need be --------------------------*/
        if (*huart).ErrorCode != 0 as libc::c_uint {
            /* UART in mode Receiver ---------------------------------------------------*/
            if isrflags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint &&
                   cr1its & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                UART_Receive_IT(huart);
            }
            /* If Overrun error occurs, or if any error occurs in DMA mode reception,
         consider error as blocking */
            if (*huart).ErrorCode & 0x8 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint ||
                   (*(*huart).Instance).CR3 &
                       (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                /* Blocking error : transfer is aborted
           Set the UART state ready to be able to start again the process,
           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
                UART_EndRxTransfer(huart);
                /* Disable the UART DMA Rx request if enabled */
                if (*(*huart).Instance).CR3 &
                       (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint))
                                                    as uint32_t as uint32_t);
                    /* Abort the UART DMA Rx channel */
                    if !(*huart).hdmarx.is_null() {
                        /* Set the UART DMA Abort callback :
            will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
                        (*(*huart).hdmarx).XferAbortCallback =
                            Some(UART_DMAAbortOnError as
                                     unsafe extern "C" fn(_:
                                                              *mut DMA_HandleTypeDef)
                                         -> ());
                        /* Abort DMA RX */
                        if HAL_DMA_Abort_IT((*huart).hdmarx) as libc::c_uint
                               != HAL_OK as libc::c_int as libc::c_uint {
                            /* Call Directly huart->hdmarx->XferAbortCallback function in case of error */
                            (*(*huart).hdmarx).XferAbortCallback.expect("non-null function pointer")((*huart).hdmarx);
                        }
                    } else {
                        /* Call user error callback */
                        HAL_UART_ErrorCallback(huart);
                    }
                } else {
                    /* Call user error callback */
                    HAL_UART_ErrorCallback(huart);
                }
            } else {
                /* Non Blocking error : transfer could go on.
           Error is notified to user through user error callback */
                HAL_UART_ErrorCallback(huart);
                ::core::ptr::write_volatile(&mut (*huart).ErrorCode as
                                                *mut uint32_t,
                                            0 as libc::c_uint)
            }
        }
        return
    }
    /* UART in mode Transmitter ------------------------------------------------*/
    if isrflags & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           cr1its & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        UART_Transmit_IT(huart);
        return
    }
    /* UART in mode Transmitter (transmission end) -----------------------------*/
    if isrflags & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           cr1its & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        UART_EndTransmit_IT(huart);
        return
    };
}
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
/* !< UART registers base address        */
/* !< UART communication parameters      */
/* !< UART Advanced Features initialization parameters */
/* !< Pointer to UART Tx transfer Buffer */
/* !< UART Tx Transfer size              */
/* !< UART Tx Transfer Counter           */
/* !< Pointer to UART Rx transfer Buffer */
/* !< UART Rx Transfer size              */
/* !< UART Rx Transfer Counter           */
/* !< UART Rx RDR register mask          */
/* !< UART Tx DMA Handle parameters      */
/* !< UART Rx DMA Handle parameters      */
/* !< Locking object                     */
/* !< UART state information related to global Handle management 
                                                  and also related to Tx operations.
                                                  This parameter can be a value of @ref HAL_UART_StateTypeDef */
/* !< UART state information related to Rx operations.
                                                  This parameter can be a value of @ref HAL_UART_StateTypeDef */
/* !< UART Error code                    */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup UART_Exported_Constants UART Exported Constants
  * @{
  */
/* * @defgroup UART_Error_Definition   UART Error Definition
  * @{
  */
/* !< No error            */
/* !< Parity error        */
/* !< Noise error         */
/* !< frame error         */
/* !< Overrun error       */
/* !< DMA transfer error  */
/* *
  * @}
  */
/* * @defgroup UART_Stop_Bits   UART Number of Stop Bits
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Parity  UART Parity
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Hardware_Flow_Control UART Hardware Flow Control
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Mode UART Transfer Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_State  UART State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Over_Sampling UART Over Sampling
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_OneBit_Sampling UART One Bit Sampling Method
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_AutoBaud_Rate_Mode    UART Advanced Feature AutoBaud Rate Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Receiver_TimeOut UART Receiver TimeOut
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_LIN    UART Local Interconnection Network mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_LIN_Break_Detection  UART LIN Break Detection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_DMA_Tx    UART DMA Tx
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_DMA_Rx   UART DMA Rx
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Half_Duplex_Selection  UART Half Duplex Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_WakeUp_Methods   UART WakeUp Methods
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Request_Parameters UART Request Parameters
  * @{
  */
/* !< Auto-Baud Rate Request */
/* !< Send Break Request */
/* !< Mute Mode Request */
/* !< Receive Data flush Request */
/* !< Transmit data flush Request */
/* *
  * @}
  */
/* * @defgroup UART_Advanced_Features_Initialization_Type  UART Advanced Feature Initialization Type
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Tx_Inv UART Advanced Feature TX Pin Active Level Inversion
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Rx_Inv UART Advanced Feature RX Pin Active Level Inversion
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Data_Inv  UART Advanced Feature Binary Data Inversion
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Rx_Tx_Swap UART Advanced Feature RX TX Pins Swap
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Overrun_Disable  UART Advanced Feature Overrun Disable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_AutoBaudRate_Enable  UART Advanced Feature Auto BaudRate Enable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_DMA_Disable_on_Rx_Error   UART Advanced Feature DMA Disable On Rx Error
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_MSB_First   UART Advanced Feature MSB First
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Mute_Mode   UART Advanced Feature Mute Mode Enable
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_CR2_ADDRESS_LSB_POS    UART Address-matching LSB Position In CR2 Register
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_DriverEnable_Polarity      UART DriverEnable Polarity
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_CR1_DEAT_ADDRESS_LSB_POS    UART Driver Enable Assertion Time LSB Position In CR1 Register
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_CR1_DEDT_ADDRESS_LSB_POS    UART Driver Enable DeAssertion Time LSB Position In CR1 Register
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Interruption_Mask    UART Interruptions Flag Mask
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_TimeOut_Value    UART polling-based communications time-out value
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Flags     UART Status Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the ISR register
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UART_Interrupt_definition   UART Interrupts Definition
  *        Elements values convention: 0000ZZZZ0XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  *           - ZZZZ  : Flag position in the ISR register(4bits)
  * @{
  */
/* *       Elements values convention: 000000000XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  */
/* *       Elements values convention: 0000ZZZZ00000000b
  *           - ZZZZ  : Flag position in the ISR register(4bits)
  */
/* *
  * @}
  */
/* * @defgroup UART_IT_CLEAR_Flags  UART Interruption Clear Flags
  * @{
  */
/* !< Parity Error Clear Flag */
/* !< Framing Error Clear Flag */
/* !< Noise detected Clear Flag */
/* !< OverRun Error Clear Flag */
/* !< IDLE line detected Clear Flag */
/* !< Transmission Complete Clear Flag */
/* !< LIN Break Detection Clear Flag */
/* !< CTS Interrupt Clear Flag */
/* !< Receiver Time Out Clear Flag */
/* !< End Of Block Clear Flag */
/* !< Character Match Clear Flag */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macros -----------------------------------------------------------*/
/* * @defgroup UART_Exported_Macros UART Exported Macros
  * @{
  */
/* * @brief Reset UART handle state
  * @param  __HANDLE__: UART handle.
  * @retval None
  */
/* * @brief  Flush the UART Data registers
  * @param  __HANDLE__: specifies the UART Handle.
  */
/* * @brief  Clears the specified UART ISR flag, in setting the proper ICR register flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __FLAG__: specifies the interrupt clear register flag that needs to be set
  *                       to clear the corresponding interrupt
  *          This parameter can be one of the following values:
  *            @arg UART_CLEAR_PEF: Parity Error Clear Flag
  *            @arg UART_CLEAR_FEF: Framing Error Clear Flag
  *            @arg UART_CLEAR_NEF: Noise detected Clear Flag
  *            @arg UART_CLEAR_OREF: OverRun Error Clear Flag
  *            @arg UART_CLEAR_IDLEF: IDLE line detected Clear Flag
  *            @arg UART_CLEAR_TCF: Transmission Complete Clear Flag
  *            @arg UART_CLEAR_LBDF: LIN Break Detection Clear Flag
  *            @arg UART_CLEAR_CTSF: CTS Interrupt Clear Flag
  *            @arg UART_CLEAR_RTOF: Receiver Time Out Clear Flag
  *            @arg UART_CLEAR_EOBF: End Of Block Clear Flag
  *            @arg UART_CLEAR_CMF: Character Match Clear Flag
  * @retval None
  */
/* * @brief  Clear the UART PE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Clear the UART FE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Clear the UART NE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Clear the UART ORE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Clear the UART IDLE pending flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Checks whether the specified UART flag is set or not.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __FLAG__: specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg UART_FLAG_REACK: Receive enable acknowledge flag
  *            @arg UART_FLAG_TEACK: Transmit enable acknowledge flag
  *            @arg UART_FLAG_WUF:   Wake up from stop mode flag
  *            @arg UART_FLAG_RWU:   Receiver wake up flag (is the UART in mute mode)
  *            @arg UART_FLAG_SBKF:  Send Break flag
  *            @arg UART_FLAG_CMF:   Character match flag
  *            @arg UART_FLAG_BUSY:  Busy flag
  *            @arg UART_FLAG_ABRF:  Auto Baud rate detection flag
  *            @arg UART_FLAG_ABRE:  Auto Baud rate detection error flag
  *            @arg UART_FLAG_EOBF:  End of block flag
  *            @arg UART_FLAG_RTOF:  Receiver timeout flag
  *            @arg UART_FLAG_CTS:   CTS Change flag (not available for UART4 and UART5)
  *            @arg UART_FLAG_LBD:   LIN Break detection flag
  *            @arg UART_FLAG_TXE:   Transmit data register empty flag
  *            @arg UART_FLAG_TC:    Transmission Complete flag
  *            @arg UART_FLAG_RXNE:  Receive data register not empty flag
  *            @arg UART_FLAG_IDLE:  Idle Line detection flag
  *            @arg UART_FLAG_ORE:   OverRun Error flag
  *            @arg UART_FLAG_NE:    Noise Error flag
  *            @arg UART_FLAG_FE:    Framing Error flag
  *            @arg UART_FLAG_PE:    Parity Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
/* * @brief  Enables the specified UART interrupt.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __INTERRUPT__: specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_WUF:  Wakeup from stop mode interrupt
  *            @arg UART_IT_CM:   Character match interrupt
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
/* * @brief  Disables the specified UART interrupt.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __INTERRUPT__: specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CM:   Character match interrupt
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
/* * @brief  Checks whether the specified UART interrupt has occurred or not.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __IT__: specifies the UART interrupt to check.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CM:   Character match interrupt
  *            @arg UART_IT_CTS:  CTS change interrupt (not available for UART4 and UART5)
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_ORE:  OverRun Error interrupt
  *            @arg UART_IT_NE:   Noise Error interrupt
  *            @arg UART_IT_FE:   Framing Error interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
/* * @brief  Checks whether the specified UART interrupt source is enabled.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __IT__: specifies the UART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS: CTS change interrupt (not available for UART4 and UART5)
  *            @arg UART_IT_LBD: LIN Break detection interrupt
  *            @arg UART_IT_TXE: Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:  Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_ORE: OverRun Error interrupt
  *            @arg UART_IT_NE: Noise Error interrupt
  *            @arg UART_IT_FE: Framing Error interrupt
  *            @arg UART_IT_PE: Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
/* * @brief  Set a specific UART request flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __REQ__: specifies the request flag to set
  *          This parameter can be one of the following values:
  *            @arg UART_AUTOBAUD_REQUEST: Auto-Baud Rate Request
  *            @arg UART_SENDBREAK_REQUEST: Send Break Request
  *            @arg UART_MUTE_MODE_REQUEST: Mute Mode Request
  *            @arg UART_RXDATA_FLUSH_REQUEST: Receive Data flush Request
  *            @arg UART_TXDATA_FLUSH_REQUEST: Transmit data flush Request
  * @retval None
  */
/* * @brief  Enables the UART one bit sample method
  * @param  __HANDLE__: specifies the UART Handle.  
  * @retval None
  */
/* * @brief  Disables the UART one bit sample method
  * @param  __HANDLE__: specifies the UART Handle.  
  * @retval None
  */
/* * @brief  Enable UART
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Disable UART
  * @param  __HANDLE__: specifies the UART Handle.
  * @retval None
  */
/* * @brief  Enable CTS flow control 
  *         This macro allows to enable CTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying CTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)).                                                                                                                  
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
/* * @brief  Disable CTS flow control 
  *         This macro allows to disable CTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying CTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)). 
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
/* * @brief  Enable RTS flow control 
  *         This macro allows to enable RTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying RTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)). 
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
/* * @brief  Disable RTS flow control 
  *         This macro allows to disable RTS hardware flow control for a given UART instance, 
  *         without need to call HAL_UART_Init() function.
  *         As involving direct access to UART registers, usage of this macro should be fully endorsed by user.
  * @note   As macro is expected to be used for modifying RTS Hw flow control feature activation, without need
  *         for USART instance Deinit/Init, following conditions for macro call should be fulfilled :
  *           - UART instance should have already been initialised (through call of HAL_UART_Init() )
  *           - macro could only be called when corresponding UART instance is disabled (i.e __HAL_UART_DISABLE(__HANDLE__))
  *             and should be followed by an Enable macro (i.e __HAL_UART_ENABLE(__HANDLE__)). 
  * @param  __HANDLE__: specifies the UART Handle.
  *         The Handle Instance can be USART1, USART2 or LPUART.
  * @retval None
  */
/* *
  * @}
  */
/* Private macros --------------------------------------------------------*/
/* * @defgroup UART_Private_Macros   UART Private Macros
  * @{
  */
/* * @brief  BRR division operation to set BRR register with LPUART
  * @param  _PCLK_: LPUART clock
  * @param  _BAUD_: Baud rate set by the user
  * @retval Division result
  */
/* * @brief  BRR division operation to set BRR register in 8-bit oversampling mode
  * @param  _PCLK_: UART clock
  * @param  _BAUD_: Baud rate set by the user
  * @retval Division result
  */
/* * @brief  BRR division operation to set BRR register in 16-bit oversampling mode
  * @param  _PCLK_: UART clock
  * @param  _BAUD_: Baud rate set by the user
  * @retval Division result
  */
/* * @brief  Check UART Baud rate
  * @param  BAUDRATE: Baudrate specified by the user
  *         The maximum Baud Rate is derived from the maximum clock on F7 (i.e. 216 MHz)
  *         divided by the smallest oversampling used on the USART (i.e. 8)
  * @retval Test result (TRUE or FALSE).
  */
/* * @brief  Check UART assertion time
  * @param  TIME: 5-bit value assertion time
  * @retval Test result (TRUE or FALSE).
  */
/* * @brief  Check UART deassertion time
  * @param  TIME: 5-bit value deassertion time
  * @retval Test result (TRUE or FALSE).
  */
/* *
  * @}
  */
/* Include UART HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup UART_Exported_Functions UART Exported Functions
  * @{
  */
/* * @addtogroup UART_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
/* Initialization and de-initialization functions  ****************************/
/* *
  * @}
  */
/* * @addtogroup UART_Exported_Functions_Group2 IO operation functions
  * @{
  */
/* IO operation functions *****************************************************/
/* *
  * @}
  */
/* * @addtogroup UART_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
/* Peripheral Control functions  ************************************************/
/* *
  * @}
  */
/* * @addtogroup UART_Exported_Functions_Group4 Peripheral State and Error functions
  * @{
  */
/* Peripheral State and Errors functions  **************************************************/
/* *
  * @}
  */
/* *
  * @}
  */
/* Private functions -----------------------------------------------------------*/
/* * @addtogroup UART_Private_Functions UART Private Functions
  * @{
  */
/* *
  * @brief  This function handles UART Communication Timeout.
  * @param  huart UART handle
  * @param  Flag specifies the UART flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn UART_WaitOnFlagUntilTimeout(mut huart:
                                                         *mut UART_HandleTypeDef,
                                                     mut Flag: uint32_t,
                                                     mut Status: FlagStatus,
                                                     mut Tickstart: uint32_t,
                                                     mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    /* Wait until flag is set */
    while (if (*(*huart).Instance).ISR & Flag == Flag {
               SET as libc::c_int
           } else { RESET as libc::c_int }) as libc::c_uint ==
              Status as libc::c_uint {
        /* Check for the Timeout */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(Tickstart) >= Timeout {
                /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       5 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           8 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           7 as libc::c_uint))
                                                as uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       0 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*huart).gState as
                                                *mut HAL_UART_StateTypeDef,
                                            HAL_UART_STATE_READY);
                ::core::ptr::write_volatile(&mut (*huart).RxState as
                                                *mut HAL_UART_StateTypeDef,
                                            HAL_UART_STATE_READY);
                /* Process Unlocked */
                (*huart).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @brief DMA UART transmit process complete callback
  * @param hdma: DMA handle
  * @retval None
  */
unsafe extern "C" fn UART_DMATransmitCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut huart: *mut UART_HandleTypeDef =
        (*hdma).Parent as *mut UART_HandleTypeDef;
    /* DMA Normal mode*/
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint ==
           0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                        *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        /* Disable the DMA transfer for transmit request by setting the DMAT bit
       in the UART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Enable the UART Transmit Complete Interrupt */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        /* DMA Circular mode */
        HAL_UART_TxCpltCallback(huart);
    };
}
/* *
  * @brief DMA UART transmit process half complete callback
  * @param hdma : DMA handle
  * @retval None
  */
unsafe extern "C" fn UART_DMATxHalfCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut huart: *mut UART_HandleTypeDef =
        (*hdma).Parent as *mut UART_HandleTypeDef;
    HAL_UART_TxHalfCpltCallback(huart);
}
/* *
  * @brief DMA UART receive process complete callback
  * @param hdma: DMA handle
  * @retval None
  */
unsafe extern "C" fn UART_DMAReceiveCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut huart: *mut UART_HandleTypeDef =
        (*hdma).Parent as *mut UART_HandleTypeDef;
    /* DMA Normal mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint ==
           0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*huart).RxXferCount as
                                        *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        /* Disable PE and ERR (Frame error, noise error, overrun error) interrupts */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               8 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the DMA transfer for the receiver request by setting the DMAR bit
    in the UART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* At end of Rx process, restore huart->RxState to Ready */
        ::core::ptr::write_volatile(&mut (*huart).RxState as
                                        *mut HAL_UART_StateTypeDef,
                                    HAL_UART_STATE_READY)
    }
    HAL_UART_RxCpltCallback(huart);
}
/* *
  * @brief DMA UART receive process half complete callback
  * @param hdma : DMA handle
  * @retval None
  */
unsafe extern "C" fn UART_DMARxHalfCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut huart: *mut UART_HandleTypeDef =
        (*hdma).Parent as *mut UART_HandleTypeDef;
    HAL_UART_RxHalfCpltCallback(huart);
}
/* *
  * @brief DMA UART communication error callback
  * @param hdma: DMA handle
  * @retval None
  */
unsafe extern "C" fn UART_DMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut huart: *mut UART_HandleTypeDef =
        (*hdma).Parent as *mut UART_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*huart).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*huart).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    /* Stop UART DMA Tx request if ongoing */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint &&
           (*(*huart).Instance).CR3 &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        UART_EndTxTransfer(huart);
    }
    /* Stop UART DMA Rx request if ongoing */
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_BUSY_RX as libc::c_int as libc::c_uint &&
           (*(*huart).Instance).CR3 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        UART_EndRxTransfer(huart);
    }
    ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*huart).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x10 as libc::c_uint)
                                    as uint32_t as uint32_t);
    HAL_UART_ErrorCallback(huart);
}
/* *
  * @brief DMA UART communication abort callback, when call by HAL services on Error
  *        (To be called at end of DMA Abort procedure following error occurrence).
  * @param hdma: DMA handle.
  * @retval None
  */
unsafe extern "C" fn UART_DMAAbortOnError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut huart: *mut UART_HandleTypeDef =
        (*hdma).Parent as *mut UART_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*huart).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*huart).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    HAL_UART_ErrorCallback(huart);
}
/* *
  * @brief Tx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_TxCpltCallback(mut huart:
                                                     *mut UART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_TxCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief  Tx Half Transfer completed callbacks.
  * @param  huart: UART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_TxHalfCpltCallback(mut huart:
                                                         *mut UART_HandleTypeDef) {
    /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_TxHalfCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_RxCpltCallback(mut huart:
                                                     *mut UART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief  Rx Half Transfer completed callbacks.
  * @param  huart: UART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_RxHalfCpltCallback(mut huart:
                                                         *mut UART_HandleTypeDef) {
    /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxHalfCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief UART error callbacks
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_ErrorCallback(mut huart:
                                                    *mut UART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file
   */
}
/* *
  * @brief Send an amount of data in interrupt mode
  *         Function called under interruption only, once
  *         interruptions have been enabled by HAL_UART_Transmit_IT()
  * @param  huart: UART handle
  * @retval HAL status
  */
unsafe extern "C" fn UART_Transmit_IT(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    /* Check that a Tx process is ongoing */
    if (*huart).gState as libc::c_uint ==
           HAL_UART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        if (*huart).TxXferCount as libc::c_uint == 0 as libc::c_uint {
            /* Disable the UART Transmit Data Register Empty Interrupt */
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   7 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Enable the UART Transmit Complete Interrupt */
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_OK
        } else {
            if (*huart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*huart).Init.Parity == 0 as libc::c_uint {
                tmp = (*huart).pTxBuffPtr as *mut uint16_t;
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).TDR as
                                                *mut uint32_t,
                                            (*tmp as libc::c_int &
                                                 0x1ff as libc::c_uint as
                                                     uint16_t as libc::c_int)
                                                as uint32_t);
                (*huart).pTxBuffPtr =
                    (*huart).pTxBuffPtr.offset(2 as libc::c_uint as isize)
            } else {
                let fresh2 = (*huart).pTxBuffPtr;
                (*huart).pTxBuffPtr = (*huart).pTxBuffPtr.offset(1);
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).TDR as
                                                *mut uint32_t,
                                            (*fresh2 as libc::c_int &
                                                 0xff as libc::c_uint as
                                                     uint8_t as libc::c_int)
                                                as uint8_t as uint32_t)
            }
            ::core::ptr::write_volatile(&mut (*huart).TxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*huart).TxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            return HAL_OK
        }
    } else { return HAL_BUSY };
}
/* *
  * @brief  Wrap up transmission in non-blocking mode.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
unsafe extern "C" fn UART_EndTransmit_IT(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Disable the UART Transmit Complete Interrupt */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Tx process is ended, restore huart->gState to Ready */
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    HAL_UART_TxCpltCallback(huart);
    return HAL_OK;
}
/* *
  * @brief Receive an amount of data in interrupt mode
  *         Function called under interruption only, once
  *         interruptions have been enabled by HAL_UART_Receive_IT()
  * @param  huart: UART handle
  * @retval HAL status
  */
unsafe extern "C" fn UART_Receive_IT(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut uhMask: uint16_t = (*huart).Mask;
    /* Check that a Rx process is ongoing */
    if (*huart).RxState as libc::c_uint ==
           HAL_UART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        if (*huart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint &&
               (*huart).Init.Parity == 0 as libc::c_uint {
            tmp = (*huart).pRxBuffPtr as *mut uint16_t;
            *tmp =
                ((*(*huart).Instance).RDR & uhMask as libc::c_uint) as
                    uint16_t;
            (*huart).pRxBuffPtr =
                (*huart).pRxBuffPtr.offset(2 as libc::c_int as isize)
        } else {
            let fresh3 = (*huart).pRxBuffPtr;
            (*huart).pRxBuffPtr = (*huart).pRxBuffPtr.offset(1);
            *fresh3 =
                ((*(*huart).Instance).RDR & uhMask as uint8_t as libc::c_uint)
                    as uint8_t
        }
        ::core::ptr::write_volatile(&mut (*huart).RxXferCount as
                                        *mut uint16_t,
                                    ::core::ptr::read_volatile::<uint16_t>(&(*huart).RxXferCount
                                                                               as
                                                                               *const uint16_t).wrapping_sub(1));
        if ::core::ptr::read_volatile::<uint16_t>(&(*huart).RxXferCount as
                                                      *const uint16_t) as
               libc::c_int == 0 as libc::c_int {
            /* Disable the UART Parity Error Interrupt and RXNE interrupt*/
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       8 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Rx process is completed, restore huart->RxState to Ready */
            ::core::ptr::write_volatile(&mut (*huart).RxState as
                                            *mut HAL_UART_StateTypeDef,
                                        HAL_UART_STATE_READY);
            HAL_UART_RxCpltCallback(huart);
            return HAL_OK
        }
        return HAL_OK
    } else {
        /* Clear RXNE interrupt flag */
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).RQR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).RQR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_BUSY
    };
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_uart.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   UART HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the Universal Asynchronous Receiver Transmitter (UART) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State and Errors functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
    The UART HAL driver can be used as follows:

    (#) Declare a UART_HandleTypeDef handle structure.

    (#) Initialize the UART low level resources by implementing the HAL_UART_MspInit() API:
        (##) Enable the USARTx interface clock.
        (##) UART pins configuration:
            (+++) Enable the clock for the UART GPIOs.
            (+++) Configure these UART pins as alternate function pull-up.
        (##) NVIC configuration if you need to use interrupt process (HAL_UART_Transmit_IT()
             and HAL_UART_Receive_IT() APIs):
            (+++) Configure the USARTx interrupt priority.
            (+++) Enable the NVIC USART IRQ handle.
        (##) DMA Configuration if you need to use DMA process (HAL_UART_Transmit_DMA()
             and HAL_UART_Receive_DMA() APIs):
            (+++) Declare a DMA handle structure for the Tx/Rx stream.
            (+++) Enable the DMAx interface clock.
            (+++) Configure the declared DMA handle structure with the required
                  Tx/Rx parameters.
            (+++) Configure the DMA Tx/Rx Stream.
            (+++) Associate the initialized DMA handle to the UART DMA Tx/Rx handle.
            (+++) Configure the priority and enable the NVIC for the transfer complete
                  interrupt on the DMA Tx/Rx Stream.

    (#) Program the Baud Rate, Word Length, Stop Bit, Parity, Hardware
        flow control and Mode(Receiver/Transmitter) in the Init structure.

    (#) For the UART asynchronous mode, initialize the UART registers by calling
        the HAL_UART_Init() API.

    (#) For the UART Half duplex mode, initialize the UART registers by calling
        the HAL_HalfDuplex_Init() API.

    (#) For the LIN mode, initialize the UART registers by calling the HAL_LIN_Init() API.

    (#) For the Multi-Processor mode, initialize the UART registers by calling
        the HAL_MultiProcessor_Init() API.

     [..]
       (@) The specific UART interrupts (Transmission complete interrupt,
            RXNE interrupt and Error Interrupts) will be managed using the macros
            __HAL_UART_ENABLE_IT() and __HAL_UART_DISABLE_IT() inside the transmit
            and receive process.

     [..]
       (@) These APIs (HAL_UART_Init() and HAL_HalfDuplex_Init()) configure also the
            low level Hardware GPIO, CLOCK, CORTEX...etc) by calling the customized
            HAL_UART_MspInit() API.

     [..]
        Three operation modes are available within this driver :

     *** Polling mode IO operation ***
     =================================
     [..]
       (+) Send an amount of data in blocking mode using HAL_UART_Transmit()
       (+) Receive an amount of data in blocking mode using HAL_UART_Receive()

     *** Interrupt mode IO operation ***
     ===================================
     [..]
       (+) Send an amount of data in non blocking mode using HAL_UART_Transmit_IT()
       (+) At transmission end of transfer HAL_UART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode using HAL_UART_Receive_IT()
       (+) At reception end of transfer HAL_UART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_RxCpltCallback
       (+) In case of transfer Error, HAL_UART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_UART_ErrorCallback

     *** DMA mode IO operation ***
     ==============================
     [..]
       (+) Send an amount of data in non blocking mode (DMA) using HAL_UART_Transmit_DMA()
       (+) At transmission end of half transfer HAL_UART_TxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_TxHalfCpltCallback
       (+) At transmission end of transfer HAL_UART_TxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_TxCpltCallback
       (+) Receive an amount of data in non blocking mode (DMA) using HAL_UART_Receive_DMA()
       (+) At reception end of half transfer HAL_UART_RxHalfCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_RxHalfCpltCallback
       (+) At reception end of transfer HAL_UART_RxCpltCallback is executed and user can
            add his own code by customization of function pointer HAL_UART_RxCpltCallback
       (+) In case of transfer Error, HAL_UART_ErrorCallback() function is executed and user can
            add his own code by customization of function pointer HAL_UART_ErrorCallback
       (+) Pause the DMA Transfer using HAL_UART_DMAPause()
       (+) Resume the DMA Transfer using HAL_UART_DMAResume()
       (+) Stop the DMA Transfer using HAL_UART_DMAStop()

     *** UART HAL driver macros list ***
     =============================================
     [..]
       Below the list of most used macros in UART HAL driver.

      (+) __HAL_UART_ENABLE: Enable the UART peripheral
      (+) __HAL_UART_DISABLE: Disable the UART peripheral
      (+) __HAL_UART_GET_FLAG : Check whether the specified UART flag is set or not
      (+) __HAL_UART_CLEAR_IT : Clears the specified UART ISR flag
      (+) __HAL_UART_ENABLE_IT: Enable the specified UART interrupt
      (+) __HAL_UART_DISABLE_IT: Disable the specified UART interrupt
      (+) __HAL_UART_GET_IT_SOURCE: Check whether the specified UART interrupt has occurred or not

     [..]
       (@) You can refer to the UART HAL driver header file for more useful macros

  @endverbatim
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
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_HAL_Driver
  * @{
  */
/* * @defgroup UART UART
  * @brief HAL UART module driver
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* * @defgroup UART_Private_Constants UART Private Constants
  * @{
  */
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* * @addtogroup UART_Private_Functions
  * @{
  */
/* *
  * @brief  End ongoing Tx transfer on UART peripheral (following error detection or Transmit completion).
  * @param  huart: UART handle.
  * @retval None
  */
unsafe extern "C" fn UART_EndTxTransfer(mut huart: *mut UART_HandleTypeDef) {
    /* Disable TXEIE and TCIE interrupts */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           7 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* At end of Tx process, restore huart->gState to Ready */
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
}
/* *
  * @brief  End ongoing Rx transfer on UART peripheral (following error detection or Reception completion).
  * @param  huart: UART handle.
  * @retval None
  */
unsafe extern "C" fn UART_EndRxTransfer(mut huart: *mut UART_HandleTypeDef) {
    /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               8 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* At end of Rx process, restore huart->RxState to Ready */
    ::core::ptr::write_volatile(&mut (*huart).RxState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
}
/* *
  * @}
  */
/* * @defgroup UART_Exported_Functions_Group3 Peripheral Control functions
  *  @brief   UART control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the UART.
     (+) HAL_UART_GetState() API is helpful to check in run-time the state of the UART peripheral.
     (+) HAL_MultiProcessor_EnableMuteMode() API enables mute mode
     (+) HAL_MultiProcessor_DisableMuteMode() API disables mute mode
     (+) HAL_MultiProcessor_EnterMuteMode() API enters mute mode
     (+) HAL_MultiProcessor_EnableMuteMode() API enables mute mode
     (+) UART_SetConfig() API configures the UART peripheral
     (+) UART_AdvFeatureConfig() API optionally configures the UART advanced features
     (+) UART_CheckIdleState() API ensures that TEACK and/or REACK are set after initialization
     (+) HAL_HalfDuplex_EnableTransmitter() API disables receiver and enables transmitter
     (+) HAL_HalfDuplex_EnableReceiver() API disables transmitter and enables receiver
     (+) HAL_LIN_SendBreak() API transmits the break characters
	 (+) HAL_MultiProcessorEx_AddressLength_Set() API optionally sets the UART node address
         detection length to more than 4 bits for multiprocessor address mark wake up.
@endverbatim
  * @{
  */
/* *
  * @brief Enable UART in mute mode (doesn't mean UART enters mute mode;
  * to enter mute mode, HAL_MultiProcessor_EnterMuteMode() API must be called)
  * @param huart: UART handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MultiProcessor_EnableMuteMode(mut huart:
                                                               *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Enable USART mute mode by setting the MME bit in the CR1 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         13 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    return UART_CheckIdleState(huart);
}
/* *
  * @brief Disable UART mute mode (doesn't mean it actually wakes up the software,
  * as it may not have been in mute mode at this very moment).
  * @param huart: uart handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MultiProcessor_DisableMuteMode(mut huart:
                                                                *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable USART mute mode by clearing the MME bit in the CR1 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           13 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    return UART_CheckIdleState(huart);
}
/* *
  * @brief Enter UART mute mode (means UART actually enters mute mode).
  * To exit from mute mode, HAL_MultiProcessor_DisableMuteMode() API must be called.
  * @param huart: uart handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MultiProcessor_EnterMuteMode(mut huart:
                                                              *mut UART_HandleTypeDef) {
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).RQR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).RQR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief return the UART state
  * @param huart: uart handle
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_GetState(mut huart: *mut UART_HandleTypeDef)
 -> HAL_UART_StateTypeDef {
    let mut temp1: uint32_t = 0 as libc::c_uint;
    let mut temp2: uint32_t = 0 as libc::c_uint;
    temp1 = (*huart).gState as uint32_t;
    temp2 = (*huart).RxState as uint32_t;
    return (temp1 | temp2) as HAL_UART_StateTypeDef;
}
/* *
* @brief  Return the UART error code
* @param  huart : pointer to a UART_HandleTypeDef structure that contains
  *              the configuration information for the specified UART.
* @retval UART Error Code
*/
#[no_mangle]
pub unsafe extern "C" fn HAL_UART_GetError(mut huart: *mut UART_HandleTypeDef)
 -> uint32_t {
    return (*huart).ErrorCode;
}
/* *
  * @brief Configure the UART peripheral
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn UART_SetConfig(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpreg: uint32_t = 0 as libc::c_uint;
    let mut clocksource: UART_ClockSourceTypeDef = UART_CLOCKSOURCE_UNDEFINED;
    let mut brrtemp: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut usartdiv: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut ret: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
   *  the UART Word Length, Parity, Mode and oversampling:
   *  set the M bits according to huart->Init.WordLength value
   *  set PCE and PS bits according to huart->Init.Parity value
   *  set TE and RE bits according to huart->Init.Mode value
   *  set OVER8 bit according to huart->Init.OverSampling value */
    tmpreg =
        (*huart).Init.WordLength | (*huart).Init.Parity | (*huart).Init.Mode |
            (*huart).Init.OverSampling;
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR1 &
                                    !((0x10001 as libc::c_uint) <<
                                          12 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              10 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              9 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              3 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              2 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              15 as libc::c_uint) | tmpreg);
    /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
   * to huart->Init.StopBits value */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR2 &
                                    !((0x3 as libc::c_uint) <<
                                          12 as libc::c_uint) |
                                    (*huart).Init.StopBits);
    /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
   * - UART HardWare Flow Control: set CTSE and RTSE bits according
   *   to huart->Init.HwFlowCtl value
   * - one-bit sampling method versus three samples' majority rule according
   *   to huart->Init.OneBitSampling */
    tmpreg = (*huart).Init.HwFlowCtl | (*huart).Init.OneBitSampling;
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR3 &
                                    !((0x1 as libc::c_uint) <<
                                          8 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              9 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              11 as libc::c_uint) | tmpreg);
    /*-------------------------- USART BRR Configuration -----------------------*/
    if (*huart).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_uint)
               as *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 0 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK2 }
            2 => { clocksource = UART_CLOCKSOURCE_HSI }
            1 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            3 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x4400 as libc::c_uint) as
                      *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 2 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK1 }
            8 => { clocksource = UART_CLOCKSOURCE_HSI }
            4 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            12 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x4800 as libc::c_uint) as
                      *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 4 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK1 }
            32 => { clocksource = UART_CLOCKSOURCE_HSI }
            16 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            48 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x4c00 as libc::c_uint) as
                      *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 6 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK1 }
            128 => { clocksource = UART_CLOCKSOURCE_HSI }
            64 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            192 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x5000 as libc::c_uint) as
                      *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 8 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK1 }
            512 => { clocksource = UART_CLOCKSOURCE_HSI }
            256 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            768 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x1400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 10 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK2 }
            2048 => { clocksource = UART_CLOCKSOURCE_HSI }
            1024 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            3072 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x7800 as libc::c_uint) as
                      *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 12 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK1 }
            8192 => { clocksource = UART_CLOCKSOURCE_HSI }
            4096 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            12288 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*huart).Instance ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x7c00 as libc::c_uint) as
                      *mut USART_TypeDef {
        match (*((0x40000000 as
                      libc::c_uint).wrapping_add(0x20000 as
                                                     libc::c_uint).wrapping_add(0x3800
                                                                                    as
                                                                                    libc::c_uint)
                     as *mut RCC_TypeDef)).DCKCFGR2 &
                  (0x3 as libc::c_uint) << 14 as libc::c_uint {
            0 => { clocksource = UART_CLOCKSOURCE_PCLK1 }
            32768 => { clocksource = UART_CLOCKSOURCE_HSI }
            16384 => { clocksource = UART_CLOCKSOURCE_SYSCLK }
            49152 => { clocksource = UART_CLOCKSOURCE_LSE }
            _ => { }
        }
    }
    /* Check UART Over Sampling to set Baud Rate Register */
    if (*huart).Init.OverSampling ==
           (0x1 as libc::c_uint) << 15 as libc::c_uint {
        match clocksource as libc::c_uint {
            0 => {
                usartdiv =
                    HAL_RCC_GetPCLK1Freq().wrapping_mul(2 as libc::c_int as
                                                            libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                               as
                                                                                                                               libc::c_int
                                                                                                                               as
                                                                                                                               libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                        as uint16_t
            }
            1 => {
                usartdiv =
                    HAL_RCC_GetPCLK2Freq().wrapping_mul(2 as libc::c_int as
                                                            libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                               as
                                                                                                                               libc::c_int
                                                                                                                               as
                                                                                                                               libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                        as uint16_t
            }
            2 => {
                usartdiv =
                    (16000000 as
                         libc::c_uint).wrapping_mul(2 as libc::c_int as
                                                        libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                        as uint16_t
            }
            4 => {
                usartdiv =
                    HAL_RCC_GetSysClockFreq().wrapping_mul(2 as libc::c_int as
                                                               libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                                  as
                                                                                                                                  libc::c_int
                                                                                                                                  as
                                                                                                                                  libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                        as uint16_t
            }
            8 => {
                usartdiv =
                    (32768 as
                         libc::c_uint).wrapping_mul(2 as libc::c_int as
                                                        libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                        as uint16_t
            }
            16 | _ => { ret = HAL_ERROR }
        }
        brrtemp =
            (usartdiv as libc::c_uint & 0xfff0 as libc::c_uint) as uint16_t;
        brrtemp =
            (brrtemp as libc::c_int |
                 ((usartdiv as libc::c_int &
                       0xf as libc::c_uint as uint16_t as libc::c_int) >>
                      1 as libc::c_uint) as uint16_t as libc::c_int) as
                uint16_t;
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).BRR as
                                        *mut uint32_t, brrtemp as uint32_t)
    } else {
        match clocksource as libc::c_uint {
            0 => {
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).BRR as
                                                *mut uint32_t,
                                            HAL_RCC_GetPCLK1Freq().wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                        as
                                                                                                                        libc::c_int
                                                                                                                        as
                                                                                                                        libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                                                as uint16_t as uint32_t)
            }
            1 => {
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).BRR as
                                                *mut uint32_t,
                                            HAL_RCC_GetPCLK2Freq().wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                        as
                                                                                                                        libc::c_int
                                                                                                                        as
                                                                                                                        libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                                                as uint16_t as uint32_t)
            }
            2 => {
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).BRR as
                                                *mut uint32_t,
                                            (16000000 as
                                                 libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                    as
                                                                                                                    libc::c_int
                                                                                                                    as
                                                                                                                    libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                                                as uint16_t as uint32_t)
            }
            4 => {
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).BRR as
                                                *mut uint32_t,
                                            HAL_RCC_GetSysClockFreq().wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                                                as uint16_t as uint32_t)
            }
            8 => {
                ::core::ptr::write_volatile(&mut (*(*huart).Instance).BRR as
                                                *mut uint32_t,
                                            (32768 as
                                                 libc::c_uint).wrapping_add((*huart).Init.BaudRate.wrapping_div(2
                                                                                                                    as
                                                                                                                    libc::c_int
                                                                                                                    as
                                                                                                                    libc::c_uint)).wrapping_div((*huart).Init.BaudRate)
                                                as uint16_t as uint32_t)
            }
            16 | _ => { ret = HAL_ERROR }
        }
    }
    return ret;
}
/* *
  * @brief Configure the UART peripheral advanced features
  * @param huart: uart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn UART_AdvFeatureConfig(mut huart:
                                                   *mut UART_HandleTypeDef) {
    /* Check whether the set of advanced features to configure is properly set */
    /* if required, configure TX pin active level inversion */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x1 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              17 as libc::c_uint) |
                                        (*huart).AdvancedInit.TxPinLevelInvert)
    }
    /* if required, configure RX pin active level inversion */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x2 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              16 as libc::c_uint) |
                                        (*huart).AdvancedInit.RxPinLevelInvert)
    }
    /* if required, configure data inversion */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              18 as libc::c_uint) |
                                        (*huart).AdvancedInit.DataInvert)
    }
    /* if required, configure RX/TX pins swap */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x8 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              15 as libc::c_uint) |
                                        (*huart).AdvancedInit.Swap)
    }
    /* if required, configure RX overrun detection disabling */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x10 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR3 &
                                        !((0x1 as libc::c_uint) <<
                                              12 as libc::c_uint) |
                                        (*huart).AdvancedInit.OverrunDisable)
    }
    /* if required, configure DMA disabling on reception error */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x20 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR3 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR3 &
                                        !((0x1 as libc::c_uint) <<
                                              13 as libc::c_uint) |
                                        (*huart).AdvancedInit.DMADisableonRxError)
    }
    /* if required, configure auto Baud rate detection scheme */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x40 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              20 as libc::c_uint) |
                                        (*huart).AdvancedInit.AutoBaudRateEnable);
        /* set auto Baudrate detection parameters if detection is enabled */
        if (*huart).AdvancedInit.AutoBaudRateEnable ==
               (0x1 as libc::c_uint) << 20 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                            *mut uint32_t,
                                        (*(*huart).Instance).CR2 &
                                            !((0x3 as libc::c_uint) <<
                                                  21 as libc::c_uint) |
                                            (*huart).AdvancedInit.AutoBaudRateMode)
        }
    }
    /* if required, configure MSB first on communication line */
    if (*huart).AdvancedInit.AdvFeatureInit & 0x80 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                        *mut uint32_t,
                                    (*(*huart).Instance).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              19 as libc::c_uint) |
                                        (*huart).AdvancedInit.MSBFirst)
    };
}
/* *
  * @brief Check the UART Idle State
  * @param huart: uart handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn UART_CheckIdleState(mut huart:
                                                 *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Initialize the UART ErrorCode */
    ::core::ptr::write_volatile(&mut (*huart).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Check if the Transmitter is enabled */
    if (*(*huart).Instance).CR1 & (0x1 as libc::c_uint) << 3 as libc::c_uint
           == (0x1 as libc::c_uint) << 3 as libc::c_uint {
        /* Wait until TEACK flag is set */
        if UART_WaitOnFlagUntilTimeout(huart,
                                       (0x1 as libc::c_uint) <<
                                           21 as libc::c_uint, RESET,
                                       tickstart, 0x1ffffff as libc::c_uint)
               as libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            /* Timeout Occurred */
            return HAL_TIMEOUT
        }
    }
    /* Initialize the UART State */
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    ::core::ptr::write_volatile(&mut (*huart).RxState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    /* Process Unlocked */
    (*huart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Enables the UART transmitter and disables the UART receiver.
  * @param  huart: UART handle
  * @retval HAL status
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_HalfDuplex_EnableTransmitter(mut huart:
                                                              *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Clear TE and RE bits */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Enable the USART's transmit interface by setting the TE bit in the USART CR1 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         3 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    /* Process Unlocked */
    (*huart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Enables the UART receiver and disables the UART transmitter.
  * @param  huart: UART handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_HalfDuplex_EnableReceiver(mut huart:
                                                           *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Clear TE and RE bits */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           3 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               2 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Enable the USART's receive interface by setting the RE bit in the USART CR1 register */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    /* Process Unlocked */
    (*huart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Transmits break characters.
  * @param  huart: UART handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_LIN_SendBreak(mut huart: *mut UART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process Locked */
    if (*huart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*huart).Lock = HAL_LOCKED }
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Send break characters */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).RQR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).RQR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_READY);
    /* Process Unlocked */
    (*huart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_uart_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of UART HAL Extension module.
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
/* * @addtogroup UARTEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup UARTEx_Exported_Constants UARTEx Exported Constants
  * @{
  */
/* * @defgroup UARTEx_Word_Length UARTEx Word Length
  * @{
  */
/* *
  * @}
  */
/* * @defgroup UARTEx_WakeUp_Address_Length UARTEx WakeUp Address Length
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup UARTEx_Exported_Macros UARTEx Exported Macros
  * @{
  */
/* * @brief  Reports the UART clock source.
  * @param  __HANDLE__: specifies the UART Handle
  * @param  __CLOCKSOURCE__: output variable   
  * @retval UART clocking source, written in __CLOCKSOURCE__.
  */
/* * @brief  Reports the UART mask to apply to retrieve the received data
  *         according to the word length and to the parity bits activation.
  *         If PCE = 1, the parity bit is not included in the data extracted
  *         by the reception API().
  *         This masking operation is not carried out in the case of
  *         DMA transfers.        
  * @param  __HANDLE__: specifies the UART Handle
  * @retval mask to apply to UART RDR register value.
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup UARTEx_Exported_Functions
  * @{
  */
/* * @addtogroup UARTEx_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions  ****************************/
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup UARTEx_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions  **********************************************/
/* *
  * @brief By default in multiprocessor mode, when the wake up method is set
  *        to address mark, the UART handles only 4-bit long addresses detection;
  *        this API allows to enable longer addresses detection (6-, 7- or 8-bit
  *        long).
  * @note  Addresses detection lengths are: 6-bit address detection in 7-bit data mode,
  *        7-bit address detection in 8-bit data mode, 8-bit address detection in 9-bit data mode.
  * @param huart: UART handle.
  * @param AddressLength: this parameter can be one of the following values:
  *          @arg @ref UART_ADDRESS_DETECT_4B 4-bit long address
  *          @arg @ref UART_ADDRESS_DETECT_7B 6-, 7- or 8-bit long address
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_MultiProcessorEx_AddressLength_Set(mut huart:
                                                                    *mut UART_HandleTypeDef,
                                                                mut AddressLength:
                                                                    uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the UART handle allocation */
    if huart.is_null() { return HAL_ERROR }
    /* Check the address length parameter */
    ::core::ptr::write_volatile(&mut (*huart).gState as
                                    *mut HAL_UART_StateTypeDef,
                                HAL_UART_STATE_BUSY);
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the address length */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR2 as
                                    *mut uint32_t,
                                (*(*huart).Instance).CR2 &
                                    !((0x1 as libc::c_uint) <<
                                          4 as libc::c_uint) | AddressLength);
    /* Enable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*huart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*huart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving huart->gState to Ready */
    return UART_CheckIdleState(huart);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_UART_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
