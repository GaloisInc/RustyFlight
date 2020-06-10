use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_RCC_GetSysClockFreq() -> uint32_t;
    #[no_mangle]
    fn HAL_RCC_GetPCLK1Freq() -> uint32_t;
    #[no_mangle]
    fn HAL_RCC_GetPCLK2Freq() -> uint32_t;
    #[no_mangle]
    fn HAL_DMA_Abort(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort_IT(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
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
/* !< DMA Stream Index                       */
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_usart.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of USART HAL module.
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
/* * @addtogroup USART
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup USART_Exported_Types USART Exported Types
  * @{
  */
/* *
  * @brief USART Init Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_InitTypeDef {
    pub BaudRate: uint32_t,
    pub WordLength: uint32_t,
    pub StopBits: uint32_t,
    pub Parity: uint32_t,
    pub Mode: uint32_t,
    pub OverSampling: uint32_t,
    pub CLKPolarity: uint32_t,
    pub CLKPhase: uint32_t,
    pub CLKLastBit: uint32_t,
}
/* *
  * @brief HAL USART State structures definition
  */
pub type HAL_USART_StateTypeDef = libc::c_uint;
/* !< Error */
/* !< Timeout state */
pub const HAL_USART_STATE_ERROR: HAL_USART_StateTypeDef = 4;
/* !< Data Transmission Reception process is ongoing */
pub const HAL_USART_STATE_TIMEOUT: HAL_USART_StateTypeDef = 3;
/* !< Data Reception process is ongoing */
pub const HAL_USART_STATE_BUSY_TX_RX: HAL_USART_StateTypeDef = 50;
/* !< Data Transmission process is ongoing */
pub const HAL_USART_STATE_BUSY_RX: HAL_USART_StateTypeDef = 34;
/* !< an internal process is ongoing */
pub const HAL_USART_STATE_BUSY_TX: HAL_USART_StateTypeDef = 18;
/* !< Peripheral Initialized and ready for use */
pub const HAL_USART_STATE_BUSY: HAL_USART_StateTypeDef = 2;
/* !< Peripheral is not initialized   */
pub const HAL_USART_STATE_READY: HAL_USART_StateTypeDef = 1;
pub const HAL_USART_STATE_RESET: HAL_USART_StateTypeDef = 0;
/* *
  * @brief  USART clock sources definitions
  */
pub type USART_ClockSourceTypeDef = libc::c_uint;
/* !< Undefined clock source */
/* !< LSE clock source       */
pub const USART_CLOCKSOURCE_UNDEFINED: USART_ClockSourceTypeDef = 16;
/* !< SYSCLK clock source */
pub const USART_CLOCKSOURCE_LSE: USART_ClockSourceTypeDef = 8;
/* !< HSI clock source    */
pub const USART_CLOCKSOURCE_SYSCLK: USART_ClockSourceTypeDef = 4;
/* !< PCLK2 clock source  */
pub const USART_CLOCKSOURCE_HSI: USART_ClockSourceTypeDef = 2;
/* !< PCLK1 clock source  */
pub const USART_CLOCKSOURCE_PCLK2: USART_ClockSourceTypeDef = 1;
pub const USART_CLOCKSOURCE_PCLK1: USART_ClockSourceTypeDef = 0;
/* *
  * @brief  USART handle Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_HandleTypeDef {
    pub Instance: *mut USART_TypeDef,
    pub Init: USART_InitTypeDef,
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
    pub State: HAL_USART_StateTypeDef,
    pub ErrorCode: uint32_t,
}
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup USART_Exported_Functions USART Exported Functions
  * @{
  */
/* * @defgroup USART_Exported_Functions_Group1 USART Initialization and de-initialization functions
  *  @brief    Initialization and Configuration functions
  *
@verbatim
 ===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to initialize the USART
    in asynchronous and in synchronous modes.
      (+) For the asynchronous mode only these parameters can be configured:
        (++) Baud Rate
        (++) Word Length
        (++) Stop Bit
        (++) Parity: If the parity is enabled, then the MSB bit of the data written
             in the data register is transmitted but is changed by the parity bit.
        (++) USART polarity
        (++) USART phase
        (++) USART LastBit
        (++) Receiver/transmitter modes

    [..]
    The HAL_USART_Init() function follows the USART  synchronous configuration
    procedure (details for the procedure are available in reference manual).

@endverbatim

   Depending on the frame length defined by the M1 and M0 bits (7-bit,
   8-bit or 9-bit), the possible USART frame formats are as listed in the
   following table:

     +---------------------------------------------------------------+
     | M1M0 bits |  PCE bit  |            USART frame                |
     |-----------------------|---------------------------------------|
     |     10    |     0     |    | SB | 7-bit data | STB |          |
     |-----------|-----------|---------------------------------------|
     |     10    |     1     |    | SB | 6-bit data | PB | STB |     |
     +---------------------------------------------------------------+

  * @{
  */
/* *
  * @brief  Initializes the USART mode according to the specified
  *         parameters in the USART_InitTypeDef and create the associated handle.
  * @param husart: USART handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Init(mut husart: *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the USART handle allocation */
    if husart.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*husart).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK */
        HAL_USART_MspInit(husart);
    }
    (*husart).State = HAL_USART_STATE_BUSY;
    /* Disable the Peripheral */
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the Usart Communication parameters */
    if USART_SetConfig(husart) as libc::c_uint ==
           HAL_ERROR as libc::c_int as libc::c_uint {
        return HAL_ERROR
    }
    /* In Synchronous mode, the following bits must be kept cleared:
  - LINEN bit in the USART_CR2 register
  - HDSEL, SCEN and IREN bits in the USART_CR3 register.*/
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           14 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
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
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* TEACK and/or REACK to check before moving husart->State to Ready */
    return USART_CheckIdleState(husart);
}
/* *
  * @brief DeInitializes the USART peripheral
  * @param husart: USART handle
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_DeInit(mut husart:
                                              *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the USART handle allocation */
    if husart.is_null() { return HAL_ERROR }
    /* Check the parameters */
    (*husart).State = HAL_USART_STATE_BUSY;
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR2 as
                                    *mut uint32_t, 0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                    *mut uint32_t, 0 as libc::c_uint);
    /* DeInit the low level hardware */
    HAL_USART_MspDeInit(husart);
    ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    (*husart).State = HAL_USART_STATE_RESET;
    /* Process Unlock */
    (*husart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief USART MSP Init
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_MspInit(mut husart:
                                               *mut USART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_MspInit can be implemented in the user file
   */
}
/* *
  * @brief USART MSP DeInit
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_MspDeInit(mut husart:
                                                 *mut USART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_MspDeInit can be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup USART_Exported_Functions_Group2 IO operation functions
  *  @brief   USART Transmit and Receive functions
  *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    This subsection provides a set of functions allowing to manage the USART synchronous
    data transfers.

    [..] The USART supports master mode only: it cannot receive or send data related to an input
         clock (SCLK is always an output).

    (#) There are two mode of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
           or DMA, These API's return the HAL status.
           The end of the data processing will be indicated through the
           dedicated USART IRQ when using Interrupt mode or the DMA IRQ when
           using DMA mode.
           The HAL_USART_TxCpltCallback(), HAL_USART_RxCpltCallback() and HAL_USART_TxRxCpltCallback() user callbacks
           will be executed respectively at the end of the transmit or Receive process
           The HAL_USART_ErrorCallback()user callback will be executed when a communication error is detected

    (#) Blocking mode API's are :
        (++) HAL_USART_Transmit()in simplex mode
        (++) HAL_USART_Receive() in full duplex receive only
        (++) HAL_USART_TransmitReceive() in full duplex mode

    (#) Non-Blocking mode API's with Interrupt are :
        (++) HAL_USART_Transmit_IT()in simplex mode
        (++) HAL_USART_Receive_IT() in full duplex receive only
        (++) HAL_USART_TransmitReceive_IT()in full duplex mode
        (++) HAL_USART_IRQHandler()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_USART_Transmit_DMA()in simplex mode
        (++) HAL_USART_Receive_DMA() in full duplex receive only
        (++) HAL_USART_TransmitReceive_DMA() in full duplex mode
        (++) HAL_USART_DMAPause()
        (++) HAL_USART_DMAResume()
        (++) HAL_USART_DMAStop()

    (#) A set of Transfer Complete Callbacks are provided in No_Blocking mode:
        (++) HAL_USART_TxCpltCallback()
        (++) HAL_USART_RxCpltCallback()
        (++) HAL_USART_TxHalfCpltCallback()
        (++) HAL_USART_RxHalfCpltCallback()
        (++) HAL_USART_ErrorCallback()
        (++) HAL_USART_TxRxCpltCallback()

@endverbatim
  * @{
  */
/* *
  * @brief  Simplex Send an amount of data in blocking mode
  * @param  husart: USART handle
  * @param pTxData: pointer to data buffer
  * @param Size: amount of data to be sent
  * @param Timeout : Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Transmit(mut husart:
                                                *mut USART_HandleTypeDef,
                                            mut pTxData: *mut uint8_t,
                                            mut Size: uint16_t,
                                            mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pTxData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_TX;
        /* Init tickstart for timeout managment*/
        tickstart = HAL_GetTick();
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                        *mut uint16_t, Size);
        /* Check the remaining data to be sent */
        while (*husart).TxXferCount as libc::c_uint > 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*husart).TxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            if USART_WaitOnFlagUntilTimeout(husart, 0x40 as libc::c_uint,
                                            RESET, tickstart, Timeout) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            if (*husart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*husart).Init.Parity == 0 as libc::c_uint {
                tmp = pTxData as *mut uint16_t;
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                                *mut uint32_t,
                                            (*tmp as libc::c_int &
                                                 0x1ff as libc::c_uint as
                                                     uint16_t as libc::c_int)
                                                as uint32_t);
                pTxData = pTxData.offset(2 as libc::c_int as isize)
            } else {
                let fresh0 = pTxData;
                pTxData = pTxData.offset(1);
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                                *mut uint32_t,
                                            (*fresh0 as libc::c_int &
                                                 0xff as libc::c_uint as
                                                     uint8_t as libc::c_int)
                                                as uint32_t)
            }
        }
        if USART_WaitOnFlagUntilTimeout(husart, 0x40 as libc::c_uint, RESET,
                                        tickstart, Timeout) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        (*husart).State = HAL_USART_STATE_READY;
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Receive an amount of data in blocking mode
  * @note To receive synchronous data, dummy data are simultaneously transmitted
  * @param husart: USART handle
  * @param pRxData: pointer to data buffer
  * @param Size: amount of data to be received
  * @param Timeout : Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Receive(mut husart:
                                               *mut USART_HandleTypeDef,
                                           mut pRxData: *mut uint8_t,
                                           mut Size: uint16_t,
                                           mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut uhMask: uint16_t = 0;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pRxData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_RX;
        /* Init tickstart for timeout managment*/
        tickstart = HAL_GetTick();
        (*husart).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                        *mut uint16_t, Size);
        /* Computation of USART mask to apply to RDR register */
        if (*husart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x1ff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0xff as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0xff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x7f as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0x10000000 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x7f as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x3f as libc::c_int as uint16_t }
        }
        uhMask = (*husart).Mask;
        /* as long as data have to be received */
        while (*husart).RxXferCount as libc::c_uint > 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*husart).RxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            /* Wait until TC flag is set to send dummy byte in order to generate the
      * clock for the slave to send data.
       * Whatever the frame length (7, 8 or 9-bit long), the same dummy value
       * can be written for all the cases. */
            if USART_WaitOnFlagUntilTimeout(husart, 0x40 as libc::c_uint,
                                            RESET, tickstart, Timeout) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                            *mut uint32_t,
                                        (0xffff as libc::c_uint as uint16_t as
                                             libc::c_int &
                                             0xff as libc::c_uint as uint16_t
                                                 as libc::c_int) as uint32_t);
            /* Wait for RXNE Flag */
            if USART_WaitOnFlagUntilTimeout(husart, 0x20 as libc::c_uint,
                                            RESET, tickstart, Timeout) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            if (*husart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*husart).Init.Parity == 0 as libc::c_uint {
                tmp = pRxData as *mut uint16_t;
                *tmp =
                    ((*(*husart).Instance).RDR & uhMask as libc::c_uint) as
                        uint16_t;
                pRxData = pRxData.offset(2 as libc::c_int as isize)
            } else {
                let fresh1 = pRxData;
                pRxData = pRxData.offset(1);
                *fresh1 =
                    ((*(*husart).Instance).RDR &
                         uhMask as uint8_t as libc::c_uint) as uint8_t
            }
        }
        (*husart).State = HAL_USART_STATE_READY;
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Full-Duplex Send and Receive an amount of data in blocking mode
  * @param husart: USART handle
  * @param pTxData: pointer to TX data buffer
  * @param pRxData: pointer to RX data buffer
  * @param Size: amount of data to be sent (same amount to be received)
  * @param Timeout : Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_TransmitReceive(mut husart:
                                                       *mut USART_HandleTypeDef,
                                                   mut pTxData: *mut uint8_t,
                                                   mut pRxData: *mut uint8_t,
                                                   mut Size: uint16_t,
                                                   mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut uhMask: uint16_t = 0;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pTxData.is_null() || pRxData.is_null() ||
               Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_RX;
        /* Init tickstart for timeout managment*/
        tickstart = HAL_GetTick();
        (*husart).RxXferSize = Size;
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                        *mut uint16_t, Size);
        ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                        *mut uint16_t, Size);
        /* Computation of USART mask to apply to RDR register */
        if (*husart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x1ff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0xff as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0xff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x7f as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0x10000000 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x7f as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x3f as libc::c_int as uint16_t }
        }
        uhMask = (*husart).Mask;
        /* Check the remain data to be sent */
        while (*husart).TxXferCount as libc::c_int > 0 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*husart).TxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*husart).RxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            /* Wait until TC flag is set to send data */
            if USART_WaitOnFlagUntilTimeout(husart, 0x40 as libc::c_uint,
                                            RESET, tickstart, Timeout) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            if (*husart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*husart).Init.Parity == 0 as libc::c_uint {
                tmp = pTxData as *mut uint16_t;
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                                *mut uint32_t,
                                            (*tmp as libc::c_int &
                                                 uhMask as libc::c_int) as
                                                uint32_t);
                pTxData = pTxData.offset(2 as libc::c_int as isize)
            } else {
                let fresh2 = pTxData;
                pTxData = pTxData.offset(1);
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                                *mut uint32_t,
                                            (*fresh2 as libc::c_int &
                                                 uhMask as uint8_t as
                                                     libc::c_int) as uint32_t)
            }
            /* Wait for RXNE Flag */
            if USART_WaitOnFlagUntilTimeout(husart, 0x20 as libc::c_uint,
                                            RESET, tickstart, Timeout) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            if (*husart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*husart).Init.Parity == 0 as libc::c_uint {
                tmp = pRxData as *mut uint16_t;
                *tmp =
                    ((*(*husart).Instance).RDR & uhMask as libc::c_uint) as
                        uint16_t;
                pRxData = pRxData.offset(2 as libc::c_uint as isize)
            } else {
                let fresh3 = pRxData;
                pRxData = pRxData.offset(1);
                *fresh3 =
                    ((*(*husart).Instance).RDR &
                         uhMask as uint8_t as libc::c_uint) as uint8_t
            }
        }
        (*husart).State = HAL_USART_STATE_READY;
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Send an amount of data in interrupt mode
  * @param  husart: USART handle
  * @param pTxData: pointer to data buffer
  * @param Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Transmit_IT(mut husart:
                                                   *mut USART_HandleTypeDef,
                                               mut pTxData: *mut uint8_t,
                                               mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pTxData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        (*husart).pTxBuffPtr = pTxData;
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                        *mut uint16_t, Size);
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_TX;
        /* The USART Error Interrupts: (Frame error, noise error, overrun error)
    are not managed by the USART Transmit Process to avoid the overrun interrupt
    when the usart mode is configured for transmit and receive "USART_MODE_TX_RX"
    to benefit for the frame error and noise interrupts the usart mode should be
    configured only for transmit "USART_MODE_TX" */
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        /* Enable the USART Transmit Data Register Empty Interrupt */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Receive an amount of data in blocking mode
  *        To receive synchronous data, dummy data are simultaneously transmitted
  * @param husart: USART handle
  * @param pRxData: pointer to data buffer
  * @param Size: amount of data to be received
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Receive_IT(mut husart:
                                                  *mut USART_HandleTypeDef,
                                              mut pRxData: *mut uint8_t,
                                              mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pRxData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        (*husart).pRxBuffPtr = pRxData;
        (*husart).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                        *mut uint16_t, Size);
        if (*husart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x1ff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0xff as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0xff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x7f as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0x10000000 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x7f as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x3f as libc::c_int as uint16_t }
        }
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_RX;
        /* Enable the USART Parity Error and Data Register not empty Interrupts */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        /* Send dummy byte in order to generate the clock for the Slave to send the next data */
        if (*husart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                            *mut uint32_t,
                                        (0xffff as libc::c_uint as uint16_t as
                                             libc::c_int &
                                             0x1ff as libc::c_uint as uint16_t
                                                 as libc::c_int) as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                            *mut uint32_t,
                                        (0xffff as libc::c_uint as uint16_t as
                                             libc::c_int &
                                             0xff as libc::c_uint as uint16_t
                                                 as libc::c_int) as uint32_t)
        }
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Full-Duplex Send and Receive an amount of data in interrupt mode
  * @param husart: USART handle
  * @param pTxData: pointer to TX data buffer
  * @param pRxData: pointer to RX data buffer
  * @param Size: amount of data to be sent (same amount to be received)
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_TransmitReceive_IT(mut husart:
                                                          *mut USART_HandleTypeDef,
                                                      mut pTxData:
                                                          *mut uint8_t,
                                                      mut pRxData:
                                                          *mut uint8_t,
                                                      mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pTxData.is_null() || pRxData.is_null() ||
               Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        (*husart).pRxBuffPtr = pRxData;
        (*husart).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                        *mut uint16_t, Size);
        (*husart).pTxBuffPtr = pTxData;
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                        *mut uint16_t, Size);
        /* Computation of USART mask to apply to RDR register */
        if (*husart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x1ff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0xff as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0xff as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x7f as libc::c_int as uint16_t }
        } else if (*husart).Init.WordLength == 0x10000000 as libc::c_uint {
            if (*husart).Init.Parity == 0 as libc::c_uint {
                (*husart).Mask = 0x7f as libc::c_int as uint16_t
            } else { (*husart).Mask = 0x3f as libc::c_int as uint16_t }
        }
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_TX_RX;
        /* Enable the USART Data Register not empty Interrupt */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the USART Parity Error Interrupt */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        /* Enable the USART Transmit Data Register Empty Interrupt */
        if 0x727 as libc::c_uint as uint16_t as uint8_t as libc::c_int >>
               5 as libc::c_uint == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (1 as libc::c_uint) <<
                                                 (0x727 as libc::c_uint as
                                                      uint16_t as libc::c_int
                                                      &
                                                      0x1f as libc::c_uint as
                                                          uint16_t as
                                                          libc::c_int)) as
                                            uint32_t as uint32_t)
        } else {
            if 0x727 as libc::c_uint as uint16_t as uint8_t as libc::c_int >>
                   5 as libc::c_uint == 2 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (1 as libc::c_uint) <<
                                                     (0x727 as libc::c_uint as
                                                          uint16_t as
                                                          libc::c_int &
                                                          0x1f as libc::c_uint
                                                              as uint16_t as
                                                              libc::c_int)) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (1 as libc::c_uint) <<
                                                     (0x727 as libc::c_uint as
                                                          uint16_t as
                                                          libc::c_int &
                                                          0x1f as libc::c_uint
                                                              as uint16_t as
                                                              libc::c_int)) as
                                                uint32_t as uint32_t)
            };
        };
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief Send an amount of data in DMA mode
  * @param husart: USART handle
  * @param pTxData: pointer to data buffer
  * @param Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Transmit_DMA(mut husart:
                                                    *mut USART_HandleTypeDef,
                                                mut pTxData: *mut uint8_t,
                                                mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint32_t = 0 as *mut uint32_t;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pTxData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        (*husart).pTxBuffPtr = pTxData;
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                        *mut uint16_t, Size);
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_TX;
        /* Set the USART DMA transfer complete callback */
        (*(*husart).hdmatx).XferCpltCallback =
            Some(USART_DMATransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Half transfer complete callback */
        (*(*husart).hdmatx).XferHalfCpltCallback =
            Some(USART_DMATxHalfCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*husart).hdmatx).XferErrorCallback =
            Some(USART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Enable the USART transmit DMA channel */
        tmp = &mut pTxData as *mut *mut uint8_t as *mut uint32_t;
        HAL_DMA_Start_IT((*husart).hdmatx, *tmp,
                         &mut (*(*husart).Instance).TDR as *mut uint32_t as
                             uint32_t, Size as uint32_t);
        /* Clear the TC flag in the SR register by writing 0 to it */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                        *mut uint32_t, 0x40 as libc::c_uint);
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
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
  * @brief Receive an amount of data in DMA mode
  * @param husart: USART handle
  * @param pRxData: pointer to data buffer
  * @param Size: amount of data to be received
  * @note   When the USART parity is enabled (PCE = 1), the received data contain
  *         the parity bit (MSB position)
  * @retval HAL status
  * @note The USART DMA transmit stream must be configured in order to generate the clock for the slave.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_Receive_DMA(mut husart:
                                                   *mut USART_HandleTypeDef,
                                               mut pRxData: *mut uint8_t,
                                               mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint32_t = 0 as *mut uint32_t;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pRxData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        (*husart).pRxBuffPtr = pRxData;
        (*husart).RxXferSize = Size;
        (*husart).pTxBuffPtr = pRxData;
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_RX;
        /* Set the USART DMA Rx transfer complete callback */
        (*(*husart).hdmarx).XferCpltCallback =
            Some(USART_DMAReceiveCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Half transfer complete callback */
        (*(*husart).hdmarx).XferHalfCpltCallback =
            Some(USART_DMARxHalfCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Rx transfer error callback */
        (*(*husart).hdmarx).XferErrorCallback =
            Some(USART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA abort callback */
        (*(*husart).hdmatx).XferAbortCallback = None;
        /* Set the USART Tx DMA transfer complete callback as NULL because the communication closing
    is performed in DMA reception complete callback  */
        (*(*husart).hdmatx).XferHalfCpltCallback = None;
        (*(*husart).hdmatx).XferCpltCallback = None;
        /* Set the DMA error callback */
        (*(*husart).hdmatx).XferErrorCallback =
            Some(USART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Enable the USART receive DMA channel */
        tmp = &mut pRxData as *mut *mut uint8_t as *mut uint32_t;
        HAL_DMA_Start_IT((*husart).hdmarx,
                         &mut (*(*husart).Instance).RDR as *mut uint32_t as
                             uint32_t, *tmp, Size as uint32_t);
        /* Enable the USART transmit DMA channel: the transmit stream is used in order
       to generate in the non-blocking mode the clock to the slave device,
       this mode isn't a simplex receive mode but a full-duplex receive mode */
        HAL_DMA_Start_IT((*husart).hdmatx, *tmp,
                         &mut (*(*husart).Instance).TDR as *mut uint32_t as
                             uint32_t, Size as uint32_t);
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        /* Enable the USART Parity Error Interrupt */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
       in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
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
  * @brief Full-Duplex Transmit Receive an amount of data in non blocking mode
  * @param husart: USART handle
  * @param pTxData: pointer to TX data buffer
  * @param pRxData: pointer to RX data buffer
  * @param Size: amount of data to be received/sent
  * @note   When the USART parity is enabled (PCE = 1) the data received contain the parity bit.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_TransmitReceive_DMA(mut husart:
                                                           *mut USART_HandleTypeDef,
                                                       mut pTxData:
                                                           *mut uint8_t,
                                                       mut pRxData:
                                                           *mut uint8_t,
                                                       mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint32_t = 0 as *mut uint32_t;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_READY as libc::c_int as libc::c_uint {
        if pTxData.is_null() || pRxData.is_null() ||
               Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*husart).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*husart).Lock = HAL_LOCKED }
        (*husart).pRxBuffPtr = pRxData;
        (*husart).RxXferSize = Size;
        (*husart).pTxBuffPtr = pTxData;
        (*husart).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*husart).State = HAL_USART_STATE_BUSY_TX_RX;
        /* Set the USART DMA Rx transfer complete callback */
        (*(*husart).hdmarx).XferCpltCallback =
            Some(USART_DMAReceiveCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Half transfer complete callback */
        (*(*husart).hdmarx).XferHalfCpltCallback =
            Some(USART_DMARxHalfCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Tx transfer complete callback */
        (*(*husart).hdmatx).XferCpltCallback =
            Some(USART_DMATransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Half transfer complete callback */
        (*(*husart).hdmatx).XferHalfCpltCallback =
            Some(USART_DMATxHalfCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Tx transfer error callback */
        (*(*husart).hdmatx).XferErrorCallback =
            Some(USART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the USART DMA Rx transfer error callback */
        (*(*husart).hdmarx).XferErrorCallback =
            Some(USART_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Enable the USART receive DMA channel */
        tmp = &mut pRxData as *mut *mut uint8_t as *mut uint32_t;
        HAL_DMA_Start_IT((*husart).hdmarx,
                         &mut (*(*husart).Instance).RDR as *mut uint32_t as
                             uint32_t, *tmp, Size as uint32_t);
        /* Enable the USART transmit DMA channel */
        tmp = &mut pTxData as *mut *mut uint8_t as *mut uint32_t;
        HAL_DMA_Start_IT((*husart).hdmatx, *tmp,
                         &mut (*(*husart).Instance).TDR as *mut uint32_t as
                             uint32_t, Size as uint32_t);
        /* Clear the TC flag in the SR register by writing 0 to it */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                        *mut uint32_t, 0x40 as libc::c_uint);
        /* Process Unlocked */
        (*husart).Lock = HAL_UNLOCKED;
        /* Enable the USART Parity Error Interrupt */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the USART Error Interrupt: (Frame error, noise error, overrun error) */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the DMA transfer for the receiver request by setting the DMAR bit
       in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the DMA transfer for transmit request by setting the DMAT bit
       in the USART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
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
  * @brief Pauses the DMA Transfer.
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_DMAPause(mut husart:
                                                *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*husart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*husart).Lock = HAL_LOCKED }
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        /* Disable the USART DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    } else if (*husart).State as libc::c_uint ==
                  HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               5 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   8 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the USART DMA Rx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    } else if (*husart).State as libc::c_uint ==
                  HAL_USART_STATE_BUSY_TX_RX as libc::c_int as libc::c_uint {
        /* Disable the USART DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the USART DMA Rx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Process Unlocked */
    (*husart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief Resumes the DMA Transfer.
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_DMAResume(mut husart:
                                                 *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*husart).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*husart).Lock = HAL_LOCKED }
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        /* Enable the USART DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else if (*husart).State as libc::c_uint ==
                  HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        /* Clear the Overrun flag before resuming the Rx transfer*/
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint);
        /* Reenable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              5 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  8 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the USART DMA Rx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else if (*husart).State as libc::c_uint ==
                  HAL_USART_STATE_BUSY_TX_RX as libc::c_int as libc::c_uint {
        /* Clear the Overrun flag before resuming the Rx transfer*/
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint);
        /* Enable the USART DMA Rx request  before the DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable the USART DMA Tx request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             7 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Process Unlocked */
    (*husart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief Stops the DMA Transfer.
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_DMAStop(mut husart:
                                               *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* The Lock is not implemented on this API to allow the user application
     to call the HAL USART API under callbacks HAL_USART_TxCpltCallback() / HAL_USART_RxCpltCallback() /
     HAL_USART_TxHalfCpltCallback / HAL_USART_RxHalfCpltCallback:
     indeed, when HAL_DMA_Abort() API is called, the DMA TX/RX Transfer or Half Transfer complete
     interrupt is generated if the DMA transfer interruption occurs at the middle or at the end of
     the stream and the corresponding call back is executed. */
    /* Stop USART DMA Tx request if ongoing */
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint &&
           (*(*husart).Instance).CR3 &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        USART_EndTxTransfer(husart);
        /* Abort the USART DMA Tx channel */
        if !(*husart).hdmatx.is_null() { HAL_DMA_Abort((*husart).hdmatx); }
        /* Disable the USART Tx DMA request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Stop USART DMA Rx request if ongoing */
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint &&
           (*(*husart).Instance).CR3 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        USART_EndRxTransfer(husart);
        /* Abort the USART DMA Rx channel */
        if !(*husart).hdmarx.is_null() { HAL_DMA_Abort((*husart).hdmarx); }
        /* Disable the USART Rx DMA request */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  This function handles USART interrupt request.
  * @param  husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_IRQHandler(mut husart:
                                                  *mut USART_HandleTypeDef) {
    let mut isrflags: uint32_t = (*(*husart).Instance).ISR;
    let mut cr1its: uint32_t = (*(*husart).Instance).CR1;
    let mut cr3its: uint32_t = (*(*husart).Instance).CR3;
    let mut errorflags: uint32_t = 0;
    /* If no error occurs */
    errorflags =
        isrflags &
            ((0x1 as libc::c_uint) << 0 as libc::c_uint |
                 (0x1 as libc::c_uint) << 1 as libc::c_uint |
                 (0x1 as libc::c_uint) << 3 as libc::c_uint |
                 (0x1 as libc::c_uint) << 2 as libc::c_uint);
    if errorflags == RESET as libc::c_int as libc::c_uint {
        /* USART in mode Receiver --------------------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr1its & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            if (*husart).State as libc::c_uint ==
                   HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
                USART_Receive_IT(husart);
            } else { USART_TransmitReceive_IT(husart); }
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
        /* USART parity error interrupt occurred ------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr1its & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*husart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*husart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* USART frame error interrupt occurred -------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            1 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*husart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*husart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* USART noise error interrupt occurred -------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            2 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*husart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*husart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x2 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* USART Over-Run interrupt occurred ----------------------------------------*/
        if isrflags & (0x1 as libc::c_uint) << 3 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
               cr3its & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*husart).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*husart).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x8 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* Call USART Error Call back function if need be --------------------------*/
        if (*husart).ErrorCode != 0 as libc::c_uint {
            /* USART in mode Receiver ---------------------------------------------------*/
            if isrflags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint &&
                   cr1its & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                USART_Receive_IT(husart);
            }
            /* If Overrun error occurs, or if any error occurs in DMA mode reception,
      consider error as blocking */
            if (*husart).ErrorCode & 0x8 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint ||
                   (*(*husart).Instance).CR3 &
                       (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                /* Blocking error : transfer is aborted
        Set the USART state ready to be able to start again the process,
        Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
                USART_EndRxTransfer(husart);
                /* Disable the USART DMA Rx request if enabled */
                if (*(*husart).Instance).CR3 &
                       (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint))
                                                    as uint32_t as uint32_t);
                    /* Abort the USART DMA Rx channel */
                    if !(*husart).hdmarx.is_null() {
                        /* Set the USART DMA Abort callback :
            will lead to call HAL_USART_ErrorCallback() at end of DMA abort procedure */
                        (*(*husart).hdmarx).XferAbortCallback =
                            Some(USART_DMAAbortOnError as
                                     unsafe extern "C" fn(_:
                                                              *mut DMA_HandleTypeDef)
                                         -> ());
                        /* Abort DMA RX */
                        if HAL_DMA_Abort_IT((*husart).hdmarx) as libc::c_uint
                               != HAL_OK as libc::c_int as libc::c_uint {
                            /* Call Directly husart->hdmarx->XferAbortCallback function in case of error */
                            (*(*husart).hdmarx).XferAbortCallback.expect("non-null function pointer")((*husart).hdmarx);
                        }
                    } else {
                        /* Call user error callback */
                        HAL_USART_ErrorCallback(husart);
                    }
                } else {
                    /* Call user error callback */
                    HAL_USART_ErrorCallback(husart);
                }
            } else {
                /* Non Blocking error : transfer could go on.
        Error is notified to user through user error callback */
                HAL_USART_ErrorCallback(husart);
                ::core::ptr::write_volatile(&mut (*husart).ErrorCode as
                                                *mut uint32_t,
                                            0 as libc::c_uint)
            }
        }
        return
    }
    /* USART in mode Transmitter -----------------------------------------------*/
    if isrflags & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           cr1its & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        if (*husart).State as libc::c_uint ==
               HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
            USART_Transmit_IT(husart);
        } else { USART_TransmitReceive_IT(husart); }
        return
    }
    /* USART in mode Transmitter (transmission end) -----------------------------*/
    if isrflags & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           cr1its & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        USART_EndTransmit_IT(husart);
        return
    };
}
/* *
  * @brief Tx Transfer completed callbacks
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_TxCpltCallback(mut husart:
                                                      *mut USART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_TxCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief  Tx Half Transfer completed callbacks.
  * @param  husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_TxHalfCpltCallback(mut husart:
                                                          *mut USART_HandleTypeDef) {
    /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_USART_TxHalfCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief  Rx Transfer completed callbacks.
  * @param  husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_RxCpltCallback(mut husart:
                                                      *mut USART_HandleTypeDef) {
    /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_USART_RxCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief Rx Half Transfer completed callbacks
  * @param husart: usart handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_RxHalfCpltCallback(mut husart:
                                                          *mut USART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_RxHalfCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief Tx/Rx Transfers completed callback for the non-blocking process
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_TxRxCpltCallback(mut husart:
                                                        *mut USART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_TxRxCpltCallback can be implemented in the user file
   */
}
/* *
  * @brief USART error callbacks
  * @param husart: USART handle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_ErrorCallback(mut husart:
                                                     *mut USART_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_USART_ErrorCallback can be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup USART_Exported_Functions_Group3 Peripheral State and Errors functions
  *  @brief   USART State and Errors functions
  *
@verbatim
  ==============================================================================
                  ##### Peripheral State and Errors functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to return the State of
    USART communication
    process, return Peripheral Errors occurred during communication process
     (+) HAL_USART_GetState() API can be helpful to check in run-time the state
         of the USART peripheral.
     (+) HAL_USART_GetError() check in run-time errors that could be occurred during
         communication.
@endverbatim
  * @{
  */
/* *
  * @brief return the USART state
  * @param husart: USART handle
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_GetState(mut husart:
                                                *mut USART_HandleTypeDef)
 -> HAL_USART_StateTypeDef {
    return (*husart).State;
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup USART_Exported_Constants USART Exported Constants
  * @{
  */
/* * @defgroup USART_Error_Code USART Error Code
  * @brief    USART Error Code 
  * @{
  */
/* !< No error            */
/* !< Parity error        */
/* !< Noise error         */
/* !< Frame error         */
/* !< Overrun error       */
/* !< DMA transfer error  */
/* *
  * @}
  */
/* * @defgroup USART_Stop_Bits  USART Number of Stop Bits
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Parity    USART Parity
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Mode   USART Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Over_Sampling USART Over Sampling
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Clock  USART Clock
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Clock_Polarity  USART Clock Polarity
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Clock_Phase   USART Clock Phase
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Last_Bit  USART Last Bit
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Request_Parameters  USART Request Parameters
  * @{
  */
/* !< Receive Data flush Request */
/* !< Transmit data flush Request */
/* *
  * @}
  */
/* * @defgroup USART_Flags      USART Flags
  *        Elements values convention: 0xXXXX
  *           - 0xXXXX  : Flag mask in the ISR register
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_Interrupt_definition USART Interrupts Definition
  *        Elements values convention: 0000ZZZZ0XXYYYYYb
  *           - YYYYY  : Interrupt source position in the XX register (5bits)
  *           - XX  : Interrupt source register (2bits)
  *                 - 01: CR1 register
  *                 - 10: CR2 register
  *                 - 11: CR3 register
  *           - ZZZZ  : Flag position in the ISR register(4bits)
  * @{
  */
/* *
  * @}
  */
/* * @defgroup USART_IT_CLEAR_Flags    USART Interruption Clear Flags
  * @{
  */
/* !< Parity Error Clear Flag */
/* !< Framing Error Clear Flag */
/* !< Noise detected Clear Flag */
/* !< OverRun Error Clear Flag */
/* !< IDLE line detected Clear Flag */
/* !< Transmission Complete Clear Flag */
/* !< CTS Interrupt Clear Flag */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macros -----------------------------------------------------------*/
/* * @defgroup USART_Exported_Macros USART Exported Macros
  * @{
  */
/* * @brief Reset USART handle state
  * @param  __HANDLE__: USART handle.
  * @retval None
  */
/* * @brief  Checks whether the specified USART flag is set or not.
  * @param  __HANDLE__: specifies the USART Handle
  * @param  __FLAG__: specifies the flag to check.
  *        This parameter can be one of the following values:
  *            @arg USART_FLAG_REACK: Receive enable acknowledge flag
  *            @arg USART_FLAG_TEACK: Transmit enable acknowledge flag
  *            @arg USART_FLAG_BUSY:  Busy flag
  *            @arg USART_FLAG_CTS:   CTS Change flag
  *            @arg USART_FLAG_TXE:   Transmit data register empty flag
  *            @arg USART_FLAG_TC:    Transmission Complete flag
  *            @arg USART_FLAG_RXNE:  Receive data register not empty flag
  *            @arg USART_FLAG_IDLE:  Idle Line detection flag
  *            @arg USART_FLAG_ORE:   OverRun Error flag
  *            @arg USART_FLAG_NE:    Noise Error flag
  *            @arg USART_FLAG_FE:    Framing Error flag
  *            @arg USART_FLAG_PE:    Parity Error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
/* * @brief  Enables the specified USART interrupt.
  * @param  __HANDLE__: specifies the USART Handle
  * @param  __INTERRUPT__: specifies the USART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg USART_IT_TC:   Transmission complete interrupt
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg USART_IT_IDLE: Idle line detection interrupt
  *            @arg USART_IT_PE:   Parity Error interrupt
  *            @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
/* * @brief  Disables the specified USART interrupt.
  * @param  __HANDLE__: specifies the USART Handle.
  * @param  __INTERRUPT__: specifies the USART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg USART_IT_TC:   Transmission complete interrupt
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg USART_IT_IDLE: Idle line detection interrupt
  *            @arg USART_IT_PE:   Parity Error interrupt
  *            @arg USART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
/* * @brief  Checks whether the specified USART interrupt has occurred or not.
  * @param  __HANDLE__: specifies the USART Handle
  * @param  __IT__: specifies the USART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_TXE: Transmit Data Register empty interrupt
  *            @arg USART_IT_TC:  Transmission complete interrupt
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg USART_IT_IDLE: Idle line detection interrupt
  *            @arg USART_IT_ORE: OverRun Error interrupt
  *            @arg USART_IT_NE: Noise Error interrupt
  *            @arg USART_IT_FE: Framing Error interrupt
  *            @arg USART_IT_PE: Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
/* * @brief  Checks whether the specified USART interrupt source is enabled.
  * @param  __HANDLE__: specifies the USART Handle.
  * @param  __IT__: specifies the USART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg USART_IT_TXE: Transmit Data Register empty interrupt
  *            @arg USART_IT_TC:  Transmission complete interrupt
  *            @arg USART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg USART_IT_IDLE: Idle line detection interrupt
  *            @arg USART_IT_ORE: OverRun Error interrupt
  *            @arg USART_IT_NE: Noise Error interrupt
  *            @arg USART_IT_FE: Framing Error interrupt
  *            @arg USART_IT_PE: Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
/* * @brief  Clears the specified USART ISR flag, in setting the proper ICR register flag.
  * @param  __HANDLE__: specifies the USART Handle.
  * @param  __IT_CLEAR__: specifies the interrupt clear register flag that needs to be set
  *                       to clear the corresponding interrupt
  *          This parameter can be one of the following values:
  *            @arg USART_CLEAR_PEF: Parity Error Clear Flag
  *            @arg USART_CLEAR_FEF: Framing Error Clear Flag
  *            @arg USART_CLEAR_NEF: Noise detected Clear Flag
  *            @arg USART_CLEAR_OREF: OverRun Error Clear Flag
  *            @arg USART_CLEAR_IDLEF: IDLE line detected Clear Flag
  *            @arg USART_CLEAR_TCF: Transmission Complete Clear Flag
  *            @arg USART_CLEAR_CTSF: CTS Interrupt Clear Flag
  * @retval None
  */
/* * @brief  Set a specific USART request flag.
  * @param  __HANDLE__: specifies the USART Handle.
  * @param  __REQ__: specifies the request flag to set
  *          This parameter can be one of the following values:
  *            @arg USART_RXDATA_FLUSH_REQUEST: Receive Data flush Request
  *            @arg USART_TXDATA_FLUSH_REQUEST: Transmit data flush Request
  *
  * @retval None
  */
/* * @brief  Enable USART
  * @param  __HANDLE__: specifies the USART Handle.
  * @retval None
  */
/* * @brief  Disable USART
  * @param  __HANDLE__: specifies the USART Handle.
  * @retval None
  */
/* *
  * @}
  */
/* Include UART HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup USART_Exported_Functions
  * @{
  */
/* * @addtogroup USART_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  **********************************/
/* *
  * @}
  */
/* * @addtogroup USART_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *******************************************************/
/* *
  * @}
  */
/* * @addtogroup USART_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  ************************************************/
/* *
  * @brief  Return the USART error code
  * @param  husart : pointer to a USART_HandleTypeDef structure that contains
  *              the configuration information for the specified USART.
  * @retval USART Error Code
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_USART_GetError(mut husart:
                                                *mut USART_HandleTypeDef)
 -> uint32_t {
    return (*husart).ErrorCode;
}
/* *
  * @}
  */
/* *
  * @brief  Simplex Send an amount of data in non-blocking mode.
  * @note   Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_Transmit_IT().
  * @param  husart: USART handle
  * @retval HAL status
  * @note   The USART errors are not managed to avoid the overrun error.
  */
unsafe extern "C" fn USART_Transmit_IT(mut husart: *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        if (*husart).TxXferCount as libc::c_uint == 0 as libc::c_uint {
            /* Disable the USART Transmit data register empty interrupt */
            if 0x727 as libc::c_uint as uint16_t as uint8_t as libc::c_int >>
                   5 as libc::c_uint == 1 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((1 as libc::c_uint) <<
                                                       (0x727 as libc::c_uint
                                                            as uint16_t as
                                                            libc::c_int &
                                                            0x1f as
                                                                libc::c_uint
                                                                as uint16_t as
                                                                libc::c_int)))
                                                as uint32_t as uint32_t)
            } else {
                if 0x727 as libc::c_uint as uint16_t as uint8_t as libc::c_int
                       >> 5 as libc::c_uint == 2 as libc::c_int {
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((1 as libc::c_uint) <<
                                                           (0x727 as
                                                                libc::c_uint
                                                                as uint16_t as
                                                                libc::c_int &
                                                                0x1f as
                                                                    libc::c_uint
                                                                    as
                                                                    uint16_t
                                                                    as
                                                                    libc::c_int)))
                                                    as uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((1 as libc::c_uint) <<
                                                           (0x727 as
                                                                libc::c_uint
                                                                as uint16_t as
                                                                libc::c_int &
                                                                0x1f as
                                                                    libc::c_uint
                                                                    as
                                                                    uint16_t
                                                                    as
                                                                    libc::c_int)))
                                                    as uint32_t as uint32_t)
                };
            };
            /* Enable the USART Transmit Complete Interrupt */
            if 0x626 as libc::c_uint as uint16_t as uint8_t as libc::c_int >>
                   5 as libc::c_uint == 1 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (1 as libc::c_uint) <<
                                                     (0x626 as libc::c_uint as
                                                          uint16_t as
                                                          libc::c_int &
                                                          0x1f as libc::c_uint
                                                              as uint16_t as
                                                              libc::c_int)) as
                                                uint32_t as uint32_t)
            } else {
                if 0x626 as libc::c_uint as uint16_t as uint8_t as libc::c_int
                       >> 5 as libc::c_uint == 2 as libc::c_int {
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (1 as libc::c_uint) <<
                                                         (0x626 as
                                                              libc::c_uint as
                                                              uint16_t as
                                                              libc::c_int &
                                                              0x1f as
                                                                  libc::c_uint
                                                                  as uint16_t
                                                                  as
                                                                  libc::c_int))
                                                    as uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (1 as libc::c_uint) <<
                                                         (0x626 as
                                                              libc::c_uint as
                                                              uint16_t as
                                                              libc::c_int &
                                                              0x1f as
                                                                  libc::c_uint
                                                                  as uint16_t
                                                                  as
                                                                  libc::c_int))
                                                    as uint32_t as uint32_t)
                };
            };
            return HAL_OK
        } else {
            if (*husart).Init.WordLength ==
                   (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                   (*husart).Init.Parity == 0 as libc::c_uint {
                tmp = (*husart).pTxBuffPtr as *mut uint16_t;
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                                *mut uint32_t,
                                            (*tmp as libc::c_int &
                                                 0x1ff as libc::c_uint as
                                                     uint16_t as libc::c_int)
                                                as uint32_t);
                (*husart).pTxBuffPtr =
                    (*husart).pTxBuffPtr.offset(2 as libc::c_uint as isize)
            } else {
                let fresh4 = (*husart).pTxBuffPtr;
                (*husart).pTxBuffPtr = (*husart).pTxBuffPtr.offset(1);
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                                *mut uint32_t,
                                            (*fresh4 as libc::c_int &
                                                 0xff as libc::c_int as
                                                     uint8_t as libc::c_int)
                                                as uint8_t as uint32_t)
            }
            ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*husart).TxXferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            return HAL_OK
        }
    } else { return HAL_BUSY };
}
/* *
  * @brief  Wraps up transmission in non-blocking mode.
  * @param  husart: pointer to a USART_HandleTypeDef structure that contains
  *                the configuration information for the specified USART module.
  * @retval HAL status
  */
unsafe extern "C" fn USART_EndTransmit_IT(mut husart:
                                              *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Disable the USART Transmit Complete Interrupt */
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    (*husart).State = HAL_USART_STATE_READY;
    HAL_USART_TxCpltCallback(husart);
    return HAL_OK;
}
/* *
  * @brief  Simplex Receive an amount of data in non-blocking mode.
  *         Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_Receive_IT()
  * @param  husart: USART handle
  * @retval HAL status
  */
unsafe extern "C" fn USART_Receive_IT(mut husart: *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut uhMask: uint16_t = (*husart).Mask;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        if (*husart).Init.WordLength ==
               (0x1 as libc::c_uint) << 12 as libc::c_uint &&
               (*husart).Init.Parity == 0 as libc::c_uint {
            tmp = (*husart).pRxBuffPtr as *mut uint16_t;
            *tmp =
                ((*(*husart).Instance).RDR & uhMask as libc::c_uint) as
                    uint16_t;
            (*husart).pRxBuffPtr =
                (*husart).pRxBuffPtr.offset(2 as libc::c_uint as isize)
        } else {
            let fresh5 = (*husart).pRxBuffPtr;
            (*husart).pRxBuffPtr = (*husart).pRxBuffPtr.offset(1);
            *fresh5 =
                ((*(*husart).Instance).RDR &
                     uhMask as uint8_t as libc::c_uint) as uint8_t
        }
        /* Send dummy byte in order to generate the clock for the Slave to Send the next data */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR as
                                        *mut uint32_t,
                                    (0xffff as libc::c_uint as uint16_t as
                                         libc::c_int &
                                         0xff as libc::c_uint as uint16_t as
                                             libc::c_int) as uint32_t);
        ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                        *mut uint16_t,
                                    ::core::ptr::read_volatile::<uint16_t>(&(*husart).RxXferCount
                                                                               as
                                                                               *const uint16_t).wrapping_sub(1));
        if ::core::ptr::read_volatile::<uint16_t>(&(*husart).RxXferCount as
                                                      *const uint16_t) as
               libc::c_uint == 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Disable the USART Parity Error Interrupt */
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   8 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            (*husart).State = HAL_USART_STATE_READY;
            HAL_USART_RxCpltCallback(husart);
            return HAL_OK
        }
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Full-Duplex Send receive an amount of data in full-duplex mode (non-blocking).
  *         Function called under interruption only, once
  *         interruptions have been enabled by HAL_USART_TransmitReceive_IT()
  * @param  husart: USART handle
  * @retval HAL status
  */
unsafe extern "C" fn USART_TransmitReceive_IT(mut husart:
                                                  *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp: *mut uint16_t = 0 as *mut uint16_t;
    let mut uhMask: uint16_t = (*husart).Mask;
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_TX_RX as libc::c_int as libc::c_uint {
        if (*husart).TxXferCount as libc::c_uint != 0 as libc::c_uint {
            if ((*(*husart).Instance).ISR & 0x80 as libc::c_uint ==
                    0x80 as libc::c_uint) as libc::c_int !=
                   RESET as libc::c_int {
                if (*husart).Init.WordLength ==
                       (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                       (*husart).Init.Parity == 0 as libc::c_uint {
                    tmp = (*husart).pTxBuffPtr as *mut uint16_t;
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR
                                                    as *mut uint32_t,
                                                (*tmp as libc::c_int &
                                                     uhMask as libc::c_int) as
                                                    uint16_t as uint32_t);
                    (*husart).pTxBuffPtr =
                        (*husart).pTxBuffPtr.offset(2 as libc::c_uint as
                                                        isize)
                } else {
                    let fresh6 = (*husart).pTxBuffPtr;
                    (*husart).pTxBuffPtr = (*husart).pTxBuffPtr.offset(1);
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).TDR
                                                    as *mut uint32_t,
                                                (*fresh6 as libc::c_int &
                                                     uhMask as uint8_t as
                                                         libc::c_int) as
                                                    uint8_t as uint32_t)
                }
                ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                                *mut uint16_t,
                                            ::core::ptr::read_volatile::<uint16_t>(&(*husart).TxXferCount
                                                                                       as
                                                                                       *const uint16_t).wrapping_sub(1));
                /* Check the latest data transmitted */
                if (*husart).TxXferCount as libc::c_uint == 0 as libc::c_uint
                   {
                    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           7 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                }
            }
        }
        if (*husart).RxXferCount as libc::c_uint != 0 as libc::c_uint {
            if ((*(*husart).Instance).ISR & 0x20 as libc::c_uint ==
                    0x20 as libc::c_uint) as libc::c_int !=
                   RESET as libc::c_int {
                if (*husart).Init.WordLength ==
                       (0x1 as libc::c_uint) << 12 as libc::c_uint &&
                       (*husart).Init.Parity == 0 as libc::c_uint {
                    tmp = (*husart).pRxBuffPtr as *mut uint16_t;
                    *tmp =
                        ((*(*husart).Instance).RDR & uhMask as libc::c_uint)
                            as uint16_t;
                    (*husart).pRxBuffPtr =
                        (*husart).pRxBuffPtr.offset(2 as libc::c_uint as
                                                        isize)
                } else {
                    let fresh7 = (*husart).pRxBuffPtr;
                    (*husart).pRxBuffPtr = (*husart).pRxBuffPtr.offset(1);
                    *fresh7 =
                        ((*(*husart).Instance).RDR &
                             uhMask as uint8_t as libc::c_uint) as uint8_t
                }
                ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                                *mut uint16_t,
                                            ::core::ptr::read_volatile::<uint16_t>(&(*husart).RxXferCount
                                                                                       as
                                                                                       *const uint16_t).wrapping_sub(1))
            }
        }
        /* Check the latest data received */
        if (*husart).RxXferCount as libc::c_uint == 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Disable the USART Parity Error Interrupt */
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   8 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            (*husart).State = HAL_USART_STATE_READY;
            HAL_USART_TxRxCpltCallback(husart);
            return HAL_OK
        }
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  This function handles USART Communication Timeout.
  * @param  husart USART handle
  * @param  Flag specifies the USART flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
unsafe extern "C" fn USART_WaitOnFlagUntilTimeout(mut husart:
                                                      *mut USART_HandleTypeDef,
                                                  mut Flag: uint32_t,
                                                  mut Status: FlagStatus,
                                                  mut Tickstart: uint32_t,
                                                  mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    /* Wait until flag is set */
    while (if (*(*husart).Instance).ISR & Flag == Flag {
               SET as libc::c_int
           } else { RESET as libc::c_int }) as libc::c_uint ==
              Status as libc::c_uint {
        /* Check for the Timeout */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(Tickstart) >= Timeout {
                /* Disable the USART Transmit Complete Interrupt */
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                /* Disable the USART RXNE Interrupt */
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       5 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                /* Disable the USART Parity Error Interrupt */
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       8 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                /* Disable the USART Error Interrupt: (Frame error, noise error, overrun error) */
                ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       0 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                (*husart).State = HAL_USART_STATE_READY;
                /* Process Unlocked */
                (*husart).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @addtogroup USART_Private_Functions
  * @{
  */
/* *
  * @brief DMA USART transmit process complete callback
  * @param  hdma: DMA handle
  * @retval None
  */
unsafe extern "C" fn USART_DMATransmitCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut husart: *mut USART_HandleTypeDef =
        (*hdma).Parent as *mut USART_HandleTypeDef;
    /* DMA Normal mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint ==
           0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*husart).TxXferCount as
                                        *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        if (*husart).State as libc::c_uint ==
               HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
            /* Disable the DMA transfer for transmit request by resetting the DMAT bit
         in the USART CR3 register */
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   7 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Enable the USART Transmit Complete Interrupt */
            ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    } else if (*husart).State as libc::c_uint ==
                  HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        HAL_USART_TxCpltCallback(husart);
    };
}
/* DMA Circular mode */
/* *
  * @brief DMA USART transmit process half complete callback
  * @param hdma : DMA handle
  * @retval None
  */
unsafe extern "C" fn USART_DMATxHalfCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut husart: *mut USART_HandleTypeDef =
        (*hdma).Parent as *mut USART_HandleTypeDef;
    HAL_USART_TxHalfCpltCallback(husart);
}
/* *
  * @brief DMA USART receive process complete callback
  * @param  hdma: DMA handle
  * @retval None
  */
unsafe extern "C" fn USART_DMAReceiveCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut husart: *mut USART_HandleTypeDef =
        (*hdma).Parent as *mut USART_HandleTypeDef;
    /* DMA Normal mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint ==
           0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*husart).RxXferCount as
                                        *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               5 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   8 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the DMA RX transfer for the receiver request by resetting the DMAR bit
    in USART CR3 register */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* similarly, disable the DMA TX transfer that was started to provide the
       clock to the slave device */
        ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*husart).State as libc::c_uint ==
               HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
            HAL_USART_RxCpltCallback(husart);
        } else {
            /* The USART state is HAL_USART_STATE_BUSY_TX_RX */
            HAL_USART_TxRxCpltCallback(husart);
        }
        (*husart).State = HAL_USART_STATE_READY
    } else if (*husart).State as libc::c_uint ==
                  HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        HAL_USART_RxCpltCallback(husart);
    } else {
        /* DMA circular mode */
        /* The USART state is HAL_USART_STATE_BUSY_TX_RX */
        HAL_USART_TxRxCpltCallback(husart);
    };
}
/* *
  * @brief DMA USART receive process half complete callback
  * @param hdma : DMA handle
  * @retval None
  */
unsafe extern "C" fn USART_DMARxHalfCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut husart: *mut USART_HandleTypeDef =
        (*hdma).Parent as *mut USART_HandleTypeDef;
    HAL_USART_RxHalfCpltCallback(husart);
}
/* *
  * @brief DMA USART communication error callback
  * @param  hdma: DMA handle
  * @retval None
  */
unsafe extern "C" fn USART_DMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut husart: *mut USART_HandleTypeDef =
        (*hdma).Parent as *mut USART_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*husart).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*husart).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    /* Stop USART DMA Tx request if ongoing */
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_TX as libc::c_int as libc::c_uint &&
           (*(*husart).Instance).CR3 &
               (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        USART_EndTxTransfer(husart);
    }
    /* Stop USART DMA Rx request if ongoing */
    if (*husart).State as libc::c_uint ==
           HAL_USART_STATE_BUSY_RX as libc::c_int as libc::c_uint &&
           (*(*husart).Instance).CR3 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        USART_EndRxTransfer(husart);
    }
    ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*husart).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x10 as libc::c_uint)
                                    as uint32_t as uint32_t);
    (*husart).State = HAL_USART_STATE_READY;
    HAL_USART_ErrorCallback(husart);
}
/* *
  * @brief DMA USART communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param hdma: DMA handle.
  * @retval None
  */
unsafe extern "C" fn USART_DMAAbortOnError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut husart: *mut USART_HandleTypeDef =
        (*hdma).Parent as *mut USART_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*husart).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*husart).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    HAL_USART_ErrorCallback(husart);
}
/* *
  * @brief  End ongoing Tx transfer on USART peripheral (following error detection or Transmit completion).
  * @param  husart: USART handle.
  * @retval None
  */
unsafe extern "C" fn USART_EndTxTransfer(mut husart:
                                             *mut USART_HandleTypeDef) {
    /* At end of Tx process, restore husart->State to Ready */
    (*husart).State = HAL_USART_STATE_READY;
    /* Disable TXEIE and TCIE interrupts */
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           7 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  End ongoing Rx transfer on USART peripheral (following error detection or Reception completion).
  * @param  husart: USART handle.
  * @retval None
  */
unsafe extern "C" fn USART_EndRxTransfer(mut husart:
                                             *mut USART_HandleTypeDef) {
    /* At end of Rx process, restore husart->RxState to Ready */
    (*husart).State = HAL_USART_STATE_READY;
    /* Disable RXNE, PE and ERR interrupts */
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               8 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR3 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*husart).Instance).CR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief Configure the USART peripheral
  * @param husart: USART handle
  * @retval None
  */
unsafe extern "C" fn USART_SetConfig(mut husart: *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmpreg: uint32_t = 0 as libc::c_uint;
    let mut clocksource: USART_ClockSourceTypeDef =
        USART_CLOCKSOURCE_UNDEFINED;
    let mut ret: HAL_StatusTypeDef = HAL_OK;
    let mut brrtemp: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut usartdiv: uint16_t = 0 as libc::c_uint as uint16_t;
    /* Check the parameters */
    /*-------------------------- USART CR1 Configuration -----------------------*/
   /* Clear M, PCE, PS, TE and RE bits and configure
   *  the USART Word Length, Parity, Mode and OverSampling:
   *  set the M bits according to husart->Init.WordLength value
   *  set PCE and PS bits according to husart->Init.Parity value
   *  set TE and RE bits according to husart->Init.Mode value
   *  force OVER8 to 1 to allow to reach the maximum speed (Fclock/8) */
    tmpreg =
        (*husart).Init.WordLength | (*husart).Init.Parity |
            (*husart).Init.Mode | (0x1 as libc::c_uint) << 15 as libc::c_uint;
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR1 as
                                    *mut uint32_t,
                                (*(*husart).Instance).CR1 &
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
    /*---------------------------- USART CR2 Configuration ---------------------*/
  /* Clear and configure the USART Clock, CPOL, CPHA, LBCL and STOP bits:
   * set CPOL bit according to husart->Init.CLKPolarity value
   * set CPHA bit according to husart->Init.CLKPhase value
   * set LBCL bit according to husart->Init.CLKLastBit value
   * set STOP[13:12] bits according to husart->Init.StopBits value */
    tmpreg = (0x1 as libc::c_uint) << 11 as libc::c_uint;
    tmpreg |= (*husart).Init.CLKPolarity | (*husart).Init.CLKPhase;
    tmpreg |= (*husart).Init.CLKLastBit | (*husart).Init.StopBits;
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).CR2 as
                                    *mut uint32_t,
                                (*(*husart).Instance).CR2 &
                                    !((0x1 as libc::c_uint) <<
                                          9 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              10 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              11 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              8 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              12 as libc::c_uint) | tmpreg);
    /*-------------------------- USART CR3 Configuration -----------------------*/
  /* no CR3 register configuration                                            */
    /*-------------------------- USART BRR Configuration -----------------------*/
  /* BRR is filled-up according to OVER8 bit setting which is forced to 1     */
    if (*husart).Instance ==
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
            0 => { clocksource = USART_CLOCKSOURCE_PCLK2 }
            2 => { clocksource = USART_CLOCKSOURCE_HSI }
            1 => { clocksource = USART_CLOCKSOURCE_SYSCLK }
            3 => { clocksource = USART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*husart).Instance ==
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
            0 => { clocksource = USART_CLOCKSOURCE_PCLK1 }
            8 => { clocksource = USART_CLOCKSOURCE_HSI }
            4 => { clocksource = USART_CLOCKSOURCE_SYSCLK }
            12 => { clocksource = USART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*husart).Instance ==
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
            0 => { clocksource = USART_CLOCKSOURCE_PCLK1 }
            32 => { clocksource = USART_CLOCKSOURCE_HSI }
            16 => { clocksource = USART_CLOCKSOURCE_SYSCLK }
            48 => { clocksource = USART_CLOCKSOURCE_LSE }
            _ => { }
        }
    } else if (*husart).Instance ==
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
            0 => { clocksource = USART_CLOCKSOURCE_PCLK2 }
            2048 => { clocksource = USART_CLOCKSOURCE_HSI }
            1024 => { clocksource = USART_CLOCKSOURCE_SYSCLK }
            3072 => { clocksource = USART_CLOCKSOURCE_LSE }
            _ => { }
        }
    }
    match clocksource as libc::c_uint {
        0 => {
            usartdiv =
                (2 as libc::c_int as
                     libc::c_uint).wrapping_mul(HAL_RCC_GetPCLK1Freq()).wrapping_add((*husart).Init.BaudRate.wrapping_div(2
                                                                                                                              as
                                                                                                                              libc::c_int
                                                                                                                              as
                                                                                                                              libc::c_uint)).wrapping_div((*husart).Init.BaudRate)
                    as uint16_t
        }
        1 => {
            usartdiv =
                (2 as libc::c_int as
                     libc::c_uint).wrapping_mul(HAL_RCC_GetPCLK2Freq()).wrapping_add((*husart).Init.BaudRate.wrapping_div(2
                                                                                                                              as
                                                                                                                              libc::c_int
                                                                                                                              as
                                                                                                                              libc::c_uint)).wrapping_div((*husart).Init.BaudRate)
                    as uint16_t
        }
        2 => {
            usartdiv =
                (2 as libc::c_int as
                     libc::c_uint).wrapping_mul(16000000 as
                                                    libc::c_uint).wrapping_add((*husart).Init.BaudRate.wrapping_div(2
                                                                                                                        as
                                                                                                                        libc::c_int
                                                                                                                        as
                                                                                                                        libc::c_uint)).wrapping_div((*husart).Init.BaudRate)
                    as uint16_t
        }
        4 => {
            usartdiv =
                (2 as libc::c_int as
                     libc::c_uint).wrapping_mul(HAL_RCC_GetSysClockFreq()).wrapping_add((*husart).Init.BaudRate.wrapping_div(2
                                                                                                                                 as
                                                                                                                                 libc::c_int
                                                                                                                                 as
                                                                                                                                 libc::c_uint)).wrapping_div((*husart).Init.BaudRate)
                    as uint16_t
        }
        8 => {
            usartdiv =
                (2 as libc::c_int as
                     libc::c_uint).wrapping_mul(32768 as
                                                    libc::c_uint).wrapping_add((*husart).Init.BaudRate.wrapping_div(2
                                                                                                                        as
                                                                                                                        libc::c_int
                                                                                                                        as
                                                                                                                        libc::c_uint)).wrapping_div((*husart).Init.BaudRate)
                    as uint16_t
        }
        16 | _ => { ret = HAL_ERROR }
    }
    brrtemp = (usartdiv as libc::c_uint & 0xfff0 as libc::c_uint) as uint16_t;
    brrtemp =
        (brrtemp as libc::c_int |
             ((usartdiv as libc::c_int &
                   0xf as libc::c_uint as uint16_t as libc::c_int) >>
                  1 as libc::c_uint) as uint16_t as libc::c_int) as uint16_t;
    ::core::ptr::write_volatile(&mut (*(*husart).Instance).BRR as
                                    *mut uint32_t, brrtemp as uint32_t);
    return ret;
}
/* *
  * @brief Check the USART Idle State
  * @param husart: USART handle
  * @retval HAL status
  */
unsafe extern "C" fn USART_CheckIdleState(mut husart:
                                              *mut USART_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Initialize the USART ErrorCode */
    ::core::ptr::write_volatile(&mut (*husart).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Check if the Transmitter is enabled */
    if (*(*husart).Instance).CR1 & (0x1 as libc::c_uint) << 3 as libc::c_uint
           == (0x1 as libc::c_uint) << 3 as libc::c_uint {
        /* Wait until TEACK flag is set */
        if USART_WaitOnFlagUntilTimeout(husart,
                                        (0x1 as libc::c_uint) <<
                                            21 as libc::c_uint, RESET,
                                        tickstart, 1000 as libc::c_uint) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            (*husart).State = HAL_USART_STATE_TIMEOUT;
            return HAL_TIMEOUT
        }
    }
    /* Initialize the USART state*/
    (*husart).State = HAL_USART_STATE_READY;
    /* Process Unlocked */
    (*husart).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_USART_MODULE_ENABLED */
/* *
  * @}
  */
