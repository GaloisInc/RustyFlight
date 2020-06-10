use ::libc;
extern "C" {
    /* *
  ******************************************************************************
  * @file    system_stm32f7xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS Cortex-M7 Device System Source File for STM32F7xx devices.       
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
    /* * @addtogroup stm32f7xx_system
  * @{
  */
    /* *
  * @brief Define to prevent recursive inclusion
  */
    /* * @addtogroup STM32F7xx_System_Includes
  * @{
  */
    /* *
  * @}
  */
    /* * @addtogroup STM32F7xx_System_Exported_Variables
  * @{
  */
  /* The SystemCoreClock variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency 
         Note: If you use this function to configure the system clock; then there
               is no need to call the 2 first functions listed above, since SystemCoreClock
               variable is updated automatically.
    */
    #[no_mangle]
    static mut SystemCoreClock: uint32_t;
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_DMA_Abort(hdma: *mut DMA_HandleTypeDef) -> HAL_StatusTypeDef;
    #[no_mangle]
    fn HAL_ADCEx_InjectedConvCpltCallback(hadc: *mut ADC_HandleTypeDef);
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_TypeDef {
    pub SR: uint32_t,
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub JOFR1: uint32_t,
    pub JOFR2: uint32_t,
    pub JOFR3: uint32_t,
    pub JOFR4: uint32_t,
    pub HTR: uint32_t,
    pub LTR: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub JSQR: uint32_t,
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub DR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_Common_TypeDef {
    pub CSR: uint32_t,
    pub CCR: uint32_t,
    pub CDR: uint32_t,
}
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const ENABLE: C2RustUnnamed_0 = 1;
pub const DISABLE: C2RustUnnamed_0 = 0;
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
pub struct ADC_InitTypeDef {
    pub ClockPrescaler: uint32_t,
    pub Resolution: uint32_t,
    pub DataAlign: uint32_t,
    pub ScanConvMode: uint32_t,
    pub EOCSelection: uint32_t,
    pub ContinuousConvMode: uint32_t,
    pub NbrOfConversion: uint32_t,
    pub DiscontinuousConvMode: uint32_t,
    pub NbrOfDiscConversion: uint32_t,
    pub ExternalTrigConv: uint32_t,
    pub ExternalTrigConvEdge: uint32_t,
    pub DMAContinuousRequests: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_ChannelConfTypeDef {
    pub Channel: uint32_t,
    pub Rank: uint32_t,
    pub SamplingTime: uint32_t,
    pub Offset: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_AnalogWDGConfTypeDef {
    pub WatchdogMode: uint32_t,
    pub HighThreshold: uint32_t,
    pub LowThreshold: uint32_t,
    pub Channel: uint32_t,
    pub ITMode: uint32_t,
    pub WatchdogNumber: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_HandleTypeDef {
    pub Instance: *mut ADC_TypeDef,
    pub Init: ADC_InitTypeDef,
    pub NbrOfCurrentConversionRank: uint32_t,
    pub DMA_Handle: *mut DMA_HandleTypeDef,
    pub Lock: HAL_LockTypeDef,
    pub State: uint32_t,
    pub ErrorCode: uint32_t,
}
#[inline(always)]
unsafe extern "C" fn __RBIT(mut value: uint32_t) -> uint32_t {
    let mut result: uint32_t = 0;
    let mut s: uint32_t =
        (4 as
             libc::c_uint).wrapping_mul(8 as
                                            libc::c_uint).wrapping_sub(1 as
                                                                           libc::c_uint);
    result = value;
    value >>= 1 as libc::c_uint;
    while value != 0 as libc::c_uint {
        result <<= 1 as libc::c_uint;
        result |= value & 1 as libc::c_uint;
        s = s.wrapping_sub(1);
        value >>= 1 as libc::c_uint
    }
    result <<= s;
    return result;
}
/* !< DMA Stream Index                       */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup ADC_Exported_Functions ADC Exported Functions
  * @{
  */
/* * @defgroup ADC_Exported_Functions_Group1 Initialization and de-initialization functions 
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Initialize and configure the ADC. 
      (+) De-initialize the ADC. 
         
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the ADCx peripheral according to the specified parameters 
  *         in the ADC_InitStruct and initializes the ADC MSP.
  *           
  * @note   This function is used to configure the global features of the ADC ( 
  *         ClockPrescaler, Resolution, Data Alignment and number of conversion), however,
  *         the rest of the configuration parameters are specific to the regular
  *         channels group (scan mode activation, continuous mode activation,
  *         External trigger source and edge, DMA continuous request after the  
  *         last transfer and End of conversion selection).
  *             
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Init(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp_hal_status: HAL_StatusTypeDef = HAL_OK;
    /* Check ADC handle */
    if hadc.is_null() { return HAL_ERROR }
    /* Check the parameters */
    ((*hadc).Init.ExternalTrigConv) !=
        ((0xf as libc::c_uint) <<
             24 as
                 libc::c_uint).wrapping_add(1 as libc::c_int as libc::c_uint);
    if (*hadc).State == 0 as libc::c_uint {
        /* Initialize ADC error code */
        ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Allocate lock resource and initialize it */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Init the low level hardware */
        HAL_ADC_MspInit(hadc);
    }
    /* Configuration of ADC parameters if previous preliminary actions are      */ 
  /* correctly completed.                                                     */
    if (*hadc).State & 0x10 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x100 as libc::c_uint |
                                              0x1000 as libc::c_uint) |
                                        0x2 as libc::c_uint);
        /* Set ADC parameters */
        ADC_Init(hadc);
        /* Set ADC error code to none */
        ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Set the ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State & !(0x2 as libc::c_uint) |
                                        0x1 as libc::c_uint)
    } else { tmp_hal_status = HAL_ERROR }
    /* Release Lock */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return tmp_hal_status;
}
/* *
  * @brief  Deinitializes the ADCx peripheral registers to their default reset values. 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_DeInit(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp_hal_status: HAL_StatusTypeDef = HAL_OK;
    /* Check ADC handle */
    if hadc.is_null() { return HAL_ERROR }
    /* Check the parameters */
    /* Set ADC state */
    ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x2 as libc::c_uint) as
                                    uint32_t as uint32_t);
    /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configuration of ADC parameters if previous preliminary actions are      */ 
  /* correctly completed.                                                     */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        /* DeInit the low level hardware */
        HAL_ADC_MspDeInit(hadc);
        /* Set ADC error code to none */
        ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    0 as libc::c_uint)
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return tmp_hal_status;
}
/* *
  * @brief  Initializes the ADC MSP.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_MspInit(mut hadc: *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_MspInit could be implemented in the user file
   */
}
/* *
  * @brief  DeInitializes the ADC MSP.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_MspDeInit(mut hadc: *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_MspDeInit could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup ADC_Exported_Functions_Group2 IO operation functions
 *  @brief    IO operation functions 
 *
@verbatim   
 ===============================================================================
             ##### IO operation functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Start conversion of regular channel.
      (+) Stop conversion of regular channel.
      (+) Start conversion of regular channel and enable interrupt.
      (+) Stop conversion of regular channel and disable interrupt.
      (+) Start conversion of regular channel and enable DMA transfer.
      (+) Stop conversion of regular channel and disable DMA transfer.
      (+) Handle ADC interrupt request. 
               
@endverbatim
  * @{
  */
/* *
  * @brief  Enables ADC and starts conversion of the regular channels.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Start(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Enable the ADC peripheral */
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
  Tstab time the ADC's stabilization */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
        ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                    (3 as
                                         libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                     as
                                                                                                     libc::c_int
                                                                                                     as
                                                                                                     libc::c_uint)));
        while counter != 0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                   as
                                                                                   *const uint32_t).wrapping_sub(1))
        }
    }
    /* Start conversion if ADC is effectively enabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x1 as libc::c_uint |
                                              0x200 as libc::c_uint |
                                              0x400 as libc::c_uint) |
                                        0x100 as libc::c_uint);
        /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
        if (*(*hadc).Instance).CR1 &
               (0x1 as libc::c_uint) << 10 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (*hadc).State &
                                            !(0x2000 as libc::c_uint) |
                                            0x1000 as libc::c_uint)
        }
        /* State machine update: Check if an injected conversion is ongoing */
        if (*hadc).State & 0x1000 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            /* Reset ADC error code fields related to conversions on group regular */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x2 as libc::c_uint |
                                                   0x4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Reset ADC all error code fields */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              5 as libc::c_uint));
        /* Check if Multimode enabled */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2300
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut ADC_Common_TypeDef)).CCR &
               (0x1f as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* if no external trigger present enable software conversion of regular channels */
            if (*(*hadc).Instance).CR2 &
                   (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the selected ADC software conversion for regular group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     30 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        } else if (*hadc).Instance ==
                      (0x40000000 as
                           libc::c_uint).wrapping_add(0x10000 as
                                                          libc::c_uint).wrapping_add(0x2000
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut ADC_TypeDef &&
                      (*(*hadc).Instance).CR2 &
                          (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                          RESET as libc::c_int as libc::c_uint {
            /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
            /* Enable the selected ADC software conversion for regular group */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 30 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Disables ADC and stop conversion of regular channels.
  * 
  * @note   Caution: This function will stop also injected channels.  
  *
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  *
  * @retval HAL status.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Stop(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check if ADC is effectively disabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x100 as libc::c_uint |
                                              0x1000 as libc::c_uint) |
                                        0x1 as libc::c_uint)
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Poll for regular conversion complete
  * @note   ADC conversion flags EOS (end of sequence) and EOC (end of
  *         conversion) are cleared by this function.
  * @note   This function cannot be used in a particular setup: ADC configured 
  *         in DMA mode and polling for end of each conversion (ADC init
  *         parameter "EOCSelection" set to ADC_EOC_SINGLE_CONV).
  *         In this case, DMA resets the flag EOC and polling cannot be
  *         performed on each conversion. Nevertheless, polling can still 
  *         be performed on the complete sequence.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  Timeout: Timeout value in millisecond.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_PollForConversion(mut hadc:
                                                       *mut ADC_HandleTypeDef,
                                                   mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Verification that ADC configuration is compliant with polling for      */
  /* each conversion:                                                       */
  /* Particular case is ADC configured in DMA mode and ADC sequencer with   */
  /* several ranks and polling for end of each conversion.                  */
  /* For code simplicity sake, this particular case is generalized to       */
  /* ADC configured in DMA mode and polling for end of each conversion.     */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 10 as libc::c_uint
           != RESET as libc::c_int as libc::c_uint &&
           (*(*hadc).Instance).CR2 &
               (0x1 as libc::c_uint) << 8 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Update ADC state machine to error */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Process unlocked */
        (*hadc).Lock = HAL_UNLOCKED;
        return HAL_ERROR
    }
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Check End of conversion flag */
    while !((*(*hadc).Instance).SR &
                (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                (0x1 as libc::c_uint) << 1 as libc::c_uint) {
        /* Check if timeout is disabled (set to infinite wait) */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_int as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(tickstart) > Timeout {
                /* Update ADC state machine to timeout */
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x4 as libc::c_uint) as
                                                uint32_t as uint32_t);
                /* Process unlocked */
                (*hadc).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    /* Clear regular group conversion flag */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as *mut uint32_t,
                                !((0x1 as libc::c_uint) << 4 as libc::c_uint |
                                      (0x1 as libc::c_uint) <<
                                          1 as libc::c_uint));
    /* Update ADC state machine */
    ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x200 as libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Determine whether any further conversion upcoming on group regular       */
  /* by external trigger, continuous mode or scan sequence on going.          */
  /* Note: On STM32F7, there is no independent flag of end of sequence.       */
  /*       The test of scan sequence on going is done either with scan        */
  /*       sequence disabled or with end of conversion flag set to            */
  /*       of end of sequence.                                                */
    if (*(*hadc).Instance).CR2 & (0x3 as libc::c_uint) << 28 as libc::c_uint
           == RESET as libc::c_int as libc::c_uint &&
           (*hadc).Init.ContinuousConvMode ==
               DISABLE as libc::c_int as libc::c_uint &&
           ((*(*hadc).Instance).SQR1 &
                (0xf as libc::c_uint) << 20 as libc::c_uint ==
                RESET as libc::c_int as libc::c_uint ||
                (*(*hadc).Instance).CR2 &
                    (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint) {
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x100 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*hadc).State & 0x1000 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    /* Return ADC state */
    return HAL_OK;
}
/* *
  * @brief  Poll for conversion event
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  EventType: the ADC event type.
  *          This parameter can be one of the following values:
  *            @arg ADC_AWD_EVENT: ADC Analog watch Dog event.
  *            @arg ADC_OVR_EVENT: ADC Overrun event.
  * @param  Timeout: Timeout value in millisecond.   
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_PollForEvent(mut hadc:
                                                  *mut ADC_HandleTypeDef,
                                              mut EventType: uint32_t,
                                              mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get tick */
    tickstart = HAL_GetTick();
    /* Check selected event flag */
    while !((*(*hadc).Instance).SR & EventType == EventType) {
        /* Check for the Timeout */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_int as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(tickstart) > Timeout {
                /* Update ADC state machine to timeout */
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x4 as libc::c_uint) as
                                                uint32_t as uint32_t);
                /* Process unlocked */
                (*hadc).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    /* Analog watchdog (level out of window) event */
    if EventType == (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10000 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Clear ADC analog watchdog flag */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          0 as libc::c_uint))
    } else {
        /* Overrun event */
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x400 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_uint) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          5 as libc::c_uint))
    }
    /* Set ADC error code to overrun */
    /* Clear ADC overrun flag */
    /* Return ADC state */
    return HAL_OK;
}
/* *
  * @brief  Enables the interrupt and starts ADC conversion of regular channels.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Start_IT(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Enable the ADC peripheral */
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
        ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                    (3 as
                                         libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                     as
                                                                                                     libc::c_int
                                                                                                     as
                                                                                                     libc::c_uint)));
        while counter != 0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                   as
                                                                                   *const uint32_t).wrapping_sub(1))
        }
    }
    /* Start conversion if ADC is effectively enabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x1 as libc::c_uint |
                                              0x200 as libc::c_uint |
                                              0x400 as libc::c_uint) |
                                        0x100 as libc::c_uint);
        /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
        if (*(*hadc).Instance).CR1 &
               (0x1 as libc::c_uint) << 10 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (*hadc).State &
                                            !(0x2000 as libc::c_uint) |
                                            0x1000 as libc::c_uint)
        }
        /* State machine update: Check if an injected conversion is ongoing */
        if (*hadc).State & 0x1000 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            /* Reset ADC error code fields related to conversions on group regular */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x2 as libc::c_uint |
                                                   0x4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Reset ADC all error code fields */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              5 as libc::c_uint));
        /* Enable end of conversion interrupt for regular group */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              5 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  26 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Check if Multimode enabled */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2300
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut ADC_Common_TypeDef)).CCR &
               (0x1f as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* if no external trigger present enable software conversion of regular channels */
            if (*(*hadc).Instance).CR2 &
                   (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the selected ADC software conversion for regular group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     30 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        } else if (*hadc).Instance ==
                      (0x40000000 as
                           libc::c_uint).wrapping_add(0x10000 as
                                                          libc::c_uint).wrapping_add(0x2000
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut ADC_TypeDef &&
                      (*(*hadc).Instance).CR2 &
                          (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                          RESET as libc::c_int as libc::c_uint {
            /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
            /* Enable the selected ADC software conversion for regular group */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 30 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Disables the interrupt and stop ADC conversion of regular channels.
  * 
  * @note   Caution: This function will stop also injected channels.  
  *
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status.
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Stop_IT(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check if ADC is effectively disabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        /* Disable ADC end of conversion interrupt for regular group */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               5 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   26 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x100 as libc::c_uint |
                                              0x1000 as libc::c_uint) |
                                        0x1 as libc::c_uint)
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Handles ADC interrupt request  
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_IRQHandler(mut hadc:
                                                *mut ADC_HandleTypeDef) {
    let mut tmp1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp1 =
        ((*(*hadc).Instance).SR & (0x1 as libc::c_uint) << 1 as libc::c_uint
             == (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int as
            uint32_t;
    tmp2 =
        ((*(*hadc).Instance).CR1 & (0x1 as libc::c_uint) << 5 as libc::c_uint
             == (0x1 as libc::c_uint) << 5 as libc::c_uint) as libc::c_int as
            uint32_t;
    /* Check End of conversion flag for regular channels */
    if tmp1 != 0 && tmp2 != 0 {
        /* Update state machine on conversion status if not in error state */
        if (*hadc).State & 0x10 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x200 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F7, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
        if (*(*hadc).Instance).CR2 &
               (0x3 as libc::c_uint) << 28 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint &&
               (*hadc).Init.ContinuousConvMode ==
                   DISABLE as libc::c_int as libc::c_uint &&
               ((*(*hadc).Instance).SQR1 &
                    (0xf as libc::c_uint) << 20 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint ||
                    (*(*hadc).Instance).CR2 &
                        (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                        RESET as libc::c_int as libc::c_uint) {
            /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*hadc).State & 0x1000 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        /* Conversion complete callback */
        HAL_ADC_ConvCpltCallback(hadc);
        /* Clear regular group conversion flag */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          4 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              1 as libc::c_uint))
    }
    tmp1 =
        ((*(*hadc).Instance).SR & (0x1 as libc::c_uint) << 2 as libc::c_uint
             == (0x1 as libc::c_uint) << 2 as libc::c_uint) as libc::c_int as
            uint32_t;
    tmp2 =
        ((*(*hadc).Instance).CR1 & (0x1 as libc::c_uint) << 7 as libc::c_uint
             == (0x1 as libc::c_uint) << 7 as libc::c_uint) as libc::c_int as
            uint32_t;
    /* Check End of conversion flag for injected channels */
    if tmp1 != 0 && tmp2 != 0 {
        /* Update state machine on conversion status if not in error state */
        if (*hadc).State & 0x10 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x2000 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Determine whether any further conversion upcoming on group injected  */
    /* by external trigger, scan sequence on going or by automatic injected */
    /* conversion from group regular (same conditions as group regular      */
    /* interruption disabling above).                                       */
        if (*(*hadc).Instance).CR2 &
               (0x3 as libc::c_uint) << 20 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint &&
               ((*(*hadc).Instance).JSQR &
                    (0x3 as libc::c_uint) << 20 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint ||
                    (*(*hadc).Instance).CR2 &
                        (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                        RESET as libc::c_int as libc::c_uint) &&
               ((*(*hadc).Instance).CR1 &
                    (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint &&
                    ((*(*hadc).Instance).CR2 &
                         (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                         RESET as libc::c_int as libc::c_uint &&
                         (*hadc).Init.ContinuousConvMode ==
                             DISABLE as libc::c_int as libc::c_uint)) {
            /* Disable ADC end of single conversion interrupt on group injected */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   7 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x1000 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*hadc).State & 0x100 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        /* Conversion complete callback */
        HAL_ADCEx_InjectedConvCpltCallback(hadc);
        /* Clear injected group conversion flag */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          3 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              2 as libc::c_uint))
    }
    tmp1 =
        ((*(*hadc).Instance).SR & (0x1 as libc::c_uint) << 0 as libc::c_uint
             == (0x1 as libc::c_uint) << 0 as libc::c_uint) as libc::c_int as
            uint32_t;
    tmp2 =
        ((*(*hadc).Instance).CR1 & (0x1 as libc::c_uint) << 6 as libc::c_uint
             == (0x1 as libc::c_uint) << 6 as libc::c_uint) as libc::c_int as
            uint32_t;
    /* Check Analog watchdog flag */
    if tmp1 != 0 && tmp2 != 0 {
        if (*(*hadc).Instance).SR & (0x1 as libc::c_uint) << 0 as libc::c_uint
               == (0x1 as libc::c_uint) << 0 as libc::c_uint {
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x10000 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Level out of window callback */
            HAL_ADC_LevelOutOfWindowCallback(hadc);
            /* Clear the ADC analog watchdog flag */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                            *mut uint32_t,
                                        !((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint))
        }
    }
    tmp1 =
        ((*(*hadc).Instance).SR & (0x1 as libc::c_uint) << 5 as libc::c_uint
             == (0x1 as libc::c_uint) << 5 as libc::c_uint) as libc::c_int as
            uint32_t;
    tmp2 =
        ((*(*hadc).Instance).CR1 & (0x1 as libc::c_uint) << 26 as libc::c_uint
             == (0x1 as libc::c_uint) << 26 as libc::c_uint) as libc::c_int as
            uint32_t;
    /* Check Overrun flag */
    if tmp1 != 0 && tmp2 != 0 {
        /* Note: On STM32F7, ADC overrun can be set through other parameters    */
    /*       refer to description of parameter "EOCSelection" for more      */
    /*       details.                                                       */
        /* Set ADC error code to overrun */
        ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Clear ADC overrun flag */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          5 as libc::c_uint));
        /* Error callback */
        HAL_ADC_ErrorCallback(hadc);
        /* Clear the Overrun flag */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          5 as libc::c_uint))
    };
}
/* *
  * @brief  Enables ADC DMA request after last transfer (Single-ADC mode) and enables ADC peripheral  
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  pData: The destination Buffer address.
  * @param  Length: The length of data to be transferred from ADC peripheral to memory.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Start_DMA(mut hadc: *mut ADC_HandleTypeDef,
                                           mut pData: *mut uint32_t,
                                           mut Length: uint32_t)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Enable the ADC peripheral */
  /* Check if ADC peripheral is disabled in order to enable it and wait during 
     Tstab time the ADC's stabilization */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Delay for ADC stabilization time */
    /* Compute number of CPU cycles to wait for */
        ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                    (3 as
                                         libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                     as
                                                                                                     libc::c_int
                                                                                                     as
                                                                                                     libc::c_uint)));
        while counter != 0 as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                   as
                                                                                   *const uint32_t).wrapping_sub(1))
        }
    }
    /* Start conversion if ADC is effectively enabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Set ADC state                                                          */
    /* - Clear state bitfield related to regular group conversion results     */
    /* - Set state bitfield related to regular group operation                */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x1 as libc::c_uint |
                                              0x200 as libc::c_uint |
                                              0x400 as libc::c_uint) |
                                        0x100 as libc::c_uint);
        /* If conversions on group regular are also triggering group injected,    */
    /* update ADC state.                                                      */
        if (*(*hadc).Instance).CR1 &
               (0x1 as libc::c_uint) << 10 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (*hadc).State &
                                            !(0x2000 as libc::c_uint) |
                                            0x1000 as libc::c_uint)
        }
        /* State machine update: Check if an injected conversion is ongoing */
        if (*hadc).State & 0x1000 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            /* Reset ADC error code fields related to conversions on group regular */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x2 as libc::c_uint |
                                                   0x4 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Reset ADC all error code fields */
            ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Process unlocked */
    /* Unlock before starting ADC conversions: in case of potential           */
    /* interruption, to let the process to ADC IRQ Handler.                   */
        (*hadc).Lock = HAL_UNLOCKED;
        /* Set the DMA transfer complete callback */
        (*(*hadc).DMA_Handle).XferCpltCallback =
            Some(ADC_DMAConvCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA half transfer complete callback */
        (*(*hadc).DMA_Handle).XferHalfCpltCallback =
            Some(ADC_DMAHalfConvCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hadc).DMA_Handle).XferErrorCallback =
            Some(ADC_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
    /* start (in case of SW start):                                           */
        /* Clear regular group conversion flag and overrun flag */
    /* (To ensure of no unknown state from potential previous ADC operations) */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SR as
                                        *mut uint32_t,
                                    !((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              5 as libc::c_uint));
        /* Enable ADC overrun interrupt */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             26 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Enable ADC DMA mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Start the DMA channel */
        HAL_DMA_Start_IT((*hadc).DMA_Handle,
                         &mut (*(*hadc).Instance).DR as *mut uint32_t as
                             uint32_t, pData as uint32_t, Length);
        /* Check if Multimode enabled */
        if (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x10000 as
                                                  libc::c_uint).wrapping_add(0x2300
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut ADC_Common_TypeDef)).CCR &
               (0x1f as libc::c_uint) << 0 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint {
            /* if no external trigger present enable software conversion of regular channels */
            if (*(*hadc).Instance).CR2 &
                   (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                /* Enable the selected ADC software conversion for regular group */
                ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     30 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        } else if (*hadc).Instance ==
                      (0x40000000 as
                           libc::c_uint).wrapping_add(0x10000 as
                                                          libc::c_uint).wrapping_add(0x2000
                                                                                         as
                                                                                         libc::c_uint)
                          as *mut ADC_TypeDef &&
                      (*(*hadc).Instance).CR2 &
                          (0x3 as libc::c_uint) << 28 as libc::c_uint ==
                          RESET as libc::c_int as libc::c_uint {
            /* if instance of handle correspond to ADC1 and  no external trigger present enable software conversion of regular channels */
            /* Enable the selected ADC software conversion for regular group */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 30 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Disables ADC DMA (Single-ADC mode) and disables ADC peripheral    
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_Stop_DMA(mut hadc: *mut ADC_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp_hal_status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* Stop potential conversion on going, on regular and injected groups */
  /* Disable ADC peripheral */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check if ADC is effectively disabled */
    if (*(*hadc).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint {
        /* Disable the selected ADC DMA mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               8 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable the DMA channel (in case of DMA in circular mode or stop while */
    /* DMA transfer is on going)                                              */
        tmp_hal_status = HAL_DMA_Abort((*hadc).DMA_Handle);
        /* Disable ADC overrun interrupt */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               26 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set ADC state */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (*hadc).State &
                                        !(0x100 as libc::c_uint |
                                              0x1000 as libc::c_uint) |
                                        0x1 as libc::c_uint)
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return tmp_hal_status;
}
/* *
  * @brief  Gets the converted value from data register of regular channel.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval Converted value
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_GetValue(mut hadc: *mut ADC_HandleTypeDef)
 -> uint32_t {
    /* Return the selected ADC converted value */
    return (*(*hadc).Instance).DR;
}
/* *
  * @brief  Regular conversion complete callback in non blocking mode 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_ConvCpltCallback(mut hadc:
                                                      *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Regular conversion half DMA transfer callback in non blocking mode 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_ConvHalfCpltCallback(mut hadc:
                                                          *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvHalfCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Analog watchdog callback in non blocking mode 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_LevelOutOfWindowCallback(mut hadc:
                                                              *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_LevelOoutOfWindowCallback could be implemented in the user file
   */
}
/* *
  * @brief  Error ADC callback.
  * @note   In case of error due to overrun when using ADC with DMA transfer 
  *         (HAL ADC handle paramater "ErrorCode" to state "HAL_ADC_ERROR_OVR"):
  *         - Reinitialize the DMA using function "HAL_ADC_Stop_DMA()".
  *         - If needed, restart a new ADC conversion using function
  *           "HAL_ADC_Start_DMA()"
  *           (this function is also clearing overrun flag)
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_ErrorCallback(mut hadc:
                                                   *mut ADC_HandleTypeDef) {
    /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ErrorCallback could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup ADC_Exported_Functions_Group3 Peripheral Control functions
 *  @brief   	Peripheral Control functions 
 *
@verbatim   
 ===============================================================================
             ##### Peripheral Control functions #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Configure regular channels. 
      (+) Configure injected channels.
      (+) Configure multimode.
      (+) Configure the analog watch dog.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Configures for the selected ADC regular channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  sConfig: ADC configuration structure. 
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_ConfigChannel(mut hadc:
                                                   *mut ADC_HandleTypeDef,
                                               mut sConfig:
                                                   *mut ADC_ChannelConfTypeDef)
 -> HAL_StatusTypeDef {
    let mut counter: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
    if (*sConfig).Channel >
           (0x8 as libc::c_uint) << 0 as libc::c_uint |
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
        /* Clear the old sample time */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x7 as libc::c_uint) <<
                                                0 as libc::c_uint) <<
                                               (3 as libc::c_int as
                                                    libc::c_uint).wrapping_mul(((*sConfig).Channel
                                                                                    as
                                                                                    uint16_t
                                                                                    as
                                                                                    uint32_t).wrapping_sub(10
                                                                                                               as
                                                                                                               libc::c_int
                                                                                                               as
                                                                                                               libc::c_uint))))
                                        as uint32_t as uint32_t);
        if (*sConfig).Channel ==
               (0x10 as libc::c_uint) << 0 as libc::c_uint |
                   (0x2 as libc::c_uint) << 0 as libc::c_uint |
                   0x10000000 as libc::c_uint {
            /* Set the new sample time */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).SamplingTime <<
                                                 (3 as libc::c_int as
                                                      libc::c_uint).wrapping_mul((((0x10
                                                                                        as
                                                                                        libc::c_uint)
                                                                                       <<
                                                                                       0
                                                                                           as
                                                                                           libc::c_uint
                                                                                       |
                                                                                       (0x2
                                                                                            as
                                                                                            libc::c_uint)
                                                                                           <<
                                                                                           0
                                                                                               as
                                                                                               libc::c_uint)
                                                                                      as
                                                                                      uint16_t
                                                                                      as
                                                                                      uint32_t).wrapping_sub(10
                                                                                                                 as
                                                                                                                 libc::c_int
                                                                                                                 as
                                                                                                                 libc::c_uint)))
                                            as uint32_t as uint32_t)
        } else {
            /* Set the new sample time */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*sConfig).SamplingTime <<
                                                 (3 as libc::c_int as
                                                      libc::c_uint).wrapping_mul(((*sConfig).Channel
                                                                                      as
                                                                                      uint16_t
                                                                                      as
                                                                                      uint32_t).wrapping_sub(10
                                                                                                                 as
                                                                                                                 libc::c_int
                                                                                                                 as
                                                                                                                 libc::c_uint)))
                                            as uint32_t as uint32_t)
        }
    } else {
        /* ADC_Channel include in ADC_Channel_[0..9] */
        /* Clear the old sample time */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(((0x7 as libc::c_uint) <<
                                                0 as libc::c_uint) <<
                                               (3 as libc::c_int as
                                                    libc::c_uint).wrapping_mul((*sConfig).Channel
                                                                                   as
                                                                                   uint16_t
                                                                                   as
                                                                                   uint32_t)))
                                        as uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SMPR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*sConfig).SamplingTime <<
                                             (3 as libc::c_int as
                                                  libc::c_uint).wrapping_mul((*sConfig).Channel
                                                                                 as
                                                                                 uint16_t
                                                                                 as
                                                                                 uint32_t))
                                        as uint32_t as uint32_t)
    }
    /* Set the new sample time */
    /* For Rank 1 to 6 */
    if (*sConfig).Rank < 7 as libc::c_int as libc::c_uint {
        /* Clear the old SQx bits for the selected rank */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((((0x1f as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                                uint16_t as uint32_t) <<
                                               (5 as libc::c_int as
                                                    libc::c_uint).wrapping_mul((*sConfig).Rank.wrapping_sub(1
                                                                                                                as
                                                                                                                libc::c_int
                                                                                                                as
                                                                                                                libc::c_uint))))
                                        as uint32_t as uint32_t);
        /* Set the SQx bits for the selected rank */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR3 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((*sConfig).Channel as uint16_t as
                                              uint32_t) <<
                                             (5 as libc::c_int as
                                                  libc::c_uint).wrapping_mul((*sConfig).Rank.wrapping_sub(1
                                                                                                              as
                                                                                                              libc::c_int
                                                                                                              as
                                                                                                              libc::c_uint)))
                                        as uint32_t as uint32_t)
    } else if (*sConfig).Rank < 13 as libc::c_int as libc::c_uint {
        /* For Rank 7 to 12 */
        /* Clear the old SQx bits for the selected rank */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((((0x1f as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                                uint16_t as uint32_t) <<
                                               (5 as libc::c_int as
                                                    libc::c_uint).wrapping_mul((*sConfig).Rank.wrapping_sub(7
                                                                                                                as
                                                                                                                libc::c_int
                                                                                                                as
                                                                                                                libc::c_uint))))
                                        as uint32_t as uint32_t);
        /* Set the SQx bits for the selected rank */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((*sConfig).Channel as uint16_t as
                                              uint32_t) <<
                                             (5 as libc::c_int as
                                                  libc::c_uint).wrapping_mul((*sConfig).Rank.wrapping_sub(7
                                                                                                              as
                                                                                                              libc::c_int
                                                                                                              as
                                                                                                              libc::c_uint)))
                                        as uint32_t as uint32_t)
    } else {
        /* For Rank 13 to 16 */
        /* Clear the old SQx bits for the selected rank */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((((0x1f as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                                uint16_t as uint32_t) <<
                                               (5 as libc::c_int as
                                                    libc::c_uint).wrapping_mul((*sConfig).Rank.wrapping_sub(13
                                                                                                                as
                                                                                                                libc::c_int
                                                                                                                as
                                                                                                                libc::c_uint))))
                                        as uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((*sConfig).Channel as uint16_t as
                                              uint32_t) <<
                                             (5 as libc::c_int as
                                                  libc::c_uint).wrapping_mul((*sConfig).Rank.wrapping_sub(13
                                                                                                              as
                                                                                                              libc::c_int
                                                                                                              as
                                                                                                              libc::c_uint)))
                                        as uint32_t as uint32_t)
    }
    /* Set the SQx bits for the selected rank */
    /* if ADC1 Channel_18 is selected enable VBAT Channel */
    if (*hadc).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_TypeDef &&
           (*sConfig).Channel ==
               (0x10 as libc::c_uint) << 0 as libc::c_uint |
                   (0x2 as libc::c_uint) << 0 as libc::c_uint {
        /* Enable the VBAT channel*/
        let ref mut fresh0 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2300
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             22 as libc::c_uint) as uint32_t
                                        as uint32_t)
    }
    /* if ADC1 Channel_18 or Channel_17 is selected enable TSVREFE Channel(Temperature sensor and VREFINT) */
    if (*hadc).Instance ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_TypeDef &&
           ((*sConfig).Channel ==
                (0x10 as libc::c_uint) << 0 as libc::c_uint |
                    (0x2 as libc::c_uint) << 0 as libc::c_uint |
                    0x10000000 as libc::c_uint ||
                (*sConfig).Channel ==
                    (0x10 as libc::c_uint) << 0 as libc::c_uint |
                        (0x1 as libc::c_uint) << 0 as libc::c_uint) {
        /* Enable the TSVREFE channel*/
        let ref mut fresh1 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x2300
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             23 as libc::c_uint) as uint32_t
                                        as uint32_t);
        if (*sConfig).Channel ==
               (0x10 as libc::c_uint) << 0 as libc::c_uint |
                   (0x2 as libc::c_uint) << 0 as libc::c_uint |
                   0x10000000 as libc::c_uint {
            /* Delay for temperature sensor stabilization time */
      /* Compute number of CPU cycles to wait for */
            ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                        (10 as
                                             libc::c_uint).wrapping_mul(SystemCoreClock.wrapping_div(1000000
                                                                                                         as
                                                                                                         libc::c_int
                                                                                                         as
                                                                                                         libc::c_uint)));
            while counter != 0 as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut counter as *mut uint32_t,
                                            ::core::ptr::read_volatile::<uint32_t>(&counter
                                                                                       as
                                                                                       *const uint32_t).wrapping_sub(1))
            }
        }
    }
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @brief  Configures the analog watchdog.
  * @note   Analog watchdog thresholds can be modified while ADC conversion
  *         is on going.
  *         In this case, some constraints must be taken into account:
  *         the programmed threshold values are effective from the next
  *         ADC EOC (end of unitary conversion).
  *         Considering that registers write delay may happen due to
  *         bus activity, this might cause an uncertainty on the
  *         effective timing of the new programmed threshold values.
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @param  AnalogWDGConfig : pointer to an ADC_AnalogWDGConfTypeDef structure 
  *         that contains the configuration information of ADC analog watchdog.
  * @retval HAL status	  
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_AnalogWDGConfig(mut hadc:
                                                     *mut ADC_HandleTypeDef,
                                                 mut AnalogWDGConfig:
                                                     *mut ADC_AnalogWDGConfTypeDef)
 -> HAL_StatusTypeDef {
    /* USE_FULL_ASSERT  */
    /* Check the parameters */
    /* USE_FULL_ASSERT  */
    /* Process locked */
    if (*hadc).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hadc).Lock = HAL_LOCKED }
    if (*AnalogWDGConfig).ITMode == ENABLE as libc::c_int as libc::c_uint {
        /* Enable the ADC Analog watchdog interrupt */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             6 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the ADC Analog watchdog interrupt */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Clear AWDEN, JAWDEN and AWDSGL bits */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           9 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               22 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               23 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Set the analog watchdog enable mode */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*AnalogWDGConfig).WatchdogMode) as
                                    uint32_t as uint32_t);
    /* Set the high threshold */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).HTR as *mut uint32_t,
                                (*AnalogWDGConfig).HighThreshold);
    /* Set the low threshold */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).LTR as *mut uint32_t,
                                (*AnalogWDGConfig).LowThreshold);
    /* Clear the Analog watchdog channel select bits */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1f as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Set the Analog watchdog channel */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*AnalogWDGConfig).Channel as uint16_t as
                                         uint32_t) as uint32_t as uint32_t);
    /* Process unlocked */
    (*hadc).Lock = HAL_UNLOCKED;
    /* Return function status */
    return HAL_OK;
}
/* *
  * @}
  */
/* * @defgroup ADC_Exported_Functions_Group4 ADC Peripheral State functions
 *  @brief   ADC Peripheral State functions 
 *
@verbatim   
 ===============================================================================
            ##### Peripheral State and errors functions #####
 ===============================================================================  
    [..]
    This subsection provides functions allowing to
      (+) Check the ADC state
      (+) Check the ADC Error
         
@endverbatim
  * @{
  */
/* *
  * @brief  return the ADC state
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_GetState(mut hadc: *mut ADC_HandleTypeDef)
 -> uint32_t {
    /* Return ADC state */
    return (*hadc).State;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_adc.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of ADC HAL extension module.
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
/* * @addtogroup ADC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup ADC_Exported_Types ADC Exported Types
  * @{
  */
/* * 
  * @brief  Structure definition of ADC and regular group initialization 
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope entire ADC (affects regular and injected groups): ClockPrescaler, Resolution, ScanConvMode, DataAlign, ScanConvMode, EOCSelection, LowPowerAutoWait, LowPowerAutoPowerOff, ChannelsBank.
  *          - Scope regular group: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode, NbrOfDiscConversion, ExternalTrigConvEdge, ExternalTrigConv.
  * @note   The setting of these parameters with function HAL_ADC_Init() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all parameters except 'Resolution', 'ScanConvMode', 'DiscontinuousConvMode', 'NbrOfDiscConversion' : ADC enabled without conversion on going on regular group.
  *          - For parameters 'ExternalTrigConv' and 'ExternalTrigConvEdge': ADC enabled, even with conversion on going.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behaviour in case of intended action to update another parameter (which fullfills the ADC state condition) on the fly).
  */
/* !< Select ADC clock prescaler. The clock is common for 
                                       all the ADCs.
                                       This parameter can be a value of @ref ADC_ClockPrescaler */
/* !< Configures the ADC resolution.
                                       This parameter can be a value of @ref ADC_Resolution */
/* !< Specifies ADC data alignment to right (MSB on register bit 11 and LSB on register bit 0) (default setting)
                                       or to left (if regular group: MSB on register bit 15 and LSB on register bit 4, if injected group (MSB kept as signed value due to potential negative value after offset application): MSB on register bit 14 and LSB on register bit 3).
                                       This parameter can be a value of @ref ADC_Data_Align */
/* !< Configures the sequencer of regular and injected groups.
                                       This parameter can be associated to parameter 'DiscontinuousConvMode' to have main sequence subdivided in successive parts.
                                       If disabled: Conversion is performed in single mode (one channel converted, the one defined in rank 1).
                                                    Parameters 'NbrOfConversion' and 'InjectedNbrOfConversion' are discarded (equivalent to set to 1).
                                       If enabled:  Conversions are performed in sequence mode (multiple ranks defined by 'NbrOfConversion'/'InjectedNbrOfConversion' and each channel rank).
                                                    Scan direction is upward: from rank1 to rank 'n'.
                                       This parameter can be a value of @ref ADC_Scan_mode.
                                       This parameter can be set to ENABLE or DISABLE */
/* !< Specifies what EOC (End Of Conversion) flag is used for conversion by polling and interruption: end of conversion of each rank or complete sequence.
                                       This parameter can be a value of @ref ADC_EOCSelection.
                                       Note: For injected group, end of conversion (flag&IT) is raised only at the end of the sequence.
                                             Therefore, if end of conversion is set to end of each conversion, injected group should not be used with interruption (HAL_ADCEx_InjectedStart_IT)
                                             or polling (HAL_ADCEx_InjectedStart and HAL_ADCEx_InjectedPollForConversion). By the way, polling is still possible since driver will use an estimated timing for end of injected conversion.
                                       Note: If overrun feature is intended to be used, use ADC in mode 'interruption' (function HAL_ADC_Start_IT() ) with parameter EOCSelection set to end of each conversion or in mode 'transfer by DMA' (function HAL_ADC_Start_DMA()).
                                             If overrun feature is intended to be bypassed, use ADC in mode 'polling' or 'interruption' with parameter EOCSelection must be set to end of sequence */
/* !< Specifies whether the conversion is performed in single mode (one conversion) or continuous mode for regular group,
                                       after the selected trigger occurred (software start or external trigger).
                                       This parameter can be set to ENABLE or DISABLE. */
/* !< Specifies the number of ranks that will be converted within the regular group sequencer.
                                       To use regular group sequencer and convert several ranks, parameter 'ScanConvMode' must be enabled.
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 16. */
/* !< Specifies whether the conversions sequence of regular group is performed in Complete-sequence/Discontinuous-sequence (main sequence subdivided in successive parts).
                                       Discontinuous mode is used only if sequencer is enabled (parameter 'ScanConvMode'). If sequencer is disabled, this parameter is discarded.
                                       Discontinuous mode can be enabled only if continuous mode is disabled. If continuous mode is enabled, this parameter setting is discarded.
                                       This parameter can be set to ENABLE or DISABLE. */
/* !< Specifies the number of discontinuous conversions in which the  main sequence of regular group (parameter NbrOfConversion) will be subdivided.
                                       If parameter 'DiscontinuousConvMode' is disabled, this parameter is discarded.
                                       This parameter must be a number between Min_Data = 1 and Max_Data = 8. */
/* !< Selects the external event used to trigger the conversion start of regular group.
                                       If set to ADC_SOFTWARE_START, external triggers are disabled.
                                       If set to external trigger source, triggering is on event rising edge by default.
                                       This parameter can be a value of @ref ADC_External_trigger_Source_Regular */
/* !< Selects the external trigger edge of regular group.
                                       If trigger is set to ADC_SOFTWARE_START, this parameter is discarded.
                                       This parameter can be a value of @ref ADC_External_trigger_edge_Regular */
/* !< Specifies whether the DMA requests are performed in one shot mode (DMA transfer stop when number of conversions is reached)
                                       or in Continuous mode (DMA transfer unlimited, whatever number of conversions).
                                       Note: In continuous mode, DMA must be configured in circular mode. Otherwise an overrun will be triggered when DMA buffer maximum pointer is reached.
                                       Note: This parameter must be modified when no conversion is on going on both regular and injected groups (ADC disabled, or ADC enabled without continuous mode or external trigger that could launch a conversion).
                                       This parameter can be set to ENABLE or DISABLE. */
/* * 
  * @brief  Structure definition of ADC channel for regular group   
  * @note   The setting of these parameters with function HAL_ADC_ConfigChannel() is conditioned to ADC state.
  *         ADC can be either disabled or enabled without conversion on going on regular group.
  */
/* !< Specifies the channel to configure into ADC regular group.
                                        This parameter can be a value of @ref ADC_channels */
/* !< Specifies the rank in the regular group sequencer.
                                        This parameter must be a number between Min_Data = 1 and Max_Data = 16 
                                        This parameter can be a value of @ref ADC_regular_rank */
/* !< Sampling time value to be set for the selected channel.
                                        Unit: ADC clock cycles
                                        Conversion time is the addition of sampling time and processing time (12 ADC clock cycles at ADC resolution 12 bits, 11 cycles at 10 bits, 9 cycles at 8 bits, 7 cycles at 6 bits).
                                        This parameter can be a value of @ref ADC_sampling_times
                                        Caution: This parameter updates the parameter property of the channel, that can be used into regular and/or injected groups.
                                                 If this same channel has been previously configured in the other group (regular/injected), it will be updated to last setting.
                                        Note: In case of usage of internal measurement channels (VrefInt/Vbat/TempSensor),
                                              sampling time constraints must be respected (sampling time can be adjusted in function of ADC clock frequency and sampling time setting)
                                              Refer to device datasheet for timings values, parameters TS_vrefint, TS_temp (values rough order: 4us min). */
/* !< Reserved for future use, can be set to 0 */
/* * 
  * @brief ADC Configuration multi-mode structure definition  
  */
/* !< Configures the ADC analog watchdog mode.
                                   This parameter can be a value of @ref ADC_analog_watchdog_selection */
/* !< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a 12-bit value. */
/* !< Configures the ADC analog watchdog High threshold value.
                                   This parameter must be a 12-bit value. */
/* !< Configures ADC channel for the analog watchdog. 
                                   This parameter has an effect only if watchdog mode is configured on single channel 
                                   This parameter can be a value of @ref ADC_channels */
/* !< Specifies whether the analog watchdog is configured
                                   is interrupt mode or in polling mode.
                                   This parameter can be set to ENABLE or DISABLE */
/* !< Reserved for future use, can be set to 0 */
/* * 
  * @brief  HAL ADC state machine: ADC states definition (bitfields)
  */ 
/* States of ADC global scope */
/* !< ADC not yet initialized or disabled */
/* !< ADC peripheral ready for use */
/* !< ADC is busy to internal process (initialization, calibration) */
/* !< TimeOut occurrence */
/* States of ADC errors */
/* !< Internal error occurrence */
/* !< Configuration error occurrence */
/* !< DMA error occurrence */
/* States of ADC group regular */
/* !< A conversion on group regular is ongoing or can occur (either by continuous mode,
                                                                       external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
/* !< Conversion data available on group regular */
/* !< Overrun occurrence */
/* States of ADC group injected */
/* !< A conversion on group injected is ongoing or can occur (either by auto-injection mode,
                                                                       external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available)) */
/* !< Conversion data available on group injected */
/* States of ADC analog watchdogs */
/* !< Out-of-window occurrence of analog watchdog 1 */
/* !< Not available on STM32F7 device: Out-of-window occurrence of analog watchdog 2 */
/* !< Not available on STM32F7 device: Out-of-window occurrence of analog watchdog 3 */
/* States of ADC multi-mode */
/* !< Not available on STM32F7 device: ADC in multimode slave state, controlled by another ADC master ( */
/* * 
  * @brief  ADC handle Structure definition
  */
/* !< Register base address */
/* !< ADC required parameters */
/* !< ADC number of current conversion rank */
/* !< Pointer DMA Handler */
/* !< ADC locking object */
/* !< ADC communication state */
/* !< ADC Error code */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup ADC_Exported_Constants ADC Exported Constants
  * @{
  */
/* * @defgroup ADC_Error_Code ADC Error Code
  * @{
  */
/* !< No error                                              */
/* !< ADC IP internal error: if problem of clocking, 
                                                          enable/disable, erroneous state                       */
/* !< Overrun error                                         */
/* !< DMA transfer error                                    */
/* *
  * @}
  */
/* * @defgroup ADC_ClockPrescaler  ADC Clock Prescaler
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_delay_between_2_sampling_phases ADC Delay Between 2 Sampling Phases
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_Resolution ADC Resolution
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_External_trigger_edge_Regular ADC External Trigger Edge Regular
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_External_trigger_Source_Regular ADC External Trigger Source Regular
  * @{
  */
/* Note: Parameter ADC_SOFTWARE_START is a software parameter used for        */
/*       compatibility with other STM32 devices.                              */
/* *
  * @}
  */
/* * @defgroup ADC_Data_Align ADC Data Align
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_Scan_mode ADC sequencer scan mode
  * @{
  */
/* !< Scan mode disabled */
/* !< Scan mode enabled  */
/* *
  * @}
  */
/* * @defgroup ADC_regular_rank ADC group regular sequencer rank
  * @{
  */
/* !< ADC regular conversion rank 1  */
/* !< ADC regular conversion rank 2  */
/* !< ADC regular conversion rank 3  */
/* !< ADC regular conversion rank 4  */
/* !< ADC regular conversion rank 5  */
/* !< ADC regular conversion rank 6  */
/* !< ADC regular conversion rank 7  */
/* !< ADC regular conversion rank 8  */
/* !< ADC regular conversion rank 9  */
/* !< ADC regular conversion rank 10 */
/* !< ADC regular conversion rank 11 */
/* !< ADC regular conversion rank 12 */
/* !< ADC regular conversion rank 13 */
/* !< ADC regular conversion rank 14 */
/* !< ADC regular conversion rank 15 */
/* !< ADC regular conversion rank 16 */
/* *
  * @}
  */
/* * @defgroup ADC_channels ADC Common Channels
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_sampling_times ADC Sampling Times
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_EOCSelection ADC EOC Selection
  * @{
  */
/* !< reserved for future use */
/* *
  * @}
  */
/* * @defgroup ADC_Event_type ADC Event Type
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_analog_watchdog_selection ADC Analog Watchdog Selection
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_interrupts_definition ADC Interrupts Definition
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_flags_definition ADC Flags Definition
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_channels_type ADC Channels Type
  * @{
  */
/* !< reserved for future use */
/* !< reserved for future use */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup ADC_Exported_Macros ADC Exported Macros
  * @{
  */
/* * @brief Reset ADC handle state
  * @param  __HANDLE__: ADC handle
  * @retval None
  */
/* *
  * @brief  Enable the ADC peripheral.
  * @param  __HANDLE__: ADC handle
  * @retval None
  */
/* *
  * @brief  Disable the ADC peripheral.
  * @param  __HANDLE__: ADC handle
  * @retval None
  */
/* *
  * @brief  Enable the ADC end of conversion interrupt.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __INTERRUPT__: ADC Interrupt.
  * @retval None
  */
/* *
  * @brief  Disable the ADC end of conversion interrupt.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __INTERRUPT__: ADC interrupt.
  * @retval None
  */
/* * @brief  Check if the specified ADC interrupt source is enabled or disabled.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __INTERRUPT__: specifies the ADC interrupt source to check.
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
/* *
  * @brief  Clear the ADC's pending flags.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __FLAG__: ADC flag.
  * @retval None
  */
/* *
  * @brief  Get the selected ADC's flag status.
  * @param  __HANDLE__: specifies the ADC Handle.
  * @param  __FLAG__: ADC flag.
  * @retval None
  */
/* *
  * @}
  */
/* Include ADC HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup ADC_Exported_Functions
  * @{
  */
/* * @addtogroup ADC_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions ***********************************/
/* *
  * @}
  */
/* * @addtogroup ADC_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions ******************************************************/
/* *
  * @}
  */
/* * @addtogroup ADC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions *************************************************/
/* *
  * @}
  */
/* * @addtogroup ADC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State functions ***************************************************/
/* *
  * @brief  Return the ADC error code
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval ADC Error Code
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_ADC_GetError(mut hadc: *mut ADC_HandleTypeDef)
 -> uint32_t {
    return (*hadc).ErrorCode;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_adc.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Analog to Digital Convertor (ADC) peripheral:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + State and errors functions
  *         
  @verbatim
  ==============================================================================
                    ##### ADC Peripheral features #####
  ==============================================================================
  [..] 
  (#) 12-bit, 10-bit, 8-bit or 6-bit configurable resolution.
  (#) Interrupt generation at the end of conversion, end of injected conversion,  
      and in case of analog watchdog or overrun events
  (#) Single and continuous conversion modes.
  (#) Scan mode for automatic conversion of channel 0 to channel x.
  (#) Data alignment with in-built data coherency.
  (#) Channel-wise programmable sampling time.
  (#) External trigger option with configurable polarity for both regular and 
      injected conversion.
  (#) Dual/Triple mode (on devices with 2 ADCs or more).
  (#) Configurable DMA data storage in Dual/Triple ADC mode. 
  (#) Configurable delay between conversions in Dual/Triple interleaved mode.
  (#) ADC conversion type (refer to the datasheets).
  (#) ADC supply requirements: 2.4 V to 3.6 V at full speed and down to 1.8 V at 
      slower speed.
  (#) ADC input range: VREF(minus) = VIN = VREF(plus).
  (#) DMA request generation during regular channel conversion.


                     ##### How to use this driver #####
  ==============================================================================
  [..]
  (#)Initialize the ADC low level resources by implementing the HAL_ADC_MspInit():
       (##) Enable the ADC interface clock using __HAL_RCC_ADC_CLK_ENABLE()
       (##) ADC pins configuration
             (+++) Enable the clock for the ADC GPIOs using the following function:
                   __HAL_RCC_GPIOx_CLK_ENABLE()  
             (+++) Configure these ADC pins in analog mode using HAL_GPIO_Init() 
       (##) In case of using interrupts (e.g. HAL_ADC_Start_IT())
             (+++) Configure the ADC interrupt priority using HAL_NVIC_SetPriority()
             (+++) Enable the ADC IRQ handler using HAL_NVIC_EnableIRQ()
             (+++) In ADC IRQ handler, call HAL_ADC_IRQHandler()
       (##) In case of using DMA to control data transfer (e.g. HAL_ADC_Start_DMA())
             (+++) Enable the DMAx interface clock using __HAL_RCC_DMAx_CLK_ENABLE()
             (+++) Configure and enable two DMA streams stream for managing data
                 transfer from peripheral to memory (output stream)
             (+++) Associate the initialized DMA handle to the CRYP DMA handle
                 using  __HAL_LINKDMA()
             (+++) Configure the priority and enable the NVIC for the transfer complete
                 interrupt on the two DMA Streams. The output stream should have higher
                 priority than the input stream.
                       
    *** Configuration of ADC, groups regular/injected, channels parameters ***
  ==============================================================================
  [..]
  (#) Configure the ADC parameters (resolution, data alignment, ...)
      and regular group parameters (conversion trigger, sequencer, ...)
      using function HAL_ADC_Init().

  (#) Configure the channels for regular group parameters (channel number, 
      channel rank into sequencer, ..., into regular group)
      using function HAL_ADC_ConfigChannel().

  (#) Optionally, configure the injected group parameters (conversion trigger, 
      sequencer, ..., of injected group)
      and the channels for injected group parameters (channel number, 
      channel rank into sequencer, ..., into injected group)
      using function HAL_ADCEx_InjectedConfigChannel().

  (#) Optionally, configure the analog watchdog parameters (channels
      monitored, thresholds, ...) using function HAL_ADC_AnalogWDGConfig().

  (#) Optionally, for devices with several ADC instances: configure the 
      multimode parameters using function HAL_ADCEx_MultiModeConfigChannel().

                       *** Execution of ADC conversions ***
  ==============================================================================
  [..]  
  (#) ADC driver can be used among three modes: polling, interruption,
      transfer by DMA.    

     *** Polling mode IO operation ***
     =================================
     [..]    
       (+) Start the ADC peripheral using HAL_ADC_Start() 
       (+) Wait for end of conversion using HAL_ADC_PollForConversion(), at this stage
           user can specify the value of timeout according to his end application      
       (+) To read the ADC converted values, use the HAL_ADC_GetValue() function.
       (+) Stop the ADC peripheral using HAL_ADC_Stop()
       
     *** Interrupt mode IO operation ***    
     ===================================
     [..]    
       (+) Start the ADC peripheral using HAL_ADC_Start_IT() 
       (+) Use HAL_ADC_IRQHandler() called under ADC_IRQHandler() Interrupt subroutine
       (+) At ADC end of conversion HAL_ADC_ConvCpltCallback() function is executed and user can 
           add his own code by customization of function pointer HAL_ADC_ConvCpltCallback 
       (+) In case of ADC Error, HAL_ADC_ErrorCallback() function is executed and user can 
           add his own code by customization of function pointer HAL_ADC_ErrorCallback
       (+) Stop the ADC peripheral using HAL_ADC_Stop_IT()     

     *** DMA mode IO operation ***    
     ==============================
     [..]    
       (+) Start the ADC peripheral using HAL_ADC_Start_DMA(), at this stage the user specify the length 
           of data to be transferred at each end of conversion 
       (+) At The end of data transfer by HAL_ADC_ConvCpltCallback() function is executed and user can 
           add his own code by customization of function pointer HAL_ADC_ConvCpltCallback 
       (+) In case of transfer Error, HAL_ADC_ErrorCallback() function is executed and user can 
           add his own code by customization of function pointer HAL_ADC_ErrorCallback
       (+) Stop the ADC peripheral using HAL_ADC_Stop_DMA()
                    
     *** ADC HAL driver macros list ***
     ============================================= 
     [..]
       Below the list of most used macros in ADC HAL driver.
       
      (+) __HAL_ADC_ENABLE : Enable the ADC peripheral
      (+) __HAL_ADC_DISABLE : Disable the ADC peripheral
      (+) __HAL_ADC_ENABLE_IT: Enable the ADC end of conversion interrupt
      (+) __HAL_ADC_DISABLE_IT: Disable the ADC end of conversion interrupt
      (+) __HAL_ADC_GET_IT_SOURCE: Check if the specified ADC interrupt source is enabled or disabled
      (+) __HAL_ADC_CLEAR_FLAG: Clear the ADC's pending flags
      (+) __HAL_ADC_GET_FLAG: Get the selected ADC's flag status
      (+) ADC_GET_RESOLUTION: Return resolution bits in CR1 register 
      
     [..] 
       (@) You can refer to the ADC HAL driver header file for more useful macros 

                      *** Deinitialization of ADC ***
  ==============================================================================
  [..]
  (#) Disable the ADC interface
     (++) ADC clock can be hard reset and disabled at RCC top level.
     (++) Hard reset of ADC peripherals
          using macro __HAL_RCC_ADC_FORCE_RESET(), __HAL_RCC_ADC_RELEASE_RESET().
     (++) ADC clock disable using the equivalent macro/functions as configuration step.
               (+++) Example:
                   Into HAL_ADC_MspDeInit() (recommended code location) or with
                   other device clock parameters configuration:
               (+++) HAL_RCC_GetOscConfig(&RCC_OscInitStructure);
               (+++) RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI;
               (+++) RCC_OscInitStructure.HSIState = RCC_HSI_OFF; (if not used for system clock)
               (+++) HAL_RCC_OscConfig(&RCC_OscInitStructure);

  (#) ADC pins configuration
     (++) Disable the clock for the ADC GPIOs using macro __HAL_RCC_GPIOx_CLK_DISABLE()

  (#) Optionally, in case of usage of ADC with interruptions:
     (++) Disable the NVIC for ADC using function HAL_NVIC_DisableIRQ(ADCx_IRQn)

  (#) Optionally, in case of usage of DMA:
        (++) Deinitialize the DMA using function HAL_DMA_DeInit().
        (++) Disable the NVIC for DMA using function HAL_NVIC_DisableIRQ(DMAx_Channelx_IRQn)   

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
/* * @defgroup ADC ADC
  * @brief ADC driver modules
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @addtogroup ADC_Private_Functions
  * @{
  */
/* Private function prototypes -----------------------------------------------*/
/* *
  * @}
  */
/* *
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/* * @defgroup ADC_Private_Functions ADC Private Functions
  * @{
  */
/* *
  * @brief  Initializes the ADCx peripheral according to the specified parameters 
  *         in the ADC_InitStruct without initializing the ADC MSP.       
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.  
  * @retval None
  */
unsafe extern "C" fn ADC_Init(mut hadc: *mut ADC_HandleTypeDef) {
    /* Set ADC parameters */
  /* Set the ADC clock prescaler */
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x2300
                                                                              as
                                                                              libc::c_uint)
               as *mut ADC_Common_TypeDef)).CCR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.ClockPrescaler) as uint32_t
                                    as uint32_t);
    /* Set ADC scan mode */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.ScanConvMode <<
                                         8 as libc::c_int) as uint32_t as
                                    uint32_t);
    /* Set ADC resolution */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3 as libc::c_uint) <<
                                           24 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.Resolution) as uint32_t as
                                    uint32_t);
    /* Set ADC data alignment */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           11 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | (*hadc).Init.DataAlign)
                                    as uint32_t as uint32_t);
    /* Enable external trigger if trigger selection is different of software  */
  /* start.                                                                 */
  /* Note: This configuration keeps the hardware feature of parameter       */
  /*       ExternalTrigConvEdge "trigger edge none" equivalent to           */
  /*       software start.                                                  */
    if (*hadc).Init.ExternalTrigConv !=
           ((0xf as libc::c_uint) <<
                24 as
                    libc::c_uint).wrapping_add(1 as libc::c_int as
                                                   libc::c_uint) {
        /* Select external trigger to start conversion */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xf as libc::c_uint) <<
                                               24 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*hadc).Init.ExternalTrigConv) as
                                        uint32_t as uint32_t);
        /* Select external trigger polarity */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               28 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*hadc).Init.ExternalTrigConvEdge) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the external trigger */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0xf as libc::c_uint) <<
                                               24 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3 as libc::c_uint) <<
                                               28 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    /* Enable or disable ADC continuous conversion mode */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.ContinuousConvMode <<
                                         1 as libc::c_int) as uint32_t as
                                    uint32_t);
    if (*hadc).Init.DiscontinuousConvMode !=
           DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC regular discontinuous mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             11 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Set the number of channels to be converted in discontinuous mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x7 as libc::c_uint) <<
                                               13 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*hadc).Init.NbrOfDiscConversion.wrapping_sub(1
                                                                                           as
                                                                                           libc::c_int
                                                                                           as
                                                                                           libc::c_uint)
                                             <<
                                             __RBIT((0x7 as libc::c_uint) <<
                                                        13 as
                                                            libc::c_uint).leading_zeros()
                                                 as i32 as uint8_t as
                                                 libc::c_int) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected ADC regular discontinuous mode */
        ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               11 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    }
    /* Set ADC number of conversion */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0xf as libc::c_uint) <<
                                           20 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).SQR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).SQR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.NbrOfConversion.wrapping_sub(1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint8_t
                                                                                   as
                                                                                   libc::c_uint)
                                         << 20 as libc::c_int) as uint32_t as
                                    uint32_t);
    /* Enable or disable ADC DMA continuous request */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           9 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.DMAContinuousRequests <<
                                         9 as libc::c_int) as uint32_t as
                                    uint32_t);
    /* Enable or disable ADC end of conversion selection */
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           10 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*hadc).Init.EOCSelection <<
                                         10 as libc::c_int) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  DMA transfer complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn ADC_DMAConvCplt(mut hdma: *mut DMA_HandleTypeDef) {
    /* Retrieve ADC handle corresponding to current DMA handle */
    let mut hadc: *mut ADC_HandleTypeDef =
        (*hdma).Parent as *mut ADC_HandleTypeDef;
    /* Update state machine on conversion status if not in error state */
    if (*hadc).State & (0x10 as libc::c_uint | 0x40 as libc::c_uint) ==
           RESET as libc::c_int as libc::c_uint {
        /* Update ADC state machine */
        ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x200 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Determine whether any further conversion upcoming on group regular   */
    /* by external trigger, continuous mode or scan sequence on going.      */
    /* Note: On STM32F7, there is no independent flag of end of sequence.   */
    /*       The test of scan sequence on going is done either with scan    */
    /*       sequence disabled or with end of conversion flag set to        */
    /*       of end of sequence.                                            */
        if (*(*hadc).Instance).CR2 &
               (0x3 as libc::c_uint) << 28 as libc::c_uint ==
               RESET as libc::c_int as libc::c_uint &&
               (*hadc).Init.ContinuousConvMode ==
                   DISABLE as libc::c_int as libc::c_uint &&
               ((*(*hadc).Instance).SQR1 &
                    (0xf as libc::c_uint) << 20 as libc::c_uint ==
                    RESET as libc::c_int as libc::c_uint ||
                    (*(*hadc).Instance).CR2 &
                        (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                        RESET as libc::c_int as libc::c_uint) {
            /* Disable ADC end of single conversion interrupt on group regular */
      /* Note: Overrun interrupt was enabled with EOC interrupt in          */
      /* HAL_ADC_Start_IT(), but is not disabled here because can be used   */
      /* by overrun IRQ process below.                                      */
            ::core::ptr::write_volatile(&mut (*(*hadc).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hadc).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Set ADC state */
            ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*hadc).State & 0x1000 as libc::c_uint ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hadc).State as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hadc).State
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        }
        /* Conversion complete callback */
        HAL_ADC_ConvCpltCallback(hadc);
    } else {
        /* Call DMA error callback */
        (*(*hadc).DMA_Handle).XferErrorCallback.expect("non-null function pointer")(hdma);
    };
}
/* *
  * @brief  DMA half transfer complete callback. 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn ADC_DMAHalfConvCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hadc: *mut ADC_HandleTypeDef =
        (*hdma).Parent as *mut ADC_HandleTypeDef;
    /* Conversion complete callback */
    HAL_ADC_ConvHalfCpltCallback(hadc);
}
/* *
  * @brief  DMA error callback 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *                the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn ADC_DMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hadc: *mut ADC_HandleTypeDef =
        (*hdma).Parent as *mut ADC_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*hadc).State as *mut uint32_t,
                                0x40 as libc::c_uint);
    /* Set ADC error code to DMA error */
    ::core::ptr::write_volatile(&mut (*hadc).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hadc).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x4 as libc::c_uint) as
                                    uint32_t as uint32_t);
    HAL_ADC_ErrorCallback(hadc);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_ADC_MODULE_ENABLED */
/* *
  * @}
  */
/* *
  * @}
  */
