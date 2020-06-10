use ::libc;
extern "C" {
    #[no_mangle]
    fn HAL_DMA_Start_IT(hdma: *mut DMA_HandleTypeDef, SrcAddress: uint32_t,
                        DstAddress: uint32_t, DataLength: uint32_t)
     -> HAL_StatusTypeDef;
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
pub type HAL_I2C_StateTypeDef = libc::c_uint;
pub const HAL_I2C_STATE_ERROR: HAL_I2C_StateTypeDef = 224;
pub const HAL_I2C_STATE_TIMEOUT: HAL_I2C_StateTypeDef = 160;
pub const HAL_I2C_STATE_ABORT: HAL_I2C_StateTypeDef = 96;
pub const HAL_I2C_STATE_BUSY_RX_LISTEN: HAL_I2C_StateTypeDef = 42;
pub const HAL_I2C_STATE_BUSY_TX_LISTEN: HAL_I2C_StateTypeDef = 41;
pub const HAL_I2C_STATE_LISTEN: HAL_I2C_StateTypeDef = 40;
pub const HAL_I2C_STATE_BUSY_RX: HAL_I2C_StateTypeDef = 34;
pub const HAL_I2C_STATE_BUSY_TX: HAL_I2C_StateTypeDef = 33;
pub const HAL_I2C_STATE_BUSY: HAL_I2C_StateTypeDef = 36;
pub const HAL_I2C_STATE_READY: HAL_I2C_StateTypeDef = 32;
pub const HAL_I2C_STATE_RESET: HAL_I2C_StateTypeDef = 0;
pub type HAL_I2C_ModeTypeDef = libc::c_uint;
pub const HAL_I2C_MODE_MEM: HAL_I2C_ModeTypeDef = 64;
pub const HAL_I2C_MODE_SLAVE: HAL_I2C_ModeTypeDef = 32;
pub const HAL_I2C_MODE_MASTER: HAL_I2C_ModeTypeDef = 16;
pub const HAL_I2C_MODE_NONE: HAL_I2C_ModeTypeDef = 0;
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
/* !< DMA Stream Index                       */
/* !< Size Management error */
/* *
  * @}
  */
/* * @defgroup I2C_handle_Structure_definition I2C handle Structure definition
  * @brief  I2C handle Structure definition
  * @{
  */
pub type I2C_HandleTypeDef = __I2C_HandleTypeDef;
/* !< I2C registers base address                */
/* !< I2C communication parameters              */
/* !< Pointer to I2C transfer buffer            */
/* !< I2C transfer size                         */
/* !< I2C transfer counter                      */
/* !< I2C sequantial transfer options, this parameter can
                                                  be a value of @ref I2C_XFEROPTIONS */
/* !< I2C communication Previous state          */
/* !< I2C transfer IRQ handler function pointer */
/* !< I2C Tx DMA handle parameters              */
/* !< I2C Rx DMA handle parameters              */
/* !< I2C locking object                        */
/* !< I2C communication state                   */
/* !< I2C communication mode                    */
/* !< I2C Error code                            */
/* !< I2C Address Event counter                 */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup I2C_Exported_Functions I2C Exported Functions
  * @{
  */
/* * @defgroup I2C_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions 
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          deinitialize the I2Cx peripheral:

      (+) User must Implement HAL_I2C_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_I2C_Init() to configure the selected device with
          the selected configuration:
        (++) Clock Timing
        (++) Own Address 1
        (++) Addressing mode (Master, Slave)
        (++) Dual Addressing mode
        (++) Own Address 2
        (++) Own Address 2 Mask
        (++) General call mode
        (++) Nostretch mode

      (+) Call the function HAL_I2C_DeInit() to restore the default configuration
          of the selected I2Cx peripheral.

@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the I2C according to the specified parameters
  *         in the I2C_InitTypeDef and initialize the associated handle.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Init(mut hi2c: *mut I2C_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the I2C handle allocation */
    if hi2c.is_null() { return HAL_ERROR }
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, CORTEX...etc */
        HAL_I2C_MspInit(hi2c);
    }
    ::core::ptr::write_volatile(&mut (*hi2c).State as
                                    *mut HAL_I2C_StateTypeDef,
                                HAL_I2C_STATE_BUSY);
    /* Disable the selected I2C peripheral */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /*---------------------------- I2Cx TIMINGR Configuration ------------------*/
  /* Configure I2Cx: Frequency range */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TIMINGR as
                                    *mut uint32_t,
                                (*hi2c).Init.Timing &
                                    0xf0ffffff as libc::c_uint);
    /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Disable Own Address1 before set the Own Address1 configuration */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).OAR1 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).OAR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           15 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configure I2Cx: Own Address1 and ack own address1 mode */
    if (*hi2c).Init.AddressingMode == 0x1 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).OAR1 as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        15 as libc::c_uint |
                                        (*hi2c).Init.OwnAddress1)
    } else {
        /* I2C_ADDRESSINGMODE_10BIT */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).OAR1 as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        15 as libc::c_uint |
                                        (0x1 as libc::c_uint) <<
                                            10 as libc::c_uint |
                                        (*hi2c).Init.OwnAddress1)
    }
    /*---------------------------- I2Cx CR2 Configuration ----------------------*/
  /* Configure I2Cx: Addressing Master mode */
    if (*hi2c).Init.AddressingMode == 0x2 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        11 as libc::c_uint)
    }
    /* Enable the AUTOEND by default, and enable NACK (should be disable only during Slave process */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          25 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              15 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /*---------------------------- I2Cx OAR2 Configuration ---------------------*/
  /* Disable Own Address2 before set the Own Address2 configuration */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).OAR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).OAR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           15 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configure I2Cx: Dual mode and Own Address2 */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).OAR2 as
                                    *mut uint32_t,
                                (*hi2c).Init.DualAddressMode |
                                    (*hi2c).Init.OwnAddress2 |
                                    (*hi2c).Init.OwnAddress2Masks <<
                                        8 as libc::c_int);
    /*---------------------------- I2Cx CR1 Configuration ----------------------*/
  /* Configure I2Cx: Generalcall and NoStretch mode */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (*hi2c).Init.GeneralCallMode |
                                    (*hi2c).Init.NoStretchMode);
    /* Enable the selected I2C peripheral */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*hi2c).State as
                                    *mut HAL_I2C_StateTypeDef,
                                HAL_I2C_STATE_READY);
    ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as *mut uint32_t,
                                HAL_I2C_MODE_NONE as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    return HAL_OK;
}
/* *
  * @brief  DeInitialize the I2C peripheral.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_DeInit(mut hi2c: *mut I2C_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the I2C handle allocation */
    if hi2c.is_null() { return HAL_ERROR }
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*hi2c).State as
                                    *mut HAL_I2C_StateTypeDef,
                                HAL_I2C_STATE_BUSY);
    /* Disable the I2C Peripheral Clock */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC */
    HAL_I2C_MspDeInit(hi2c);
    ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*hi2c).State as
                                    *mut HAL_I2C_StateTypeDef,
                                HAL_I2C_STATE_RESET);
    ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as *mut uint32_t,
                                HAL_I2C_MODE_NONE as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    /* Release Lock */
    (*hi2c).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief Initialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_MspInit(mut hi2c: *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspInit could be implemented in the user file
   */
}
/* *
  * @brief DeInitialize the I2C MSP.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_MspDeInit(mut hi2c: *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MspDeInit could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup I2C_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions 
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the I2C data
    transfers.

    (#) There are two modes of transfer:
       (++) Blocking mode : The communication is performed in the polling mode.
            The status of all data processing is returned by the same function 
            after finishing transfer.
       (++) No-Blocking mode : The communication is performed using Interrupts
            or DMA. These functions return the status of the transfer startup.
            The end of the data processing will be indicated through the
            dedicated I2C IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.

    (#) Blocking mode functions are :
        (++) HAL_I2C_Master_Transmit()
        (++) HAL_I2C_Master_Receive()
        (++) HAL_I2C_Slave_Transmit()
        (++) HAL_I2C_Slave_Receive()
        (++) HAL_I2C_Mem_Write()
        (++) HAL_I2C_Mem_Read()
        (++) HAL_I2C_IsDeviceReady()

    (#) No-Blocking mode functions with Interrupt are :
        (++) HAL_I2C_Master_Transmit_IT()
        (++) HAL_I2C_Master_Receive_IT()
        (++) HAL_I2C_Slave_Transmit_IT()
        (++) HAL_I2C_Slave_Receive_IT()
        (++) HAL_I2C_Mem_Write_IT()
        (++) HAL_I2C_Mem_Read_IT()

    (#) No-Blocking mode functions with DMA are :
        (++) HAL_I2C_Master_Transmit_DMA()
        (++) HAL_I2C_Master_Receive_DMA()
        (++) HAL_I2C_Slave_Transmit_DMA()
        (++) HAL_I2C_Slave_Receive_DMA()
        (++) HAL_I2C_Mem_Write_DMA()
        (++) HAL_I2C_Mem_Read_DMA()

    (#) A set of Transfer Complete Callbacks are provided in non Blocking mode:
        (++) HAL_I2C_MemTxCpltCallback()
        (++) HAL_I2C_MemRxCpltCallback()
        (++) HAL_I2C_MasterTxCpltCallback()
        (++) HAL_I2C_MasterRxCpltCallback()
        (++) HAL_I2C_SlaveTxCpltCallback()
        (++) HAL_I2C_SlaveRxCpltCallback()
        (++) HAL_I2C_ErrorCallback()

@endverbatim
  * @{
  */
/* *
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Transmit(mut hi2c:
                                                     *mut I2C_HandleTypeDef,
                                                 mut DevAddress: uint16_t,
                                                 mut pData: *mut uint8_t,
                                                 mut Size: uint16_t,
                                                 mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          15 as libc::c_uint, SET,
                                      25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferISR = None;
        /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 24 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint);
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 25 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint);
        }
        while (*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint {
            /* Wait until TXIS flag is set */
            if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                    return HAL_ERROR
                } else { return HAL_TIMEOUT }
            }
            /* Write data to TXDR */
            let fresh0 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                            *mut uint32_t,
                                        *fresh0 as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            if (*hi2c).XferSize as libc::c_uint == 0 as libc::c_uint &&
                   (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
                /* Wait until TCR flag is set */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  7 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
                    (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           24 as libc::c_uint,
                                       0 as libc::c_uint);
                } else {
                    (*hi2c).XferSize = (*hi2c).XferCount;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           25 as libc::c_uint,
                                       0 as libc::c_uint);
                }
            }
        }
        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
        if I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        /* Clear STOP Flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Clear Configuration Register 2 */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               0 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint |
                                               (0xff as libc::c_uint) <<
                                                   16 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   24 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Receive(mut hi2c:
                                                    *mut I2C_HandleTypeDef,
                                                mut DevAddress: uint16_t,
                                                mut pData: *mut uint8_t,
                                                mut Size: uint16_t,
                                                mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          15 as libc::c_uint, SET,
                                      25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferISR = None;
        /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 24 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint |
                                   (0x1 as libc::c_uint) <<
                                       10 as libc::c_uint);
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 25 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint |
                                   (0x1 as libc::c_uint) <<
                                       10 as libc::c_uint);
        }
        while (*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint {
            /* Wait until RXNE flag is set */
            if I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                    return HAL_ERROR
                } else { return HAL_TIMEOUT }
            }
            /* Read data from RXDR */
            let fresh1 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            *fresh1 = (*(*hi2c).Instance).RXDR as uint8_t;
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            if (*hi2c).XferSize as libc::c_uint == 0 as libc::c_uint &&
                   (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
                /* Wait until TCR flag is set */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  7 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
                    (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           24 as libc::c_uint,
                                       0 as libc::c_uint);
                } else {
                    (*hi2c).XferSize = (*hi2c).XferCount;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           25 as libc::c_uint,
                                       0 as libc::c_uint);
                }
            }
        }
        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
        if I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        /* Clear STOP Flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Clear Configuration Register 2 */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               0 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint |
                                               (0xff as libc::c_uint) <<
                                                   16 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   24 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Transmits in slave mode an amount of data in blocking mode. 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Transmit(mut hi2c:
                                                    *mut I2C_HandleTypeDef,
                                                mut pData: *mut uint8_t,
                                                mut Size: uint16_t,
                                                mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferISR = None;
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Wait until ADDR flag is set */
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          3 as libc::c_uint, RESET, Timeout,
                                      tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_TIMEOUT
        }
        /* Clear ADDR flag */
        if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint)
        };
        /* If 10bit addressing mode is selected */
        if (*hi2c).Init.AddressingMode == 0x2 as libc::c_uint {
            /* Wait until ADDR flag is set */
            if I2C_WaitOnFlagUntilTimeout(hi2c,
                                          (0x1 as libc::c_uint) <<
                                              3 as libc::c_uint, RESET,
                                          Timeout, tickstart) as libc::c_uint
                   != HAL_OK as libc::c_int as libc::c_uint {
                /* Disable Address Acknowledge */
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t);
                return HAL_TIMEOUT
            }
            /* Clear ADDR flag */
            if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     3 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                3 as libc::c_uint)
            };
        }
        /* Wait until DIR flag is set Transmitter mode */
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          16 as libc::c_uint, RESET, Timeout,
                                      tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_TIMEOUT
        }
        while (*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint {
            /* Wait until TXIS flag is set */
            if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                /* Disable Address Acknowledge */
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t);
                if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                    return HAL_ERROR
                } else { return HAL_TIMEOUT }
            }
            /* Write data to TXDR */
            let fresh2 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                            *mut uint32_t,
                                        *fresh2 as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1))
        }
        /* Wait until STOP flag is set */
        if I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Normal use case for Transmitter mode */
        /* A NACK is generated to confirm the end of transfer */
                ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                                *mut uint32_t,
                                            0 as libc::c_uint)
            } else { return HAL_TIMEOUT }
        }
        /* Clear STOP flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Wait until BUSY flag is reset */
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          15 as libc::c_uint, SET, Timeout,
                                      tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_TIMEOUT
        }
        /* Disable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Receive in slave mode an amount of data in blocking mode
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Receive(mut hi2c:
                                                   *mut I2C_HandleTypeDef,
                                               mut pData: *mut uint8_t,
                                               mut Size: uint16_t,
                                               mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferISR = None;
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Wait until ADDR flag is set */
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          3 as libc::c_uint, RESET, Timeout,
                                      tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_TIMEOUT
        }
        /* Clear ADDR flag */
        if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint)
        };
        /* Wait until DIR flag is reset Receiver mode */
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          16 as libc::c_uint, SET, Timeout,
                                      tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_TIMEOUT
        }
        while (*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint {
            /* Wait until RXNE flag is set */
            if I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                /* Disable Address Acknowledge */
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t);
                /* Store Last receive data if any */
                if (if (*(*hi2c).Instance).ISR &
                           (0x1 as libc::c_uint) << 2 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 2 as libc::c_uint {
                        SET as libc::c_int
                    } else { RESET as libc::c_int }) == SET as libc::c_int {
                    /* Read data from RXDR */
                    let fresh3 = (*hi2c).pBuffPtr;
                    (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
                    *fresh3 = (*(*hi2c).Instance).RXDR as uint8_t;
                    ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                                    *mut uint16_t,
                                                ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                           as
                                                                                           *const uint16_t).wrapping_sub(1))
                }
                if (*hi2c).ErrorCode == 0x20 as libc::c_uint {
                    return HAL_TIMEOUT
                } else { return HAL_ERROR }
            }
            /* Read data from RXDR */
            let fresh4 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            *fresh4 = (*(*hi2c).Instance).RXDR as uint8_t;
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1))
        }
        /* Wait until STOP flag is set */
        if I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        /* Clear STOP flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Wait until BUSY flag is reset */
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          15 as libc::c_uint, SET, Timeout,
                                      tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Disable Address Acknowledge */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t);
            return HAL_TIMEOUT
        }
        /* Disable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Transmit in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Transmit_IT(mut hi2c:
                                                        *mut I2C_HandleTypeDef,
                                                    mut DevAddress: uint16_t,
                                                    mut pData: *mut uint8_t,
                                                    mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode,
                           (0x1 as libc::c_uint) << 13 as libc::c_uint);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Receive in master mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Receive_IT(mut hi2c:
                                                       *mut I2C_HandleTypeDef,
                                                   mut DevAddress: uint16_t,
                                                   mut pData: *mut uint8_t,
                                                   mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode,
                           (0x1 as libc::c_uint) << 13 as libc::c_uint |
                               (0x1 as libc::c_uint) << 10 as libc::c_uint);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with Interrupt
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Transmit_IT(mut hi2c:
                                                       *mut I2C_HandleTypeDef,
                                                   mut pData: *mut uint8_t,
                                                   mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferSize = (*hi2c).XferCount;
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c,
                       (0x1 as libc::c_uint | 0x4 as libc::c_uint) as
                           uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Receive in slave mode an amount of data in non-blocking mode with Interrupt 
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Receive_IT(mut hi2c:
                                                      *mut I2C_HandleTypeDef,
                                                  mut pData: *mut uint8_t,
                                                  mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferSize = (*hi2c).XferCount;
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c,
                       (0x2 as libc::c_uint | 0x4 as libc::c_uint) as
                           uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Transmit in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Transmit_DMA(mut hi2c:
                                                         *mut I2C_HandleTypeDef,
                                                     mut DevAddress: uint16_t,
                                                     mut pData: *mut uint8_t,
                                                     mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_DMA as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        if (*hi2c).XferSize as libc::c_uint > 0 as libc::c_uint {
            /* Set the I2C DMA transfer complete callback */
            (*(*hi2c).hdmatx).XferCpltCallback =
                Some(I2C_DMAMasterTransmitCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*hi2c).hdmatx).XferErrorCallback =
                Some(I2C_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the unused DMA callbacks to NULL */
            (*(*hi2c).hdmatx).XferHalfCpltCallback = None;
            (*(*hi2c).hdmatx).XferAbortCallback = None;
            /* Enable the DMA channel */
            HAL_DMA_Start_IT((*hi2c).hdmatx, pData as uint32_t,
                             &mut (*(*hi2c).Instance).TXDR as *mut uint32_t as
                                 uint32_t, (*hi2c).XferSize as uint32_t);
            /* Send Slave Address */
      /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               xfermode,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint);
            /* Update XferCount value */
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        (::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int -
                                             (*hi2c).XferSize as libc::c_int)
                                            as uint16_t as uint16_t);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR and NACK interrupts */
            I2C_Enable_IRQ(hi2c, 0x11 as libc::c_uint as uint16_t);
            /* Enable DMA Request */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 14 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            /* Update Transfer ISR function pointer */
            (*hi2c).XferISR =
                Some(I2C_Master_ISR_IT as
                         unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                              _: uint32_t, _: uint32_t)
                             -> HAL_StatusTypeDef);
            /* Send Slave Address */
      /* Set NBYTES to write and generate START condition */
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 25 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
            I2C_Enable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        }
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Receive in master mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Receive_DMA(mut hi2c:
                                                        *mut I2C_HandleTypeDef,
                                                    mut DevAddress: uint16_t,
                                                    mut pData: *mut uint8_t,
                                                    mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_DMA as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        if (*hi2c).XferSize as libc::c_uint > 0 as libc::c_uint {
            /* Set the I2C DMA transfer complete callback */
            (*(*hi2c).hdmarx).XferCpltCallback =
                Some(I2C_DMAMasterReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*hi2c).hdmarx).XferErrorCallback =
                Some(I2C_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the unused DMA callbacks to NULL */
            (*(*hi2c).hdmarx).XferHalfCpltCallback = None;
            (*(*hi2c).hdmarx).XferAbortCallback = None;
            /* Enable the DMA channel */
            HAL_DMA_Start_IT((*hi2c).hdmarx,
                             &mut (*(*hi2c).Instance).RXDR as *mut uint32_t as
                                 uint32_t, pData as uint32_t,
                             (*hi2c).XferSize as uint32_t);
            /* Send Slave Address */
      /* Set NBYTES to read and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               xfermode,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint |
                                   (0x1 as libc::c_uint) <<
                                       10 as libc::c_uint);
            /* Update XferCount value */
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        (::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int -
                                             (*hi2c).XferSize as libc::c_int)
                                            as uint16_t as uint16_t);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR and NACK interrupts */
            I2C_Enable_IRQ(hi2c, 0x11 as libc::c_uint as uint16_t);
            /* Enable DMA Request */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 15 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            /* Update Transfer ISR function pointer */
            (*hi2c).XferISR =
                Some(I2C_Master_ISR_IT as
                         unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                              _: uint32_t, _: uint32_t)
                             -> HAL_StatusTypeDef);
            /* Send Slave Address */
      /* Set NBYTES to read and generate START condition */
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 25 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint |
                                   (0x1 as libc::c_uint) <<
                                       10 as libc::c_uint);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            /* Note : The I2C interrupts must be enabled after unlocking current process
                to avoid the risk of I2C interrupt handle execution before current
                process unlock */
      /* Enable ERR, TC, STOP, NACK, TXI interrupt */
      /* possible to enable all of these */
      /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
            I2C_Enable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        }
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Transmit in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Transmit_DMA(mut hi2c:
                                                        *mut I2C_HandleTypeDef,
                                                    mut pData: *mut uint8_t,
                                                    mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferSize = (*hi2c).XferCount;
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_DMA as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* Set the I2C DMA transfer complete callback */
        (*(*hi2c).hdmatx).XferCpltCallback =
            Some(I2C_DMASlaveTransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hi2c).hdmatx).XferErrorCallback =
            Some(I2C_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the unused DMA callbacks to NULL */
        (*(*hi2c).hdmatx).XferHalfCpltCallback = None;
        (*(*hi2c).hdmatx).XferAbortCallback = None;
        /* Enable the DMA channel */
        HAL_DMA_Start_IT((*hi2c).hdmatx, pData as uint32_t,
                         &mut (*(*hi2c).Instance).TXDR as *mut uint32_t as
                             uint32_t, (*hi2c).XferSize as uint32_t);
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    /* Enable ERR, STOP, NACK, ADDR interrupts */
        I2C_Enable_IRQ(hi2c, 0x4 as libc::c_uint as uint16_t);
        /* Enable DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             14 as libc::c_uint) as uint32_t
                                        as uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Receive in slave mode an amount of data in non-blocking mode with DMA
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Receive_DMA(mut hi2c:
                                                       *mut I2C_HandleTypeDef,
                                                   mut pData: *mut uint8_t,
                                                   mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferSize = (*hi2c).XferCount;
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_DMA as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* Set the I2C DMA transfer complete callback */
        (*(*hi2c).hdmarx).XferCpltCallback =
            Some(I2C_DMASlaveReceiveCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hi2c).hdmarx).XferErrorCallback =
            Some(I2C_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the unused DMA callbacks to NULL */
        (*(*hi2c).hdmarx).XferHalfCpltCallback = None;
        (*(*hi2c).hdmarx).XferAbortCallback = None;
        /* Enable the DMA channel */
        HAL_DMA_Start_IT((*hi2c).hdmarx,
                         &mut (*(*hi2c).Instance).RXDR as *mut uint32_t as
                             uint32_t, pData as uint32_t,
                         (*hi2c).XferSize as uint32_t);
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    /* Enable ERR, STOP, NACK, ADDR interrupts */
        I2C_Enable_IRQ(hi2c, 0x4 as libc::c_uint as uint16_t);
        /* Enable DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Mem_Write(mut hi2c: *mut I2C_HandleTypeDef,
                                           mut DevAddress: uint16_t,
                                           mut MemAddress: uint16_t,
                                           mut MemAddSize: uint16_t,
                                           mut pData: *mut uint8_t,
                                           mut Size: uint16_t,
                                           mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          15 as libc::c_uint, SET,
                                      25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MEM);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferISR = None;
        /* Send Slave Address and Memory Address */
        if I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize,
                                  Timeout, tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_ERROR
            } else {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 24 as libc::c_uint,
                               0 as libc::c_uint);
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 25 as libc::c_uint,
                               0 as libc::c_uint);
        }
        loop  {
            /* Wait until TXIS flag is set */
            if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) as
                   libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                    return HAL_ERROR
                } else { return HAL_TIMEOUT }
            }
            /* Write data to TXDR */
            let fresh5 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                            *mut uint32_t,
                                        *fresh5 as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            if (*hi2c).XferSize as libc::c_uint == 0 as libc::c_uint &&
                   (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
                /* Wait until TCR flag is set */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  7 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
                    (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           24 as libc::c_uint,
                                       0 as libc::c_uint);
                } else {
                    (*hi2c).XferSize = (*hi2c).XferCount;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           25 as libc::c_uint,
                                       0 as libc::c_uint);
                }
            }
            if !((*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint) {
                break ;
            }
        }
        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
        if I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        /* Clear STOP Flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Clear Configuration Register 2 */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               0 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint |
                                               (0xff as libc::c_uint) <<
                                                   16 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   24 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Mem_Read(mut hi2c: *mut I2C_HandleTypeDef,
                                          mut DevAddress: uint16_t,
                                          mut MemAddress: uint16_t,
                                          mut MemAddSize: uint16_t,
                                          mut pData: *mut uint8_t,
                                          mut Size: uint16_t,
                                          mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        if I2C_WaitOnFlagUntilTimeout(hi2c,
                                      (0x1 as libc::c_uint) <<
                                          15 as libc::c_uint, SET,
                                      25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_TIMEOUT
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MEM);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferISR = None;
        /* Send Slave Address and Memory Address */
        if I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize,
                                 Timeout, tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_ERROR
            } else {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
        /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 24 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint |
                                   (0x1 as libc::c_uint) <<
                                       10 as libc::c_uint);
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                               (0x1 as libc::c_uint) << 25 as libc::c_uint,
                               (0x1 as libc::c_uint) << 13 as libc::c_uint |
                                   (0x1 as libc::c_uint) <<
                                       10 as libc::c_uint);
        }
        loop  {
            /* Wait until RXNE flag is set */
            if I2C_WaitOnFlagUntilTimeout(hi2c,
                                          (0x1 as libc::c_uint) <<
                                              2 as libc::c_uint, RESET,
                                          Timeout, tickstart) as libc::c_uint
                   != HAL_OK as libc::c_int as libc::c_uint {
                return HAL_TIMEOUT
            }
            /* Read data from RXDR */
            let fresh6 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            *fresh6 = (*(*hi2c).Instance).RXDR as uint8_t;
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            if (*hi2c).XferSize as libc::c_uint == 0 as libc::c_uint &&
                   (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
                /* Wait until TCR flag is set */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  7 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
                    (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           24 as libc::c_uint,
                                       0 as libc::c_uint);
                } else {
                    (*hi2c).XferSize = (*hi2c).XferCount;
                    I2C_TransferConfig(hi2c, DevAddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           25 as libc::c_uint,
                                       0 as libc::c_uint);
                }
            }
            if !((*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint) {
                break ;
            }
        }
        /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
        if I2C_WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        /* Clear STOP Flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Clear Configuration Register 2 */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               0 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint |
                                               (0xff as libc::c_uint) <<
                                                   16 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   24 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Write an amount of data in non-blocking mode with Interrupt to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Mem_Write_IT(mut hi2c:
                                                  *mut I2C_HandleTypeDef,
                                              mut DevAddress: uint16_t,
                                              mut MemAddress: uint16_t,
                                              mut MemAddSize: uint16_t,
                                              mut pData: *mut uint8_t,
                                              mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MEM);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        /* Send Slave Address and Memory Address */
        if I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize,
                                  25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_ERROR
            } else {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode, 0 as libc::c_uint);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process 
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        /* Enable ERR, TC, STOP, NACK, TXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Read an amount of data in non-blocking mode with Interrupt from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Mem_Read_IT(mut hi2c: *mut I2C_HandleTypeDef,
                                             mut DevAddress: uint16_t,
                                             mut MemAddress: uint16_t,
                                             mut MemAddSize: uint16_t,
                                             mut pData: *mut uint8_t,
                                             mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MEM);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        /* Send Slave Address and Memory Address */
        if I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize,
                                 25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_ERROR
            } else {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode,
                           (0x1 as libc::c_uint) << 13 as libc::c_uint |
                               (0x1 as libc::c_uint) << 10 as libc::c_uint);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        /* Enable ERR, TC, STOP, NACK, RXI interrupt */
    /* possible to enable all of these */
    /* I2C_IT_ERRI | I2C_IT_TCI| I2C_IT_STOPI| I2C_IT_NACKI | I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_TXI */
        I2C_Enable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Write an amount of data in non-blocking mode with DMA to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Mem_Write_DMA(mut hi2c:
                                                   *mut I2C_HandleTypeDef,
                                               mut DevAddress: uint16_t,
                                               mut MemAddress: uint16_t,
                                               mut MemAddSize: uint16_t,
                                               mut pData: *mut uint8_t,
                                               mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MEM);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_DMA as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        /* Send Slave Address and Memory Address */
        if I2C_RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize,
                                  25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_ERROR
            } else {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
        /* Set the I2C DMA transfer complete callback */
        (*(*hi2c).hdmatx).XferCpltCallback =
            Some(I2C_DMAMasterTransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hi2c).hdmatx).XferErrorCallback =
            Some(I2C_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the unused DMA callbacks to NULL */
        (*(*hi2c).hdmatx).XferHalfCpltCallback = None;
        (*(*hi2c).hdmatx).XferAbortCallback = None;
        /* Enable the DMA channel */
        HAL_DMA_Start_IT((*hi2c).hdmatx, pData as uint32_t,
                         &mut (*(*hi2c).Instance).TXDR as *mut uint32_t as
                             uint32_t, (*hi2c).XferSize as uint32_t);
        /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode, 0 as libc::c_uint);
        /* Update XferCount value */
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int -
                                         (*hi2c).XferSize as libc::c_int) as
                                        uint16_t as uint16_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    /* Enable ERR and NACK interrupts */
        I2C_Enable_IRQ(hi2c, 0x11 as libc::c_uint as uint16_t);
        /* Enable DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             14 as libc::c_uint) as uint32_t
                                        as uint32_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Reads an amount of data in non-blocking mode with DMA from a specific memory address.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be read
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Mem_Read_DMA(mut hi2c:
                                                  *mut I2C_HandleTypeDef,
                                              mut DevAddress: uint16_t,
                                              mut MemAddress: uint16_t,
                                              mut MemAddSize: uint16_t,
                                              mut pData: *mut uint8_t,
                                              mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Init tickstart for timeout management*/
        tickstart = HAL_GetTick();
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MEM);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_DMA as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
        }
        /* Send Slave Address and Memory Address */
        if I2C_RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize,
                                 25 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_ERROR
            } else {
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
        /* Set the I2C DMA transfer complete callback */
        (*(*hi2c).hdmarx).XferCpltCallback =
            Some(I2C_DMAMasterReceiveCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hi2c).hdmarx).XferErrorCallback =
            Some(I2C_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the unused DMA callbacks to NULL */
        (*(*hi2c).hdmarx).XferHalfCpltCallback = None;
        (*(*hi2c).hdmarx).XferAbortCallback = None;
        /* Enable the DMA channel */
        HAL_DMA_Start_IT((*hi2c).hdmarx,
                         &mut (*(*hi2c).Instance).RXDR as *mut uint32_t as
                             uint32_t, pData as uint32_t,
                         (*hi2c).XferSize as uint32_t);
        /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode,
                           (0x1 as libc::c_uint) << 13 as libc::c_uint |
                               (0x1 as libc::c_uint) << 10 as libc::c_uint);
        /* Update XferCount value */
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int -
                                         (*hi2c).XferSize as libc::c_int) as
                                        uint16_t as uint16_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Enable DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             15 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
    /* Enable ERR and NACK interrupts */
        I2C_Enable_IRQ(hi2c, 0x11 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Checks if target device is ready for communication.
  * @note   This function is used with Memory devices
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  Trials Number of trials
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_IsDeviceReady(mut hi2c:
                                                   *mut I2C_HandleTypeDef,
                                               mut DevAddress: uint16_t,
                                               mut Trials: uint32_t,
                                               mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut I2C_Trials: uint32_t = 0 as libc::c_uint;
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 15 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            return HAL_BUSY
        }
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        loop  {
            /* Generate Start */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        if (*hi2c).Init.AddressingMode ==
                                               0x1 as libc::c_uint {
                                            (DevAddress as uint32_t &
                                                 (0x3ff as libc::c_uint) <<
                                                     0 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     13 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     25 as libc::c_uint) &
                                                !((0x1 as libc::c_uint) <<
                                                      10 as libc::c_uint)
                                        } else {
                                            (DevAddress as uint32_t &
                                                 (0x3ff as libc::c_uint) <<
                                                     0 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     11 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     13 as libc::c_uint) &
                                                !((0x1 as libc::c_uint) <<
                                                      10 as libc::c_uint)
                                        });
            /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
      /* Wait until STOPF flag is set or a NACK flag is set*/
            tickstart = HAL_GetTick();
            while (if (*(*hi2c).Instance).ISR &
                          (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                          (0x1 as libc::c_uint) << 5 as libc::c_uint {
                       SET as libc::c_int
                   } else { RESET as libc::c_int }) == RESET as libc::c_int &&
                      (if (*(*hi2c).Instance).ISR &
                              (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                              (0x1 as libc::c_uint) << 4 as libc::c_uint {
                           SET as libc::c_int
                       } else { RESET as libc::c_int }) ==
                          RESET as libc::c_int &&
                      (*hi2c).State as libc::c_uint !=
                          HAL_I2C_STATE_TIMEOUT as libc::c_int as libc::c_uint
                  {
                if Timeout != 0xffffffff as libc::c_uint {
                    if Timeout == 0 as libc::c_uint ||
                           HAL_GetTick().wrapping_sub(tickstart) > Timeout {
                        /* Device is ready */
                        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                                        *mut HAL_I2C_StateTypeDef,
                                                    HAL_I2C_STATE_READY);
                        /* Process Unlocked */
                        (*hi2c).Lock = HAL_UNLOCKED;
                        return HAL_TIMEOUT
                    }
                }
            }
            /* Check if the NACKF flag has not been set */
            if (if (*(*hi2c).Instance).ISR &
                       (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 4 as libc::c_uint {
                    SET as libc::c_int
                } else { RESET as libc::c_int }) == RESET as libc::c_int {
                /* Wait until STOPF flag is reset */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                /* Clear STOP Flag */
                if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         5 as libc::c_uint) as
                                                    uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                    as *mut uint32_t,
                                                (0x1 as libc::c_uint) <<
                                                    5 as libc::c_uint)
                };
                /* Device is ready */
                ::core::ptr::write_volatile(&mut (*hi2c).State as
                                                *mut HAL_I2C_StateTypeDef,
                                            HAL_I2C_STATE_READY);
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_OK
            } else {
                /* Wait until STOPF flag is reset */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                /* Clear NACK Flag */
                if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         4 as libc::c_uint) as
                                                    uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                    as *mut uint32_t,
                                                (0x1 as libc::c_uint) <<
                                                    4 as libc::c_uint)
                };
                /* Clear STOP Flag, auto generated with autoend*/
                if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         5 as libc::c_uint) as
                                                    uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                    as *mut uint32_t,
                                                (0x1 as libc::c_uint) <<
                                                    5 as libc::c_uint)
                };
            }
            /* Check if the maximum allowed number of trials has been reached */
            let fresh7 =
                ::core::ptr::read_volatile::<uint32_t>(&I2C_Trials as
                                                           *const uint32_t);
            ::core::ptr::write_volatile(&mut I2C_Trials as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&I2C_Trials
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if fresh7 == Trials {
                /* Generate Stop */
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     14 as libc::c_uint) as
                                                uint32_t as uint32_t);
                /* Wait until STOPF flag is reset */
                if I2C_WaitOnFlagUntilTimeout(hi2c,
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint, RESET,
                                              Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    return HAL_TIMEOUT
                }
                /* Clear STOP Flag */
                if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         5 as libc::c_uint) as
                                                    uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                    as *mut uint32_t,
                                                (0x1 as libc::c_uint) <<
                                                    5 as libc::c_uint)
                };
            }
            if !(I2C_Trials < Trials) { break ; }
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_TIMEOUT
    } else { return HAL_BUSY };
}
/* *
  * @brief  Sequential transmit in master I2C mode an amount of data in non-blocking mode with Interrupt.
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Sequential_Transmit_IT(mut hi2c:
                                                                   *mut I2C_HandleTypeDef,
                                                               mut DevAddress:
                                                                   uint16_t,
                                                               mut pData:
                                                                   *mut uint8_t,
                                                               mut Size:
                                                                   uint16_t,
                                                               mut XferOptions:
                                                                   uint32_t)
 -> HAL_StatusTypeDef {
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    let mut xferrequest: uint32_t =
        (0x1 as libc::c_uint) << 13 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    XferOptions);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* If size > MAX_NBYTE_SIZE, use reload mode */
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (*hi2c).XferOptions
        }
        /* If transfer direction not change, do not generate Restart Condition */
    /* Mean Previous state is same as current state */
        if (*hi2c).PreviousState ==
               HAL_I2C_STATE_BUSY_TX as libc::c_int as libc::c_uint &
                   ((HAL_I2C_STATE_BUSY_TX as libc::c_int |
                         HAL_I2C_STATE_BUSY_RX as libc::c_int) as libc::c_uint
                        & !(HAL_I2C_STATE_READY as libc::c_int as uint32_t)) |
                   HAL_I2C_MODE_MASTER as libc::c_int as libc::c_uint {
            xferrequest = 0 as libc::c_uint
        }
        /* Send Slave Address and set NBYTES to write */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode, xferrequest);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        I2C_Enable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Sequential receive in master I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Sequential_Receive_IT(mut hi2c:
                                                                  *mut I2C_HandleTypeDef,
                                                              mut DevAddress:
                                                                  uint16_t,
                                                              mut pData:
                                                                  *mut uint8_t,
                                                              mut Size:
                                                                  uint16_t,
                                                              mut XferOptions:
                                                                  uint32_t)
 -> HAL_StatusTypeDef {
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    let mut xferrequest: uint32_t =
        (0x1 as libc::c_uint) << 13 as libc::c_uint |
            (0x1 as libc::c_uint) << 10 as libc::c_uint;
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_MASTER);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    XferOptions);
        (*hi2c).XferISR =
            Some(I2C_Master_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* If hi2c->XferCount > MAX_NBYTE_SIZE, use reload mode */
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
            xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
        } else {
            (*hi2c).XferSize = (*hi2c).XferCount;
            xfermode = (*hi2c).XferOptions
        }
        /* If transfer direction not change, do not generate Restart Condition */
    /* Mean Previous state is same as current state */
        if (*hi2c).PreviousState ==
               HAL_I2C_STATE_BUSY_RX as libc::c_int as libc::c_uint &
                   ((HAL_I2C_STATE_BUSY_TX as libc::c_int |
                         HAL_I2C_STATE_BUSY_RX as libc::c_int) as libc::c_uint
                        & !(HAL_I2C_STATE_READY as libc::c_int as uint32_t)) |
                   HAL_I2C_MODE_MASTER as libc::c_int as libc::c_uint {
            xferrequest = 0 as libc::c_uint
        }
        /* Send Slave Address and set NBYTES to read */
        I2C_TransferConfig(hi2c, DevAddress, (*hi2c).XferSize as uint8_t,
                           xfermode, xferrequest);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        I2C_Enable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Sequential transmit in slave/device I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Sequential_Transmit_IT(mut hi2c:
                                                                  *mut I2C_HandleTypeDef,
                                                              mut pData:
                                                                  *mut uint8_t,
                                                              mut Size:
                                                                  uint16_t,
                                                              mut XferOptions:
                                                                  uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint &
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint ==
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
        I2C_Disable_IRQ(hi2c,
                        (0x4 as libc::c_uint | 0x1 as libc::c_uint) as
                            uint16_t);
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave RX state to TX state */
        if (*hi2c).State as libc::c_uint ==
               HAL_I2C_STATE_BUSY_RX_LISTEN as libc::c_int as libc::c_uint {
            /* Disable associated Interrupts */
            I2C_Disable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_TX_LISTEN);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferSize = (*hi2c).XferCount;
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    XferOptions);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if ((*(*hi2c).Instance).ISR &
                (0x1 as libc::c_uint) << 16 as libc::c_uint) >>
               16 as libc::c_uint == 0x1 as libc::c_uint {
            /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
            if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     3 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                3 as libc::c_uint)
            };
        }
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
        I2C_Enable_IRQ(hi2c,
                       (0x1 as libc::c_uint | 0x4 as libc::c_uint) as
                           uint16_t);
        return HAL_OK
    } else { return HAL_ERROR };
}
/* *
  * @brief  Sequential receive in slave/device I2C mode an amount of data in non-blocking mode with Interrupt
  * @note   This interface allow to manage repeated start condition when a direction change during transfer
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  XferOptions Options of Transfer, value of @ref I2C_XFEROPTIONS
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Slave_Sequential_Receive_IT(mut hi2c:
                                                                 *mut I2C_HandleTypeDef,
                                                             mut pData:
                                                                 *mut uint8_t,
                                                             mut Size:
                                                                 uint16_t,
                                                             mut XferOptions:
                                                                 uint32_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    if (*hi2c).State as libc::c_uint &
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint ==
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
        if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
            return HAL_ERROR
        }
        /* Disable Interrupts, to prevent preemption during treatment in case of multicall */
        I2C_Disable_IRQ(hi2c,
                        (0x4 as libc::c_uint | 0x2 as libc::c_uint) as
                            uint16_t);
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* I2C cannot manage full duplex exchange so disable previous IT enabled if any */
    /* and then toggle the HAL slave TX state to RX state */
        if (*hi2c).State as libc::c_uint ==
               HAL_I2C_STATE_BUSY_TX_LISTEN as libc::c_int as libc::c_uint {
            /* Disable associated Interrupts */
            I2C_Disable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        }
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_BUSY_RX_LISTEN);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_SLAVE);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Enable Address Acknowledge */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Prepare transfer parameters */
        (*hi2c).pBuffPtr = pData;
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    Size);
        (*hi2c).XferSize = (*hi2c).XferCount;
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    XferOptions);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        if ((*(*hi2c).Instance).ISR &
                (0x1 as libc::c_uint) << 16 as libc::c_uint) >>
               16 as libc::c_uint == 0 as libc::c_uint {
            /* Clear ADDR flag after prepare the transfer parameters */
      /* This action will generate an acknowledge to the Master */
            if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     3 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                3 as libc::c_uint)
            };
        }
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process
    to avoid the risk of I2C interrupt handle execution before current
    process unlock */
    /* REnable ADDR interrupt */
        I2C_Enable_IRQ(hi2c,
                       (0x2 as libc::c_uint | 0x4 as libc::c_uint) as
                           uint16_t);
        return HAL_OK
    } else { return HAL_ERROR };
}
/* *
  * @brief  Enable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_EnableListen_IT(mut hi2c:
                                                     *mut I2C_HandleTypeDef)
 -> HAL_StatusTypeDef {
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_READY as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_LISTEN);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef);
        /* Enable the Address Match interrupt */
        I2C_Enable_IRQ(hi2c, 0x4 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Disable the Address listen mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_DisableListen_IT(mut hi2c:
                                                      *mut I2C_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Declaration of tmp to prevent undefined behavior of volatile usage */
    let mut tmp: uint32_t = 0;
    /* Disable Address listen mode only if a transfer is not ongoing */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
        tmp =
            (*hi2c).State as uint32_t &
                ((HAL_I2C_STATE_BUSY_TX as libc::c_int |
                      HAL_I2C_STATE_BUSY_RX as libc::c_int) as libc::c_uint &
                     !(HAL_I2C_STATE_READY as libc::c_int as uint32_t));
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    tmp | (*hi2c).Mode as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        (*hi2c).XferISR = None;
        /* Disable the Address Match interrupt */
        I2C_Disable_IRQ(hi2c, 0x4 as libc::c_uint as uint16_t);
        return HAL_OK
    } else { return HAL_BUSY };
}
/* *
  * @brief  Abort a master I2C IT or DMA process communication with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_Master_Abort_IT(mut hi2c:
                                                     *mut I2C_HandleTypeDef,
                                                 mut DevAddress: uint16_t)
 -> HAL_StatusTypeDef {
    if (*hi2c).Mode as libc::c_uint ==
           HAL_I2C_MODE_MASTER as libc::c_int as libc::c_uint {
        /* Process Locked */
        if (*hi2c).Lock as libc::c_uint ==
               HAL_LOCKED as libc::c_int as libc::c_uint {
            return HAL_BUSY
        } else { (*hi2c).Lock = HAL_LOCKED }
        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        I2C_Disable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        /* Set State at HAL_I2C_STATE_ABORT */
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_ABORT);
        /* Set NBYTES to 1 to generate a dummy read on I2C peripheral */
    /* Set AUTOEND mode, this will generate a NACK then STOP condition to abort the current transfer */
        I2C_TransferConfig(hi2c, DevAddress, 1 as libc::c_int as uint8_t,
                           (0x1 as libc::c_uint) << 25 as libc::c_uint,
                           (0x1 as libc::c_uint) << 14 as libc::c_uint);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Note : The I2C interrupts must be enabled after unlocking current process 
              to avoid the risk of I2C interrupt handle execution before current
              process unlock */
        I2C_Enable_IRQ(hi2c, 0x12 as libc::c_uint as uint16_t);
        return HAL_OK
    } else {
        /* Wrong usage of abort function */
    /* This function should be used only in case of abort monitored by master device */
        return HAL_ERROR
    };
}
/* *
  * @}
  */
/* * @defgroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */
/* *
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_EV_IRQHandler(mut hi2c:
                                                   *mut I2C_HandleTypeDef) {
    /* Get current IT Flags and IT sources value */
    let mut itflags: uint32_t = (*(*hi2c).Instance).ISR;
    let mut itsources: uint32_t = (*(*hi2c).Instance).CR1;
    /* I2C events treatment -------------------------------------*/
    if (*hi2c).XferISR.is_some() {
        (*hi2c).XferISR.expect("non-null function pointer")(hi2c, itflags,
                                                            itsources);
    };
}
/* *
  * @brief  This function handles I2C error interrupt request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_ER_IRQHandler(mut hi2c:
                                                   *mut I2C_HandleTypeDef) {
    let mut itflags: uint32_t = (*(*hi2c).Instance).ISR;
    let mut itsources: uint32_t = (*(*hi2c).Instance).CR1;
    /* I2C Bus error interrupt occurred ------------------------------------*/
    if itflags & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           itsources & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Clear BERR flag */
        if (0x1 as libc::c_uint) << 8 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 8 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            8 as libc::c_uint)
        };
    }
    /* I2C Over-Run/Under-Run interrupt occurred ----------------------------------------*/
    if itflags & (0x1 as libc::c_uint) << 10 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           itsources & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Clear OVR flag */
        if (0x1 as libc::c_uint) << 10 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 10 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            10 as libc::c_uint)
        };
    }
    /* I2C Arbitration Loss error interrupt occurred -------------------------------------*/
    if itflags & (0x1 as libc::c_uint) << 9 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           itsources & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Clear ARLO flag */
        if (0x1 as libc::c_uint) << 9 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 9 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            9 as libc::c_uint)
        };
    }
    /* Call the Error Callback in case of Error detected */
    if (*hi2c).ErrorCode &
           (0x1 as libc::c_uint | 0x8 as libc::c_uint | 0x2 as libc::c_uint)
           != 0 as libc::c_uint {
        I2C_ITError(hi2c, (*hi2c).ErrorCode);
    };
}
/* *
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_MasterTxCpltCallback(mut hi2c:
                                                          *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterTxCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_MasterRxCpltCallback(mut hi2c:
                                                          *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MasterRxCpltCallback could be implemented in the user file
   */
}
/* * @brief  Slave Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_SlaveTxCpltCallback(mut hi2c:
                                                         *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_SlaveTxCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_SlaveRxCpltCallback(mut hi2c:
                                                         *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_SlaveRxCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XFERDIRECTION
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_AddrCallback(mut hi2c:
                                                  *mut I2C_HandleTypeDef,
                                              mut TransferDirection: uint8_t,
                                              mut AddrMatchCode: uint16_t) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_AddrCallback() could be implemented in the user file
   */
}
/* *
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_ListenCpltCallback(mut hi2c:
                                                        *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_ListenCpltCallback() could be implemented in the user file
   */
}
/* *
  * @brief  Memory Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_MemTxCpltCallback(mut hi2c:
                                                       *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MemTxCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  Memory Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_MemRxCpltCallback(mut hi2c:
                                                       *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_MemRxCpltCallback could be implemented in the user file
   */
}
/* *
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_ErrorCallback(mut hi2c:
                                                   *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_ErrorCallback could be implemented in the user file
   */
}
/* *
  * @brief  I2C abort callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_AbortCpltCallback(mut hi2c:
                                                       *mut I2C_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_I2C_AbortCpltCallback could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
 *  @brief   Peripheral State, Mode and Error functions
 *
@verbatim
 ===============================================================================
            ##### Peripheral State, Mode and Error functions #####
 ===============================================================================
    [..]
    This subsection permit to get in run-time the status of the peripheral
    and the data flow.

@endverbatim
  * @{
  */
/* *
  * @brief  Return the I2C handle state.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_GetState(mut hi2c: *mut I2C_HandleTypeDef)
 -> HAL_I2C_StateTypeDef {
    /* Return I2C handle state */
    return (*hi2c).State;
}
/* *
  * @brief  Returns the I2C Master, Slave, Memory or no mode.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval HAL mode
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_GetMode(mut hi2c: *mut I2C_HandleTypeDef)
 -> HAL_I2C_ModeTypeDef {
    return (*hi2c).Mode;
}
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
/* *
  * @}
  */
/* * @addtogroup I2C_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
/* IO operation functions  ****************************************************/
 /* ****** Blocking mode: Polling */
/* ****** Non-Blocking mode: Interrupt */
/* ****** Non-Blocking mode: DMA */
/* *
  * @}
  */
/* * @addtogroup I2C_IRQ_Handler_and_Callbacks IRQ Handler and Callbacks
 * @{
 */
/* ****** I2C IRQHandler and Callbacks used in non blocking modes (Interrupt and DMA) */
/* *
  * @}
  */
/* * @addtogroup I2C_Exported_Functions_Group3 Peripheral State, Mode and Error functions
  * @{
  */
/* Peripheral State, Mode and Error functions  *********************************/
/* *
* @brief  Return the I2C error code.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *              the configuration information for the specified I2C.
* @retval I2C Error Code
*/
#[no_mangle]
pub unsafe extern "C" fn HAL_I2C_GetError(mut hi2c: *mut I2C_HandleTypeDef)
 -> uint32_t {
    return (*hi2c).ErrorCode;
}
/* Private functions for I2C transfer IRQ handler */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup I2C_Private_Functions
  * @{
  */
/* *
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
unsafe extern "C" fn I2C_Master_ISR_IT(mut hi2c: *mut __I2C_HandleTypeDef,
                                       mut ITFlags: uint32_t,
                                       mut ITSources: uint32_t)
 -> HAL_StatusTypeDef {
    let mut devaddress: uint16_t = 0 as libc::c_uint as uint16_t;
    /* Process Locked */
    if (*hi2c).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hi2c).Lock = HAL_LOCKED }
    if ITFlags & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           ITSources & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Clear NACK Flag */
        if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            4 as libc::c_uint)
        };
        /* Set corresponding Error Code */
    /* No need to generate STOP, it is automatically done */
    /* Error callback will be send during stop flag treatment */
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);
    } else if ITFlags & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Read data from RXDR */
        let fresh8 = (*hi2c).pBuffPtr;
        (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
        *fresh8 = (*(*hi2c).Instance).RXDR as uint8_t;
        (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                               as
                                                                               *const uint16_t).wrapping_sub(1))
    } else if ITFlags & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Write data to TXDR */
        let fresh9 = (*hi2c).pBuffPtr;
        (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t, *fresh9 as uint32_t);
        (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                               as
                                                                               *const uint16_t).wrapping_sub(1))
    } else if ITFlags & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        if (*hi2c).XferSize as libc::c_uint == 0 as libc::c_uint &&
               (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
            devaddress =
                ((*(*hi2c).Instance).CR2 &
                     (0x3ff as libc::c_uint) << 0 as libc::c_uint) as
                    uint16_t;
            if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
                (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
                I2C_TransferConfig(hi2c, devaddress,
                                   (*hi2c).XferSize as uint8_t,
                                   (0x1 as libc::c_uint) <<
                                       24 as libc::c_uint, 0 as libc::c_uint);
            } else {
                (*hi2c).XferSize = (*hi2c).XferCount;
                if (*hi2c).XferOptions != 0xffff0000 as libc::c_uint {
                    I2C_TransferConfig(hi2c, devaddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (*hi2c).XferOptions,
                                       0 as libc::c_uint);
                } else {
                    I2C_TransferConfig(hi2c, devaddress,
                                       (*hi2c).XferSize as uint8_t,
                                       (0x1 as libc::c_uint) <<
                                           25 as libc::c_uint,
                                       0 as libc::c_uint);
                }
            }
        } else if (*(*hi2c).Instance).CR2 &
                      (0x1 as libc::c_uint) << 25 as libc::c_uint !=
                      (0x1 as libc::c_uint) << 25 as libc::c_uint {
            /* Call TxCpltCallback() if no stop mode is set */
            /* Call I2C Master Sequential complete process */
            I2C_ITMasterSequentialCplt(hi2c);
        } else {
            /* Wrong size Status regarding TCR flag event */
        /* Call the corresponding callback to inform upper layer of End of Transfer */
            I2C_ITError(hi2c, 0x40 as libc::c_uint);
        }
    } else if ITFlags & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        if (*hi2c).XferCount as libc::c_uint == 0 as libc::c_uint {
            if (*(*hi2c).Instance).CR2 &
                   (0x1 as libc::c_uint) << 25 as libc::c_uint !=
                   (0x1 as libc::c_uint) << 25 as libc::c_uint {
                /* Generate a stop condition in case of no transfer option */
                if (*hi2c).XferOptions == 0xffff0000 as libc::c_uint {
                    /* Generate Stop */
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         14 as libc::c_uint)
                                                    as uint32_t as uint32_t)
                } else {
                    /* Call I2C Master Sequential complete process */
                    I2C_ITMasterSequentialCplt(hi2c);
                }
            }
        } else {
            /* Wrong size Status regarding TC flag event */
      /* Call the corresponding callback to inform upper layer of End of Transfer */
            I2C_ITError(hi2c, 0x40 as libc::c_uint);
        }
    }
    if ITFlags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           ITSources & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Call I2C Master complete process */
        I2C_ITMasterCplt(hi2c, ITFlags);
    }
    /* Process Unlocked */
    (*hi2c).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with Interrupt.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
unsafe extern "C" fn I2C_Slave_ISR_IT(mut hi2c: *mut __I2C_HandleTypeDef,
                                      mut ITFlags: uint32_t,
                                      mut ITSources: uint32_t)
 -> HAL_StatusTypeDef {
    /* Process locked */
    if (*hi2c).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hi2c).Lock = HAL_LOCKED }
    if ITFlags & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           ITSources & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean XferCount == 0*/
    /* So clear Flag NACKF only */
        if (*hi2c).XferCount as libc::c_uint == 0 as libc::c_uint {
            if ((*hi2c).XferOptions ==
                    (0x1 as libc::c_uint) << 25 as libc::c_uint ||
                    (*hi2c).XferOptions ==
                        (0x1 as libc::c_uint) << 25 as libc::c_uint) &&
                   (*hi2c).State as libc::c_uint ==
                       HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
                /* Call I2C Listen complete process */
                I2C_ITListenCplt(hi2c, ITFlags);
            } else if (*hi2c).XferOptions != 0xffff0000 as libc::c_uint &&
                          (*hi2c).State as libc::c_uint ==
                              HAL_I2C_STATE_BUSY_TX_LISTEN as libc::c_int as
                                  libc::c_uint {
                /* Clear NACK Flag */
                if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         4 as libc::c_uint) as
                                                    uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                    as *mut uint32_t,
                                                (0x1 as libc::c_uint) <<
                                                    4 as libc::c_uint)
                };
                /* Flush TX register */
                I2C_Flush_TXDR(hi2c);
                /* Last Byte is Transmitted */
        /* Call I2C Slave Sequential complete process */
                I2C_ITSlaveSequentialCplt(hi2c);
            } else {
                /* Clear NACK Flag */
                if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         4 as libc::c_uint) as
                                                    uint32_t as uint32_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                    as *mut uint32_t,
                                                (0x1 as libc::c_uint) <<
                                                    4 as libc::c_uint)
                };
            }
        } else {
            /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
      /* Clear NACK Flag */
            if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     4 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                4 as libc::c_uint)
            };
            /* Set ErrorCode corresponding to a Non-Acknowledge */
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    } else if ITFlags & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        if (*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint {
            /* Read data from RXDR */
            let fresh10 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            *fresh10 = (*(*hi2c).Instance).RXDR as uint8_t;
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1))
        }
        if (*hi2c).XferCount as libc::c_uint == 0 as libc::c_uint &&
               (*hi2c).XferOptions != 0xffff0000 as libc::c_uint {
            /* Call I2C Slave Sequential complete process */
            I2C_ITSlaveSequentialCplt(hi2c);
        }
    } else if ITFlags & (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        I2C_ITAddrCplt(hi2c, ITFlags);
    } else if ITFlags & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Write data to TXDR only if XferCount not reach "0" */
    /* A TXIS flag can be set, during STOP treatment      */
    /* Check if all Datas have already been sent */
    /* If it is the case, this last write in TXDR is not sent, correspond to a dummy TXIS event */
        if (*hi2c).XferCount as libc::c_uint > 0 as libc::c_uint {
            /* Write data to TXDR */
            let fresh11 = (*hi2c).pBuffPtr;
            (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                            *mut uint32_t,
                                        *fresh11 as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1)
        } else if (*hi2c).XferOptions ==
                      (0x1 as libc::c_uint) << 24 as libc::c_uint |
                          0 as libc::c_uint ||
                      (*hi2c).XferOptions == 0 as libc::c_uint {
            /* Last Byte is Transmitted */
        /* Call I2C Slave Sequential complete process */
            I2C_ITSlaveSequentialCplt(hi2c);
        }
    }
    /* Check if STOPF is set */
    if ITFlags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           ITSources & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Call I2C Slave complete process */
        I2C_ITSlaveCplt(hi2c, ITFlags);
    }
    /* Process Unlocked */
    (*hi2c).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Master Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
unsafe extern "C" fn I2C_Master_ISR_DMA(mut hi2c: *mut __I2C_HandleTypeDef,
                                        mut ITFlags: uint32_t,
                                        mut ITSources: uint32_t)
 -> HAL_StatusTypeDef {
    let mut devaddress: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut xfermode: uint32_t = 0 as libc::c_uint;
    /* Process Locked */
    if (*hi2c).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hi2c).Lock = HAL_LOCKED }
    if ITFlags & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           ITSources & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Clear NACK Flag */
        if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            4 as libc::c_uint)
        };
        /* Set corresponding Error Code */
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* No need to generate STOP, it is automatically done */
    /* But enable STOP interrupt, to treat it */
    /* Error callback will be send during stop flag treatment */
        I2C_Enable_IRQ(hi2c, 0x12 as libc::c_uint as uint16_t);
        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);
    } else if ITFlags & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Disable TC interrupt */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
            /* Recover Slave address */
            devaddress =
                ((*(*hi2c).Instance).CR2 &
                     (0x3ff as libc::c_uint) << 0 as libc::c_uint) as
                    uint16_t;
            /* Prepare the new XferSize to transfer */
            if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
                (*hi2c).XferSize = 255 as libc::c_uint as uint16_t;
                xfermode = (0x1 as libc::c_uint) << 24 as libc::c_uint
            } else {
                (*hi2c).XferSize = (*hi2c).XferCount;
                xfermode = (0x1 as libc::c_uint) << 25 as libc::c_uint
            }
            /* Set the new XferSize in Nbytes register */
            I2C_TransferConfig(hi2c, devaddress, (*hi2c).XferSize as uint8_t,
                               xfermode, 0 as libc::c_uint);
            /* Update XferCount value */
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        (::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int -
                                             (*hi2c).XferSize as libc::c_int)
                                            as uint16_t as uint16_t);
            /* Enable DMA Request */
            if (*hi2c).State as libc::c_uint ==
                   HAL_I2C_STATE_BUSY_RX as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     15 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     14 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
        } else {
            /* Wrong size Status regarding TCR flag event */
      /* Call the corresponding callback to inform upper layer of End of Transfer */
            I2C_ITError(hi2c, 0x40 as libc::c_uint);
        }
    } else if ITFlags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Call I2C Master complete process */
        I2C_ITMasterCplt(hi2c, ITFlags);
    }
    /* Process Unlocked */
    (*hi2c).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Interrupt Sub-Routine which handle the Interrupt Flags Slave Mode with DMA.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  ITFlags Interrupt flags to handle.
  * @param  ITSources Interrupt sources enabled.
  * @retval HAL status
  */
unsafe extern "C" fn I2C_Slave_ISR_DMA(mut hi2c: *mut __I2C_HandleTypeDef,
                                       mut ITFlags: uint32_t,
                                       mut ITSources: uint32_t)
 -> HAL_StatusTypeDef {
    /* Process locked */
    if (*hi2c).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hi2c).Lock = HAL_LOCKED }
    if ITFlags & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           ITSources & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Check that I2C transfer finished */
    /* if yes, normal use case, a NACK is sent by the MASTER when Transfer is finished */
    /* Mean XferCount == 0 */
    /* So clear Flag NACKF only */
        if (if (*hi2c).State as libc::c_uint ==
                   HAL_I2C_STATE_BUSY_TX as libc::c_int as libc::c_uint {
                (*(*(*hi2c).hdmatx).Instance).NDTR
            } else { (*(*(*hi2c).hdmarx).Instance).NDTR }) ==
               0 as libc::c_uint {
            /* Clear NACK Flag */
            if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     4 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                4 as libc::c_uint)
            };
        } else {
            /* if no, error use case, a Non-Acknowledge of last Data is generated by the MASTER*/
      /* Clear NACK Flag */
            if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     4 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                4 as libc::c_uint)
            };
            /* Set ErrorCode corresponding to a Non-Acknowledge */
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    } else if ITFlags & (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Clear ADDR flag */
        if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint)
        };
    } else if ITFlags & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint &&
                  ITSources & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
                      RESET as libc::c_int as libc::c_uint {
        /* Call I2C Slave complete process */
        I2C_ITSlaveCplt(hi2c, ITFlags);
    }
    /* Process Unlocked */
    (*hi2c).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* Private functions to handle IT transfer */
/* *
  * @brief  Master sends target device address followed by internal memory address for write request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_RequestMemoryWrite(mut hi2c: *mut I2C_HandleTypeDef,
                                            mut DevAddress: uint16_t,
                                            mut MemAddress: uint16_t,
                                            mut MemAddSize: uint16_t,
                                            mut Timeout: uint32_t,
                                            mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    I2C_TransferConfig(hi2c, DevAddress, MemAddSize as uint8_t,
                       (0x1 as libc::c_uint) << 24 as libc::c_uint,
                       (0x1 as libc::c_uint) << 13 as libc::c_uint);
    /* Wait until TXIS flag is set */
    if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
            return HAL_ERROR
        } else { return HAL_TIMEOUT }
    }
    /* If Memory address size is 8Bit */
    if MemAddSize as libc::c_uint == 0x1 as libc::c_uint {
        /* Send Memory Address */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t,
                                    (MemAddress as libc::c_int &
                                         0xff as libc::c_uint as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint8_t as uint32_t)
    } else {
        /* If Memory address size is 16Bit */
        /* Send MSB of Memory Address */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t,
                                    ((MemAddress as libc::c_int &
                                          0xff00 as libc::c_uint as uint16_t
                                              as libc::c_int) as uint16_t as
                                         libc::c_int >> 8 as libc::c_uint) as
                                        uint16_t as uint8_t as uint32_t);
        if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t,
                                    (MemAddress as libc::c_int &
                                         0xff as libc::c_uint as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint8_t as uint32_t)
    }
    /* Wait until TXIS flag is set */
    /* Send LSB of Memory Address */
    /* Wait until TCR flag is set */
    if I2C_WaitOnFlagUntilTimeout(hi2c,
                                  (0x1 as libc::c_uint) << 7 as libc::c_uint,
                                  RESET, Timeout, Tickstart) as libc::c_uint
           != HAL_OK as libc::c_int as libc::c_uint {
        return HAL_TIMEOUT
    }
    return HAL_OK;
}
/* *
  * @brief  Master sends target device address followed by internal memory address for read request.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shift at right before call interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_RequestMemoryRead(mut hi2c: *mut I2C_HandleTypeDef,
                                           mut DevAddress: uint16_t,
                                           mut MemAddress: uint16_t,
                                           mut MemAddSize: uint16_t,
                                           mut Timeout: uint32_t,
                                           mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    I2C_TransferConfig(hi2c, DevAddress, MemAddSize as uint8_t,
                       0 as libc::c_uint,
                       (0x1 as libc::c_uint) << 13 as libc::c_uint);
    /* Wait until TXIS flag is set */
    if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
            return HAL_ERROR
        } else { return HAL_TIMEOUT }
    }
    /* If Memory address size is 8Bit */
    if MemAddSize as libc::c_uint == 0x1 as libc::c_uint {
        /* Send Memory Address */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t,
                                    (MemAddress as libc::c_int &
                                         0xff as libc::c_uint as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint8_t as uint32_t)
    } else {
        /* If Memory address size is 16Bit */
        /* Send MSB of Memory Address */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t,
                                    ((MemAddress as libc::c_int &
                                          0xff00 as libc::c_uint as uint16_t
                                              as libc::c_int) as uint16_t as
                                         libc::c_int >> 8 as libc::c_uint) as
                                        uint16_t as uint8_t as uint32_t);
        if I2C_WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            if (*hi2c).ErrorCode == 0x4 as libc::c_uint {
                return HAL_ERROR
            } else { return HAL_TIMEOUT }
        }
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t,
                                    (MemAddress as libc::c_int &
                                         0xff as libc::c_uint as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint8_t as uint32_t)
    }
    /* Wait until TXIS flag is set */
    /* Send LSB of Memory Address */
    /* Wait until TC flag is set */
    if I2C_WaitOnFlagUntilTimeout(hi2c,
                                  (0x1 as libc::c_uint) << 6 as libc::c_uint,
                                  RESET, Timeout, Tickstart) as libc::c_uint
           != HAL_OK as libc::c_int as libc::c_uint {
        return HAL_TIMEOUT
    }
    return HAL_OK;
}
/* Private functions to handle IT transfer */
/* *
  * @brief  I2C Address complete process callback.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITAddrCplt(mut hi2c: *mut I2C_HandleTypeDef,
                                    mut ITFlags: uint32_t) {
    let mut transferdirection: uint8_t = 0 as libc::c_uint as uint8_t;
    let mut slaveaddrcode: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut ownadd1code: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut ownadd2code: uint16_t = 0 as libc::c_uint as uint16_t;
    /* Prevent unused argument(s) compilation warning */
    /* In case of Listen state, need to inform upper layer of address match code event */
    if (*hi2c).State as libc::c_uint &
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint ==
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
        transferdirection =
            (((*(*hi2c).Instance).ISR &
                  (0x1 as libc::c_uint) << 16 as libc::c_uint) >>
                 16 as libc::c_uint) as uint8_t;
        slaveaddrcode =
            (((*(*hi2c).Instance).ISR &
                  (0x7f as libc::c_uint) << 17 as libc::c_uint) >>
                 16 as libc::c_uint) as uint16_t;
        ownadd1code =
            ((*(*hi2c).Instance).OAR1 &
                 (0x3ff as libc::c_uint) << 0 as libc::c_uint) as uint16_t;
        ownadd2code =
            ((*(*hi2c).Instance).OAR2 &
                 (0x7f as libc::c_uint) << 1 as libc::c_uint) as uint16_t;
        /* If 10bits addressing mode is selected */
        if (*hi2c).Init.AddressingMode == 0x2 as libc::c_uint {
            if slaveaddrcode as libc::c_uint & 0x6 as libc::c_uint ==
                   (ownadd1code as libc::c_int >> 7 as libc::c_uint) as
                       libc::c_uint & 0x6 as libc::c_uint {
                slaveaddrcode = ownadd1code;
                ::core::ptr::write_volatile(&mut (*hi2c).AddrEventCount as
                                                *mut uint32_t,
                                            ::core::ptr::read_volatile::<uint32_t>(&(*hi2c).AddrEventCount
                                                                                       as
                                                                                       *const uint32_t).wrapping_add(1));
                if (*hi2c).AddrEventCount == 2 as libc::c_uint {
                    /* Reset Address Event counter */
                    ::core::ptr::write_volatile(&mut (*hi2c).AddrEventCount as
                                                    *mut uint32_t,
                                                0 as libc::c_uint);
                    /* Clear ADDR flag */
                    if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 0 as libc::c_uint {
                        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR
                                                        as *mut uint32_t,
                                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                                as
                                                                                                *const uint32_t)
                                                         as libc::c_uint |
                                                         (0x1 as libc::c_uint)
                                                             <<
                                                             3 as
                                                                 libc::c_uint)
                                                        as uint32_t as
                                                        uint32_t)
                    } else {
                        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR
                                                        as *mut uint32_t,
                                                    (0x1 as libc::c_uint) <<
                                                        3 as libc::c_uint)
                    };
                    /* Process Unlocked */
                    (*hi2c).Lock = HAL_UNLOCKED;
                    /* Call Slave Addr callback */
                    HAL_I2C_AddrCallback(hi2c, transferdirection,
                                         slaveaddrcode);
                }
            } else {
                slaveaddrcode = ownadd2code;
                /* Disable ADDR Interrupts */
                I2C_Disable_IRQ(hi2c, 0x4 as libc::c_uint as uint16_t);
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                /* Call Slave Addr callback */
                HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
            }
        } else {
            /* else 7 bits addressing mode is selected */
            /* Disable ADDR Interrupts */
            I2C_Disable_IRQ(hi2c, 0x4 as libc::c_uint as uint16_t);
            (*hi2c).Lock = HAL_UNLOCKED;
            HAL_I2C_AddrCallback(hi2c, transferdirection, slaveaddrcode);
        }
    } else {
        /* Process Unlocked */
        /* Call Slave Addr callback */
        /* Else clear address flag only */
        /* Clear ADDR flag */
        if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            3 as libc::c_uint)
        };
        (*hi2c).Lock = HAL_UNLOCKED
    };
}
/* Process Unlocked */
/* *
  * @brief  I2C Master sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITMasterSequentialCplt(mut hi2c:
                                                    *mut I2C_HandleTypeDef) {
    /* Reset I2C handle mode */
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    /* No Generate Stop, to permit restart mode */
  /* The stop will be done at the end of transfer, when I2C_AUTOEND_MODE enable */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    HAL_I2C_STATE_BUSY_TX as libc::c_int as
                                        libc::c_uint &
                                        ((HAL_I2C_STATE_BUSY_TX as libc::c_int
                                              |
                                              HAL_I2C_STATE_BUSY_RX as
                                                  libc::c_int) as libc::c_uint
                                             &
                                             !(HAL_I2C_STATE_READY as
                                                   libc::c_int as uint32_t)) |
                                        HAL_I2C_MODE_MASTER as libc::c_int as
                                            libc::c_uint);
        (*hi2c).XferISR = None;
        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_MasterTxCpltCallback(hi2c);
    } else {
        /* hi2c->State == HAL_I2C_STATE_BUSY_RX */
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    HAL_I2C_STATE_BUSY_RX as libc::c_int as
                                        libc::c_uint &
                                        ((HAL_I2C_STATE_BUSY_TX as libc::c_int
                                              |
                                              HAL_I2C_STATE_BUSY_RX as
                                                  libc::c_int) as libc::c_uint
                                             &
                                             !(HAL_I2C_STATE_READY as
                                                   libc::c_int as uint32_t)) |
                                        HAL_I2C_MODE_MASTER as libc::c_int as
                                            libc::c_uint);
        (*hi2c).XferISR = None;
        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_MasterRxCpltCallback(hi2c);
    };
}
/* *
  * @brief  I2C Slave sequential complete process.
  * @param  hi2c I2C handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITSlaveSequentialCplt(mut hi2c:
                                                   *mut I2C_HandleTypeDef) {
    /* Reset I2C handle mode */
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_BUSY_TX_LISTEN as libc::c_int as libc::c_uint {
        /* Remove HAL_I2C_STATE_SLAVE_BUSY_TX, keep only HAL_I2C_STATE_LISTEN */
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_LISTEN);
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    HAL_I2C_STATE_BUSY_TX as libc::c_int as
                                        libc::c_uint &
                                        ((HAL_I2C_STATE_BUSY_TX as libc::c_int
                                              |
                                              HAL_I2C_STATE_BUSY_RX as
                                                  libc::c_int) as libc::c_uint
                                             &
                                             !(HAL_I2C_STATE_READY as
                                                   libc::c_int as uint32_t)) |
                                        HAL_I2C_MODE_SLAVE as libc::c_int as
                                            libc::c_uint);
        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, 0x1 as libc::c_uint as uint16_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the Tx complete callback to inform upper layer of the end of transmit process */
        HAL_I2C_SlaveTxCpltCallback(hi2c);
    } else if (*hi2c).State as libc::c_uint ==
                  HAL_I2C_STATE_BUSY_RX_LISTEN as libc::c_int as libc::c_uint
     {
        /* Remove HAL_I2C_STATE_SLAVE_BUSY_RX, keep only HAL_I2C_STATE_LISTEN */
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_LISTEN);
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    HAL_I2C_STATE_BUSY_RX as libc::c_int as
                                        libc::c_uint &
                                        ((HAL_I2C_STATE_BUSY_TX as libc::c_int
                                              |
                                              HAL_I2C_STATE_BUSY_RX as
                                                  libc::c_int) as libc::c_uint
                                             &
                                             !(HAL_I2C_STATE_READY as
                                                   libc::c_int as uint32_t)) |
                                        HAL_I2C_MODE_SLAVE as libc::c_int as
                                            libc::c_uint);
        /* Disable Interrupts */
        I2C_Disable_IRQ(hi2c, 0x2 as libc::c_uint as uint16_t);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the Rx complete callback to inform upper layer of the end of receive process */
        HAL_I2C_SlaveRxCpltCallback(hi2c);
    };
}
/* *
  * @brief  I2C Master complete process.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITMasterCplt(mut hi2c: *mut I2C_HandleTypeDef,
                                      mut ITFlags: uint32_t) {
    /* Clear STOP Flag */
    if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        5 as libc::c_uint)
    };
    /* Clear Configuration Register 2 */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3ff as libc::c_uint) <<
                                           0 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               12 as libc::c_uint |
                                           (0xff as libc::c_uint) <<
                                               16 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               24 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Reset handle parameters */
    ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as *mut uint32_t,
                                HAL_I2C_MODE_NONE as libc::c_int as uint32_t);
    (*hi2c).XferISR = None;
    ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                0xffff0000 as libc::c_uint);
    if ITFlags & (0x1 as libc::c_uint) << 4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Clear NACK Flag */
        if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            4 as libc::c_uint)
        };
        /* Set acknowledge error code */
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Flush TX register */
    I2C_Flush_TXDR(hi2c);
    /* Disable Interrupts */
    I2C_Disable_IRQ(hi2c,
                    (0x1 as libc::c_uint | 0x2 as libc::c_uint) as uint16_t);
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    if (*hi2c).ErrorCode != 0 as libc::c_uint ||
           (*hi2c).State as libc::c_uint ==
               HAL_I2C_STATE_ABORT as libc::c_int as libc::c_uint {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, (*hi2c).ErrorCode);
    } else if (*hi2c).State as libc::c_uint ==
                  HAL_I2C_STATE_BUSY_TX as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        if (*hi2c).Mode as libc::c_uint ==
               HAL_I2C_MODE_MEM as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                            *mut HAL_I2C_ModeTypeDef,
                                        HAL_I2C_MODE_NONE);
            /* hi2c->State == HAL_I2C_STATE_BUSY_TX */
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            /* Call the corresponding callback to inform upper layer of End of Transfer */
            HAL_I2C_MemTxCpltCallback(hi2c);
        } else {
            ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                            *mut HAL_I2C_ModeTypeDef,
                                        HAL_I2C_MODE_NONE);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            /* Call the corresponding callback to inform upper layer of End of Transfer */
            HAL_I2C_MasterTxCpltCallback(hi2c);
        }
    } else if (*hi2c).State as libc::c_uint ==
                  HAL_I2C_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        if (*hi2c).Mode as libc::c_uint ==
               HAL_I2C_MODE_MEM as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                            *mut HAL_I2C_ModeTypeDef,
                                        HAL_I2C_MODE_NONE);
            /* hi2c->State == HAL_I2C_STATE_BUSY_RX */
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            HAL_I2C_MemRxCpltCallback(hi2c);
        } else {
            ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                            *mut HAL_I2C_ModeTypeDef,
                                        HAL_I2C_MODE_NONE);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            HAL_I2C_MasterRxCpltCallback(hi2c);
        }
    };
}
/* *
  * @brief  I2C Slave complete process.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITSlaveCplt(mut hi2c: *mut I2C_HandleTypeDef,
                                     mut ITFlags: uint32_t) {
    /* Clear STOP Flag */
    if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        5 as libc::c_uint)
    };
    /* Clear ADDR flag */
    if (0x1 as libc::c_uint) << 3 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        3 as libc::c_uint)
    };
    /* Disable all interrupts */
    I2C_Disable_IRQ(hi2c,
                    (0x4 as libc::c_uint | 0x1 as libc::c_uint |
                         0x2 as libc::c_uint) as uint16_t);
    /* Disable Address Acknowledge */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Clear Configuration Register 2 */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x3ff as libc::c_uint) <<
                                           0 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               12 as libc::c_uint |
                                           (0xff as libc::c_uint) <<
                                               16 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               24 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               10 as libc::c_uint)) as
                                    uint32_t as uint32_t);
    /* Flush TX register */
    I2C_Flush_TXDR(hi2c);
    /* If a DMA is ongoing, Update handle size context */
    if (*(*hi2c).Instance).CR1 & (0x1 as libc::c_uint) << 14 as libc::c_uint
           == (0x1 as libc::c_uint) << 14 as libc::c_uint ||
           (*(*hi2c).Instance).CR1 &
               (0x1 as libc::c_uint) << 15 as libc::c_uint ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                    if (*hi2c).State as libc::c_uint ==
                                           HAL_I2C_STATE_BUSY_TX as
                                               libc::c_int as libc::c_uint {
                                        (*(*(*hi2c).hdmatx).Instance).NDTR
                                    } else {
                                        (*(*(*hi2c).hdmarx).Instance).NDTR
                                    } as uint16_t)
    }
    /* All data are not transferred, so set error code accordingly */
    if (*hi2c).XferCount as libc::c_uint != 0 as libc::c_uint {
        /* Set ErrorCode corresponding to a Non-Acknowledge */
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Store Last receive data if any */
    if ITFlags & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Read data from RXDR */
        let fresh12 = (*hi2c).pBuffPtr;
        (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
        *fresh12 = (*(*hi2c).Instance).RXDR as uint8_t;
        if (*hi2c).XferSize as libc::c_uint > 0 as libc::c_uint {
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            /* Set ErrorCode corresponding to a Non-Acknowledge */
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as *mut uint32_t,
                                HAL_I2C_MODE_NONE as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    (*hi2c).XferISR = None;
    if (*hi2c).ErrorCode != 0 as libc::c_uint {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        I2C_ITError(hi2c, (*hi2c).ErrorCode);
        /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
        if (*hi2c).State as libc::c_uint ==
               HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
            /* Call I2C Listen complete process */
            I2C_ITListenCplt(hi2c, ITFlags);
        }
    } else if (*hi2c).XferOptions != 0xffff0000 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                    0xffff0000 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
        HAL_I2C_ListenCpltCallback(hi2c);
    } else if (*hi2c).State as libc::c_uint ==
                  HAL_I2C_STATE_BUSY_RX as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the Slave Rx Complete callback */
        HAL_I2C_SlaveRxCpltCallback(hi2c);
    } else {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the Slave Tx Complete callback */
        HAL_I2C_SlaveTxCpltCallback(hi2c);
    };
}
/* *
  * @brief  I2C Listen complete process.
  * @param  hi2c I2C handle.
  * @param  ITFlags Interrupt flags to handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITListenCplt(mut hi2c: *mut I2C_HandleTypeDef,
                                      mut ITFlags: uint32_t) {
    /* Reset handle parameters */
    ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                0xffff0000 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as *mut uint32_t,
                                HAL_I2C_MODE_NONE as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*hi2c).State as
                                    *mut HAL_I2C_StateTypeDef,
                                HAL_I2C_STATE_READY);
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    (*hi2c).XferISR = None;
    /* Store Last receive data if any */
    if ITFlags & (0x1 as libc::c_uint) << 2 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Read data from RXDR */
        let fresh13 = (*hi2c).pBuffPtr;
        (*hi2c).pBuffPtr = (*hi2c).pBuffPtr.offset(1);
        *fresh13 = (*(*hi2c).Instance).RXDR as uint8_t;
        if (*hi2c).XferSize as libc::c_uint > 0 as libc::c_uint {
            (*hi2c).XferSize = (*hi2c).XferSize.wrapping_sub(1);
            ::core::ptr::write_volatile(&mut (*hi2c).XferCount as
                                            *mut uint16_t,
                                        ::core::ptr::read_volatile::<uint16_t>(&(*hi2c).XferCount
                                                                                   as
                                                                                   *const uint16_t).wrapping_sub(1));
            /* Set ErrorCode corresponding to a Non-Acknowledge */
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    /* Disable all Interrupts*/
    I2C_Disable_IRQ(hi2c,
                    (0x4 as libc::c_uint | 0x2 as libc::c_uint |
                         0x1 as libc::c_uint) as uint16_t);
    /* Clear NACK Flag */
    if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
           (0x1 as libc::c_uint) << 0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        4 as libc::c_uint)
    };
    /* Process Unlocked */
    (*hi2c).Lock = HAL_UNLOCKED;
    /* Call the Listen Complete callback, to inform upper layer of the end of Listen usecase */
    HAL_I2C_ListenCpltCallback(hi2c);
}
/* *
  * @brief  I2C interrupts error process.
  * @param  hi2c I2C handle.
  * @param  ErrorCode Error code to handle.
  * @retval None
  */
unsafe extern "C" fn I2C_ITError(mut hi2c: *mut I2C_HandleTypeDef,
                                 mut ErrorCode: uint32_t) {
    /* Reset handle parameters */
    ::core::ptr::write_volatile(&mut (*hi2c).Mode as *mut HAL_I2C_ModeTypeDef,
                                HAL_I2C_MODE_NONE);
    ::core::ptr::write_volatile(&mut (*hi2c).XferOptions as *mut uint32_t,
                                0xffff0000 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*hi2c).XferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    /* Set new error code */
    ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | ErrorCode) as uint32_t
                                    as uint32_t);
    /* Disable Interrupts */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint ||
           (*hi2c).State as libc::c_uint ==
               HAL_I2C_STATE_BUSY_TX_LISTEN as libc::c_int as libc::c_uint ||
           (*hi2c).State as libc::c_uint ==
               HAL_I2C_STATE_BUSY_RX_LISTEN as libc::c_int as libc::c_uint {
        /* Disable all interrupts, except interrupts related to LISTEN state */
        I2C_Disable_IRQ(hi2c,
                        (0x2 as libc::c_uint | 0x1 as libc::c_uint) as
                            uint16_t);
        /* keep HAL_I2C_STATE_LISTEN if set */
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_LISTEN);
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    HAL_I2C_MODE_NONE as libc::c_int as
                                        uint32_t);
        (*hi2c).XferISR =
            Some(I2C_Slave_ISR_IT as
                     unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                          _: uint32_t, _: uint32_t)
                         -> HAL_StatusTypeDef)
    } else {
        /* Disable all interrupts */
        I2C_Disable_IRQ(hi2c,
                        (0x4 as libc::c_uint | 0x2 as libc::c_uint |
                             0x1 as libc::c_uint) as uint16_t);
        /* If state is an abort treatment on goind, don't change state */
    /* This change will be do later */
        if (*hi2c).State as libc::c_uint !=
               HAL_I2C_STATE_ABORT as libc::c_int as libc::c_uint {
            /* Set HAL_I2C_STATE_READY */
            ::core::ptr::write_volatile(&mut (*hi2c).State as
                                            *mut HAL_I2C_StateTypeDef,
                                        HAL_I2C_STATE_READY)
        }
        ::core::ptr::write_volatile(&mut (*hi2c).PreviousState as
                                        *mut uint32_t,
                                    HAL_I2C_MODE_NONE as libc::c_int as
                                        uint32_t);
        (*hi2c).XferISR = None
    }
    /* Abort DMA TX transfer if any */
    if (*(*hi2c).Instance).CR1 & (0x1 as libc::c_uint) << 14 as libc::c_uint
           == (0x1 as libc::c_uint) << 14 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               14 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Set the I2C DMA Abort callback :
       will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        (*(*hi2c).hdmatx).XferAbortCallback =
            Some(I2C_DMAAbort as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Abort DMA TX */
        if HAL_DMA_Abort_IT((*hi2c).hdmatx) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Call Directly XferAbortCallback function in case of error */
            (*(*hi2c).hdmatx).XferAbortCallback.expect("non-null function pointer")((*hi2c).hdmatx);
        }
    } else if (*(*hi2c).Instance).CR1 &
                  (0x1 as libc::c_uint) << 15 as libc::c_uint ==
                  (0x1 as libc::c_uint) << 15 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               15 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Abort DMA RX transfer if any */
        /* Set the I2C DMA Abort callback :
       will lead to call HAL_I2C_ErrorCallback() at end of DMA abort procedure */
        (*(*hi2c).hdmarx).XferAbortCallback =
            Some(I2C_DMAAbort as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Abort DMA RX */
        if HAL_DMA_Abort_IT((*hi2c).hdmarx) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            /* Call Directly hi2c->hdmarx->XferAbortCallback function in case of error */
            (*(*hi2c).hdmarx).XferAbortCallback.expect("non-null function pointer")((*hi2c).hdmarx);
        }
    } else if (*hi2c).State as libc::c_uint ==
                  HAL_I2C_STATE_ABORT as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_AbortCpltCallback(hi2c);
    } else {
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_ErrorCallback(hi2c);
    };
}
/* Private functions to flush TXDR register */
/* *
  * @brief  I2C Tx data register flush process.
  * @param  hi2c I2C handle.
  * @retval None
  */
unsafe extern "C" fn I2C_Flush_TXDR(mut hi2c: *mut I2C_HandleTypeDef) {
    /* If a pending TXIS flag is set */
  /* Write a dummy data in TXDR to clear it */
    if (if (*(*hi2c).Instance).ISR &
               (0x1 as libc::c_uint) << 1 as libc::c_uint ==
               (0x1 as libc::c_uint) << 1 as libc::c_uint {
            SET as libc::c_int
        } else { RESET as libc::c_int }) != RESET as libc::c_int {
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).TXDR as
                                        *mut uint32_t, 0 as libc::c_uint)
    }
    /* Flush TX register if not empty */
    if (if (*(*hi2c).Instance).ISR &
               (0x1 as libc::c_uint) << 0 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            SET as libc::c_int
        } else { RESET as libc::c_int }) == RESET as libc::c_int {
        if (0x1 as libc::c_uint) << 0 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            0 as libc::c_uint)
        };
    };
}
/* *
  * @}
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* * @defgroup I2C_Private_Functions I2C Private Functions
  * @{
  */
/* Private functions to handle DMA transfer */
/* *
  * @brief  DMA I2C master transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
unsafe extern "C" fn I2C_DMAMasterTransmitCplt(mut hdma:
                                                   *mut DMA_HandleTypeDef) {
    let mut hi2c: *mut I2C_HandleTypeDef =
        (*hdma).Parent as *mut I2C_HandleTypeDef;
    /* Disable DMA Request */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           14 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* If last transfer, enable STOP interrupt */
    if (*hi2c).XferCount as libc::c_uint == 0 as libc::c_uint {
        /* Enable STOP interrupt */
        I2C_Enable_IRQ(hi2c, 0x12 as libc::c_uint as uint16_t);
    } else {
        /* else prepare a new DMA transfer and enable TCReload interrupt */
        /* Update Buffer pointer */
        (*hi2c).pBuffPtr =
            (*hi2c).pBuffPtr.offset((*hi2c).XferSize as libc::c_int as isize);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t
        } else { (*hi2c).XferSize = (*hi2c).XferCount }
        HAL_DMA_Start_IT((*hi2c).hdmatx, (*hi2c).pBuffPtr as uint32_t,
                         &mut (*(*hi2c).Instance).TXDR as *mut uint32_t as
                             uint32_t, (*hi2c).XferSize as uint32_t);
        I2C_Enable_IRQ(hi2c, 0x12 as libc::c_uint as uint16_t);
    };
}
/* Set the XferSize to transfer */
/* Enable the DMA channel */
/* Enable TC interrupts */
/* *
  * @brief  DMA I2C slave transmit process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
unsafe extern "C" fn I2C_DMASlaveTransmitCplt(mut hdma:
                                                  *mut DMA_HandleTypeDef) {
    /* No specific action, Master fully manage the generation of STOP condition */
  /* Mean that this generation can arrive at any time, at the end or during DMA process */
  /* So STOP condition should be manage through Interrupt treatment */
}
/* *
  * @brief DMA I2C master receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
unsafe extern "C" fn I2C_DMAMasterReceiveCplt(mut hdma:
                                                  *mut DMA_HandleTypeDef) {
    let mut hi2c: *mut I2C_HandleTypeDef =
        (*hdma).Parent as *mut I2C_HandleTypeDef;
    /* Disable DMA Request */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           15 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* If last transfer, enable STOP interrupt */
    if (*hi2c).XferCount as libc::c_uint == 0 as libc::c_uint {
        /* Enable STOP interrupt */
        I2C_Enable_IRQ(hi2c, 0x12 as libc::c_uint as uint16_t);
    } else {
        /* else prepare a new DMA transfer and enable TCReload interrupt */
        /* Update Buffer pointer */
        (*hi2c).pBuffPtr =
            (*hi2c).pBuffPtr.offset((*hi2c).XferSize as libc::c_int as isize);
        if (*hi2c).XferCount as libc::c_uint > 255 as libc::c_uint {
            (*hi2c).XferSize = 255 as libc::c_uint as uint16_t
        } else { (*hi2c).XferSize = (*hi2c).XferCount }
        HAL_DMA_Start_IT((*hi2c).hdmarx,
                         &mut (*(*hi2c).Instance).RXDR as *mut uint32_t as
                             uint32_t, (*hi2c).pBuffPtr as uint32_t,
                         (*hi2c).XferSize as uint32_t);
        I2C_Enable_IRQ(hi2c, 0x12 as libc::c_uint as uint16_t);
    };
}
/* Set the XferSize to transfer */
/* Enable the DMA channel */
/* Enable TC interrupts */
/* *
  * @brief  DMA I2C slave receive process complete callback.
  * @param  hdma DMA handle
  * @retval None
  */
unsafe extern "C" fn I2C_DMASlaveReceiveCplt(mut hdma:
                                                 *mut DMA_HandleTypeDef) {
    /* No specific action, Master fully manage the generation of STOP condition */
  /* Mean that this generation can arrive at any time, at the end or during DMA process */
  /* So STOP condition should be manage through Interrupt treatment */
}
/* *
  * @brief  DMA I2C communication error callback.
  * @param hdma DMA handle
  * @retval None
  */
unsafe extern "C" fn I2C_DMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hi2c: *mut I2C_HandleTypeDef =
        (*hdma).Parent as *mut I2C_HandleTypeDef;
    /* Disable Acknowledge */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Call the corresponding callback to inform upper layer of End of Transfer */
    I2C_ITError(hi2c, 0x10 as libc::c_uint);
}
/* *
  * @brief DMA I2C communication abort callback
  *        (To be called at end of DMA Abort procedure).
  * @param hdma: DMA handle.
  * @retval None
  */
unsafe extern "C" fn I2C_DMAAbort(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hi2c: *mut I2C_HandleTypeDef =
        (*hdma).Parent as *mut I2C_HandleTypeDef;
    /* Disable Acknowledge */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         15 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Reset AbortCpltCallback */
    (*(*hi2c).hdmatx).XferAbortCallback = None;
    (*(*hi2c).hdmarx).XferAbortCallback = None;
    /* Check if come from abort from user */
    if (*hi2c).State as libc::c_uint ==
           HAL_I2C_STATE_ABORT as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_AbortCpltCallback(hi2c);
    } else {
        /* Call the corresponding callback to inform upper layer of End of Transfer */
        HAL_I2C_ErrorCallback(hi2c);
    };
}
/* Private functions to handle flags during polling transfer */
/* *
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Flag Specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_WaitOnFlagUntilTimeout(mut hi2c:
                                                    *mut I2C_HandleTypeDef,
                                                mut Flag: uint32_t,
                                                mut Status: FlagStatus,
                                                mut Timeout: uint32_t,
                                                mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    while (if (*(*hi2c).Instance).ISR & Flag == Flag {
               SET as libc::c_int
           } else { RESET as libc::c_int }) as libc::c_uint ==
              Status as libc::c_uint {
        /* Check for the Timeout */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(Tickstart) > Timeout {
                ::core::ptr::write_volatile(&mut (*hi2c).State as
                                                *mut HAL_I2C_StateTypeDef,
                                            HAL_I2C_STATE_READY);
                ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                                *mut HAL_I2C_ModeTypeDef,
                                            HAL_I2C_MODE_NONE);
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @brief  This function handles I2C Communication Timeout for specific usage of TXIS flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_WaitOnTXISFlagUntilTimeout(mut hi2c:
                                                        *mut I2C_HandleTypeDef,
                                                    mut Timeout: uint32_t,
                                                    mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    while (if (*(*hi2c).Instance).ISR &
                  (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                  (0x1 as libc::c_uint) << 1 as libc::c_uint {
               SET as libc::c_int
           } else { RESET as libc::c_int }) == RESET as libc::c_int {
        /* Check if a NACK is detected */
        if I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) as libc::c_uint
               != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_ERROR
        }
        /* Check for the Timeout */
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(Tickstart) > Timeout {
                ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x20 as libc::c_uint) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*hi2c).State as
                                                *mut HAL_I2C_StateTypeDef,
                                            HAL_I2C_STATE_READY);
                ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                                *mut HAL_I2C_ModeTypeDef,
                                            HAL_I2C_MODE_NONE);
                /* Process Unlocked */
                (*hi2c).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @brief  This function handles I2C Communication Timeout for specific usage of STOP flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_WaitOnSTOPFlagUntilTimeout(mut hi2c:
                                                        *mut I2C_HandleTypeDef,
                                                    mut Timeout: uint32_t,
                                                    mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    while (if (*(*hi2c).Instance).ISR &
                  (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                  (0x1 as libc::c_uint) << 5 as libc::c_uint {
               SET as libc::c_int
           } else { RESET as libc::c_int }) == RESET as libc::c_int {
        /* Check if a NACK is detected */
        if I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) as libc::c_uint
               != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_ERROR
        }
        /* Check for the Timeout */
        if Timeout == 0 as libc::c_uint ||
               HAL_GetTick().wrapping_sub(Tickstart) > Timeout {
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x20 as libc::c_uint) as uint32_t
                                            as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).State as
                                            *mut HAL_I2C_StateTypeDef,
                                        HAL_I2C_STATE_READY);
            ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                            *mut HAL_I2C_ModeTypeDef,
                                        HAL_I2C_MODE_NONE);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief  This function handles I2C Communication Timeout for specific usage of RXNE flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_WaitOnRXNEFlagUntilTimeout(mut hi2c:
                                                        *mut I2C_HandleTypeDef,
                                                    mut Timeout: uint32_t,
                                                    mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    while (if (*(*hi2c).Instance).ISR &
                  (0x1 as libc::c_uint) << 2 as libc::c_uint ==
                  (0x1 as libc::c_uint) << 2 as libc::c_uint {
               SET as libc::c_int
           } else { RESET as libc::c_int }) == RESET as libc::c_int {
        /* Check if a NACK is detected */
        if I2C_IsAcknowledgeFailed(hi2c, Timeout, Tickstart) as libc::c_uint
               != HAL_OK as libc::c_int as libc::c_uint {
            return HAL_ERROR
        }
        /* Check if a STOPF is detected */
        if (if (*(*hi2c).Instance).ISR &
                   (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 5 as libc::c_uint {
                SET as libc::c_int
            } else { RESET as libc::c_int }) == SET as libc::c_int {
            /* Clear STOP Flag */
            if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                   (0x1 as libc::c_uint) << 0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     5 as libc::c_uint) as
                                                uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                                *mut uint32_t,
                                            (0x1 as libc::c_uint) <<
                                                5 as libc::c_uint)
            };
            /* Clear Configuration Register 2 */
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x3ff as libc::c_uint) <<
                                                   0 as libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       12 as libc::c_uint |
                                                   (0xff as libc::c_uint) <<
                                                       16 as libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       24 as libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       10 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*hi2c).State as
                                            *mut HAL_I2C_StateTypeDef,
                                        HAL_I2C_STATE_READY);
            ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                            *mut HAL_I2C_ModeTypeDef,
                                        HAL_I2C_MODE_NONE);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            return HAL_ERROR
        }
        /* Check for the Timeout */
        if Timeout == 0 as libc::c_uint ||
               HAL_GetTick().wrapping_sub(Tickstart) > Timeout {
            ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hi2c).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x20 as libc::c_uint) as uint32_t
                                            as uint32_t);
            ::core::ptr::write_volatile(&mut (*hi2c).State as
                                            *mut HAL_I2C_StateTypeDef,
                                        HAL_I2C_STATE_READY);
            /* Process Unlocked */
            (*hi2c).Lock = HAL_UNLOCKED;
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
unsafe extern "C" fn I2C_IsAcknowledgeFailed(mut hi2c: *mut I2C_HandleTypeDef,
                                             mut Timeout: uint32_t,
                                             mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    if (if (*(*hi2c).Instance).ISR &
               (0x1 as libc::c_uint) << 4 as libc::c_uint ==
               (0x1 as libc::c_uint) << 4 as libc::c_uint {
            SET as libc::c_int
        } else { RESET as libc::c_int }) == SET as libc::c_int {
        /* Wait until STOP Flag is reset */
    /* AutoEnd should be initiate after AF */
        while (if (*(*hi2c).Instance).ISR &
                      (0x1 as libc::c_uint) << 5 as libc::c_uint ==
                      (0x1 as libc::c_uint) << 5 as libc::c_uint {
                   SET as libc::c_int
               } else { RESET as libc::c_int }) == RESET as libc::c_int {
            /* Check for the Timeout */
            if Timeout != 0xffffffff as libc::c_uint {
                if Timeout == 0 as libc::c_uint ||
                       HAL_GetTick().wrapping_sub(Tickstart) > Timeout {
                    ::core::ptr::write_volatile(&mut (*hi2c).State as
                                                    *mut HAL_I2C_StateTypeDef,
                                                HAL_I2C_STATE_READY);
                    ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                                    *mut HAL_I2C_ModeTypeDef,
                                                HAL_I2C_MODE_NONE);
                    /* Process Unlocked */
                    (*hi2c).Lock = HAL_UNLOCKED;
                    return HAL_TIMEOUT
                }
            }
        }
        /* Clear NACKF Flag */
        if (0x1 as libc::c_uint) << 4 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 4 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            4 as libc::c_uint)
        };
        /* Clear STOP Flag */
        if (0x1 as libc::c_uint) << 5 as libc::c_uint ==
               (0x1 as libc::c_uint) << 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ISR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).ISR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).ICR as
                                            *mut uint32_t,
                                        (0x1 as libc::c_uint) <<
                                            5 as libc::c_uint)
        };
        /* Flush TX register */
        I2C_Flush_TXDR(hi2c);
        /* Clear Configuration Register 2 */
        ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x3ff as libc::c_uint) <<
                                               0 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint |
                                               (0xff as libc::c_uint) <<
                                                   16 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   24 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   10 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*hi2c).ErrorCode as *mut uint32_t,
                                    0x4 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*hi2c).State as
                                        *mut HAL_I2C_StateTypeDef,
                                    HAL_I2C_STATE_READY);
        ::core::ptr::write_volatile(&mut (*hi2c).Mode as
                                        *mut HAL_I2C_ModeTypeDef,
                                    HAL_I2C_MODE_NONE);
        /* Process Unlocked */
        (*hi2c).Lock = HAL_UNLOCKED;
        return HAL_ERROR
    }
    return HAL_OK;
}
/* Private functions to handle  start, restart or stop a transfer */
/* *
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  hi2c I2C handle.
  * @param  DevAddress Specifies the slave address to be programmed.
  * @param  Size Specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  Mode New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_RELOAD_MODE Enable Reload mode .
  *     @arg @ref I2C_AUTOEND_MODE Enable Automatic end mode.
  *     @arg @ref I2C_SOFTEND_MODE Enable Software end mode.
  * @param  Request New state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg @ref I2C_NO_STARTSTOP Don't Generate stop and start condition.
  *     @arg @ref I2C_GENERATE_STOP Generate stop condition (Size should be set to 0).
  *     @arg @ref I2C_GENERATE_START_READ Generate Restart for read request.
  *     @arg @ref I2C_GENERATE_START_WRITE Generate Restart for write request.
  * @retval None
  */
unsafe extern "C" fn I2C_TransferConfig(mut hi2c: *mut I2C_HandleTypeDef,
                                        mut DevAddress: uint16_t,
                                        mut Size: uint8_t, mut Mode: uint32_t,
                                        mut Request: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Get the CR2 register value */
    tmpreg = (*(*hi2c).Instance).CR2;
    /* clear tmpreg specific bits */
    tmpreg &=
        !((0x3ff as libc::c_uint) << 0 as libc::c_uint |
              (0xff as libc::c_uint) << 16 as libc::c_uint |
              (0x1 as libc::c_uint) << 24 as libc::c_uint |
              (0x1 as libc::c_uint) << 25 as libc::c_uint |
              (0x1 as libc::c_uint) << 10 as libc::c_uint |
              (0x1 as libc::c_uint) << 13 as libc::c_uint |
              (0x1 as libc::c_uint) << 14 as libc::c_uint);
    /* update tmpreg */
    tmpreg |=
        DevAddress as uint32_t & (0x3ff as libc::c_uint) << 0 as libc::c_uint
            |
            (Size as uint32_t) << 16 as libc::c_int &
                (0xff as libc::c_uint) << 16 as libc::c_uint | Mode | Request;
    /* update CR2 register */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR2 as *mut uint32_t,
                                tmpreg);
}
/* Private functions to centralize the enable/disable of Interrupts */
/* *
  * @brief  Manage the enabling of Interrupts.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  InterruptRequest Value of @ref I2C_Interrupt_configuration_definition.
  * @retval HAL status
  */
unsafe extern "C" fn I2C_Enable_IRQ(mut hi2c: *mut I2C_HandleTypeDef,
                                    mut InterruptRequest: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmpisr: uint32_t = 0 as libc::c_uint;
    if (*hi2c).XferISR ==
           Some(I2C_Master_ISR_DMA as
                    unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                         _: uint32_t, _: uint32_t)
                        -> HAL_StatusTypeDef) ||
           (*hi2c).XferISR ==
               Some(I2C_Slave_ISR_DMA as
                        unsafe extern "C" fn(_: *mut __I2C_HandleTypeDef,
                                             _: uint32_t, _: uint32_t)
                            -> HAL_StatusTypeDef) {
        if InterruptRequest as libc::c_uint & 0x4 as libc::c_uint ==
               0x4 as libc::c_uint {
            /* Enable ERR, STOP, NACK and ADDR interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 3 as libc::c_uint |
                    (0x1 as libc::c_uint) << 5 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 7 as libc::c_uint
        }
        if InterruptRequest as libc::c_uint & 0x11 as libc::c_uint ==
               0x11 as libc::c_uint {
            /* Enable ERR and NACK interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 7 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint
        }
        if InterruptRequest as libc::c_uint & 0x12 as libc::c_uint ==
               0x12 as libc::c_uint {
            /* Enable STOP interrupts */
            tmpisr |= (0x1 as libc::c_uint) << 5 as libc::c_uint
        }
        if InterruptRequest as libc::c_uint & 0x12 as libc::c_uint ==
               0x12 as libc::c_uint {
            /* Enable TC interrupts */
            tmpisr |= (0x1 as libc::c_uint) << 6 as libc::c_uint
        }
    } else {
        if InterruptRequest as libc::c_uint & 0x4 as libc::c_uint ==
               0x4 as libc::c_uint {
            /* Enable ERR, STOP, NACK, and ADDR interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 3 as libc::c_uint |
                    (0x1 as libc::c_uint) << 5 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 7 as libc::c_uint
        }
        if InterruptRequest as libc::c_uint & 0x1 as libc::c_uint ==
               0x1 as libc::c_uint {
            /* Enable ERR, TC, STOP, NACK and RXI interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 7 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 5 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 1 as libc::c_uint
        }
        if InterruptRequest as libc::c_uint & 0x2 as libc::c_uint ==
               0x2 as libc::c_uint {
            /* Enable ERR, TC, STOP, NACK and TXI interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 7 as libc::c_uint |
                    (0x1 as libc::c_uint) << 6 as libc::c_uint |
                    (0x1 as libc::c_uint) << 5 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 2 as libc::c_uint
        }
        if InterruptRequest as libc::c_uint & 0x12 as libc::c_uint ==
               0x12 as libc::c_uint {
            /* Enable STOP interrupts */
            tmpisr |= (0x1 as libc::c_uint) << 5 as libc::c_uint
        }
    }
    /* Enable interrupts only at the end */
  /* to avoid the risk of I2C interrupt handle execution before */
  /* all interrupts requested done */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | tmpisr) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  Manage the disabling of Interrupts.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  InterruptRequest Value of @ref I2C_Interrupt_configuration_definition.
  * @retval HAL status
  */
unsafe extern "C" fn I2C_Disable_IRQ(mut hi2c: *mut I2C_HandleTypeDef,
                                     mut InterruptRequest: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmpisr: uint32_t = 0 as libc::c_uint;
    if InterruptRequest as libc::c_uint & 0x1 as libc::c_uint ==
           0x1 as libc::c_uint {
        /* Disable TC and TXI interrupts */
        tmpisr |=
            (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 1 as libc::c_uint;
        if (*hi2c).State as libc::c_uint &
               HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint !=
               HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
            /* Disable NACK and STOP interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 7 as libc::c_uint
        }
    }
    if InterruptRequest as libc::c_uint & 0x2 as libc::c_uint ==
           0x2 as libc::c_uint {
        /* Disable TC and RXI interrupts */
        tmpisr |=
            (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 2 as libc::c_uint;
        if (*hi2c).State as libc::c_uint &
               HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint !=
               HAL_I2C_STATE_LISTEN as libc::c_int as libc::c_uint {
            /* Disable NACK and STOP interrupts */
            tmpisr |=
                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                    (0x1 as libc::c_uint) << 4 as libc::c_uint |
                    (0x1 as libc::c_uint) << 7 as libc::c_uint
        }
    }
    if InterruptRequest as libc::c_uint & 0x4 as libc::c_uint ==
           0x4 as libc::c_uint {
        /* Disable ADDR, NACK and STOP interrupts */
        tmpisr |=
            (0x1 as libc::c_uint) << 3 as libc::c_uint |
                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 7 as libc::c_uint
    }
    if InterruptRequest as libc::c_uint & 0x11 as libc::c_uint ==
           0x11 as libc::c_uint {
        /* Enable ERR and NACK interrupts */
        tmpisr |=
            (0x1 as libc::c_uint) << 7 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint
    }
    if InterruptRequest as libc::c_uint & 0x12 as libc::c_uint ==
           0x12 as libc::c_uint {
        /* Enable STOP interrupts */
        tmpisr |= (0x1 as libc::c_uint) << 5 as libc::c_uint
    }
    if InterruptRequest as libc::c_uint & 0x12 as libc::c_uint ==
           0x12 as libc::c_uint {
        /* Enable TC interrupts */
        tmpisr |= (0x1 as libc::c_uint) << 6 as libc::c_uint
    }
    /* Disable interrupts only at the end */
  /* to avoid a breaking situation like at "t" time */
  /* all disable interrupts request are not done */
    ::core::ptr::write_volatile(&mut (*(*hi2c).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hi2c).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmpisr) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_I2C_MODULE_ENABLED */
/* *
  * @}
  */
