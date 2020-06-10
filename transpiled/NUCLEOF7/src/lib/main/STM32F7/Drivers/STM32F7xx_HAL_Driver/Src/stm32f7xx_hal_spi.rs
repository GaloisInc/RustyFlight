use ::libc;
extern "C" {
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
pub type HAL_SPI_StateTypeDef = libc::c_uint;
pub const HAL_SPI_STATE_ABORT: HAL_SPI_StateTypeDef = 7;
pub const HAL_SPI_STATE_ERROR: HAL_SPI_StateTypeDef = 6;
pub const HAL_SPI_STATE_BUSY_TX_RX: HAL_SPI_StateTypeDef = 5;
pub const HAL_SPI_STATE_BUSY_RX: HAL_SPI_StateTypeDef = 4;
pub const HAL_SPI_STATE_BUSY_TX: HAL_SPI_StateTypeDef = 3;
pub const HAL_SPI_STATE_BUSY: HAL_SPI_StateTypeDef = 2;
pub const HAL_SPI_STATE_READY: HAL_SPI_StateTypeDef = 1;
pub const HAL_SPI_STATE_RESET: HAL_SPI_StateTypeDef = 0;
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
/* !< DMA Stream Index                       */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup SPI_Exported_Functions SPI Exported Functions
  * @{
  */
/* * @defgroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
              ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This subsection provides a set of functions allowing to initialize and
          de-initialize the SPIx peripheral:

      (+) User must implement HAL_SPI_MspInit() function in which he configures
          all related peripherals resources (CLOCK, GPIO, DMA, IT and NVIC ).

      (+) Call the function HAL_SPI_Init() to configure the selected device with
          the selected configuration:
        (++) Mode
        (++) Direction
        (++) Data Size
        (++) Clock Polarity and Phase
        (++) NSS Management
        (++) BaudRate Prescaler
        (++) FirstBit
        (++) TIMode
        (++) CRC Calculation
        (++) CRC Polynomial if CRC enabled
        (++) CRC Length, used only with Data8 and Data16
        (++) FIFO reception threshold

      (+) Call the function HAL_SPI_DeInit() to restore the default configuration
          of the selected SPIx peripheral.

@endverbatim
  * @{
  */
/* *
  * @brief  Initialize the SPI according to the specified parameters
  *         in the SPI_InitTypeDef and initialize the associated handle.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Init(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut frxth: uint32_t = 0;
    /* Check the SPI handle allocation */
    if hspi.is_null() { return HAL_ERROR }
    /* Check the parameters */
    ((*hspi).Init.TIMode) == 0 as libc::c_uint;
    (*hspi).Init.CRCCalculation = 0 as libc::c_uint;
    /* USE_SPI_CRC */
    if (*hspi).State as libc::c_uint ==
           HAL_SPI_STATE_RESET as libc::c_int as libc::c_uint {
        /* Allocate lock resource and initialize it */
        (*hspi).Lock = HAL_UNLOCKED;
        /* Init the low level hardware : GPIO, CLOCK, NVIC... */
        HAL_SPI_MspInit(hspi);
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_BUSY);
    /* Disable the selected SPI peripheral */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Align by default the rs fifo threshold on the data size */
    if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
        frxth = 0 as libc::c_uint
    } else { frxth = (0x1 as libc::c_uint) << 12 as libc::c_uint }
    /* CRC calculation is valid only for 16Bit and 8 Bit */
    if (*hspi).Init.DataSize != 0xf00 as libc::c_uint &&
           (*hspi).Init.DataSize != 0x700 as libc::c_uint {
        /* CRC must be disabled */
        (*hspi).Init.CRCCalculation = 0 as libc::c_uint
    }
    /* Align the CRC Length on the data size */
    if (*hspi).Init.CRCLength == 0 as libc::c_uint {
        /* CRC Length aligned on the data size : value set by default */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            (*hspi).Init.CRCLength = 0x2 as libc::c_uint
        } else { (*hspi).Init.CRCLength = 0x1 as libc::c_uint }
    }
    /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
  /* Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
  Communication speed, First bit, CRC calculation state */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (*hspi).Init.Mode | (*hspi).Init.Direction |
                                    (*hspi).Init.CLKPolarity |
                                    (*hspi).Init.CLKPhase |
                                    (*hspi).Init.NSS &
                                        (0x1 as libc::c_uint) <<
                                            9 as libc::c_uint |
                                    (*hspi).Init.BaudRatePrescaler |
                                    (*hspi).Init.FirstBit |
                                    (*hspi).Init.CRCCalculation);
    /* USE_SPI_CRC */
    /* Configure : NSS management, TI Mode and Rx Fifo Threshold */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (*hspi).Init.NSS >> 16 as libc::c_int &
                                    (0x1 as libc::c_uint) << 2 as libc::c_uint
                                    | (*hspi).Init.TIMode |
                                    (*hspi).Init.NSSPMode |
                                    (*hspi).Init.DataSize | frxth);
    /* USE_SPI_CRC */
    /* Activate the SPI mode (Make sure that I2SMOD bit in I2SCFGR register is reset) */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).I2SCFGR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).I2SCFGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           11 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* SPI_I2SCFGR_I2SMOD */
    ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  De-Initialize the SPI peripheral.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_DeInit(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Check the SPI handle allocation */
    if hspi.is_null() { return HAL_ERROR }
    /* Check SPI Instance parameter */
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_BUSY);
    /* Disable the SPI Peripheral Clock */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* DeInit the low level hardware: GPIO, CLOCK, NVIC... */
    HAL_SPI_MspDeInit(hspi);
    ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_RESET);
    /* Release Lock */
    (*hspi).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Initialize the SPI MSP.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_MspInit(mut hspi: *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_MspInit should be implemented in the user file
   */
}
/* *
  * @brief  De-Initialize the SPI MSP.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_MspDeInit(mut hspi: *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_MspDeInit should be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup SPI_Exported_Functions_Group2 IO operation functions
 *  @brief   Data transfers functions
 *
@verbatim
  ==============================================================================
                      ##### IO operation functions #####
 ===============================================================================
 [..]
    This subsection provides a set of functions allowing to manage the SPI
    data transfers.

    [..] The SPI supports master and slave mode :

    (#) There are two modes of transfer:
       (++) Blocking mode: The communication is performed in polling mode.
            The HAL status of all data processing is returned by the same function
            after finishing transfer.
       (++) No-Blocking mode: The communication is performed using Interrupts
            or DMA, These APIs return the HAL status.
            The end of the data processing will be indicated through the
            dedicated SPI IRQ when using Interrupt mode or the DMA IRQ when
            using DMA mode.
            The HAL_SPI_TxCpltCallback(), HAL_SPI_RxCpltCallback() and HAL_SPI_TxRxCpltCallback() user callbacks
            will be executed respectively at the end of the transmit or Receive process
            The HAL_SPI_ErrorCallback()user callback will be executed when a communication error is detected

    (#) APIs provided for these 2 transfer modes (Blocking mode or Non blocking mode using either Interrupt or DMA)
        exist for 1Line (simplex) and 2Lines (full duplex) modes.

@endverbatim
  * @{
  */
/* *
  * @brief  Transmit an amount of data in blocking mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Transmit(mut hspi: *mut SPI_HandleTypeDef,
                                          mut pData: *const uint8_t,
                                          mut Size: uint16_t,
                                          mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut current_block: u64;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    /* Check Direction parameter */
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    if (*hspi).State as libc::c_uint !=
           HAL_SPI_STATE_READY as libc::c_int as libc::c_uint {
        errorcode = HAL_BUSY
    } else if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pTxBuffPtr = pData as *mut uint8_t;
        (*hspi).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    Size);
        /*Init field not used in handle to zero */
        (*hspi).pRxBuffPtr = 0 as *mut libc::c_void as *mut uint8_t;
        (*hspi).RxXferSize = 0 as libc::c_uint as uint16_t;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        (*hspi).TxISR = None;
        (*hspi).RxISR = None;
        /* Configure communication direction : 1Line */
        if (*hspi).Init.Direction ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 14 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* USE_SPI_CRC */
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Transmit data in 16 Bit mode */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            loop 
                 /* Transmit data in 16 Bit mode */
                 {
                if !((*hspi).TxXferCount as libc::c_uint > 0 as libc::c_uint)
                   {
                    current_block = 2290177392965769716;
                    break ;
                }
                /* Wait until TXE flag is set to send data */
                if (*(*hspi).Instance).SR &
                       (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 1 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as
                                                    *mut uint32_t,
                                                *(pData as *mut uint16_t) as
                                                    uint32_t);
                    pData =
                        pData.offset(::core::mem::size_of::<uint16_t>() as
                                         libc::c_ulong as isize);
                    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                    *mut uint16_t,
                                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                           as
                                                                                           *const uint16_t).wrapping_sub(1))
                } else {
                    /* Timeout management */
                    if !(Timeout == 0 as libc::c_uint ||
                             Timeout != 0xffffffff as libc::c_uint &&
                                 HAL_GetTick().wrapping_sub(tickstart) >=
                                     Timeout) {
                        continue ;
                    }
                    errorcode = HAL_TIMEOUT;
                    current_block = 11264357029779558864;
                    break ;
                }
            }
        } else {
            loop 
                 /* Transmit data in 8 Bit mode */
                 {
                if !((*hspi).TxXferCount as libc::c_uint > 0 as libc::c_uint)
                   {
                    current_block = 2290177392965769716;
                    break ;
                }
                /* Wait until TXE flag is set to send data */
                if (*(*hspi).Instance).SR &
                       (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 1 as libc::c_uint {
                    if (*hspi).TxXferCount as libc::c_uint > 1 as libc::c_uint
                       {
                        /* write on the data register in packing mode */
                        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR
                                                        as *mut uint32_t,
                                                    *(pData as *mut uint16_t)
                                                        as uint32_t);
                        pData =
                            pData.offset(::core::mem::size_of::<uint16_t>() as
                                             libc::c_ulong as isize);
                        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount
                                                        as *mut uint16_t,
                                                    (::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                                as
                                                                                                *const uint16_t)
                                                         as
                                                         libc::c_uint).wrapping_sub(2
                                                                                        as
                                                                                        libc::c_uint)
                                                        as uint16_t as
                                                        uint16_t)
                    } else {
                        let fresh0 = pData;
                        pData = pData.offset(1);
                        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR
                                                        as *mut uint32_t as
                                                        *mut uint8_t,
                                                    *fresh0);
                        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount
                                                        as *mut uint16_t,
                                                    ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                               as
                                                                                               *const uint16_t).wrapping_sub(1))
                    }
                } else {
                    /* Timeout management */
                    if !(Timeout == 0 as libc::c_uint ||
                             Timeout != 0xffffffff as libc::c_uint &&
                                 HAL_GetTick().wrapping_sub(tickstart) >=
                                     Timeout) {
                        continue ;
                    }
                    errorcode = HAL_TIMEOUT;
                    current_block = 11264357029779558864;
                    break ;
                }
            }
        }
        match current_block {
            11264357029779558864 => { }
            _ => {
                /* USE_SPI_CRC */
                /* Check the end of the transaction */
                if SPI_EndRxTxTransaction(hspi, Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                    *mut uint32_t,
                                                0x20 as libc::c_uint)
                }
                /* Clear overrun flag in 2 Lines communication mode because received is not read */
                if (*hspi).Init.Direction == 0 as libc::c_uint {
                    let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
                    ::core::ptr::write_volatile(&mut tmpreg_ovr as
                                                    *mut uint32_t,
                                                (*(*hspi).Instance).DR);
                    ::core::ptr::write_volatile(&mut tmpreg_ovr as
                                                    *mut uint32_t,
                                                (*(*hspi).Instance).SR)
                }
                if (*hspi).ErrorCode != 0 as libc::c_uint {
                    errorcode = HAL_ERROR
                }
            }
        }
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Receive an amount of data in blocking mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be received
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Receive(mut hspi: *mut SPI_HandleTypeDef,
                                         mut pData: *mut uint8_t,
                                         mut Size: uint16_t,
                                         mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut current_block: u64;
    /* USE_SPI_CRC */
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    if (*hspi).Init.Mode ==
           (0x1 as libc::c_uint) << 2 as libc::c_uint |
               (0x1 as libc::c_uint) << 8 as libc::c_uint &&
           (*hspi).Init.Direction == 0 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_RX);
        /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
        return HAL_SPI_TransmitReceive(hspi, pData, pData, Size, Timeout)
    }
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    if (*hspi).State as libc::c_uint !=
           HAL_SPI_STATE_READY as libc::c_int as libc::c_uint {
        errorcode = HAL_BUSY
    } else if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pRxBuffPtr = pData;
        (*hspi).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    Size);
        /*Init field not used in handle to zero */
        (*hspi).pTxBuffPtr = 0 as *mut libc::c_void as *mut uint8_t;
        (*hspi).TxXferSize = 0 as libc::c_uint as uint16_t;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        (*hspi).RxISR = None;
        (*hspi).TxISR = None;
        /* USE_SPI_CRC */
        /* Set the Rx Fifo threshold */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            /* set fiforxthreshold according the reception data length: 16bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* set fiforxthreshold according the reception data length: 8bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Configure communication direction: 1Line */
        if (*hspi).Init.Direction ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   14 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Receive data in 8 Bit mode */
        if (*hspi).Init.DataSize <= 0x700 as libc::c_uint {
            loop 
                 /* Transfer loop */
                 {
                if !((*hspi).RxXferCount as libc::c_uint > 0 as libc::c_uint)
                   {
                    current_block = 7343950298149844727;
                    break ;
                }
                /* Check the RXNE flag */
                if (*(*hspi).Instance).SR &
                       (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    /* read the received data */
                    *pData =
                        *(&mut (*(*hspi).Instance).DR as *mut uint32_t as
                              *mut uint8_t);
                    pData =
                        pData.offset(::core::mem::size_of::<uint8_t>() as
                                         libc::c_ulong as isize);
                    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as
                                                    *mut uint16_t,
                                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                                           as
                                                                                           *const uint16_t).wrapping_sub(1))
                } else {
                    /* Timeout management */
                    if !(Timeout == 0 as libc::c_uint ||
                             Timeout != 0xffffffff as libc::c_uint &&
                                 HAL_GetTick().wrapping_sub(tickstart) >=
                                     Timeout) {
                        continue ;
                    }
                    errorcode = HAL_TIMEOUT;
                    current_block = 2144631550213097087;
                    break ;
                }
            }
        } else {
            loop 
                 /* Transfer loop */
                 {
                if !((*hspi).RxXferCount as libc::c_uint > 0 as libc::c_uint)
                   {
                    current_block = 7343950298149844727;
                    break ;
                }
                /* Check the RXNE flag */
                if (*(*hspi).Instance).SR &
                       (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                       (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    *(pData as *mut uint16_t) =
                        (*(*hspi).Instance).DR as uint16_t;
                    pData =
                        pData.offset(::core::mem::size_of::<uint16_t>() as
                                         libc::c_ulong as isize);
                    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as
                                                    *mut uint16_t,
                                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                                           as
                                                                                           *const uint16_t).wrapping_sub(1))
                } else {
                    /* Timeout management */
                    if !(Timeout == 0 as libc::c_uint ||
                             Timeout != 0xffffffff as libc::c_uint &&
                                 HAL_GetTick().wrapping_sub(tickstart) >=
                                     Timeout) {
                        continue ;
                    }
                    errorcode = HAL_TIMEOUT;
                    current_block = 2144631550213097087;
                    break ;
                }
            }
        }
        match current_block {
            2144631550213097087 => { }
            _ => {
                /* USE_SPI_CRC */
                /* Check the end of the transaction */
                if SPI_EndRxTransaction(hspi, Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                    *mut uint32_t,
                                                0x20 as libc::c_uint)
                }
                /* USE_SPI_CRC */
                if (*hspi).ErrorCode != 0 as libc::c_uint {
                    errorcode = HAL_ERROR
                }
            }
        }
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Transmit and Receive an amount of data in blocking mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer
  * @param  Size: amount of data to be sent and received
  * @param  Timeout: Timeout duration
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TransmitReceive(mut hspi:
                                                     *mut SPI_HandleTypeDef,
                                                 mut pTxData: *const uint8_t,
                                                 mut pRxData: *mut uint8_t,
                                                 mut Size: uint16_t,
                                                 mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut current_block: u64;
    let mut tmp: uint32_t = 0 as libc::c_uint;
    let mut tmp1: uint32_t = 0 as libc::c_uint;
    /* USE_SPI_CRC */
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Variable used to alternate Rx and Tx during transfer */
    let mut txallowed: uint32_t = 1 as libc::c_uint;
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    /* Check Direction parameter */
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    tmp = (*hspi).State as uint32_t;
    tmp1 = (*hspi).Init.Mode;
    if !(tmp == HAL_SPI_STATE_READY as libc::c_int as libc::c_uint ||
             tmp1 ==
                 (0x1 as libc::c_uint) << 2 as libc::c_uint |
                     (0x1 as libc::c_uint) << 8 as libc::c_uint &&
                 (*hspi).Init.Direction == 0 as libc::c_uint &&
                 tmp == HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint)
       {
        errorcode = HAL_BUSY
    } else if pTxData.is_null() || pRxData.is_null() ||
                  Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
        if (*hspi).State as libc::c_uint !=
               HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).State as
                                            *mut HAL_SPI_StateTypeDef,
                                        HAL_SPI_STATE_BUSY_TX_RX)
        }
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pRxBuffPtr = pRxData;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    Size);
        (*hspi).RxXferSize = Size;
        (*hspi).pTxBuffPtr = pTxData as *mut uint8_t;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    Size);
        (*hspi).TxXferSize = Size;
        /*Init field not used in handle to zero */
        (*hspi).RxISR = None;
        (*hspi).TxISR = None;
        /* USE_SPI_CRC */
        /* Set the Rx Fifo threshold */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint ||
               (*hspi).RxXferCount as libc::c_int > 1 as libc::c_int {
            /* set fiforxthreshold according the reception data length: 16bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* set fiforxthreshold according the reception data length: 8bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Transmit and Receive data in 16 Bit mode */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            if (*hspi).Init.Mode == 0 as libc::c_uint ||
                   (*hspi).TxXferCount as libc::c_int == 0x1 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as
                                                *mut uint32_t,
                                            *(pTxData as *mut uint16_t) as
                                                uint32_t);
                pTxData =
                    pTxData.offset(::core::mem::size_of::<uint16_t>() as
                                       libc::c_ulong as isize);
                ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                *mut uint16_t,
                                            ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                       as
                                                                                       *const uint16_t).wrapping_sub(1))
            }
            loop  {
                if !((*hspi).TxXferCount as libc::c_uint > 0 as libc::c_uint
                         ||
                         (*hspi).RxXferCount as libc::c_uint >
                             0 as libc::c_uint) {
                    current_block = 16789764818708874114;
                    break ;
                }
                /* Check TXE flag */
                if txallowed != 0 &&
                       (*hspi).TxXferCount as libc::c_uint > 0 as libc::c_uint
                       &&
                       (*(*hspi).Instance).SR &
                           (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 1 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as
                                                    *mut uint32_t,
                                                *(pTxData as *mut uint16_t) as
                                                    uint32_t);
                    pTxData =
                        pTxData.offset(::core::mem::size_of::<uint16_t>() as
                                           libc::c_ulong as isize);
                    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                    *mut uint16_t,
                                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                           as
                                                                                           *const uint16_t).wrapping_sub(1));
                    /* USE_SPI_CRC */
                    txallowed = 0 as libc::c_uint
                }
                /* Next Data is a reception (Rx). Tx not allowed */
                /* Check RXNE flag */
                if (*hspi).RxXferCount as libc::c_uint > 0 as libc::c_uint &&
                       (*(*hspi).Instance).SR &
                           (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    *(pRxData as *mut uint16_t) =
                        (*(*hspi).Instance).DR as uint16_t;
                    pRxData =
                        pRxData.offset(::core::mem::size_of::<uint16_t>() as
                                           libc::c_ulong as isize);
                    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as
                                                    *mut uint16_t,
                                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                                           as
                                                                                           *const uint16_t).wrapping_sub(1));
                    /* Next Data is a Transmission (Tx). Tx is allowed */
                    txallowed = 1 as libc::c_uint
                }
                if !(Timeout != 0xffffffff as libc::c_uint &&
                         HAL_GetTick().wrapping_sub(tickstart) >= Timeout) {
                    continue ;
                }
                errorcode = HAL_TIMEOUT;
                current_block = 8595026708052935897;
                break ;
            }
        } else {
            /* Transmit and Receive data in 8 Bit mode */
            if (*hspi).Init.Mode == 0 as libc::c_uint ||
                   (*hspi).TxXferCount as libc::c_int == 0x1 as libc::c_int {
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as
                                                *mut uint32_t as *mut uint8_t,
                                            *pTxData);
                pTxData =
                    pTxData.offset(::core::mem::size_of::<uint8_t>() as
                                       libc::c_ulong as isize);
                ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                *mut uint16_t,
                                            ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                       as
                                                                                       *const uint16_t).wrapping_sub(1))
            }
            loop  {
                if !((*hspi).TxXferCount as libc::c_uint > 0 as libc::c_uint
                         ||
                         (*hspi).RxXferCount as libc::c_uint >
                             0 as libc::c_uint) {
                    current_block = 16789764818708874114;
                    break ;
                }
                /* check TXE flag */
                if txallowed != 0 &&
                       (*hspi).TxXferCount as libc::c_uint > 0 as libc::c_uint
                       &&
                       (*(*hspi).Instance).SR &
                           (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 1 as libc::c_uint {
                    if (*hspi).TxXferCount as libc::c_uint > 1 as libc::c_uint
                       {
                        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR
                                                        as *mut uint32_t,
                                                    *(pTxData as
                                                          *mut uint16_t) as
                                                        uint32_t);
                        pTxData =
                            pTxData.offset(::core::mem::size_of::<uint16_t>()
                                               as libc::c_ulong as isize);
                        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount
                                                        as *mut uint16_t,
                                                    (::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                                as
                                                                                                *const uint16_t)
                                                         as
                                                         libc::c_uint).wrapping_sub(2
                                                                                        as
                                                                                        libc::c_uint)
                                                        as uint16_t as
                                                        uint16_t)
                    } else {
                        let fresh1 = pTxData;
                        pTxData = pTxData.offset(1);
                        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR
                                                        as *mut uint32_t as
                                                        *mut uint8_t,
                                                    *fresh1);
                        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount
                                                        as *mut uint16_t,
                                                    ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                               as
                                                                                               *const uint16_t).wrapping_sub(1))
                    }
                    /* USE_SPI_CRC */
                    txallowed = 0 as libc::c_uint
                }
                /* Next Data is a reception (Rx). Tx not allowed */
                /* Wait until RXNE flag is reset */
                if (*hspi).RxXferCount as libc::c_uint > 0 as libc::c_uint &&
                       (*(*hspi).Instance).SR &
                           (0x1 as libc::c_uint) << 0 as libc::c_uint ==
                           (0x1 as libc::c_uint) << 0 as libc::c_uint {
                    if (*hspi).RxXferCount as libc::c_uint > 1 as libc::c_uint
                       {
                        *(pRxData as *mut uint16_t) =
                            (*(*hspi).Instance).DR as uint16_t;
                        pRxData =
                            pRxData.offset(::core::mem::size_of::<uint16_t>()
                                               as libc::c_ulong as isize);
                        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount
                                                        as *mut uint16_t,
                                                    (::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                                                as
                                                                                                *const uint16_t)
                                                         as
                                                         libc::c_uint).wrapping_sub(2
                                                                                        as
                                                                                        libc::c_uint)
                                                        as uint16_t as
                                                        uint16_t);
                        if (*hspi).RxXferCount as libc::c_uint <=
                               1 as libc::c_uint {
                            /* set fiforxthreshold before to switch on 8 bit data size */
                            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2
                                                            as *mut uint32_t,
                                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                                    as
                                                                                                    *const uint32_t)
                                                             as libc::c_uint |
                                                             (0x1 as
                                                                  libc::c_uint)
                                                                 <<
                                                                 12 as
                                                                     libc::c_uint)
                                                            as uint32_t as
                                                            uint32_t)
                        }
                    } else {
                        let fresh2 = pRxData;
                        pRxData = pRxData.offset(1);
                        *fresh2 =
                            *(&mut (*(*hspi).Instance).DR as *mut uint32_t as
                                  *mut uint8_t);
                        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount
                                                        as *mut uint16_t,
                                                    ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                                               as
                                                                                               *const uint16_t).wrapping_sub(1))
                    }
                    /* Next Data is a Transmission (Tx). Tx is allowed */
                    txallowed = 1 as libc::c_uint
                }
                if !(Timeout != 0xffffffff as libc::c_uint &&
                         HAL_GetTick().wrapping_sub(tickstart) >= Timeout) {
                    continue ;
                }
                errorcode = HAL_TIMEOUT;
                current_block = 8595026708052935897;
                break ;
            }
        }
        match current_block {
            8595026708052935897 => { }
            _ => {
                /* USE_SPI_CRC */
                /* Check the end of the transaction */
                if SPI_EndRxTxTransaction(hspi, Timeout, tickstart) as
                       libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                    *mut uint32_t,
                                                0x20 as libc::c_uint)
                }
                if (*hspi).ErrorCode != 0 as libc::c_uint {
                    errorcode = HAL_ERROR
                }
            }
        }
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Transmit an amount of data in non-blocking mode with Interrupt.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Transmit_IT(mut hspi: *mut SPI_HandleTypeDef,
                                             mut pData: *mut uint8_t,
                                             mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    /* Check Direction parameter */
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else if (*hspi).State as libc::c_uint !=
                  HAL_SPI_STATE_READY as libc::c_int as libc::c_uint {
        errorcode = HAL_BUSY
    } else {
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pTxBuffPtr = pData;
        (*hspi).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    Size);
        /* Init field not used in handle to zero */
        (*hspi).pRxBuffPtr = 0 as *mut libc::c_void as *mut uint8_t;
        (*hspi).RxXferSize = 0 as libc::c_uint as uint16_t;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        (*hspi).RxISR = None;
        /* Set the function for IT treatment */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            (*hspi).TxISR =
                Some(SPI_TxISR_16BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ())
        } else {
            (*hspi).TxISR =
                Some(SPI_TxISR_8BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ())
        }
        /* Configure communication direction : 1Line */
        if (*hspi).Init.Direction ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 14 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* USE_SPI_CRC */
        /* Enable TXE and ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              7 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Receive_IT(mut hspi: *mut SPI_HandleTypeDef,
                                            mut pData: *mut uint8_t,
                                            mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    if (*hspi).Init.Direction == 0 as libc::c_uint &&
           (*hspi).Init.Mode ==
               (0x1 as libc::c_uint) << 2 as libc::c_uint |
                   (0x1 as libc::c_uint) << 8 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_RX);
        /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
        return HAL_SPI_TransmitReceive_IT(hspi, pData, pData, Size)
    }
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    if (*hspi).State as libc::c_uint !=
           HAL_SPI_STATE_READY as libc::c_int as libc::c_uint {
        errorcode = HAL_BUSY
    } else if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pRxBuffPtr = pData;
        (*hspi).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    Size);
        /* Init field not used in handle to zero */
        (*hspi).pTxBuffPtr = 0 as *mut libc::c_void as *mut uint8_t;
        (*hspi).TxXferSize = 0 as libc::c_uint as uint16_t;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        (*hspi).TxISR = None;
        /* check the data size to adapt Rx threshold and the set the function for IT treatment */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            /* set fiforxthreshold according the reception data length: 16 bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            (*hspi).RxISR =
                Some(SPI_RxISR_16BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ())
        } else {
            /* set fiforxthreshold according the reception data length: 8 bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t);
            (*hspi).RxISR =
                Some(SPI_RxISR_8BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ())
        }
        /* Configure communication direction : 1Line */
        if (*hspi).Init.Direction ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   14 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        /* USE_SPI_CRC */
        /* Enable TXE and ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              6 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Note : The SPI must be enabled after unlocking current process
            to avoid the risk of SPI interrupt handle execution before current
            process unlock */
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Transmit and Receive an amount of data in non-blocking mode with Interrupt.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer
  * @param  Size: amount of data to be sent and received
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TransmitReceive_IT(mut hspi:
                                                        *mut SPI_HandleTypeDef,
                                                    mut pTxData: *mut uint8_t,
                                                    mut pRxData: *mut uint8_t,
                                                    mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: uint32_t = 0 as libc::c_uint;
    let mut tmp1: uint32_t = 0 as libc::c_uint;
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    /* Check Direction parameter */
    /* Process locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    tmp = (*hspi).State as uint32_t;
    tmp1 = (*hspi).Init.Mode;
    if !(tmp == HAL_SPI_STATE_READY as libc::c_int as libc::c_uint ||
             tmp1 ==
                 (0x1 as libc::c_uint) << 2 as libc::c_uint |
                     (0x1 as libc::c_uint) << 8 as libc::c_uint &&
                 (*hspi).Init.Direction == 0 as libc::c_uint &&
                 tmp == HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint)
       {
        errorcode = HAL_BUSY
    } else if pTxData.is_null() || pRxData.is_null() ||
                  Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
        if (*hspi).State as libc::c_uint !=
               HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).State as
                                            *mut HAL_SPI_StateTypeDef,
                                        HAL_SPI_STATE_BUSY_TX_RX)
        }
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pTxBuffPtr = pTxData;
        (*hspi).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    Size);
        (*hspi).pRxBuffPtr = pRxData;
        (*hspi).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    Size);
        /* Set the function for IT treatment */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            (*hspi).RxISR =
                Some(SPI_2linesRxISR_16BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ());
            (*hspi).TxISR =
                Some(SPI_2linesTxISR_16BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ())
        } else {
            (*hspi).RxISR =
                Some(SPI_2linesRxISR_8BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ());
            (*hspi).TxISR =
                Some(SPI_2linesTxISR_8BIT as
                         unsafe extern "C" fn(_: *mut __SPI_HandleTypeDef)
                             -> ())
        }
        /* USE_SPI_CRC */
        /* check if packing mode is enabled and if there is more than 2 data to receive */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint ||
               (*hspi).RxXferCount as libc::c_uint >= 2 as libc::c_uint {
            /* set fiforxthreshold according the reception data length: 16 bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* set fiforxthreshold according the reception data length: 8 bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Enable TXE, RXNE and ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              7 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  6 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Transmit an amount of data in non-blocking mode with DMA.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Transmit_DMA(mut hspi:
                                                  *mut SPI_HandleTypeDef,
                                              mut pData: *mut uint8_t,
                                              mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    /* Check Direction parameter */
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    if (*hspi).State as libc::c_uint !=
           HAL_SPI_STATE_READY as libc::c_int as libc::c_uint {
        errorcode = HAL_BUSY
    } else if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_TX);
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pTxBuffPtr = pData;
        (*hspi).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    Size);
        /* Init field not used in handle to zero */
        (*hspi).pRxBuffPtr = 0 as *mut libc::c_void as *mut uint8_t;
        (*hspi).TxISR = None;
        (*hspi).RxISR = None;
        (*hspi).RxXferSize = 0 as libc::c_uint as uint16_t;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        /* Configure communication direction : 1Line */
        if (*hspi).Init.Direction ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 14 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* USE_SPI_CRC */
        /* Set the SPI TxDMA Half transfer complete callback */
        (*(*hspi).hdmatx).XferHalfCpltCallback =
            Some(SPI_DMAHalfTransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the SPI TxDMA transfer complete callback */
        (*(*hspi).hdmatx).XferCpltCallback =
            Some(SPI_DMATransmitCplt as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA error callback */
        (*(*hspi).hdmatx).XferErrorCallback =
            Some(SPI_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA AbortCpltCallback */
        (*(*hspi).hdmatx).XferAbortCallback = None;
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               14 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* packing mode is enabled only if the DMA setting is HALWORD */
        if (*hspi).Init.DataSize <= 0x700 as libc::c_uint &&
               (*(*hspi).hdmatx).Init.MemDataAlignment ==
                   (0x1 as libc::c_uint) << 13 as libc::c_uint {
            /* Check the even/odd of the data size + crc if enabled */
            if (*hspi).TxXferCount as libc::c_uint & 0x1 as libc::c_uint ==
                   0 as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       14 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                *mut uint16_t,
                                            ((*hspi).TxXferCount as
                                                 libc::c_int >>
                                                 1 as libc::c_uint) as
                                                uint16_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     14 as libc::c_uint) as
                                                uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                *mut uint16_t,
                                            (((*hspi).TxXferCount as
                                                  libc::c_int >>
                                                  1 as libc::c_uint) as
                                                 libc::c_uint).wrapping_add(1
                                                                                as
                                                                                libc::c_uint)
                                                as uint16_t)
            }
        }
        /* Enable the Tx DMA Stream/Channel */
        HAL_DMA_Start_IT((*hspi).hdmatx, (*hspi).pTxBuffPtr as uint32_t,
                         &mut (*(*hspi).Instance).DR as *mut uint32_t as
                             uint32_t, (*hspi).TxXferCount as uint32_t);
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Enable the SPI Error Interrupt Bit */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable Tx DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             1 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pData: pointer to data buffer
  * @note   When the CRC feature is enabled the pData Length must be Size + 1.
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Receive_DMA(mut hspi: *mut SPI_HandleTypeDef,
                                             mut pData: *mut uint8_t,
                                             mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    if (*hspi).Init.Direction == 0 as libc::c_uint &&
           (*hspi).Init.Mode ==
               (0x1 as libc::c_uint) << 2 as libc::c_uint |
                   (0x1 as libc::c_uint) << 8 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_RX);
        /* Call transmit-receive function to send Dummy data on Tx line and generate clock on CLK line */
        return HAL_SPI_TransmitReceive_DMA(hspi, pData, pData, Size)
    }
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    if (*hspi).State as libc::c_uint !=
           HAL_SPI_STATE_READY as libc::c_int as libc::c_uint {
        errorcode = HAL_BUSY
    } else if pData.is_null() || Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_BUSY_RX);
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pRxBuffPtr = pData;
        (*hspi).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    Size);
        /*Init field not used in handle to zero */
        (*hspi).RxISR = None;
        (*hspi).TxISR = None;
        (*hspi).TxXferSize = 0 as libc::c_uint as uint16_t;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        /* Configure communication direction : 1Line */
        if (*hspi).Init.Direction ==
               (0x1 as libc::c_uint) << 15 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   14 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        /* USE_SPI_CRC */
        /* packing mode management is enabled by the DMA settings */
        if (*hspi).Init.DataSize <= 0x700 as libc::c_uint &&
               (*(*hspi).hdmarx).Init.MemDataAlignment ==
                   (0x1 as libc::c_uint) << 13 as libc::c_uint {
            /* Restriction the DMA data received is not allowed in this mode */
            errorcode = HAL_ERROR
        } else {
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   13 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
                /* set fiforxthreshold according the reception data length: 16bit */
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       12 as libc::c_uint)) as
                                                uint32_t as uint32_t)
            } else {
                /* set fiforxthreshold according the reception data length: 8bit */
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     12 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
            /* Set the SPI RxDMA Half transfer complete callback */
            (*(*hspi).hdmarx).XferHalfCpltCallback =
                Some(SPI_DMAHalfReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the SPI Rx DMA transfer complete callback */
            (*(*hspi).hdmarx).XferCpltCallback =
                Some(SPI_DMAReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA error callback */
            (*(*hspi).hdmarx).XferErrorCallback =
                Some(SPI_DMAError as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            /* Set the DMA AbortCpltCallback */
            (*(*hspi).hdmarx).XferAbortCallback = None;
            /* Enable the Rx DMA Stream/Channel  */
            HAL_DMA_Start_IT((*hspi).hdmarx,
                             &mut (*(*hspi).Instance).DR as *mut uint32_t as
                                 uint32_t, (*hspi).pRxBuffPtr as uint32_t,
                             (*hspi).RxXferCount as uint32_t);
            /* Check if the SPI is already enabled */
            if (*(*hspi).Instance).CR1 &
                   (0x1 as libc::c_uint) << 6 as libc::c_uint !=
                   (0x1 as libc::c_uint) << 6 as libc::c_uint {
                /* Enable SPI peripheral */
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 (0x1 as libc::c_uint) <<
                                                     6 as libc::c_uint) as
                                                uint32_t as uint32_t)
            }
            /* Enable the SPI Error Interrupt Bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 5 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Enable Rx DMA Request */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    }
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Transmit and Receive an amount of data in non-blocking mode with DMA.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  pTxData: pointer to transmission data buffer
  * @param  pRxData: pointer to reception data buffer
  * @note   When the CRC feature is enabled the pRxData Length must be Size + 1
  * @param  Size: amount of data to be sent
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TransmitReceive_DMA(mut hspi:
                                                         *mut SPI_HandleTypeDef,
                                                     mut pTxData:
                                                         *mut uint8_t,
                                                     mut pRxData:
                                                         *mut uint8_t,
                                                     mut Size: uint16_t)
 -> HAL_StatusTypeDef {
    let mut tmp: uint32_t = 0 as libc::c_uint;
    let mut tmp1: uint32_t = 0 as libc::c_uint;
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    /* Check Direction parameter */
    /* Process locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    tmp = (*hspi).State as uint32_t;
    tmp1 = (*hspi).Init.Mode;
    if !(tmp == HAL_SPI_STATE_READY as libc::c_int as libc::c_uint ||
             tmp1 ==
                 (0x1 as libc::c_uint) << 2 as libc::c_uint |
                     (0x1 as libc::c_uint) << 8 as libc::c_uint &&
                 (*hspi).Init.Direction == 0 as libc::c_uint &&
                 tmp == HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint)
       {
        errorcode = HAL_BUSY
    } else if pTxData.is_null() || pRxData.is_null() ||
                  Size as libc::c_uint == 0 as libc::c_uint {
        errorcode = HAL_ERROR
    } else {
        /* Don't overwrite in case of HAL_SPI_STATE_BUSY_RX */
        if (*hspi).State as libc::c_uint !=
               HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).State as
                                            *mut HAL_SPI_StateTypeDef,
                                        HAL_SPI_STATE_BUSY_TX_RX)
        }
        /* Set the transaction information */
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        (*hspi).pTxBuffPtr = pTxData;
        (*hspi).TxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    Size);
        (*hspi).pRxBuffPtr = pRxData;
        (*hspi).RxXferSize = Size;
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    Size);
        /* Init field not used in handle to zero */
        (*hspi).RxISR = None;
        (*hspi).TxISR = None;
        /* USE_SPI_CRC */
        /* Reset the threshold bit */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               14 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   13 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* the packing mode management is enabled by the DMA settings according the spi data size */
        if (*hspi).Init.DataSize > 0x700 as libc::c_uint {
            /* set fiforxthreshold according the reception data length: 16bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   12 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* set fiforxthreshold according the reception data length: 8bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t);
            if (*(*hspi).hdmatx).Init.MemDataAlignment ==
                   (0x1 as libc::c_uint) << 13 as libc::c_uint {
                if (*hspi).TxXferSize as libc::c_uint & 0x1 as libc::c_uint ==
                       0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           14 as
                                                               libc::c_uint))
                                                    as uint32_t as uint32_t);
                    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                    *mut uint16_t,
                                                ((*hspi).TxXferCount as
                                                     libc::c_int >>
                                                     1 as libc::c_uint) as
                                                    uint16_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         14 as libc::c_uint)
                                                    as uint32_t as uint32_t);
                    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as
                                                    *mut uint16_t,
                                                (((*hspi).TxXferCount as
                                                      libc::c_int >>
                                                      1 as libc::c_uint) as
                                                     libc::c_uint).wrapping_add(1
                                                                                    as
                                                                                    libc::c_uint)
                                                    as uint16_t)
                }
            }
            if (*(*hspi).hdmarx).Init.MemDataAlignment ==
                   (0x1 as libc::c_uint) << 13 as libc::c_uint {
                /* set fiforxthreshold according the reception data length: 16bit */
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       12 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                if (*hspi).RxXferCount as libc::c_uint & 0x1 as libc::c_uint
                       == 0 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           13 as
                                                               libc::c_uint))
                                                    as uint32_t as uint32_t);
                    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as
                                                    *mut uint16_t,
                                                ((*hspi).RxXferCount as
                                                     libc::c_int >>
                                                     1 as libc::c_uint) as
                                                    uint16_t)
                } else {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         13 as libc::c_uint)
                                                    as uint32_t as uint32_t);
                    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as
                                                    *mut uint16_t,
                                                (((*hspi).RxXferCount as
                                                      libc::c_int >>
                                                      1 as libc::c_uint) as
                                                     libc::c_uint).wrapping_add(1
                                                                                    as
                                                                                    libc::c_uint)
                                                    as uint16_t)
                }
            }
        }
        /* Check if we are in Rx only or in Rx/Tx Mode and configure the DMA transfer complete callback */
        if (*hspi).State as libc::c_uint ==
               HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint {
            /* Set the SPI Rx DMA Half transfer complete callback */
            (*(*hspi).hdmarx).XferHalfCpltCallback =
                Some(SPI_DMAHalfReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            (*(*hspi).hdmarx).XferCpltCallback =
                Some(SPI_DMAReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ())
        } else {
            /* Set the SPI Tx/Rx DMA Half transfer complete callback */
            (*(*hspi).hdmarx).XferHalfCpltCallback =
                Some(SPI_DMAHalfTransmitReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ());
            (*(*hspi).hdmarx).XferCpltCallback =
                Some(SPI_DMATransmitReceiveCplt as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ())
        }
        /* Set the DMA error callback */
        (*(*hspi).hdmarx).XferErrorCallback =
            Some(SPI_DMAError as
                     unsafe extern "C" fn(_: *mut DMA_HandleTypeDef) -> ());
        /* Set the DMA AbortCpltCallback */
        (*(*hspi).hdmarx).XferAbortCallback = None;
        /* Enable the Rx DMA Stream/Channel  */
        HAL_DMA_Start_IT((*hspi).hdmarx,
                         &mut (*(*hspi).Instance).DR as *mut uint32_t as
                             uint32_t, (*hspi).pRxBuffPtr as uint32_t,
                         (*hspi).RxXferCount as uint32_t);
        /* Enable Rx DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Set the SPI Tx DMA transfer complete callback as NULL because the communication closing
  is performed in DMA reception complete callback  */
        (*(*hspi).hdmatx).XferHalfCpltCallback = None;
        (*(*hspi).hdmatx).XferCpltCallback = None;
        (*(*hspi).hdmatx).XferErrorCallback = None;
        (*(*hspi).hdmatx).XferAbortCallback = None;
        /* Enable the Tx DMA Stream/Channel  */
        HAL_DMA_Start_IT((*hspi).hdmatx, (*hspi).pTxBuffPtr as uint32_t,
                         &mut (*(*hspi).Instance).DR as *mut uint32_t as
                             uint32_t, (*hspi).TxXferCount as uint32_t);
        /* Check if the SPI is already enabled */
        if (*(*hspi).Instance).CR1 &
               (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               (0x1 as libc::c_uint) << 6 as libc::c_uint {
            /* Enable SPI peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 6 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Enable the SPI Error Interrupt Bit */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as uint32_t as
                                        uint32_t);
        /* Enable Tx DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             1 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return errorcode;
}
/* *
  * @brief  Abort ongoing transfer (blocking mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort (in case of transfer in DMA mode)
  *           - Set handle State to READY
  * @note   This procedure is executed in blocking mode : when exiting function, Abort is considered as completed.
  * @retval HAL status
*/
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Abort(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Initialized local variable  */
    errorcode = HAL_OK;
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        (*hspi).TxISR =
            Some(SPI_AbortTx_ISR as
                     unsafe extern "C" fn(_: *mut SPI_HandleTypeDef) -> ())
    }
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        (*hspi).RxISR =
            Some(SPI_AbortRx_ISR as
                     unsafe extern "C" fn(_: *mut SPI_HandleTypeDef) -> ())
    }
    while (*hspi).State as libc::c_uint !=
              HAL_SPI_STATE_ABORT as libc::c_int as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >= 0xffffffff as libc::c_uint
           {
            return HAL_TIMEOUT
        }
    }
    /* Clear ERRIE interrupts in case of DMA Mode */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable the SPI DMA Tx or SPI DMA Rx request if enabled */
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint ||
           (*(*hspi).Instance).CR2 &
               (0x1 as libc::c_uint) << 0 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Abort the SPI DMA Tx Stream/Channel : use blocking DMA Abort API (no callback) */
        if !(*hspi).hdmatx.is_null() {
            /* Set the SPI DMA Abort callback :
      will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
            (*(*hspi).hdmatx).XferAbortCallback = None;
            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if HAL_DMA_Abort((*hspi).hdmatx) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            }
            /* Disable Tx DMA Request */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint,
                                      HAL_GetTick()) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            }
            /* Disable SPI Peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   6 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Empty the FRLVL fifo */
            if SPI_WaitFifoStateUntilTimeout(hspi,
                                             (0x3 as libc::c_uint) <<
                                                 9 as libc::c_uint,
                                             0 as libc::c_uint,
                                             100 as libc::c_uint,
                                             HAL_GetTick()) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            }
        }
        /* Abort the SPI DMA Rx Stream/Channel : use blocking DMA Abort API (no callback) */
        if !(*hspi).hdmarx.is_null() {
            /* Set the SPI DMA Abort callback :
      will lead to call HAL_SPI_AbortCpltCallback() at end of DMA abort procedure */
            (*(*hspi).hdmarx).XferAbortCallback = None;
            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if HAL_DMA_Abort((*hspi).hdmarx) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            }
            /* Disable peripheral */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   6 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Control the BSY flag */
            if SPI_WaitFlagStateUntilTimeout(hspi,
                                             (0x1 as libc::c_uint) <<
                                                 7 as libc::c_uint,
                                             RESET as libc::c_int as uint32_t,
                                             100 as libc::c_uint,
                                             HAL_GetTick()) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            }
            /* Empty the FRLVL fifo */
            if SPI_WaitFifoStateUntilTimeout(hspi,
                                             (0x3 as libc::c_uint) <<
                                                 9 as libc::c_uint,
                                             0 as libc::c_uint,
                                             100 as libc::c_uint,
                                             HAL_GetTick()) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            }
            /* Disable Rx DMA Request */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
    }
    /* Reset Tx and Rx transfer counters */
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    /* Check error during Abort procedure */
    if (*hspi).ErrorCode == 0x40 as libc::c_uint {
        /* return HAL_Error in case of error during Abort procedure */
        errorcode = HAL_ERROR
    } else {
        /* Reset errorCode */
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint)
    }
    /* Clear the Error flags in the SR register */
    let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
    ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                (*(*hspi).Instance).DR);
    ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                (*(*hspi).Instance).SR);
    let mut tmpreg_fre: uint32_t = 0 as libc::c_uint;
    ::core::ptr::write_volatile(&mut tmpreg_fre as *mut uint32_t,
                                (*(*hspi).Instance).SR);
    /* Restore hspi->state to ready */
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    return errorcode;
}
/* *
  * @brief  Abort ongoing transfer (Interrupt mode).
  * @param  hspi SPI handle.
  * @note   This procedure could be used for aborting any ongoing transfer (Tx and Rx),
  *         started in Interrupt or DMA mode.
  *         This procedure performs following operations :
  *           - Disable SPI Interrupts (depending of transfer direction)
  *           - Disable the DMA transfer in the peripheral register (if enabled)
  *           - Abort DMA transfer by calling HAL_DMA_Abort_IT (in case of transfer in DMA mode)
  *           - Set handle State to READY
  *           - At abort completion, call user abort complete callback
  * @note   This procedure is executed in Interrupt mode, meaning that abort procedure could be
  *         considered as completed only when user abort complete callback is executed (not when exiting function).
  * @retval HAL status
*/
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_Abort_IT(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut errorcode: HAL_StatusTypeDef = HAL_OK;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    let mut abortcplt: uint32_t = 0;
    /* Initialized local variable  */
    errorcode = HAL_OK;
    abortcplt = 1 as libc::c_uint;
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Change Rx and Tx Irq Handler to Disable TXEIE, RXNEIE and ERRIE interrupts */
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        (*hspi).TxISR =
            Some(SPI_AbortTx_ISR as
                     unsafe extern "C" fn(_: *mut SPI_HandleTypeDef) -> ())
    }
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        (*hspi).RxISR =
            Some(SPI_AbortRx_ISR as
                     unsafe extern "C" fn(_: *mut SPI_HandleTypeDef) -> ())
    }
    while (*hspi).State as libc::c_uint !=
              HAL_SPI_STATE_ABORT as libc::c_int as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >= 0xffffffff as libc::c_uint
           {
            return HAL_TIMEOUT
        }
    }
    /* Clear ERRIE interrupts in case of DMA Mode */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* If DMA Tx and/or DMA Rx Handles are associated to SPI Handle, DMA Abort complete callbacks should be initialised
     before any call to DMA Abort functions */
  /* DMA Tx Handle is valid */
    if !(*hspi).hdmatx.is_null() {
        /* Set DMA Abort Complete callback if UART DMA Tx request if enabled.
       Otherwise, set it to NULL */
        if (*(*hspi).Instance).CR2 &
               (0x1 as libc::c_uint) << 1 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            (*(*hspi).hdmatx).XferAbortCallback =
                Some(SPI_DMATxAbortCallback as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ())
        } else { (*(*hspi).hdmatx).XferAbortCallback = None }
    }
    /* DMA Rx Handle is valid */
    if !(*hspi).hdmarx.is_null() {
        /* Set DMA Abort Complete callback if UART DMA Rx request if enabled.
       Otherwise, set it to NULL */
        if (*(*hspi).Instance).CR2 &
               (0x1 as libc::c_uint) << 0 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            (*(*hspi).hdmarx).XferAbortCallback =
                Some(SPI_DMARxAbortCallback as
                         unsafe extern "C" fn(_: *mut DMA_HandleTypeDef)
                             -> ())
        } else { (*(*hspi).hdmarx).XferAbortCallback = None }
    }
    /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           (*(*hspi).Instance).CR2 &
               (0x1 as libc::c_uint) << 0 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* Abort the SPI DMA Tx Stream/Channel */
        if !(*hspi).hdmatx.is_null() {
            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if HAL_DMA_Abort_IT((*hspi).hdmatx) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                (*(*hspi).hdmatx).XferAbortCallback = None;
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            } else { abortcplt = 0 as libc::c_uint }
        }
        /* Abort the SPI DMA Rx Stream/Channel */
        if !(*hspi).hdmarx.is_null() {
            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if HAL_DMA_Abort_IT((*hspi).hdmarx) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                (*(*hspi).hdmarx).XferAbortCallback = None;
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint);
                abortcplt = 1 as libc::c_uint
            } else { abortcplt = 0 as libc::c_uint }
        }
    }
    /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Abort the SPI DMA Tx Stream/Channel */
        if !(*hspi).hdmatx.is_null() {
            /* Abort DMA Tx Handle linked to SPI Peripheral */
            if HAL_DMA_Abort_IT((*hspi).hdmatx) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                (*(*hspi).hdmatx).XferAbortCallback = None;
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            } else { abortcplt = 0 as libc::c_uint }
        }
    }
    /* Disable the SPI DMA Tx or the SPI Rx request if enabled */
    if (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Abort the SPI DMA Rx Stream/Channel */
        if !(*hspi).hdmarx.is_null() {
            /* Abort DMA Rx Handle linked to SPI Peripheral */
            if HAL_DMA_Abort_IT((*hspi).hdmarx) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                (*(*hspi).hdmarx).XferAbortCallback = None;
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint)
            } else { abortcplt = 0 as libc::c_uint }
        }
    }
    if abortcplt == 1 as libc::c_uint {
        /* Reset Tx and Rx transfer counters */
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        /* Check error during Abort procedure */
        if (*hspi).ErrorCode == 0x40 as libc::c_uint {
            /* return HAL_Error in case of error during Abort procedure */
            errorcode = HAL_ERROR
        } else {
            /* Reset errorCode */
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t, 0 as libc::c_uint)
        }
        /* Clear the Error flags in the SR register */
        let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
        ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                    (*(*hspi).Instance).DR);
        ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                    (*(*hspi).Instance).SR);
        let mut tmpreg_fre: uint32_t = 0 as libc::c_uint;
        ::core::ptr::write_volatile(&mut tmpreg_fre as *mut uint32_t,
                                    (*(*hspi).Instance).SR);
        /* Restore hspi->State to Ready */
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_READY);
        /* As no DMA to be aborted, call directly user Abort complete callback */
        HAL_SPI_AbortCpltCallback(hspi);
    }
    return errorcode;
}
/* *
  * @brief  Pause the DMA Transfer.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_DMAPause(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    /* Disable the SPI DMA Tx & Rx requests */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief  Resume the DMA Transfer.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_DMAResume(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* Process Locked */
    if (*hspi).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hspi).Lock = HAL_LOCKED }
    /* Enable the SPI DMA Tx & Rx requests */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              0 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Process Unlocked */
    (*hspi).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @brief Stop the DMA Transfer.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_DMAStop(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* The Lock is not implemented on this API to allow the user application
     to call the HAL SPI API under callbacks HAL_SPI_TxCpltCallback() or HAL_SPI_RxCpltCallback() or HAL_SPI_TxRxCpltCallback():
     when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed HAL_SPI_TxCpltCallback() or HAL_SPI_RxCpltCallback() or HAL_SPI_TxRxCpltCallback()
     */
    /* Abort the SPI DMA tx Stream/Channel  */
    if !(*hspi).hdmatx.is_null() { HAL_DMA_Abort((*hspi).hdmatx); }
    /* Abort the SPI DMA rx Stream/Channel  */
    if !(*hspi).hdmarx.is_null() { HAL_DMA_Abort((*hspi).hdmarx); }
    /* Disable the SPI DMA Tx & Rx requests */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  Handle SPI interrupt request.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for the specified SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_IRQHandler(mut hspi:
                                                *mut SPI_HandleTypeDef) {
    let mut itsource: uint32_t = (*(*hspi).Instance).CR2;
    let mut itflag: uint32_t = (*(*hspi).Instance).SR;
    /* SPI in mode Receiver ----------------------------------------------------*/
    if itflag & (0x1 as libc::c_uint) << 6 as libc::c_uint ==
           RESET as libc::c_int as libc::c_uint &&
           itflag & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint &&
           itsource & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        (*hspi).RxISR.expect("non-null function pointer")(hspi);
        return
    }
    /* SPI in mode Transmitter -------------------------------------------------*/
    if itflag & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint &&
           itsource & (0x1 as libc::c_uint) << 7 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        (*hspi).TxISR.expect("non-null function pointer")(hspi);
        return
    }
    /* SPI in Error Treatment --------------------------------------------------*/
    if itflag &
           ((0x1 as libc::c_uint) << 5 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 8 as libc::c_uint) !=
           RESET as libc::c_int as libc::c_uint &&
           itsource & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
        /* SPI Overrun error interrupt occurred ----------------------------------*/
        if itflag & (0x1 as libc::c_uint) << 6 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            if (*hspi).State as libc::c_uint !=
                   HAL_SPI_STATE_BUSY_TX as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x4 as libc::c_uint) as
                                                uint32_t as uint32_t);
                let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
                ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                            (*(*hspi).Instance).DR);
                ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                            (*(*hspi).Instance).SR)
            } else {
                let mut tmpreg_ovr_0: uint32_t = 0 as libc::c_uint;
                ::core::ptr::write_volatile(&mut tmpreg_ovr_0 as
                                                *mut uint32_t,
                                            (*(*hspi).Instance).DR);
                ::core::ptr::write_volatile(&mut tmpreg_ovr_0 as
                                                *mut uint32_t,
                                            (*(*hspi).Instance).SR);
                return
            }
        }
        /* SPI Mode Fault error interrupt occurred -------------------------------*/
        if itflag & (0x1 as libc::c_uint) << 5 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t);
            let mut tmpreg_modf: uint32_t = 0 as libc::c_uint;
            ::core::ptr::write_volatile(&mut tmpreg_modf as *mut uint32_t,
                                        (*(*hspi).Instance).SR);
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   6 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        /* SPI Frame error interrupt occurred ------------------------------------*/
        if itflag & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x8 as libc::c_uint) as uint32_t
                                            as uint32_t);
            let mut tmpreg_fre: uint32_t = 0 as libc::c_uint;
            ::core::ptr::write_volatile(&mut tmpreg_fre as *mut uint32_t,
                                        (*(*hspi).Instance).SR)
        }
        if (*hspi).ErrorCode != 0 as libc::c_uint {
            /* Disable all interrupts */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   6 as libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint |
                                                   (0x1 as libc::c_uint) <<
                                                       5 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*hspi).State as
                                            *mut HAL_SPI_StateTypeDef,
                                        HAL_SPI_STATE_READY);
            /* Disable the SPI DMA requests if enabled */
            if itsource & (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint ||
                   itsource & (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                       RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       1 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           0 as libc::c_uint))
                                                as uint32_t as uint32_t);
                /* Abort the SPI DMA Rx channel */
                if !(*hspi).hdmarx.is_null() {
                    /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
                    (*(*hspi).hdmarx).XferAbortCallback =
                        Some(SPI_DMAAbortOnError as
                                 unsafe extern "C" fn(_:
                                                          *mut DMA_HandleTypeDef)
                                     -> ());
                    HAL_DMA_Abort_IT((*hspi).hdmarx);
                }
                /* Abort the SPI DMA Tx channel */
                if !(*hspi).hdmatx.is_null() {
                    /* Set the SPI DMA Abort callback :
          will lead to call HAL_SPI_ErrorCallback() at end of DMA abort procedure */
                    (*(*hspi).hdmatx).XferAbortCallback =
                        Some(SPI_DMAAbortOnError as
                                 unsafe extern "C" fn(_:
                                                          *mut DMA_HandleTypeDef)
                                     -> ());
                    HAL_DMA_Abort_IT((*hspi).hdmatx);
                }
            } else {
                /* Call user error callback */
                HAL_SPI_ErrorCallback(hspi);
            }
        }
        return
    };
}
/* *
  * @brief Tx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TxCpltCallback(mut hspi:
                                                    *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxCpltCallback should be implemented in the user file
   */
}
/* *
  * @brief Rx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_RxCpltCallback(mut hspi:
                                                    *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxCpltCallback should be implemented in the user file
   */
}
/* *
  * @brief Tx and Rx Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TxRxCpltCallback(mut hspi:
                                                      *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxRxCpltCallback should be implemented in the user file
   */
}
/* *
  * @brief Tx Half Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TxHalfCpltCallback(mut hspi:
                                                        *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxHalfCpltCallback should be implemented in the user file
   */
}
/* *
  * @brief Rx Half Transfer completed callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_RxHalfCpltCallback(mut hspi:
                                                        *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_RxHalfCpltCallback() should be implemented in the user file
   */
}
/* *
  * @brief Tx and Rx Half Transfer callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_TxRxHalfCpltCallback(mut hspi:
                                                          *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxRxHalfCpltCallback() should be implemented in the user file
   */
}
/* *
  * @brief SPI error callback.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_ErrorCallback(mut hspi:
                                                   *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_ErrorCallback should be implemented in the user file
   */
  /* NOTE : The ErrorCode parameter in the hspi handle is updated by the SPI processes
            and user can use HAL_SPI_GetError() API to check the latest error occurred
   */
}
/* *
  * @brief  SPI Abort Complete callback.
  * @param  hspi SPI handle.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_AbortCpltCallback(mut hspi:
                                                       *mut SPI_HandleTypeDef) {
    /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_AbortCpltCallback can be implemented in the user file.
   */
}
/* *
  * @}
  */
/* * @defgroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @brief   SPI control functions
  *
@verbatim
 ===============================================================================
                      ##### Peripheral State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the SPI.
     (+) HAL_SPI_GetState() API can be helpful to check in run-time the state of the SPI peripheral
     (+) HAL_SPI_GetError() check in run-time Errors occurring during communication
@endverbatim
  * @{
  */
/* *
  * @brief  Return the SPI handle state.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_GetState(mut hspi: *mut SPI_HandleTypeDef)
 -> HAL_SPI_StateTypeDef {
    /* Return SPI handle state */
    return (*hspi).State;
}
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
/* !< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode */
/* !< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction */
/* !< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size */
/* !< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity */
/* !< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase */
/* !< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management */
/* !< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set. */
/* !< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */
/* !< Specifies if the TI mode is enabled or not.
                                     This parameter can be a value of @ref SPI_TI_mode */
/* !< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation */
/* !< Specifies the polynomial used for the CRC calculation.
                                     This parameter must be an odd number between Min_Data = 0 and Max_Data = 65535 */
/* !< Specifies the CRC Length used for the CRC calculation.
                                     CRC Length is only used with Data8 and Data16, not other data size
                                     This parameter can be a value of @ref SPI_CRC_length */
/* !< Specifies whether the NSSP signal is enabled or not .
                                     This parameter can be a value of @ref SPI_NSSP_Mode
                                     This mode is activated by the NSSP bit in the SPIx_CR2 register and
                                     it takes effect only if the SPI interface is configured as Motorola SPI
                                     master (FRF=0) with capture on the first edge (SPIx_CR1 CPHA = 0,
                                     CPOL setting is ignored).. */
/* *
  * @brief  HAL SPI State structure definition
  */
/* !< Peripheral not Initialized                         */
/* !< Peripheral Initialized and ready for use           */
/* !< an internal process is ongoing                     */
/* !< Data Transmission process is ongoing               */
/* !< Data Reception process is ongoing                  */
/* !< Data Transmission and Reception process is ongoing */
/* !< SPI error state                                    */
/* !< SPI abort is ongoing                               */
/* *
  * @brief  SPI handle Structure definition
  */
/* !< SPI registers base address               */
/* !< SPI communication parameters             */
/* !< Pointer to SPI Tx transfer Buffer        */
/* !< SPI Tx Transfer size                     */
/* !< SPI Tx Transfer Counter                  */
/* !< Pointer to SPI Rx transfer Buffer        */
/* !< SPI Rx Transfer size                     */
/* !< SPI Rx Transfer Counter                  */
/* !< SPI CRC size used for the transfer       */
/* !< function pointer on Rx ISR */
/* !< function pointer on Tx ISR */
/* !< SPI Tx DMA Handle parameters             */
/* !< SPI Rx DMA Handle parameters             */
/* !< Locking object                           */
/* !< SPI communication state                  */
/* !< SPI Error code                           */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */
/* * @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
/* !< No error                               */
/* !< MODF error                             */
/* !< CRC error                              */
/* !< OVR error                              */
/* !< FRE error                              */
/* !< DMA transfer error                     */
/* !< Error on RXNE/TXE/BSY/FTLVL/FRLVL Flag */
/* !< Error during SPI Abort procedure       */
/* *
  * @}
  */
/* * @defgroup SPI_Mode SPI Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Direction SPI Direction Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Slave_Select_management SPI Slave Select Management
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_NSSP_Mode SPI NSS Pulse Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_MSB_LSB_transmission SPI MSB LSB Transmission
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_TI_mode SPI TI Mode
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_Calculation SPI CRC Calculation
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_length SPI CRC Length
  * @{
  * This parameter can be one of the following values:
  *     SPI_CRC_LENGTH_DATASIZE: aligned with the data size
  *     SPI_CRC_LENGTH_8BIT    : CRC 8bit
  *     SPI_CRC_LENGTH_16BIT   : CRC 16bit
  */
/* *
  * @}
  */
/* * @defgroup SPI_FIFO_reception_threshold SPI FIFO Reception Threshold
  * @{
  * This parameter can be one of the following values:
  *     SPI_RXFIFO_THRESHOLD or SPI_RXFIFO_THRESHOLD_QF :
  *          RXNE event is generated if the FIFO
  *          level is greater or equal to 1/2(16-bits).
  *     SPI_RXFIFO_THRESHOLD_HF: RXNE event is generated if the FIFO
  *          level is greater or equal to 1/4(8 bits). */
/* *
  * @}
  */
/* * @defgroup SPI_Interrupt_definition SPI Interrupt Definition
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Flags_definition SPI Flags Definition
  * @{
  */
/* SPI status flag: Rx buffer not empty flag       */
/* SPI status flag: Tx buffer empty flag           */
/* SPI status flag: Busy flag                      */
/* SPI Error flag: CRC error flag                  */
/* SPI Error flag: Mode fault flag                 */
/* SPI Error flag: Overrun flag                    */
/* SPI Error flag: TI mode frame format error flag */
/* SPI fifo transmission level                     */
/* SPI fifo reception level                        */
/* *
  * @}
  */
/* * @defgroup SPI_transmission_fifo_status_level SPI Transmission FIFO Status Level
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_reception_fifo_status_level SPI Reception FIFO Status Level
  * @{
  */
/* *
  * @}
  */
/* Exported macros -----------------------------------------------------------*/
/* * @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */
/* * @brief  Reset SPI handle state.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Enable or disable the specified SPI interrupts.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__: specifies the interrupt source to enable or disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
/* * @brief  Check whether the specified SPI interrupt source is enabled or not.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__: specifies the SPI interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
/* * @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @param  __FLAG__: specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  *            @arg SPI_FLAG_FRE: Frame format error flag
  *            @arg SPI_FLAG_FTLVL: SPI fifo transmission level
  *            @arg SPI_FLAG_FRLVL: SPI fifo reception level
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
/* * @brief  Clear the SPI CRCERR pending flag.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Clear the SPI MODF pending flag.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Clear the SPI OVR pending flag.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Clear the SPI FRE pending flag.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Enable the SPI peripheral.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Disable the SPI peripheral.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup SPI_Private_Macros SPI Private Macros
  * @{
  */
/* * @brief  Set the SPI transmit-only mode.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Set the SPI receive-only mode.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* * @brief  Reset the CRC calculation of the SPI.
  * @param  __HANDLE__: specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2, or 3 to select the SPI peripheral.
  * @retval None
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup SPI_Exported_Functions
  * @{
  */
/* * @addtogroup SPI_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
/* *
  * @}
  */
/* * @addtogroup SPI_Exported_Functions_Group2 IO operation functions
  * @{
  */
/* I/O operation functions  ***************************************************/
/* Transfer Abort functions */
/* *
  * @}
  */
/* * @addtogroup SPI_Exported_Functions_Group3 Peripheral State and Errors functions
  * @{
  */
/* Peripheral State and Error functions ***************************************/
/* *
  * @brief  Return the SPI error code.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval SPI error code in bitmap format
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_SPI_GetError(mut hspi: *mut SPI_HandleTypeDef)
 -> uint32_t {
    /* Return SPI ErrorCode */
    return (*hspi).ErrorCode;
}
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* * @defgroup SPI_Private_Functions SPI Private Functions
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup SPI_Private_Functions
  * @brief   Private functions
  * @{
  */
/* *
  * @brief DMA SPI transmit process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMATransmitCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* DMA Normal Mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
           (0x1 as libc::c_uint) << 8 as libc::c_uint {
        /* Disable ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Disable Tx DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               1 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Check the end of the transaction */
        if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x20 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* Clear overrun flag in 2 Lines communication mode because received data is not read */
        if (*hspi).Init.Direction == 0 as libc::c_uint {
            let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
            ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                        (*(*hspi).Instance).DR);
            ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                        (*(*hspi).Instance).SR)
        }
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_READY);
        if (*hspi).ErrorCode != 0 as libc::c_uint {
            HAL_SPI_ErrorCallback(hspi);
            return
        }
    }
    HAL_SPI_TxCpltCallback(hspi);
}
/* *
  * @brief DMA SPI receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMAReceiveCplt(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* USE_SPI_CRC */
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    /* DMA Normal Mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
           (0x1 as libc::c_uint) << 8 as libc::c_uint {
        /* Disable ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* USE_SPI_CRC */
        /* Disable Rx/Tx DMA Request (done by default to handle the case master rx direction 2 lines) */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               1 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Check the end of the transaction */
        if SPI_EndRxTransaction(hspi, 100 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        0x20 as libc::c_uint)
        }
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_READY);
        /* USE_SPI_CRC */
        if (*hspi).ErrorCode != 0 as libc::c_uint {
            HAL_SPI_ErrorCallback(hspi);
            return
        }
    }
    HAL_SPI_RxCpltCallback(hspi);
}
/* *
  * @brief  DMA SPI transmit receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMATransmitReceiveCplt(mut hdma:
                                                    *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* USE_SPI_CRC */
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    /* DMA Normal Mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
           (0x1 as libc::c_uint) << 8 as libc::c_uint {
        /* Disable ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* USE_SPI_CRC */
        /* Check the end of the transaction */
        if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint, tickstart) as
               libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x20 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
        /* Disable Rx/Tx DMA Request */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               1 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    0 as libc::c_uint as uint16_t);
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_READY);
        /* USE_SPI_CRC */
        if (*hspi).ErrorCode != 0 as libc::c_uint {
            HAL_SPI_ErrorCallback(hspi);
            return
        }
    }
    HAL_SPI_TxRxCpltCallback(hspi);
}
/* *
  * @brief  DMA SPI half transmit process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMAHalfTransmitCplt(mut hdma:
                                                 *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    HAL_SPI_TxHalfCpltCallback(hspi);
}
/* *
  * @brief  DMA SPI half receive process complete callback
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMAHalfReceiveCplt(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    HAL_SPI_RxHalfCpltCallback(hspi);
}
/* *
  * @brief  DMA SPI half transmit receive process complete callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMAHalfTransmitReceiveCplt(mut hdma:
                                                        *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    HAL_SPI_TxRxHalfCpltCallback(hspi);
}
/* *
  * @brief  DMA SPI communication error callback.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA module.
  * @retval None
  */
unsafe extern "C" fn SPI_DMAError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    /* Stop the disable DMA transfer on SPI side */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x10 as libc::c_uint)
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    HAL_SPI_ErrorCallback(hspi);
}
/* *
  * @brief  DMA SPI communication abort callback, when initiated by HAL services on Error
  *         (To be called at end of DMA Abort procedure following error occurrence).
  * @param  hdma DMA handle.
  * @retval None
  */
unsafe extern "C" fn SPI_DMAAbortOnError(mut hdma: *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    HAL_SPI_ErrorCallback(hspi);
}
/* *
  * @brief  DMA SPI Tx communication abort callback, when initiated by user
  *         (To be called at end of DMA Tx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Rx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
unsafe extern "C" fn SPI_DMATxAbortCallback(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    (*(*hspi).hdmatx).XferAbortCallback = None;
    /* Disable Tx DMA Request */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    /* Disable SPI Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Empty the FRLVL fifo */
    if SPI_WaitFifoStateUntilTimeout(hspi,
                                     (0x3 as libc::c_uint) <<
                                         9 as libc::c_uint, 0 as libc::c_uint,
                                     100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    /* Check if an Abort process is still ongoing */
    if !(*hspi).hdmarx.is_null() {
        if (*(*hspi).hdmarx).XferAbortCallback.is_some() { return }
    }
    /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    /* Check no error during Abort procedure */
    if (*hspi).ErrorCode != 0x40 as libc::c_uint {
        /* Reset errorCode */
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint)
    }
    /* Clear the Error flags in the SR register */
    let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
    ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                (*(*hspi).Instance).DR);
    ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                (*(*hspi).Instance).SR);
    let mut tmpreg_fre: uint32_t = 0 as libc::c_uint;
    ::core::ptr::write_volatile(&mut tmpreg_fre as *mut uint32_t,
                                (*(*hspi).Instance).SR);
    /* Restore hspi->State to Ready */
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    /* Call user Abort complete callback */
    HAL_SPI_AbortCpltCallback(hspi);
}
/* *
  * @brief  DMA SPI Rx communication abort callback, when initiated by user
  *         (To be called at end of DMA Rx Abort procedure following user abort request).
  * @note   When this callback is executed, User Abort complete call back is called only if no
  *         Abort still ongoing for Tx DMA Handle.
  * @param  hdma DMA handle.
  * @retval None
  */
unsafe extern "C" fn SPI_DMARxAbortCallback(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut hspi: *mut SPI_HandleTypeDef =
        (*hdma).Parent as *mut SPI_HandleTypeDef;
    /* Disable SPI Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    (*(*hspi).hdmarx).XferAbortCallback = None;
    /* Disable Rx DMA Request */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Control the BSY flag */
    if SPI_WaitFlagStateUntilTimeout(hspi,
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint,
                                     RESET as libc::c_int as uint32_t,
                                     100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    /* Empty the FRLVL fifo */
    if SPI_WaitFifoStateUntilTimeout(hspi,
                                     (0x3 as libc::c_uint) <<
                                         9 as libc::c_uint, 0 as libc::c_uint,
                                     100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    /* Check if an Abort process is still ongoing */
    if !(*hspi).hdmatx.is_null() {
        if (*(*hspi).hdmatx).XferAbortCallback.is_some() { return }
    }
    /* No Abort process still ongoing : All DMA Stream/Channel are aborted, call user Abort Complete callback */
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                0 as libc::c_uint as uint16_t);
    /* Check no error during Abort procedure */
    if (*hspi).ErrorCode != 0x40 as libc::c_uint {
        /* Reset errorCode */
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint)
    }
    /* Clear the Error flags in the SR register */
    let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
    ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                (*(*hspi).Instance).DR);
    ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                (*(*hspi).Instance).SR);
    let mut tmpreg_fre: uint32_t = 0 as libc::c_uint;
    ::core::ptr::write_volatile(&mut tmpreg_fre as *mut uint32_t,
                                (*(*hspi).Instance).SR);
    /* Restore hspi->State to Ready */
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    /* Call user Abort complete callback */
    HAL_SPI_AbortCpltCallback(hspi);
}
/* *
  * @brief  Rx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_2linesRxISR_8BIT(mut hspi:
                                              *mut __SPI_HandleTypeDef) {
    /* Receive data in packing mode */
    if (*hspi).RxXferCount as libc::c_uint > 1 as libc::c_uint {
        *((*hspi).pRxBuffPtr as *mut uint16_t) =
            (*(*hspi).Instance).DR as uint16_t;
        (*hspi).pRxBuffPtr =
            (*hspi).pRxBuffPtr.offset(::core::mem::size_of::<uint16_t>() as
                                          libc::c_ulong as isize);
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                                as
                                                                                *const uint16_t)
                                         as
                                         libc::c_uint).wrapping_sub(2 as
                                                                        libc::c_uint)
                                        as uint16_t as uint16_t);
        if (*hspi).RxXferCount as libc::c_uint == 1 as libc::c_uint {
            /* set fiforxthreshold according the reception data length: 8bit */
            ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 12 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
    } else {
        /* Receive data in 8 Bit mode */
        let fresh3 = (*hspi).pRxBuffPtr;
        (*hspi).pRxBuffPtr = (*hspi).pRxBuffPtr.offset(1);
        *fresh3 =
            *(&mut (*(*hspi).Instance).DR as *mut uint32_t as *mut uint8_t);
        ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                    ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                               as
                                                                               *const uint16_t).wrapping_sub(1))
    }
    /* check end of the reception */
    if (*hspi).RxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        /* Disable RXNE  and ERR interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   5 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        if (*hspi).TxXferCount as libc::c_uint == 0 as libc::c_uint {
            SPI_CloseRxTx_ISR(hspi);
        }
    };
}
/* USE_SPI_CRC */
/* *
  * @brief  Tx 8-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_2linesTxISR_8BIT(mut hspi:
                                              *mut __SPI_HandleTypeDef) {
    /* Transmit data in packing Bit mode */
    if (*hspi).TxXferCount as libc::c_uint >= 2 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as
                                        *mut uint32_t,
                                    *((*hspi).pTxBuffPtr as *mut uint16_t) as
                                        uint32_t);
        (*hspi).pTxBuffPtr =
            (*hspi).pTxBuffPtr.offset(::core::mem::size_of::<uint16_t>() as
                                          libc::c_ulong as isize);
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                                as
                                                                                *const uint16_t)
                                         as
                                         libc::c_uint).wrapping_sub(2 as
                                                                        libc::c_uint)
                                        as uint16_t as uint16_t)
    } else {
        /* Transmit data in 8 Bit mode */
        let fresh4 = (*hspi).pTxBuffPtr;
        (*hspi).pTxBuffPtr = (*hspi).pTxBuffPtr.offset(1);
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as
                                        *mut uint32_t as *mut uint8_t,
                                    *fresh4);
        ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                    ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                               as
                                                                               *const uint16_t).wrapping_sub(1))
    }
    /* check the end of the transmission */
    if (*hspi).TxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        /* Disable TXE interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*hspi).RxXferCount as libc::c_uint == 0 as libc::c_uint {
            SPI_CloseRxTx_ISR(hspi);
        }
    };
}
/* *
  * @brief  Rx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_2linesRxISR_16BIT(mut hspi:
                                               *mut __SPI_HandleTypeDef) {
    /* Receive data in 16 Bit mode */
    *((*hspi).pRxBuffPtr as *mut uint16_t) =
        (*(*hspi).Instance).DR as uint16_t;
    (*hspi).pRxBuffPtr =
        (*hspi).pRxBuffPtr.offset(::core::mem::size_of::<uint16_t>() as
                                      libc::c_ulong as isize);
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                           as
                                                                           *const uint16_t).wrapping_sub(1));
    if (*hspi).RxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        /* Disable RXNE interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*hspi).TxXferCount as libc::c_uint == 0 as libc::c_uint {
            SPI_CloseRxTx_ISR(hspi);
        }
    };
}
/* USE_SPI_CRC */
/* *
  * @brief  Tx 16-bit handler for Transmit and Receive in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_2linesTxISR_16BIT(mut hspi:
                                               *mut __SPI_HandleTypeDef) {
    /* Transmit data in 16 Bit mode */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as *mut uint32_t,
                                *((*hspi).pTxBuffPtr as *mut uint16_t) as
                                    uint32_t);
    (*hspi).pTxBuffPtr =
        (*hspi).pTxBuffPtr.offset(::core::mem::size_of::<uint16_t>() as
                                      libc::c_ulong as isize);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                           as
                                                                           *const uint16_t).wrapping_sub(1));
    /* Enable CRC Transmission */
    if (*hspi).TxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        /* Disable TXE interrupt */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               7 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*hspi).RxXferCount as libc::c_uint == 0 as libc::c_uint {
            SPI_CloseRxTx_ISR(hspi);
        }
    };
}
/* USE_SPI_CRC */
/* *
  * @brief  Manage the receive 8-bit in Interrupt context.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_RxISR_8BIT(mut hspi: *mut __SPI_HandleTypeDef) {
    let fresh5 = (*hspi).pRxBuffPtr;
    (*hspi).pRxBuffPtr = (*hspi).pRxBuffPtr.offset(1);
    *fresh5 = *(&mut (*(*hspi).Instance).DR as *mut uint32_t as *mut uint8_t);
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                           as
                                                                           *const uint16_t).wrapping_sub(1));
    /* USE_SPI_CRC */
    if (*hspi).RxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        SPI_CloseRx_ISR(hspi);
    };
}
/* USE_SPI_CRC */
/* *
  * @brief  Manage the 16-bit receive in Interrupt context.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_RxISR_16BIT(mut hspi: *mut __SPI_HandleTypeDef) {
    *((*hspi).pRxBuffPtr as *mut uint16_t) =
        (*(*hspi).Instance).DR as uint16_t;
    (*hspi).pRxBuffPtr =
        (*hspi).pRxBuffPtr.offset(::core::mem::size_of::<uint16_t>() as
                                      libc::c_ulong as isize);
    ::core::ptr::write_volatile(&mut (*hspi).RxXferCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).RxXferCount
                                                                           as
                                                                           *const uint16_t).wrapping_sub(1));
    /* USE_SPI_CRC */
    if (*hspi).RxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        SPI_CloseRx_ISR(hspi);
    };
}
/* *
  * @brief  Handle the data 8-bit transmit in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_TxISR_8BIT(mut hspi: *mut __SPI_HandleTypeDef) {
    let fresh6 = (*hspi).pTxBuffPtr;
    (*hspi).pTxBuffPtr = (*hspi).pTxBuffPtr.offset(1);
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as *mut uint32_t
                                    as *mut uint8_t, *fresh6);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                           as
                                                                           *const uint16_t).wrapping_sub(1));
    if (*hspi).TxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        SPI_CloseTx_ISR(hspi);
    };
}
/* *
  * @brief  Handle the data 16-bit transmit in Interrupt mode.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_TxISR_16BIT(mut hspi: *mut __SPI_HandleTypeDef) {
    /* Transmit data in 16 Bit mode */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).DR as *mut uint32_t,
                                *((*hspi).pTxBuffPtr as *mut uint16_t) as
                                    uint32_t);
    (*hspi).pTxBuffPtr =
        (*hspi).pTxBuffPtr.offset(::core::mem::size_of::<uint16_t>() as
                                      libc::c_ulong as isize);
    ::core::ptr::write_volatile(&mut (*hspi).TxXferCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&(*hspi).TxXferCount
                                                                           as
                                                                           *const uint16_t).wrapping_sub(1));
    if (*hspi).TxXferCount as libc::c_uint == 0 as libc::c_uint {
        /* USE_SPI_CRC */
        SPI_CloseTx_ISR(hspi);
    };
}
/* *
  * @brief Handle SPI Communication Timeout.
  * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Flag: SPI flag to check
  * @param State: flag state to check
  * @param Timeout: Timeout duration
  * @param Tickstart: tick start value
  * @retval HAL status
  */
unsafe extern "C" fn SPI_WaitFlagStateUntilTimeout(mut hspi:
                                                       *mut SPI_HandleTypeDef,
                                                   mut Flag: uint32_t,
                                                   mut State: uint32_t,
                                                   mut Timeout: uint32_t,
                                                   mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    while (*(*hspi).Instance).SR & Flag != State {
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(Tickstart) >= Timeout {
                /* Disable the SPI and reset the CRC: the CRC value should be cleared
        on both master and slave sides in order to resynchronize the master
        and slave for their respective CRC calculation */
                /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           5 as libc::c_uint))
                                                as uint32_t as uint32_t);
                if (*hspi).Init.Mode ==
                       (0x1 as libc::c_uint) << 2 as libc::c_uint |
                           (0x1 as libc::c_uint) << 8 as libc::c_uint &&
                       ((*hspi).Init.Direction ==
                            (0x1 as libc::c_uint) << 15 as libc::c_uint ||
                            (*hspi).Init.Direction ==
                                (0x1 as libc::c_uint) << 10 as libc::c_uint) {
                    /* Disable SPI peripheral */
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                }
                /* Reset CRC Calculation */
                if (*hspi).Init.CRCCalculation ==
                       (0x1 as libc::c_uint) << 13 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           13 as libc::c_uint)
                                                         as uint16_t as
                                                         libc::c_uint) as
                                                    uint32_t as uint32_t);
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         13 as libc::c_uint)
                                                    as uint32_t as uint32_t)
                }
                ::core::ptr::write_volatile(&mut (*hspi).State as
                                                *mut HAL_SPI_StateTypeDef,
                                            HAL_SPI_STATE_READY);
                /* Process Unlocked */
                (*hspi).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @brief Handle SPI FIFO Communication Timeout.
  * @param hspi: pointer to a SPI_HandleTypeDef structure that contains
  *              the configuration information for SPI module.
  * @param Fifo: Fifo to check
  * @param State: Fifo state to check
  * @param Timeout: Timeout duration
  * @param Tickstart: tick start value
  * @retval HAL status
  */
unsafe extern "C" fn SPI_WaitFifoStateUntilTimeout(mut hspi:
                                                       *mut SPI_HandleTypeDef,
                                                   mut Fifo: uint32_t,
                                                   mut State: uint32_t,
                                                   mut Timeout: uint32_t,
                                                   mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tmpreg: uint8_t = 0;
    while (*(*hspi).Instance).SR & Fifo != State {
        if Fifo == (0x3 as libc::c_uint) << 9 as libc::c_uint &&
               State == 0 as libc::c_uint {
            ::core::ptr::write_volatile(&mut tmpreg as *mut uint8_t,
                                        *(&mut (*(*hspi).Instance).DR as
                                              *mut uint32_t as *mut uint8_t))
        }
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_int as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(Tickstart) >= Timeout {
                /* Disable the SPI and reset the CRC: the CRC value should be cleared
           on both master and slave sides in order to resynchronize the master
           and slave for their respective CRC calculation */
                /* Disable TXE, RXNE and ERR interrupts for the interrupt process */
                ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       7 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           5 as libc::c_uint))
                                                as uint32_t as uint32_t);
                if (*hspi).Init.Mode ==
                       (0x1 as libc::c_uint) << 2 as libc::c_uint |
                           (0x1 as libc::c_uint) << 8 as libc::c_uint &&
                       ((*hspi).Init.Direction ==
                            (0x1 as libc::c_uint) << 15 as libc::c_uint ||
                            (*hspi).Init.Direction ==
                                (0x1 as libc::c_uint) << 10 as libc::c_uint) {
                    /* Disable SPI peripheral */
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           6 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                }
                /* Reset CRC Calculation */
                if (*hspi).Init.CRCCalculation ==
                       (0x1 as libc::c_uint) << 13 as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           13 as libc::c_uint)
                                                         as uint16_t as
                                                         libc::c_uint) as
                                                    uint32_t as uint32_t);
                    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1
                                                    as *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint |
                                                     (0x1 as libc::c_uint) <<
                                                         13 as libc::c_uint)
                                                    as uint32_t as uint32_t)
                }
                ::core::ptr::write_volatile(&mut (*hspi).State as
                                                *mut HAL_SPI_StateTypeDef,
                                            HAL_SPI_STATE_READY);
                /* Process Unlocked */
                (*hspi).Lock = HAL_UNLOCKED;
                return HAL_TIMEOUT
            }
        }
    }
    return HAL_OK;
}
/* *
  * @brief  Handle the check of the RX transaction complete.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @param  Timeout: Timeout duration
  * @param  Tickstart: tick start value
  * @retval None.
  */
unsafe extern "C" fn SPI_EndRxTransaction(mut hspi: *mut SPI_HandleTypeDef,
                                          mut Timeout: uint32_t,
                                          mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    if (*hspi).Init.Mode ==
           (0x1 as libc::c_uint) << 2 as libc::c_uint |
               (0x1 as libc::c_uint) << 8 as libc::c_uint &&
           ((*hspi).Init.Direction ==
                (0x1 as libc::c_uint) << 15 as libc::c_uint ||
                (*hspi).Init.Direction ==
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) {
        /* Disable SPI peripheral */
        ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               6 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Control the BSY flag */
    if SPI_WaitFlagStateUntilTimeout(hspi,
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint,
                                     RESET as libc::c_int as uint32_t,
                                     Timeout, Tickstart) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_TIMEOUT
    }
    if (*hspi).Init.Mode ==
           (0x1 as libc::c_uint) << 2 as libc::c_uint |
               (0x1 as libc::c_uint) << 8 as libc::c_uint &&
           ((*hspi).Init.Direction ==
                (0x1 as libc::c_uint) << 15 as libc::c_uint ||
                (*hspi).Init.Direction ==
                    (0x1 as libc::c_uint) << 10 as libc::c_uint) {
        /* Empty the FRLVL fifo */
        if SPI_WaitFifoStateUntilTimeout(hspi,
                                         (0x3 as libc::c_uint) <<
                                             9 as libc::c_uint,
                                         0 as libc::c_uint, Timeout,
                                         Tickstart) as libc::c_uint !=
               HAL_OK as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x20 as libc::c_uint) as uint32_t
                                            as uint32_t);
            return HAL_TIMEOUT
        }
    }
    return HAL_OK;
}
/* *
  * @brief Handle the check of the RXTX or TX transaction complete.
  * @param hspi: SPI handle
  * @param Timeout: Timeout duration
  * @param Tickstart: tick start value
  */
unsafe extern "C" fn SPI_EndRxTxTransaction(mut hspi: *mut SPI_HandleTypeDef,
                                            mut Timeout: uint32_t,
                                            mut Tickstart: uint32_t)
 -> HAL_StatusTypeDef {
    /* Control if the TX fifo is empty */
    if SPI_WaitFifoStateUntilTimeout(hspi,
                                     (0x3 as libc::c_uint) <<
                                         11 as libc::c_uint,
                                     0 as libc::c_uint, Timeout, Tickstart) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_TIMEOUT
    }
    /* Control the BSY flag */
    if SPI_WaitFlagStateUntilTimeout(hspi,
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint,
                                     RESET as libc::c_int as uint32_t,
                                     Timeout, Tickstart) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_TIMEOUT
    }
    /* Control if the RX fifo is empty */
    if SPI_WaitFifoStateUntilTimeout(hspi,
                                     (0x3 as libc::c_uint) <<
                                         9 as libc::c_uint, 0 as libc::c_uint,
                                     Timeout, Tickstart) as libc::c_uint !=
           HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t);
        return HAL_TIMEOUT
    }
    return HAL_OK;
}
/* *
  * @brief  Handle the end of the RXTX transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_CloseRxTx_ISR(mut hspi: *mut SPI_HandleTypeDef) {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Disable ERR interrupt */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           5 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check the end of the transaction */
    if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint, tickstart) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* USE_SPI_CRC */
    if (*hspi).ErrorCode == 0 as libc::c_uint {
        if (*hspi).State as libc::c_uint ==
               HAL_SPI_STATE_BUSY_RX as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hspi).State as
                                            *mut HAL_SPI_StateTypeDef,
                                        HAL_SPI_STATE_READY);
            HAL_SPI_RxCpltCallback(hspi);
        } else {
            ::core::ptr::write_volatile(&mut (*hspi).State as
                                            *mut HAL_SPI_StateTypeDef,
                                        HAL_SPI_STATE_READY);
            HAL_SPI_TxRxCpltCallback(hspi);
        }
    } else {
        ::core::ptr::write_volatile(&mut (*hspi).State as
                                        *mut HAL_SPI_StateTypeDef,
                                    HAL_SPI_STATE_READY);
        HAL_SPI_ErrorCallback(hspi);
    };
    /* USE_SPI_CRC */
}
/* *
  * @brief  Handle the end of the RX transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_CloseRx_ISR(mut hspi: *mut SPI_HandleTypeDef) {
    /* Disable RXNE and ERR interrupt */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Check the end of the transaction */
    if SPI_EndRxTransaction(hspi, 100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    /* USE_SPI_CRC */
    if (*hspi).ErrorCode == 0 as libc::c_uint {
        HAL_SPI_RxCpltCallback(hspi);
    } else { HAL_SPI_ErrorCallback(hspi); };
    /* USE_SPI_CRC */
}
/* *
  * @brief  Handle the end of the TX transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_CloseTx_ISR(mut hspi: *mut SPI_HandleTypeDef) {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Init tickstart for timeout management*/
    tickstart = HAL_GetTick();
    /* Disable TXE and ERR interrupt */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           7 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Check the end of the transaction */
    if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint, tickstart) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*hspi).ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Clear overrun flag in 2 Lines communication mode because received is not read */
    if (*hspi).Init.Direction == 0 as libc::c_uint {
        let mut tmpreg_ovr: uint32_t = 0 as libc::c_uint;
        ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                    (*(*hspi).Instance).DR);
        ::core::ptr::write_volatile(&mut tmpreg_ovr as *mut uint32_t,
                                    (*(*hspi).Instance).SR)
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_READY);
    if (*hspi).ErrorCode != 0 as libc::c_uint {
        HAL_SPI_ErrorCallback(hspi);
    } else { HAL_SPI_TxCpltCallback(hspi); };
}
/* USE_SPI_CRC */
/* *
  * @brief  Handle abort a Rx transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_AbortRx_ISR(mut hspi: *mut SPI_HandleTypeDef) {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Disable SPI Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           7 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               6 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    while (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 6 as libc::c_uint
              != RESET as libc::c_int as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >= 0xffffffff as libc::c_uint
           {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        0x40 as libc::c_uint)
        }
    }
    /* Control the BSY flag */
    if SPI_WaitFlagStateUntilTimeout(hspi,
                                     (0x1 as libc::c_uint) <<
                                         7 as libc::c_uint,
                                     RESET as libc::c_int as uint32_t,
                                     100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    /* Empty the FRLVL fifo */
    if SPI_WaitFifoStateUntilTimeout(hspi,
                                     (0x3 as libc::c_uint) <<
                                         9 as libc::c_uint, 0 as libc::c_uint,
                                     100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_ABORT);
}
/* *
  * @brief  Handle abort a Tx or Rx/Tx transaction.
  * @param  hspi: pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
unsafe extern "C" fn SPI_AbortTx_ISR(mut hspi: *mut SPI_HandleTypeDef) {
    let mut tickstart: uint32_t = 0 as libc::c_uint;
    /* Init tickstart for timeout managment*/
    tickstart = HAL_GetTick();
    /* Disable TXEIE, RXNEIE and ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           7 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               6 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               5 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    while (*(*hspi).Instance).CR2 & (0x1 as libc::c_uint) << 7 as libc::c_uint
              != RESET as libc::c_int as libc::c_uint {
        if HAL_GetTick().wrapping_sub(tickstart) >= 0xffffffff as libc::c_uint
           {
            ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as
                                            *mut uint32_t,
                                        0x40 as libc::c_uint)
        }
    }
    if SPI_EndRxTxTransaction(hspi, 100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    /* Disable SPI Peripheral */
    ::core::ptr::write_volatile(&mut (*(*hspi).Instance).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hspi).Instance).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           6 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Empty the FRLVL fifo */
    if SPI_WaitFifoStateUntilTimeout(hspi,
                                     (0x3 as libc::c_uint) <<
                                         9 as libc::c_uint, 0 as libc::c_uint,
                                     100 as libc::c_uint, HAL_GetTick()) as
           libc::c_uint != HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hspi).ErrorCode as *mut uint32_t,
                                    0x40 as libc::c_uint)
    }
    ::core::ptr::write_volatile(&mut (*hspi).State as
                                    *mut HAL_SPI_StateTypeDef,
                                HAL_SPI_STATE_ABORT);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_SPI_MODULE_ENABLED */
/* *
  * @}
  */
