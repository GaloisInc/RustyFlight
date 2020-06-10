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
    fn HAL_GetTick() -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
pub type HAL_DMA_StateTypeDef = libc::c_uint;
pub const HAL_DMA_STATE_ABORT: HAL_DMA_StateTypeDef = 5;
pub const HAL_DMA_STATE_ERROR: HAL_DMA_StateTypeDef = 4;
pub const HAL_DMA_STATE_TIMEOUT: HAL_DMA_StateTypeDef = 3;
pub const HAL_DMA_STATE_BUSY: HAL_DMA_StateTypeDef = 2;
pub const HAL_DMA_STATE_READY: HAL_DMA_StateTypeDef = 1;
pub const HAL_DMA_STATE_RESET: HAL_DMA_StateTypeDef = 0;
pub type HAL_DMA_LevelCompleteTypeDef = libc::c_uint;
pub const HAL_DMA_HALF_TRANSFER: HAL_DMA_LevelCompleteTypeDef = 1;
pub const HAL_DMA_FULL_TRANSFER: HAL_DMA_LevelCompleteTypeDef = 0;
pub type HAL_DMA_CallbackIDTypeDef = libc::c_uint;
pub const HAL_DMA_XFER_ALL_CB_ID: HAL_DMA_CallbackIDTypeDef = 6;
pub const HAL_DMA_XFER_ABORT_CB_ID: HAL_DMA_CallbackIDTypeDef = 5;
pub const HAL_DMA_XFER_ERROR_CB_ID: HAL_DMA_CallbackIDTypeDef = 4;
pub const HAL_DMA_XFER_M1HALFCPLT_CB_ID: HAL_DMA_CallbackIDTypeDef = 3;
pub const HAL_DMA_XFER_M1CPLT_CB_ID: HAL_DMA_CallbackIDTypeDef = 2;
pub const HAL_DMA_XFER_HALFCPLT_CB_ID: HAL_DMA_CallbackIDTypeDef = 1;
pub const HAL_DMA_XFER_CPLT_CB_ID: HAL_DMA_CallbackIDTypeDef = 0;
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
pub struct DMA_Base_Registers {
    pub ISR: uint32_t,
    pub Reserved0: uint32_t,
    pub IFCR: uint32_t,
}
/* *
  * @}
  */
/* Exported functions ---------------------------------------------------------*/
/* * @addtogroup DMA_Exported_Functions
  * @{
  */
/* * @addtogroup DMA_Exported_Functions_Group1
  *
@verbatim
 ===============================================================================
             ##### Initialization and de-initialization functions  #####
 ===============================================================================
    [..]
    This section provides functions allowing to initialize the DMA Stream source
    and destination addresses, incrementation and data sizes, transfer direction, 
    circular/normal mode selection, memory-to-memory mode selection and Stream priority value.
    [..]
    The HAL_DMA_Init() function follows the DMA configuration procedures as described in
    reference manual.

@endverbatim
  * @{
  */
/* *
  * @brief  Initialize the DMA according to the specified
  *         parameters in the DMA_InitTypeDef and create the associated handle.
  * @param  hdma: Pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_Init(mut hdma: *mut DMA_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut tmp: uint32_t = 0 as libc::c_uint;
    let mut tickstart: uint32_t = HAL_GetTick();
    let mut regs: *mut DMA_Base_Registers = 0 as *mut DMA_Base_Registers;
    /* Check the DMA peripheral state */
    if hdma.is_null() { return HAL_ERROR }
    /* Check the parameters */
    /* Check the memory burst, peripheral burst and FIFO threshold parameters only
     when FIFO mode is enabled */
    ((*hdma).Init.FIFOMode) != 0 as libc::c_uint;
    /* Allocate lock resource */
    (*hdma).Lock = HAL_UNLOCKED;
    /* Change DMA peripheral state */
    ::core::ptr::write_volatile(&mut (*hdma).State as
                                    *mut HAL_DMA_StateTypeDef,
                                HAL_DMA_STATE_BUSY);
    /* Disable the peripheral */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check if the DMA Stream is effectively disabled */
    while (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 0 as libc::c_uint
              != RESET as libc::c_int as libc::c_uint {
        /* Check for the Timeout */
        if HAL_GetTick().wrapping_sub(tickstart) >
               5 as libc::c_int as uint32_t {
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        0x20 as libc::c_uint);
            /* Change the DMA state */
            ::core::ptr::write_volatile(&mut (*hdma).State as
                                            *mut HAL_DMA_StateTypeDef,
                                        HAL_DMA_STATE_TIMEOUT);
            return HAL_TIMEOUT
        }
    }
    /* Get the CR register value */
    tmp = (*(*hdma).Instance).CR;
    /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC, DIR, CT and DBM bits */
    tmp &=
        !((0xf as libc::c_uint) << 25 as libc::c_uint |
              (0x3 as libc::c_uint) << 23 as libc::c_uint |
              (0x3 as libc::c_uint) << 21 as libc::c_uint |
              (0x3 as libc::c_uint) << 16 as libc::c_uint |
              (0x3 as libc::c_uint) << 13 as libc::c_uint |
              (0x3 as libc::c_uint) << 11 as libc::c_uint |
              (0x1 as libc::c_uint) << 10 as libc::c_uint |
              (0x1 as libc::c_uint) << 9 as libc::c_uint |
              (0x1 as libc::c_uint) << 8 as libc::c_uint |
              (0x3 as libc::c_uint) << 6 as libc::c_uint |
              (0x1 as libc::c_uint) << 19 as libc::c_uint |
              (0x1 as libc::c_uint) << 18 as libc::c_uint);
    /* Prepare the DMA Stream configuration */
    tmp |=
        (*hdma).Init.Channel | (*hdma).Init.Direction | (*hdma).Init.PeriphInc
            | (*hdma).Init.MemInc | (*hdma).Init.PeriphDataAlignment |
            (*hdma).Init.MemDataAlignment | (*hdma).Init.Mode |
            (*hdma).Init.Priority;
    /* the Memory burst and peripheral burst are not used when the FIFO is disabled */
    if (*hdma).Init.FIFOMode == (0x1 as libc::c_uint) << 2 as libc::c_uint {
        /* Get memory burst and peripheral burst */
        tmp |= (*hdma).Init.MemBurst | (*hdma).Init.PeriphBurst
    }
    /* Write to DMA Stream CR register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as *mut uint32_t,
                                tmp);
    /* Get the FCR register value */
    tmp = (*(*hdma).Instance).FCR;
    /* Clear Direct mode and FIFO threshold bits */
    tmp &=
        !((0x1 as libc::c_uint) << 2 as libc::c_uint |
              (0x3 as libc::c_uint) << 0 as libc::c_uint);
    /* Prepare the DMA Stream FIFO configuration */
    tmp |= (*hdma).Init.FIFOMode;
    /* The FIFO threshold is not used when the FIFO mode is disabled */
    if (*hdma).Init.FIFOMode == (0x1 as libc::c_uint) << 2 as libc::c_uint {
        /* Get the FIFO threshold */
        tmp |= (*hdma).Init.FIFOThreshold;
        /* Check compatibility between FIFO threshold level and size of the memory burst */
    /* for INCR4, INCR8, INCR16 bursts */
        if (*hdma).Init.MemBurst != 0 as libc::c_uint {
            if DMA_CheckFifoParam(hdma) as libc::c_uint !=
                   HAL_OK as libc::c_int as libc::c_uint {
                /* Update error code */
                ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                                *mut uint32_t,
                                            0x40 as libc::c_uint);
                /* Change the DMA state */
                ::core::ptr::write_volatile(&mut (*hdma).State as
                                                *mut HAL_DMA_StateTypeDef,
                                            HAL_DMA_STATE_READY);
                return HAL_ERROR
            }
        }
    }
    /* Write to DMA Stream FCR */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).FCR as *mut uint32_t,
                                tmp);
    /* Initialize StreamBaseAddress and StreamIndex parameters to be used to calculate
     DMA steam Base Address needed by HAL_DMA_IRQHandler() and HAL_DMA_PollForTransfer() */
    regs = DMA_CalcBaseAndBitshift(hdma) as *mut DMA_Base_Registers;
    /* Clear all interrupt flags */
    ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                (0x3f as libc::c_uint) <<
                                    (*hdma).StreamIndex);
    /* Initialize the error code */
    ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Initialize the DMA state */
    ::core::ptr::write_volatile(&mut (*hdma).State as
                                    *mut HAL_DMA_StateTypeDef,
                                HAL_DMA_STATE_READY);
    return HAL_OK;
}
/* *
  * @brief  DeInitializes the DMA peripheral 
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_DeInit(mut hdma: *mut DMA_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut regs: *mut DMA_Base_Registers = 0 as *mut DMA_Base_Registers;
    /* Check the DMA peripheral state */
    if hdma.is_null() { return HAL_ERROR }
    /* Check the DMA peripheral state */
    if (*hdma).State as libc::c_uint ==
           HAL_DMA_STATE_BUSY as libc::c_int as libc::c_uint {
        /* Return error status */
        return HAL_BUSY
    }
    /* Check the parameters */
    /* Disable the selected DMA Streamx */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Reset DMA Streamx control register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as *mut uint32_t,
                                0 as libc::c_uint);
    /* Reset DMA Streamx number of data to transfer register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).NDTR as
                                    *mut uint32_t, 0 as libc::c_uint);
    /* Reset DMA Streamx peripheral address register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).PAR as *mut uint32_t,
                                0 as libc::c_uint);
    /* Reset DMA Streamx memory 0 address register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M0AR as
                                    *mut uint32_t, 0 as libc::c_uint);
    /* Reset DMA Streamx memory 1 address register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M1AR as
                                    *mut uint32_t, 0 as libc::c_uint);
    /* Reset DMA Streamx FIFO control register */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).FCR as *mut uint32_t,
                                0x21 as libc::c_uint);
    /* Get DMA steam Base Address */
    regs = DMA_CalcBaseAndBitshift(hdma) as *mut DMA_Base_Registers;
    /* Clear all interrupt flags at correct offset within the register */
    ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                (0x3f as libc::c_uint) <<
                                    (*hdma).StreamIndex);
    /* Initialize the error code */
    ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Initialize the DMA state */
    ::core::ptr::write_volatile(&mut (*hdma).State as
                                    *mut HAL_DMA_StateTypeDef,
                                HAL_DMA_STATE_RESET);
    /* Release Lock */
    (*hdma).Lock = HAL_UNLOCKED;
    return HAL_OK;
}
/* *
  * @}
  */
/* * @addtogroup DMA_Exported_Functions_Group2
  *
@verbatim   
 ===============================================================================
                      #####  IO operation functions  #####
 ===============================================================================
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and Start DMA transfer
      (+) Configure the source, destination address and data length and 
          Start DMA transfer with interrupt
      (+) Abort DMA transfer
      (+) Poll for transfer complete
      (+) Handle DMA interrupt request  

@endverbatim
  * @{
  */
/* *
  * @brief  Starts the DMA Transfer.
  * @param  hdma      : pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_Start(mut hdma: *mut DMA_HandleTypeDef,
                                       mut SrcAddress: uint32_t,
                                       mut DstAddress: uint32_t,
                                       mut DataLength: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Process locked */
    if (*hdma).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdma).Lock = HAL_LOCKED }
    if HAL_DMA_STATE_READY as libc::c_int as libc::c_uint ==
           (*hdma).State as libc::c_uint {
        /* Change DMA peripheral state */
        ::core::ptr::write_volatile(&mut (*hdma).State as
                                        *mut HAL_DMA_StateTypeDef,
                                    HAL_DMA_STATE_BUSY);
        /* Initialize the error code */
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Configure the source, destination address and the data length */
        DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        /* Process unlocked */
        (*hdma).Lock = HAL_UNLOCKED;
        /* Return error status */
        status = HAL_BUSY
    }
    return status;
}
/* *
  * @brief  Start the DMA Transfer with interrupt enabled.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_Start_IT(mut hdma: *mut DMA_HandleTypeDef,
                                          mut SrcAddress: uint32_t,
                                          mut DstAddress: uint32_t,
                                          mut DataLength: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* calculate DMA base and stream number */
    let mut regs: *mut DMA_Base_Registers =
        (*hdma).StreamBaseAddress as *mut DMA_Base_Registers;
    /* Check the parameters */
    /* Process locked */
    if (*hdma).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdma).Lock = HAL_LOCKED }
    if HAL_DMA_STATE_READY as libc::c_int as libc::c_uint ==
           (*hdma).State as libc::c_uint {
        /* Change DMA peripheral state */
        ::core::ptr::write_volatile(&mut (*hdma).State as
                                        *mut HAL_DMA_StateTypeDef,
                                    HAL_DMA_STATE_BUSY);
        /* Initialize the error code */
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0 as libc::c_uint);
        /* Configure the source, destination address and the data length */
        DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);
        /* Clear all interrupt flags at correct offset within the register */
        ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                    (0x3f as libc::c_uint) <<
                                        (*hdma).StreamIndex);
        /* Enable Common interrupts*/
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         ((0x1 as libc::c_uint) <<
                                              4 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).FCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).FCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80 as libc::c_uint) as uint32_t as
                                        uint32_t);
        if (*hdma).XferHalfCpltCallback.is_some() {
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 3 as libc::c_uint) as
                                            uint32_t as uint32_t)
        }
        /* Enable the Peripheral */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        /* Process unlocked */
        (*hdma).Lock = HAL_UNLOCKED;
        /* Return error status */
        status = HAL_BUSY
    }
    return status;
}
/* *
  * @brief  Aborts the DMA Transfer.
  * @param  hdma  : pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  *                   
  * @note  After disabling a DMA Stream, a check for wait until the DMA Stream is 
  *        effectively disabled is added. If a Stream is disabled 
  *        while a data transfer is ongoing, the current data will be transferred
  *        and the Stream will be effectively disabled only after the transfer of
  *        this single data is finished.  
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_Abort(mut hdma: *mut DMA_HandleTypeDef)
 -> HAL_StatusTypeDef {
    /* calculate DMA base and stream number */
    let mut regs: *mut DMA_Base_Registers =
        (*hdma).StreamBaseAddress as *mut DMA_Base_Registers;
    let mut tickstart: uint32_t = HAL_GetTick();
    if (*hdma).State as libc::c_uint !=
           HAL_DMA_STATE_BUSY as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0x80 as libc::c_uint);
        /* Process Unlocked */
        (*hdma).Lock = HAL_UNLOCKED;
        return HAL_ERROR
    } else {
        /* Disable all the transfer interrupts */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               4 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint |
                                               (0x1 as libc::c_uint) <<
                                                   1 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).FCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).FCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        if (*hdma).XferHalfCpltCallback.is_some() ||
               (*hdma).XferM1HalfCpltCallback.is_some() {
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   3 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        }
        /* Disable the stream */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t);
        /* Check if the DMA Stream is effectively disabled */
        while (*(*hdma).Instance).CR &
                  (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                  RESET as libc::c_int as libc::c_uint {
            /* Check for the Timeout */
            if HAL_GetTick().wrapping_sub(tickstart) >
                   5 as libc::c_int as uint32_t {
                /* Update error code */
                ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                                *mut uint32_t,
                                            0x20 as libc::c_uint);
                /* Process Unlocked */
                (*hdma).Lock = HAL_UNLOCKED;
                /* Change the DMA state */
                ::core::ptr::write_volatile(&mut (*hdma).State as
                                                *mut HAL_DMA_StateTypeDef,
                                            HAL_DMA_STATE_TIMEOUT);
                return HAL_TIMEOUT
            }
        }
        /* Clear all interrupt flags at correct offset within the register */
        ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                    (0x3f as libc::c_uint) <<
                                        (*hdma).StreamIndex);
        /* Process Unlocked */
        (*hdma).Lock = HAL_UNLOCKED;
        /* Change the DMA state*/
        ::core::ptr::write_volatile(&mut (*hdma).State as
                                        *mut HAL_DMA_StateTypeDef,
                                    HAL_DMA_STATE_READY)
    }
    return HAL_OK;
}
/* *
  * @brief  Aborts the DMA Transfer in Interrupt mode.
  * @param  hdma  : pointer to a DMA_HandleTypeDef structure that contains
  *                 the configuration information for the specified DMA Stream.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_Abort_IT(mut hdma: *mut DMA_HandleTypeDef)
 -> HAL_StatusTypeDef {
    if (*hdma).State as libc::c_uint !=
           HAL_DMA_STATE_BUSY as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0x80 as libc::c_uint);
        return HAL_ERROR
    } else {
        /* Set Abort State  */
        ::core::ptr::write_volatile(&mut (*hdma).State as
                                        *mut HAL_DMA_StateTypeDef,
                                    HAL_DMA_STATE_ABORT);
        /* Disable the stream */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_uint) <<
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    return HAL_OK;
}
/* *
  * @brief  Polling for transfer complete.
  * @param  hdma:          pointer to a DMA_HandleTypeDef structure that contains
  *                        the configuration information for the specified DMA Stream.
  * @param  CompleteLevel: Specifies the DMA level complete.
  * @note   The polling mode is kept in this version for legacy. it is recommanded to use the IT model instead.
  *         This model could be used for debug purpose.
  * @note   The HAL_DMA_PollForTransfer API cannot be used in circular and double buffering mode (automatic circular mode). 
  * @param  Timeout:       Timeout duration.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_PollForTransfer(mut hdma:
                                                     *mut DMA_HandleTypeDef,
                                                 mut CompleteLevel:
                                                     HAL_DMA_LevelCompleteTypeDef,
                                                 mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    let mut mask_cpltlevel: uint32_t = 0;
    let mut tickstart: uint32_t = HAL_GetTick();
    let mut tmpisr: uint32_t = 0;
    /* calculate DMA base and stream number */
    let mut regs: *mut DMA_Base_Registers = 0 as *mut DMA_Base_Registers;
    if HAL_DMA_STATE_BUSY as libc::c_int as libc::c_uint !=
           (*hdma).State as libc::c_uint {
        /* No transfer ongoing */
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0x80 as libc::c_uint);
        (*hdma).Lock = HAL_UNLOCKED;
        return HAL_ERROR
    }
    /* Polling mode not supported in circular mode and double buffering mode */
    if (*(*hdma).Instance).CR & (0x1 as libc::c_uint) << 8 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0x100 as libc::c_uint);
        return HAL_ERROR
    }
    /* Get the level transfer complete flag */
    if CompleteLevel as libc::c_uint ==
           HAL_DMA_FULL_TRANSFER as libc::c_int as libc::c_uint {
        /* Transfer Complete flag */
        mask_cpltlevel = (0x20 as libc::c_uint) << (*hdma).StreamIndex
    } else {
        /* Half Transfer Complete flag */
        mask_cpltlevel = (0x10 as libc::c_uint) << (*hdma).StreamIndex
    }
    regs = (*hdma).StreamBaseAddress as *mut DMA_Base_Registers;
    tmpisr = (*regs).ISR;
    while tmpisr & mask_cpltlevel == RESET as libc::c_int as libc::c_uint &&
              (*hdma).ErrorCode & 0x1 as libc::c_uint ==
                  RESET as libc::c_int as libc::c_uint {
        /* Check for the Timeout (Not applicable in circular mode)*/
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_int as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(tickstart) > Timeout {
                /* Update error code */
                ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                                *mut uint32_t,
                                            0x20 as libc::c_uint);
                /* Process Unlocked */
                (*hdma).Lock = HAL_UNLOCKED;
                /* Change the DMA state */
                ::core::ptr::write_volatile(&mut (*hdma).State as
                                                *mut HAL_DMA_StateTypeDef,
                                            HAL_DMA_STATE_READY);
                return HAL_TIMEOUT
            }
        }
        /* Get the ISR register value */
        tmpisr = (*regs).ISR;
        if tmpisr & (0x8 as libc::c_uint) << (*hdma).StreamIndex !=
               RESET as libc::c_int as libc::c_uint {
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hdma).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t);
            /* Clear the transfer error flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x8 as libc::c_uint) <<
                                            (*hdma).StreamIndex)
        }
        if tmpisr & (0x800001 as libc::c_uint) << (*hdma).StreamIndex !=
               RESET as libc::c_int as libc::c_uint {
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hdma).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x2 as libc::c_uint) as uint32_t
                                            as uint32_t);
            /* Clear the FIFO error flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x800001 as libc::c_uint) <<
                                            (*hdma).StreamIndex)
        }
        if tmpisr & (0x800004 as libc::c_uint) << (*hdma).StreamIndex !=
               RESET as libc::c_int as libc::c_uint {
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hdma).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t);
            /* Clear the Direct Mode error flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x800004 as libc::c_uint) <<
                                            (*hdma).StreamIndex)
        }
    }
    if (*hdma).ErrorCode != 0 as libc::c_uint {
        if (*hdma).ErrorCode & 0x1 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            HAL_DMA_Abort(hdma);
            /* Clear the half transfer and transfer complete flags */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x10 as libc::c_uint |
                                             0x20 as libc::c_uint) <<
                                            (*hdma).StreamIndex);
            /* Process Unlocked */
            (*hdma).Lock = HAL_UNLOCKED;
            /* Change the DMA state */
            ::core::ptr::write_volatile(&mut (*hdma).State as
                                            *mut HAL_DMA_StateTypeDef,
                                        HAL_DMA_STATE_READY);
            return HAL_ERROR
        }
    }
    /* Get the level transfer complete flag */
    if CompleteLevel as libc::c_uint ==
           HAL_DMA_FULL_TRANSFER as libc::c_int as libc::c_uint {
        /* Clear the half transfer and transfer complete flags */
        ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                    (0x10 as libc::c_uint |
                                         0x20 as libc::c_uint) <<
                                        (*hdma).StreamIndex);
        /* Process Unlocked */
        (*hdma).Lock = HAL_UNLOCKED;
        ::core::ptr::write_volatile(&mut (*hdma).State as
                                        *mut HAL_DMA_StateTypeDef,
                                    HAL_DMA_STATE_READY)
    } else {
        /* Clear the half transfer and transfer complete flags */
        ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                    (0x10 as libc::c_uint) <<
                                        (*hdma).StreamIndex)
    }
    return status;
}
/* *
  * @brief  Handles DMA interrupt request.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_IRQHandler(mut hdma:
                                                *mut DMA_HandleTypeDef) {
    let mut tmpisr: uint32_t = 0;
    let mut count: uint32_t = 0 as libc::c_int as uint32_t;
    let mut timeout: uint32_t =
        SystemCoreClock.wrapping_div(9600 as libc::c_int as libc::c_uint);
    /* calculate DMA base and stream number */
    let mut regs: *mut DMA_Base_Registers =
        (*hdma).StreamBaseAddress as *mut DMA_Base_Registers;
    tmpisr = (*regs).ISR;
    /* Transfer Error Interrupt management ***************************************/
    if tmpisr & (0x8 as libc::c_uint) << (*hdma).StreamIndex !=
           RESET as libc::c_int as libc::c_uint {
        if (if (0x1 as libc::c_uint) << 2 as libc::c_uint !=
                   0x80 as libc::c_uint {
                ((*(*hdma).Instance).CR) &
                    (0x1 as libc::c_uint) << 2 as libc::c_uint
            } else {
                ((*(*hdma).Instance).FCR) &
                    (0x1 as libc::c_uint) << 2 as libc::c_uint
            }) != RESET as libc::c_int as libc::c_uint {
            /* Disable the transfer error interrupt */
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            /* Clear the transfer error flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x8 as libc::c_uint) <<
                                            (*hdma).StreamIndex);
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hdma).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    /* FIFO Error Interrupt management ******************************************/
    if tmpisr & (0x800001 as libc::c_uint) << (*hdma).StreamIndex !=
           RESET as libc::c_int as libc::c_uint {
        if (if 0x80 as libc::c_uint != 0x80 as libc::c_uint {
                ((*(*hdma).Instance).CR) & 0x80 as libc::c_uint
            } else { ((*(*hdma).Instance).FCR) & 0x80 as libc::c_uint }) !=
               RESET as libc::c_int as libc::c_uint {
            /* Clear the FIFO error flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x800001 as libc::c_uint) <<
                                            (*hdma).StreamIndex);
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hdma).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x2 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    /* Direct Mode Error Interrupt management ***********************************/
    if tmpisr & (0x800004 as libc::c_uint) << (*hdma).StreamIndex !=
           RESET as libc::c_int as libc::c_uint {
        if (if (0x1 as libc::c_uint) << 1 as libc::c_uint !=
                   0x80 as libc::c_uint {
                ((*(*hdma).Instance).CR) &
                    (0x1 as libc::c_uint) << 1 as libc::c_uint
            } else {
                ((*(*hdma).Instance).FCR) &
                    (0x1 as libc::c_uint) << 1 as libc::c_uint
            }) != RESET as libc::c_int as libc::c_uint {
            /* Clear the direct mode error flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x800004 as libc::c_uint) <<
                                            (*hdma).StreamIndex);
            /* Update error code */
            ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*hdma).ErrorCode
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x4 as libc::c_uint) as uint32_t
                                            as uint32_t)
        }
    }
    /* Half Transfer Complete Interrupt management ******************************/
    if tmpisr & (0x10 as libc::c_uint) << (*hdma).StreamIndex !=
           RESET as libc::c_int as libc::c_uint {
        if (if (0x1 as libc::c_uint) << 3 as libc::c_uint !=
                   0x80 as libc::c_uint {
                ((*(*hdma).Instance).CR) &
                    (0x1 as libc::c_uint) << 3 as libc::c_uint
            } else {
                ((*(*hdma).Instance).FCR) &
                    (0x1 as libc::c_uint) << 3 as libc::c_uint
            }) != RESET as libc::c_int as libc::c_uint {
            /* Clear the half transfer complete flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x10 as libc::c_uint) <<
                                            (*hdma).StreamIndex);
            /* Multi_Buffering mode enabled */
            if (*(*hdma).Instance).CR &
                   (0x1 as libc::c_uint) << 18 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
                /* Current memory buffer used is Memory 0 */
                if (*(*hdma).Instance).CR &
                       (0x1 as libc::c_uint) << 19 as libc::c_uint ==
                       RESET as libc::c_int as libc::c_uint {
                    if (*hdma).XferHalfCpltCallback.is_some() {
                        /* Half transfer callback */
                        (*hdma).XferHalfCpltCallback.expect("non-null function pointer")(hdma);
                    }
                } else if (*hdma).XferM1HalfCpltCallback.is_some() {
                    /* Current memory buffer used is Memory 1 */
                    /* Half transfer callback */
                    (*hdma).XferM1HalfCpltCallback.expect("non-null function pointer")(hdma);
                }
            } else {
                /* Disable the half transfer interrupt if the DMA mode is not CIRCULAR */
                if (*(*hdma).Instance).CR &
                       (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                       RESET as libc::c_int as libc::c_uint {
                    /* Disable the half transfer interrupt */
                    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                                    *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                }
                if (*hdma).XferHalfCpltCallback.is_some() {
                    /* Half transfer callback */
                    (*hdma).XferHalfCpltCallback.expect("non-null function pointer")(hdma);
                }
            }
        }
    }
    /* Transfer Complete Interrupt management ***********************************/
    if tmpisr & (0x20 as libc::c_uint) << (*hdma).StreamIndex !=
           RESET as libc::c_int as libc::c_uint {
        if (if (0x1 as libc::c_uint) << 4 as libc::c_uint !=
                   0x80 as libc::c_uint {
                ((*(*hdma).Instance).CR) &
                    (0x1 as libc::c_uint) << 4 as libc::c_uint
            } else {
                ((*(*hdma).Instance).FCR) &
                    (0x1 as libc::c_uint) << 4 as libc::c_uint
            }) != RESET as libc::c_int as libc::c_uint {
            /* Clear the transfer complete flag */
            ::core::ptr::write_volatile(&mut (*regs).IFCR as *mut uint32_t,
                                        (0x20 as libc::c_uint) <<
                                            (*hdma).StreamIndex);
            if HAL_DMA_STATE_ABORT as libc::c_int as libc::c_uint ==
                   (*hdma).State as libc::c_uint {
                /* Disable all the transfer interrupts */
                ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       4 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           2 as libc::c_uint |
                                                       (0x1 as libc::c_uint)
                                                           <<
                                                           1 as libc::c_uint))
                                                as uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*(*hdma).Instance).FCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).FCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x80 as libc::c_uint)) as
                                                uint32_t as uint32_t);
                if (*hdma).XferHalfCpltCallback.is_some() ||
                       (*hdma).XferM1HalfCpltCallback.is_some() {
                    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                                    *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint))
                                                    as uint32_t as uint32_t)
                }
                /* Clear all interrupt flags at correct offset within the register */
                ::core::ptr::write_volatile(&mut (*regs).IFCR as
                                                *mut uint32_t,
                                            (0x3f as libc::c_uint) <<
                                                (*hdma).StreamIndex);
                /* Process Unlocked */
                (*hdma).Lock = HAL_UNLOCKED;
                /* Change the DMA state */
                ::core::ptr::write_volatile(&mut (*hdma).State as
                                                *mut HAL_DMA_StateTypeDef,
                                            HAL_DMA_STATE_READY);
                if (*hdma).XferAbortCallback.is_some() {
                    (*hdma).XferAbortCallback.expect("non-null function pointer")(hdma);
                }
                return
            }
            if (*(*hdma).Instance).CR &
                   (0x1 as libc::c_uint) << 18 as libc::c_uint !=
                   RESET as libc::c_int as libc::c_uint {
                /* Current memory buffer used is Memory 0 */
                if (*(*hdma).Instance).CR &
                       (0x1 as libc::c_uint) << 19 as libc::c_uint ==
                       RESET as libc::c_int as libc::c_uint {
                    if (*hdma).XferM1CpltCallback.is_some() {
                        /* Transfer complete Callback for memory1 */
                        (*hdma).XferM1CpltCallback.expect("non-null function pointer")(hdma);
                    }
                } else if (*hdma).XferCpltCallback.is_some() {
                    /* Current memory buffer used is Memory 1 */
                    /* Transfer complete Callback for memory0 */
                    (*hdma).XferCpltCallback.expect("non-null function pointer")(hdma);
                }
            } else {
                /* Disable the transfer complete interrupt if the DMA mode is not CIRCULAR */
                if (*(*hdma).Instance).CR &
                       (0x1 as libc::c_uint) << 8 as libc::c_uint ==
                       RESET as libc::c_int as libc::c_uint {
                    /* Disable the transfer complete interrupt */
                    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                                    *mut uint32_t,
                                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                            as
                                                                                            *const uint32_t)
                                                     as libc::c_uint &
                                                     !((0x1 as libc::c_uint)
                                                           <<
                                                           4 as libc::c_uint))
                                                    as uint32_t as uint32_t);
                    /* Process Unlocked */
                    (*hdma).Lock = HAL_UNLOCKED;
                    /* Change the DMA state */
                    ::core::ptr::write_volatile(&mut (*hdma).State as
                                                    *mut HAL_DMA_StateTypeDef,
                                                HAL_DMA_STATE_READY)
                }
                if (*hdma).XferCpltCallback.is_some() {
                    /* Transfer complete callback */
                    (*hdma).XferCpltCallback.expect("non-null function pointer")(hdma);
                }
            }
        }
    }
    /* manage error case */
    if (*hdma).ErrorCode != 0 as libc::c_uint {
        if (*hdma).ErrorCode & 0x1 as libc::c_uint !=
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*hdma).State as
                                            *mut HAL_DMA_StateTypeDef,
                                        HAL_DMA_STATE_ABORT);
            /* Disable the stream */
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   0 as libc::c_uint)) as
                                            uint32_t as uint32_t);
            loop  {
                ::core::ptr::write_volatile(&mut count as *mut uint32_t,
                                            ::core::ptr::read_volatile::<uint32_t>(&count
                                                                                       as
                                                                                       *const uint32_t).wrapping_add(1));
                if ::core::ptr::read_volatile::<uint32_t>(&count as
                                                              *const uint32_t)
                       > timeout {
                    break ;
                }
                if !((*(*hdma).Instance).CR &
                         (0x1 as libc::c_uint) << 0 as libc::c_uint !=
                         RESET as libc::c_int as libc::c_uint) {
                    break ;
                }
            }
            /* Process Unlocked */
            (*hdma).Lock = HAL_UNLOCKED;
            /* Change the DMA state */
            ::core::ptr::write_volatile(&mut (*hdma).State as
                                            *mut HAL_DMA_StateTypeDef,
                                        HAL_DMA_STATE_READY)
        }
        if (*hdma).XferErrorCallback.is_some() {
            /* Transfer error callback */
            (*hdma).XferErrorCallback.expect("non-null function pointer")(hdma);
        }
    };
}
/* *
  * @brief  Register callbacks
  * @param  hdma:                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Stream.
  * @param  CallbackID:           User Callback identifer
  *                               a DMA_HandleTypeDef structure as parameter.
  * @param  pCallback:            pointer to private callbacsk function which has pointer to 
  *                               a DMA_HandleTypeDef structure as parameter.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_RegisterCallback(mut hdma:
                                                      *mut DMA_HandleTypeDef,
                                                  mut CallbackID:
                                                      HAL_DMA_CallbackIDTypeDef,
                                                  mut pCallback:
                                                      Option<unsafe extern "C" fn(_:
                                                                                      *mut DMA_HandleTypeDef)
                                                                 -> ()>)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Process locked */
    if (*hdma).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdma).Lock = HAL_LOCKED }
    if HAL_DMA_STATE_READY as libc::c_int as libc::c_uint ==
           (*hdma).State as libc::c_uint {
        match CallbackID as libc::c_uint {
            0 => { (*hdma).XferCpltCallback = pCallback }
            1 => { (*hdma).XferHalfCpltCallback = pCallback }
            2 => { (*hdma).XferM1CpltCallback = pCallback }
            3 => { (*hdma).XferM1HalfCpltCallback = pCallback }
            4 => { (*hdma).XferErrorCallback = pCallback }
            5 => { (*hdma).XferAbortCallback = pCallback }
            _ => { }
        }
    } else {
        /* Return error status */
        status = HAL_ERROR
    }
    /* Release Lock */
    (*hdma).Lock = HAL_UNLOCKED;
    return status;
}
/* *
  * @brief  UnRegister callbacks
  * @param  hdma:                 pointer to a DMA_HandleTypeDef structure that contains
  *                               the configuration information for the specified DMA Stream.
  * @param  CallbackID:           User Callback identifer
  *                               a HAL_DMA_CallbackIDTypeDef ENUM as parameter.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_UnRegisterCallback(mut hdma:
                                                        *mut DMA_HandleTypeDef,
                                                    mut CallbackID:
                                                        HAL_DMA_CallbackIDTypeDef)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Process locked */
    if (*hdma).Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { (*hdma).Lock = HAL_LOCKED }
    if HAL_DMA_STATE_READY as libc::c_int as libc::c_uint ==
           (*hdma).State as libc::c_uint {
        match CallbackID as libc::c_uint {
            0 => { (*hdma).XferCpltCallback = None }
            1 => { (*hdma).XferHalfCpltCallback = None }
            2 => { (*hdma).XferM1CpltCallback = None }
            3 => { (*hdma).XferM1HalfCpltCallback = None }
            4 => { (*hdma).XferErrorCallback = None }
            5 => { (*hdma).XferAbortCallback = None }
            6 => {
                (*hdma).XferCpltCallback = None;
                (*hdma).XferHalfCpltCallback = None;
                (*hdma).XferM1CpltCallback = None;
                (*hdma).XferM1HalfCpltCallback = None;
                (*hdma).XferErrorCallback = None;
                (*hdma).XferAbortCallback = None
            }
            _ => { status = HAL_ERROR }
        }
    } else { status = HAL_ERROR }
    /* Release Lock */
    (*hdma).Lock = HAL_UNLOCKED;
    return status;
}
/* *
  * @}
  */
/* * @addtogroup DMA_Exported_Functions_Group3
  *
@verbatim
 ===============================================================================
                    ##### State and Errors functions #####
 ===============================================================================
    [..]
    This subsection provides functions allowing to
      (+) Check the DMA state
      (+) Get error code

@endverbatim
  * @{
  */
/* *
  * @brief  Returns the DMA state.
  * @param  hdma: pointer to a DMA_HandleTypeDef structure that contains
  *               the configuration information for the specified DMA Stream.
  * @retval HAL state
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_GetState(mut hdma: *mut DMA_HandleTypeDef)
 -> HAL_DMA_StateTypeDef {
    return (*hdma).State;
}
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
/* !< Specifies the channel used for the specified stream. 
                                      This parameter can be a value of @ref DMAEx_Channel_selection                  */
/* !< Specifies if the data will be transferred from memory to peripheral, 
                                      from memory to memory or from peripheral to memory.
                                      This parameter can be a value of @ref DMA_Data_transfer_direction              */
/* !< Specifies whether the Peripheral address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Peripheral_incremented_mode          */
/* !< Specifies whether the memory address register should be incremented or not.
                                      This parameter can be a value of @ref DMA_Memory_incremented_mode              */
/* !< Specifies the Peripheral data width.
                                      This parameter can be a value of @ref DMA_Peripheral_data_size                 */
/* !< Specifies the Memory data width.
                                      This parameter can be a value of @ref DMA_Memory_data_size                     */
/* !< Specifies the operation mode of the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_mode
                                      @note The circular buffer mode cannot be used if the memory-to-memory
                                            data transfer is configured on the selected Stream                        */
/* !< Specifies the software priority for the DMAy Streamx.
                                      This parameter can be a value of @ref DMA_Priority_level                       */
/* !< Specifies if the FIFO mode or Direct mode will be used for the specified stream.
                                      This parameter can be a value of @ref DMA_FIFO_direct_mode
                                      @note The Direct mode (FIFO mode disabled) cannot be used if the 
                                            memory-to-memory data transfer is configured on the selected stream       */
/* !< Specifies the FIFO threshold level.
                                      This parameter can be a value of @ref DMA_FIFO_threshold_level                  */
/* !< Specifies the Burst transfer configuration for the memory transfers. 
                                      It specifies the amount of data to be transferred in a single non interruptible 
                                      transaction.
                                      This parameter can be a value of @ref DMA_Memory_burst 
                                      @note The burst mode is possible only if the address Increment mode is enabled. */
/* !< Specifies the Burst transfer configuration for the peripheral transfers. 
                                      It specifies the amount of data to be transferred in a single non interruptible 
                                      transaction. 
                                      This parameter can be a value of @ref DMA_Peripheral_burst
                                      @note The burst mode is possible only if the address Increment mode is enabled. */
/* * 
  * @brief  HAL DMA State structures definition
  */
/* !< DMA not yet initialized or disabled */
/* !< DMA initialized and ready for use   */
/* !< DMA process is ongoing              */
/* !< DMA timeout state                   */
/* !< DMA error state                     */
/* !< DMA Abort state                     */
/* * 
  * @brief  HAL DMA Error Code structure definition
  */
/* !< Full transfer     */
/* !< Half Transfer     */
/* * 
  * @brief  HAL DMA Error Code structure definition
  */
/* !< Full transfer     */
/* !< Half Transfer     */
/* !< M1 Full Transfer  */
/* !< M1 Half Transfer  */
/* !< Error             */
/* !< Abort             */
/* !< All               */
/* * 
  * @brief  DMA handle Structure definition
  */
/* !< Register base address                  */
/* !< DMA communication parameters           */
/* !< DMA locking object                     */
/* !< DMA transfer state                     */
/* !< Parent object state                    */
/* !< DMA transfer complete callback         */
/* !< DMA Half transfer complete callback    */
/* !< DMA transfer complete Memory1 callback */
/* !< DMA transfer Half complete Memory1 callback */
/* !< DMA transfer error callback            */
/* !< DMA transfer Abort callback            */
/* !< DMA Error code                          */
/* !< DMA Stream Base Address                */
/* !< DMA Stream Index                       */
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DMA_Exported_Constants DMA Exported Constants
  * @brief    DMA Exported constants 
  * @{
  */
/* * @defgroup DMA_Error_Code DMA Error Code
  * @brief    DMA Error Code 
  * @{
  */
/* !< No error                               */
/* !< Transfer error                         */
/* !< FIFO error                             */
/* !< Direct Mode error                      */
/* !< Timeout error                          */
/* !< Parameter error                        */
/* !< Abort requested with no Xfer ongoing   */
/* !< Not supported mode                     */
/* *
  * @}
  */
/* * @defgroup DMA_Data_transfer_direction DMA Data transfer direction
  * @brief    DMA data transfer direction 
  * @{
  */
/* !< Peripheral to memory direction */
/* !< Memory to peripheral direction */
/* !< Memory to memory direction     */
/* *
  * @}
  */
/* * @defgroup DMA_Peripheral_incremented_mode DMA Peripheral incremented mode
  * @brief    DMA peripheral incremented mode 
  * @{
  */
/* !< Peripheral increment mode enable  */
/* !< Peripheral increment mode disable */
/* *
  * @}
  */
/* * @defgroup DMA_Memory_incremented_mode DMA Memory incremented mode
  * @brief    DMA memory incremented mode 
  * @{
  */
/* !< Memory increment mode enable  */
/* !< Memory increment mode disable */
/* *
  * @}
  */
/* * @defgroup DMA_Peripheral_data_size DMA Peripheral data size
  * @brief    DMA peripheral data size 
  * @{
  */
/* !< Peripheral data alignment: Byte     */
/* !< Peripheral data alignment: HalfWord */
/* !< Peripheral data alignment: Word     */
/* *
  * @}
  */
/* * @defgroup DMA_Memory_data_size DMA Memory data size
  * @brief    DMA memory data size 
  * @{ 
  */
/* !< Memory data alignment: Byte     */
/* !< Memory data alignment: HalfWord */
/* !< Memory data alignment: Word     */
/* *
  * @}
  */
/* * @defgroup DMA_mode DMA mode
  * @brief    DMA mode 
  * @{
  */
/* !< Normal mode                  */
/* !< Circular mode                */
/* !< Peripheral flow control mode */
/* *
  * @}
  */
/* * @defgroup DMA_Priority_level DMA Priority level
  * @brief    DMA priority levels 
  * @{
  */
/* !< Priority level: Low       */
/* !< Priority level: Medium    */
/* !< Priority level: High      */
/* !< Priority level: Very High */
/* *
  * @}
  */
/* * @defgroup DMA_FIFO_direct_mode DMA FIFO direct mode
  * @brief    DMA FIFO direct mode
  * @{
  */
/* !< FIFO mode disable */
/* !< FIFO mode enable  */
/* *
  * @}
  */
/* * @defgroup DMA_FIFO_threshold_level DMA FIFO threshold level
  * @brief    DMA FIFO level 
  * @{
  */
/* !< FIFO threshold 1 quart full configuration  */
/* !< FIFO threshold half full configuration     */
/* !< FIFO threshold 3 quarts full configuration */
/* !< FIFO threshold full configuration          */
/* *
  * @}
  */
/* * @defgroup DMA_Memory_burst DMA Memory burst
  * @brief    DMA memory burst 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Peripheral_burst DMA Peripheral burst
  * @brief    DMA peripheral burst 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_interrupt_enable_definitions DMA interrupt enable definitions
  * @brief    DMA interrupts definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_flag_definitions DMA flag definitions
  * @brief    DMA flag definitions 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @brief Reset DMA handle state
  * @param  __HANDLE__: specifies the DMA handle.
  * @retval None
  */
/* *
  * @brief  Return the current DMA Stream FIFO filled level.
  * @param  __HANDLE__: DMA handle
  * @retval The FIFO filling state.
  *           - DMA_FIFOStatus_Less1QuarterFull: when FIFO is less than 1 quarter-full 
  *                                              and not empty.
  *           - DMA_FIFOStatus_1QuarterFull: if more than 1 quarter-full.
  *           - DMA_FIFOStatus_HalfFull: if more than 1 half-full.
  *           - DMA_FIFOStatus_3QuartersFull: if more than 3 quarters-full.
  *           - DMA_FIFOStatus_Empty: when FIFO is empty
  *           - DMA_FIFOStatus_Full: when FIFO is full
  */
/* *
  * @brief  Enable the specified DMA Stream.
  * @param  __HANDLE__: DMA handle
  * @retval None
  */
/* *
  * @brief  Disable the specified DMA Stream.
  * @param  __HANDLE__: DMA handle
  * @retval None
  */
/* Interrupt & Flag management */
/* *
  * @brief  Return the current DMA Stream transfer complete flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer complete flag index.
  */
/* *
  * @brief  Return the current DMA Stream half transfer complete flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified half transfer complete flag index.
  */
/* *
  * @brief  Return the current DMA Stream transfer error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified transfer error flag index.
  */
/* *
  * @brief  Return the current DMA Stream FIFO error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified FIFO error flag index.
  */
/* *
  * @brief  Return the current DMA Stream direct mode error flag.
  * @param  __HANDLE__: DMA handle
  * @retval The specified direct mode error flag index.
  */
/* *
  * @brief  Get the DMA Stream pending flags.
  * @param  __HANDLE__: DMA handle
  * @param  __FLAG__: Get the specified flag.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCIFx: Transfer complete flag.
  *            @arg DMA_FLAG_HTIFx: Half transfer complete flag.
  *            @arg DMA_FLAG_TEIFx: Transfer error flag.
  *            @arg DMA_FLAG_DMEIFx: Direct mode error flag.
  *            @arg DMA_FLAG_FEIFx: FIFO error flag.
  *         Where x can be 0_4, 1_5, 2_6 or 3_7 to select the DMA Stream flag.   
  * @retval The state of FLAG (SET or RESET).
  */
/* *
  * @brief  Clear the DMA Stream pending flags.
  * @param  __HANDLE__: DMA handle
  * @param  __FLAG__: specifies the flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DMA_FLAG_TCIFx: Transfer complete flag.
  *            @arg DMA_FLAG_HTIFx: Half transfer complete flag.
  *            @arg DMA_FLAG_TEIFx: Transfer error flag.
  *            @arg DMA_FLAG_DMEIFx: Direct mode error flag.
  *            @arg DMA_FLAG_FEIFx: FIFO error flag.
  *         Where x can be 0_4, 1_5, 2_6 or 3_7 to select the DMA Stream flag.   
  * @retval None
  */
/* *
  * @brief  Enable the specified DMA Stream interrupts.
  * @param  __HANDLE__: DMA handle
  * @param  __INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled. 
  *        This parameter can be one of the following values:
  *           @arg DMA_IT_TC: Transfer complete interrupt mask.
  *           @arg DMA_IT_HT: Half transfer complete interrupt mask.
  *           @arg DMA_IT_TE: Transfer error interrupt mask.
  *           @arg DMA_IT_FE: FIFO error interrupt mask.
  *           @arg DMA_IT_DME: Direct mode error interrupt.
  * @retval None
  */
/* *
  * @brief  Disable the specified DMA Stream interrupts.
  * @param  __HANDLE__: DMA handle
  * @param  __INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled. 
  *         This parameter can be one of the following values:
  *            @arg DMA_IT_TC: Transfer complete interrupt mask.
  *            @arg DMA_IT_HT: Half transfer complete interrupt mask.
  *            @arg DMA_IT_TE: Transfer error interrupt mask.
  *            @arg DMA_IT_FE: FIFO error interrupt mask.
  *            @arg DMA_IT_DME: Direct mode error interrupt.
  * @retval None
  */
/* *
  * @brief  Check whether the specified DMA Stream interrupt is enabled or not.
  * @param  __HANDLE__: DMA handle
  * @param  __INTERRUPT__: specifies the DMA interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg DMA_IT_TC: Transfer complete interrupt mask.
  *            @arg DMA_IT_HT: Half transfer complete interrupt mask.
  *            @arg DMA_IT_TE: Transfer error interrupt mask.
  *            @arg DMA_IT_FE: FIFO error interrupt mask.
  *            @arg DMA_IT_DME: Direct mode error interrupt.
  * @retval The state of DMA_IT.
  */
/* *
  * @brief  Writes the number of data units to be transferred on the DMA Stream.
  * @param  __HANDLE__: DMA handle
  * @param  __COUNTER__: Number of data units to be transferred (from 0 to 65535) 
  *          Number of data items depends only on the Peripheral data format.
  *            
  * @note   If Peripheral data format is Bytes: number of data units is equal 
  *         to total number of bytes to be transferred.
  *           
  * @note   If Peripheral data format is Half-Word: number of data units is  
  *         equal to total number of bytes to be transferred / 2.
  *           
  * @note   If Peripheral data format is Word: number of data units is equal 
  *         to total  number of bytes to be transferred / 4.
  *      
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
/* *
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  __HANDLE__: DMA handle
  *   
  * @retval The number of remaining data units in the current DMA Stream transfer.
  */
/* Include DMA HAL Extension module */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup DMA_Exported_Functions DMA Exported Functions
  * @brief    DMA Exported functions 
  * @{
  */
/* * @defgroup DMA_Exported_Functions_Group1 Initialization and de-initialization functions
  * @brief   Initialization and de-initialization functions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Exported_Functions_Group2 I/O operation functions
  * @brief   I/O operation functions  
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DMA_Exported_Functions_Group3 Peripheral State functions
  * @brief    Peripheral State functions 
  * @{
  */
/* *
  * @brief  Return the DMA error code
  * @param  hdma : pointer to a DMA_HandleTypeDef structure that contains
  *              the configuration information for the specified DMA Stream.
  * @retval DMA Error Code
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMA_GetError(mut hdma: *mut DMA_HandleTypeDef)
 -> uint32_t {
    return (*hdma).ErrorCode;
}
/* 5 ms */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @addtogroup DMA_Private_Functions
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup DMA_Private_Functions
  * @{
  */
/* *
  * @brief  Sets the DMA Transfer parameter.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
unsafe extern "C" fn DMA_SetConfig(mut hdma: *mut DMA_HandleTypeDef,
                                   mut SrcAddress: uint32_t,
                                   mut DstAddress: uint32_t,
                                   mut DataLength: uint32_t) {
    /* Clear DBM bit */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           18 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Configure DMA Stream data length */
    ::core::ptr::write_volatile(&mut (*(*hdma).Instance).NDTR as
                                    *mut uint32_t, DataLength);
    /* Peripheral to Memory */
    if (*hdma).Init.Direction == (0x1 as libc::c_uint) << 6 as libc::c_uint {
        /* Configure DMA Stream destination address */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).PAR as
                                        *mut uint32_t, DstAddress);
        /* Configure DMA Stream source address */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M0AR as
                                        *mut uint32_t, SrcAddress)
    } else {
        /* Memory to Peripheral */
        /* Configure DMA Stream source address */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).PAR as
                                        *mut uint32_t, SrcAddress);
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M0AR as
                                        *mut uint32_t, DstAddress)
    };
}
/* Configure DMA Stream destination address */
/* *
  * @brief  Returns the DMA Stream base address depending on stream number
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream. 
  * @retval Stream base address
  */
unsafe extern "C" fn DMA_CalcBaseAndBitshift(mut hdma: *mut DMA_HandleTypeDef)
 -> uint32_t {
    let mut stream_number: uint32_t =
        ((*hdma).Instance as uint32_t &
             0xff as
                 libc::c_uint).wrapping_sub(16 as
                                                libc::c_uint).wrapping_div(24
                                                                               as
                                                                               libc::c_uint);
    /* lookup table for necessary bitshift of flags within status registers */
    static mut flagBitshiftOffset: [uint8_t; 8] =
        [0 as libc::c_uint as uint8_t, 6 as libc::c_uint as uint8_t,
         16 as libc::c_uint as uint8_t, 22 as libc::c_uint as uint8_t,
         0 as libc::c_uint as uint8_t, 6 as libc::c_uint as uint8_t,
         16 as libc::c_uint as uint8_t, 22 as libc::c_uint as uint8_t];
    (*hdma).StreamIndex =
        flagBitshiftOffset[stream_number as usize] as uint32_t;
    if stream_number > 3 as libc::c_uint {
        /* return pointer to HISR and HIFCR */
        (*hdma).StreamBaseAddress =
            ((*hdma).Instance as uint32_t &
                 !(0x3ff as libc::c_uint)).wrapping_add(4 as libc::c_uint)
    } else {
        /* return pointer to LISR and LIFCR */
        (*hdma).StreamBaseAddress =
            (*hdma).Instance as uint32_t & !(0x3ff as libc::c_uint)
    }
    return (*hdma).StreamBaseAddress;
}
/* *
  * @brief  Check compatibility between FIFO threshold level and size of the memory burst
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream. 
  * @retval HAL status
  */
unsafe extern "C" fn DMA_CheckFifoParam(mut hdma: *mut DMA_HandleTypeDef)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    let mut tmp: uint32_t = (*hdma).Init.FIFOThreshold;
    /* Memory Data size equal to Byte */
    if (*hdma).Init.MemDataAlignment == 0 as libc::c_uint {
        match tmp {
            0 | 2 => {
                if (*hdma).Init.MemBurst &
                       (0x2 as libc::c_uint) << 23 as libc::c_uint ==
                       (0x2 as libc::c_uint) << 23 as libc::c_uint {
                    status = HAL_ERROR
                }
            }
            1 => {
                if (*hdma).Init.MemBurst ==
                       (0x3 as libc::c_uint) << 23 as libc::c_uint {
                    status = HAL_ERROR
                }
            }
            3 | _ => { }
        }
    } else if (*hdma).Init.MemDataAlignment ==
                  (0x1 as libc::c_uint) << 13 as libc::c_uint {
        match tmp {
            0 | 2 => { status = HAL_ERROR }
            1 => {
                if (*hdma).Init.MemBurst &
                       (0x2 as libc::c_uint) << 23 as libc::c_uint ==
                       (0x2 as libc::c_uint) << 23 as libc::c_uint {
                    status = HAL_ERROR
                }
            }
            3 => {
                if (*hdma).Init.MemBurst ==
                       (0x3 as libc::c_uint) << 23 as libc::c_uint {
                    status = HAL_ERROR
                }
            }
            _ => { }
        }
    } else {
        /* Memory Data size equal to Half-Word */
        /* Memory Data size equal to Word */
        match tmp {
            0 | 1 | 2 => { status = HAL_ERROR }
            3 => {
                if (*hdma).Init.MemBurst &
                       (0x2 as libc::c_uint) << 23 as libc::c_uint ==
                       (0x2 as libc::c_uint) << 23 as libc::c_uint {
                    status = HAL_ERROR
                }
            }
            _ => { }
        }
    }
    return status;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_DMA_MODULE_ENABLED */
/* *
  * @}
  */
