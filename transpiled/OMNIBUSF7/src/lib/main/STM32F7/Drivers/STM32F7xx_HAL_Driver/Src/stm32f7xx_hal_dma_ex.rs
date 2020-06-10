use ::libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
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
  * @file    stm32f7xx_hal_dma_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of DMA HAL extension module.
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
/* * @addtogroup DMAEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup DMAEx_Exported_Types DMAEx Exported Types
  * @brief DMAEx Exported types
  * @{
  */
/* * 
  * @brief  HAL DMA Memory definition  
  */
pub type HAL_DMA_MemoryTypeDef = libc::c_uint;
/* !< Memory 1     */
/* !< Memory 0     */
pub const MEMORY1: HAL_DMA_MemoryTypeDef = 1;
pub const MEMORY0: HAL_DMA_MemoryTypeDef = 0;
/* *
  * @}
  */
/* Exported functions ---------------------------------------------------------*/
/* * @addtogroup DMAEx_Exported_Functions
  * @{
  */
/* * @addtogroup DMAEx_Exported_Functions_Group1
  *
@verbatim
 ===============================================================================
                #####  Extended features functions  #####
 ===============================================================================  
    [..]  This section provides functions allowing to:
      (+) Configure the source, destination address and data length and 
          Start MultiBuffer DMA transfer
      (+) Configure the source, destination address and data length and 
          Start MultiBuffer DMA transfer with interrupt
      (+) Change on the fly the memory0 or memory1 address.
      
@endverbatim
  * @{
  */
/* *
  * @brief  Starts the multi_buffer DMA Transfer.
  * @param  hdma      : pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  SecondMemAddress: The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMAEx_MultiBufferStart(mut hdma:
                                                        *mut DMA_HandleTypeDef,
                                                    mut SrcAddress: uint32_t,
                                                    mut DstAddress: uint32_t,
                                                    mut SecondMemAddress:
                                                        uint32_t,
                                                    mut DataLength: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Memory-to-memory transfer not supported in double buffering mode */
    if (*hdma).Init.Direction == (0x2 as libc::c_uint) << 6 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0x100 as libc::c_uint);
        status = HAL_ERROR
    } else {
        /* Process Locked */
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
            /* Enable the double buffer mode */
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 18 as libc::c_uint) as
                                            uint32_t as uint32_t);
            /* Configure DMA Stream destination address */
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M1AR as
                                            *mut uint32_t, SecondMemAddress);
            /* Configure the source, destination address and the data length */
            DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress,
                                     DataLength);
            /* Enable the peripheral */
            ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x1 as libc::c_uint) <<
                                                 0 as libc::c_uint) as
                                            uint32_t as uint32_t)
        } else {
            /* Return error status */
            status = HAL_BUSY
        }
    }
    return status;
}
/* *
  * @brief  Starts the multi_buffer DMA Transfer with interrupt enabled.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  SecondMemAddress: The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMAEx_MultiBufferStart_IT(mut hdma:
                                                           *mut DMA_HandleTypeDef,
                                                       mut SrcAddress:
                                                           uint32_t,
                                                       mut DstAddress:
                                                           uint32_t,
                                                       mut SecondMemAddress:
                                                           uint32_t,
                                                       mut DataLength:
                                                           uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Memory-to-memory transfer not supported in double buffering mode */
    if (*hdma).Init.Direction == (0x2 as libc::c_uint) << 6 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*hdma).ErrorCode as *mut uint32_t,
                                    0x100 as libc::c_uint);
        return HAL_ERROR
    }
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
        /* Enable the Double buffer mode */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).CR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*(*hdma).Instance).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             18 as libc::c_uint) as uint32_t
                                        as uint32_t);
        /* Configure DMA Stream destination address */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M1AR as
                                        *mut uint32_t, SecondMemAddress);
        /* Configure the source, destination address and the data length */
        DMA_MultiBufferSetConfig(hdma, SrcAddress, DstAddress, DataLength);
        /* Clear all flags */
        if (*hdma).Instance as uint32_t >
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6400
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x58
                                                                                                                 as
                                                                                                                 libc::c_uint)
                   as *mut DMA_Stream_TypeDef as uint32_t {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        if (*hdma).Instance as uint32_t ==
                                               (0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x6000
                                                                                                                  as
                                                                                                                  libc::c_uint).wrapping_add(0x10
                                                                                                                                                 as
                                                                                                                                                 libc::c_uint)
                                                   as *mut DMA_Stream_TypeDef
                                                   as uint32_t {
                                            0x20 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x10
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x20 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x20 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x20 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200000 as libc::c_uint
                                        } else { 0x8000000 as libc::c_uint })
        } else {
            if (*hdma).Instance as uint32_t >
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6000
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xb8
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as *mut DMA_Stream_TypeDef as uint32_t {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            if (*hdma).Instance as uint32_t ==
                                                   (0x40000000 as
                                                        libc::c_uint).wrapping_add(0x20000
                                                                                       as
                                                                                       libc::c_uint).wrapping_add(0x6000
                                                                                                                      as
                                                                                                                      libc::c_uint).wrapping_add(0x10
                                                                                                                                                     as
                                                                                                                                                     libc::c_uint)
                                                       as
                                                       *mut DMA_Stream_TypeDef
                                                       as uint32_t {
                                                0x20 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x10
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x20 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x20 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x20 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200000 as libc::c_uint
                                            } else {
                                                0x8000000 as libc::c_uint
                                            })
            } else {
                if (*hdma).Instance as uint32_t >
                       (0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x6000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x58
                                                                                                                         as
                                                                                                                         libc::c_uint)
                           as *mut DMA_Stream_TypeDef as uint32_t {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).HIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else {
                                                    0x8000000 as libc::c_uint
                                                })
                } else {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).LIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x20 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200000 as libc::c_uint
                                                } else {
                                                    0x8000000 as libc::c_uint
                                                })
                };
            };
        };
        if (*hdma).Instance as uint32_t >
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6400
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x58
                                                                                                                 as
                                                                                                                 libc::c_uint)
                   as *mut DMA_Stream_TypeDef as uint32_t {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        if (*hdma).Instance as uint32_t ==
                                               (0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x6000
                                                                                                                  as
                                                                                                                  libc::c_uint).wrapping_add(0x10
                                                                                                                                                 as
                                                                                                                                                 libc::c_uint)
                                                   as *mut DMA_Stream_TypeDef
                                                   as uint32_t {
                                            0x10 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x10
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x400 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x400 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x400 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x400 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100000 as libc::c_uint
                                        } else { 0x4000000 as libc::c_uint })
        } else {
            if (*hdma).Instance as uint32_t >
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6000
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xb8
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as *mut DMA_Stream_TypeDef as uint32_t {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            if (*hdma).Instance as uint32_t ==
                                                   (0x40000000 as
                                                        libc::c_uint).wrapping_add(0x20000
                                                                                       as
                                                                                       libc::c_uint).wrapping_add(0x6000
                                                                                                                      as
                                                                                                                      libc::c_uint).wrapping_add(0x10
                                                                                                                                                     as
                                                                                                                                                     libc::c_uint)
                                                       as
                                                       *mut DMA_Stream_TypeDef
                                                       as uint32_t {
                                                0x10 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x10
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x400 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x400 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x400 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x400 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100000 as libc::c_uint
                                            } else {
                                                0x4000000 as libc::c_uint
                                            })
            } else {
                if (*hdma).Instance as uint32_t >
                       (0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x6000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x58
                                                                                                                         as
                                                                                                                         libc::c_uint)
                           as *mut DMA_Stream_TypeDef as uint32_t {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).HIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else {
                                                    0x4000000 as libc::c_uint
                                                })
                } else {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).LIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x400 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100000 as libc::c_uint
                                                } else {
                                                    0x4000000 as libc::c_uint
                                                })
                };
            };
        };
        if (*hdma).Instance as uint32_t >
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6400
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x58
                                                                                                                 as
                                                                                                                 libc::c_uint)
                   as *mut DMA_Stream_TypeDef as uint32_t {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        if (*hdma).Instance as uint32_t ==
                                               (0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x6000
                                                                                                                  as
                                                                                                                  libc::c_uint).wrapping_add(0x10
                                                                                                                                                 as
                                                                                                                                                 libc::c_uint)
                                                   as *mut DMA_Stream_TypeDef
                                                   as uint32_t {
                                            0x8 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x10
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x8 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x8 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x8 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x200 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x80000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x80000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x80000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x80000 as libc::c_uint
                                        } else { 0x2000000 as libc::c_uint })
        } else {
            if (*hdma).Instance as uint32_t >
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6000
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xb8
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as *mut DMA_Stream_TypeDef as uint32_t {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            if (*hdma).Instance as uint32_t ==
                                                   (0x40000000 as
                                                        libc::c_uint).wrapping_add(0x20000
                                                                                       as
                                                                                       libc::c_uint).wrapping_add(0x6000
                                                                                                                      as
                                                                                                                      libc::c_uint).wrapping_add(0x10
                                                                                                                                                     as
                                                                                                                                                     libc::c_uint)
                                                       as
                                                       *mut DMA_Stream_TypeDef
                                                       as uint32_t {
                                                0x8 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x10
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x8 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x8 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x8 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x200 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x80000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x80000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x80000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x80000 as libc::c_uint
                                            } else {
                                                0x2000000 as libc::c_uint
                                            })
            } else {
                if (*hdma).Instance as uint32_t >
                       (0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x6000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x58
                                                                                                                         as
                                                                                                                         libc::c_uint)
                           as *mut DMA_Stream_TypeDef as uint32_t {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).HIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else {
                                                    0x2000000 as libc::c_uint
                                                })
                } else {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).LIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x8 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x200 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x80000 as libc::c_uint
                                                } else {
                                                    0x2000000 as libc::c_uint
                                                })
                };
            };
        };
        if (*hdma).Instance as uint32_t >
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6400
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x58
                                                                                                                 as
                                                                                                                 libc::c_uint)
                   as *mut DMA_Stream_TypeDef as uint32_t {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        if (*hdma).Instance as uint32_t ==
                                               (0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x6000
                                                                                                                  as
                                                                                                                  libc::c_uint).wrapping_add(0x10
                                                                                                                                                 as
                                                                                                                                                 libc::c_uint)
                                                   as *mut DMA_Stream_TypeDef
                                                   as uint32_t {
                                            0x800004 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x10
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800004 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800004 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800004 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x100 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40000 as libc::c_uint
                                        } else { 0x1000000 as libc::c_uint })
        } else {
            if (*hdma).Instance as uint32_t >
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6000
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xb8
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as *mut DMA_Stream_TypeDef as uint32_t {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            if (*hdma).Instance as uint32_t ==
                                                   (0x40000000 as
                                                        libc::c_uint).wrapping_add(0x20000
                                                                                       as
                                                                                       libc::c_uint).wrapping_add(0x6000
                                                                                                                      as
                                                                                                                      libc::c_uint).wrapping_add(0x10
                                                                                                                                                     as
                                                                                                                                                     libc::c_uint)
                                                       as
                                                       *mut DMA_Stream_TypeDef
                                                       as uint32_t {
                                                0x800004 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x10
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800004 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800004 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800004 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x100 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40000 as libc::c_uint
                                            } else {
                                                0x1000000 as libc::c_uint
                                            })
            } else {
                if (*hdma).Instance as uint32_t >
                       (0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x6000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x58
                                                                                                                         as
                                                                                                                         libc::c_uint)
                           as *mut DMA_Stream_TypeDef as uint32_t {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).HIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else {
                                                    0x1000000 as libc::c_uint
                                                })
                } else {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).LIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800004 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x100 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40000 as libc::c_uint
                                                } else {
                                                    0x1000000 as libc::c_uint
                                                })
                };
            };
        };
        if (*hdma).Instance as uint32_t >
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x6400
                                                                                  as
                                                                                  libc::c_uint).wrapping_add(0x58
                                                                                                                 as
                                                                                                                 libc::c_uint)
                   as *mut DMA_Stream_TypeDef as uint32_t {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x6400
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut DMA_TypeDef)).HIFCR
                                            as *mut uint32_t,
                                        if (*hdma).Instance as uint32_t ==
                                               (0x40000000 as
                                                    libc::c_uint).wrapping_add(0x20000
                                                                                   as
                                                                                   libc::c_uint).wrapping_add(0x6000
                                                                                                                  as
                                                                                                                  libc::c_uint).wrapping_add(0x10
                                                                                                                                                 as
                                                                                                                                                 libc::c_uint)
                                                   as *mut DMA_Stream_TypeDef
                                                   as uint32_t {
                                            0x800001 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x10
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800001 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800001 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x70
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x800001 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x28
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x88
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x40 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0x40
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6000
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10000 as libc::c_uint
                                        } else if (*hdma).Instance as uint32_t
                                                      ==
                                                      (0x40000000 as
                                                           libc::c_uint).wrapping_add(0x20000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x6400
                                                                                                                         as
                                                                                                                         libc::c_uint).wrapping_add(0xa0
                                                                                                                                                        as
                                                                                                                                                        libc::c_uint)
                                                          as
                                                          *mut DMA_Stream_TypeDef
                                                          as uint32_t {
                                            0x10000 as libc::c_uint
                                        } else { 0x400000 as libc::c_uint })
        } else {
            if (*hdma).Instance as uint32_t >
                   (0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x6000
                                                                                      as
                                                                                      libc::c_uint).wrapping_add(0xb8
                                                                                                                     as
                                                                                                                     libc::c_uint)
                       as *mut DMA_Stream_TypeDef as uint32_t {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                         libc::c_uint).wrapping_add(0x20000
                                                                                        as
                                                                                        libc::c_uint).wrapping_add(0x6400
                                                                                                                       as
                                                                                                                       libc::c_uint)
                                                        as
                                                        *mut DMA_TypeDef)).LIFCR
                                                as *mut uint32_t,
                                            if (*hdma).Instance as uint32_t ==
                                                   (0x40000000 as
                                                        libc::c_uint).wrapping_add(0x20000
                                                                                       as
                                                                                       libc::c_uint).wrapping_add(0x6000
                                                                                                                      as
                                                                                                                      libc::c_uint).wrapping_add(0x10
                                                                                                                                                     as
                                                                                                                                                     libc::c_uint)
                                                       as
                                                       *mut DMA_Stream_TypeDef
                                                       as uint32_t {
                                                0x800001 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x10
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800001 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800001 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x70
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x800001 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x28
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x88
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x40 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0x40
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6000
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10000 as libc::c_uint
                                            } else if (*hdma).Instance as
                                                          uint32_t ==
                                                          (0x40000000 as
                                                               libc::c_uint).wrapping_add(0x20000
                                                                                              as
                                                                                              libc::c_uint).wrapping_add(0x6400
                                                                                                                             as
                                                                                                                             libc::c_uint).wrapping_add(0xa0
                                                                                                                                                            as
                                                                                                                                                            libc::c_uint)
                                                              as
                                                              *mut DMA_Stream_TypeDef
                                                              as uint32_t {
                                                0x10000 as libc::c_uint
                                            } else {
                                                0x400000 as libc::c_uint
                                            })
            } else {
                if (*hdma).Instance as uint32_t >
                       (0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x6000
                                                                                          as
                                                                                          libc::c_uint).wrapping_add(0x58
                                                                                                                         as
                                                                                                                         libc::c_uint)
                           as *mut DMA_Stream_TypeDef as uint32_t {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).HIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else {
                                                    0x400000 as libc::c_uint
                                                })
                } else {
                    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                             libc::c_uint).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x6000
                                                                                                                           as
                                                                                                                           libc::c_uint)
                                                            as
                                                            *mut DMA_TypeDef)).LIFCR
                                                    as *mut uint32_t,
                                                if (*hdma).Instance as
                                                       uint32_t ==
                                                       (0x40000000 as
                                                            libc::c_uint).wrapping_add(0x20000
                                                                                           as
                                                                                           libc::c_uint).wrapping_add(0x6000
                                                                                                                          as
                                                                                                                          libc::c_uint).wrapping_add(0x10
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                                                           as
                                                           *mut DMA_Stream_TypeDef
                                                           as uint32_t {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x10
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x70
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x800001 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x28
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x88
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x40 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0x40
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6000
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else if (*hdma).Instance as
                                                              uint32_t ==
                                                              (0x40000000 as
                                                                   libc::c_uint).wrapping_add(0x20000
                                                                                                  as
                                                                                                  libc::c_uint).wrapping_add(0x6400
                                                                                                                                 as
                                                                                                                                 libc::c_uint).wrapping_add(0xa0
                                                                                                                                                                as
                                                                                                                                                                libc::c_uint)
                                                                  as
                                                                  *mut DMA_Stream_TypeDef
                                                                  as uint32_t
                                                 {
                                                    0x10000 as libc::c_uint
                                                } else {
                                                    0x400000 as libc::c_uint
                                                })
                };
            };
        };
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
        if (*hdma).XferHalfCpltCallback.is_some() ||
               (*hdma).XferM1HalfCpltCallback.is_some() {
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
        /* Enable the peripheral */
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
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DMA_Exported_Constants DMA Exported Constants
  * @brief    DMA Exported constants 
  * @{
  */
/* * @defgroup DMAEx_Channel_selection DMA Channel selection
  * @brief    DMAEx channel selection 
  * @{
  */
/* !< DMA Channel 0 */
/* !< DMA Channel 1 */
/* !< DMA Channel 2 */
/* !< DMA Channel 3 */
/* !< DMA Channel 4 */
/* !< DMA Channel 5 */
/* !< DMA Channel 6 */
/* !< DMA Channel 7 */
/* STM32F722xx || STM32F723xx || STM32F732xx || STM32F733xx || STM32F765xx || STM32F767xx ||
          STM32F769xx || STM32F777xx || STM32F779xx */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup DMAEx_Exported_Functions DMAEx Exported Functions
  * @brief   DMAEx Exported functions
  * @{
  */
/* * @defgroup DMAEx_Exported_Functions_Group1 Extended features functions
  * @brief   Extended features functions
  * @{
  */
/* IO operation functions *******************************************************/
/* *
  * @brief  Change the memory0 or memory1 address on the fly.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  Address:    The new address
  * @param  memory:     the memory to be changed, This parameter can be one of 
  *                     the following values:
  *                      MEMORY0 /
  *                      MEMORY1
  * @note   The MEMORY0 address can be changed only when the current transfer use
  *         MEMORY1 and the MEMORY1 address can be changed only when the current 
  *         transfer use MEMORY0.
  * @retval HAL status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_DMAEx_ChangeMemory(mut hdma:
                                                    *mut DMA_HandleTypeDef,
                                                mut Address: uint32_t,
                                                mut memory:
                                                    HAL_DMA_MemoryTypeDef)
 -> HAL_StatusTypeDef {
    if memory as libc::c_uint == MEMORY0 as libc::c_int as libc::c_uint {
        /* change the memory0 address */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M0AR as
                                        *mut uint32_t, Address)
    } else {
        /* change the memory1 address */
        ::core::ptr::write_volatile(&mut (*(*hdma).Instance).M1AR as
                                        *mut uint32_t, Address)
    }
    return HAL_OK;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_dma_ex.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   DMA Extension HAL module driver
  *         This file provides firmware functions to manage the following 
  *         functionalities of the DMA Extension peripheral:
  *           + Extended features functions
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
  [..]
  The DMA Extension HAL driver can be used as follows:
   (+) Start a multi buffer transfer using the HAL_DMA_MultiBufferStart() function
       for polling mode or HAL_DMA_MultiBufferStart_IT() for interrupt mode.

     -@-  In Memory-to-Memory transfer mode, Multi (Double) Buffer mode is not allowed.
     -@-  When Multi (Double) Buffer mode is enabled, the transfer is circular by default.
     -@-  In Multi (Double) buffer mode, it is possible to update the base address for 
          the AHB memory port on the fly (DMA_SxM0AR or DMA_SxM1AR) when the stream is enabled.
  
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
/* * @defgroup DMAEx DMAEx
  * @brief DMA Extended HAL module driver
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private Constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @addtogroup DMAEx_Private_Functions
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup DMAEx_Private_Functions
  * @{
  */
/* *
  * @brief  Set the DMA Transfer parameter.
  * @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
  *                     the configuration information for the specified DMA Stream.  
  * @param  SrcAddress: The source memory Buffer address
  * @param  DstAddress: The destination memory Buffer address
  * @param  DataLength: The length of data to be transferred from source to destination
  * @retval HAL status
  */
unsafe extern "C" fn DMA_MultiBufferSetConfig(mut hdma:
                                                  *mut DMA_HandleTypeDef,
                                              mut SrcAddress: uint32_t,
                                              mut DstAddress: uint32_t,
                                              mut DataLength: uint32_t) {
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
