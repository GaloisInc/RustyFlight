use ::libc;
extern "C" {
    /* *
  * @}
  */
    /* Exported constants --------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Constants FLASH Exported Constants
  * @{
  */
    /* * @defgroup FLASH_Error_Code FLASH Error Code
  * @brief    FLASH Error Code 
  * @{
  */
    /* !< No error                      */
    /* !< Programming Sequence error    */
    /* !< Programming Parallelism error */
    /* !< Programming Alignment error   */
    /* !< Write protection error        */
    /* !< Operation Error               */
    /* !< Read Protection Error         */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Type_Program FLASH Type Program
  * @{
  */
    /* !< Program byte (8-bit) at a specified address           */
    /* !< Program a half-word (16-bit) at a specified address   */
    /* !< Program a word (32-bit) at a specified address        */
    /* !< Program a double word (64-bit) at a specified address */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Flag_definition FLASH Flag definition
  * @brief Flag definition
  * @{
  */
    /* !< FLASH End of Operation flag               */
    /* !< FLASH operation Error flag                */
    /* !< FLASH Write protected error flag          */
    /* !< FLASH Programming Alignment error flag    */
    /* !< FLASH Programming Parallelism error flag  */
    /* !< FLASH Erasing Sequence error flag         */
    /* !< FLASH Busy flag                           */
    /* !< FLASH Read protection error flag          */
    /* FLASH_OPTCR2_PCROP */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Interrupt_definition FLASH Interrupt definition
  * @brief FLASH Interrupt definition
  * @{
  */
    /* !< End of FLASH Operation Interrupt source */
    /* !< Error Interrupt source                  */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Program_Parallelism FLASH Program Parallelism
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Keys FLASH Keys
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup FLASH_Sectors FLASH Sectors
  * @{
  */
    /* !< Sector Number 0   */
    /* !< Sector Number 1   */
    /* !< Sector Number 2   */
    /* !< Sector Number 3   */
    /* !< Sector Number 4   */
    /* !< Sector Number 5   */
    /* !< Sector Number 6   */
    /* !< Sector Number 7   */
    /* *
  * @}
  */
    /* *
  * @}
  */
    /* Exported macro ------------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Macros FLASH Exported Macros
  * @{
  */
/* *
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__: FLASH Latency                   
  *         The value of this parameter depend on device used within the same series
  * @retval none
  */
    /* *
  * @brief  Get the FLASH Latency.
  * @retval FLASH Latency                   
  *          The value of this parameter depend on device used within the same series
  */
    /* *
  * @brief  Enable the FLASH prefetch buffer.
  * @retval none
  */
    /* *
  * @brief  Disable the FLASH prefetch buffer.
  * @retval none
  */
    /* *
  * @brief  Enable the FLASH Adaptive Real-Time memory accelerator.
  * @note   The ART accelerator is available only for flash access on ITCM interface.
  * @retval none
  */
    /* *
  * @brief  Disable the FLASH Adaptive Real-Time memory accelerator.
  * @retval none
  */
    /* *
  * @brief  Resets the FLASH Adaptive Real-Time memory accelerator.
  * @note   This function must be used only when the Adaptive Real-Time memory accelerator
  *         is disabled.  
  * @retval None
  */
    /* *
  * @brief  Enable the specified FLASH interrupt.
  * @param  __INTERRUPT__ : FLASH interrupt 
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt    
  * @retval none
  */
    /* *
  * @brief  Disable the specified FLASH interrupt.
  * @param  __INTERRUPT__ : FLASH interrupt 
  *         This parameter can be any combination of the following values:
  *     @arg FLASH_IT_EOP: End of FLASH Operation Interrupt
  *     @arg FLASH_IT_ERR: Error Interrupt    
  * @retval none
  */
    /* *
  * @brief  Get the specified FLASH flag status. 
  * @param  __FLAG__: specifies the FLASH flag to check.
  *          This parameter can be one of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag 
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_ERSERR : FLASH Erasing Sequence error flag 
  *            @arg FLASH_FLAG_BSY   : FLASH Busy flag
  * @retval The new state of __FLAG__ (SET or RESET).
  */
    /* *
  * @brief  Clear the specified FLASH flag.
  * @param  __FLAG__: specifies the FLASH flags to clear.
  *          This parameter can be any combination of the following values:
  *            @arg FLASH_FLAG_EOP   : FLASH End of Operation flag 
  *            @arg FLASH_FLAG_OPERR : FLASH operation Error flag 
  *            @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag 
  *            @arg FLASH_FLAG_PGAERR: FLASH Programming Alignment error flag 
  *            @arg FLASH_FLAG_PGPERR: FLASH Programming Parallelism error flag
  *            @arg FLASH_FLAG_ERSERR : FLASH Erasing Sequence error flag    
  * @retval none
  */
    /* *
  * @}
  */
    /* Include FLASH HAL Extension module */
    /* Exported functions --------------------------------------------------------*/
/* * @addtogroup FLASH_Exported_Functions
  * @{
  */
/* * @addtogroup FLASH_Exported_Functions_Group1
  * @{
  */
/* Program operation functions  ***********************************************/
    /* FLASH IRQ handler method */
    /* Callbacks in non blocking modes */
    /* *
  * @}
  */
    /* * @addtogroup FLASH_Exported_Functions_Group2
  * @{
  */
/* Peripheral Control functions  **********************************************/
    /* Option bytes control */
    /* *
  * @}
  */
    /* * @addtogroup FLASH_Exported_Functions_Group3
  * @{
  */
/* Peripheral State functions  ************************************************/
    /* FLASH_OPTCR2_PCROP */
    #[no_mangle]
    fn FLASH_WaitForLastOperation(Timeout: uint32_t) -> HAL_StatusTypeDef;
    /* 50 s */
    /* *
  * @}
  */
    /* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @addtogroup FLASHEx_Private_Variables
  * @{
  */
    #[no_mangle]
    static mut pFlash: FLASH_ProcessTypeDef;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_TypeDef {
    pub ACR: uint32_t,
    pub KEYR: uint32_t,
    pub OPTKEYR: uint32_t,
    pub SR: uint32_t,
    pub CR: uint32_t,
    pub OPTCR: uint32_t,
    pub OPTCR1: uint32_t,
    pub OPTCR2: uint32_t,
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
  * @file    stm32f7xx_hal_flash.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of FLASH HAL module.
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
/* * @addtogroup FLASH
  * @{
  */
/* Exported types ------------------------------------------------------------*/ 
/* * @defgroup FLASH_Exported_Types FLASH Exported Types
  * @{
  */
/* *
  * @brief  FLASH Procedure structure definition
  */
pub type FLASH_ProcedureTypeDef = libc::c_uint;
pub const FLASH_PROC_PROGRAM: FLASH_ProcedureTypeDef = 3;
pub const FLASH_PROC_MASSERASE: FLASH_ProcedureTypeDef = 2;
pub const FLASH_PROC_SECTERASE: FLASH_ProcedureTypeDef = 1;
pub const FLASH_PROC_NONE: FLASH_ProcedureTypeDef = 0;
/* * 
  * @brief  FLASH handle Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_ProcessTypeDef {
    pub ProcedureOnGoing: FLASH_ProcedureTypeDef,
    pub NbSectorsToErase: uint32_t,
    pub VoltageForErase: uint8_t,
    pub Sector: uint32_t,
    pub Address: uint32_t,
    pub Lock: HAL_LockTypeDef,
    pub ErrorCode: uint32_t,
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_hal_flash_ex.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of FLASH HAL Extension module.
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
/* * @addtogroup FLASHEx
  * @{
  */
/* Exported types ------------------------------------------------------------*/ 
/* * @defgroup FLASHEx_Exported_Types FLASH Exported Types
  * @{
  */
/* *
  * @brief  FLASH Erase structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_EraseInitTypeDef {
    pub TypeErase: uint32_t,
    pub Sector: uint32_t,
    pub NbSectors: uint32_t,
    pub VoltageRange: uint32_t,
}
/* *
  * @brief  FLASH Option Bytes Program structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_OBProgramInitTypeDef {
    pub OptionType: uint32_t,
    pub WRPState: uint32_t,
    pub WRPSector: uint32_t,
    pub RDPLevel: uint32_t,
    pub BORLevel: uint32_t,
    pub USERConfig: uint32_t,
    pub BootAddr0: uint32_t,
    pub BootAddr1: uint32_t,
    pub PCROPSector: uint32_t,
    pub PCROPRdp: uint32_t,
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
#[inline(always)]
unsafe extern "C" fn __DSB() { asm!("dsb 0xF" : : : "memory" : "volatile"); }
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup FLASHEx_Exported_Functions FLASHEx Exported Functions
  * @{
  */
/* * @defgroup FLASHEx_Exported_Functions_Group1 Extended IO operation functions
 *  @brief   Extended IO operation functions 
 *
@verbatim   
 ===============================================================================
                ##### Extended programming operation functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the Extension FLASH 
    programming operations Operations.

@endverbatim
  * @{
  */
/* *
  * @brief  Perform a mass erase or erase the specified FLASH memory sectors 
  * @param[in]  pEraseInit: pointer to an FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  * 
  * @param[out]  SectorError: pointer to variable  that
  *         contains the configuration information on faulty sector in case of error 
  *         (0xFFFFFFFF means that all the sectors have been correctly erased)
  * 
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASHEx_Erase(mut pEraseInit:
                                               *mut FLASH_EraseInitTypeDef,
                                           mut SectorError: *mut uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_ERROR;
    let mut index: uint32_t = 0 as libc::c_int as uint32_t;
    /* Process Locked */
    if pFlash.Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { pFlash.Lock = HAL_LOCKED }
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        /*Initialization of SectorError variable*/
        *SectorError = 0xffffffff as libc::c_uint;
        if (*pEraseInit).TypeErase == 0x1 as libc::c_uint {
            /*Mass erase to be done*/
            FLASH_MassErase((*pEraseInit).VoltageRange as uint8_t);
            /* FLASH_OPTCR_nDBANK */
            /* Wait for last operation to be completed */
            status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
            /* if the erase operation is completed, disable the MER Bit */
            let ref mut fresh0 =
                (*((0x40000000 as
                        libc::c_uint).wrapping_add(0x20000 as
                                                       libc::c_uint).wrapping_add(0x3c00
                                                                                      as
                                                                                      libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh0,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x1 as libc::c_uint) <<
                                                   2 as libc::c_uint)) as
                                            uint32_t as uint32_t)
        } else {
            /* Check the parameters */
            /* Erase by sector by sector to be done*/
            index = (*pEraseInit).Sector;
            while index <
                      (*pEraseInit).NbSectors.wrapping_add((*pEraseInit).Sector)
                  {
                FLASH_Erase_Sector(index,
                                   (*pEraseInit).VoltageRange as uint8_t);
                /* Wait for last operation to be completed */
                status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
                /* If the erase operation is completed, disable the SER Bit and SNB Bits */
                let ref mut fresh1 =
                    (*((0x40000000 as
                            libc::c_uint).wrapping_add(0x20000 as
                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                          as
                                                                                          libc::c_uint)
                           as *mut FLASH_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh1,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_uint) <<
                                                       1 as libc::c_uint |
                                                       (0xf as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint))
                                                as uint32_t as uint32_t);
                if status as libc::c_uint !=
                       HAL_OK as libc::c_int as libc::c_uint {
                    /* In case of error, stop erase procedure and return the faulty sector*/
                    *SectorError = index;
                    break ;
                } else { index = index.wrapping_add(1) }
            }
        }
    }
    /* Process Unlocked */
    pFlash.Lock = HAL_UNLOCKED;
    return status;
}
/* *
  * @brief  Perform a mass erase or erase the specified FLASH memory sectors  with interrupt enabled
  * @param  pEraseInit: pointer to an FLASH_EraseInitTypeDef structure that
  *         contains the configuration information for the erasing.
  * 
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASHEx_Erase_IT(mut pEraseInit:
                                                  *mut FLASH_EraseInitTypeDef)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Process Locked */
    if pFlash.Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { pFlash.Lock = HAL_LOCKED }
    /* Check the parameters */
    /* Enable End of FLASH Operation interrupt */
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         24 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable Error source interrupt */
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x2000000 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Clear pending flags (if any) */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut FLASH_TypeDef)).SR as
                                    *mut uint32_t,
                                (0x1 as libc::c_uint) << 0 as libc::c_uint |
                                    (0x1 as libc::c_uint) << 1 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 4 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 5 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 6 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) <<
                                        7 as libc::c_uint);
    if (*pEraseInit).TypeErase == 0x1 as libc::c_uint {
        /*Mass erase to be done*/
        ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing as
                                        *mut FLASH_ProcedureTypeDef,
                                    FLASH_PROC_MASSERASE);
        FLASH_MassErase((*pEraseInit).VoltageRange as uint8_t);
        /* FLASH_OPTCR_nDBANK */
    } else {
        /* Erase by sector to be done*/
        /* Check the parameters */
        ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing as
                                        *mut FLASH_ProcedureTypeDef,
                                    FLASH_PROC_SECTERASE);
        ::core::ptr::write_volatile(&mut pFlash.NbSectorsToErase as
                                        *mut uint32_t,
                                    (*pEraseInit).NbSectors);
        ::core::ptr::write_volatile(&mut pFlash.Sector as *mut uint32_t,
                                    (*pEraseInit).Sector);
        ::core::ptr::write_volatile(&mut pFlash.VoltageForErase as
                                        *mut uint8_t,
                                    (*pEraseInit).VoltageRange as uint8_t);
        /*Erase 1st sector and wait for IT*/
        FLASH_Erase_Sector((*pEraseInit).Sector,
                           (*pEraseInit).VoltageRange as uint8_t);
    }
    return status;
}
/* *
  * @brief  Program option bytes
  * @param  pOBInit: pointer to an FLASH_OBInitStruct structure that
  *         contains the configuration information for the programming.
  * 
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASHEx_OBProgram(mut pOBInit:
                                                   *mut FLASH_OBProgramInitTypeDef)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_ERROR;
    /* Process Locked */
    if pFlash.Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { pFlash.Lock = HAL_LOCKED }
    /* Check the parameters */
    /* Write protection configuration */
    if (*pOBInit).OptionType & 0x1 as libc::c_uint == 0x1 as libc::c_uint {
        if (*pOBInit).WRPState == 0x1 as libc::c_uint {
            /*Enable of Write protection on the selected Sector*/
            status = FLASH_OB_EnableWRP((*pOBInit).WRPSector)
        } else {
            /*Disable of Write protection on the selected Sector*/
            status = FLASH_OB_DisableWRP((*pOBInit).WRPSector)
        }
    }
    /* Read protection configuration */
    if (*pOBInit).OptionType & 0x2 as libc::c_uint == 0x2 as libc::c_uint {
        status = FLASH_OB_RDP_LevelConfig((*pOBInit).RDPLevel as uint8_t)
    }
    /* USER  configuration */
    if (*pOBInit).OptionType & 0x4 as libc::c_uint == 0x4 as libc::c_uint {
        status =
            FLASH_OB_UserConfig((*pOBInit).USERConfig & 0x10 as libc::c_uint,
                                (*pOBInit).USERConfig & 0x20 as libc::c_uint,
                                (*pOBInit).USERConfig & 0x40 as libc::c_uint,
                                (*pOBInit).USERConfig & 0x80 as libc::c_uint,
                                (*pOBInit).USERConfig &
                                    0x80000000 as libc::c_uint,
                                (*pOBInit).USERConfig &
                                    0x40000000 as libc::c_uint)
        /* FLASH_OPTCR_nDBANK */
    }
    /* BOR Level  configuration */
    if (*pOBInit).OptionType & 0x8 as libc::c_uint == 0x8 as libc::c_uint {
        status = FLASH_OB_BOR_LevelConfig((*pOBInit).BORLevel as uint8_t)
    }
    /* Boot 0 Address configuration */
    if (*pOBInit).OptionType & 0x10 as libc::c_uint == 0x10 as libc::c_uint {
        status =
            FLASH_OB_BootAddressConfig(0x10 as libc::c_uint,
                                       (*pOBInit).BootAddr0)
    }
    /* Boot 1 Address configuration */
    if (*pOBInit).OptionType & 0x20 as libc::c_uint == 0x20 as libc::c_uint {
        status =
            FLASH_OB_BootAddressConfig(0x20 as libc::c_uint,
                                       (*pOBInit).BootAddr1)
    }
    /* PCROP configuration */
    if (*pOBInit).OptionType & 0x40 as libc::c_uint == 0x40 as libc::c_uint {
        status = FLASH_OB_PCROP_Config((*pOBInit).PCROPSector)
    }
    /* PCROP_RDP configuration */
    if (*pOBInit).OptionType & 0x80 as libc::c_uint == 0x80 as libc::c_uint {
        status = FLASH_OB_PCROP_RDP_Config((*pOBInit).PCROPRdp)
    }
    /* FLASH_OPTCR2_PCROP */
    /* Process Unlocked */
    pFlash.Lock = HAL_UNLOCKED;
    return status;
}
/* *
  * @brief  Get the Option byte configuration
  * @param  pOBInit: pointer to an FLASH_OBInitStruct structure that
  *         contains the configuration information for the programming.
  * 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASHEx_OBGetConfig(mut pOBInit:
                                                     *mut FLASH_OBProgramInitTypeDef) {
    (*pOBInit).OptionType =
        0x1 as libc::c_uint | 0x2 as libc::c_uint | 0x4 as libc::c_uint |
            0x8 as libc::c_uint | 0x10 as libc::c_uint | 0x20 as libc::c_uint;
    /*Get WRP*/
    (*pOBInit).WRPSector = FLASH_OB_GetWRP();
    /*Get RDP Level*/
    (*pOBInit).RDPLevel = FLASH_OB_GetRDP() as uint32_t;
    /*Get USER*/
    (*pOBInit).USERConfig = FLASH_OB_GetUser();
    /*Get BOR Level*/
    (*pOBInit).BORLevel = FLASH_OB_GetBOR();
    /*Get Boot Address when Boot pin = 0 */
    (*pOBInit).BootAddr0 = FLASH_OB_GetBootAddress(0x10 as libc::c_uint);
    /*Get Boot Address when Boot pin = 1 */
    (*pOBInit).BootAddr1 = FLASH_OB_GetBootAddress(0x20 as libc::c_uint);
    /*Get PCROP Sectors */
    (*pOBInit).PCROPSector = FLASH_OB_GetPCROP();
    /*Get PCROP_RDP Value */
    (*pOBInit).PCROPRdp = FLASH_OB_GetPCROPRDP();
    /* FLASH_OPTCR2_PCROP */
}
/* *
  * @}
  */
/* *
  * @brief  Full erase of FLASH memory sectors 
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.  
  *          This parameter can be one of the following values:
  *            @arg VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V, 
  *                                  the operation will be done by byte (8-bit) 
  *            @arg VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp, 
  *                                  the operation will be done by double word (64-bit)
  *
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_MassErase(mut VoltageRange: uint8_t) {
    /* Check the parameters */
    /* if the previous operation is completed, proceed to erase all sectors */
    let ref mut fresh4 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffcff as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh5 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         2 as libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh6 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          16 as libc::c_uint |
                                          (VoltageRange as uint32_t) <<
                                              8 as libc::c_int)) as uint32_t
                                    as uint32_t);
    /* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*/
    __DSB();
}
/* *
  * @}
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup FLASHEx_Exported_Constants FLASH Exported Constants
  * @{
  */
/* * @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
  */
/* !< Sectors erase only          */
/* !< Flash Mass erase activation */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Voltage_Range FLASH Voltage Range
  * @{
  */
/* !< Device operating range: 1.8V to 2.1V                */
/* !< Device operating range: 2.1V to 2.7V                */
/* !< Device operating range: 2.7V to 3.6V                */
/* !< Device operating range: 2.7V to 3.6V + External Vpp */
/* *
  * @}
  */
/* * @defgroup FLASHEx_WRP_State FLASH WRP State
  * @{
  */
/* !< Disable the write protection of the desired bank 1 sectors */
/* !< Enable the write protection of the desired bank 1 sectors  */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Type FLASH Option Type
  * @{
  */
/* !< WRP option byte configuration  */
/* !< RDP option byte configuration  */
/* !< USER option byte configuration */
/* !< BOR option byte configuration  */
/* !< Boot 0 Address configuration   */
/* !< Boot 1 Address configuration   */
/* !< PCROP configuration            */
/* !< PCROP_RDP configuration        */
/* FLASH_OPTCR2_PCROP */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_Read_Protection FLASH Option Bytes Read Protection
  * @{
  */
/* !< Warning: When enabling read protection level 2 
                                                  it s no more possible to go back to level 1 or 0 */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_WWatchdog FLASH Option Bytes WWatchdog
  * @{
  */
/* !< Software WWDG selected */
/* !< Hardware WWDG selected */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_IWatchdog FLASH Option Bytes IWatchdog
  * @{
  */
/* !< Software IWDG selected */
/* !< Hardware IWDG selected */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_nRST_STOP FLASH Option Bytes nRST_STOP
  * @{
  */
/* !< No reset generated when entering in STOP */
/* !< Reset generated when entering in STOP    */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_nRST_STDBY FLASH Option Bytes nRST_STDBY
  * @{
  */
/* !< No reset generated when entering in STANDBY */
/* !< Reset generated when entering in STANDBY    */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_IWDG_FREEZE_STOP FLASH IWDG Counter Freeze in STOP
  * @{
  */
/* !< Freeze IWDG counter in STOP mode */
/* !< IWDG counter active in STOP mode */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_IWDG_FREEZE_SANDBY FLASH IWDG Counter Freeze in STANDBY
  * @{
  */
/* !< Freeze IWDG counter in STANDBY mode */
/* !< IWDG counter active in STANDBY mode */
/* *
  * @}
  */
/* * @defgroup FLASHEx_BOR_Reset_Level FLASH BOR Reset Level
  * @{
  */
/* !< Supply voltage ranges from 2.70 to 3.60 V */
/* !< Supply voltage ranges from 2.40 to 2.70 V */
/* !< Supply voltage ranges from 2.10 to 2.40 V */
/* !< Supply voltage ranges from 1.62 to 2.10 V */
/* *
  * @}
  */
/* FLASH_OPTCR_nDBOOT */
/* FLASH_OPTCR_nDBANK */
/* * @defgroup FLASHEx_Boot_Address FLASH Boot Address
  * @{
  */
/* !< Boot from ITCM RAM (0x00000000)                 */
/* !< Boot from System memory bootloader (0x00100000) */
/* !< Boot from Flash on ITCM interface (0x00200000)  */
/* !< Boot from Flash on AXIM interface (0x08000000)  */
/* !< Boot from DTCM RAM (0x20000000)                 */
/* !< Boot from SRAM1 (0x20010000)                    */
/* !< Boot from SRAM2 (0x2003C000)                    */
/* SRAM2_BASE == 0x2003C000U */
/* *
  * @}
  */
/* * @defgroup FLASH_Latency FLASH Latency
  * @{
  */
/* !< FLASH Zero Latency cycle      */
/* !< FLASH One Latency cycle       */
/* !< FLASH Two Latency cycles      */
/* !< FLASH Three Latency cycles    */
/* !< FLASH Four Latency cycles     */
/* !< FLASH Five Latency cycles     */
/* !< FLASH Six Latency cycles      */
/* !< FLASH Seven Latency cycles    */
/* !< FLASH Eight Latency cycles    */
/* !< FLASH Nine Latency cycles     */
/* !< FLASH Ten Latency cycles      */
/* !< FLASH Eleven Latency cycles   */
/* !< FLASH Twelve Latency cycles   */
/* !< FLASH Thirteen Latency cycles */
/* !< FLASH Fourteen Latency cycles */
/* !< FLASH Fifteen Latency cycles  */
/* *
  * @}
  */
/* FLASH_OPTCR_nDBANK */
/* * @defgroup FLASHEx_MassErase_bit FLASH Mass Erase bit
  * @{
  */
/* !< only 1 MER bit */
/* FLASH_OPTCR_nDBANK */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Sectors FLASH Sectors
  * @{
  */
/* FLASH_SECTOR_TOTAL == 24 */
/* *
  * @}
  */
/* FLASH_SECTOR_TOTAL == 24 */
/* * @defgroup FLASHEx_Option_Bytes_Write_Protection FLASH Option Bytes Write Protection
  * @{
  */
/* !< Write protection of Sector0     */
/* !< Write protection of Sector1     */
/* !< Write protection of Sector2     */
/* !< Write protection of Sector3     */
/* !< Write protection of Sector4     */
/* !< Write protection of Sector5     */
/* !< Write protection of Sector6     */
/* !< Write protection of Sector7     */
/* !< Write protection of all Sectors */
/* *
  * @}
  */
/* FLASH_SECTOR_TOTAL == 8 */
/* * @defgroup FLASHEx_Option_Bytes_PCROP_Sectors FLASH Option Bytes PCROP Sectors
  * @{
  */
/* !< PC Readout protection of Sector0      */
/* !< PC Readout protection of Sector1      */
/* !< PC Readout protection of Sector2      */
/* !< PC Readout protection of Sector3      */
/* !< PC Readout protection of Sector4      */
/* !< PC Readout protection of Sector5      */
/* !< PC Readout protection of Sector6      */
/* !< PC Readout protection of Sector7      */
/* !< PC Readout protection of all Sectors  */
/* *
  * @}
  */
/* * @defgroup FLASHEx_Option_Bytes_PCROP_RDP FLASH Option Bytes PCROP_RDP Bit
  * @{
  */
/* !< PCROP_RDP Enable      */
/* !< PCROP_RDP Disable     */
/* *
  * @}
  */
/* FLASH_OPTCR2_PCROP */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Macros FLASH Exported Macros
  * @{
  */
/* *
  * @brief  Calculate the FLASH Boot Base Adress (BOOT_ADD0 or BOOT_ADD1)
  * @note   Returned value BOOT_ADDx[15:0] corresponds to boot address [29:14].
  * @param  __ADDRESS__: FLASH Boot Address (in the range 0x0000 0000 to 0x2004 FFFF with a granularity of 16KB)
  * @retval The FLASH Boot Base Adress
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup FLASHEx_Exported_Functions
  * @{
  */
/* * @addtogroup FLASHEx_Exported_Functions_Group1
  * @{
  */
/* Extension Program operation functions  *************************************/
/* *
  * @}
  */
/* *
  * @}
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @defgroup FLASHEx_Private_Macros FLASH Private Macros
  * @{
  */
/* * @defgroup FLASHEx_IS_FLASH_Definitions FLASH Private macros to check input parameters
  * @{
  */
/* FLASH_OPTCR2_PCROP */
/* FLASH_SECTOR_TOTAL == 8 */
/* FLASH_SECTOR_TOTAL == 24 */
/* FLASH_OPTCR_nDBANK */
/* FLASH_OPTCR_nDBOOT */
/* FLASH_OPTCR2_PCROP */
/* *
  * @}
  */
/* *
  * @}
  */
/* Private functions ---------------------------------------------------------*/
/* * @defgroup FLASHEx_Private_Functions FLASH Private Functions
  * @{
  */
/* *
  * @brief  Erase the specified FLASH memory sector
  * @param  Sector: FLASH sector to erase
  *         The value of this parameter depend on device used within the same series      
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.  
  *          This parameter can be one of the following values:
  *            @arg FLASH_VOLTAGE_RANGE_1: when the device voltage range is 1.8V to 2.1V, 
  *                                  the operation will be done by byte (8-bit) 
  *            @arg FLASH_VOLTAGE_RANGE_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg FLASH_VOLTAGE_RANGE_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg FLASH_VOLTAGE_RANGE_4: when the device voltage range is 2.7V to 3.6V + External Vpp, 
  *                                  the operation will be done by double word (64-bit)
  * 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_Erase_Sector(mut Sector: uint32_t,
                                            mut VoltageRange: uint8_t) {
    let mut tmp_psize: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    if VoltageRange as libc::c_uint == 0 as libc::c_uint {
        tmp_psize = 0 as libc::c_uint
    } else if VoltageRange as libc::c_uint == 0x1 as libc::c_uint {
        tmp_psize = (0x1 as libc::c_uint) << 8 as libc::c_uint
    } else if VoltageRange as libc::c_uint == 0x2 as libc::c_uint {
        tmp_psize = (0x2 as libc::c_uint) << 8 as libc::c_uint
    } else { tmp_psize = (0x3 as libc::c_uint) << 8 as libc::c_uint }
    /* If the previous operation is completed, proceed to erase the sector */
    let ref mut fresh7 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffcff as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh8 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | tmp_psize) as uint32_t
                                    as uint32_t);
    let ref mut fresh9 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xffffff07 as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint |
                                          Sector <<
                                              __RBIT((0xf as libc::c_uint) <<
                                                         3 as
                                                             libc::c_uint).leading_zeros()
                                                  as i32 as uint8_t as
                                                  libc::c_int)) as uint32_t as
                                    uint32_t);
    let ref mut fresh11 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         16 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*/
    __DSB();
}
/* *
  * @brief  Return the FLASH Write Protection Option Bytes value.
  * @retval uint32_t FLASH Write Protection Option Bytes value
  */
unsafe extern "C" fn FLASH_OB_GetWRP() -> uint32_t {
    /* Return the FLASH write protection Register value */
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).OPTCR &
               0xff0000 as libc::c_int as libc::c_uint;
}
/* *
  * @brief  Program the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.    
  * @param  Wwdg: Selects the IWDG mode
  *          This parameter can be one of the following values:
  *            @arg OB_WWDG_SW: Software WWDG selected
  *            @arg OB_WWDG_HW: Hardware WWDG selected
  * @param  Iwdg: Selects the WWDG mode
  *          This parameter can be one of the following values:
  *            @arg OB_IWDG_SW: Software IWDG selected
  *            @arg OB_IWDG_HW: Hardware IWDG selected
  * @param  Stop: Reset event when entering STOP mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STOP_NO_RST: No reset generated when entering in STOP
  *            @arg OB_STOP_RST: Reset generated when entering in STOP
  * @param  Stdby: Reset event when entering Standby mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_STDBY_NO_RST: No reset generated when entering in STANDBY
  *            @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @param  Iwdgstop: Independent watchdog counter freeze in Stop mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_IWDG_STOP_FREEZE: Freeze IWDG counter in STOP
  *            @arg OB_IWDG_STOP_ACTIVE: IWDG counter active in STOP
  * @param  Iwdgstdby: Independent watchdog counter freeze in standby mode.
  *          This parameter  can be one of the following values:
  *            @arg OB_IWDG_STDBY_FREEZE: Freeze IWDG counter in STANDBY
  *            @arg OB_IWDG_STDBY_ACTIVE: IWDG counter active in STANDBY           
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_OB_UserConfig(mut Wwdg: uint32_t,
                                         mut Iwdg: uint32_t,
                                         mut Stop: uint32_t,
                                         mut Stdby: uint32_t,
                                         mut Iwdgstop: uint32_t,
                                         mut Iwdgstdby: uint32_t)
 -> HAL_StatusTypeDef {
    let mut useroptionmask: uint32_t = 0 as libc::c_int as uint32_t;
    let mut useroptionvalue: uint32_t = 0 as libc::c_int as uint32_t;
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        useroptionmask =
            (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 7 as libc::c_uint |
                (0x1 as libc::c_uint) << 31 as libc::c_uint |
                (0x1 as libc::c_uint) << 30 as libc::c_uint;
        useroptionvalue = Iwdg | Wwdg | Stop | Stdby | Iwdgstop | Iwdgstdby;
        /* Update User Option Byte */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).OPTCR
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut FLASH_TypeDef)).OPTCR &
                                        !useroptionmask | useroptionvalue)
    }
    return status;
}
/* *
  * @brief  Return the FLASH User Option Byte value.
  * @retval uint32_t FLASH User Option Bytes values: WWDG_SW(Bit4), IWDG_SW(Bit5), nRST_STOP(Bit6), 
  *         nRST_STDBY(Bit7), IWDG_STDBY(Bit30) and IWDG_STOP(Bit31).
  */
unsafe extern "C" fn FLASH_OB_GetUser() -> uint32_t {
    /* Return the User Option Byte */
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).OPTCR & 0xc00000f0 as libc::c_uint;
}
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* * @addtogroup FLASHEx_Private_Functions
  * @{
  */
/* Option bytes control */
/* FLASH_OPTCR_nDBANK */
/* *
  * @brief  Enable the write protection of the desired bank1 or bank2 sectors
  *
  * @note   When the memory read protection level is selected (RDP level = 1), 
  *         it is not possible to program or erase the flash sector i if CortexM7  
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1    
  * 
  * @param  WRPSector: specifies the sector(s) to be write protected.
  *          This parameter can be one of the following values:
  *            @arg WRPSector: A value between OB_WRP_SECTOR_0 and OB_WRP_SECTOR_7 (for STM32F74xxx/STM32F75xxx devices)
  *              or a value between OB_WRP_SECTOR_0 and OB_WRP_SECTOR_11 (in Single Bank mode for STM32F76xxx/STM32F77xxx devices)
  *              or a value between OB_WRP_DB_SECTOR_0 and OB_WRP_DB_SECTOR_23 (in Dual Bank mode for STM32F76xxx/STM32F77xxx devices)
  *            @arg OB_WRP_SECTOR_All
  *
  * @retval HAL FLASH State   
  */
unsafe extern "C" fn FLASH_OB_EnableWRP(mut WRPSector: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        /*Write protection enabled on sectors */
        let ref mut fresh12 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut FLASH_TypeDef)).OPTCR;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !WRPSector) as
                                        uint32_t as uint32_t)
    }
    return status;
}
/* *
  * @brief  Disable the write protection of the desired bank1 or bank 2 sectors
  *
  * @note   When the memory read protection level is selected (RDP level = 1), 
  *         it is not possible to program or erase the flash sector i if CortexM4  
  *         debug features are connected or boot code is executed in RAM, even if nWRPi = 1  
  * 
  * @param  WRPSector: specifies the sector(s) to be write protected.
  *          This parameter can be one of the following values:
  *            @arg WRPSector: A value between OB_WRP_SECTOR_0 and OB_WRP_SECTOR_7 (for STM32F74xxx/STM32F75xxx devices)
  *              or a value between OB_WRP_SECTOR_0 and OB_WRP_SECTOR_11 (in Single Bank mode for STM32F76xxx/STM32F77xxx devices)
  *              or a value between OB_WRP_DB_SECTOR_0 and OB_WRP_DB_SECTOR_23 (in Dual Bank mode for STM32F76xxx/STM32F77xxx devices)                      
  *            @arg OB_WRP_Sector_All
  *
  *
  * @retval HAL Status   
  */
unsafe extern "C" fn FLASH_OB_DisableWRP(mut WRPSector: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        /* Write protection disabled on sectors */
        let ref mut fresh13 =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut FLASH_TypeDef)).OPTCR;
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | WRPSector) as
                                        uint32_t as uint32_t)
    }
    return status;
}
/* *
  * @brief  Set the read protection level.
  * @param  Level: specifies the read protection level.
  *          This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  *   
  * @note WARNING: When enabling OB_RDP level 2 it's no more possible to go back to level 1 or 0
  *    
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_OB_RDP_LevelConfig(mut Level: uint8_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(0x40023c15 as libc::c_int as uint32_t as
                                        *mut uint8_t, Level)
    }
    return status;
}
/* *
  * @brief  Set the BOR Level. 
  * @param  Level: specifies the Option Bytes BOR Reset Level.
  *          This parameter can be one of the following values:
  *            @arg OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *            @arg OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *            @arg OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *            @arg OB_BOR_OFF: Supply voltage ranges from 1.62 to 2.1 V
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_OB_BOR_LevelConfig(mut Level: uint8_t)
 -> HAL_StatusTypeDef {
    /* Check the parameters */
    /* Set the BOR Level */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTCR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3c00
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut FLASH_TypeDef)).OPTCR &
                                    !((0x3 as libc::c_uint) <<
                                          2 as libc::c_uint) |
                                    Level as libc::c_uint);
    return HAL_OK;
}
/* *
  * @brief  Configure Boot base address.
  * 
  * @param   BootOption : specifies Boot base address depending from Boot pin = 0 or pin = 1
  *          This parameter can be one of the following values:
  *            @arg OPTIONBYTE_BOOTADDR_0 : Boot address based when Boot pin = 0                 
  *            @arg OPTIONBYTE_BOOTADDR_1 : Boot address based when Boot pin = 1  
  * @param   Address: specifies Boot base address
  *          This parameter can be one of the following values:
  *            @arg OB_BOOTADDR_ITCM_RAM : Boot from ITCM RAM (0x00000000)                 
  *            @arg OB_BOOTADDR_SYSTEM : Boot from System memory bootloader (0x00100000) 
  *            @arg OB_BOOTADDR_ITCM_FLASH : Boot from Flash on ITCM interface (0x00200000)  
  *            @arg OB_BOOTADDR_AXIM_FLASH : Boot from Flash on AXIM interface (0x08000000)  
  *            @arg OB_BOOTADDR_DTCM_RAM : Boot from DTCM RAM (0x20000000)                 
  *            @arg OB_BOOTADDR_SRAM1 : Boot from SRAM1 (0x20010000)                    
  *            @arg OB_BOOTADDR_SRAM2 : Boot from SRAM2 (0x2004C000)              
  *    
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_OB_BootAddressConfig(mut BootOption: uint32_t,
                                                mut Address: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        if BootOption == 0x10 as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3c00
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut FLASH_TypeDef)).OPTCR1
                                            as *mut uint32_t,
                                        (*((0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x3c00
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut FLASH_TypeDef)).OPTCR1
                                            &
                                            !((0xffff as libc::c_uint) <<
                                                  0 as libc::c_uint) |
                                            Address)
        } else {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                     libc::c_uint).wrapping_add(0x20000
                                                                                    as
                                                                                    libc::c_uint).wrapping_add(0x3c00
                                                                                                                   as
                                                                                                                   libc::c_uint)
                                                    as
                                                    *mut FLASH_TypeDef)).OPTCR1
                                            as *mut uint32_t,
                                        (*((0x40000000 as
                                                libc::c_uint).wrapping_add(0x20000
                                                                               as
                                                                               libc::c_uint).wrapping_add(0x3c00
                                                                                                              as
                                                                                                              libc::c_uint)
                                               as *mut FLASH_TypeDef)).OPTCR1
                                            &
                                            !((0xffff as libc::c_uint) <<
                                                  16 as libc::c_uint) |
                                            Address << 16 as libc::c_int)
        }
    }
    return status;
}
/* *
  * @brief  Returns the FLASH Read Protection level.
  * @retval FlagStatus FLASH ReadOut Protection Status:
  *         This parameter can be one of the following values:
  *            @arg OB_RDP_LEVEL_0: No protection
  *            @arg OB_RDP_LEVEL_1: Read protection of the memory
  *            @arg OB_RDP_LEVEL_2: Full chip protection
  */
unsafe extern "C" fn FLASH_OB_GetRDP() -> uint8_t {
    let mut readstatus: uint8_t = 0xaa as libc::c_uint as uint8_t;
    if *(0x40023c15 as libc::c_int as uint32_t as *mut uint8_t) as libc::c_int
           == 0xaa as libc::c_uint as uint8_t as libc::c_int {
        readstatus = 0xaa as libc::c_uint as uint8_t
    } else if *(0x40023c15 as libc::c_int as uint32_t as *mut uint8_t) as
                  libc::c_int ==
                  0xcc as libc::c_uint as uint8_t as libc::c_int {
        readstatus = 0xcc as libc::c_uint as uint8_t
    } else { readstatus = 0x55 as libc::c_uint as uint8_t }
    return readstatus;
}
/* *
  * @brief  Returns the FLASH BOR level.
  * @retval uint32_t The FLASH BOR level:
  *           - OB_BOR_LEVEL3: Supply voltage ranges from 2.7 to 3.6 V
  *           - OB_BOR_LEVEL2: Supply voltage ranges from 2.4 to 2.7 V
  *           - OB_BOR_LEVEL1: Supply voltage ranges from 2.1 to 2.4 V
  *           - OB_BOR_OFF   : Supply voltage ranges from 1.62 to 2.1 V  
  */
unsafe extern "C" fn FLASH_OB_GetBOR() -> uint32_t {
    /* Return the FLASH BOR level */
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).OPTCR &
               0xc as libc::c_int as libc::c_uint;
}
/* *
  * @brief  Configure Boot base address.
  * 
  * @param   BootOption : specifies Boot base address depending from Boot pin = 0 or pin = 1
  *          This parameter can be one of the following values:
  *            @arg OPTIONBYTE_BOOTADDR_0 : Boot address based when Boot pin = 0                 
  *            @arg OPTIONBYTE_BOOTADDR_1 : Boot address based when Boot pin = 1       
  *    
  * @retval uint32_t Boot Base Address:
  *            - OB_BOOTADDR_ITCM_RAM : Boot from ITCM RAM (0x00000000)                 
  *            - OB_BOOTADDR_SYSTEM : Boot from System memory bootloader (0x00100000) 
  *            - OB_BOOTADDR_ITCM_FLASH : Boot from Flash on ITCM interface (0x00200000)  
  *            - OB_BOOTADDR_AXIM_FLASH : Boot from Flash on AXIM interface (0x08000000)  
  *            - OB_BOOTADDR_DTCM_RAM : Boot from DTCM RAM (0x20000000)                 
  *            - OB_BOOTADDR_SRAM1 : Boot from SRAM1 (0x20010000)                    
  *            - OB_BOOTADDR_SRAM2 : Boot from SRAM2 (0x2004C000) 
  */
unsafe extern "C" fn FLASH_OB_GetBootAddress(mut BootOption: uint32_t)
 -> uint32_t {
    let mut Address: uint32_t = 0 as libc::c_int as uint32_t;
    /* Return the Boot base Address */
    if BootOption == 0x10 as libc::c_uint {
        Address =
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3c00
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut FLASH_TypeDef)).OPTCR1 &
                (0xffff as libc::c_uint) << 0 as libc::c_uint
    } else {
        Address =
            ((*((0x40000000 as
                     libc::c_uint).wrapping_add(0x20000 as
                                                    libc::c_uint).wrapping_add(0x3c00
                                                                                   as
                                                                                   libc::c_uint)
                    as *mut FLASH_TypeDef)).OPTCR1 &
                 (0xffff as libc::c_uint) << 16 as libc::c_uint) >>
                16 as libc::c_int
    }
    return Address;
}
/* FLASH_OPTCR_nDBANK */
/* *
  * @brief  Set the PCROP protection for sectors.
  * @param  PCROPSector: specifies the sector(s) to be PCROP protected.
  *         This parameter can be one of the following values:
  *            @arg OB_PCROP_SECTOR_x: A value between OB_PCROP_SECTOR_0 and OB_PCROP_SECTOR_7
  *            @arg OB_PCROP_SECTOR_ALL
  *    
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_OB_PCROP_Config(mut PCROPSector: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).OPTCR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut FLASH_TypeDef)).OPTCR2 &
                                        !((0xff as libc::c_uint) <<
                                              0 as libc::c_uint) |
                                        PCROPSector)
    }
    return status;
}
/* *
  * @brief  Set the PCROP_RDP value
  * @param  Pcrop_Rdp: specifies the PCROP_RDP bit value.
  *    
  * @retval HAL Status
  */
unsafe extern "C" fn FLASH_OB_PCROP_RDP_Config(mut Pcrop_Rdp: uint32_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).OPTCR2
                                        as *mut uint32_t,
                                    (*((0x40000000 as
                                            libc::c_uint).wrapping_add(0x20000
                                                                           as
                                                                           libc::c_uint).wrapping_add(0x3c00
                                                                                                          as
                                                                                                          libc::c_uint)
                                           as *mut FLASH_TypeDef)).OPTCR2 &
                                        !((0x1 as libc::c_uint) <<
                                              31 as libc::c_uint) | Pcrop_Rdp)
    }
    return status;
}
/* *
  * @brief  Return the FLASH PCROP Protection Option Bytes value.
  * @retval uint32_t FLASH PCROP Protection Option Bytes value
  */
unsafe extern "C" fn FLASH_OB_GetPCROP() -> uint32_t {
    /* Return the FLASH write protection Register value */
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).OPTCR2 &
               (0xff as libc::c_uint) << 0 as libc::c_uint;
}
/* *
  * @brief  Return the FLASH PCROP_RDP option byte value.
  * @retval uint32_t FLASH PCROP_RDP option byte value
  */
unsafe extern "C" fn FLASH_OB_GetPCROPRDP() -> uint32_t {
    /* Return the FLASH write protection Register value */
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut FLASH_TypeDef)).OPTCR2 &
               (0x1 as libc::c_uint) << 31 as libc::c_uint;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* HAL_FLASH_MODULE_ENABLED */
/* *
  * @}
  */
/* FLASH_OPTCR2_PCROP */
