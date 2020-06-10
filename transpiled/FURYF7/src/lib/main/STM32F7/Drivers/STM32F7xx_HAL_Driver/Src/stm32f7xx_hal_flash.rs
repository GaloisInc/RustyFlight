use ::libc;
extern "C" {
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
    #[no_mangle]
    fn FLASH_Erase_Sector(Sector: uint32_t, VoltageRange: uint8_t);
    #[no_mangle]
    fn HAL_GetTick() -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
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
pub type FLASH_ProcedureTypeDef = libc::c_uint;
pub const FLASH_PROC_PROGRAM: FLASH_ProcedureTypeDef = 3;
pub const FLASH_PROC_MASSERASE: FLASH_ProcedureTypeDef = 2;
pub const FLASH_PROC_SECTERASE: FLASH_ProcedureTypeDef = 1;
pub const FLASH_PROC_NONE: FLASH_ProcedureTypeDef = 0;
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
#[inline(always)]
unsafe extern "C" fn __DSB() { asm!("dsb 0xF" : : : "memory" : "volatile"); }
/* 50 s */
/* *
  * @}
  */         
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @addtogroup FLASH_Private_Variables
  * @{
  */
/* Variable used for Erase sectors under interruption */
#[no_mangle]
pub static mut pFlash: FLASH_ProcessTypeDef =
    FLASH_ProcessTypeDef{ProcedureOnGoing: FLASH_PROC_NONE,
                         NbSectorsToErase: 0,
                         VoltageForErase: 0,
                         Sector: 0,
                         Address: 0,
                         Lock: HAL_UNLOCKED,
                         ErrorCode: 0,};
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Functions FLASH Exported Functions
  * @{
  */
/* * @defgroup FLASH_Exported_Functions_Group1 Programming operation functions 
 *  @brief   Programming operation functions 
 *
@verbatim   
 ===============================================================================
                  ##### Programming operation functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the FLASH 
    program operations.

@endverbatim
  * @{
  */
/* *
  * @brief  Program byte, halfword, word or double word at a specified address
  * @param  TypeProgram:  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address:  specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed
  * 
  * @retval HAL_StatusTypeDef HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_Program(mut TypeProgram: uint32_t,
                                           mut Address: uint32_t,
                                           mut Data: uint64_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_ERROR;
    /* Process Locked */
    if pFlash.Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { pFlash.Lock = HAL_LOCKED }
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
    if status as libc::c_uint == HAL_OK as libc::c_int as libc::c_uint {
        match TypeProgram {
            0 => {
                /*Program byte (8-bit) at a specified address.*/
                FLASH_Program_Byte(Address, Data as uint8_t);
            }
            1 => {
                /*Program halfword (16-bit) at a specified address.*/
                FLASH_Program_HalfWord(Address, Data as uint16_t);
            }
            2 => {
                /*Program word (32-bit) at a specified address.*/
                FLASH_Program_Word(Address, Data as uint32_t);
            }
            3 => {
                /*Program double word (64-bit) at a specified address.*/
                FLASH_Program_DoubleWord(Address, Data);
            }
            _ => { }
        }
        /* Wait for last operation to be completed */
        status = FLASH_WaitForLastOperation(50000 as libc::c_uint);
        /* If the program operation is completed, disable the PG Bit */
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
                                               0 as libc::c_uint)) as uint32_t
                                        as uint32_t)
    }
    /* Process Unlocked */
    pFlash.Lock = HAL_UNLOCKED;
    return status;
}
/* *
  * @brief   Program byte, halfword, word or double word at a specified address  with interrupt enabled.
  * @param  TypeProgram:  Indicate the way to program at a specified address.
  *                           This parameter can be a value of @ref FLASH_Type_Program
  * @param  Address:  specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed
  * 
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_Program_IT(mut TypeProgram: uint32_t,
                                              mut Address: uint32_t,
                                              mut Data: uint64_t)
 -> HAL_StatusTypeDef {
    let mut status: HAL_StatusTypeDef = HAL_OK;
    /* Process Locked */
    if pFlash.Lock as libc::c_uint ==
           HAL_LOCKED as libc::c_int as libc::c_uint {
        return HAL_BUSY
    } else { pFlash.Lock = HAL_LOCKED }
    /* Check the parameters */
    /* Enable End of FLASH Operation interrupt */
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
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         24 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Enable Error source interrupt */
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
    ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing as
                                    *mut FLASH_ProcedureTypeDef,
                                FLASH_PROC_PROGRAM);
    ::core::ptr::write_volatile(&mut pFlash.Address as *mut uint32_t,
                                Address);
    match TypeProgram {
        0 => {
            /*Program byte (8-bit) at a specified address.*/
            FLASH_Program_Byte(Address, Data as uint8_t);
        }
        1 => {
            /*Program halfword (16-bit) at a specified address.*/
            FLASH_Program_HalfWord(Address, Data as uint16_t);
        }
        2 => {
            /*Program word (32-bit) at a specified address.*/
            FLASH_Program_Word(Address, Data as uint32_t);
        }
        3 => {
            /*Program double word (64-bit) at a specified address.*/
            FLASH_Program_DoubleWord(Address, Data);
        }
        _ => { }
    }
    return status;
}
/* *
  * @brief This function handles FLASH interrupt request.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_IRQHandler() {
    let mut temp: uint32_t = 0 as libc::c_int as uint32_t;
    /* If the program operation is completed, disable the PG Bit */
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
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* If the erase operation is completed, disable the SER Bit */
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
                                     !((0x1 as libc::c_uint) <<
                                           1 as libc::c_uint)) as uint32_t as
                                    uint32_t);
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
                                     as libc::c_uint &
                                     0xffffff07 as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* if the erase operation is completed, disable the MER Bit */
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
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           2 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Check FLASH End of Operation flag  */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Clear FLASH End of Operation pending bit */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).SR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint);
        match pFlash.ProcedureOnGoing as libc::c_uint {
            1 => {
                /* Nb of sector to erased can be decreased */
                ::core::ptr::write_volatile(&mut pFlash.NbSectorsToErase as
                                                *mut uint32_t,
                                            ::core::ptr::read_volatile::<uint32_t>(&pFlash.NbSectorsToErase
                                                                                       as
                                                                                       *const uint32_t).wrapping_sub(1));
                /* Check if there are still sectors to erase */
                if pFlash.NbSectorsToErase != 0 as libc::c_int as libc::c_uint
                   {
                    temp = pFlash.Sector;
                    /* Indicate user which sector has been erased */
                    HAL_FLASH_EndOfOperationCallback(temp);
                    /* Increment sector number */
                    ::core::ptr::write_volatile(&mut pFlash.Sector as
                                                    *mut uint32_t,
                                                ::core::ptr::read_volatile::<uint32_t>(&pFlash.Sector
                                                                                           as
                                                                                           *const uint32_t).wrapping_add(1));
                    temp =
                        ::core::ptr::read_volatile::<uint32_t>(&pFlash.Sector
                                                                   as
                                                                   *const uint32_t);
                    FLASH_Erase_Sector(temp, pFlash.VoltageForErase);
                } else {
                    /* No more sectors to Erase, user callback can be called.*/
          /* Reset Sector and stop Erase sectors procedure */
                    temp = 0xffffffff as libc::c_uint;
                    ::core::ptr::write_volatile(&mut pFlash.Sector as
                                                    *mut uint32_t, temp);
                    /* FLASH EOP interrupt user callback */
                    HAL_FLASH_EndOfOperationCallback(temp);
                    /* Sector Erase procedure is completed */
                    ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing
                                                    as
                                                    *mut FLASH_ProcedureTypeDef,
                                                FLASH_PROC_NONE)
                }
            }
            2 => {
                /* MassErase ended. Return the selected bank : in this product we don't have Banks */
        /* FLASH EOP interrupt user callback */
                HAL_FLASH_EndOfOperationCallback(0 as libc::c_int as
                                                     uint32_t);
                /* MAss Erase procedure is completed */
                ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing as
                                                *mut FLASH_ProcedureTypeDef,
                                            FLASH_PROC_NONE)
            }
            3 => {
                /*Program ended. Return the selected address*/
        /* FLASH EOP interrupt user callback */
                HAL_FLASH_EndOfOperationCallback(pFlash.Address);
                /* Programming procedure is completed */
                ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing as
                                                *mut FLASH_ProcedureTypeDef,
                                            FLASH_PROC_NONE)
            }
            _ => { }
        }
    }
    /* Check FLASH operation error flags */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           ((0x1 as libc::c_uint) << 1 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 7 as libc::c_uint) !=
           RESET as libc::c_int as libc::c_uint {
        match pFlash.ProcedureOnGoing as libc::c_uint {
            1 => {
                /* return the faulty sector */
                temp = pFlash.Sector;
                ::core::ptr::write_volatile(&mut pFlash.Sector as
                                                *mut uint32_t,
                                            0xffffffff as libc::c_uint)
            }
            2 => {
                /* No return in case of Mass Erase */
                temp = 0 as libc::c_int as uint32_t
            }
            3 => {
                /*return the faulty address*/
                temp = pFlash.Address
            }
            _ => { }
        }
        /*Save the Error code*/
        FLASH_SetErrorCode();
        /* FLASH error interrupt user callback */
        HAL_FLASH_OperationErrorCallback(temp);
        /*Stop the procedure ongoing */
        ::core::ptr::write_volatile(&mut pFlash.ProcedureOnGoing as
                                        *mut FLASH_ProcedureTypeDef,
                                    FLASH_PROC_NONE)
    }
    if pFlash.ProcedureOnGoing as libc::c_uint ==
           FLASH_PROC_NONE as libc::c_int as libc::c_uint {
        /* Disable End of FLASH Operation interrupt */
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
                                         !((0x1 as libc::c_uint) <<
                                               24 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Disable Error source interrupt */
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
                                         as libc::c_uint &
                                         !(0x2000000 as libc::c_uint)) as
                                        uint32_t as uint32_t);
        /* Process Unlocked */
        pFlash.Lock = HAL_UNLOCKED
    };
}
/* *
  * @brief  FLASH end of operation interrupt callback
  * @param  ReturnValue: The value saved in this parameter depends on the ongoing procedure
  *                 - Sectors Erase: Sector which has been erased (if 0xFFFFFFFF, it means that 
  *                                  all the selected sectors have been erased)
  *                 - Program      : Address which was selected for data program
  *                 - Mass Erase   : No return value expected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_EndOfOperationCallback(mut ReturnValue:
                                                              uint32_t) {
    /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_FLASH_EndOfOperationCallback could be implemented in the user file
  */
}
/* *
  * @brief  FLASH operation error interrupt callback
  * @param  ReturnValue: The value saved in this parameter depends on the ongoing procedure
  *                 - Sectors Erase: Sector which has been erased (if 0xFFFFFFFF, it means that 
  *                                  all the selected sectors have been erased)
  *                 - Program      : Address which was selected for data program
  *                 - Mass Erase   : No return value expected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_OperationErrorCallback(mut ReturnValue:
                                                              uint32_t) {
    /* NOTE : This function Should not be modified, when the callback is needed,
  the HAL_FLASH_OperationErrorCallback could be implemented in the user file
   */
}
/* *
  * @}
  */
/* * @defgroup FLASH_Exported_Functions_Group2 Peripheral Control functions 
 *  @brief   management functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the FLASH 
    memory operations.

@endverbatim
  * @{
  */
/* *
  * @brief  Unlock the FLASH control register access
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_Unlock() -> HAL_StatusTypeDef {
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).CR &
           (0x1 as libc::c_uint) << 31 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Authorize the FLASH Registers access */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).KEYR
                                        as *mut uint32_t,
                                    0x45670123 as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).KEYR
                                        as *mut uint32_t,
                                    0xcdef89ab as libc::c_uint)
    } else { return HAL_ERROR }
    return HAL_OK;
}
/* *
  * @brief  Locks the FLASH control register access
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_Lock() -> HAL_StatusTypeDef {
    /* Set the LOCK Bit to lock the FLASH Registers access */
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
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         31 as libc::c_uint) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  Unlock the FLASH Option Control Registers access.
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_OB_Unlock() -> HAL_StatusTypeDef {
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).OPTCR &
           (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Authorizes the Option Byte register programming */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as
                                                *mut FLASH_TypeDef)).OPTKEYR
                                        as *mut uint32_t,
                                    0x8192a3b as libc::c_uint);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as
                                                *mut FLASH_TypeDef)).OPTKEYR
                                        as *mut uint32_t,
                                    0x4c5d6e7f as libc::c_uint)
    } else { return HAL_ERROR }
    return HAL_OK;
}
/* *
  * @brief  Lock the FLASH Option Control Registers access.
  * @retval HAL Status 
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_OB_Lock() -> HAL_StatusTypeDef {
    /* Set the OPTLOCK Bit to lock the FLASH Option Byte Registers access */
    let ref mut fresh10 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).OPTCR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    return HAL_OK;
}
/* *
  * @brief  Launch the option byte loading.
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_OB_Launch() -> HAL_StatusTypeDef {
    /* Set the OPTSTRT bit in OPTCR register */
    let ref mut fresh11 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).OPTCR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         1 as libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Wait for last operation to be completed */
    return FLASH_WaitForLastOperation(50000 as libc::c_uint);
}
/* *
  * @}
  */
/* * @defgroup FLASH_Exported_Functions_Group3 Peripheral State and Errors functions 
 *  @brief   Peripheral Errors functions 
 *
@verbatim   
 ===============================================================================
                ##### Peripheral Errors functions #####
 ===============================================================================  
    [..]
    This subsection permits to get in run-time Errors of the FLASH peripheral.

@endverbatim
  * @{
  */
/* *
  * @brief  Get the specific FLASH error flag.
  * @retval FLASH_ErrorCode: The returned value can be:
  *            @arg FLASH_ERROR_ERS: FLASH Erasing Sequence error flag 
  *            @arg FLASH_ERROR_PGP: FLASH Programming Parallelism error flag  
  *            @arg FLASH_ERROR_PGA: FLASH Programming Alignment error flag
  *            @arg FLASH_ERROR_WRP: FLASH Write protected error flag
  *            @arg FLASH_ERROR_OPERATION: FLASH operation Error flag 
  */
#[no_mangle]
pub unsafe extern "C" fn HAL_FLASH_GetError() -> uint32_t {
    return pFlash.ErrorCode;
}
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
/* * 
  * @brief  FLASH handle Structure definition  
  */
/* Internal variable to indicate which procedure is ongoing or not in IT context */
/* Internal variable to save the remaining sectors to erase in IT context        */
/* Internal variable to provide voltage range selected by user in IT context     */
/* Internal variable to define the current sector which is erasing               */
/* Internal variable to save address selected for program                        */
/* FLASH locking object                                                          */
/* FLASH error code                                                              */
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
/* *
  * @}
  */
/* *
  * @brief  Wait for a FLASH operation to complete.
  * @param  Timeout: maximum flash operationtimeout
  * @retval HAL Status
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_WaitForLastOperation(mut Timeout: uint32_t)
 -> HAL_StatusTypeDef {
    let mut tickstart: uint32_t = 0 as libc::c_int as uint32_t;
    /* Clear Error Code */
    ::core::ptr::write_volatile(&mut pFlash.ErrorCode as *mut uint32_t,
                                0 as libc::c_uint);
    /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  /* Get tick */
    tickstart = HAL_GetTick();
    while (*((0x40000000 as
                  libc::c_uint).wrapping_add(0x20000 as
                                                 libc::c_uint).wrapping_add(0x3c00
                                                                                as
                                                                                libc::c_uint)
                 as *mut FLASH_TypeDef)).SR &
              (0x1 as libc::c_uint) << 16 as libc::c_uint !=
              RESET as libc::c_int as libc::c_uint {
        if Timeout != 0xffffffff as libc::c_uint {
            if Timeout == 0 as libc::c_int as libc::c_uint ||
                   HAL_GetTick().wrapping_sub(tickstart) > Timeout {
                return HAL_TIMEOUT
            }
        }
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           ((0x1 as libc::c_uint) << 1 as libc::c_uint |
                (0x1 as libc::c_uint) << 4 as libc::c_uint |
                (0x1 as libc::c_uint) << 5 as libc::c_uint |
                (0x1 as libc::c_uint) << 6 as libc::c_uint |
                (0x1 as libc::c_uint) << 7 as libc::c_uint) !=
           RESET as libc::c_int as libc::c_uint {
        /*Save the error code*/
        FLASH_SetErrorCode();
        return HAL_ERROR
    }
    /* Check FLASH End of Operation flag  */
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 0 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        /* Clear FLASH End of Operation pending bit */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x20000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
                                                                                                               as
                                                                                                               libc::c_uint)
                                                as *mut FLASH_TypeDef)).SR as
                                        *mut uint32_t,
                                    (0x1 as libc::c_uint) <<
                                        0 as libc::c_uint)
    }
    /* If there is an error flag set */
    return HAL_OK;
}
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* * @addtogroup FLASH_Private_Functions
  * @{
  */
/* Program operations */
/* *
  * @brief  Program a double word (64-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V and an External Vpp is present.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
unsafe extern "C" fn FLASH_Program_DoubleWord(mut Address: uint32_t,
                                              mut Data: uint64_t) {
    /* Check the parameters */
    /* If the previous operation is completed, proceed to program the new data */
    let ref mut fresh12 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffcff as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh13 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x3 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh14 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(Address as *mut uint64_t, Data);
    /* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*/
    __DSB();
}
/* *
  * @brief  Program word (32-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
unsafe extern "C" fn FLASH_Program_Word(mut Address: uint32_t,
                                        mut Data: uint32_t) {
    /* Check the parameters */
    /* If the previous operation is completed, proceed to program the new data */
    let ref mut fresh15 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh15,
                                (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffcff as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh16 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x2 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh17 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(Address as *mut uint32_t, Data);
    /* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*/
    __DSB();
}
/* *
  * @brief  Program a half-word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
unsafe extern "C" fn FLASH_Program_HalfWord(mut Address: uint32_t,
                                            mut Data: uint16_t) {
    /* Check the parameters */
    /* If the previous operation is completed, proceed to program the new data */
    let ref mut fresh18 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffcff as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh19 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         8 as libc::c_uint) as uint32_t as
                                    uint32_t);
    let ref mut fresh20 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(Address as *mut uint16_t, Data);
    /* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*/
    __DSB();
}
/* *
  * @brief  Program byte (8-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from
  *         2.7V to 3.6V.
  *
  * @note   If an erase and a program operations are requested simultaneously,    
  *         the erase operation is performed before the program one.
  *  
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval None
  */
unsafe extern "C" fn FLASH_Program_Byte(mut Address: uint32_t,
                                        mut Data: uint8_t) {
    /* Check the parameters */
    /* If the previous operation is completed, proceed to program the new data */
    let ref mut fresh21 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh21,
                                (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffffcff as libc::c_uint) as uint32_t
                                    as uint32_t);
    let ref mut fresh22 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh22,
                                (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0 as libc::c_uint) as
                                    uint32_t as uint32_t);
    let ref mut fresh23 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh23,
                                (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(Address as *mut uint8_t, Data);
    /* Data synchronous Barrier (DSB) Just after the write operation
     This will force the CPU to respect the sequence of instruction (no optimization).*/
    __DSB();
}
/* *
  * @brief  Set the specific FLASH error flag.
  * @retval None
  */
unsafe extern "C" fn FLASH_SetErrorCode() {
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 1 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut pFlash.ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&pFlash.ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 4 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut pFlash.ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&pFlash.ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 5 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut pFlash.ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&pFlash.ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 6 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut pFlash.ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&pFlash.ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    if (*((0x40000000 as
               libc::c_uint).wrapping_add(0x20000 as
                                              libc::c_uint).wrapping_add(0x3c00
                                                                             as
                                                                             libc::c_uint)
              as *mut FLASH_TypeDef)).SR &
           (0x1 as libc::c_uint) << 7 as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut pFlash.ErrorCode as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&pFlash.ErrorCode
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* FLASH_OPTCR2_PCROP */
    /* Clear error programming flags */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3c00
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut FLASH_TypeDef)).SR as
                                    *mut uint32_t,
                                (0x1 as libc::c_uint) << 1 as libc::c_uint |
                                    (0x1 as libc::c_uint) << 4 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 5 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) << 6 as libc::c_uint
                                    |
                                    (0x1 as libc::c_uint) <<
                                        7 as libc::c_uint);
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
