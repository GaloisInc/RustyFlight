use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief FLASH Registers
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct FLASH_TypeDef {
    pub ACR: uint32_t,
    pub KEYR: uint32_t,
    pub OPTKEYR: uint32_t,
    pub SR: uint32_t,
    pub CR: uint32_t,
    pub AR: uint32_t,
    pub RESERVED: uint32_t,
    pub OBR: uint32_t,
    pub WRPR: uint32_t,
}
/* * 
  * @brief Option Bytes Registers
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct OB_TypeDef {
    pub RDP: uint16_t,
    pub USER: uint16_t,
    pub RESERVED0: uint16_t,
    pub RESERVED1: uint16_t,
    pub WRP0: uint16_t,
    pub WRP1: uint16_t,
    pub WRP2: uint16_t,
    pub WRP3: uint16_t,
}
pub type FLASH_Status = libc::c_uint;
pub const FLASH_TIMEOUT: FLASH_Status = 5;
pub const FLASH_COMPLETE: FLASH_Status = 4;
pub const FLASH_ERROR_PROGRAM: FLASH_Status = 3;
pub const FLASH_ERROR_WRP: FLASH_Status = 2;
pub const FLASH_BUSY: FLASH_Status = 1;
/* *
  ******************************************************************************
  * @file    stm32f30x_flash.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the FLASH peripheral:
  *            + FLASH Interface configuration
  *            + FLASH Memory Programming
  *            + Option Bytes Programming
  *            + Interrupts and flags management
  *  
  @verbatim
  
 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
    [..] This driver provides functions to configure and program the FLASH 
         memory of all STM32F30x devices. These functions are split in 4 groups:
         (#) FLASH Interface configuration functions: this group includes the
             management of following features:
             (++) Set the latency.
             (++) Enable/Disable the Half Cycle Access.
             (++) Enable/Disable the prefetch buffer.
         (#) FLASH Memory Programming functions: this group includes all needed
             functions to erase and program the main memory:
             (++) Lock and Unlock the FLASH interface.
             (++) Erase function: Erase page, erase all pages.
             (++) Program functions: Half Word and Word write.
         (#) FLASH Option Bytes Programming functions: this group includes all 
             needed functions to manage the Option Bytes:
             (++) Lock and Unlock the Flash Option bytes.
             (++) Launch the Option Bytes loader
             (++) Erase the Option Bytes
             (++) Set/Reset the write protection
             (++) Set the Read protection Level
             (++) Program the user option Bytes
             (++) Set/Reset the BOOT1 bit
             (++) Enable/Disable the VDDA Analog Monitoring
             (++) Enable/Disable the SRAM parity
             (++) Get the user option bytes
             (++) Get the Write protection
             (++) Get the read protection status
         (#) FLASH Interrupts and flags management functions: this group includes 
             all needed functions to:
             (++) Enable/Disable the FLASH interrupt sources.
             (++) Get flags status.
             (++) Clear flags.
             (++) Get FLASH operation status.
             (++) Wait for last FLASH operation.
 
  @endverbatim
                      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @defgroup FLASH 
  * @brief FLASH driver modules
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* FLASH Mask */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup FLASH_Private_Functions
  * @{
  */
/* * @defgroup FLASH_Group1 FLASH Interface configuration functions
  *  @brief   FLASH Interface configuration functions 
 *

@verbatim   
 ===============================================================================
            ##### FLASH Interface configuration functions #####
 ===============================================================================
    [..] This group includes the following functions:
         (+) void FLASH_SetLatency(uint32_t FLASH_Latency); 
         (+) void FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);     
         (+) void FLASH_PrefetchBufferCmd(FunctionalState NewState);
    [..] The unlock sequence is not needed for these functions.
 
@endverbatim
  * @{
  */
/* *
  * @brief  Sets the code latency value.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *          This parameter can be one of the following values:
  *            @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *            @arg FLASH_Latency_1: FLASH One Latency cycle
  *            @arg FLASH_Latency_2: FLASH Two Latency cycles      
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_SetLatency(mut FLASH_Latency: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Read the ACR register */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).ACR;
    /* Sets the Latency value */
    tmpreg &= !(0x3 as libc::c_int as uint8_t as uint32_t);
    tmpreg |= FLASH_Latency;
    /* Write the ACR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).ACR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the Half cycle flash access.
  * @param  FLASH_HalfCycleAccess: specifies the FLASH Half cycle Access mode.
  *          This parameter can be one of the following values:
  *            @arg FLASH_HalfCycleAccess_Enable: FLASH Half Cycle Enable
  *            @arg FLASH_HalfCycleAccess_Disable: FLASH Half Cycle Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_HalfCycleAccessCmd(mut NewState:
                                                      FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).ACR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8 as libc::c_int as uint8_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).ACR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8 as libc::c_int as uint8_t as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Prefetch Buffer.
  * @param  NewState: new state of the Prefetch Buffer.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_PrefetchBufferCmd(mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh2 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).ACR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint8_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        let ref mut fresh3 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).ACR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10 as libc::c_int as uint8_t as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup FLASH_Group2 FLASH Memory Programming functions
 *  @brief   FLASH Memory Programming functions
 *
@verbatim   
 ===============================================================================
              ##### FLASH Memory Programming functions #####
 ===============================================================================   
    [..] This group includes the following functions:
         (+) void FLASH_Unlock(void);
         (+) void FLASH_Lock(void);
         (+) FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
         (+) FLASH_Status FLASH_EraseAllPages(void);
         (+) FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
         (+) FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
    [..] Any operation of erase or program should follow these steps:
         (#) Call the FLASH_Unlock() function to enable the FLASH control register 
             program memory access.
         (#) Call the desired function to erase page or program data.
         (#) Call the FLASH_Lock() function to disable the FLASH control register 
             access (recommended to protect the FLASH memory against possible 
             unwanted operation).
    
@endverbatim
  * @{
  */
/* *
  * @brief  Unlocks the FLASH control register access
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_Unlock() {
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).CR & 0x80 as libc::c_int as uint32_t !=
           RESET as libc::c_int as libc::c_uint {
        /* Authorize the FLASH Registers access */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2000
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut FLASH_TypeDef)).KEYR
                                        as *mut uint32_t,
                                    0x45670123 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2000
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut FLASH_TypeDef)).KEYR
                                        as *mut uint32_t,
                                    0xcdef89ab as libc::c_uint)
    };
}
/* *
  * @brief  Locks the FLASH control register access
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_Lock() {
    /* Set the LOCK Bit to lock the FLASH Registers access */
    let ref mut fresh4 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x80 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Erases a specified page in program memory.
  * @note   To correctly run this function, the FLASH_Unlock() function
  *         must be called before.
  * @note   Call the FLASH_Lock() to disable the flash memory access 
  *         (recommended to protect the FLASH memory against possible unwanted operation)  
  * @param  Page_Address: The page address in program memory to be erased.
  * @note   A Page is erased in the Program memory only if the address to load 
  *         is the start address of a page (multiple of 1024 bytes).  
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ErasePage(mut Page_Address: uint32_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* If the previous operation is completed, proceed to erase the page */
        let ref mut fresh5 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2000
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut FLASH_TypeDef)).AR as
                                        *mut uint32_t, Page_Address);
        let ref mut fresh6 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        /* Disable the PER Bit */
        let ref mut fresh7 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    /* Return the Erase Status */
    return status;
}
/* *
  * @brief  Erases all FLASH pages.
  * @note   To correctly run this function, the FLASH_Unlock() function
  *         must be called before.
  *         all the FLASH_Lock() to disable the flash memory access 
  *         (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_EraseAllPages() -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* if the previous operation is completed, proceed to erase all pages */
        let ref mut fresh8 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        let ref mut fresh9 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        /* Disable the MER Bit */
        let ref mut fresh10 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    /* Return the Erase Status */
    return status;
}
/* *
  * @brief  Programs a word at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function
  *         must be called before.
  *         Call the FLASH_Lock() to disable the flash memory access 
  *         (recommended to protect the FLASH memory against possible unwanted operation)  
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ProgramWord(mut Address: uint32_t,
                                           mut Data: uint32_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* If the previous operation is completed, proceed to program the new first 
    half word */
        let ref mut fresh11 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(Address as *mut uint16_t,
                                    Data as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            /* If the previous operation is completed, proceed to program the new second 
      half word */
            ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                        Address.wrapping_add(2 as libc::c_int
                                                                 as
                                                                 libc::c_uint));
            ::core::ptr::write_volatile(tmp as *mut uint16_t,
                                        (Data >> 16 as libc::c_int) as
                                            uint16_t);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0xb0000 as libc::c_int as
                                               uint32_t);
            /* Disable the PG Bit */
            let ref mut fresh12 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh12,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x1 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        } else {
            /* Disable the PG Bit */
            let ref mut fresh13 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh13,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x1 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Program Status */
    return status;
}
/* *
  * @brief  Programs a half word at a specified address.
  * @note   To correctly run this function, the FLASH_Unlock() function
  *         must be called before.
  *         Call the FLASH_Lock() to disable the flash memory access 
  *         (recommended to protect the FLASH memory against possible unwanted operation) 
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ProgramHalfWord(mut Address: uint32_t,
                                               mut Data: uint16_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* If the previous operation is completed, proceed to program the new data */
        let ref mut fresh14 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh14,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(Address as *mut uint16_t, Data);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        /* Disable the PG Bit */
        let ref mut fresh15 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh15,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    /* Return the Program Status */
    return status;
}
/* *
  * @}
  */
/* * @defgroup FLASH_Group3 Option Bytes Programming functions
 *  @brief   Option Bytes Programming functions 
 *
@verbatim   
 ===============================================================================
                ##### Option Bytes Programming functions #####
 ===============================================================================  
    [..] This group includes the following functions:
         (+) void FLASH_OB_Unlock(void);
         (+) void FLASH_OB_Lock(void);
         (+) void FLASH_OB_Erase(void);
         (+) FLASH_Status FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState);
         (+) FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP);
         (+) FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY);
         (+) FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1);
         (+) FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG);
         (+) FLASH_Status FLASH_OB_SRMParityConfig(uint8_t OB_SRAM_Parity);
         (+) FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER);					
         (+) FLASH_Status FLASH_OB_Launch(void);
         (+) uint32_t FLASH_OB_GetUser(void);						
         (+) uint8_t FLASH_OB_GetWRP(void);						
         (+) uint8_t FLASH_OB_GetRDP(void);							
    [..] Any operation of erase or program should follow these steps:
         (#) Call the FLASH_OB_Unlock() function to enable the FLASH option control 
             register access.
         (#) Call one or several functions to program the desired Option Bytes:
             (++) void FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState); 
                  => to Enable/Disable the desired sector write protection.
             (++) FLASH_Status FLASH_OB_RDPConfig(uint8_t OB_RDP) => to set the 
                  desired read Protection Level.
             (++) FLASH_Status FLASH_OB_UserConfig(uint8_t OB_IWDG, uint8_t OB_STOP, uint8_t OB_STDBY); 
                  => to configure the user Option Bytes.
 	         (++) FLASH_Status FLASH_OB_BOOTConfig(uint8_t OB_BOOT1); 
                  => to set the boot1 mode
             (++) FLASH_Status FLASH_OB_VDDAConfig(uint8_t OB_VDDA_ANALOG); 
                  => to Enable/Disable the VDDA monotoring.
             (++) FLASH_Status FLASH_OB_SRMParityConfig(uint8_t OB_SRAM_Parity); 
                  => to Enable/Disable the SRAM Parity check.		 
	         (++) FLASH_Status FLASH_OB_WriteUser(uint8_t OB_USER); 
                  => to write all user option bytes: OB_IWDG, OB_STOP, OB_STDBY, 
                     OB_BOOT1, OB_VDDA_ANALOG and OB_VDD_SD12.  
         (#) Once all needed Option Bytes to be programmed are correctly written, 
             call the FLASH_OB_Launch() function to launch the Option Bytes 
             programming process.
         (#@) When changing the IWDG mode from HW to SW or from SW to HW, a system 
              reset is needed to make the change effective.  
         (#) Call the FLASH_OB_Lock() function to disable the FLASH option control 
             register access (recommended to protect the Option Bytes against 
             possible unwanted operations).
    
@endverbatim
  * @{
  */
/* *
  * @brief  Unlocks the option bytes block access.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_Unlock() {
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).CR & 0x200 as libc::c_int as uint32_t ==
           RESET as libc::c_int as libc::c_uint {
        /* Unlocking the option bytes block access */
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2000
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as
                                                *mut FLASH_TypeDef)).OPTKEYR
                                        as *mut uint32_t,
                                    0x45670123 as libc::c_int as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x2000
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as
                                                *mut FLASH_TypeDef)).OPTKEYR
                                        as *mut uint32_t,
                                    0xcdef89ab as libc::c_uint)
    };
}
/* *
  * @brief  Locks the option bytes block access.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_Lock() {
    /* Set the OPTWREN Bit to lock the option bytes block access */
    let ref mut fresh16 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x200 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Launch the option byte loading.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_Launch() {
    /* Set the OBL_Launch bit to launch the option byte loading */
    let ref mut fresh17 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x2000 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Erases the FLASH option bytes.
  * @note   This functions erases all option bytes except the Read protection (RDP). 
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_Erase() -> FLASH_Status {
    let mut rdptmp: uint16_t = 0xaa as libc::c_int as uint8_t as uint16_t;
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Get the actual read protection Option Byte value */
    if FLASH_OB_GetRDP() as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        rdptmp = 0 as libc::c_int as uint16_t
    }
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* If the previous operation is completed, proceed to erase the option bytes */
        let ref mut fresh18 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh18,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        let ref mut fresh19 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh19,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            /* If the erase operation is completed, disable the OPTER Bit */
            let ref mut fresh20 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh20,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t);
            /* Enable the Option Bytes Programming operation */
            let ref mut fresh21 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh21,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x10 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t);
            /* Restore the last read protection Option Byte value */
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).RDP as
                                            *mut uint16_t, rdptmp);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0xb0000 as libc::c_int as
                                               uint32_t);
            if status as libc::c_uint !=
                   FLASH_TIMEOUT as libc::c_int as libc::c_uint {
                /* if the program operation is completed, disable the OPTPG Bit */
                let ref mut fresh22 =
                    (*((0x40000000 as libc::c_int as
                            uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                       libc::c_uint).wrapping_add(0x2000
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                           as *mut FLASH_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh22,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x10 as libc::c_int as
                                                       uint32_t)) as uint32_t
                                                as uint32_t)
            }
        } else if status as libc::c_uint !=
                      FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* Disable the OPTPG Bit */
            let ref mut fresh23 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh23,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the erase status */
    return status;
}
/* *
  * @brief  Write protects the desired pages
  * @note   To correctly run this function, the FLASH_OB_Unlock() function
  *         must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option bytes 
  *         (recommended to protect the FLASH memory against possible unwanted operation)    
  * @param  OB_WRP: specifies the address of the pages to be write protected.
  *   This parameter can be:
  *     @arg  value between OB_WRP_Pages0to35 and OB_WRP_Pages60to63
  *     @arg OB_WRP_AllPages
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_EnableWRP(mut OB_WRP: uint32_t)
 -> FLASH_Status {
    let mut WRP0_Data: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut WRP1_Data: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    OB_WRP = !OB_WRP;
    WRP0_Data = (OB_WRP & 0xff as libc::c_int as uint32_t) as uint16_t;
    WRP1_Data =
        ((OB_WRP & 0xff00 as libc::c_int as uint32_t) >> 8 as libc::c_int) as
            uint16_t;
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        let ref mut fresh24 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh24,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        if WRP0_Data as libc::c_int != 0xff as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).WRP0 as
                                            *mut uint16_t, WRP0_Data);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t)
        }
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint &&
               WRP1_Data as libc::c_int != 0xff as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).WRP1 as
                                            *mut uint16_t, WRP1_Data);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t)
        }
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh25 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh25,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the write protection operation Status */
    return status;
}
/* *
  * @brief  Enables or disables the read out protection.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function
  *         must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option bytes 
  *         (recommended to protect the FLASH memory against possible unwanted operation)   
  * @param  FLASH_ReadProtection_Level: specifies the read protection level. 
  *   This parameter can be:
  *     @arg OB_RDP_Level_0: No protection
  *     @arg OB_RDP_Level_1: Read protection of the memory                     
  *     @arg OB_RDP_Level_2: Chip protection
  *     @retval FLASH Status: The returned value can be: 
  * FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_RDPConfig(mut OB_RDP: uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        let ref mut fresh26 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh26,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        let ref mut fresh27 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh27,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            /* If the erase operation is completed, disable the OPTER Bit */
            let ref mut fresh28 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh28,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t);
            /* Enable the Option Bytes Programming operation */
            let ref mut fresh29 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh29,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x10 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).RDP as
                                            *mut uint16_t,
                                        OB_RDP as uint16_t);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0xb0000 as libc::c_int as
                                               uint32_t);
            if status as libc::c_uint !=
                   FLASH_TIMEOUT as libc::c_int as libc::c_uint {
                /* if the program operation is completed, disable the OPTPG Bit */
                let ref mut fresh30 =
                    (*((0x40000000 as libc::c_int as
                            uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                       libc::c_uint).wrapping_add(0x2000
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                           as *mut FLASH_TypeDef)).CR;
                ::core::ptr::write_volatile(fresh30,
                                            (::core::ptr::read_volatile::<uint32_t>(fresh30
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x10 as libc::c_int as
                                                       uint32_t)) as uint32_t
                                                as uint32_t)
            }
        } else if status as libc::c_uint !=
                      FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* Disable the OPTER Bit */
            let ref mut fresh31 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh31,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh31
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the protection operation Status */
    return status;
}
/* *
  * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.
  * @param  OB_IWDG: Selects the IWDG mode
  *   This parameter can be one of the following values:
  *     @arg OB_IWDG_SW: Software IWDG selected
  *     @arg OB_IWDG_HW: Hardware IWDG selected
  * @param  OB_STOP: Reset event when entering STOP mode.
  *   This parameter can be one of the following values:
  *     @arg OB_STOP_NoRST: No reset generated when entering in STOP
  *     @arg OB_STOP_RST: Reset generated when entering in STOP
  * @param  OB_STDBY: Reset event when entering Standby mode.
  *   This parameter can be one of the following values:
  *     @arg OB_STDBY_NoRST: No reset generated when entering in STANDBY
  *     @arg OB_STDBY_RST: Reset generated when entering in STANDBY
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG, 
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_UserConfig(mut OB_IWDG: uint8_t,
                                             mut OB_STOP: uint8_t,
                                             mut OB_STDBY: uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    /* Authorize the small information block programming */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0x45670123 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0xcdef89ab as libc::c_uint);
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Enable the Option Bytes Programming operation */
        let ref mut fresh32 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh32,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh32
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                uint32_t as
                                                *mut OB_TypeDef)).USER as
                                        *mut uint16_t,
                                    ((OB_IWDG as libc::c_int |
                                          OB_STOP as libc::c_int) as uint8_t
                                         as libc::c_int |
                                         (OB_STDBY as libc::c_int |
                                              0xf8 as libc::c_int) as uint8_t
                                             as libc::c_int) as uint8_t as
                                        uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh33 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh33,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh33
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
/* *
  * @brief  Sets or resets the BOOT1. 
  * @param  OB_BOOT1: Set or Reset the BOOT1.
  *   This parameter can be one of the following values:
  *     @arg OB_BOOT1_RESET: BOOT1 Reset
  *     @arg OB_BOOT1_SET: BOOT1 Set
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_BOOTConfig(mut OB_BOOT1: uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    /* Authorize the small information block programming */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0x45670123 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0xcdef89ab as libc::c_uint);
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Enable the Option Bytes Programming operation */
        let ref mut fresh34 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh34,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh34
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                uint32_t as
                                                *mut OB_TypeDef)).USER as
                                        *mut uint16_t,
                                    (OB_BOOT1 as libc::c_int |
                                         0xef as libc::c_int) as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh35 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh35,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh35
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
/* *
  * @brief  Sets or resets the analogue monitoring on VDDA Power source. 
  * @param  OB_VDDA_ANALOG: Selects the analog monitoring on VDDA Power source.
  *   This parameter can be one of the following values:
  *     @arg OB_VDDA_ANALOG_ON: Analog monitoring on VDDA Power source ON
  *     @arg OB_VDDA_ANALOG_OFF: Analog monitoring on VDDA Power source OFF
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_VDDAConfig(mut OB_VDDA_ANALOG: uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    /* Authorize the small information block programming */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0x45670123 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0xcdef89ab as libc::c_uint);
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Enable the Option Bytes Programming operation */
        let ref mut fresh36 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh36,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh36
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                uint32_t as
                                                *mut OB_TypeDef)).USER as
                                        *mut uint16_t,
                                    (OB_VDDA_ANALOG as libc::c_int |
                                         0xdf as libc::c_int) as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh37 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh37,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh37
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
/* *
  * @brief  Sets or resets the SRAM partiy.
  * @param  OB_SRAM_Parity: Set or Reset the SRAM partiy enable bit.
  *         This parameter can be one of the following values:
  *             @arg OB_SRAM_PARITY_SET: Set SRAM partiy.
  *             @arg OB_SRAM_PARITY_RESET: Reset SRAM partiy.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_SRAMParityConfig(mut OB_SRAM_Parity:
                                                       uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Enable the Option Bytes Programming operation */
        let ref mut fresh38 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh38,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh38
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                uint32_t as
                                                *mut OB_TypeDef)).USER as
                                        *mut uint16_t,
                                    (OB_SRAM_Parity as libc::c_int |
                                         0xbf as libc::c_int) as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh39 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh39,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh39
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
/* *
  * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY/ BOOT1 and OB_VDDA_ANALOG.
  * @note   To correctly run this function, the FLASH_OB_Unlock() function
  *         must be called before.
  * @note   Call the FLASH_OB_Lock() to disable the flash control register access and the option bytes 
  *         (recommended to protect the FLASH memory against possible unwanted operation)   
  * @param  OB_USER: Selects all user option bytes
  *   This parameter is a combination of the following values:
  *     @arg OB_IWDG_SW / OB_IWDG_HW: Software / Hardware WDG selected
  *     @arg OB_STOP_NoRST / OB_STOP_RST: No reset / Reset generated when entering in STOP
  *     @arg OB_STDBY_NoRST / OB_STDBY_RST: No reset / Reset generated when entering in STANDBY
  *     @arg OB_BOOT1_RESET / OB_BOOT1_SET: BOOT1 Reset / Set
  *     @arg OB_VDDA_ANALOG_ON / OB_VDDA_ANALOG_OFF: Analog monitoring on VDDA Power source ON / OFF
  * @retval FLASH Status: The returned value can be: 
  * FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_WriteUser(mut OB_USER: uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Authorize the small information block programming */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0x45670123 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).OPTKEYR as
                                    *mut uint32_t,
                                0xcdef89ab as libc::c_uint);
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Enable the Option Bytes Programming operation */
        let ref mut fresh40 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh40,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh40
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                uint32_t as
                                                *mut OB_TypeDef)).USER as
                                        *mut uint16_t,
                                    (OB_USER as libc::c_int |
                                         0x88 as libc::c_int) as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh41 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh41,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh41
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
/* *
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note    To correctly run this function, the FLASH_OB_Unlock() function
  *           must be called before.
  *          Call the FLASH_OB_Lock() to disable the flash control register access and the option bytes 
  *          (recommended to protect the FLASH memory against possible unwanted operation)
  * @param  Address: specifies the address to be programmed.
  *   This parameter can be 0x1FFFF804 or 0x1FFFF806. 
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ProgramOptionByteData(mut Address: uint32_t,
                                                     mut Data: uint8_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Enables the Option Bytes Programming operation */
        let ref mut fresh42 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh42,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh42
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(Address as *mut uint16_t,
                                    Data as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* If the program operation is completed, disable the OPTPG Bit */
            let ref mut fresh43 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x2000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut FLASH_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh43,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh43
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10 as libc::c_int as
                                                   uint32_t)) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte Data Program Status */
    return status;
}
/* *
  * @brief  Returns the FLASH User Option Bytes values.
  * @param  None
  * @retval The FLASH User Option Bytes .
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_GetUser() -> uint8_t {
    /* Return the User Option Byte */
    return ((*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).OBR >> 8 as libc::c_int) as
               uint8_t;
}
/* *
  * @brief  Returns the FLASH Write Protection Option Bytes value.
  * @param  None
  * @retval The FLASH Write Protection Option Bytes value
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_GetWRP() -> uint32_t {
    /* Return the FLASH write protection Register value */
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x20000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x2000
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut FLASH_TypeDef)).WRPR;
}
/* *
  * @brief  Checks whether the FLASH Read out Protection Status is set or not.
  * @param  None
  * @retval FLASH ReadOut Protection Status(SET or RESET)
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_OB_GetRDP() -> FlagStatus {
    let mut readstatus: FlagStatus = RESET;
    if ((*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut FLASH_TypeDef)).OBR &
            (0x2 as libc::c_int as uint32_t | 0x4 as libc::c_int as uint32_t))
           as uint8_t as libc::c_int != RESET as libc::c_int {
        readstatus = SET
    } else { readstatus = RESET }
    return readstatus;
}
/* *
  * @}
  */
/* * @defgroup FLASH_Group4 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
 ===============================================================================
             ##### Interrupts and flags management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified FLASH interrupts.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or 
  *         disabled.
  *   This parameter can be any combination of the following values:     
  *     @arg FLASH_IT_EOP: FLASH end of programming Interrupt
  *     @arg FLASH_IT_ERR: FLASH Error Interrupt 
  * @retval None 
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ITConfig(mut FLASH_IT: uint32_t,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the interrupt sources */
        let ref mut fresh44 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh44,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh44
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | FLASH_IT) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the interrupt sources */
        let ref mut fresh45 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut FLASH_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh45,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh45
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !FLASH_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @param  FLASH_FLAG: specifies the FLASH flag to check.
  *   This parameter can be one of the following values:
  *     @arg FLASH_FLAG_BSY: FLASH write/erase operations in progress flag 
  *     @arg FLASH_FLAG_PGERR: FLASH Programming error flag flag
  *     @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *     @arg FLASH_FLAG_EOP: FLASH End of Programming flag        
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetFlagStatus(mut FLASH_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).SR & FLASH_FLAG !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the new state of FLASH_FLAG (SET or RESET) */
    return bitstatus;
}
/* *
  * @brief  Clears the FLASH's pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_FLAG_PGERR: FLASH Programming error flag flag
  *     @arg FLASH_FLAG_WRPERR: FLASH Write protected error flag
  *     @arg FLASH_FLAG_EOP: FLASH End of Programming flag                
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ClearFlag(mut FLASH_FLAG: uint32_t) {
    /* Check the parameters */
    /* Clear the flags */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).SR as
                                    *mut uint32_t, FLASH_FLAG);
}
/* *
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: 
  *         FLASH_BUSY, FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP or FLASH_COMPLETE.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetStatus() -> FLASH_Status {
    let mut FLASHstatus: FLASH_Status = FLASH_COMPLETE;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).SR & 0x1 as libc::c_int as uint32_t ==
           0x1 as libc::c_int as uint32_t {
        FLASHstatus = FLASH_BUSY
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR &
                  0x10 as libc::c_int as uint32_t !=
                  0 as libc::c_int as uint32_t {
        FLASHstatus = FLASH_ERROR_WRP
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR &
                  0x4 as libc::c_int as uint32_t !=
                  0 as libc::c_int as uint32_t {
        FLASHstatus = FLASH_ERROR_PROGRAM
    } else { FLASHstatus = FLASH_COMPLETE }
    /* Return the FLASH Status */
    return FLASHstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_flash.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the FLASH 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup FLASH
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief FLASH Status  
  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup FLASH_Exported_Constants
  * @{
  */
/* * @defgroup Flash_Latency 
  * @{
  */
/* !< FLASH Zero Latency cycle */
/* !< FLASH One Latency cycle */
/* !< FLASH Two Latency cycles */
/* *
  * @}
  */
/* * @defgroup FLASH_Interrupts 
  * @{
  */
/* !< End of programming interrupt source */
/* !< Error interrupt source */
/* *
  * @}
  */
/* * @defgroup FLASH_Address 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FLASH_OB_DATA_ADDRESS 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Option_Bytes_Write_Protection 
  * @{
  */
/* Write protection of page 0 to 1 */
/* Write protection of page 2 to 3 */
/* Write protection of page 4 to 5 */
/* Write protection of page 6 to 7 */
/* Write protection of page 8 to 9 */
/* Write protection of page 10 to 11 */
/* Write protection of page 12 to 13 */
/* Write protection of page 14 to 15 */
/* Write protection of page 16 to 17 */
/* Write protection of page 18 to 19 */
/* Write protection of page 20 to 21 */
/* Write protection of page 22 to 23 */
/* Write protection of page 24 to 25 */
/* Write protection of page 26 to 27 */
/* Write protection of page 28 to 29 */
/* Write protection of page 30 to 31 */
/* Write protection of page 32 to 33 */
/* Write protection of page 34 to 35 */
/* Write protection of page 36 to 37 */
/* Write protection of page 38 to 39 */
/* Write protection of page 40 to 41 */
/* Write protection of page 42 to 43 */
/* Write protection of page 44 to 45 */
/* Write protection of page 46 to 47 */
/* Write protection of page 48 to 49 */
/* Write protection of page 50 to 51 */
/* Write protection of page 52 to 53 */
/* Write protection of page 54 to 55 */
/* Write protection of page 56 to 57 */
/* Write protection of page 58 to 59 */
/* Write protection of page 60 to 61 */
/* Write protection of page 62 to 127 */
/* !< Write protection of all Sectors */
/* *
  * @}
  */
/* * @defgroup Option_Bytes_Read_Protection 
  * @{
  */
/* * 
  * @brief  Read Protection Level  
  */
/*#define OB_RDP_Level_2   ((uint8_t)0xCC)*/
/* Warning: When enabling read protection level 2 
                                                it's no more possible to go back to level 1 or 0 */
/*||\
                          ((LEVEL) == OB_RDP_Level_2))*/
/* *
  * @}
  */
/* * @defgroup Option_Bytes_IWatchdog 
  * @{
  */
/* !< Software IWDG selected */
/* !< Hardware IWDG selected */
/* *
  * @}
  */
/* * @defgroup Option_Bytes_nRST_STOP 
  * @{
  */
/* !< No reset generated when entering in STOP */
/* !< Reset generated when entering in STOP */
/* *
  * @}
  */
/* * @defgroup Option_Bytes_nRST_STDBY 
  * @{
  */
/* !< No reset generated when entering in STANDBY */
/* !< Reset generated when entering in STANDBY */
/* *
  * @}
  */
/* * @defgroup Option_Bytes_BOOT1
  * @{
  */
/* !< BOOT1 Reset */
/* !< BOOT1 Set */
/* *
  * @}
  */  
/* * @defgroup Option_Bytes_VDDA_Analog_Monitoring
  * @{
  */
/* !< Analog monitoring on VDDA Power source ON */
/* !< Analog monitoring on VDDA Power source OFF */
/* *
  * @}
  */
/* * @defgroup FLASH_Option_Bytes_SRAM_Parity_Enable 
  * @{
  */
/* !< SRAM parity enable Set */
/* !< SRAM parity enable reset */
/* *
  * @}
  */
/* * @defgroup FLASH_Flags 
  * @{
  */
/* !< FLASH Busy flag */
/* !< FLASH Programming error flag */
/* !< FLASH Write protected error flag */
/* !< FLASH End of Programming flag */
/* *
  * @}
  */ 
/* * @defgroup Timeout_definition 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* FLASH Interface configuration functions ************************************/
/* FLASH Memory Programming functions *****************************************/
/* Option Bytes Programming functions *****************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Waits for a FLASH operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, 
  *         FLASH_ERROR_PROGRAM, FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_WaitForLastOperation(mut Timeout: uint32_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check for the FLASH Status */
    status = FLASH_GetStatus();
    /* Wait for a FLASH operation to complete or a TIMEOUT to occur */
    while status as libc::c_uint == FLASH_BUSY as libc::c_int as libc::c_uint
              && Timeout != 0 as libc::c_int as libc::c_uint {
        status = FLASH_GetStatus();
        Timeout = Timeout.wrapping_sub(1)
    }
    if Timeout == 0 as libc::c_int as libc::c_uint { status = FLASH_TIMEOUT }
    /* Return the operation status */
    return status;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
