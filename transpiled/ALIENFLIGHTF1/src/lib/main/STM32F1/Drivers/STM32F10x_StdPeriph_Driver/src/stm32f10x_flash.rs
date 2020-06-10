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
    pub Data0: uint16_t,
    pub Data1: uint16_t,
    pub WRP0: uint16_t,
    pub WRP1: uint16_t,
    pub WRP2: uint16_t,
    pub WRP3: uint16_t,
}
pub type FLASH_Status = libc::c_uint;
pub const FLASH_TIMEOUT: FLASH_Status = 5;
pub const FLASH_COMPLETE: FLASH_Status = 4;
pub const FLASH_ERROR_WRP: FLASH_Status = 3;
pub const FLASH_ERROR_PG: FLASH_Status = 2;
pub const FLASH_BUSY: FLASH_Status = 1;
/* *
  * @}
  */
/* * @defgroup FLASH_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FLASH_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FLASH_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FLASH_Private_Functions
  * @{
  */
/* *
@code  
 
 This driver provides functions to configure and program the Flash memory of all STM32F10x devices,
 including the latest STM32F10x_XL density devices. 

 STM32F10x_XL devices feature up to 1 Mbyte with dual bank architecture for read-while-write (RWW) capability:
    - bank1: fixed size of 512 Kbytes (256 pages of 2Kbytes each)
    - bank2: up to 512 Kbytes (up to 256 pages of 2Kbytes each)
 While other STM32F10x devices features only one bank with memory up to 512 Kbytes.

 In version V3.3.0, some functions were updated and new ones were added to support
 STM32F10x_XL devices. Thus some functions manages all devices, while other are 
 dedicated for XL devices only.
 
 The table below presents the list of available functions depending on the used STM32F10x devices.  
      
   ***************************************************
   * Legacy functions used for all STM32F10x devices *
   ***************************************************
   +----------------------------------------------------------------------------------------------------------------------------------+
   |       Functions prototypes         |STM32F10x_XL|Other STM32F10x|    Comments                                                    |
   |                                    |   devices  |  devices      |                                                                |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_SetLatency                    |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_HalfCycleAccessCmd            |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_PrefetchBufferCmd             |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_Unlock                        |    Yes     |      Yes      | - For STM32F10X_XL devices: unlock Bank1 and Bank2.            |
   |                                    |            |               | - For other devices: unlock Bank1 and it is equivalent         |
   |                                    |            |               |   to FLASH_UnlockBank1 function.                               |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_Lock                          |    Yes     |      Yes      | - For STM32F10X_XL devices: lock Bank1 and Bank2.              |
   |                                    |            |               | - For other devices: lock Bank1 and it is equivalent           |
   |                                    |            |               |   to FLASH_LockBank1 function.                                 |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ErasePage                     |    Yes     |      Yes      | - For STM32F10x_XL devices: erase a page in Bank1 and Bank2    |
   |                                    |            |               | - For other devices: erase a page in Bank1                     |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_EraseAllPages                 |    Yes     |      Yes      | - For STM32F10x_XL devices: erase all pages in Bank1 and Bank2 |
   |                                    |            |               | - For other devices: erase all pages in Bank1                  |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_EraseOptionBytes              |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ProgramWord                   |    Yes     |      Yes      | Updated to program up to 1MByte (depending on the used device) |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ProgramHalfWord               |    Yes     |      Yes      | Updated to program up to 1MByte (depending on the used device) |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ProgramOptionByteData         |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_EnableWriteProtection         |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ReadOutProtection             |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_UserOptionByteConfig          |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_GetUserOptionByte             |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_GetWriteProtectionOptionByte  |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_GetReadOutProtectionStatus    |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_GetPrefetchBufferStatus       |    Yes     |      Yes      | No change                                                      |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ITConfig                      |    Yes     |      Yes      | - For STM32F10x_XL devices: enable Bank1 and Bank2's interrupts|
   |                                    |            |               | - For other devices: enable Bank1's interrupts                 |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_GetFlagStatus                 |    Yes     |      Yes      | - For STM32F10x_XL devices: return Bank1 and Bank2's flag status|
   |                                    |            |               | - For other devices: return Bank1's flag status                |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_ClearFlag                     |    Yes     |      Yes      | - For STM32F10x_XL devices: clear Bank1 and Bank2's flag       |
   |                                    |            |               | - For other devices: clear Bank1's flag                        |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_GetStatus                     |    Yes     |      Yes      | - Return the status of Bank1 (for all devices)                 |
   |                                    |            |               |   equivalent to FLASH_GetBank1Status function                  |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_WaitForLastOperation          |    Yes     |      Yes      | - Wait for Bank1 last operation (for all devices)              |
   |                                    |            |               |   equivalent to: FLASH_WaitForLastBank1Operation function      |
   +----------------------------------------------------------------------------------------------------------------------------------+

   ************************************************************************************************************************
   * New functions used for all STM32F10x devices to manage Bank1:                                                        *
   *   - These functions are mainly useful for STM32F10x_XL density devices, to have separate control for Bank1 and bank2 *
   *   - For other devices, these functions are optional (covered by functions listed above)                              *
   ************************************************************************************************************************
   +----------------------------------------------------------------------------------------------------------------------------------+
   |       Functions prototypes         |STM32F10x_XL|Other STM32F10x|    Comments                                                    |
   |                                    |   devices  |  devices      |                                                                |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_UnlockBank1                  |    Yes     |      Yes      | - Unlock Bank1                                                 |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_LockBank1                     |    Yes     |      Yes      | - Lock Bank1                                                   |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_EraseAllBank1Pages           |    Yes     |      Yes      | - Erase all pages in Bank1                                     |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_GetBank1Status               |    Yes     |      Yes      | - Return the status of Bank1                                   |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_WaitForLastBank1Operation    |    Yes     |      Yes      | - Wait for Bank1 last operation                                |
   +----------------------------------------------------------------------------------------------------------------------------------+

   *****************************************************************************
   * New Functions used only with STM32F10x_XL density devices to manage Bank2 *
   *****************************************************************************
   +----------------------------------------------------------------------------------------------------------------------------------+
   |       Functions prototypes         |STM32F10x_XL|Other STM32F10x|    Comments                                                    |
   |                                    |   devices  |  devices      |                                                                |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_UnlockBank2                  |    Yes     |      No       | - Unlock Bank2                                                 |
   |----------------------------------------------------------------------------------------------------------------------------------|
   |FLASH_LockBank2                     |    Yes     |      No       | - Lock Bank2                                                   |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_EraseAllBank2Pages           |    Yes     |      No       | - Erase all pages in Bank2                                     |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_GetBank2Status               |    Yes     |      No       | - Return the status of Bank2                                   |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_WaitForLastBank2Operation    |    Yes     |      No       | - Wait for Bank2 last operation                                |
   |----------------------------------------------------------------------------------------------------------------------------------|
   | FLASH_BootConfig                   |    Yes     |      No       | - Configure to boot from Bank1 or Bank2                        |
   +----------------------------------------------------------------------------------------------------------------------------------+
@endcode
*/
/* *
  * @brief  Sets the code latency value.
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *   This parameter can be one of the following values:
  *     @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *     @arg FLASH_Latency_1: FLASH One Latency cycle
  *     @arg FLASH_Latency_2: FLASH Two Latency cycles
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
    tmpreg &= 0x38 as libc::c_int as uint32_t;
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
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_HalfCycleAccess: specifies the FLASH Half cycle Access mode.
  *   This parameter can be one of the following values:
  *     @arg FLASH_HalfCycleAccess_Enable: FLASH Half Cycle Enable
  *     @arg FLASH_HalfCycleAccess_Disable: FLASH Half Cycle Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_HalfCycleAccessCmd(mut FLASH_HalfCycleAccess:
                                                      uint32_t) {
    /* Check the parameters */
    /* Enable or disable the Half cycle access */
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
                                     as libc::c_uint &
                                     0xfffffff7 as libc::c_uint) as uint32_t
                                    as uint32_t);
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
                                     as libc::c_uint | FLASH_HalfCycleAccess)
                                    as uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the Prefetch Buffer.
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_PrefetchBuffer: specifies the Prefetch buffer status.
  *   This parameter can be one of the following values:
  *     @arg FLASH_PrefetchBuffer_Enable: FLASH Prefetch Buffer Enable
  *     @arg FLASH_PrefetchBuffer_Disable: FLASH Prefetch Buffer Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_PrefetchBufferCmd(mut FLASH_PrefetchBuffer:
                                                     uint32_t) {
    /* Check the parameters */
    /* Enable or disable the Prefetch Buffer */
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
                                     as libc::c_uint &
                                     0xffffffef as libc::c_uint) as uint32_t
                                    as uint32_t);
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
                                     as libc::c_uint | FLASH_PrefetchBuffer)
                                    as uint32_t as uint32_t);
}
/* *
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function unlocks Bank1 and Bank2.
  *         - For all other devices it unlocks Bank1 and it is equivalent 
  *           to FLASH_UnlockBank1 function.. 
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_Unlock() {
    /* Authorize the FPEC of Bank1 Access */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).KEYR as
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
                                            as *mut FLASH_TypeDef)).KEYR as
                                    *mut uint32_t,
                                0xcdef89ab as libc::c_uint);
    /* STM32F10X_XL */
}
/* *
  * @brief  Unlocks the FLASH Bank1 Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function unlocks Bank1.
  *         - For all other devices it unlocks Bank1 and it is 
  *           equivalent to FLASH_Unlock function.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_UnlockBank1() {
    /* Authorize the FPEC of Bank1 Access */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x2000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut FLASH_TypeDef)).KEYR as
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
                                            as *mut FLASH_TypeDef)).KEYR as
                                    *mut uint32_t,
                                0xcdef89ab as libc::c_uint);
}
/* STM32F10X_XL */
/* *
  * @brief  Locks the FLASH Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function Locks Bank1 and Bank2.
  *         - For all other devices it Locks Bank1 and it is equivalent 
  *           to FLASH_LockBank1 function.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_Lock() {
    /* Set the Lock Bit to lock the FPEC and the CR of  Bank1 */
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
    /* STM32F10X_XL */
}
/* *
  * @brief  Locks the FLASH Bank1 Program Erase Controller.
  * @note   this function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function Locks Bank1.
  *         - For all other devices it Locks Bank1 and it is equivalent 
  *           to FLASH_Lock function.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_LockBank1() {
    /* Set the Lock Bit to lock the FPEC and the CR of  Bank1 */
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
                                     0x80 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* STM32F10X_XL */
/* *
  * @brief  Erases a specified FLASH page.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
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
        /* if the previous operation is completed, proceed to erase the page */
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
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        /* Disable the PER Bit */
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
                                         as libc::c_uint &
                                         0x1ffd as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    }
    /* STM32F10X_XL */
    /* Return the Erase Status */
    return status;
}
/* *
  * @brief  Erases all FLASH pages.
  * @note   This function can be used for all STM32F10x devices.
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
                                         0x4 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
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
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        /* Disable the MER Bit */
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
                                         as libc::c_uint &
                                         0x1ffb as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    }
    /* STM32F10X_XL */
    /* Return the Erase Status */
    return status;
}
/* *
  * @brief  Erases all Bank1 FLASH pages.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function erases all Bank1 pages.
  *         - For all other devices it erases all Bank1 pages and it is equivalent 
  *           to FLASH_EraseAllPages function.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_EraseAllBank1Pages() -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Wait for last operation to be completed */
    status =
        FLASH_WaitForLastBank1Operation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* if the previous operation is completed, proceed to erase all pages */
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
                                         as libc::c_uint |
                                         0x4 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
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
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastBank1Operation(0xb0000 as libc::c_int as
                                                uint32_t);
        /* Disable the MER Bit */
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
                                         as libc::c_uint &
                                         0x1ffb as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    }
    /* Return the Erase Status */
    return status;
}
/* STM32F10X_XL */
/* *
  * @brief  Erases the FLASH option bytes.
  * @note   This functions erases all option bytes except the Read protection (RDP). 
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_EraseOptionBytes() -> FLASH_Status {
    let mut rdptmp: uint16_t = 0xa5 as libc::c_int as uint16_t;
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Get the actual read protection Option Byte value */
    if FLASH_GetReadOutProtectionStatus() as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        rdptmp = 0 as libc::c_int as uint16_t
    }
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Authorize the small information block programming */
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
                                    0xcdef89ab as libc::c_uint);
        /* if the previous operation is completed, proceed to erase the option bytes */
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
                                         as libc::c_uint |
                                         0x20 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
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
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            /* if the erase operation is completed, disable the OPTER Bit */
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
                                             as libc::c_uint &
                                             0x1fdf as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t);
            /* Enable the Option Bytes Programming operation */
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
                                             0x10 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t);
            /* Restore the last read protection Option Byte value */
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).RDP as
                                            *mut uint16_t, rdptmp);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
            if status as libc::c_uint !=
                   FLASH_TIMEOUT as libc::c_int as libc::c_uint {
                /* if the program operation is completed, disable the OPTPG Bit */
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
                                                 as libc::c_uint &
                                                 0x1fef as libc::c_int as
                                                     uint32_t) as uint32_t as
                                                uint32_t)
            }
        } else if status as libc::c_uint !=
                      FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* Disable the OPTPG Bit */
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
                                             0x1fef as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the erase status */
    return status;
}
/* *
  * @brief  Programs a word at a specified address.
  * @note   This function can be used for all STM32F10x devices.
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
    status = FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* if the previous operation is completed, proceed to program the new first 
    half word */
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
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(Address as *mut uint16_t,
                                    Data as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            /* if the previous operation is completed, proceed to program the new second 
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
                FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
            /* Disable the PG Bit */
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
                                             0x1ffe as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        } else {
            /* Disable the PG Bit */
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
                                             0x1ffe as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    }
    /* STM32F10X_XL */
    /* Return the Program Status */
    return status;
}
/* *
  * @brief  Programs a half word at a specified address.
  * @note   This function can be used for all STM32F10x devices.
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
    status = FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* if the previous operation is completed, proceed to program the new data */
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
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(Address as *mut uint16_t, Data);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
        /* Disable the PG Bit */
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
                                         0x1ffe as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    }
    /* STM32F10X_XL */
    /* Return the Program Status */
    return status;
}
/* *
  * @brief  Programs a half word at a specified Option Byte Data address.
  * @note   This function can be used for all STM32F10x devices.
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
    status = FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Authorize the small information block programming */
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
                                    0xcdef89ab as libc::c_uint);
        /* Enables the Option Bytes Programming operation */
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
                                         0x10 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(Address as *mut uint16_t,
                                    Data as uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
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
                                             as libc::c_uint &
                                             0x1fef as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte Data Program Status */
    return status;
}
/* *
  * @brief  Write protects the desired pages
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_Pages: specifies the address of the pages to be write protected.
  *   This parameter can be:
  *     @arg For @b STM32_Low-density_devices: value between FLASH_WRProt_Pages0to3 and FLASH_WRProt_Pages28to31  
  *     @arg For @b STM32_Medium-density_devices: value between FLASH_WRProt_Pages0to3
  *       and FLASH_WRProt_Pages124to127
  *     @arg For @b STM32_High-density_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to255
  *     @arg For @b STM32_Connectivity_line_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to127    
  *     @arg For @b STM32_XL-density_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to511
  *     @arg FLASH_WRProt_AllPages
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_EnableWriteProtection(mut FLASH_Pages:
                                                         uint32_t)
 -> FLASH_Status {
    let mut WRP0_Data: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut WRP1_Data: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut WRP2_Data: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut WRP3_Data: uint16_t = 0xffff as libc::c_int as uint16_t;
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    FLASH_Pages = !FLASH_Pages;
    WRP0_Data = (FLASH_Pages & 0xff as libc::c_int as uint32_t) as uint16_t;
    WRP1_Data =
        ((FLASH_Pages & 0xff00 as libc::c_int as uint32_t) >>
             8 as libc::c_int) as uint16_t;
    WRP2_Data =
        ((FLASH_Pages & 0xff0000 as libc::c_int as uint32_t) >>
             16 as libc::c_int) as uint16_t;
    WRP3_Data =
        ((FLASH_Pages & 0xff000000 as libc::c_uint) >> 24 as libc::c_int) as
            uint16_t;
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Authorizes the small information block programming */
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
                                    0xcdef89ab as libc::c_uint);
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
                FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t)
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
                FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t)
        }
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint &&
               WRP2_Data as libc::c_int != 0xff as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).WRP2 as
                                            *mut uint16_t, WRP2_Data);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t)
        }
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint &&
               WRP3_Data as libc::c_int != 0xff as libc::c_int {
            ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int as
                                                    uint32_t as
                                                    *mut OB_TypeDef)).WRP3 as
                                            *mut uint16_t, WRP3_Data);
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t)
        }
        if status as libc::c_uint !=
               FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* if the program operation is completed, disable the OPTPG Bit */
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
                                             as libc::c_uint &
                                             0x1fef as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the write protection operation Status */
    return status;
}
/* *
  * @brief  Enables or disables the read out protection.
  * @note   If the user has already programmed the other option bytes before calling 
  *   this function, he must re-program them since this function erases all option bytes.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Newstate: new state of the ReadOut Protection.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ReadOutProtection(mut NewState:
                                                     FunctionalState)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check the parameters */
    status = FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
    if status as libc::c_uint == FLASH_COMPLETE as libc::c_int as libc::c_uint
       {
        /* Authorizes the small information block programming */
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
                                    0xcdef89ab as libc::c_uint);
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
                                         as libc::c_uint |
                                         0x20 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
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
                                         as libc::c_uint |
                                         0x40 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0xb0000 as libc::c_int as uint32_t);
        if status as libc::c_uint ==
               FLASH_COMPLETE as libc::c_int as libc::c_uint {
            /* if the erase operation is completed, disable the OPTER Bit */
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
                                             as libc::c_uint &
                                             0x1fdf as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t);
            /* Enable the Option Bytes Programming operation */
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
                                             as libc::c_uint |
                                             0x10 as libc::c_int as uint32_t)
                                            as uint32_t as uint32_t);
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int
                                                        as uint32_t as
                                                        *mut OB_TypeDef)).RDP
                                                as *mut uint16_t,
                                            0 as libc::c_int as uint16_t)
            } else {
                ::core::ptr::write_volatile(&mut (*(0x1ffff800 as libc::c_int
                                                        as uint32_t as
                                                        *mut OB_TypeDef)).RDP
                                                as *mut uint16_t,
                                            0xa5 as libc::c_int as uint16_t)
            }
            /* Wait for last operation to be completed */
            status =
                FLASH_WaitForLastOperation(0xb0000 as libc::c_int as
                                               uint32_t);
            if status as libc::c_uint !=
                   FLASH_TIMEOUT as libc::c_int as libc::c_uint {
                /* if the program operation is completed, disable the OPTPG Bit */
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
                                                 as libc::c_uint &
                                                 0x1fef as libc::c_int as
                                                     uint32_t) as uint32_t as
                                                uint32_t)
            }
        } else if status as libc::c_uint !=
                      FLASH_TIMEOUT as libc::c_int as libc::c_uint {
            /* Disable the OPTER Bit */
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
                                             0x1fdf as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the protection operation Status */
    return status;
}
/* *
  * @brief  Programs the FLASH User Option Byte: IWDG_SW / RST_STOP / RST_STDBY.
  * @note   This function can be used for all STM32F10x devices.
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
pub unsafe extern "C" fn FLASH_UserOptionByteConfig(mut OB_IWDG: uint16_t,
                                                    mut OB_STOP: uint16_t,
                                                    mut OB_STDBY: uint16_t)
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
    status = FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
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
                                    (OB_IWDG as libc::c_int |
                                         (OB_STOP as libc::c_int |
                                              (OB_STDBY as libc::c_int |
                                                   0xf8 as libc::c_int as
                                                       uint16_t as
                                                       libc::c_int) as
                                                  uint16_t as libc::c_int) as
                                             uint16_t as libc::c_int) as
                                        uint16_t);
        /* Wait for last operation to be completed */
        status =
            FLASH_WaitForLastOperation(0x2000 as libc::c_int as uint32_t);
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
                                             0x1fef as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
    }
    /* Return the Option Byte program Status */
    return status;
}
/* STM32F10X_XL */
/* *
  * @brief  Returns the FLASH User Option Bytes values.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval The FLASH User Option Bytes values:IWDG_SW(Bit0), RST_STOP(Bit1)
  *         and RST_STDBY(Bit2).
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetUserOptionByte() -> uint32_t {
    /* Return the User Option Byte */
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x20000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x2000
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut FLASH_TypeDef)).OBR >> 2 as libc::c_int;
}
/* *
  * @brief  Returns the FLASH Write Protection Option Bytes Register value.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval The FLASH Write Protection  Option Bytes Register value
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetWriteProtectionOptionByte() -> uint32_t {
    /* Return the Flash write protection Register value */
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
  * @brief  Checks whether the FLASH Read Out Protection Status is set or not.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH ReadOut Protection Status(SET or RESET)
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetReadOutProtectionStatus() -> FlagStatus {
    let mut readoutstatus: FlagStatus = RESET;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).OBR & 0x2 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        readoutstatus = SET
    } else { readoutstatus = RESET }
    return readoutstatus;
}
/* *
  * @brief  Checks whether the FLASH Prefetch Buffer status is set or not.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH Prefetch Buffer Status (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetPrefetchBufferStatus() -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).ACR & 0x20 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the new state of FLASH Prefetch Buffer Status (SET or RESET) */
    return bitstatus;
}
/* *
  * @brief  Enables or disables the specified FLASH interrupts.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices, enables or disables the specified FLASH interrupts
              for Bank1 and Bank2.
  *         - For other devices it enables or disables the specified FLASH interrupts for Bank1.
  * @param  FLASH_IT: specifies the FLASH interrupt sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg FLASH_IT_ERROR: FLASH Error Interrupt
  *     @arg FLASH_IT_EOP: FLASH end of operation Interrupt
  * @param  NewState: new state of the specified Flash interrupts.
  *   This parameter can be: ENABLE or DISABLE.      
  * @retval None 
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_ITConfig(mut FLASH_IT: uint32_t,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the interrupt sources */
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
                                         as libc::c_uint | FLASH_IT) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the interrupt sources */
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
                                         as libc::c_uint & !FLASH_IT) as
                                        uint32_t as uint32_t)
    };
    /* STM32F10X_XL */
}
/* *
  * @brief  Checks whether the specified FLASH flag is set or not.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices, this function checks whether the specified 
  *           Bank1 or Bank2 flag is set or not.
  *         - For other devices, it checks whether the specified Bank1 flag is 
  *           set or not.
  * @param  FLASH_FLAG: specifies the FLASH flag to check.
  *   This parameter can be one of the following values:
  *     @arg FLASH_FLAG_BSY: FLASH Busy flag           
  *     @arg FLASH_FLAG_PGERR: FLASH Program error flag       
  *     @arg FLASH_FLAG_WRPRTERR: FLASH Write protected error flag      
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag           
  *     @arg FLASH_FLAG_OPTERR:  FLASH Option Byte error flag     
  * @retval The new state of FLASH_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetFlagStatus(mut FLASH_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if FLASH_FLAG == 0x1 as libc::c_int as uint32_t {
        if (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x20000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x2000
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut FLASH_TypeDef)).OBR & 0x1 as libc::c_int as uint32_t
               != RESET as libc::c_int as uint32_t {
            bitstatus = SET
        } else { bitstatus = RESET }
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR & FLASH_FLAG !=
                  RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* STM32F10X_XL */
    /* Return the new state of FLASH_FLAG (SET or RESET) */
    return bitstatus;
}
/* *
  * @brief  Clears the FLASH's pending flags.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices, this function clears Bank1 or Bank2�s pending flags
  *         - For other devices, it clears Bank1�s pending flags.
  * @param  FLASH_FLAG: specifies the FLASH flags to clear.
  *   This parameter can be any combination of the following values:         
  *     @arg FLASH_FLAG_PGERR: FLASH Program error flag       
  *     @arg FLASH_FLAG_WRPRTERR: FLASH Write protected error flag      
  *     @arg FLASH_FLAG_EOP: FLASH End of Operation flag           
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
    /* STM32F10X_XL */
}
/* *
  * @brief  Returns the FLASH Status.
  * @note   This function can be used for all STM32F10x devices, it is equivalent
  *         to FLASH_GetBank1Status function.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP or FLASH_COMPLETE
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetStatus() -> FLASH_Status {
    let mut flashstatus: FLASH_Status = FLASH_COMPLETE;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).SR & 0x1 as libc::c_int as uint32_t ==
           0x1 as libc::c_int as uint32_t {
        flashstatus = FLASH_BUSY
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR &
                  0x4 as libc::c_int as uint32_t !=
                  0 as libc::c_int as libc::c_uint {
        flashstatus = FLASH_ERROR_PG
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR &
                  0x10 as libc::c_int as uint32_t !=
                  0 as libc::c_int as libc::c_uint {
        flashstatus = FLASH_ERROR_WRP
    } else { flashstatus = FLASH_COMPLETE }
    /* Return the Flash Status */
    return flashstatus;
}
/* *
  * @brief  Returns the FLASH Bank1 Status.
  * @note   This function can be used for all STM32F10x devices, it is equivalent
  *         to FLASH_GetStatus function.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP or FLASH_COMPLETE
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_GetBank1Status() -> FLASH_Status {
    let mut flashstatus: FLASH_Status = FLASH_COMPLETE;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x2000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut FLASH_TypeDef)).SR & 0x1 as libc::c_int as uint32_t ==
           0x1 as libc::c_int as uint32_t {
        flashstatus = FLASH_BUSY
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR &
                  0x4 as libc::c_int as uint32_t !=
                  0 as libc::c_int as libc::c_uint {
        flashstatus = FLASH_ERROR_PG
    } else if (*((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x2000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut FLASH_TypeDef)).SR &
                  0x10 as libc::c_int as uint32_t !=
                  0 as libc::c_int as libc::c_uint {
        flashstatus = FLASH_ERROR_WRP
    } else { flashstatus = FLASH_COMPLETE }
    /* Return the Flash Status */
    return flashstatus;
}
/* STM32F10X_XL */
/* *
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @note   This function can be used for all STM32F10x devices, 
  *         it is equivalent to FLASH_WaitForLastBank1Operation.
  *         - For STM32F10X_XL devices this function waits for a Bank1 Flash operation
  *           to complete or a TIMEOUT to occur.
  *         - For all other devices it waits for a Flash operation to complete 
  *           or a TIMEOUT to occur.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_WaitForLastOperation(mut Timeout: uint32_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check for the Flash Status */
    status = FLASH_GetBank1Status();
    /* Wait for a Flash operation to complete or a TIMEOUT to occur */
    while status as libc::c_uint == FLASH_BUSY as libc::c_int as libc::c_uint
              && Timeout != 0 as libc::c_int as libc::c_uint {
        status = FLASH_GetBank1Status();
        Timeout = Timeout.wrapping_sub(1)
    }
    if Timeout == 0 as libc::c_int as libc::c_uint { status = FLASH_TIMEOUT }
    /* Return the operation status */
    return status;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_flash.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the FLASH 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup FLASH
  * @{
  */
/* * @defgroup FLASH_Exported_Types
  * @{
  */
/* * 
  * @brief  FLASH Status  
  */
/* *
  * @}
  */
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
/* * @defgroup Half_Cycle_Enable_Disable 
  * @{
  */
/* !< FLASH Half Cycle Enable */
/* !< FLASH Half Cycle Disable */
/* *
  * @}
  */
/* * @defgroup Prefetch_Buffer_Enable_Disable 
  * @{
  */
/* !< FLASH Prefetch Buffer Enable */
/* !< FLASH Prefetch Buffer Disable */
/* *
  * @}
  */
/* * @defgroup Option_Bytes_Write_Protection 
  * @{
  */
/* Values to be used with STM32 Low and Medium density devices */
/* !< STM32 Low and Medium density devices: Write protection of page 0 to 3 */
/* !< STM32 Low and Medium density devices: Write protection of page 4 to 7 */
/* !< STM32 Low and Medium density devices: Write protection of page 8 to 11 */
/* !< STM32 Low and Medium density devices: Write protection of page 12 to 15 */
/* !< STM32 Low and Medium density devices: Write protection of page 16 to 19 */
/* !< STM32 Low and Medium density devices: Write protection of page 20 to 23 */
/* !< STM32 Low and Medium density devices: Write protection of page 24 to 27 */
/* !< STM32 Low and Medium density devices: Write protection of page 28 to 31 */
/* Values to be used with STM32 Medium-density devices */
/* !< STM32 Medium-density devices: Write protection of page 32 to 35 */
/* !< STM32 Medium-density devices: Write protection of page 36 to 39 */
/* !< STM32 Medium-density devices: Write protection of page 40 to 43 */
/* !< STM32 Medium-density devices: Write protection of page 44 to 47 */
/* !< STM32 Medium-density devices: Write protection of page 48 to 51 */
/* !< STM32 Medium-density devices: Write protection of page 52 to 55 */
/* !< STM32 Medium-density devices: Write protection of page 56 to 59 */
/* !< STM32 Medium-density devices: Write protection of page 60 to 63 */
/* !< STM32 Medium-density devices: Write protection of page 64 to 67 */
/* !< STM32 Medium-density devices: Write protection of page 68 to 71 */
/* !< STM32 Medium-density devices: Write protection of page 72 to 75 */
/* !< STM32 Medium-density devices: Write protection of page 76 to 79 */
/* !< STM32 Medium-density devices: Write protection of page 80 to 83 */
/* !< STM32 Medium-density devices: Write protection of page 84 to 87 */
/* !< STM32 Medium-density devices: Write protection of page 88 to 91 */
/* !< STM32 Medium-density devices: Write protection of page 92 to 95 */
/* !< STM32 Medium-density devices: Write protection of page 96 to 99 */
/* !< STM32 Medium-density devices: Write protection of page 100 to 103 */
/* !< STM32 Medium-density devices: Write protection of page 104 to 107 */
/* !< STM32 Medium-density devices: Write protection of page 108 to 111 */
/* !< STM32 Medium-density devices: Write protection of page 112 to 115 */
/* !< STM32 Medium-density devices: Write protection of page 115 to 119 */
/* !< STM32 Medium-density devices: Write protection of page 120 to 123 */
/* !< STM32 Medium-density devices: Write protection of page 124 to 127 */
/* Values to be used with STM32 High-density and STM32F10X Connectivity line devices */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 0 to 1 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 2 to 3 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 4 to 5 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 6 to 7 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 8 to 9 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 10 to 11 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 12 to 13 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 14 to 15 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 16 to 17 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 18 to 19 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 20 to 21 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 22 to 23 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 24 to 25 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 26 to 27 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 28 to 29 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 30 to 31 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 32 to 33 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 34 to 35 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 36 to 37 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 38 to 39 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 40 to 41 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 42 to 43 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 44 to 45 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 46 to 47 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 48 to 49 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 50 to 51 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 52 to 53 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 54 to 55 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 56 to 57 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 58 to 59 */
/* !< STM32 High-density, XL-density and Connectivity line devices:
                                                                   Write protection of page 60 to 61 */
/* !< STM32 Connectivity line devices: Write protection of page 62 to 127 */
/* !< STM32 Medium-density devices: Write protection of page 62 to 255 */
/* !< STM32 XL-density devices: Write protection of page 62 to 511 */
/* !< Write protection of all Pages */
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
/* * @defgroup FLASH_Interrupts 
  * @{
  */
/* !< FPEC error interrupt source */
/* !< End of FLASH Operation Interrupt source */
/* !< FPEC BANK1 error interrupt source */
/* !< End of FLASH BANK1 Operation Interrupt source */
/* *
  * @}
  */
/* * @defgroup FLASH_Flags 
  * @{
  */
/* !< FLASH Busy flag */
/* !< FLASH End of Operation flag */
/* !< FLASH Program error flag */
/* !< FLASH Write protected error flag */
/* !< FLASH Option Byte error flag */
/* !< FLASH BANK1 Busy flag*/
/* !< FLASH BANK1 End of Operation flag */
/* !< FLASH BANK1 Program error flag */
/* !< FLASH BANK1 Write protected error flag */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup FLASH_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup FLASH_Exported_Functions
  * @{
  */
/*------------ Functions used for all STM32F10x devices -----*/
/*------------ New function used for all STM32F10x devices -----*/
/* *
  * @brief  Waits for a Flash operation on Bank1 to complete or a TIMEOUT to occur.
  * @note   This function can be used for all STM32F10x devices, 
  *         it is equivalent to FLASH_WaitForLastOperation.
  * @param  Timeout: FLASH programming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
#[no_mangle]
pub unsafe extern "C" fn FLASH_WaitForLastBank1Operation(mut Timeout:
                                                             uint32_t)
 -> FLASH_Status {
    let mut status: FLASH_Status = FLASH_COMPLETE;
    /* Check for the Flash Status */
    status = FLASH_GetBank1Status();
    /* Wait for a Flash operation to complete or a TIMEOUT to occur */
    while status as libc::c_uint == 0x1 as libc::c_int as uint32_t &&
              Timeout != 0 as libc::c_int as libc::c_uint {
        status = FLASH_GetBank1Status();
        Timeout = Timeout.wrapping_sub(1)
    }
    if Timeout == 0 as libc::c_int as libc::c_uint { status = FLASH_TIMEOUT }
    /* Return the operation status */
    return status;
}
/* ****************** (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* STM32F10X_XL */
