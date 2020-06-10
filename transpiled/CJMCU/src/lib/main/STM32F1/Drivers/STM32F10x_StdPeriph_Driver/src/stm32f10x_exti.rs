use ::libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EXTI_TypeDef {
    pub IMR: uint32_t,
    pub EMR: uint32_t,
    pub RTSR: uint32_t,
    pub FTSR: uint32_t,
    pub SWIER: uint32_t,
    pub PR: uint32_t,
}
pub type EXTIMode_TypeDef = libc::c_uint;
pub const EXTI_Mode_Event: EXTIMode_TypeDef = 4;
pub const EXTI_Mode_Interrupt: EXTIMode_TypeDef = 0;
pub type EXTITrigger_TypeDef = libc::c_uint;
pub const EXTI_Trigger_Rising_Falling: EXTITrigger_TypeDef = 16;
pub const EXTI_Trigger_Falling: EXTITrigger_TypeDef = 12;
pub const EXTI_Trigger_Rising: EXTITrigger_TypeDef = 8;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct EXTI_InitTypeDef {
    pub EXTI_Line: uint32_t,
    pub EXTI_Mode: EXTIMode_TypeDef,
    pub EXTI_Trigger: EXTITrigger_TypeDef,
    pub EXTI_LineCmd: FunctionalState,
}
/* No interrupt selected */
/* *
  * @}
  */
/* * @defgroup EXTI_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup EXTI_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup EXTI_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup EXTI_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the EXTI peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_DeInit() {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).IMR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).EMR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).RTSR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).FTSR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).PR as
                                    *mut uint32_t,
                                0xfffff as libc::c_int as uint32_t);
}
/* *
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure
  *         that contains the configuration information for the EXTI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_Init(mut EXTI_InitStruct:
                                       *mut EXTI_InitTypeDef) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp =
        (0x40000000 as libc::c_int as
             uint32_t).wrapping_add(0x10000 as libc::c_int as
                                        libc::c_uint).wrapping_add(0x400 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint);
    if (*EXTI_InitStruct).EXTI_LineCmd as libc::c_uint !=
           DISABLE as libc::c_int as libc::c_uint {
        /* Clear EXTI line configuration */
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).IMR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(*EXTI_InitStruct).EXTI_Line) as
                                        uint32_t as uint32_t);
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).EMR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(*EXTI_InitStruct).EXTI_Line) as
                                        uint32_t as uint32_t);
        tmp =
            (tmp as
                 libc::c_uint).wrapping_add((*EXTI_InitStruct).EXTI_Mode as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        let ref mut fresh2 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*EXTI_InitStruct).EXTI_Line) as
                                        uint32_t as uint32_t);
        /* Clear Rising Falling edge configuration */
        let ref mut fresh3 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).RTSR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(*EXTI_InitStruct).EXTI_Line) as
                                        uint32_t as uint32_t);
        let ref mut fresh4 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).FTSR;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(*EXTI_InitStruct).EXTI_Line) as
                                        uint32_t as uint32_t);
        /* Select the trigger for the selected external interrupts */
        if (*EXTI_InitStruct).EXTI_Trigger as libc::c_uint ==
               EXTI_Trigger_Rising_Falling as libc::c_int as libc::c_uint {
            /* Rising Falling edge */
            let ref mut fresh5 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut EXTI_TypeDef)).RTSR;
            ::core::ptr::write_volatile(fresh5,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*EXTI_InitStruct).EXTI_Line) as
                                            uint32_t as uint32_t);
            let ref mut fresh6 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut EXTI_TypeDef)).FTSR;
            ::core::ptr::write_volatile(fresh6,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*EXTI_InitStruct).EXTI_Line) as
                                            uint32_t as uint32_t)
        } else {
            tmp =
                (0x40000000 as libc::c_int as
                     uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                libc::c_uint).wrapping_add(0x400
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint);
            tmp =
                (tmp as
                     libc::c_uint).wrapping_add((*EXTI_InitStruct).EXTI_Trigger
                                                    as libc::c_uint) as
                    uint32_t as uint32_t;
            let ref mut fresh7 = *(tmp as *mut uint32_t);
            ::core::ptr::write_volatile(fresh7,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (*EXTI_InitStruct).EXTI_Line) as
                                            uint32_t as uint32_t)
        }
    } else {
        tmp =
            (tmp as
                 libc::c_uint).wrapping_add((*EXTI_InitStruct).EXTI_Mode as
                                                libc::c_uint) as uint32_t as
                uint32_t;
        /* Disable the selected external lines */
        let ref mut fresh8 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(*EXTI_InitStruct).EXTI_Line) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Fills each EXTI_InitStruct member with its reset value.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_StructInit(mut EXTI_InitStruct:
                                             *mut EXTI_InitTypeDef) {
    (*EXTI_InitStruct).EXTI_Line = 0 as libc::c_int as uint32_t;
    (*EXTI_InitStruct).EXTI_Mode = EXTI_Mode_Interrupt;
    (*EXTI_InitStruct).EXTI_Trigger = EXTI_Trigger_Falling;
    (*EXTI_InitStruct).EXTI_LineCmd = DISABLE;
}
/* *
  * @brief  Generates a Software interrupt.
  * @param  EXTI_Line: specifies the EXTI lines to be enabled or disabled.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_GenerateSWInterrupt(mut EXTI_Line: uint32_t) {
    /* Check the parameters */
    let ref mut fresh9 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x400 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut EXTI_TypeDef)).SWIER;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | EXTI_Line) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Checks whether the specified EXTI line flag is set or not.
  * @param  EXTI_Line: specifies the EXTI line flag to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The new state of EXTI_Line (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_GetFlagStatus(mut EXTI_Line: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x10000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x400 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut EXTI_TypeDef)).PR & EXTI_Line !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the EXTI's line pending flags.
  * @param  EXTI_Line: specifies the EXTI lines flags to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_ClearFlag(mut EXTI_Line: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).PR as
                                    *mut uint32_t, EXTI_Line);
}
/* *
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The new state of EXTI_Line (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_GetITStatus(mut EXTI_Line: uint32_t)
 -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut enablestatus: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    enablestatus =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x400 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut EXTI_TypeDef)).IMR & EXTI_Line;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x10000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x400 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut EXTI_TypeDef)).PR & EXTI_Line !=
           RESET as libc::c_int as uint32_t &&
           enablestatus != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_exti.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the EXTI firmware
  *          library.
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
/* * @addtogroup EXTI
  * @{
  */
/* * @defgroup EXTI_Exported_Types
  * @{
  */
/* * 
  * @brief  EXTI mode enumeration  
  */
/* * 
  * @brief  EXTI Trigger enumeration  
  */
/* * 
  * @brief  EXTI Init Structure definition  
  */
/* !< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination of @ref EXTI_Lines */
/* !< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */
/* !< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */
/* !< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */
/* *
  * @}
  */
/* * @defgroup EXTI_Exported_Constants
  * @{
  */
/* * @defgroup EXTI_Lines 
  * @{
  */
/* !< External interrupt line 0 */
/* !< External interrupt line 1 */
/* !< External interrupt line 2 */
/* !< External interrupt line 3 */
/* !< External interrupt line 4 */
/* !< External interrupt line 5 */
/* !< External interrupt line 6 */
/* !< External interrupt line 7 */
/* !< External interrupt line 8 */
/* !< External interrupt line 9 */
/* !< External interrupt line 10 */
/* !< External interrupt line 11 */
/* !< External interrupt line 12 */
/* !< External interrupt line 13 */
/* !< External interrupt line 14 */
/* !< External interrupt line 15 */
/* !< External interrupt line 16 Connected to the PVD Output */
/* !< External interrupt line 17 Connected to the RTC Alarm event */
/* !< External interrupt line 18 Connected to the USB Device/USB OTG FS
                                                   Wakeup from suspend event */
/* !< External interrupt line 19 Connected to the Ethernet Wakeup event */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup EXTI_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup EXTI_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the EXTI's line pending bits.
  * @param  EXTI_Line: specifies the EXTI lines to clear.
  *   This parameter can be any combination of EXTI_Linex where x can be (0..19).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn EXTI_ClearITPendingBit(mut EXTI_Line: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x400
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut EXTI_TypeDef)).PR as
                                    *mut uint32_t, EXTI_Line);
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
