use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_APB1PeriphResetCmd(RCC_APB1Periph: uint32_t,
                              NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SCB_Type {
    pub CPUID: uint32_t,
    pub ICSR: uint32_t,
    pub VTOR: uint32_t,
    pub AIRCR: uint32_t,
    pub SCR: uint32_t,
    pub CCR: uint32_t,
    pub SHP: [uint8_t; 12],
    pub SHCSR: uint32_t,
    pub CFSR: uint32_t,
    pub HFSR: uint32_t,
    pub DFSR: uint32_t,
    pub MMFAR: uint32_t,
    pub BFAR: uint32_t,
    pub AFSR: uint32_t,
    pub PFR: [uint32_t; 2],
    pub DFR: uint32_t,
    pub ADR: uint32_t,
    pub MMFR: [uint32_t; 4],
    pub ISAR: [uint32_t; 5],
    pub RESERVED0: [uint32_t; 5],
    pub CPACR: uint32_t,
}
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief Power Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct PWR_TypeDef {
    pub CR: uint32_t,
    pub CSR: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup PWR_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the PWR peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_DeInit() {
    RCC_APB1PeriphResetCmd(0x10000000 as libc::c_int as uint32_t, ENABLE);
    RCC_APB1PeriphResetCmd(0x10000000 as libc::c_int as uint32_t, DISABLE);
}
/* *
  * @brief  Enables or disables access to the RTC and backup registers.
  * @param  NewState: new state of the access to the RTC and backup registers.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_BackupAccessCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x7000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x8
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Enables or disables the Power Voltage Detector(PVD).
  * @param  NewState: new state of the PVD.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_PVDCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x7000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x4
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Configures the voltage threshold detected by the Power Voltage Detector(PVD).
  * @param  PWR_PVDLevel: specifies the PVD detection level
  *   This parameter can be one of the following values:
  *     @arg PWR_PVDLevel_2V2: PVD detection level set to 2.2V
  *     @arg PWR_PVDLevel_2V3: PVD detection level set to 2.3V
  *     @arg PWR_PVDLevel_2V4: PVD detection level set to 2.4V
  *     @arg PWR_PVDLevel_2V5: PVD detection level set to 2.5V
  *     @arg PWR_PVDLevel_2V6: PVD detection level set to 2.6V
  *     @arg PWR_PVDLevel_2V7: PVD detection level set to 2.7V
  *     @arg PWR_PVDLevel_2V8: PVD detection level set to 2.8V
  *     @arg PWR_PVDLevel_2V9: PVD detection level set to 2.9V
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_PVDLevelConfig(mut PWR_PVDLevel: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
               as *mut PWR_TypeDef)).CR;
    /* Clear PLS[7:5] bits */
    tmpreg &= 0xffffff1f as libc::c_uint;
    /* Set PLS[7:5] bits according to PWR_PVDLevel value */
    tmpreg |= PWR_PVDLevel;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x7000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut PWR_TypeDef)).CR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the WakeUp Pin functionality.
  * @param  NewState: new state of the WakeUp Pin functionality.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_WakeUpPinCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x7000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x4
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x8
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Enters STOP mode.
  * @param  PWR_Regulator: specifies the regulator state in STOP mode.
  *   This parameter can be one of the following values:
  *     @arg PWR_Regulator_ON: STOP mode with regulator ON
  *     @arg PWR_Regulator_LowPower: STOP mode with regulator in low power mode
  * @param  PWR_STOPEntry: specifies if STOP mode in entered with WFI or WFE instruction.
  *   This parameter can be one of the following values:
  *     @arg PWR_STOPEntry_WFI: enter STOP mode with WFI instruction
  *     @arg PWR_STOPEntry_WFE: enter STOP mode with WFE instruction
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_EnterSTOPMode(mut PWR_Regulator: uint32_t,
                                           mut PWR_STOPEntry: uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Select the regulator state in STOP mode ---------------------------------*/
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
               as *mut PWR_TypeDef)).CR;
    /* Clear PDDS and LPDS bits */
    tmpreg &= 0xfffffffc as libc::c_uint;
    /* Set LPDS bit according to PWR_Regulator value */
    tmpreg |= PWR_Regulator;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x7000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut PWR_TypeDef)).CR as
                                    *mut uint32_t, tmpreg);
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh0 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x4 as libc::c_int as uint8_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select STOP mode entry --------------------------------------------------*/
    if PWR_STOPEntry as libc::c_int ==
           0x1 as libc::c_int as uint8_t as libc::c_int {
        /* Request Wait For Interrupt */
        asm!("wfi" : : : : "volatile")
    } else {
        /* Request Wait For Event */
        asm!("wfe" : : : : "volatile")
    }
    /* Reset SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh1 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x4 as libc::c_int as uint8_t as
                                           uint32_t)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enters STANDBY mode.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_EnterSTANDBYMode() {
    /* Clear Wake-up flag */
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
               as *mut PWR_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x4 as libc::c_int as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select STANDBY mode */
    let ref mut fresh3 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
               as *mut PWR_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x2 as libc::c_int as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    let ref mut fresh4 =
        (*((0xe000e000 as libc::c_ulong).wrapping_add(0xd00 as libc::c_ulong)
               as *mut SCB_Type)).SCR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x4 as libc::c_int as uint8_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* This option is used to ensure that store operations are completed */
    /* Request Wait For Interrupt */
    asm!("wfi" : : : : "volatile");
}
/* *
  * @brief  Checks whether the specified PWR flag is set or not.
  * @param  PWR_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg PWR_FLAG_WU: Wake Up flag
  *     @arg PWR_FLAG_SB: StandBy flag
  *     @arg PWR_FLAG_PVDO: PVD Output
  * @retval The new state of PWR_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_GetFlagStatus(mut PWR_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
              as *mut PWR_TypeDef)).CSR & PWR_FLAG !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the flag status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_pwr.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the PWR firmware 
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
/* * @addtogroup PWR
  * @{
  */
/* * @defgroup PWR_Exported_Types
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Exported_Constants
  * @{
  */
/* * @defgroup PVD_detection_level 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Regulator_state_is_STOP_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup STOP_mode_entry 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Flag 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup PWR_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PWR_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the PWR's pending flags.
  * @param  PWR_FLAG: specifies the flag to clear.
  *   This parameter can be one of the following values:
  *     @arg PWR_FLAG_WU: Wake Up flag
  *     @arg PWR_FLAG_SB: StandBy flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn PWR_ClearFlag(mut PWR_FLAG: uint32_t) {
    /* Check the parameters */
    let ref mut fresh5 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7000 as libc::c_int as libc::c_uint)
               as *mut PWR_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     PWR_FLAG << 2 as libc::c_int) as uint32_t
                                    as uint32_t);
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
