use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_BackupResetCmd(NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
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
pub struct BKP_TypeDef {
    pub RESERVED0: uint32_t,
    pub DR1: uint16_t,
    pub RESERVED1: uint16_t,
    pub DR2: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR3: uint16_t,
    pub RESERVED3: uint16_t,
    pub DR4: uint16_t,
    pub RESERVED4: uint16_t,
    pub DR5: uint16_t,
    pub RESERVED5: uint16_t,
    pub DR6: uint16_t,
    pub RESERVED6: uint16_t,
    pub DR7: uint16_t,
    pub RESERVED7: uint16_t,
    pub DR8: uint16_t,
    pub RESERVED8: uint16_t,
    pub DR9: uint16_t,
    pub RESERVED9: uint16_t,
    pub DR10: uint16_t,
    pub RESERVED10: uint16_t,
    pub RTCCR: uint16_t,
    pub RESERVED11: uint16_t,
    pub CR: uint16_t,
    pub RESERVED12: uint16_t,
    pub CSR: uint16_t,
    pub RESERVED13: [uint16_t; 5],
    pub DR11: uint16_t,
    pub RESERVED14: uint16_t,
    pub DR12: uint16_t,
    pub RESERVED15: uint16_t,
    pub DR13: uint16_t,
    pub RESERVED16: uint16_t,
    pub DR14: uint16_t,
    pub RESERVED17: uint16_t,
    pub DR15: uint16_t,
    pub RESERVED18: uint16_t,
    pub DR16: uint16_t,
    pub RESERVED19: uint16_t,
    pub DR17: uint16_t,
    pub RESERVED20: uint16_t,
    pub DR18: uint16_t,
    pub RESERVED21: uint16_t,
    pub DR19: uint16_t,
    pub RESERVED22: uint16_t,
    pub DR20: uint16_t,
    pub RESERVED23: uint16_t,
    pub DR21: uint16_t,
    pub RESERVED24: uint16_t,
    pub DR22: uint16_t,
    pub RESERVED25: uint16_t,
    pub DR23: uint16_t,
    pub RESERVED26: uint16_t,
    pub DR24: uint16_t,
    pub RESERVED27: uint16_t,
    pub DR25: uint16_t,
    pub RESERVED28: uint16_t,
    pub DR26: uint16_t,
    pub RESERVED29: uint16_t,
    pub DR27: uint16_t,
    pub RESERVED30: uint16_t,
    pub DR28: uint16_t,
    pub RESERVED31: uint16_t,
    pub DR29: uint16_t,
    pub RESERVED32: uint16_t,
    pub DR30: uint16_t,
    pub RESERVED33: uint16_t,
    pub DR31: uint16_t,
    pub RESERVED34: uint16_t,
    pub DR32: uint16_t,
    pub RESERVED35: uint16_t,
    pub DR33: uint16_t,
    pub RESERVED36: uint16_t,
    pub DR34: uint16_t,
    pub RESERVED37: uint16_t,
    pub DR35: uint16_t,
    pub RESERVED38: uint16_t,
    pub DR36: uint16_t,
    pub RESERVED39: uint16_t,
    pub DR37: uint16_t,
    pub RESERVED40: uint16_t,
    pub DR38: uint16_t,
    pub RESERVED41: uint16_t,
    pub DR39: uint16_t,
    pub RESERVED42: uint16_t,
    pub DR40: uint16_t,
    pub RESERVED43: uint16_t,
    pub DR41: uint16_t,
    pub RESERVED44: uint16_t,
    pub DR42: uint16_t,
    pub RESERVED45: uint16_t,
}
/* *
  * @}
  */
/* * @defgroup BKP_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup BKP_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup BKP_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup BKP_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the BKP peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_DeInit() {
    RCC_BackupResetCmd(ENABLE);
    RCC_BackupResetCmd(DISABLE);
}
/* *
  * @brief  Configures the Tamper Pin active level.
  * @param  BKP_TamperPinLevel: specifies the Tamper Pin active level.
  *   This parameter can be one of the following values:
  *     @arg BKP_TamperPinLevel_High: Tamper pin active on high level
  *     @arg BKP_TamperPinLevel_Low: Tamper pin active on low level
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_TamperPinLevelConfig(mut BKP_TamperPinLevel:
                                                      uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x6c00
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x30
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x1
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t,
                                BKP_TamperPinLevel as uint32_t);
}
/* *
  * @brief  Enables or disables the Tamper Pin activation.
  * @param  NewState: new state of the Tamper Pin activation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_TamperPinCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x6c00
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x30
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0
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
  * @brief  Enables or disables the Tamper Pin Interrupt.
  * @param  NewState: new state of the Tamper Pin Interrupt.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_ITConfig(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x6c00
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_sub(0x40000000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           uint32_t).wrapping_add(0x34
                                                                                                                                                      as
                                                                                                                                                      libc::c_int
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x2
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
  * @brief  Select the RTC output source to output on the Tamper pin.
  * @param  BKP_RTCOutputSource: specifies the RTC output source.
  *   This parameter can be one of the following values:
  *     @arg BKP_RTCOutputSource_None: no RTC output on the Tamper pin.
  *     @arg BKP_RTCOutputSource_CalibClock: output the RTC clock with frequency
  *                                          divided by 64 on the Tamper pin.
  *     @arg BKP_RTCOutputSource_Alarm: output the RTC Alarm pulse signal on
  *                                     the Tamper pin.
  *     @arg BKP_RTCOutputSource_Second: output the RTC Second pulse signal on
  *                                      the Tamper pin.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_RTCOutputConfig(mut BKP_RTCOutputSource:
                                                 uint16_t) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x6c00 as libc::c_int as libc::c_uint)
               as *mut BKP_TypeDef)).RTCCR;
    /* Clear CCO, ASOE and ASOS bits */
    tmpreg =
        (tmpreg as libc::c_int &
             0xfc7f as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    /* Set CCO, ASOE and ASOS bits according to BKP_RTCOutputSource value */
    tmpreg =
        (tmpreg as libc::c_int | BKP_RTCOutputSource as libc::c_int) as
            uint16_t;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x6c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut BKP_TypeDef)).RTCCR as
                                    *mut uint16_t, tmpreg);
}
/* *
  * @brief  Sets RTC Clock Calibration value.
  * @param  CalibrationValue: specifies the RTC Clock Calibration value.
  *   This parameter must be a number between 0 and 0x7F.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_SetRTCCalibrationValue(mut CalibrationValue:
                                                        uint8_t) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x6c00 as libc::c_int as libc::c_uint)
               as *mut BKP_TypeDef)).RTCCR;
    /* Clear CAL[6:0] bits */
    tmpreg =
        (tmpreg as libc::c_int &
             0xff80 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    /* Set CAL[6:0] bits according to CalibrationValue value */
    tmpreg =
        (tmpreg as libc::c_int | CalibrationValue as libc::c_int) as uint16_t;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x6c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut BKP_TypeDef)).RTCCR as
                                    *mut uint16_t, tmpreg);
}
/* *
  * @brief  Writes user data to the specified Data Backup Register.
  * @param  BKP_DR: specifies the Data Backup Register.
  *   This parameter can be BKP_DRx where x:[1, 42]
  * @param  Data: data to write
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_WriteBackupRegister(mut BKP_DR: uint16_t,
                                                 mut Data: uint16_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x6c00 as
                                                                libc::c_int as
                                                                libc::c_uint));
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(BKP_DR as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(tmp as *mut uint32_t, Data as uint32_t);
}
/* *
  * @brief  Reads data from the specified Data Backup Register.
  * @param  BKP_DR: specifies the Data Backup Register.
  *   This parameter can be BKP_DRx where x:[1, 42]
  * @retval The content of the specified Data Backup Register
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_ReadBackupRegister(mut BKP_DR: uint16_t)
 -> uint16_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x6c00 as
                                                                libc::c_int as
                                                                libc::c_uint));
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(BKP_DR as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t);
    return *(tmp as *mut uint16_t);
}
/* *
  * @brief  Checks whether the Tamper Pin Event flag is set or not.
  * @param  None
  * @retval The new state of the Tamper Pin Event flag (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_GetFlagStatus() -> FlagStatus {
    return *((0x42000000 as libc::c_int as
                  uint32_t).wrapping_add((0x40000000 as libc::c_int as
                                              uint32_t).wrapping_add(0x6c00 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x40000000
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        uint32_t).wrapping_add(0x34
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
                 as *mut uint32_t) as FlagStatus;
}
/* *
  * @brief  Clears Tamper Pin Event pending flag.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_ClearFlag() {
    /* Set CTE bit to clear Tamper Pin Event flag */
    let ref mut fresh0 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x6c00 as libc::c_int as libc::c_uint)
               as *mut BKP_TypeDef)).CSR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint16_t>(fresh0
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x1 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Checks whether the Tamper Pin Interrupt has occurred or not.
  * @param  None
  * @retval The new state of the Tamper Pin Interrupt (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_GetITStatus() -> ITStatus {
    return *((0x42000000 as libc::c_int as
                  uint32_t).wrapping_add((0x40000000 as libc::c_int as
                                              uint32_t).wrapping_add(0x6c00 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_sub(0x40000000
                                                                                                        as
                                                                                                        libc::c_int
                                                                                                        as
                                                                                                        uint32_t).wrapping_add(0x34
                                                                                                                                   as
                                                                                                                                   libc::c_int
                                                                                                                                   as
                                                                                                                                   libc::c_uint).wrapping_mul(32
                                                                                                                                                                  as
                                                                                                                                                                  libc::c_int
                                                                                                                                                                  as
                                                                                                                                                                  libc::c_uint)).wrapping_add((0x9
                                                                                                                                                                                                   as
                                                                                                                                                                                                   libc::c_int
                                                                                                                                                                                                   *
                                                                                                                                                                                                   4
                                                                                                                                                                                                       as
                                                                                                                                                                                                       libc::c_int)
                                                                                                                                                                                                  as
                                                                                                                                                                                                  libc::c_uint)
                 as *mut uint32_t) as ITStatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_bkp.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the BKP firmware 
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
/* * @addtogroup BKP
  * @{
  */
/* * @defgroup BKP_Exported_Types
  * @{
  */
/* *
  * @}
  */
/* * @defgroup BKP_Exported_Constants
  * @{
  */
/* * @defgroup Tamper_Pin_active_level 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_output_source_to_output_on_the_Tamper_pin 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Data_Backup_Register 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup BKP_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup BKP_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears Tamper Pin Interrupt pending bit.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn BKP_ClearITPendingBit() {
    /* Set CTI bit to clear Tamper Pin Interrupt pending bit */
    let ref mut fresh1 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x6c00 as libc::c_int as libc::c_uint)
               as *mut BKP_TypeDef)).CSR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint16_t>(fresh1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x2 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
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
