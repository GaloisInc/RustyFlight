use core;
use libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn EXTI_Init(EXTI_InitStruct: *mut EXTI_InitTypeDef);
    #[no_mangle]
    fn EXTI_ClearITPendingBit(EXTI_Line: uint32_t);
    #[no_mangle]
    fn SYSCFG_DMAChannelRemapConfig(SYSCFG_DMARemap: uint32_t,
                                    NewState: FunctionalState);
    #[no_mangle]
    fn SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOx: uint8_t,
                             EXTI_PinSourcex: uint8_t);
    #[no_mangle]
    fn RCC_APB2PeriphClockCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    #[no_mangle]
    fn IO_GPIOPinIdx(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_EXTI_PortSourceGPIO(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_EXTI_PinSource(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_EXTI_Line(io: IO_t) -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type IRQn = libc::c_int;
pub const FPU_IRQn: IRQn = 81;
pub const USBWakeUp_RMP_IRQn: IRQn = 76;
pub const USB_LP_IRQn: IRQn = 75;
pub const USB_HP_IRQn: IRQn = 74;
pub const COMP7_IRQn: IRQn = 66;
pub const COMP4_5_6_IRQn: IRQn = 65;
pub const COMP1_2_3_IRQn: IRQn = 64;
pub const ADC4_IRQn: IRQn = 61;
pub const DMA2_Channel5_IRQn: IRQn = 60;
pub const DMA2_Channel4_IRQn: IRQn = 59;
pub const DMA2_Channel3_IRQn: IRQn = 58;
pub const DMA2_Channel2_IRQn: IRQn = 57;
pub const DMA2_Channel1_IRQn: IRQn = 56;
pub const TIM7_IRQn: IRQn = 55;
pub const TIM6_DAC_IRQn: IRQn = 54;
pub const UART5_IRQn: IRQn = 53;
pub const UART4_IRQn: IRQn = 52;
pub const SPI3_IRQn: IRQn = 51;
pub const ADC3_IRQn: IRQn = 47;
pub const TIM8_CC_IRQn: IRQn = 46;
pub const TIM8_TRG_COM_IRQn: IRQn = 45;
pub const TIM8_UP_IRQn: IRQn = 44;
pub const TIM8_BRK_IRQn: IRQn = 43;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTC_Alarm_IRQn: IRQn = 41;
pub const EXTI15_10_IRQn: IRQn = 40;
pub const USART3_IRQn: IRQn = 39;
pub const USART2_IRQn: IRQn = 38;
pub const USART1_IRQn: IRQn = 37;
pub const SPI2_IRQn: IRQn = 36;
pub const SPI1_IRQn: IRQn = 35;
pub const I2C2_ER_IRQn: IRQn = 34;
pub const I2C2_EV_IRQn: IRQn = 33;
pub const I2C1_ER_IRQn: IRQn = 32;
pub const I2C1_EV_IRQn: IRQn = 31;
pub const TIM4_IRQn: IRQn = 30;
pub const TIM3_IRQn: IRQn = 29;
pub const TIM2_IRQn: IRQn = 28;
pub const TIM1_CC_IRQn: IRQn = 27;
pub const TIM1_TRG_COM_TIM17_IRQn: IRQn = 26;
pub const TIM1_UP_TIM16_IRQn: IRQn = 25;
pub const TIM1_BRK_TIM15_IRQn: IRQn = 24;
pub const EXTI9_5_IRQn: IRQn = 23;
pub const CAN1_SCE_IRQn: IRQn = 22;
pub const CAN1_RX1_IRQn: IRQn = 21;
pub const USB_LP_CAN1_RX0_IRQn: IRQn = 20;
pub const USB_HP_CAN1_TX_IRQn: IRQn = 19;
pub const ADC1_2_IRQn: IRQn = 18;
pub const DMA1_Channel7_IRQn: IRQn = 17;
pub const DMA1_Channel6_IRQn: IRQn = 16;
pub const DMA1_Channel5_IRQn: IRQn = 15;
pub const DMA1_Channel4_IRQn: IRQn = 14;
pub const DMA1_Channel3_IRQn: IRQn = 13;
pub const DMA1_Channel2_IRQn: IRQn = 12;
pub const DMA1_Channel1_IRQn: IRQn = 11;
pub const EXTI4_IRQn: IRQn = 10;
pub const EXTI3_IRQn: IRQn = 9;
pub const EXTI2_TS_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_WKUP_IRQn: IRQn = 3;
pub const TAMPER_STAMP_IRQn: IRQn = 2;
pub const PVD_IRQn: IRQn = 1;
pub const WWDG_IRQn: IRQn = 0;
pub const SysTick_IRQn: IRQn = -1;
pub const PendSV_IRQn: IRQn = -2;
pub const DebugMonitor_IRQn: IRQn = -4;
pub const SVCall_IRQn: IRQn = -5;
pub const UsageFault_IRQn: IRQn = -10;
pub const BusFault_IRQn: IRQn = -11;
pub const MemoryManagement_IRQn: IRQn = -12;
pub const NonMaskableInt_IRQn: IRQn = -14;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief External Interrupt/Event Controller
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct EXTI_TypeDef {
    pub IMR: uint32_t,
    pub EMR: uint32_t,
    pub RTSR: uint32_t,
    pub FTSR: uint32_t,
    pub SWIER: uint32_t,
    pub PR: uint32_t,
    pub RESERVED1: uint32_t,
    pub RESERVED2: uint32_t,
    pub IMR2: uint32_t,
    pub EMR2: uint32_t,
    pub RTSR2: uint32_t,
    pub FTSR2: uint32_t,
    pub SWIER2: uint32_t,
    pub PR2: uint32_t,
    /* !< EXTI Pending register,                       Address offset: 0x34 */
}
/* *
  ******************************************************************************
  * @file    stm32f30x_exti.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the EXTI 
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
/* * @addtogroup EXTI
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  EXTI mode enumeration  
  */
pub type EXTIMode_TypeDef = libc::c_uint;
pub const EXTI_Mode_Event: EXTIMode_TypeDef = 4;
pub const EXTI_Mode_Interrupt: EXTIMode_TypeDef = 0;
/* * 
  * @brief  EXTI Trigger enumeration  
  */
pub type EXTITrigger_TypeDef = libc::c_uint;
pub const EXTI_Trigger_Rising_Falling: EXTITrigger_TypeDef = 16;
pub const EXTI_Trigger_Falling: EXTITrigger_TypeDef = 12;
pub const EXTI_Trigger_Rising: EXTITrigger_TypeDef = 8;
/* * 
  * @brief  EXTI Init Structure definition  
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct EXTI_InitTypeDef {
    pub EXTI_Line: uint32_t,
    pub EXTI_Mode: EXTIMode_TypeDef,
    pub EXTI_Trigger: EXTITrigger_TypeDef,
    pub EXTI_LineCmd: FunctionalState,
    /* !< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */
}
/* *
  ******************************************************************************
  * @file    stm32f30x_misc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the miscellaneous
  *          firmware library functions (add-on to CMSIS functions).
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
/* * @addtogroup MISC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  NVIC Init Structure definition  
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
    /* !< Specifies whether the IRQ channel defined in NVIC_IRQChannel
                                                   will be enabled or disabled. 
                                                   This parameter can be set either to ENABLE or DISABLE */
}
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ()>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
pub type extiCallbackRec_t = extiCallbackRec_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct extiChannelRec_t {
    pub handler: *mut extiCallbackRec_t,
}
#[no_mangle]
pub static mut extiChannelRecs: [extiChannelRec_t; 16] =
    [extiChannelRec_t{handler:
                          0 as *const extiCallbackRec_t as
                              *mut extiCallbackRec_t,}; 16];
// IRQ gouping, same on 103 and 303
//                                      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
static mut extiGroups: [uint8_t; 16] =
    [0i32 as uint8_t, 1i32 as uint8_t, 2i32 as uint8_t, 3i32 as uint8_t,
     4i32 as uint8_t, 5i32 as uint8_t, 5i32 as uint8_t, 5i32 as uint8_t,
     5i32 as uint8_t, 5i32 as uint8_t, 6i32 as uint8_t, 6i32 as uint8_t,
     6i32 as uint8_t, 6i32 as uint8_t, 6i32 as uint8_t, 6i32 as uint8_t];
static mut extiGroupPriority: [uint8_t; 7] = [0; 7];
static mut extiGroupIRQn: [uint8_t; 7] =
    [EXTI0_IRQn as libc::c_int as uint8_t,
     EXTI1_IRQn as libc::c_int as uint8_t,
     EXTI2_TS_IRQn as libc::c_int as uint8_t,
     EXTI3_IRQn as libc::c_int as uint8_t,
     EXTI4_IRQn as libc::c_int as uint8_t,
     EXTI9_5_IRQn as libc::c_int as uint8_t,
     EXTI15_10_IRQn as libc::c_int as uint8_t];
#[no_mangle]
pub unsafe extern "C" fn EXTIInit() {
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(0x1i32 as uint32_t, ENABLE);
    SYSCFG_DMAChannelRemapConfig(0x1000i32 as uint32_t, ENABLE);
    memset(extiChannelRecs.as_mut_ptr() as *mut libc::c_void, 0i32,
           ::core::mem::size_of::<[extiChannelRec_t; 16]>() as libc::c_ulong);
    memset(extiGroupPriority.as_mut_ptr() as *mut libc::c_void, 0xffi32,
           ::core::mem::size_of::<[uint8_t; 7]>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn EXTIHandlerInit(mut self_0: *mut extiCallbackRec_t,
                                         mut fn_0:
                                             Option<unsafe extern "C" fn(_:
                                                                             *mut extiCallbackRec_t)
                                                        -> ()>) {
    (*self_0).fn_0 = fn_0;
}
#[no_mangle]
pub unsafe extern "C" fn EXTIConfig(mut io: IO_t,
                                    mut cb: *mut extiCallbackRec_t,
                                    mut irqPriority: libc::c_int,
                                    mut trigger: EXTITrigger_TypeDef) {
    let mut chIdx: libc::c_int = 0;
    chIdx = IO_GPIOPinIdx(io);
    if chIdx < 0i32 { return }
    let mut rec: *mut extiChannelRec_t =
        &mut *extiChannelRecs.as_mut_ptr().offset(chIdx as isize) as
            *mut extiChannelRec_t;
    let mut group: libc::c_int = extiGroups[chIdx as usize] as libc::c_int;
    (*rec).handler = cb;
    SYSCFG_EXTILineConfig(IO_EXTI_PortSourceGPIO(io) as uint8_t,
                          IO_EXTI_PinSource(io) as uint8_t);
    let mut extiLine: uint32_t = IO_EXTI_Line(io);
    EXTI_ClearITPendingBit(extiLine);
    let mut EXTIInit_0: EXTI_InitTypeDef =
        EXTI_InitTypeDef{EXTI_Line: 0,
                         EXTI_Mode: EXTI_Mode_Interrupt,
                         EXTI_Trigger: 0 as EXTITrigger_TypeDef,
                         EXTI_LineCmd: DISABLE,};
    EXTIInit_0.EXTI_Line = extiLine;
    EXTIInit_0.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit_0.EXTI_Trigger = trigger;
    EXTIInit_0.EXTI_LineCmd = ENABLE;
    EXTI_Init(&mut EXTIInit_0);
    if extiGroupPriority[group as usize] as libc::c_int > irqPriority {
        extiGroupPriority[group as usize] = irqPriority as uint8_t;
        let mut NVIC_InitStructure: NVIC_InitTypeDef =
            NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                             NVIC_IRQChannelPreemptionPriority: 0,
                             NVIC_IRQChannelSubPriority: 0,
                             NVIC_IRQChannelCmd: DISABLE,};
        NVIC_InitStructure.NVIC_IRQChannel = extiGroupIRQn[group as usize];
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
            (irqPriority >>
                 (4i32 as
                      libc::c_uint).wrapping_sub((7i32 as
                                                      libc::c_uint).wrapping_sub(0x500i32
                                                                                     as
                                                                                     uint32_t
                                                                                     >>
                                                                                     8i32))
                 >> 4i32) as uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority =
            ((irqPriority &
                  0xfi32 >>
                      (7i32 as
                           libc::c_uint).wrapping_sub(0x500i32 as uint32_t >>
                                                          8i32)) >> 4i32) as
                uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&mut NVIC_InitStructure);
    };
}
#[no_mangle]
pub unsafe extern "C" fn EXTIRelease(mut io: IO_t) {
    // don't forget to match cleanup with config
    EXTIEnable(io, 0i32 != 0);
    let mut chIdx: libc::c_int = 0;
    chIdx = IO_GPIOPinIdx(io);
    if chIdx < 0i32 { return }
    let mut rec: *mut extiChannelRec_t =
        &mut *extiChannelRecs.as_mut_ptr().offset(chIdx as isize) as
            *mut extiChannelRec_t;
    (*rec).handler = 0 as *mut extiCallbackRec_t;
}
#[no_mangle]
pub unsafe extern "C" fn EXTIEnable(mut io: IO_t, mut enable: bool) {
    let mut extiLine: libc::c_int = IO_EXTI_Line(io) as libc::c_int;
    if extiLine < 0i32 { return }
    // assume extiLine < 32 (valid for all EXTI pins)
    if enable {
        let ref mut fresh0 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x400i32
                                                                              as
                                                                              libc::c_uint)
                   as
                   *mut EXTI_TypeDef)).IMR; // clear pending mask (by writing 1)
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (1i32 << extiLine) as libc::c_uint)
                                        as uint32_t as uint32_t)
    } else {
        let ref mut fresh1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).IMR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(1i32 << extiLine) as libc::c_uint)
                                        as uint32_t as uint32_t)
    };
}
#[no_mangle]
pub unsafe extern "C" fn EXTI_IRQHandler() {
    let mut exti_active: uint32_t =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x400i32
                                                                          as
                                                                          libc::c_uint)
               as *mut EXTI_TypeDef)).IMR &
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).PR;
    while exti_active != 0 {
        let mut idx: libc::c_uint =
            (31i32 - exti_active.leading_zeros() as i32) as libc::c_uint;
        let mut mask: uint32_t = (1i32 << idx) as uint32_t;
        (*extiChannelRecs[idx as
                              usize].handler).fn_0.expect("non-null function pointer")(extiChannelRecs[idx
                                                                                                           as
                                                                                                           usize].handler);
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x10000i32
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400i32
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut EXTI_TypeDef)).PR as
                                        *mut uint32_t, mask);
        exti_active &= !mask
    };
}
/* */
#[no_mangle]
pub unsafe extern "C" fn EXTI0_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI1_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI2_TS_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI3_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI4_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI9_5_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI15_10_IRQHandler() { EXTI_IRQHandler(); }
