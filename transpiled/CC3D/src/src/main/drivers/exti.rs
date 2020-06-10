use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn EXTI_Init(EXTI_InitStruct: *mut EXTI_InitTypeDef);
    #[no_mangle]
    fn EXTI_ClearITPendingBit(EXTI_Line: uint32_t);
    #[no_mangle]
    fn GPIO_EXTILineConfig(GPIO_PortSource: uint8_t, GPIO_PinSource: uint8_t);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    #[no_mangle]
    fn RCC_APB2PeriphClockCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
    #[no_mangle]
    fn IO_GPIOPinIdx(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_GPIO_PinSource(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_GPIO_PortSource(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_EXTI_Line(io: IO_t) -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type IRQn = libc::c_int;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTCAlarm_IRQn: IRQn = 41;
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
pub const TIM1_TRG_COM_IRQn: IRQn = 26;
pub const TIM1_UP_IRQn: IRQn = 25;
pub const TIM1_BRK_IRQn: IRQn = 24;
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
pub const EXTI2_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_IRQn: IRQn = 3;
pub const TAMPER_IRQn: IRQn = 2;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
}
pub type IO_t = *mut libc::c_void;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
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
#[derive(Copy, Clone)]
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
    [0 as libc::c_int as uint8_t, 1 as libc::c_int as uint8_t,
     2 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t,
     4 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
     5 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 6 as libc::c_int as uint8_t];
static mut extiGroupPriority: [uint8_t; 7] = [0; 7];
static mut extiGroupIRQn: [uint8_t; 7] =
    [EXTI0_IRQn as libc::c_int as uint8_t,
     EXTI1_IRQn as libc::c_int as uint8_t,
     EXTI2_IRQn as libc::c_int as uint8_t,
     EXTI3_IRQn as libc::c_int as uint8_t,
     EXTI4_IRQn as libc::c_int as uint8_t,
     EXTI9_5_IRQn as libc::c_int as uint8_t,
     EXTI15_10_IRQn as libc::c_int as uint8_t];
#[no_mangle]
pub unsafe extern "C" fn EXTIInit() {
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(0x1 as libc::c_int as uint32_t, ENABLE);
    memset(extiChannelRecs.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::core::mem::size_of::<[extiChannelRec_t; 16]>() as libc::c_ulong);
    memset(extiGroupPriority.as_mut_ptr() as *mut libc::c_void,
           0xff as libc::c_int,
           ::core::mem::size_of::<[uint8_t; 7]>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn EXTIHandlerInit(mut self_0: *mut extiCallbackRec_t,
                                         mut fn_0:
                                             Option<extiHandlerCallback>) {
    (*self_0).fn_0 = fn_0;
}
#[no_mangle]
pub unsafe extern "C" fn EXTIConfig(mut io: IO_t,
                                    mut cb: *mut extiCallbackRec_t,
                                    mut irqPriority: libc::c_int,
                                    mut trigger: EXTITrigger_TypeDef) {
    let mut chIdx: libc::c_int = 0;
    chIdx = IO_GPIOPinIdx(io);
    if chIdx < 0 as libc::c_int { return }
    let mut rec: *mut extiChannelRec_t =
        &mut *extiChannelRecs.as_mut_ptr().offset(chIdx as isize) as
            *mut extiChannelRec_t;
    let mut group: libc::c_int = extiGroups[chIdx as usize] as libc::c_int;
    (*rec).handler = cb;
    GPIO_EXTILineConfig(IO_GPIO_PortSource(io) as uint8_t,
                        IO_GPIO_PinSource(io) as uint8_t);
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
                 (4 as libc::c_int as
                      libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                      libc::c_uint).wrapping_sub(0x500
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     uint32_t
                                                                                     >>
                                                                                     8
                                                                                         as
                                                                                         libc::c_int))
                 >> 4 as libc::c_int) as uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority =
            ((irqPriority &
                  0xf as libc::c_int >>
                      (7 as libc::c_int as
                           libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                          uint32_t >>
                                                          8 as libc::c_int))
                 >> 4 as libc::c_int) as uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&mut NVIC_InitStructure);
    };
}
#[no_mangle]
pub unsafe extern "C" fn EXTIRelease(mut io: IO_t) {
    // don't forget to match cleanup with config
    EXTIEnable(io,
               0 as libc::c_int != 0); // clear pending mask (by writing 1)
    let mut chIdx: libc::c_int = 0;
    chIdx = IO_GPIOPinIdx(io);
    if chIdx < 0 as libc::c_int { return }
    let mut rec: *mut extiChannelRec_t =
        &mut *extiChannelRecs.as_mut_ptr().offset(chIdx as isize) as
            *mut extiChannelRec_t;
    (*rec).handler = 0 as *mut extiCallbackRec_t;
}
#[no_mangle]
pub unsafe extern "C" fn EXTIEnable(mut io: IO_t, mut enable: bool) {
    let mut extiLine: uint32_t = IO_EXTI_Line(io);
    if extiLine == 0 { return }
    if enable {
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
                                         as libc::c_uint | extiLine) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).IMR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !extiLine) as
                                        uint32_t as uint32_t)
    };
}
#[no_mangle]
pub unsafe extern "C" fn EXTI_IRQHandler() {
    let mut exti_active: uint32_t =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x400 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut EXTI_TypeDef)).IMR &
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut EXTI_TypeDef)).PR;
    while exti_active != 0 {
        let mut idx: libc::c_uint =
            (31 as libc::c_int - exti_active.leading_zeros() as i32) as
                libc::c_uint;
        let mut mask: uint32_t = ((1 as libc::c_int) << idx) as uint32_t;
        (*extiChannelRecs[idx as
                              usize].handler).fn_0.expect("non-null function pointer")(extiChannelRecs[idx
                                                                                                           as
                                                                                                           usize].handler);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
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
pub unsafe extern "C" fn EXTI2_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI3_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI4_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI9_5_IRQHandler() { EXTI_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn EXTI15_10_IRQHandler() { EXTI_IRQHandler(); }
