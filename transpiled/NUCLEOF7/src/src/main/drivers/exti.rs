use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn HAL_NVIC_EnableIRQ(IRQn: IRQn_Type);
    #[no_mangle]
    fn HAL_NVIC_SetPriority(IRQn: IRQn_Type, PreemptPriority: uint32_t,
                            SubPriority: uint32_t);
    #[no_mangle]
    fn HAL_GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                     GPIO_Init: *mut GPIO_InitTypeDef);
    #[no_mangle]
    fn IO_GPIOPinIdx(io: IO_t) -> libc::c_int;
    #[no_mangle]
    fn IO_GPIO(io: IO_t) -> *mut GPIO_TypeDef;
    #[no_mangle]
    fn IO_Pin(io: IO_t) -> uint16_t;
    #[no_mangle]
    fn IO_EXTI_Line(io: IO_t) -> uint32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn_Type = libc::c_int;
pub const SPDIF_RX_IRQn: IRQn_Type = 97;
pub const I2C4_ER_IRQn: IRQn_Type = 96;
pub const I2C4_EV_IRQn: IRQn_Type = 95;
pub const CEC_IRQn: IRQn_Type = 94;
pub const LPTIM1_IRQn: IRQn_Type = 93;
pub const QUADSPI_IRQn: IRQn_Type = 92;
pub const SAI2_IRQn: IRQn_Type = 91;
pub const DMA2D_IRQn: IRQn_Type = 90;
pub const LTDC_ER_IRQn: IRQn_Type = 89;
pub const LTDC_IRQn: IRQn_Type = 88;
pub const SAI1_IRQn: IRQn_Type = 87;
pub const SPI6_IRQn: IRQn_Type = 86;
pub const SPI5_IRQn: IRQn_Type = 85;
pub const SPI4_IRQn: IRQn_Type = 84;
pub const UART8_IRQn: IRQn_Type = 83;
pub const UART7_IRQn: IRQn_Type = 82;
pub const FPU_IRQn: IRQn_Type = 81;
pub const RNG_IRQn: IRQn_Type = 80;
pub const DCMI_IRQn: IRQn_Type = 78;
pub const OTG_HS_IRQn: IRQn_Type = 77;
pub const OTG_HS_WKUP_IRQn: IRQn_Type = 76;
pub const OTG_HS_EP1_IN_IRQn: IRQn_Type = 75;
pub const OTG_HS_EP1_OUT_IRQn: IRQn_Type = 74;
pub const I2C3_ER_IRQn: IRQn_Type = 73;
pub const I2C3_EV_IRQn: IRQn_Type = 72;
pub const USART6_IRQn: IRQn_Type = 71;
pub const DMA2_Stream7_IRQn: IRQn_Type = 70;
pub const DMA2_Stream6_IRQn: IRQn_Type = 69;
pub const DMA2_Stream5_IRQn: IRQn_Type = 68;
pub const OTG_FS_IRQn: IRQn_Type = 67;
pub const CAN2_SCE_IRQn: IRQn_Type = 66;
pub const CAN2_RX1_IRQn: IRQn_Type = 65;
pub const CAN2_RX0_IRQn: IRQn_Type = 64;
pub const CAN2_TX_IRQn: IRQn_Type = 63;
pub const ETH_WKUP_IRQn: IRQn_Type = 62;
pub const ETH_IRQn: IRQn_Type = 61;
pub const DMA2_Stream4_IRQn: IRQn_Type = 60;
pub const DMA2_Stream3_IRQn: IRQn_Type = 59;
pub const DMA2_Stream2_IRQn: IRQn_Type = 58;
pub const DMA2_Stream1_IRQn: IRQn_Type = 57;
pub const DMA2_Stream0_IRQn: IRQn_Type = 56;
pub const TIM7_IRQn: IRQn_Type = 55;
pub const TIM6_DAC_IRQn: IRQn_Type = 54;
pub const UART5_IRQn: IRQn_Type = 53;
pub const UART4_IRQn: IRQn_Type = 52;
pub const SPI3_IRQn: IRQn_Type = 51;
pub const TIM5_IRQn: IRQn_Type = 50;
pub const SDMMC1_IRQn: IRQn_Type = 49;
pub const FMC_IRQn: IRQn_Type = 48;
pub const DMA1_Stream7_IRQn: IRQn_Type = 47;
pub const TIM8_CC_IRQn: IRQn_Type = 46;
pub const TIM8_TRG_COM_TIM14_IRQn: IRQn_Type = 45;
pub const TIM8_UP_TIM13_IRQn: IRQn_Type = 44;
pub const TIM8_BRK_TIM12_IRQn: IRQn_Type = 43;
pub const OTG_FS_WKUP_IRQn: IRQn_Type = 42;
pub const RTC_Alarm_IRQn: IRQn_Type = 41;
pub const EXTI15_10_IRQn: IRQn_Type = 40;
pub const USART3_IRQn: IRQn_Type = 39;
pub const USART2_IRQn: IRQn_Type = 38;
pub const USART1_IRQn: IRQn_Type = 37;
pub const SPI2_IRQn: IRQn_Type = 36;
pub const SPI1_IRQn: IRQn_Type = 35;
pub const I2C2_ER_IRQn: IRQn_Type = 34;
pub const I2C2_EV_IRQn: IRQn_Type = 33;
pub const I2C1_ER_IRQn: IRQn_Type = 32;
pub const I2C1_EV_IRQn: IRQn_Type = 31;
pub const TIM4_IRQn: IRQn_Type = 30;
pub const TIM3_IRQn: IRQn_Type = 29;
pub const TIM2_IRQn: IRQn_Type = 28;
pub const TIM1_CC_IRQn: IRQn_Type = 27;
pub const TIM1_TRG_COM_TIM11_IRQn: IRQn_Type = 26;
pub const TIM1_UP_TIM10_IRQn: IRQn_Type = 25;
pub const TIM1_BRK_TIM9_IRQn: IRQn_Type = 24;
pub const EXTI9_5_IRQn: IRQn_Type = 23;
pub const CAN1_SCE_IRQn: IRQn_Type = 22;
pub const CAN1_RX1_IRQn: IRQn_Type = 21;
pub const CAN1_RX0_IRQn: IRQn_Type = 20;
pub const CAN1_TX_IRQn: IRQn_Type = 19;
pub const ADC_IRQn: IRQn_Type = 18;
pub const DMA1_Stream6_IRQn: IRQn_Type = 17;
pub const DMA1_Stream5_IRQn: IRQn_Type = 16;
pub const DMA1_Stream4_IRQn: IRQn_Type = 15;
pub const DMA1_Stream3_IRQn: IRQn_Type = 14;
pub const DMA1_Stream2_IRQn: IRQn_Type = 13;
pub const DMA1_Stream1_IRQn: IRQn_Type = 12;
pub const DMA1_Stream0_IRQn: IRQn_Type = 11;
pub const EXTI4_IRQn: IRQn_Type = 10;
pub const EXTI3_IRQn: IRQn_Type = 9;
pub const EXTI2_IRQn: IRQn_Type = 8;
pub const EXTI1_IRQn: IRQn_Type = 7;
pub const EXTI0_IRQn: IRQn_Type = 6;
pub const RCC_IRQn: IRQn_Type = 5;
pub const FLASH_IRQn: IRQn_Type = 4;
pub const RTC_WKUP_IRQn: IRQn_Type = 3;
pub const TAMP_STAMP_IRQn: IRQn_Type = 2;
pub const PVD_IRQn: IRQn_Type = 1;
pub const WWDG_IRQn: IRQn_Type = 0;
pub const SysTick_IRQn: IRQn_Type = -1;
pub const PendSV_IRQn: IRQn_Type = -2;
pub const DebugMonitor_IRQn: IRQn_Type = -4;
pub const SVCall_IRQn: IRQn_Type = -5;
pub const UsageFault_IRQn: IRQn_Type = -10;
pub const BusFault_IRQn: IRQn_Type = -11;
pub const MemoryManagement_IRQn: IRQn_Type = -12;
pub const NonMaskableInt_IRQn: IRQn_Type = -14;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint32_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Pull: uint32_t,
    pub Speed: uint32_t,
    pub Alternate: uint32_t,
}
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
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
                                    mut config: ioConfig_t) {
    let mut chIdx: libc::c_int = 0;
    chIdx = IO_GPIOPinIdx(io);
    if chIdx < 0 as libc::c_int { return }
    let mut rec: *mut extiChannelRec_t =
        &mut *extiChannelRecs.as_mut_ptr().offset(chIdx as isize) as
            *mut extiChannelRec_t;
    let mut group: libc::c_int = extiGroups[chIdx as usize] as libc::c_int;
    let mut init: GPIO_InitTypeDef =
        {
            let mut init =
                GPIO_InitTypeDef{Pin: IO_Pin(io) as uint32_t,
                                 Mode: 0x10110000 as libc::c_uint,
                                 Pull: 0 as libc::c_uint,
                                 Speed: 0 as libc::c_uint,
                                 Alternate: 0,};
            init
        };
    HAL_GPIO_Init(IO_GPIO(io), &mut init);
    (*rec).handler = cb;
    //uint32_t extiLine = IO_EXTI_Line(io);
    //EXTI_ClearITPendingBit(extiLine);
    if extiGroupPriority[group as usize] as libc::c_int > irqPriority {
        extiGroupPriority[group as usize] = irqPriority as uint8_t;
        HAL_NVIC_SetPriority(extiGroupIRQn[group as usize] as IRQn_Type,
                             (irqPriority >>
                                  (4 as libc::c_int as
                                       libc::c_uint).wrapping_sub((7 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_uint).wrapping_sub(0x5
                                                                                                      as
                                                                                                      libc::c_uint))
                                  >> 4 as libc::c_int) as uint32_t,
                             ((irqPriority &
                                   0xf as libc::c_int >>
                                       (7 as libc::c_int as
                                            libc::c_uint).wrapping_sub(0x5 as
                                                                           libc::c_uint))
                                  >> 4 as libc::c_int) as uint32_t);
        HAL_NVIC_EnableIRQ(extiGroupIRQn[group as usize] as IRQn_Type);
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
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
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
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
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
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3c00
                                                                              as
                                                                              libc::c_uint)
               as *mut EXTI_TypeDef)).IMR &
            (*((0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x3c00
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
        ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                                 libc::c_uint).wrapping_add(0x10000
                                                                                as
                                                                                libc::c_uint).wrapping_add(0x3c00
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
