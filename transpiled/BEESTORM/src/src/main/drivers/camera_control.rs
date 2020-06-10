use ::libc;
extern "C" {
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    // TIMUP
    // bidirectional pin for softserial
    // timer channel is used for softserial. No IO function on pin
    // This interface should be replaced.
    // TODO - just for migration
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn pwmOutConfig(channel: *mut timerChannel_t,
                    timerHardware: *const timerHardware_t, hz: uint32_t,
                    period: uint16_t, value: uint16_t, inversion: uint8_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut resumeRefreshAt: timeUs_t;
}
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
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
/* *
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub APB1RSTR: uint32_t,
    pub AHBENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub APB1ENR: uint32_t,
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub AHBRSTR: uint32_t,
    pub CFGR2: uint32_t,
    pub CFGR3: uint32_t,
}
/* *
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
}
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
// packet tag to specify IO pin
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_2 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_2 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_2 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_2 = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_3,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_3 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type cameraControlKey_e = libc::c_uint;
pub const CAMERA_CONTROL_KEYS_COUNT: cameraControlKey_e = 5;
pub const CAMERA_CONTROL_KEY_DOWN: cameraControlKey_e = 4;
pub const CAMERA_CONTROL_KEY_RIGHT: cameraControlKey_e = 3;
pub const CAMERA_CONTROL_KEY_UP: cameraControlKey_e = 2;
pub const CAMERA_CONTROL_KEY_LEFT: cameraControlKey_e = 1;
pub const CAMERA_CONTROL_KEY_ENTER: cameraControlKey_e = 0;
pub type cameraControlMode_e = libc::c_uint;
pub const CAMERA_CONTROL_MODES_COUNT: cameraControlMode_e = 3;
pub const CAMERA_CONTROL_MODE_DAC: cameraControlMode_e = 2;
pub const CAMERA_CONTROL_MODE_SOFTWARE_PWM: cameraControlMode_e = 1;
pub const CAMERA_CONTROL_MODE_HARDWARE_PWM: cameraControlMode_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cameraControlConfig_s {
    pub mode: cameraControlMode_e,
    pub refVoltage: uint16_t,
    pub keyDelayMs: uint16_t,
    pub internalResistance: uint16_t,
    pub ioTag: ioTag_t,
    pub inverted: uint8_t,
}
pub type cameraControlConfig_t = cameraControlConfig_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// Output is inverted externally
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_4 {
    pub enabled: bool,
    pub io: IO_t,
    pub channel: timerChannel_t,
    pub period: uint32_t,
    pub inverted: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type timerHardware_t = timerHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Channel_TypeDef,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Channel_TypeDef,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
#[inline]
unsafe extern "C" fn cameraControlConfig() -> *const cameraControlConfig_t {
    return &mut cameraControlConfig_System;
}
// set BASEPRI_MAX, with global memory barrier, returns true
#[inline]
unsafe extern "C" fn __basepriSetMemRetVal(mut prio: uint8_t) -> uint8_t {
    __set_BASEPRI_MAX(prio as libc::c_int);
    return 1 as libc::c_int as uint8_t;
}
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut cameraControlConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (522 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<cameraControlConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &cameraControlConfig_System as
                                     *const cameraControlConfig_t as
                                     *mut cameraControlConfig_t as
                                     *mut uint8_t,
                             copy:
                                 &cameraControlConfig_Copy as
                                     *const cameraControlConfig_t as
                                     *mut cameraControlConfig_t as
                                     *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_3{ptr:
                                                     &pgResetTemplate_cameraControlConfig
                                                         as
                                                         *const cameraControlConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
#[no_mangle]
pub static mut cameraControlConfig_System: cameraControlConfig_t =
    cameraControlConfig_t{mode: CAMERA_CONTROL_MODE_HARDWARE_PWM,
                          refVoltage: 0,
                          keyDelayMs: 0,
                          internalResistance: 0,
                          ioTag: 0,
                          inverted: 0,};
#[no_mangle]
pub static mut cameraControlConfig_Copy: cameraControlConfig_t =
    cameraControlConfig_t{mode: CAMERA_CONTROL_MODE_HARDWARE_PWM,
                          refVoltage: 0,
                          keyDelayMs: 0,
                          internalResistance: 0,
                          ioTag: 0,
                          inverted: 0,};
#[no_mangle]
#[link_section = ".pg_resetdata"]
#[used]
pub static mut pgResetTemplate_cameraControlConfig: cameraControlConfig_t =
    {
        let mut init =
            cameraControlConfig_s{mode: CAMERA_CONTROL_MODE_HARDWARE_PWM,
                                  refVoltage: 330 as libc::c_int as uint16_t,
                                  keyDelayMs: 180 as libc::c_int as uint16_t,
                                  internalResistance:
                                      470 as libc::c_int as uint16_t,
                                  ioTag: 0 as libc::c_int as ioTag_t,
                                  inverted: 0 as libc::c_int as uint8_t,};
        init
    };
static mut cameraControlRuntime: C2RustUnnamed_4 =
    C2RustUnnamed_4{enabled: false,
                    io: 0 as *const libc::c_void as *mut libc::c_void,
                    channel:
                        timerChannel_t{ccr:
                                           0 as *const timCCR_t as
                                               *mut timCCR_t,
                                       tim:
                                           0 as *const TIM_TypeDef as
                                               *mut TIM_TypeDef,},
                    period: 0,
                    inverted: 0,};
static mut endTimeMillis: uint32_t = 0;
unsafe extern "C" fn cameraControlHi() {
    if cameraControlRuntime.inverted != 0 {
        IOLo(cameraControlRuntime.io);
    } else { IOHi(cameraControlRuntime.io); };
}
unsafe extern "C" fn cameraControlLo() {
    if cameraControlRuntime.inverted != 0 {
        IOHi(cameraControlRuntime.io);
    } else { IOLo(cameraControlRuntime.io); };
}
#[no_mangle]
pub unsafe extern "C" fn TIM6_DAC_IRQHandler() {
    cameraControlHi();
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x1000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut TIM_TypeDef)).SR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn TIM7_IRQHandler() {
    cameraControlLo();
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x1400 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut TIM_TypeDef)).SR as
                                    *mut uint32_t,
                                0 as libc::c_int as uint32_t);
}
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
// measured in 10 mV steps
// measured 100 Ohm steps
#[no_mangle]
pub unsafe extern "C" fn cameraControlInit() {
    if (*cameraControlConfig()).ioTag as libc::c_int == 0 as libc::c_int {
        return
    }
    cameraControlRuntime.inverted = (*cameraControlConfig()).inverted;
    cameraControlRuntime.io = IOGetByTag((*cameraControlConfig()).ioTag);
    IOInit(cameraControlRuntime.io, OWNER_CAMERA_CONTROL,
           0 as libc::c_int as uint8_t);
    if CAMERA_CONTROL_MODE_HARDWARE_PWM as libc::c_int as libc::c_uint ==
           (*cameraControlConfig()).mode as libc::c_uint {
        let mut timerHardware: *const timerHardware_t =
            timerGetByTag((*cameraControlConfig()).ioTag);
        if timerHardware.is_null() { return }
        IOConfigGPIOAF(cameraControlRuntime.io,
                       (GPIO_Mode_AF as libc::c_int |
                            (0 as libc::c_int) << 2 as libc::c_int |
                            (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int
                            |
                            (GPIO_PuPd_NOPULL as libc::c_int) <<
                                5 as libc::c_int) as ioConfig_t,
                       (*timerHardware).alternateFunction);
        pwmOutConfig(&mut cameraControlRuntime.channel, timerHardware,
                     (72 as libc::c_int * 1000000 as libc::c_int) as uint32_t,
                     128 as libc::c_int as uint16_t,
                     0 as libc::c_int as uint16_t,
                     cameraControlRuntime.inverted);
        cameraControlRuntime.period = 128 as libc::c_int as uint32_t;
        ::core::ptr::write_volatile(cameraControlRuntime.channel.ccr,
                                    cameraControlRuntime.period);
        cameraControlRuntime.enabled = 1 as libc::c_int != 0
    } else if CAMERA_CONTROL_MODE_SOFTWARE_PWM as libc::c_int as libc::c_uint
                  == (*cameraControlConfig()).mode as libc::c_uint {
        IOConfigGPIO(cameraControlRuntime.io,
                     (GPIO_Mode_OUT as libc::c_int |
                          (0 as libc::c_int) << 2 as libc::c_int |
                          (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                          (GPIO_PuPd_NOPULL as libc::c_int) <<
                              5 as libc::c_int) as ioConfig_t);
        cameraControlHi();
        cameraControlRuntime.period = 448 as libc::c_int as uint32_t;
        cameraControlRuntime.enabled = 1 as libc::c_int != 0;
        let mut nvicTIM6: NVIC_InitTypeDef =
            {
                let mut init =
                    NVIC_InitTypeDef{NVIC_IRQChannel:
                                         TIM6_DAC_IRQn as libc::c_int as
                                             uint8_t,
                                     NVIC_IRQChannelPreemptionPriority:
                                         ((((1 as libc::c_int) <<
                                                (4 as libc::c_int as
                                                     libc::c_uint).wrapping_sub((7
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint).wrapping_sub(0x500
                                                                                                                    as
                                                                                                                    libc::c_int
                                                                                                                    as
                                                                                                                    uint32_t
                                                                                                                    >>
                                                                                                                    8
                                                                                                                        as
                                                                                                                        libc::c_int))
                                                |
                                                1 as libc::c_int &
                                                    0xf as libc::c_int >>
                                                        (7 as libc::c_int as
                                                             libc::c_uint).wrapping_sub(0x500
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            uint32_t
                                                                                            >>
                                                                                            8
                                                                                                as
                                                                                                libc::c_int))
                                               << 4 as libc::c_int &
                                               0xf0 as libc::c_int) >>
                                              (4 as libc::c_int as
                                                   libc::c_uint).wrapping_sub((7
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint).wrapping_sub(0x500
                                                                                                                  as
                                                                                                                  libc::c_int
                                                                                                                  as
                                                                                                                  uint32_t
                                                                                                                  >>
                                                                                                                  8
                                                                                                                      as
                                                                                                                      libc::c_int))
                                              >> 4 as libc::c_int) as uint8_t,
                                     NVIC_IRQChannelSubPriority:
                                         ((((1 as libc::c_int) <<
                                                (4 as libc::c_int as
                                                     libc::c_uint).wrapping_sub((7
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint).wrapping_sub(0x500
                                                                                                                    as
                                                                                                                    libc::c_int
                                                                                                                    as
                                                                                                                    uint32_t
                                                                                                                    >>
                                                                                                                    8
                                                                                                                        as
                                                                                                                        libc::c_int))
                                                |
                                                1 as libc::c_int &
                                                    0xf as libc::c_int >>
                                                        (7 as libc::c_int as
                                                             libc::c_uint).wrapping_sub(0x500
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            uint32_t
                                                                                            >>
                                                                                            8
                                                                                                as
                                                                                                libc::c_int))
                                               << 4 as libc::c_int &
                                               0xf0 as libc::c_int &
                                               0xf as libc::c_int >>
                                                   (7 as libc::c_int as
                                                        libc::c_uint).wrapping_sub(0x500
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       uint32_t
                                                                                       >>
                                                                                       8
                                                                                           as
                                                                                           libc::c_int))
                                              >> 4 as libc::c_int) as uint8_t,
                                     NVIC_IRQChannelCmd: ENABLE,};
                init
            };
        NVIC_Init(&mut nvicTIM6);
        let mut nvicTIM7: NVIC_InitTypeDef =
            {
                let mut init =
                    NVIC_InitTypeDef{NVIC_IRQChannel:
                                         TIM7_IRQn as libc::c_int as uint8_t,
                                     NVIC_IRQChannelPreemptionPriority:
                                         ((((1 as libc::c_int) <<
                                                (4 as libc::c_int as
                                                     libc::c_uint).wrapping_sub((7
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint).wrapping_sub(0x500
                                                                                                                    as
                                                                                                                    libc::c_int
                                                                                                                    as
                                                                                                                    uint32_t
                                                                                                                    >>
                                                                                                                    8
                                                                                                                        as
                                                                                                                        libc::c_int))
                                                |
                                                1 as libc::c_int &
                                                    0xf as libc::c_int >>
                                                        (7 as libc::c_int as
                                                             libc::c_uint).wrapping_sub(0x500
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            uint32_t
                                                                                            >>
                                                                                            8
                                                                                                as
                                                                                                libc::c_int))
                                               << 4 as libc::c_int &
                                               0xf0 as libc::c_int) >>
                                              (4 as libc::c_int as
                                                   libc::c_uint).wrapping_sub((7
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint).wrapping_sub(0x500
                                                                                                                  as
                                                                                                                  libc::c_int
                                                                                                                  as
                                                                                                                  uint32_t
                                                                                                                  >>
                                                                                                                  8
                                                                                                                      as
                                                                                                                      libc::c_int))
                                              >> 4 as libc::c_int) as uint8_t,
                                     NVIC_IRQChannelSubPriority:
                                         ((((1 as libc::c_int) <<
                                                (4 as libc::c_int as
                                                     libc::c_uint).wrapping_sub((7
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint).wrapping_sub(0x500
                                                                                                                    as
                                                                                                                    libc::c_int
                                                                                                                    as
                                                                                                                    uint32_t
                                                                                                                    >>
                                                                                                                    8
                                                                                                                        as
                                                                                                                        libc::c_int))
                                                |
                                                1 as libc::c_int &
                                                    0xf as libc::c_int >>
                                                        (7 as libc::c_int as
                                                             libc::c_uint).wrapping_sub(0x500
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            uint32_t
                                                                                            >>
                                                                                            8
                                                                                                as
                                                                                                libc::c_int))
                                               << 4 as libc::c_int &
                                               0xf0 as libc::c_int &
                                               0xf as libc::c_int >>
                                                   (7 as libc::c_int as
                                                        libc::c_uint).wrapping_sub(0x500
                                                                                       as
                                                                                       libc::c_int
                                                                                       as
                                                                                       uint32_t
                                                                                       >>
                                                                                       8
                                                                                           as
                                                                                           libc::c_int))
                                              >> 4 as libc::c_int) as uint8_t,
                                     NVIC_IRQChannelCmd: ENABLE,};
                init
            };
        NVIC_Init(&mut nvicTIM7);
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1ENR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x10 as libc::c_int as uint32_t |
                                              0x20 as libc::c_int as
                                                  uint32_t)) as uint32_t as
                                        uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x1000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                                as *mut TIM_TypeDef)).PSC as
                                        *mut uint16_t,
                                    0 as libc::c_int as uint16_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x1400
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                                                as *mut TIM_TypeDef)).PSC as
                                        *mut uint16_t,
                                    0 as libc::c_int as uint16_t)
    } else {
        (CAMERA_CONTROL_MODE_DAC as libc::c_int as libc::c_uint) ==
            (*cameraControlConfig()).mode as libc::c_uint;
    };
}
#[no_mangle]
pub unsafe extern "C" fn cameraControlProcess(mut currentTimeUs: uint32_t) {
    if endTimeMillis != 0 &&
           currentTimeUs >=
               (1000 as libc::c_int as
                    libc::c_uint).wrapping_mul(endTimeMillis) {
        if CAMERA_CONTROL_MODE_HARDWARE_PWM as libc::c_int as libc::c_uint ==
               (*cameraControlConfig()).mode as libc::c_uint {
            ::core::ptr::write_volatile(cameraControlRuntime.channel.ccr,
                                        cameraControlRuntime.period)
        } else {
            (CAMERA_CONTROL_MODE_SOFTWARE_PWM as libc::c_int as libc::c_uint)
                == (*cameraControlConfig()).mode as libc::c_uint;
        }
        endTimeMillis = 0 as libc::c_int as uint32_t
    };
}
static mut buttonResistanceValues: [libc::c_int; 5] =
    [45000 as libc::c_int, 27000 as libc::c_int, 15000 as libc::c_int,
     6810 as libc::c_int, 0 as libc::c_int];
unsafe extern "C" fn calculateKeyPressVoltage(key: cameraControlKey_e)
 -> libc::c_float {
    let buttonResistance: libc::c_int = buttonResistanceValues[key as usize];
    return 1.0e-2f32 *
               (*cameraControlConfig()).refVoltage as libc::c_int as
                   libc::c_float * buttonResistance as libc::c_float /
               (100 as libc::c_int *
                    (*cameraControlConfig()).internalResistance as libc::c_int
                    + buttonResistance) as libc::c_float;
}
unsafe extern "C" fn calculatePWMDutyCycle(key: cameraControlKey_e)
 -> libc::c_float {
    let voltage: libc::c_float = calculateKeyPressVoltage(key);
    return voltage / 3.3f32;
}
#[no_mangle]
pub unsafe extern "C" fn cameraControlKeyPress(mut key: cameraControlKey_e,
                                               mut holdDurationMs: uint32_t) {
    if !cameraControlRuntime.enabled { return }
    if key as libc::c_uint >=
           CAMERA_CONTROL_KEYS_COUNT as libc::c_int as libc::c_uint {
        return
    }
    let dutyCycle: libc::c_float = calculatePWMDutyCycle(key);
    // Force OSD timeout so we are alone on the display.
    resumeRefreshAt = 0 as libc::c_int as timeUs_t;
    if CAMERA_CONTROL_MODE_HARDWARE_PWM as libc::c_int as libc::c_uint ==
           (*cameraControlConfig()).mode as libc::c_uint {
        ::core::ptr::write_volatile(cameraControlRuntime.channel.ccr,
                                    lrintf(dutyCycle *
                                               cameraControlRuntime.period as
                                                   libc::c_float) as
                                        timCCR_t);
        endTimeMillis =
            millis().wrapping_add((*cameraControlConfig()).keyDelayMs as
                                      libc::c_uint).wrapping_add(holdDurationMs)
    } else if CAMERA_CONTROL_MODE_SOFTWARE_PWM as libc::c_int as libc::c_uint
                  == (*cameraControlConfig()).mode as libc::c_uint {
        let hiTime: uint32_t =
            lrintf(dutyCycle * cameraControlRuntime.period as libc::c_float)
                as uint32_t;
        if 0 as libc::c_int as libc::c_uint == hiTime {
            cameraControlLo();
            delay(((*cameraControlConfig()).keyDelayMs as
                       libc::c_uint).wrapping_add(holdDurationMs));
            cameraControlHi();
        } else {
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).CNT
                                            as *mut uint32_t, hiTime);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).ARR
                                            as *mut uint32_t,
                                        cameraControlRuntime.period);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1400
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).CNT
                                            as *mut uint32_t,
                                        0 as libc::c_int as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1400
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).ARR
                                            as *mut uint32_t,
                                        cameraControlRuntime.period);
            // Start two timers as simultaneously as possible
            let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
            let mut __ToDo: uint8_t =
                __basepriSetMemRetVal((((1 as libc::c_int) <<
                                            (4 as libc::c_int as
                                                 libc::c_uint).wrapping_sub((7
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint).wrapping_sub(0x500
                                                                                                                as
                                                                                                                libc::c_int
                                                                                                                as
                                                                                                                uint32_t
                                                                                                                >>
                                                                                                                8
                                                                                                                    as
                                                                                                                    libc::c_int))
                                            |
                                            1 as libc::c_int &
                                                0xf as libc::c_int >>
                                                    (7 as libc::c_int as
                                                         libc::c_uint).wrapping_sub(0x500
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        uint32_t
                                                                                        >>
                                                                                        8
                                                                                            as
                                                                                            libc::c_int))
                                           << 4 as libc::c_int &
                                           0xf0 as libc::c_int) as uint8_t);
            while __ToDo != 0 {
                ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int
                                                         as
                                                         uint32_t).wrapping_add(0x1000
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                                        as
                                                        *mut TIM_TypeDef)).CR1
                                                as *mut uint16_t,
                                            0x1 as libc::c_int as uint16_t);
                ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int
                                                         as
                                                         uint32_t).wrapping_add(0x1400
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                                        as
                                                        *mut TIM_TypeDef)).CR1
                                                as *mut uint16_t,
                                            0x1 as libc::c_int as uint16_t);
                __ToDo = 0 as libc::c_int as uint8_t
            }
            // Enable interrupt generation
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).DIER
                                            as *mut uint32_t,
                                        0x1 as libc::c_int as uint16_t as
                                            uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1400
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).DIER
                                            as *mut uint32_t,
                                        0x1 as libc::c_int as uint16_t as
                                            uint32_t);
            let endTime: uint32_t =
                millis().wrapping_add((*cameraControlConfig()).keyDelayMs as
                                          libc::c_uint).wrapping_add(holdDurationMs);
            // Wait to give the camera a chance at registering the key press
            while millis() < endTime { }
            // Disable timers and interrupt generation
            let ref mut fresh1 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x1000 as libc::c_int as
                                                   libc::c_uint) as
                       *mut TIM_TypeDef)).CR1;
            ::core::ptr::write_volatile(fresh1,
                                        (::core::ptr::read_volatile::<uint16_t>(fresh1
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int &
                                             !(0x1 as libc::c_int as uint16_t
                                                   as libc::c_int)) as
                                            uint16_t as uint16_t);
            let ref mut fresh2 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x1400 as libc::c_int as
                                                   libc::c_uint) as
                       *mut TIM_TypeDef)).CR1;
            ::core::ptr::write_volatile(fresh2,
                                        (::core::ptr::read_volatile::<uint16_t>(fresh2
                                                                                    as
                                                                                    *const uint16_t)
                                             as libc::c_int &
                                             !(0x1 as libc::c_int as uint16_t
                                                   as libc::c_int)) as
                                            uint16_t as uint16_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1000
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).DIER
                                            as *mut uint32_t,
                                        0 as libc::c_int as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                     uint32_t).wrapping_add(0x1400
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut TIM_TypeDef)).DIER
                                            as *mut uint32_t,
                                        0 as libc::c_int as uint32_t);
            // Reset to idle state
            IOHi(cameraControlRuntime.io);
        }
    } else {
        (CAMERA_CONTROL_MODE_DAC as libc::c_int as libc::c_uint) ==
            (*cameraControlConfig()).mode as libc::c_uint;
    };
}
