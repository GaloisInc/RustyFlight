use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn ceilf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn TIM_OCStructInit(TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_Cmd(TIMx: *mut TIM_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn TIM_CtrlPWMOutputs(TIMx: *mut TIM_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn timerChCCR(timHw: *const timerHardware_t) -> *mut timCCR_t;
    #[no_mangle]
    fn timerForceOverflow(tim: *mut TIM_TypeDef);
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
    #[no_mangle]
    fn configTimeBase(tim: *mut TIM_TypeDef, period: uint16_t, hz: uint32_t);
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn timerOCInit(tim: *mut TIM_TypeDef, channel: uint8_t,
                   init: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn timerOCPreloadConfig(tim: *mut TIM_TypeDef, channel: uint8_t,
                            preload: uint16_t);
}
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
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SMCR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DIER: uint16_t,
    pub RESERVED3: uint16_t,
    pub SR: uint16_t,
    pub RESERVED4: uint16_t,
    pub EGR: uint16_t,
    pub RESERVED5: uint16_t,
    pub CCMR1: uint16_t,
    pub RESERVED6: uint16_t,
    pub CCMR2: uint16_t,
    pub RESERVED7: uint16_t,
    pub CCER: uint16_t,
    pub RESERVED8: uint16_t,
    pub CNT: uint16_t,
    pub RESERVED9: uint16_t,
    pub PSC: uint16_t,
    pub RESERVED10: uint16_t,
    pub ARR: uint16_t,
    pub RESERVED11: uint16_t,
    pub RCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub CCR1: uint16_t,
    pub RESERVED13: uint16_t,
    pub CCR2: uint16_t,
    pub RESERVED14: uint16_t,
    pub CCR3: uint16_t,
    pub RESERVED15: uint16_t,
    pub CCR4: uint16_t,
    pub RESERVED16: uint16_t,
    pub BDTR: uint16_t,
    pub RESERVED17: uint16_t,
    pub DCR: uint16_t,
    pub RESERVED18: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED19: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
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
/* * @addtogroup GPIO
  * @{
  */
/* * @defgroup GPIO_Exported_Types
  * @{
  */
/* * 
  * @brief  Output Maximum frequency selection  
  */
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Speed_50MHz: C2RustUnnamed = 3;
pub const GPIO_Speed_2MHz: C2RustUnnamed = 2;
pub const GPIO_Speed_10MHz: C2RustUnnamed = 1;
/* * 
  * @brief  Configuration Mode enumeration  
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_Mode_AF_PP: C2RustUnnamed_0 = 24;
pub const GPIO_Mode_AF_OD: C2RustUnnamed_0 = 28;
pub const GPIO_Mode_Out_PP: C2RustUnnamed_0 = 16;
pub const GPIO_Mode_Out_OD: C2RustUnnamed_0 = 20;
pub const GPIO_Mode_IPU: C2RustUnnamed_0 = 72;
pub const GPIO_Mode_IPD: C2RustUnnamed_0 = 40;
pub const GPIO_Mode_IN_FLOATING: C2RustUnnamed_0 = 4;
pub const GPIO_Mode_AIN: C2RustUnnamed_0 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_OCInitTypeDef {
    pub TIM_OCMode: uint16_t,
    pub TIM_OutputState: uint16_t,
    pub TIM_OutputNState: uint16_t,
    pub TIM_Pulse: uint16_t,
    pub TIM_OCPolarity: uint16_t,
    pub TIM_OCNPolarity: uint16_t,
    pub TIM_OCIdleState: uint16_t,
    pub TIM_OCNIdleState: uint16_t,
}
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
pub type ioTag_t = uint8_t;
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
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint16_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed_1 = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed_1 = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed_1 = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const PWM_TYPE_MAX: C2RustUnnamed_2 = 5;
pub const PWM_TYPE_BRUSHED: C2RustUnnamed_2 = 4;
pub const PWM_TYPE_MULTISHOT: C2RustUnnamed_2 = 3;
pub const PWM_TYPE_ONESHOT42: C2RustUnnamed_2 = 2;
pub const PWM_TYPE_ONESHOT125: C2RustUnnamed_2 = 1;
pub const PWM_TYPE_STANDARD: C2RustUnnamed_2 = 0;
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
/*
  DshotSettingRequest (KISS24). Spin direction, 3d and save Settings reqire 10 requests.. and the TLM Byte must always be high if 1-47 are used to send settings

  3D Mode:
  0 = stop
  48   (low) - 1047 (high) -> negative direction
  1048 (low) - 2047 (high) -> positive direction
 */
// V2 includes settings
// Currently not implemented
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// BLHeli32 only
// KISS audio Stream mode on/Off
// KISS silent Mode on/Off
/* resolution + frame reset (2us) */
/* resolution + frame reset (2us) */
pub type pwmWriteFn
    =
    unsafe extern "C" fn(_: uint8_t, _: libc::c_float) -> ();
pub type pwmCompleteWriteFn = unsafe extern "C" fn(_: uint8_t) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannel_t {
    pub ccr: *mut timCCR_t,
    pub tim: *mut TIM_TypeDef,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pwmOutputPort_t {
    pub channel: timerChannel_t,
    pub pulseScale: libc::c_float,
    pub pulseOffset: libc::c_float,
    pub forceOverflow: bool,
    pub enabled: bool,
    pub io: IO_t,
}
// function pointer used to write motors
// function pointer used after motors are written
//CAVEAT: This is used in the `motorConfig_t` parameter group, so the parameter group constraints apply
#[derive(Copy, Clone)]
#[repr(C)]
pub struct motorDevConfig_s {
    pub motorPwmRate: uint16_t,
    pub motorPwmProtocol: uint8_t,
    pub motorPwmInversion: uint8_t,
    pub useUnsyncedPwm: uint8_t,
    pub useBurstDshot: uint8_t,
    pub ioTags: [ioTag_t; 4],
}
pub type motorDevConfig_t = motorDevConfig_s;
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
static mut pwmWrite: Option<pwmWriteFn> = None;
static mut motors: [pwmOutputPort_t; 4] =
    [pwmOutputPort_t{channel:
                         timerChannel_t{ccr:
                                            0 as *const timCCR_t as
                                                *mut timCCR_t,
                                        tim:
                                            0 as *const TIM_TypeDef as
                                                *mut TIM_TypeDef,},
                     pulseScale: 0.,
                     pulseOffset: 0.,
                     forceOverflow: false,
                     enabled: false,
                     io: 0 as *const libc::c_void as *mut libc::c_void,}; 4];
static mut pwmCompleteWrite: Option<pwmCompleteWriteFn> = None;
static mut pwmMotorsEnabled: bool = 0 as libc::c_int != 0;
static mut isDshot: bool = 0 as libc::c_int != 0;
unsafe extern "C" fn pwmOCConfig(mut tim: *mut TIM_TypeDef,
                                 mut channel: uint8_t, mut value: uint16_t,
                                 mut output: uint8_t) {
    let mut TIM_OCInitStructure: TIM_OCInitTypeDef =
        TIM_OCInitTypeDef{TIM_OCMode: 0,
                          TIM_OutputState: 0,
                          TIM_OutputNState: 0,
                          TIM_Pulse: 0,
                          TIM_OCPolarity: 0,
                          TIM_OCNPolarity: 0,
                          TIM_OCIdleState: 0,
                          TIM_OCNIdleState: 0,};
    TIM_OCStructInit(&mut TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = 0x60 as libc::c_int as uint16_t;
    if output as libc::c_int & TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        TIM_OCInitStructure.TIM_OutputNState = 0x4 as libc::c_int as uint16_t;
        TIM_OCInitStructure.TIM_OCNIdleState = 0 as libc::c_int as uint16_t;
        TIM_OCInitStructure.TIM_OCNPolarity =
            if output as libc::c_int & TIMER_OUTPUT_INVERTED as libc::c_int !=
                   0 {
                0x8 as libc::c_int as uint16_t as libc::c_int
            } else { 0 as libc::c_int as uint16_t as libc::c_int } as uint16_t
    } else {
        TIM_OCInitStructure.TIM_OutputState = 0x1 as libc::c_int as uint16_t;
        TIM_OCInitStructure.TIM_OCIdleState =
            0x100 as libc::c_int as uint16_t;
        TIM_OCInitStructure.TIM_OCPolarity =
            if output as libc::c_int & TIMER_OUTPUT_INVERTED as libc::c_int !=
                   0 {
                0x2 as libc::c_int as uint16_t as libc::c_int
            } else { 0 as libc::c_int as uint16_t as libc::c_int } as uint16_t
    }
    TIM_OCInitStructure.TIM_Pulse = value;
    timerOCInit(tim, channel, &mut TIM_OCInitStructure);
    timerOCPreloadConfig(tim, channel, 0x8 as libc::c_int as uint16_t);
}
#[no_mangle]
pub unsafe extern "C" fn pwmOutConfig(mut channel: *mut timerChannel_t,
                                      mut timerHardware:
                                          *const timerHardware_t,
                                      mut hz: uint32_t, mut period: uint16_t,
                                      mut value: uint16_t,
                                      mut inversion: uint8_t) {
    configTimeBase((*timerHardware).tim, period, hz);
    pwmOCConfig((*timerHardware).tim, (*timerHardware).channel, value,
                if inversion as libc::c_int != 0 {
                    ((*timerHardware).output as libc::c_int) ^
                        TIMER_OUTPUT_INVERTED as libc::c_int
                } else { (*timerHardware).output as libc::c_int } as uint8_t);
    TIM_CtrlPWMOutputs((*timerHardware).tim, ENABLE);
    TIM_Cmd((*timerHardware).tim, ENABLE);
    (*channel).ccr = timerChCCR(timerHardware);
    (*channel).tim = (*timerHardware).tim;
    ::core::ptr::write_volatile((*channel).ccr, 0 as libc::c_int as timCCR_t);
}
unsafe extern "C" fn pwmWriteUnused(mut index: uint8_t,
                                    mut value: libc::c_float) {
}
unsafe extern "C" fn pwmWriteStandard(mut index: uint8_t,
                                      mut value: libc::c_float) {
    /* TODO: move value to be a number between 0-1 (i.e. percent throttle from mixer) */
    ::core::ptr::write_volatile(motors[index as usize].channel.ccr,
                                lrintf(value *
                                           motors[index as usize].pulseScale +
                                           motors[index as usize].pulseOffset)
                                    as timCCR_t);
}
#[no_mangle]
pub unsafe extern "C" fn pwmWriteMotor(mut index: uint8_t,
                                       mut value: libc::c_float) {
    pwmWrite.expect("non-null function pointer")(index, value);
}
#[no_mangle]
pub unsafe extern "C" fn pwmShutdownPulsesForAllMotors(mut motorCount:
                                                           uint8_t) {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < motorCount as libc::c_int {
        // Set the compare register to 0, which stops the output pulsing if the timer overflows
        if !motors[index as usize].channel.ccr.is_null() {
            ::core::ptr::write_volatile(motors[index as usize].channel.ccr,
                                        0 as libc::c_int as timCCR_t)
        }
        index += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmDisableMotors() {
    pwmShutdownPulsesForAllMotors(4 as libc::c_int as uint8_t);
    pwmMotorsEnabled = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pwmEnableMotors() {
    /* check motors can be enabled */
    pwmMotorsEnabled =
        pwmWrite !=
            Some(pwmWriteUnused as
                     unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                         -> ());
}
#[no_mangle]
pub unsafe extern "C" fn pwmAreMotorsEnabled() -> bool {
    return pwmMotorsEnabled;
}
unsafe extern "C" fn pwmCompleteWriteUnused(mut motorCount: uint8_t) { }
unsafe extern "C" fn pwmCompleteOneshotMotorUpdate(mut motorCount: uint8_t) {
    let mut index: libc::c_int = 0 as libc::c_int;
    while index < motorCount as libc::c_int {
        if motors[index as usize].forceOverflow {
            timerForceOverflow(motors[index as usize].channel.tim);
        }
        // Set the compare register to 0, which stops the output pulsing if the timer overflows before the main loop completes again.
        // This compare register will be set to the output value on the next main loop.
        ::core::ptr::write_volatile(motors[index as usize].channel.ccr,
                                    0 as libc::c_int as timCCR_t);
        index += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn pwmCompleteMotorUpdate(mut motorCount: uint8_t) {
    pwmCompleteWrite.expect("non-null function pointer")(motorCount);
}
#[no_mangle]
pub unsafe extern "C" fn motorDevInit(mut motorConfig:
                                          *const motorDevConfig_t,
                                      mut idlePulse: uint16_t,
                                      mut motorCount: uint8_t) {
    memset(motors.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[pwmOutputPort_t; 4]>() as libc::c_ulong);
    let mut useUnsyncedPwm: bool = (*motorConfig).useUnsyncedPwm != 0;
    let mut sMin: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut sLen: libc::c_float = 0 as libc::c_int as libc::c_float;
    match (*motorConfig).motorPwmProtocol as libc::c_int {
        2 => { sMin = 42e-6f32; sLen = 42e-6f32 }
        3 => { sMin = 5e-6f32; sLen = 20e-6f32 }
        4 => {
            sMin = 0 as libc::c_int as libc::c_float;
            useUnsyncedPwm = 1 as libc::c_int != 0;
            idlePulse = 0 as libc::c_int as uint16_t
        }
        0 => {
            sMin = 1e-3f32;
            sLen = 1e-3f32;
            useUnsyncedPwm = 1 as libc::c_int != 0;
            idlePulse = 0 as libc::c_int as uint16_t
        }
        1 | _ => { sMin = 125e-6f32; sLen = 125e-6f32 }
    }
    if !isDshot {
        pwmWrite =
            Some(pwmWriteStandard as
                     unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                         -> ());
        pwmCompleteWrite =
            if useUnsyncedPwm as libc::c_int != 0 {
                Some(pwmCompleteWriteUnused as
                         unsafe extern "C" fn(_: uint8_t) -> ())
            } else {
                Some(pwmCompleteOneshotMotorUpdate as
                         unsafe extern "C" fn(_: uint8_t) -> ())
            }
    }
    let mut motorIndex: libc::c_int = 0 as libc::c_int;
    while motorIndex < 4 as libc::c_int &&
              motorIndex < motorCount as libc::c_int {
        let tag: ioTag_t = (*motorConfig).ioTags[motorIndex as usize];
        let mut timerHardware: *const timerHardware_t = timerGetByTag(tag);
        if timerHardware.is_null() {
            /* not enough motors initialised for the mixer or a break in the motors */
            pwmWrite =
                Some(pwmWriteUnused as
                         unsafe extern "C" fn(_: uint8_t, _: libc::c_float)
                             -> ());
            pwmCompleteWrite =
                Some(pwmCompleteWriteUnused as
                         unsafe extern "C" fn(_: uint8_t) -> ());
            /* TODO: block arming and add reason system cannot arm */
            return
        }
        motors[motorIndex as usize].io = IOGetByTag(tag);
        IOInit(motors[motorIndex as usize].io, OWNER_MOTOR,
               (motorIndex + 1 as libc::c_int) as uint8_t);
        IOConfigGPIO(motors[motorIndex as usize].io,
                     (GPIO_Mode_AF_PP as libc::c_int |
                          GPIO_Speed_2MHz as libc::c_int) as ioConfig_t);
        /* standard PWM outputs */
        // margin of safety is 4 periods when unsynced
        let pwmRateHz: libc::c_uint =
            if useUnsyncedPwm as libc::c_int != 0 {
                (*motorConfig).motorPwmRate as libc::c_int as libc::c_float
            } else {
                ceilf(1 as libc::c_int as libc::c_float /
                          ((sMin + sLen) * 4 as libc::c_int as libc::c_float))
            } as libc::c_uint;
        let clock: uint32_t = timerClock((*timerHardware).tim);
        /* used to find the desired timer frequency for max resolution */
        let prescaler: libc::c_uint =
            clock.wrapping_div(pwmRateHz).wrapping_add(0xffff as libc::c_int
                                                           as
                                                           libc::c_uint).wrapping_div(0x10000
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint); /* rounding up */
        let hz: uint32_t = clock.wrapping_div(prescaler);
        let period: libc::c_uint =
            if useUnsyncedPwm as libc::c_int != 0 {
                hz.wrapping_div(pwmRateHz)
            } else { 0xffff as libc::c_int as libc::c_uint };
        /*
            if brushed then it is the entire length of the period.
            TODO: this can be moved back to periodMin and periodLen
            once mixer outputs a 0..1 float value.
        */
        motors[motorIndex as usize].pulseScale =
            (if (*motorConfig).motorPwmProtocol as libc::c_int ==
                    PWM_TYPE_BRUSHED as libc::c_int {
                 period as libc::c_float
             } else { (sLen) * hz as libc::c_float }) / 1000.0f32;
        motors[motorIndex as usize].pulseOffset =
            sMin * hz as libc::c_float -
                motors[motorIndex as usize].pulseScale *
                    1000 as libc::c_int as libc::c_float;
        pwmOutConfig(&mut (*motors.as_mut_ptr().offset(motorIndex as
                                                           isize)).channel,
                     timerHardware, hz, period as uint16_t, idlePulse,
                     (*motorConfig).motorPwmInversion);
        let mut timerAlreadyUsed: bool = 0 as libc::c_int != 0;
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < motorIndex {
            if motors[i as usize].channel.tim ==
                   motors[motorIndex as usize].channel.tim {
                timerAlreadyUsed = 1 as libc::c_int != 0;
                break ;
            } else { i += 1 }
        }
        motors[motorIndex as usize].forceOverflow = !timerAlreadyUsed;
        motors[motorIndex as usize].enabled = 1 as libc::c_int != 0;
        motorIndex += 1
    }
    pwmMotorsEnabled = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn pwmGetMotors() -> *mut pwmOutputPort_t {
    return motors.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn isMotorProtocolDshot() -> bool { return isDshot; }
