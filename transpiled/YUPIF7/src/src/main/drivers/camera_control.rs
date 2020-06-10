use ::libc;
extern "C" {
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn pwmOutConfig(channel: *mut timerChannel_t,
                    timerHardware: *const timerHardware_t, hz: uint32_t,
                    period: uint16_t, value: uint16_t, inversion: uint8_t);
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut resumeRefreshAt: timeUs_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Stream_TypeDef {
    pub CR: uint32_t,
    pub NDTR: uint32_t,
    pub PAR: uint32_t,
    pub M0AR: uint32_t,
    pub M1AR: uint32_t,
    pub FCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
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
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
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
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
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
// measured in 10 mV steps
// measured 100 Ohm steps
// Output is inverted externally
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_1 {
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
    pub dmaRef: *mut DMA_Stream_TypeDef,
    pub dmaChannel: uint32_t,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Stream_TypeDef,
    pub dmaTimUPChannel: uint32_t,
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
                                 C2RustUnnamed_0{ptr:
                                                     &pgResetTemplate_cameraControlConfig
                                                         as
                                                         *const cameraControlConfig_t
                                                         as
                                                         *mut libc::c_void,},};
            init
        }
    };
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
                                  ioTag:
                                      ((1 as libc::c_int + 1 as libc::c_int)
                                           << 4 as libc::c_int |
                                           7 as libc::c_int) as ioTag_t,
                                  inverted: 0 as libc::c_int as uint8_t,};
        init
    };
static mut cameraControlRuntime: C2RustUnnamed_1 =
    C2RustUnnamed_1{enabled: false,
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
                       (0x2 as libc::c_uint |
                            (0 as libc::c_uint) << 2 as libc::c_int |
                            (0 as libc::c_uint) << 5 as libc::c_int) as
                           ioConfig_t, (*timerHardware).alternateFunction);
        pwmOutConfig(&mut cameraControlRuntime.channel, timerHardware,
                     (216 as libc::c_int * 1000000 as libc::c_int) as
                         uint32_t, 128 as libc::c_int as uint16_t,
                     0 as libc::c_int as uint16_t,
                     cameraControlRuntime.inverted);
        cameraControlRuntime.period = 128 as libc::c_int as uint32_t;
        ::core::ptr::write_volatile(cameraControlRuntime.channel.ccr,
                                    cameraControlRuntime.period);
        cameraControlRuntime.enabled = 1 as libc::c_int != 0
    } else if !(CAMERA_CONTROL_MODE_SOFTWARE_PWM as libc::c_int as
                    libc::c_uint ==
                    (*cameraControlConfig()).mode as libc::c_uint) {
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
    } else if !(CAMERA_CONTROL_MODE_SOFTWARE_PWM as libc::c_int as
                    libc::c_uint ==
                    (*cameraControlConfig()).mode as libc::c_uint) {
        (CAMERA_CONTROL_MODE_DAC as libc::c_int as libc::c_uint) ==
            (*cameraControlConfig()).mode as libc::c_uint;
    };
}
