use ::libc;
extern "C" {
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
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IORelease(io: IO_t);
    // unimplemented
    #[no_mangle]
    fn IOGetOwner(io: IO_t) -> resourceOwner_e;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type timeDelta_t = int32_t;
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rangefinderDev_s {
    pub delayMs: timeMs_t,
    pub maxRangeCm: int16_t,
    pub detectionConeDeciDegrees: int16_t,
    pub detectionConeExtendedDeciDegrees: int16_t,
    pub init: rangefinderOpInitFuncPtr,
    pub update: rangefinderOpStartFuncPtr,
    pub read: rangefinderOpReadFuncPtr,
}
pub type rangefinderOpReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> int32_t>;
pub type rangefinderOpStartFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> ()>;
pub type rangefinderOpInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut rangefinderDev_s) -> ()>;
pub type rangefinderDev_t = rangefinderDev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sonarConfig_s {
    pub triggerTag: ioTag_t,
    pub echoTag: ioTag_t,
}
pub type sonarConfig_t = sonarConfig_s;
// these are full detection cone angles, maximum tilt is half of this
// detection cone angle as in device spec
// device spec is conservative, in practice have slightly larger detection cone
// function pointers
// in practice 45 degrees seems to work well
/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */
static mut hcsr04SonarPulseTravelTime: timeDelta_t = 0 as libc::c_int;
static mut lastMeasurementReceivedAt: timeMs_t = 0;
static mut lastCalculatedDistance: int32_t = -(1 as libc::c_int);
static mut lastMeasurementStartedAt: timeMs_t = 0 as libc::c_int as timeMs_t;
static mut echoIO: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut triggerIO: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
#[no_mangle]
pub unsafe extern "C" fn hcsr04_extiHandler(mut cb: *mut extiCallbackRec_t) {
    static mut timing_start: timeUs_t = 0;
    if IORead(echoIO) as libc::c_int != 0 as libc::c_int {
        timing_start = micros()
    } else {
        let timing_stop: timeUs_t = micros();
        if timing_stop > timing_start {
            ::core::ptr::write_volatile(&mut lastMeasurementReceivedAt as
                                            *mut timeMs_t, millis());
            ::core::ptr::write_volatile(&mut hcsr04SonarPulseTravelTime as
                                            *mut timeDelta_t,
                                        timing_stop.wrapping_sub(timing_start)
                                            as timeDelta_t)
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn hcsr04_init(mut dev: *mut rangefinderDev_t) { }
/*
 * Start a range reading
 * Called periodically by the scheduler
 * Measurement reading is done asynchronously, using interrupt
 */
#[no_mangle]
pub unsafe extern "C" fn hcsr04_start_reading() {
    IOHi(triggerIO);
    delayMicroseconds(11 as libc::c_int as timeUs_t);
    IOLo(triggerIO);
}
#[no_mangle]
pub unsafe extern "C" fn hcsr04_update(mut dev: *mut rangefinderDev_t) {
    let timeNowMs: timeMs_t = millis();
    // the firing interval of the trigger signal should be greater than 60ms
    // to avoid interference between consecutive measurements
    if timeNowMs >
           lastMeasurementStartedAt.wrapping_add(60 as libc::c_int as
                                                     libc::c_uint) {
        // We should have a valid measurement within 60ms of trigger
        if lastMeasurementReceivedAt.wrapping_sub(lastMeasurementStartedAt) <=
               60 as libc::c_int as libc::c_uint {
            // The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
            // The ping travels out and back, so to find the distance of the
            // object we take half of the distance traveled.
            // 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59
            ::core::ptr::write_volatile(&mut lastCalculatedDistance as
                                            *mut int32_t,
                                        hcsr04SonarPulseTravelTime /
                                            59 as libc::c_int);
            if lastCalculatedDistance > 400 as libc::c_int {
                ::core::ptr::write_volatile(&mut lastCalculatedDistance as
                                                *mut int32_t,
                                            -(1 as libc::c_int))
            }
        } else {
            // No measurement within reasonable time - indicate failure
            ::core::ptr::write_volatile(&mut lastCalculatedDistance as
                                            *mut int32_t, -(2 as libc::c_int))
        }
        // Trigger a new measurement
        lastMeasurementStartedAt = timeNowMs;
        hcsr04_start_reading();
    };
}
/* *
 * Get the distance that was measured by the last pulse, in centimeters.
 */
#[no_mangle]
pub unsafe extern "C" fn hcsr04_get_distance(mut dev: *mut rangefinderDev_t)
 -> int32_t {
    return lastCalculatedDistance;
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
#[no_mangle]
pub unsafe extern "C" fn hcsr04Detect(mut dev: *mut rangefinderDev_t,
                                      mut rangefinderHardwarePins:
                                          *const sonarConfig_t) -> bool {
    let mut detected: bool = 0 as libc::c_int != 0;
    triggerIO = IOGetByTag((*rangefinderHardwarePins).triggerTag);
    echoIO = IOGetByTag((*rangefinderHardwarePins).echoTag);
    if IOGetOwner(triggerIO) as libc::c_uint !=
           OWNER_FREE as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    if IOGetOwner(echoIO) as libc::c_uint !=
           OWNER_FREE as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    // trigger pin
    IOInit(triggerIO, OWNER_SONAR_TRIGGER, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(triggerIO,
                 (0x1 as libc::c_uint |
                      (0 as libc::c_uint) << 2 as libc::c_int |
                      (0 as libc::c_uint) << 5 as libc::c_int) as ioConfig_t);
    IOLo(triggerIO);
    delay(100 as libc::c_int as timeMs_t);
    // echo pin
    IOInit(echoIO, OWNER_SONAR_ECHO, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(echoIO,
                 (0 as libc::c_uint | (0 as libc::c_uint) << 2 as libc::c_int
                      | (0 as libc::c_uint) << 5 as libc::c_int) as
                     ioConfig_t);
    // HC-SR04 echo line should be low by default and should return a response pulse when triggered
    if IORead(echoIO) as libc::c_int == 0 as libc::c_int {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 5 as libc::c_int && !detected {
            let mut requestTime: timeMs_t = millis();
            hcsr04_start_reading();
            while millis().wrapping_sub(requestTime) <
                      60 as libc::c_int as libc::c_uint {
                if !(IORead(echoIO) as libc::c_int == 1 as libc::c_int) {
                    continue ;
                }
                detected = 1 as libc::c_int != 0;
                break ;
            }
            i += 1
        }
    }
    if detected {
        // Hardware detected - configure the driver
        (*dev).delayMs = 100 as libc::c_int as timeMs_t;
        (*dev).maxRangeCm = 400 as libc::c_int as int16_t;
        (*dev).detectionConeDeciDegrees = 300 as libc::c_int as int16_t;
        (*dev).detectionConeExtendedDeciDegrees =
            450 as libc::c_int as int16_t;
        (*dev).init =
            Some(hcsr04_init as
                     unsafe extern "C" fn(_: *mut rangefinderDev_t) -> ());
        (*dev).update =
            Some(hcsr04_update as
                     unsafe extern "C" fn(_: *mut rangefinderDev_t) -> ());
        (*dev).read =
            Some(hcsr04_get_distance as
                     unsafe extern "C" fn(_: *mut rangefinderDev_t)
                         -> int32_t);
        return 1 as libc::c_int != 0
    } else {
        // Not detected - free resources
        IORelease(triggerIO);
        IORelease(echoIO);
        return 0 as libc::c_int != 0
    };
}
