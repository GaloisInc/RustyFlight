use ::libc;
extern "C" {
    #[no_mangle]
    static mut pidConfig_System: pidConfig_t;
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
    // gyro alignment
    // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    // Gyro sample divider
    // gyro DLPF setting
    // gyro 32khz DLPF setting
    // Lowpass primary/secondary
    // Gyro calibration duration in 1/100 second
    // bandpass quality factor, 100 for steep sided bandpass
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
// Limit to the accumulated error
pub type pidConfig_t = pidConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pidConfig_s {
    pub pid_process_denom: uint8_t,
    pub runaway_takeoff_prevention: uint8_t,
    pub runaway_takeoff_deactivate_delay: uint16_t,
    pub runaway_takeoff_deactivate_throttle: uint8_t,
}
pub type gyroConfig_t = gyroConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroConfig_s {
    pub gyro_align: uint8_t,
    pub gyroMovementCalibrationThreshold: uint8_t,
    pub gyro_sync_denom: uint8_t,
    pub gyro_hardware_lpf: uint8_t,
    pub gyro_32khz_hardware_lpf: uint8_t,
    pub gyro_high_fsr: uint8_t,
    pub gyro_use_32khz: uint8_t,
    pub gyro_to_use: uint8_t,
    pub gyro_lowpass_hz: uint16_t,
    pub gyro_lowpass2_hz: uint16_t,
    pub gyro_soft_notch_hz_1: uint16_t,
    pub gyro_soft_notch_cutoff_1: uint16_t,
    pub gyro_soft_notch_hz_2: uint16_t,
    pub gyro_soft_notch_cutoff_2: uint16_t,
    pub gyro_offset_yaw: int16_t,
    pub checkOverflow: uint8_t,
    pub gyro_lowpass_type: uint8_t,
    pub gyro_lowpass2_type: uint8_t,
    pub yaw_spin_recovery: uint8_t,
    pub yaw_spin_threshold: int16_t,
    pub gyroCalibrationDuration: uint16_t,
    pub dyn_notch_quality: uint8_t,
    pub dyn_notch_width_percent: uint8_t,
}
#[inline]
unsafe extern "C" fn pidConfigMutable() -> *mut pidConfig_t {
    return &mut pidConfig_System;
}
#[inline]
unsafe extern "C" fn gyroConfigMutable() -> *mut gyroConfig_t {
    return &mut gyroConfig_System;
}
// Processing denominator for PID controller vs gyro sampling rate
// off, on - enables pidsum runaway disarm logic
// delay in ms for "in-flight" conditions before deactivation (successful flight)
// minimum throttle percent required during deactivation phase
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
// Motolab target supports 2 different type of boards Tornado / Cyclone.
#[no_mangle]
pub unsafe extern "C" fn targetConfiguration() {
    (*gyroConfigMutable()).gyro_sync_denom = 4 as libc::c_int as uint8_t;
    (*pidConfigMutable()).pid_process_denom = 1 as libc::c_int as uint8_t;
}
