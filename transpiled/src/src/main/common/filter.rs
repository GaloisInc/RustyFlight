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
    pub type filter_s;
    #[no_mangle]
    fn sqrtf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn sin_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn cos_approx(x: libc::c_float) -> libc::c_float;
}
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type filter_t = filter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pt1Filter_s {
    pub state: libc::c_float,
    pub k: libc::c_float,
}
pub type pt1Filter_t = pt1Filter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct slewFilter_s {
    pub state: libc::c_float,
    pub slewLimit: libc::c_float,
    pub threshold: libc::c_float,
}
pub type slewFilter_t = slewFilter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct biquadFilter_s {
    pub b0: libc::c_float,
    pub b1: libc::c_float,
    pub b2: libc::c_float,
    pub a1: libc::c_float,
    pub a2: libc::c_float,
    pub x1: libc::c_float,
    pub x2: libc::c_float,
    pub y1: libc::c_float,
    pub y2: libc::c_float,
}
/* this holds the data required to update samples thru a filter */
pub type biquadFilter_t = biquadFilter_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct laggedMovingAverage_s {
    pub movingWindowIndex: uint16_t,
    pub windowSize: uint16_t,
    pub movingSum: libc::c_float,
    pub buf: *mut libc::c_float,
    pub primed: bool,
}
pub type laggedMovingAverage_t = laggedMovingAverage_s;
pub type biquadFilterType_e = libc::c_uint;
pub const FILTER_BPF: biquadFilterType_e = 2;
pub const FILTER_NOTCH: biquadFilterType_e = 1;
pub const FILTER_LPF: biquadFilterType_e = 0;
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
// SITL (software in the loop) simulator
// use simulatior's attitude directly
// disable this if wants to test AHRS algorithm
//#define SIMULATOR_ACC_SYNC
//#define SIMULATOR_GYRO_SYNC
//#define SIMULATOR_IMU_SYNC
//#define SIMULATOR_GYROPID_SYNC
// file name to save config
//#define USE_SOFTSERIAL1
//#define USE_SOFTSERIAL2
// I think SITL don't need this
// suppress 'no pins defined' warning
// belows are internal stuff
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
/* quality factor - 2nd order butterworth*/
// NULL filter
#[no_mangle]
pub unsafe extern "C" fn nullFilterApply(mut filter: *mut filter_t,
                                         mut input: libc::c_float)
 -> libc::c_float {
    return input;
}
// PT1 Low Pass filter
#[no_mangle]
pub unsafe extern "C" fn pt1FilterGain(mut f_cut: uint16_t,
                                       mut dT: libc::c_float)
 -> libc::c_float {
    let mut RC: libc::c_float =
        1 as libc::c_int as libc::c_float /
            (2 as libc::c_int as libc::c_float * 3.14159265358979323846f32 *
                 f_cut as libc::c_int as libc::c_float);
    return dT / (RC + dT);
}
#[no_mangle]
pub unsafe extern "C" fn pt1FilterInit(mut filter: *mut pt1Filter_t,
                                       mut k: libc::c_float) {
    (*filter).state = 0.0f32;
    (*filter).k = k;
}
#[no_mangle]
pub unsafe extern "C" fn pt1FilterUpdateCutoff(mut filter: *mut pt1Filter_t,
                                               mut k: libc::c_float) {
    (*filter).k = k;
}
#[no_mangle]
pub unsafe extern "C" fn pt1FilterApply(mut filter: *mut pt1Filter_t,
                                        mut input: libc::c_float)
 -> libc::c_float {
    (*filter).state =
        (*filter).state + (*filter).k * (input - (*filter).state);
    return (*filter).state;
}
// Slew filter with limit
#[no_mangle]
pub unsafe extern "C" fn slewFilterInit(mut filter: *mut slewFilter_t,
                                        mut slewLimit: libc::c_float,
                                        mut threshold: libc::c_float) {
    (*filter).state = 0.0f32;
    (*filter).slewLimit = slewLimit;
    (*filter).threshold = threshold;
}
#[no_mangle]
pub unsafe extern "C" fn slewFilterApply(mut filter: *mut slewFilter_t,
                                         mut input: libc::c_float)
 -> libc::c_float {
    if (*filter).state >= (*filter).threshold {
        if input >= (*filter).state - (*filter).slewLimit {
            (*filter).state = input
        }
    } else if (*filter).state <= -(*filter).threshold {
        if input <= (*filter).state + (*filter).slewLimit {
            (*filter).state = input
        }
    } else { (*filter).state = input }
    return (*filter).state;
}
// get notch filter Q given center frequency (f0) and lower cutoff frequency (f1)
// Q = f0 / (f2 - f1) ; f2 = f0^2 / f1
#[no_mangle]
pub unsafe extern "C" fn filterGetNotchQ(mut centerFreq: libc::c_float,
                                         mut cutoffFreq: libc::c_float)
 -> libc::c_float {
    return centerFreq * cutoffFreq /
               (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}
/* sets up a biquad Filter */
#[no_mangle]
pub unsafe extern "C" fn biquadFilterInitLPF(mut filter: *mut biquadFilter_t,
                                             mut filterFreq: libc::c_float,
                                             mut refreshRate: uint32_t) {
    biquadFilterInit(filter, filterFreq, refreshRate, 1.0f32 / sqrtf(2.0f32),
                     FILTER_LPF);
}
#[no_mangle]
pub unsafe extern "C" fn biquadFilterInit(mut filter: *mut biquadFilter_t,
                                          mut filterFreq: libc::c_float,
                                          mut refreshRate: uint32_t,
                                          mut Q: libc::c_float,
                                          mut filterType:
                                              biquadFilterType_e) {
    // setup variables
    let omega: libc::c_float =
        2.0f32 * 3.14159265358979323846f32 * filterFreq *
            refreshRate as libc::c_float * 0.000001f32;
    let sn: libc::c_float = sin_approx(omega);
    let cs: libc::c_float = cos_approx(omega);
    let alpha: libc::c_float = sn / (2.0f32 * Q);
    let mut b0: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut b1: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut b2: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut a0: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut a1: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut a2: libc::c_float = 0 as libc::c_int as libc::c_float;
    match filterType as libc::c_uint {
        0 => {
            // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
        // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
            b0 = (1 as libc::c_int as libc::c_float - cs) * 0.5f32;
            b1 = 1 as libc::c_int as libc::c_float - cs;
            b2 = (1 as libc::c_int as libc::c_float - cs) * 0.5f32;
            a0 = 1 as libc::c_int as libc::c_float + alpha;
            a1 = -(2 as libc::c_int) as libc::c_float * cs;
            a2 = 1 as libc::c_int as libc::c_float - alpha
        }
        1 => {
            b0 = 1 as libc::c_int as libc::c_float;
            b1 = -(2 as libc::c_int) as libc::c_float * cs;
            b2 = 1 as libc::c_int as libc::c_float;
            a0 = 1 as libc::c_int as libc::c_float + alpha;
            a1 = -(2 as libc::c_int) as libc::c_float * cs;
            a2 = 1 as libc::c_int as libc::c_float - alpha
        }
        2 => {
            b0 = alpha;
            b1 = 0 as libc::c_int as libc::c_float;
            b2 = -alpha;
            a0 = 1 as libc::c_int as libc::c_float + alpha;
            a1 = -(2 as libc::c_int) as libc::c_float * cs;
            a2 = 1 as libc::c_int as libc::c_float - alpha
        }
        _ => { }
    }
    // precompute the coefficients
    (*filter).b0 = b0 / a0;
    (*filter).b1 = b1 / a0;
    (*filter).b2 = b2 / a0;
    (*filter).a1 = a1 / a0;
    (*filter).a2 = a2 / a0;
    // zero initial samples
    (*filter).x2 = 0 as libc::c_int as libc::c_float;
    (*filter).x1 = (*filter).x2;
    (*filter).y2 = 0 as libc::c_int as libc::c_float;
    (*filter).y1 = (*filter).y2;
}
#[no_mangle]
pub unsafe extern "C" fn biquadFilterUpdate(mut filter: *mut biquadFilter_t,
                                            mut filterFreq: libc::c_float,
                                            mut refreshRate: uint32_t,
                                            mut Q: libc::c_float,
                                            mut filterType:
                                                biquadFilterType_e) {
    // backup state
    let mut x1: libc::c_float = (*filter).x1;
    let mut x2: libc::c_float = (*filter).x2;
    let mut y1: libc::c_float = (*filter).y1;
    let mut y2: libc::c_float = (*filter).y2;
    biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);
    // restore state
    (*filter).x1 = x1;
    (*filter).x2 = x2;
    (*filter).y1 = y1;
    (*filter).y2 = y2;
}
#[no_mangle]
pub unsafe extern "C" fn biquadFilterUpdateLPF(mut filter:
                                                   *mut biquadFilter_t,
                                               mut filterFreq: libc::c_float,
                                               mut refreshRate: uint32_t) {
    biquadFilterUpdate(filter, filterFreq, refreshRate,
                       1.0f32 / sqrtf(2.0f32), FILTER_LPF);
}
/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
#[no_mangle]
pub unsafe extern "C" fn biquadFilterApplyDF1(mut filter: *mut biquadFilter_t,
                                              mut input: libc::c_float)
 -> libc::c_float {
    /* compute result */
    let result: libc::c_float =
        (*filter).b0 * input + (*filter).b1 * (*filter).x1 +
            (*filter).b2 * (*filter).x2 - (*filter).a1 * (*filter).y1 -
            (*filter).a2 * (*filter).y2;
    /* shift x1 to x2, input to x1 */
    (*filter).x2 = (*filter).x1;
    (*filter).x1 = input;
    /* shift y1 to y2, result to y1 */
    (*filter).y2 = (*filter).y1;
    (*filter).y1 = result;
    return result;
}
/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
#[no_mangle]
pub unsafe extern "C" fn biquadFilterApply(mut filter: *mut biquadFilter_t,
                                           mut input: libc::c_float)
 -> libc::c_float {
    let result: libc::c_float = (*filter).b0 * input + (*filter).x1;
    (*filter).x1 =
        (*filter).b1 * input - (*filter).a1 * result + (*filter).x2;
    (*filter).x2 = (*filter).b2 * input - (*filter).a2 * result;
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn laggedMovingAverageInit(mut filter:
                                                     *mut laggedMovingAverage_t,
                                                 mut windowSize: uint16_t,
                                                 mut buf:
                                                     *mut libc::c_float) {
    (*filter).movingWindowIndex = 0 as libc::c_int as uint16_t;
    (*filter).windowSize = windowSize;
    (*filter).buf = buf;
    (*filter).primed = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn laggedMovingAverageUpdate(mut filter:
                                                       *mut laggedMovingAverage_t,
                                                   mut input: libc::c_float)
 -> libc::c_float {
    (*filter).movingSum -=
        *(*filter).buf.offset((*filter).movingWindowIndex as isize);
    *(*filter).buf.offset((*filter).movingWindowIndex as isize) = input;
    (*filter).movingSum += input;
    (*filter).movingWindowIndex = (*filter).movingWindowIndex.wrapping_add(1);
    if (*filter).movingWindowIndex as libc::c_int ==
           (*filter).windowSize as libc::c_int {
        (*filter).movingWindowIndex = 0 as libc::c_int as uint16_t;
        (*filter).primed = 1 as libc::c_int != 0
    }
    let denom: uint16_t =
        if (*filter).primed as libc::c_int != 0 {
            (*filter).windowSize as libc::c_int
        } else { (*filter).movingWindowIndex as libc::c_int } as uint16_t;
    return (*filter).movingSum / denom as libc::c_int as libc::c_float;
}
