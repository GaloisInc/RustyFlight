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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterInit(filter: *mut biquadFilter_t,
                        filterFreq: libc::c_float, refreshRate: uint32_t,
                        Q: libc::c_float, filterType: biquadFilterType_e);
    #[no_mangle]
    fn biquadFilterUpdate(filter: *mut biquadFilter_t,
                          filterFreq: libc::c_float, refreshRate: uint32_t,
                          Q: libc::c_float, filterType: biquadFilterType_e);
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn filterGetNotchQ(centerFreq: libc::c_float, cutoffFreq: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn cos_approx(x: libc::c_float) -> libc::c_float;
    #[no_mangle]
    static mut gyro: gyro_t;
    #[no_mangle]
    static mut gyroConfig_System: gyroConfig_t;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn fmax(_: libc::c_double, _: libc::c_double) -> libc::c_double;
    /* *
   * @brief Floating-point vector multiplication.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
    #[no_mangle]
    fn arm_mult_f32(pSrcA: *mut float32_t, pSrcB: *mut float32_t,
                    pDst: *mut float32_t, blockSize: uint32_t);
    #[no_mangle]
    fn arm_rfft_fast_init_f32(S: *mut arm_rfft_fast_instance_f32,
                              fftLen: uint16_t) -> arm_status;
    /* *
   * @brief  Floating-point complex magnitude
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
    #[no_mangle]
    fn arm_cmplx_mag_f32(pSrc: *mut float32_t, pDst: *mut float32_t,
                         numSamples: uint32_t);
    #[no_mangle]
    fn stage_rfft_f32(S: *mut arm_rfft_fast_instance_f32, p: *mut float32_t,
                      pOut: *mut float32_t);
    #[no_mangle]
    fn arm_cfft_radix8by2_f32(S: *mut arm_cfft_instance_f32,
                              p1: *mut float32_t);
    #[no_mangle]
    fn arm_cfft_radix8by4_f32(S: *mut arm_cfft_instance_f32,
                              p1: *mut float32_t);
    #[no_mangle]
    fn arm_radix8_butterfly_f32(pSrc: *mut float32_t, fftLen: uint16_t,
                                pCoef: *const float32_t,
                                twidCoefModifier: uint16_t);
    #[no_mangle]
    fn arm_bitreversal_32(pSrc: *mut uint32_t, bitRevLen: uint16_t,
                          pBitRevTable: *const uint16_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed = 38;
pub const DEBUG_RTH: C2RustUnnamed = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed = 36;
pub const DEBUG_USB: C2RustUnnamed = 35;
pub const DEBUG_CURRENT: C2RustUnnamed = 34;
pub const DEBUG_SDIO: C2RustUnnamed = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed = 28;
pub const DEBUG_FPORT: C2RustUnnamed = 27;
pub const DEBUG_SBUS: C2RustUnnamed = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed = 16;
pub const DEBUG_FFT: C2RustUnnamed = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed = 12;
pub const DEBUG_STACK: C2RustUnnamed = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed = 3;
pub const DEBUG_BATTERY: C2RustUnnamed = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed = 1;
pub const DEBUG_NONE: C2RustUnnamed = 0;
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
pub type biquadFilterType_e = libc::c_uint;
pub const FILTER_BPF: biquadFilterType_e = 2;
pub const FILTER_NOTCH: biquadFilterType_e = 1;
pub const FILTER_LPF: biquadFilterType_e = 0;
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub type gyro_t = gyro_s;
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
pub type gyroConfig_t = gyroConfig_s;
// gyro alignment
// people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
// Gyro sample divider
// gyro DLPF setting
// gyro 32khz DLPF setting
// Lowpass primary/secondary
// Gyro calibration duration in 1/100 second
// bandpass quality factor, 100 for steep sided bandpass
/* *****************************************************************************
 * @file     arm_math.h
 * @brief    Public header file for CMSIS DSP LibraryU
 * @version  V1.5.3
 * @date     10. January 2018
 ******************************************************************************/
/*
 * Copyright (c) 2010-2018 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* *
   \mainpage CMSIS DSP Software Library
   *
   * Introduction
   * ------------
   *
   * This user manual describes the CMSIS DSP software library,
   * a suite of common signal processing functions for use on Cortex-M processor based devices.
   *
   * The library is divided into a number of functions each covering a specific category:
   * - Basic math functions
   * - Fast math functions
   * - Complex math functions
   * - Filters
   * - Matrix functions
   * - Transforms
   * - Motor control functions
   * - Statistical functions
   * - Support functions
   * - Interpolation functions
   *
   * The library has separate functions for operating on 8-bit integers, 16-bit integers,
   * 32-bit integer and 32-bit floating-point values.
   *
   * Using the Library
   * ------------
   *
   * The library installer contains prebuilt versions of the libraries in the <code>Lib</code> folder.
   * - arm_cortexM7lfdp_math.lib (Cortex-M7, Little endian, Double Precision Floating Point Unit)
   * - arm_cortexM7bfdp_math.lib (Cortex-M7, Big endian, Double Precision Floating Point Unit)
   * - arm_cortexM7lfsp_math.lib (Cortex-M7, Little endian, Single Precision Floating Point Unit)
   * - arm_cortexM7bfsp_math.lib (Cortex-M7, Big endian and Single Precision Floating Point Unit on)
   * - arm_cortexM7l_math.lib (Cortex-M7, Little endian)
   * - arm_cortexM7b_math.lib (Cortex-M7, Big endian)
   * - arm_cortexM4lf_math.lib (Cortex-M4, Little endian, Floating Point Unit)
   * - arm_cortexM4bf_math.lib (Cortex-M4, Big endian, Floating Point Unit)
   * - arm_cortexM4l_math.lib (Cortex-M4, Little endian)
   * - arm_cortexM4b_math.lib (Cortex-M4, Big endian)
   * - arm_cortexM3l_math.lib (Cortex-M3, Little endian)
   * - arm_cortexM3b_math.lib (Cortex-M3, Big endian)
   * - arm_cortexM0l_math.lib (Cortex-M0 / Cortex-M0+, Little endian)
   * - arm_cortexM0b_math.lib (Cortex-M0 / Cortex-M0+, Big endian)
   * - arm_ARMv8MBLl_math.lib (Armv8-M Baseline, Little endian)
   * - arm_ARMv8MMLl_math.lib (Armv8-M Mainline, Little endian)
   * - arm_ARMv8MMLlfsp_math.lib (Armv8-M Mainline, Little endian, Single Precision Floating Point Unit)
   * - arm_ARMv8MMLld_math.lib (Armv8-M Mainline, Little endian, DSP instructions)
   * - arm_ARMv8MMLldfsp_math.lib (Armv8-M Mainline, Little endian, DSP instructions, Single Precision Floating Point Unit)
   *
   * The library functions are declared in the public file <code>arm_math.h</code> which is placed in the <code>Include</code> folder.
   * Simply include this file and link the appropriate library in the application and begin calling the library functions. The Library supports single
   * public header file <code> arm_math.h</code> for Cortex-M cores with little endian and big endian. Same header file will be used for floating point unit(FPU) variants.
   * Define the appropriate preprocessor macro ARM_MATH_CM7 or ARM_MATH_CM4 or ARM_MATH_CM3 or
   * ARM_MATH_CM0 or ARM_MATH_CM0PLUS depending on the target processor in the application.
   * For Armv8-M cores define preprocessor macro ARM_MATH_ARMV8MBL or ARM_MATH_ARMV8MML.
   * Set preprocessor macro __DSP_PRESENT if Armv8-M Mainline core supports DSP instructions.
   * 
   *
   * Examples
   * --------
   *
   * The library ships with a number of examples which demonstrate how to use the library functions.
   *
   * Toolchain Support
   * ------------
   *
   * The library has been developed and tested with MDK version 5.14.0.0
   * The library is being tested in GCC and IAR toolchains and updates on this activity will be made available shortly.
   *
   * Building the Library
   * ------------
   *
   * The library installer contains a project file to rebuild libraries on MDK toolchain in the <code>CMSIS\\DSP_Lib\\Source\\ARM</code> folder.
   * - arm_cortexM_math.uvprojx
   *
   *
   * The libraries can be built by opening the arm_cortexM_math.uvprojx project in MDK-ARM, selecting a specific target, and defining the optional preprocessor macros detailed above.
   *
   * Preprocessor Macros
   * ------------
   *
   * Each library project have different preprocessor macros.
   *
   * - UNALIGNED_SUPPORT_DISABLE:
   *
   * Define macro UNALIGNED_SUPPORT_DISABLE, If the silicon does not support unaligned memory access
   *
   * - ARM_MATH_BIG_ENDIAN:
   *
   * Define macro ARM_MATH_BIG_ENDIAN to build the library for big endian targets. By default library builds for little endian targets.
   *
   * - ARM_MATH_MATRIX_CHECK:
   *
   * Define macro ARM_MATH_MATRIX_CHECK for checking on the input and output sizes of matrices
   *
   * - ARM_MATH_ROUNDING:
   *
   * Define macro ARM_MATH_ROUNDING for rounding on support functions
   *
   * - ARM_MATH_CMx:
   *
   * Define macro ARM_MATH_CM4 for building the library on Cortex-M4 target, ARM_MATH_CM3 for building library on Cortex-M3 target
   * and ARM_MATH_CM0 for building library on Cortex-M0 target, ARM_MATH_CM0PLUS for building library on Cortex-M0+ target, and
   * ARM_MATH_CM7 for building the library on cortex-M7.
   *
   * - ARM_MATH_ARMV8MxL:
   *
   * Define macro ARM_MATH_ARMV8MBL for building the library on Armv8-M Baseline target, ARM_MATH_ARMV8MML for building library
   * on Armv8-M Mainline target.
   *
   * - __FPU_PRESENT:
   *
   * Initialize macro __FPU_PRESENT = 1 when building on FPU supported Targets. Enable this macro for floating point libraries.
   *
   * - __DSP_PRESENT:
   *
   * Initialize macro __DSP_PRESENT = 1 when Armv8-M Mainline core supports DSP instructions.
   *
   * <hr>
   * CMSIS-DSP in ARM::CMSIS Pack
   * -----------------------------
   *
   * The following files relevant to CMSIS-DSP are present in the <b>ARM::CMSIS</b> Pack directories:
   * |File/Folder                   |Content                                                                 |
   * |------------------------------|------------------------------------------------------------------------|
   * |\b CMSIS\\Documentation\\DSP  | This documentation                                                     |
   * |\b CMSIS\\DSP_Lib             | Software license agreement (license.txt)                               |
   * |\b CMSIS\\DSP_Lib\\Examples   | Example projects demonstrating the usage of the library functions      |
   * |\b CMSIS\\DSP_Lib\\Source     | Source files for rebuilding the library                                |
   *
   * <hr>
   * Revision History of CMSIS-DSP
   * ------------
   * Please refer to \ref ChangeLog_pg.
   *
   * Copyright Notice
   * ------------
   *
   * Copyright (C) 2010-2015 Arm Limited. All rights reserved.
   */
/* *
 * @defgroup groupMath Basic Math Functions
 */
/* *
 * @defgroup groupFastMath Fast Math Functions
 * This set of functions provides a fast approximation to sine, cosine, and square root.
 * As compared to most of the other functions in the CMSIS math library, the fast math functions
 * operate on individual values and not arrays.
 * There are separate functions for Q15, Q31, and floating-point data.
 *
 */
/* *
 * @defgroup groupCmplxMath Complex Math Functions
 * This set of functions operates on complex data vectors.
 * The data in the complex arrays is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * In the API functions, the number of samples in a complex array refers
 * to the number of complex values; the array contains twice this number of
 * real values.
 */
/* *
 * @defgroup groupFilters Filtering Functions
 */
/* *
 * @defgroup groupMatrix Matrix Functions
 *
 * This set of functions provides basic matrix math operations.
 * The functions operate on matrix data structures.  For example,
 * the type
 * definition for the floating-point matrix structure is shown
 * below:
 * <pre>
 *     typedef struct
 *     {
 *       uint16_t numRows;     // number of rows of the matrix.
 *       uint16_t numCols;     // number of columns of the matrix.
 *       float32_t *pData;     // points to the data of the matrix.
 *     } arm_matrix_instance_f32;
 * </pre>
 * There are similar definitions for Q15 and Q31 data types.
 *
 * The structure specifies the size of the matrix and then points to
 * an array of data.  The array is of size <code>numRows X numCols</code>
 * and the values are arranged in row order.  That is, the
 * matrix element (i, j) is stored at:
 * <pre>
 *     pData[i*numCols + j]
 * </pre>
 *
 * \par Init Functions
 * There is an associated initialization function for each type of matrix
 * data structure.
 * The initialization function sets the values of the internal structure fields.
 * Refer to the function <code>arm_mat_init_f32()</code>, <code>arm_mat_init_q31()</code>
 * and <code>arm_mat_init_q15()</code> for floating-point, Q31 and Q15 types,  respectively.
 *
 * \par
 * Use of the initialization function is optional. However, if initialization function is used
 * then the instance structure cannot be placed into a const data section.
 * To place the instance structure in a const data
 * section, manually initialize the data structure.  For example:
 * <pre>
 * <code>arm_matrix_instance_f32 S = {nRows, nColumns, pData};</code>
 * <code>arm_matrix_instance_q31 S = {nRows, nColumns, pData};</code>
 * <code>arm_matrix_instance_q15 S = {nRows, nColumns, pData};</code>
 * </pre>
 * where <code>nRows</code> specifies the number of rows, <code>nColumns</code>
 * specifies the number of columns, and <code>pData</code> points to the
 * data array.
 *
 * \par Size Checking
 * By default all of the matrix functions perform size checking on the input and
 * output matrices. For example, the matrix addition function verifies that the
 * two input matrices and the output matrix all have the same number of rows and
 * columns. If the size check fails the functions return:
 * <pre>
 *     ARM_MATH_SIZE_MISMATCH
 * </pre>
 * Otherwise the functions return
 * <pre>
 *     ARM_MATH_SUCCESS
 * </pre>
 * There is some overhead associated with this matrix size checking.
 * The matrix size checking is enabled via the \#define
 * <pre>
 *     ARM_MATH_MATRIX_CHECK
 * </pre>
 * within the library project settings.  By default this macro is defined
 * and size checking is enabled. By changing the project settings and
 * undefining this macro size checking is eliminated and the functions
 * run a bit faster. With size checking disabled the functions always
 * return <code>ARM_MATH_SUCCESS</code>.
 */
/* *
 * @defgroup groupTransforms Transform Functions
 */
/* *
 * @defgroup groupController Controller Functions
 */
/* *
 * @defgroup groupStats Statistics Functions
 */
/* *
 * @defgroup groupSupport Support Functions
 */
/* *
 * @defgroup groupInterpolation Interpolation Functions
 * These functions perform 1- and 2-dimensional interpolation of data.
 * Linear interpolation is used for 1-dimensional data and
 * bilinear interpolation is used for 2-dimensional data.
 */
/* *
 * @defgroup groupExamples Examples
 */
/* Compiler specific diagnostic adjustment */
/* disable NVIC and Systick functions */
/* enable NVIC and Systick functions */
/* *
   * @brief Macros required for reciprocal calculation in Normalized LMS
   */
/* *
   * @brief Macros required for SINE and COSINE Fast math approximations
   */
/* *
   * @brief Macros required for SINE and COSINE Controller functions
   */
  /* 1.31(q31) Fixed value of 2/360 */
  /* -1 to +1 is divided into 360 values so total spacing is (2/360) */
/* *
   * @brief Macro for Unaligned Support
   */
/* #ifndef UNALIGNED_SUPPORT_DISABLE */
/* *
   * @brief Error status returned by some functions in the library.
   */
pub type arm_status = libc::c_int;
/* *< Test Failed  */
/* *< Generated by matrix inversion if the input matrix is singular and cannot be inverted. */
pub const ARM_MATH_TEST_FAILURE: arm_status = -6;
/* *< Not-a-number (NaN) or infinity is generated */
pub const ARM_MATH_SINGULAR: arm_status = -5;
/* *< Size of matrices is not compatible with the operation. */
pub const ARM_MATH_NANINF: arm_status = -4;
/* *< Length of data buffer is incorrect */
pub const ARM_MATH_SIZE_MISMATCH: arm_status = -3;
/* *< One or more arguments are incorrect */
pub const ARM_MATH_LENGTH_ERROR: arm_status = -2;
/* *< No error */
pub const ARM_MATH_ARGUMENT_ERROR: arm_status = -1;
pub const ARM_MATH_SUCCESS: arm_status = 0;
/* *
   * @brief 32-bit floating-point type definition.
   */
pub type float32_t = libc::c_float;
/* *
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct arm_cfft_instance_f32 {
    pub fftLen: uint16_t,
    pub pTwiddle: *const float32_t,
    pub pBitRevTable: *const uint16_t,
    pub bitRevLength: uint16_t,
}
/* *
   * @brief Instance structure for the floating-point RFFT/RIFFT function.
   */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct arm_rfft_fast_instance_f32 {
    pub Sint: arm_cfft_instance_f32,
    pub fftLenRFFT: uint16_t,
    pub pTwiddleRFFT: *mut float32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyroAnalyseState_s {
    pub sampleCount: uint8_t,
    pub maxSampleCount: uint8_t,
    pub maxSampleCountRcp: libc::c_float,
    pub oversampledGyroAccumulator: [libc::c_float; 3],
    pub gyroBandpassFilter: [biquadFilter_t; 3],
    pub circularBufferIdx: uint8_t,
    pub downsampledGyroData: [[libc::c_float; 32]; 3],
    pub updateTicks: uint8_t,
    pub updateStep: uint8_t,
    pub updateAxis: uint8_t,
    pub fftInstance: arm_rfft_fast_instance_f32,
    pub fftData: [libc::c_float; 32],
    pub rfftData: [libc::c_float; 32],
    pub detectedFrequencyFilter: [biquadFilter_t; 3],
    pub centerFreq: [uint16_t; 3],
}
pub type gyroAnalyseState_t = gyroAnalyseState_s;
pub const STEP_COUNT: C2RustUnnamed_0 = 7;
pub const STEP_HANNING: C2RustUnnamed_0 = 6;
pub const STEP_UPDATE_FILTERS: C2RustUnnamed_0 = 5;
pub const STEP_CALC_FREQUENCIES: C2RustUnnamed_0 = 4;
pub const STEP_ARM_CMPLX_MAG_F32: C2RustUnnamed_0 = 3;
pub const STEP_STAGE_RFFT_F32: C2RustUnnamed_0 = 2;
pub const STEP_BITREVERSAL: C2RustUnnamed_0 = 1;
pub const STEP_ARM_CFFT_F32: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn gyroConfig() -> *const gyroConfig_t {
    return &mut gyroConfig_System;
}
static mut fftSamplingRateHz: uint16_t = 0;
// centre frequency of bandpass that constrains input to FFT
static mut fftBpfHz: uint16_t = 0;
// Hz per bin
static mut fftResolution: libc::c_float = 0.;
// maximum notch centre frequency limited by Nyquist
static mut dynNotchMaxCentreHz: uint16_t = 0;
static mut fftBinOffset: uint8_t = 0;
// Hanning window, see https://en.wikipedia.org/wiki/Window_function#Hann_.28Hanning.29_window
static mut hanningWindow: [libc::c_float; 32] = [0.; 32];
static mut dynamicNotchCutoff: libc::c_float = 0.;
#[no_mangle]
pub unsafe extern "C" fn gyroDataAnalyseInit(mut targetLooptimeUs: uint32_t) {
    let gyroLoopRateHz: libc::c_int =
        lrintf(1.0f32 / targetLooptimeUs as libc::c_float * 1e6f32) as
            libc::c_int;
    // If we get at least 3 samples then use the default FFT sample frequency
    // otherwise we need to calculate a FFT sample frequency to ensure we get 3 samples (gyro loops < 4K)
    fftSamplingRateHz =
        ({
             let mut _a: libc::c_int = gyroLoopRateHz / 3 as libc::c_int;
             let mut _b: libc::c_int = 1333 as libc::c_int;
             if _a < _b { _a } else { _b }
         }) as uint16_t;
    fftBpfHz =
        (fftSamplingRateHz as libc::c_int / 4 as libc::c_int) as uint16_t;
    fftResolution =
        fftSamplingRateHz as libc::c_float /
            32 as libc::c_int as libc::c_float;
    dynNotchMaxCentreHz =
        (fftSamplingRateHz as libc::c_int / 2 as libc::c_int) as uint16_t;
    // Calculate the FFT bin offset to try and get the lowest bin used
    // in the center calc close to 90hz
    // > 1333hz = 1, 889hz (2.67K) = 2, 666hz (2K) = 3
    fftBinOffset =
        ({
             let mut _a: libc::c_int = 1 as libc::c_int;
             let mut _b: libc::c_long =
                 lrintf(90 as libc::c_int as libc::c_float / fftResolution -
                            1.5f32);
             if _a as libc::c_long > _b { _a as libc::c_long } else { _b }
         }) as uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 32 as libc::c_int {
        hanningWindow[i as usize] =
            0.5f32 -
                0.5f32 *
                    cos_approx(2 as libc::c_int as libc::c_float *
                                   3.14159265358979323846f32 *
                                   i as libc::c_float /
                                   (32 as libc::c_int - 1 as libc::c_int) as
                                       libc::c_float);
        i += 1
    }
    dynamicNotchCutoff =
        (100.0f32 -
             (*gyroConfig()).dyn_notch_width_percent as libc::c_int as
                 libc::c_float) / 100 as libc::c_int as libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn gyroDataAnalyseStateInit(mut state:
                                                      *mut gyroAnalyseState_t,
                                                  mut targetLooptimeUs:
                                                      uint32_t) {
    // initialise even if FEATURE_DYNAMIC_FILTER not set, since it may be set later
    gyroDataAnalyseInit(targetLooptimeUs);
    let samplingFrequency: uint16_t =
        (1000000 as libc::c_int as
             libc::c_uint).wrapping_div(targetLooptimeUs) as uint16_t;
    (*state).maxSampleCount =
        (samplingFrequency as libc::c_int / fftSamplingRateHz as libc::c_int)
            as uint8_t;
    (*state).maxSampleCountRcp =
        1.0f32 / (*state).maxSampleCount as libc::c_int as libc::c_float;
    arm_rfft_fast_init_f32(&mut (*state).fftInstance,
                           32 as libc::c_int as uint16_t);
    // recalculation of filters takes 4 calls per axis => each filter gets updated every DYN_NOTCH_CALC_TICKS calls
    // at 4khz gyro loop rate this means 4khz / 4 / 3 = 333Hz => update every 3ms
    // for gyro rate > 16kHz, we have update frequency of 1kHz => 1ms
    let looptime: libc::c_float =
        ({
             let mut _a: libc::c_uint =
                 (1000000 as
                      libc::c_uint).wrapping_div(fftSamplingRateHz as
                                                     libc::c_uint);
             let mut _b: libc::c_uint =
                 targetLooptimeUs.wrapping_mul((3 as libc::c_int *
                                                    4 as libc::c_int) as
                                                   libc::c_uint);
             if _a > _b { _a } else { _b }
         }) as libc::c_float;
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        // any init value
        (*state).centerFreq[axis as usize] = 200 as libc::c_int as uint16_t;
        biquadFilterInit(&mut *(*state).gyroBandpassFilter.as_mut_ptr().offset(axis
                                                                                   as
                                                                                   isize),
                         fftBpfHz as libc::c_float,
                         (1000000 as libc::c_int /
                              fftSamplingRateHz as libc::c_int) as uint32_t,
                         0.01f32 *
                             (*gyroConfig()).dyn_notch_quality as libc::c_int
                                 as libc::c_float, FILTER_BPF);
        biquadFilterInitLPF(&mut *(*state).detectedFrequencyFilter.as_mut_ptr().offset(axis
                                                                                           as
                                                                                           isize),
                            60 as libc::c_int as libc::c_float,
                            looptime as uint32_t);
        axis += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn gyroDataAnalysePush(mut state:
                                                 *mut gyroAnalyseState_t,
                                             axis: libc::c_int,
                                             sample: libc::c_float) {
    (*state).oversampledGyroAccumulator[axis as usize] += sample;
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
// max for F3 targets
// accumulator for oversampled data => no aliasing and less noise
// filter for downsampled accumulated gyro
// downsampled gyro data circular buffer for frequency analysis
// update state machine step information
/*
 * Collect gyro data, to be analysed in gyroDataAnalyseUpdate function
 */
#[no_mangle]
pub unsafe extern "C" fn gyroDataAnalyse(mut state: *mut gyroAnalyseState_t,
                                         mut notchFilterDyn:
                                             *mut biquadFilter_t) {
    // samples should have been pushed by `gyroDataAnalysePush`
    // if gyro sampling is > 1kHz, accumulate multiple samples
    (*state).sampleCount = (*state).sampleCount.wrapping_add(1);
    // this runs at 1kHz
    if (*state).sampleCount as libc::c_int ==
           (*state).maxSampleCount as libc::c_int {
        (*state).sampleCount = 0 as libc::c_int as uint8_t;
        // calculate mean value of accumulated samples
        let mut axis: libc::c_int = 0 as libc::c_int;
        while axis < 3 as libc::c_int {
            let mut sample: libc::c_float =
                (*state).oversampledGyroAccumulator[axis as usize] *
                    (*state).maxSampleCountRcp;
            sample =
                biquadFilterApply(&mut *(*state).gyroBandpassFilter.as_mut_ptr().offset(axis
                                                                                            as
                                                                                            isize),
                                  sample);
            (*state).downsampledGyroData[axis as
                                             usize][(*state).circularBufferIdx
                                                        as usize] = sample;
            if axis == 0 as libc::c_int {
                if debugMode as libc::c_int == DEBUG_FFT as libc::c_int {
                    debug[2 as libc::c_int as usize] =
                        lrintf(sample) as int16_t
                }
            }
            (*state).oversampledGyroAccumulator[axis as usize] =
                0 as libc::c_int as libc::c_float;
            axis += 1
        }
        (*state).circularBufferIdx =
            (((*state).circularBufferIdx as libc::c_int + 1 as libc::c_int) %
                 32 as libc::c_int) as uint8_t;
        // We need DYN_NOTCH_CALC_TICKS tick to update all axis with newly sampled value
        (*state).updateTicks =
            (3 as libc::c_int * 4 as libc::c_int) as uint8_t
    }
    // calculate FFT and update filters
    if (*state).updateTicks as libc::c_int > 0 as libc::c_int {
        gyroDataAnalyseUpdate(state, notchFilterDyn);
        (*state).updateTicks = (*state).updateTicks.wrapping_sub(1)
    };
}
/*
 * Analyse last gyro data from the last FFT_WINDOW_SIZE milliseconds
 */
unsafe extern "C" fn gyroDataAnalyseUpdate(mut state: *mut gyroAnalyseState_t,
                                           mut notchFilterDyn:
                                               *mut biquadFilter_t) {
    let mut Sint: *mut arm_cfft_instance_f32 = &mut (*state).fftInstance.Sint;
    let mut startTime: uint32_t = 0 as libc::c_int as uint32_t;
    if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
        startTime = micros()
    }
    if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
        debug[0 as libc::c_int as usize] = (*state).updateStep as int16_t
    }
    let mut current_block_97: u64;
    match (*state).updateStep as libc::c_int {
        0 => {
            match 32 as libc::c_int / 2 as libc::c_int {
                16 => {
                    // 16us
                    arm_cfft_radix8by2_f32(Sint,
                                           (*state).fftData.as_mut_ptr());
                }
                32 => {
                    // 35us
                    arm_cfft_radix8by4_f32(Sint,
                                           (*state).fftData.as_mut_ptr());
                }
                64 => {
                    // 70us
                    arm_radix8_butterfly_f32((*state).fftData.as_mut_ptr(),
                                             (32 as libc::c_int /
                                                  2 as libc::c_int) as
                                                 uint16_t, (*Sint).pTwiddle,
                                             1 as libc::c_int as uint16_t);
                }
                _ => { }
            }
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
            current_block_97 = 2432552683059077439;
        }
        1 => {
            // 6us
            arm_bitreversal_32((*state).fftData.as_mut_ptr() as *mut uint32_t,
                               (*Sint).bitRevLength, (*Sint).pBitRevTable);
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
            (*state).updateStep = (*state).updateStep.wrapping_add(1);
            current_block_97 = 652864300344834934;
        }
        2 => { current_block_97 = 652864300344834934; }
        3 => {
            // 8us
            arm_cmplx_mag_f32((*state).rfftData.as_mut_ptr(),
                              (*state).fftData.as_mut_ptr(),
                              (32 as libc::c_int / 2 as libc::c_int) as
                                  uint32_t);
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[2 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
            (*state).updateStep = (*state).updateStep.wrapping_add(1);
            current_block_97 = 1924505913685386279;
        }
        4 => { current_block_97 = 1924505913685386279; }
        5 => {
            // 7us
            // calculate cutoffFreq and notch Q, update notch filter
            let cutoffFreq: libc::c_float =
                fmax(((*state).centerFreq[(*state).updateAxis as usize] as
                          libc::c_int as libc::c_float * dynamicNotchCutoff)
                         as libc::c_double,
                     105 as libc::c_int as libc::c_double) as libc::c_float;
            let notchQ: libc::c_float =
                filterGetNotchQ((*state).centerFreq[(*state).updateAxis as
                                                        usize] as
                                    libc::c_float, cutoffFreq);
            biquadFilterUpdate(&mut *notchFilterDyn.offset((*state).updateAxis
                                                               as isize),
                               (*state).centerFreq[(*state).updateAxis as
                                                       usize] as
                                   libc::c_float, gyro.targetLooptime, notchQ,
                               FILTER_NOTCH);
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
            (*state).updateAxis =
                (((*state).updateAxis as libc::c_int + 1 as libc::c_int) %
                     3 as libc::c_int) as uint8_t;
            (*state).updateStep = (*state).updateStep.wrapping_add(1);
            current_block_97 = 11959176018832582207;
        }
        6 => { current_block_97 = 11959176018832582207; }
        _ => { current_block_97 = 2432552683059077439; }
    }
    match current_block_97 {
        1924505913685386279 =>
        // 13us
            // calculate FFT centreFreq
        {
            let mut fftSum: libc::c_float = 0 as libc::c_int as libc::c_float;
            let mut fftWeightedSum: libc::c_float =
                0 as libc::c_int as libc::c_float;
            let mut fftIncreasing: bool = 0 as libc::c_int != 0;
            // iterate over fft data and calculate weighted indices
            let mut i: libc::c_int =
                1 as libc::c_int + fftBinOffset as libc::c_int;
            while i < 32 as libc::c_int / 2 as libc::c_int {
                let data: libc::c_float = (*state).fftData[i as usize];
                let prevData: libc::c_float =
                    (*state).fftData[(i - 1 as libc::c_int) as usize];
                if fftIncreasing as libc::c_int != 0 ||
                       data > prevData * 2 as libc::c_int as libc::c_float {
                    let mut cubedData: libc::c_float = data * data * data;
                    // add previous bin before first rise
                    if !fftIncreasing {
                        cubedData += prevData * prevData * prevData;
                        fftIncreasing = 1 as libc::c_int != 0
                    }
                    fftSum += cubedData;
                    // calculate weighted index starting at 1, not 0
                    fftWeightedSum +=
                        cubedData * (i + 1 as libc::c_int) as libc::c_float
                }
                i += 1
            }
            // get weighted center of relevant frequency range (this way we have a better resolution than 31.25Hz)
            // if no peak, go to highest point to minimise delay
            let mut centerFreq: libc::c_float =
                dynNotchMaxCentreHz as libc::c_float;
            let mut fftMeanIndex: libc::c_float =
                0 as libc::c_int as libc::c_float;
            if fftSum > 0 as libc::c_int as libc::c_float {
                // idx was shifted by 1 to start at 1, not 0
                fftMeanIndex =
                    fftWeightedSum / fftSum -
                        1 as libc::c_int as libc::c_float;
                // the index points at the center frequency of each bin so index 0 is actually 16.125Hz
                centerFreq =
                    constrain((fftMeanIndex * fftResolution) as libc::c_int,
                              125 as libc::c_int,
                              dynNotchMaxCentreHz as libc::c_int) as
                        libc::c_float
            }
            centerFreq =
                biquadFilterApply(&mut *(*state).detectedFrequencyFilter.as_mut_ptr().offset((*state).updateAxis
                                                                                                 as
                                                                                                 isize),
                                  centerFreq);
            centerFreq =
                constrain(centerFreq as libc::c_int, 125 as libc::c_int,
                          dynNotchMaxCentreHz as libc::c_int) as
                    libc::c_float;
            (*state).centerFreq[(*state).updateAxis as usize] =
                centerFreq as uint16_t;
            if (*state).updateAxis as libc::c_int == 0 as libc::c_int {
                if debugMode as libc::c_int == DEBUG_FFT as libc::c_int {
                    debug[3 as libc::c_int as usize] =
                        lrintf(fftMeanIndex *
                                   100 as libc::c_int as libc::c_float) as
                            int16_t
                }
            }
            if debugMode as libc::c_int == DEBUG_FFT_FREQ as libc::c_int {
                debug[(*state).updateAxis as usize] =
                    (*state).centerFreq[(*state).updateAxis as usize] as
                        int16_t
            }
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
        }
        652864300344834934 => {
            // 14us
            // this does not work in place => fftData AND rfftData needed
            stage_rfft_f32(&mut (*state).fftInstance,
                           (*state).fftData.as_mut_ptr(),
                           (*state).rfftData.as_mut_ptr());
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
        }
        11959176018832582207 => {
            // 5us
            // apply hanning window to gyro samples and store result in fftData
            // hanning starts and ends with 0, could be skipped for minor speed improvement
            let ringBufIdx: uint8_t =
                (32 as libc::c_int -
                     (*state).circularBufferIdx as libc::c_int) as uint8_t;
            arm_mult_f32(&mut *(*(*state).downsampledGyroData.as_mut_ptr().offset((*state).updateAxis
                                                                                      as
                                                                                      isize)).as_mut_ptr().offset((*state).circularBufferIdx
                                                                                                                      as
                                                                                                                      isize),
                         &mut *hanningWindow.as_mut_ptr().offset(0 as
                                                                     libc::c_int
                                                                     as
                                                                     isize),
                         &mut *(*state).fftData.as_mut_ptr().offset(0 as
                                                                        libc::c_int
                                                                        as
                                                                        isize),
                         ringBufIdx as uint32_t);
            if (*state).circularBufferIdx as libc::c_int > 0 as libc::c_int {
                arm_mult_f32(&mut *(*(*state).downsampledGyroData.as_mut_ptr().offset((*state).updateAxis
                                                                                          as
                                                                                          isize)).as_mut_ptr().offset(0
                                                                                                                          as
                                                                                                                          libc::c_int
                                                                                                                          as
                                                                                                                          isize),
                             &mut *hanningWindow.as_mut_ptr().offset(ringBufIdx
                                                                         as
                                                                         isize),
                             &mut *(*state).fftData.as_mut_ptr().offset(ringBufIdx
                                                                            as
                                                                            isize),
                             (*state).circularBufferIdx as uint32_t);
            }
            if debugMode as libc::c_int == DEBUG_FFT_TIME as libc::c_int {
                debug[1 as libc::c_int as usize] =
                    micros().wrapping_sub(startTime) as int16_t
            }
        }
        _ => { }
    }
    (*state).updateStep =
        (((*state).updateStep as libc::c_int + 1 as libc::c_int) %
             STEP_COUNT as libc::c_int) as uint8_t;
}
// USE_GYRO_DATA_ANALYSE
