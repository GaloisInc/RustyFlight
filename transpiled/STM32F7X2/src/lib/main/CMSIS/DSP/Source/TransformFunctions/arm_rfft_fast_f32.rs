use ::libc;
extern "C" {
    #[no_mangle]
    fn arm_cfft_f32(S: *const arm_cfft_instance_f32, p1: *mut float32_t,
                    ifftFlag: uint8_t, bitReverseFlag: uint8_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type float32_t = libc::c_float;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct arm_cfft_instance_f32 {
    pub fftLen: uint16_t,
    pub pTwiddle: *const float32_t,
    pub pBitRevTable: *const uint16_t,
    pub bitRevLength: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct arm_rfft_fast_instance_f32 {
    pub Sint: arm_cfft_instance_f32,
    pub fftLenRFFT: uint16_t,
    pub pTwiddleRFFT: *mut float32_t,
}
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_rfft_f32.c
 * Description:  RFFT & RIFFT Floating point process function
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
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
#[no_mangle]
pub unsafe extern "C" fn stage_rfft_f32(mut S:
                                            *mut arm_rfft_fast_instance_f32,
                                        mut p: *mut float32_t,
                                        mut pOut: *mut float32_t) {
    let mut k: uint32_t = 0; /* Loop Counter                     */
    let mut twR: float32_t = 0.; /* RFFT Twiddle coefficients        */
    let mut twI: float32_t = 0.; /* Points to RFFT Twiddle factors   */
    let mut pCoeff: *mut float32_t =
        (*S).pTwiddleRFFT; /* increasing pointer               */
    let mut pA: *mut float32_t = p; /* decreasing pointer               */
    let mut pB: *mut float32_t = p; /* temporary variables              */
    let mut xAR: float32_t = 0.; /* temporary variables              */
    let mut xAI: float32_t = 0.; /* temporary variables              */
    let mut xBR: float32_t = 0.;
    let mut xBI: float32_t = 0.;
    let mut t1a: float32_t = 0.;
    let mut t1b: float32_t = 0.;
    let mut p0: float32_t = 0.;
    let mut p1: float32_t = 0.;
    let mut p2: float32_t = 0.;
    let mut p3: float32_t = 0.;
    k = ((*S).Sint.fftLen as libc::c_int - 1 as libc::c_int) as uint32_t;
    /* Pack first and last sample of the frequency domain together */
    xBR = *pB.offset(0 as libc::c_int as isize);
    xBI = *pB.offset(1 as libc::c_int as isize);
    xAR = *pA.offset(0 as libc::c_int as isize);
    xAI = *pA.offset(1 as libc::c_int as isize);
    let fresh0 = pCoeff;
    pCoeff = pCoeff.offset(1);
    twR = *fresh0;
    let fresh1 = pCoeff;
    pCoeff = pCoeff.offset(1);
    twI = *fresh1;
    // U1 = XA(1) + XB(1); % It is real
    t1a = xBR + xAR;
    // U2 = XB(1) - XA(1); % It is imaginary
    t1b = xBI + xAI;
    // real(tw * (xB - xA)) = twR * (xBR - xAR) - twI * (xBI - xAI);
   // imag(tw * (xB - xA)) = twI * (xBR - xAR) + twR * (xBI - xAI);
    let fresh2 = pOut;
    pOut = pOut.offset(1);
    *fresh2 = 0.5f32 * (t1a + t1b);
    let fresh3 = pOut;
    pOut = pOut.offset(1);
    *fresh3 = 0.5f32 * (t1a - t1b);
    // XA(1) = 1/2*( U1 - imag(U2) +  i*( U1 +imag(U2) ));
    pB =
        p.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(k) as isize);
    pA = pA.offset(2 as libc::c_int as isize);
    loop  {
        /*
         function X = my_split_rfft(X, ifftFlag)
         % X is a series of real numbers
         L  = length(X);
         XC = X(1:2:end) +i*X(2:2:end);
         XA = fft(XC);
         XB = conj(XA([1 end:-1:2]));
         TW = i*exp(-2*pi*i*[0:L/2-1]/L).';
         for l = 2:L/2
            XA(l) = 1/2 * (XA(l) + XB(l) + TW(l) * (XB(l) - XA(l)));
         end
         XA(1) = 1/2* (XA(1) + XB(1) + TW(1) * (XB(1) - XA(1))) + i*( 1/2*( XA(1) + XB(1) + i*( XA(1) - XB(1))));
         X = XA;
      */
        xBI = *pB.offset(1 as libc::c_int as isize);
        xBR = *pB.offset(0 as libc::c_int as isize);
        xAR = *pA.offset(0 as libc::c_int as isize);
        xAI = *pA.offset(1 as libc::c_int as isize);
        let fresh4 = pCoeff;
        pCoeff = pCoeff.offset(1);
        twR = *fresh4;
        let fresh5 = pCoeff;
        pCoeff = pCoeff.offset(1);
        twI = *fresh5;
        t1a = xBR - xAR;
        t1b = xBI + xAI;
        // real(tw * (xB - xA)) = twR * (xBR - xAR) - twI * (xBI - xAI);
      // imag(tw * (xB - xA)) = twI * (xBR - xAR) + twR * (xBI - xAI);
        p0 = twR * t1a; //xAR
        p1 = twI * t1a; //xAI
        p2 = twR * t1b;
        p3 = twI * t1b;
        let fresh6 = pOut;
        pOut = pOut.offset(1);
        *fresh6 = 0.5f32 * (xAR + xBR + p0 + p3);
        let fresh7 = pOut;
        pOut = pOut.offset(1);
        *fresh7 = 0.5f32 * (xAI - xBI + p1 - p2);
        pA = pA.offset(2 as libc::c_int as isize);
        pB = pB.offset(-(2 as libc::c_int as isize));
        k = k.wrapping_sub(1);
        if !(k > 0 as libc::c_uint) { break ; }
    };
}
/* Prepares data for inverse cfft */
#[no_mangle]
pub unsafe extern "C" fn merge_rfft_f32(mut S:
                                            *mut arm_rfft_fast_instance_f32,
                                        mut p: *mut float32_t,
                                        mut pOut: *mut float32_t) {
    let mut k: uint32_t = 0; /* Loop Counter                     */
    let mut twR: float32_t = 0.; /* RFFT Twiddle coefficients        */
    let mut twI: float32_t = 0.; /* Points to RFFT Twiddle factors   */
    let mut pCoeff: *mut float32_t =
        (*S).pTwiddleRFFT; /* increasing pointer               */
    let mut pA: *mut float32_t = p; /* decreasing pointer               */
    let mut pB: *mut float32_t = p; /* temporary variables              */
    let mut xAR: float32_t = 0.; /* temporary variables              */
    let mut xAI: float32_t = 0.;
    let mut xBR: float32_t = 0.;
    let mut xBI: float32_t = 0.;
    let mut t1a: float32_t = 0.;
    let mut t1b: float32_t = 0.;
    let mut r: float32_t = 0.;
    let mut s: float32_t = 0.;
    let mut t: float32_t = 0.;
    let mut u: float32_t = 0.;
    k = ((*S).Sint.fftLen as libc::c_int - 1 as libc::c_int) as uint32_t;
    xAR = *pA.offset(0 as libc::c_int as isize);
    xAI = *pA.offset(1 as libc::c_int as isize);
    pCoeff = pCoeff.offset(2 as libc::c_int as isize);
    let fresh8 = pOut;
    pOut = pOut.offset(1);
    *fresh8 = 0.5f32 * (xAR + xAI);
    let fresh9 = pOut;
    pOut = pOut.offset(1);
    *fresh9 = 0.5f32 * (xAR - xAI);
    pB =
        p.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(k) as isize);
    pA = pA.offset(2 as libc::c_int as isize);
    while k > 0 as libc::c_uint {
        /* G is half of the frequency complex spectrum */
      //for k = 2:N
      //    Xk(k) = 1/2 * (G(k) + conj(G(N-k+2)) + Tw(k)*( G(k) - conj(G(N-k+2))));
        xBI = *pB.offset(1 as libc::c_int as isize);
        xBR = *pB.offset(0 as libc::c_int as isize);
        xAR = *pA.offset(0 as libc::c_int as isize);
        xAI = *pA.offset(1 as libc::c_int as isize);
        let fresh10 = pCoeff;
        pCoeff = pCoeff.offset(1);
        twR = *fresh10;
        let fresh11 = pCoeff;
        pCoeff = pCoeff.offset(1);
        twI = *fresh11;
        t1a = xAR - xBR;
        t1b = xAI + xBI;
        r = twR * t1a;
        s = twI * t1b;
        t = twI * t1a;
        u = twR * t1b;
        // real(tw * (xA - xB)) = twR * (xAR - xBR) - twI * (xAI - xBI);
      // imag(tw * (xA - xB)) = twI * (xAR - xBR) + twR * (xAI - xBI);
        let fresh12 = pOut; //xAR
        pOut = pOut.offset(1); //xAI
        *fresh12 = 0.5f32 * (xAR + xBR - r - s);
        let fresh13 = pOut;
        pOut = pOut.offset(1);
        *fresh13 = 0.5f32 * (xAI - xBI + t - u);
        pA = pA.offset(2 as libc::c_int as isize);
        pB = pB.offset(-(2 as libc::c_int as isize));
        k = k.wrapping_sub(1)
    };
}
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
/* *< No error */
/* *< One or more arguments are incorrect */
/* *< Length of data buffer is incorrect */
/* *< Size of matrices is not compatible with the operation. */
/* *< Not-a-number (NaN) or infinity is generated */
/* *< Generated by matrix inversion if the input matrix is singular and cannot be inverted. */
/* *< Test Failed  */
/* *
   * @brief 8-bit fractional data type in 1.7 format.
   */
/* *
   * @brief 16-bit fractional data type in 1.15 format.
   */
/* *
   * @brief 32-bit fractional data type in 1.31 format.
   */
/* *
   * @brief 64-bit fractional data type in 1.63 format.
   */
/* *
   * @brief 32-bit floating-point type definition.
   */
/* *
   * @brief 64-bit floating-point type definition.
   */
/* *
   * @brief definition to read/write two 16 bit values.
   */
/* !defined (ARM_MATH_DSP) */
/* *
   * @brief definition to pack four 8 bit values.
   */
/* *
   * @brief Clips Q63 to Q31 values.
   */
/* *
   * @brief Clips Q63 to Q15 values.
   */
/* *
   * @brief Clips Q31 to Q7 values.
   */
/* *
   * @brief Clips Q31 to Q15 values.
   */
/* *
   * @brief Multiplies 32 X 64 and returns 32 bit result in 2.30 format.
   */
/* *
   * @brief Function to Calculates 1/in (reciprocal) value of Q31 Data type.
   */
/* Convert input sample to 1.31 format */
/* calculation of index for initial approximated Val */
/* 1.31 with exp 1 */
/* calculation of reciprocal value */
    /* running approximation for two iterations */
/*      1.31 with exp 1 */
      /* out = (q31_t) (((q63_t) out * tempVal) >> 30); */
/* write output */
/* return num of signbits of out = 1/in value */
/* *
   * @brief Function to Calculates 1/in (reciprocal) value of Q15 Data type.
   */
/* Convert input sample to 1.15 format */
/* calculation of index for initial approximated Val */
/*      1.15 with exp 1  */
/* calculation of reciprocal value */
    /* running approximation for two iterations */
/*      1.15 with exp 1 */
/* out = clip_q31_to_q15(((q31_t) out * tempVal) >> 14); */
/* write output */
/* return num of signbits of out = 1/in value */
/*
 * @brief C custom defined intrinsic function for M3 and M0 processors
 */
/* !defined (ARM_MATH_DSP) */
/* *
   * @brief Instance structure for the Q7 FIR filter.
   */
/* *< number of filter coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *
   * @brief Instance structure for the Q15 FIR filter.
   */
/* *< number of filter coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *
   * @brief Instance structure for the Q31 FIR filter.
   */
/* *< number of filter coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *
   * @brief Instance structure for the floating-point FIR filter.
   */
/* *< number of filter coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *
   * @brief Processing function for the Q7 FIR filter.
   * @param[in]  S          points to an instance of the Q7 FIR filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the Q7 FIR filter.
   * @param[in,out] S          points to an instance of the Q7 FIR structure.
   * @param[in]     numTaps    Number of filter coefficients in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of samples that are processed.
   */
/* *
   * @brief Processing function for the Q15 FIR filter.
   * @param[in]  S          points to an instance of the Q15 FIR structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the fast Q15 FIR filter for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q15 FIR filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the Q15 FIR filter.
   * @param[in,out] S          points to an instance of the Q15 FIR filter structure.
   * @param[in]     numTaps    Number of filter coefficients in the filter. Must be even and greater than or equal to 4.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of samples that are processed at a time.
   * @return The function returns ARM_MATH_SUCCESS if initialization was successful or ARM_MATH_ARGUMENT_ERROR if
   * <code>numTaps</code> is not a supported value.
   */
/* *
   * @brief Processing function for the Q31 FIR filter.
   * @param[in]  S          points to an instance of the Q31 FIR filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the fast Q31 FIR filter for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q31 FIR structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the Q31 FIR filter.
   * @param[in,out] S          points to an instance of the Q31 FIR structure.
   * @param[in]     numTaps    Number of filter coefficients in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of samples that are processed at a time.
   */
/* *
   * @brief Processing function for the floating-point FIR filter.
   * @param[in]  S          points to an instance of the floating-point FIR structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the floating-point FIR filter.
   * @param[in,out] S          points to an instance of the floating-point FIR filter structure.
   * @param[in]     numTaps    Number of filter coefficients in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of samples that are processed at a time.
   */
/* *
   * @brief Instance structure for the Q15 Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< Points to the array of state coefficients.  The array is of length 4*numStages. */
/* *< Points to the array of coefficients.  The array is of length 5*numStages. */
/* *< Additional shift, in bits, applied to each output sample. */
/* *
   * @brief Instance structure for the Q31 Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< Points to the array of state coefficients.  The array is of length 4*numStages. */
/* *< Points to the array of coefficients.  The array is of length 5*numStages. */
/* *< Additional shift, in bits, applied to each output sample. */
/* *
   * @brief Instance structure for the floating-point Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< Points to the array of state coefficients.  The array is of length 4*numStages. */
/* *< Points to the array of coefficients.  The array is of length 5*numStages. */
/* *
   * @brief Processing function for the Q15 Biquad cascade filter.
   * @param[in]  S          points to an instance of the Q15 Biquad cascade structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the Q15 Biquad cascade filter.
   * @param[in,out] S          points to an instance of the Q15 Biquad cascade structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     postShift  Shift to be applied to the output. Varies according to the coefficients format
   */
/* *
   * @brief Fast but less precise processing function for the Q15 Biquad cascade filter for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q15 Biquad cascade structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the Q31 Biquad cascade filter
   * @param[in]  S          points to an instance of the Q31 Biquad cascade structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Fast but less precise processing function for the Q31 Biquad cascade filter for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q31 Biquad cascade structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the Q31 Biquad cascade filter.
   * @param[in,out] S          points to an instance of the Q31 Biquad cascade structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     postShift  Shift to be applied to the output. Varies according to the coefficients format
   */
/* *
   * @brief Processing function for the floating-point Biquad cascade filter.
   * @param[in]  S          points to an instance of the floating-point Biquad cascade structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the floating-point Biquad cascade filter.
   * @param[in,out] S          points to an instance of the floating-point Biquad cascade structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   */
/* *
   * @brief Instance structure for the floating-point matrix structure.
   */
/* *< number of rows of the matrix.     */
/* *< number of columns of the matrix.  */
/* *< points to the data of the matrix. */
/* *
   * @brief Instance structure for the floating-point matrix structure.
   */
/* *< number of rows of the matrix.     */
/* *< number of columns of the matrix.  */
/* *< points to the data of the matrix. */
/* *
   * @brief Instance structure for the Q15 matrix structure.
   */
/* *< number of rows of the matrix.     */
/* *< number of columns of the matrix.  */
/* *< points to the data of the matrix. */
/* *
   * @brief Instance structure for the Q31 matrix structure.
   */
/* *< number of rows of the matrix.     */
/* *< number of columns of the matrix.  */
/* *< points to the data of the matrix. */
/* *
   * @brief Floating-point matrix addition.
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15 matrix addition.
   * @param[in]   pSrcA  points to the first input matrix structure
   * @param[in]   pSrcB  points to the second input matrix structure
   * @param[out]  pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31 matrix addition.
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Floating-point, complex, matrix multiplication.
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15, complex,  matrix multiplication.
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31, complex, matrix multiplication.
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Floating-point matrix transpose.
   * @param[in]  pSrc  points to the input matrix
   * @param[out] pDst  points to the output matrix
   * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
   * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15 matrix transpose.
   * @param[in]  pSrc  points to the input matrix
   * @param[out] pDst  points to the output matrix
   * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
   * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31 matrix transpose.
   * @param[in]  pSrc  points to the input matrix
   * @param[out] pDst  points to the output matrix
   * @return    The function returns either  <code>ARM_MATH_SIZE_MISMATCH</code>
   * or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Floating-point matrix multiplication
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15 matrix multiplication
   * @param[in]  pSrcA   points to the first input matrix structure
   * @param[in]  pSrcB   points to the second input matrix structure
   * @param[out] pDst    points to output matrix structure
   * @param[in]  pState  points to the array for storing intermediate results
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15 matrix multiplication (fast variant) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA   points to the first input matrix structure
   * @param[in]  pSrcB   points to the second input matrix structure
   * @param[out] pDst    points to output matrix structure
   * @param[in]  pState  points to the array for storing intermediate results
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31 matrix multiplication
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31 matrix multiplication (fast variant) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Floating-point matrix subtraction
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15 matrix subtraction
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31 matrix subtraction
   * @param[in]  pSrcA  points to the first input matrix structure
   * @param[in]  pSrcB  points to the second input matrix structure
   * @param[out] pDst   points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Floating-point matrix scaling.
   * @param[in]  pSrc   points to the input matrix
   * @param[in]  scale  scale factor
   * @param[out] pDst   points to the output matrix
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q15 matrix scaling.
   * @param[in]  pSrc        points to input matrix
   * @param[in]  scaleFract  fractional portion of the scale factor
   * @param[in]  shift       number of bits to shift the result by
   * @param[out] pDst        points to output matrix
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief Q31 matrix scaling.
   * @param[in]  pSrc        points to input matrix
   * @param[in]  scaleFract  fractional portion of the scale factor
   * @param[in]  shift       number of bits to shift the result by
   * @param[out] pDst        points to output matrix structure
   * @return     The function returns either
   * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
   */
/* *
   * @brief  Q31 matrix initialization.
   * @param[in,out] S         points to an instance of the floating-point matrix structure.
   * @param[in]     nRows     number of rows in the matrix.
   * @param[in]     nColumns  number of columns in the matrix.
   * @param[in]     pData     points to the matrix data array.
   */
/* *
   * @brief  Q15 matrix initialization.
   * @param[in,out] S         points to an instance of the floating-point matrix structure.
   * @param[in]     nRows     number of rows in the matrix.
   * @param[in]     nColumns  number of columns in the matrix.
   * @param[in]     pData     points to the matrix data array.
   */
/* *
   * @brief  Floating-point matrix initialization.
   * @param[in,out] S         points to an instance of the floating-point matrix structure.
   * @param[in]     nRows     number of rows in the matrix.
   * @param[in]     nColumns  number of columns in the matrix.
   * @param[in]     pData     points to the matrix data array.
   */
/* *
   * @brief Instance structure for the Q15 PID Control.
   */
/* *< The derived gain, A0 = Kp + Ki + Kd . */
/* *< The derived gain A1 = -Kp - 2Kd | Kd.*/
/* *< The state array of length 3. */
/* *< The proportional gain. */
/* *< The integral gain. */
/* *< The derivative gain. */
/* *
   * @brief Instance structure for the Q31 PID Control.
   */
/* *< The derived gain, A0 = Kp + Ki + Kd . */
/* *< The derived gain, A1 = -Kp - 2Kd. */
/* *< The derived gain, A2 = Kd . */
/* *< The state array of length 3. */
/* *< The proportional gain. */
/* *< The integral gain. */
/* *< The derivative gain. */
/* *
   * @brief Instance structure for the floating-point PID Control.
   */
/* *< The derived gain, A0 = Kp + Ki + Kd . */
/* *< The derived gain, A1 = -Kp - 2Kd. */
/* *< The derived gain, A2 = Kd . */
/* *< The state array of length 3. */
/* *< The proportional gain. */
/* *< The integral gain. */
/* *< The derivative gain. */
/* *
   * @brief  Initialization function for the floating-point PID Control.
   * @param[in,out] S               points to an instance of the PID structure.
   * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
   */
/* *
   * @brief  Reset function for the floating-point PID Control.
   * @param[in,out] S  is an instance of the floating-point PID Control structure
   */
/* *
   * @brief  Initialization function for the Q31 PID Control.
   * @param[in,out] S               points to an instance of the Q15 PID structure.
   * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
   */
/* *
   * @brief  Reset function for the Q31 PID Control.
   * @param[in,out] S   points to an instance of the Q31 PID Control structure
   */
/* *
   * @brief  Initialization function for the Q15 PID Control.
   * @param[in,out] S               points to an instance of the Q15 PID structure.
   * @param[in]     resetStateFlag  flag to reset the state. 0 = no change in state 1 = reset the state.
   */
/* *
   * @brief  Reset function for the Q15 PID Control.
   * @param[in,out] S  points to an instance of the q15 PID Control structure
   */
/* *
   * @brief Instance structure for the floating-point Linear Interpolate function.
   */
/* *< nValues */
/* *< x1 */
/* *< xSpacing */
/* *< pointer to the table of Y values */
/* *
   * @brief Instance structure for the floating-point bilinear interpolation function.
   */
/* *< number of rows in the data table. */
/* *< number of columns in the data table. */
/* *< points to the data table. */
/* *
   * @brief Instance structure for the Q31 bilinear interpolation function.
   */
/* *< number of rows in the data table. */
/* *< number of columns in the data table. */
/* *< points to the data table. */
/* *
   * @brief Instance structure for the Q15 bilinear interpolation function.
   */
/* *< number of rows in the data table. */
/* *< number of columns in the data table. */
/* *< points to the data table. */
/* *
   * @brief Instance structure for the Q15 bilinear interpolation function.
   */
/* *< number of rows in the data table. */
/* *< number of columns in the data table. */
/* *< points to the data table. */
/* *
   * @brief Q7 vector multiplication.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q15 vector multiplication.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q31 vector multiplication.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Floating-point vector multiplication.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Instance structure for the Q15 CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
/* *< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
/* *< points to the Sin twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
/* Deprecated */
/* Deprecated */
/* *
   * @brief Instance structure for the Q15 CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
/* *< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
/* *< points to the twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
/* Deprecated */
/* Deprecated */
/* *
   * @brief Instance structure for the Radix-2 Q31 CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
/* *< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
/* *< points to the Twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
/* Deprecated */
/* Deprecated */
/* *
   * @brief Instance structure for the Q31 CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
/* *< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
/* *< points to the twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
/* Deprecated */
/* Deprecated */
/* *
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
/* *< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
/* *< points to the Twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
/* *< value of 1/fftLen. */
/* Deprecated */
/* Deprecated */
/* *
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
/* *< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
/* *< points to the Twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
/* *< value of 1/fftLen. */
/* Deprecated */
/* Deprecated */
/* *
   * @brief Instance structure for the fixed-point CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< points to the Twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< bit reversal table length. */
/* *
   * @brief Instance structure for the fixed-point CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< points to the Twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< bit reversal table length. */
/* *
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
/* *< length of the FFT. */
/* *< points to the Twiddle factor table. */
/* *< points to the bit reversal table. */
/* *< bit reversal table length. */
/* *
   * @brief Instance structure for the Q15 RFFT/RIFFT function.
   */
/* *< length of the real FFT. */
/* *< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
/* *< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< points to the real twiddle factor table. */
/* *< points to the imag twiddle factor table. */
/* *< points to the complex FFT instance. */
/* *
   * @brief Instance structure for the Q31 RFFT/RIFFT function.
   */
/* *< length of the real FFT. */
/* *< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
/* *< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< points to the real twiddle factor table. */
/* *< points to the imag twiddle factor table. */
/* *< points to the complex FFT instance. */
/* *
   * @brief Instance structure for the floating-point RFFT/RIFFT function.
   */
/* *< length of the real FFT. */
/* *< length of the complex FFT. */
/* *< flag that selects forward (ifftFlagR=0) or inverse (ifftFlagR=1) transform. */
/* *< flag that enables (bitReverseFlagR=1) or disables (bitReverseFlagR=0) bit reversal of output. */
/* *< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
/* *< points to the real twiddle factor table. */
/* *< points to the imag twiddle factor table. */
/* *< points to the complex FFT instance. */
/* *
   * @brief Instance structure for the floating-point RFFT/RIFFT function.
   */
/* *< Internal CFFT structure. */
/* *< length of the real sequence */
/* *< Twiddle factors real stage  */
/* *
* @ingroup groupTransforms
*/
/* *
 * @defgroup RealFFT Real FFT Functions
 *
 * \par
 * The CMSIS DSP library includes specialized algorithms for computing the
 * FFT of real data sequences.  The FFT is defined over complex data but
 * in many applications the input is real.  Real FFT algorithms take advantage
 * of the symmetry properties of the FFT and have a speed advantage over complex
 * algorithms of the same length.
 * \par
 * The Fast RFFT algorith relays on the mixed radix CFFT that save processor usage.
 * \par
 * The real length N forward FFT of a sequence is computed using the steps shown below.
 * \par
 * \image html RFFT.gif "Real Fast Fourier Transform"
 * \par
 * The real sequence is initially treated as if it were complex to perform a CFFT.
 * Later, a processing stage reshapes the data to obtain half of the frequency spectrum
 * in complex format. Except the first complex number that contains the two real numbers
 * X[0] and X[N/2] all the data is complex. In other words, the first complex sample
 * contains two real values packed.
 * \par
 * The input for the inverse RFFT should keep the same format as the output of the
 * forward RFFT. A first processing stage pre-process the data to later perform an
 * inverse CFFT.
 * \par
 * \image html RIFFT.gif "Real Inverse Fast Fourier Transform"
 * \par
 * The algorithms for floating-point, Q15, and Q31 data are slightly different
 * and we describe each algorithm in turn.
 * \par Floating-point
 * The main functions are arm_rfft_fast_f32() and arm_rfft_fast_init_f32().
 * The older functions arm_rfft_f32() and arm_rfft_init_f32() have been
 * deprecated but are still documented.
 * \par
 * The FFT of a real N-point sequence has even symmetry in the frequency
 * domain. The second half of the data equals the conjugate of the first
 * half flipped in frequency. Looking at the data, we see that we can
 * uniquely represent the FFT using only N/2 complex numbers. These are
 * packed into the output array in alternating real and imaginary
 * components:
 * \par
 * X = { real[0], imag[0], real[1], imag[1], real[2], imag[2] ...
 * real[(N/2)-1], imag[(N/2)-1 }
 * \par
 * It happens that the first complex number (real[0], imag[0]) is actually
 * all real. real[0] represents the DC offset, and imag[0] should be 0.
 * (real[1], imag[1]) is the fundamental frequency, (real[2], imag[2]) is
 * the first harmonic and so on.
 * \par
 * The real FFT functions pack the frequency domain data in this fashion.
 * The forward transform outputs the data in this form and the inverse
 * transform expects input data in this form. The function always performs
 * the needed bitreversal so that the input and output data is always in
 * normal order. The functions support lengths of [32, 64, 128, ..., 4096]
 * samples.
 * \par Q15 and Q31
 * The real algorithms are defined in a similar manner and utilize N/2 complex
 * transforms behind the scenes.
 * \par
 * The complex transforms used internally include scaling to prevent fixed-point
 * overflows.  The overall scaling equals 1/(fftLen/2).
 * \par
 * A separate instance structure must be defined for each transform used but
 * twiddle factor and bit reversal tables can be reused.
 * \par
 * There is also an associated initialization function for each data type.
 * The initialization function performs the following operations:
 * - Sets the values of the internal structure fields.
 * - Initializes twiddle factor table and bit reversal table pointers.
 * - Initializes the internal complex FFT data structure.
 * \par
 * Use of the initialization function is optional.
 * However, if the initialization function is used, then the instance structure
 * cannot be placed into a const data section. To place an instance structure
 * into a const data section, the instance structure should be manually
 * initialized as follows:
 * <pre>
 *arm_rfft_instance_q31 S = {fftLenReal, fftLenBy2, ifftFlagR, bitReverseFlagR, twidCoefRModifier, pTwiddleAReal, pTwiddleBReal, pCfft};
 *arm_rfft_instance_q15 S = {fftLenReal, fftLenBy2, ifftFlagR, bitReverseFlagR, twidCoefRModifier, pTwiddleAReal, pTwiddleBReal, pCfft};
 * </pre>
 * where <code>fftLenReal</code> is the length of the real transform;
 * <code>fftLenBy2</code> length of  the internal complex transform.
 * <code>ifftFlagR</code> Selects forward (=0) or inverse (=1) transform.
 * <code>bitReverseFlagR</code> Selects bit reversed output (=0) or normal order
 * output (=1).
 * <code>twidCoefRModifier</code> stride modifier for the twiddle factor table.
 * The value is based on the FFT length;
 * <code>pTwiddleAReal</code>points to the A array of twiddle coefficients;
 * <code>pTwiddleBReal</code>points to the B array of twiddle coefficients;
 * <code>pCfft</code> points to the CFFT Instance structure. The CFFT structure
 * must also be initialized.  Refer to arm_cfft_radix4_f32() for details regarding
 * static initialization of the complex FFT instance structure.
 */
/* *
* @addtogroup RealFFT
* @{
*/
/* *
* @brief Processing function for the floating-point real FFT.
* @param[in]  *S              points to an arm_rfft_fast_instance_f32 structure.
* @param[in]  *p              points to the input buffer.
* @param[in]  *pOut           points to the output buffer.
* @param[in]  ifftFlag        RFFT if flag is 0, RIFFT if flag is 1
* @return none.
*/
#[no_mangle]
pub unsafe extern "C" fn arm_rfft_fast_f32(mut S:
                                               *mut arm_rfft_fast_instance_f32,
                                           mut p: *mut float32_t,
                                           mut pOut: *mut float32_t,
                                           mut ifftFlag: uint8_t) {
    let mut Sint: *mut arm_cfft_instance_f32 = &mut (*S).Sint;
    (*Sint).fftLen =
        ((*S).fftLenRFFT as libc::c_int / 2 as libc::c_int) as uint16_t;
    /* Calculation of Real FFT */
    if ifftFlag != 0 {
        /*  Real FFT compression */
        merge_rfft_f32(S, p, pOut);
        /* Complex radix-4 IFFT process */
        arm_cfft_f32(Sint, pOut, ifftFlag, 1 as libc::c_int as uint8_t);
    } else {
        /* Calculation of RFFT of input */
        arm_cfft_f32(Sint, p, ifftFlag, 1 as libc::c_int as uint8_t);
        /*  Real FFT extraction */
        stage_rfft_f32(S, p, pOut);
    };
}
/* *
* @} end of RealFFT group
*/
