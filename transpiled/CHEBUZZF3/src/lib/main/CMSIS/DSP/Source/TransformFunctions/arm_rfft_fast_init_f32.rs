use ::libc;
extern "C" {
    #[no_mangle]
    static twiddleCoef_16: [float32_t; 32];
    #[no_mangle]
    static twiddleCoef_32: [float32_t; 64];
    #[no_mangle]
    static twiddleCoef_64: [float32_t; 128];
    #[no_mangle]
    static twiddleCoef_128: [float32_t; 256];
    #[no_mangle]
    static twiddleCoef_256: [float32_t; 512];
    #[no_mangle]
    static twiddleCoef_512: [float32_t; 1024];
    #[no_mangle]
    static twiddleCoef_1024: [float32_t; 2048];
    #[no_mangle]
    static twiddleCoef_2048: [float32_t; 4096];
    #[no_mangle]
    static twiddleCoef_rfft_32: [float32_t; 32];
    #[no_mangle]
    static twiddleCoef_rfft_64: [float32_t; 64];
    #[no_mangle]
    static twiddleCoef_rfft_128: [float32_t; 128];
    #[no_mangle]
    static twiddleCoef_rfft_256: [float32_t; 256];
    #[no_mangle]
    static twiddleCoef_rfft_512: [float32_t; 512];
    #[no_mangle]
    static twiddleCoef_rfft_1024: [float32_t; 1024];
    #[no_mangle]
    static twiddleCoef_rfft_2048: [float32_t; 2048];
    #[no_mangle]
    static twiddleCoef_rfft_4096: [float32_t; 4096];
    #[no_mangle]
    static armBitRevIndexTable16: [uint16_t; 20];
    #[no_mangle]
    static armBitRevIndexTable32: [uint16_t; 48];
    #[no_mangle]
    static armBitRevIndexTable64: [uint16_t; 56];
    #[no_mangle]
    static armBitRevIndexTable128: [uint16_t; 208];
    #[no_mangle]
    static armBitRevIndexTable256: [uint16_t; 440];
    #[no_mangle]
    static armBitRevIndexTable512: [uint16_t; 448];
    #[no_mangle]
    static armBitRevIndexTable1024: [uint16_t; 1800];
    #[no_mangle]
    static armBitRevIndexTable2048: [uint16_t; 3808];
}
pub type __uint16_t = libc::c_ushort;
pub type uint16_t = __uint16_t;
pub type arm_status = libc::c_int;
pub const ARM_MATH_TEST_FAILURE: arm_status = -6;
pub const ARM_MATH_SINGULAR: arm_status = -5;
pub const ARM_MATH_NANINF: arm_status = -4;
pub const ARM_MATH_SIZE_MISMATCH: arm_status = -3;
pub const ARM_MATH_LENGTH_ERROR: arm_status = -2;
pub const ARM_MATH_ARGUMENT_ERROR: arm_status = -1;
pub const ARM_MATH_SUCCESS: arm_status = 0;
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
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cfft_init_f32.c
 * Description:  Split Radix Decimation in Frequency CFFT Floating point processing function
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
/* *
 * @ingroup groupTransforms
 */
/* *
 * @addtogroup RealFFT
 * @{
 */
/* *
* @brief  Initialization function for the floating-point real FFT.
* @param[in,out] *S             points to an arm_rfft_fast_instance_f32 structure.
* @param[in]     fftLen         length of the Real Sequence.
* @return        The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>fftLen</code> is not a supported value.
*
* \par Description:
* \par
* The parameter <code>fftLen</code>	Specifies length of RFFT/CIFFT process. Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096.
* \par
* This Function also initializes Twiddle factor table pointer and Bit reversal table pointer.
*/
#[no_mangle]
pub unsafe extern "C" fn arm_rfft_fast_init_f32(mut S:
                                                    *mut arm_rfft_fast_instance_f32,
                                                mut fftLen: uint16_t)
 -> arm_status {
    let mut Sint: *mut arm_cfft_instance_f32 =
        0 as *mut arm_cfft_instance_f32;
    /*  Initialise the default arm status */
    let mut status: arm_status = ARM_MATH_SUCCESS;
    /*  Initialise the FFT length */
    Sint = &mut (*S).Sint;
    (*Sint).fftLen = (fftLen as libc::c_int / 2 as libc::c_int) as uint16_t;
    (*S).fftLenRFFT = fftLen;
    /*  Initializations of structure parameters depending on the FFT length */
    match (*Sint).fftLen as libc::c_int {
        2048 => {
            /*  Initializations of structure parameters for 2048 point FFT */
    /*  Initialise the bit reversal table length */
            (*Sint).bitRevLength = 3808 as libc::c_int as uint16_t;
            /*  Initialise the bit reversal table pointer */
            (*Sint).pBitRevTable =
                armBitRevIndexTable2048.as_ptr() as *mut uint16_t;
            /*  Initialise the Twiddle coefficient pointers */
            (*Sint).pTwiddle = twiddleCoef_2048.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT =
                twiddleCoef_rfft_4096.as_ptr() as *mut float32_t
        }
        1024 => {
            (*Sint).bitRevLength = 1800 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable1024.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_1024.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT =
                twiddleCoef_rfft_2048.as_ptr() as *mut float32_t
        }
        512 => {
            (*Sint).bitRevLength = 448 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable512.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_512.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT =
                twiddleCoef_rfft_1024.as_ptr() as *mut float32_t
        }
        256 => {
            (*Sint).bitRevLength = 440 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable256.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_256.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT =
                twiddleCoef_rfft_512.as_ptr() as *mut float32_t
        }
        128 => {
            (*Sint).bitRevLength = 208 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable128.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_128.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT =
                twiddleCoef_rfft_256.as_ptr() as *mut float32_t
        }
        64 => {
            (*Sint).bitRevLength = 56 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable64.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_64.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT =
                twiddleCoef_rfft_128.as_ptr() as *mut float32_t
        }
        32 => {
            (*Sint).bitRevLength = 48 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable32.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_32.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT = twiddleCoef_rfft_64.as_ptr() as *mut float32_t
        }
        16 => {
            (*Sint).bitRevLength = 20 as libc::c_int as uint16_t;
            (*Sint).pBitRevTable =
                armBitRevIndexTable16.as_ptr() as *mut uint16_t;
            (*Sint).pTwiddle = twiddleCoef_16.as_ptr() as *mut float32_t;
            (*S).pTwiddleRFFT = twiddleCoef_rfft_32.as_ptr() as *mut float32_t
        }
        _ => {
            /*  Reporting argument error if fftSize is not valid value */
            status = ARM_MATH_ARGUMENT_ERROR
        }
    }
    return status;
}
/* *
 * @} end of RealFFT group
 */
