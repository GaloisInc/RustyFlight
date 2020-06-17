use core;
use libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
pub type float32_t = libc::c_float;
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
   * @brief Instance structure for the floating-point DCT4/IDCT4 function.
   */
/* *< length of the DCT4. */
/* *< half of the length of the DCT4. */
/* *< normalizing factor. */
/* *< points to the twiddle factor table. */
/* *< points to the cosFactor table. */
/* *< points to the real FFT instance. */
/* *< points to the complex FFT instance. */
/* *
   * @brief  Initialization function for the floating-point DCT4/IDCT4.
   * @param[in,out] S          points to an instance of floating-point DCT4/IDCT4 structure.
   * @param[in]     S_RFFT     points to an instance of floating-point RFFT/RIFFT structure.
   * @param[in]     S_CFFT     points to an instance of floating-point CFFT/CIFFT structure.
   * @param[in]     N          length of the DCT4.
   * @param[in]     Nby2       half of the length of the DCT4.
   * @param[in]     normalize  normalizing factor.
   * @return      arm_status function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>fftLenReal</code> is not a supported transform length.
   */
/* *
   * @brief Processing function for the floating-point DCT4/IDCT4.
   * @param[in]     S              points to an instance of the floating-point DCT4/IDCT4 structure.
   * @param[in]     pState         points to state buffer.
   * @param[in,out] pInlineBuffer  points to the in-place input and output buffer.
   */
/* *
   * @brief Instance structure for the Q31 DCT4/IDCT4 function.
   */
/* *< length of the DCT4. */
/* *< half of the length of the DCT4. */
/* *< normalizing factor. */
/* *< points to the twiddle factor table. */
/* *< points to the cosFactor table. */
/* *< points to the real FFT instance. */
/* *< points to the complex FFT instance. */
/* *
   * @brief  Initialization function for the Q31 DCT4/IDCT4.
   * @param[in,out] S          points to an instance of Q31 DCT4/IDCT4 structure.
   * @param[in]     S_RFFT     points to an instance of Q31 RFFT/RIFFT structure
   * @param[in]     S_CFFT     points to an instance of Q31 CFFT/CIFFT structure
   * @param[in]     N          length of the DCT4.
   * @param[in]     Nby2       half of the length of the DCT4.
   * @param[in]     normalize  normalizing factor.
   * @return      arm_status function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>N</code> is not a supported transform length.
   */
/* *
   * @brief Processing function for the Q31 DCT4/IDCT4.
   * @param[in]     S              points to an instance of the Q31 DCT4 structure.
   * @param[in]     pState         points to state buffer.
   * @param[in,out] pInlineBuffer  points to the in-place input and output buffer.
   */
/* *
   * @brief Instance structure for the Q15 DCT4/IDCT4 function.
   */
/* *< length of the DCT4. */
/* *< half of the length of the DCT4. */
/* *< normalizing factor. */
/* *< points to the twiddle factor table. */
/* *< points to the cosFactor table. */
/* *< points to the real FFT instance. */
/* *< points to the complex FFT instance. */
/* *
   * @brief  Initialization function for the Q15 DCT4/IDCT4.
   * @param[in,out] S          points to an instance of Q15 DCT4/IDCT4 structure.
   * @param[in]     S_RFFT     points to an instance of Q15 RFFT/RIFFT structure.
   * @param[in]     S_CFFT     points to an instance of Q15 CFFT/CIFFT structure.
   * @param[in]     N          length of the DCT4.
   * @param[in]     Nby2       half of the length of the DCT4.
   * @param[in]     normalize  normalizing factor.
   * @return      arm_status function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>N</code> is not a supported transform length.
   */
/* *
   * @brief Processing function for the Q15 DCT4/IDCT4.
   * @param[in]     S              points to an instance of the Q15 DCT4 structure.
   * @param[in]     pState         points to state buffer.
   * @param[in,out] pInlineBuffer  points to the in-place input and output buffer.
   */
/* *
   * @brief Floating-point vector addition.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q7 vector addition.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q15 vector addition.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q31 vector addition.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Floating-point vector subtraction.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q7 vector subtraction.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q15 vector subtraction.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q31 vector subtraction.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Multiplies a floating-point vector by a scalar.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  scale      scale factor to be applied
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief Multiplies a Q7 vector by a scalar.
   * @param[in]  pSrc        points to the input vector
   * @param[in]  scaleFract  fractional portion of the scale value
   * @param[in]  shift       number of bits to shift the result by
   * @param[out] pDst        points to the output vector
   * @param[in]  blockSize   number of samples in the vector
   */
/* *
   * @brief Multiplies a Q15 vector by a scalar.
   * @param[in]  pSrc        points to the input vector
   * @param[in]  scaleFract  fractional portion of the scale value
   * @param[in]  shift       number of bits to shift the result by
   * @param[out] pDst        points to the output vector
   * @param[in]  blockSize   number of samples in the vector
   */
/* *
   * @brief Multiplies a Q31 vector by a scalar.
   * @param[in]  pSrc        points to the input vector
   * @param[in]  scaleFract  fractional portion of the scale value
   * @param[in]  shift       number of bits to shift the result by
   * @param[out] pDst        points to the output vector
   * @param[in]  blockSize   number of samples in the vector
   */
/* *
   * @brief Q7 vector absolute value.
   * @param[in]  pSrc       points to the input buffer
   * @param[out] pDst       points to the output buffer
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Floating-point vector absolute value.
   * @param[in]  pSrc       points to the input buffer
   * @param[out] pDst       points to the output buffer
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q15 vector absolute value.
   * @param[in]  pSrc       points to the input buffer
   * @param[out] pDst       points to the output buffer
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Q31 vector absolute value.
   * @param[in]  pSrc       points to the input buffer
   * @param[out] pDst       points to the output buffer
   * @param[in]  blockSize  number of samples in each vector
   */
/* *
   * @brief Dot product of floating-point vectors.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[in]  blockSize  number of samples in each vector
   * @param[out] result     output result returned here
   */
/* *
   * @brief Dot product of Q7 vectors.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[in]  blockSize  number of samples in each vector
   * @param[out] result     output result returned here
   */
/* *
   * @brief Dot product of Q15 vectors.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[in]  blockSize  number of samples in each vector
   * @param[out] result     output result returned here
   */
/* *
   * @brief Dot product of Q31 vectors.
   * @param[in]  pSrcA      points to the first input vector
   * @param[in]  pSrcB      points to the second input vector
   * @param[in]  blockSize  number of samples in each vector
   * @param[out] result     output result returned here
   */
/* *
   * @brief  Shifts the elements of a Q7 vector a specified number of bits.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  shiftBits  number of bits to shift.  A positive value shifts left; a negative value shifts right.
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Shifts the elements of a Q15 vector a specified number of bits.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  shiftBits  number of bits to shift.  A positive value shifts left; a negative value shifts right.
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Shifts the elements of a Q31 vector a specified number of bits.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  shiftBits  number of bits to shift.  A positive value shifts left; a negative value shifts right.
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Adds a constant offset to a floating-point vector.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  offset     is the offset to be added
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Adds a constant offset to a Q7 vector.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  offset     is the offset to be added
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Adds a constant offset to a Q15 vector.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  offset     is the offset to be added
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Adds a constant offset to a Q31 vector.
   * @param[in]  pSrc       points to the input vector
   * @param[in]  offset     is the offset to be added
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Negates the elements of a floating-point vector.
   * @param[in]  pSrc       points to the input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Negates the elements of a Q7 vector.
   * @param[in]  pSrc       points to the input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Negates the elements of a Q15 vector.
   * @param[in]  pSrc       points to the input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Negates the elements of a Q31 vector.
   * @param[in]  pSrc       points to the input vector
   * @param[out] pDst       points to the output vector
   * @param[in]  blockSize  number of samples in the vector
   */
/* *
   * @brief  Copies the elements of a floating-point vector.
   * @param[in]  pSrc       input pointer
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Copies the elements of a Q7 vector.
   * @param[in]  pSrc       input pointer
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Copies the elements of a Q15 vector.
   * @param[in]  pSrc       input pointer
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Copies the elements of a Q31 vector.
   * @param[in]  pSrc       input pointer
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Fills a constant value into a floating-point vector.
   * @param[in]  value      input value to be filled
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Fills a constant value into a Q7 vector.
   * @param[in]  value      input value to be filled
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Fills a constant value into a Q15 vector.
   * @param[in]  value      input value to be filled
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @brief  Fills a constant value into a Q31 vector.
   * @param[in]  value      input value to be filled
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
 * @brief Convolution of floating-point sequences.
 * @param[in]  pSrcA    points to the first input sequence.
 * @param[in]  srcALen  length of the first input sequence.
 * @param[in]  pSrcB    points to the second input sequence.
 * @param[in]  srcBLen  length of the second input sequence.
 * @param[out] pDst     points to the location where the output result is written.  Length srcALen+srcBLen-1.
 */
/* *
   * @brief Convolution of Q15 sequences.
   * @param[in]  pSrcA      points to the first input sequence.
   * @param[in]  srcALen    length of the first input sequence.
   * @param[in]  pSrcB      points to the second input sequence.
   * @param[in]  srcBLen    length of the second input sequence.
   * @param[out] pDst       points to the block of output data  Length srcALen+srcBLen-1.
   * @param[in]  pScratch1  points to scratch buffer of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2  points to scratch buffer of size min(srcALen, srcBLen).
   */
/* *
 * @brief Convolution of Q15 sequences.
 * @param[in]  pSrcA    points to the first input sequence.
 * @param[in]  srcALen  length of the first input sequence.
 * @param[in]  pSrcB    points to the second input sequence.
 * @param[in]  srcBLen  length of the second input sequence.
 * @param[out] pDst     points to the location where the output result is written.  Length srcALen+srcBLen-1.
 */
/* *
   * @brief Convolution of Q15 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length srcALen+srcBLen-1.
   */
/* *
   * @brief Convolution of Q15 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA      points to the first input sequence.
   * @param[in]  srcALen    length of the first input sequence.
   * @param[in]  pSrcB      points to the second input sequence.
   * @param[in]  srcBLen    length of the second input sequence.
   * @param[out] pDst       points to the block of output data  Length srcALen+srcBLen-1.
   * @param[in]  pScratch1  points to scratch buffer of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2  points to scratch buffer of size min(srcALen, srcBLen).
   */
/* *
   * @brief Convolution of Q31 sequences.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length srcALen+srcBLen-1.
   */
/* *
   * @brief Convolution of Q31 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length srcALen+srcBLen-1.
   */
/* *
   * @brief Convolution of Q7 sequences.
   * @param[in]  pSrcA      points to the first input sequence.
   * @param[in]  srcALen    length of the first input sequence.
   * @param[in]  pSrcB      points to the second input sequence.
   * @param[in]  srcBLen    length of the second input sequence.
   * @param[out] pDst       points to the block of output data  Length srcALen+srcBLen-1.
   * @param[in]  pScratch1  points to scratch buffer(of type q15_t) of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2  points to scratch buffer (of type q15_t) of size min(srcALen, srcBLen).
   */
/* *
   * @brief Convolution of Q7 sequences.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length srcALen+srcBLen-1.
   */
/* *
   * @brief Partial convolution of floating-point sequences.
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q15 sequences.
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @param[in]  pScratch1   points to scratch buffer of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2   points to scratch buffer of size min(srcALen, srcBLen).
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q15 sequences.
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q15 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q15 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @param[in]  pScratch1   points to scratch buffer of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2   points to scratch buffer of size min(srcALen, srcBLen).
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q31 sequences.
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q31 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q7 sequences
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @param[in]  pScratch1   points to scratch buffer(of type q15_t) of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2   points to scratch buffer (of type q15_t) of size min(srcALen, srcBLen).
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Partial convolution of Q7 sequences.
   * @param[in]  pSrcA       points to the first input sequence.
   * @param[in]  srcALen     length of the first input sequence.
   * @param[in]  pSrcB       points to the second input sequence.
   * @param[in]  srcBLen     length of the second input sequence.
   * @param[out] pDst        points to the block of output data
   * @param[in]  firstIndex  is the first output sample to start with.
   * @param[in]  numPoints   is the number of output points to be computed.
   * @return  Returns either ARM_MATH_SUCCESS if the function completed correctly or ARM_MATH_ARGUMENT_ERROR if the requested subset is not in the range [0 srcALen+srcBLen-2].
   */
/* *
   * @brief Instance structure for the Q15 FIR decimator.
   */
/* *< decimation factor. */
/* *< number of coefficients in the filter. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *
   * @brief Instance structure for the Q31 FIR decimator.
   */
/* *< decimation factor. */
/* *< number of coefficients in the filter. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *
   * @brief Instance structure for the floating-point FIR decimator.
   */
/* *< decimation factor. */
/* *< number of coefficients in the filter. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *
   * @brief Processing function for the floating-point FIR decimator.
   * @param[in]  S          points to an instance of the floating-point FIR decimator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the floating-point FIR decimator.
   * @param[in,out] S          points to an instance of the floating-point FIR decimator structure.
   * @param[in]     numTaps    number of coefficients in the filter.
   * @param[in]     M          decimation factor.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of input samples to process per call.
   * @return    The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_LENGTH_ERROR if
   * <code>blockSize</code> is not a multiple of <code>M</code>.
   */
/* *
   * @brief Processing function for the Q15 FIR decimator.
   * @param[in]  S          points to an instance of the Q15 FIR decimator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief Processing function for the Q15 FIR decimator (fast variant) for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q15 FIR decimator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q15 FIR decimator.
   * @param[in,out] S          points to an instance of the Q15 FIR decimator structure.
   * @param[in]     numTaps    number of coefficients in the filter.
   * @param[in]     M          decimation factor.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of input samples to process per call.
   * @return    The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_LENGTH_ERROR if
   * <code>blockSize</code> is not a multiple of <code>M</code>.
   */
/* *
   * @brief Processing function for the Q31 FIR decimator.
   * @param[in]  S     points to an instance of the Q31 FIR decimator structure.
   * @param[in]  pSrc  points to the block of input data.
   * @param[out] pDst  points to the block of output data
   * @param[in] blockSize number of input samples to process per call.
   */
/* *
   * @brief Processing function for the Q31 FIR decimator (fast variant) for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q31 FIR decimator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q31 FIR decimator.
   * @param[in,out] S          points to an instance of the Q31 FIR decimator structure.
   * @param[in]     numTaps    number of coefficients in the filter.
   * @param[in]     M          decimation factor.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of input samples to process per call.
   * @return    The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_LENGTH_ERROR if
   * <code>blockSize</code> is not a multiple of <code>M</code>.
   */
/* *
   * @brief Instance structure for the Q15 FIR interpolator.
   */
/* *< upsample factor. */
/* *< length of each polyphase filter component. */
/* *< points to the coefficient array. The array is of length L*phaseLength. */
/* *< points to the state variable array. The array is of length blockSize+phaseLength-1. */
/* *
   * @brief Instance structure for the Q31 FIR interpolator.
   */
/* *< upsample factor. */
/* *< length of each polyphase filter component. */
/* *< points to the coefficient array. The array is of length L*phaseLength. */
/* *< points to the state variable array. The array is of length blockSize+phaseLength-1. */
/* *
   * @brief Instance structure for the floating-point FIR interpolator.
   */
/* *< upsample factor. */
/* *< length of each polyphase filter component. */
/* *< points to the coefficient array. The array is of length L*phaseLength. */
/* *< points to the state variable array. The array is of length phaseLength+numTaps-1. */
/* *
   * @brief Processing function for the Q15 FIR interpolator.
   * @param[in]  S          points to an instance of the Q15 FIR interpolator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q15 FIR interpolator.
   * @param[in,out] S          points to an instance of the Q15 FIR interpolator structure.
   * @param[in]     L          upsample factor.
   * @param[in]     numTaps    number of filter coefficients in the filter.
   * @param[in]     pCoeffs    points to the filter coefficient buffer.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of input samples to process per call.
   * @return        The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_LENGTH_ERROR if
   * the filter length <code>numTaps</code> is not a multiple of the interpolation factor <code>L</code>.
   */
/* *
   * @brief Processing function for the Q31 FIR interpolator.
   * @param[in]  S          points to an instance of the Q15 FIR interpolator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q31 FIR interpolator.
   * @param[in,out] S          points to an instance of the Q31 FIR interpolator structure.
   * @param[in]     L          upsample factor.
   * @param[in]     numTaps    number of filter coefficients in the filter.
   * @param[in]     pCoeffs    points to the filter coefficient buffer.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of input samples to process per call.
   * @return        The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_LENGTH_ERROR if
   * the filter length <code>numTaps</code> is not a multiple of the interpolation factor <code>L</code>.
   */
/* *
   * @brief Processing function for the floating-point FIR interpolator.
   * @param[in]  S          points to an instance of the floating-point FIR interpolator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the floating-point FIR interpolator.
   * @param[in,out] S          points to an instance of the floating-point FIR interpolator structure.
   * @param[in]     L          upsample factor.
   * @param[in]     numTaps    number of filter coefficients in the filter.
   * @param[in]     pCoeffs    points to the filter coefficient buffer.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     blockSize  number of input samples to process per call.
   * @return        The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_LENGTH_ERROR if
   * the filter length <code>numTaps</code> is not a multiple of the interpolation factor <code>L</code>.
   */
/* *
   * @brief Instance structure for the high precision Q31 Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< points to the array of state coefficients.  The array is of length 4*numStages. */
/* *< points to the array of coefficients.  The array is of length 5*numStages. */
/* *< additional shift, in bits, applied to each output sample. */
/* *
   * @param[in]  S          points to an instance of the high precision Q31 Biquad cascade filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @param[in,out] S          points to an instance of the high precision Q31 Biquad cascade filter structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     postShift  shift to be applied to the output. Varies according to the coefficients format
   */
/* *
   * @brief Instance structure for the floating-point transposed direct form II Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< points to the array of state coefficients.  The array is of length 2*numStages. */
/* *< points to the array of coefficients.  The array is of length 5*numStages. */
/* *
   * @brief Instance structure for the floating-point transposed direct form II Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< points to the array of state coefficients.  The array is of length 4*numStages. */
/* *< points to the array of coefficients.  The array is of length 5*numStages. */
/* *
   * @brief Instance structure for the floating-point transposed direct form II Biquad cascade filter.
   */
/* *< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
/* *< points to the array of state coefficients.  The array is of length 2*numStages. */
/* *< points to the array of coefficients.  The array is of length 5*numStages. */
/* *
   * @brief Processing function for the floating-point transposed direct form II Biquad cascade filter.
   * @param[in]  S          points to an instance of the filter data structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the floating-point transposed direct form II Biquad cascade filter. 2 channels
   * @param[in]  S          points to an instance of the filter data structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the floating-point transposed direct form II Biquad cascade filter.
   * @param[in]  S          points to an instance of the filter data structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief  Initialization function for the floating-point transposed direct form II Biquad cascade filter.
   * @param[in,out] S          points to an instance of the filter data structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   */
/* *
   * @brief  Initialization function for the floating-point transposed direct form II Biquad cascade filter.
   * @param[in,out] S          points to an instance of the filter data structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   */
/* *
   * @brief  Initialization function for the floating-point transposed direct form II Biquad cascade filter.
   * @param[in,out] S          points to an instance of the filter data structure.
   * @param[in]     numStages  number of 2nd order stages in the filter.
   * @param[in]     pCoeffs    points to the filter coefficients.
   * @param[in]     pState     points to the state buffer.
   */
/* *
   * @brief Instance structure for the Q15 FIR lattice filter.
   */
/* *< number of filter stages. */
/* *< points to the state variable array. The array is of length numStages. */
/* *< points to the coefficient array. The array is of length numStages. */
/* *
   * @brief Instance structure for the Q31 FIR lattice filter.
   */
/* *< number of filter stages. */
/* *< points to the state variable array. The array is of length numStages. */
/* *< points to the coefficient array. The array is of length numStages. */
/* *
   * @brief Instance structure for the floating-point FIR lattice filter.
   */
/* *< number of filter stages. */
/* *< points to the state variable array. The array is of length numStages. */
/* *< points to the coefficient array. The array is of length numStages. */
/* *
   * @brief Initialization function for the Q15 FIR lattice filter.
   * @param[in] S          points to an instance of the Q15 FIR lattice structure.
   * @param[in] numStages  number of filter stages.
   * @param[in] pCoeffs    points to the coefficient buffer.  The array is of length numStages.
   * @param[in] pState     points to the state buffer.  The array is of length numStages.
   */
/* *
   * @brief Processing function for the Q15 FIR lattice filter.
   * @param[in]  S          points to an instance of the Q15 FIR lattice structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for the Q31 FIR lattice filter.
   * @param[in] S          points to an instance of the Q31 FIR lattice structure.
   * @param[in] numStages  number of filter stages.
   * @param[in] pCoeffs    points to the coefficient buffer.  The array is of length numStages.
   * @param[in] pState     points to the state buffer.   The array is of length numStages.
   */
/* *
   * @brief Processing function for the Q31 FIR lattice filter.
   * @param[in]  S          points to an instance of the Q31 FIR lattice structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of samples to process.
   */
/* *
 * @brief Initialization function for the floating-point FIR lattice filter.
 * @param[in] S          points to an instance of the floating-point FIR lattice structure.
 * @param[in] numStages  number of filter stages.
 * @param[in] pCoeffs    points to the coefficient buffer.  The array is of length numStages.
 * @param[in] pState     points to the state buffer.  The array is of length numStages.
 */
/* *
   * @brief Processing function for the floating-point FIR lattice filter.
   * @param[in]  S          points to an instance of the floating-point FIR lattice structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Instance structure for the Q15 IIR lattice filter.
   */
/* *< number of stages in the filter. */
/* *< points to the state variable array. The array is of length numStages+blockSize. */
/* *< points to the reflection coefficient array. The array is of length numStages. */
/* *< points to the ladder coefficient array. The array is of length numStages+1. */
/* *
   * @brief Instance structure for the Q31 IIR lattice filter.
   */
/* *< number of stages in the filter. */
/* *< points to the state variable array. The array is of length numStages+blockSize. */
/* *< points to the reflection coefficient array. The array is of length numStages. */
/* *< points to the ladder coefficient array. The array is of length numStages+1. */
/* *
   * @brief Instance structure for the floating-point IIR lattice filter.
   */
/* *< number of stages in the filter. */
/* *< points to the state variable array. The array is of length numStages+blockSize. */
/* *< points to the reflection coefficient array. The array is of length numStages. */
/* *< points to the ladder coefficient array. The array is of length numStages+1. */
/* *
   * @brief Processing function for the floating-point IIR lattice filter.
   * @param[in]  S          points to an instance of the floating-point IIR lattice structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for the floating-point IIR lattice filter.
   * @param[in] S          points to an instance of the floating-point IIR lattice structure.
   * @param[in] numStages  number of stages in the filter.
   * @param[in] pkCoeffs   points to the reflection coefficient buffer.  The array is of length numStages.
   * @param[in] pvCoeffs   points to the ladder coefficient buffer.  The array is of length numStages+1.
   * @param[in] pState     points to the state buffer.  The array is of length numStages+blockSize-1.
   * @param[in] blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the Q31 IIR lattice filter.
   * @param[in]  S          points to an instance of the Q31 IIR lattice structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for the Q31 IIR lattice filter.
   * @param[in] S          points to an instance of the Q31 IIR lattice structure.
   * @param[in] numStages  number of stages in the filter.
   * @param[in] pkCoeffs   points to the reflection coefficient buffer.  The array is of length numStages.
   * @param[in] pvCoeffs   points to the ladder coefficient buffer.  The array is of length numStages+1.
   * @param[in] pState     points to the state buffer.  The array is of length numStages+blockSize.
   * @param[in] blockSize  number of samples to process.
   */
/* *
   * @brief Processing function for the Q15 IIR lattice filter.
   * @param[in]  S          points to an instance of the Q15 IIR lattice structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
 * @brief Initialization function for the Q15 IIR lattice filter.
 * @param[in] S          points to an instance of the fixed-point Q15 IIR lattice structure.
 * @param[in] numStages  number of stages in the filter.
 * @param[in] pkCoeffs   points to reflection coefficient buffer.  The array is of length numStages.
 * @param[in] pvCoeffs   points to ladder coefficient buffer.  The array is of length numStages+1.
 * @param[in] pState     points to state buffer.  The array is of length numStages+blockSize.
 * @param[in] blockSize  number of samples to process per call.
 */
/* *
   * @brief Instance structure for the floating-point LMS filter.
   */
/* *< number of coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *< step size that controls filter coefficient updates. */
/* *
   * @brief Processing function for floating-point LMS filter.
   * @param[in]  S          points to an instance of the floating-point LMS filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[in]  pRef       points to the block of reference data.
   * @param[out] pOut       points to the block of output data.
   * @param[out] pErr       points to the block of error data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for floating-point LMS filter.
   * @param[in] S          points to an instance of the floating-point LMS filter structure.
   * @param[in] numTaps    number of filter coefficients.
   * @param[in] pCoeffs    points to the coefficient buffer.
   * @param[in] pState     points to state buffer.
   * @param[in] mu         step size that controls filter coefficient updates.
   * @param[in] blockSize  number of samples to process.
   */
/* *
   * @brief Instance structure for the Q15 LMS filter.
   */
/* *< number of coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *< step size that controls filter coefficient updates. */
/* *< bit shift applied to coefficients. */
/* *
   * @brief Initialization function for the Q15 LMS filter.
   * @param[in] S          points to an instance of the Q15 LMS filter structure.
   * @param[in] numTaps    number of filter coefficients.
   * @param[in] pCoeffs    points to the coefficient buffer.
   * @param[in] pState     points to the state buffer.
   * @param[in] mu         step size that controls filter coefficient updates.
   * @param[in] blockSize  number of samples to process.
   * @param[in] postShift  bit shift applied to coefficients.
   */
/* *
   * @brief Processing function for Q15 LMS filter.
   * @param[in]  S          points to an instance of the Q15 LMS filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[in]  pRef       points to the block of reference data.
   * @param[out] pOut       points to the block of output data.
   * @param[out] pErr       points to the block of error data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Instance structure for the Q31 LMS filter.
   */
/* *< number of coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *< step size that controls filter coefficient updates. */
/* *< bit shift applied to coefficients. */
/* *
   * @brief Processing function for Q31 LMS filter.
   * @param[in]  S          points to an instance of the Q15 LMS filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[in]  pRef       points to the block of reference data.
   * @param[out] pOut       points to the block of output data.
   * @param[out] pErr       points to the block of error data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for Q31 LMS filter.
   * @param[in] S          points to an instance of the Q31 LMS filter structure.
   * @param[in] numTaps    number of filter coefficients.
   * @param[in] pCoeffs    points to coefficient buffer.
   * @param[in] pState     points to state buffer.
   * @param[in] mu         step size that controls filter coefficient updates.
   * @param[in] blockSize  number of samples to process.
   * @param[in] postShift  bit shift applied to coefficients.
   */
/* *
   * @brief Instance structure for the floating-point normalized LMS filter.
   */
/* *< number of coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *< step size that control filter coefficient updates. */
/* *< saves previous frame energy. */
/* *< saves previous input sample. */
/* *
   * @brief Processing function for floating-point normalized LMS filter.
   * @param[in]  S          points to an instance of the floating-point normalized LMS filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[in]  pRef       points to the block of reference data.
   * @param[out] pOut       points to the block of output data.
   * @param[out] pErr       points to the block of error data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for floating-point normalized LMS filter.
   * @param[in] S          points to an instance of the floating-point LMS filter structure.
   * @param[in] numTaps    number of filter coefficients.
   * @param[in] pCoeffs    points to coefficient buffer.
   * @param[in] pState     points to state buffer.
   * @param[in] mu         step size that controls filter coefficient updates.
   * @param[in] blockSize  number of samples to process.
   */
/* *
   * @brief Instance structure for the Q31 normalized LMS filter.
   */
/* *< number of coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *< step size that controls filter coefficient updates. */
/* *< bit shift applied to coefficients. */
/* *< points to the reciprocal initial value table. */
/* *< saves previous frame energy. */
/* *< saves previous input sample. */
/* *
   * @brief Processing function for Q31 normalized LMS filter.
   * @param[in]  S          points to an instance of the Q31 normalized LMS filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[in]  pRef       points to the block of reference data.
   * @param[out] pOut       points to the block of output data.
   * @param[out] pErr       points to the block of error data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for Q31 normalized LMS filter.
   * @param[in] S          points to an instance of the Q31 normalized LMS filter structure.
   * @param[in] numTaps    number of filter coefficients.
   * @param[in] pCoeffs    points to coefficient buffer.
   * @param[in] pState     points to state buffer.
   * @param[in] mu         step size that controls filter coefficient updates.
   * @param[in] blockSize  number of samples to process.
   * @param[in] postShift  bit shift applied to coefficients.
   */
/* *
   * @brief Instance structure for the Q15 normalized LMS filter.
   */
/* *< Number of coefficients in the filter. */
/* *< points to the state variable array. The array is of length numTaps+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps. */
/* *< step size that controls filter coefficient updates. */
/* *< bit shift applied to coefficients. */
/* *< Points to the reciprocal initial value table. */
/* *< saves previous frame energy. */
/* *< saves previous input sample. */
/* *
   * @brief Processing function for Q15 normalized LMS filter.
   * @param[in]  S          points to an instance of the Q15 normalized LMS filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[in]  pRef       points to the block of reference data.
   * @param[out] pOut       points to the block of output data.
   * @param[out] pErr       points to the block of error data.
   * @param[in]  blockSize  number of samples to process.
   */
/* *
   * @brief Initialization function for Q15 normalized LMS filter.
   * @param[in] S          points to an instance of the Q15 normalized LMS filter structure.
   * @param[in] numTaps    number of filter coefficients.
   * @param[in] pCoeffs    points to coefficient buffer.
   * @param[in] pState     points to state buffer.
   * @param[in] mu         step size that controls filter coefficient updates.
   * @param[in] blockSize  number of samples to process.
   * @param[in] postShift  bit shift applied to coefficients.
   */
/* *
   * @brief Correlation of floating-point sequences.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   */
/* *
   * @brief Correlation of Q15 sequences
   * @param[in]  pSrcA     points to the first input sequence.
   * @param[in]  srcALen   length of the first input sequence.
   * @param[in]  pSrcB     points to the second input sequence.
   * @param[in]  srcBLen   length of the second input sequence.
   * @param[out] pDst      points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   * @param[in]  pScratch  points to scratch buffer of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   */
/* *
   * @brief Correlation of Q15 sequences.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   */
/* *
   * @brief Correlation of Q15 sequences (fast version) for Cortex-M3 and Cortex-M4.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   */
/* *
   * @brief Correlation of Q15 sequences (fast version) for Cortex-M3 and Cortex-M4.
   * @param[in]  pSrcA     points to the first input sequence.
   * @param[in]  srcALen   length of the first input sequence.
   * @param[in]  pSrcB     points to the second input sequence.
   * @param[in]  srcBLen   length of the second input sequence.
   * @param[out] pDst      points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   * @param[in]  pScratch  points to scratch buffer of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   */
/* *
   * @brief Correlation of Q31 sequences.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   */
/* *
   * @brief Correlation of Q31 sequences (fast version) for Cortex-M3 and Cortex-M4
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   */
/* *
   * @brief Correlation of Q7 sequences.
   * @param[in]  pSrcA      points to the first input sequence.
   * @param[in]  srcALen    length of the first input sequence.
   * @param[in]  pSrcB      points to the second input sequence.
   * @param[in]  srcBLen    length of the second input sequence.
   * @param[out] pDst       points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   * @param[in]  pScratch1  points to scratch buffer(of type q15_t) of size max(srcALen, srcBLen) + 2*min(srcALen, srcBLen) - 2.
   * @param[in]  pScratch2  points to scratch buffer (of type q15_t) of size min(srcALen, srcBLen).
   */
/* *
   * @brief Correlation of Q7 sequences.
   * @param[in]  pSrcA    points to the first input sequence.
   * @param[in]  srcALen  length of the first input sequence.
   * @param[in]  pSrcB    points to the second input sequence.
   * @param[in]  srcBLen  length of the second input sequence.
   * @param[out] pDst     points to the block of output data  Length 2 * max(srcALen, srcBLen) - 1.
   */
/* *
   * @brief Instance structure for the floating-point sparse FIR filter.
   */
/* *< number of coefficients in the filter. */
/* *< state buffer index.  Points to the oldest sample in the state buffer. */
/* *< points to the state buffer array. The array is of length maxDelay+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< maximum offset specified by the pTapDelay array. */
/* *< points to the array of delay values.  The array is of length numTaps. */
/* *
   * @brief Instance structure for the Q31 sparse FIR filter.
   */
/* *< number of coefficients in the filter. */
/* *< state buffer index.  Points to the oldest sample in the state buffer. */
/* *< points to the state buffer array. The array is of length maxDelay+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< maximum offset specified by the pTapDelay array. */
/* *< points to the array of delay values.  The array is of length numTaps. */
/* *
   * @brief Instance structure for the Q15 sparse FIR filter.
   */
/* *< number of coefficients in the filter. */
/* *< state buffer index.  Points to the oldest sample in the state buffer. */
/* *< points to the state buffer array. The array is of length maxDelay+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< maximum offset specified by the pTapDelay array. */
/* *< points to the array of delay values.  The array is of length numTaps. */
/* *
   * @brief Instance structure for the Q7 sparse FIR filter.
   */
/* *< number of coefficients in the filter. */
/* *< state buffer index.  Points to the oldest sample in the state buffer. */
/* *< points to the state buffer array. The array is of length maxDelay+blockSize-1. */
/* *< points to the coefficient array. The array is of length numTaps.*/
/* *< maximum offset specified by the pTapDelay array. */
/* *< points to the array of delay values.  The array is of length numTaps. */
/* *
   * @brief Processing function for the floating-point sparse FIR filter.
   * @param[in]  S           points to an instance of the floating-point sparse FIR structure.
   * @param[in]  pSrc        points to the block of input data.
   * @param[out] pDst        points to the block of output data
   * @param[in]  pScratchIn  points to a temporary buffer of size blockSize.
   * @param[in]  blockSize   number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the floating-point sparse FIR filter.
   * @param[in,out] S          points to an instance of the floating-point sparse FIR structure.
   * @param[in]     numTaps    number of nonzero coefficients in the filter.
   * @param[in]     pCoeffs    points to the array of filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     pTapDelay  points to the array of offset times.
   * @param[in]     maxDelay   maximum offset time supported.
   * @param[in]     blockSize  number of samples that will be processed per block.
   */
/* *
   * @brief Processing function for the Q31 sparse FIR filter.
   * @param[in]  S           points to an instance of the Q31 sparse FIR structure.
   * @param[in]  pSrc        points to the block of input data.
   * @param[out] pDst        points to the block of output data
   * @param[in]  pScratchIn  points to a temporary buffer of size blockSize.
   * @param[in]  blockSize   number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q31 sparse FIR filter.
   * @param[in,out] S          points to an instance of the Q31 sparse FIR structure.
   * @param[in]     numTaps    number of nonzero coefficients in the filter.
   * @param[in]     pCoeffs    points to the array of filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     pTapDelay  points to the array of offset times.
   * @param[in]     maxDelay   maximum offset time supported.
   * @param[in]     blockSize  number of samples that will be processed per block.
   */
/* *
   * @brief Processing function for the Q15 sparse FIR filter.
   * @param[in]  S            points to an instance of the Q15 sparse FIR structure.
   * @param[in]  pSrc         points to the block of input data.
   * @param[out] pDst         points to the block of output data
   * @param[in]  pScratchIn   points to a temporary buffer of size blockSize.
   * @param[in]  pScratchOut  points to a temporary buffer of size blockSize.
   * @param[in]  blockSize    number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q15 sparse FIR filter.
   * @param[in,out] S          points to an instance of the Q15 sparse FIR structure.
   * @param[in]     numTaps    number of nonzero coefficients in the filter.
   * @param[in]     pCoeffs    points to the array of filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     pTapDelay  points to the array of offset times.
   * @param[in]     maxDelay   maximum offset time supported.
   * @param[in]     blockSize  number of samples that will be processed per block.
   */
/* *
   * @brief Processing function for the Q7 sparse FIR filter.
   * @param[in]  S            points to an instance of the Q7 sparse FIR structure.
   * @param[in]  pSrc         points to the block of input data.
   * @param[out] pDst         points to the block of output data
   * @param[in]  pScratchIn   points to a temporary buffer of size blockSize.
   * @param[in]  pScratchOut  points to a temporary buffer of size blockSize.
   * @param[in]  blockSize    number of input samples to process per call.
   */
/* *
   * @brief  Initialization function for the Q7 sparse FIR filter.
   * @param[in,out] S          points to an instance of the Q7 sparse FIR structure.
   * @param[in]     numTaps    number of nonzero coefficients in the filter.
   * @param[in]     pCoeffs    points to the array of filter coefficients.
   * @param[in]     pState     points to the state buffer.
   * @param[in]     pTapDelay  points to the array of offset times.
   * @param[in]     maxDelay   maximum offset time supported.
   * @param[in]     blockSize  number of samples that will be processed per block.
   */
/* *
   * @brief  Floating-point sin_cos function.
   * @param[in]  theta   input value in degrees
   * @param[out] pSinVal  points to the processed sine output.
   * @param[out] pCosVal  points to the processed cos output.
   */
/* *
   * @brief  Q31 sin_cos function.
   * @param[in]  theta    scaled input value in degrees
   * @param[out] pSinVal  points to the processed sine output.
   * @param[out] pCosVal  points to the processed cosine output.
   */
/* *
   * @brief  Floating-point complex conjugate.
   * @param[in]  pSrc        points to the input vector
   * @param[out] pDst        points to the output vector
   * @param[in]  numSamples  number of complex samples in each vector
   */
/* *
   * @brief  Q31 complex conjugate.
   * @param[in]  pSrc        points to the input vector
   * @param[out] pDst        points to the output vector
   * @param[in]  numSamples  number of complex samples in each vector
   */
/* *
   * @brief  Q15 complex conjugate.
   * @param[in]  pSrc        points to the input vector
   * @param[out] pDst        points to the output vector
   * @param[in]  numSamples  number of complex samples in each vector
   */
/* *
   * @brief  Floating-point complex magnitude squared
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
/* *
   * @brief  Q31 complex magnitude squared
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
/* *
   * @brief  Q15 complex magnitude squared
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
/* *
   * @ingroup groupController
   */
/* *
   * @defgroup PID PID Motor Control
   *
   * A Proportional Integral Derivative (PID) controller is a generic feedback control
   * loop mechanism widely used in industrial control systems.
   * A PID controller is the most commonly used type of feedback controller.
   *
   * This set of functions implements (PID) controllers
   * for Q15, Q31, and floating-point data types.  The functions operate on a single sample
   * of data and each call to the function returns a single processed value.
   * <code>S</code> points to an instance of the PID control data structure.  <code>in</code>
   * is the input sample value. The functions return the output value.
   *
   * \par Algorithm:
   * <pre>
   *    y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]
   *    A0 = Kp + Ki + Kd
   *    A1 = (-Kp ) - (2 * Kd )
   *    A2 = Kd  </pre>
   *
   * \par
   * where \c Kp is proportional constant, \c Ki is Integral constant and \c Kd is Derivative constant
   *
   * \par
   * \image html PID.gif "Proportional Integral Derivative Controller"
   *
   * \par
   * The PID controller calculates an "error" value as the difference between
   * the measured output and the reference input.
   * The controller attempts to minimize the error by adjusting the process control inputs.
   * The proportional value determines the reaction to the current error,
   * the integral value determines the reaction based on the sum of recent errors,
   * and the derivative value determines the reaction based on the rate at which the error has been changing.
   *
   * \par Instance Structure
   * The Gains A0, A1, A2 and state variables for a PID controller are stored together in an instance data structure.
   * A separate instance structure must be defined for each PID Controller.
   * There are separate instance structure declarations for each of the 3 supported data types.
   *
   * \par Reset Functions
   * There is also an associated reset function for each data type which clears the state array.
   *
   * \par Initialization Functions
   * There is also an associated initialization function for each data type.
   * The initialization function performs the following operations:
   * - Initializes the Gains A0, A1, A2 from Kp,Ki, Kd gains.
   * - Zeros out the values in the state buffer.
   *
   * \par
   * Instance structure cannot be placed into a const data section and it is recommended to use the initialization function.
   *
   * \par Fixed-Point Behavior
   * Care must be taken when using the fixed-point versions of the PID Controller functions.
   * In particular, the overflow and saturation behavior of the accumulator used in each function must be considered.
   * Refer to the function specific documentation below for usage guidelines.
   */
/* *
   * @addtogroup PID
   * @{
   */
/* *
   * @brief  Process function for the floating-point PID Control.
   * @param[in,out] S   is an instance of the floating-point PID Control structure
   * @param[in]     in  input sample to process
   * @return out processed output sample.
   */
/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
/* Update state */
/* return to application */
/* *
   * @brief  Process function for the Q31 PID Control.
   * @param[in,out] S  points to an instance of the Q31 PID Control structure
   * @param[in]     in  input sample to process
   * @return out processed output sample.
   *
   * <b>Scaling and Overflow Behavior:</b>
   * \par
   * The function is implemented using an internal 64-bit accumulator.
   * The accumulator has a 2.62 format and maintains full precision of the intermediate multiplication results but provides only a single guard bit.
   * Thus, if the accumulator result overflows it wraps around rather than clip.
   * In order to avoid overflows completely the input signal must be scaled down by 2 bits as there are four additions.
   * After all multiply-accumulates are performed, the 2.62 accumulator is truncated to 1.32 format and then saturated to 1.31 format.
   */
/* acc = A0 * x[n]  */
/* acc += A1 * x[n-1] */
/* acc += A2 * x[n-2]  */
/* convert output to 1.31 format to add y[n-1] */
/* out += y[n-1] */
/* Update state */
/* return to application */
/* *
   * @brief  Process function for the Q15 PID Control.
   * @param[in,out] S   points to an instance of the Q15 PID Control structure
   * @param[in]     in  input sample to process
   * @return out processed output sample.
   *
   * <b>Scaling and Overflow Behavior:</b>
   * \par
   * The function is implemented using a 64-bit internal accumulator.
   * Both Gains and state variables are represented in 1.15 format and multiplications yield a 2.30 result.
   * The 2.30 intermediate results are accumulated in a 64-bit accumulator in 34.30 format.
   * There is no risk of internal overflow with this approach and the full precision of intermediate multiplications is preserved.
   * After all additions have been performed, the accumulator is truncated to 34.15 format by discarding low 15 bits.
   * Lastly, the accumulator is saturated to yield a result in 1.15 format.
   */
/* Implementation of PID controller */
/* acc = A0 * x[n]  */
/* acc += A1 * x[n-1] + A2 * x[n-2]  */
/* acc += y[n-1] */
/* saturate the output */
/* Update state */
/* return to application */
/* *
   * @} end of PID group
   */
/* *
   * @brief Floating-point matrix inverse.
   * @param[in]  src   points to the instance of the input floating-point matrix structure.
   * @param[out] dst   points to the instance of the output floating-point matrix structure.
   * @return The function returns ARM_MATH_SIZE_MISMATCH, if the dimensions do not match.
   * If the input matrix is singular (does not have an inverse), then the algorithm terminates and returns error status ARM_MATH_SINGULAR.
   */
/* *
   * @brief Floating-point matrix inverse.
   * @param[in]  src   points to the instance of the input floating-point matrix structure.
   * @param[out] dst   points to the instance of the output floating-point matrix structure.
   * @return The function returns ARM_MATH_SIZE_MISMATCH, if the dimensions do not match.
   * If the input matrix is singular (does not have an inverse), then the algorithm terminates and returns error status ARM_MATH_SINGULAR.
   */
/* *
   * @ingroup groupController
   */
/* *
   * @defgroup clarke Vector Clarke Transform
   * Forward Clarke transform converts the instantaneous stator phases into a two-coordinate time invariant vector.
   * Generally the Clarke transform uses three-phase currents <code>Ia, Ib and Ic</code> to calculate currents
   * in the two-phase orthogonal stator axis <code>Ialpha</code> and <code>Ibeta</code>.
   * When <code>Ialpha</code> is superposed with <code>Ia</code> as shown in the figure below
   * \image html clarke.gif Stator current space vector and its components in (a,b).
   * and <code>Ia + Ib + Ic = 0</code>, in this condition <code>Ialpha</code> and <code>Ibeta</code>
   * can be calculated using only <code>Ia</code> and <code>Ib</code>.
   *
   * The function operates on a single sample of data and each call to the function returns the processed output.
   * The library provides separate functions for Q31 and floating-point data types.
   * \par Algorithm
   * \image html clarkeFormula.gif
   * where <code>Ia</code> and <code>Ib</code> are the instantaneous stator phases and
   * <code>pIalpha</code> and <code>pIbeta</code> are the two coordinates of time invariant vector.
   * \par Fixed-Point Behavior
   * Care must be taken when using the Q31 version of the Clarke transform.
   * In particular, the overflow and saturation behavior of the accumulator used must be considered.
   * Refer to the function specific documentation below for usage guidelines.
   */
/* *
   * @addtogroup clarke
   * @{
   */
/* *
   *
   * @brief  Floating-point Clarke transform
   * @param[in]  Ia       input three-phase coordinate <code>a</code>
   * @param[in]  Ib       input three-phase coordinate <code>b</code>
   * @param[out] pIalpha  points to output two-phase orthogonal vector axis alpha
   * @param[out] pIbeta   points to output two-phase orthogonal vector axis beta
   */
/* Calculate pIalpha using the equation, pIalpha = Ia */
/* Calculate pIbeta using the equation, pIbeta = (1/sqrt(3)) * Ia + (2/sqrt(3)) * Ib */
/* *
   * @brief  Clarke transform for Q31 version
   * @param[in]  Ia       input three-phase coordinate <code>a</code>
   * @param[in]  Ib       input three-phase coordinate <code>b</code>
   * @param[out] pIalpha  points to output two-phase orthogonal vector axis alpha
   * @param[out] pIbeta   points to output two-phase orthogonal vector axis beta
   *
   * <b>Scaling and Overflow Behavior:</b>
   * \par
   * The function is implemented using an internal 32-bit accumulator.
   * The accumulator maintains 1.31 format by truncating lower 31 bits of the intermediate multiplication in 2.62 format.
   * There is saturation on the addition, hence there is no risk of overflow.
   */
/* Temporary variables used to store intermediate results */
/* Calculating pIalpha from Ia by equation pIalpha = Ia */
/* Intermediate product is calculated by (1/(sqrt(3)) * Ia) */
/* Intermediate product is calculated by (2/sqrt(3) * Ib) */
/* pIbeta is calculated by adding the intermediate products */
/* *
   * @} end of clarke group
   */
/* *
   * @brief  Converts the elements of the Q7 vector to Q31 vector.
   * @param[in]  pSrc       input pointer
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @ingroup groupController
   */
/* *
   * @defgroup inv_clarke Vector Inverse Clarke Transform
   * Inverse Clarke transform converts the two-coordinate time invariant vector into instantaneous stator phases.
   *
   * The function operates on a single sample of data and each call to the function returns the processed output.
   * The library provides separate functions for Q31 and floating-point data types.
   * \par Algorithm
   * \image html clarkeInvFormula.gif
   * where <code>pIa</code> and <code>pIb</code> are the instantaneous stator phases and
   * <code>Ialpha</code> and <code>Ibeta</code> are the two coordinates of time invariant vector.
   * \par Fixed-Point Behavior
   * Care must be taken when using the Q31 version of the Clarke transform.
   * In particular, the overflow and saturation behavior of the accumulator used must be considered.
   * Refer to the function specific documentation below for usage guidelines.
   */
/* *
   * @addtogroup inv_clarke
   * @{
   */
/* *
   * @brief  Floating-point Inverse Clarke transform
   * @param[in]  Ialpha  input two-phase orthogonal vector axis alpha
   * @param[in]  Ibeta   input two-phase orthogonal vector axis beta
   * @param[out] pIa     points to output three-phase coordinate <code>a</code>
   * @param[out] pIb     points to output three-phase coordinate <code>b</code>
   */
/* Calculating pIa from Ialpha by equation pIa = Ialpha */
/* Calculating pIb from Ialpha and Ibeta by equation pIb = -(1/2) * Ialpha + (sqrt(3)/2) * Ibeta */
/* *
   * @brief  Inverse Clarke transform for Q31 version
   * @param[in]  Ialpha  input two-phase orthogonal vector axis alpha
   * @param[in]  Ibeta   input two-phase orthogonal vector axis beta
   * @param[out] pIa     points to output three-phase coordinate <code>a</code>
   * @param[out] pIb     points to output three-phase coordinate <code>b</code>
   *
   * <b>Scaling and Overflow Behavior:</b>
   * \par
   * The function is implemented using an internal 32-bit accumulator.
   * The accumulator maintains 1.31 format by truncating lower 31 bits of the intermediate multiplication in 2.62 format.
   * There is saturation on the subtraction, hence there is no risk of overflow.
   */
/* Temporary variables used to store intermediate results */
/* Calculating pIa from Ialpha by equation pIa = Ialpha */
/* Intermediate product is calculated by (1/(2*sqrt(3)) * Ia) */
/* Intermediate product is calculated by (1/sqrt(3) * pIb) */
/* pIb is calculated by subtracting the products */
/* *
   * @} end of inv_clarke group
   */
/* *
   * @brief  Converts the elements of the Q7 vector to Q15 vector.
   * @param[in]  pSrc       input pointer
   * @param[out] pDst       output pointer
   * @param[in]  blockSize  number of samples to process
   */
/* *
   * @ingroup groupController
   */
/* *
   * @defgroup park Vector Park Transform
   *
   * Forward Park transform converts the input two-coordinate vector to flux and torque components.
   * The Park transform can be used to realize the transformation of the <code>Ialpha</code> and the <code>Ibeta</code> currents
   * from the stationary to the moving reference frame and control the spatial relationship between
   * the stator vector current and rotor flux vector.
   * If we consider the d axis aligned with the rotor flux, the diagram below shows the
   * current vector and the relationship from the two reference frames:
   * \image html park.gif "Stator current space vector and its component in (a,b) and in the d,q rotating reference frame"
   *
   * The function operates on a single sample of data and each call to the function returns the processed output.
   * The library provides separate functions for Q31 and floating-point data types.
   * \par Algorithm
   * \image html parkFormula.gif
   * where <code>Ialpha</code> and <code>Ibeta</code> are the stator vector components,
   * <code>pId</code> and <code>pIq</code> are rotor vector components and <code>cosVal</code> and <code>sinVal</code> are the
   * cosine and sine values of theta (rotor flux position).
   * \par Fixed-Point Behavior
   * Care must be taken when using the Q31 version of the Park transform.
   * In particular, the overflow and saturation behavior of the accumulator used must be considered.
   * Refer to the function specific documentation below for usage guidelines.
   */
/* *
   * @addtogroup park
   * @{
   */
/* *
   * @brief Floating-point Park transform
   * @param[in]  Ialpha  input two-phase vector coordinate alpha
   * @param[in]  Ibeta   input two-phase vector coordinate beta
   * @param[out] pId     points to output   rotor reference frame d
   * @param[out] pIq     points to output   rotor reference frame q
   * @param[in]  sinVal  sine value of rotation angle theta
   * @param[in]  cosVal  cosine value of rotation angle theta
   *
   * The function implements the forward Park transform.
   *
   */
/* Calculate pId using the equation, pId = Ialpha * cosVal + Ibeta * sinVal */
/* Calculate pIq using the equation, pIq = - Ialpha * sinVal + Ibeta * cosVal */
/* *
   * @brief  Park transform for Q31 version
   * @param[in]  Ialpha  input two-phase vector coordinate alpha
   * @param[in]  Ibeta   input two-phase vector coordinate beta
   * @param[out] pId     points to output rotor reference frame d
   * @param[out] pIq     points to output rotor reference frame q
   * @param[in]  sinVal  sine value of rotation angle theta
   * @param[in]  cosVal  cosine value of rotation angle theta
   *
   * <b>Scaling and Overflow Behavior:</b>
   * \par
   * The function is implemented using an internal 32-bit accumulator.
   * The accumulator maintains 1.31 format by truncating lower 31 bits of the intermediate multiplication in 2.62 format.
   * There is saturation on the addition and subtraction, hence there is no risk of overflow.
   */
/* Temporary variables used to store intermediate results */
/* Temporary variables used to store intermediate results */
/* Intermediate product is calculated by (Ialpha * cosVal) */
/* Intermediate product is calculated by (Ibeta * sinVal) */
/* Intermediate product is calculated by (Ialpha * sinVal) */
/* Intermediate product is calculated by (Ibeta * cosVal) */
/* Calculate pId by adding the two intermediate products 1 and 2 */
/* Calculate pIq by subtracting the two intermediate products 3 from 4 */
/* *
   * @} end of park group
   */
/* *
   * @brief  Converts the elements of the Q7 vector to floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[out] pDst       is output pointer
   * @param[in]  blockSize  is the number of samples to process
   */
/* *
   * @ingroup groupController
   */
/* *
   * @defgroup inv_park Vector Inverse Park transform
   * Inverse Park transform converts the input flux and torque components to two-coordinate vector.
   *
   * The function operates on a single sample of data and each call to the function returns the processed output.
   * The library provides separate functions for Q31 and floating-point data types.
   * \par Algorithm
   * \image html parkInvFormula.gif
   * where <code>pIalpha</code> and <code>pIbeta</code> are the stator vector components,
   * <code>Id</code> and <code>Iq</code> are rotor vector components and <code>cosVal</code> and <code>sinVal</code> are the
   * cosine and sine values of theta (rotor flux position).
   * \par Fixed-Point Behavior
   * Care must be taken when using the Q31 version of the Park transform.
   * In particular, the overflow and saturation behavior of the accumulator used must be considered.
   * Refer to the function specific documentation below for usage guidelines.
   */
/* *
   * @addtogroup inv_park
   * @{
   */
/* *
   * @brief  Floating-point Inverse Park transform
   * @param[in]  Id       input coordinate of rotor reference frame d
   * @param[in]  Iq       input coordinate of rotor reference frame q
   * @param[out] pIalpha  points to output two-phase orthogonal vector axis alpha
   * @param[out] pIbeta   points to output two-phase orthogonal vector axis beta
   * @param[in]  sinVal   sine value of rotation angle theta
   * @param[in]  cosVal   cosine value of rotation angle theta
   */
/* Calculate pIalpha using the equation, pIalpha = Id * cosVal - Iq * sinVal */
/* Calculate pIbeta using the equation, pIbeta = Id * sinVal + Iq * cosVal */
/* *
   * @brief  Inverse Park transform for   Q31 version
   * @param[in]  Id       input coordinate of rotor reference frame d
   * @param[in]  Iq       input coordinate of rotor reference frame q
   * @param[out] pIalpha  points to output two-phase orthogonal vector axis alpha
   * @param[out] pIbeta   points to output two-phase orthogonal vector axis beta
   * @param[in]  sinVal   sine value of rotation angle theta
   * @param[in]  cosVal   cosine value of rotation angle theta
   *
   * <b>Scaling and Overflow Behavior:</b>
   * \par
   * The function is implemented using an internal 32-bit accumulator.
   * The accumulator maintains 1.31 format by truncating lower 31 bits of the intermediate multiplication in 2.62 format.
   * There is saturation on the addition, hence there is no risk of overflow.
   */
/* Temporary variables used to store intermediate results */
/* Temporary variables used to store intermediate results */
/* Intermediate product is calculated by (Id * cosVal) */
/* Intermediate product is calculated by (Iq * sinVal) */
/* Intermediate product is calculated by (Id * sinVal) */
/* Intermediate product is calculated by (Iq * cosVal) */
/* Calculate pIalpha by using the two intermediate products 1 and 2 */
/* Calculate pIbeta by using the two intermediate products 3 and 4 */
/* *
   * @} end of Inverse park group
   */
/* *
   * @brief  Converts the elements of the Q31 vector to floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[out] pDst       is output pointer
   * @param[in]  blockSize  is the number of samples to process
   */
/* *
   * @ingroup groupInterpolation
   */
/* *
   * @defgroup LinearInterpolate Linear Interpolation
   *
   * Linear interpolation is a method of curve fitting using linear polynomials.
   * Linear interpolation works by effectively drawing a straight line between two neighboring samples and returning the appropriate point along that line
   *
   * \par
   * \image html LinearInterp.gif "Linear interpolation"
   *
   * \par
   * A  Linear Interpolate function calculates an output value(y), for the input(x)
   * using linear interpolation of the input values x0, x1( nearest input values) and the output values y0 and y1(nearest output values)
   *
   * \par Algorithm:
   * <pre>
   *       y = y0 + (x - x0) * ((y1 - y0)/(x1-x0))
   *       where x0, x1 are nearest values of input x
   *             y0, y1 are nearest values to output y
   * </pre>
   *
   * \par
   * This set of functions implements Linear interpolation process
   * for Q7, Q15, Q31, and floating-point data types.  The functions operate on a single
   * sample of data and each call to the function returns a single processed value.
   * <code>S</code> points to an instance of the Linear Interpolate function data structure.
   * <code>x</code> is the input sample value. The functions returns the output value.
   *
   * \par
   * if x is outside of the table boundary, Linear interpolation returns first value of the table
   * if x is below input range and returns last value of table if x is above range.
   */
/* *
   * @addtogroup LinearInterpolate
   * @{
   */
/* *
   * @brief  Process function for the floating-point Linear Interpolation Function.
   * @param[in,out] S  is an instance of the floating-point Linear Interpolation structure
   * @param[in]     x  input sample to process
   * @return y processed output sample.
   *
   */
/* Nearest input values */
/* Nearest output values */
/* spacing between input values */
/* Index variable */
/* pointer to output table */
/* Calculation of index */
/* Iniatilize output for below specified range as least output value of table */
/* Iniatilize output for above specified range as last output value of table */
/* Calculation of nearest input values */
/* Read of nearest output values */
/* Calculation of output */
/* returns output value */
/* *
   *
   * @brief  Process function for the Q31 Linear Interpolation Function.
   * @param[in] pYData   pointer to Q31 Linear Interpolation table
   * @param[in] x        input sample to process
   * @param[in] nValues  number of table values
   * @return y processed output sample.
   *
   * \par
   * Input sample <code>x</code> is in 12.20 format which contains 12 bits for table index and 20 bits for fractional part.
   * This function can support maximum of table size 2^12.
   *
   */
/* output */
/* Nearest output values */
/* fractional part */
/* Index to read nearest output values */
/* Input is in 12.20 format */
    /* 12 bits for the table index */
    /* Index value calculation */
/* 20 bits for the fractional part */
      /* shift left by 11 to keep fract in 1.31 format */
/* Read two nearest output values from the index in 1.31(q31) format */
/* Calculation of y0 * (1-fract) and y is in 2.30 format */
/* Calculation of y0 * (1-fract) + y1 *fract and y is in 2.30 format */
/* Convert y to 1.31 format */
/* *
   *
   * @brief  Process function for the Q15 Linear Interpolation Function.
   * @param[in] pYData   pointer to Q15 Linear Interpolation table
   * @param[in] x        input sample to process
   * @param[in] nValues  number of table values
   * @return y processed output sample.
   *
   * \par
   * Input sample <code>x</code> is in 12.20 format which contains 12 bits for table index and 20 bits for fractional part.
   * This function can support maximum of table size 2^12.
   *
   */
/* output */
/* Nearest output values */
/* fractional part */
/* Index to read nearest output values */
/* Input is in 12.20 format */
    /* 12 bits for the table index */
    /* Index value calculation */
/* 20 bits for the fractional part */
      /* fract is in 12.20 format */
/* Read two nearest output values from the index */
/* Calculation of y0 * (1-fract) and y is in 13.35 format */
/* Calculation of (y0 * (1-fract) + y1 * fract) and y is in 13.35 format */
/* convert y to 1.15 format */
/* *
   *
   * @brief  Process function for the Q7 Linear Interpolation Function.
   * @param[in] pYData   pointer to Q7 Linear Interpolation table
   * @param[in] x        input sample to process
   * @param[in] nValues  number of table values
   * @return y processed output sample.
   *
   * \par
   * Input sample <code>x</code> is in 12.20 format which contains 12 bits for table index and 20 bits for fractional part.
   * This function can support maximum of table size 2^12.
   */
/* output */
/* Nearest output values */
/* fractional part */
/* Index to read nearest output values */
/* Input is in 12.20 format */
    /* 12 bits for the table index */
    /* Index value calculation */
/* 20 bits for the fractional part */
      /* fract is in 12.20 format */
/* Read two nearest output values from the index and are in 1.7(q7) format */
/* Calculation of y0 * (1-fract ) and y is in 13.27(q27) format */
/* Calculation of y1 * fract + y0 * (1-fract) and y is in 13.27(q27) format */
/* convert y to 1.7(q7) format */
/* *
   * @} end of LinearInterpolate group
   */
/* *
   * @brief  Fast approximation to the trigonometric sine function for floating-point data.
   * @param[in] x  input value in radians.
   * @return  sin(x).
   */
/* *
   * @brief  Fast approximation to the trigonometric sine function for Q31 data.
   * @param[in] x  Scaled input value in radians.
   * @return  sin(x).
   */
/* *
   * @brief  Fast approximation to the trigonometric sine function for Q15 data.
   * @param[in] x  Scaled input value in radians.
   * @return  sin(x).
   */
/* *
   * @brief  Fast approximation to the trigonometric cosine function for floating-point data.
   * @param[in] x  input value in radians.
   * @return  cos(x).
   */
/* *
   * @brief Fast approximation to the trigonometric cosine function for Q31 data.
   * @param[in] x  Scaled input value in radians.
   * @return  cos(x).
   */
/* *
   * @brief  Fast approximation to the trigonometric cosine function for Q15 data.
   * @param[in] x  Scaled input value in radians.
   * @return  cos(x).
   */
/* *
   * @ingroup groupFastMath
   */
/* *
   * @defgroup SQRT Square Root
   *
   * Computes the square root of a number.
   * There are separate functions for Q15, Q31, and floating-point data types.
   * The square root function is computed using the Newton-Raphson algorithm.
   * This is an iterative algorithm of the form:
   * <pre>
   *      x1 = x0 - f(x0)/f'(x0)
   * </pre>
   * where <code>x1</code> is the current estimate,
   * <code>x0</code> is the previous estimate, and
   * <code>f'(x0)</code> is the derivative of <code>f()</code> evaluated at <code>x0</code>.
   * For the square root function, the algorithm reduces to:
   * <pre>
   *     x0 = in/2                         [initial guess]
   *     x1 = 1/2 * ( x0 + in / x0)        [each iteration]
   * </pre>
   */
/* *
   * @addtogroup SQRT
   * @{
   */
/* *
   * @brief  Floating-point square root function.
   * @param[in]  in    input value.
   * @param[out] pOut  square root of input value.
   * @return The function returns ARM_MATH_SUCCESS if input value is positive value or ARM_MATH_ARGUMENT_ERROR if
   * <code>in</code> is negative value and returns zero output for negative values.
   */
/* *
   * @brief Q31 square root function.
   * @param[in]  in    input value.  The range of the input value is [0 +1) or 0x00000000 to 0x7FFFFFFF.
   * @param[out] pOut  square root of input value.
   * @return The function returns ARM_MATH_SUCCESS if input value is positive value or ARM_MATH_ARGUMENT_ERROR if
   * <code>in</code> is negative value and returns zero output for negative values.
   */
/* *
   * @brief  Q15 square root function.
   * @param[in]  in    input value.  The range of the input value is [0 +1) or 0x0000 to 0x7FFF.
   * @param[out] pOut  square root of input value.
   * @return The function returns ARM_MATH_SUCCESS if input value is positive value or ARM_MATH_ARGUMENT_ERROR if
   * <code>in</code> is negative value and returns zero output for negative values.
   */
/* *
   * @} end of SQRT group
   */
/* *
   * @brief floating-point Circular write function.
   */
/* Copy the value of Index pointer that points
     * to the current location where the input samples to be copied */
/* Loop over the blockSize */
/* copy the input sample to the circular buffer */
/* Update the input pointer */
/* Circularly update wOffset.  Watch out for positive and negative value */
/* Decrement the loop counter */
/* Update the index pointer */
/* *
   * @brief floating-point Circular Read function.
   */
/* Copy the value of Index pointer that points
     * to the current location from where the input samples to be read */
/* Loop over the blockSize */
/* copy the sample from the circular buffer to the destination buffer */
/* Update the input pointer */
/* Circularly update rOffset.  Watch out for positive and negative value  */
/* Decrement the loop counter */
/* Update the index pointer */
/* *
   * @brief Q15 Circular write function.
   */
/* Copy the value of Index pointer that points
     * to the current location where the input samples to be copied */
/* Loop over the blockSize */
/* copy the input sample to the circular buffer */
/* Update the input pointer */
/* Circularly update wOffset.  Watch out for positive and negative value */
/* Decrement the loop counter */
/* Update the index pointer */
/* *
   * @brief Q15 Circular Read function.
   */
/* Copy the value of Index pointer that points
     * to the current location from where the input samples to be read */
/* Loop over the blockSize */
/* copy the sample from the circular buffer to the destination buffer */
/* Update the input pointer */
/* Circularly update wOffset.  Watch out for positive and negative value */
/* Decrement the loop counter */
/* Update the index pointer */
/* *
   * @brief Q7 Circular write function.
   */
/* Copy the value of Index pointer that points
     * to the current location where the input samples to be copied */
/* Loop over the blockSize */
/* copy the input sample to the circular buffer */
/* Update the input pointer */
/* Circularly update wOffset.  Watch out for positive and negative value */
/* Decrement the loop counter */
/* Update the index pointer */
/* *
   * @brief Q7 Circular Read function.
   */
/* Copy the value of Index pointer that points
     * to the current location from where the input samples to be read */
/* Loop over the blockSize */
/* copy the sample from the circular buffer to the destination buffer */
/* Update the input pointer */
/* Circularly update rOffset.  Watch out for positive and negative value */
/* Decrement the loop counter */
/* Update the index pointer */
/* *
   * @brief  Sum of the squares of the elements of a Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Sum of the squares of the elements of a floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Sum of the squares of the elements of a Q15 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Sum of the squares of the elements of a Q7 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Mean value of a Q7 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Mean value of a Q15 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Mean value of a Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Mean value of a floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Variance of the elements of a floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Variance of the elements of a Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Variance of the elements of a Q15 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Root Mean Square of the elements of a floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Root Mean Square of the elements of a Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Root Mean Square of the elements of a Q15 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Standard deviation of the elements of a floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Standard deviation of the elements of a Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Standard deviation of the elements of a Q15 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output value.
   */
/* *
   * @brief  Floating-point complex magnitude
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
/* *
   * @brief  Q31 complex magnitude
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
/* *
   * @brief  Q15 complex magnitude
   * @param[in]  pSrc        points to the complex input vector
   * @param[out] pDst        points to the real output vector
   * @param[in]  numSamples  number of complex samples in the input vector
   */
/* *
   * @brief  Q15 complex dot product
   * @param[in]  pSrcA       points to the first input vector
   * @param[in]  pSrcB       points to the second input vector
   * @param[in]  numSamples  number of complex samples in each vector
   * @param[out] realResult  real part of the result returned here
   * @param[out] imagResult  imaginary part of the result returned here
   */
/* *
   * @brief  Q31 complex dot product
   * @param[in]  pSrcA       points to the first input vector
   * @param[in]  pSrcB       points to the second input vector
   * @param[in]  numSamples  number of complex samples in each vector
   * @param[out] realResult  real part of the result returned here
   * @param[out] imagResult  imaginary part of the result returned here
   */
/* *
   * @brief  Floating-point complex dot product
   * @param[in]  pSrcA       points to the first input vector
   * @param[in]  pSrcB       points to the second input vector
   * @param[in]  numSamples  number of complex samples in each vector
   * @param[out] realResult  real part of the result returned here
   * @param[out] imagResult  imaginary part of the result returned here
   */
/* *
   * @brief  Q15 complex-by-real multiplication
   * @param[in]  pSrcCmplx   points to the complex input vector
   * @param[in]  pSrcReal    points to the real input vector
   * @param[out] pCmplxDst   points to the complex output vector
   * @param[in]  numSamples  number of samples in each vector
   */
/* *
   * @brief  Q31 complex-by-real multiplication
   * @param[in]  pSrcCmplx   points to the complex input vector
   * @param[in]  pSrcReal    points to the real input vector
   * @param[out] pCmplxDst   points to the complex output vector
   * @param[in]  numSamples  number of samples in each vector
   */
/* *
   * @brief  Floating-point complex-by-real multiplication
   * @param[in]  pSrcCmplx   points to the complex input vector
   * @param[in]  pSrcReal    points to the real input vector
   * @param[out] pCmplxDst   points to the complex output vector
   * @param[in]  numSamples  number of samples in each vector
   */
/* *
   * @brief  Minimum value of a Q7 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] result     is output pointer
   * @param[in]  index      is the array index of the minimum value in the input buffer.
   */
/* *
   * @brief  Minimum value of a Q15 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output pointer
   * @param[in]  pIndex     is the array index of the minimum value in the input buffer.
   */
/* *
   * @brief  Minimum value of a Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output pointer
   * @param[out] pIndex     is the array index of the minimum value in the input buffer.
   */
/* *
   * @brief  Minimum value of a floating-point vector.
   * @param[in]  pSrc       is input pointer
   * @param[in]  blockSize  is the number of samples to process
   * @param[out] pResult    is output pointer
   * @param[out] pIndex     is the array index of the minimum value in the input buffer.
   */
/* *
 * @brief Maximum value of a Q7 vector.
 * @param[in]  pSrc       points to the input buffer
 * @param[in]  blockSize  length of the input vector
 * @param[out] pResult    maximum value returned here
 * @param[out] pIndex     index of maximum value returned here
 */
/* *
 * @brief Maximum value of a Q15 vector.
 * @param[in]  pSrc       points to the input buffer
 * @param[in]  blockSize  length of the input vector
 * @param[out] pResult    maximum value returned here
 * @param[out] pIndex     index of maximum value returned here
 */
/* *
 * @brief Maximum value of a Q31 vector.
 * @param[in]  pSrc       points to the input buffer
 * @param[in]  blockSize  length of the input vector
 * @param[out] pResult    maximum value returned here
 * @param[out] pIndex     index of maximum value returned here
 */
/* *
 * @brief Maximum value of a floating-point vector.
 * @param[in]  pSrc       points to the input buffer
 * @param[in]  blockSize  length of the input vector
 * @param[out] pResult    maximum value returned here
 * @param[out] pIndex     index of maximum value returned here
 */
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_max_f32.c
 * Description:  Maximum value of a floating-point vector
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
 * @ingroup groupStats
 */
/* *
 * @defgroup Max Maximum
 *
 * Computes the maximum value of an array of data.
 * The function returns both the maximum value and its position within the array.
 * There are separate functions for floating-point, Q31, Q15, and Q7 data types.
 */
/* *
 * @addtogroup Max
 * @{
 */
/* *
 * @brief Maximum value of a floating-point vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult maximum value returned here
 * @param[out]      *pIndex index of maximum value returned here
 * @return none.
 */
#[no_mangle]
pub unsafe extern "C" fn arm_max_f32(mut pSrc: *mut float32_t,
                                     mut blockSize: uint32_t,
                                     mut pResult: *mut float32_t,
                                     mut pIndex: *mut uint32_t) {
    /* Run the below code for Cortex-M4 and Cortex-M3 */
    let mut maxVal1: float32_t =
        0.; /* Temporary variables to store the output value. */
    let mut maxVal2: float32_t = 0.; /* loop counter */
    let mut out: float32_t = 0.;
    let mut blkCnt: uint32_t = 0;
    let mut outIndex: uint32_t = 0;
    let mut count: uint32_t = 0;
    /* Initialise the count value. */
    count = 0u32;
    /* Initialise the index value to zero. */
    outIndex = 0u32;
    /* Load first input value that act as reference value for comparision */
    let fresh0 = pSrc;
    pSrc = pSrc.offset(1);
    out = *fresh0;
    /* Loop unrolling */
    blkCnt = blockSize.wrapping_sub(1u32) >> 2u32;
    while blkCnt > 0u32 {
        /* Initialize maxVal to the next consecutive values one by one */
        let fresh1 = pSrc;
        pSrc = pSrc.offset(1);
        maxVal1 = *fresh1;
        let fresh2 = pSrc;
        pSrc = pSrc.offset(1);
        maxVal2 = *fresh2;
        /* compare for the maximum value */
        if out < maxVal1 {
            /* Update the maximum value and its index */
            out = maxVal1;
            outIndex = count.wrapping_add(1u32)
        }
        /* compare for the maximum value */
        if out < maxVal2 {
            /* Update the maximum value and its index */
            out = maxVal2;
            outIndex = count.wrapping_add(2u32)
        }
        /* Initialize maxVal to the next consecutive values one by one */
        let fresh3 = pSrc;
        pSrc = pSrc.offset(1);
        maxVal1 = *fresh3;
        let fresh4 = pSrc;
        pSrc = pSrc.offset(1);
        maxVal2 = *fresh4;
        /* compare for the maximum value */
        if out < maxVal1 {
            /* Update the maximum value and its index */
            out = maxVal1;
            outIndex = count.wrapping_add(3u32)
        }
        /* compare for the maximum value */
        if out < maxVal2 {
            /* Update the maximum value and its index */
            out = maxVal2;
            outIndex = count.wrapping_add(4u32)
        }
        count =
            (count as libc::c_uint).wrapping_add(4u32) as uint32_t as
                uint32_t;
        /* Decrement the loop counter */
        blkCnt = blkCnt.wrapping_sub(1)
    }
    /* if (blockSize - 1U) is not multiple of 4 */
    blkCnt = blockSize.wrapping_sub(1u32).wrapping_rem(4u32);
    /* #if defined (ARM_MATH_DSP) */
    while blkCnt > 0u32 {
        /* Initialize maxVal to the next consecutive values one by one */
        let fresh5 = pSrc;
        pSrc = pSrc.offset(1);
        maxVal1 = *fresh5;
        /* compare for the maximum value */
        if out < maxVal1 {
            /* Update the maximum value and it's index */
            out = maxVal1;
            outIndex = blockSize.wrapping_sub(blkCnt)
        }
        /* Decrement the loop counter */
        blkCnt = blkCnt.wrapping_sub(1)
    }
    /* Store the maximum value and it's index into destination pointers */
    *pResult = out;
    *pIndex = outIndex;
}
/* *
 * @} end of Max group
 */
