use core;
use libc;
extern "C" {
    /* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cfft_f32.c
 * Description:  Combined Radix Decimation in Frequency CFFT Floating point processing function
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
    fn arm_radix8_butterfly_f32(pSrc: *mut float32_t, fftLen: uint16_t,
                                pCoef: *const float32_t,
                                twidCoefModifier: uint16_t);
    #[no_mangle]
    fn arm_bitreversal_32(pSrc: *mut uint32_t, bitRevLen: uint16_t,
                          pBitRevTable: *const uint16_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type float32_t = libc::c_float;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct arm_cfft_instance_f32 {
    pub fftLen: uint16_t,
    pub pTwiddle: *const float32_t,
    pub pBitRevTable: *const uint16_t,
    pub bitRevLength: uint16_t,
}
/* *
* @ingroup groupTransforms
*/
/* *
* @defgroup ComplexFFT Complex FFT Functions
*
* \par
* The Fast Fourier Transform (FFT) is an efficient algorithm for computing the
* Discrete Fourier Transform (DFT).  The FFT can be orders of magnitude faster
* than the DFT, especially for long lengths.
* The algorithms described in this section
* operate on complex data.  A separate set of functions is devoted to handling
* of real sequences.
* \par
* There are separate algorithms for handling floating-point, Q15, and Q31 data
* types.  The algorithms available for each data type are described next.
* \par
* The FFT functions operate in-place.  That is, the array holding the input data
* will also be used to hold the corresponding result.  The input data is complex
* and contains <code>2*fftLen</code> interleaved values as shown below.
* <pre> {real[0], imag[0], real[1], imag[1],..} </pre>
* The FFT result will be contained in the same array and the frequency domain
* values will have the same interleaving.
*
* \par Floating-point
* The floating-point complex FFT uses a mixed-radix algorithm.  Multiple radix-8
* stages are performed along with a single radix-2 or radix-4 stage, as needed.
* The algorithm supports lengths of [16, 32, 64, ..., 4096] and each length uses
* a different twiddle factor table.
* \par
* The function uses the standard FFT definition and output values may grow by a
* factor of <code>fftLen</code> when computing the forward transform.  The
* inverse transform includes a scale of <code>1/fftLen</code> as part of the
* calculation and this matches the textbook definition of the inverse FFT.
* \par
* Pre-initialized data structures containing twiddle factors and bit reversal
* tables are provided and defined in <code>arm_const_structs.h</code>.  Include
* this header in your function and then pass one of the constant structures as
* an argument to arm_cfft_f32.  For example:
* \par
* <code>arm_cfft_f32(arm_cfft_sR_f32_len64, pSrc, 1, 1)</code>
* \par
* computes a 64-point inverse complex FFT including bit reversal.
* The data structures are treated as constant data and not modified during the
* calculation.  The same data structure can be reused for multiple transforms
* including mixing forward and inverse transforms.
* \par
* Earlier releases of the library provided separate radix-2 and radix-4
* algorithms that operated on floating-point data.  These functions are still
* provided but are deprecated.  The older functions are slower and less general
* than the new functions.
* \par
* An example of initialization of the constants for the arm_cfft_f32 function follows:
* \code
* const static arm_cfft_instance_f32 *S;
* ...
*   switch (length) {
*     case 16:
*       S = &arm_cfft_sR_f32_len16;
*       break;
*     case 32:
*       S = &arm_cfft_sR_f32_len32;
*       break;
*     case 64:
*       S = &arm_cfft_sR_f32_len64;
*       break;
*     case 128:
*       S = &arm_cfft_sR_f32_len128;
*       break;
*     case 256:
*       S = &arm_cfft_sR_f32_len256;
*       break;
*     case 512:
*       S = &arm_cfft_sR_f32_len512;
*       break;
*     case 1024:
*       S = &arm_cfft_sR_f32_len1024;
*       break;
*     case 2048:
*       S = &arm_cfft_sR_f32_len2048;
*       break;
*     case 4096:
*       S = &arm_cfft_sR_f32_len4096;
*       break;
*   }
* \endcode
* \par Q15 and Q31
* The floating-point complex FFT uses a mixed-radix algorithm.  Multiple radix-4
* stages are performed along with a single radix-2 stage, as needed.
* The algorithm supports lengths of [16, 32, 64, ..., 4096] and each length uses
* a different twiddle factor table.
* \par
* The function uses the standard FFT definition and output values may grow by a
* factor of <code>fftLen</code> when computing the forward transform.  The
* inverse transform includes a scale of <code>1/fftLen</code> as part of the
* calculation and this matches the textbook definition of the inverse FFT.
* \par
* Pre-initialized data structures containing twiddle factors and bit reversal
* tables are provided and defined in <code>arm_const_structs.h</code>.  Include
* this header in your function and then pass one of the constant structures as
* an argument to arm_cfft_q31.  For example:
* \par
* <code>arm_cfft_q31(arm_cfft_sR_q31_len64, pSrc, 1, 1)</code>
* \par
* computes a 64-point inverse complex FFT including bit reversal.
* The data structures are treated as constant data and not modified during the
* calculation.  The same data structure can be reused for multiple transforms
* including mixing forward and inverse transforms.
* \par
* Earlier releases of the library provided separate radix-2 and radix-4
* algorithms that operated on floating-point data.  These functions are still
* provided but are deprecated.  The older functions are slower and less general
* than the new functions.
* \par
* An example of initialization of the constants for the arm_cfft_q31 function follows:
* \code
* const static arm_cfft_instance_q31 *S;
* ...
*   switch (length) {
*     case 16:
*       S = &arm_cfft_sR_q31_len16;
*       break;
*     case 32:
*       S = &arm_cfft_sR_q31_len32;
*       break;
*     case 64:
*       S = &arm_cfft_sR_q31_len64;
*       break;
*     case 128:
*       S = &arm_cfft_sR_q31_len128;
*       break;
*     case 256:
*       S = &arm_cfft_sR_q31_len256;
*       break;
*     case 512:
*       S = &arm_cfft_sR_q31_len512;
*       break;
*     case 1024:
*       S = &arm_cfft_sR_q31_len1024;
*       break;
*     case 2048:
*       S = &arm_cfft_sR_q31_len2048;
*       break;
*     case 4096:
*       S = &arm_cfft_sR_q31_len4096;
*       break;
*   }
* \endcode
*
*/
#[no_mangle]
pub unsafe extern "C" fn arm_cfft_radix8by2_f32(mut S:
                                                    *mut arm_cfft_instance_f32,
                                                mut p1: *mut float32_t) {
    let mut L: uint32_t = (*S).fftLen as uint32_t;
    let mut pCol1: *mut float32_t = 0 as *mut float32_t;
    let mut pCol2: *mut float32_t = 0 as *mut float32_t;
    let mut pMid1: *mut float32_t = 0 as *mut float32_t;
    let mut pMid2: *mut float32_t = 0 as *mut float32_t;
    let mut p2: *mut float32_t = p1.offset(L as isize);
    let mut tw: *const float32_t = (*S).pTwiddle as *mut float32_t;
    let mut t1: [float32_t; 4] = [0.; 4];
    let mut t2: [float32_t; 4] = [0.; 4];
    let mut t3: [float32_t; 4] = [0.; 4];
    let mut t4: [float32_t; 4] = [0.; 4];
    let mut twR: float32_t = 0.;
    let mut twI: float32_t = 0.;
    let mut m0: float32_t = 0.;
    let mut m1: float32_t = 0.;
    let mut m2: float32_t = 0.;
    let mut m3: float32_t = 0.;
    let mut l: uint32_t = 0;
    pCol1 = p1;
    pCol2 = p2;
    //    Define new length
    L >>= 1i32;
    //    Initialize mid pointers
    pMid1 = p1.offset(L as isize);
    pMid2 = p2.offset(L as isize);
    // do two dot Fourier transform
    l = L >> 2i32; // col 1
    while l > 0i32 as libc::c_uint {
        t1[0] = *p1.offset(0); // for col 2
        t1[1] = *p1.offset(1); // col 1
        t1[2] = *p1.offset(2); // for col 2
        t1[3] = *p1.offset(3);
        t2[0] = *p2.offset(0);
        t2[1] = *p2.offset(1);
        t2[2] = *p2.offset(2);
        t2[3] = *p2.offset(3);
        t3[0] = *pMid1.offset(0);
        t3[1] = *pMid1.offset(1);
        t3[2] = *pMid1.offset(2);
        t3[3] = *pMid1.offset(3);
        t4[0] = *pMid2.offset(0);
        t4[1] = *pMid2.offset(1);
        t4[2] = *pMid2.offset(2);
        t4[3] = *pMid2.offset(3);
        let fresh0 = p1;
        p1 = p1.offset(1);
        *fresh0 = t1[0] + t2[0];
        let fresh1 = p1;
        p1 = p1.offset(1);
        *fresh1 = t1[1] + t2[1];
        let fresh2 = p1;
        p1 = p1.offset(1);
        *fresh2 = t1[2] + t2[2];
        let fresh3 = p1;
        p1 = p1.offset(1);
        *fresh3 = t1[3] + t2[3];
        t2[0] = t1[0] - t2[0];
        t2[1] = t1[1] - t2[1];
        t2[2] = t1[2] - t2[2];
        t2[3] = t1[3] - t2[3];
        let fresh4 = pMid1;
        pMid1 = pMid1.offset(1);
        *fresh4 = t3[0] + t4[0];
        let fresh5 = pMid1;
        pMid1 = pMid1.offset(1);
        *fresh5 = t3[1] + t4[1];
        let fresh6 = pMid1;
        pMid1 = pMid1.offset(1);
        *fresh6 = t3[2] + t4[2];
        let fresh7 = pMid1;
        pMid1 = pMid1.offset(1);
        *fresh7 = t3[3] + t4[3];
        t4[0] = t4[0] - t3[0];
        t4[1] = t4[1] - t3[1];
        t4[2] = t4[2] - t3[2];
        t4[3] = t4[3] - t3[3];
        let fresh8 = tw;
        tw = tw.offset(1);
        twR = *fresh8;
        let fresh9 = tw;
        tw = tw.offset(1);
        twI = *fresh9;
        // multiply by twiddle factors
        m0 = t2[0] * twR;
        m1 = t2[1] * twI;
        m2 = t2[1] * twR;
        m3 = t2[0] * twI;
        // R  =  R  *  Tr - I * Ti
        let fresh10 = p2;
        p2 = p2.offset(1);
        *fresh10 = m0 + m1;
        // I  =  I  *  Tr + R * Ti
        let fresh11 = p2;
        p2 = p2.offset(1);
        *fresh11 = m2 - m3;
        // use vertical symmetry
        //  0.9988 - 0.0491i <==> -0.0491 - 0.9988i
        m0 = t4[0] * twI;
        m1 = t4[1] * twR;
        m2 = t4[1] * twI;
        m3 = t4[0] * twR;
        let fresh12 = pMid2;
        pMid2 = pMid2.offset(1);
        *fresh12 = m0 - m1;
        let fresh13 = pMid2;
        pMid2 = pMid2.offset(1);
        *fresh13 = m2 + m3;
        let fresh14 = tw;
        tw = tw.offset(1);
        twR = *fresh14;
        let fresh15 = tw;
        tw = tw.offset(1);
        twI = *fresh15;
        m0 = t2[2] * twR;
        m1 = t2[3] * twI;
        m2 = t2[3] * twR;
        m3 = t2[2] * twI;
        let fresh16 = p2;
        p2 = p2.offset(1);
        *fresh16 = m0 + m1;
        let fresh17 = p2;
        p2 = p2.offset(1);
        *fresh17 = m2 - m3;
        m0 = t4[2] * twI;
        m1 = t4[3] * twR;
        m2 = t4[3] * twI;
        m3 = t4[2] * twR;
        let fresh18 = pMid2;
        pMid2 = pMid2.offset(1);
        *fresh18 = m0 - m1;
        let fresh19 = pMid2;
        pMid2 = pMid2.offset(1);
        *fresh19 = m2 + m3;
        l = l.wrapping_sub(1)
    }
    // first col
    arm_radix8_butterfly_f32(pCol1, L as uint16_t,
                             (*S).pTwiddle as *mut float32_t,
                             2u32 as uint16_t);
    // second col
    arm_radix8_butterfly_f32(pCol2, L as uint16_t,
                             (*S).pTwiddle as *mut float32_t,
                             2u32 as
                                 uint16_t); // points to real values by default
}
#[no_mangle]
pub unsafe extern "C" fn arm_cfft_radix8by4_f32(mut S:
                                                    *mut arm_cfft_instance_f32,
                                                mut p1: *mut float32_t) {
    let mut L: uint32_t =
        ((*S).fftLen as libc::c_int >> 1i32) as
            uint32_t; // points to imaginary values by default
    let mut pCol1: *mut float32_t = 0 as *mut float32_t;
    let mut pCol2: *mut float32_t = 0 as *mut float32_t;
    let mut pCol3: *mut float32_t = 0 as *mut float32_t;
    let mut pCol4: *mut float32_t = 0 as *mut float32_t;
    let mut pEnd1: *mut float32_t = 0 as *mut float32_t;
    let mut pEnd2: *mut float32_t = 0 as *mut float32_t;
    let mut pEnd3: *mut float32_t = 0 as *mut float32_t;
    let mut pEnd4: *mut float32_t = 0 as *mut float32_t;
    let mut tw2: *const float32_t = 0 as *const float32_t;
    let mut tw3: *const float32_t = 0 as *const float32_t;
    let mut tw4: *const float32_t = 0 as *const float32_t;
    let mut p2: *mut float32_t = p1.offset(L as isize);
    let mut p3: *mut float32_t = p2.offset(L as isize);
    let mut p4: *mut float32_t = p3.offset(L as isize);
    let mut t2: [float32_t; 4] = [0.; 4];
    let mut t3: [float32_t; 4] = [0.; 4];
    let mut t4: [float32_t; 4] = [0.; 4];
    let mut twR: float32_t = 0.;
    let mut twI: float32_t = 0.;
    let mut p1ap3_0: float32_t = 0.;
    let mut p1sp3_0: float32_t = 0.;
    let mut p1ap3_1: float32_t = 0.;
    let mut p1sp3_1: float32_t = 0.;
    let mut m0: float32_t = 0.;
    let mut m1: float32_t = 0.;
    let mut m2: float32_t = 0.;
    let mut m3: float32_t = 0.;
    let mut l: uint32_t = 0;
    let mut twMod2: uint32_t = 0;
    let mut twMod3: uint32_t = 0;
    let mut twMod4: uint32_t = 0;
    pCol1 = p1;
    pCol2 = p2;
    pCol3 = p3;
    pCol4 = p4;
    pEnd1 = p2.offset(-1);
    pEnd2 = p3.offset(-1);
    pEnd3 = p4.offset(-1);
    pEnd4 = pEnd3.offset(L as isize);
    tw4 = (*S).pTwiddle as *mut float32_t;
    tw3 = tw4;
    tw2 = tw3;
    L >>= 1i32;
    // do four dot Fourier transform
    twMod2 = 2i32 as uint32_t;
    twMod3 = 4i32 as uint32_t;
    twMod4 = 6i32 as uint32_t;
    // TOP
    p1ap3_0 = *p1.offset(0) + *p3.offset(0);
    p1sp3_0 = *p1.offset(0) - *p3.offset(0);
    p1ap3_1 = *p1.offset(1) + *p3.offset(1);
    p1sp3_1 = *p1.offset(1) - *p3.offset(1);
    // col 2
    t2[0] = p1sp3_0 + *p2.offset(1) - *p4.offset(1);
    t2[1] = p1sp3_1 - *p2.offset(0) + *p4.offset(0);
    // col 3
    t3[0] = p1ap3_0 - *p2.offset(0) - *p4.offset(0);
    t3[1] = p1ap3_1 - *p2.offset(1) - *p4.offset(1);
    // col 4
    t4[0] = p1sp3_0 - *p2.offset(1) + *p4.offset(1);
    t4[1] = p1sp3_1 + *p2.offset(0) - *p4.offset(0);
    // col 1
    let fresh20 = p1;
    p1 = p1.offset(1);
    *fresh20 = p1ap3_0 + *p2.offset(0) + *p4.offset(0);
    let fresh21 = p1;
    p1 = p1.offset(1);
    *fresh21 = p1ap3_1 + *p2.offset(1) + *p4.offset(1);
    // Twiddle factors are ones
    let fresh22 = p2;
    p2 = p2.offset(1);
    *fresh22 = t2[0];
    let fresh23 = p2;
    p2 = p2.offset(1);
    *fresh23 = t2[1];
    let fresh24 = p3;
    p3 = p3.offset(1);
    *fresh24 = t3[0];
    let fresh25 = p3;
    p3 = p3.offset(1);
    *fresh25 = t3[1];
    let fresh26 = p4;
    p4 = p4.offset(1);
    *fresh26 = t4[0];
    let fresh27 = p4;
    p4 = p4.offset(1);
    *fresh27 = t4[1];
    tw2 = tw2.offset(twMod2 as isize);
    tw3 = tw3.offset(twMod3 as isize);
    tw4 = tw4.offset(twMod4 as isize);
    l = L.wrapping_sub(2i32 as libc::c_uint) >> 1i32;
    while l > 0i32 as libc::c_uint {
        // TOP
        p1ap3_0 = *p1.offset(0) + *p3.offset(0);
        p1sp3_0 = *p1.offset(0) - *p3.offset(0);
        p1ap3_1 = *p1.offset(1) + *p3.offset(1);
        p1sp3_1 = *p1.offset(1) - *p3.offset(1);
        // col 2
        t2[0] = p1sp3_0 + *p2.offset(1) - *p4.offset(1);
        t2[1] = p1sp3_1 - *p2.offset(0) + *p4.offset(0);
        // col 3
        t3[0] = p1ap3_0 - *p2.offset(0) - *p4.offset(0);
        t3[1] = p1ap3_1 - *p2.offset(1) - *p4.offset(1);
        // col 4
        t4[0] = p1sp3_0 - *p2.offset(1) + *p4.offset(1);
        t4[1] = p1sp3_1 + *p2.offset(0) - *p4.offset(0);
        // col 1 - top
        let fresh28 = p1;
        p1 = p1.offset(1);
        *fresh28 = p1ap3_0 + *p2.offset(0) + *p4.offset(0);
        let fresh29 = p1;
        p1 = p1.offset(1);
        *fresh29 = p1ap3_1 + *p2.offset(1) + *p4.offset(1);
        // BOTTOM
        p1ap3_1 =
            *pEnd1.offset(-1i32 as isize) + *pEnd3.offset(-1i32 as isize);
        p1sp3_1 =
            *pEnd1.offset(-1i32 as isize) - *pEnd3.offset(-1i32 as isize);
        p1ap3_0 = *pEnd1.offset(0) + *pEnd3.offset(0);
        p1sp3_0 = *pEnd1.offset(0) - *pEnd3.offset(0);
        // col 2
        t2[2] = *pEnd2.offset(0) - *pEnd4.offset(0) + p1sp3_1;
        t2[3] =
            *pEnd1.offset(0) - *pEnd3.offset(0) -
                *pEnd2.offset(-1i32 as isize) + *pEnd4.offset(-1i32 as isize);
        // col 3
        t3[2] =
            p1ap3_1 - *pEnd2.offset(-1i32 as isize) -
                *pEnd4.offset(-1i32 as isize);
        t3[3] = p1ap3_0 - *pEnd2.offset(0) - *pEnd4.offset(0);
        // col 4
        t4[2] = *pEnd2.offset(0) - *pEnd4.offset(0) - p1sp3_1;
        t4[3] =
            *pEnd4.offset(-1i32 as isize) - *pEnd2.offset(-1i32 as isize) -
                p1sp3_0;
        // col 1 - Bottom
        let fresh30 = pEnd1;
        pEnd1 = pEnd1.offset(-1);
        *fresh30 = p1ap3_0 + *pEnd2.offset(0) + *pEnd4.offset(0);
        let fresh31 = pEnd1;
        pEnd1 = pEnd1.offset(-1);
        *fresh31 =
            p1ap3_1 + *pEnd2.offset(-1i32 as isize) +
                *pEnd4.offset(-1i32 as isize);
        // COL 2
        // read twiddle factors
        let fresh32 = tw2;
        tw2 = tw2.offset(1);
        twR = *fresh32;
        let fresh33 = tw2;
        tw2 = tw2.offset(1);
        twI = *fresh33;
        // multiply by twiddle factors
        //  let    Z1 = a + i(b),   Z2 = c + i(d)
        //   =>  Z1 * Z2  =  (a*c - b*d) + i(b*c + a*d)
        // Top
        m0 = t2[0] * twR;
        m1 = t2[1] * twI;
        m2 = t2[1] * twR;
        m3 = t2[0] * twI;
        let fresh34 = p2;
        p2 = p2.offset(1);
        *fresh34 = m0 + m1;
        let fresh35 = p2;
        p2 = p2.offset(1);
        *fresh35 = m2 - m3;
        // use vertical symmetry col 2
        // 0.9997 - 0.0245i  <==>  0.0245 - 0.9997i
        // Bottom
        m0 = t2[3] * twI;
        m1 = t2[2] * twR;
        m2 = t2[2] * twI;
        m3 = t2[3] * twR;
        let fresh36 = pEnd2;
        pEnd2 = pEnd2.offset(-1);
        *fresh36 = m0 - m1;
        let fresh37 = pEnd2;
        pEnd2 = pEnd2.offset(-1);
        *fresh37 = m2 + m3;
        // COL 3
        twR = *tw3.offset(0);
        twI = *tw3.offset(1);
        tw3 = tw3.offset(twMod3 as isize);
        // Top
        m0 = t3[0] * twR;
        m1 = t3[1] * twI;
        m2 = t3[1] * twR;
        m3 = t3[0] * twI;
        let fresh38 = p3;
        p3 = p3.offset(1);
        *fresh38 = m0 + m1;
        let fresh39 = p3;
        p3 = p3.offset(1);
        *fresh39 = m2 - m3;
        // use vertical symmetry col 3
        // 0.9988 - 0.0491i  <==>  -0.9988 - 0.0491i
        // Bottom
        m0 = -t3[3] * twR;
        m1 = t3[2] * twI;
        m2 = t3[2] * twR;
        m3 = t3[3] * twI;
        let fresh40 = pEnd3;
        pEnd3 = pEnd3.offset(-1);
        *fresh40 = m0 - m1;
        let fresh41 = pEnd3;
        pEnd3 = pEnd3.offset(-1);
        *fresh41 = m3 - m2;
        // COL 4
        twR = *tw4.offset(0);
        twI = *tw4.offset(1);
        tw4 = tw4.offset(twMod4 as isize);
        // Top
        m0 = t4[0] * twR;
        m1 = t4[1] * twI;
        m2 = t4[1] * twR;
        m3 = t4[0] * twI;
        let fresh42 = p4;
        p4 = p4.offset(1);
        *fresh42 = m0 + m1;
        let fresh43 = p4;
        p4 = p4.offset(1);
        *fresh43 = m2 - m3;
        // use vertical symmetry col 4
        // 0.9973 - 0.0736i  <==>  -0.0736 + 0.9973i
        // Bottom
        m0 = t4[3] * twI;
        m1 = t4[2] * twR;
        m2 = t4[2] * twI;
        m3 = t4[3] * twR;
        let fresh44 = pEnd4;
        pEnd4 = pEnd4.offset(-1);
        *fresh44 = m0 - m1;
        let fresh45 = pEnd4;
        pEnd4 = pEnd4.offset(-1);
        *fresh45 = m2 + m3;
        l = l.wrapping_sub(1)
    }
    //MIDDLE
    // Twiddle factors are
    //  1.0000  0.7071-0.7071i  -1.0000i  -0.7071-0.7071i
    p1ap3_0 = *p1.offset(0) + *p3.offset(0);
    p1sp3_0 = *p1.offset(0) - *p3.offset(0);
    p1ap3_1 = *p1.offset(1) + *p3.offset(1);
    p1sp3_1 = *p1.offset(1) - *p3.offset(1);
    // col 2
    t2[0] = p1sp3_0 + *p2.offset(1) - *p4.offset(1);
    t2[1] = p1sp3_1 - *p2.offset(0) + *p4.offset(0);
    // col 3
    t3[0] = p1ap3_0 - *p2.offset(0) - *p4.offset(0);
    t3[1] = p1ap3_1 - *p2.offset(1) - *p4.offset(1);
    // col 4
    t4[0] = p1sp3_0 - *p2.offset(1) + *p4.offset(1);
    t4[1] = p1sp3_1 + *p2.offset(0) - *p4.offset(0);
    // col 1 - Top
    let fresh46 = p1;
    p1 = p1.offset(1);
    *fresh46 = p1ap3_0 + *p2.offset(0) + *p4.offset(0);
    let fresh47 = p1;
    p1 = p1.offset(1);
    *fresh47 = p1ap3_1 + *p2.offset(1) + *p4.offset(1);
    // COL 2
    twR = *tw2.offset(0);
    twI = *tw2.offset(1);
    m0 = t2[0] * twR;
    m1 = t2[1] * twI;
    m2 = t2[1] * twR;
    m3 = t2[0] * twI;
    let fresh48 = p2;
    p2 = p2.offset(1);
    *fresh48 = m0 + m1;
    let fresh49 = p2;
    p2 = p2.offset(1);
    *fresh49 = m2 - m3;
    // COL 3
    twR = *tw3.offset(0);
    twI = *tw3.offset(1);
    m0 = t3[0] * twR;
    m1 = t3[1] * twI;
    m2 = t3[1] * twR;
    m3 = t3[0] * twI;
    let fresh50 = p3;
    p3 = p3.offset(1);
    *fresh50 = m0 + m1;
    let fresh51 = p3;
    p3 = p3.offset(1);
    *fresh51 = m2 - m3;
    // COL 4
    twR = *tw4.offset(0);
    twI = *tw4.offset(1);
    m0 = t4[0] * twR;
    m1 = t4[1] * twI;
    m2 = t4[1] * twR;
    m3 = t4[0] * twI;
    let fresh52 = p4;
    p4 = p4.offset(1);
    *fresh52 = m0 + m1;
    let fresh53 = p4;
    p4 = p4.offset(1);
    *fresh53 = m2 - m3;
    // first col
    arm_radix8_butterfly_f32(pCol1, L as uint16_t,
                             (*S).pTwiddle as *mut float32_t,
                             4u32 as uint16_t);
    // second col
    arm_radix8_butterfly_f32(pCol2, L as uint16_t,
                             (*S).pTwiddle as *mut float32_t,
                             4u32 as uint16_t);
    // third col
    arm_radix8_butterfly_f32(pCol3, L as uint16_t,
                             (*S).pTwiddle as *mut float32_t,
                             4u32 as uint16_t);
    // fourth col
    arm_radix8_butterfly_f32(pCol4, L as uint16_t,
                             (*S).pTwiddle as *mut float32_t,
                             4u32 as uint16_t);
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
* @addtogroup ComplexFFT
* @{
*/
/* *
* @details
* @brief       Processing function for the floating-point complex FFT.
* @param[in]      *S    points to an instance of the floating-point CFFT structure.
* @param[in, out] *p1   points to the complex data buffer of size <code>2*fftLen</code>. Processing occurs in-place.
* @param[in]     ifftFlag       flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform.
* @param[in]     bitReverseFlag flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output.
* @return none.
*/
#[no_mangle]
pub unsafe extern "C" fn arm_cfft_f32(mut S: *const arm_cfft_instance_f32,
                                      mut p1: *mut float32_t,
                                      mut ifftFlag: uint8_t,
                                      mut bitReverseFlag: uint8_t) {
    let mut L: uint32_t = (*S).fftLen as uint32_t;
    let mut l: uint32_t = 0;
    let mut invL: float32_t = 0.;
    let mut pSrc: *mut float32_t = 0 as *mut float32_t;
    if ifftFlag as libc::c_uint == 1u32 {
        /*  Conjugate input data  */
        pSrc = p1.offset(1);
        l = 0i32 as uint32_t;
        while l < L {
            *pSrc = -*pSrc;
            pSrc = pSrc.offset(2);
            l = l.wrapping_add(1)
        }
    }
    match L {
        16 | 128 | 1024 => {
            arm_cfft_radix8by2_f32(S as *mut arm_cfft_instance_f32, p1);
        }
        32 | 256 | 2048 => {
            arm_cfft_radix8by4_f32(S as *mut arm_cfft_instance_f32, p1);
        }
        64 | 512 | 4096 => {
            arm_radix8_butterfly_f32(p1, L as uint16_t,
                                     (*S).pTwiddle as *mut float32_t,
                                     1i32 as uint16_t);
        }
        _ => { }
    }
    if bitReverseFlag != 0 {
        arm_bitreversal_32(p1 as *mut uint32_t, (*S).bitRevLength,
                           (*S).pBitRevTable);
    }
    if ifftFlag as libc::c_uint == 1u32 {
        invL = 1.0f32 / L as float32_t;
        /*  Conjugate and scale output data */
        pSrc = p1;
        l = 0i32 as uint32_t;
        while l < L {
            let fresh54 = pSrc;
            pSrc = pSrc.offset(1);
            *fresh54 *= invL;
            *pSrc = -*pSrc * invL;
            pSrc = pSrc.offset(1);
            l = l.wrapping_add(1)
        }
    };
}
/* *
* @} end of ComplexFFT group
*/
