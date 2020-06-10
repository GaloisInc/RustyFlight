use ::libc;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type float32_t = libc::c_float;
/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cfft_radix8_f32.c
 * Description:  Radix-8 Decimation in Frequency CFFT & CIFFT Floating point processing function
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
/* ----------------------------------------------------------------------
 * Internal helper function used by the FFTs
 * -------------------------------------------------------------------- */
/*
* @brief  Core function for the floating-point CFFT butterfly process.
* @param[in, out] *pSrc            points to the in-place buffer of floating-point data type.
* @param[in]      fftLen           length of the FFT.
* @param[in]      *pCoef           points to the twiddle coefficient buffer.
* @param[in]      twidCoefModifier twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table.
* @return none.
*/
#[no_mangle]
pub unsafe extern "C" fn arm_radix8_butterfly_f32(mut pSrc: *mut float32_t,
                                                  mut fftLen: uint16_t,
                                                  mut pCoef: *const float32_t,
                                                  mut twidCoefModifier:
                                                      uint16_t) {
    let mut ia1: uint32_t = 0;
    let mut ia2: uint32_t = 0;
    let mut ia3: uint32_t = 0;
    let mut ia4: uint32_t = 0;
    let mut ia5: uint32_t = 0;
    let mut ia6: uint32_t = 0;
    let mut ia7: uint32_t = 0;
    let mut i1: uint32_t = 0;
    let mut i2: uint32_t = 0;
    let mut i3: uint32_t = 0;
    let mut i4: uint32_t = 0;
    let mut i5: uint32_t = 0;
    let mut i6: uint32_t = 0;
    let mut i7: uint32_t = 0;
    let mut i8: uint32_t = 0;
    let mut id: uint32_t = 0;
    let mut n1: uint32_t = 0;
    let mut n2: uint32_t = 0;
    let mut j: uint32_t = 0;
    let mut r1: float32_t = 0.;
    let mut r2: float32_t = 0.;
    let mut r3: float32_t = 0.;
    let mut r4: float32_t = 0.;
    let mut r5: float32_t = 0.;
    let mut r6: float32_t = 0.;
    let mut r7: float32_t = 0.;
    let mut r8: float32_t = 0.;
    let mut t1: float32_t = 0.;
    let mut t2: float32_t = 0.;
    let mut s1: float32_t = 0.;
    let mut s2: float32_t = 0.;
    let mut s3: float32_t = 0.;
    let mut s4: float32_t = 0.;
    let mut s5: float32_t = 0.;
    let mut s6: float32_t = 0.;
    let mut s7: float32_t = 0.;
    let mut s8: float32_t = 0.;
    let mut p1: float32_t = 0.;
    let mut p2: float32_t = 0.;
    let mut p3: float32_t = 0.;
    let mut p4: float32_t = 0.;
    let mut co2: float32_t = 0.;
    let mut co3: float32_t = 0.;
    let mut co4: float32_t = 0.;
    let mut co5: float32_t = 0.;
    let mut co6: float32_t = 0.;
    let mut co7: float32_t = 0.;
    let mut co8: float32_t = 0.;
    let mut si2: float32_t = 0.;
    let mut si3: float32_t = 0.;
    let mut si4: float32_t = 0.;
    let mut si5: float32_t = 0.;
    let mut si6: float32_t = 0.;
    let mut si7: float32_t = 0.;
    let mut si8: float32_t = 0.;
    let C81: float32_t = 0.70710678118f32;
    n2 = fftLen as uint32_t;
    loop  {
        n1 = n2;
        n2 = n2 >> 3 as libc::c_int;
        i1 = 0 as libc::c_int as uint32_t;
        loop  {
            i2 = i1.wrapping_add(n2);
            i3 = i2.wrapping_add(n2);
            i4 = i3.wrapping_add(n2);
            i5 = i4.wrapping_add(n2);
            i6 = i5.wrapping_add(n2);
            i7 = i6.wrapping_add(n2);
            i8 = i7.wrapping_add(n2);
            r1 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i1) as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i5) as
                                     isize);
            r5 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i1) as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i5) as
                                     isize);
            r2 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i2) as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i6) as
                                     isize);
            r6 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i2) as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i6) as
                                     isize);
            r3 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i3) as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i7) as
                                     isize);
            r7 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i3) as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i7) as
                                     isize);
            r4 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i4) as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i8) as
                                     isize);
            r8 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i4) as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i8) as
                                     isize);
            t1 = r1 - r3;
            r1 = r1 + r3;
            r3 = r2 - r4;
            r2 = r2 + r4;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i1)
                             as isize) = r1 + r2;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i5)
                             as isize) = r1 - r2;
            r1 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i1).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i5).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            s5 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i1).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i5).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            r2 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i2).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i6).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            s6 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i2).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i6).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            s3 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i3).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i7).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            s7 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i3).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i7).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            r4 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i4).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) +
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i8).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            s8 =
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i4).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) -
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i8).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize);
            t2 = r1 - s3;
            r1 = r1 + s3;
            s3 = r2 - r4;
            r2 = r2 + r4;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i1).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = r1 + r2;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i5).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = r1 - r2;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i3)
                             as isize) = t1 + s3;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i7)
                             as isize) = t1 - s3;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i3).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = t2 - r3;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i7).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = t2 + r3;
            r1 = (r6 - r8) * C81;
            r6 = (r6 + r8) * C81;
            r2 = (s6 - s8) * C81;
            s6 = (s6 + s8) * C81;
            t1 = r5 - r1;
            r5 = r5 + r1;
            r8 = r7 - r6;
            r7 = r7 + r6;
            t2 = s5 - r2;
            s5 = s5 + r2;
            s8 = s7 - s6;
            s7 = s7 + s6;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i2)
                             as isize) = r5 + s7;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i8)
                             as isize) = r5 - s7;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i6)
                             as isize) = t1 + s8;
            *pSrc.offset((2 as libc::c_int as libc::c_uint).wrapping_mul(i4)
                             as isize) = t1 - s8;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i2).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = s5 - r7;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i8).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = s5 + r7;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i6).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = t2 - r8;
            *pSrc.offset((2 as libc::c_int as
                              libc::c_uint).wrapping_mul(i4).wrapping_add(1 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                             as isize) = t2 + r8;
            i1 =
                (i1 as libc::c_uint).wrapping_add(n1) as uint32_t as uint32_t;
            if !(i1 < fftLen as libc::c_uint) { break ; }
        }
        if n2 < 8 as libc::c_int as libc::c_uint { break ; }
        ia1 = 0 as libc::c_int as uint32_t;
        j = 1 as libc::c_int as uint32_t;
        loop  {
            /*  index calculation for the coefficients */
            id = ia1.wrapping_add(twidCoefModifier as libc::c_uint);
            ia1 = id;
            ia2 = ia1.wrapping_add(id);
            ia3 = ia2.wrapping_add(id);
            ia4 = ia3.wrapping_add(id);
            ia5 = ia4.wrapping_add(id);
            ia6 = ia5.wrapping_add(id);
            ia7 = ia6.wrapping_add(id);
            co2 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia1) as isize);
            co3 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia2) as isize);
            co4 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia3) as isize);
            co5 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia4) as isize);
            co6 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia5) as isize);
            co7 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia6) as isize);
            co8 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia7) as isize);
            si2 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia1).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            si3 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia2).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            si4 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia3).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            si5 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia4).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            si6 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia5).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            si7 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia6).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            si8 =
                *pCoef.offset((2 as libc::c_int as
                                   libc::c_uint).wrapping_mul(ia7).wrapping_add(1
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    libc::c_uint)
                                  as isize);
            i1 = j;
            loop  {
                /*  index calculation for the input */
                i2 = i1.wrapping_add(n2);
                i3 = i2.wrapping_add(n2);
                i4 = i3.wrapping_add(n2);
                i5 = i4.wrapping_add(n2);
                i6 = i5.wrapping_add(n2);
                i7 = i6.wrapping_add(n2);
                i8 = i7.wrapping_add(n2);
                r1 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i1) as isize)
                        +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i5) as
                                         isize);
                r5 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i1) as isize)
                        -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i5) as
                                         isize);
                r2 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i2) as isize)
                        +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i6) as
                                         isize);
                r6 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i2) as isize)
                        -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i6) as
                                         isize);
                r3 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i3) as isize)
                        +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i7) as
                                         isize);
                r7 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i3) as isize)
                        -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i7) as
                                         isize);
                r4 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i4) as isize)
                        +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i8) as
                                         isize);
                r8 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i4) as isize)
                        -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i8) as
                                         isize);
                t1 = r1 - r3;
                r1 = r1 + r3;
                r3 = r2 - r4;
                r2 = r2 + r4;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i1) as isize) =
                    r1 + r2;
                r2 = r1 - r2;
                s1 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i1).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i5).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s5 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i1).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i5).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s2 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i2).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i6).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s6 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i2).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i6).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s3 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i3).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i7).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s7 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i3).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i7).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s4 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i4).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) +
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i8).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                s8 =
                    *pSrc.offset((2 as libc::c_int as
                                      libc::c_uint).wrapping_mul(i4).wrapping_add(1
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      libc::c_uint)
                                     as isize) -
                        *pSrc.offset((2 as libc::c_int as
                                          libc::c_uint).wrapping_mul(i8).wrapping_add(1
                                                                                          as
                                                                                          libc::c_int
                                                                                          as
                                                                                          libc::c_uint)
                                         as isize);
                t2 = s1 - s3;
                s1 = s1 + s3;
                s3 = s2 - s4;
                s2 = s2 + s4;
                r1 = t1 + s3;
                t1 = t1 - s3;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i1).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = s1 + s2;
                s2 = s1 - s2;
                s1 = t2 - r3;
                t2 = t2 + r3;
                p1 = co5 * r2;
                p2 = si5 * s2;
                p3 = co5 * s2;
                p4 = si5 * r2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i5) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i5).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                p1 = co3 * r1;
                p2 = si3 * s1;
                p3 = co3 * s1;
                p4 = si3 * r1;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i3) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i3).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                p1 = co7 * t1;
                p2 = si7 * t2;
                p3 = co7 * t2;
                p4 = si7 * t1;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i7) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i7).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                r1 = (r6 - r8) * C81;
                r6 = (r6 + r8) * C81;
                s1 = (s6 - s8) * C81;
                s6 = (s6 + s8) * C81;
                t1 = r5 - r1;
                r5 = r5 + r1;
                r8 = r7 - r6;
                r7 = r7 + r6;
                t2 = s5 - s1;
                s5 = s5 + s1;
                s8 = s7 - s6;
                s7 = s7 + s6;
                r1 = r5 + s7;
                r5 = r5 - s7;
                r6 = t1 + s8;
                t1 = t1 - s8;
                s1 = s5 - r7;
                s5 = s5 + r7;
                s6 = t2 - r8;
                t2 = t2 + r8;
                p1 = co2 * r1;
                p2 = si2 * s1;
                p3 = co2 * s1;
                p4 = si2 * r1;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i2) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i2).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                p1 = co8 * r5;
                p2 = si8 * s5;
                p3 = co8 * s5;
                p4 = si8 * r5;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i8) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i8).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                p1 = co6 * r6;
                p2 = si6 * s6;
                p3 = co6 * s6;
                p4 = si6 * r6;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i6) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i6).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                p1 = co4 * t1;
                p2 = si4 * t2;
                p3 = co4 * t2;
                p4 = si4 * t1;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i4) as isize) =
                    p1 + p2;
                *pSrc.offset((2 as libc::c_int as
                                  libc::c_uint).wrapping_mul(i4).wrapping_add(1
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                                 as isize) = p3 - p4;
                i1 =
                    (i1 as libc::c_uint).wrapping_add(n1) as uint32_t as
                        uint32_t;
                if !(i1 < fftLen as libc::c_uint) { break ; }
            }
            j = j.wrapping_add(1);
            if !(j < n2) { break ; }
        }
        twidCoefModifier =
            ((twidCoefModifier as libc::c_int) << 3 as libc::c_int) as
                uint16_t;
        if !(n2 > 7 as libc::c_int as libc::c_uint) { break ; }
    };
}
