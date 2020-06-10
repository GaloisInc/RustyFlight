use ::libc;
pub type __int32_t = libc::c_int;
pub type int32_t = __int32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
    pub i: int32_t,
    pub f: libc::c_float,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub f: libc::c_float,
    pub i: int32_t,
}
/*

The MIT License (MIT)

Copyright (c) 2015 Jacques-Henri Jourdan <jourgun@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Taken from https://github.com/jhjourdan/SIMD-math-prims/blob/master/simd_math_prims.h
Stripped down for BF use

*/
/* Workaround a lack of optimization in gcc */
#[no_mangle]
pub static mut exp_cst1: libc::c_float = 2139095040.0f32;
#[no_mangle]
pub static mut exp_cst2: libc::c_float = 0.0f32;
/* Relative error bounded by 1e-5 for normalized outputs
   Returns invalid outputs for nan inputs
   Continuous error */
#[no_mangle]
pub unsafe extern "C" fn exp_approx(mut val: libc::c_float) -> libc::c_float {
    let mut xu: C2RustUnnamed =
        C2RustUnnamed{i:
                          0,}; // mask exponent  / round down to neareset 2^n (implicit mantisa bit)
    let mut xu2: C2RustUnnamed = C2RustUnnamed{i: 0,}; // force exponent to 0
    let mut val2: libc::c_float = 0.;
    let mut val3: libc::c_float = 0.;
    let mut val4: libc::c_float = 0.;
    let mut b: libc::c_float = 0.;
    let mut val4i: int32_t = 0;
    val2 = 12102203.1615614f32 * val + 1065353216.0f32;
    val3 = if val2 < exp_cst1 { val2 } else { exp_cst1 };
    val4 = if val3 > exp_cst2 { val3 } else { exp_cst2 };
    val4i = val4 as int32_t;
    xu.i = val4i & 0x7f800000 as libc::c_int;
    xu2.i = val4i & 0x7fffff as libc::c_int | 0x3f800000 as libc::c_int;
    b = xu2.f;
    /* Generated in Sollya with:
     > f=remez(1-x*exp(-(x-1)*log(2)),
               [|(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x|],
               [1.000001,1.999999], exp(-(x-1)*log(2)));
     > plot(exp((x-1)*log(2))/(f+x)-1, [1,2]);
     > f+x;
  */
    return xu.f *
               (0.509871020343597804469416f32 +
                    b *
                        (0.312146713032169896138863f32 +
                             b *
                                 (0.166617139319965966118107f32 +
                                      b *
                                          (-2.19061993049215080032874e-3f32 +
                                               b *
                                                   1.3555747234758484073940937e-2f32))));
}
/* Absolute error bounded by 1e-6 for normalized inputs
   Returns a finite number for +inf input
   Returns -inf for nan and <= 0 inputs.
   Continuous error. */
#[no_mangle]
pub unsafe extern "C" fn log_approx(mut val: libc::c_float) -> libc::c_float {
    let mut valu: C2RustUnnamed_0 = C2RustUnnamed_0{f: 0.,};
    let mut exp: libc::c_float = 0.;
    let mut addcst: libc::c_float = 0.;
    let mut x: libc::c_float = 0.;
    valu.f = val;
    exp = (valu.i >> 23 as libc::c_int) as libc::c_float;
    /* 89.970756366f = 127 * log(2) - constant term of polynomial */
    addcst =
        if val > 0 as libc::c_int as libc::c_float {
            -89.970756366f32
        } else { -::core::f32::INFINITY };
    valu.i = valu.i & 0x7fffff as libc::c_int | 0x3f800000 as libc::c_int;
    x = valu.f;
    /* Generated in Sollya using:
    > f = remez(log(x)-(x-1)*log(2),
            [|1,(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x,
              (x-1)*(x-2)*x*x*x|], [1,2], 1, 1e-8);
    > plot(f+(x-1)*log(2)-log(x), [1,2]);
    > f+(x-1)*log(2)
 */
    return x *
               (3.529304993f32 +
                    x *
                        (-2.461222105f32 +
                             x *
                                 (1.130626167f32 +
                                      x *
                                          (-0.288739945f32 +
                                               x * 3.110401639e-2f32)))) +
               (addcst + 0.69314718055995f32 * exp);
}
#[no_mangle]
pub unsafe extern "C" fn pow_approx(mut a: libc::c_float,
                                    mut b: libc::c_float) -> libc::c_float {
    return exp_approx(b * log_approx(a));
}
