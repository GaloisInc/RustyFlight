use ::libc;
extern "C" {
    #[no_mangle]
    fn sqrtf(_: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn fabsf(_: libc::c_float) -> libc::c_float;
}
pub type __int16_t = libc::c_short;
pub type __int32_t = libc::c_int;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
pub type fix12_t = int32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct stdev_s {
    pub m_oldM: libc::c_float,
    pub m_newM: libc::c_float,
    pub m_oldS: libc::c_float,
    pub m_newS: libc::c_float,
    pub m_n: libc::c_int,
}
pub type stdev_t = stdev_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct fp_vector {
    pub X: libc::c_float,
    pub Y: libc::c_float,
    pub Z: libc::c_float,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct fp_angles {
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
}
// Floating point Euler angles.
// Be carefull, could be either of degrees or radians.
pub type fp_angles_def = fp_angles;
#[derive(Copy, Clone)]
#[repr(C)]
pub union fp_angles_t {
    pub raw: [libc::c_float; 3],
    pub angles: fp_angles_def,
}
#[no_mangle]
pub unsafe extern "C" fn sin_approx(mut x: libc::c_float) -> libc::c_float {
    let mut xint: int32_t =
        x as int32_t; // Stop here on error input (5 * 360 Deg)
    if xint < -(32 as libc::c_int) || xint > 32 as libc::c_int {
        return 0.0f32
    } // always wrap input angle to -PI..PI
    while x > 3.14159265358979323846f32 {
        x -= 2.0f32 * 3.14159265358979323846f32
    } // We just pick -90..+90 Degree
    while x < -3.14159265358979323846f32 {
        x += 2.0f32 * 3.14159265358979323846f32
    }
    if x > 0.5f32 * 3.14159265358979323846f32 {
        x =
            0.5f32 * 3.14159265358979323846f32 -
                (x - 0.5f32 * 3.14159265358979323846f32)
    } else if x < -(0.5f32 * 3.14159265358979323846f32) {
        x =
            -(0.5f32 * 3.14159265358979323846f32) -
                (0.5f32 * 3.14159265358979323846f32 + x)
    }
    let mut x2: libc::c_float = x * x;
    return x +
               x * x2 *
                   (-1.666568107e-1f32 +
                        x2 *
                            (8.312366210e-3f32 +
                                 x2 *
                                     (-1.849218155e-4f32 +
                                          x2 *
                                              0 as libc::c_int as
                                                  libc::c_float)));
}
#[no_mangle]
pub unsafe extern "C" fn cos_approx(mut x: libc::c_float) -> libc::c_float {
    return sin_approx(x + 0.5f32 * 3.14159265358979323846f32);
}
// Initial implementation by Crashpilot1000 (https://github.com/Crashpilot1000/HarakiriWebstore1/blob/396715f73c6fcf859e0db0f34e12fe44bace6483/src/mw.c#L1292)
// Polynomial coefficients by Andor (http://www.dsprelated.com/showthread/comp.dsp/21872-1.php) optimized by Ledvinap to save one multiplication
// Max absolute error 0,000027 degree
// atan2_approx maximum absolute error = 7.152557e-07 rads (4.098114e-05 degree)
#[no_mangle]
pub unsafe extern "C" fn atan2_approx(mut y: libc::c_float,
                                      mut x: libc::c_float) -> libc::c_float {
    let mut res: libc::c_float = 0.;
    let mut absX: libc::c_float = 0.;
    let mut absY: libc::c_float = 0.;
    absX = fabsf(x);
    absY = fabsf(y);
    res =
        ({
             let mut _a: libc::c_float = absX;
             let mut _b: libc::c_float = absY;
             if _a > _b { _a } else { _b }
         });
    if res != 0. {
        res =
            ({
                 let mut _a: libc::c_float = absX;
                 let mut _b: libc::c_float = absY;
                 (if _a < _b { _a } else { _b })
             }) / res
    } else { res = 0.0f32 }
    res =
        -((((0.05030176425872175f32 * res - 0.3099814292351353f32) * res -
                0.14744007058297684f32) * res - 0.99997356613987f32) * res -
              3.14551665884836e-07f32) /
            ((0.6444640676891548f32 * res + 0.1471039133652469f32) * res +
                 1.0f32);
    if absY > absX { res = 3.14159265358979323846f32 / 2.0f32 - res }
    if x < 0 as libc::c_int as libc::c_float {
        res = 3.14159265358979323846f32 - res
    }
    if y < 0 as libc::c_int as libc::c_float { res = -res }
    return res;
}
// http://http.developer.nvidia.com/Cg/acos.html
// Handbook of Mathematical Functions
// M. Abramowitz and I.A. Stegun, Ed.
// acos_approx maximum absolute error = 6.760856e-05 rads (3.873685e-03 degree)
#[no_mangle]
pub unsafe extern "C" fn acos_approx(mut x: libc::c_float) -> libc::c_float {
    let mut xa: libc::c_float = fabsf(x);
    let mut result: libc::c_float =
        sqrtf(1.0f32 - xa) *
            (1.5707288f32 +
                 xa *
                     (-0.2121144f32 +
                          xa * (0.0742610f32 + -0.0187293f32 * xa)));
    if x < 0.0f32 {
        return 3.14159265358979323846f32 - result
    } else { return result };
}
#[no_mangle]
pub unsafe extern "C" fn gcd(mut num: libc::c_int, mut denom: libc::c_int)
 -> libc::c_int {
    if denom == 0 as libc::c_int { return num }
    return gcd(denom, num % denom);
}
#[no_mangle]
pub unsafe extern "C" fn powerf(mut base: libc::c_float, mut exp: libc::c_int)
 -> libc::c_float {
    let mut result: libc::c_float = base;
    let mut count: libc::c_int = 1 as libc::c_int;
    while count < exp { result *= base; count += 1 }
    return result;
}
#[no_mangle]
pub unsafe extern "C" fn applyDeadband(value: int32_t, deadband: int32_t)
 -> int32_t {
    if ({
            let _x: int32_t = value;
            (if _x > 0 as libc::c_int { _x } else { -_x })
        }) < deadband {
        return 0 as libc::c_int
    }
    return if value >= 0 as libc::c_int {
               (value) - deadband
           } else { (value) + deadband };
}
#[no_mangle]
pub unsafe extern "C" fn fapplyDeadband(value: libc::c_float,
                                        deadband: libc::c_float)
 -> libc::c_float {
    if fabsf(value) < deadband { return 0 as libc::c_int as libc::c_float }
    return if value >= 0 as libc::c_int as libc::c_float {
               (value) - deadband
           } else { (value) + deadband };
}
#[no_mangle]
pub unsafe extern "C" fn devClear(mut dev: *mut stdev_t) {
    (*dev).m_n = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn devPush(mut dev: *mut stdev_t,
                                 mut x: libc::c_float) {
    (*dev).m_n += 1;
    if (*dev).m_n == 1 as libc::c_int {
        (*dev).m_newM = x;
        (*dev).m_oldM = (*dev).m_newM;
        (*dev).m_oldS = 0.0f32
    } else {
        (*dev).m_newM =
            (*dev).m_oldM + (x - (*dev).m_oldM) / (*dev).m_n as libc::c_float;
        (*dev).m_newS =
            (*dev).m_oldS + (x - (*dev).m_oldM) * (x - (*dev).m_newM);
        (*dev).m_oldM = (*dev).m_newM;
        (*dev).m_oldS = (*dev).m_newS
    };
}
#[no_mangle]
pub unsafe extern "C" fn devVariance(mut dev: *mut stdev_t) -> libc::c_float {
    return if (*dev).m_n > 1 as libc::c_int {
               ((*dev).m_newS) /
                   ((*dev).m_n - 1 as libc::c_int) as libc::c_float
           } else { 0.0f32 };
}
#[no_mangle]
pub unsafe extern "C" fn devStandardDeviation(mut dev: *mut stdev_t)
 -> libc::c_float {
    return sqrtf(devVariance(dev));
}
#[no_mangle]
pub unsafe extern "C" fn degreesToRadians(mut degrees: int16_t)
 -> libc::c_float {
    return degrees as libc::c_int as libc::c_float *
               (3.14159265358979323846f32 / 180.0f32);
}
#[no_mangle]
pub unsafe extern "C" fn scaleRange(mut x: libc::c_int,
                                    mut srcFrom: libc::c_int,
                                    mut srcTo: libc::c_int,
                                    mut destFrom: libc::c_int,
                                    mut destTo: libc::c_int) -> libc::c_int {
    let mut a: libc::c_long =
        (destTo as libc::c_long - destFrom as libc::c_long) *
            (x as libc::c_long - srcFrom as libc::c_long);
    let mut b: libc::c_long = srcTo as libc::c_long - srcFrom as libc::c_long;
    return (a / b + destFrom as libc::c_long) as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn scaleRangef(mut x: libc::c_float,
                                     mut srcFrom: libc::c_float,
                                     mut srcTo: libc::c_float,
                                     mut destFrom: libc::c_float,
                                     mut destTo: libc::c_float)
 -> libc::c_float {
    let mut a: libc::c_float = (destTo - destFrom) * (x - srcFrom);
    let mut b: libc::c_float = srcTo - srcFrom;
    return a / b + destFrom;
}
// Normalize a vector
#[no_mangle]
pub unsafe extern "C" fn normalizeV(mut src: *mut fp_vector,
                                    mut dest: *mut fp_vector) {
    let mut length: libc::c_float = 0.;
    length =
        sqrtf((*src).X * (*src).X + (*src).Y * (*src).Y +
                  (*src).Z * (*src).Z);
    if length != 0 as libc::c_int as libc::c_float {
        (*dest).X = (*src).X / length;
        (*dest).Y = (*src).Y / length;
        (*dest).Z = (*src).Z / length
    };
}
#[no_mangle]
pub unsafe extern "C" fn buildRotationMatrix(mut delta: *mut fp_angles_t,
                                             mut matrix:
                                                 *mut [libc::c_float; 3]) {
    let mut cosx: libc::c_float = 0.;
    let mut sinx: libc::c_float = 0.;
    let mut cosy: libc::c_float = 0.;
    let mut siny: libc::c_float = 0.;
    let mut cosz: libc::c_float = 0.;
    let mut sinz: libc::c_float = 0.;
    let mut coszcosx: libc::c_float = 0.;
    let mut sinzcosx: libc::c_float = 0.;
    let mut coszsinx: libc::c_float = 0.;
    let mut sinzsinx: libc::c_float = 0.;
    cosx = cos_approx((*delta).angles.roll);
    sinx = sin_approx((*delta).angles.roll);
    cosy = cos_approx((*delta).angles.pitch);
    siny = sin_approx((*delta).angles.pitch);
    cosz = cos_approx((*delta).angles.yaw);
    sinz = sin_approx((*delta).angles.yaw);
    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;
    (*matrix.offset(0 as libc::c_int as isize))[X as libc::c_int as usize] =
        cosz * cosy;
    (*matrix.offset(0 as libc::c_int as isize))[Y as libc::c_int as usize] =
        -cosy * sinz;
    (*matrix.offset(0 as libc::c_int as isize))[Z as libc::c_int as usize] =
        siny;
    (*matrix.offset(1 as libc::c_int as isize))[X as libc::c_int as usize] =
        sinzcosx + coszsinx * siny;
    (*matrix.offset(1 as libc::c_int as isize))[Y as libc::c_int as usize] =
        coszcosx - sinzsinx * siny;
    (*matrix.offset(1 as libc::c_int as isize))[Z as libc::c_int as usize] =
        -sinx * cosy;
    (*matrix.offset(2 as libc::c_int as isize))[X as libc::c_int as usize] =
        sinzsinx - coszcosx * siny;
    (*matrix.offset(2 as libc::c_int as isize))[Y as libc::c_int as usize] =
        coszsinx + sinzcosx * siny;
    (*matrix.offset(2 as libc::c_int as isize))[Z as libc::c_int as usize] =
        cosy * cosx;
}
// Rotate a vector *v by the euler angles defined by the 3-vector *delta.
#[no_mangle]
pub unsafe extern "C" fn rotateV(mut v: *mut fp_vector,
                                 mut delta: *mut fp_angles_t) {
    let mut v_tmp: fp_vector = *v;
    let mut matrix: [[libc::c_float; 3]; 3] = [[0.; 3]; 3];
    buildRotationMatrix(delta, matrix.as_mut_ptr());
    (*v).X =
        v_tmp.X * matrix[0 as libc::c_int as usize][X as libc::c_int as usize]
            +
            v_tmp.Y *
                matrix[1 as libc::c_int as usize][X as libc::c_int as usize] +
            v_tmp.Z *
                matrix[2 as libc::c_int as usize][X as libc::c_int as usize];
    (*v).Y =
        v_tmp.X * matrix[0 as libc::c_int as usize][Y as libc::c_int as usize]
            +
            v_tmp.Y *
                matrix[1 as libc::c_int as usize][Y as libc::c_int as usize] +
            v_tmp.Z *
                matrix[2 as libc::c_int as usize][Y as libc::c_int as usize];
    (*v).Z =
        v_tmp.X * matrix[0 as libc::c_int as usize][Z as libc::c_int as usize]
            +
            v_tmp.Y *
                matrix[1 as libc::c_int as usize][Z as libc::c_int as usize] +
            v_tmp.Z *
                matrix[2 as libc::c_int as usize][Z as libc::c_int as usize];
}
// Quick median filter implementation
// (c) N. Devillard - 1998
// http://ndevilla.free.fr/median/median.pdf
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter3(mut v: *mut int32_t) -> int32_t {
    let mut p: [int32_t; 3] = [0; 3];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_0: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_0
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp_1: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp_1
    }
    return p[1 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter5(mut v: *mut int32_t) -> int32_t {
    let mut p: [int32_t; 5] = [0; 5];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 5 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp
    }
    if p[3 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_0: int32_t = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_0
    }
    if p[0 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_1: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_1
    }
    if p[1 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_2: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_2
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_3: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_3
    }
    if p[2 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_4: int32_t = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_4
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_5: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_5
    }
    return p[2 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter7(mut v: *mut int32_t) -> int32_t {
    let mut p: [int32_t; 7] = [0; 7];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 7 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[0 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp
    }
    if p[0 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_0: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_0
    }
    if p[1 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_1: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_1
    }
    if p[2 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_2: int32_t = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_2
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp_3: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp_3
    }
    if p[3 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_4: int32_t = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_4
    }
    if p[2 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_5: int32_t = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_5
    }
    if p[2 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_6: int32_t = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_6
    }
    if p[3 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_7: int32_t = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_7
    }
    if p[4 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_8: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_8
    }
    if p[1 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_9: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_9
    }
    if p[1 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_10: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_10
    }
    if p[3 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_11: int32_t = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_11
    }
    return p[3 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter9(mut v: *mut int32_t) -> int32_t {
    let mut p: [int32_t; 9] = [0; 9];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 9 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp
    }
    if p[4 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_0: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_0
    }
    if p[7 as libc::c_int as usize] > p[8 as libc::c_int as usize] {
        let mut temp_1: int32_t = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = p[8 as libc::c_int as usize];
        p[8 as libc::c_int as usize] = temp_1
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp_2: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp_2
    }
    if p[3 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_3: int32_t = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_3
    }
    if p[6 as libc::c_int as usize] > p[7 as libc::c_int as usize] {
        let mut temp_4: int32_t = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = temp_4
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_5: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_5
    }
    if p[4 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_6: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_6
    }
    if p[7 as libc::c_int as usize] > p[8 as libc::c_int as usize] {
        let mut temp_7: int32_t = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = p[8 as libc::c_int as usize];
        p[8 as libc::c_int as usize] = temp_7
    }
    if p[0 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_8: int32_t = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_8
    }
    if p[5 as libc::c_int as usize] > p[8 as libc::c_int as usize] {
        let mut temp_9: int32_t = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = p[8 as libc::c_int as usize];
        p[8 as libc::c_int as usize] = temp_9
    }
    if p[4 as libc::c_int as usize] > p[7 as libc::c_int as usize] {
        let mut temp_10: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = temp_10
    }
    if p[3 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_11: int32_t = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_11
    }
    if p[1 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_12: int32_t = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_12
    }
    if p[2 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_13: int32_t = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_13
    }
    if p[4 as libc::c_int as usize] > p[7 as libc::c_int as usize] {
        let mut temp_14: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = temp_14
    }
    if p[4 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_15: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_15
    }
    if p[6 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_16: int32_t = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_16
    }
    if p[4 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_17: int32_t = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_17
    }
    return p[4 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter3f(mut v: *mut libc::c_float)
 -> libc::c_float {
    let mut p: [libc::c_float; 3] = [0.; 3];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_0: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_0
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp_1: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp_1
    }
    return p[1 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter5f(mut v: *mut libc::c_float)
 -> libc::c_float {
    let mut p: [libc::c_float; 5] = [0.; 5];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 5 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp
    }
    if p[3 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_0: libc::c_float = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_0
    }
    if p[0 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_1: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_1
    }
    if p[1 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_2: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_2
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_3: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_3
    }
    if p[2 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_4: libc::c_float = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_4
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_5: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_5
    }
    return p[2 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter7f(mut v: *mut libc::c_float)
 -> libc::c_float {
    let mut p: [libc::c_float; 7] = [0.; 7];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 7 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[0 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp
    }
    if p[0 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_0: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_0
    }
    if p[1 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_1: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_1
    }
    if p[2 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_2: libc::c_float = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_2
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp_3: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp_3
    }
    if p[3 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_4: libc::c_float = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_4
    }
    if p[2 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_5: libc::c_float = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_5
    }
    if p[2 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_6: libc::c_float = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_6
    }
    if p[3 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_7: libc::c_float = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_7
    }
    if p[4 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_8: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_8
    }
    if p[1 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_9: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_9
    }
    if p[1 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_10: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_10
    }
    if p[3 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_11: libc::c_float = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_11
    }
    return p[3 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn quickMedianFilter9f(mut v: *mut libc::c_float)
 -> libc::c_float {
    let mut p: [libc::c_float; 9] = [0.; 9];
    let mut i: int32_t = 0;
    i = 0 as libc::c_int;
    while i < 9 as libc::c_int {
        p[i as usize] = *v.offset(i as isize);
        i += 1
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp
    }
    if p[4 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_0: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_0
    }
    if p[7 as libc::c_int as usize] > p[8 as libc::c_int as usize] {
        let mut temp_1: libc::c_float = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = p[8 as libc::c_int as usize];
        p[8 as libc::c_int as usize] = temp_1
    }
    if p[0 as libc::c_int as usize] > p[1 as libc::c_int as usize] {
        let mut temp_2: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = temp_2
    }
    if p[3 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_3: libc::c_float = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_3
    }
    if p[6 as libc::c_int as usize] > p[7 as libc::c_int as usize] {
        let mut temp_4: libc::c_float = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = temp_4
    }
    if p[1 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_5: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_5
    }
    if p[4 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_6: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_6
    }
    if p[7 as libc::c_int as usize] > p[8 as libc::c_int as usize] {
        let mut temp_7: libc::c_float = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = p[8 as libc::c_int as usize];
        p[8 as libc::c_int as usize] = temp_7
    }
    if p[0 as libc::c_int as usize] > p[3 as libc::c_int as usize] {
        let mut temp_8: libc::c_float = p[0 as libc::c_int as usize];
        p[0 as libc::c_int as usize] = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = temp_8
    }
    if p[5 as libc::c_int as usize] > p[8 as libc::c_int as usize] {
        let mut temp_9: libc::c_float = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = p[8 as libc::c_int as usize];
        p[8 as libc::c_int as usize] = temp_9
    }
    if p[4 as libc::c_int as usize] > p[7 as libc::c_int as usize] {
        let mut temp_10: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = temp_10
    }
    if p[3 as libc::c_int as usize] > p[6 as libc::c_int as usize] {
        let mut temp_11: libc::c_float = p[3 as libc::c_int as usize];
        p[3 as libc::c_int as usize] = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = temp_11
    }
    if p[1 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_12: libc::c_float = p[1 as libc::c_int as usize];
        p[1 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_12
    }
    if p[2 as libc::c_int as usize] > p[5 as libc::c_int as usize] {
        let mut temp_13: libc::c_float = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = p[5 as libc::c_int as usize];
        p[5 as libc::c_int as usize] = temp_13
    }
    if p[4 as libc::c_int as usize] > p[7 as libc::c_int as usize] {
        let mut temp_14: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[7 as libc::c_int as usize];
        p[7 as libc::c_int as usize] = temp_14
    }
    if p[4 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_15: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_15
    }
    if p[6 as libc::c_int as usize] > p[4 as libc::c_int as usize] {
        let mut temp_16: libc::c_float = p[6 as libc::c_int as usize];
        p[6 as libc::c_int as usize] = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = temp_16
    }
    if p[4 as libc::c_int as usize] > p[2 as libc::c_int as usize] {
        let mut temp_17: libc::c_float = p[4 as libc::c_int as usize];
        p[4 as libc::c_int as usize] = p[2 as libc::c_int as usize];
        p[2 as libc::c_int as usize] = temp_17
    }
    return p[4 as libc::c_int as usize];
}
#[no_mangle]
pub unsafe extern "C" fn arraySubInt32(mut dest: *mut int32_t,
                                       mut array1: *mut int32_t,
                                       mut array2: *mut int32_t,
                                       mut count: libc::c_int) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < count {
        *dest.offset(i as isize) =
            *array1.offset(i as isize) - *array2.offset(i as isize);
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn qPercent(mut q: fix12_t) -> int16_t {
    return (100 as libc::c_int * q >> 12 as libc::c_int) as int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn qMultiply(mut q: fix12_t, mut input: int16_t)
 -> int16_t {
    return (input as libc::c_int * q >> 12 as libc::c_int) as int16_t;
}
#[no_mangle]
pub unsafe extern "C" fn qConstruct(mut num: int16_t, mut den: int16_t)
 -> fix12_t {
    return ((num as libc::c_int) << 12 as libc::c_int) / den as libc::c_int;
}
