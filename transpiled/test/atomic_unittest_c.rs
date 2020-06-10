use ::libc;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct barrierTrace {
    pub enter: libc::c_int,
    pub leave: libc::c_int,
}
#[no_mangle]
pub unsafe extern "C" fn testAtomicBarrier_C(mut b0: *mut barrierTrace,
                                             mut b1: *mut barrierTrace,
                                             mut sample:
                                                 *mut [barrierTrace; 2])
 -> libc::c_int {
    let mut sIdx: libc::c_int = 0 as libc::c_int;
    // replace barrier macros to track barrier invocation
// pass known struct as barrier variable, keep track inside it
    (*b0).enter = 0 as libc::c_int;
    (*b0).leave = 0 as libc::c_int;
    (*b1).enter = 0 as libc::c_int;
    (*b1).leave = 0 as libc::c_int;
    (*sample.offset(sIdx as isize))[0 as libc::c_int as usize] = *b0;
    (*sample.offset(sIdx as isize))[1 as libc::c_int as usize] = *b1;
    sIdx += 1;
    ATOMIC_BARRIER(*b0);
    ATOMIC_BARRIER(*b1);
    (*sample.offset(sIdx as isize))[0 as libc::c_int as usize] = *b0;
    (*sample.offset(sIdx as isize))[1 as libc::c_int as usize] = *b1;
    sIdx += 1;
    ATOMIC_BARRIER(*b0);
    (*sample.offset(sIdx as isize))[0 as libc::c_int as usize] = *b0;
    (*sample.offset(sIdx as isize))[1 as libc::c_int as usize] = *b1;
    sIdx += 1;
    (*sample.offset(sIdx as isize))[0 as libc::c_int as usize] = *b0;
    (*sample.offset(sIdx as isize))[1 as libc::c_int as usize] = *b1;
    sIdx += 1;
    (*sample.offset(sIdx as isize))[0 as libc::c_int as usize] = *b0;
    (*sample.offset(sIdx as isize))[1 as libc::c_int as usize] = *b1;
    sIdx += 1;
    return sIdx;
    // ATOMIC_BARRIER is broken in rest of this file
}
