#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
// reset BASEPRI
// locked
// priority increase
// restore priority
// lower priority, no change to value
// restore priority
// restore priority to unlocked
#[derive(Copy, Clone)]
#[repr(C)]
pub struct barrierTrace {
    pub enter: libc::c_int,
    pub leave: libc::c_int,
}
/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub unsafe extern "C" fn TEST(mut AtomicUnittest: libc::c_int,
                              mut TestAtomicBlock: libc::c_int)
 -> libc::c_int {
    // reset BASEPRI
    ATOMIC_BLOCK(10 as libc::c_int);
    // locked
    ATOMIC_BLOCK(5 as libc::c_int);
    // restore priority
    // priority increase
    // restore priority
    ATOMIC_BLOCK(20 as libc::c_int);
    panic!("Reached end of non-void function without returning");
    // restore priority to unlocked
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut AtomicUnittest: libc::c_int,
                                mut TestAtomicBarrier: libc::c_int)
 -> libc::c_int {
    let mut b0: barrierTrace = barrierTrace{enter: 0, leave: 0,};
    let mut b1: barrierTrace = barrierTrace{enter: 0, leave: 0,};
    let mut sample: [[barrierTrace; 2]; 10] =
        [[barrierTrace{enter: 0, leave: 0,}; 2]; 10];
    let mut sampled: libc::c_int =
        testAtomicBarrier_C(&mut b0, &mut b1, sample.as_mut_ptr());
    let mut sIdx: libc::c_int = 0 as libc::c_int;
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].enter,
              0 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].leave,
              0 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].enter,
              0 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].leave,
              0 as libc::c_int);
    sIdx += 1;
    // do {
    //   ATOMIC_BARRIER(*b0);
    //   ATOMIC_BARRIER(*b1);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].enter,
              1 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].leave,
              0 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].enter,
              1 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].leave,
              0 as libc::c_int);
    sIdx += 1;
    //    do {
    //        ATOMIC_BARRIER(*b0);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].enter,
              2 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].leave,
              0 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].enter,
              1 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].leave,
              0 as libc::c_int);
    sIdx += 1;
    //    } while(0);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].enter,
              2 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].leave,
              1 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].enter,
              1 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].leave,
              0 as libc::c_int);
    sIdx += 1;
    //} while(0);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].enter,
              2 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][0 as libc::c_int as usize].leave,
              2 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].enter,
              1 as libc::c_int);
    EXPECT_EQ(sample[sIdx as usize][1 as libc::c_int as usize].leave,
              1 as libc::c_int);
    sIdx += 1;
    //return sIdx;
    EXPECT_EQ(sIdx, sampled);
    panic!("Reached end of non-void function without returning");
}
