#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type uint8_t = libc::c_uchar;
pub type uint32_t = libc::c_uint;
// given
// and
// and
// and
// value lower that range minimum should be treated the same as the lowest range value
// value higher than the range maximum should be treated the same as the highest range value
// value equal to range step upper boundary should not activate the mode
// and
// when
// then
pub type C2RustUnnamed = libc::c_uint;
pub const COUNTER_CHANGE_CONTROL_RATE_PROFILE: C2RustUnnamed = 1;
pub const COUNTER_QUEUE_CONFIRMATION_BEEP: C2RustUnnamed = 0;
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
//#define DEBUG_RC_CONTROLS
#[no_mangle]
pub unsafe extern "C" fn unsetArmingDisabled(mut flag: libc::c_int) { }
#[no_mangle]
pub static mut RcControlsModesTest: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn TEST_F(mut RcControlsModesTest_0: libc::c_int,
                                mut updateActivatedModesWithAllInputsAtMidde:
                                    libc::c_int) -> libc::c_int {
    // given
    // and
    // and
    // when
    updateActivatedModes();
    panic!("Reached end of non-void function without returning");
    // then
}
static mut callCounts: [libc::c_int; 2] = [0; 2];
#[no_mangle]
pub unsafe extern "C" fn resetCallCounters() {
    memset(&mut callCounts as *mut [libc::c_int; 2] as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<[libc::c_int; 2]>() as libc::c_ulong);
}
#[no_mangle]
pub static mut fixedMillis: uint32_t = 0;
#[no_mangle]
pub unsafe extern "C" fn resetMillis() {
    fixedMillis = 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub static mut RcControlsAdjustmentsTest: libc::c_int = 0;
#[no_mangle]
pub static mut public: libc::c_int = 0;
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_0(mut RcControlsAdjustmentsTest_0:
                                      libc::c_int,
                                  mut processRcAdjustmentsSticksInMiddle:
                                      libc::c_int) -> libc::c_int {
    // given
    // and
    // and
    resetCallCounters();
    resetMillis();
    // when
    // then
    EXPECT_EQ(callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP as libc::c_int as
                             usize], 0 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_1(mut RcControlsAdjustmentsTest_0:
                                      libc::c_int,
                                  mut processRcAdjustmentsWithRcRateFunctionSwitchUp:
                                      libc::c_int) -> libc::c_int {
    // given
    // and
    // and
    // and
    // and
    resetCallCounters();
    resetMillis();
    // and
    // and
    let mut expectedAdjustmentStateMask: uint8_t =
        ((1 as libc::c_int) << 0 as libc::c_int) as uint8_t;
    // and
    fixedMillis = 496 as libc::c_int as uint32_t;
    // when
    // then
    EXPECT_EQ(callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP as libc::c_int as
                             usize], 1 as libc::c_int);
    //
    // now pretend a short amount of time has passed, but not enough time to allow the value to have been increased
    //
    // given
    fixedMillis = 497 as libc::c_int as uint32_t;
    // when
    //
    // moving the switch back to the middle should immediately reset the state flag without increasing the value
    //
    // given
    // and
    fixedMillis = 498 as libc::c_int as uint32_t;
    // and
    // when
    //
    // flipping the switch again, before the state reset would have occurred, allows the value to be increased again
    // given
    // and
    expectedAdjustmentStateMask =
        ((1 as libc::c_int) << 0 as libc::c_int) as uint8_t;
    // and
    fixedMillis = 499 as libc::c_int as uint32_t;
    // when
    // then
    EXPECT_EQ(callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP as libc::c_int as
                             usize], 2 as libc::c_int);
    //
    // leaving the switch up, after the original timer would have reset the state should now NOT cause
    // the rate to increase, it should only increase after another 500ms from when the state was reset.
    //
    // given
    fixedMillis = 500 as libc::c_int as uint32_t;
    // when
    // then
    //
    // should still not be able to be increased
    //
    // given
    fixedMillis = 997 as libc::c_int as uint32_t;
    // when
    // then
    //
    // 500ms has now passed since the switch was returned to the middle, now that
    // switch is still in the UP position after the timer has elapses it should
    // be increased again.
    //
    // given
    fixedMillis = 998 as libc::c_int as uint32_t;
    // when
    // then
    EXPECT_EQ(callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP as libc::c_int as
                             usize], 3 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_2(mut RcControlsAdjustmentsTest_0:
                                      libc::c_int,
                                  mut processRcRateProfileAdjustments:
                                      libc::c_int) -> libc::c_int {
    // given
    let mut adjustmentIndex: libc::c_int = 3 as libc::c_int;
    // and
    // and
    resetCallCounters();
    resetMillis();
    // and
    // and
    let mut expectedAdjustmentStateMask: uint8_t =
        ((1 as libc::c_int) << adjustmentIndex) as uint8_t;
    // when
    // then
    EXPECT_EQ(callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP as libc::c_int as
                             usize], 1 as libc::c_int);
    EXPECT_EQ(callCounts[COUNTER_CHANGE_CONTROL_RATE_PROFILE as libc::c_int as
                             usize], 1 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_3(mut RcControlsAdjustmentsTest_0:
                                      libc::c_int,
                                  mut processPIDIncreasePidController0:
                                      libc::c_int) -> libc::c_int {
    // given
    // and
    // and
    // and
    resetCallCounters();
    resetMillis();
    // and
    // and
    let mut expectedAdjustmentStateMask: uint8_t =
        ((1 as libc::c_int) << 0 as libc::c_int |
             (1 as libc::c_int) << 1 as libc::c_int |
             (1 as libc::c_int) << 2 as libc::c_int |
             (1 as libc::c_int) << 3 as libc::c_int |
             (1 as libc::c_int) << 4 as libc::c_int |
             (1 as libc::c_int) << 5 as libc::c_int) as uint8_t;
    // when
    // then
    EXPECT_EQ(callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP as libc::c_int as
                             usize], 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
    // and
}
// only one PID controller
#[no_mangle]
pub unsafe extern "C" fn getArmingDisableFlags() -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn isTryingToArm() -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn resetTryingToArm() { }
