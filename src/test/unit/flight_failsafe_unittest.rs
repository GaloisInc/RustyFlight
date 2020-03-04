#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type uint16_t = libc::c_ushort;
pub type uint32_t = libc::c_uint;
pub type C2RustUnnamed = libc::c_uint;
pub const COUNTER_MW_DISARM: C2RustUnnamed = 0;
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
pub static mut testFeatureMask: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut testMinThrottle: uint16_t = 0 as libc::c_int as uint16_t;
#[no_mangle]
pub static mut throttleStatus: libc::c_int = 0;
static mut callCounts: [libc::c_int; 1] = [0; 1];
#[no_mangle]
pub unsafe extern "C" fn resetCallCounters() {
    memset(&mut callCounts as *mut [libc::c_int; 1] as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<[libc::c_int; 1]>() as libc::c_ulong);
}
#[no_mangle]
pub static mut sysTickUptime: uint32_t = 0;
#[no_mangle]
pub unsafe extern "C" fn configureFailsafe() {
    // 1 second
    // 5 seconds
    // 5 seconds
    sysTickUptime = 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn activateBoxFailsafe() { }
#[no_mangle]
pub unsafe extern "C" fn deactivateBoxFailsafe() { }
//
// Stepwise tests
//
/* ***************************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn TEST(mut FlightFailsafeTest: libc::c_int,
                              mut TestFailsafeInitialState: libc::c_int)
 -> libc::c_int {
    // given
    configureFailsafe();
    // and
    // when
    failsafeInit();
    failsafeReset();
    // then
    EXPECT_EQ(0 as libc::c_int, failsafeIsMonitoring());
    EXPECT_EQ(0 as libc::c_int, failsafeIsActive());
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
// when
// then
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeFirstArmedCycle: libc::c_int)
 -> libc::c_int {
    // given
    // when
    failsafeOnValidDataFailed(); // set last invalid sample at current time
    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived(); // cause a recovered link
    // and
    failsafeUpdateState();
    // then
    EXPECT_EQ(0 as libc::c_int, failsafeIsActive());
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeNotActivatedWhenReceivingData:
                                    libc::c_int) -> libc::c_int {
    // when
    sysTickUptime = 0 as libc::c_int as uint32_t;
    while sysTickUptime < 10000 as libc::c_int as libc::c_uint {
        failsafeOnValidDataReceived();
        failsafeUpdateState();
        // then
        EXPECT_EQ(0 as libc::c_int, failsafeIsActive());
        sysTickUptime = sysTickUptime.wrapping_add(1)
    }
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeDetectsRxLossAndStartsLanding:
                                    libc::c_int) -> libc::c_int {
    // given
    // and
    failsafeStartMonitoring();
    // throttle HIGH to go for a failsafe landing procedure
    sysTickUptime = 0 as libc::c_int as uint32_t; // restart time from 0
    failsafeOnValidDataReceived(); // set last valid sample at current time
    // when
    // then
    // given
    sysTickUptime =
        sysTickUptime.wrapping_add(1); // adjust time to point just past the failure time to
    failsafeOnValidDataFailed(); // cause a lost link
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeCausesLanding: libc::c_int)
 -> libc::c_int {
    // given
    sysTickUptime = sysTickUptime.wrapping_add(1);
    // when
    // no call to failsafeOnValidDataReceived();
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_TRUE(isArmingDisabled());
    // given
    failsafeOnValidDataFailed(); // set last invalid sample at current time
    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived(); // cause a recovered link
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_TRUE(isArmingDisabled());
    // given
    // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(0 as libc::c_int,
              failsafeIsActive()); // disarm not called repeatedly.
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeDetectsRxLossAndJustDisarms:
                                    libc::c_int) -> libc::c_int {
    // given
    resetCallCounters();
    // and
    failsafeStartMonitoring();
    // throttle LOW to go for a failsafe just-disarm procedure
    sysTickUptime = 0 as libc::c_int as uint32_t; // restart time from 0
    failsafeOnValidDataReceived(); // set last valid sample at current time
    // when
    // then
    // given
    sysTickUptime =
        sysTickUptime.wrapping_add(1); // adjust time to point just past the failure time to
    failsafeOnValidDataFailed(); // cause a lost link
    // armed from here (disarmed state has cleared throttleLowPeriod).
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_TRUE(isArmingDisabled());
    // given
    failsafeOnValidDataFailed(); // set last invalid sample at current time
    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived(); // cause a recovered link
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_TRUE(isArmingDisabled());
    // given
    // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(0 as libc::c_int,
              failsafeIsActive()); // disarm not called repeatedly.
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeSwitchModeKill: libc::c_int)
 -> libc::c_int {
    // given
    resetCallCounters();
    failsafeStartMonitoring();
    // and
    // throttle HIGH to go for a failsafe landing procedure
    activateBoxFailsafe(); // restart time from 0
    sysTickUptime =
        0 as libc::c_int as uint32_t; // set last valid sample at current time
    failsafeOnValidDataReceived();
    // adjust time to point just past the failure time to
    failsafeOnValidDataFailed(); // cause a lost link
    // when
    failsafeUpdateState(); // kill switch handling should come first
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    // given
    failsafeOnValidDataFailed(); // set last invalid sample at current time
    // adjust time to point just past the recovery time to
    failsafeOnValidDataReceived(); // cause a recovered link
    deactivateBoxFailsafe();
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    // given
    // adjust time to point just past the required additional recovery time
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(0 as libc::c_int,
              failsafeIsActive()); // disarm not called repeatedly.
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeSwitchModeStage2Drop:
                                    libc::c_int) -> libc::c_int {
    // given
    resetCallCounters();
    // and
    // throttle HIGH to go for a failsafe landing procedure
    sysTickUptime = 0 as libc::c_int as uint32_t; // restart time from 0
    activateBoxFailsafe(); // box failsafe causes data to be invalid
    failsafeOnValidDataFailed();
    // when
    failsafeUpdateState(); // should activate stage2 immediately
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    // given
    // adjust time to point just past the required additional recovery time
    deactivateBoxFailsafe(); // inactive box failsafe gives valid data
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(0 as libc::c_int,
              failsafeIsActive()); // disarm not called repeatedly.
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_7(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeSwitchModeStage2Land:
                                    libc::c_int) -> libc::c_int {
    // given
    resetCallCounters();
    // and
    // throttle HIGH to go for a failsafe landing procedure
    sysTickUptime = 0 as libc::c_int as uint32_t; // restart time from 0
    activateBoxFailsafe(); // box failsafe causes data to be invalid
    failsafeOnValidDataFailed();
    // when
    failsafeUpdateState(); // should activate stage2 immediately
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    // given
    failsafeOnValidDataFailed(); // set last invalid sample at current time
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsActive());
    EXPECT_TRUE(isArmingDisabled());
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    // given
    // adjust time to point just past the required additional recovery time
    // and
    deactivateBoxFailsafe(); // inactive box failsafe gives valid data
    failsafeOnValidDataReceived();
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(0 as libc::c_int,
              failsafeIsActive()); // disarm not called repeatedly.
    EXPECT_EQ(1 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
/* ***************************************************************************************/
//
// Additional non-stepwise tests
//
/* ***************************************************************************************/
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_8(mut FlightFailsafeTest: libc::c_int,
                                mut TestFailsafeNotActivatedWhenDisarmedAndRXLossIsDetected:
                                    libc::c_int) -> libc::c_int {
    // given
    resetCallCounters();
    configureFailsafe();
    // and
    failsafeInit();
    // and
    // when
    failsafeStartMonitoring();
    // and
    sysTickUptime = 0 as libc::c_int as uint32_t; // restart time from 0
    failsafeOnValidDataReceived(); // set last valid sample at current time
    // when
    // then
    // given
    sysTickUptime =
        sysTickUptime.wrapping_add(1); // adjust time to point just past the failure time to
    failsafeOnValidDataFailed(); // cause a lost link
    // when
    failsafeUpdateState();
    // then
    EXPECT_EQ(1 as libc::c_int, failsafeIsMonitoring());
    EXPECT_EQ(0 as libc::c_int, failsafeIsActive());
    EXPECT_EQ(0 as libc::c_int,
              callCounts[COUNTER_MW_DISARM as libc::c_int as usize]);
    EXPECT_TRUE(isArmingDisabled());
    // given
    // enough valid data is received
    let mut sysTickTarget: uint32_t = 0;
    while sysTickUptime < sysTickTarget {
        failsafeOnValidDataReceived();
        failsafeUpdateState();
        EXPECT_TRUE(isArmingDisabled());
        sysTickUptime = sysTickUptime.wrapping_add(1)
    }
    // and
    sysTickUptime =
        sysTickUptime.wrapping_add(1); // adjust time to point just past the failure time to
    failsafeOnValidDataReceived(); // cause link recovery
    // then
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
// Return system uptime in milliseconds (rollover in 49 days)
// STUBS
