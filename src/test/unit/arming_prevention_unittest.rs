#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
pub type uint32_t = libc::c_uint;
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
pub static mut simulationFeatureFlags: uint32_t =
    0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut simulationTime: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub static mut gyroCalibDone: libc::c_int = 0;
#[no_mangle]
pub static mut simulationHaveRx: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn TEST(mut ArmingPreventionTest: libc::c_int,
                              mut CalibrationPowerOnGraceAngleThrottleArmSwitch:
                                  libc::c_int) -> libc::c_int {
    // given
    simulationTime = 0 as libc::c_int as uint32_t;
    // and
    // and
    // and
    // default channel positions
    // and
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    // given
    // gyro calibration is done
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    // given
    // quad is level
    // when
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    // given
    // when
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    // given
    // arming grace time has elapsed
    // when
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    // given
    // when
    // arm guard time elapses
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
// given
// and
// and
// and
// when
// expect
// given
    // arm channel takes a safe default value from the RX after power on
// and
    // a short time passes while calibration is in progress
// and
    // during calibration RF link is established and ARM switch is on
// when
// expect
// given
    // calibration is done
// when
// expect
// given
    // arm switch is switched off by user
// when
// expect
    // arming enabled as arm switch has been off for sufficient time
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut ArmingPreventionTest: libc::c_int,
                                mut Prearm: libc::c_int) -> libc::c_int {
    // given
    simulationTime = 0 as libc::c_int as uint32_t;
    // and
    // and
    // given
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // prearm is enabled
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    // arming enabled as arm switch has been off for sufficient time
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    EXPECT_FALSE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut ArmingPreventionTest: libc::c_int,
                                mut RadioTurnedOnAtAnyTimeArmed: libc::c_int)
 -> libc::c_int {
    // given
    simulationTime = 30e6f64 as uint32_t; // 30 seconds after boot
    // and
    // and
    // and
    // and
    // RX has no link to radio
    // and
    // arm channel has a safe default value
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    // RF link is established and arm switch is turned on on radio
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // arm switch turned off by user
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut ArmingPreventionTest: libc::c_int,
                                mut In3DModeAllowArmingWhenEnteringThrottleDeadband:
                                    libc::c_int) -> libc::c_int {
    // given
    // Using 3D mode
    simulationTime = 30e6f64 as uint32_t; // 30 seconds after boot
    // and
    // and
    // and
    // and
    // arm channel has a safe default value
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // attempt to arm
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // throttle moved to centre
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut ArmingPreventionTest: libc::c_int,
                                mut When3DModeDisabledThenNormalThrottleArmingConditionApplies:
                                    libc::c_int) -> libc::c_int {
    // given
    // Using 3D mode
    simulationTime = 30e6f64 as uint32_t; // 30 seconds after boot
    // and
    // and
    // and
    // safe throttle value for 3D mode
    // and
    // arm channel has a safe default value
    // and
    // disable 3D mode is off (i.e. 3D mode is on)
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    // disable 3D mode
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // attempt to arm
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // throttle moved low
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // arm switch turned off
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut ArmingPreventionTest: libc::c_int,
                                mut WhenUsingSwitched3DModeThenNormalThrottleArmingConditionApplies:
                                    libc::c_int) -> libc::c_int {
    // given
    // Using 3D mode
    simulationTime = 30e6f64 as uint32_t; // 30 seconds after boot
    // and
    // and
    // and
    // and
    // arm channel has a safe default value
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    // raise throttle to unsafe position
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    // ok to arm in 3D mode
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // attempt to arm
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // throttle moved low
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_TRUE(isArmingDisabled());
    // given
    // arm switch turned off
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isUsingSticksForArming());
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut ArmingPreventionTest: libc::c_int,
                                mut ParalyzeOnAtBoot: libc::c_int)
 -> libc::c_int {
    // given
    simulationFeatureFlags = 0 as libc::c_int as uint32_t;
    simulationTime = 0 as libc::c_int as uint32_t;
    // and
    // and
    // given
    // Paralyze on at boot
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // when
    updateActivatedModes();
    panic!("Reached end of non-void function without returning");
    // expect
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut ArmingPreventionTest: libc::c_int,
                                mut Paralyze: libc::c_int) -> libc::c_int {
    // given
    simulationFeatureFlags = 0 as libc::c_int as uint32_t;
    simulationTime = 0 as libc::c_int as uint32_t;
    // and
    // and
    // given
    // Start out with paralyze enabled
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    // arm
    // when
    tryArm();
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    // disarm
    // when
    disarm();
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    simulationTime = 10e6f64 as uint32_t; // 10 seconds after boot
    // when
    updateActivatedModes();
    // expect
    EXPECT_FALSE(isArmingDisabled());
    EXPECT_EQ(0 as libc::c_int, getArmingDisableFlags());
    // given
    // disable paralyze once after the startup timer
    // when
    updateActivatedModes();
    // enable paralyze again
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    // given
    // enable beeper
    // when
    updateActivatedModes();
    // expect
    // given
    // try exiting paralyze mode and ensure arming and pit mode are still disabled
    // when
    updateActivatedModes();
    updateArmingStatus();
    // expect
    EXPECT_TRUE(isArmingDisabled());
    panic!("Reached end of non-void function without returning");
}
// STUBS
