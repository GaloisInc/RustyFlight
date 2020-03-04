#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(const_raw_ptr_to_usize_cast, register_tool)]
extern "C" {
    #[no_mangle]
    fn fabs(_: libc::c_double) -> libc::c_double;
}
pub type int16_t = libc::c_short;
pub type uint8_t = libc::c_uchar;
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
pub static mut simulateMixerSaturated: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut simulatedSetpointRate: [libc::c_float; 3] =
    [0 as libc::c_int as libc::c_float, 0 as libc::c_int as libc::c_float,
     0 as libc::c_int as libc::c_float];
#[no_mangle]
pub static mut simulatedRcDeflection: [libc::c_float; 3] =
    [0 as libc::c_int as libc::c_float, 0 as libc::c_int as libc::c_float,
     0 as libc::c_int as libc::c_float];
#[no_mangle]
pub static mut simulatedThrottlePIDAttenuation: libc::c_float = 1.0f32;
#[no_mangle]
pub static mut simulatedMotorMixRange: libc::c_float = 0.0f32;
#[no_mangle]
pub static mut debug: int16_t = 0;
#[no_mangle]
pub static mut debugMode: uint8_t = 0;
#[no_mangle]
pub static mut pidProfile: *mut libc::c_int =
    0 as *const libc::c_int as *mut libc::c_int;
#[no_mangle]
pub static mut rollAndPitchTrims: libc::c_int = 0;
#[no_mangle]
pub static mut loopIter: libc::c_int = 0 as libc::c_int;
// Always use same defaults for testing in future releases even when defaults change
#[no_mangle]
pub unsafe extern "C" fn setDefaultTestSettings() { pgResetAll(); }
#[no_mangle]
pub unsafe extern "C" fn currentTestTime() -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn resetTest() {
    loopIter = 0 as libc::c_int;
    simulateMixerSaturated = 0 as libc::c_int != 0;
    simulatedThrottlePIDAttenuation = 1.0f32;
    simulatedMotorMixRange = 0.0f32;
    setDefaultTestSettings();
    // Run pidloop for a while after reset
    let mut loop_0: libc::c_int = 0 as libc::c_int;
    while loop_0 < 20 as libc::c_int { loop_0 += 1 };
}
#[no_mangle]
pub unsafe extern "C" fn setStickPosition(mut axis: libc::c_int,
                                          mut stickRatio: libc::c_float) {
    simulatedSetpointRate[axis as usize] = 1998.0f32 * stickRatio;
    simulatedRcDeflection[axis as usize] = stickRatio;
}
// All calculations will have 10% tolerance
#[no_mangle]
pub unsafe extern "C" fn calculateTolerance(mut input: libc::c_float)
 -> libc::c_float {
    return fabs((input * 0.1f32) as libc::c_double) as libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn TEST(mut pidControllerTest: libc::c_int,
                              mut testInitialisation: libc::c_int)
 -> libc::c_int {
    resetTest();
    panic!("Reached end of non-void function without returning");
    // In initial state PIDsums should be 0
}
// Run few loops to make sure there is no error building up when stabilisation disabled
// PID controller should not do anything, while stabilisation disabled
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut pidControllerTest: libc::c_int,
                                mut testPidLoop: libc::c_int) -> libc::c_int {
    // Make sure to start with fresh values
    resetTest();
    // Loop 1 - Expecting zero since there is no error
    // Add some rotation on ROLL to generate error
    // Loop 2 - Expect PID loop reaction to ROLL error
    // Add some rotation on PITCH to generate error
    // Loop 3 - Expect PID loop reaction to PITCH error, ROLL is still in error
    // Add some rotation on YAW to generate error
    // Loop 4 - Expect PID loop reaction to PITCH error, ROLL and PITCH are still in error
    // Match the stick to gyro to stop error
    let mut loop_0: libc::c_int = 0 as libc::c_int;
    while loop_0 < 5 as libc::c_int { loop_0 += 1 }
    panic!("Reached end of non-void function without returning");
    // Iterm is stalled as it is not accumulating anymore
    // Now disable Stabilisation
    // Should all be zero again
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut pidControllerTest: libc::c_int,
                                mut testPidLevel: libc::c_int)
 -> libc::c_int {
    // Make sure to start with fresh values
    resetTest();
    panic!("Reached end of non-void function without returning");
    // Test Angle mode response
    // Loop 1
    // Test attitude response
    // Loop 2
    // Disable ANGLE_MODE on full stick inputs
    // Expect full rate output
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut pidControllerTest: libc::c_int,
                                mut testPidHorizon: libc::c_int)
 -> libc::c_int {
    resetTest();
    panic!("Reached end of non-void function without returning");
    // Loop 1
    // Test full stick response
    // Expect full rate output on full stick
    // Test full stick response
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut pidControllerTest: libc::c_int,
                                mut testMixerSaturation: libc::c_int)
 -> libc::c_int {
    resetTest();
    // Test full stick response
    simulateMixerSaturated = 1 as libc::c_int != 0;
    panic!("Reached end of non-void function without returning");
    // Expect no iterm accumulation
}
// TODO - Add more scenarios
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut pidControllerTest: libc::c_int,
                                mut testCrashRecoveryMode: libc::c_int)
 -> libc::c_int {
    resetTest();
    EXPECT_FALSE(crashRecoveryModeActive());
    let mut loopsToCrashTime: libc::c_int = 0;
    // generate crash detection for roll axis
    simulatedMotorMixRange = 1.2f32;
    let mut loop_0: libc::c_int = 0 as libc::c_int;
    while loop_0 <= loopsToCrashTime { loop_0 += 1 }
    EXPECT_TRUE(crashRecoveryModeActive());
    panic!("Reached end of non-void function without returning");
    // Add additional verifications
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut pidControllerTest: libc::c_int,
                                mut pidSetpointTransition: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // TODO
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut pidControllerTest: libc::c_int,
                                mut testDtermFiltering: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // TODO
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_7(mut pidControllerTest: libc::c_int,
                                mut testItermRotationHandling: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // TODO
}
