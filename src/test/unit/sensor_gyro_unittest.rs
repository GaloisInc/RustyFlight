#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
#[no_mangle]
pub unsafe extern "C" fn TEST(mut SensorGyro: libc::c_int,
                              mut Detect: libc::c_int) -> libc::c_int {
    let detected: libc::c_int = 0;
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut SensorGyro: libc::c_int,
                                mut Read: libc::c_int) -> libc::c_int {
    pgResetAll();
    gyroInit();
    let read: bool = false;
    EXPECT_EQ(1 as libc::c_int, read as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut SensorGyro: libc::c_int,
                                mut Calibrate: libc::c_int) -> libc::c_int {
    pgResetAll();
    gyroInit();
    let read: bool = false;
    EXPECT_EQ(1 as libc::c_int, read as libc::c_int);
    static mut gyroMovementCalibrationThreshold: libc::c_int =
        32 as libc::c_int;
    gyroStartCalibration(0 as libc::c_int);
    EXPECT_EQ(0 as libc::c_int, isGyroCalibrationComplete());
    while isGyroCalibrationComplete() == 0 { }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut SensorGyro: libc::c_int,
                                mut Update: libc::c_int) -> libc::c_int {
    pgResetAll();
    // turn off filters
    gyroInit();
    gyroStartCalibration(0 as libc::c_int);
    EXPECT_EQ(0 as libc::c_int, isGyroCalibrationComplete());
    while isGyroCalibrationComplete() == 0 { }
    EXPECT_EQ(1 as libc::c_int, isGyroCalibrationComplete());
    panic!("Reached end of non-void function without returning");
    // expect zero values since gyro is calibrated
    // gyroADCf values are scaled
}
// STUBS
