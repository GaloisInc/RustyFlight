#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn time(__timer: *mut time_t) -> time_t;
}
pub type __time_t = libc::c_long;
pub type time_t = __time_t;
#[no_mangle]
pub unsafe extern "C" fn TEST(mut AlignSensorTest: libc::c_int,
                              mut ClockwiseZeroDegrees: libc::c_int)
 -> libc::c_int {
    srand(time(0 as *mut time_t));
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut AlignSensorTest: libc::c_int,
                                mut ClockwiseOneEightyDegrees: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut AlignSensorTest: libc::c_int,
                                mut ClockwiseTwoSeventyDegrees: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut AlignSensorTest: libc::c_int,
                                mut ClockwiseZeroDegreesFlip: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut AlignSensorTest: libc::c_int,
                                mut ClockwiseNinetyDegreesFlip: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut AlignSensorTest: libc::c_int,
                                mut ClockwiseOneEightyDegreesFlip:
                                    libc::c_int) -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut AlignSensorTest: libc::c_int,
                                mut ClockwiseTwoSeventyDegreesFlip:
                                    libc::c_int) -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
