#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
/*
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
pub static mut gyroDev: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn TEST(mut BlackboxTest: libc::c_int,
                              mut TestInitIntervals: libc::c_int)
 -> libc::c_int {
    // 250Hz PIDloop
    blackboxInit();
    // 500Hz PIDloop
    blackboxInit();
    // 1kHz PIDloop
    blackboxInit();
    // 2kHz PIDloop
    blackboxInit();
    // 4kHz PIDloop
    blackboxInit();
    // 8kHz PIDloop
    blackboxInit();
    // 16kHz PIDloop
    // rounded from 62.5
    blackboxInit();
    // note rounding
    // 32kHz PIDloop
    // rounded from 31.25
    blackboxInit();
    panic!("Reached end of non-void function without returning");
    // note rounding
}
// 500Hz PIDloop
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut BlackboxTest: libc::c_int,
                                mut Test_1kHz: libc::c_int) -> libc::c_int {
    // 1kHz PIDloop
    blackboxInit();
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 31 as libc::c_int { blackboxAdvanceIterationTimers(); ii += 1 }
    blackboxAdvanceIterationTimers();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut BlackboxTest: libc::c_int,
                                mut Test_2kHz: libc::c_int) -> libc::c_int {
    // 2kHz PIDloop
    blackboxInit();
    blackboxAdvanceIterationTimers();
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 31 as libc::c_int {
        blackboxAdvanceIterationTimers();
        blackboxAdvanceIterationTimers();
        ii += 1
    }
    blackboxAdvanceIterationTimers();
    blackboxAdvanceIterationTimers();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut BlackboxTest: libc::c_int,
                                mut Test_8kHz: libc::c_int) -> libc::c_int {
    // 8kHz PIDloop
    blackboxInit();
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 255 as libc::c_int {
        blackboxAdvanceIterationTimers();
        ii += 1
    }
    blackboxAdvanceIterationTimers();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut BlackboxTest: libc::c_int,
                                mut Test_zero_p_ratio: libc::c_int)
 -> libc::c_int {
    // 1kHz PIDloop
    blackboxInit();
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 31 as libc::c_int { blackboxAdvanceIterationTimers(); ii += 1 }
    blackboxAdvanceIterationTimers();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut BlackboxTest: libc::c_int,
                                mut Test_CalculatePDenom: libc::c_int)
 -> libc::c_int {
    // note I-frame is logged every 32ms regardless of PIDloop rate
    // so p_ratio is 32 when blackbox logging rate is 1kHz
    // 1kHz PIDloop
    blackboxInit(); // 1kHz logging
    EXPECT_EQ(32 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 1 as libc::c_int));
    EXPECT_EQ(16 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 2 as libc::c_int));
    EXPECT_EQ(8 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 4 as libc::c_int));
    // 2kHz PIDloop
    blackboxInit(); // 1kHz logging
    EXPECT_EQ(64 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 1 as libc::c_int));
    EXPECT_EQ(32 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 2 as libc::c_int));
    EXPECT_EQ(16 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 4 as libc::c_int));
    // 4kHz PIDloop
    blackboxInit(); // 1kHz logging
    EXPECT_EQ(128 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 1 as libc::c_int));
    EXPECT_EQ(64 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 2 as libc::c_int));
    EXPECT_EQ(32 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 4 as libc::c_int));
    EXPECT_EQ(16 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 8 as libc::c_int));
    // 8kHz PIDloop
    blackboxInit(); // 1kHz logging
    EXPECT_EQ(256 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 1 as libc::c_int));
    EXPECT_EQ(128 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 2 as libc::c_int));
    EXPECT_EQ(64 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 4 as libc::c_int));
    EXPECT_EQ(32 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 8 as libc::c_int));
    EXPECT_EQ(16 as libc::c_int,
              blackboxCalculatePDenom(1 as libc::c_int, 16 as libc::c_int));
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut BlackboxTest: libc::c_int,
                                mut Test_CalculateRates: libc::c_int)
 -> libc::c_int {
    // 1kHz PIDloop
    blackboxInit();
    EXPECT_EQ(1 as libc::c_int, blackboxGetRateDenom());
    blackboxInit();
    EXPECT_EQ(2 as libc::c_int, blackboxGetRateDenom());
    blackboxInit();
    EXPECT_EQ(4 as libc::c_int, blackboxGetRateDenom());
    // 8kHz PIDloop
    // 1kHz logging
    blackboxInit();
    EXPECT_EQ(8 as libc::c_int, blackboxGetRateDenom());
    // 1.5kHz logging
    blackboxInit();
    EXPECT_EQ(5 as libc::c_int, blackboxGetRateDenom());
    // 2kHz logging
    blackboxInit();
    EXPECT_EQ(4 as libc::c_int, blackboxGetRateDenom());
    // 4kHz logging
    blackboxInit();
    EXPECT_EQ(2 as libc::c_int, blackboxGetRateDenom());
    // 8kHz logging
    blackboxInit();
    EXPECT_EQ(1 as libc::c_int, blackboxGetRateDenom());
    // 0.126 PIDloop
    // 1kHz logging
    blackboxInit();
    EXPECT_EQ(7 as libc::c_int, blackboxGetRateDenom());
    panic!("Reached end of non-void function without returning");
}
// see baudRate_e
// STUBS
