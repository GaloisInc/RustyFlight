#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
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
/* #define DEBUG_OSD */
#[no_mangle]
pub unsafe extern "C" fn setDefualtSimulationState() { }
/*
 * Auxiliary function. Test is there're stats that must be shown
 */
#[no_mangle]
pub unsafe extern "C" fn isSomeStatEnabled() -> bool {
    panic!("Reached end of non-void function without returning");
}
/*
 * Performs a test of the OSD actions on disarming.
 * (reused throughout the test suite)
 */
#[no_mangle]
pub unsafe extern "C" fn doTestDisarm() {
    // given
    // craft is disarmed after having been armed
    // when
    // sufficient OSD updates have been called
    // then
    // post flight statistics displayed
    if isSomeStatEnabled() {
        displayPortTestBufferSubstring(2 as libc::c_int, 2 as libc::c_int,
                                       b"  --- STATS ---\x00" as *const u8 as
                                           *const libc::c_char);
    };
}
/*
 * Tests initialisation of the OSD and the power on splash screen.
 */
#[no_mangle]
pub unsafe extern "C" fn TEST(mut OsdTest: libc::c_int,
                              mut TestInit: libc::c_int) -> libc::c_int {
    // given
    // display port is initialised
    displayPortTestInit();
    // and
    // default state values are set
    setDefualtSimulationState();
    // and
    // this battery configuration (used for battery voltage elements)
    // when
    // OSD is initialised
    // then
    // display buffer should contain splash screen
    displayPortTestBufferSubstring(7 as libc::c_int, 8 as libc::c_int,
                                   b"MENU:THR MID\x00" as *const u8 as
                                       *const libc::c_char);
    displayPortTestBufferSubstring(11 as libc::c_int, 9 as libc::c_int,
                                   b"+ YAW LEFT\x00" as *const u8 as
                                       *const libc::c_char);
    displayPortTestBufferSubstring(11 as libc::c_int, 10 as libc::c_int,
                                   b"+ PITCH UP\x00" as *const u8 as
                                       *const libc::c_char);
    // when
    // splash screen timeout has elapsed
    // then
    // display buffer should be empty
    displayPortTestBufferIsEmpty();
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests visibility of the ARMED notification after arming.
 */
/*
 * Tests display and timeout of the post flight statistics screen after disarming.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut OsdTest: libc::c_int,
                                mut TestDisarm: libc::c_int) -> libc::c_int {
    doTestDisarm();
    // given
    // post flight stats times out (60 seconds)
    // when
    // sufficient OSD updates have been called
    // then
    // post flight stats screen disappears
    displayPortTestBufferIsEmpty();
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests disarming and immediately rearming clears post flight stats and shows ARMED notification.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut OsdTest: libc::c_int,
                                mut TestDisarmWithImmediateRearm: libc::c_int)
 -> libc::c_int {
    doTestDisarm();
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests dismissing the statistics screen with pitch stick after disarming.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut OsdTest: libc::c_int,
                                mut TestDisarmWithDismissStats: libc::c_int)
 -> libc::c_int {
    // Craft is alread armed after previous test
    doTestDisarm();
    // given
    // sticks have been moved
    // when
    // sufficient OSD updates have been called
    // then
    // post flight stats screen disappears
    displayPortTestBufferIsEmpty();
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests the calculation of statistics with imperial unit output.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut OsdTest: libc::c_int,
                                mut TestStatsImperial: libc::c_int)
 -> libc::c_int {
    // given
    // this set of enabled post flight statistics
    // and
    // using imperial unit system
    // and
    // this timer 1 configuration
    // and
    // this timer 2 configuration
    // and
    // a GPS fix is present
    // and
    // this RTC time
    // when
    // the craft is armed
    // and
    // these conditions occur during flight
    // and
    // the craft is disarmed
    doTestDisarm();
    // then
    // statistics screen should display the following
    let mut row: libc::c_int = 3 as libc::c_int;
    let fresh0 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh0,
                                   b"2017-11-19 10:12:\x00" as *const u8 as
                                       *const libc::c_char);
    let fresh1 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh1,
                                   b"TOTAL ARM         : 00:05.00\x00" as
                                       *const u8 as *const libc::c_char);
    let fresh2 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh2,
                                   b"LAST ARM          : 00:03\x00" as
                                       *const u8 as *const libc::c_char);
    let fresh3 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh3,
                                   b"MAX SPEED         : 17\x00" as *const u8
                                       as *const libc::c_char);
    let fresh4 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh4,
                                   b"MIN RSSI          : 25%%\x00" as
                                       *const u8 as *const libc::c_char);
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests the calculation of statistics with metric unit output.
 * (essentially an abridged version of the previous test
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut OsdTest: libc::c_int,
                                mut TestStatsMetric: libc::c_int)
 -> libc::c_int {
    // given
    // using metric unit system
    // and
    // default state values are set
    setDefualtSimulationState();
    // when
    // the craft is armed
    // and
    // these conditions occur during flight (simplified to less assignments than previous test)
    // and
    // the craft is disarmed
    doTestDisarm();
    // then
    // statistics screen should display the following
    let mut row: libc::c_int = 3 as libc::c_int;
    let fresh5 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh5,
                                   b"2017-11-19 10:12:\x00" as *const u8 as
                                       *const libc::c_char);
    let fresh6 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh6,
                                   b"TOTAL ARM         : 00:07.50\x00" as
                                       *const u8 as *const libc::c_char);
    let fresh7 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh7,
                                   b"LAST ARM          : 00:02\x00" as
                                       *const u8 as *const libc::c_char);
    let fresh8 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh8,
                                   b"MAX SPEED         : 28\x00" as *const u8
                                       as *const libc::c_char);
    let fresh9 = row;
    row = row + 1;
    displayPortTestBufferSubstring(2 as libc::c_int, fresh9,
                                   b"MIN RSSI          : 25%%\x00" as
                                       *const u8 as *const libc::c_char);
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests activation of alarms and element flashing.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut OsdTest: libc::c_int,
                                mut TestAlarms: libc::c_int) -> libc::c_int {
    // given
    // default state is set
    setDefualtSimulationState();
    // and
    // the following OSD elements are visible
    // and
    // this set of alarm values
    // meters
    // and
    // this timer 1 configuration
    // and
    // this timer 2 configuration
    // and
    // using the metric unit system
    // when
    // the craft is armed
    // then
    // no elements should flash as all values are out of alarm range
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 30 as libc::c_int {
        // Check for visibility every 100ms, elements should always be visible
        // only test the minute part of the timer
        // only test the minute part of the timer
        i += 1
    }
    // when
    // all values are out of range
    // then
    // elements showing values in alarm range should flash
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < 15 as libc::c_int {
        // Blinking should happen at 5Hz
        if !(i_0 % 2 as libc::c_int == 0 as libc::c_int) {
            displayPortTestBufferIsEmpty();
        }
        i_0 += 1
    }
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests the RSSI OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut OsdTest: libc::c_int,
                                mut TestElementRssi: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // given
    // when
    // then
    // when
    // then
    // when
    // then
}
/*
 * Tests the instantaneous battery current OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_7(mut OsdTest: libc::c_int,
                                mut TestElementAmperage: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // given
    // when
    // then
    // when
    // then
    // when
    // then
}
/*
 * Tests the battery capacity drawn OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_8(mut OsdTest: libc::c_int,
                                mut TestElementMahDrawn: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // given
    // when
    // then
    // when
    // then
    // when
    // then
    // when
    // then
    // when
    // then
}
/*
 * Tests the instantaneous electrical power OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_9(mut OsdTest: libc::c_int,
                                mut TestElementPower: libc::c_int)
 -> libc::c_int {
    // given
    // and
    // 10V
    // and
    // 0A
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 10 as libc::c_int,
                                   b"   0W\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // 0.1A
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 10 as libc::c_int,
                                   b"   1W\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // 1.2A
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 10 as libc::c_int,
                                   b"  12W\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // 12.3A
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 10 as libc::c_int,
                                   b" 123W\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // 123.4A
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 10 as libc::c_int,
                                   b"1234W\x00" as *const u8 as
                                       *const libc::c_char);
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests the altitude OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_10(mut OsdTest: libc::c_int,
                                 mut TestElementAltitude: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    // given
    // and
    // when
    // then
    // when
    // then
    // when
    // then
    // when
    // then
    // when
    // then
}
/*
 * Tests the core temperature OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_11(mut OsdTest: libc::c_int,
                                 mut TestElementCoreTemperature: libc::c_int)
 -> libc::c_int {
    // given
    // and
    // and
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 8 as libc::c_int,
                                   b"  0C\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 8 as libc::c_int,
                                   b" 33C\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // when
    // then
    displayPortTestBufferSubstring(1 as libc::c_int, 8 as libc::c_int,
                                   b" 91F\x00" as *const u8 as
                                       *const libc::c_char);
    panic!("Reached end of non-void function without returning");
}
/*
 * Tests the battery notifications shown on the warnings OSD element.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_12(mut OsdTest: libc::c_int,
                                 mut TestElementWarningsBattery: libc::c_int)
 -> libc::c_int {
    // given
    // disable all warnings
    // and
    // and
    // 4S battery
    // and
    // full battery
    // when
    // then
    displayPortTestBufferSubstring(9 as libc::c_int, 10 as libc::c_int,
                                   b"           \x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // low battery
    // when
    // then
    displayPortTestBufferSubstring(9 as libc::c_int, 10 as libc::c_int,
                                   b"LOW BATTERY \x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // crtical battery
    // when
    // then
    displayPortTestBufferSubstring(9 as libc::c_int, 10 as libc::c_int,
                                   b" LAND NOW   \x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // used battery
    // when
    // then
    displayPortTestBufferSubstring(9 as libc::c_int, 10 as libc::c_int,
                                   b"BATT < FULL\x00" as *const u8 as
                                       *const libc::c_char);
    // given
    // full battery
    // when
    // then
    displayPortTestBufferSubstring(9 as libc::c_int, 10 as libc::c_int,
                                   b"             \x00" as *const u8 as
                                       *const libc::c_char);
    panic!("Reached end of non-void function without returning");
    // TODO
}
/*
 * Tests the time string formatting function with a series of precision settings and time values.
 */
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_13(mut OsdTest: libc::c_int,
                                 mut TestFormatTimeString: libc::c_int)
 -> libc::c_int {
    let mut buff: libc::c_char = 0;
    panic!("Reached end of non-void function without returning");
    /* Seconds precision, 0 us */
    /* Seconds precision, 0.9 seconds */
    /* Seconds precision, 10 seconds */
    /* Seconds precision, 1 minute */
    /* Seconds precision, 1 minute 59 seconds */
    /* Hundredths precision, 0 us */
    /* Hundredths precision, 10 milliseconds (one 100th of a second) */
    /* Hundredths precision, 0.9 seconds */
    /* Hundredths precision, 10 seconds */
    /* Hundredths precision, 1 minute */
    /* Hundredths precision, 1 minute 59 seconds */
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_14(mut OsdTest: libc::c_int,
                                 mut TestConvertTemperatureUnits: libc::c_int)
 -> libc::c_int {
    /* In Celsius */
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(330 as libc::c_int),
              330 as libc::c_int);
    /* In Fahrenheit */
    EXPECT_EQ(osdConvertTemperatureToSelectedUnit(330 as libc::c_int),
              914 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
// STUBS
