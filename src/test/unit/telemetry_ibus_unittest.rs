#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
pub type int16_t = libc::c_short;
pub type uint8_t = libc::c_uchar;
pub type uint16_t = libc::c_ushort;
pub type uint32_t = libc::c_uint;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortStub_s {
    pub buffer: [uint8_t; 256],
    pub pos: libc::c_int,
    pub end: libc::c_int,
}
pub type serialPortStub_t = serialPortStub_s;
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
static mut gyroTemperature: int16_t = 0;
#[no_mangle]
pub unsafe extern "C" fn gyroGetTemperature() -> int16_t {
    return gyroTemperature;
}
static mut vbat: uint16_t = 100 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn getVbat() -> uint16_t { return vbat; }
static mut testBatteryVoltage: uint16_t = 100 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn getBatteryVoltage() -> uint16_t {
    return testBatteryVoltage;
}
#[no_mangle]
pub unsafe extern "C" fn getBatteryCellCount() -> uint8_t {
    panic!("Reached end of non-void function without returning");
}
static mut serialWriteStub: serialPortStub_t =
    serialPortStub_t{buffer: [0; 256], pos: 0, end: 0,};
static mut serialReadStub: serialPortStub_t =
    serialPortStub_t{buffer: [0; 256], pos: 0, end: 0,};
#[no_mangle]
pub static mut serialTestInstance: libc::c_int = 0;
#[no_mangle]
pub static mut serialTestInstanceConfig: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn rescheduleTask(mut taskId: libc::c_int,
                                        mut newPeriodMicros: uint32_t) {
    EXPECT_EQ(1000 as libc::c_int, newPeriodMicros);
}
#[no_mangle]
pub unsafe extern "C" fn findSerialPortConfig(mut function: libc::c_int)
 -> *mut libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn determinePortSharing(mut portConfig:
                                                  *const libc::c_int,
                                              mut function: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn telemetryDetermineEnabledState(mut portSharing:
                                                            libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn isSerialPortShared(mut portConfig:
                                                *const libc::c_int,
                                            mut functionMask: uint16_t,
                                            mut sharedWithFunction:
                                                libc::c_int) -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[export_name = "findSerialPortConfig"]
pub unsafe extern "C" fn findSerialPortConfig_0(mut mask: uint16_t)
 -> *mut libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn openSerialPort(mut identifier: libc::c_int,
                                        mut function: libc::c_int,
                                        mut callback: libc::c_int,
                                        mut callbackData: *mut libc::c_void,
                                        mut baudrate: uint32_t,
                                        mut mode: libc::c_int,
                                        mut options: libc::c_int)
 -> *mut libc::c_int {
    UNUSED(callbackData);
    EXPECT_EQ(115200 as libc::c_int, baudrate);
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn closeSerialPort(mut serialPort: *mut libc::c_int) { }
#[no_mangle]
pub unsafe extern "C" fn serialWrite(mut instance: *mut libc::c_int,
                                     mut ch: uint8_t) {
    EXPECT_LT(serialWriteStub.pos,
              ::std::mem::size_of::<[uint8_t; 256]>() as libc::c_ulong);
    let fresh0 = serialWriteStub.pos;
    serialWriteStub.pos = serialWriteStub.pos + 1;
    serialWriteStub.buffer[fresh0 as usize] = ch;
    let fresh1 = serialReadStub.end;
    serialReadStub.end = serialReadStub.end + 1;
    serialReadStub.buffer[fresh1 as usize] = ch;
    //characters echoes back on the shared wire
    //printf("w: %02d 0x%02x\n", serialWriteStub.pos, ch);
}
#[no_mangle]
pub unsafe extern "C" fn serialRxBytesWaiting(mut instance:
                                                  *const libc::c_int)
 -> uint32_t {
    EXPECT_GE(serialReadStub.end, serialReadStub.pos);
    let mut ret: libc::c_int = serialReadStub.end - serialReadStub.pos;
    if ret < 0 as libc::c_int { ret = 0 as libc::c_int }
    //printf("serialRxBytesWaiting: %d\n", ret);
    return ret as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn serialRead(mut instance: *mut libc::c_int)
 -> uint8_t {
    EXPECT_LT(serialReadStub.pos, serialReadStub.end);
    let fresh2 = serialReadStub.pos;
    serialReadStub.pos = serialReadStub.pos + 1;
    let ch: uint8_t = serialReadStub.buffer[fresh2 as usize];
    return ch;
}
#[no_mangle]
pub unsafe extern "C" fn serialTestResetBuffers() {
    memset(&mut serialReadStub as *mut serialPortStub_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<serialPortStub_t>() as libc::c_ulong);
    memset(&mut serialWriteStub as *mut serialPortStub_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<serialPortStub_t>() as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn setTestSensors() { }
#[no_mangle]
pub unsafe extern "C" fn serialTestResetPort() { serialTestResetBuffers(); }
#[no_mangle]
pub static mut IbusTelemteryInitUnitTest: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn TEST_F(mut IbusTelemteryInitUnitTest_0: libc::c_int,
                                mut Test_IbusInitNotEnabled: libc::c_int)
 -> libc::c_int {
    //given stuff in serial read
    serialReadStub.end += 1;
    //when initializing and polling ibus
    initIbusTelemetry();
    checkIbusTelemetryState();
    handleIbusTelemetry();
    //then nothing is read from serial port
    EXPECT_NE(serialReadStub.pos, serialReadStub.end);
    panic!("Reached end of non-void function without returning");
}
//given stuff in serial read
//when initializing and polling ibus
//then all is read from serial port
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_0(mut IbusTelemteryInitUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusInitSerialRxAndTelemetryEnabled:
                                      libc::c_int) -> libc::c_int {
    //given stuff in serial read
    serialReadStub.end += 1;
    //and serial rx enabled too
    //when initializing and polling ibus
    initIbusTelemetry();
    checkIbusTelemetryState();
    handleIbusTelemetry();
    //then all is read from serial port
    EXPECT_NE(serialReadStub.pos, serialReadStub.end);
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub static mut IbusTelemetryProtocolUnitTestBase: libc::c_int = 0;
#[no_mangle]
pub static mut public: libc::c_int = 0;
//when polling ibus
#[no_mangle]
pub static mut IbusTelemteryProtocolUnitTest: libc::c_int = 0;
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_1(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusNoRespondToDiscoveryCrcErr:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Hello sensor at address 1, are you there (with bad crc)?
    //then we do not respond
    checkResponseToCommand(b"\x04\x81\x00\x00\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           0 as *mut libc::c_void, 0 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_2(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToDiscovery:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Hello sensor at address 1, are you there?
    //then we respond with: Yes, i'm here, hello!
    checkResponseToCommand(b"\x04\x81z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x04\x81z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_3(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToSensorTypeQueryVbatt:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 1, what type are you?
    //then we respond with: I'm a voltage sensor
    checkResponseToCommand(b"\x04\x91j\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\x91\x03\x02c\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_4(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToSensorTypeQueryTemperature:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 1, what type are you?
    //then we respond with: I'm a thermometer
    checkResponseToCommand(b"\x04\x92i\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\x92\x01\x02d\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_5(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToSensorTypeQueryRpm:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 3, what type are you?
    //then we respond with: I'm a rpm sensor
    checkResponseToCommand(b"\x04\x93h\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\x93\x02\x02b\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_6(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToGetMeasurementVbattZero:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 1, please send your measurement
    //then we respond with: I'm reading 0 volts
    testBatteryVoltage = 0 as libc::c_int as uint16_t;
    checkResponseToCommand(b"\x04\xa1Z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa1\x00\x00X\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_7(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToGetMeasurementVbattCellVoltage:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 1, please send your measurement
    //then we respond with: I'm reading 0.1 volts
    testBatteryVoltage = 30 as libc::c_int as uint16_t;
    checkResponseToCommand(b"\x04\xa1Z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa1d\x00\xf4\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 1, please send your measurement
    //then we respond with: I'm reading 0.1 volts
    testBatteryVoltage = 10 as libc::c_int as uint16_t;
    checkResponseToCommand(b"\x04\xa1Z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa1d\x00\xf4\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_8(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToGetMeasurementVbattPackVoltage:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 1, please send your measurement
    //then we respond with: I'm reading 0.1 volts
    testBatteryVoltage = 10 as libc::c_int as uint16_t;
    checkResponseToCommand(b"\x04\xa1Z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa1d\x00\xf4\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 1, please send your measurement
    //then we respond with: I'm reading 0.1 volts
    testBatteryVoltage = 10 as libc::c_int as uint16_t;
    checkResponseToCommand(b"\x04\xa1Z\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa1d\x00\xf4\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_9(mut IbusTelemteryProtocolUnitTest_0:
                                      libc::c_int,
                                  mut Test_IbusRespondToGetMeasurementTemperature:
                                      libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 2, please send your measurement
    //then we respond
    gyroTemperature = 50 as libc::c_int as int16_t;
    checkResponseToCommand(b"\x04\xa2Y\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa2\x84\x03\xd0\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 2, please send your measurement
    //then we respond
    gyroTemperature = 59 as libc::c_int as int16_t; //test integer rounding
    checkResponseToCommand(b"\x04\xa2Y\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa2\xde\x03v\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 2, please send your measurement
    //then we respond
    gyroTemperature = 150 as libc::c_int as int16_t;
    checkResponseToCommand(b"\x04\xa2Y\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa2l\x07\xe4\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_10(mut IbusTelemteryProtocolUnitTest_0:
                                       libc::c_int,
                                   mut Test_IbusRespondToGetMeasurementRpm:
                                       libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 3, please send your measurement
    //then we respond with: I'm reading 0 rpm
    checkResponseToCommand(b"\x04\xa3X\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa3\x00\x00V\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 3, please send your measurement
    //then we respond with: I'm reading 100 rpm
    checkResponseToCommand(b"\x04\xa3X\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa3d\x00\xf2\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub static mut IbusTelemteryProtocolUnitTestDaisyChained: libc::c_int = 0;
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_11(mut IbusTelemteryProtocolUnitTestDaisyChained_0:
                                       libc::c_int,
                                   mut Test_IbusRespondToDiscoveryBaseAddressThree:
                                       libc::c_int) -> libc::c_int {
    //Given ibus commands: Hello sensor at address 3, 4, 5 are you there?
    //then we respond with: Yes, we're here, hello!
    checkResponseToCommand(b"\x04\x83x\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x04\x83x\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int);
    checkResponseToCommand(b"\x04\x84w\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x04\x84w\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int);
    checkResponseToCommand(b"\x04\x85v\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x04\x85v\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_12(mut IbusTelemteryProtocolUnitTestDaisyChained_0:
                                       libc::c_int,
                                   mut Test_IbusRespondToSensorTypeQueryWrongAddress:
                                       libc::c_int) -> libc::c_int {
    //Given ibus commands: Sensor at address 1, 2, 6, what type are you?
    //then we do not respond
    checkResponseToCommand(b"\x04\x91j\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x00" as *const u8 as *const libc::c_char,
                           0 as libc::c_int);
    checkResponseToCommand(b"\x04\x92i\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x00" as *const u8 as *const libc::c_char,
                           0 as libc::c_int);
    checkResponseToCommand(b"\x04\x96e\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x00" as *const u8 as *const libc::c_char,
                           0 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_13(mut IbusTelemteryProtocolUnitTestDaisyChained_0:
                                       libc::c_int,
                                   mut Test_IbusRespondToSensorTypeQueryVbattBaseThree:
                                       libc::c_int) -> libc::c_int {
    //Given ibus commands: Sensor at address 3, 4, 5, what type are you?
    //then we respond with: I'm a voltage sensor
    checkResponseToCommand(b"\x04\x93h\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\x93\x03\x02a\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //then we respond with: I'm a thermometer
    checkResponseToCommand(b"\x04\x94g\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\x94\x01\x02b\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //then we respond with: I'm a rpm sensor
    checkResponseToCommand(b"\x04\x95f\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\x95\x02\x02`\xff\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_14(mut IbusTelemteryProtocolUnitTestDaisyChained_0:
                                       libc::c_int,
                                   mut Test_IbusRespondToGetMeasurementsBaseThree:
                                       libc::c_int) -> libc::c_int {
    //Given ibus command: Sensor at address 3, please send your measurement
    //then we respond with: I'm reading 0.1 volts
    testBatteryVoltage = 10 as libc::c_int as uint16_t;
    checkResponseToCommand(b"\x04\xa3X\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa3d\x00\xf2\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 4, please send your measurement
    //then we respond
    gyroTemperature = 150 as libc::c_int as int16_t;
    checkResponseToCommand(b"\x04\xa4W\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa4l\x07\xe2\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    //Given ibus command: Sensor at address 5, please send your measurement
    //then we respond with: I'm reading 100 rpm
    checkResponseToCommand(b"\x04\xa5V\xff\x00" as *const u8 as
                               *const libc::c_char, 4 as libc::c_int,
                           b"\x06\xa5d\x00\xf0\xfe\x00" as *const u8 as
                               *const libc::c_char, 6 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
