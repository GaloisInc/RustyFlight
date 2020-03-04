#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(const_raw_ptr_to_usize_cast, register_tool)]
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memcmp(_: *const libc::c_void, _: *const libc::c_void,
              _: libc::c_ulong) -> libc::c_int;
}
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
#[no_mangle]
pub unsafe extern "C" fn telemetryCheckRxPortShared(mut portConfig:
                                                        *const libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    //TODO: implement
}
#[no_mangle]
pub static mut telemetrySharedPort: *mut libc::c_int =
    0 as *const libc::c_int as *mut libc::c_int;
static mut vbat: uint16_t = 100 as libc::c_int as uint16_t;
#[no_mangle]
pub unsafe extern "C" fn getVbat() -> uint16_t { return vbat; }
#[no_mangle]
pub static mut microseconds_stub_value: uint32_t =
    0 as libc::c_int as uint32_t;
#[no_mangle]
pub unsafe extern "C" fn micros() -> uint32_t {
    return microseconds_stub_value;
}
static mut serialWriteStub: serialPortStub_t =
    serialPortStub_t{buffer: [0; 256], pos: 0, end: 0,};
#[no_mangle]
pub unsafe extern "C" fn isSerialPortShared(mut portConfig:
                                                *const libc::c_int,
                                            mut functionMask: uint16_t,
                                            mut sharedWithFunction:
                                                libc::c_int) -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn findSerialPortConfig(mut function: libc::c_int)
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
    EXPECT_EQ(baudrate, 115200 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn serialWrite(mut instance: *mut libc::c_int,
                                     mut ch: uint8_t) {
    EXPECT_LT(serialWriteStub.pos,
              ::std::mem::size_of::<[uint8_t; 256]>() as libc::c_ulong);
    let fresh0 = serialWriteStub.pos;
    serialWriteStub.pos = serialWriteStub.pos + 1;
    serialWriteStub.buffer[fresh0 as usize] = ch;
    //TODO serialReadStub.buffer[serialReadStub.end++] = ch; //characters echoes back on the shared wire
    //printf("w: %02d 0x%02x\n", serialWriteStub.pos, ch);
}
#[no_mangle]
pub unsafe extern "C" fn serialTestResetPort() { }
#[no_mangle]
pub unsafe extern "C" fn isChecksumOkIa6b(mut ibusPacket: *const uint8_t,
                                          length: uint8_t) -> libc::c_int {
    panic!("Reached end of non-void function without returning");
}
#[no_mangle]
pub unsafe extern "C" fn initSharedIbusTelemetry(mut port: *mut libc::c_int) {
}
static mut stubTelemetryPacket: [uint8_t; 100] = [0; 100];
static mut stubTelemetryIgnoreRxChars: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn respondToIbusRequest(ibusPacket: *const uint8_t)
 -> uint8_t {
    let mut len: uint8_t = *ibusPacket.offset(0 as libc::c_int as isize);
    EXPECT_LT(len as libc::c_int,
              ::std::mem::size_of::<[uint8_t; 100]>() as libc::c_ulong);
    memcpy(stubTelemetryPacket.as_mut_ptr() as *mut libc::c_void,
           ibusPacket as *const libc::c_void, len as libc::c_ulong);
    return stubTelemetryIgnoreRxChars;
}
#[no_mangle]
pub unsafe extern "C" fn resetStubTelemetry() {
    memset(stubTelemetryPacket.as_mut_ptr() as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<[uint8_t; 100]>() as libc::c_ulong);
    stubTelemetryIgnoreRxChars = 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub static mut IbusRxInitUnitTest: libc::c_int = 0;
#[no_mangle]
pub unsafe extern "C" fn TEST_F(mut IbusRxInitUnitTest_0: libc::c_int,
                                mut Test_IbusRxNotEnabled: libc::c_int)
 -> libc::c_int {
    let initialRxConfig: libc::c_int = 0;
    panic!("Reached end of non-void function without returning");
    //TODO: Question: I'd expect that runtime conf was not initialized unless there was a serial port to run but the implementation states otherwise
    // EXPECT_EQ(0, rxRuntimeConfig.channelCount);
    // EXPECT_EQ(0, rxRuntimeConfig.rxRefreshRate);
    // EXPECT_EQ(NULL, rxRuntimeConfig.rcReadRawFn);
    // EXPECT_EQ(NULL, rxRuntimeConfig.rcFrameStatusFn);
}
#[no_mangle]
pub static mut IbusRxProtocollUnitTest: libc::c_int = 0;
#[no_mangle]
pub static mut public: libc::c_int = 0;
//handle that internal ibus position is not set to zero at init
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_0(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_InitialFrameState: libc::c_int)
 -> libc::c_int {
    panic!("Reached end of non-void function without returning");
    //TODO: is it ok to have undefined channel values after init?
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_1(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_IA6B_OnePacketReceived:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 32] =
        [0x20 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x7 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x9 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xa as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xb as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xc as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xd as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x84 as libc::c_int as uint8_t,
         0xff as libc::c_int as uint8_t]; //checksum
    //report frame complete once
    //check that channel values have been updated
    let mut i: libc::c_int = 0 as libc::c_int; //checksum
    while i < 14 as libc::c_int { i += 1 }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_2(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_IA6B_OnePacketReceivedWithBadCrc:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 32] =
        [0x20 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x1 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x2 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x3 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x4 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x5 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x6 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x7 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x8 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0x9 as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0xa as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0xb as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0xc as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0xd as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t];
    //no frame complete
    //check that channel values have not been updated
    let mut i: libc::c_int = 0 as libc::c_int; //checksum
    while i < 14 as libc::c_int { i += 1 }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_3(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_IA6B_HalfPacketReceived_then_interPacketGapReset:
                                      libc::c_int) -> libc::c_int {
    let packet_half: [uint8_t; 14] =
        [0x20 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0xab as libc::c_int as uint8_t,
         0x1 as libc::c_int as uint8_t, 0xab as libc::c_int as uint8_t,
         0x2 as libc::c_int as uint8_t, 0xab as libc::c_int as uint8_t,
         0x3 as libc::c_int as uint8_t, 0xab as libc::c_int as uint8_t,
         0x4 as libc::c_int as uint8_t, 0xab as libc::c_int as uint8_t,
         0x5 as libc::c_int as uint8_t, 0xab as libc::c_int as uint8_t];
    let packet_full: [uint8_t; 32] =
        [0x20 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x7 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x9 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xa as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xb as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xc as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xd as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x84 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t];
    microseconds_stub_value =
        (microseconds_stub_value as
             libc::c_uint).wrapping_add(5000 as libc::c_int as libc::c_uint)
            as uint32_t as uint32_t;
    //report frame complete once
    //check that channel values have been updated
    let mut i: libc::c_int = 0 as libc::c_int; //checksum
    while i < 14 as libc::c_int { i += 1 }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_4(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_IA6_OnePacketReceived: libc::c_int)
 -> libc::c_int {
    let mut packet: [uint8_t; 31] =
        [0x55 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0xb as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0xc as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0x5b as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t];
    //report frame complete once
    //check that channel values have been updated
    let mut i: libc::c_int = 0 as libc::c_int; //checksum
    while i < 14 as libc::c_int { i += 1 }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_5(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_IA6_OnePacketReceivedBadCrc:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 31] =
        [0x55 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x1 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x3 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x4 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x5 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x6 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x7 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x8 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0x9 as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0xa as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0xb as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0xc as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0xd as libc::c_int as uint8_t,
         0x33 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t];
    //no frame complete
    //check that channel values have not been updated
    let mut i: libc::c_int = 0 as libc::c_int; //checksum
    while i < 14 as libc::c_int { i += 1 }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_6(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_IA6B_OnePacketReceived_not_shared_port:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 32] =
        [0x20 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x1 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x2 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x3 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x4 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x7 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x8 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x9 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xa as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xb as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xc as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0xd as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
         0x84 as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t];
    serialTestResetPort();
    resetStubTelemetry();
    let initialRxConfig: libc::c_int = 0;
    //handle that internal ibus position is not set to zero at init
    microseconds_stub_value =
        (microseconds_stub_value as
             libc::c_uint).wrapping_add(5000 as libc::c_int as libc::c_uint)
            as uint32_t as uint32_t;
    //report frame complete once
    //check that channel values have been updated
    let mut i: libc::c_int = 0 as libc::c_int; //ibus sensor discovery
    while i < 14 as libc::c_int { i += 1 }
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_7(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_OneTelemetryPacketReceived:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 4] =
        [0x4 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
         0x7a as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t];
    resetStubTelemetry();
    receivePacket(packet.as_mut_ptr(),
                  ::std::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong);
    //no frame complete signal to rx system, but telemetry system is called
    EXPECT_TRUE((0 as libc::c_int ==
                     memcmp(stubTelemetryPacket.as_mut_ptr() as
                                *const libc::c_void,
                            packet.as_mut_ptr() as *const libc::c_void,
                            ::std::mem::size_of::<[uint8_t; 4]>() as
                                libc::c_ulong)) as
                    libc::c_int); //ibus sensor discovery
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_8(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_OneTelemetryIgnoreTxEchoToRx:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 4] =
        [0x4 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
         0x7a as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t];
    resetStubTelemetry();
    stubTelemetryIgnoreRxChars = 4 as libc::c_int as uint8_t;
    //given one packet received, that will respond with four characters to be ignored
    receivePacket(packet.as_mut_ptr(),
                  ::std::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong);
    //when those four bytes are sent and looped back
    resetStubTelemetry();
    receivePacket(packet.as_mut_ptr(),
                  ::std::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong);
    //then they are ignored
    //and then next packet can be received
    receivePacket(packet.as_mut_ptr(),
                  ::std::mem::size_of::<[uint8_t; 4]>() as
                      libc::c_ulong); //ibus sensor discovery
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST_F"]
pub unsafe extern "C" fn TEST_F_9(mut IbusRxProtocollUnitTest_0: libc::c_int,
                                  mut Test_OneTelemetryShouldNotIgnoreTxEchoAfterInterFrameGap:
                                      libc::c_int) -> libc::c_int {
    let mut packet: [uint8_t; 4] =
        [0x4 as libc::c_int as uint8_t, 0x81 as libc::c_int as uint8_t,
         0x7a as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t];
    resetStubTelemetry();
    stubTelemetryIgnoreRxChars = 4 as libc::c_int as uint8_t;
    //given one packet received, that will respond with four characters to be ignored
    receivePacket(packet.as_mut_ptr(),
                  ::std::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong);
    //when there is an interPacketGap
    microseconds_stub_value =
        (microseconds_stub_value as
             libc::c_uint).wrapping_add(5000 as libc::c_int as libc::c_uint)
            as uint32_t as uint32_t;
    resetStubTelemetry();
    //then next packet can be received
    receivePacket(packet.as_mut_ptr(),
                  ::std::mem::size_of::<[uint8_t; 4]>() as libc::c_ulong);
    panic!("Reached end of non-void function without returning");
}
