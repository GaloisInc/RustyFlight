#![allow(dead_code, mutable_transmutes, non_camel_case_types, non_snake_case,
         non_upper_case_globals, unused_assignments, unused_mut)]
#![register_tool(c2rust)]
#![feature(register_tool)]
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct testData_s {
    pub isRunCamSplitPortConfigurated: bool,
    pub isRunCamSplitOpenPortSupported: bool,
    pub maxTimesOfRespDataAvailable: libc::c_int,
    pub isAllowBufferReadWrite: bool,
    pub indexOfCurrentRespBuf: libc::c_int,
    pub responseBufCount: libc::c_int,
    pub responesBufs: [libc::c_int; 10],
    pub responseBufsLen: [libc::c_int; 10],
    pub responseDataReadPos: libc::c_int,
    pub millis: libc::c_int,
}
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
// interval [1000;2000]
pub type testData_t = testData_s;
static mut testData: testData_t =
    testData_t{isRunCamSplitPortConfigurated: false,
               isRunCamSplitOpenPortSupported: false,
               maxTimesOfRespDataAvailable: 0,
               isAllowBufferReadWrite: false,
               indexOfCurrentRespBuf: 0,
               responseBufCount: 0,
               responesBufs: [0; 10],
               responseBufsLen: [0; 10],
               responseDataReadPos: 0,
               millis: 0,};
unsafe extern "C" fn clearResponseBuff() { }
#[no_mangle]
pub unsafe extern "C" fn TEST(mut RCDeviceTest: libc::c_int,
                              mut TestRCSplitInitWithoutPortConfigurated:
                                  libc::c_int) -> libc::c_int {
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_0(mut RCDeviceTest: libc::c_int,
                                mut TestInitDevice: libc::c_int)
 -> libc::c_int {
    // test correct response
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_1(mut RCDeviceTest: libc::c_int,
                                mut TestInitDeviceWithInvalidResponse:
                                    libc::c_int) -> libc::c_int {
    // test correct response data with incorrect len
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // invalid crc
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // incomplete response data
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // test timeout
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_2(mut RCDeviceTest: libc::c_int,
                                mut TestWifiModeChangeWithDeviceUnready:
                                    libc::c_int) -> libc::c_int {
    // test correct response
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    // wrong response
    rcdeviceInit();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    // bind aux1 to wifi button with range [900,1600]
    // bind aux2 to power button with range [1900, 2100]
    // bind aux3 to change mode with range [1300, 1600]
    // make the binded mode inactive
    updateActivatedModes();
    // runn process loop
    rcdeviceUpdate(0 as libc::c_int);
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_3(mut RCDeviceTest: libc::c_int,
                                mut TestWifiModeChangeWithDeviceReady:
                                    libc::c_int) -> libc::c_int {
    // test correct response
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceInit();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    // bind aux1 to wifi button with range [900,1600]
    // bind aux2 to power button with range [1900, 2100]
    // bind aux3 to change mode with range [1300, 1600]
    updateActivatedModes();
    panic!("Reached end of non-void function without returning");
    // runn process loop
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_4(mut RCDeviceTest: libc::c_int,
                                mut TestWifiModeChangeCombine: libc::c_int)
 -> libc::c_int {
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceInit();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    // bind aux1, aux2, aux3 channel to wifi button, power button and change mode
    // bind aux1 to wifi button with range [900,1600]
    // bind aux2 to power button with range [1900, 2100]
    // bind aux3 to change mode with range [1300, 1600]
    // // make the binded mode inactive
    updateActivatedModes();
    // runn process loop
    // // make the binded mode inactive
    updateActivatedModes();
    updateActivatedModes();
    updateActivatedModes();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_5(mut RCDeviceTest: libc::c_int,
                                mut Test5KeyOSDCableSimulationProtocol:
                                    libc::c_int) -> libc::c_int {
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceInit();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // test timeout of open connection
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // open connection with correct response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // open connection with correct response but wrong data length 
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // open connection with invalid crc
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    // when crc wrong won't change the menu state
    clearResponseBuff();
    // test timeout of close connection
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    // close menu timeout won't change the menu state
    clearResponseBuff();
    // close connection with correct response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // close connection with correct response but wrong data length 
    // open menu again
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // close connection with response that invalid crc
    // open menu again
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // release button first
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate press button with no response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate press button with correct response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate press button with correct response but wrong data length 
    // release first
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate press button event with incorrect response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate release button with correct response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate release button with correct response but wrong data length
    // press first
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // simulate release button with incorrect response
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    panic!("Reached end of non-void function without returning");
}
#[export_name = "TEST"]
pub unsafe extern "C" fn TEST_6(mut RCDeviceTest: libc::c_int,
                                mut Test5KeyOSDCableSimulationWithout5KeyFeatureSupport:
                                    libc::c_int) -> libc::c_int {
    // test simulation without device init
    // THROTTLE Mid
    // ROLL Mid
    // PITCH Mid
    // Yaw High
    rcdeviceUpdate(0 as libc::c_int);
    // init device that have not 5 key OSD cable simulation feature
    memset(&mut testData as *mut testData_t as *mut libc::c_void,
           0 as libc::c_int,
           ::std::mem::size_of::<testData_t>() as libc::c_ulong);
    testData.isRunCamSplitOpenPortSupported = 1 as libc::c_int != 0;
    testData.isRunCamSplitPortConfigurated = 1 as libc::c_int != 0;
    testData.isAllowBufferReadWrite = 1 as libc::c_int != 0;
    rcdeviceInit();
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    rcdeviceReceive(millis() * 1000 as libc::c_int);
    clearResponseBuff();
    // open connection, rcdeviceInMenu will be false if the codes is right
    rcdeviceUpdate(0 as libc::c_int);
    clearResponseBuff();
    panic!("Reached end of non-void function without returning");
}
// testData.maxTimesOfRespDataAvailable = testData.responseDataLen + 1;
// reset the input buffer
// modifies streambuf so that written data are prepared for reading
// callback works for IRQ-based RX ONLY
// common serial initialisation code should move to serialPort::init()
