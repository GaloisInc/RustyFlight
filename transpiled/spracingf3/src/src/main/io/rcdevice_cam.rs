use core;
use libc;
extern "C" {
    #[no_mangle]
    static mut cmsInMenu: bool;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn getArmingDisableFlags() -> armingDisableFlags_e;
    #[no_mangle]
    fn beeper(mode: beeperMode_e);
    #[no_mangle]
    fn runcamDeviceInit(device: *mut runcamDevice_t);
    #[no_mangle]
    fn rcdeviceReceive(currentTimeUs: timeUs_t);
    // camera button simulation
    #[no_mangle]
    fn runcamDeviceSimulateCameraButton(device: *mut runcamDevice_t,
                                        operation: uint8_t) -> bool;
    // 5 key osd cable simulation
    #[no_mangle]
    fn runcamDeviceOpen5KeyOSDCableConnection(device: *mut runcamDevice_t,
                                              parseFunc:
                                                  rcdeviceRespParseFunc);
    #[no_mangle]
    fn runcamDeviceSimulate5KeyOSDCableButtonPress(device:
                                                       *mut runcamDevice_t,
                                                   operation: uint8_t,
                                                   parseFunc:
                                                       rcdeviceRespParseFunc);
    #[no_mangle]
    fn runcamDeviceSimulate5KeyOSDCableButtonRelease(device:
                                                         *mut runcamDevice_t,
                                                     parseFunc:
                                                         rcdeviceRespParseFunc);
    #[no_mangle]
    fn IS_RC_MODE_ACTIVE(boxId: boxId_e) -> bool;
    #[no_mangle]
    fn runcamDeviceClose5KeyOSDCableConnection(device: *mut runcamDevice_t,
                                               parseFunc:
                                                   rcdeviceRespParseFunc);
    #[no_mangle]
    static mut rcData: [int16_t; 18];
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct displayPortVTable_s {
    pub grab: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                         -> libc::c_int>,
    pub release: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                            -> libc::c_int>,
    pub clearScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                                -> libc::c_int>,
    pub drawScreen: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                               -> libc::c_int>,
    pub screenSize: Option<unsafe extern "C" fn(_: *const displayPort_t)
                               -> libc::c_int>,
    pub writeString: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                                 _: uint8_t, _: uint8_t,
                                                 _: *const libc::c_char)
                                -> libc::c_int>,
    pub writeChar: Option<unsafe extern "C" fn(_: *mut displayPort_t,
                                               _: uint8_t, _: uint8_t,
                                               _: uint8_t) -> libc::c_int>,
    pub isTransferInProgress: Option<unsafe extern "C" fn(_:
                                                              *const displayPort_t)
                                         -> bool>,
    pub heartbeat: Option<unsafe extern "C" fn(_: *mut displayPort_t)
                              -> libc::c_int>,
    pub resync: Option<unsafe extern "C" fn(_: *mut displayPort_t) -> ()>,
    pub isSynced: Option<unsafe extern "C" fn(_: *const displayPort_t)
                             -> bool>,
    pub txBytesFree: Option<unsafe extern "C" fn(_: *const displayPort_t)
                                -> uint32_t>,
}
pub type displayPort_t = displayPort_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct displayPort_s {
    pub vTable: *const displayPortVTable_s,
    pub device: *mut libc::c_void,
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub posX: uint8_t,
    pub posY: uint8_t,
    pub cleared: bool,
    pub cursorRow: int8_t,
    pub grabCount: int8_t,
}
pub type rc_alias = libc::c_uint;
pub const AUX8: rc_alias = 11;
pub const AUX7: rc_alias = 10;
pub const AUX6: rc_alias = 9;
pub const AUX5: rc_alias = 8;
pub const AUX4: rc_alias = 7;
pub const AUX3: rc_alias = 6;
pub const AUX2: rc_alias = 5;
pub const AUX1: rc_alias = 4;
pub const THROTTLE: rc_alias = 3;
pub const YAW: rc_alias = 2;
pub const PITCH: rc_alias = 1;
pub const ROLL: rc_alias = 0;
pub type boxId_e = libc::c_uint;
pub const CHECKBOX_ITEM_COUNT: boxId_e = 41;
pub const BOXACROTRAINER: boxId_e = 40;
pub const BOXPIDAUDIO: boxId_e = 39;
pub const BOXUSER4: boxId_e = 38;
pub const BOXUSER3: boxId_e = 37;
pub const BOXUSER2: boxId_e = 36;
pub const BOXUSER1: boxId_e = 35;
pub const BOXPARALYZE: boxId_e = 34;
pub const BOXVTXPITMODE: boxId_e = 33;
pub const BOXBEEPGPSCOUNT: boxId_e = 32;
pub const BOXPREARM: boxId_e = 31;
pub const BOXFLIPOVERAFTERCRASH: boxId_e = 30;
pub const BOXCAMERA3: boxId_e = 29;
pub const BOXCAMERA2: boxId_e = 28;
pub const BOXCAMERA1: boxId_e = 27;
pub const BOXBLACKBOXERASE: boxId_e = 26;
pub const BOXFPVANGLEMIX: boxId_e = 25;
pub const BOX3D: boxId_e = 24;
pub const BOXAIRMODE: boxId_e = 23;
pub const BOXBLACKBOX: boxId_e = 22;
pub const BOXSERVO3: boxId_e = 21;
pub const BOXSERVO2: boxId_e = 20;
pub const BOXSERVO1: boxId_e = 19;
pub const BOXTELEMETRY: boxId_e = 18;
pub const BOXOSD: boxId_e = 17;
pub const BOXCALIB: boxId_e = 16;
pub const BOXLEDLOW: boxId_e = 15;
pub const BOXBEEPERON: boxId_e = 14;
pub const BOXCAMSTAB: boxId_e = 13;
pub const BOXHEADADJ: boxId_e = 12;
pub const BOXANTIGRAVITY: boxId_e = 11;
pub const BOXID_FLIGHTMODE_LAST: boxId_e = 10;
pub const BOXGPSRESCUE: boxId_e = 10;
pub const BOXFAILSAFE: boxId_e = 9;
pub const BOXPASSTHRU: boxId_e = 8;
pub const BOXHEADFREE: boxId_e = 7;
pub const BOXGPSHOLD: boxId_e = 6;
pub const BOXGPSHOME: boxId_e = 5;
pub const BOXBARO: boxId_e = 4;
pub const BOXMAG: boxId_e = 3;
pub const BOXHORIZON: boxId_e = 2;
pub const BOXANGLE: boxId_e = 1;
pub const BOXARM: boxId_e = 0;
pub type C2RustUnnamed = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed = 2;
pub const ARMED: C2RustUnnamed = 1;
pub type armingDisableFlags_e = libc::c_uint;
pub const ARMING_DISABLED_ARM_SWITCH: armingDisableFlags_e = 524288;
pub const ARMING_DISABLED_GPS: armingDisableFlags_e = 262144;
pub const ARMING_DISABLED_PARALYZE: armingDisableFlags_e = 131072;
pub const ARMING_DISABLED_MSP: armingDisableFlags_e = 65536;
pub const ARMING_DISABLED_BST: armingDisableFlags_e = 32768;
pub const ARMING_DISABLED_OSD_MENU: armingDisableFlags_e = 16384;
pub const ARMING_DISABLED_CMS_MENU: armingDisableFlags_e = 8192;
pub const ARMING_DISABLED_CLI: armingDisableFlags_e = 4096;
pub const ARMING_DISABLED_CALIBRATING: armingDisableFlags_e = 2048;
pub const ARMING_DISABLED_LOAD: armingDisableFlags_e = 1024;
pub const ARMING_DISABLED_NOPREARM: armingDisableFlags_e = 512;
pub const ARMING_DISABLED_BOOT_GRACE_TIME: armingDisableFlags_e = 256;
pub const ARMING_DISABLED_ANGLE: armingDisableFlags_e = 128;
pub const ARMING_DISABLED_THROTTLE: armingDisableFlags_e = 64;
pub const ARMING_DISABLED_RUNAWAY_TAKEOFF: armingDisableFlags_e = 32;
pub const ARMING_DISABLED_BOXFAILSAFE: armingDisableFlags_e = 16;
pub const ARMING_DISABLED_BAD_RX_RECOVERY: armingDisableFlags_e = 8;
pub const ARMING_DISABLED_RX_FAILSAFE: armingDisableFlags_e = 4;
pub const ARMING_DISABLED_FAILSAFE: armingDisableFlags_e = 2;
pub const ARMING_DISABLED_NO_GYRO: armingDisableFlags_e = 1;
pub type beeperMode_e = libc::c_uint;
pub const BEEPER_ALL: beeperMode_e = 24;
pub const BEEPER_RC_SMOOTHING_INIT_FAIL: beeperMode_e = 23;
pub const BEEPER_CAM_CONNECTION_CLOSE: beeperMode_e = 22;
pub const BEEPER_CAM_CONNECTION_OPEN: beeperMode_e = 21;
pub const BEEPER_CRASH_FLIP_MODE: beeperMode_e = 20;
pub const BEEPER_BLACKBOX_ERASE: beeperMode_e = 19;
pub const BEEPER_USB: beeperMode_e = 18;
pub const BEEPER_SYSTEM_INIT: beeperMode_e = 17;
pub const BEEPER_ARMED: beeperMode_e = 16;
pub const BEEPER_DISARM_REPEAT: beeperMode_e = 15;
pub const BEEPER_MULTI_BEEPS: beeperMode_e = 14;
pub const BEEPER_READY_BEEP: beeperMode_e = 13;
pub const BEEPER_ACC_CALIBRATION_FAIL: beeperMode_e = 12;
pub const BEEPER_ACC_CALIBRATION: beeperMode_e = 11;
pub const BEEPER_RX_SET: beeperMode_e = 10;
pub const BEEPER_GPS_STATUS: beeperMode_e = 9;
pub const BEEPER_BAT_LOW: beeperMode_e = 8;
pub const BEEPER_BAT_CRIT_LOW: beeperMode_e = 7;
pub const BEEPER_ARMING_GPS_FIX: beeperMode_e = 6;
pub const BEEPER_ARMING: beeperMode_e = 5;
pub const BEEPER_DISARMING: beeperMode_e = 4;
pub const BEEPER_RX_LOST_LANDING: beeperMode_e = 3;
pub const BEEPER_RX_LOST: beeperMode_e = 2;
pub const BEEPER_GYRO_CALIBRATED: beeperMode_e = 1;
pub const BEEPER_SILENCE: beeperMode_e = 0;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
pub type serialPort_t = serialPort_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const RCDEVICE_PROTOCOL_FEATURE_CMS_MENU: C2RustUnnamed_0 = 256;
pub const RCDEVICE_PROTOCOL_FEATURE_STOP_RECORDING: C2RustUnnamed_0 = 128;
pub const RCDEVICE_PROTOCOL_FEATURE_START_RECORDING: C2RustUnnamed_0 = 64;
pub const RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE: C2RustUnnamed_0
          =
    8;
pub const RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE: C2RustUnnamed_0 = 4;
pub const RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON: C2RustUnnamed_0 = 2;
pub const RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON: C2RustUnnamed_0 =
    1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION: C2RustUnnamed_1
          =
    255;
pub const RCDEVICE_PROTOCOL_CAM_CTRL_STOP_RECORDING: C2RustUnnamed_1 = 4;
pub const RCDEVICE_PROTOCOL_CAM_CTRL_START_RECORDING: C2RustUnnamed_1 = 3;
pub const RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE: C2RustUnnamed_1 = 2;
pub const RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN: C2RustUnnamed_1 = 1;
pub const RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN: C2RustUnnamed_1 = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN: C2RustUnnamed_2 = 5;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP: C2RustUnnamed_2 = 4;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT: C2RustUnnamed_2 = 3;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT: C2RustUnnamed_2 = 2;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET: C2RustUnnamed_2 = 1;
pub const RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE: C2RustUnnamed_3 = 2;
pub const RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN: C2RustUnnamed_3 = 1;
pub type rcdeviceCamSimulationKeyEvent_e = libc::c_uint;
pub const RCDEVICE_CAM_KEY_RELEASE: rcdeviceCamSimulationKeyEvent_e = 8;
pub const RCDEVICE_CAM_KEY_CONNECTION_OPEN: rcdeviceCamSimulationKeyEvent_e =
    7;
pub const RCDEVICE_CAM_KEY_CONNECTION_CLOSE: rcdeviceCamSimulationKeyEvent_e =
    6;
pub const RCDEVICE_CAM_KEY_DOWN: rcdeviceCamSimulationKeyEvent_e = 5;
pub const RCDEVICE_CAM_KEY_RIGHT: rcdeviceCamSimulationKeyEvent_e = 4;
pub const RCDEVICE_CAM_KEY_UP: rcdeviceCamSimulationKeyEvent_e = 3;
pub const RCDEVICE_CAM_KEY_LEFT: rcdeviceCamSimulationKeyEvent_e = 2;
pub const RCDEVICE_CAM_KEY_ENTER: rcdeviceCamSimulationKeyEvent_e = 1;
pub const RCDEVICE_CAM_KEY_NONE: rcdeviceCamSimulationKeyEvent_e = 0;
pub type rcdevice_protocol_version_e = libc::c_uint;
pub const RCDEVICE_PROTOCOL_UNKNOWN: rcdevice_protocol_version_e = 2;
pub const RCDEVICE_PROTOCOL_VERSION_1_0: rcdevice_protocol_version_e = 1;
pub const RCDEVICE_PROTOCOL_RCSPLIT_VERSION: rcdevice_protocol_version_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct runcamDeviceInfo_s {
    pub protocolVersion: rcdevice_protocol_version_e,
    pub features: uint16_t,
}
// end of Runcam Device definition
pub type runcamDeviceInfo_t = runcamDeviceInfo_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct runcamDevice_s {
    pub serialPort: *mut serialPort_t,
    pub buffer: [uint8_t; 64],
    pub info: runcamDeviceInfo_t,
    pub isReady: bool,
}
pub type runcamDevice_t = runcamDevice_s;
pub type rcdeviceResponseStatus_e = libc::c_uint;
pub const RCDEVICE_RESP_TIMEOUT: rcdeviceResponseStatus_e = 2;
pub const RCDEVICE_RESP_INCORRECT_CRC: rcdeviceResponseStatus_e = 1;
pub const RCDEVICE_RESP_SUCCESS: rcdeviceResponseStatus_e = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rcdeviceResponseParseContext_s {
    pub command: uint8_t,
    pub expectedRespLen: uint8_t,
    pub recvRespLen: uint8_t,
    pub recvBuf: *mut uint8_t,
    pub timeout: timeMs_t,
    pub timeoutTimestamp: timeMs_t,
    pub parserFunc: rcdeviceRespParseFunc,
    pub device: *mut runcamDevice_t,
    pub paramData: [uint8_t; 62],
    pub paramDataLen: uint8_t,
    pub protocolVer: uint8_t,
    pub maxRetryTimes: libc::c_int,
    pub userInfo: *mut libc::c_void,
    pub result: rcdeviceResponseStatus_e,
}
pub type rcdeviceRespParseFunc
    =
    Option<unsafe extern "C" fn(_: *mut rcdeviceResponseParseContext_t)
               -> ()>;
pub type rcdeviceResponseParseContext_t = rcdeviceResponseParseContext_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct rcdeviceSwitchState_s {
    pub isActivated: bool,
}
pub type rcdeviceSwitchState_t = rcdeviceSwitchState_s;
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
static mut runcamDevice: runcamDevice_t =
    runcamDevice_t{serialPort: 0 as *const serialPort_t as *mut serialPort_t,
                   buffer: [0; 64],
                   info:
                       runcamDeviceInfo_t{protocolVersion:
                                              RCDEVICE_PROTOCOL_RCSPLIT_VERSION,
                                          features: 0,},
                   isReady: false,};
#[no_mangle]
pub static mut camDevice: *mut runcamDevice_t =
    unsafe { &runcamDevice as *const runcamDevice_t as *mut runcamDevice_t };
#[no_mangle]
pub static mut switchStates: [rcdeviceSwitchState_t; 3] =
    [rcdeviceSwitchState_t{isActivated: false,}; 3];
#[no_mangle]
pub static mut rcdeviceInMenu: bool = 0i32 != 0;
#[no_mangle]
pub static mut isButtonPressed: bool = 0i32 != 0;
#[no_mangle]
pub static mut waitingDeviceResponse: bool = 0i32 != 0;
unsafe extern "C" fn isFeatureSupported(mut feature: uint8_t) -> bool {
    if (*camDevice).info.features as libc::c_int & feature as libc::c_int != 0
       {
        return 1i32 != 0
    }
    return 0i32 != 0;
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub unsafe extern "C" fn rcdeviceIsEnabled() -> bool {
    return !(*camDevice).serialPort.is_null();
}
unsafe extern "C" fn rcdeviceIs5KeyEnabled() -> bool {
    if !(*camDevice).serialPort.is_null() &&
           isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_5_KEY_OSD_CABLE
                                  as libc::c_int as uint8_t) as libc::c_int !=
               0 {
        return 1i32 != 0
    }
    return 0i32 != 0;
}
unsafe extern "C" fn rcdeviceCameraControlProcess() {
    let mut i: boxId_e = BOXCAMERA1;
    while i as libc::c_uint <= BOXCAMERA3 as libc::c_int as libc::c_uint {
        let mut switchIndex: uint8_t =
            (i as
                 libc::c_uint).wrapping_sub(BOXCAMERA1 as libc::c_int as
                                                libc::c_uint) as uint8_t;
        if IS_RC_MODE_ACTIVE(i) {
            // check last state of this mode, if it's true, then ignore it.
            // Here is a logic to make a toggle control for this mode
            if !switchStates[switchIndex as usize].isActivated {
                let mut behavior: uint8_t =
                    RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION as
                        libc::c_int as uint8_t;
                match i as libc::c_uint {
                    27 => {
                        if isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_WIFI_BUTTON
                                                  as libc::c_int as uint8_t) {
                            // avoid display wifi page when arming, in the next firmware(>2.0) of rcsplit we have change the wifi page logic:
                    // when the wifi was turn on it won't turn off the analog video output, 
                    // and just put a wifi indicator on the right top of the video output. here is for the old split firmware
                            if armingFlags as libc::c_int &
                                   ARMED as libc::c_int == 0 &&
                                   getArmingDisableFlags() as libc::c_uint &
                                       ARMING_DISABLED_RUNAWAY_TAKEOFF as
                                           libc::c_int as libc::c_uint ==
                                       0i32 as libc::c_uint {
                                behavior =
                                    RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_WIFI_BTN
                                        as libc::c_int as uint8_t
                            }
                        }
                    }
                    28 => {
                        if isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_SIMULATE_POWER_BUTTON
                                                  as libc::c_int as uint8_t) {
                            behavior =
                                RCDEVICE_PROTOCOL_CAM_CTRL_SIMULATE_POWER_BTN
                                    as libc::c_int as uint8_t
                        }
                    }
                    29 => {
                        if isFeatureSupported(RCDEVICE_PROTOCOL_FEATURE_CHANGE_MODE
                                                  as libc::c_int as uint8_t) {
                            // avoid change camera mode when arming
                            if armingFlags as libc::c_int &
                                   ARMED as libc::c_int == 0 &&
                                   getArmingDisableFlags() as libc::c_uint &
                                       ARMING_DISABLED_RUNAWAY_TAKEOFF as
                                           libc::c_int as libc::c_uint ==
                                       0i32 as libc::c_uint {
                                behavior =
                                    RCDEVICE_PROTOCOL_CAM_CTRL_CHANGE_MODE as
                                        libc::c_int as uint8_t
                            }
                        }
                    }
                    _ => { }
                }
                if behavior as libc::c_int !=
                       RCDEVICE_PROTOCOL_CAM_CTRL_UNKNOWN_CAMERA_OPERATION as
                           libc::c_int {
                    runcamDeviceSimulateCameraButton(camDevice, behavior);
                    switchStates[switchIndex as usize].isActivated = 1i32 != 0
                }
            }
        } else { switchStates[switchIndex as usize].isActivated = 0i32 != 0 }
        i += 1
    };
}
unsafe extern "C" fn rcdeviceSimulationOSDCableFailed(mut ctx:
                                                          *mut rcdeviceResponseParseContext_t) {
    if (*ctx).command as libc::c_int == 0x4i32 {
        let mut operationID: uint8_t = (*ctx).paramData[0];
        if operationID as libc::c_int ==
               RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE as libc::c_int {
            waitingDeviceResponse = 0i32 != 0;
            return
        }
    } else { rcdeviceInMenu = 0i32 != 0; waitingDeviceResponse = 0i32 != 0 };
}
unsafe extern "C" fn rcdeviceSimulationRespHandle(mut ctx:
                                                      *mut rcdeviceResponseParseContext_t) {
    if (*ctx).result as libc::c_uint !=
           RCDEVICE_RESP_SUCCESS as libc::c_int as libc::c_uint {
        rcdeviceSimulationOSDCableFailed(ctx);
        return
    }
    match (*ctx).command as libc::c_int {
        3 => { isButtonPressed = 0i32 != 0 }
        4 => {
            // the high 4 bits is the operationID that we sent
        // the low 4 bits is the result code
            isButtonPressed = 1i32 != 0;
            let mut operationID: uint8_t = (*ctx).paramData[0];
            let mut errorCode: bool =
                *(*ctx).recvBuf.offset(1) as libc::c_int & 0xfi32 != 0;
            if operationID as libc::c_int ==
                   RCDEVICE_PROTOCOL_5KEY_CONNECTION_OPEN as libc::c_int {
                if errorCode as libc::c_int == 1i32 {
                    rcdeviceInMenu = 1i32 != 0;
                    beeper(BEEPER_CAM_CONNECTION_OPEN);
                } else {
                    rcdeviceInMenu = 0i32 != 0;
                    beeper(BEEPER_CAM_CONNECTION_CLOSE);
                }
            } else if operationID as libc::c_int ==
                          RCDEVICE_PROTOCOL_5KEY_CONNECTION_CLOSE as
                              libc::c_int {
                if errorCode as libc::c_int == 1i32 {
                    rcdeviceInMenu = 0i32 != 0;
                    beeper(BEEPER_CAM_CONNECTION_CLOSE);
                }
            }
        }
        2 => { isButtonPressed = 1i32 != 0 }
        _ => { }
    }
    waitingDeviceResponse = 0i32 != 0;
}
unsafe extern "C" fn rcdeviceCamSimulate5KeyCablePress(mut key:
                                                           rcdeviceCamSimulationKeyEvent_e) {
    let mut operation: uint8_t =
        RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE as libc::c_int as uint8_t;
    match key as libc::c_uint {
        2 => {
            operation =
                RCDEVICE_PROTOCOL_5KEY_SIMULATION_LEFT as libc::c_int as
                    uint8_t
        }
        3 => {
            operation =
                RCDEVICE_PROTOCOL_5KEY_SIMULATION_UP as libc::c_int as uint8_t
        }
        4 => {
            operation =
                RCDEVICE_PROTOCOL_5KEY_SIMULATION_RIGHT as libc::c_int as
                    uint8_t
        }
        5 => {
            operation =
                RCDEVICE_PROTOCOL_5KEY_SIMULATION_DOWN as libc::c_int as
                    uint8_t
        }
        1 => {
            operation =
                RCDEVICE_PROTOCOL_5KEY_SIMULATION_SET as libc::c_int as
                    uint8_t
        }
        0 | _ => {
            operation =
                RCDEVICE_PROTOCOL_5KEY_SIMULATION_NONE as libc::c_int as
                    uint8_t
        }
    }
    runcamDeviceSimulate5KeyOSDCableButtonPress(camDevice, operation,
                                                Some(rcdeviceSimulationRespHandle
                                                         as
                                                         unsafe extern "C" fn(_:
                                                                                  *mut rcdeviceResponseParseContext_t)
                                                             -> ()));
}
// used for unit test
#[no_mangle]
pub unsafe extern "C" fn rcdeviceSend5KeyOSDCableSimualtionEvent(mut key:
                                                                     rcdeviceCamSimulationKeyEvent_e) {
    match key as libc::c_uint {
        7 => {
            runcamDeviceOpen5KeyOSDCableConnection(camDevice,
                                                   Some(rcdeviceSimulationRespHandle
                                                            as
                                                            unsafe extern "C" fn(_:
                                                                                     *mut rcdeviceResponseParseContext_t)
                                                                -> ()));
        }
        6 => {
            runcamDeviceClose5KeyOSDCableConnection(camDevice,
                                                    Some(rcdeviceSimulationRespHandle
                                                             as
                                                             unsafe extern "C" fn(_:
                                                                                      *mut rcdeviceResponseParseContext_t)
                                                                 -> ()));
        }
        1 | 2 | 3 | 4 | 5 => { rcdeviceCamSimulate5KeyCablePress(key); }
        8 => {
            runcamDeviceSimulate5KeyOSDCableButtonRelease(camDevice,
                                                          Some(rcdeviceSimulationRespHandle
                                                                   as
                                                                   unsafe extern "C" fn(_:
                                                                                            *mut rcdeviceResponseParseContext_t)
                                                                       ->
                                                                           ()));
        }
        0 | _ => { }
    };
}
unsafe extern "C" fn rcdevice5KeySimulationProcess(mut currentTimeUs:
                                                       timeUs_t) {
    if cmsInMenu { return }
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 ||
           getArmingDisableFlags() as libc::c_uint &
               ARMING_DISABLED_RUNAWAY_TAKEOFF as libc::c_int as libc::c_uint
               != 0 {
        return
    }
    if waitingDeviceResponse { return }
    if isButtonPressed {
        if rcData[YAW as libc::c_int as usize] as libc::c_int > 1350i32 &&
               (rcData[YAW as libc::c_int as usize] as libc::c_int) < 1650i32
               &&
               (rcData[PITCH as libc::c_int as usize] as libc::c_int > 1350i32
                    &&
                    (rcData[PITCH as libc::c_int as usize] as libc::c_int) <
                        1650i32) &&
               (rcData[ROLL as libc::c_int as usize] as libc::c_int > 1350i32
                    &&
                    (rcData[ROLL as libc::c_int as usize] as libc::c_int) <
                        1650i32) {
            if rcdeviceIs5KeyEnabled() {
                rcdeviceSend5KeyOSDCableSimualtionEvent(RCDEVICE_CAM_KEY_RELEASE);
                waitingDeviceResponse = 1i32 != 0
            }
        }
    } else {
        let mut key: rcdeviceCamSimulationKeyEvent_e = RCDEVICE_CAM_KEY_NONE;
        if rcData[THROTTLE as libc::c_int as usize] as libc::c_int > 1350i32
               &&
               (rcData[THROTTLE as libc::c_int as usize] as libc::c_int) <
                   1650i32 &&
               (rcData[ROLL as libc::c_int as usize] as libc::c_int > 1350i32
                    &&
                    (rcData[ROLL as libc::c_int as usize] as libc::c_int) <
                        1650i32) &&
               (rcData[PITCH as libc::c_int as usize] as libc::c_int > 1350i32
                    &&
                    (rcData[PITCH as libc::c_int as usize] as libc::c_int) <
                        1650i32) &&
               (rcData[YAW as libc::c_int as usize] as libc::c_int) < 1080i32
           {
            // Disconnect HI YAW
            if rcdeviceInMenu { key = RCDEVICE_CAM_KEY_CONNECTION_CLOSE }
        } else if rcdeviceInMenu {
            if (rcData[ROLL as libc::c_int as usize] as libc::c_int) < 1080i32
               {
                // Left LO ROLL
                key = RCDEVICE_CAM_KEY_LEFT
            } else if rcData[PITCH as libc::c_int as usize] as libc::c_int >
                          1920i32 {
                // Up HI PITCH
                key = RCDEVICE_CAM_KEY_UP
            } else if rcData[ROLL as libc::c_int as usize] as libc::c_int >
                          1920i32 {
                // Right HI ROLL
                key = RCDEVICE_CAM_KEY_RIGHT
            } else if (rcData[PITCH as libc::c_int as usize] as libc::c_int) <
                          1080i32 {
                // Down LO PITCH
                key = RCDEVICE_CAM_KEY_DOWN
            } else if rcData[THROTTLE as libc::c_int as usize] as libc::c_int
                          > 1350i32 &&
                          (rcData[THROTTLE as libc::c_int as usize] as
                               libc::c_int) < 1650i32 &&
                          (rcData[ROLL as libc::c_int as usize] as libc::c_int
                               > 1350i32 &&
                               (rcData[ROLL as libc::c_int as usize] as
                                    libc::c_int) < 1650i32) &&
                          (rcData[PITCH as libc::c_int as usize] as
                               libc::c_int > 1350i32 &&
                               (rcData[PITCH as libc::c_int as usize] as
                                    libc::c_int) < 1650i32) &&
                          rcData[YAW as libc::c_int as usize] as libc::c_int >
                              1920i32 {
                // Enter HI YAW
                key = RCDEVICE_CAM_KEY_ENTER
            }
        } else if rcData[THROTTLE as libc::c_int as usize] as libc::c_int >
                      1350i32 &&
                      (rcData[THROTTLE as libc::c_int as usize] as
                           libc::c_int) < 1650i32 &&
                      (rcData[ROLL as libc::c_int as usize] as libc::c_int >
                           1350i32 &&
                           (rcData[ROLL as libc::c_int as usize] as
                                libc::c_int) < 1650i32) &&
                      (rcData[PITCH as libc::c_int as usize] as libc::c_int >
                           1350i32 &&
                           (rcData[PITCH as libc::c_int as usize] as
                                libc::c_int) < 1650i32) &&
                      rcData[YAW as libc::c_int as usize] as libc::c_int >
                          1920i32 {
            // Enter HI YAW
            key = RCDEVICE_CAM_KEY_CONNECTION_OPEN
        }
        if key as libc::c_uint !=
               RCDEVICE_CAM_KEY_NONE as libc::c_int as libc::c_uint {
            if rcdeviceIs5KeyEnabled() {
                rcdeviceSend5KeyOSDCableSimualtionEvent(key);
                waitingDeviceResponse = 1i32 != 0
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn rcdeviceUpdate(mut currentTimeUs: timeUs_t) {
    rcdeviceReceive(currentTimeUs);
    rcdeviceCameraControlProcess();
    rcdevice5KeySimulationProcess(currentTimeUs);
}
#[no_mangle]
pub unsafe extern "C" fn rcdeviceInit() {
    // open serial port
    runcamDeviceInit(camDevice);
    let mut i: boxId_e = BOXCAMERA1;
    while i as libc::c_uint <= BOXCAMERA3 as libc::c_int as libc::c_uint {
        let mut switchIndex: uint8_t =
            (i as
                 libc::c_uint).wrapping_sub(BOXCAMERA1 as libc::c_int as
                                                libc::c_uint) as uint8_t;
        switchStates[switchIndex as usize].isActivated = 1i32 != 0;
        i += 1
    };
}
