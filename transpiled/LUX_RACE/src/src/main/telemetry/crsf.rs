use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn crc8_dvb_s2_sbuf_append(dst: *mut sbuf_s, start: *mut uint8_t);
    // Disabling this, in favour of tfp_format to be used in cli.c
//int tfp_printf(const char *fmt, ...);
    #[no_mangle]
    fn tfp_sprintf(s: *mut libc::c_char, fmt: *const libc::c_char, _: ...)
     -> libc::c_int;
    #[no_mangle]
    fn sbufWriteU8(dst: *mut sbuf_t, val: uint8_t);
    #[no_mangle]
    fn sbufWriteU16BigEndian(dst: *mut sbuf_t, val: uint16_t);
    #[no_mangle]
    fn sbufWriteU32BigEndian(dst: *mut sbuf_t, val: uint32_t);
    #[no_mangle]
    fn sbufWriteData(dst: *mut sbuf_t, data: *const libc::c_void,
                     len: libc::c_int);
    #[no_mangle]
    fn sbufWriteString(dst: *mut sbuf_t, string: *const libc::c_char);
    #[no_mangle]
    fn sbufWriteStringWithZeroTerminator(dst: *mut sbuf_t,
                                         string: *const libc::c_char);
    #[no_mangle]
    fn sbufReadU8(src: *mut sbuf_t) -> uint8_t;
    #[no_mangle]
    fn sbufBytesRemaining(buf: *mut sbuf_t) -> libc::c_int;
    #[no_mangle]
    fn sbufPtr(buf: *mut sbuf_t) -> *mut uint8_t;
    #[no_mangle]
    fn sbufSwitchToReader(buf: *mut sbuf_t, base: *mut uint8_t);
    #[no_mangle]
    fn cmsDisplayPortRegister(pDisplay: *mut displayPort_t) -> bool;
    #[no_mangle]
    static mut systemConfig_System: systemConfig_t;
    #[no_mangle]
    fn isAirmodeActive() -> bool;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn displayPortCrsfInit() -> *mut displayPort_s;
    #[no_mangle]
    fn crsfDisplayPortScreen() -> *mut crsfDisplayPortScreen_t;
    #[no_mangle]
    fn crsfDisplayPortMenuOpen();
    #[no_mangle]
    fn crsfDisplayPortMenuExit();
    #[no_mangle]
    fn crsfDisplayPortRefresh();
    #[no_mangle]
    fn crsfDisplayPortNextRow() -> libc::c_int;
    #[no_mangle]
    fn crsfDisplayPortSetDimensions(rows: uint8_t, cols: uint8_t);
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    fn crsfRxWriteTelemetryData(data: *const libc::c_void, len: libc::c_int);
    #[no_mangle]
    fn crsfRxSendTelemetryData();
    #[no_mangle]
    fn crsfRxIsActive() -> bool;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
    #[no_mangle]
    fn isBatteryVoltageConfigured() -> bool;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn isAmperageConfigured() -> bool;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
    #[no_mangle]
    fn handleMspFrame(frameStart: *mut uint8_t, frameLength: libc::c_int)
     -> bool;
    #[no_mangle]
    fn sendMspReply(payloadSize: uint8_t, responseFn: mspResponseFnPtr)
     -> bool;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed = 1048576;
pub const FEATURE_OSD: C2RustUnnamed = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed = 8192;
pub const FEATURE_3D: C2RustUnnamed = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed = 512;
pub const FEATURE_GPS: C2RustUnnamed = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
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
// simple buffer-based serializer/deserializer without implicit size check
pub type sbuf_t = sbuf_s;
#[derive(Copy, Clone)]
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
// data pointer must be first (sbuf_t* is equivalent to uint8_t **)
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
pub type displayPort_t = displayPort_s;
#[derive(Copy, Clone)]
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
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct systemConfig_s {
    pub pidProfileIndex: uint8_t,
    pub activeRateProfile: uint8_t,
    pub debug_mode: uint8_t,
    pub task_statistics: uint8_t,
    pub rateProfile6PosSwitch: uint8_t,
    pub cpu_overclock: uint8_t,
    pub powerOnArmingGraceTime: uint8_t,
    pub boardIdentifier: [libc::c_char; 5],
}
pub type systemConfig_t = systemConfig_s;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPS_RESCUE_MODE: C2RustUnnamed_0 = 2048;
pub const FAILSAFE_MODE: C2RustUnnamed_0 = 1024;
pub const PASSTHRU_MODE: C2RustUnnamed_0 = 256;
pub const HEADFREE_MODE: C2RustUnnamed_0 = 64;
pub const GPS_HOLD_MODE: C2RustUnnamed_0 = 32;
pub const GPS_HOME_MODE: C2RustUnnamed_0 = 16;
pub const BARO_MODE: C2RustUnnamed_0 = 8;
pub const MAG_MODE: C2RustUnnamed_0 = 4;
pub const HORIZON_MODE: C2RustUnnamed_0 = 2;
pub const ANGLE_MODE: C2RustUnnamed_0 = 1;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_1 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_1 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_1 = 4;
pub const GPS_FIX: C2RustUnnamed_1 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_1 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed_2,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed_2 {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type crsfFrameType_e = libc::c_uint;
pub const CRSF_FRAMETYPE_DISPLAYPORT_CMD: crsfFrameType_e = 125;
pub const CRSF_FRAMETYPE_MSP_WRITE: crsfFrameType_e = 124;
pub const CRSF_FRAMETYPE_MSP_RESP: crsfFrameType_e = 123;
pub const CRSF_FRAMETYPE_MSP_REQ: crsfFrameType_e = 122;
pub const CRSF_FRAMETYPE_COMMAND: crsfFrameType_e = 50;
pub const CRSF_FRAMETYPE_PARAMETER_WRITE: crsfFrameType_e = 45;
pub const CRSF_FRAMETYPE_PARAMETER_READ: crsfFrameType_e = 44;
pub const CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: crsfFrameType_e = 43;
pub const CRSF_FRAMETYPE_DEVICE_INFO: crsfFrameType_e = 41;
pub const CRSF_FRAMETYPE_DEVICE_PING: crsfFrameType_e = 40;
pub const CRSF_FRAMETYPE_FLIGHT_MODE: crsfFrameType_e = 33;
pub const CRSF_FRAMETYPE_ATTITUDE: crsfFrameType_e = 30;
pub const CRSF_FRAMETYPE_RC_CHANNELS_PACKED: crsfFrameType_e = 22;
pub const CRSF_FRAMETYPE_LINK_STATISTICS: crsfFrameType_e = 20;
pub const CRSF_FRAMETYPE_BATTERY_SENSOR: crsfFrameType_e = 8;
pub const CRSF_FRAMETYPE_GPS: crsfFrameType_e = 2;
pub type C2RustUnnamed_3 = libc::c_uint;
// CMS state
// in seconds
// client request to poll/refresh cms menu
// client request to close cms menu
pub const CRSF_DISPLAYPORT_SUBCMD_POLL: C2RustUnnamed_3 = 5;
// client request to open cms menu
pub const CRSF_DISPLAYPORT_SUBCMD_CLOSE: C2RustUnnamed_3 = 4;
// clear client screen
pub const CRSF_DISPLAYPORT_SUBCMD_OPEN: C2RustUnnamed_3 = 3;
// transmit displayport buffer to remote
pub const CRSF_DISPLAYPORT_SUBCMD_CLEAR: C2RustUnnamed_3 = 2;
pub const CRSF_DISPLAYPORT_SUBCMD_UPDATE: C2RustUnnamed_3 = 1;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const CRSF_DISPLAYPORT_OPEN_COLS_OFFSET: C2RustUnnamed_4 = 2;
pub const CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET: C2RustUnnamed_4 = 1;
pub type C2RustUnnamed_5 = libc::c_uint;
// 11 bits per channel * 16 channels = 22 bytes.
pub const CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE: C2RustUnnamed_5 = 6;
pub const CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE: C2RustUnnamed_5 = 22;
pub const CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE: C2RustUnnamed_5 = 10;
pub const CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE: C2RustUnnamed_5 = 8;
pub const CRSF_FRAME_GPS_PAYLOAD_SIZE: C2RustUnnamed_5 = 15;
pub type C2RustUnnamed_6 = libc::c_uint;
// combined length of all fields except payload
// length of Extended Dest/Origin, TYPE and CRC fields combined
pub const CRSF_FRAME_LENGTH_NON_PAYLOAD: C2RustUnnamed_6 = 4;
// length of TYPE and CRC fields combined
pub const CRSF_FRAME_LENGTH_EXT_TYPE_CRC: C2RustUnnamed_6 = 4;
// length of CRC field
pub const CRSF_FRAME_LENGTH_TYPE_CRC: C2RustUnnamed_6 = 2;
// length of TYPE field
pub const CRSF_FRAME_LENGTH_CRC: C2RustUnnamed_6 = 1;
// length of FRAMELENGTH field
pub const CRSF_FRAME_LENGTH_TYPE: C2RustUnnamed_6 = 1;
// length of ADDRESS field
pub const CRSF_FRAME_LENGTH_FRAMELENGTH: C2RustUnnamed_6 = 1;
pub const CRSF_FRAME_LENGTH_ADDRESS: C2RustUnnamed_6 = 1;
pub type C2RustUnnamed_7 = libc::c_uint;
pub const CRSF_FRAME_ORIGIN_DEST_SIZE: C2RustUnnamed_7 = 2;
pub const CRSF_FRAME_RX_MSP_FRAME_SIZE: C2RustUnnamed_7 = 8;
pub const CRSF_FRAME_TX_MSP_FRAME_SIZE: C2RustUnnamed_7 = 58;
pub type C2RustUnnamed_8 = libc::c_uint;
pub const CRSF_ADDRESS_CRSF_TRANSMITTER: C2RustUnnamed_8 = 238;
pub const CRSF_ADDRESS_CRSF_RECEIVER: C2RustUnnamed_8 = 236;
pub const CRSF_ADDRESS_RADIO_TRANSMITTER: C2RustUnnamed_8 = 234;
pub const CRSF_ADDRESS_RACE_TAG: C2RustUnnamed_8 = 204;
pub const CRSF_ADDRESS_RESERVED2: C2RustUnnamed_8 = 202;
pub const CRSF_ADDRESS_FLIGHT_CONTROLLER: C2RustUnnamed_8 = 200;
pub const CRSF_ADDRESS_TBS_BLACKBOX: C2RustUnnamed_8 = 196;
pub const CRSF_ADDRESS_GPS: C2RustUnnamed_8 = 194;
pub const CRSF_ADDRESS_CURRENT_SENSOR: C2RustUnnamed_8 = 192;
pub const CRSF_ADDRESS_RESERVED1: C2RustUnnamed_8 = 138;
pub const CRSF_ADDRESS_TBS_CORE_PNP_PRO: C2RustUnnamed_8 = 128;
pub const CRSF_ADDRESS_USB: C2RustUnnamed_8 = 16;
pub const CRSF_ADDRESS_BROADCAST: C2RustUnnamed_8 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct crsfDisplayPortScreen_s {
    pub buffer: [libc::c_char; 288],
    pub pendingTransport: [bool; 9],
    pub rows: uint8_t,
    pub cols: uint8_t,
    pub reset: bool,
}
pub type crsfDisplayPortScreen_t = crsfDisplayPortScreen_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsLocation_s {
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
}
/* LLH Location in NEU axis system */
pub type gpsLocation_t = gpsLocation_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gpsSolutionData_s {
    pub llh: gpsLocation_t,
    pub groundSpeed: uint16_t,
    pub groundCourse: uint16_t,
    pub hdop: uint16_t,
    pub numSat: uint8_t,
}
pub type gpsSolutionData_t = gpsSolutionData_s;
pub type C2RustUnnamed_9 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_9 = 64;
pub const SENSOR_GPS: C2RustUnnamed_9 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_9 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_9 = 16;
pub const SENSOR_MAG: C2RustUnnamed_9 = 8;
pub const SENSOR_BARO: C2RustUnnamed_9 = 4;
pub const SENSOR_ACC: C2RustUnnamed_9 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_9 = 1;
pub const CRSF_FRAME_GPS_INDEX: C2RustUnnamed_10 = 3;
pub const CRSF_FRAME_FLIGHT_MODE_INDEX: C2RustUnnamed_10 = 2;
pub const CRSF_FRAME_BATTERY_SENSOR_INDEX: C2RustUnnamed_10 = 1;
pub const CRSF_FRAME_ATTITUDE_INDEX: C2RustUnnamed_10 = 0;
pub type mspResponseFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t) -> ()>;
pub type mspBuffer_t = mspBuffer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspBuffer_s {
    pub bytes: [uint8_t; 96],
    pub len: libc::c_int,
}
pub type C2RustUnnamed_10 = libc::c_uint;
pub const CRSF_SCHEDULE_COUNT_MAX: C2RustUnnamed_10 = 4;
pub const CRSF_FRAME_START_INDEX: C2RustUnnamed_10 = 0;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
// set BASEPRI_MAX, with global memory barrier, returns true
#[inline]
unsafe extern "C" fn __basepriSetMemRetVal(mut prio: uint8_t) -> uint8_t {
    __set_BASEPRI_MAX(prio as
                          libc::c_int); // start at byte 2, since CRC does not include device address and frame length
    return 1 as libc::c_int as uint8_t;
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn systemConfig() -> *const systemConfig_t {
    return &mut systemConfig_System;
}
static mut crsfTelemetryEnabled: bool = false;
static mut deviceInfoReplyPending: bool = false;
static mut crsfFrame: [uint8_t; 64] = [0; 64];
static mut mspRxBuffer: mspBuffer_t = mspBuffer_t{bytes: [0; 96], len: 0,};
#[no_mangle]
pub unsafe extern "C" fn initCrsfMspBuffer() {
    mspRxBuffer.len = 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn bufferCrsfMspFrame(mut frameStart: *mut uint8_t,
                                            mut frameLength: libc::c_int)
 -> bool {
    if mspRxBuffer.len + 1 as libc::c_int + frameLength > 96 as libc::c_int {
        return 0 as libc::c_int != 0
    } else {
        let mut p: *mut uint8_t =
            mspRxBuffer.bytes.as_mut_ptr().offset(mspRxBuffer.len as isize);
        let fresh0 = p;
        p = p.offset(1);
        *fresh0 = frameLength as uint8_t;
        memcpy(p as *mut libc::c_void, frameStart as *const libc::c_void,
               frameLength as libc::c_ulong);
        mspRxBuffer.len += 1 as libc::c_int + frameLength;
        return 1 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn handleCrsfMspFrameBuffer(mut payloadSize: uint8_t,
                                                  mut responseFn:
                                                      mspResponseFnPtr)
 -> bool {
    let mut requestHandled: bool = 0 as libc::c_int != 0;
    if mspRxBuffer.len == 0 { return 0 as libc::c_int != 0 }
    let mut pos: libc::c_int = 0 as libc::c_int;
    loop  {
        let mspFrameLength: libc::c_int =
            mspRxBuffer.bytes[pos as usize] as libc::c_int;
        if handleMspFrame(&mut *mspRxBuffer.bytes.as_mut_ptr().offset((1 as
                                                                           libc::c_int
                                                                           +
                                                                           pos)
                                                                          as
                                                                          isize),
                          mspFrameLength) {
            requestHandled =
                (requestHandled as libc::c_int |
                     sendMspReply(payloadSize, responseFn) as libc::c_int) as
                    bool
        }
        pos += 1 as libc::c_int + mspFrameLength;
        let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
        let mut __ToDo: uint8_t =
            __basepriSetMemRetVal((((1 as libc::c_int) <<
                                        (4 as libc::c_int as
                                             libc::c_uint).wrapping_sub((7 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_sub(0x500
                                                                                                            as
                                                                                                            libc::c_int
                                                                                                            as
                                                                                                            uint32_t
                                                                                                            >>
                                                                                                            8
                                                                                                                as
                                                                                                                libc::c_int))
                                        |
                                        1 as libc::c_int &
                                            0xf as libc::c_int >>
                                                (7 as libc::c_int as
                                                     libc::c_uint).wrapping_sub(0x500
                                                                                    as
                                                                                    libc::c_int
                                                                                    as
                                                                                    uint32_t
                                                                                    >>
                                                                                    8
                                                                                        as
                                                                                        libc::c_int))
                                       << 4 as libc::c_int &
                                       0xf0 as libc::c_int) as uint8_t);
        while __ToDo != 0 {
            if pos >= mspRxBuffer.len {
                mspRxBuffer.len = 0 as libc::c_int;
                return requestHandled
            }
            __ToDo = 0 as libc::c_int as uint8_t
        }
    };
}
unsafe extern "C" fn crsfInitializeFrame(mut dst: *mut sbuf_t) {
    (*dst).ptr = crsfFrame.as_mut_ptr();
    (*dst).end =
        &mut *crsfFrame.as_mut_ptr().offset((::core::mem::size_of::<[uint8_t; 64]>()
                                                 as
                                                 libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                                                 as
                                                                                 libc::c_ulong)
                                                as isize) as *mut uint8_t;
    sbufWriteU8(dst, 0xc8 as libc::c_int as uint8_t);
}
unsafe extern "C" fn crsfFinalize(mut dst: *mut sbuf_t) {
    crc8_dvb_s2_sbuf_append(dst,
                            &mut *crsfFrame.as_mut_ptr().offset(2 as
                                                                    libc::c_int
                                                                    as
                                                                    isize));
    sbufSwitchToReader(dst, crsfFrame.as_mut_ptr());
    // write the telemetry frame to the receiver.
    crsfRxWriteTelemetryData(sbufPtr(dst) as *const libc::c_void,
                             sbufBytesRemaining(dst)); // start at byte 2, since CRC does not include device address and frame length
}
unsafe extern "C" fn crsfFinalizeBuf(mut dst: *mut sbuf_t,
                                     mut frame: *mut uint8_t) -> libc::c_int {
    crc8_dvb_s2_sbuf_append(dst,
                            &mut *crsfFrame.as_mut_ptr().offset(2 as
                                                                    libc::c_int
                                                                    as
                                                                    isize));
    sbufSwitchToReader(dst, crsfFrame.as_mut_ptr());
    let frameSize: libc::c_int = sbufBytesRemaining(dst);
    let mut ii: libc::c_int = 0 as libc::c_int;
    while sbufBytesRemaining(dst) != 0 {
        *frame.offset(ii as isize) = sbufReadU8(dst);
        ii += 1
    }
    return frameSize;
}
/*
CRSF frame has the structure:
<Device address> <Frame length> <Type> <Payload> <CRC>
Device address: (uint8_t)
Frame length:   length in  bytes including Type (uint8_t)
Type:           (uint8_t)
CRC:            (uint8_t), crc of <Type> and <Payload>
*/
/*
0x02 GPS
Payload:
int32_t     Latitude ( degree / 10`000`000 )
int32_t     Longitude (degree / 10`000`000 )
uint16_t    Groundspeed ( km/h / 10 )
uint16_t    GPS heading ( degree / 100 )
uint16      Altitude ( meter ­1000m offset )
uint8_t     Satellites in use ( counter )
*/
#[no_mangle]
pub unsafe extern "C" fn crsfFrameGps(mut dst: *mut sbuf_t) {
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst,
                (CRSF_FRAME_GPS_PAYLOAD_SIZE as libc::c_int +
                     CRSF_FRAME_LENGTH_TYPE_CRC as libc::c_int) as
                    uint8_t); // CRSF and betaflight use same units for degrees
    sbufWriteU8(dst,
                CRSF_FRAMETYPE_GPS as libc::c_int as
                    uint8_t); // gpsSol.groundSpeed is in 0.1m/s
    sbufWriteU32BigEndian(dst,
                          gpsSol.llh.lat as
                              uint32_t); // gpsSol.groundCourse is degrees * 10
    sbufWriteU32BigEndian(dst, gpsSol.llh.lon as uint32_t);
    sbufWriteU16BigEndian(dst,
                          ((gpsSol.groundSpeed as libc::c_int *
                                36 as libc::c_int + 5 as libc::c_int) /
                               10 as libc::c_int) as uint16_t);
    sbufWriteU16BigEndian(dst,
                          (gpsSol.groundCourse as libc::c_int *
                               10 as libc::c_int) as uint16_t);
    //Send real GPS altitude only if it's reliable (there's a GPS fix)
    let altitude: uint16_t =
        ((if stateFlags as libc::c_int & GPS_FIX as libc::c_int != 0 {
              (gpsSol.llh.alt) / 100 as libc::c_int
          } else { 0 as libc::c_int }) + 1000 as libc::c_int) as uint16_t;
    sbufWriteU16BigEndian(dst, altitude);
    sbufWriteU8(dst, gpsSol.numSat);
}
/*
0x08 Battery sensor
Payload:
uint16_t    Voltage ( mV * 100 )
uint16_t    Current ( mA * 100 )
uint24_t    Fuel ( drawn mAh )
uint8_t     Battery remaining ( percent )
*/
#[no_mangle]
pub unsafe extern "C" fn crsfFrameBatterySensor(mut dst: *mut sbuf_t) {
    // use sbufWrite since CRC does not include frame length
    sbufWriteU8(dst,
                (CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE as libc::c_int +
                     CRSF_FRAME_LENGTH_TYPE_CRC as libc::c_int) as
                    uint8_t); // vbat is in units of 0.1V
    sbufWriteU8(dst, CRSF_FRAMETYPE_BATTERY_SENSOR as libc::c_int as uint8_t);
    sbufWriteU16BigEndian(dst, getBatteryVoltage());
    sbufWriteU16BigEndian(dst,
                          (getAmperage() / 10 as libc::c_int) as uint16_t);
    let mAhDrawn: uint32_t = getMAhDrawn() as uint32_t;
    let batteryRemainingPercentage: uint8_t =
        calculateBatteryPercentageRemaining();
    sbufWriteU8(dst, (mAhDrawn >> 16 as libc::c_int) as uint8_t);
    sbufWriteU8(dst, (mAhDrawn >> 8 as libc::c_int) as uint8_t);
    sbufWriteU8(dst, mAhDrawn as uint8_t);
    sbufWriteU8(dst, batteryRemainingPercentage);
}
/*
0x1E Attitude
Payload:
int16_t     Pitch angle ( rad / 10000 )
int16_t     Roll angle ( rad / 10000 )
int16_t     Yaw angle ( rad / 10000 )
*/
#[no_mangle]
pub unsafe extern "C" fn crsfFrameAttitude(mut dst: *mut sbuf_t) {
    sbufWriteU8(dst,
                (CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE as libc::c_int +
                     CRSF_FRAME_LENGTH_TYPE_CRC as libc::c_int) as uint8_t);
    sbufWriteU8(dst, CRSF_FRAMETYPE_ATTITUDE as libc::c_int as uint8_t);
    sbufWriteU16BigEndian(dst,
                          (1000.0f32 *
                               attitude.values.pitch as libc::c_int as
                                   libc::c_float *
                               (3.14159265358979323846f32 / 180.0f32)) as
                              int16_t as uint16_t);
    sbufWriteU16BigEndian(dst,
                          (1000.0f32 *
                               attitude.values.roll as libc::c_int as
                                   libc::c_float *
                               (3.14159265358979323846f32 / 180.0f32)) as
                              int16_t as uint16_t);
    sbufWriteU16BigEndian(dst,
                          (1000.0f32 *
                               attitude.values.yaw as libc::c_int as
                                   libc::c_float *
                               (3.14159265358979323846f32 / 180.0f32)) as
                              int16_t as uint16_t);
}
/*
0x21 Flight mode text based
Payload:
char[]      Flight mode ( Null terminated string )
*/
#[no_mangle]
pub unsafe extern "C" fn crsfFrameFlightMode(mut dst: *mut sbuf_t) {
    // write zero for frame length, since we don't know it yet
    let mut lengthPtr: *mut uint8_t = sbufPtr(dst);
    sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
    sbufWriteU8(dst, CRSF_FRAMETYPE_FLIGHT_MODE as libc::c_int as uint8_t);
    // use same logic as OSD, so telemetry displays same flight text as OSD
    let mut flightMode: *const libc::c_char =
        b"ACRO\x00" as *const u8 as
            *const libc::c_char; // zero-terminate string
    if isAirmodeActive() {
        flightMode = b"AIR\x00" as *const u8 as *const libc::c_char
    }
    if flightModeFlags as libc::c_int & FAILSAFE_MODE as libc::c_int != 0 {
        flightMode = b"!FS\x00" as *const u8 as *const libc::c_char
    } else if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0
     {
        flightMode = b"STAB\x00" as *const u8 as *const libc::c_char
    } else if flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int !=
                  0 {
        flightMode = b"HOR\x00" as *const u8 as *const libc::c_char
    }
    sbufWriteString(dst, flightMode);
    sbufWriteU8(dst, '\u{0}' as i32 as uint8_t);
    // write in the frame length
    *lengthPtr =
        sbufPtr(dst).wrapping_offset_from(lengthPtr) as libc::c_long as
            uint8_t;
}
/*
0x29 Device Info
Payload:
uint8_t     Destination
uint8_t     Origin
char[]      Device Name ( Null terminated string )
uint32_t    Null Bytes
uint32_t    Null Bytes
uint32_t    Null Bytes
uint8_t     255 (Max MSP Parameter)
uint8_t     0x01 (Parameter version 1)
*/
#[no_mangle]
pub unsafe extern "C" fn crsfFrameDeviceInfo(mut dst: *mut sbuf_t) {
    let mut buff: [libc::c_char; 30] = [0; 30];
    tfp_sprintf(buff.as_mut_ptr(),
                b"%s %s: %s\x00" as *const u8 as *const libc::c_char,
                b"Cleanflight\x00" as *const u8 as *const libc::c_char,
                b"2.5.0\x00" as *const u8 as *const libc::c_char,
                (*systemConfig()).boardIdentifier.as_ptr());
    let mut lengthPtr: *mut uint8_t = sbufPtr(dst);
    sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
    sbufWriteU8(dst, CRSF_FRAMETYPE_DEVICE_INFO as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_RADIO_TRANSMITTER as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_FLIGHT_CONTROLLER as libc::c_int as uint8_t);
    sbufWriteStringWithZeroTerminator(dst, buff.as_mut_ptr());
    let mut ii: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while ii < 12 as libc::c_int as libc::c_uint {
        sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
        ii = ii.wrapping_add(1)
    }
    sbufWriteU8(dst, 0 as libc::c_int as uint8_t);
    sbufWriteU8(dst, 0x1 as libc::c_int as uint8_t);
    *lengthPtr =
        sbufPtr(dst).wrapping_offset_from(lengthPtr) as libc::c_long as
            uint8_t;
}
unsafe extern "C" fn crsfFrameDisplayPortRow(mut dst: *mut sbuf_t,
                                             mut row: uint8_t) {
    let mut lengthPtr: *mut uint8_t = sbufPtr(dst);
    let mut buflen: uint8_t = (*crsfDisplayPortScreen()).cols;
    let mut rowStart: *mut libc::c_char =
        &mut *(*(crsfDisplayPortScreen as
                     unsafe extern "C" fn()
                         ->
                             *mut crsfDisplayPortScreen_t)()).buffer.as_mut_ptr().offset((row
                                                                                              as
                                                                                              libc::c_int
                                                                                              *
                                                                                              buflen
                                                                                                  as
                                                                                                  libc::c_int)
                                                                                             as
                                                                                             isize)
            as *mut libc::c_char;
    let frameLength: uint8_t =
        (CRSF_FRAME_LENGTH_EXT_TYPE_CRC as libc::c_int +
             buflen as libc::c_int) as uint8_t;
    sbufWriteU8(dst, frameLength);
    sbufWriteU8(dst,
                CRSF_FRAMETYPE_DISPLAYPORT_CMD as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_RADIO_TRANSMITTER as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_FLIGHT_CONTROLLER as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_DISPLAYPORT_SUBCMD_UPDATE as libc::c_int as uint8_t);
    sbufWriteU8(dst, row);
    sbufWriteData(dst, rowStart as *const libc::c_void,
                  buflen as libc::c_int);
    *lengthPtr =
        sbufPtr(dst).wrapping_offset_from(lengthPtr) as libc::c_long as
            uint8_t;
}
unsafe extern "C" fn crsfFrameDisplayPortClear(mut dst: *mut sbuf_t) {
    let mut lengthPtr: *mut uint8_t = sbufPtr(dst);
    sbufWriteU8(dst,
                (32 as libc::c_int +
                     CRSF_FRAME_LENGTH_EXT_TYPE_CRC as libc::c_int) as
                    uint8_t);
    sbufWriteU8(dst,
                CRSF_FRAMETYPE_DISPLAYPORT_CMD as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_RADIO_TRANSMITTER as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_FLIGHT_CONTROLLER as libc::c_int as uint8_t);
    sbufWriteU8(dst, CRSF_DISPLAYPORT_SUBCMD_CLEAR as libc::c_int as uint8_t);
    *lengthPtr =
        sbufPtr(dst).wrapping_offset_from(lengthPtr) as libc::c_long as
            uint8_t;
}
static mut crsfScheduleCount: uint8_t = 0;
static mut crsfSchedule: [uint8_t; 4] = [0; 4];
static mut mspReplyPending: bool = false;
#[no_mangle]
pub unsafe extern "C" fn crsfScheduleMspResponse() {
    mspReplyPending = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn crsfSendMspResponse(mut payload: *mut uint8_t) {
    let mut crsfPayloadBuf: sbuf_t =
        sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
    let mut dst: *mut sbuf_t = &mut crsfPayloadBuf;
    crsfInitializeFrame(dst);
    sbufWriteU8(dst,
                (CRSF_FRAME_TX_MSP_FRAME_SIZE as libc::c_int +
                     CRSF_FRAME_LENGTH_EXT_TYPE_CRC as libc::c_int) as
                    uint8_t);
    sbufWriteU8(dst, CRSF_FRAMETYPE_MSP_RESP as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_RADIO_TRANSMITTER as libc::c_int as uint8_t);
    sbufWriteU8(dst,
                CRSF_ADDRESS_FLIGHT_CONTROLLER as libc::c_int as uint8_t);
    sbufWriteData(dst, payload as *const libc::c_void,
                  CRSF_FRAME_TX_MSP_FRAME_SIZE as libc::c_int);
    crsfFinalize(dst);
}
unsafe extern "C" fn processCrsf() {
    static mut crsfScheduleIndex: uint8_t = 0 as libc::c_int as uint8_t;
    let currentSchedule: uint8_t = crsfSchedule[crsfScheduleIndex as usize];
    let mut crsfPayloadBuf: sbuf_t =
        sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
    let mut dst: *mut sbuf_t = &mut crsfPayloadBuf;
    if currentSchedule as libc::c_int &
           (1 as libc::c_int) << CRSF_FRAME_ATTITUDE_INDEX as libc::c_int != 0
       {
        crsfInitializeFrame(dst);
        crsfFrameAttitude(dst);
        crsfFinalize(dst);
    }
    if currentSchedule as libc::c_int &
           (1 as libc::c_int) <<
               CRSF_FRAME_BATTERY_SENSOR_INDEX as libc::c_int != 0 {
        crsfInitializeFrame(dst);
        crsfFrameBatterySensor(dst);
        crsfFinalize(dst);
    }
    if currentSchedule as libc::c_int &
           (1 as libc::c_int) << CRSF_FRAME_FLIGHT_MODE_INDEX as libc::c_int
           != 0 {
        crsfInitializeFrame(dst);
        crsfFrameFlightMode(dst);
        crsfFinalize(dst);
    }
    crsfScheduleIndex =
        ((crsfScheduleIndex as libc::c_int + 1 as libc::c_int) %
             crsfScheduleCount as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn crsfScheduleDeviceInfoResponse() {
    deviceInfoReplyPending = 1 as libc::c_int != 0;
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
pub unsafe extern "C" fn initCrsfTelemetry() {
    // check if there is a serial port open for CRSF telemetry (ie opened by the CRSF RX)
    // and feature is enabled, if so, set CRSF telemetry enabled
    crsfTelemetryEnabled = crsfRxIsActive();
    deviceInfoReplyPending = 0 as libc::c_int != 0;
    mspReplyPending = 0 as libc::c_int != 0;
    cmsDisplayPortRegister(displayPortCrsfInit());
    let mut index: libc::c_int = 0 as libc::c_int;
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        let fresh1 = index;
        index = index + 1;
        crsfSchedule[fresh1 as usize] =
            ((1 as libc::c_int) << CRSF_FRAME_ATTITUDE_INDEX as libc::c_int)
                as uint8_t
    }
    if isBatteryVoltageConfigured() as libc::c_int != 0 ||
           isAmperageConfigured() as libc::c_int != 0 {
        let fresh2 = index;
        index = index + 1;
        crsfSchedule[fresh2 as usize] =
            ((1 as libc::c_int) <<
                 CRSF_FRAME_BATTERY_SENSOR_INDEX as libc::c_int) as uint8_t
    }
    let fresh3 = index;
    index = index + 1;
    crsfSchedule[fresh3 as usize] =
        ((1 as libc::c_int) << CRSF_FRAME_FLIGHT_MODE_INDEX as libc::c_int) as
            uint8_t;
    if feature(FEATURE_GPS as libc::c_int as uint32_t) {
        let fresh4 = index;
        index = index + 1;
        crsfSchedule[fresh4 as usize] =
            ((1 as libc::c_int) << CRSF_FRAME_GPS_INDEX as libc::c_int) as
                uint8_t
    }
    crsfScheduleCount = index as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn checkCrsfTelemetryState() -> bool {
    return crsfTelemetryEnabled;
}
#[no_mangle]
pub unsafe extern "C" fn crsfProcessDisplayPortCmd(mut frameStart:
                                                       *mut uint8_t) {
    let mut cmd: uint8_t = *frameStart;
    let mut rows: uint8_t = 0;
    let mut cols: uint8_t = 0;
    match cmd as libc::c_int {
        3 => {
            rows =
                *frameStart.offset(CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET as
                                       libc::c_int as isize);
            cols =
                *frameStart.offset(CRSF_DISPLAYPORT_OPEN_COLS_OFFSET as
                                       libc::c_int as isize);
            crsfDisplayPortSetDimensions(rows, cols);
            crsfDisplayPortMenuOpen();
        }
        4 => { crsfDisplayPortMenuExit(); }
        5 => { crsfDisplayPortRefresh(); }
        _ => { }
    };
}
/*
 * Called periodically by the scheduler
 */
#[no_mangle]
pub unsafe extern "C" fn handleCrsfTelemetry(mut currentTimeUs: timeUs_t) {
    static mut crsfLastCycleTime: uint32_t = 0;
    if !crsfTelemetryEnabled { return }
    // Give the receiver a chance to send any outstanding telemetry data.
    // This needs to be done at high frequency, to enable the RX to send the telemetry frame
    // in between the RX frames.
    crsfRxSendTelemetryData();
    // Send ad-hoc response frames as soon as possible
    if mspReplyPending {
        mspReplyPending =
            handleCrsfMspFrameBuffer(CRSF_FRAME_TX_MSP_FRAME_SIZE as
                                         libc::c_int as uint8_t,
                                     Some(crsfSendMspResponse as
                                              unsafe extern "C" fn(_:
                                                                       *mut uint8_t)
                                                  ->
                                                      ())); // reset telemetry timing due to ad-hoc request
        crsfLastCycleTime =
            currentTimeUs; // reset telemetry timing due to ad-hoc request
        return
    }
    if deviceInfoReplyPending {
        let mut crsfPayloadBuf: sbuf_t =
            sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
        let mut dst: *mut sbuf_t = &mut crsfPayloadBuf;
        crsfInitializeFrame(dst);
        crsfFrameDeviceInfo(dst);
        crsfFinalize(dst);
        deviceInfoReplyPending = 0 as libc::c_int != 0;
        crsfLastCycleTime = currentTimeUs;
        return
    }
    if (*crsfDisplayPortScreen()).reset {
        (*crsfDisplayPortScreen()).reset = 0 as libc::c_int != 0;
        let mut crsfDisplayPortBuf: sbuf_t =
            sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
        let mut dst_0: *mut sbuf_t = &mut crsfDisplayPortBuf;
        crsfInitializeFrame(dst_0);
        crsfFrameDisplayPortClear(dst_0);
        crsfFinalize(dst_0);
        crsfLastCycleTime = currentTimeUs;
        return
    }
    let nextRow: libc::c_int = crsfDisplayPortNextRow();
    if nextRow >= 0 as libc::c_int {
        let mut crsfDisplayPortBuf_0: sbuf_t =
            sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
        let mut dst_1: *mut sbuf_t = &mut crsfDisplayPortBuf_0;
        crsfInitializeFrame(dst_1);
        crsfFrameDisplayPortRow(dst_1, nextRow as uint8_t);
        crsfFinalize(dst_1);
        (*crsfDisplayPortScreen()).pendingTransport[nextRow as usize] =
            0 as libc::c_int != 0;
        crsfLastCycleTime = currentTimeUs;
        return
    }
    // Actual telemetry data only needs to be sent at a low frequency, ie 10Hz
    // Spread out scheduled frames evenly so each frame is sent at the same frequency.
    if currentTimeUs >=
           crsfLastCycleTime.wrapping_add((100000 as libc::c_int /
                                               crsfScheduleCount as
                                                   libc::c_int) as
                                              libc::c_uint) {
        crsfLastCycleTime = currentTimeUs;
        processCrsf();
    };
}
#[no_mangle]
pub unsafe extern "C" fn getCrsfFrame(mut frame: *mut uint8_t,
                                      mut frameType: crsfFrameType_e)
 -> libc::c_int {
    let mut crsfFrameBuf: sbuf_t =
        sbuf_t{ptr: 0 as *mut uint8_t, end: 0 as *mut uint8_t,};
    let mut sbuf: *mut sbuf_t = &mut crsfFrameBuf;
    crsfInitializeFrame(sbuf);
    match frameType as libc::c_uint {
        8 => { crsfFrameBatterySensor(sbuf); }
        33 => { crsfFrameFlightMode(sbuf); }
        30 | _ => { crsfFrameAttitude(sbuf); }
    }
    let frameSize: libc::c_int = crsfFinalizeBuf(sbuf, frame);
    return frameSize;
}
