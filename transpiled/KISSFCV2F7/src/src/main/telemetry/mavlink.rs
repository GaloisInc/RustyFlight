use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn scaleRange(x: libc::c_int, srcFrom: libc::c_int, srcTo: libc::c_int,
                  destFrom: libc::c_int, destTo: libc::c_int) -> libc::c_int;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    static mut flightModeFlags: uint16_t;
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut mixerConfig_System: mixerConfig_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    fn failsafeIsActive() -> bool;
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    static baudRates: [uint32_t; 0];
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
    #[no_mangle]
    fn determinePortSharing(portConfig_0: *const serialPortConfig_t,
                            function: serialPortFunction_e) -> portSharing_e;
    //
// runtime
//
    #[no_mangle]
    fn openSerialPort(identifier: serialPortIdentifier_e,
                      function: serialPortFunction_e,
                      rxCallback: serialReceiveCallbackPtr,
                      rxCallbackData: *mut libc::c_void, baudrate: uint32_t,
                      mode: portMode_e, options: portOptions_e)
     -> *mut serialPort_t;
    #[no_mangle]
    fn closeSerialPort(serialPort: *mut serialPort_t);
    #[no_mangle]
    static mut GPS_home: [int32_t; 2];
    #[no_mangle]
    static mut gpsSol: gpsSolutionData_t;
    #[no_mangle]
    static mut rcData: [int16_t; 18];
    #[no_mangle]
    static mut rxRuntimeConfig: rxRuntimeConfig_t;
    #[no_mangle]
    fn getRssi() -> uint16_t;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn isAmperageConfigured() -> bool;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
    #[no_mangle]
    fn isBatteryVoltageConfigured() -> bool;
    #[no_mangle]
    fn getBatteryState() -> batteryState_e;
    #[no_mangle]
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    static mut telemetrySharedPort: *mut serialPort_t;
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig_0: *const serialPortConfig_t)
     -> bool;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __uint64_t = libc::c_ulong;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type uint64_t = __uint64_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
pub type timeMs_t = uint32_t;
pub type timeUs_t = uint32_t;
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
pub type C2RustUnnamed = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed = 2;
pub const ARMED: C2RustUnnamed = 1;
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
pub type mixerMode = libc::c_uint;
pub const MIXER_QUADX_1234: mixerMode = 26;
pub const MIXER_CUSTOM_TRI: mixerMode = 25;
pub const MIXER_CUSTOM_AIRPLANE: mixerMode = 24;
pub const MIXER_CUSTOM: mixerMode = 23;
pub const MIXER_ATAIL4: mixerMode = 22;
pub const MIXER_SINGLECOPTER: mixerMode = 21;
pub const MIXER_DUALCOPTER: mixerMode = 20;
pub const MIXER_RX_TO_SERVO: mixerMode = 19;
pub const MIXER_HEX6H: mixerMode = 18;
pub const MIXER_VTAIL4: mixerMode = 17;
pub const MIXER_HELI_90_DEG: mixerMode = 16;
pub const MIXER_HELI_120_CCPM: mixerMode = 15;
pub const MIXER_AIRPLANE: mixerMode = 14;
pub const MIXER_OCTOFLATX: mixerMode = 13;
pub const MIXER_OCTOFLATP: mixerMode = 12;
pub const MIXER_OCTOX8: mixerMode = 11;
pub const MIXER_HEX6X: mixerMode = 10;
pub const MIXER_Y4: mixerMode = 9;
pub const MIXER_FLYING_WING: mixerMode = 8;
pub const MIXER_HEX6: mixerMode = 7;
pub const MIXER_Y6: mixerMode = 6;
pub const MIXER_GIMBAL: mixerMode = 5;
pub const MIXER_BICOPTER: mixerMode = 4;
pub const MIXER_QUADX: mixerMode = 3;
pub const MIXER_QUADP: mixerMode = 2;
pub const MIXER_TRI: mixerMode = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mixerConfig_s {
    pub mixerMode: uint8_t,
    pub yaw_motors_reversed: bool,
    pub crashflip_motor_percent: uint8_t,
}
pub type mixerConfig_t = mixerConfig_s;
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
// 0 - 359
// 0 - 255
// 0 - 255
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
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
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
pub type portSharing_e = libc::c_uint;
pub const PORTSHARING_SHARED: portSharing_e = 2;
pub const PORTSHARING_NOT_SHARED: portSharing_e = 1;
pub const PORTSHARING_UNUSED: portSharing_e = 0;
pub type serialPortFunction_e = libc::c_uint;
pub const FUNCTION_LIDAR_TF: serialPortFunction_e = 32768;
pub const FUNCTION_RCDEVICE: serialPortFunction_e = 16384;
pub const FUNCTION_VTX_TRAMP: serialPortFunction_e = 8192;
pub const FUNCTION_TELEMETRY_IBUS: serialPortFunction_e = 4096;
pub const FUNCTION_VTX_SMARTAUDIO: serialPortFunction_e = 2048;
pub const FUNCTION_ESC_SENSOR: serialPortFunction_e = 1024;
pub const FUNCTION_TELEMETRY_MAVLINK: serialPortFunction_e = 512;
pub const FUNCTION_BLACKBOX: serialPortFunction_e = 128;
pub const FUNCTION_RX_SERIAL: serialPortFunction_e = 64;
pub const FUNCTION_TELEMETRY_SMARTPORT: serialPortFunction_e = 32;
pub const FUNCTION_TELEMETRY_LTM: serialPortFunction_e = 16;
pub const FUNCTION_TELEMETRY_HOTT: serialPortFunction_e = 8;
pub const FUNCTION_TELEMETRY_FRSKY_HUB: serialPortFunction_e = 4;
pub const FUNCTION_GPS: serialPortFunction_e = 2;
pub const FUNCTION_MSP: serialPortFunction_e = 1;
pub const FUNCTION_NONE: serialPortFunction_e = 0;
pub type baudRate_e = libc::c_uint;
pub const BAUD_2470000: baudRate_e = 15;
pub const BAUD_2000000: baudRate_e = 14;
pub const BAUD_1500000: baudRate_e = 13;
pub const BAUD_1000000: baudRate_e = 12;
pub const BAUD_921600: baudRate_e = 11;
pub const BAUD_500000: baudRate_e = 10;
pub const BAUD_460800: baudRate_e = 9;
pub const BAUD_400000: baudRate_e = 8;
pub const BAUD_250000: baudRate_e = 7;
pub const BAUD_230400: baudRate_e = 6;
pub const BAUD_115200: baudRate_e = 5;
pub const BAUD_57600: baudRate_e = 4;
pub const BAUD_38400: baudRate_e = 3;
pub const BAUD_19200: baudRate_e = 2;
pub const BAUD_9600: baudRate_e = 1;
pub const BAUD_AUTO: baudRate_e = 0;
pub type serialPortIdentifier_e = libc::c_int;
pub const SERIAL_PORT_SOFTSERIAL2: serialPortIdentifier_e = 31;
pub const SERIAL_PORT_SOFTSERIAL1: serialPortIdentifier_e = 30;
pub const SERIAL_PORT_USB_VCP: serialPortIdentifier_e = 20;
pub const SERIAL_PORT_USART8: serialPortIdentifier_e = 7;
pub const SERIAL_PORT_USART7: serialPortIdentifier_e = 6;
pub const SERIAL_PORT_USART6: serialPortIdentifier_e = 5;
pub const SERIAL_PORT_UART5: serialPortIdentifier_e = 4;
pub const SERIAL_PORT_UART4: serialPortIdentifier_e = 3;
pub const SERIAL_PORT_USART3: serialPortIdentifier_e = 2;
pub const SERIAL_PORT_USART2: serialPortIdentifier_e = 1;
pub const SERIAL_PORT_USART1: serialPortIdentifier_e = 0;
pub const SERIAL_PORT_NONE: serialPortIdentifier_e = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortConfig_s {
    pub functionMask: uint16_t,
    pub identifier: serialPortIdentifier_e,
    pub msp_baudrateIndex: uint8_t,
    pub gps_baudrateIndex: uint8_t,
    pub blackbox_baudrateIndex: uint8_t,
    pub telemetry_baudrateIndex: uint8_t,
}
pub type serialPortConfig_t = serialPortConfig_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxRuntimeConfig_s {
    pub channelCount: uint8_t,
    pub rxRefreshRate: uint16_t,
    pub rcReadRawFn: rcReadRawDataFnPtr,
    pub rcFrameStatusFn: rcFrameStatusFnPtr,
    pub rcProcessFrameFn: rcProcessFrameFnPtr,
    pub channelData: *mut uint16_t,
    pub frameData: *mut libc::c_void,
}
pub type rcProcessFrameFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s) -> bool>;
// latitude * 1e+7
// longitude * 1e+7
// altitude in 0.01m
// speed in 0.1m/s
// degrees * 10
// generic HDOP value (*100)
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_3 = 64;
pub const SENSOR_GPS: C2RustUnnamed_3 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_3 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_3 = 16;
pub const SENSOR_MAG: C2RustUnnamed_3 = 8;
pub const SENSOR_BARO: C2RustUnnamed_3 = 4;
pub const SENSOR_ACC: C2RustUnnamed_3 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_3 = 1;
pub type batteryState_e = libc::c_uint;
pub const BATTERY_INIT: batteryState_e = 4;
pub const BATTERY_NOT_PRESENT: batteryState_e = 3;
pub const BATTERY_CRITICAL: batteryState_e = 2;
pub const BATTERY_WARNING: batteryState_e = 1;
pub const BATTERY_OK: batteryState_e = 0;
pub type frskyGpsCoordFormat_e = libc::c_uint;
pub const FRSKY_FORMAT_NMEA: frskyGpsCoordFormat_e = 1;
pub const FRSKY_FORMAT_DMS: frskyGpsCoordFormat_e = 0;
pub type frskyUnit_e = libc::c_uint;
pub const FRSKY_UNIT_IMPERIALS: frskyUnit_e = 1;
pub const FRSKY_UNIT_METRICS: frskyUnit_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct telemetryConfig_s {
    pub gpsNoFixLatitude: int16_t,
    pub gpsNoFixLongitude: int16_t,
    pub telemetry_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub frsky_coordinate_format: frskyGpsCoordFormat_e,
    pub frsky_unit: frskyUnit_e,
    pub frsky_vfas_precision: uint8_t,
    pub hottAlarmSoundInterval: uint8_t,
    pub pidValuesAsTelemetry: uint8_t,
    pub report_cell_voltage: uint8_t,
    pub flysky_sensors: [uint8_t; 15],
    pub smartport_use_extra_sensors: uint8_t,
}
pub type telemetryConfig_t = telemetryConfig_s;
pub type mavlink_message_t = __mavlink_message;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct __mavlink_message {
    pub checksum: uint16_t,
    pub magic: uint8_t,
    pub len: uint8_t,
    pub seq: uint8_t,
    pub sysid: uint8_t,
    pub compid: uint8_t,
    pub msgid: uint8_t,
    pub payload64: [uint64_t; 33],
}
pub const MAV_MODE_FLAG_MANUAL_INPUT_ENABLED: MAV_MODE_FLAG = 64;
pub const MAV_AUTOPILOT_GENERIC: MAV_AUTOPILOT = 0;
pub const MAVLINK_COMM_0: C2RustUnnamed_4 = 0;
// number of RC channels as reported by current input driver
// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
// Macro to define packed structures
// it is possible to override this, but be careful!
// /< Maximum payload length
// /< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
// /< Length of all header bytes, including core and checksum
// /< Maximum packet length
/* full fledged 32bit++ OS */
/* *
 * Old-style 4 byte param union
 *
 * This struct is the data format to be used when sending
 * parameters. The parameter should be copied to the native
 * type (without type conversion)
 * and re-instanted on the receiving side using the
 * native type as well.
 */
/* *
 * New-style 8 byte param union
 * mavlink_param_union_double_t will be 8 bytes long, and treated as needing 8 byte alignment for the purposes of MAVLink 1.0 field ordering.
 * The mavlink_param_union_double_t will be treated as a little-endian structure.
 *
 * If is_double is 1 then the type is a double, and the remaining 63 bits are the double, with the lowest bit of the mantissa zero.
 * The intention is that by replacing the is_double bit with 0 the type can be directly used as a double (as the is_double bit corresponds to the
 * lowest mantissa bit of a double). If is_double is 0 then mavlink_type gives the type in the union.
 * The mavlink_types.h header will also need to have shifts/masks to define the bit boundaries in the above,
 * as bitfield ordering isn’t consistent between platforms. The above is intended to be for gcc on x86,
 * which should be the same as gcc on little-endian arm. When using shifts/masks the value will be treated as a 64 bit unsigned number,
 * and the bits pulled out using the shifts/masks.
*/
/* *
 * This structure is required to make the mavlink_send_xxx convenience functions
 * work, as it tells the library what the current system and component ID are.
 */
// /< Used by the MAVLink message_xx_send() convenience function
// /< Used by the MAVLink message_xx_send() convenience function
// /< sent at end of packet
// /< protocol magic marker
// /< Length of payload
// /< Sequence of packet
// /< ID of message sender system/aircraft
// /< ID of the message sender component
// /< ID of message in payload
// /< Length of extended payload if any
// name of this field
// printing format hint, or NULL
// type of this field
// if non-zero, field is an array
// offset of each field in the payload
// offset in a C structure
// note that in this structure the order of fields is the order
// in the XML file, not necessary the wire order
// name of the message
// how many fields in this message
// field information
// checksum is immediately after the payload bytes
/*
 * applications can set MAVLINK_COMM_NUM_BUFFERS to the maximum number
 * of buffers they will use. If more are used, then the result will be
 * a stack overrun
 */
// /< The state machine for the comm parser
pub type mavlink_status_t = __mavlink_status;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_status {
    pub msg_received: uint8_t,
    pub buffer_overrun: uint8_t,
    pub parse_error: uint8_t,
    pub parse_state: mavlink_parse_state_t,
    pub packet_idx: uint8_t,
    pub current_rx_seq: uint8_t,
    pub current_tx_seq: uint8_t,
    pub packet_rx_success_count: uint16_t,
    pub packet_rx_drop_count: uint16_t,
}
pub type mavlink_parse_state_t = libc::c_uint;
pub const MAVLINK_PARSE_STATE_GOT_CRC1: mavlink_parse_state_t = 9;
pub const MAVLINK_PARSE_STATE_GOT_PAYLOAD: mavlink_parse_state_t = 8;
pub const MAVLINK_PARSE_STATE_GOT_MSGID: mavlink_parse_state_t = 7;
pub const MAVLINK_PARSE_STATE_GOT_COMPID: mavlink_parse_state_t = 6;
pub const MAVLINK_PARSE_STATE_GOT_SYSID: mavlink_parse_state_t = 5;
pub const MAVLINK_PARSE_STATE_GOT_LENGTH: mavlink_parse_state_t = 4;
pub const MAVLINK_PARSE_STATE_GOT_SEQ: mavlink_parse_state_t = 3;
pub const MAVLINK_PARSE_STATE_GOT_STX: mavlink_parse_state_t = 2;
pub const MAVLINK_PARSE_STATE_IDLE: mavlink_parse_state_t = 1;
pub const MAVLINK_PARSE_STATE_UNINIT: mavlink_parse_state_t = 0;
pub type mavlink_heartbeat_t = __mavlink_heartbeat_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_heartbeat_t {
    pub custom_mode: uint32_t,
    pub type_0: uint8_t,
    pub autopilot: uint8_t,
    pub base_mode: uint8_t,
    pub system_status: uint8_t,
    pub mavlink_version: uint8_t,
}
pub const MAV_STATE_STANDBY: MAV_STATE = 3;
pub const MAV_STATE_ACTIVE: MAV_STATE = 4;
pub const MAV_STATE_CRITICAL: MAV_STATE = 5;
pub const MAV_MODE_FLAG_STABILIZE_ENABLED: MAV_MODE_FLAG = 16;
pub const MAV_TYPE_GENERIC: MAV_TYPE = 0;
pub const MAV_TYPE_HELICOPTER: MAV_TYPE = 4;
pub const MAV_TYPE_FIXED_WING: MAV_TYPE = 1;
pub const MAV_TYPE_OCTOROTOR: MAV_TYPE = 14;
pub const MAV_TYPE_HEXAROTOR: MAV_TYPE = 13;
pub const MAV_TYPE_QUADROTOR: MAV_TYPE = 2;
pub const MAV_TYPE_TRICOPTER: MAV_TYPE = 15;
pub const MAV_MODE_FLAG_SAFETY_ARMED: MAV_MODE_FLAG = 128;
pub type mavlink_vfr_hud_t = __mavlink_vfr_hud_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_vfr_hud_t {
    pub airspeed: libc::c_float,
    pub groundspeed: libc::c_float,
    pub alt: libc::c_float,
    pub climb: libc::c_float,
    pub heading: int16_t,
    pub throttle: uint16_t,
}
// /< Number of received messages
// /< Number of buffer overruns
// /< Number of parse errors
// /< Parsing state machine
// /< Index in current packet
// /< Sequence number of last packet received
// /< Sequence number of last packet sent
// /< Received packets
// /< Number of packet drops
/* * @file
 *	@brief MAVLink comm protocol generated from common.xml
 *	@see http://mavlink.org
 */
// MESSAGE LENGTHS AND CRCS
// ENUM DEFINITIONS
/* * @brief Micro air vehicle / autopilot classes. This identifies the individual model. */
/* Generic autopilot, full support for everything | */
/* PIXHAWK autopilot, http://pixhawk.ethz.ch | */
/* SLUGS autopilot, http://slugsuav.soe.ucsc.edu | */
/* ArduPilotMega / ArduCopter, http://diydrones.com | */
/* OpenPilot, http://openpilot.org | */
/* Generic autopilot only supporting simple waypoints | */
/* Generic autopilot supporting waypoints and other simple navigation commands | */
/* Generic autopilot supporting the full mission command set | */
/* No valid autopilot, e.g. a GCS or other MAVLink component | */
/* PPZ UAV - http://nongnu.org/paparazzi | */
/* UAV Dev Board | */
/* FlexiPilot | */
/* PX4 Autopilot - http://pixhawk.ethz.ch/px4/ | */
/* SMACCMPilot - http://smaccmpilot.org | */
/* AutoQuad -- http://autoquad.org | */
/* Armazila -- http://armazila.com | */
/* Aerob -- http://aerob.ru | */
/* ASLUAV autopilot -- http://www.asl.ethz.ch | */
/*  | */
/* * @brief  */
/* Generic micro air vehicle. | */
/* Fixed wing aircraft. | */
/* Quadrotor | */
/* Coaxial helicopter | */
/* Normal helicopter with tail rotor. | */
/* Ground installation | */
/* Operator control unit / ground control station | */
/* Airship, controlled | */
/* Free balloon, uncontrolled | */
/* Rocket | */
/* Ground rover | */
/* Surface vessel, boat, ship | */
/* Submarine | */
/* Hexarotor | */
/* Octorotor | */
/* Octorotor | */
/* Flapping wing | */
/* Flapping wing | */
/* Onboard companion controller | */
/* Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | */
/* Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | */
/* VTOL reserved 1 | */
/* VTOL reserved 2 | */
/* VTOL reserved 3 | */
/* VTOL reserved 4 | */
/* VTOL reserved 5 | */
/* Onboard gimbal | */
/*  | */
/* * @brief These flags encode the MAV mode. */
/* 0b00000001 Reserved for future use. | */
/* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. | */
/* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. | */
/* 0b00001000 guided mode enabled, system flies MISSIONs / mission items. | */
/* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. | */
/* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. | */
/* 0b01000000 remote control input is enabled. | */
/* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. | */
/*  | */
/* * @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
/* Eighth bit: 00000001 | */
/* Seventh bit: 00000010 | */
/* Sixt bit:   00000100 | */
/* Fifth bit:  00001000 | */
/* Fourth bit: 00010000 | */
/* Third bit:  00100000 | */
/* Second bit: 01000000 | */
/* First bit:  10000000 | */
/*  | */
/* * @brief Override command, pauses current mission execution and moves immediately to a position */
/* Hold at the current position. | */
/* Continue with the next item in mission execution. | */
/* Hold at the current position of the system | */
/* Hold at the position specified in the parameters of the DO_HOLD action | */
/*  | */
/* * @brief These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override. */
/* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
/* System is allowed to be active, under manual (RC) control, no stabilization | */
/* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
/* System is allowed to be active, under assisted RC control. | */
/* System is allowed to be active, under autonomous control, manual setpoint | */
/* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
/* System is allowed to be active, under manual (RC) control, no stabilization | */
/* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
/* System is allowed to be active, under assisted RC control. | */
/* System is allowed to be active, under autonomous control, manual setpoint | */
/* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs) | */
/*  | */
/* * @brief  */
/* Uninitialized system, state is unknown. | */
/* System is booting up. | */
/* System is calibrating and not flight-ready. | */
/* System is grounded and on standby. It can be launched any time. | */
/* System is active and might be already airborne. Motors are engaged. | */
/* System is in a non-normal flight mode. It can however still navigate. | */
/* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
/* System just initialized its power-down sequence, will shut down now. | */
/*  | */
/* * @brief  */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/* * @brief These encode the sensors whose status is sent as part of the SYS_STATUS message. */
/* 0x01 3D gyro | */
/* 0x02 3D accelerometer | */
/* 0x04 3D magnetometer | */
/* 0x08 absolute pressure | */
/* 0x10 differential pressure | */
/* 0x20 GPS | */
/* 0x40 optical flow | */
/* 0x80 computer vision position | */
/* 0x100 laser based position | */
/* 0x200 external ground truth (Vicon or Leica) | */
/* 0x400 3D angular rate control | */
/* 0x800 attitude stabilization | */
/* 0x1000 yaw position | */
/* 0x2000 z/altitude control | */
/* 0x4000 x/y position control | */
/* 0x8000 motor outputs / control | */
/* 0x10000 rc receiver | */
/* 0x20000 2nd 3D gyro | */
/* 0x40000 2nd 3D accelerometer | */
/* 0x80000 2nd 3D magnetometer | */
/* 0x100000 geofence | */
/* 0x200000 AHRS subsystem health | */
/* 0x400000 Terrain subsystem health | */
/*  | */
/* * @brief  */
/* Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL) | */
/* Local coordinate frame, Z-up (x: north, y: east, z: down). | */
/* NOT a coordinate frame, indicates a mission command. | */
/* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location. | */
/* Local coordinate frame, Z-down (x: east, y: north, z: up) | */
/* Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | */
/* Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | */
/* Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | */
/* Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | */
/* Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | */
/* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
/* Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | */
/*  | */
/* * @brief  */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/*  | */
/* * @brief  */
/* Disable fenced mode | */
/* Switched to guided mode to return point (fence point 0) | */
/* Report fence breach, but don't take action | */
/* Switched to guided mode to return point (fence point 0) with manual throttle control | */
/*  | */
/* * @brief  */
/* No last fence breach | */
/* Breached minimum altitude | */
/* Breached maximum altitude | */
/* Breached fence boundary | */
/*  | */
/* * @brief Enumeration of possible mount operation modes */
/* Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization | */
/* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
/* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
/* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
/* Load neutral position and start to point to Lat,Lon,Alt | */
/*  | */
/* * @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. */
/* Navigate to MISSION. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude|  */
/* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
/* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
/* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
/* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
/* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
/* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Empty| Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  */
/* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
/* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
/* Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
/* hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
/* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
/* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Set system mode. |Mode, as defined by ENUM MAV_MODE| Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty| Empty|  */
/* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
/* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
/* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
/* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
/* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
/* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
/* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
/* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
/* Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  */
/* Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  */
/* Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
/* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
/* Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  */
/* Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  */
/* Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  */
/* Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|  */
/* Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Mission command to enable the geofence |enable? (0=disable, 1=enable)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  */
/* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
/* set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  */
/* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
/* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Accelerometer calibration: 0: no, 1: yes| Compass/Motor interference calibration: 0: no, 1: yes| Empty|  */
/* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
/* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
/* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
/* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
/* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
/* Arms / Disarms a component |1 to arm, 0 to disarm|  */
/* Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  */
/* Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  */
/* Start image capture sequence |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  */
/* Stop image capture sequence |Reserved| Reserved|  */
/* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start)| Shutter integration time (in ms)| Reserved|  */
/* Starts video capture |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  */
/* Stop the current video capture |Reserved| Reserved|  */
/* Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  */
/* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  */
/* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
/*  | */
/* * @brief Data stream IDs. A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages. */
pub type MAV_DATA_STREAM = libc::c_uint;
/*  | */
/* Dependent on the autopilot | */
pub const MAV_DATA_STREAM_ENUM_END: MAV_DATA_STREAM = 13;
/* Dependent on the autopilot | */
pub const MAV_DATA_STREAM_EXTRA3: MAV_DATA_STREAM = 12;
/* Dependent on the autopilot | */
pub const MAV_DATA_STREAM_EXTRA2: MAV_DATA_STREAM = 11;
/* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */
pub const MAV_DATA_STREAM_EXTRA1: MAV_DATA_STREAM = 10;
/* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
pub const MAV_DATA_STREAM_POSITION: MAV_DATA_STREAM = 6;
/* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
pub const MAV_DATA_STREAM_RAW_CONTROLLER: MAV_DATA_STREAM = 4;
/* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
pub const MAV_DATA_STREAM_RC_CHANNELS: MAV_DATA_STREAM = 3;
/* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
pub const MAV_DATA_STREAM_EXTENDED_STATUS: MAV_DATA_STREAM = 2;
/* Enable all data streams | */
pub const MAV_DATA_STREAM_RAW_SENSORS: MAV_DATA_STREAM = 1;
pub const MAV_DATA_STREAM_ALL: MAV_DATA_STREAM = 0;
pub type mavlink_attitude_t = __mavlink_attitude_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_attitude_t {
    pub time_boot_ms: uint32_t,
    pub roll: libc::c_float,
    pub pitch: libc::c_float,
    pub yaw: libc::c_float,
    pub rollspeed: libc::c_float,
    pub pitchspeed: libc::c_float,
    pub yawspeed: libc::c_float,
}
pub type mavlink_gps_global_origin_t = __mavlink_gps_global_origin_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_gps_global_origin_t {
    pub latitude: int32_t,
    pub longitude: int32_t,
    pub altitude: int32_t,
}
pub type mavlink_global_position_int_t = __mavlink_global_position_int_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_global_position_int_t {
    pub time_boot_ms: uint32_t,
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
    pub relative_alt: int32_t,
    pub vx: int16_t,
    pub vy: int16_t,
    pub vz: int16_t,
    pub hdg: uint16_t,
}
pub type mavlink_gps_raw_int_t = __mavlink_gps_raw_int_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_gps_raw_int_t {
    pub time_usec: uint64_t,
    pub lat: int32_t,
    pub lon: int32_t,
    pub alt: int32_t,
    pub eph: uint16_t,
    pub epv: uint16_t,
    pub vel: uint16_t,
    pub cog: uint16_t,
    pub fix_type: uint8_t,
    pub satellites_visible: uint8_t,
}
pub type mavlink_rc_channels_raw_t = __mavlink_rc_channels_raw_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_rc_channels_raw_t {
    pub time_boot_ms: uint32_t,
    pub chan1_raw: uint16_t,
    pub chan2_raw: uint16_t,
    pub chan3_raw: uint16_t,
    pub chan4_raw: uint16_t,
    pub chan5_raw: uint16_t,
    pub chan6_raw: uint16_t,
    pub chan7_raw: uint16_t,
    pub chan8_raw: uint16_t,
    pub port: uint8_t,
    pub rssi: uint8_t,
}
pub type mavlink_sys_status_t = __mavlink_sys_status_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct __mavlink_sys_status_t {
    pub onboard_control_sensors_present: uint32_t,
    pub onboard_control_sensors_enabled: uint32_t,
    pub onboard_control_sensors_health: uint32_t,
    pub load: uint16_t,
    pub voltage_battery: uint16_t,
    pub current_battery: int16_t,
    pub drop_rate_comm: uint16_t,
    pub errors_comm: uint16_t,
    pub errors_count1: uint16_t,
    pub errors_count2: uint16_t,
    pub errors_count3: uint16_t,
    pub errors_count4: uint16_t,
    pub battery_remaining: int8_t,
}
pub type C2RustUnnamed_4 = libc::c_uint;
pub const MAVLINK_COMM_3: C2RustUnnamed_4 = 3;
pub const MAVLINK_COMM_2: C2RustUnnamed_4 = 2;
pub const MAVLINK_COMM_1: C2RustUnnamed_4 = 1;
pub type MAV_AUTOPILOT = libc::c_uint;
pub const MAV_AUTOPILOT_ENUM_END: MAV_AUTOPILOT = 18;
pub const MAV_AUTOPILOT_ASLUAV: MAV_AUTOPILOT = 17;
pub const MAV_AUTOPILOT_AEROB: MAV_AUTOPILOT = 16;
pub const MAV_AUTOPILOT_ARMAZILA: MAV_AUTOPILOT = 15;
pub const MAV_AUTOPILOT_AUTOQUAD: MAV_AUTOPILOT = 14;
pub const MAV_AUTOPILOT_SMACCMPILOT: MAV_AUTOPILOT = 13;
pub const MAV_AUTOPILOT_PX4: MAV_AUTOPILOT = 12;
pub const MAV_AUTOPILOT_FP: MAV_AUTOPILOT = 11;
pub const MAV_AUTOPILOT_UDB: MAV_AUTOPILOT = 10;
pub const MAV_AUTOPILOT_PPZ: MAV_AUTOPILOT = 9;
pub const MAV_AUTOPILOT_INVALID: MAV_AUTOPILOT = 8;
pub const MAV_AUTOPILOT_GENERIC_MISSION_FULL: MAV_AUTOPILOT = 7;
pub const MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY:
          MAV_AUTOPILOT =
    6;
pub const MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY: MAV_AUTOPILOT = 5;
pub const MAV_AUTOPILOT_OPENPILOT: MAV_AUTOPILOT = 4;
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: MAV_AUTOPILOT = 3;
pub const MAV_AUTOPILOT_SLUGS: MAV_AUTOPILOT = 2;
pub const MAV_AUTOPILOT_PIXHAWK: MAV_AUTOPILOT = 1;
pub type MAV_TYPE = libc::c_uint;
pub const MAV_TYPE_ENUM_END: MAV_TYPE = 27;
pub const MAV_TYPE_GIMBAL: MAV_TYPE = 26;
pub const MAV_TYPE_VTOL_RESERVED5: MAV_TYPE = 25;
pub const MAV_TYPE_VTOL_RESERVED4: MAV_TYPE = 24;
pub const MAV_TYPE_VTOL_RESERVED3: MAV_TYPE = 23;
pub const MAV_TYPE_VTOL_RESERVED2: MAV_TYPE = 22;
pub const MAV_TYPE_VTOL_RESERVED1: MAV_TYPE = 21;
pub const MAV_TYPE_VTOL_QUADROTOR: MAV_TYPE = 20;
pub const MAV_TYPE_VTOL_DUOROTOR: MAV_TYPE = 19;
pub const MAV_TYPE_ONBOARD_CONTROLLER: MAV_TYPE = 18;
pub const MAV_TYPE_KITE: MAV_TYPE = 17;
pub const MAV_TYPE_FLAPPING_WING: MAV_TYPE = 16;
pub const MAV_TYPE_SUBMARINE: MAV_TYPE = 12;
pub const MAV_TYPE_SURFACE_BOAT: MAV_TYPE = 11;
pub const MAV_TYPE_GROUND_ROVER: MAV_TYPE = 10;
pub const MAV_TYPE_ROCKET: MAV_TYPE = 9;
pub const MAV_TYPE_FREE_BALLOON: MAV_TYPE = 8;
pub const MAV_TYPE_AIRSHIP: MAV_TYPE = 7;
pub const MAV_TYPE_GCS: MAV_TYPE = 6;
pub const MAV_TYPE_ANTENNA_TRACKER: MAV_TYPE = 5;
pub const MAV_TYPE_COAXIAL: MAV_TYPE = 3;
pub type MAV_MODE_FLAG = libc::c_uint;
pub const MAV_MODE_FLAG_ENUM_END: MAV_MODE_FLAG = 129;
pub const MAV_MODE_FLAG_HIL_ENABLED: MAV_MODE_FLAG = 32;
pub const MAV_MODE_FLAG_GUIDED_ENABLED: MAV_MODE_FLAG = 8;
pub const MAV_MODE_FLAG_AUTO_ENABLED: MAV_MODE_FLAG = 4;
pub const MAV_MODE_FLAG_TEST_ENABLED: MAV_MODE_FLAG = 2;
pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: MAV_MODE_FLAG = 1;
pub type MAV_STATE = libc::c_uint;
pub const MAV_STATE_ENUM_END: MAV_STATE = 8;
pub const MAV_STATE_POWEROFF: MAV_STATE = 7;
pub const MAV_STATE_EMERGENCY: MAV_STATE = 6;
pub const MAV_STATE_CALIBRATING: MAV_STATE = 2;
pub const MAV_STATE_BOOT: MAV_STATE = 1;
pub const MAV_STATE_UNINIT: MAV_STATE = 0;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn mixerConfig() -> *const mixerConfig_t {
    return &mut mixerConfig_System;
}
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
}
/*
 * Internal function to give access to the channel status for each channel
 */
/*
 * Internal function to give access to the channel buffer for each channel
 */
/* *
 * @brief Reset the status of a channel.
 */
/* *
 * @brief Finalize a MAVLink message with channel assignment
 *
 * This function calculates the checksum and sets length and aircraft id correctly.
 * It assumes that the message id and the payload are already correctly set. This function
 * can also be used if the message header has already been written before (as in mavlink_msg_xxx_pack
 * instead of mavlink_msg_xxx_pack_headerless), it just introduces little extra overhead.
 *
 * @param msg Message to finalize
 * @param system_id Id of the sending (this) system, 1-127
 * @param length Message length
 */
// This code part is the same for all messages;
// One sequence number per channel
/* *
 * @brief Finalize a MAVLink message with MAVLINK_COMM_0 as default channel
 */
#[inline]
unsafe extern "C" fn mavlink_finalize_message(mut msg: *mut mavlink_message_t,
                                              mut system_id: uint8_t,
                                              mut component_id: uint8_t,
                                              mut length: uint8_t,
                                              mut crc_extra: uint8_t)
 -> uint16_t {
    return mavlink_finalize_message_chan(msg, system_id, component_id,
                                         MAVLINK_COMM_0 as libc::c_int as
                                             uint8_t, length, crc_extra);
}
#[inline]
unsafe extern "C" fn mavlink_finalize_message_chan(mut msg:
                                                       *mut mavlink_message_t,
                                                   mut system_id: uint8_t,
                                                   mut component_id: uint8_t,
                                                   mut chan: uint8_t,
                                                   mut length: uint8_t,
                                                   mut crc_extra: uint8_t)
 -> uint16_t {
    (*msg).magic = 254 as libc::c_int as uint8_t;
    (*msg).len = length;
    (*msg).sysid = system_id;
    (*msg).compid = component_id;
    (*msg).seq = (*mavlink_get_channel_status(chan)).current_tx_seq;
    (*mavlink_get_channel_status(chan)).current_tx_seq =
        ((*mavlink_get_channel_status(chan)).current_tx_seq as libc::c_int +
             1 as libc::c_int) as uint8_t;
    (*msg).checksum =
        crc_calculate((msg as
                           *const uint8_t).offset(3 as libc::c_int as isize),
                      5 as libc::c_int as uint16_t);
    crc_accumulate_buffer(&mut (*msg).checksum,
                          &mut *(*msg).payload64.as_mut_ptr().offset(0 as
                                                                         libc::c_int
                                                                         as
                                                                         isize)
                              as *mut uint64_t as *const libc::c_char,
                          (*msg).len as uint16_t);
    crc_accumulate(crc_extra, &mut (*msg).checksum);
    *(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as isize) as
          *mut uint64_t as *mut libc::c_char as
          *mut uint8_t).offset((*msg).len as libc::c_int as isize) =
        ((*msg).checksum as libc::c_int & 0xff as libc::c_int) as uint8_t;
    *(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as isize) as
          *mut uint64_t as *mut libc::c_char as
          *mut uint8_t).offset(((*msg).len as libc::c_int +
                                    1 as libc::c_int as uint16_t as
                                        libc::c_int) as isize) =
        ((*msg).checksum as libc::c_int >> 8 as libc::c_int) as uint8_t;
    return (length as libc::c_int +
                (5 as libc::c_int + 1 as libc::c_int + 2 as libc::c_int)) as
               uint16_t;
}
// Visual Studio versions before 2010 don't have stdint.h, so we just error out.
/* *
 *
 *  CALCULATE THE CHECKSUM
 *
 */
/* *
 * @brief Accumulate the X.25 CRC by adding one char at a time.
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new char to hash
 * @param crcAccum the already accumulated checksum
 **/
#[inline]
unsafe extern "C" fn crc_accumulate(mut data: uint8_t,
                                    mut crcAccum: *mut uint16_t) {
    /*Accumulate one byte of data into the CRC*/
    let mut tmp: uint8_t = 0;
    tmp =
        (data as libc::c_int ^
             (*crcAccum as libc::c_int & 0xff as libc::c_int) as uint8_t as
                 libc::c_int) as uint8_t;
    tmp =
        (tmp as libc::c_int ^ (tmp as libc::c_int) << 4 as libc::c_int) as
            uint8_t;
    *crcAccum =
        (*crcAccum as libc::c_int >> 8 as libc::c_int ^
             (tmp as libc::c_int) << 8 as libc::c_int ^
             (tmp as libc::c_int) << 3 as libc::c_int ^
             tmp as libc::c_int >> 4 as libc::c_int) as uint16_t;
}
/* *
 * @brief Initiliaze the buffer for the X.25 CRC
 *
 * @param crcAccum the 16 bit X.25 CRC
 */
/* *
 * @brief Calculates the X.25 checksum on a byte buffer
 *
 * @param  pBuffer buffer containing the byte array to hash
 * @param  length  length of the byte array
 * @return the checksum over the buffer bytes
 **/
/* *
 * @brief Accumulate the X.25 CRC by adding an array of bytes
 *
 * The checksum function adds the hash of one char at a time to the
 * 16 bit checksum (uint16_t).
 *
 * @param data new bytes to hash
 * @param crcAccum the already accumulated checksum
 **/
#[inline]
unsafe extern "C" fn crc_accumulate_buffer(mut crcAccum: *mut uint16_t,
                                           mut pBuffer: *const libc::c_char,
                                           mut length: uint16_t) {
    let mut p: *const uint8_t = pBuffer as *const uint8_t;
    loop  {
        let fresh0 = length;
        length = length.wrapping_sub(1);
        if !(fresh0 != 0) { break ; }
        let fresh1 = p;
        p = p.offset(1);
        crc_accumulate(*fresh1, crcAccum);
    };
}
#[inline]
unsafe extern "C" fn crc_calculate(mut pBuffer: *const uint8_t,
                                   mut length: uint16_t) -> uint16_t {
    let mut crcTmp: uint16_t = 0;
    crc_init(&mut crcTmp);
    loop  {
        let fresh2 = length;
        length = length.wrapping_sub(1);
        if !(fresh2 != 0) { break ; }
        let fresh3 = pBuffer;
        pBuffer = pBuffer.offset(1);
        crc_accumulate(*fresh3, &mut crcTmp);
    }
    return crcTmp;
}
#[inline]
unsafe extern "C" fn crc_init(mut crcAccum: *mut uint16_t) {
    *crcAccum = 0xffff as libc::c_int as uint16_t;
}
#[inline]
unsafe extern "C" fn mavlink_get_channel_status(mut chan: uint8_t)
 -> *mut mavlink_status_t {
    static mut m_mavlink_status: [mavlink_status_t; 16] =
        [mavlink_status_t{msg_received: 0,
                          buffer_overrun: 0,
                          parse_error: 0,
                          parse_state: MAVLINK_PARSE_STATE_UNINIT,
                          packet_idx: 0,
                          current_rx_seq: 0,
                          current_tx_seq: 0,
                          packet_rx_success_count: 0,
                          packet_rx_drop_count: 0,}; 16];
    return &mut *m_mavlink_status.as_mut_ptr().offset(chan as isize) as
               *mut mavlink_status_t;
}
// MESSAGE VFR_HUD PACKING
// /< Current airspeed in m/s
// /< Current ground speed in m/s
// /< Current altitude (MSL), in meters
// /< Current climb rate in meters/second
// /< Current heading in degrees, in compass units (0..360, 0=north)
// /< Current throttle setting in integer percent, 0 to 100
/* *
 * @brief Pack a vfr_hud message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param airspeed Current airspeed in m/s
 * @param groundspeed Current ground speed in m/s
 * @param heading Current heading in degrees, in compass units (0..360, 0=north)
 * @param throttle Current throttle setting in integer percent, 0 to 100
 * @param alt Current altitude (MSL), in meters
 * @param climb Current climb rate in meters/second
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_vfr_hud_pack(mut system_id: uint8_t,
                                              mut component_id: uint8_t,
                                              mut msg: *mut mavlink_message_t,
                                              mut airspeed: libc::c_float,
                                              mut groundspeed: libc::c_float,
                                              mut heading: int16_t,
                                              mut throttle: uint16_t,
                                              mut alt: libc::c_float,
                                              mut climb: libc::c_float)
 -> uint16_t {
    let mut packet: mavlink_vfr_hud_t =
        mavlink_vfr_hud_t{airspeed: 0.,
                          groundspeed: 0.,
                          alt: 0.,
                          climb: 0.,
                          heading: 0,
                          throttle: 0,};
    packet.airspeed = airspeed;
    packet.groundspeed = groundspeed;
    packet.alt = alt;
    packet.climb = climb;
    packet.heading = heading;
    packet.throttle = throttle;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_vfr_hud_t as *const libc::c_void,
           20 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 74 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    20 as libc::c_int as uint8_t,
                                    20 as libc::c_int as uint8_t);
}
// MESSAGE ATTITUDE PACKING
// /< Timestamp (milliseconds since system boot)
// /< Roll angle (rad, -pi..+pi)
// /< Pitch angle (rad, -pi..+pi)
// /< Yaw angle (rad, -pi..+pi)
// /< Roll angular speed (rad/s)
// /< Pitch angular speed (rad/s)
// /< Yaw angular speed (rad/s)
/* *
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_attitude_pack(mut system_id: uint8_t,
                                               mut component_id: uint8_t,
                                               mut msg:
                                                   *mut mavlink_message_t,
                                               mut time_boot_ms: uint32_t,
                                               mut roll: libc::c_float,
                                               mut pitch: libc::c_float,
                                               mut yaw: libc::c_float,
                                               mut rollspeed: libc::c_float,
                                               mut pitchspeed: libc::c_float,
                                               mut yawspeed: libc::c_float)
 -> uint16_t {
    let mut packet: mavlink_attitude_t =
        mavlink_attitude_t{time_boot_ms: 0,
                           roll: 0.,
                           pitch: 0.,
                           yaw: 0.,
                           rollspeed: 0.,
                           pitchspeed: 0.,
                           yawspeed: 0.,};
    packet.time_boot_ms = time_boot_ms;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.rollspeed = rollspeed;
    packet.pitchspeed = pitchspeed;
    packet.yawspeed = yawspeed;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_attitude_t as *const libc::c_void,
           28 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 30 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    28 as libc::c_int as uint8_t,
                                    39 as libc::c_int as uint8_t);
}
// MESSAGE GPS_GLOBAL_ORIGIN PACKING
// /< Latitude (WGS84), in degrees * 1E7
// /< Longitude (WGS84), in degrees * 1E7
// /< Altitude (AMSL), in meters * 1000 (positive for up)
/* *
 * @brief Pack a gps_global_origin message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (AMSL), in meters * 1000 (positive for up)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_gps_global_origin_pack(mut system_id:
                                                            uint8_t,
                                                        mut component_id:
                                                            uint8_t,
                                                        mut msg:
                                                            *mut mavlink_message_t,
                                                        mut latitude: int32_t,
                                                        mut longitude:
                                                            int32_t,
                                                        mut altitude: int32_t)
 -> uint16_t {
    let mut packet: mavlink_gps_global_origin_t =
        mavlink_gps_global_origin_t{latitude: 0, longitude: 0, altitude: 0,};
    packet.latitude = latitude;
    packet.longitude = longitude;
    packet.altitude = altitude;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_gps_global_origin_t as
               *const libc::c_void, 12 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 49 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    12 as libc::c_int as uint8_t,
                                    39 as libc::c_int as uint8_t);
}
// MESSAGE GLOBAL_POSITION_INT PACKING
// /< Timestamp (milliseconds since system boot)
// /< Latitude, expressed as * 1E7
// /< Longitude, expressed as * 1E7
// /< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
// /< Altitude above ground in meters, expressed as * 1000 (millimeters)
// /< Ground X Speed (Latitude), expressed as m/s * 100
// /< Ground Y Speed (Longitude), expressed as m/s * 100
// /< Ground Z Speed (Altitude), expressed as m/s * 100
// /< Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
/* *
 * @brief Pack a global_position_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param lat Latitude, expressed as * 1E7
 * @param lon Longitude, expressed as * 1E7
 * @param alt Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)
 * @param relative_alt Altitude above ground in meters, expressed as * 1000 (millimeters)
 * @param vx Ground X Speed (Latitude), expressed as m/s * 100
 * @param vy Ground Y Speed (Longitude), expressed as m/s * 100
 * @param vz Ground Z Speed (Altitude), expressed as m/s * 100
 * @param hdg Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_global_position_int_pack(mut system_id:
                                                              uint8_t,
                                                          mut component_id:
                                                              uint8_t,
                                                          mut msg:
                                                              *mut mavlink_message_t,
                                                          mut time_boot_ms:
                                                              uint32_t,
                                                          mut lat: int32_t,
                                                          mut lon: int32_t,
                                                          mut alt: int32_t,
                                                          mut relative_alt:
                                                              int32_t,
                                                          mut vx: int16_t,
                                                          mut vy: int16_t,
                                                          mut vz: int16_t,
                                                          mut hdg: uint16_t)
 -> uint16_t {
    let mut packet: mavlink_global_position_int_t =
        mavlink_global_position_int_t{time_boot_ms: 0,
                                      lat: 0,
                                      lon: 0,
                                      alt: 0,
                                      relative_alt: 0,
                                      vx: 0,
                                      vy: 0,
                                      vz: 0,
                                      hdg: 0,};
    packet.time_boot_ms = time_boot_ms;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.relative_alt = relative_alt;
    packet.vx = vx;
    packet.vy = vy;
    packet.vz = vz;
    packet.hdg = hdg;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_global_position_int_t as
               *const libc::c_void, 28 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 33 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    28 as libc::c_int as uint8_t,
                                    104 as libc::c_int as uint8_t);
}
// MESSAGE GPS_RAW_INT PACKING
// /< Timestamp (microseconds since UNIX epoch or microseconds since system boot)
// /< Latitude (WGS84), in degrees * 1E7
// /< Longitude (WGS84), in degrees * 1E7
// /< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
// /< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
// /< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
// /< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
// /< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
// /< 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
// /< Number of satellites visible. If unknown, set to 255
/* *
 * @brief Pack a gps_raw_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec Timestamp (microseconds since UNIX epoch or microseconds since system boot)
 * @param fix_type 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param lat Latitude (WGS84), in degrees * 1E7
 * @param lon Longitude (WGS84), in degrees * 1E7
 * @param alt Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
 * @param eph GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param epv GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
 * @param vel GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
 * @param cog Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
 * @param satellites_visible Number of satellites visible. If unknown, set to 255
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_gps_raw_int_pack(mut system_id: uint8_t,
                                                  mut component_id: uint8_t,
                                                  mut msg:
                                                      *mut mavlink_message_t,
                                                  mut time_usec: uint64_t,
                                                  mut fix_type: uint8_t,
                                                  mut lat: int32_t,
                                                  mut lon: int32_t,
                                                  mut alt: int32_t,
                                                  mut eph: uint16_t,
                                                  mut epv: uint16_t,
                                                  mut vel: uint16_t,
                                                  mut cog: uint16_t,
                                                  mut satellites_visible:
                                                      uint8_t) -> uint16_t {
    let mut packet: mavlink_gps_raw_int_t =
        mavlink_gps_raw_int_t{time_usec: 0,
                              lat: 0,
                              lon: 0,
                              alt: 0,
                              eph: 0,
                              epv: 0,
                              vel: 0,
                              cog: 0,
                              fix_type: 0,
                              satellites_visible: 0,};
    packet.time_usec = time_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.eph = eph;
    packet.epv = epv;
    packet.vel = vel;
    packet.cog = cog;
    packet.fix_type = fix_type;
    packet.satellites_visible = satellites_visible;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_gps_raw_int_t as *const libc::c_void,
           30 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 24 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    30 as libc::c_int as uint8_t,
                                    24 as libc::c_int as uint8_t);
}
// MESSAGE RC_CHANNELS_RAW PACKING
// /< Timestamp (milliseconds since system boot)
// /< RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
// /< Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
// /< Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
/* *
 * @brief Pack a rc_channels_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param port Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
 * @param chan1_raw RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan2_raw RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan3_raw RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan4_raw RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan5_raw RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan6_raw RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan7_raw RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param chan8_raw RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
 * @param rssi Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_rc_channels_raw_pack(mut system_id: uint8_t,
                                                      mut component_id:
                                                          uint8_t,
                                                      mut msg:
                                                          *mut mavlink_message_t,
                                                      mut time_boot_ms:
                                                          uint32_t,
                                                      mut port: uint8_t,
                                                      mut chan1_raw: uint16_t,
                                                      mut chan2_raw: uint16_t,
                                                      mut chan3_raw: uint16_t,
                                                      mut chan4_raw: uint16_t,
                                                      mut chan5_raw: uint16_t,
                                                      mut chan6_raw: uint16_t,
                                                      mut chan7_raw: uint16_t,
                                                      mut chan8_raw: uint16_t,
                                                      mut rssi: uint8_t)
 -> uint16_t {
    let mut packet: mavlink_rc_channels_raw_t =
        mavlink_rc_channels_raw_t{time_boot_ms: 0,
                                  chan1_raw: 0,
                                  chan2_raw: 0,
                                  chan3_raw: 0,
                                  chan4_raw: 0,
                                  chan5_raw: 0,
                                  chan6_raw: 0,
                                  chan7_raw: 0,
                                  chan8_raw: 0,
                                  port: 0,
                                  rssi: 0,};
    packet.time_boot_ms = time_boot_ms;
    packet.chan1_raw = chan1_raw;
    packet.chan2_raw = chan2_raw;
    packet.chan3_raw = chan3_raw;
    packet.chan4_raw = chan4_raw;
    packet.chan5_raw = chan5_raw;
    packet.chan6_raw = chan6_raw;
    packet.chan7_raw = chan7_raw;
    packet.chan8_raw = chan8_raw;
    packet.port = port;
    packet.rssi = rssi;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_rc_channels_raw_t as
               *const libc::c_void, 22 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 35 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    22 as libc::c_int as uint8_t,
                                    244 as libc::c_int as uint8_t);
}
// MESSAGE SYS_STATUS PACKING
// /< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
// /< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
// /< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
// /< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
// /< Battery voltage, in millivolts (1 = 1 millivolt)
// /< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
// /< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
// /< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
// /< Autopilot-specific errors
// /< Autopilot-specific errors
// /< Autopilot-specific errors
// /< Autopilot-specific errors
// /< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
/* *
 * @brief Pack a sys_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param onboard_control_sensors_present Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param onboard_control_sensors_enabled Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param onboard_control_sensors_health Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
 * @param load Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
 * @param voltage_battery Battery voltage, in millivolts (1 = 1 millivolt)
 * @param current_battery Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
 * @param battery_remaining Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
 * @param drop_rate_comm Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_comm Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
 * @param errors_count1 Autopilot-specific errors
 * @param errors_count2 Autopilot-specific errors
 * @param errors_count3 Autopilot-specific errors
 * @param errors_count4 Autopilot-specific errors
 * @return length of the message in bytes (excluding serial stream start sign)
 */
#[inline]
unsafe extern "C" fn mavlink_msg_sys_status_pack(mut system_id: uint8_t,
                                                 mut component_id: uint8_t,
                                                 mut msg:
                                                     *mut mavlink_message_t,
                                                 mut onboard_control_sensors_present:
                                                     uint32_t,
                                                 mut onboard_control_sensors_enabled:
                                                     uint32_t,
                                                 mut onboard_control_sensors_health:
                                                     uint32_t,
                                                 mut load: uint16_t,
                                                 mut voltage_battery:
                                                     uint16_t,
                                                 mut current_battery: int16_t,
                                                 mut battery_remaining:
                                                     int8_t,
                                                 mut drop_rate_comm: uint16_t,
                                                 mut errors_comm: uint16_t,
                                                 mut errors_count1: uint16_t,
                                                 mut errors_count2: uint16_t,
                                                 mut errors_count3: uint16_t,
                                                 mut errors_count4: uint16_t)
 -> uint16_t {
    let mut packet: mavlink_sys_status_t =
        mavlink_sys_status_t{onboard_control_sensors_present: 0,
                             onboard_control_sensors_enabled: 0,
                             onboard_control_sensors_health: 0,
                             load: 0,
                             voltage_battery: 0,
                             current_battery: 0,
                             drop_rate_comm: 0,
                             errors_comm: 0,
                             errors_count1: 0,
                             errors_count2: 0,
                             errors_count3: 0,
                             errors_count4: 0,
                             battery_remaining: 0,};
    packet.onboard_control_sensors_present = onboard_control_sensors_present;
    packet.onboard_control_sensors_enabled = onboard_control_sensors_enabled;
    packet.onboard_control_sensors_health = onboard_control_sensors_health;
    packet.load = load;
    packet.voltage_battery = voltage_battery;
    packet.current_battery = current_battery;
    packet.drop_rate_comm = drop_rate_comm;
    packet.errors_comm = errors_comm;
    packet.errors_count1 = errors_count1;
    packet.errors_count2 = errors_count2;
    packet.errors_count3 = errors_count3;
    packet.errors_count4 = errors_count4;
    packet.battery_remaining = battery_remaining;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_sys_status_t as *const libc::c_void,
           31 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 1 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    31 as libc::c_int as uint8_t,
                                    124 as libc::c_int as uint8_t);
}
#[inline]
unsafe extern "C" fn mavlink_msg_heartbeat_pack(mut system_id: uint8_t,
                                                mut component_id: uint8_t,
                                                mut msg:
                                                    *mut mavlink_message_t,
                                                mut type_0: uint8_t,
                                                mut autopilot: uint8_t,
                                                mut base_mode: uint8_t,
                                                mut custom_mode: uint32_t,
                                                mut system_status: uint8_t)
 -> uint16_t {
    let mut packet: mavlink_heartbeat_t =
        mavlink_heartbeat_t{custom_mode: 0,
                            type_0: 0,
                            autopilot: 0,
                            base_mode: 0,
                            system_status: 0,
                            mavlink_version: 0,};
    packet.custom_mode = custom_mode;
    packet.type_0 = type_0;
    packet.autopilot = autopilot;
    packet.base_mode = base_mode;
    packet.system_status = system_status;
    packet.mavlink_version = 3 as libc::c_int as uint8_t;
    memcpy(&mut *(*msg).payload64.as_mut_ptr().offset(0 as libc::c_int as
                                                          isize) as
               *mut uint64_t as *mut libc::c_char as *mut libc::c_void,
           &mut packet as *mut mavlink_heartbeat_t as *const libc::c_void,
           9 as libc::c_int as libc::c_ulong);
    (*msg).msgid = 0 as libc::c_int as uint8_t;
    return mavlink_finalize_message(msg, system_id, component_id,
                                    9 as libc::c_int as uint8_t,
                                    50 as libc::c_int as uint8_t);
}
#[inline]
unsafe extern "C" fn mavlink_msg_to_send_buffer(mut buffer: *mut uint8_t,
                                                mut msg:
                                                    *const mavlink_message_t)
 -> uint16_t {
    memcpy(buffer as *mut libc::c_void,
           &(*msg).magic as *const uint8_t as *const libc::c_void,
           (5 as libc::c_int + 1 as libc::c_int +
                (*msg).len as uint16_t as libc::c_int) as libc::c_ulong);
    let mut ck: *mut uint8_t =
        buffer.offset((5 as libc::c_int + 1 as libc::c_int +
                           (*msg).len as uint16_t as libc::c_int) as isize);
    *ck.offset(0 as libc::c_int as isize) =
        ((*msg).checksum as libc::c_int & 0xff as libc::c_int) as uint8_t;
    *ck.offset(1 as libc::c_int as isize) =
        ((*msg).checksum as libc::c_int >> 8 as libc::c_int) as uint8_t;
    return (5 as libc::c_int + 1 as libc::c_int + 2 as libc::c_int +
                (*msg).len as uint16_t as libc::c_int) as uint16_t;
}
// FIXME dependency on mw.c
static mut mavlinkPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut portConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
static mut mavlinkTelemetryEnabled: bool = 0 as libc::c_int != 0;
static mut mavlinkPortSharing: portSharing_e = PORTSHARING_UNUSED;
/* MAVLink datastream rates in Hz */
static mut mavRates: [uint8_t; 12] =
    [0, 0, 2 as libc::c_int as uint8_t, 5 as libc::c_int as uint8_t, 0, 0,
     2 as libc::c_int as uint8_t, 0, 0, 0, 10 as libc::c_int as uint8_t,
     10 as libc::c_int as uint8_t];
static mut mavTicks: [uint8_t; 12] = [0; 12];
static mut mavMsg: mavlink_message_t =
    mavlink_message_t{checksum: 0,
                      magic: 0,
                      len: 0,
                      seq: 0,
                      sysid: 0,
                      compid: 0,
                      msgid: 0,
                      payload64: [0; 33],};
static mut mavBuffer: [uint8_t; 263] = [0; 263];
static mut lastMavlinkMessage: uint32_t = 0 as libc::c_int as uint32_t;
unsafe extern "C" fn mavlinkStreamTrigger(mut streamNum: MAV_DATA_STREAM)
 -> libc::c_int {
    let mut rate: uint8_t = mavRates[streamNum as usize];
    if rate as libc::c_int == 0 as libc::c_int { return 0 as libc::c_int }
    if mavTicks[streamNum as usize] as libc::c_int == 0 as libc::c_int {
        // we're triggering now, setup the next trigger point
        if rate as libc::c_int > 50 as libc::c_int {
            rate = 50 as libc::c_int as uint8_t
        }
        mavTicks[streamNum as usize] =
            (50 as libc::c_int / rate as libc::c_int) as uint8_t;
        return 1 as libc::c_int
    }
    // count down at TASK_RATE_HZ
    mavTicks[streamNum as usize] =
        mavTicks[streamNum as usize].wrapping_sub(1);
    return 0 as libc::c_int;
}
unsafe extern "C" fn mavlinkSerialWrite(mut buf: *mut uint8_t,
                                        mut length: uint16_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < length as libc::c_int {
        serialWrite(mavlinkPort, *buf.offset(i as isize));
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn freeMAVLinkTelemetryPort() {
    closeSerialPort(mavlinkPort);
    mavlinkPort = 0 as *mut serialPort_t;
    mavlinkTelemetryEnabled = 0 as libc::c_int != 0;
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
pub unsafe extern "C" fn initMAVLinkTelemetry() {
    portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_MAVLINK);
    mavlinkPortSharing =
        determinePortSharing(portConfig, FUNCTION_TELEMETRY_MAVLINK);
}
#[no_mangle]
pub unsafe extern "C" fn configureMAVLinkTelemetryPort() {
    if portConfig.is_null() { return }
    let mut baudRateIndex: baudRate_e =
        (*portConfig).telemetry_baudrateIndex as baudRate_e;
    if baudRateIndex as libc::c_uint ==
           BAUD_AUTO as libc::c_int as libc::c_uint {
        // default rate for minimOSD
        baudRateIndex = BAUD_57600
    }
    mavlinkPort =
        openSerialPort((*portConfig).identifier, FUNCTION_TELEMETRY_MAVLINK,
                       None, 0 as *mut libc::c_void,
                       *baudRates.as_ptr().offset(baudRateIndex as isize),
                       MODE_TX,
                       if (*telemetryConfig()).telemetry_inverted as
                              libc::c_int != 0 {
                           SERIAL_INVERTED as libc::c_int
                       } else { SERIAL_NOT_INVERTED as libc::c_int } as
                           portOptions_e);
    if mavlinkPort.is_null() { return }
    mavlinkTelemetryEnabled = 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn checkMAVLinkTelemetryState() {
    if !portConfig.is_null() &&
           telemetryCheckRxPortShared(portConfig) as libc::c_int != 0 {
        if !mavlinkTelemetryEnabled && !telemetrySharedPort.is_null() {
            mavlinkPort = telemetrySharedPort;
            mavlinkTelemetryEnabled = 1 as libc::c_int != 0
        }
    } else {
        let mut newTelemetryEnabledValue: bool =
            telemetryDetermineEnabledState(mavlinkPortSharing);
        if newTelemetryEnabledValue as libc::c_int ==
               mavlinkTelemetryEnabled as libc::c_int {
            return
        }
        if newTelemetryEnabledValue {
            configureMAVLinkTelemetryPort();
        } else { freeMAVLinkTelemetryPort(); }
    };
}
#[no_mangle]
pub unsafe extern "C" fn mavlinkSendSystemStatus() {
    let mut msgLength: uint16_t = 0;
    let mut onboardControlAndSensors: uint32_t =
        35843 as libc::c_int as uint32_t;
    /*
    onboard_control_sensors_present Bitmask
    fedcba9876543210
    1000110000000011    For all   = 35843
    0001000000000100    With Mag  = 4100
    0010000000001000    With Baro = 8200
    0100000000100000    With GPS  = 16416
    0000001111111111
    */
    if sensors(SENSOR_MAG as libc::c_int as uint32_t) {
        onboardControlAndSensors |= 4100 as libc::c_int as libc::c_uint
    }
    if sensors(SENSOR_BARO as libc::c_int as uint32_t) {
        onboardControlAndSensors |= 8200 as libc::c_int as libc::c_uint
    }
    if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
        onboardControlAndSensors |= 16416 as libc::c_int as libc::c_uint
    }
    let mut batteryVoltage: uint16_t = 0 as libc::c_int as uint16_t;
    let mut batteryAmperage: int16_t = -(1 as libc::c_int) as int16_t;
    let mut batteryRemaining: int8_t = 100 as libc::c_int as int8_t;
    if (getBatteryState() as libc::c_uint) <
           BATTERY_NOT_PRESENT as libc::c_int as libc::c_uint {
        batteryVoltage =
            if isBatteryVoltageConfigured() as libc::c_int != 0 {
                (getBatteryVoltage() as libc::c_int) * 100 as libc::c_int
            } else { batteryVoltage as libc::c_int } as uint16_t;
        batteryAmperage =
            if isAmperageConfigured() as libc::c_int != 0 {
                getAmperage()
            } else { batteryAmperage as libc::c_int } as int16_t;
        batteryRemaining =
            if isBatteryVoltageConfigured() as libc::c_int != 0 {
                calculateBatteryPercentageRemaining() as libc::c_int
            } else { batteryRemaining as libc::c_int } as int8_t
    }
    mavlink_msg_sys_status_pack(0 as libc::c_int as uint8_t,
                                200 as libc::c_int as uint8_t, &mut mavMsg,
                                onboardControlAndSensors,
                                onboardControlAndSensors,
                                onboardControlAndSensors &
                                    1023 as libc::c_int as libc::c_uint,
                                0 as libc::c_int as uint16_t, batteryVoltage,
                                batteryAmperage, batteryRemaining,
                                0 as libc::c_int as uint16_t,
                                0 as libc::c_int as uint16_t,
                                0 as libc::c_int as uint16_t,
                                0 as libc::c_int as uint16_t,
                                0 as libc::c_int as uint16_t,
                                0 as libc::c_int as uint16_t);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
}
#[no_mangle]
pub unsafe extern "C" fn mavlinkSendRCChannelsAndRSSI() {
    let mut msgLength: uint16_t = 0;
    mavlink_msg_rc_channels_raw_pack(0 as libc::c_int as uint8_t,
                                     200 as libc::c_int as uint8_t,
                                     &mut mavMsg, millis(),
                                     0 as libc::c_int as uint8_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 1 as libc::c_int {
                                         rcData[0 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 2 as libc::c_int {
                                         rcData[1 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 3 as libc::c_int {
                                         rcData[2 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 4 as libc::c_int {
                                         rcData[3 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 5 as libc::c_int {
                                         rcData[4 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 6 as libc::c_int {
                                         rcData[5 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 7 as libc::c_int {
                                         rcData[6 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     if rxRuntimeConfig.channelCount as
                                            libc::c_int >= 8 as libc::c_int {
                                         rcData[7 as libc::c_int as usize] as
                                             libc::c_int
                                     } else { 0 as libc::c_int } as uint16_t,
                                     constrain(scaleRange(getRssi() as
                                                              libc::c_int,
                                                          0 as libc::c_int,
                                                          1023 as libc::c_int,
                                                          0 as libc::c_int,
                                                          255 as libc::c_int),
                                               0 as libc::c_int,
                                               255 as libc::c_int) as
                                         uint8_t);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
}
#[no_mangle]
pub unsafe extern "C" fn mavlinkSendPosition() {
    let mut msgLength: uint16_t = 0;
    let mut gpsFixType: uint8_t = 0 as libc::c_int as uint8_t;
    if !sensors(SENSOR_GPS as libc::c_int as uint32_t) { return }
    if stateFlags as libc::c_int & GPS_FIX as libc::c_int == 0 {
        gpsFixType = 1 as libc::c_int as uint8_t
    } else if (gpsSol.numSat as libc::c_int) < 5 as libc::c_int {
        gpsFixType = 2 as libc::c_int as uint8_t
    } else { gpsFixType = 3 as libc::c_int as uint8_t }
    mavlink_msg_gps_raw_int_pack(0 as libc::c_int as uint8_t,
                                 200 as libc::c_int as uint8_t, &mut mavMsg,
                                 micros() as uint64_t, gpsFixType,
                                 gpsSol.llh.lat, gpsSol.llh.lon,
                                 gpsSol.llh.alt * 1000 as libc::c_int,
                                 65535 as libc::c_int as uint16_t,
                                 65535 as libc::c_int as uint16_t,
                                 gpsSol.groundSpeed,
                                 (gpsSol.groundCourse as libc::c_int *
                                      10 as libc::c_int) as uint16_t,
                                 gpsSol.numSat);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
    // Global position
    mavlink_msg_global_position_int_pack(0 as libc::c_int as uint8_t,
                                         200 as libc::c_int as uint8_t,
                                         &mut mavMsg, micros(),
                                         gpsSol.llh.lat, gpsSol.llh.lon,
                                         gpsSol.llh.alt * 1000 as libc::c_int,
                                         gpsSol.llh.alt * 1000 as libc::c_int,
                                         0 as libc::c_int as int16_t,
                                         0 as libc::c_int as int16_t,
                                         0 as libc::c_int as int16_t,
                                         (attitude.values.yaw as libc::c_int /
                                              10 as libc::c_int) as uint16_t);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
    mavlink_msg_gps_global_origin_pack(0 as libc::c_int as uint8_t,
                                       200 as libc::c_int as uint8_t,
                                       &mut mavMsg,
                                       GPS_home[0 as libc::c_int as usize],
                                       GPS_home[1 as libc::c_int as usize],
                                       0 as libc::c_int);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
}
#[no_mangle]
pub unsafe extern "C" fn mavlinkSendAttitude() {
    let mut msgLength: uint16_t = 0;
    mavlink_msg_attitude_pack(0 as libc::c_int as uint8_t,
                              200 as libc::c_int as uint8_t, &mut mavMsg,
                              millis(),
                              attitude.values.roll as libc::c_int as
                                  libc::c_float / 10.0f32 * 0.0174532925f32,
                              -(attitude.values.pitch as libc::c_int) as
                                  libc::c_float / 10.0f32 * 0.0174532925f32,
                              attitude.values.yaw as libc::c_int as
                                  libc::c_float / 10.0f32 * 0.0174532925f32,
                              0 as libc::c_int as libc::c_float,
                              0 as libc::c_int as libc::c_float,
                              0 as libc::c_int as libc::c_float);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
}
#[no_mangle]
pub unsafe extern "C" fn mavlinkSendHUDAndHeartbeat() {
    let mut msgLength: uint16_t = 0;
    let mut mavAltitude: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut mavGroundSpeed: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut mavAirSpeed: libc::c_float = 0 as libc::c_int as libc::c_float;
    let mut mavClimbRate: libc::c_float = 0 as libc::c_int as libc::c_float;
    // use ground speed if source available
    if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
        mavGroundSpeed =
            gpsSol.groundSpeed as libc::c_int as libc::c_float / 100.0f32
    }
    // select best source for altitude
    if sensors(SENSOR_GPS as libc::c_int as uint32_t) {
        // No sonar or baro, just display altitude above MLS
        mavAltitude = gpsSol.llh.alt as libc::c_float
    }
    mavlink_msg_vfr_hud_pack(0 as libc::c_int as uint8_t,
                             200 as libc::c_int as uint8_t, &mut mavMsg,
                             mavAirSpeed, mavGroundSpeed,
                             (attitude.values.yaw as libc::c_int /
                                  10 as libc::c_int) as int16_t,
                             scaleRange(constrain(rcData[THROTTLE as
                                                             libc::c_int as
                                                             usize] as
                                                      libc::c_int,
                                                  1000 as libc::c_int,
                                                  2000 as libc::c_int),
                                        1000 as libc::c_int,
                                        2000 as libc::c_int, 0 as libc::c_int,
                                        100 as libc::c_int) as uint16_t,
                             mavAltitude, mavClimbRate);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
    let mut mavModes: uint8_t =
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED as libc::c_int as uint8_t;
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        mavModes =
            (mavModes as libc::c_int |
                 MAV_MODE_FLAG_SAFETY_ARMED as libc::c_int) as uint8_t
    }
    let mut mavSystemType: uint8_t = 0;
    match (*mixerConfig()).mixerMode as libc::c_int {
        1 => { mavSystemType = MAV_TYPE_TRICOPTER as libc::c_int as uint8_t }
        2 | 3 | 9 | 17 => {
            mavSystemType = MAV_TYPE_QUADROTOR as libc::c_int as uint8_t
        }
        6 | 7 | 10 => {
            mavSystemType = MAV_TYPE_HEXAROTOR as libc::c_int as uint8_t
        }
        11 | 12 | 13 => {
            mavSystemType = MAV_TYPE_OCTOROTOR as libc::c_int as uint8_t
        }
        8 | 14 | 24 => {
            mavSystemType = MAV_TYPE_FIXED_WING as libc::c_int as uint8_t
        }
        15 | 16 => {
            mavSystemType = MAV_TYPE_HELICOPTER as libc::c_int as uint8_t
        }
        _ => { mavSystemType = MAV_TYPE_GENERIC as libc::c_int as uint8_t }
    }
    // Custom mode for compatibility with APM OSDs
    let mut mavCustomMode: uint8_t =
        1 as libc::c_int as uint8_t; // Acro by default
    if flightModeFlags as libc::c_int & ANGLE_MODE as libc::c_int != 0 ||
           flightModeFlags as libc::c_int & HORIZON_MODE as libc::c_int != 0 {
        mavCustomMode = 0 as libc::c_int as uint8_t; //Stabilize
        mavModes =
            (mavModes as libc::c_int |
                 MAV_MODE_FLAG_STABILIZE_ENABLED as libc::c_int) as uint8_t
    }
    if flightModeFlags as libc::c_int & BARO_MODE as libc::c_int != 0 {
        mavCustomMode = 2 as libc::c_int as uint8_t
        //Alt Hold
    }
    if flightModeFlags as libc::c_int & GPS_HOME_MODE as libc::c_int != 0 {
        mavCustomMode = 6 as libc::c_int as uint8_t
        //Return to Launch
    }
    if flightModeFlags as libc::c_int & GPS_HOLD_MODE as libc::c_int != 0 {
        mavCustomMode = 16 as libc::c_int as uint8_t
        //Position Hold (Earlier called Hybrid)
    }
    let mut mavSystemState: uint8_t = 0 as libc::c_int as uint8_t;
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        if failsafeIsActive() {
            mavSystemState = MAV_STATE_CRITICAL as libc::c_int as uint8_t
        } else { mavSystemState = MAV_STATE_ACTIVE as libc::c_int as uint8_t }
    } else { mavSystemState = MAV_STATE_STANDBY as libc::c_int as uint8_t }
    mavlink_msg_heartbeat_pack(0 as libc::c_int as uint8_t,
                               200 as libc::c_int as uint8_t, &mut mavMsg,
                               mavSystemType,
                               MAV_AUTOPILOT_GENERIC as libc::c_int as
                                   uint8_t, mavModes,
                               mavCustomMode as uint32_t, mavSystemState);
    msgLength =
        mavlink_msg_to_send_buffer(mavBuffer.as_mut_ptr(), &mut mavMsg);
    mavlinkSerialWrite(mavBuffer.as_mut_ptr(), msgLength);
}
#[no_mangle]
pub unsafe extern "C" fn processMAVLinkTelemetry() {
    // is executed @ TELEMETRY_MAVLINK_MAXRATE rate
    if mavlinkStreamTrigger(MAV_DATA_STREAM_EXTENDED_STATUS) != 0 {
        mavlinkSendSystemStatus();
    }
    if mavlinkStreamTrigger(MAV_DATA_STREAM_RC_CHANNELS) != 0 {
        mavlinkSendRCChannelsAndRSSI();
    }
    if mavlinkStreamTrigger(MAV_DATA_STREAM_POSITION) != 0 {
        mavlinkSendPosition();
    }
    if mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA1) != 0 {
        mavlinkSendAttitude();
    }
    if mavlinkStreamTrigger(MAV_DATA_STREAM_EXTRA2) != 0 {
        mavlinkSendHUDAndHeartbeat();
    };
}
#[no_mangle]
pub unsafe extern "C" fn handleMAVLinkTelemetry() {
    if !mavlinkTelemetryEnabled { return }
    if mavlinkPort.is_null() { return }
    let mut now: uint32_t = micros();
    if now.wrapping_sub(lastMavlinkMessage) >=
           (1000 as libc::c_int * 1000 as libc::c_int / 50 as libc::c_int) as
               libc::c_uint {
        processMAVLinkTelemetry();
        lastMavlinkMessage = now
    };
}
