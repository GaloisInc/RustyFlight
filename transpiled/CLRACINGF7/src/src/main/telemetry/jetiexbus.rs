use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn strlen(_: *const libc::c_char) -> libc::c_ulong;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
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
    fn bitArrayGet(array: *const libc::c_void, bit: libc::c_uint) -> bool;
    #[no_mangle]
    fn bitArraySet(array: *mut libc::c_void, bit: libc::c_uint);
    #[no_mangle]
    fn serialRxBytesWaiting(instance: *const serialPort_t) -> uint32_t;
    #[no_mangle]
    fn serialWriteBuf(instance: *mut serialPort_t, data: *const uint8_t,
                      count: libc::c_int);
    #[no_mangle]
    fn serialSetMode(instance: *mut serialPort_t, mode: portMode_e);
    #[no_mangle]
    fn isSerialTransmitBufferEmpty(instance: *const serialPort_t) -> bool;
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn getEstimatedAltitude() -> int32_t;
    #[no_mangle]
    fn getEstimatedVario() -> int16_t;
    #[no_mangle]
    static mut attitude: attitudeEulerAngles_t;
    #[no_mangle]
    static mut jetiExBusRequestState: uint8_t;
    #[no_mangle]
    static mut jetiTimeStampRequest: uint32_t;
    #[no_mangle]
    static mut jetiExBusRequestFrame: [uint8_t; 9];
    #[no_mangle]
    static mut jetiExBusPort: *mut serialPort_s;
    #[no_mangle]
    fn jetiExBusCalcCRC16(pt: *mut uint8_t, msgLen: uint8_t) -> uint16_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn getBatteryVoltageLatest() -> uint16_t;
    #[no_mangle]
    fn getAmperageLatest() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type serialPort_t = serialPort_s;
// microsecond time
pub type timeUs_t = uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub union attitudeEulerAngles_t {
    pub raw: [int16_t; 3],
    pub values: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct C2RustUnnamed {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type exBusHeader_e = libc::c_uint;
pub const EXBUS_HEADER_DATA: exBusHeader_e = 6;
pub const EXBUS_HEADER_SUBLEN: exBusHeader_e = 5;
pub const EXBUS_HEADER_DATA_ID: exBusHeader_e = 4;
pub const EXBUS_HEADER_PACKET_ID: exBusHeader_e = 3;
pub const EXBUS_HEADER_MSG_LEN: exBusHeader_e = 2;
pub const EXBUS_HEADER_REQ: exBusHeader_e = 1;
pub const EXBUS_HEADER_SYNC: exBusHeader_e = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const EXBUS_STATE_PROCESSED: C2RustUnnamed_0 = 3;
pub const EXBUS_STATE_RECEIVED: C2RustUnnamed_0 = 2;
pub const EXBUS_STATE_IN_PROGRESS: C2RustUnnamed_0 = 1;
pub const EXBUS_STATE_ZERO: C2RustUnnamed_0 = 0;
pub type currentMeterSource_e = libc::c_uint;
pub const CURRENT_METER_COUNT: currentMeterSource_e = 5;
pub const CURRENT_METER_MSP: currentMeterSource_e = 4;
pub const CURRENT_METER_ESC: currentMeterSource_e = 3;
pub const CURRENT_METER_VIRTUAL: currentMeterSource_e = 2;
pub const CURRENT_METER_ADC: currentMeterSource_e = 1;
pub const CURRENT_METER_NONE: currentMeterSource_e = 0;
pub type voltageMeterSource_e = libc::c_uint;
pub const VOLTAGE_METER_COUNT: voltageMeterSource_e = 3;
pub const VOLTAGE_METER_ESC: voltageMeterSource_e = 2;
pub const VOLTAGE_METER_ADC: voltageMeterSource_e = 1;
pub const VOLTAGE_METER_NONE: voltageMeterSource_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct batteryConfig_s {
    pub vbatmaxcellvoltage: uint8_t,
    pub vbatmincellvoltage: uint8_t,
    pub vbatwarningcellvoltage: uint8_t,
    pub vbatnotpresentcellvoltage: uint8_t,
    pub lvcPercentage: uint8_t,
    pub voltageMeterSource: voltageMeterSource_e,
    pub currentMeterSource: currentMeterSource_e,
    pub batteryCapacity: uint16_t,
    pub useVBatAlerts: bool,
    pub useConsumptionAlerts: bool,
    pub consumptionWarningPercentage: uint8_t,
    pub vbathysteresis: uint8_t,
    pub vbatfullcellvoltage: uint8_t,
}
pub type batteryConfig_t = batteryConfig_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_1 = 64;
pub const SENSOR_GPS: C2RustUnnamed_1 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_1 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_1 = 16;
pub const SENSOR_MAG: C2RustUnnamed_1 = 8;
pub const SENSOR_BARO: C2RustUnnamed_1 = 4;
pub const SENSOR_ACC: C2RustUnnamed_1 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_1 = 1;
pub type exBusSensor_t = exBusSensor_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct exBusSensor_s {
    pub label: *const libc::c_char,
    pub unit: *const libc::c_char,
    pub exDataType: uint8_t,
    pub decimals: uint8_t,
}
// int14_t Data type 14b (-8191 ¸8191)
pub const EX_TYPE_22b: exDataType_e = 4;
// int6_t  Data type 6b (-31 ¸31)
pub const EX_TYPE_14b: exDataType_e = 1;
// int30_t Special data type – GPS coordinates:  lo/hi minute - lo/hi degree.
pub const EX_TYPE_DES: exDataType_e = 255;
pub const EX_HEADING: exSensors_e = 8;
pub const EX_PITCH_ANGLE: exSensors_e = 7;
pub const EX_ROLL_ANGLE: exSensors_e = 6;
pub const EX_VARIO: exSensors_e = 9;
pub const EX_ALTITUDE: exSensors_e = 3;
pub const EX_CAPACITY: exSensors_e = 4;
pub const EX_POWER: exSensors_e = 5;
pub const EX_CURRENT: exSensors_e = 2;
pub const EX_VOLTAGE: exSensors_e = 1;
pub const EXTEL_HEADER_RES: exTelHeader_e = 6;
pub const EXTEL_HEADER_LSN_HB: exTelHeader_e = 5;
pub const EXTEL_HEADER_LSN_LB: exTelHeader_e = 4;
pub const EXTEL_HEADER_USN_HB: exTelHeader_e = 3;
pub const EXTEL_HEADER_USN_LB: exTelHeader_e = 2;
pub const EXTEL_HEADER_SYNC: exTelHeader_e = 0;
pub const EXBUS_TRANS_RX: C2RustUnnamed_2 = 2;
pub const EXBUS_TRANS_IS_TX_COMPLETED: C2RustUnnamed_2 = 3;
pub const EXTEL_HEADER_TYPE_LEN: exTelHeader_e = 1;
pub const EXTEL_HEADER_ID: exTelHeader_e = 7;
pub const EXTEL_HEADER_DATA: exTelHeader_e = 8;
pub const EXBUS_TRANS_TX: C2RustUnnamed_2 = 4;
pub type exTelHeader_e = libc::c_uint;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const EXBUS_TRANS_RX_READY: C2RustUnnamed_2 = 1;
pub const EXBUS_TRANS_ZERO: C2RustUnnamed_2 = 0;
pub type exDataType_e = libc::c_uint;
// only for devicedescription
// int30_t Data type 30b (-536870911 ¸536870911)
pub const EX_TYPE_GPS: exDataType_e = 9;
// int22_t Special data type – time and date
pub const EX_TYPE_30b: exDataType_e = 8;
// int22_t Data type 22b (-2097151 ¸2097151)
pub const EX_TYPE_DT: exDataType_e = 5;
pub const EX_TYPE_6b: exDataType_e = 0;
// after every 15 sensors increment the step by 2 (e.g. ...EX_VAL15, EX_VAL16 = 17) to skip the device description
pub type exSensors_e = libc::c_uint;
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[no_mangle]
pub static mut exDataTypeLen: [uint8_t; 10] =
    [1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t, 0, 0,
     3 as libc::c_int as uint8_t, 3 as libc::c_int as uint8_t, 0, 0,
     4 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t];
// list of telemetry messages
// after every 15 sensors a new header has to be inserted (e.g. "BF D2")
#[no_mangle]
pub static mut jetiExSensors: [exBusSensor_t; 10] =
    [{
         let mut init =
             exBusSensor_s{label:
                               b"BF D1\x00" as *const u8 as
                                   *const libc::c_char,
                           unit: b"\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_DES as libc::c_int as uint8_t,
                           decimals: 0 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Voltage\x00" as *const u8 as
                                   *const libc::c_char,
                           unit: b"V\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_14b as libc::c_int as uint8_t,
                           decimals:
                               ((1 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Current\x00" as *const u8 as
                                   *const libc::c_char,
                           unit: b"A\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_14b as libc::c_int as uint8_t,
                           decimals:
                               ((2 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Altitude\x00" as *const u8 as
                                   *const libc::c_char,
                           unit: b"m\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_14b as libc::c_int as uint8_t,
                           decimals:
                               ((2 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Capacity\x00" as *const u8 as
                                   *const libc::c_char,
                           unit:
                               b"mAh\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_22b as libc::c_int as uint8_t,
                           decimals:
                               ((0 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Power\x00" as *const u8 as
                                   *const libc::c_char,
                           unit: b"W\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_22b as libc::c_int as uint8_t,
                           decimals:
                               ((1 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Roll angle\x00" as *const u8 as
                                   *const libc::c_char,
                           unit:
                               b"\xb0\x00" as *const u8 as
                                   *const libc::c_char,
                           exDataType: EX_TYPE_14b as libc::c_int as uint8_t,
                           decimals:
                               ((1 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Pitch angle\x00" as *const u8 as
                                   *const libc::c_char,
                           unit:
                               b"\xb0\x00" as *const u8 as
                                   *const libc::c_char,
                           exDataType: EX_TYPE_14b as libc::c_int as uint8_t,
                           decimals:
                               ((1 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Heading\x00" as *const u8 as
                                   *const libc::c_char,
                           unit:
                               b"\xb0\x00" as *const u8 as
                                   *const libc::c_char,
                           exDataType: EX_TYPE_14b as libc::c_int as uint8_t,
                           decimals:
                               ((1 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     },
     {
         let mut init =
             exBusSensor_s{label:
                               b"Vario\x00" as *const u8 as
                                   *const libc::c_char,
                           unit:
                               b"m/s\x00" as *const u8 as *const libc::c_char,
                           exDataType: EX_TYPE_22b as libc::c_int as uint8_t,
                           decimals:
                               ((2 as libc::c_int) << 5 as libc::c_int) as
                                   uint8_t,};
         init
     }];
static mut jetiExBusTelemetryFrame: [uint8_t; 40] = [0; 40];
static mut jetiExBusTransceiveState: uint8_t =
    EXBUS_TRANS_RX as libc::c_int as uint8_t;
static mut firstActiveSensor: uint8_t = 0 as libc::c_int as uint8_t;
static mut exSensorEnabled: uint32_t = 0 as libc::c_int as uint32_t;
// Jeti Ex Telemetry CRC calculations for a frame
#[no_mangle]
pub unsafe extern "C" fn calcCRC8(mut pt: *mut uint8_t, mut msgLen: uint8_t)
 -> uint8_t {
    let mut crc: uint8_t = 0 as libc::c_int as uint8_t;
    let mut mlen: uint8_t = 0 as libc::c_int as uint8_t;
    while (mlen as libc::c_int) < msgLen as libc::c_int {
        crc =
            (crc as libc::c_int ^ *pt.offset(mlen as isize) as libc::c_int) as
                uint8_t;
        crc =
            (crc as libc::c_int ^ (crc as libc::c_int) << 1 as libc::c_int ^
                 (crc as libc::c_int) << 2 as libc::c_int ^
                 0xe090700 as libc::c_int >>
                     (crc as libc::c_int >> 3 as libc::c_int &
                          0x18 as libc::c_int)) as uint8_t;
        mlen = mlen.wrapping_add(1)
    }
    return crc;
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
/*
 * -----------------------------------------------
 *  Jeti Ex Bus Telemetry
 * -----------------------------------------------
 */
#[no_mangle]
pub unsafe extern "C" fn initJetiExBusTelemetry() {
    // Init Ex Bus Frame header
    jetiExBusTelemetryFrame[EXBUS_HEADER_SYNC as libc::c_int as usize] =
        0x3b as libc::c_int as uint8_t; // Startbytes
    jetiExBusTelemetryFrame[EXBUS_HEADER_REQ as libc::c_int as usize] =
        0x1 as libc::c_int as uint8_t; // Ex Telemetry
    jetiExBusTelemetryFrame[EXBUS_HEADER_DATA_ID as libc::c_int as usize] =
        0x3a as libc::c_int as uint8_t;
    // Init Ex Telemetry header
    let mut jetiExTelemetryFrame: *mut uint8_t =
        &mut *jetiExBusTelemetryFrame.as_mut_ptr().offset(EXBUS_HEADER_DATA as
                                                              libc::c_int as
                                                              isize) as
            *mut uint8_t; // Startbyte
    *jetiExTelemetryFrame.offset(EXTEL_HEADER_SYNC as libc::c_int as isize) =
        0x9f as libc::c_int as uint8_t; // Serial Number 4 Byte
    *jetiExTelemetryFrame.offset(EXTEL_HEADER_USN_LB as libc::c_int as isize)
        =
        0x1e as libc::c_int as
            uint8_t; // increment by telemetry count (%16) > only 15 values per device possible
    *jetiExTelemetryFrame.offset(EXTEL_HEADER_USN_HB as libc::c_int as isize)
        = 0xa4 as libc::c_int as uint8_t; // reserved, by default 0x00
    *jetiExTelemetryFrame.offset(EXTEL_HEADER_LSN_LB as libc::c_int as isize)
        = 0 as libc::c_int as uint8_t;
    *jetiExTelemetryFrame.offset(EXTEL_HEADER_LSN_HB as libc::c_int as isize)
        = 0 as libc::c_int as uint8_t;
    *jetiExTelemetryFrame.offset(EXTEL_HEADER_RES as libc::c_int as isize) =
        0 as libc::c_int as uint8_t;
    //exSensorEnabled = 0x3fe;
    // Check which sensors are available
    if (*batteryConfig()).voltageMeterSource as libc::c_uint !=
           VOLTAGE_METER_NONE as libc::c_int as libc::c_uint {
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_VOLTAGE as libc::c_int as libc::c_uint);
    }
    if (*batteryConfig()).currentMeterSource as libc::c_uint !=
           CURRENT_METER_NONE as libc::c_int as libc::c_uint {
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_CURRENT as libc::c_int as libc::c_uint);
    }
    if (*batteryConfig()).voltageMeterSource as libc::c_uint !=
           VOLTAGE_METER_NONE as libc::c_int as libc::c_uint &&
           (*batteryConfig()).currentMeterSource as libc::c_uint !=
               CURRENT_METER_NONE as libc::c_int as libc::c_uint {
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_POWER as libc::c_int as libc::c_uint);
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_CAPACITY as libc::c_int as libc::c_uint);
    }
    if sensors(SENSOR_BARO as libc::c_int as uint32_t) {
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_ALTITUDE as libc::c_int as libc::c_uint);
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_VARIO as libc::c_int as libc::c_uint);
    }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_ROLL_ANGLE as libc::c_int as libc::c_uint);
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_PITCH_ANGLE as libc::c_int as libc::c_uint);
    }
    if sensors(SENSOR_MAG as libc::c_int as uint32_t) {
        bitArraySet(&mut exSensorEnabled as *mut uint32_t as
                        *mut libc::c_void,
                    EX_HEADING as libc::c_int as libc::c_uint);
    }
    firstActiveSensor = getNextActiveSensor(0 as libc::c_int as uint8_t);
    // find the first active sensor
}
#[no_mangle]
pub unsafe extern "C" fn createExTelemetryTextMessage(mut exMessage:
                                                          *mut uint8_t,
                                                      mut messageID: uint8_t,
                                                      mut sensor:
                                                          *const exBusSensor_t) {
    let mut labelLength: uint8_t =
        strlen((*sensor).label) as uint8_t; // Device ID
    let mut unitLength: uint8_t =
        strlen((*sensor).unit) as uint8_t; // Sensor ID (%16)
    *exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int as isize) =
        (1 as libc::c_int + 6 as libc::c_int + 1 as libc::c_int +
             labelLength as libc::c_int + unitLength as libc::c_int) as
            uint8_t; // Device ID
    *exMessage.offset(EXTEL_HEADER_LSN_LB as libc::c_int as isize) =
        (messageID as libc::c_int & 0xf0 as libc::c_int) as
            uint8_t; // Sensor ID (%16) | EX Data Type
    *exMessage.offset(EXTEL_HEADER_ID as libc::c_int as isize) =
        (messageID as libc::c_int & 0xf as libc::c_int) as uint8_t;
    *exMessage.offset(EXTEL_HEADER_DATA as libc::c_int as isize) =
        (((labelLength as libc::c_int) << 3 as libc::c_int) +
             unitLength as libc::c_int) as uint8_t;
    memcpy(&mut *exMessage.offset((EXTEL_HEADER_DATA as libc::c_int +
                                       1 as libc::c_int) as isize) as
               *mut uint8_t as *mut libc::c_void,
           (*sensor).label as *const libc::c_void,
           labelLength as libc::c_ulong);
    memcpy(&mut *exMessage.offset((EXTEL_HEADER_DATA as libc::c_int +
                                       1 as libc::c_int +
                                       labelLength as libc::c_int) as isize)
               as *mut uint8_t as *mut libc::c_void,
           (*sensor).unit as *const libc::c_void,
           unitLength as libc::c_ulong);
    *exMessage.offset((*exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int
                                             as isize) as libc::c_int +
                           1 as libc::c_int) as isize) =
        calcCRC8(&mut *exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int
                                            as isize),
                 *exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int as
                                       isize));
}
#[no_mangle]
pub unsafe extern "C" fn getSensorValue(mut sensor: uint8_t) -> int32_t {
    match sensor as libc::c_int {
        1 => { return getBatteryVoltageLatest() as int32_t }
        2 => { return getAmperageLatest() }
        3 => { return getEstimatedAltitude() }
        4 => { return getMAhDrawn() }
        5 => {
            return getBatteryVoltageLatest() as libc::c_int *
                       getAmperageLatest() / 100 as libc::c_int
        }
        6 => { return attitude.values.roll as int32_t }
        7 => { return attitude.values.pitch as int32_t }
        8 => { return attitude.values.yaw as int32_t }
        9 => { return getEstimatedVario() as int32_t }
        _ => { return -(1 as libc::c_int) }
    };
}
unsafe extern "C" fn getNextActiveSensor(mut currentSensor: uint8_t)
 -> uint8_t {
    loop  {
        currentSensor = currentSensor.wrapping_add(1);
        if !((currentSensor as libc::c_ulong) <
                 (::core::mem::size_of::<[exBusSensor_t; 10]>() as
                      libc::c_ulong).wrapping_div(::core::mem::size_of::<exBusSensor_t>()
                                                      as libc::c_ulong)) {
            break ;
        }
        if bitArrayGet(&mut exSensorEnabled as *mut uint32_t as
                           *const libc::c_void, currentSensor as libc::c_uint)
           {
            break ;
        }
    }
    if currentSensor as libc::c_ulong ==
           (::core::mem::size_of::<[exBusSensor_t; 10]>() as
                libc::c_ulong).wrapping_div(::core::mem::size_of::<exBusSensor_t>()
                                                as libc::c_ulong) {
        currentSensor = firstActiveSensor
    }
    return currentSensor;
}
#[no_mangle]
pub unsafe extern "C" fn createExTelemetryValueMessage(mut exMessage:
                                                           *mut uint8_t,
                                                       mut item: uint8_t)
 -> uint8_t {
    let mut startItem: uint8_t = item;
    let mut sensorItemMaxGroup: uint8_t =
        ((item as libc::c_int & 0xf0 as libc::c_int) + 0x10 as libc::c_int) as
            uint8_t;
    let mut iCount: uint8_t = 0;
    let mut messageSize: uint8_t = 0;
    let mut sensorValue: uint32_t = 0;
    *exMessage.offset(EXTEL_HEADER_LSN_LB as libc::c_int as isize) =
        (item as libc::c_int & 0xf0 as libc::c_int) as uint8_t;
    let mut p: *mut uint8_t =
        &mut *exMessage.offset(EXTEL_HEADER_ID as libc::c_int as isize) as
            *mut uint8_t;
    while (item as libc::c_int) < sensorItemMaxGroup as libc::c_int {
        let fresh0 = p;
        p = p.offset(1);
        *fresh0 =
            ((item as libc::c_int & 0xf as libc::c_int) << 4 as libc::c_int |
                 jetiExSensors[item as usize].exDataType as libc::c_int) as
                uint8_t;
        sensorValue = getSensorValue(item) as uint32_t;
        iCount =
            exDataTypeLen[jetiExSensors[item as usize].exDataType as usize];
        while iCount as libc::c_int > 1 as libc::c_int {
            let fresh1 = p;
            p = p.offset(1);
            *fresh1 = sensorValue as uint8_t;
            sensorValue = sensorValue >> 8 as libc::c_int;
            iCount = iCount.wrapping_sub(1)
        }
        let fresh2 = p;
        p = p.offset(1);
        *fresh2 =
            (sensorValue & 0x9f as libc::c_int as libc::c_uint |
                 jetiExSensors[item as usize].decimals as libc::c_uint) as
                uint8_t;
        item = getNextActiveSensor(item);
        if startItem as libc::c_int >= item as libc::c_int { break ; }
        if p.wrapping_offset_from(&mut *exMessage.offset(EXTEL_HEADER_ID as
                                                             libc::c_int as
                                                             isize) as
                                      *mut uint8_t) as libc::c_long +
               exDataTypeLen[jetiExSensors[item as usize].exDataType as usize]
                   as libc::c_long + 1 as libc::c_int as libc::c_long >=
               (28 as libc::c_int -
                    (1 as libc::c_int + 6 as libc::c_int + 1 as libc::c_int))
                   as libc::c_long {
            break ;
        }
    }
    messageSize =
        (6 as libc::c_int as libc::c_long +
             p.wrapping_offset_from(&mut *exMessage.offset(EXTEL_HEADER_ID as
                                                               libc::c_int as
                                                               isize) as
                                        *mut uint8_t) as libc::c_long) as
            uint8_t;
    *exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int as isize) =
        (0x40 as libc::c_int | messageSize as libc::c_int) as uint8_t;
    *exMessage.offset((messageSize as libc::c_int + 1 as libc::c_int) as
                          isize) =
        calcCRC8(&mut *exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int
                                            as isize), messageSize);
    return item;
    // return the next item
}
#[no_mangle]
pub unsafe extern "C" fn createExBusMessage(mut exBusMessage: *mut uint8_t,
                                            mut exMessage: *mut uint8_t,
                                            mut packetID: uint8_t) {
    let mut crc16: uint16_t = 0; // +2: startbyte & CRC8
    *exBusMessage.offset(EXBUS_HEADER_PACKET_ID as libc::c_int as isize) =
        packetID; // only for debug
    *exBusMessage.offset(EXBUS_HEADER_SUBLEN as libc::c_int as isize) =
        ((*exMessage.offset(EXTEL_HEADER_TYPE_LEN as libc::c_int as isize) as
              libc::c_int & 0x3f as libc::c_int) + 2 as libc::c_int) as
            uint8_t;
    *exBusMessage.offset(EXBUS_HEADER_MSG_LEN as libc::c_int as isize) =
        (6 as libc::c_int + 2 as libc::c_int +
             *exBusMessage.offset(EXBUS_HEADER_SUBLEN as libc::c_int as isize)
                 as libc::c_int) as uint8_t;
    crc16 =
        jetiExBusCalcCRC16(exBusMessage,
                           (*exBusMessage.offset(EXBUS_HEADER_MSG_LEN as
                                                     libc::c_int as isize) as
                                libc::c_int - 2 as libc::c_int) as uint8_t);
    *exBusMessage.offset((*exBusMessage.offset(EXBUS_HEADER_MSG_LEN as
                                                   libc::c_int as isize) as
                              libc::c_int - 2 as libc::c_int) as isize) =
        crc16 as uint8_t;
    *exBusMessage.offset((*exBusMessage.offset(EXBUS_HEADER_MSG_LEN as
                                                   libc::c_int as isize) as
                              libc::c_int - 1 as libc::c_int) as isize) =
        (crc16 as libc::c_int >> 8 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn checkJetiExBusTelemetryState() { }
#[no_mangle]
pub unsafe extern "C" fn handleJetiExBusTelemetry() {
    static mut framesLost: uint16_t = 0 as libc::c_int as uint16_t;
    static mut item: uint8_t = 0 as libc::c_int as uint8_t;
    let mut timeDiff: uint32_t = 0;
    // Check if we shall reset frame position due to time
    if jetiExBusRequestState as libc::c_int ==
           EXBUS_STATE_RECEIVED as libc::c_int {
        // to prevent timing issues from request to answer - max. 4ms
        timeDiff = micros().wrapping_sub(jetiTimeStampRequest);
        if timeDiff > 3000 as libc::c_int as libc::c_uint {
            // include reserved time
            jetiExBusRequestState =
                EXBUS_STATE_ZERO as libc::c_int as uint8_t;
            framesLost = framesLost.wrapping_add(1);
            return
        }
        if jetiExBusRequestFrame[EXBUS_HEADER_DATA_ID as libc::c_int as usize]
               as libc::c_int == 0x3a as libc::c_int &&
               jetiExBusCalcCRC16(jetiExBusRequestFrame.as_mut_ptr(),
                                  jetiExBusRequestFrame[EXBUS_HEADER_MSG_LEN
                                                            as libc::c_int as
                                                            usize]) as
                   libc::c_int == 0 as libc::c_int {
            // switch to TX mode
            if serialRxBytesWaiting(jetiExBusPort) ==
                   0 as libc::c_int as libc::c_uint {
                serialSetMode(jetiExBusPort, MODE_TX);
                jetiExBusTransceiveState =
                    EXBUS_TRANS_TX as libc::c_int as uint8_t;
                item =
                    sendJetiExBusTelemetry(jetiExBusRequestFrame[EXBUS_HEADER_PACKET_ID
                                                                     as
                                                                     libc::c_int
                                                                     as
                                                                     usize],
                                           item);
                jetiExBusRequestState =
                    EXBUS_STATE_PROCESSED as libc::c_int as uint8_t;
                return
            }
        } else {
            jetiExBusRequestState =
                EXBUS_STATE_ZERO as libc::c_int as uint8_t;
            return
        }
    }
    // check the state if transmit is ready
    if jetiExBusTransceiveState as libc::c_int ==
           EXBUS_TRANS_IS_TX_COMPLETED as libc::c_int {
        if isSerialTransmitBufferEmpty(jetiExBusPort) {
            serialSetMode(jetiExBusPort, MODE_RX);
            jetiExBusTransceiveState =
                EXBUS_TRANS_RX as libc::c_int as uint8_t;
            jetiExBusRequestState = EXBUS_STATE_ZERO as libc::c_int as uint8_t
        }
    };
}
unsafe extern "C" fn sendJetiExBusTelemetry(mut packetID: uint8_t,
                                            mut item: uint8_t) -> uint8_t {
    static mut sensorDescriptionCounter: uint8_t =
        0xff as libc::c_int as uint8_t;
    static mut requestLoop: uint8_t = 0xff as libc::c_int as uint8_t;
    let mut jetiExTelemetryFrame: *mut uint8_t =
        &mut *jetiExBusTelemetryFrame.as_mut_ptr().offset(EXBUS_HEADER_DATA as
                                                              libc::c_int as
                                                              isize) as
            *mut uint8_t;
    if requestLoop != 0 {
        loop  {
            sensorDescriptionCounter =
                sensorDescriptionCounter.wrapping_add(1);
            if !((sensorDescriptionCounter as libc::c_ulong) <
                     (::core::mem::size_of::<[exBusSensor_t; 10]>() as
                          libc::c_ulong).wrapping_div(::core::mem::size_of::<exBusSensor_t>()
                                                          as libc::c_ulong)) {
                break ;
            }
            if bitArrayGet(&mut exSensorEnabled as *mut uint32_t as
                               *const libc::c_void,
                           sensorDescriptionCounter as libc::c_uint) as
                   libc::c_int != 0 ||
                   jetiExSensors[sensorDescriptionCounter as usize].exDataType
                       as libc::c_int == EX_TYPE_DES as libc::c_int {
                break ;
            }
        }
        if sensorDescriptionCounter as libc::c_ulong ==
               (::core::mem::size_of::<[exBusSensor_t; 10]>() as
                    libc::c_ulong).wrapping_div(::core::mem::size_of::<exBusSensor_t>()
                                                    as libc::c_ulong) {
            sensorDescriptionCounter = 0 as libc::c_int as uint8_t
        }
        createExTelemetryTextMessage(jetiExTelemetryFrame,
                                     sensorDescriptionCounter,
                                     &*jetiExSensors.as_ptr().offset(sensorDescriptionCounter
                                                                         as
                                                                         isize));
        createExBusMessage(jetiExBusTelemetryFrame.as_mut_ptr(),
                           jetiExTelemetryFrame, packetID);
        requestLoop = requestLoop.wrapping_sub(1);
        if requestLoop as libc::c_int == 0 as libc::c_int {
            item = firstActiveSensor
        }
    } else {
        item = createExTelemetryValueMessage(jetiExTelemetryFrame, item);
        createExBusMessage(jetiExBusTelemetryFrame.as_mut_ptr(),
                           jetiExTelemetryFrame, packetID);
    }
    serialWriteBuf(jetiExBusPort, jetiExBusTelemetryFrame.as_mut_ptr(),
                   jetiExBusTelemetryFrame[EXBUS_HEADER_MSG_LEN as libc::c_int
                                               as usize] as libc::c_int);
    jetiExBusTransceiveState =
        EXBUS_TRANS_IS_TX_COMPLETED as libc::c_int as uint8_t;
    return item;
}
// SERIAL_RX
// TELEMETRY
