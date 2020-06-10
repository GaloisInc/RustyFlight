use ::libc;
use ::c2rust_bitfields;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn millis() -> timeMs_t;
    #[no_mangle]
    fn adcGetChannel(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn setRssiDirect(newRssi: uint16_t, source: rssiSource_e);
    // Flush the RX FIFO buffer.
    // Flush the TX FIFO buffer.
    // Reset real time clock.
    // No operation. May be used to pad strobe commands to two
         // bytes for simpler software.
    //----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------
    // Bit fields in the chip status byte
    // Chip states
    //----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
    #[no_mangle]
    fn cc2500ReadFifo(dpbuffer: *mut uint8_t, len: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500WriteFifo(dpbuffer: *mut uint8_t, len: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500ReadReg(reg: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500Strobe(address: uint8_t);
    #[no_mangle]
    fn cc2500SetPower(power: uint8_t);
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
    // preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
    // expand pinid to to ioTag_t
    // TODO
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    static mut rxFrSkySpiConfig_System: rxFrSkySpiConfig_t;
    #[no_mangle]
    static mut listLength: uint8_t;
    #[no_mangle]
    static mut missingPackets: uint32_t;
    #[no_mangle]
    static mut timeoutUs: timeDelta_t;
    #[no_mangle]
    static mut rssiDbm: int16_t;
    #[no_mangle]
    static mut gdoPin: IO_t;
    #[no_mangle]
    fn setRssiDbm(value: uint8_t);
    #[no_mangle]
    fn TxEnable();
    #[no_mangle]
    fn TxDisable();
    #[no_mangle]
    fn LedOn();
    #[no_mangle]
    fn LedOff();
    #[no_mangle]
    fn switchAntennae();
    #[no_mangle]
    fn initialiseData(adr: uint8_t);
    #[no_mangle]
    fn checkBindRequested(reset: bool) -> bool;
    #[no_mangle]
    fn nextChannel(skip: uint8_t);
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn initSmartPortTelemetryExternal(smartPortWriteFrameExternal:
                                          Option<smartPortWriteFrameFn>)
     -> bool;
    #[no_mangle]
    fn processSmartPortTelemetry(payload: *mut smartPortPayload_t,
                                 hasRequest: *mut bool,
                                 requestTimeout: *const uint32_t);
    #[no_mangle]
    fn smartPortDataReceive(c: uint16_t, clearToSend: *mut bool,
                            checkQueueEmpty:
                                Option<smartPortCheckQueueEmptyFn>,
                            withChecksum: bool) -> *mut smartPortPayload_t;
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
pub const DEBUG_COUNT: C2RustUnnamed = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed = 38;
pub const DEBUG_RTH: C2RustUnnamed = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed = 36;
pub const DEBUG_USB: C2RustUnnamed = 35;
pub const DEBUG_CURRENT: C2RustUnnamed = 34;
pub const DEBUG_SDIO: C2RustUnnamed = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed = 28;
pub const DEBUG_FPORT: C2RustUnnamed = 27;
pub const DEBUG_SBUS: C2RustUnnamed = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed = 16;
pub const DEBUG_FFT: C2RustUnnamed = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed = 12;
pub const DEBUG_STACK: C2RustUnnamed = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed = 3;
pub const DEBUG_BATTERY: C2RustUnnamed = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed = 1;
pub const DEBUG_NONE: C2RustUnnamed = 0;
pub type IO_t = *mut libc::c_void;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_0 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_0 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_0 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_0 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_0 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_0 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_0 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_0 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_0 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_0 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_0 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_0 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_0 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_0 = 8192;
pub const FEATURE_3D: C2RustUnnamed_0 = 4096;
pub const FEATURE_TELEMETRY: C2RustUnnamed_0 = 1024;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_0 = 512;
pub const FEATURE_GPS: C2RustUnnamed_0 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_0 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_0 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_0 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_0 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_0 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_0 = 1;
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
// time difference, 32 bits always sufficient
pub type timeDelta_t = int32_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const ADC_CHANNEL_COUNT: C2RustUnnamed_1 = 4;
pub const ADC_RSSI: C2RustUnnamed_1 = 3;
pub const ADC_EXTERNAL1: C2RustUnnamed_1 = 2;
pub const ADC_CURRENT: C2RustUnnamed_1 = 1;
pub const ADC_BATTERY: C2RustUnnamed_1 = 0;
pub type rssiSource_e = libc::c_uint;
pub const RSSI_SOURCE_FRAME_ERRORS: rssiSource_e = 5;
pub const RSSI_SOURCE_MSP: rssiSource_e = 4;
pub const RSSI_SOURCE_RX_PROTOCOL: rssiSource_e = 3;
pub const RSSI_SOURCE_RX_CHANNEL: rssiSource_e = 2;
pub const RSSI_SOURCE_ADC: rssiSource_e = 1;
pub const RSSI_SOURCE_NONE: rssiSource_e = 0;
pub type rx_spi_received_e = libc::c_uint;
pub const RX_SPI_RECEIVED_DATA: rx_spi_received_e = 2;
pub const RX_SPI_RECEIVED_BIND: rx_spi_received_e = 1;
pub const RX_SPI_RECEIVED_NONE: rx_spi_received_e = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const CC2500_3F_RXFIFO: C2RustUnnamed_2 = 63;
pub const CC2500_3F_TXFIFO: C2RustUnnamed_2 = 63;
pub const CC2500_3E_PATABLE: C2RustUnnamed_2 = 62;
pub const CC2500_3B_RXBYTES: C2RustUnnamed_2 = 59;
pub const CC2500_3A_TXBYTES: C2RustUnnamed_2 = 58;
pub const CC2500_39_VCO_VC_DAC: C2RustUnnamed_2 = 57;
pub const CC2500_38_PKTSTATUS: C2RustUnnamed_2 = 56;
pub const CC2500_37_WORTIME0: C2RustUnnamed_2 = 55;
pub const CC2500_36_WORTIME1: C2RustUnnamed_2 = 54;
pub const CC2500_35_MARCSTATE: C2RustUnnamed_2 = 53;
pub const CC2500_34_RSSI: C2RustUnnamed_2 = 52;
pub const CC2500_33_LQI: C2RustUnnamed_2 = 51;
pub const CC2500_32_FREQEST: C2RustUnnamed_2 = 50;
pub const CC2500_31_VERSION: C2RustUnnamed_2 = 49;
pub const CC2500_30_PARTNUM: C2RustUnnamed_2 = 48;
pub const CC2500_2E_TEST0: C2RustUnnamed_2 = 46;
pub const CC2500_2D_TEST1: C2RustUnnamed_2 = 45;
pub const CC2500_2C_TEST2: C2RustUnnamed_2 = 44;
pub const CC2500_2B_AGCTEST: C2RustUnnamed_2 = 43;
pub const CC2500_2A_PTEST: C2RustUnnamed_2 = 42;
pub const CC2500_29_FSTEST: C2RustUnnamed_2 = 41;
pub const CC2500_28_RCCTRL0: C2RustUnnamed_2 = 40;
pub const CC2500_27_RCCTRL1: C2RustUnnamed_2 = 39;
pub const CC2500_26_FSCAL0: C2RustUnnamed_2 = 38;
pub const CC2500_25_FSCAL1: C2RustUnnamed_2 = 37;
pub const CC2500_24_FSCAL2: C2RustUnnamed_2 = 36;
pub const CC2500_23_FSCAL3: C2RustUnnamed_2 = 35;
pub const CC2500_22_FREND0: C2RustUnnamed_2 = 34;
pub const CC2500_21_FREND1: C2RustUnnamed_2 = 33;
pub const CC2500_20_WORCTRL: C2RustUnnamed_2 = 32;
pub const CC2500_1F_WOREVT0: C2RustUnnamed_2 = 31;
pub const CC2500_1E_WOREVT1: C2RustUnnamed_2 = 30;
pub const CC2500_1D_AGCCTRL0: C2RustUnnamed_2 = 29;
pub const CC2500_1C_AGCCTRL1: C2RustUnnamed_2 = 28;
pub const CC2500_1B_AGCCTRL2: C2RustUnnamed_2 = 27;
pub const CC2500_1A_BSCFG: C2RustUnnamed_2 = 26;
pub const CC2500_19_FOCCFG: C2RustUnnamed_2 = 25;
pub const CC2500_18_MCSM0: C2RustUnnamed_2 = 24;
pub const CC2500_17_MCSM1: C2RustUnnamed_2 = 23;
pub const CC2500_16_MCSM2: C2RustUnnamed_2 = 22;
pub const CC2500_15_DEVIATN: C2RustUnnamed_2 = 21;
pub const CC2500_14_MDMCFG0: C2RustUnnamed_2 = 20;
pub const CC2500_13_MDMCFG1: C2RustUnnamed_2 = 19;
pub const CC2500_12_MDMCFG2: C2RustUnnamed_2 = 18;
pub const CC2500_11_MDMCFG3: C2RustUnnamed_2 = 17;
pub const CC2500_10_MDMCFG4: C2RustUnnamed_2 = 16;
pub const CC2500_0F_FREQ0: C2RustUnnamed_2 = 15;
pub const CC2500_0E_FREQ1: C2RustUnnamed_2 = 14;
pub const CC2500_0D_FREQ2: C2RustUnnamed_2 = 13;
pub const CC2500_0C_FSCTRL0: C2RustUnnamed_2 = 12;
pub const CC2500_0B_FSCTRL1: C2RustUnnamed_2 = 11;
pub const CC2500_0A_CHANNR: C2RustUnnamed_2 = 10;
pub const CC2500_09_ADDR: C2RustUnnamed_2 = 9;
pub const CC2500_08_PKTCTRL0: C2RustUnnamed_2 = 8;
pub const CC2500_07_PKTCTRL1: C2RustUnnamed_2 = 7;
pub const CC2500_06_PKTLEN: C2RustUnnamed_2 = 6;
pub const CC2500_05_SYNC0: C2RustUnnamed_2 = 5;
pub const CC2500_04_SYNC1: C2RustUnnamed_2 = 4;
pub const CC2500_03_FIFOTHR: C2RustUnnamed_2 = 3;
pub const CC2500_02_IOCFG0: C2RustUnnamed_2 = 2;
pub const CC2500_01_IOCFG1: C2RustUnnamed_2 = 1;
pub const CC2500_00_IOCFG2: C2RustUnnamed_2 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxFrSkySpiConfig_s {
    pub autoBind: uint8_t,
    pub bindTxId: [uint8_t; 2],
    pub bindOffset: int8_t,
    pub bindHopData: [uint8_t; 50],
    pub rxNum: uint8_t,
    pub useExternalAdc: uint8_t,
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
pub type rxFrSkySpiConfig_t = rxFrSkySpiConfig_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const STATE_RESUME: C2RustUnnamed_3 = 10;
pub const STATE_TELEMETRY: C2RustUnnamed_3 = 9;
pub const STATE_DATA: C2RustUnnamed_3 = 8;
pub const STATE_UPDATE: C2RustUnnamed_3 = 7;
pub const STATE_STARTING: C2RustUnnamed_3 = 6;
pub const STATE_BIND_COMPLETE: C2RustUnnamed_3 = 5;
pub const STATE_BIND_BINDING2: C2RustUnnamed_3 = 4;
pub const STATE_BIND_BINDING1: C2RustUnnamed_3 = 3;
pub const STATE_BIND_TUNING: C2RustUnnamed_3 = 2;
pub const STATE_BIND: C2RustUnnamed_3 = 1;
pub const STATE_INIT: C2RustUnnamed_3 = 0;
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
 * smartport.h
 *
 *  Created on: 25 October 2014
 *      Author: Frank26080115
 */
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FSSP_SENSOR_ID4: C2RustUnnamed_4 = 103;
pub const FSSP_SENSOR_ID3: C2RustUnnamed_4 = 52;
pub const FSSP_SENSOR_ID2: C2RustUnnamed_4 = 13;
// there are 32 ID's polled by smartport master
    // remaining 3 bits are crc (according to comments in openTx code)
// MSP server frame
// ID of sensor. Must be something that is polled by FrSky RX
pub const FSSP_SENSOR_ID1: C2RustUnnamed_4 = 27;
// MSP client frame
pub const FSSP_MSPS_FRAME: C2RustUnnamed_4 = 50;
// MSP client frame
pub const FSSP_MSPC_FRAME_FPORT: C2RustUnnamed_4 = 49;
pub const FSSP_MSPC_FRAME_SMARTPORT: C2RustUnnamed_4 = 48;
pub const FSSP_DATA_FRAME: C2RustUnnamed_4 = 16;
pub const FSSP_DLE_XOR: C2RustUnnamed_4 = 32;
pub const FSSP_DLE: C2RustUnnamed_4 = 125;
pub const FSSP_START_STOP: C2RustUnnamed_4 = 126;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct smartPortPayload_s {
    pub frameId: uint8_t,
    pub valueId: uint16_t,
    pub data: uint32_t,
}
pub type smartPortPayload_t = smartPortPayload_s;
pub type smartPortWriteFrameFn
    =
    unsafe extern "C" fn(_: *const smartPortPayload_t) -> ();
pub type smartPortCheckQueueEmptyFn = unsafe extern "C" fn() -> bool;
pub type telemetryBuffer_t = telemetryBuffer_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct telemetryBuffer_s {
    pub data: telemetryData_t,
    pub needsProcessing: uint8_t,
}
pub type telemetryData_t = telemetryData_s;
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct telemetryData_s {
    pub dataLength: uint8_t,
    pub data: [uint8_t; 5],
}
pub type telemetrySequenceMarkerData_t = telemetrySequenceMarkerData_s;
#[derive(Copy, Clone, BitfieldStruct)]
#[repr(C, packed)]
pub struct telemetrySequenceMarkerData_s {
    #[bitfield(name = "packetSequenceId", ty = "libc::c_uint", bits =
               "0..=1")]
    #[bitfield(name = "unused", ty = "libc::c_uint", bits = "2..=2")]
    #[bitfield(name = "initRequest", ty = "libc::c_uint", bits = "3..=3")]
    #[bitfield(name = "ackSequenceId", ty = "libc::c_uint", bits = "4..=5")]
    #[bitfield(name = "retransmissionRequested", ty = "libc::c_uint", bits =
               "6..=6")]
    #[bitfield(name = "initResponse", ty = "libc::c_uint", bits = "7..=7")]
    pub packetSequenceId_unused_initRequest_ackSequenceId_retransmissionRequested_initResponse: [u8; 1],
}
pub type telemetrySequenceMarker_t = telemetrySequenceMarker_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union telemetrySequenceMarker_s {
    pub raw: uint8_t,
    pub data: telemetrySequenceMarkerData_t,
}
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn rxFrSkySpiConfig() -> *const rxFrSkySpiConfig_t {
    return &mut rxFrSkySpiConfig_System;
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
pub static mut crcTable: [uint16_t; 256] =
    [0 as libc::c_int as uint16_t, 0x1189 as libc::c_int as uint16_t,
     0x2312 as libc::c_int as uint16_t, 0x329b as libc::c_int as uint16_t,
     0x4624 as libc::c_int as uint16_t, 0x57ad as libc::c_int as uint16_t,
     0x6536 as libc::c_int as uint16_t, 0x74bf as libc::c_int as uint16_t,
     0x8c48 as libc::c_int as uint16_t, 0x9dc1 as libc::c_int as uint16_t,
     0xaf5a as libc::c_int as uint16_t, 0xbed3 as libc::c_int as uint16_t,
     0xca6c as libc::c_int as uint16_t, 0xdbe5 as libc::c_int as uint16_t,
     0xe97e as libc::c_int as uint16_t, 0xf8f7 as libc::c_int as uint16_t,
     0x1081 as libc::c_int as uint16_t, 0x108 as libc::c_int as uint16_t,
     0x3393 as libc::c_int as uint16_t, 0x221a as libc::c_int as uint16_t,
     0x56a5 as libc::c_int as uint16_t, 0x472c as libc::c_int as uint16_t,
     0x75b7 as libc::c_int as uint16_t, 0x643e as libc::c_int as uint16_t,
     0x9cc9 as libc::c_int as uint16_t, 0x8d40 as libc::c_int as uint16_t,
     0xbfdb as libc::c_int as uint16_t, 0xae52 as libc::c_int as uint16_t,
     0xdaed as libc::c_int as uint16_t, 0xcb64 as libc::c_int as uint16_t,
     0xf9ff as libc::c_int as uint16_t, 0xe876 as libc::c_int as uint16_t,
     0x2102 as libc::c_int as uint16_t, 0x308b as libc::c_int as uint16_t,
     0x210 as libc::c_int as uint16_t, 0x1399 as libc::c_int as uint16_t,
     0x6726 as libc::c_int as uint16_t, 0x76af as libc::c_int as uint16_t,
     0x4434 as libc::c_int as uint16_t, 0x55bd as libc::c_int as uint16_t,
     0xad4a as libc::c_int as uint16_t, 0xbcc3 as libc::c_int as uint16_t,
     0x8e58 as libc::c_int as uint16_t, 0x9fd1 as libc::c_int as uint16_t,
     0xeb6e as libc::c_int as uint16_t, 0xfae7 as libc::c_int as uint16_t,
     0xc87c as libc::c_int as uint16_t, 0xd9f5 as libc::c_int as uint16_t,
     0x3183 as libc::c_int as uint16_t, 0x200a as libc::c_int as uint16_t,
     0x1291 as libc::c_int as uint16_t, 0x318 as libc::c_int as uint16_t,
     0x77a7 as libc::c_int as uint16_t, 0x662e as libc::c_int as uint16_t,
     0x54b5 as libc::c_int as uint16_t, 0x453c as libc::c_int as uint16_t,
     0xbdcb as libc::c_int as uint16_t, 0xac42 as libc::c_int as uint16_t,
     0x9ed9 as libc::c_int as uint16_t, 0x8f50 as libc::c_int as uint16_t,
     0xfbef as libc::c_int as uint16_t, 0xea66 as libc::c_int as uint16_t,
     0xd8fd as libc::c_int as uint16_t, 0xc974 as libc::c_int as uint16_t,
     0x4204 as libc::c_int as uint16_t, 0x538d as libc::c_int as uint16_t,
     0x6116 as libc::c_int as uint16_t, 0x709f as libc::c_int as uint16_t,
     0x420 as libc::c_int as uint16_t, 0x15a9 as libc::c_int as uint16_t,
     0x2732 as libc::c_int as uint16_t, 0x36bb as libc::c_int as uint16_t,
     0xce4c as libc::c_int as uint16_t, 0xdfc5 as libc::c_int as uint16_t,
     0xed5e as libc::c_int as uint16_t, 0xfcd7 as libc::c_int as uint16_t,
     0x8868 as libc::c_int as uint16_t, 0x99e1 as libc::c_int as uint16_t,
     0xab7a as libc::c_int as uint16_t, 0xbaf3 as libc::c_int as uint16_t,
     0x5285 as libc::c_int as uint16_t, 0x430c as libc::c_int as uint16_t,
     0x7197 as libc::c_int as uint16_t, 0x601e as libc::c_int as uint16_t,
     0x14a1 as libc::c_int as uint16_t, 0x528 as libc::c_int as uint16_t,
     0x37b3 as libc::c_int as uint16_t, 0x263a as libc::c_int as uint16_t,
     0xdecd as libc::c_int as uint16_t, 0xcf44 as libc::c_int as uint16_t,
     0xfddf as libc::c_int as uint16_t, 0xec56 as libc::c_int as uint16_t,
     0x98e9 as libc::c_int as uint16_t, 0x8960 as libc::c_int as uint16_t,
     0xbbfb as libc::c_int as uint16_t, 0xaa72 as libc::c_int as uint16_t,
     0x6306 as libc::c_int as uint16_t, 0x728f as libc::c_int as uint16_t,
     0x4014 as libc::c_int as uint16_t, 0x519d as libc::c_int as uint16_t,
     0x2522 as libc::c_int as uint16_t, 0x34ab as libc::c_int as uint16_t,
     0x630 as libc::c_int as uint16_t, 0x17b9 as libc::c_int as uint16_t,
     0xef4e as libc::c_int as uint16_t, 0xfec7 as libc::c_int as uint16_t,
     0xcc5c as libc::c_int as uint16_t, 0xddd5 as libc::c_int as uint16_t,
     0xa96a as libc::c_int as uint16_t, 0xb8e3 as libc::c_int as uint16_t,
     0x8a78 as libc::c_int as uint16_t, 0x9bf1 as libc::c_int as uint16_t,
     0x7387 as libc::c_int as uint16_t, 0x620e as libc::c_int as uint16_t,
     0x5095 as libc::c_int as uint16_t, 0x411c as libc::c_int as uint16_t,
     0x35a3 as libc::c_int as uint16_t, 0x242a as libc::c_int as uint16_t,
     0x16b1 as libc::c_int as uint16_t, 0x738 as libc::c_int as uint16_t,
     0xffcf as libc::c_int as uint16_t, 0xee46 as libc::c_int as uint16_t,
     0xdcdd as libc::c_int as uint16_t, 0xcd54 as libc::c_int as uint16_t,
     0xb9eb as libc::c_int as uint16_t, 0xa862 as libc::c_int as uint16_t,
     0x9af9 as libc::c_int as uint16_t, 0x8b70 as libc::c_int as uint16_t,
     0x8408 as libc::c_int as uint16_t, 0x9581 as libc::c_int as uint16_t,
     0xa71a as libc::c_int as uint16_t, 0xb693 as libc::c_int as uint16_t,
     0xc22c as libc::c_int as uint16_t, 0xd3a5 as libc::c_int as uint16_t,
     0xe13e as libc::c_int as uint16_t, 0xf0b7 as libc::c_int as uint16_t,
     0x840 as libc::c_int as uint16_t, 0x19c9 as libc::c_int as uint16_t,
     0x2b52 as libc::c_int as uint16_t, 0x3adb as libc::c_int as uint16_t,
     0x4e64 as libc::c_int as uint16_t, 0x5fed as libc::c_int as uint16_t,
     0x6d76 as libc::c_int as uint16_t, 0x7cff as libc::c_int as uint16_t,
     0x9489 as libc::c_int as uint16_t, 0x8500 as libc::c_int as uint16_t,
     0xb79b as libc::c_int as uint16_t, 0xa612 as libc::c_int as uint16_t,
     0xd2ad as libc::c_int as uint16_t, 0xc324 as libc::c_int as uint16_t,
     0xf1bf as libc::c_int as uint16_t, 0xe036 as libc::c_int as uint16_t,
     0x18c1 as libc::c_int as uint16_t, 0x948 as libc::c_int as uint16_t,
     0x3bd3 as libc::c_int as uint16_t, 0x2a5a as libc::c_int as uint16_t,
     0x5ee5 as libc::c_int as uint16_t, 0x4f6c as libc::c_int as uint16_t,
     0x7df7 as libc::c_int as uint16_t, 0x6c7e as libc::c_int as uint16_t,
     0xa50a as libc::c_int as uint16_t, 0xb483 as libc::c_int as uint16_t,
     0x8618 as libc::c_int as uint16_t, 0x9791 as libc::c_int as uint16_t,
     0xe32e as libc::c_int as uint16_t, 0xf2a7 as libc::c_int as uint16_t,
     0xc03c as libc::c_int as uint16_t, 0xd1b5 as libc::c_int as uint16_t,
     0x2942 as libc::c_int as uint16_t, 0x38cb as libc::c_int as uint16_t,
     0xa50 as libc::c_int as uint16_t, 0x1bd9 as libc::c_int as uint16_t,
     0x6f66 as libc::c_int as uint16_t, 0x7eef as libc::c_int as uint16_t,
     0x4c74 as libc::c_int as uint16_t, 0x5dfd as libc::c_int as uint16_t,
     0xb58b as libc::c_int as uint16_t, 0xa402 as libc::c_int as uint16_t,
     0x9699 as libc::c_int as uint16_t, 0x8710 as libc::c_int as uint16_t,
     0xf3af as libc::c_int as uint16_t, 0xe226 as libc::c_int as uint16_t,
     0xd0bd as libc::c_int as uint16_t, 0xc134 as libc::c_int as uint16_t,
     0x39c3 as libc::c_int as uint16_t, 0x284a as libc::c_int as uint16_t,
     0x1ad1 as libc::c_int as uint16_t, 0xb58 as libc::c_int as uint16_t,
     0x7fe7 as libc::c_int as uint16_t, 0x6e6e as libc::c_int as uint16_t,
     0x5cf5 as libc::c_int as uint16_t, 0x4d7c as libc::c_int as uint16_t,
     0xc60c as libc::c_int as uint16_t, 0xd785 as libc::c_int as uint16_t,
     0xe51e as libc::c_int as uint16_t, 0xf497 as libc::c_int as uint16_t,
     0x8028 as libc::c_int as uint16_t, 0x91a1 as libc::c_int as uint16_t,
     0xa33a as libc::c_int as uint16_t, 0xb2b3 as libc::c_int as uint16_t,
     0x4a44 as libc::c_int as uint16_t, 0x5bcd as libc::c_int as uint16_t,
     0x6956 as libc::c_int as uint16_t, 0x78df as libc::c_int as uint16_t,
     0xc60 as libc::c_int as uint16_t, 0x1de9 as libc::c_int as uint16_t,
     0x2f72 as libc::c_int as uint16_t, 0x3efb as libc::c_int as uint16_t,
     0xd68d as libc::c_int as uint16_t, 0xc704 as libc::c_int as uint16_t,
     0xf59f as libc::c_int as uint16_t, 0xe416 as libc::c_int as uint16_t,
     0x90a9 as libc::c_int as uint16_t, 0x8120 as libc::c_int as uint16_t,
     0xb3bb as libc::c_int as uint16_t, 0xa232 as libc::c_int as uint16_t,
     0x5ac5 as libc::c_int as uint16_t, 0x4b4c as libc::c_int as uint16_t,
     0x79d7 as libc::c_int as uint16_t, 0x685e as libc::c_int as uint16_t,
     0x1ce1 as libc::c_int as uint16_t, 0xd68 as libc::c_int as uint16_t,
     0x3ff3 as libc::c_int as uint16_t, 0x2e7a as libc::c_int as uint16_t,
     0xe70e as libc::c_int as uint16_t, 0xf687 as libc::c_int as uint16_t,
     0xc41c as libc::c_int as uint16_t, 0xd595 as libc::c_int as uint16_t,
     0xa12a as libc::c_int as uint16_t, 0xb0a3 as libc::c_int as uint16_t,
     0x8238 as libc::c_int as uint16_t, 0x93b1 as libc::c_int as uint16_t,
     0x6b46 as libc::c_int as uint16_t, 0x7acf as libc::c_int as uint16_t,
     0x4854 as libc::c_int as uint16_t, 0x59dd as libc::c_int as uint16_t,
     0x2d62 as libc::c_int as uint16_t, 0x3ceb as libc::c_int as uint16_t,
     0xe70 as libc::c_int as uint16_t, 0x1ff9 as libc::c_int as uint16_t,
     0xf78f as libc::c_int as uint16_t, 0xe606 as libc::c_int as uint16_t,
     0xd49d as libc::c_int as uint16_t, 0xc514 as libc::c_int as uint16_t,
     0xb1ab as libc::c_int as uint16_t, 0xa022 as libc::c_int as uint16_t,
     0x92b9 as libc::c_int as uint16_t, 0x8330 as libc::c_int as uint16_t,
     0x7bc7 as libc::c_int as uint16_t, 0x6a4e as libc::c_int as uint16_t,
     0x58d5 as libc::c_int as uint16_t, 0x495c as libc::c_int as uint16_t,
     0x3de3 as libc::c_int as uint16_t, 0x2c6a as libc::c_int as uint16_t,
     0x1ef1 as libc::c_int as uint16_t, 0xf78 as libc::c_int as uint16_t];
static mut telemetryTxBuffer: [telemetryData_t; 4] =
    [telemetryData_t{dataLength: 0, data: [0; 5],}; 4];
static mut responseToSend: telemetrySequenceMarker_t =
    telemetrySequenceMarker_s{raw: 0,};
static mut frame: [uint8_t; 20] = [0; 20];
static mut telemetryOutWriter: uint8_t = 0;
static mut telemetryOutBuffer: [uint8_t; 64] = [0; 64];
static mut telemetryEnabled: bool = 0 as libc::c_int != 0;
// USE_RX_FRSKY_SPI_TELEMETRY
unsafe extern "C" fn calculateCrc(mut data: *mut uint8_t, mut len: uint8_t)
 -> uint16_t {
    let mut crc: uint16_t = 0 as libc::c_int as uint16_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < len as libc::c_uint {
        let fresh0 = data;
        data = data.offset(1);
        crc =
            ((crc as libc::c_int) << 8 as libc::c_int ^
                 crcTable[(((crc as libc::c_int >> 8 as libc::c_int) as
                                uint8_t as libc::c_int ^
                                *fresh0 as libc::c_int) & 0xff as libc::c_int)
                              as usize] as libc::c_int) as uint16_t;
        i = i.wrapping_add(1)
    }
    return crc;
}
unsafe extern "C" fn appendSmartPortData(mut buf: *mut uint8_t) -> uint8_t {
    static mut telemetryOutReader: uint8_t = 0 as libc::c_int as uint8_t;
    let mut index: uint8_t = 0;
    index = 0 as libc::c_int as uint8_t;
    while (index as libc::c_int) < 5 as libc::c_int {
        // max 5 bytes in a frame
        if telemetryOutReader as libc::c_int ==
               telemetryOutWriter as libc::c_int {
            break ; //length
        }
        *buf.offset(index as isize) =
            telemetryOutBuffer[telemetryOutReader as usize];
        telemetryOutReader =
            ((telemetryOutReader as libc::c_int + 1 as libc::c_int) %
                 64 as libc::c_int) as uint8_t;
        index = index.wrapping_add(1)
    }
    return index;
}
unsafe extern "C" fn buildTelemetryFrame(mut packet: *mut uint8_t) {
    static mut localPacketId: uint8_t = 0;
    static mut evenRun: bool = 0 as libc::c_int != 0;
    frame[0 as libc::c_int as usize] = 0xe as libc::c_int as uint8_t;
    frame[1 as libc::c_int as usize] =
        (*rxFrSkySpiConfig()).bindTxId[0 as libc::c_int as usize];
    frame[2 as libc::c_int as usize] =
        (*rxFrSkySpiConfig()).bindTxId[1 as libc::c_int as usize];
    frame[3 as libc::c_int as usize] =
        *packet.offset(3 as libc::c_int as isize);
    if evenRun {
        frame[4 as libc::c_int as usize] =
            (rssiDbm as uint8_t as libc::c_int | 0x80 as libc::c_int) as
                uint8_t
    } else {
        let mut a1Value: uint8_t = 0;
        if (*rxFrSkySpiConfig()).useExternalAdc != 0 {
            a1Value =
                ((adcGetChannel(ADC_EXTERNAL1 as libc::c_int as uint8_t) as
                      libc::c_int & 0xfe0 as libc::c_int) >> 5 as libc::c_int)
                    as uint8_t
        } else {
            a1Value =
                (getBatteryVoltage() as libc::c_int & 0x7f as libc::c_int) as
                    uint8_t
        }
        frame[4 as libc::c_int as usize] = a1Value
    }
    evenRun = !evenRun;
    let mut inFrameMarker: *mut telemetrySequenceMarker_t =
        &mut *packet.offset(21 as libc::c_int as isize) as *mut uint8_t as
            *mut telemetrySequenceMarker_t;
    let mut outFrameMarker: *mut telemetrySequenceMarker_t =
        &mut *frame.as_mut_ptr().offset(5 as libc::c_int as isize) as
            *mut uint8_t as *mut telemetrySequenceMarker_t;
    if (*inFrameMarker).data.initRequest() != 0 {
        // check syncronization at startup ok if not no sport telemetry
        (*outFrameMarker).raw = 0 as libc::c_int as uint8_t;
        (*outFrameMarker).data.set_initRequest(1 as libc::c_int as
                                                   libc::c_uint);
        (*outFrameMarker).data.set_initResponse(1 as libc::c_int as
                                                    libc::c_uint);
        localPacketId = 0 as libc::c_int as uint8_t
    } else if (*inFrameMarker).data.retransmissionRequested() != 0 {
        let mut retransmissionFrameId: uint8_t =
            (*inFrameMarker).data.ackSequenceId() as uint8_t;
        (*outFrameMarker).raw =
            (responseToSend.raw as libc::c_int & 0xf0 as libc::c_int) as
                uint8_t;
        (*outFrameMarker).data.set_packetSequenceId(retransmissionFrameId as
                                                        libc::c_uint);
        memcpy(&mut *frame.as_mut_ptr().offset(6 as libc::c_int as isize) as
                   *mut uint8_t as *mut libc::c_void,
               &mut *telemetryTxBuffer.as_mut_ptr().offset(retransmissionFrameId
                                                               as isize) as
                   *mut telemetryData_t as *const libc::c_void,
               ::core::mem::size_of::<telemetryData_t>() as libc::c_ulong);
    } else {
        let mut localAckId: uint8_t =
            (*inFrameMarker).data.ackSequenceId() as uint8_t;
        if localPacketId as libc::c_int !=
               (localAckId as libc::c_int + 1 as libc::c_int) %
                   4 as libc::c_int {
            (*outFrameMarker).raw =
                (responseToSend.raw as libc::c_int & 0xf0 as libc::c_int) as
                    uint8_t;
            (*outFrameMarker).data.set_packetSequenceId(localPacketId as
                                                            libc::c_uint);
            frame[6 as libc::c_int as usize] =
                appendSmartPortData(&mut *frame.as_mut_ptr().offset(7 as
                                                                        libc::c_int
                                                                        as
                                                                        isize));
            memcpy(&mut *telemetryTxBuffer.as_mut_ptr().offset(localPacketId
                                                                   as isize)
                       as *mut telemetryData_t as *mut libc::c_void,
                   &mut *frame.as_mut_ptr().offset(6 as libc::c_int as isize)
                       as *mut uint8_t as *const libc::c_void,
                   ::core::mem::size_of::<telemetryData_t>() as
                       libc::c_ulong);
            localPacketId =
                ((localPacketId as libc::c_int + 1 as libc::c_int) %
                     4 as libc::c_int) as uint8_t
        }
    }
    let mut lcrc: uint16_t =
        calculateCrc(&mut *frame.as_mut_ptr().offset(3 as libc::c_int as
                                                         isize),
                     10 as libc::c_int as uint8_t);
    frame[13 as libc::c_int as usize] =
        (lcrc as libc::c_int >> 8 as libc::c_int) as uint8_t;
    frame[14 as libc::c_int as usize] = lcrc as uint8_t;
}
unsafe extern "C" fn frSkyXCheckQueueEmpty() -> bool {
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn frSkyXTelemetrySendByte(mut c: uint8_t) {
    if c as libc::c_int == FSSP_DLE as libc::c_int ||
           c as libc::c_int == FSSP_START_STOP as libc::c_int {
        telemetryOutBuffer[telemetryOutWriter as usize] =
            FSSP_DLE as libc::c_int as uint8_t;
        telemetryOutWriter =
            ((telemetryOutWriter as libc::c_int + 1 as libc::c_int) %
                 64 as libc::c_int) as uint8_t;
        telemetryOutBuffer[telemetryOutWriter as usize] =
            (c as libc::c_int ^ FSSP_DLE_XOR as libc::c_int) as uint8_t;
        telemetryOutWriter =
            ((telemetryOutWriter as libc::c_int + 1 as libc::c_int) %
                 64 as libc::c_int) as uint8_t
    } else {
        telemetryOutBuffer[telemetryOutWriter as usize] = c;
        telemetryOutWriter =
            ((telemetryOutWriter as libc::c_int + 1 as libc::c_int) %
                 64 as libc::c_int) as uint8_t
    };
}
unsafe extern "C" fn frSkyXTelemetryWriteFrame(mut payload:
                                                   *const smartPortPayload_t) {
    telemetryOutBuffer[telemetryOutWriter as usize] =
        FSSP_START_STOP as libc::c_int as uint8_t;
    telemetryOutWriter =
        ((telemetryOutWriter as libc::c_int + 1 as libc::c_int) %
             64 as libc::c_int) as uint8_t;
    telemetryOutBuffer[telemetryOutWriter as usize] =
        (FSSP_SENSOR_ID1 as libc::c_int & 0x1f as libc::c_int) as uint8_t;
    telemetryOutWriter =
        ((telemetryOutWriter as libc::c_int + 1 as libc::c_int) %
             64 as libc::c_int) as uint8_t;
    let mut data: *mut uint8_t = payload as *mut uint8_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              ::core::mem::size_of::<smartPortPayload_t>() as libc::c_ulong {
        let fresh1 = data;
        data = data.offset(1);
        frSkyXTelemetrySendByte(*fresh1);
        i = i.wrapping_add(1)
    };
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
// USE_RX_FRSKY_SPI_TELEMETRY
#[no_mangle]
pub unsafe extern "C" fn frSkyXSetRcData(mut rcData: *mut uint16_t,
                                         mut packet: *const uint8_t) {
    let mut c: [uint16_t; 8] = [0; 8]; // again set for receive
    c[0 as libc::c_int as usize] =
        (((*packet.offset(10 as libc::c_int as isize) as libc::c_int) <<
              8 as libc::c_int & 0xf00 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(9 as libc::c_int as isize) as libc::c_int) as
            uint16_t;
    c[1 as libc::c_int as usize] =
        (((*packet.offset(11 as libc::c_int as isize) as libc::c_int) <<
              4 as libc::c_int & 0xff0 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(10 as libc::c_int as isize) as libc::c_int >>
                 4 as libc::c_int) as uint16_t;
    c[2 as libc::c_int as usize] =
        (((*packet.offset(13 as libc::c_int as isize) as libc::c_int) <<
              8 as libc::c_int & 0xf00 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(12 as libc::c_int as isize) as libc::c_int) as
            uint16_t;
    c[3 as libc::c_int as usize] =
        (((*packet.offset(14 as libc::c_int as isize) as libc::c_int) <<
              4 as libc::c_int & 0xff0 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(13 as libc::c_int as isize) as libc::c_int >>
                 4 as libc::c_int) as uint16_t;
    c[4 as libc::c_int as usize] =
        (((*packet.offset(16 as libc::c_int as isize) as libc::c_int) <<
              8 as libc::c_int & 0xf00 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(15 as libc::c_int as isize) as libc::c_int) as
            uint16_t;
    c[5 as libc::c_int as usize] =
        (((*packet.offset(17 as libc::c_int as isize) as libc::c_int) <<
              4 as libc::c_int & 0xff0 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(16 as libc::c_int as isize) as libc::c_int >>
                 4 as libc::c_int) as uint16_t;
    c[6 as libc::c_int as usize] =
        (((*packet.offset(19 as libc::c_int as isize) as libc::c_int) <<
              8 as libc::c_int & 0xf00 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(18 as libc::c_int as isize) as libc::c_int) as
            uint16_t;
    c[7 as libc::c_int as usize] =
        (((*packet.offset(20 as libc::c_int as isize) as libc::c_int) <<
              4 as libc::c_int & 0xff0 as libc::c_int) as uint16_t as
             libc::c_int |
             *packet.offset(19 as libc::c_int as isize) as libc::c_int >>
                 4 as libc::c_int) as uint16_t;
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 8 as libc::c_int as libc::c_uint {
        let mut channelIsShifted: bool = 0 as libc::c_int != 0;
        if c[i as usize] as libc::c_int & 0x800 as libc::c_int != 0 {
            channelIsShifted = 1 as libc::c_int != 0
        }
        let channelValue: uint16_t =
            (c[i as usize] as libc::c_int & 0x7ff as libc::c_int) as uint16_t;
        *rcData.offset(if channelIsShifted as libc::c_int != 0 {
                           i.wrapping_add(8 as libc::c_int as libc::c_uint)
                       } else { i } as isize) =
            (channelValue as libc::c_int as libc::c_float * 2.0f32 /
                 3 as libc::c_int as libc::c_float +
                 860 as libc::c_int as libc::c_float -
                 (64 as libc::c_int * 2 as libc::c_int / 3 as libc::c_int) as
                     libc::c_float) as uint16_t;
        i = i.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn frSkyXHandlePacket(packet: *mut uint8_t,
                                            protocolState: *mut uint8_t)
 -> rx_spi_received_e {
    static mut receiveTelemetryRetryCount: libc::c_uint =
        0 as libc::c_int as libc::c_uint;
    static mut pollingTimeMs: timeMs_t = 0 as libc::c_int as timeMs_t;
    static mut skipChannels: bool = 1 as libc::c_int != 0;
    static mut ledIsOn: bool = false;
    static mut remoteProcessedId: uint8_t = 0 as libc::c_int as uint8_t;
    static mut remoteAckId: uint8_t = 0 as libc::c_int as uint8_t;
    static mut remoteToProcessIndex: uint8_t = 0 as libc::c_int as uint8_t;
    static mut packetTimerUs: timeUs_t = 0;
    static mut frameReceived: bool = false;
    static mut receiveDelayUs: timeDelta_t = 0;
    static mut channelsToSkip: uint8_t = 1 as libc::c_int as uint8_t;
    static mut packetErrors: uint32_t = 0 as libc::c_int as uint32_t;
    static mut telemetryRxBuffer: [telemetryBuffer_t; 4] =
        [telemetryBuffer_t{data:
                               telemetryData_t{dataLength: 0, data: [0; 5],},
                           needsProcessing: 0,}; 4];
    static mut telemetryReceived: bool = 0 as libc::c_int != 0;
    let mut ret: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    let mut current_block_159: u64;
    match *protocolState as libc::c_int {
        6 => {
            listLength = 47 as libc::c_int as uint8_t;
            initialiseData(0 as libc::c_int as uint8_t);
            *protocolState = STATE_UPDATE as libc::c_int as uint8_t;
            nextChannel(1 as libc::c_int as uint8_t);
            cc2500Strobe(0x34 as libc::c_int as uint8_t);
            current_block_159 = 145651165234646754;
        }
        7 => {
            packetTimerUs = micros();
            *protocolState = STATE_DATA as libc::c_int as uint8_t;
            frameReceived = 0 as libc::c_int != 0;
            receiveDelayUs = 5300 as libc::c_int;
            if checkBindRequested(0 as libc::c_int != 0) {
                packetTimerUs = 0 as libc::c_int as timeUs_t;
                timeoutUs = 50 as libc::c_int;
                missingPackets = 0 as libc::c_int as uint32_t;
                *protocolState = STATE_INIT as libc::c_int as uint8_t;
                current_block_159 = 145651165234646754;
            } else { current_block_159 = 2477171227625166794; }
        }
        8 => { current_block_159 = 2477171227625166794; }
        9 => {
            if cmpTimeUs(micros(), packetTimerUs) >=
                   receiveDelayUs + 400 as libc::c_int {
                // if received or not received in this time sent telemetry data
                cc2500Strobe(0x36 as libc::c_int as uint8_t);
                cc2500SetPower(6 as libc::c_int as uint8_t);
                cc2500Strobe(0x3a as libc::c_int as uint8_t);
                delayMicroseconds(30 as libc::c_int as timeUs_t);
                TxEnable();
                cc2500Strobe(0x36 as libc::c_int as uint8_t);
                cc2500WriteFifo(frame.as_mut_ptr(),
                                (frame[0 as libc::c_int as usize] as
                                     libc::c_int + 1 as libc::c_int) as
                                    uint8_t);
                if telemetryEnabled {
                    let mut clearToSend: bool = 0 as libc::c_int != 0;
                    let mut now: timeMs_t = millis();
                    let mut payload: *mut smartPortPayload_t =
                        0 as *mut smartPortPayload_t;
                    if now.wrapping_sub(pollingTimeMs) >
                           24 as libc::c_int as libc::c_uint {
                        pollingTimeMs = now;
                        clearToSend = 1 as libc::c_int != 0
                    } else {
                        let mut remoteToProcessId: uint8_t =
                            ((remoteProcessedId as libc::c_int +
                                  1 as libc::c_int) % 4 as libc::c_int) as
                                uint8_t;
                        while telemetryRxBuffer[remoteToProcessId as
                                                    usize].needsProcessing as
                                  libc::c_int != 0 && payload.is_null() {
                            while (remoteToProcessIndex as libc::c_int) <
                                      telemetryRxBuffer[remoteToProcessId as
                                                            usize].data.dataLength
                                          as libc::c_int && payload.is_null()
                                  {
                                payload =
                                    smartPortDataReceive(telemetryRxBuffer[remoteToProcessId
                                                                               as
                                                                               usize].data.data[remoteToProcessIndex
                                                                                                    as
                                                                                                    usize]
                                                             as uint16_t,
                                                         &mut clearToSend,
                                                         Some(frSkyXCheckQueueEmpty
                                                                  as
                                                                  unsafe extern "C" fn()
                                                                      ->
                                                                          bool),
                                                         0 as libc::c_int !=
                                                             0);
                                remoteToProcessIndex =
                                    (remoteToProcessIndex as libc::c_int +
                                         1 as libc::c_int) as uint8_t
                            }
                            if remoteToProcessIndex as libc::c_int ==
                                   telemetryRxBuffer[remoteToProcessId as
                                                         usize].data.dataLength
                                       as libc::c_int {
                                remoteToProcessIndex =
                                    0 as libc::c_int as uint8_t;
                                telemetryRxBuffer[remoteToProcessId as
                                                      usize].needsProcessing =
                                    0 as libc::c_int as uint8_t;
                                remoteProcessedId = remoteToProcessId;
                                remoteToProcessId =
                                    ((remoteProcessedId as libc::c_int +
                                          1 as libc::c_int) %
                                         4 as libc::c_int) as uint8_t
                            }
                        }
                    }
                    processSmartPortTelemetry(payload,
                                              &mut clearToSend as *mut bool as
                                                  *mut bool,
                                              0 as *const uint32_t);
                }
                *protocolState = STATE_RESUME as libc::c_int as uint8_t;
                if frameReceived { ret = RX_SPI_RECEIVED_DATA }
            }
            current_block_159 = 145651165234646754;
        }
        10 => {
            // USE_RX_FRSKY_SPI_TELEMETRY
            if cmpTimeUs(micros(), packetTimerUs) >
                   receiveDelayUs + 3700 as libc::c_int {
                packetTimerUs = micros(); // again set for receive
                receiveDelayUs = 5300 as libc::c_int;
                frameReceived = 0 as libc::c_int != 0;
                nextChannel(channelsToSkip);
                cc2500Strobe(0x34 as libc::c_int as uint8_t);
                TxDisable();
                if missingPackets >= 2 as libc::c_int as libc::c_uint {
                    switchAntennae();
                }
                // USE_RX_FRSKY_SPI_PA_LNA
                if missingPackets > 100 as libc::c_int as libc::c_uint {
                    timeoutUs = 50 as libc::c_int;
                    skipChannels = 1 as libc::c_int != 0;
                    telemetryReceived = 0 as libc::c_int != 0;
                    *protocolState = STATE_UPDATE as libc::c_int as uint8_t
                } else {
                    missingPackets = missingPackets.wrapping_add(1);
                    if debugMode as libc::c_int ==
                           DEBUG_RX_FRSKY_SPI as libc::c_int {
                        debug[1 as libc::c_int as usize] =
                            missingPackets as int16_t
                    }
                    *protocolState = STATE_DATA as libc::c_int as uint8_t
                }
                current_block_159 = 145651165234646754;
            } else { current_block_159 = 145651165234646754; }
        }
        _ => { current_block_159 = 145651165234646754; }
    }
    match current_block_159 {
        2477171227625166794 =>
        // here FS code could be
        {
            if IORead(gdoPin) as libc::c_int != 0 &&
                   frameReceived as libc::c_int == 0 as libc::c_int {
                let mut ccLen: uint8_t =
                    (cc2500ReadReg((CC2500_3B_RXBYTES as libc::c_int |
                                        0xc0 as libc::c_int) as uint8_t) as
                         libc::c_int & 0x7f as libc::c_int) as
                        uint8_t; // read 2 times to avoid reading errors
                ccLen =
                    (cc2500ReadReg((CC2500_3B_RXBYTES as libc::c_int |
                                        0xc0 as libc::c_int) as uint8_t) as
                         libc::c_int & 0x7f as libc::c_int) as uint8_t;
                if ccLen as libc::c_int > 32 as libc::c_int {
                    ccLen = 32 as libc::c_int as uint8_t
                }
                if ccLen != 0 {
                    cc2500ReadFifo(packet, ccLen);
                    let mut lcrc: uint16_t =
                        calculateCrc(&mut *packet.offset(3 as libc::c_int as
                                                             isize),
                                     (ccLen as libc::c_int - 7 as libc::c_int)
                                         as uint8_t);
                    if lcrc as libc::c_int >> 8 as libc::c_int ==
                           *packet.offset((ccLen as libc::c_int -
                                               4 as libc::c_int) as isize) as
                               libc::c_int &&
                           lcrc as libc::c_int & 0xff as libc::c_int ==
                               *packet.offset((ccLen as libc::c_int -
                                                   3 as libc::c_int) as isize)
                                   as libc::c_int {
                        // check calculateCrc
                        if *packet.offset(0 as libc::c_int as isize) as
                               libc::c_int == 0x1d as libc::c_int {
                            if *packet.offset(1 as libc::c_int as isize) as
                                   libc::c_int ==
                                   (*rxFrSkySpiConfig()).bindTxId[0 as
                                                                      libc::c_int
                                                                      as
                                                                      usize]
                                       as libc::c_int &&
                                   *packet.offset(2 as libc::c_int as isize)
                                       as libc::c_int ==
                                       (*rxFrSkySpiConfig()).bindTxId[1 as
                                                                          libc::c_int
                                                                          as
                                                                          usize]
                                           as libc::c_int &&
                                   ((*rxFrSkySpiConfig()).rxNum as libc::c_int
                                        == 0 as libc::c_int ||
                                        *packet.offset(6 as libc::c_int as
                                                           isize) as
                                            libc::c_int == 0 as libc::c_int ||
                                        *packet.offset(6 as libc::c_int as
                                                           isize) as
                                            libc::c_int ==
                                            (*rxFrSkySpiConfig()).rxNum as
                                                libc::c_int) {
                                missingPackets = 0 as libc::c_int as uint32_t;
                                timeoutUs = 1 as libc::c_int;
                                receiveDelayUs = 0 as libc::c_int;
                                LedOn();
                                if skipChannels {
                                    channelsToSkip =
                                        ((*packet.offset(5 as libc::c_int as
                                                             isize) as
                                              libc::c_int) <<
                                             2 as libc::c_int) as uint8_t;
                                    if *packet.offset(4 as libc::c_int as
                                                          isize) as
                                           libc::c_int >=
                                           listLength as libc::c_int {
                                        if (*packet.offset(4 as libc::c_int as
                                                               isize) as
                                                libc::c_int) <
                                               64 as libc::c_int +
                                                   listLength as libc::c_int {
                                            channelsToSkip =
                                                (channelsToSkip as libc::c_int
                                                     + 1 as libc::c_int) as
                                                    uint8_t
                                        } else if (*packet.offset(4 as
                                                                      libc::c_int
                                                                      as
                                                                      isize)
                                                       as libc::c_int) <
                                                      128 as libc::c_int +
                                                          listLength as
                                                              libc::c_int {
                                            channelsToSkip =
                                                (channelsToSkip as libc::c_int
                                                     + 2 as libc::c_int) as
                                                    uint8_t
                                        } else if (*packet.offset(4 as
                                                                      libc::c_int
                                                                      as
                                                                      isize)
                                                       as libc::c_int) <
                                                      192 as libc::c_int +
                                                          listLength as
                                                              libc::c_int {
                                            channelsToSkip =
                                                (channelsToSkip as libc::c_int
                                                     + 3 as libc::c_int) as
                                                    uint8_t
                                        }
                                    }
                                    // no need to process frame again.
                                    telemetryReceived =
                                        1 as libc::c_int !=
                                            0; // now telemetry can be sent
                                    skipChannels = 0 as libc::c_int != 0
                                }
                                setRssiDbm(*packet.offset((ccLen as
                                                               libc::c_int -
                                                               2 as
                                                                   libc::c_int)
                                                              as isize));
                                let mut inFrameMarker:
                                        *mut telemetrySequenceMarker_t =
                                    &mut *packet.offset(21 as libc::c_int as
                                                            isize) as
                                        *mut uint8_t as
                                        *mut telemetrySequenceMarker_t;
                                let mut remoteNewPacketId: uint8_t =
                                    (*inFrameMarker).data.packetSequenceId()
                                        as uint8_t;
                                memcpy(&mut (*telemetryRxBuffer.as_mut_ptr().offset(remoteNewPacketId
                                                                                        as
                                                                                        isize)).data
                                           as *mut telemetryData_t as
                                           *mut libc::c_void,
                                       &mut *packet.offset(22 as libc::c_int
                                                               as isize) as
                                           *mut uint8_t as
                                           *const libc::c_void,
                                       ::core::mem::size_of::<telemetryData_t>()
                                           as libc::c_ulong);
                                telemetryRxBuffer[remoteNewPacketId as
                                                      usize].needsProcessing =
                                    1 as libc::c_int as uint8_t;
                                responseToSend.raw =
                                    0 as libc::c_int as uint8_t;
                                let mut remoteToAckId: uint8_t =
                                    ((remoteAckId as libc::c_int +
                                          1 as libc::c_int) %
                                         4 as libc::c_int) as uint8_t;
                                if remoteNewPacketId as libc::c_int !=
                                       remoteToAckId as libc::c_int {
                                    while remoteToAckId as libc::c_int !=
                                              remoteNewPacketId as libc::c_int
                                          {
                                        if telemetryRxBuffer[remoteToAckId as
                                                                 usize].needsProcessing
                                               == 0 {
                                            responseToSend.data.set_ackSequenceId(remoteToAckId
                                                                                      as
                                                                                      libc::c_uint);
                                            responseToSend.data.set_retransmissionRequested(1
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint);
                                            receiveTelemetryRetryCount =
                                                receiveTelemetryRetryCount.wrapping_add(1);
                                            break ;
                                        } else {
                                            remoteToAckId =
                                                ((remoteToAckId as libc::c_int
                                                      + 1 as libc::c_int) %
                                                     4 as libc::c_int) as
                                                    uint8_t
                                        }
                                    }
                                }
                                if responseToSend.data.retransmissionRequested()
                                       == 0 {
                                    receiveTelemetryRetryCount =
                                        0 as libc::c_int as libc::c_uint;
                                    remoteToAckId =
                                        ((remoteAckId as libc::c_int +
                                              1 as libc::c_int) %
                                             4 as libc::c_int) as uint8_t;
                                    let mut remoteNextAckId: uint8_t = 0;
                                    while telemetryRxBuffer[remoteToAckId as
                                                                usize].needsProcessing
                                              as libc::c_int != 0 &&
                                              remoteToAckId as libc::c_int !=
                                                  remoteAckId as libc::c_int {
                                        remoteNextAckId = remoteToAckId;
                                        remoteToAckId =
                                            ((remoteToAckId as libc::c_int +
                                                  1 as libc::c_int) %
                                                 4 as libc::c_int) as uint8_t
                                    }
                                    remoteAckId = remoteNextAckId;
                                    responseToSend.data.set_ackSequenceId(remoteAckId
                                                                              as
                                                                              libc::c_uint)
                                }
                                if receiveTelemetryRetryCount >=
                                       5 as libc::c_int as libc::c_uint {
                                    remoteProcessedId =
                                        (4 as libc::c_int - 1 as libc::c_int)
                                            as uint8_t;
                                    remoteAckId =
                                        (4 as libc::c_int - 1 as libc::c_int)
                                            as uint8_t;
                                    let mut i: libc::c_uint =
                                        0 as libc::c_int as libc::c_uint;
                                    while i < 4 as libc::c_int as libc::c_uint
                                          {
                                        telemetryRxBuffer[i as
                                                              usize].needsProcessing
                                            = 0 as libc::c_int as uint8_t;
                                        i = i.wrapping_add(1)
                                    }
                                    receiveTelemetryRetryCount =
                                        0 as libc::c_int as libc::c_uint
                                }
                                packetTimerUs = micros();
                                frameReceived = 1 as libc::c_int != 0
                            }
                        }
                    }
                    if !frameReceived {
                        packetErrors = packetErrors.wrapping_add(1);
                        if debugMode as libc::c_int ==
                               DEBUG_RX_FRSKY_SPI as libc::c_int {
                            debug[2 as libc::c_int as usize] =
                                packetErrors as int16_t
                        }
                    }
                }
            }
            if telemetryReceived {
                if cmpTimeUs(micros(), packetTimerUs) > receiveDelayUs {
                    // if received or not received in this time sent telemetry data
                    *protocolState =
                        STATE_TELEMETRY as libc::c_int as uint8_t;
                    buildTelemetryFrame(packet);
                }
            }
            if cmpTimeUs(micros(), packetTimerUs) >
                   timeoutUs * 9000 as libc::c_int {
                if ledIsOn { LedOff(); } else { LedOn(); }
                ledIsOn = !ledIsOn;
                setRssiDirect(0 as libc::c_int as uint16_t,
                              RSSI_SOURCE_RX_PROTOCOL);
                nextChannel(1 as libc::c_int as uint8_t);
                cc2500Strobe(0x34 as libc::c_int as uint8_t);
                *protocolState = STATE_UPDATE as libc::c_int as uint8_t
            }
        }
        _ => { }
    }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn frSkyXInit() {
    if feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) {
        telemetryEnabled =
            initSmartPortTelemetryExternal(Some(frSkyXTelemetryWriteFrame as
                                                    unsafe extern "C" fn(_:
                                                                             *const smartPortPayload_t)
                                                        -> ()))
    };
}
