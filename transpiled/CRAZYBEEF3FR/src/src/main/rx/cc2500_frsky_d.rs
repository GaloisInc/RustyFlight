use ::libc;
extern "C" {
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
    fn micros() -> timeUs_t;
    #[no_mangle]
    fn adcGetChannel(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn setRssiDirect(newRssi: uint16_t, source: rssiSource_e);
    #[no_mangle]
    fn setRssi(rssiValue: uint16_t, source: rssiSource_e);
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
 CC2500 SPI drivers
*/
    // GDO2 output pin configuration
    // GDO1 output pin configuration
    // GDO0 output pin configuration
    // RX FIFO and TX FIFO thresholds
    // Sync word, high byte
    // Sync word, low byte
    // Packet length
    // Packet automation control
    // Packet automation control
    // Device address
    // Channel number
    // Frequency synthesizer control
    // Frequency synthesizer control
    // Frequency control word, high byte
    // Frequency control word, middle byte
    // Frequency control word, low byte
    // Modem configuration
    // Modem configuration
    // Modem configuration
    // Modem configuration
    // Modem configuration
    // Modem deviation setting
    // Main Radio Cntrl State Machine config
    // Main Radio Cntrl State Machine config
    // Main Radio Cntrl State Machine config
    // Frequency Offset Compensation config
    // Bit Synchronization configuration
    // AGC control
    // AGC control
    // AGC control
    // High byte Event 0 timeout
    // Low byte Event 0 timeout
    // Wake On Radio control
    // Front end RX configuration
    // Front end TX configuration
    // Frequency synthesizer calibration
    // Frequency synthesizer calibration
    // Frequency synthesizer calibration
    // Frequency synthesizer calibration
    // RC oscillator configuration
    // RC oscillator configuration
    // Frequency synthesizer cal control
    // Production test
    // AGC test
    // Various test settings
    // Various test settings
    // Various test settings
    // Status registers
    // Part number
    // Current version number
    // Frequency offset estimate
    // Demodulator estimate for link quality
    // Received signal strength indication
    // Control state machine state
    // High byte of WOR timer
    // Low byte of WOR timer
    // Current GDOx status and packet status
    // Current setting from PLL cal module
    // Underflow and # of bytes in TXFIFO
    // Overflow and # of bytes in RXFIFO
    // Multi byte memory locations
    // Definitions for burst/single access to registers
    // Strobe commands
    // Reset chip.
    // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
         // If in RX/TX: Go to a wait state where only the synthesizer is
         // running (for quick RX / TX turnaround).
    // Turn off crystal oscillator.
    // Calibrate frequency synthesizer and turn it off
                          // (enables quick start).
    // Enable RX. Perform calibration first if coming from IDLE and
         // MCSM0.FS_AUTOCAL=1.
    // In IDLE state: Enable TX. Perform calibration first if
         // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
         // Only go to TX if channel is clear.
    // Exit RX / TX, turn off frequency synthesizer and exit
         // Wake-On-Radio mode if applicable.
    // Perform AFC adjustment of the frequency synthesizer
    // Start automatic RX polling sequence (Wake-on-Radio)
    // Enter power down mode when CSn goes high.
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
    fn cc2500WriteFifo(dpbuffer: *mut uint8_t, len: uint8_t) -> uint8_t;
    #[no_mangle]
    fn cc2500ReadFifo(dpbuffer: *mut uint8_t, len: uint8_t) -> uint8_t;
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
    fn LedOn();
    #[no_mangle]
    fn LedOff();
    #[no_mangle]
    fn initialiseData(adr: uint8_t);
    #[no_mangle]
    fn checkBindRequested(reset: bool) -> bool;
    #[no_mangle]
    fn nextChannel(skip: uint8_t);
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn initFrSkyHubTelemetryExternal(frSkyWriteFrameExternal:
                                         Option<frSkyHubWriteByteFn>) -> bool;
    #[no_mangle]
    fn processFrSkyHubTelemetry(currentTimeUs: timeUs_t);
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
// packet tag to specify IO pin
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
pub type frSkyHubWriteByteFn = unsafe extern "C" fn(_: libc::c_char) -> ();
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
static mut frame: [uint8_t; 20] = [0; 20];
static mut telemetryId: uint8_t = 0;
static mut telemetryEnabled: bool = 0 as libc::c_int != 0;
static mut telemetryBytesGenerated: uint8_t = 0;
static mut serialBuffer: [uint8_t; 64] = [0; 64];
// buffer for telemetry serial data
unsafe extern "C" fn appendFrSkyHubData(mut buf: *mut uint8_t) -> uint8_t {
    static mut telemetryBytesSent: uint8_t =
        0 as libc::c_int as uint8_t; // rx re-requests last packet
    static mut telemetryBytesAcknowledged: uint8_t =
        0 as libc::c_int as uint8_t; // length
    static mut telemetryIdExpected: uint8_t = 0 as libc::c_int as uint8_t;
    if telemetryId as libc::c_int == telemetryIdExpected as libc::c_int {
        telemetryBytesAcknowledged = telemetryBytesSent;
        telemetryIdExpected =
            (telemetryId as libc::c_int + 1 as libc::c_int &
                 0x1f as libc::c_int) as uint8_t;
        if telemetryBytesGenerated == 0 {
            telemetryBytesSent = 0 as libc::c_int as uint8_t;
            processFrSkyHubTelemetry(micros());
        }
    } else { telemetryBytesSent = telemetryBytesAcknowledged }
    let mut index: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 10 as libc::c_int {
        if telemetryBytesSent as libc::c_int ==
               telemetryBytesGenerated as libc::c_int {
            telemetryBytesGenerated = 0 as libc::c_int as uint8_t;
            break ;
        } else {
            *buf.offset(i as isize) =
                serialBuffer[telemetryBytesSent as usize];
            telemetryBytesSent =
                (telemetryBytesSent as libc::c_int + 1 as libc::c_int &
                     64 as libc::c_int - 1 as libc::c_int) as uint8_t;
            index = index.wrapping_add(1);
            i = i.wrapping_add(1)
        }
    }
    return index;
}
unsafe extern "C" fn frSkyDTelemetryWriteByte(data: libc::c_char) {
    if (telemetryBytesGenerated as libc::c_int) < 64 as libc::c_int {
        let fresh0 = telemetryBytesGenerated;
        telemetryBytesGenerated = telemetryBytesGenerated.wrapping_add(1);
        serialBuffer[fresh0 as usize] = data as uint8_t
    };
}
unsafe extern "C" fn buildTelemetryFrame(mut packet: *mut uint8_t) {
    let mut a1Value: uint8_t = 0;
    if (*rxFrSkySpiConfig()).useExternalAdc != 0 {
        a1Value =
            ((adcGetChannel(ADC_EXTERNAL1 as libc::c_int as uint8_t) as
                  libc::c_int & 0xff0 as libc::c_int) >> 4 as libc::c_int) as
                uint8_t
    } else {
        a1Value =
            (2 as libc::c_int * getBatteryVoltage() as libc::c_int &
                 0xff as libc::c_int) as uint8_t
    }
    let a2Value: uint8_t =
        (adcGetChannel(ADC_RSSI as libc::c_int as uint8_t) as libc::c_int >>
             4 as libc::c_int) as uint8_t;
    telemetryId = *packet.offset(4 as libc::c_int as isize);
    frame[0 as libc::c_int as usize] = 0x11 as libc::c_int as uint8_t;
    frame[1 as libc::c_int as usize] =
        (*rxFrSkySpiConfig()).bindTxId[0 as libc::c_int as usize];
    frame[2 as libc::c_int as usize] =
        (*rxFrSkySpiConfig()).bindTxId[1 as libc::c_int as usize];
    frame[3 as libc::c_int as usize] = a1Value;
    frame[4 as libc::c_int as usize] = a2Value;
    frame[5 as libc::c_int as usize] = rssiDbm as uint8_t;
    let mut bytesUsed: uint8_t = 0 as libc::c_int as uint8_t;
    if telemetryEnabled {
        bytesUsed =
            appendFrSkyHubData(&mut *frame.as_mut_ptr().offset(8 as
                                                                   libc::c_int
                                                                   as isize))
    }
    frame[6 as libc::c_int as usize] = bytesUsed;
    frame[7 as libc::c_int as usize] = telemetryId;
}
unsafe extern "C" fn decodeChannelPair(mut channels: *mut uint16_t,
                                       mut packet: *const uint8_t,
                                       highNibbleOffset: uint8_t) {
    *channels.offset(0 as libc::c_int as isize) =
        (2.0f32 / 3 as libc::c_int as libc::c_float *
             ((*packet.offset(highNibbleOffset as isize) as libc::c_int &
                   0xf as libc::c_int) << 8 as libc::c_int |
                  *packet.offset(0 as libc::c_int as isize) as libc::c_int) as
                 uint16_t as libc::c_int as libc::c_float) as uint16_t;
    *channels.offset(1 as libc::c_int as isize) =
        (2.0f32 / 3 as libc::c_int as libc::c_float *
             ((*packet.offset(highNibbleOffset as isize) as libc::c_int &
                   0xf0 as libc::c_int) << 4 as libc::c_int |
                  *packet.offset(1 as libc::c_int as isize) as libc::c_int) as
                 uint16_t as libc::c_int as libc::c_float) as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn frSkyDSetRcData(mut rcData: *mut uint16_t,
                                         mut packet: *const uint8_t) {
    static mut dataErrorCount: uint16_t = 0 as libc::c_int as uint16_t;
    let mut channels: [uint16_t; 8] = [0; 8];
    let mut dataError: bool = 0 as libc::c_int != 0;
    decodeChannelPair(channels.as_mut_ptr(),
                      packet.offset(6 as libc::c_int as isize),
                      4 as libc::c_int as uint8_t);
    decodeChannelPair(channels.as_mut_ptr().offset(2 as libc::c_int as isize),
                      packet.offset(8 as libc::c_int as isize),
                      3 as libc::c_int as uint8_t);
    decodeChannelPair(channels.as_mut_ptr().offset(4 as libc::c_int as isize),
                      packet.offset(12 as libc::c_int as isize),
                      4 as libc::c_int as uint8_t);
    decodeChannelPair(channels.as_mut_ptr().offset(6 as libc::c_int as isize),
                      packet.offset(14 as libc::c_int as isize),
                      3 as libc::c_int as uint8_t);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 8 as libc::c_int {
        if (channels[i as usize] as libc::c_int) < 800 as libc::c_int ||
               channels[i as usize] as libc::c_int > 2200 as libc::c_int {
            dataError = 1 as libc::c_int != 0;
            break ;
        } else { i += 1 }
    }
    if !dataError {
        let mut i_0: libc::c_int = 0 as libc::c_int;
        while i_0 < 8 as libc::c_int {
            *rcData.offset(i_0 as isize) = channels[i_0 as usize];
            i_0 += 1
        }
    } else if debugMode as libc::c_int == DEBUG_RX_FRSKY_SPI as libc::c_int {
        dataErrorCount = dataErrorCount.wrapping_add(1);
        debug[0 as libc::c_int as usize] = dataErrorCount as int16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn frSkyDHandlePacket(packet: *mut uint8_t,
                                            protocolState: *mut uint8_t)
 -> rx_spi_received_e {
    static mut lastPacketReceivedTime: timeUs_t =
        0 as libc::c_int as timeUs_t;
    static mut telemetryTimeUs: timeUs_t = 0;
    static mut ledIsOn: bool = false;
    let mut ret: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    let currentPacketReceivedTime: timeUs_t = micros();
    let mut current_block_69: u64;
    match *protocolState as libc::c_int {
        6 => {
            listLength = 47 as libc::c_int as uint8_t;
            initialiseData(0 as libc::c_int as uint8_t);
            *protocolState = STATE_UPDATE as libc::c_int as uint8_t;
            nextChannel(1 as libc::c_int as uint8_t);
            cc2500Strobe(0x34 as libc::c_int as uint8_t);
            current_block_69 = 10512632378975961025;
        }
        7 => {
            lastPacketReceivedTime = currentPacketReceivedTime;
            *protocolState = STATE_DATA as libc::c_int as uint8_t;
            if checkBindRequested(0 as libc::c_int != 0) {
                lastPacketReceivedTime = 0 as libc::c_int as timeUs_t;
                timeoutUs = 50 as libc::c_int;
                missingPackets = 0 as libc::c_int as uint32_t;
                *protocolState = STATE_INIT as libc::c_int as uint8_t;
                current_block_69 = 10512632378975961025;
            } else { current_block_69 = 12152728707720876685; }
        }
        8 => { current_block_69 = 12152728707720876685; }
        9 => {
            if cmpTimeUs(micros(), telemetryTimeUs) >= 1380 as libc::c_int {
                cc2500Strobe(0x36 as libc::c_int as uint8_t);
                cc2500SetPower(6 as libc::c_int as uint8_t);
                cc2500Strobe(0x3a as libc::c_int as uint8_t);
                cc2500Strobe(0x36 as libc::c_int as uint8_t);
                cc2500WriteFifo(frame.as_mut_ptr(),
                                (frame[0 as libc::c_int as usize] as
                                     libc::c_int + 1 as libc::c_int) as
                                    uint8_t);
                *protocolState = STATE_DATA as libc::c_int as uint8_t;
                ret = RX_SPI_RECEIVED_DATA;
                lastPacketReceivedTime = currentPacketReceivedTime
            }
            current_block_69 = 10512632378975961025;
        }
        _ => { current_block_69 = 10512632378975961025; }
    }
    match current_block_69 {
        12152728707720876685 =>
        // !!TODO -check this fall through is correct
    // here FS code could be
        {
            if IORead(gdoPin) {
                let mut ccLen: uint8_t =
                    (cc2500ReadReg((CC2500_3B_RXBYTES as libc::c_int |
                                        0xc0 as libc::c_int) as uint8_t) as
                         libc::c_int & 0x7f as libc::c_int) as uint8_t;
                if ccLen as libc::c_int >= 20 as libc::c_int {
                    cc2500ReadFifo(packet, 20 as libc::c_int as uint8_t);
                    if *packet.offset(19 as libc::c_int as isize) as
                           libc::c_int & 0x80 as libc::c_int != 0 {
                        missingPackets = 0 as libc::c_int as uint32_t;
                        timeoutUs = 1 as libc::c_int;
                        if *packet.offset(0 as libc::c_int as isize) as
                               libc::c_int == 0x11 as libc::c_int {
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
                                           as libc::c_int {
                                LedOn();
                                nextChannel(1 as libc::c_int as uint8_t);
                                if *packet.offset(3 as libc::c_int as isize)
                                       as libc::c_int % 4 as libc::c_int ==
                                       2 as libc::c_int {
                                    telemetryTimeUs = micros();
                                    setRssiDbm(*packet.offset(18 as
                                                                  libc::c_int
                                                                  as isize));
                                    buildTelemetryFrame(packet);
                                    *protocolState =
                                        STATE_TELEMETRY as libc::c_int as
                                            uint8_t
                                } else {
                                    cc2500Strobe(0x34 as libc::c_int as
                                                     uint8_t);
                                    *protocolState =
                                        STATE_UPDATE as libc::c_int as uint8_t
                                }
                                ret = RX_SPI_RECEIVED_DATA;
                                lastPacketReceivedTime =
                                    currentPacketReceivedTime
                            }
                        }
                    }
                }
            }
            if cmpTimeUs(currentPacketReceivedTime, lastPacketReceivedTime) >
                   timeoutUs * 9000 as libc::c_int {
                if timeoutUs == 1 as libc::c_int {
                    // SE4311 chip
                    if missingPackets > 100 as libc::c_int as libc::c_uint {
                        timeoutUs = 50 as libc::c_int;
                        setRssiDirect(0 as libc::c_int as uint16_t,
                                      RSSI_SOURCE_RX_PROTOCOL);
                    }
                    missingPackets = missingPackets.wrapping_add(1);
                    nextChannel(1 as libc::c_int as uint8_t);
                } else {
                    if ledIsOn { LedOff(); } else { LedOn(); }
                    ledIsOn = !ledIsOn;
                    setRssi(0 as libc::c_int as uint16_t,
                            RSSI_SOURCE_RX_PROTOCOL);
                    nextChannel(13 as libc::c_int as uint8_t);
                }
                cc2500Strobe(0x34 as libc::c_int as uint8_t);
                *protocolState = STATE_UPDATE as libc::c_int as uint8_t
            }
        }
        _ => { }
    }
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn frSkyDInit() {
    if feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) {
        telemetryEnabled =
            initFrSkyHubTelemetryExternal(Some(frSkyDTelemetryWriteByte as
                                                   unsafe extern "C" fn(_:
                                                                            libc::c_char)
                                                       -> ()))
    };
}
