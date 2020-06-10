use ::libc;
extern "C" {
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
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
    // gyro alignment
    // people keep forgetting that moving model while init results in wrong gyro offsets. and then they never reset gyro. so this is now on by default.
    // Gyro sample divider
    // gyro DLPF setting
    // gyro 32khz DLPF setting
    // Lowpass primary/secondary
    // Gyro calibration duration in 1/100 second
    // bandpass quality factor, 100 for steep sided bandpass
    #[no_mangle]
    fn gyroGetTemperature() -> int16_t;
    #[no_mangle]
    fn serialWrite(instance: *mut serialPort_t, ch: uint8_t);
    #[no_mangle]
    fn millis() -> timeMs_t;
    // (Super) rates are constrained to [0, 100] for Betaflight rates, so values higher than 100 won't make a difference. Range extended for RaceFlight rates.
    #[no_mangle]
    static mut rcCommand: [libc::c_float; 4];
    #[no_mangle]
    fn calculateThrottleStatus() -> throttleStatus_e;
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn calculateBatteryPercentageRemaining() -> uint8_t;
    #[no_mangle]
    fn isBatteryVoltageConfigured() -> bool;
    #[no_mangle]
    fn getBatteryVoltage() -> uint16_t;
    #[no_mangle]
    fn getBatteryCellCount() -> uint8_t;
    #[no_mangle]
    fn isAmperageConfigured() -> bool;
    #[no_mangle]
    fn getAmperage() -> int32_t;
    #[no_mangle]
    fn getMAhDrawn() -> int32_t;
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
    static mut telemetryConfig_System: telemetryConfig_t;
    #[no_mangle]
    static mut telemetrySharedPort: *mut serialPort_t;
    #[no_mangle]
    fn telemetryDetermineEnabledState(portSharing: portSharing_e) -> bool;
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig_0: *const serialPortConfig_t)
     -> bool;
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
/* * 
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
pub const CW0_DEG: sensor_align_e = 1;
pub const ALIGN_DEFAULT: sensor_align_e = 0;
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
// initialize function
// read 3 axis data function
// read temperature if available
// scalefactor
// gyro data after calibration and alignment
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accDev_s {
    pub initFn: sensorAccInitFuncPtr,
    pub readFn: sensorAccReadFuncPtr,
    pub bus: busDevice_t,
    pub acc_1G: uint16_t,
    pub ADCRaw: [int16_t; 3],
    pub mpuDetectionResult: mpuDetectionResult_t,
    pub accAlign: sensor_align_e,
    pub dataReady: bool,
    pub acc_high_fsr: bool,
    pub revisionCode: libc::c_char,
    pub filler: [uint8_t; 2],
}
pub type mpuDetectionResult_t = mpuDetectionResult_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
}
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
pub type mpuSensor_e = libc::c_uint;
pub const BMI_160_SPI: mpuSensor_e = 12;
pub const ICM_20689_SPI: mpuSensor_e = 11;
pub const ICM_20649_SPI: mpuSensor_e = 10;
pub const ICM_20608_SPI: mpuSensor_e = 9;
pub const ICM_20602_SPI: mpuSensor_e = 8;
pub const ICM_20601_SPI: mpuSensor_e = 7;
pub const MPU_9250_SPI: mpuSensor_e = 6;
pub const MPU_65xx_SPI: mpuSensor_e = 5;
pub const MPU_65xx_I2C: mpuSensor_e = 4;
pub const MPU_60x0_SPI: mpuSensor_e = 3;
pub const MPU_60x0: mpuSensor_e = 2;
pub const MPU_3050: mpuSensor_e = 1;
pub const MPU_NONE: mpuSensor_e = 0;
pub type sensorAccReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> bool>;
pub type sensorAccInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut accDev_s) -> ()>;
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
pub type accDev_t = accDev_s;
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
pub type throttleStatus_e = libc::c_uint;
pub const THROTTLE_HIGH: throttleStatus_e = 1;
pub const THROTTLE_LOW: throttleStatus_e = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_1 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_1 = 2;
pub const ARMED: C2RustUnnamed_1 = 1;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_2 = 64;
pub const SENSOR_GPS: C2RustUnnamed_2 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_2 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_2 = 16;
pub const SENSOR_MAG: C2RustUnnamed_2 = 8;
pub const SENSOR_BARO: C2RustUnnamed_2 = 4;
pub const SENSOR_ACC: C2RustUnnamed_2 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_2 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct acc_s {
    pub dev: accDev_t,
    pub accSamplingInterval: uint32_t,
    pub accADC: [libc::c_float; 3],
    pub isAccelUpdatedAtLeastOnce: bool,
}
pub type acc_t = acc_s;
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
pub type batteryConfig_t = batteryConfig_s;
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
// voltage
// maximum voltage per cell, used for auto-detecting battery voltage in 0.1V units, default is 43 (4.3V)
// minimum voltage per cell, this triggers battery critical alarm, in 0.1V units, default is 33 (3.3V)
// warning voltage per cell, this triggers battery warning alarm, in 0.1V units, default is 35 (3.5V)
// Between vbatmaxcellvoltage and 2*this is considered to be USB powered. Below this it is notpresent
// Percentage of throttle when lvc is triggered
// source of battery voltage meter used, either ADC or ESC
// current
// source of battery current meter used, either ADC, Virtual or ESC
// mAh
// warnings / alerts
// Issue alerts based on VBat readings
// Issue alerts based on total power consumption
// Percentage of remaining capacity that should trigger a battery warning
// hysteresis for alarm, default 1 = 0.1V
// Cell voltage at which the battery is deemed to be "full" 0.1V units, default is 41 (4.1V)
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FRSKY_VFAS_PRECISION_HIGH: C2RustUnnamed_3 = 1;
pub const FRSKY_VFAS_PRECISION_LOW: C2RustUnnamed_3 = 0;
pub type frSkyHubWriteByteFn = unsafe extern "C" fn(_: libc::c_char) -> ();
pub const TELEMETRY_STATE_INITIALIZED_SERIAL: C2RustUnnamed_4 = 1;
pub const TELEMETRY_STATE_UNINITIALIZED: C2RustUnnamed_4 = 0;
pub const TELEMETRY_STATE_INITIALIZED_EXTERNAL: C2RustUnnamed_4 = 2;
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// should set 12 blades in Taranis
pub type C2RustUnnamed_4 = libc::c_uint;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
}
#[inline]
unsafe extern "C" fn cmpTimeUs(mut a: timeUs_t, mut b: timeUs_t)
 -> timeDelta_t {
    return a.wrapping_sub(b) as timeDelta_t;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
}
#[inline]
unsafe extern "C" fn telemetryConfig() -> *const telemetryConfig_t {
    return &mut telemetryConfig_System;
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
 * Initial FrSky Hub Telemetry implementation by silpstream @ rcgroups.
 * Addition protocol work by airmamaf @ github.
 */
static mut frSkyHubPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut portConfig: *mut serialPortConfig_t =
    0 as *const serialPortConfig_t as *mut serialPortConfig_t;
static mut frSkyHubPortSharing: portSharing_e = PORTSHARING_UNUSED;
static mut frSkyHubWriteByte: Option<frSkyHubWriteByteFn> = None;
static mut telemetryState: uint8_t =
    TELEMETRY_STATE_UNINITIALIZED as libc::c_int as uint8_t;
unsafe extern "C" fn serializeFrSkyHub(mut data: uint8_t) {
    // take care of byte stuffing
    if data as libc::c_int == 0x5e as libc::c_int {
        frSkyHubWriteByte.expect("non-null function pointer")(0x5d as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
        frSkyHubWriteByte.expect("non-null function pointer")(0x3e as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
    } else if data as libc::c_int == 0x5d as libc::c_int {
        frSkyHubWriteByte.expect("non-null function pointer")(0x5d as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
        frSkyHubWriteByte.expect("non-null function pointer")(0x3d as
                                                                  libc::c_int
                                                                  as
                                                                  libc::c_char);
    } else {
        frSkyHubWriteByte.expect("non-null function pointer")(data as
                                                                  libc::c_char);
    };
}
unsafe extern "C" fn frSkyHubWriteFrame(id: uint8_t, data: int16_t) {
    frSkyHubWriteByte.expect("non-null function pointer")(0x5e as libc::c_int
                                                              as
                                                              libc::c_char);
    frSkyHubWriteByte.expect("non-null function pointer")(id as libc::c_char);
    serializeFrSkyHub(data as uint8_t);
    serializeFrSkyHub((data as libc::c_int >> 8 as libc::c_int) as uint8_t);
}
unsafe extern "C" fn sendTelemetryTail() {
    frSkyHubWriteByte.expect("non-null function pointer")(0x5e as libc::c_int
                                                              as
                                                              libc::c_char);
}
unsafe extern "C" fn frSkyHubWriteByteInternal(data: libc::c_char) {
    serialWrite(frSkyHubPort, data as uint8_t);
}
unsafe extern "C" fn sendAccel() {
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 3 as libc::c_int as libc::c_uint {
        frSkyHubWriteFrame((0x24 as libc::c_int as
                                libc::c_uint).wrapping_add(i) as uint8_t,
                           ((acc.accADC[i as usize] /
                                 acc.dev.acc_1G as libc::c_int as
                                     libc::c_float) as int16_t as libc::c_int
                                * 1000 as libc::c_int) as int16_t);
        i = i.wrapping_add(1)
    };
}
unsafe extern "C" fn sendThrottleOrBatterySizeAsRpm() {
    let mut data: int16_t = 0 as libc::c_int as int16_t;
    if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
        let throttleStatus: throttleStatus_e = calculateThrottleStatus();
        let mut throttleForRPM: uint16_t =
            (rcCommand[THROTTLE as libc::c_int as usize] /
                 5 as libc::c_int as libc::c_float) as uint16_t;
        if throttleStatus as libc::c_uint ==
               THROTTLE_LOW as libc::c_int as libc::c_uint &&
               feature(FEATURE_MOTOR_STOP as libc::c_int as uint32_t) as
                   libc::c_int != 0 {
            throttleForRPM = 0 as libc::c_int as uint16_t
        }
        data = throttleForRPM as int16_t
    } else {
        data =
            ((*batteryConfig()).batteryCapacity as libc::c_int /
                 5 as libc::c_int) as int16_t
    }
    frSkyHubWriteFrame(0x3 as libc::c_int as uint8_t, data);
}
unsafe extern "C" fn sendTemperature1() {
    let mut data: int16_t = 0 as libc::c_int as int16_t;
    data =
        (gyroGetTemperature() as libc::c_int / 10 as libc::c_int) as int16_t;
    frSkyHubWriteFrame(0x2 as libc::c_int as uint8_t, data);
}
unsafe extern "C" fn sendTime() {
    let mut seconds: uint32_t =
        millis().wrapping_div(1000 as libc::c_int as libc::c_uint);
    let mut minutes: uint8_t =
        seconds.wrapping_div(60 as libc::c_int as
                                 libc::c_uint).wrapping_rem(60 as libc::c_int
                                                                as
                                                                libc::c_uint)
            as uint8_t;
    // if we fly for more than an hour, something's wrong anyway
    frSkyHubWriteFrame(0x17 as libc::c_int as uint8_t,
                       ((minutes as libc::c_int) << 8 as libc::c_int) as
                           int16_t);
    frSkyHubWriteFrame(0x18 as libc::c_int as uint8_t,
                       seconds.wrapping_rem(60 as libc::c_int as libc::c_uint)
                           as int16_t);
}
/*
 * Send voltage via ID_VOLT
 *
 * NOTE: This sends voltage divided by batteryCellCount. To get the real
 * battery voltage, you need to multiply the value by batteryCellCount.
 */
unsafe extern "C" fn sendVoltageCells() {
    static mut currentCell: uint16_t = 0;
    let mut cellVoltage: uint32_t = 0 as libc::c_int as uint32_t;
    let cellCount: uint8_t = getBatteryCellCount();
    if cellCount != 0 {
        currentCell =
            (currentCell as libc::c_int % cellCount as libc::c_int) as
                uint16_t;
        /*
        * Format for Voltage Data for single cells is like this:
        *
        *  llll llll cccc hhhh
        *  l: Low voltage bits
        *  h: High voltage bits
        *  c: Cell number (starting at 0)
        *
        * The actual value sent for cell voltage has resolution of 0.002 volts
        * Since vbat has resolution of 0.1 volts it has to be multiplied by 50
        */
        cellVoltage =
            (getBatteryVoltage() as
                 uint32_t).wrapping_mul(100 as libc::c_int as
                                            libc::c_uint).wrapping_add(cellCount
                                                                           as
                                                                           libc::c_uint).wrapping_div((cellCount
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           *
                                                                                                           2
                                                                                                               as
                                                                                                               libc::c_int)
                                                                                                          as
                                                                                                          libc::c_uint)
    } else { currentCell = 0 as libc::c_int as uint16_t }
    // Cell number is at bit 9-12
    let mut data: uint16_t =
        ((currentCell as libc::c_int) << 4 as libc::c_int) as uint16_t;
    // Lower voltage bits are at bit 0-8
    data =
        (data as libc::c_uint |
             (cellVoltage & 0xff as libc::c_int as libc::c_uint) <<
                 8 as libc::c_int) as uint16_t;
    // Higher voltage bits are at bits 13-15
    data =
        (data as libc::c_uint |
             (cellVoltage & 0xf00 as libc::c_int as libc::c_uint) >>
                 8 as libc::c_int) as uint16_t;
    frSkyHubWriteFrame(0x6 as libc::c_int as uint8_t, data as int16_t);
    currentCell = currentCell.wrapping_add(1);
}
/*
 * Send voltage with ID_VOLTAGE_AMP
 */
unsafe extern "C" fn sendVoltageAmp() {
    let mut voltage: uint16_t = getBatteryVoltage();
    let cellCount: uint8_t = getBatteryCellCount();
    if (*telemetryConfig()).frsky_vfas_precision as libc::c_int ==
           FRSKY_VFAS_PRECISION_HIGH as libc::c_int {
        // Use new ID 0x39 to send voltage directly in 0.1 volts resolution
        if (*telemetryConfig()).report_cell_voltage as libc::c_int != 0 &&
               cellCount as libc::c_int != 0 {
            voltage =
                (voltage as libc::c_int / cellCount as libc::c_int) as
                    uint16_t
        }
        frSkyHubWriteFrame(0x39 as libc::c_int as uint8_t,
                           voltage as int16_t);
    } else {
        // send in 0.2 volts resolution
        voltage =
            (voltage as libc::c_int *
                 (110 as libc::c_int / 21 as libc::c_int)) as uint16_t;
        if (*telemetryConfig()).report_cell_voltage as libc::c_int != 0 &&
               cellCount as libc::c_int != 0 {
            voltage =
                (voltage as libc::c_int / cellCount as libc::c_int) as
                    uint16_t
        }
        frSkyHubWriteFrame(0x3a as libc::c_int as uint8_t,
                           (voltage as libc::c_int / 100 as libc::c_int) as
                               int16_t);
        frSkyHubWriteFrame(0x3b as libc::c_int as uint8_t,
                           ((voltage as libc::c_int % 100 as libc::c_int +
                                 5 as libc::c_int) / 10 as libc::c_int) as
                               int16_t);
    };
}
unsafe extern "C" fn sendAmperage() {
    frSkyHubWriteFrame(0x28 as libc::c_int as uint8_t,
                       (getAmperage() / 10 as libc::c_int) as uint16_t as
                           int16_t);
}
unsafe extern "C" fn sendFuelLevel() {
    let mut data: int16_t = 0;
    if (*batteryConfig()).batteryCapacity as libc::c_int > 0 as libc::c_int {
        data = calculateBatteryPercentageRemaining() as uint16_t as int16_t
    } else {
        data =
            constrain(getMAhDrawn(), 0 as libc::c_int, 0xffff as libc::c_int)
                as uint16_t as int16_t
    }
    frSkyHubWriteFrame(0x4 as libc::c_int as uint8_t, data);
}
#[no_mangle]
pub unsafe extern "C" fn initFrSkyHubTelemetry() -> bool {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_UNINITIALIZED as libc::c_int {
        portConfig = findSerialPortConfig(FUNCTION_TELEMETRY_FRSKY_HUB);
        if !portConfig.is_null() {
            frSkyHubPortSharing =
                determinePortSharing(portConfig,
                                     FUNCTION_TELEMETRY_FRSKY_HUB);
            frSkyHubWriteByte =
                Some(frSkyHubWriteByteInternal as
                         unsafe extern "C" fn(_: libc::c_char) -> ());
            telemetryState =
                TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int as uint8_t
        }
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn initFrSkyHubTelemetryExternal(mut frSkyHubWriteByteExternal:
                                                           Option<frSkyHubWriteByteFn>)
 -> bool {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_UNINITIALIZED as libc::c_int {
        frSkyHubWriteByte = frSkyHubWriteByteExternal;
        telemetryState =
            TELEMETRY_STATE_INITIALIZED_EXTERNAL as libc::c_int as uint8_t;
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn freeFrSkyHubTelemetryPort() {
    closeSerialPort(frSkyHubPort);
    frSkyHubPort = 0 as *mut serialPort_t;
}
unsafe extern "C" fn configureFrSkyHubTelemetryPort() {
    if !portConfig.is_null() {
        frSkyHubPort =
            openSerialPort((*portConfig).identifier,
                           FUNCTION_TELEMETRY_FRSKY_HUB, None,
                           0 as *mut libc::c_void,
                           9600 as libc::c_int as uint32_t, MODE_TX,
                           if (*telemetryConfig()).telemetry_inverted as
                                  libc::c_int != 0 {
                               SERIAL_NOT_INVERTED as libc::c_int
                           } else { SERIAL_INVERTED as libc::c_int } as
                               portOptions_e)
    };
}
#[no_mangle]
pub unsafe extern "C" fn checkFrSkyHubTelemetryState() {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int {
        if telemetryCheckRxPortShared(portConfig) {
            if frSkyHubPort.is_null() && !telemetrySharedPort.is_null() {
                frSkyHubPort = telemetrySharedPort
            }
        } else {
            let mut enableSerialTelemetry: bool =
                telemetryDetermineEnabledState(frSkyHubPortSharing);
            if enableSerialTelemetry as libc::c_int != 0 &&
                   frSkyHubPort.is_null() {
                configureFrSkyHubTelemetryPort();
            } else if !enableSerialTelemetry && !frSkyHubPort.is_null() {
                freeFrSkyHubTelemetryPort();
            }
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn processFrSkyHubTelemetry(mut currentTimeUs:
                                                      timeUs_t) {
    static mut frSkyHubLastCycleTime: uint32_t = 0 as libc::c_int as uint32_t;
    static mut cycleNum: uint8_t = 0 as libc::c_int as uint8_t;
    if cmpTimeUs(currentTimeUs, frSkyHubLastCycleTime) < 125000 as libc::c_int
       {
        return
    }
    frSkyHubLastCycleTime = currentTimeUs;
    cycleNum = cycleNum.wrapping_add(1);
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        // Sent every 125ms
        sendAccel();
    }
    // Sent every 1s
    if cycleNum as libc::c_int % 8 as libc::c_int == 0 as libc::c_int {
        sendTemperature1();
        sendThrottleOrBatterySizeAsRpm();
        if isBatteryVoltageConfigured() {
            sendVoltageCells();
            sendVoltageAmp();
            if isAmperageConfigured() { sendAmperage(); sendFuelLevel(); }
        }
    }
    // Sent every 5s
    if cycleNum as libc::c_int == 40 as libc::c_int {
        cycleNum = 0 as libc::c_int as uint8_t;
        sendTime();
    }
    sendTelemetryTail();
}
#[no_mangle]
pub unsafe extern "C" fn handleFrSkyHubTelemetry(mut currentTimeUs:
                                                     timeUs_t) {
    if telemetryState as libc::c_int ==
           TELEMETRY_STATE_INITIALIZED_SERIAL as libc::c_int &&
           !frSkyHubPort.is_null() {
        processFrSkyHubTelemetry(currentTimeUs);
    };
}
