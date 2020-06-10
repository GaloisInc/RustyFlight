use ::libc;
extern "C" {
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
    #[no_mangle]
    static mut gyro: gyro_t;
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
    static mut isRXDataNew: bool;
    #[no_mangle]
    fn processRx(currentTimeUs: timeUs_t) -> bool;
    #[no_mangle]
    fn updateArmingStatus();
    #[no_mangle]
    fn taskMainPidLoop(currentTimeUs: timeUs_t);
    #[no_mangle]
    static mut currentRxRefreshRate: uint16_t;
    #[no_mangle]
    fn updateRcCommands();
    #[no_mangle]
    fn dispatchIsEnabled() -> bool;
    #[no_mangle]
    fn dispatchProcess(currentTime: uint32_t);
    #[no_mangle]
    static mut armingFlags: uint8_t;
    #[no_mangle]
    fn sensors(mask: uint32_t) -> bool;
    #[no_mangle]
    fn imuUpdateAttitude(currentTimeUs: timeUs_t);
    // msp post process function, used for gracefully handling reboots, etc.
    #[no_mangle]
    fn mspFcProcessReply(reply: *mut mspPacket_t);
    #[no_mangle]
    fn mspFcProcessCommand(cmd: *mut mspPacket_t, reply: *mut mspPacket_t,
                           mspPostProcessFn: *mut mspPostProcessFnPtr)
     -> mspResult_e;
    #[no_mangle]
    static mut serialConfig_System: serialConfig_t;
    #[no_mangle]
    fn mspSerialProcess(evaluateNonMspData: mspEvaluateNonMspData_e,
                        mspProcessCommandFn: mspProcessCommandFnPtr,
                        mspProcessReplyFn: mspProcessReplyFnPtr);
    #[no_mangle]
    fn rxUpdateCheck(currentTimeUs: timeUs_t, currentDeltaTimeUs: timeDelta_t)
     -> bool;
    #[no_mangle]
    static mut acc: acc_t;
    #[no_mangle]
    static mut accelerometerConfig_System: accelerometerConfig_t;
    #[no_mangle]
    fn accUpdate(currentTimeUs: timeUs_t,
                 rollAndPitchTrims: *mut rollAndPitchTrims_t);
    #[no_mangle]
    static mut batteryConfig_System: batteryConfig_t;
    #[no_mangle]
    fn batteryUpdateVoltage(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn batteryUpdateStates(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn batteryUpdateAlarms();
    #[no_mangle]
    fn batteryUpdatePresence();
    #[no_mangle]
    fn batteryUpdateCurrentMeter(currentTimeUs: timeUs_t);
    #[no_mangle]
    fn rescheduleTask(taskId: cfTaskId_e, newPeriodMicros: uint32_t);
    #[no_mangle]
    fn setTaskEnabled(taskId: cfTaskId_e, newEnabledState: bool);
    #[no_mangle]
    fn schedulerInit();
    #[no_mangle]
    fn taskSystemLoad(currentTime: timeUs_t);
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
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
// 0 - 359
// 0 - 255
// 0 - 255
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
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
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
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
// Slave I2C on SPI master
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
pub type sensor_align_e = libc::c_uint;
pub const CW270_DEG_FLIP: sensor_align_e = 8;
pub const CW180_DEG_FLIP: sensor_align_e = 7;
pub const CW90_DEG_FLIP: sensor_align_e = 6;
pub const CW0_DEG_FLIP: sensor_align_e = 5;
pub const CW270_DEG: sensor_align_e = 4;
pub const CW180_DEG: sensor_align_e = 3;
pub const CW90_DEG: sensor_align_e = 2;
// driver-provided alignment
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct gyro_s {
    pub targetLooptime: uint32_t,
    pub gyroADCf: [libc::c_float; 3],
}
pub type gyro_t = gyro_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub union rollAndPitchTrims_u {
    pub raw: [int16_t; 2],
    pub values: rollAndPitchTrims_t_def,
}
pub type rollAndPitchTrims_t_def = rollAndPitchTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rollAndPitchTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
}
pub type C2RustUnnamed_1 = libc::c_uint;
pub const WAS_ARMED_WITH_PREARM: C2RustUnnamed_1 = 4;
pub const WAS_EVER_ARMED: C2RustUnnamed_1 = 2;
pub const ARMED: C2RustUnnamed_1 = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialConfig_s {
    pub portConfigs: [serialPortConfig_t; 2],
    pub serial_update_rate_hz: uint16_t,
    pub reboot_character: uint8_t,
}
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
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
pub struct sbuf_s {
    pub ptr: *mut uint8_t,
    pub end: *mut uint8_t,
}
pub type sbuf_t = sbuf_s;
pub type mspResult_e = libc::c_int;
pub const MSP_RESULT_CMD_UNKNOWN: mspResult_e = -2;
pub const MSP_RESULT_NO_REPLY: mspResult_e = 0;
pub const MSP_RESULT_ERROR: mspResult_e = -1;
pub const MSP_RESULT_ACK: mspResult_e = 1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mspPacket_s {
    pub buf: sbuf_t,
    pub cmd: int16_t,
    pub flags: uint8_t,
    pub result: int16_t,
    pub direction: uint8_t,
}
pub type mspPacket_t = mspPacket_s;
pub type mspPostProcessFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut serialPort_s) -> ()>;
pub type mspProcessCommandFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut mspPacket_t, _: *mut mspPacket_t,
                                _: *mut mspPostProcessFnPtr) -> mspResult_e>;
pub type mspProcessReplyFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut mspPacket_t) -> ()>;
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
pub type serialConfig_t = serialConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rcdeviceSwitchState_s {
    pub isActivated: bool,
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
// which byte is used to reboot. Default 'R', could be changed carefully to something else.
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
pub type rcdeviceSwitchState_t = rcdeviceSwitchState_s;
pub type mspEvaluateNonMspData_e = libc::c_uint;
pub const MSP_SKIP_NON_MSP_DATA: mspEvaluateNonMspData_e = 1;
pub const MSP_EVALUATE_NON_MSP_DATA: mspEvaluateNonMspData_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct int16_flightDynamicsTrims_s {
    pub roll: int16_t,
    pub pitch: int16_t,
    pub yaw: int16_t,
}
pub type flightDynamicsTrims_def_t = int16_flightDynamicsTrims_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub union flightDynamicsTrims_u {
    pub raw: [int16_t; 3],
    pub values: flightDynamicsTrims_def_t,
}
pub type flightDynamicsTrims_t = flightDynamicsTrims_u;
// 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_2 = 64;
pub const SENSOR_GPS: C2RustUnnamed_2 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_2 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_2 = 16;
pub const SENSOR_MAG: C2RustUnnamed_2 = 8;
pub const SENSOR_BARO: C2RustUnnamed_2 = 4;
// always present
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
pub type rollAndPitchTrims_t = rollAndPitchTrims_u;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct accelerometerConfig_s {
    pub acc_lpf_hz: uint16_t,
    pub acc_align: sensor_align_e,
    pub acc_hardware: uint8_t,
    pub acc_high_fsr: bool,
    pub accZero: flightDynamicsTrims_t,
    pub accelerometerTrims: rollAndPitchTrims_t,
}
pub type accelerometerConfig_t = accelerometerConfig_s;
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const TASK_PRIORITY_MAX: C2RustUnnamed_3 = 255;
pub const TASK_PRIORITY_REALTIME: C2RustUnnamed_3 = 6;
pub const TASK_PRIORITY_HIGH: C2RustUnnamed_3 = 5;
pub const TASK_PRIORITY_MEDIUM_HIGH: C2RustUnnamed_3 = 4;
pub const TASK_PRIORITY_MEDIUM: C2RustUnnamed_3 = 3;
pub const TASK_PRIORITY_LOW: C2RustUnnamed_3 = 1;
pub const TASK_PRIORITY_IDLE: C2RustUnnamed_3 = 0;
pub type cfTaskId_e = libc::c_uint;
pub const TASK_SELF: cfTaskId_e = 12;
pub const TASK_NONE: cfTaskId_e = 11;
pub const TASK_COUNT: cfTaskId_e = 11;
pub const TASK_BATTERY_ALERTS: cfTaskId_e = 10;
pub const TASK_BATTERY_CURRENT: cfTaskId_e = 9;
pub const TASK_BATTERY_VOLTAGE: cfTaskId_e = 8;
pub const TASK_DISPATCH: cfTaskId_e = 7;
pub const TASK_SERIAL: cfTaskId_e = 6;
pub const TASK_RX: cfTaskId_e = 5;
pub const TASK_ATTITUDE: cfTaskId_e = 4;
pub const TASK_ACCEL: cfTaskId_e = 3;
pub const TASK_GYROPID: cfTaskId_e = 2;
pub const TASK_MAIN: cfTaskId_e = 1;
pub const TASK_SYSTEM: cfTaskId_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct cfTask_t {
    pub taskName: *const libc::c_char,
    pub subTaskName: *const libc::c_char,
    pub checkFunc: Option<unsafe extern "C" fn(_: timeUs_t, _: timeDelta_t)
                              -> bool>,
    pub taskFunc: Option<unsafe extern "C" fn(_: timeUs_t) -> ()>,
    pub desiredPeriod: timeDelta_t,
    pub staticPriority: uint8_t,
    pub dynamicPriority: uint16_t,
    pub taskAgeCycles: uint16_t,
    pub taskLatestDeltaTime: timeDelta_t,
    pub lastExecutedAt: timeUs_t,
    pub lastSignaledAt: timeUs_t,
    pub movingSumExecutionTime: timeUs_t,
    pub maxExecutionTime: timeUs_t,
    pub totalExecutionTime: timeUs_t,
}
#[no_mangle]
pub static mut pCurrentDisplay: *mut displayPort_t =
    0 as *const displayPort_t as *mut displayPort_t;
#[inline]
unsafe extern "C" fn constrain(mut amt: libc::c_int, mut low: libc::c_int,
                               mut high: libc::c_int) -> libc::c_int {
    if amt < low {
        return low
    } else if amt > high { return high } else { return amt };
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
unsafe extern "C" fn serialConfig() -> *const serialConfig_t {
    return &mut serialConfig_System;
}
#[no_mangle]
pub static mut switchStates: [rcdeviceSwitchState_t; 3] =
    [rcdeviceSwitchState_t{isActivated: false,}; 3];
#[inline]
unsafe extern "C" fn accelerometerConfigMutable()
 -> *mut accelerometerConfig_t {
    return &mut accelerometerConfig_System;
}
#[inline]
unsafe extern "C" fn batteryConfig() -> *const batteryConfig_t {
    return &mut batteryConfig_System;
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
unsafe extern "C" fn taskMain(mut currentTimeUs: timeUs_t) { }
unsafe extern "C" fn taskHandleSerial(mut currentTimeUs: timeUs_t) {
    let mut evaluateMspData: bool =
        if armingFlags as libc::c_int & ARMED as libc::c_int != 0 {
            MSP_SKIP_NON_MSP_DATA as libc::c_int
        } else { MSP_EVALUATE_NON_MSP_DATA as libc::c_int } != 0;
    mspSerialProcess(evaluateMspData as mspEvaluateNonMspData_e,
                     Some(mspFcProcessCommand as
                              unsafe extern "C" fn(_: *mut mspPacket_t,
                                                   _: *mut mspPacket_t,
                                                   _:
                                                       *mut mspPostProcessFnPtr)
                                  -> mspResult_e),
                     Some(mspFcProcessReply as
                              unsafe extern "C" fn(_: *mut mspPacket_t)
                                  -> ()));
}
unsafe extern "C" fn taskBatteryAlerts(mut currentTimeUs: timeUs_t) {
    if armingFlags as libc::c_int & ARMED as libc::c_int == 0 {
        // the battery *might* fall out in flight, but if that happens the FC will likely be off too unless the user has battery backup.
        batteryUpdatePresence();
    }
    batteryUpdateStates(currentTimeUs);
    batteryUpdateAlarms();
}
unsafe extern "C" fn taskUpdateAccelerometer(mut currentTimeUs: timeUs_t) {
    accUpdate(currentTimeUs,
              &mut (*(accelerometerConfigMutable as
                          unsafe extern "C" fn()
                              ->
                                  *mut accelerometerConfig_t)()).accelerometerTrims);
}
unsafe extern "C" fn taskUpdateRxMain(mut currentTimeUs: timeUs_t) {
    if !processRx(currentTimeUs) { return }
    static mut lastRxTimeUs: timeUs_t = 0;
    currentRxRefreshRate =
        constrain(currentTimeUs.wrapping_sub(lastRxTimeUs) as libc::c_int,
                  1000 as libc::c_int, 30000 as libc::c_int) as uint16_t;
    lastRxTimeUs = currentTimeUs;
    isRXDataNew = 1 as libc::c_int != 0;
    // updateRcCommands sets rcCommand, which is needed by updateAltHoldState and updateSonarAltHoldState
    updateRcCommands();
    updateArmingStatus();
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
// Prevent too long busy wait times
// USE_BARO || USE_GPS
#[no_mangle]
pub unsafe extern "C" fn fcTasksInit() {
    schedulerInit();
    setTaskEnabled(TASK_MAIN, 1 as libc::c_int != 0);
    setTaskEnabled(TASK_SERIAL, 1 as libc::c_int != 0);
    rescheduleTask(TASK_SERIAL,
                   (1000000 as libc::c_int /
                        (*serialConfig()).serial_update_rate_hz as
                            libc::c_int) as uint32_t);
    let useBatteryVoltage: bool =
        (*batteryConfig()).voltageMeterSource as libc::c_uint !=
            VOLTAGE_METER_NONE as libc::c_int as libc::c_uint;
    setTaskEnabled(TASK_BATTERY_VOLTAGE, useBatteryVoltage);
    let useBatteryCurrent: bool =
        (*batteryConfig()).currentMeterSource as libc::c_uint !=
            CURRENT_METER_NONE as libc::c_int as libc::c_uint;
    setTaskEnabled(TASK_BATTERY_CURRENT, useBatteryCurrent);
    let useBatteryAlerts: bool =
        (*batteryConfig()).useVBatAlerts as libc::c_int != 0 ||
            (*batteryConfig()).useConsumptionAlerts as libc::c_int != 0 ||
            feature(FEATURE_OSD as libc::c_int as uint32_t) as libc::c_int !=
                0;
    setTaskEnabled(TASK_BATTERY_ALERTS,
                   (useBatteryVoltage as libc::c_int != 0 ||
                        useBatteryCurrent as libc::c_int != 0) &&
                       useBatteryAlerts as libc::c_int != 0);
    if sensors(SENSOR_GYRO as libc::c_int as uint32_t) {
        rescheduleTask(TASK_GYROPID, gyro.targetLooptime);
        setTaskEnabled(TASK_GYROPID, 1 as libc::c_int != 0);
    }
    if sensors(SENSOR_ACC as libc::c_int as uint32_t) {
        setTaskEnabled(TASK_ACCEL, 1 as libc::c_int != 0);
        rescheduleTask(TASK_ACCEL, acc.accSamplingInterval);
        setTaskEnabled(TASK_ATTITUDE, 1 as libc::c_int != 0);
    }
    setTaskEnabled(TASK_RX, 1 as libc::c_int != 0);
    setTaskEnabled(TASK_DISPATCH, dispatchIsEnabled());
}
#[no_mangle]
pub static mut cfTasks: [cfTask_t; 11] =
    unsafe {
        [{
             let mut init =
                 cfTask_t{taskName:
                              b"SYSTEM\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName:
                              b"LOAD\x00" as *const u8 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskSystemLoad as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 10 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM_HIGH as libc::c_int as
                                  uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"SYSTEM\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName:
                              b"UPDATE\x00" as *const u8 as
                                  *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskMain as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 1000 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM_HIGH as libc::c_int as
                                  uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"PID\x00" as *const u8 as *const libc::c_char,
                          subTaskName:
                              b"GYRO\x00" as *const u8 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskMainPidLoop as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod: 1000 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_REALTIME as libc::c_int as
                                  uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"ACC\x00" as *const u8 as *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskUpdateAccelerometer as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 1000 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"ATTITUDE\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(imuUpdateAttitude as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 100 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"RX\x00" as *const u8 as *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc:
                              Some(rxUpdateCheck as
                                       unsafe extern "C" fn(_: timeUs_t,
                                                            _: timeDelta_t)
                                           -> bool),
                          taskFunc:
                              Some(taskUpdateRxMain as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 33 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_HIGH as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"SERIAL\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskHandleSerial as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 100 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_LOW as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"DISPATCH\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(dispatchProcess as
                                       unsafe extern "C" fn(_: uint32_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 1000 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_HIGH as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"BATTERY_VOLTAGE\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(batteryUpdateVoltage as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 50 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"BATTERY_CURRENT\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(batteryUpdateCurrentMeter as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 50 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         },
         {
             let mut init =
                 cfTask_t{taskName:
                              b"BATTERY_ALERTS\x00" as *const u8 as
                                  *const libc::c_char,
                          subTaskName: 0 as *const libc::c_char,
                          checkFunc: None,
                          taskFunc:
                              Some(taskBatteryAlerts as
                                       unsafe extern "C" fn(_: timeUs_t)
                                           -> ()),
                          desiredPeriod:
                              1000000 as libc::c_int / 5 as libc::c_int,
                          staticPriority:
                              TASK_PRIORITY_MEDIUM as libc::c_int as uint8_t,
                          dynamicPriority: 0,
                          taskAgeCycles: 0,
                          taskLatestDeltaTime: 0,
                          lastExecutedAt: 0,
                          lastSignaledAt: 0,
                          movingSumExecutionTime: 0,
                          maxExecutionTime: 0,
                          totalExecutionTime: 0,};
             init
         }]
    };