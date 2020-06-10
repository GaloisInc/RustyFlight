use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    // declare available IO pins. Available pins are specified per target
    #[no_mangle]
    fn IORead(io: IO_t) -> bool;
    #[no_mangle]
    fn IOWrite(io: IO_t, value: bool);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn isMPUSoftReset() -> bool;
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
    fn delayMicroseconds(us: timeUs_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    static mut serialPinConfig_System: serialPinConfig_t;
    #[no_mangle]
    fn serialWriteBuf(instance: *mut serialPort_t, data: *const uint8_t,
                      count: libc::c_int);
    #[no_mangle]
    fn findSerialPortConfig(function: serialPortFunction_e)
     -> *mut serialPortConfig_t;
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
    fn saveConfigAndNotify();
    #[no_mangle]
    fn dispatchEnable();
    #[no_mangle]
    fn dispatchAdd(entry: *mut dispatchEntry_t, delayUs: libc::c_int);
    #[no_mangle]
    fn spektrumHandleRSSI(spekFrame_0: *mut uint8_t);
    #[no_mangle]
    fn spektrumHandleVtxControl(vtxControl: uint32_t);
    #[no_mangle]
    static mut telemetrySharedPort: *mut serialPort_t;
    #[no_mangle]
    fn telemetryCheckRxPortShared(portConfig: *const serialPortConfig_t)
     -> bool;
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
    fn srxlCollectTelemetryNow();
    #[no_mangle]
    static mut rxConfig_System: rxConfig_t;
    #[no_mangle]
    fn feature(mask: uint32_t) -> bool;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library. 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup Configuration_Mode_enumeration 
  * @{
  */
pub type C2RustUnnamed = libc::c_uint;
/* !< GPIO Analog In/Out Mode      */
/* !< GPIO Alternate function Mode */
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
/* !< GPIO Output Mode */
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
/* !< GPIO Input Mode */
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
pub type ioTag_t = uint8_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
// packet tag to specify IO pin
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
// millisecond time
pub type timeMs_t = uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPinConfig_s {
    pub ioTagTx: [ioTag_t; 10],
    pub ioTagRx: [ioTag_t; 10],
    pub ioTagInverter: [ioTag_t; 10],
}
pub type serialPinConfig_t = serialPinConfig_s;
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
//
// configuration
//
pub type serialPortConfig_t = serialPortConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dispatchEntry_s {
    pub dispatch: Option<dispatchFunc>,
    pub delayedUntil: uint32_t,
    pub next: *mut dispatchEntry_s,
}
// not used for all telemetry systems, e.g. HoTT only works at 19200.
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
pub type dispatchFunc = unsafe extern "C" fn(_: *mut dispatchEntry_s) -> ();
pub type dispatchEntry_t = dispatchEntry_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct rxConfig_s {
    pub rcmap: [uint8_t; 8],
    pub serialrx_provider: uint8_t,
    pub serialrx_inverted: uint8_t,
    pub halfDuplex: uint8_t,
    pub spektrum_bind_pin_override_ioTag: ioTag_t,
    pub spektrum_bind_plug_ioTag: ioTag_t,
    pub spektrum_sat_bind: uint8_t,
    pub spektrum_sat_bind_autoreset: uint8_t,
    pub rssi_channel: uint8_t,
    pub rssi_scale: uint8_t,
    pub rssi_invert: uint8_t,
    pub midrc: uint16_t,
    pub mincheck: uint16_t,
    pub maxcheck: uint16_t,
    pub rcInterpolation: uint8_t,
    pub rcInterpolationChannels: uint8_t,
    pub rcInterpolationInterval: uint8_t,
    pub fpvCamAngleDegrees: uint8_t,
    pub airModeActivateThreshold: uint8_t,
    pub rx_min_usec: uint16_t,
    pub rx_max_usec: uint16_t,
    pub max_aux_channel: uint8_t,
    pub rssi_src_frame_errors: uint8_t,
    pub rssi_offset: int8_t,
    pub rc_smoothing_type: uint8_t,
    pub rc_smoothing_input_cutoff: uint8_t,
    pub rc_smoothing_derivative_cutoff: uint8_t,
    pub rc_smoothing_debug_axis: uint8_t,
    pub rc_smoothing_input_type: uint8_t,
    pub rc_smoothing_derivative_type: uint8_t,
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
pub type rxConfig_t = rxConfig_s;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed_2 = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed_2 = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed_2 = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed_2 = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const SERIALRX_FPORT: C2RustUnnamed_3 = 12;
pub const SERIALRX_TARGET_CUSTOM: C2RustUnnamed_3 = 11;
pub const SERIALRX_SRXL: C2RustUnnamed_3 = 10;
pub const SERIALRX_CRSF: C2RustUnnamed_3 = 9;
pub const SERIALRX_JETIEXBUS: C2RustUnnamed_3 = 8;
pub const SERIALRX_IBUS: C2RustUnnamed_3 = 7;
pub const SERIALRX_XBUS_MODE_B_RJ01: C2RustUnnamed_3 = 6;
pub const SERIALRX_XBUS_MODE_B: C2RustUnnamed_3 = 5;
pub const SERIALRX_SUMH: C2RustUnnamed_3 = 4;
pub const SERIALRX_SUMD: C2RustUnnamed_3 = 3;
pub const SERIALRX_SBUS: C2RustUnnamed_3 = 2;
pub const SERIALRX_SPEKTRUM2048: C2RustUnnamed_3 = 1;
pub const SERIALRX_SPEKTRUM1024: C2RustUnnamed_3 = 0;
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
// mapping of radio channels to internal RPYTA+ order
// type of UART-based receiver (0 = spek 10, 1 = spek 11, 2 = sbus). Must be enabled by FEATURE_RX_SERIAL first.
// invert the serial RX protocol compared to it's default setting
// allow rx to operate in half duplex mode on F4, ignored for F1 and F3.
// number of bind pulses for Spektrum satellite receivers
// whenever we will reset (exit) binding mode after hard reboot
// Some radios have not a neutral point centered on 1500. can be changed here
// minimum rc end
// maximum rc end
// Camera angle to be scaled into rc commands
// Throttle setpoint percent where airmode gets activated
// true to use frame drop flags in the rx protocol
// offset applied to the RSSI value before it is returned
// Determines the smoothing algorithm to use: INTERPOLATION or FILTER
// Filter cutoff frequency for the input filter (0 = auto)
// Filter cutoff frequency for the setpoint weight derivative filter (0 = auto)
// Axis to log as debug values when debug_mode = RC_SMOOTHING
// Input filter type (0 = PT1, 1 = BIQUAD)
// Derivative filter type (0 = OFF, 1 = PT1, 2 = BIQUAD)
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub const FEATURE_TELEMETRY: C2RustUnnamed_4 = 1024;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const FEATURE_DYNAMIC_FILTER: C2RustUnnamed_4 = 536870912;
pub const FEATURE_ANTI_GRAVITY: C2RustUnnamed_4 = 268435456;
pub const FEATURE_ESC_SENSOR: C2RustUnnamed_4 = 134217728;
pub const FEATURE_SOFTSPI: C2RustUnnamed_4 = 67108864;
pub const FEATURE_RX_SPI: C2RustUnnamed_4 = 33554432;
pub const FEATURE_AIRMODE: C2RustUnnamed_4 = 4194304;
pub const FEATURE_TRANSPONDER: C2RustUnnamed_4 = 2097152;
pub const FEATURE_CHANNEL_FORWARDING: C2RustUnnamed_4 = 1048576;
pub const FEATURE_OSD: C2RustUnnamed_4 = 262144;
pub const FEATURE_DASHBOARD: C2RustUnnamed_4 = 131072;
pub const FEATURE_LED_STRIP: C2RustUnnamed_4 = 65536;
pub const FEATURE_RSSI_ADC: C2RustUnnamed_4 = 32768;
pub const FEATURE_RX_MSP: C2RustUnnamed_4 = 16384;
pub const FEATURE_RX_PARALLEL_PWM: C2RustUnnamed_4 = 8192;
pub const FEATURE_3D: C2RustUnnamed_4 = 4096;
pub const FEATURE_RANGEFINDER: C2RustUnnamed_4 = 512;
pub const FEATURE_GPS: C2RustUnnamed_4 = 128;
pub const FEATURE_SOFTSERIAL: C2RustUnnamed_4 = 64;
pub const FEATURE_SERVO_TILT: C2RustUnnamed_4 = 32;
pub const FEATURE_MOTOR_STOP: C2RustUnnamed_4 = 16;
pub const FEATURE_RX_SERIAL: C2RustUnnamed_4 = 8;
pub const FEATURE_INFLIGHT_ACC_CAL: C2RustUnnamed_4 = 4;
pub const FEATURE_RX_PPM: C2RustUnnamed_4 = 1;
#[inline]
unsafe extern "C" fn serialPinConfig() -> *const serialPinConfig_t {
    return &mut serialPinConfig_System;
}
#[inline]
unsafe extern "C" fn rxConfig() -> *const rxConfig_t {
    return &mut rxConfig_System;
}
// number of RC channels as reported by current input driver
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
// driver for spektrum satellite receiver / sbus
#[no_mangle]
pub static mut srxlEnabled: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut resolution: int32_t = 0;
#[no_mangle]
pub static mut rssi_channel: uint8_t = 0;
static mut spek_chan_shift: uint8_t = 0;
static mut spek_chan_mask: uint8_t = 0;
static mut rcFrameComplete: bool = 0 as libc::c_int != 0;
static mut spekHiRes: bool = 0 as libc::c_int != 0;
static mut spekFrame: [uint8_t; 16] = [0; 16];
static mut rxRuntimeConfigPtr: *mut rxRuntimeConfig_t =
    0 as *const rxRuntimeConfig_t as *mut rxRuntimeConfig_t;
static mut serialPort: *mut serialPort_t =
    0 as *const serialPort_t as *mut serialPort_t;
static mut telemetryBuf: [uint8_t; 21] = [0; 21];
static mut telemetryBufLen: uint8_t = 0 as libc::c_int as uint8_t;
static mut srxlTelemetryDispatch: dispatchEntry_t =
    unsafe {
        {
            let mut init =
                dispatchEntry_s{dispatch:
                                    Some(srxlRxSendTelemetryDataDispatch as
                                             unsafe extern "C" fn(_:
                                                                      *mut dispatchEntry_t)
                                                 -> ()),
                                delayedUntil: 0,
                                next:
                                    0 as *const dispatchEntry_s as
                                        *mut dispatchEntry_s,};
            init
        }
    };
// Receive ISR callback
unsafe extern "C" fn spektrumDataReceive(mut c: uint16_t,
                                         mut data: *mut libc::c_void) {
    let mut spekTime: uint32_t = 0;
    let mut spekTimeInterval: uint32_t = 0;
    static mut spekTimeLast: uint32_t = 0 as libc::c_int as uint32_t;
    static mut spekFramePosition: uint8_t = 0 as libc::c_int as uint8_t;
    spekTime = micros();
    spekTimeInterval = spekTime.wrapping_sub(spekTimeLast);
    spekTimeLast = spekTime;
    if spekTimeInterval > 5000 as libc::c_int as libc::c_uint {
        spekFramePosition = 0 as libc::c_int as uint8_t
    }
    if (spekFramePosition as libc::c_int) < 16 as libc::c_int {
        let fresh0 = spekFramePosition;
        spekFramePosition = spekFramePosition.wrapping_add(1);
        ::core::ptr::write_volatile(&mut spekFrame[fresh0 as usize] as
                                        *mut uint8_t, c as uint8_t);
        if (spekFramePosition as libc::c_int) < 16 as libc::c_int {
            rcFrameComplete = 0 as libc::c_int != 0
        } else { rcFrameComplete = 1 as libc::c_int != 0 }
    };
}
#[no_mangle]
pub static mut spekChannelData: [uint32_t; 12] = [0; 12];
unsafe extern "C" fn spektrumFrameStatus(mut rxRuntimeConfig:
                                             *mut rxRuntimeConfig_t)
 -> uint8_t {
    if !rcFrameComplete { return RX_FRAME_PENDING as libc::c_int as uint8_t }
    rcFrameComplete = 0 as libc::c_int != 0;
    spektrumHandleRSSI(spekFrame.as_mut_ptr());
    // Get the VTX control bytes in a frame
    let mut vtxControl: uint32_t =
        ((spekFrame[(16 as libc::c_int - 4 as libc::c_int) as usize] as
              libc::c_int) << 24 as libc::c_int |
             (spekFrame[(16 as libc::c_int - 3 as libc::c_int) as usize] as
                  libc::c_int) << 16 as libc::c_int |
             (spekFrame[(16 as libc::c_int - 2 as libc::c_int) as usize] as
                  libc::c_int) << 8 as libc::c_int |
             (spekFrame[(16 as libc::c_int - 1 as libc::c_int) as usize] as
                  libc::c_int) << 0 as libc::c_int) as uint32_t;
    let mut spektrumRcDataSize: int8_t = 0;
    // Handle VTX control frame.
    if vtxControl & 0xf000f000 as libc::c_uint == 0xe000e000 as libc::c_uint
           &&
           spekFrame[2 as libc::c_int as usize] as libc::c_int &
               0x80 as libc::c_int == 0 as libc::c_int {
        spektrumHandleVtxControl(vtxControl);
        spektrumRcDataSize = (16 as libc::c_int - 4 as libc::c_int) as int8_t
    } else { spektrumRcDataSize = 16 as libc::c_int as int8_t }
    // Get the RC control channel inputs
    let mut b: libc::c_int = 3 as libc::c_int;
    while b < spektrumRcDataSize as libc::c_int {
        let spekChannel: uint8_t =
            (0xf as libc::c_int &
                 spekFrame[(b - 1 as libc::c_int) as usize] as libc::c_int >>
                     spek_chan_shift as libc::c_int) as uint8_t;
        if (spekChannel as libc::c_int) <
               (*rxRuntimeConfigPtr).channelCount as libc::c_int &&
               (spekChannel as libc::c_int) < 12 as libc::c_int {
            if rssi_channel as libc::c_int == 0 as libc::c_int ||
                   spekChannel as libc::c_int != rssi_channel as libc::c_int {
                spekChannelData[spekChannel as usize] =
                    (((spekFrame[(b - 1 as libc::c_int) as usize] as
                           libc::c_int & spek_chan_mask as libc::c_int) as
                          uint32_t) <<
                         8 as
                             libc::c_int).wrapping_add(spekFrame[b as usize]
                                                           as libc::c_uint)
            }
        }
        b += 2 as libc::c_int
    }
    if srxlEnabled {
        /* Only dispatch for transmission if there are some data in buffer AND servos in phase 0 */
        if telemetryBufLen as libc::c_int != 0 &&
               spekFrame[2 as libc::c_int as usize] as libc::c_int &
                   0x80 as libc::c_int == 0 as libc::c_int {
            dispatchAdd(&mut srxlTelemetryDispatch, 1000 as libc::c_int);
        }
        /* Trigger tm data collection if buffer has been sent and is empty, 
           so data will be ready to transmit in the next phase 0 */
        if telemetryBufLen as libc::c_int == 0 as libc::c_int {
            srxlCollectTelemetryNow(); // 1024 mode
        }
    } // 2048 mode
    return RX_FRAME_COMPLETE as libc::c_int as uint8_t;
}
unsafe extern "C" fn spektrumReadRawRC(mut rxRuntimeConfig:
                                           *const rxRuntimeConfig_t,
                                       mut chan: uint8_t) -> uint16_t {
    let mut data: uint16_t = 0;
    if chan as libc::c_int >= (*rxRuntimeConfig).channelCount as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    if spekHiRes {
        data =
            (988 as libc::c_int as
                 libc::c_uint).wrapping_add(spekChannelData[chan as usize] >>
                                                1 as libc::c_int) as uint16_t
    } else {
        data =
            (988 as libc::c_int as
                 libc::c_uint).wrapping_add(spekChannelData[chan as usize]) as
                uint16_t
    }
    return data;
}
#[no_mangle]
pub unsafe extern "C" fn spekShouldBind(mut spektrum_sat_bind: uint8_t)
 -> bool {
    let mut BindPlug: IO_t =
        IOGetByTag((*rxConfig()).spektrum_bind_plug_ioTag);
    if !BindPlug.is_null() {
        IOInit(BindPlug, OWNER_RX_BIND, 0 as libc::c_int as uint8_t);
        IOConfigGPIO(BindPlug,
                     (GPIO_Mode_IN as libc::c_int |
                          (0 as libc::c_int) << 2 as libc::c_int |
                          (0 as libc::c_int) << 4 as libc::c_int |
                          (GPIO_PuPd_UP as libc::c_int) << 5 as libc::c_int)
                         as ioConfig_t);
        // Check status of bind plug and exit if not active
        delayMicroseconds(10 as libc::c_int as
                              timeUs_t); // allow configuration to settle
        if IORead(BindPlug) { return 0 as libc::c_int != 0 }
    }
    // USE_SPEKTRUM_BIND_PLUG
    return !(isMPUSoftReset() as libc::c_int != 0 ||
                 spektrum_sat_bind as libc::c_int == 0 as libc::c_int ||
                 spektrum_sat_bind as libc::c_int > 10 as libc::c_int);
}
// Spektrum system type values
// Stores the RX RSSI channel.
/* spektrumBind function ported from Baseflight. It's used to bind satellite receiver to TX.
 * Function must be called immediately after startup so that we don't miss satellite bind window.
 * Known parameters. Tested with DSMX satellite and DX8 radio. Framerate (11ms or 22ms) must be selected from TX.
 * 9 = DSMX 11ms / DSMX 22ms
 * 5 = DSM2 11ms 2048 / DSM2 22ms 1024
 */
#[no_mangle]
pub unsafe extern "C" fn spektrumBind(mut rxConfig_0: *mut rxConfig_t) {
    if !spekShouldBind((*rxConfig_0).spektrum_sat_bind) { return }
    // Determine a pin to use
    let mut bindPin: ioTag_t = 0 as libc::c_int as ioTag_t;
    if (*rxConfig_0).spektrum_bind_pin_override_ioTag != 0 {
        bindPin = (*rxConfig_0).spektrum_bind_pin_override_ioTag
    } else {
        let mut portConfig: *const serialPortConfig_t =
            findSerialPortConfig(FUNCTION_RX_SERIAL);
        if portConfig.is_null() { return }
        let mut index: libc::c_int =
            if (*portConfig).identifier as libc::c_int <=
                   SERIAL_PORT_USART8 as libc::c_int {
                (*portConfig).identifier as libc::c_int
            } else {
                (10 as libc::c_int) +
                    ((*portConfig).identifier as libc::c_int -
                         SERIAL_PORT_SOFTSERIAL1 as libc::c_int)
            };
        let mut txPin: ioTag_t = (*serialPinConfig()).ioTagTx[index as usize];
        let mut rxPin: ioTag_t = (*serialPinConfig()).ioTagRx[index as usize];
        // Take care half-duplex case
        match (*rxConfig_0).serialrx_provider as libc::c_int {
            10 => {
                if feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) as
                       libc::c_int != 0 &&
                       !telemetryCheckRxPortShared(portConfig) {
                    bindPin = txPin
                }
            }
            _ => {
                // USE_TELEMETRY && USE_TELEMETRY_SRXL
                bindPin =
                    if (*rxConfig_0).halfDuplex as libc::c_int != 0 {
                        txPin as libc::c_int
                    } else { rxPin as libc::c_int } as ioTag_t
            }
        }
        if bindPin == 0 { return }
    }
    let mut bindIO: IO_t = IOGetByTag(bindPin);
    IOInit(bindIO, OWNER_RX_BIND, 0 as libc::c_int as uint8_t);
    IOConfigGPIO(bindIO,
                 (GPIO_Mode_OUT as libc::c_int |
                      (0 as libc::c_int) << 2 as libc::c_int |
                      (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    // RX line, set high
    IOWrite(bindIO, 1 as libc::c_int != 0);
    // Bind window is around 20-140ms after powerup
    delay(60 as libc::c_int as timeMs_t);
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < (*rxConfig_0).spektrum_sat_bind as libc::c_int {
        ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
        ledSet(2 as libc::c_int, 0 as libc::c_int != 0);
        // RX line, drive low for 120us
        IOWrite(bindIO, 0 as libc::c_int != 0);
        delayMicroseconds(120 as libc::c_int as timeUs_t);
        ledSet(0 as libc::c_int, 1 as libc::c_int != 0);
        ledSet(2 as libc::c_int, 1 as libc::c_int != 0);
        // RX line, drive high for 120us
        IOWrite(bindIO, 1 as libc::c_int != 0);
        delayMicroseconds(120 as libc::c_int as timeUs_t);
        i += 1
    }
    // Release the bind pin to avoid interference with an actual rx pin,
    // when rxConfig->spektrum_bind_pin_override_ioTag is used.
    // This happens when the bind pin is connected in parallel to the rx pin.
    if (*rxConfig_0).spektrum_bind_pin_override_ioTag != 0 {
        delay(50 as libc::c_int as timeMs_t); // Keep it high for 50msec
        IOConfigGPIO(bindIO,
                     (GPIO_Mode_IN as libc::c_int |
                          (0 as libc::c_int) << 2 as libc::c_int |
                          (0 as libc::c_int) << 4 as libc::c_int |
                          (GPIO_PuPd_NOPULL as libc::c_int) <<
                              5 as libc::c_int) as ioConfig_t);
    }
    // If we came here as a result of hard  reset (power up, with spektrum_sat_bind set), then reset it back to zero and write config
    // Don't reset if hardware bind plug is present
    // Reset only when autoreset is enabled
    if (*rxConfig_0).spektrum_bind_plug_ioTag == 0 &&
           (*rxConfig_0).spektrum_sat_bind_autoreset as libc::c_int ==
               1 as libc::c_int && !isMPUSoftReset() {
        (*rxConfig_0).spektrum_sat_bind = 0 as libc::c_int as uint8_t;
        saveConfigAndNotify();
    };
}
// USE_SPEKTRUM_BIND
#[no_mangle]
pub unsafe extern "C" fn spektrumInit(mut rxConfig_0: *const rxConfig_t,
                                      mut rxRuntimeConfig:
                                          *mut rxRuntimeConfig_t) -> bool {
    rxRuntimeConfigPtr = rxRuntimeConfig;
    let mut portConfig: *const serialPortConfig_t =
        findSerialPortConfig(FUNCTION_RX_SERIAL);
    if portConfig.is_null() { return 0 as libc::c_int != 0 }
    srxlEnabled = 0 as libc::c_int != 0;
    let mut portShared: bool = telemetryCheckRxPortShared(portConfig);
    let mut current_block_19: u64;
    match (*rxConfig_0).serialrx_provider as libc::c_int {
        10 => {
            srxlEnabled =
                feature(FEATURE_TELEMETRY as libc::c_int as uint32_t) as
                    libc::c_int != 0 && !portShared;
            current_block_19 = 6368400105866424821;
        }
        1 => { current_block_19 = 6368400105866424821; }
        0 => {
            // 10 bit frames
            spek_chan_shift = 2 as libc::c_int as uint8_t;
            spek_chan_mask = 0x3 as libc::c_int as uint8_t;
            spekHiRes = 0 as libc::c_int != 0;
            resolution = 1024 as libc::c_int;
            (*rxRuntimeConfig).channelCount = 7 as libc::c_int as uint8_t;
            (*rxRuntimeConfig).rxRefreshRate =
                22000 as libc::c_int as uint16_t;
            current_block_19 = 12147880666119273379;
        }
        _ => { current_block_19 = 12147880666119273379; }
    }
    match current_block_19 {
        6368400105866424821 => {
            // 11 bit frames
            spek_chan_shift =
                3 as libc::c_int as
                    uint8_t; // -1 because rxConfig->rssi_channel is 1-based and rssi_channel is 0-based.
            spek_chan_mask = 0x7 as libc::c_int as uint8_t;
            spekHiRes = 1 as libc::c_int != 0;
            resolution = 2048 as libc::c_int;
            (*rxRuntimeConfig).channelCount = 12 as libc::c_int as uint8_t;
            (*rxRuntimeConfig).rxRefreshRate =
                11000 as libc::c_int as uint16_t
        }
        _ => { }
    }
    (*rxRuntimeConfig).rcReadRawFn =
        Some(spektrumReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(spektrumFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    serialPort =
        openSerialPort((*portConfig).identifier, FUNCTION_RX_SERIAL,
                       Some(spektrumDataReceive as
                                unsafe extern "C" fn(_: uint16_t,
                                                     _: *mut libc::c_void)
                                    -> ()), 0 as *mut libc::c_void,
                       115200 as libc::c_int as uint32_t,
                       if portShared as libc::c_int != 0 ||
                              srxlEnabled as libc::c_int != 0 {
                           MODE_RXTX as libc::c_int
                       } else { MODE_RX as libc::c_int } as portMode_e,
                       ((if (*rxConfig_0).serialrx_inverted as libc::c_int !=
                                0 {
                             SERIAL_INVERTED as libc::c_int
                         } else { 0 as libc::c_int }) |
                            (if srxlEnabled as libc::c_int != 0 ||
                                    (*rxConfig_0).halfDuplex as libc::c_int !=
                                        0 {
                                 SERIAL_BIDIR as libc::c_int
                             } else { 0 as libc::c_int })) as portOptions_e);
    if portShared { telemetrySharedPort = serialPort }
    rssi_channel =
        ((*rxConfig_0).rssi_channel as libc::c_int - 1 as libc::c_int) as
            uint8_t;
    if rssi_channel as libc::c_int >=
           (*rxRuntimeConfig).channelCount as libc::c_int {
        rssi_channel = 0 as libc::c_int as uint8_t
    }
    if !serialPort.is_null() && srxlEnabled as libc::c_int != 0 {
        dispatchEnable();
    }
    return !serialPort.is_null();
}
#[no_mangle]
pub unsafe extern "C" fn srxlRxWriteTelemetryData(mut data:
                                                      *const libc::c_void,
                                                  mut len: libc::c_int) {
    len =
        ({
             let mut _a: libc::c_int = len;
             let mut _b: libc::c_int =
                 ::core::mem::size_of::<[uint8_t; 21]>() as libc::c_ulong as
                     libc::c_int;
             if _a < _b { _a } else { _b }
         });
    memcpy(telemetryBuf.as_mut_ptr() as *mut libc::c_void, data,
           len as libc::c_ulong);
    telemetryBufLen = len as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn srxlRxSendTelemetryDataDispatch(mut self_0:
                                                             *mut dispatchEntry_t) {
    // if there is telemetry data to write
    if telemetryBufLen as libc::c_int > 0 as libc::c_int {
        serialWriteBuf(serialPort, telemetryBuf.as_mut_ptr(),
                       telemetryBufLen as libc::c_int);
        telemetryBufLen = 0 as libc::c_int as uint8_t
        // reset telemetry buffer
    };
}
#[no_mangle]
pub unsafe extern "C" fn srxlRxIsActive() -> bool {
    return !serialPort.is_null();
}
// SERIAL_RX
