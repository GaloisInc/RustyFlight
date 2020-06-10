use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn biquadFilterInitLPF(filter: *mut biquadFilter_t,
                           filterFreq: libc::c_float, refreshRate: uint32_t);
    #[no_mangle]
    fn biquadFilterApply(filter: *mut biquadFilter_t, input: libc::c_float)
     -> libc::c_float;
    #[no_mangle]
    fn adcGetChannel(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn getVrefMv() -> uint16_t;
    #[no_mangle]
    fn getEscSensorData(motorNumber: uint8_t) -> *mut escSensorData_t;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct biquadFilter_s {
    pub b0: libc::c_float,
    pub b1: libc::c_float,
    pub b2: libc::c_float,
    pub a1: libc::c_float,
    pub a2: libc::c_float,
    pub x1: libc::c_float,
    pub x2: libc::c_float,
    pub y1: libc::c_float,
    pub y2: libc::c_float,
}
/* this holds the data required to update samples thru a filter */
pub type biquadFilter_t = biquadFilter_s;
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const ADC_CHANNEL_COUNT: C2RustUnnamed_1 = 4;
pub const ADC_RSSI: C2RustUnnamed_1 = 3;
pub const ADC_EXTERNAL1: C2RustUnnamed_1 = 2;
pub const ADC_CURRENT: C2RustUnnamed_1 = 1;
pub const ADC_BATTERY: C2RustUnnamed_1 = 0;
pub type voltageMeterId_e = libc::c_uint;
pub const VOLTAGE_METER_ID_CELL_40: voltageMeterId_e = 119;
pub const VOLTAGE_METER_ID_CELL_2: voltageMeterId_e = 81;
pub const VOLTAGE_METER_ID_CELL_1: voltageMeterId_e = 80;
pub const VOLTAGE_METER_ID_ESC_MOTOR_20: voltageMeterId_e = 79;
pub const VOLTAGE_METER_ID_ESC_MOTOR_12: voltageMeterId_e = 71;
pub const VOLTAGE_METER_ID_ESC_MOTOR_11: voltageMeterId_e = 70;
pub const VOLTAGE_METER_ID_ESC_MOTOR_10: voltageMeterId_e = 69;
pub const VOLTAGE_METER_ID_ESC_MOTOR_9: voltageMeterId_e = 68;
pub const VOLTAGE_METER_ID_ESC_MOTOR_8: voltageMeterId_e = 67;
pub const VOLTAGE_METER_ID_ESC_MOTOR_7: voltageMeterId_e = 66;
pub const VOLTAGE_METER_ID_ESC_MOTOR_6: voltageMeterId_e = 65;
pub const VOLTAGE_METER_ID_ESC_MOTOR_5: voltageMeterId_e = 64;
pub const VOLTAGE_METER_ID_ESC_MOTOR_4: voltageMeterId_e = 63;
pub const VOLTAGE_METER_ID_ESC_MOTOR_3: voltageMeterId_e = 62;
pub const VOLTAGE_METER_ID_ESC_MOTOR_2: voltageMeterId_e = 61;
pub const VOLTAGE_METER_ID_ESC_MOTOR_1: voltageMeterId_e = 60;
pub const VOLTAGE_METER_ID_ESC_COMBINED_10: voltageMeterId_e = 59;
pub const VOLTAGE_METER_ID_ESC_COMBINED_1: voltageMeterId_e = 50;
pub const VOLTAGE_METER_ID_12V_10: voltageMeterId_e = 49;
pub const VOLTAGE_METER_ID_12V_2: voltageMeterId_e = 41;
pub const VOLTAGE_METER_ID_12V_1: voltageMeterId_e = 40;
pub const VOLTAGE_METER_ID_9V_10: voltageMeterId_e = 39;
pub const VOLTAGE_METER_ID_9V_2: voltageMeterId_e = 31;
pub const VOLTAGE_METER_ID_9V_1: voltageMeterId_e = 30;
pub const VOLTAGE_METER_ID_5V_10: voltageMeterId_e = 29;
pub const VOLTAGE_METER_ID_5V_2: voltageMeterId_e = 21;
pub const VOLTAGE_METER_ID_5V_1: voltageMeterId_e = 20;
pub const VOLTAGE_METER_ID_BATTERY_10: voltageMeterId_e = 19;
pub const VOLTAGE_METER_ID_BATTERY_2: voltageMeterId_e = 11;
pub const VOLTAGE_METER_ID_BATTERY_1: voltageMeterId_e = 10;
pub const VOLTAGE_METER_ID_NONE: voltageMeterId_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageMeter_s {
    pub filtered: uint16_t,
    pub unfiltered: uint16_t,
    pub lowVoltageCutoff: bool,
}
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// WARNING - do not mix usage of VOLTAGE_METER_* and VOLTAGE_SENSOR_*, they are separate concerns.
pub type voltageMeter_t = voltageMeter_s;
pub type voltageSensorADC_e = libc::c_uint;
pub const VOLTAGE_SENSOR_ADC_5V: voltageSensorADC_e = 3;
pub const VOLTAGE_SENSOR_ADC_9V: voltageSensorADC_e = 2;
pub const VOLTAGE_SENSOR_ADC_12V: voltageSensorADC_e = 1;
pub const VOLTAGE_SENSOR_ADC_VBAT: voltageSensorADC_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageSensorADCConfig_s {
    pub vbatscale: uint8_t,
    pub vbatresdivval: uint8_t,
    pub vbatresdivmultiplier: uint8_t,
}
// voltage in 0.1V steps
// voltage in 0.1V steps
// see also voltageMeterADCtoIDMap
pub type voltageSensorADCConfig_t = voltageSensorADCConfig_s;
// adjust this to match battery voltage to reported value
// resistor divider R2 (default NAZE 10(K))
// multiplier for scale (e.g. 2.5:1 ratio with multiplier of 4 can use '100' instead of '25' in ratio) to get better precision
//
// ADC
//
pub type voltageMeterADCState_t = voltageMeterADCState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageMeterADCState_s {
    pub voltageFiltered: uint16_t,
    pub voltageUnfiltered: uint16_t,
    pub filter: biquadFilter_t,
}
// battery voltage in 0.1V steps (filtered)
// battery voltage in 0.1V steps (unfiltered)
//
// ESC
//
pub type voltageMeterESCState_t = voltageMeterESCState_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct voltageMeterESCState_s {
    pub voltageFiltered: uint16_t,
    pub voltageUnfiltered: uint16_t,
    pub filter: biquadFilter_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct escSensorData_t {
    pub dataAge: uint8_t,
    pub temperature: int8_t,
    pub voltage: int16_t,
    pub current: int32_t,
    pub consumption: int32_t,
    pub rpm: int16_t,
}
#[inline]
unsafe extern "C" fn voltageSensorADCConfig(mut _index: libc::c_int)
 -> *const voltageSensorADCConfig_t {
    return &mut *voltageSensorADCConfig_SystemArray.as_mut_ptr().offset(_index
                                                                            as
                                                                            isize)
               as *mut voltageSensorADCConfig_t;
}
// battery voltage in 0.1V steps (filtered)
// battery voltage in 0.1V steps (unfiltered)
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
pub static mut voltageMeterSourceNames: [*const libc::c_char; 3] =
    [b"NONE\x00" as *const u8 as *const libc::c_char,
     b"ADC\x00" as *const u8 as *const libc::c_char,
     b"ESC\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub static mut voltageMeterIds: [uint8_t; 14] =
    [VOLTAGE_METER_ID_BATTERY_1 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_COMBINED_1 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_1 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_2 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_3 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_4 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_5 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_6 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_7 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_8 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_9 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_10 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_11 as libc::c_int as uint8_t,
     VOLTAGE_METER_ID_ESC_MOTOR_12 as libc::c_int as uint8_t];
// Initialized in run_static_initializers
#[no_mangle]
pub static mut supportedVoltageMeterCount: uint8_t = 0;
//
// Main API
//
//
// ADC/ESC shared
//
#[no_mangle]
pub unsafe extern "C" fn voltageMeterReset(mut meter: *mut voltageMeter_t) {
    (*meter).filtered = 0 as libc::c_int as uint16_t;
    (*meter).unfiltered = 0 as libc::c_int as uint16_t;
}
#[no_mangle]
pub static mut voltageMeterADCStates: [voltageMeterADCState_t; 1] =
    [voltageMeterADCState_t{voltageFiltered: 0,
                            voltageUnfiltered: 0,
                            filter:
                                biquadFilter_t{b0: 0.,
                                               b1: 0.,
                                               b2: 0.,
                                               a1: 0.,
                                               a2: 0.,
                                               x1: 0.,
                                               x2: 0.,
                                               y1: 0.,
                                               y2: 0.,},}; 1];
#[no_mangle]
pub unsafe extern "C" fn getVoltageMeterADC(mut index: uint8_t)
 -> *mut voltageMeterADCState_t {
    return &mut *voltageMeterADCStates.as_mut_ptr().offset(index as isize) as
               *mut voltageMeterADCState_t;
}
#[no_mangle]
pub static mut voltageSensorADCConfig_SystemArray:
           [voltageSensorADCConfig_t; 1] =
    [voltageSensorADCConfig_t{vbatscale: 0,
                              vbatresdivval: 0,
                              vbatresdivmultiplier: 0,}; 1];
#[no_mangle]
pub static mut voltageSensorADCConfig_CopyArray: [voltageSensorADCConfig_t; 1]
           =
    [voltageSensorADCConfig_t{vbatscale: 0,
                              vbatresdivval: 0,
                              vbatresdivmultiplier: 0,}; 1];
// Initialized in run_static_initializers
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut voltageSensorADCConfig_Registry: pgRegistry_t =
    pgRegistry_t{pgn: 0,
                 size: 0,
                 address: 0 as *const uint8_t as *mut uint8_t,
                 copy: 0 as *const uint8_t as *mut uint8_t,
                 ptr: 0 as *const *mut uint8_t as *mut *mut uint8_t,
                 reset:
                     C2RustUnnamed_0{ptr:
                                         0 as *const libc::c_void as
                                             *mut libc::c_void,},};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_voltageSensorADCConfig(mut instance:
                                                              *mut voltageSensorADCConfig_t) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 1 as libc::c_int {
        static mut _reset_template_127: voltageSensorADCConfig_t =
            {
                let mut init =
                    voltageSensorADCConfig_s{vbatscale:
                                                 119 as libc::c_int as
                                                     uint8_t,
                                             vbatresdivval:
                                                 10 as libc::c_int as uint8_t,
                                             vbatresdivmultiplier:
                                                 1 as libc::c_int as
                                                     uint8_t,};
                init
            };
        memcpy(&mut *instance.offset(i as isize) as
                   *mut voltageSensorADCConfig_t as *mut libc::c_void,
               &_reset_template_127 as *const voltageSensorADCConfig_t as
                   *const libc::c_void,
               ::core::mem::size_of::<voltageSensorADCConfig_t>() as
                   libc::c_ulong);
        i += 1
    };
}
static mut voltageMeterAdcChannelMap: [uint8_t; 1] =
    [ADC_BATTERY as libc::c_int as uint8_t];
unsafe extern "C" fn voltageAdcToVoltage(src: uint16_t,
                                         mut config:
                                             *const voltageSensorADCConfig_t)
 -> uint16_t {
    // calculate battery voltage based on ADC reading
    // result is Vbatt in 0.1V steps. 3.3V = ADC Vref, 0xFFF = 12bit adc, 110 = 10:1 voltage divider (10k:1k) * 10 for 0.1V
    return (src as
                uint32_t).wrapping_mul((*config).vbatscale as
                                           libc::c_uint).wrapping_mul(getVrefMv()
                                                                          as
                                                                          libc::c_uint).wrapping_div(100
                                                                                                         as
                                                                                                         libc::c_int
                                                                                                         as
                                                                                                         libc::c_uint).wrapping_add((0xfff
                                                                                                                                         as
                                                                                                                                         libc::c_int
                                                                                                                                         *
                                                                                                                                         5
                                                                                                                                             as
                                                                                                                                             libc::c_int)
                                                                                                                                        as
                                                                                                                                        libc::c_uint).wrapping_div((0xfff
                                                                                                                                                                        as
                                                                                                                                                                        libc::c_int
                                                                                                                                                                        *
                                                                                                                                                                        (*config).vbatresdivval
                                                                                                                                                                            as
                                                                                                                                                                            libc::c_int)
                                                                                                                                                                       as
                                                                                                                                                                       libc::c_uint).wrapping_div((*config).vbatresdivmultiplier
                                                                                                                                                                                                      as
                                                                                                                                                                                                      libc::c_uint)
               as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn voltageMeterADCRefresh() {
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 1 as libc::c_int &&
              (i as libc::c_ulong) <
                  (::core::mem::size_of::<[uint8_t; 1]>() as
                       libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                       as libc::c_ulong) {
        let mut state: *mut voltageMeterADCState_t =
            &mut *voltageMeterADCStates.as_mut_ptr().offset(i as isize) as
                *mut voltageMeterADCState_t;
        // store the battery voltage with some other recent battery voltage readings
        let mut config: *const voltageSensorADCConfig_t =
            voltageSensorADCConfig(i as libc::c_int);
        let mut channel: uint8_t = voltageMeterAdcChannelMap[i as usize];
        let mut rawSample: uint16_t = adcGetChannel(channel);
        let mut filteredSample: uint16_t =
            biquadFilterApply(&mut (*state).filter,
                              rawSample as libc::c_float) as uint16_t;
        // always calculate the latest voltage, see getLatestVoltage() which does the calculation on demand.
        (*state).voltageFiltered =
            voltageAdcToVoltage(filteredSample, config);
        (*state).voltageUnfiltered = voltageAdcToVoltage(rawSample, config);
        i = i.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn voltageMeterADCRead(mut adcChannel:
                                                 voltageSensorADC_e,
                                             mut voltageMeter:
                                                 *mut voltageMeter_t) {
    let mut state: *mut voltageMeterADCState_t =
        &mut *voltageMeterADCStates.as_mut_ptr().offset(adcChannel as isize)
            as *mut voltageMeterADCState_t;
    (*voltageMeter).filtered = (*state).voltageFiltered;
    (*voltageMeter).unfiltered = (*state).voltageUnfiltered;
}
#[no_mangle]
pub unsafe extern "C" fn voltageMeterADCInit() {
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 1 as libc::c_int &&
              (i as libc::c_ulong) <
                  (::core::mem::size_of::<[uint8_t; 1]>() as
                       libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>()
                                                       as libc::c_ulong) {
        // store the battery voltage with some other recent battery voltage readings
        let mut state: *mut voltageMeterADCState_t =
            &mut *voltageMeterADCStates.as_mut_ptr().offset(i as isize) as
                *mut voltageMeterADCState_t;
        memset(state as *mut libc::c_void, 0 as libc::c_int,
               ::core::mem::size_of::<voltageMeterADCState_t>() as
                   libc::c_ulong);
        biquadFilterInitLPF(&mut (*state).filter, 0.1f32,
                            (1000000 as libc::c_int / 50 as libc::c_int) as
                                uint32_t);
        i = i.wrapping_add(1)
    };
}
static mut voltageMeterESCState: voltageMeterESCState_t =
    voltageMeterESCState_t{voltageFiltered: 0,
                           voltageUnfiltered: 0,
                           filter:
                               biquadFilter_t{b0: 0.,
                                              b1: 0.,
                                              b2: 0.,
                                              a1: 0.,
                                              a2: 0.,
                                              x1: 0.,
                                              x2: 0.,
                                              y1: 0.,
                                              y2: 0.,},};
#[no_mangle]
pub unsafe extern "C" fn voltageMeterESCInit() {
    memset(&mut voltageMeterESCState as *mut voltageMeterESCState_t as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<voltageMeterESCState_t>() as libc::c_ulong);
    biquadFilterInitLPF(&mut voltageMeterESCState.filter, 0.1f32,
                        (1000000 as libc::c_int / 50 as libc::c_int) as
                            uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn voltageMeterESCRefresh() {
    let mut escData: *mut escSensorData_t =
        getEscSensorData(255 as libc::c_int as uint8_t);
    if !escData.is_null() {
        voltageMeterESCState.voltageUnfiltered =
            if (*escData).dataAge as libc::c_int <= 10 as libc::c_int {
                ((*escData).voltage as libc::c_int) / 10 as libc::c_int
            } else { 0 as libc::c_int } as uint16_t;
        voltageMeterESCState.voltageFiltered =
            biquadFilterApply(&mut voltageMeterESCState.filter,
                              voltageMeterESCState.voltageUnfiltered as
                                  libc::c_float) as uint16_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn voltageMeterESCReadMotor(mut motorNumber: uint8_t,
                                                  mut voltageMeter:
                                                      *mut voltageMeter_t) {
    let mut escData: *mut escSensorData_t = getEscSensorData(motorNumber);
    if !escData.is_null() {
        (*voltageMeter).unfiltered =
            if (*escData).dataAge as libc::c_int <= 10 as libc::c_int {
                ((*escData).voltage as libc::c_int) / 10 as libc::c_int
            } else { 0 as libc::c_int } as uint16_t;
        (*voltageMeter).filtered = (*voltageMeter).unfiltered
        // no filtering for ESC motors currently.
    } else { voltageMeterReset(voltageMeter); };
}
#[no_mangle]
pub unsafe extern "C" fn voltageMeterESCReadCombined(mut voltageMeter:
                                                         *mut voltageMeter_t) {
    (*voltageMeter).filtered = voltageMeterESCState.voltageFiltered;
    (*voltageMeter).unfiltered = voltageMeterESCState.voltageUnfiltered;
}
//
// API for using voltage meters using IDs
//
// This API is used by MSP, for configuration/status.
//
// the order of these much match the indexes in voltageSensorADC_e
#[no_mangle]
pub static mut voltageMeterADCtoIDMap: [uint8_t; 1] =
    [VOLTAGE_METER_ID_BATTERY_1 as libc::c_int as uint8_t];
//
// API for reading/configuring current meters by id.
//
#[no_mangle]
pub unsafe extern "C" fn voltageMeterRead(mut id: voltageMeterId_e,
                                          mut meter: *mut voltageMeter_t) {
    if id as libc::c_uint ==
           VOLTAGE_METER_ID_BATTERY_1 as libc::c_int as libc::c_uint {
        voltageMeterADCRead(VOLTAGE_SENSOR_ADC_VBAT, meter);
    } else if id as libc::c_uint ==
                  VOLTAGE_METER_ID_ESC_COMBINED_1 as libc::c_int as
                      libc::c_uint {
        voltageMeterESCReadCombined(meter);
    } else if id as libc::c_uint >=
                  VOLTAGE_METER_ID_ESC_MOTOR_1 as libc::c_int as libc::c_uint
                  &&
                  id as libc::c_uint <=
                      VOLTAGE_METER_ID_ESC_MOTOR_20 as libc::c_int as
                          libc::c_uint {
        let mut motor: libc::c_int =
            (id as
                 libc::c_uint).wrapping_sub(VOLTAGE_METER_ID_ESC_MOTOR_1 as
                                                libc::c_int as libc::c_uint)
                as libc::c_int;
        voltageMeterESCReadMotor(motor as uint8_t, meter);
    } else { voltageMeterReset(meter); };
}
unsafe extern "C" fn run_static_initializers() {
    supportedVoltageMeterCount =
        (::core::mem::size_of::<[uint8_t; 14]>() as
             libc::c_ulong).wrapping_div(::core::mem::size_of::<uint8_t>() as
                                             libc::c_ulong) as uint8_t;
    voltageSensorADCConfig_Registry =
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (258 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 ((::core::mem::size_of::<voltageSensorADCConfig_t>()
                                       as
                                       libc::c_ulong).wrapping_mul(1 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_ulong)
                                      |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &mut voltageSensorADCConfig_SystemArray as
                                     *mut [voltageSensorADCConfig_t; 1] as
                                     *mut uint8_t,
                             copy:
                                 &mut voltageSensorADCConfig_CopyArray as
                                     *mut [voltageSensorADCConfig_t; 1] as
                                     *mut uint8_t,
                             ptr: 0 as *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut voltageSensorADCConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_voltageSensorADCConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut voltageSensorADCConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
