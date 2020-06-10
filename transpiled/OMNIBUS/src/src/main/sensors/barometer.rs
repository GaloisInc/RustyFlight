use ::libc;
extern "C" {
    #[no_mangle]
    fn lrintf(_: libc::c_float) -> libc::c_long;
    #[no_mangle]
    fn quickMedianFilter3(v: *mut int32_t) -> int32_t;
    #[no_mangle]
    fn pow_approx(a: libc::c_float, b: libc::c_float) -> libc::c_float;
    #[no_mangle]
    fn spiDeviceByInstance(instance: *mut SPI_TypeDef) -> SPIDevice;
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
    #[no_mangle]
    fn spiBusSetInstance(bus: *mut busDevice_t, instance: *mut SPI_TypeDef);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
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
    /* Chip ID Register */
    /* Softreset Register */
    /* Status Register */
    /* Ctrl Measure Register */
    /* Configuration Register */
    /* Pressure MSB Register */
    /* Pressure LSB Register */
    /* Pressure XLSB Register */
    /* Temperature MSB Reg */
    /* Temperature LSB Reg */
    /* Temperature XLSB Reg */
    // configure pressure and temperature oversampling, forced sampling mode
    // 20/16 = 1.25 ms
    // 37/16 = 2.3125 ms
    // 10/16 = 0.625 ms
    #[no_mangle]
    fn bmp280Detect(baro_0: *mut baroDev_t) -> bool;
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    static mut detectedSensors: [uint8_t; 5];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
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
pub type ioTag_t = uint8_t;
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
    pub busdev_u: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
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
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baroDev_s {
    pub busdev: busDevice_t,
    pub ut_delay: uint16_t,
    pub up_delay: uint16_t,
    pub start_ut: baroOpFuncPtr,
    pub get_ut: baroOpFuncPtr,
    pub start_up: baroOpFuncPtr,
    pub get_up: baroOpFuncPtr,
    pub calculate: baroCalculateFuncPtr,
}
// baro start operation
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
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
// XXX
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
// baro calculation (filled params are pressure and temperature)
pub type baroDev_t = baroDev_s;
pub type baroSensor_e = libc::c_uint;
pub const BARO_QMP6988: baroSensor_e = 6;
pub const BARO_LPS: baroSensor_e = 5;
pub const BARO_BMP280: baroSensor_e = 4;
pub const BARO_MS5611: baroSensor_e = 3;
pub const BARO_BMP085: baroSensor_e = 2;
pub const BARO_NONE: baroSensor_e = 1;
pub const BARO_DEFAULT: baroSensor_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct barometerConfig_s {
    pub baro_bustype: uint8_t,
    pub baro_spi_device: uint8_t,
    pub baro_spi_csn: ioTag_t,
    pub baro_i2c_device: uint8_t,
    pub baro_i2c_address: uint8_t,
    pub baro_hardware: uint8_t,
    pub baro_sample_count: uint8_t,
    pub baro_noise_lpf: uint16_t,
    pub baro_cf_vel: uint16_t,
    pub baro_cf_alt: uint16_t,
}
pub type barometerConfig_t = barometerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct baro_s {
    pub dev: baroDev_t,
    pub BaroAlt: int32_t,
    pub baroTemperature: int32_t,
    pub baroPressure: int32_t,
}
pub type baro_t = baro_s;
pub const SENSOR_BARO: C2RustUnnamed_3 = 4;
pub const SENSOR_INDEX_BARO: C2RustUnnamed_2 = 2;
pub type barometerState_e = libc::c_uint;
pub const BAROMETER_NEEDS_CALCULATION: barometerState_e = 1;
pub const BAROMETER_NEEDS_SAMPLES: barometerState_e = 0;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const SENSOR_INDEX_COUNT: C2RustUnnamed_2 = 5;
pub const SENSOR_INDEX_RANGEFINDER: C2RustUnnamed_2 = 4;
pub const SENSOR_INDEX_MAG: C2RustUnnamed_2 = 3;
pub const SENSOR_INDEX_ACC: C2RustUnnamed_2 = 1;
pub const SENSOR_INDEX_GYRO: C2RustUnnamed_2 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_3 = 64;
pub const SENSOR_GPS: C2RustUnnamed_3 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_3 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_3 = 16;
pub const SENSOR_MAG: C2RustUnnamed_3 = 8;
pub const SENSOR_ACC: C2RustUnnamed_3 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_3 = 1;
#[inline]
unsafe extern "C" fn barometerConfig() -> *const barometerConfig_t {
    return &mut barometerConfig_System;
}
// Also used as XCLR (positive logic) for BMP085
// Barometer hardware to use
// size of baro filter array
// additional LPF to reduce baro noise
// apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity)
// apply CF to use ACC for height estimation
// Use temperature for telemetry
// Use pressure for telemetry
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
pub static mut baro: baro_t =
    baro_t{dev:
               baroDev_t{busdev:
                             busDevice_t{bustype: BUSTYPE_NONE,
                                         busdev_u:
                                             C2RustUnnamed_1{spi:
                                                                 deviceSpi_s{instance:
                                                                                 0
                                                                                     as
                                                                                     *const SPI_TypeDef
                                                                                     as
                                                                                     *mut SPI_TypeDef,
                                                                             csnPin:
                                                                                 0
                                                                                     as
                                                                                     *const libc::c_void
                                                                                     as
                                                                                     *mut libc::c_void,},},},
                         ut_delay: 0,
                         up_delay: 0,
                         start_ut: None,
                         get_ut: None,
                         start_up: None,
                         get_up: None,
                         calculate: None,},
           BaroAlt: 0,
           baroTemperature: 0,
           baroPressure: 0,};
// barometer access functions
#[no_mangle]
pub static mut barometerConfig_Copy: barometerConfig_t =
    barometerConfig_t{baro_bustype: 0,
                      baro_spi_device: 0,
                      baro_spi_csn: 0,
                      baro_i2c_device: 0,
                      baro_i2c_address: 0,
                      baro_hardware: 0,
                      baro_sample_count: 0,
                      baro_noise_lpf: 0,
                      baro_cf_vel: 0,
                      baro_cf_alt: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut barometerConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (38 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<barometerConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &barometerConfig_System as
                                     *const barometerConfig_t as
                                     *mut barometerConfig_t as *mut uint8_t,
                             copy:
                                 &barometerConfig_Copy as
                                     *const barometerConfig_t as
                                     *mut barometerConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut barometerConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_barometerConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut barometerConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub static mut barometerConfig_System: barometerConfig_t =
    barometerConfig_t{baro_bustype: 0,
                      baro_spi_device: 0,
                      baro_spi_csn: 0,
                      baro_i2c_device: 0,
                      baro_i2c_address: 0,
                      baro_hardware: 0,
                      baro_sample_count: 0,
                      baro_noise_lpf: 0,
                      baro_cf_vel: 0,
                      baro_cf_alt: 0,};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_barometerConfig(mut barometerConfig_0:
                                                       *mut barometerConfig_t) {
    (*barometerConfig_0).baro_sample_count = 21 as libc::c_int as uint8_t;
    (*barometerConfig_0).baro_noise_lpf = 600 as libc::c_int as uint16_t;
    (*barometerConfig_0).baro_cf_vel = 985 as libc::c_int as uint16_t;
    (*barometerConfig_0).baro_cf_alt = 965 as libc::c_int as uint16_t;
    (*barometerConfig_0).baro_hardware =
        BARO_DEFAULT as libc::c_int as uint8_t;
    // For backward compatibility; ceate a valid default value for bus parameters
    //
    // 1. If DEFAULT_BARO_xxx is defined, use it.
    // 2. Determine default based on USE_BARO_xxx
    //   a. Precedence is in the order of popularity; BMP280, MS5611 then BMP085, then
    //   b. If SPI variant is specified, it is likely onboard, so take it.
    (*barometerConfig_0).baro_bustype = BUSTYPE_SPI as libc::c_int as uint8_t;
    (*barometerConfig_0).baro_spi_device =
        (spiDeviceByInstance((0x40000000 as libc::c_int as
                                  uint32_t).wrapping_add(0x10000 as
                                                             libc::c_int as
                                                             libc::c_uint).wrapping_add(0x3000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint)
                                 as *mut SPI_TypeDef) as libc::c_int +
             1 as libc::c_int) as uint8_t;
    (*barometerConfig_0).baro_spi_csn =
        ((0 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             13 as libc::c_int) as ioTag_t;
    (*barometerConfig_0).baro_i2c_device =
        (I2CINVALID as libc::c_int + 1 as libc::c_int) as uint8_t;
    (*barometerConfig_0).baro_i2c_address = 0 as libc::c_int as uint8_t;
}
static mut calibratingB: uint16_t = 0 as libc::c_int as uint16_t;
// baro calibration = get new ground pressure value
static mut baroPressure: int32_t = 0 as libc::c_int;
static mut baroTemperature: int32_t = 0 as libc::c_int;
static mut baroGroundAltitude: int32_t = 0 as libc::c_int;
static mut baroGroundPressure: int32_t =
    8 as libc::c_int * 101325 as libc::c_int;
static mut baroPressureSum: uint32_t = 0 as libc::c_int as uint32_t;
#[no_mangle]
pub unsafe extern "C" fn baroDetect(mut dev: *mut baroDev_t,
                                    mut baroHardwareToUse: baroSensor_e)
 -> bool {
    // Detect what pressure sensors are available. baro->update() is set to sensor-specific update function
    let mut baroHardware: baroSensor_e = baroHardwareToUse;
    match (*barometerConfig()).baro_bustype as libc::c_int {
        1 => {
            (*dev).busdev.bustype = BUSTYPE_I2C;
            (*dev).busdev.busdev_u.i2c.device =
                ((*barometerConfig()).baro_i2c_device as libc::c_int -
                     1 as libc::c_int) as I2CDevice;
            (*dev).busdev.busdev_u.i2c.address =
                (*barometerConfig()).baro_i2c_address
        }
        2 => {
            (*dev).busdev.bustype = BUSTYPE_SPI;
            spiBusSetInstance(&mut (*dev).busdev,
                              spiInstanceByDevice(((*barometerConfig()).baro_spi_device
                                                       as libc::c_int -
                                                       1 as libc::c_int) as
                                                      SPIDevice));
            (*dev).busdev.busdev_u.spi.csnPin =
                IOGetByTag((*barometerConfig()).baro_spi_csn)
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    let mut current_block_21: u64;
    match baroHardware as libc::c_uint {
        0 | 2 | 3 | 5 | 4 => {
            if bmp280Detect(dev) {
                baroHardware = BARO_BMP280;
                current_block_21 = 11932355480408055363;
            } else { current_block_21 = 17760584797385305199; }
        }
        6 | 1 => { current_block_21 = 17760584797385305199; }
        _ => { current_block_21 = 11932355480408055363; }
    }
    match current_block_21 {
        17760584797385305199 => { baroHardware = BARO_NONE }
        _ => { }
    }
    if baroHardware as libc::c_uint ==
           BARO_NONE as libc::c_int as libc::c_uint {
        return 0 as libc::c_int != 0
    }
    detectedSensors[SENSOR_INDEX_BARO as libc::c_int as usize] =
        baroHardware as uint8_t;
    sensorsSet(SENSOR_BARO as libc::c_int as uint32_t);
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn isBaroCalibrationComplete() -> bool {
    return calibratingB as libc::c_int == 0 as libc::c_int;
}
#[no_mangle]
pub unsafe extern "C" fn baroSetCalibrationCycles(mut calibrationCyclesRequired:
                                                      uint16_t) {
    calibratingB = calibrationCyclesRequired;
}
static mut baroReady: bool = 0 as libc::c_int != 0;
unsafe extern "C" fn applyBarometerMedianFilter(mut newPressureReading:
                                                    int32_t) -> int32_t {
    static mut barometerFilterSamples: [int32_t; 3] = [0; 3];
    static mut currentFilterSampleIndex: libc::c_int = 0 as libc::c_int;
    static mut medianFilterReady: bool = 0 as libc::c_int != 0;
    let mut nextSampleIndex: libc::c_int = 0;
    nextSampleIndex = currentFilterSampleIndex + 1 as libc::c_int;
    if nextSampleIndex == 3 as libc::c_int {
        nextSampleIndex = 0 as libc::c_int;
        medianFilterReady = 1 as libc::c_int != 0
    }
    barometerFilterSamples[currentFilterSampleIndex as usize] =
        newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;
    if medianFilterReady {
        return quickMedianFilter3(barometerFilterSamples.as_mut_ptr())
    } else { return newPressureReading };
}
unsafe extern "C" fn recalculateBarometerTotal(mut baroSampleCount: uint8_t,
                                               mut pressureTotal: uint32_t,
                                               mut newPressureReading:
                                                   int32_t) -> uint32_t {
    static mut barometerSamples: [int32_t; 48] = [0; 48];
    static mut currentSampleIndex: libc::c_int = 0 as libc::c_int;
    let mut nextSampleIndex: libc::c_int = 0;
    // store current pressure in barometerSamples
    nextSampleIndex = currentSampleIndex + 1 as libc::c_int;
    if nextSampleIndex == baroSampleCount as libc::c_int {
        nextSampleIndex = 0 as libc::c_int;
        baroReady = 1 as libc::c_int != 0
    }
    barometerSamples[currentSampleIndex as usize] =
        applyBarometerMedianFilter(newPressureReading);
    // recalculate pressure total
    // Note, the pressure total is made up of baroSampleCount - 1 samples - See PRESSURE_SAMPLE_COUNT
    pressureTotal =
        (pressureTotal as
             libc::c_uint).wrapping_add(barometerSamples[currentSampleIndex as
                                                             usize] as
                                            libc::c_uint) as uint32_t as
            uint32_t;
    pressureTotal =
        (pressureTotal as
             libc::c_uint).wrapping_sub(barometerSamples[nextSampleIndex as
                                                             usize] as
                                            libc::c_uint) as uint32_t as
            uint32_t;
    currentSampleIndex = nextSampleIndex;
    return pressureTotal;
}
#[no_mangle]
pub unsafe extern "C" fn isBaroReady() -> bool { return baroReady; }
#[no_mangle]
pub unsafe extern "C" fn baroUpdate() -> uint32_t {
    static mut state: barometerState_e = BAROMETER_NEEDS_SAMPLES;
    match state as libc::c_uint {
        1 => {
            baro.dev.get_up.expect("non-null function pointer")(&mut baro.dev);
            baro.dev.start_ut.expect("non-null function pointer")(&mut baro.dev);
            baro.dev.calculate.expect("non-null function pointer")(&mut baroPressure,
                                                                   &mut baroTemperature);
            baro.baroPressure = baroPressure;
            baro.baroTemperature = baroTemperature;
            baroPressureSum =
                recalculateBarometerTotal((*barometerConfig()).baro_sample_count,
                                          baroPressureSum, baroPressure);
            state = BAROMETER_NEEDS_SAMPLES;
            return baro.dev.ut_delay as uint32_t
        }
        0 | _ => {
            baro.dev.get_ut.expect("non-null function pointer")(&mut baro.dev);
            baro.dev.start_up.expect("non-null function pointer")(&mut baro.dev);
            state = BAROMETER_NEEDS_CALCULATION;
            return baro.dev.up_delay as uint32_t
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn baroCalculateAltitude() -> int32_t {
    let mut BaroAlt_tmp: int32_t = 0;
    // calculates height from ground via baro readings
    // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
    if isBaroCalibrationComplete() {
        BaroAlt_tmp =
            lrintf((1.0f32 -
                        pow_approx(baroPressureSum.wrapping_div(((*barometerConfig()).baro_sample_count
                                                                     as
                                                                     libc::c_int
                                                                     -
                                                                     1 as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                       as libc::c_float / 101325.0f32,
                                   0.190295f32)) * 4433000.0f32) as int32_t;
        BaroAlt_tmp -= baroGroundAltitude;
        baro.BaroAlt =
            lrintf(baro.BaroAlt as libc::c_float *
                       (0.001f32 *
                            (*barometerConfig()).baro_noise_lpf as libc::c_int
                                as libc::c_float) +
                       BaroAlt_tmp as libc::c_float *
                           (1.0f32 -
                                0.001f32 *
                                    (*barometerConfig()).baro_noise_lpf as
                                        libc::c_int as libc::c_float)) as
                int32_t // in cm
        // additional LPF to reduce baro noise
    } else { baro.BaroAlt = 0 as libc::c_int }
    return baro.BaroAlt;
}
#[no_mangle]
pub unsafe extern "C" fn performBaroCalibrationCycle() {
    static mut savedGroundPressure: int32_t = 0 as libc::c_int;
    baroGroundPressure -= baroGroundPressure / 8 as libc::c_int;
    baroGroundPressure =
        (baroGroundPressure as
             libc::c_uint).wrapping_add(baroPressureSum.wrapping_div(((*barometerConfig()).baro_sample_count
                                                                          as
                                                                          libc::c_int
                                                                          -
                                                                          1 as
                                                                              libc::c_int)
                                                                         as
                                                                         libc::c_uint))
            as int32_t as int32_t;
    baroGroundAltitude =
        ((1.0f32 -
              pow_approx((baroGroundPressure / 8 as libc::c_int) as
                             libc::c_float / 101325.0f32, 0.190295f32)) *
             4433000.0f32) as int32_t;
    if baroGroundPressure == savedGroundPressure {
        calibratingB = 0 as libc::c_int as uint16_t
    } else {
        calibratingB = calibratingB.wrapping_sub(1);
        savedGroundPressure = baroGroundPressure
    };
}
/* BARO */
