use ::libc;
extern "C" {
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
    #[no_mangle]
    fn spiBusSetInstance(bus: *mut busDevice_t, instance: *mut SPI_TypeDef);
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
    fn ak8963Detect(mag_0: *mut magDev_t) -> bool;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
    #[no_mangle]
    fn ledSet(led: libc::c_int, state: bool);
    #[no_mangle]
    fn saveConfigAndNotify();
    #[no_mangle]
    static mut stateFlags: uint8_t;
    #[no_mangle]
    fn sensorsSet(mask: uint32_t);
    #[no_mangle]
    fn alignSensors(dest: *mut libc::c_float, rotation: uint8_t);
    #[no_mangle]
    static mut detectedSensors: [uint8_t; 5];
    #[no_mangle]
    fn gyroSensorBus() -> *const busDevice_t;
    #[no_mangle]
    fn gyroMpuDetectionResult() -> *const mpuDetectionResult_s;
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int16_t = __int16_t;
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
pub const Z: C2RustUnnamed = 2;
pub const Y: C2RustUnnamed = 1;
pub const X: C2RustUnnamed = 0;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed_0 = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed_0 = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed_0 = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed_0 = 4095;
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
    pub reset: C2RustUnnamed_1,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_1 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
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
    pub busdev_u: C2RustUnnamed_2,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_2 {
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct extiCallbackRec_s {
    pub fn_0: Option<extiHandlerCallback>,
}
pub type extiHandlerCallback
    =
    unsafe extern "C" fn(_: *mut extiCallbackRec_t) -> ();
pub type extiCallbackRec_t = extiCallbackRec_s;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct magDev_s {
    pub init: sensorMagInitFuncPtr,
    pub read: sensorMagReadFuncPtr,
    pub exti: extiCallbackRec_t,
    pub busdev: busDevice_t,
    pub magAlign: sensor_align_e,
    pub magIntExtiTag: ioTag_t,
    pub magGain: [int16_t; 3],
}
pub type sensorMagReadFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut magDev_s, _: *mut int16_t) -> bool>;
pub type sensorMagInitFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut magDev_s) -> bool>;
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
pub type mpu6050Resolution_e = libc::c_uint;
pub const MPU_FULL_RESOLUTION: mpu6050Resolution_e = 1;
pub const MPU_HALF_RESOLUTION: mpu6050Resolution_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mpuDetectionResult_s {
    pub sensor: mpuSensor_e,
    pub resolution: mpu6050Resolution_e,
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
pub type magDev_t = magDev_s;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const FIXED_WING: C2RustUnnamed_3 = 16;
pub const SMALL_ANGLE: C2RustUnnamed_3 = 8;
pub const CALIBRATE_MAG: C2RustUnnamed_3 = 4;
pub const GPS_FIX: C2RustUnnamed_3 = 2;
pub const GPS_FIX_HOME: C2RustUnnamed_3 = 1;
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed_4 = libc::c_uint;
pub const SENSOR_INDEX_COUNT: C2RustUnnamed_4 = 5;
pub const SENSOR_INDEX_RANGEFINDER: C2RustUnnamed_4 = 4;
pub const SENSOR_INDEX_MAG: C2RustUnnamed_4 = 3;
pub const SENSOR_INDEX_BARO: C2RustUnnamed_4 = 2;
pub const SENSOR_INDEX_ACC: C2RustUnnamed_4 = 1;
pub const SENSOR_INDEX_GYRO: C2RustUnnamed_4 = 0;
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
pub type C2RustUnnamed_5 = libc::c_uint;
pub const SENSOR_GPSMAG: C2RustUnnamed_5 = 64;
pub const SENSOR_GPS: C2RustUnnamed_5 = 32;
pub const SENSOR_RANGEFINDER: C2RustUnnamed_5 = 16;
pub const SENSOR_SONAR: C2RustUnnamed_5 = 16;
pub const SENSOR_MAG: C2RustUnnamed_5 = 8;
pub const SENSOR_BARO: C2RustUnnamed_5 = 4;
pub const SENSOR_ACC: C2RustUnnamed_5 = 2;
pub const SENSOR_GYRO: C2RustUnnamed_5 = 1;
pub type magSensor_e = libc::c_uint;
pub const MAG_QMC5883: magSensor_e = 5;
pub const MAG_AK8963: magSensor_e = 4;
pub const MAG_AK8975: magSensor_e = 3;
pub const MAG_HMC5883: magSensor_e = 2;
pub const MAG_NONE: magSensor_e = 1;
pub const MAG_DEFAULT: magSensor_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct mag_s {
    pub magADC: [libc::c_float; 3],
    pub magneticDeclination: libc::c_float,
}
pub type mag_t = mag_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct compassConfig_s {
    pub mag_declination: int16_t,
    pub mag_align: sensor_align_e,
    pub mag_hardware: uint8_t,
    pub mag_bustype: uint8_t,
    pub mag_i2c_device: uint8_t,
    pub mag_i2c_address: uint8_t,
    pub mag_spi_device: uint8_t,
    pub mag_spi_csn: ioTag_t,
    pub interruptTag: ioTag_t,
    pub magZero: flightDynamicsTrims_t,
}
pub type compassConfig_t = compassConfig_s;
#[inline]
unsafe extern "C" fn compassConfigMutable() -> *mut compassConfig_t {
    return &mut compassConfig_System;
}
#[inline]
unsafe extern "C" fn compassConfig() -> *const compassConfig_t {
    return &mut compassConfig_System;
}
// initialize function
// read 3 axis data function
// Get your magnetic decliniation from here : http://magnetic-declination.com/
                                            // For example, -6deg 37min, = -637 Japan, format is [sign]dddmm (degreesminutes) default is zero.
// mag alignment
// Which mag hardware to use on boards with more than one device
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
pub static mut magDev: magDev_t =
    magDev_t{init: None,
             read: None,
             exti: extiCallbackRec_t{fn_0: None,},
             busdev:
                 busDevice_t{bustype: BUSTYPE_NONE,
                             busdev_u:
                                 C2RustUnnamed_2{spi:
                                                     deviceSpi_s{instance:
                                                                     0 as
                                                                         *const SPI_TypeDef
                                                                         as
                                                                         *mut SPI_TypeDef,
                                                                 csnPin:
                                                                     0 as
                                                                         *const libc::c_void
                                                                         as
                                                                         *mut libc::c_void,},},},
             magAlign: ALIGN_DEFAULT,
             magIntExtiTag: 0,
             magGain: [0; 3],};
#[no_mangle]
pub static mut mag: mag_t = mag_t{magADC: [0.; 3], magneticDeclination: 0.,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut compassConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (40 as libc::c_int |
                                      (1 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<compassConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &compassConfig_System as
                                     *const compassConfig_t as
                                     *mut compassConfig_t as *mut uint8_t,
                             copy:
                                 &compassConfig_Copy as *const compassConfig_t
                                     as *mut compassConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_1{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut compassConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_compassConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut compassConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub static mut compassConfig_Copy: compassConfig_t =
    compassConfig_t{mag_declination: 0,
                    mag_align: ALIGN_DEFAULT,
                    mag_hardware: 0,
                    mag_bustype: 0,
                    mag_i2c_device: 0,
                    mag_i2c_address: 0,
                    mag_spi_device: 0,
                    mag_spi_csn: 0,
                    interruptTag: 0,
                    magZero: flightDynamicsTrims_u{raw: [0; 3],},};
#[no_mangle]
pub static mut compassConfig_System: compassConfig_t =
    compassConfig_t{mag_declination: 0,
                    mag_align: ALIGN_DEFAULT,
                    mag_hardware: 0,
                    mag_bustype: 0,
                    mag_i2c_device: 0,
                    mag_i2c_address: 0,
                    mag_spi_device: 0,
                    mag_spi_csn: 0,
                    interruptTag: 0,
                    magZero: flightDynamicsTrims_u{raw: [0; 3],},};
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_compassConfig(mut compassConfig_0:
                                                     *mut compassConfig_t) {
    (*compassConfig_0).mag_align = ALIGN_DEFAULT;
    (*compassConfig_0).mag_declination = 0 as libc::c_int as int16_t;
    (*compassConfig_0).mag_hardware = MAG_DEFAULT as libc::c_int as uint8_t;
    // Generate a reasonable default for backward compatibility
// Strategy is
// 1. If SPI device is defined, it will take precedence, assuming it's onboard.
// 2. I2C devices are will be handled by address = 0 (per device default).
// 3. Slave I2C device on SPI gyro
    (*compassConfig_0).mag_bustype =
        BUSTYPE_MPU_SLAVE as libc::c_int as uint8_t;
    (*compassConfig_0).mag_i2c_device =
        (I2CINVALID as libc::c_int + 1 as libc::c_int) as uint8_t;
    (*compassConfig_0).mag_i2c_address = 0 as libc::c_int as uint8_t;
    (*compassConfig_0).mag_spi_device =
        (SPIINVALID as libc::c_int + 1 as libc::c_int) as uint8_t;
    (*compassConfig_0).mag_spi_csn = 0 as libc::c_int as ioTag_t;
    (*compassConfig_0).interruptTag = 0 as libc::c_int as ioTag_t;
}
static mut magADCRaw: [int16_t; 3] = [0; 3];
static mut magInit: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn compassDetect(mut dev: *mut magDev_t) -> bool {
    let mut magHardware: magSensor_e = MAG_NONE;
    let mut busdev: *mut busDevice_t = &mut (*dev).busdev;
    match (*compassConfig()).mag_bustype as libc::c_int {
        1 => {
            (*busdev).bustype = BUSTYPE_I2C;
            (*busdev).busdev_u.i2c.device =
                ((*compassConfig()).mag_i2c_device as libc::c_int -
                     1 as libc::c_int) as I2CDevice;
            (*busdev).busdev_u.i2c.address =
                (*compassConfig()).mag_i2c_address
        }
        2 => {
            (*busdev).bustype = BUSTYPE_SPI;
            spiBusSetInstance(busdev,
                              spiInstanceByDevice(((*compassConfig()).mag_spi_device
                                                       as libc::c_int -
                                                       1 as libc::c_int) as
                                                      SPIDevice));
            (*busdev).busdev_u.spi.csnPin =
                IOGetByTag((*compassConfig()).mag_spi_csn)
        }
        3 => {
            if (*gyroMpuDetectionResult()).sensor as libc::c_uint ==
                   MPU_9250_SPI as libc::c_int as libc::c_uint {
                (*busdev).bustype = BUSTYPE_MPU_SLAVE;
                (*busdev).busdev_u.mpuSlave.master = gyroSensorBus();
                (*busdev).busdev_u.mpuSlave.address =
                    (*compassConfig()).mag_i2c_address
            } else { return 0 as libc::c_int != 0 }
        }
        _ => { return 0 as libc::c_int != 0 }
    }
    (*dev).magAlign = ALIGN_DEFAULT;
    let mut current_block_37: u64;
    match (*compassConfig()).mag_hardware as libc::c_int {
        0 | 2 | 5 | 3 | 4 => {
            if (*busdev).bustype as libc::c_uint ==
                   BUSTYPE_I2C as libc::c_int as libc::c_uint {
                (*busdev).busdev_u.i2c.address =
                    (*compassConfig()).mag_i2c_address
            }
            if (*gyroMpuDetectionResult()).sensor as libc::c_uint ==
                   MPU_9250_SPI as libc::c_int as libc::c_uint {
                (*dev).busdev.bustype = BUSTYPE_MPU_SLAVE;
                (*busdev).busdev_u.mpuSlave.address =
                    (*compassConfig()).mag_i2c_address;
                (*dev).busdev.busdev_u.mpuSlave.master = gyroSensorBus()
            }
            if ak8963Detect(dev) {
                (*dev).magAlign = CW180_DEG_FLIP;
                magHardware = MAG_AK8963;
                current_block_37 = 1622411330066726685;
            } else { current_block_37 = 11814172370226575913; }
        }
        1 => { current_block_37 = 11814172370226575913; }
        _ => { current_block_37 = 1622411330066726685; }
    }
    match current_block_37 {
        11814172370226575913 => { magHardware = MAG_NONE }
        _ => { }
    }
    if magHardware as libc::c_uint == MAG_NONE as libc::c_int as libc::c_uint
       {
        return 0 as libc::c_int != 0
    }
    detectedSensors[SENSOR_INDEX_MAG as libc::c_int as usize] =
        magHardware as uint8_t;
    sensorsSet(SENSOR_MAG as libc::c_int as uint32_t);
    return 1 as libc::c_int != 0;
}
// !SIMULATOR_BUILD
#[no_mangle]
pub unsafe extern "C" fn compassInit() -> bool {
    // initialize and calibration. turn on led during mag calibration (calibration routine blinks it)
    // calculate magnetic declination
    mag.magneticDeclination =
        0.0f32; // TODO investigate if this is actually needed if there is no mag sensor or if the value stored in the config should be used.
    if !compassDetect(&mut magDev) {
        return 0 as libc::c_int != 0
    } // heading is in 0.1deg units
    let deg: int16_t =
        ((*compassConfig()).mag_declination as libc::c_int /
             100 as libc::c_int) as int16_t;
    let min: int16_t =
        ((*compassConfig()).mag_declination as libc::c_int %
             100 as libc::c_int) as int16_t;
    mag.magneticDeclination =
        (deg as libc::c_int as libc::c_float +
             min as libc::c_float * (1.0f32 / 60.0f32)) *
            10 as libc::c_int as libc::c_float;
    ledSet(1 as libc::c_int, 1 as libc::c_int != 0);
    magDev.init.expect("non-null function pointer")(&mut magDev);
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    magInit = 1 as libc::c_int as uint8_t;
    if (*compassConfig()).mag_align as libc::c_uint !=
           ALIGN_DEFAULT as libc::c_int as libc::c_uint {
        magDev.magAlign = (*compassConfig()).mag_align
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn compassIsHealthy() -> bool {
    return mag.magADC[X as libc::c_int as usize] !=
               0 as libc::c_int as libc::c_float &&
               mag.magADC[Y as libc::c_int as usize] !=
                   0 as libc::c_int as libc::c_float &&
               mag.magADC[Z as libc::c_int as usize] !=
                   0 as libc::c_int as libc::c_float;
}
#[no_mangle]
pub unsafe extern "C" fn compassUpdate(mut currentTimeUs: timeUs_t) {
    static mut tCal: timeUs_t = 0 as libc::c_int as timeUs_t;
    static mut magZeroTempMin: flightDynamicsTrims_t =
        flightDynamicsTrims_u{raw: [0; 3],};
    static mut magZeroTempMax: flightDynamicsTrims_t =
        flightDynamicsTrims_u{raw: [0; 3],};
    magDev.read.expect("non-null function pointer")(&mut magDev,
                                                    magADCRaw.as_mut_ptr());
    let mut axis: libc::c_int = 0 as libc::c_int;
    while axis < 3 as libc::c_int {
        mag.magADC[axis as usize] = magADCRaw[axis as usize] as libc::c_float;
        axis += 1
    }
    alignSensors(mag.magADC.as_mut_ptr(), magDev.magAlign as uint8_t);
    let mut magZero: *mut flightDynamicsTrims_t =
        &mut (*(compassConfigMutable as
                    unsafe extern "C" fn()
                        -> *mut compassConfig_t)()).magZero;
    if stateFlags as libc::c_int & CALIBRATE_MAG as libc::c_int != 0 {
        tCal = currentTimeUs;
        let mut axis_0: libc::c_int = 0 as libc::c_int;
        while axis_0 < 3 as libc::c_int {
            (*magZero).raw[axis_0 as usize] = 0 as libc::c_int as int16_t;
            magZeroTempMin.raw[axis_0 as usize] =
                mag.magADC[axis_0 as usize] as int16_t;
            magZeroTempMax.raw[axis_0 as usize] =
                mag.magADC[axis_0 as usize] as int16_t;
            axis_0 += 1
        }
        stateFlags =
            (stateFlags as libc::c_int & !(CALIBRATE_MAG as libc::c_int)) as
                uint8_t
    }
    if magInit != 0 {
        // we apply offset only once mag calibration is done
        mag.magADC[X as libc::c_int as usize] -=
            (*magZero).raw[X as libc::c_int as usize] as libc::c_int as
                libc::c_float;
        mag.magADC[Y as libc::c_int as usize] -=
            (*magZero).raw[Y as libc::c_int as usize] as libc::c_int as
                libc::c_float;
        mag.magADC[Z as libc::c_int as usize] -=
            (*magZero).raw[Z as libc::c_int as usize] as libc::c_int as
                libc::c_float
    }
    if tCal != 0 as libc::c_int as libc::c_uint {
        if currentTimeUs.wrapping_sub(tCal) <
               30000000 as libc::c_int as libc::c_uint {
            // 30s: you have 30s to turn the multi in all directions
            ledToggle(0 as libc::c_int);
            let mut axis_1: libc::c_int = 0 as libc::c_int;
            while axis_1 < 3 as libc::c_int {
                if mag.magADC[axis_1 as usize] <
                       magZeroTempMin.raw[axis_1 as usize] as libc::c_int as
                           libc::c_float {
                    magZeroTempMin.raw[axis_1 as usize] =
                        mag.magADC[axis_1 as usize] as int16_t
                }
                if mag.magADC[axis_1 as usize] >
                       magZeroTempMax.raw[axis_1 as usize] as libc::c_int as
                           libc::c_float {
                    magZeroTempMax.raw[axis_1 as usize] =
                        mag.magADC[axis_1 as usize] as int16_t
                }
                axis_1 += 1
            }
        } else {
            tCal = 0 as libc::c_int as timeUs_t;
            let mut axis_2: libc::c_int = 0 as libc::c_int;
            while axis_2 < 3 as libc::c_int {
                (*magZero).raw[axis_2 as usize] =
                    ((magZeroTempMin.raw[axis_2 as usize] as libc::c_int +
                          magZeroTempMax.raw[axis_2 as usize] as libc::c_int)
                         / 2 as libc::c_int) as int16_t;
                axis_2 += 1
                // Calculate offsets
            }
            saveConfigAndNotify();
        }
    };
}
// USE_MAG
