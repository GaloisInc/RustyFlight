use ::libc;
extern "C" {
    #[no_mangle]
    fn busWriteRegister(bus: *const busDevice_t, reg: uint8_t, data: uint8_t)
     -> bool;
    #[no_mangle]
    fn busReadRegisterBuffer(bus: *const busDevice_t, reg: uint8_t,
                             data: *mut uint8_t, length: uint8_t) -> bool;
    #[no_mangle]
    fn delay(ms: timeMs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type __int64_t = libc::c_long;
pub type int16_t = __int16_t;
pub type int32_t = __int32_t;
pub type int64_t = __int64_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
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
    pub busdev_u: C2RustUnnamed,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed {
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
// baro start operation
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
// baro calculation (filled params are pressure and temperature)
pub type baroDev_t = baroDev_s;
// millisecond time
pub type timeMs_t = uint32_t;
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
pub type bmp280_calib_param_t = bmp280_calib_param_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bmp280_calib_param_s {
    pub dig_T1: uint16_t,
    pub dig_T2: int16_t,
    pub dig_T3: int16_t,
    pub dig_P1: uint16_t,
    pub dig_P2: int16_t,
    pub dig_P3: int16_t,
    pub dig_P4: int16_t,
    pub dig_P5: int16_t,
    pub dig_P6: int16_t,
    pub dig_P7: int16_t,
    pub dig_P8: int16_t,
    pub dig_P9: int16_t,
    pub t_fine: int32_t,
}
static mut bmp280_chip_id: uint8_t = 0 as libc::c_int as uint8_t;
static mut bmp280_cal: bmp280_calib_param_t =
    bmp280_calib_param_t{dig_T1: 0,
                         dig_T2: 0,
                         dig_T3: 0,
                         dig_P1: 0,
                         dig_P2: 0,
                         dig_P3: 0,
                         dig_P4: 0,
                         dig_P5: 0,
                         dig_P6: 0,
                         dig_P7: 0,
                         dig_P8: 0,
                         dig_P9: 0,
                         t_fine: 0,};
/* calibration T1 data */
/* calibration T2 data */
/* calibration T3 data */
/* calibration P1 data */
/* calibration P2 data */
/* calibration P3 data */
/* calibration P4 data */
/* calibration P5 data */
/* calibration P6 data */
/* calibration P7 data */
/* calibration P8 data */
/* calibration P9 data */
/* calibration t_fine data */
// uncompensated pressure and temperature
#[no_mangle]
pub static mut bmp280_up: int32_t = 0 as libc::c_int;
#[no_mangle]
pub static mut bmp280_ut: int32_t = 0 as libc::c_int;
#[no_mangle]
pub unsafe extern "C" fn bmp280BusInit(mut busdev: *mut busDevice_t) { }
#[no_mangle]
pub unsafe extern "C" fn bmp280BusDeinit(mut busdev: *mut busDevice_t) { }
// 10/16 = 0.625 ms
#[no_mangle]
pub unsafe extern "C" fn bmp280Detect(mut baro: *mut baroDev_t) -> bool {
    delay(20 as libc::c_int as timeMs_t);
    let mut busdev: *mut busDevice_t = &mut (*baro).busdev;
    let mut defaultAddressApplied: bool = 0 as libc::c_int != 0;
    bmp280BusInit(busdev);
    if (*busdev).bustype as libc::c_uint ==
           BUSTYPE_I2C as libc::c_int as libc::c_uint &&
           (*busdev).busdev_u.i2c.address as libc::c_int == 0 as libc::c_int {
        // Default address for BMP280
        (*busdev).busdev_u.i2c.address =
            0x76 as libc::c_int as uint8_t; /* read Chip Id */
        defaultAddressApplied = 1 as libc::c_int != 0
    }
    busReadRegisterBuffer(busdev, 0xd0 as libc::c_int as uint8_t,
                          &mut bmp280_chip_id, 1 as libc::c_int as uint8_t);
    if bmp280_chip_id as libc::c_int != 0x58 as libc::c_int {
        bmp280BusDeinit(busdev);
        if defaultAddressApplied {
            (*busdev).busdev_u.i2c.address = 0 as libc::c_int as uint8_t
        }
        return 0 as libc::c_int != 0
    }
    // read calibration
    busReadRegisterBuffer(busdev, 0x88 as libc::c_int as uint8_t,
                          &mut bmp280_cal as *mut bmp280_calib_param_t as
                              *mut uint8_t, 24 as libc::c_int as uint8_t);
    // set oversampling + power mode (forced), and start sampling
    busWriteRegister(busdev, 0xf4 as libc::c_int as uint8_t,
                     ((0x4 as libc::c_int) << 2 as libc::c_int |
                          (0x1 as libc::c_int) << 5 as libc::c_int |
                          0x1 as libc::c_int) as uint8_t);
    // these are dummy as temperature is measured as part of pressure
    (*baro).ut_delay = 0 as libc::c_int as uint16_t;
    (*baro).get_ut =
        Some(bmp280_get_ut as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).start_ut =
        Some(bmp280_start_ut as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    // only _up part is executed, and gets both temperature and pressure
    (*baro).start_up =
        Some(bmp280_start_up as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).get_up =
        Some(bmp280_get_up as unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).up_delay =
        ((20 as libc::c_int +
              37 as libc::c_int *
                  (((1 as libc::c_int) << 0x1 as libc::c_int >>
                        1 as libc::c_int) +
                       ((1 as libc::c_int) << 0x4 as libc::c_int >>
                            1 as libc::c_int)) +
              (if 0x4 as libc::c_int != 0 {
                   10 as libc::c_int
               } else { 0 as libc::c_int }) + 15 as libc::c_int) /
             16 as libc::c_int * 1000 as libc::c_int) as uint16_t;
    (*baro).calculate =
        Some(bmp280_calculate as
                 unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t)
                     -> ());
    return 1 as libc::c_int != 0;
}
unsafe extern "C" fn bmp280_start_ut(mut baro: *mut baroDev_t) {
    // dummy
}
unsafe extern "C" fn bmp280_get_ut(mut baro: *mut baroDev_t) {
    // dummy
}
unsafe extern "C" fn bmp280_start_up(mut baro: *mut baroDev_t) {
    // start measurement
    // set oversampling + power mode (forced), and start sampling
    busWriteRegister(&mut (*baro).busdev, 0xf4 as libc::c_int as uint8_t,
                     ((0x4 as libc::c_int) << 2 as libc::c_int |
                          (0x1 as libc::c_int) << 5 as libc::c_int |
                          0x1 as libc::c_int) as uint8_t);
}
unsafe extern "C" fn bmp280_get_up(mut baro: *mut baroDev_t) {
    let mut data: [uint8_t; 6] = [0; 6];
    // read data from sensor
    busReadRegisterBuffer(&mut (*baro).busdev, 0xf7 as libc::c_int as uint8_t,
                          data.as_mut_ptr(), 6 as libc::c_int as uint8_t);
    bmp280_up =
        ((data[0 as libc::c_int as usize] as uint32_t) << 12 as libc::c_int |
             (data[1 as libc::c_int as usize] as uint32_t) << 4 as libc::c_int
             |
             data[2 as libc::c_int as usize] as uint32_t >> 4 as libc::c_int)
            as int32_t;
    bmp280_ut =
        ((data[3 as libc::c_int as usize] as uint32_t) << 12 as libc::c_int |
             (data[4 as libc::c_int as usize] as uint32_t) << 4 as libc::c_int
             |
             data[5 as libc::c_int as usize] as uint32_t >> 4 as libc::c_int)
            as int32_t;
}
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
unsafe extern "C" fn bmp280_compensate_T(mut adc_T: int32_t) -> int32_t {
    let mut var1: int32_t = 0;
    let mut var2: int32_t = 0;
    let mut T: int32_t = 0;
    var1 =
        ((adc_T >> 3 as libc::c_int) -
             ((bmp280_cal.dig_T1 as int32_t) << 1 as libc::c_int)) *
            bmp280_cal.dig_T2 as int32_t >> 11 as libc::c_int;
    var2 =
        (((adc_T >> 4 as libc::c_int) - bmp280_cal.dig_T1 as int32_t) *
             ((adc_T >> 4 as libc::c_int) - bmp280_cal.dig_T1 as int32_t) >>
             12 as libc::c_int) * bmp280_cal.dig_T3 as int32_t >>
            14 as libc::c_int;
    bmp280_cal.t_fine = var1 + var2;
    T =
        bmp280_cal.t_fine * 5 as libc::c_int + 128 as libc::c_int >>
            8 as libc::c_int;
    return T;
}
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
unsafe extern "C" fn bmp280_compensate_P(mut adc_P: int32_t) -> uint32_t {
    let mut var1: int64_t = 0;
    let mut var2: int64_t = 0;
    let mut p: int64_t = 0;
    var1 =
        bmp280_cal.t_fine as int64_t - 128000 as libc::c_int as libc::c_long;
    var2 = var1 * var1 * bmp280_cal.dig_P6 as int64_t;
    var2 =
        var2 + ((var1 * bmp280_cal.dig_P5 as int64_t) << 17 as libc::c_int);
    var2 = var2 + ((bmp280_cal.dig_P4 as int64_t) << 35 as libc::c_int);
    var1 =
        (var1 * var1 * bmp280_cal.dig_P3 as int64_t >> 8 as libc::c_int) +
            ((var1 * bmp280_cal.dig_P2 as int64_t) << 12 as libc::c_int);
    var1 =
        (((1 as libc::c_int as int64_t) << 47 as libc::c_int) + var1) *
            bmp280_cal.dig_P1 as int64_t >> 33 as libc::c_int;
    if var1 == 0 as libc::c_int as libc::c_long {
        return 0 as libc::c_int as uint32_t
    }
    p = (1048576 as libc::c_int - adc_P) as int64_t;
    p =
        ((p << 31 as libc::c_int) - var2) *
            3125 as libc::c_int as libc::c_long / var1;
    var1 =
        bmp280_cal.dig_P9 as int64_t * (p >> 13 as libc::c_int) *
            (p >> 13 as libc::c_int) >> 25 as libc::c_int;
    var2 = bmp280_cal.dig_P8 as int64_t * p >> 19 as libc::c_int;
    p =
        (p + var1 + var2 >> 8 as libc::c_int) +
            ((bmp280_cal.dig_P7 as int64_t) << 4 as libc::c_int);
    return p as uint32_t;
}
unsafe extern "C" fn bmp280_calculate(mut pressure: *mut int32_t,
                                      mut temperature: *mut int32_t) {
    // calculate
    let mut t: int32_t = 0;
    let mut p: uint32_t = 0;
    t = bmp280_compensate_T(bmp280_ut);
    p = bmp280_compensate_P(bmp280_up);
    if !pressure.is_null() {
        *pressure =
            p.wrapping_div(256 as libc::c_int as libc::c_uint) as int32_t
    }
    if !temperature.is_null() { *temperature = t };
}
