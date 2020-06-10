use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __int32_t = libc::c_int;
pub type __uint32_t = libc::c_uint;
pub type int32_t = __int32_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub test: *mut libc::c_void,
}
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
// baro calculation (filled params are pressure and temperature)
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
pub type baroCalculateFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t) -> ()>;
pub type baroOpFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut baroDev_s) -> ()>;
pub type baroDev_t = baroDev_s;
#[no_mangle]
pub static mut SystemCoreClock: uint32_t = 0;
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
static mut fakePressure: int32_t = 0;
static mut fakeTemperature: int32_t = 0;
unsafe extern "C" fn fakeBaroStartGet(mut baro: *mut baroDev_t) { }
unsafe extern "C" fn fakeBaroCalculate(mut pressure: *mut int32_t,
                                       mut temperature: *mut int32_t) {
    if !pressure.is_null() {
        *pressure = fakePressure
    } // pressure in Pa (0m MSL)
    if !temperature.is_null() {
        *temperature = fakeTemperature
    }; // temperature in 0.01 C = 25 deg
}
#[no_mangle]
pub unsafe extern "C" fn fakeBaroSet(mut pressure: int32_t,
                                     mut temperature: int32_t) {
    fakePressure = pressure;
    fakeTemperature = temperature;
}
#[no_mangle]
pub unsafe extern "C" fn fakeBaroDetect(mut baro: *mut baroDev_t) -> bool {
    fakePressure = 101325 as libc::c_int;
    fakeTemperature = 2500 as libc::c_int;
    // these are dummy as temperature is measured as part of pressure
    (*baro).ut_delay = 10000 as libc::c_int as uint16_t;
    (*baro).get_ut =
        Some(fakeBaroStartGet as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).start_ut =
        Some(fakeBaroStartGet as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    // only _up part is executed, and gets both temperature and pressure
    (*baro).up_delay = 10000 as libc::c_int as uint16_t;
    (*baro).start_up =
        Some(fakeBaroStartGet as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).get_up =
        Some(fakeBaroStartGet as
                 unsafe extern "C" fn(_: *mut baroDev_t) -> ());
    (*baro).calculate =
        Some(fakeBaroCalculate as
                 unsafe extern "C" fn(_: *mut int32_t, _: *mut int32_t)
                     -> ());
    return 1 as libc::c_int != 0;
}
// USE_FAKE_BARO
