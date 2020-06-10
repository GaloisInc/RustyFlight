use ::libc;
extern "C" {
    #[no_mangle]
    fn IOWrite(io: IO_t, value: bool);
    #[no_mangle]
    fn IOToggle(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct statusLedConfig_s {
    pub ioTags: [ioTag_t; 3],
    pub inversion: uint8_t,
}
pub type statusLedConfig_t = statusLedConfig_s;
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
#[no_mangle]
pub static mut statusLedConfig_Copy: statusLedConfig_t =
    statusLedConfig_t{ioTags: [0; 3], inversion: 0,};
#[no_mangle]
pub static mut statusLedConfig_System: statusLedConfig_t =
    statusLedConfig_t{ioTags: [0; 3], inversion: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut statusLedConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (505 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<statusLedConfig_t>()
                                      as libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &statusLedConfig_System as
                                     *const statusLedConfig_t as
                                     *mut statusLedConfig_t as *mut uint8_t,
                             copy:
                                 &statusLedConfig_Copy as
                                     *const statusLedConfig_t as
                                     *mut statusLedConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut statusLedConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_statusLedConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut statusLedConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
static mut leds: [IO_t; 3] =
    [0 as *const libc::c_void as *mut libc::c_void; 3];
static mut ledInversion: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_statusLedConfig(mut statusLedConfig:
                                                       *mut statusLedConfig_t) {
    (*statusLedConfig).ioTags[0 as libc::c_int as usize] =
        ((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             6 as libc::c_int) as ioTag_t;
    (*statusLedConfig).ioTags[1 as libc::c_int as usize] =
        ((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             5 as libc::c_int) as ioTag_t;
    (*statusLedConfig).ioTags[2 as libc::c_int as usize] =
        ((1 as libc::c_int + 1 as libc::c_int) << 4 as libc::c_int |
             4 as libc::c_int) as ioTag_t;
    (*statusLedConfig).inversion = 0 as libc::c_int as uint8_t;
}
// Helpful macros
#[no_mangle]
pub unsafe extern "C" fn ledInit(mut statusLedConfig:
                                     *const statusLedConfig_t) {
    ledInversion = (*statusLedConfig).inversion;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 3 as libc::c_int {
        if (*statusLedConfig).ioTags[i as usize] != 0 {
            leds[i as usize] =
                IOGetByTag((*statusLedConfig).ioTags[i as usize]);
            IOInit(leds[i as usize], OWNER_LED,
                   (i + 1 as libc::c_int) as uint8_t);
            IOConfigGPIO(leds[i as usize],
                         (0x1 as libc::c_uint |
                              (0 as libc::c_uint) << 2 as libc::c_int |
                              (0 as libc::c_uint) << 5 as libc::c_int) as
                             ioConfig_t);
        } else { leds[i as usize] = 0 as IO_t }
        i += 1
    }
    ledSet(0 as libc::c_int, 0 as libc::c_int != 0);
    ledSet(1 as libc::c_int, 0 as libc::c_int != 0);
    ledSet(2 as libc::c_int, 0 as libc::c_int != 0);
}
#[no_mangle]
pub unsafe extern "C" fn ledToggle(mut led: libc::c_int) {
    IOToggle(leds[led as usize]);
}
#[no_mangle]
pub unsafe extern "C" fn ledSet(mut led: libc::c_int, mut on: bool) {
    let inverted: bool =
        (1 as libc::c_int) << led & ledInversion as libc::c_int != 0;
    IOWrite(leds[led as usize],
            if on as libc::c_int != 0 {
                inverted as libc::c_int
            } else { !inverted as libc::c_int } != 0);
}
