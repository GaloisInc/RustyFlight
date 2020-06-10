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
    fn rxSpiDeviceInit(rxSpiConfig: *const rxSpiConfig_s) -> bool;
    #[no_mangle]
    fn cx10Nrf24Init(rxSpiConfig: *const rxSpiConfig_s,
                     rxRuntimeConfig: *mut rxRuntimeConfig_s) -> bool;
    #[no_mangle]
    fn cx10Nrf24SetRcDataFromPayload(rcData: *mut uint16_t,
                                     payload: *const uint8_t);
    #[no_mangle]
    fn cx10Nrf24DataReceived(payload: *mut uint8_t) -> rx_spi_received_e;
    #[no_mangle]
    fn v202Nrf24Init(rxSpiConfig: *const rxSpiConfig_s,
                     rxRuntimeConfig: *mut rxRuntimeConfig_s) -> bool;
    #[no_mangle]
    fn v202Nrf24SetRcDataFromPayload(rcData: *mut uint16_t,
                                     payload: *const uint8_t);
    #[no_mangle]
    fn v202Nrf24DataReceived(payload: *mut uint8_t) -> rx_spi_received_e;
    #[no_mangle]
    fn h8_3dNrf24Init(rxSpiConfig: *const rxSpiConfig_s,
                      rxRuntimeConfig: *mut rxRuntimeConfig_s) -> bool;
    #[no_mangle]
    fn h8_3dNrf24SetRcDataFromPayload(rcData: *mut uint16_t,
                                      payload: *const uint8_t);
    #[no_mangle]
    fn h8_3dNrf24DataReceived(payload: *mut uint8_t) -> rx_spi_received_e;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub struct rxSpiConfig_s {
    pub rx_spi_protocol: uint8_t,
    pub rx_spi_id: uint32_t,
    pub rx_spi_rf_channel_count: uint8_t,
    pub csnTag: ioTag_t,
    pub spibus: uint8_t,
}
pub type ioTag_t = uint8_t;
pub type C2RustUnnamed = libc::c_uint;
pub const RX_FRAME_DROPPED: C2RustUnnamed = 8;
pub const RX_FRAME_PROCESSING_REQUIRED: C2RustUnnamed = 4;
pub const RX_FRAME_FAILSAFE: C2RustUnnamed = 2;
pub const RX_FRAME_COMPLETE: C2RustUnnamed = 1;
pub const RX_FRAME_PENDING: C2RustUnnamed = 0;
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
// used by receiver driver to return channel data
pub type rcFrameStatusFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut rxRuntimeConfig_s) -> uint8_t>;
pub type rcReadRawDataFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxRuntimeConfig_s, _: uint8_t)
               -> uint16_t>;
pub type rxRuntimeConfig_t = rxRuntimeConfig_s;
pub type rxSpiConfig_t = rxSpiConfig_s;
pub type rx_spi_protocol_e = libc::c_uint;
pub const RX_SPI_PROTOCOL_COUNT: rx_spi_protocol_e = 13;
pub const RX_SPI_NRF24_KN: rx_spi_protocol_e = 12;
pub const RX_SPI_A7105_FLYSKY_2A: rx_spi_protocol_e = 11;
pub const RX_SPI_A7105_FLYSKY: rx_spi_protocol_e = 10;
pub const RX_SPI_FRSKY_X: rx_spi_protocol_e = 9;
pub const RX_SPI_FRSKY_D: rx_spi_protocol_e = 8;
pub const RX_SPI_NRF24_INAV: rx_spi_protocol_e = 7;
pub const RX_SPI_NRF24_H8_3D: rx_spi_protocol_e = 6;
pub const RX_SPI_NRF24_CX10A: rx_spi_protocol_e = 5;
pub const RX_SPI_NRF24_CX10: rx_spi_protocol_e = 4;
pub const RX_SPI_NRF24_SYMA_X5C: rx_spi_protocol_e = 3;
pub const RX_SPI_NRF24_SYMA_X: rx_spi_protocol_e = 2;
pub const RX_SPI_NRF24_V202_1M: rx_spi_protocol_e = 1;
pub const RX_SPI_NRF24_V202_250K: rx_spi_protocol_e = 0;
pub type rx_spi_received_e = libc::c_uint;
pub const RX_SPI_RECEIVED_DATA: rx_spi_received_e = 2;
pub const RX_SPI_RECEIVED_BIND: rx_spi_received_e = 1;
pub const RX_SPI_RECEIVED_NONE: rx_spi_received_e = 0;
pub type protocolDataReceivedFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut uint8_t) -> rx_spi_received_e>;
pub type protocolSetRcDataFromPayloadFnPtr
    =
    Option<unsafe extern "C" fn(_: *mut uint16_t, _: *const uint8_t) -> ()>;
// number of RC channels as reported by current input driver
// RX protocol
// type of SPI RX protocol
                                            // nrf24: 0 = v202 250kbps. (Must be enabled by FEATURE_RX_NRF24 first.)
// SPI Bus
// set true when a new packet is received
pub type protocolInitFnPtr
    =
    Option<unsafe extern "C" fn(_: *const rxSpiConfig_t,
                                _: *mut rxRuntimeConfig_t) -> bool>;
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
pub static mut rxSpiRcData: [uint16_t; 18] = [0; 18];
static mut rxSpiPayload: [uint8_t; 32] = [0; 32];
static mut rxSpiNewPacketAvailable: uint8_t = 0;
static mut protocolInit: protocolInitFnPtr = None;
static mut protocolDataReceived: protocolDataReceivedFnPtr = None;
static mut protocolSetRcDataFromPayload: protocolSetRcDataFromPayloadFnPtr =
    None;
unsafe extern "C" fn rxSpiReadRawRC(mut rxRuntimeConfig:
                                        *const rxRuntimeConfig_t,
                                    mut channel: uint8_t) -> uint16_t {
    if channel as libc::c_int >=
           (*rxRuntimeConfig).channelCount as libc::c_int {
        return 0 as libc::c_int as uint16_t
    }
    if rxSpiNewPacketAvailable != 0 {
        protocolSetRcDataFromPayload.expect("non-null function pointer")(rxSpiRcData.as_mut_ptr(),
                                                                         rxSpiPayload.as_mut_ptr());
        rxSpiNewPacketAvailable = 0 as libc::c_int as uint8_t
    }
    return rxSpiRcData[channel as usize];
}
unsafe extern "C" fn rxSpiSetProtocol(mut protocol: rx_spi_protocol_e)
 -> bool {
    match protocol as libc::c_uint {
        4 | 5 => {
            protocolInit =
                Some(cx10Nrf24Init as
                         unsafe extern "C" fn(_: *const rxSpiConfig_s,
                                              _: *mut rxRuntimeConfig_s)
                             -> bool);
            protocolDataReceived =
                Some(cx10Nrf24DataReceived as
                         unsafe extern "C" fn(_: *mut uint8_t)
                             -> rx_spi_received_e);
            protocolSetRcDataFromPayload =
                Some(cx10Nrf24SetRcDataFromPayload as
                         unsafe extern "C" fn(_: *mut uint16_t,
                                              _: *const uint8_t) -> ())
        }
        6 => {
            protocolInit =
                Some(h8_3dNrf24Init as
                         unsafe extern "C" fn(_: *const rxSpiConfig_s,
                                              _: *mut rxRuntimeConfig_s)
                             -> bool);
            protocolDataReceived =
                Some(h8_3dNrf24DataReceived as
                         unsafe extern "C" fn(_: *mut uint8_t)
                             -> rx_spi_received_e);
            protocolSetRcDataFromPayload =
                Some(h8_3dNrf24SetRcDataFromPayload as
                         unsafe extern "C" fn(_: *mut uint16_t,
                                              _: *const uint8_t) -> ())
        }
        0 | 1 | _ => {
            protocolInit =
                Some(v202Nrf24Init as
                         unsafe extern "C" fn(_: *const rxSpiConfig_s,
                                              _: *mut rxRuntimeConfig_s)
                             -> bool);
            protocolDataReceived =
                Some(v202Nrf24DataReceived as
                         unsafe extern "C" fn(_: *mut uint8_t)
                             -> rx_spi_received_e);
            protocolSetRcDataFromPayload =
                Some(v202Nrf24SetRcDataFromPayload as
                         unsafe extern "C" fn(_: *mut uint16_t,
                                              _: *const uint8_t) -> ())
            // USE_RX_FRSKY_SPI
        }
    }
    return 1 as libc::c_int != 0;
}
/*
 * Returns true if the RX has received new data.
 * Called from updateRx in rx.c, updateRx called from taskUpdateRxCheck.
 * If taskUpdateRxCheck returns true, then taskUpdateRxMain will shortly be called.
 */
unsafe extern "C" fn rxSpiFrameStatus(mut rxRuntimeConfig:
                                          *mut rxRuntimeConfig_t) -> uint8_t {
    if protocolDataReceived.expect("non-null function pointer")(rxSpiPayload.as_mut_ptr())
           as libc::c_uint ==
           RX_SPI_RECEIVED_DATA as libc::c_int as libc::c_uint {
        rxSpiNewPacketAvailable = 1 as libc::c_int as uint8_t;
        return RX_FRAME_COMPLETE as libc::c_int as uint8_t
    }
    return RX_FRAME_PENDING as libc::c_int as uint8_t;
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
// Used in MSP. Append at end.
// RC channels in AETR order
// RC channels as used by deviation
// return to home
/*
 * Set and initialize the RX protocol
 */
#[no_mangle]
pub unsafe extern "C" fn rxSpiInit(mut rxSpiConfig: *const rxSpiConfig_t,
                                   mut rxRuntimeConfig:
                                       *mut rxRuntimeConfig_t) -> bool {
    let mut ret: bool = 0 as libc::c_int != 0;
    if !rxSpiDeviceInit(rxSpiConfig) { return 0 as libc::c_int != 0 }
    if rxSpiSetProtocol((*rxSpiConfig).rx_spi_protocol as rx_spi_protocol_e) {
        ret =
            protocolInit.expect("non-null function pointer")(rxSpiConfig,
                                                             rxRuntimeConfig)
    }
    rxSpiNewPacketAvailable = 0 as libc::c_int as uint8_t;
    (*rxRuntimeConfig).rxRefreshRate = 20000 as libc::c_int as uint16_t;
    (*rxRuntimeConfig).rcReadRawFn =
        Some(rxSpiReadRawRC as
                 unsafe extern "C" fn(_: *const rxRuntimeConfig_t, _: uint8_t)
                     -> uint16_t);
    (*rxRuntimeConfig).rcFrameStatusFn =
        Some(rxSpiFrameStatus as
                 unsafe extern "C" fn(_: *mut rxRuntimeConfig_t) -> uint8_t);
    return ret;
}
