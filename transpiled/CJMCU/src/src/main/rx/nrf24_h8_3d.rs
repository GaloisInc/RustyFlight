use ::libc;
extern "C" {
    #[no_mangle]
    fn memcpy(_: *mut libc::c_void, _: *const libc::c_void, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn NRF24L01_Initialize(baseConfig: uint8_t);
    #[no_mangle]
    fn NRF24L01_WriteReg(reg: uint8_t, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn NRF24L01_WriteRegisterMulti(reg: uint8_t, data: *const uint8_t,
                                   length: uint8_t) -> uint8_t;
    #[no_mangle]
    fn NRF24L01_SetupBasic();
    #[no_mangle]
    fn NRF24L01_SetRxMode();
    #[no_mangle]
    fn NRF24L01_SetChannel(channel: uint8_t);
    #[no_mangle]
    fn NRF24L01_ReadPayloadIfAvailable(data: *mut uint8_t, length: uint8_t)
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
    fn XN297_UnscramblePayload(data: *mut uint8_t, len: libc::c_int,
                               rxAddr: *const uint8_t) -> uint16_t;
    #[no_mangle]
    fn micros() -> timeUs_t;
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
pub type timeUs_t = uint32_t;
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
pub type rxSpiConfig_t = rxSpiConfig_s;
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
// RC channels in AETR order
pub type C2RustUnnamed = libc::c_uint;
pub const RC_SPI_AUX14: C2RustUnnamed = 17;
pub const RC_SPI_AUX13: C2RustUnnamed = 16;
pub const RC_SPI_AUX12: C2RustUnnamed = 15;
pub const RC_SPI_AUX11: C2RustUnnamed = 14;
pub const RC_SPI_AUX10: C2RustUnnamed = 13;
pub const RC_SPI_AUX9: C2RustUnnamed = 12;
pub const RC_SPI_AUX8: C2RustUnnamed = 11;
pub const RC_SPI_AUX7: C2RustUnnamed = 10;
pub const RC_SPI_AUX6: C2RustUnnamed = 9;
pub const RC_SPI_AUX5: C2RustUnnamed = 8;
pub const RC_SPI_AUX4: C2RustUnnamed = 7;
pub const RC_SPI_AUX3: C2RustUnnamed = 6;
pub const RC_SPI_AUX2: C2RustUnnamed = 5;
pub const RC_SPI_AUX1: C2RustUnnamed = 4;
pub const RC_SPI_YAW: C2RustUnnamed = 3;
pub const RC_SPI_THROTTLE: C2RustUnnamed = 2;
pub const RC_SPI_PITCH: C2RustUnnamed = 1;
pub const RC_SPI_ROLL: C2RustUnnamed = 0;
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
// This file is copied with modifications from project Deviation,
// see http://deviationtx.com, file iface_nrf24l01.h
// bit value
// Register map of nRF24L01
pub type C2RustUnnamed_0 = libc::c_uint;
// DYNamic PayloaD
pub const NRF24L01_1D_FEATURE: C2RustUnnamed_0 = 29;
pub const NRF24L01_1C_DYNPD: C2RustUnnamed_0 = 28;
pub const NRF24L01_17_FIFO_STATUS: C2RustUnnamed_0 = 23;
pub const NRF24L01_16_RX_PW_P5: C2RustUnnamed_0 = 22;
pub const NRF24L01_15_RX_PW_P4: C2RustUnnamed_0 = 21;
pub const NRF24L01_14_RX_PW_P3: C2RustUnnamed_0 = 20;
pub const NRF24L01_13_RX_PW_P2: C2RustUnnamed_0 = 19;
// Payload Width
pub const NRF24L01_12_RX_PW_P1: C2RustUnnamed_0 = 18;
pub const NRF24L01_11_RX_PW_P0: C2RustUnnamed_0 = 17;
pub const NRF24L01_10_TX_ADDR: C2RustUnnamed_0 = 16;
pub const NRF24L01_0F_RX_ADDR_P5: C2RustUnnamed_0 = 15;
pub const NRF24L01_0E_RX_ADDR_P4: C2RustUnnamed_0 = 14;
pub const NRF24L01_0D_RX_ADDR_P3: C2RustUnnamed_0 = 13;
pub const NRF24L01_0C_RX_ADDR_P2: C2RustUnnamed_0 = 12;
pub const NRF24L01_0B_RX_ADDR_P1: C2RustUnnamed_0 = 11;
//Received Power Detector in the nRF23L01+, called CD (Carrier Detect) in the nRF24L01
pub const NRF24L01_0A_RX_ADDR_P0: C2RustUnnamed_0 = 10;
pub const NRF24L01_09_RPD: C2RustUnnamed_0 = 9;
pub const NRF24L01_08_OBSERVE_TX: C2RustUnnamed_0 = 8;
pub const NRF24L01_07_STATUS: C2RustUnnamed_0 = 7;
// RF CHannel
pub const NRF24L01_06_RF_SETUP: C2RustUnnamed_0 = 6;
// automatic RETRansmission
pub const NRF24L01_05_RF_CH: C2RustUnnamed_0 = 5;
// Address Width
pub const NRF24L01_04_SETUP_RETR: C2RustUnnamed_0 = 4;
pub const NRF24L01_03_SETUP_AW: C2RustUnnamed_0 = 3;
// Auto Acknowledge
pub const NRF24L01_02_EN_RXADDR: C2RustUnnamed_0 = 2;
pub const NRF24L01_01_EN_AA: C2RustUnnamed_0 = 1;
pub const NRF24L01_00_CONFIG: C2RustUnnamed_0 = 0;
// Pre-shifted and combined bits
pub type C2RustUnnamed_1 = libc::c_uint;
pub const NRF24L01_1C_DYNPD_ALL_PIPES: C2RustUnnamed_1 = 63;
pub const NRF24L01_06_RF_SETUP_RF_PWR_0dbm: C2RustUnnamed_1 = 6;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n6dbm: C2RustUnnamed_1 = 4;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n12dbm: C2RustUnnamed_1 = 2;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n18dbm: C2RustUnnamed_1 = 1;
pub const NRF24L01_06_RF_SETUP_RF_DR_250Kbps: C2RustUnnamed_1 = 32;
pub const NRF24L01_06_RF_SETUP_RF_DR_1Mbps: C2RustUnnamed_1 = 0;
pub const NRF24L01_06_RF_SETUP_RF_DR_2Mbps: C2RustUnnamed_1 = 8;
pub const NRF24L01_04_SETUP_RETR_ARC_15: C2RustUnnamed_1 = 15;
pub const NRF24L01_04_SETUP_RETR_ARC_14: C2RustUnnamed_1 = 14;
pub const NRF24L01_04_SETUP_RETR_ARC_13: C2RustUnnamed_1 = 13;
pub const NRF24L01_04_SETUP_RETR_ARC_12: C2RustUnnamed_1 = 12;
pub const NRF24L01_04_SETUP_RETR_ARC_11: C2RustUnnamed_1 = 11;
pub const NRF24L01_04_SETUP_RETR_ARC_10: C2RustUnnamed_1 = 10;
pub const NRF24L01_04_SETUP_RETR_ARC_9: C2RustUnnamed_1 = 9;
pub const NRF24L01_04_SETUP_RETR_ARC_8: C2RustUnnamed_1 = 8;
pub const NRF24L01_04_SETUP_RETR_ARC_7: C2RustUnnamed_1 = 7;
pub const NRF24L01_04_SETUP_RETR_ARC_6: C2RustUnnamed_1 = 6;
pub const NRF24L01_04_SETUP_RETR_ARC_5: C2RustUnnamed_1 = 5;
pub const NRF24L01_04_SETUP_RETR_ARC_4: C2RustUnnamed_1 = 4;
pub const NRF24L01_04_SETUP_RETR_ARC_3: C2RustUnnamed_1 = 3;
pub const NRF24L01_04_SETUP_RETR_ARC_2: C2RustUnnamed_1 = 2;
pub const NRF24L01_04_SETUP_RETR_ARC_1: C2RustUnnamed_1 = 1;
pub const NRF24L01_04_SETUP_RETR_ARC_0: C2RustUnnamed_1 = 0;
pub const NRF24L01_04_SETUP_RETR_ARD_4000us: C2RustUnnamed_1 = 240;
pub const NRF24L01_04_SETUP_RETR_ARD_3750us: C2RustUnnamed_1 = 224;
pub const NRF24L01_04_SETUP_RETR_ARD_3500us: C2RustUnnamed_1 = 208;
pub const NRF24L01_04_SETUP_RETR_ARD_3250us: C2RustUnnamed_1 = 192;
pub const NRF24L01_04_SETUP_RETR_ARD_3000us: C2RustUnnamed_1 = 176;
pub const NRF24L01_04_SETUP_RETR_ARD_2750us: C2RustUnnamed_1 = 160;
pub const NRF24L01_04_SETUP_RETR_ARD_2500us: C2RustUnnamed_1 = 144;
pub const NRF24L01_04_SETUP_RETR_ARD_2250us: C2RustUnnamed_1 = 128;
pub const NRF24L01_04_SETUP_RETR_ARD_2000us: C2RustUnnamed_1 = 112;
pub const NRF24L01_04_SETUP_RETR_ARD_1750us: C2RustUnnamed_1 = 96;
pub const NRF24L01_04_SETUP_RETR_ARD_1500us: C2RustUnnamed_1 = 80;
pub const NRF24L01_04_SETUP_RETR_ARD_1250us: C2RustUnnamed_1 = 64;
pub const NRF24L01_04_SETUP_RETR_ARD_1000us: C2RustUnnamed_1 = 48;
pub const NRF24L01_04_SETUP_RETR_ARD_750us: C2RustUnnamed_1 = 32;
pub const NRF24L01_04_SETUP_RETR_ARD_500us: C2RustUnnamed_1 = 16;
pub const NRF24L01_04_SETUP_RETR_ARD_250us: C2RustUnnamed_1 = 0;
pub const NRF24L01_03_SETUP_AW_5BYTES: C2RustUnnamed_1 = 3;
pub const NRF24L01_03_SETUP_AW_4BYTES: C2RustUnnamed_1 = 2;
pub const NRF24L01_03_SETUP_AW_3BYTES: C2RustUnnamed_1 = 1;
pub const NRF24L01_02_EN_RXADDR_ERX_ALL_PIPES: C2RustUnnamed_1 = 63;
pub const NRF24L01_01_EN_AA_ALL_PIPES: C2RustUnnamed_1 = 63;
pub type protocol_state_t = libc::c_uint;
pub const STATE_DATA: protocol_state_t = 1;
pub const STATE_BIND: protocol_state_t = 0;
static mut protocolState: protocol_state_t = STATE_BIND;
static mut payloadSize: uint8_t = 0;
static mut rxTxAddrXN297: [uint8_t; 5] =
    [0x41 as libc::c_int as uint8_t, 0xbd as libc::c_int as uint8_t,
     0x42 as libc::c_int as uint8_t, 0xd4 as libc::c_int as uint8_t,
     0xc2 as libc::c_int as uint8_t];
// converted XN297 address
static mut txId: [uint8_t; 4] = [0; 4];
#[no_mangle]
pub static mut rxSpiIdPtr: *mut uint32_t =
    0 as *const uint32_t as *mut uint32_t;
static mut h8_3dRfChannelCount: uint8_t = 4 as libc::c_int as uint8_t;
static mut h8_3dRfChannelIndex: uint8_t = 0;
static mut h8_3dRfChannels: [uint8_t; 4] = [0; 4];
// 1ms, to find the bind channel as quickly as possible
static mut hopTimeout: uint32_t = 1000 as libc::c_int as uint32_t;
static mut timeOfLastHop: uint32_t = 0;
unsafe extern "C" fn h8_3dCheckBindPacket(mut payload: *const uint8_t)
 -> bool {
    let mut bindPacket: bool = 0 as libc::c_int != 0;
    if *payload.offset(5 as libc::c_int as isize) as libc::c_int ==
           0 as libc::c_int &&
           *payload.offset(6 as libc::c_int as isize) as libc::c_int ==
               0 as libc::c_int &&
           *payload.offset(7 as libc::c_int as isize) as libc::c_int ==
               0x1 as libc::c_int {
        let checkSumTxId: uint32_t =
            (*payload.offset(1 as libc::c_int as isize) as libc::c_int +
                 *payload.offset(2 as libc::c_int as isize) as libc::c_int +
                 *payload.offset(3 as libc::c_int as isize) as libc::c_int +
                 *payload.offset(4 as libc::c_int as isize) as libc::c_int &
                 0xff as libc::c_int) as uint32_t;
        if checkSumTxId ==
               *payload.offset(8 as libc::c_int as isize) as libc::c_uint {
            bindPacket = 1 as libc::c_int != 0;
            txId[0 as libc::c_int as usize] =
                *payload.offset(1 as libc::c_int as isize);
            txId[1 as libc::c_int as usize] =
                *payload.offset(2 as libc::c_int as isize);
            txId[2 as libc::c_int as usize] =
                *payload.offset(3 as libc::c_int as isize);
            txId[3 as libc::c_int as usize] =
                *payload.offset(4 as libc::c_int as isize);
            if !rxSpiIdPtr.is_null() &&
                   *rxSpiIdPtr == 0 as libc::c_int as libc::c_uint {
                // copy the txId so it can be saved
                memcpy(rxSpiIdPtr as *mut libc::c_void,
                       txId.as_mut_ptr() as *const libc::c_void,
                       ::core::mem::size_of::<uint32_t>() as
                           libc::c_ulong); // aileron
            }
        }
    } // elevator
    return bindPacket; // throttle
}
unsafe extern "C" fn h8_3dConvertToPwm(mut val: uint8_t, mut _min: int16_t,
                                       mut _max: int16_t) -> uint16_t {
    let mut ret: int32_t = val as int32_t; // rudder
    let range: int32_t = _max as libc::c_int - _min as libc::c_int;
    ret =
        1000 as libc::c_int +
            (ret - _min as libc::c_int) *
                (2000 as libc::c_int - 1000 as libc::c_int) / range;
    return ret as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn h8_3dNrf24SetRcDataFromPayload(mut rcData:
                                                            *mut uint16_t,
                                                        mut payload:
                                                            *const uint8_t) {
    *rcData.offset(RC_SPI_ROLL as libc::c_int as isize) =
        h8_3dConvertToPwm(*payload.offset(12 as libc::c_int as isize),
                          0xbb as libc::c_int as int16_t,
                          0x43 as libc::c_int as int16_t);
    *rcData.offset(RC_SPI_PITCH as libc::c_int as isize) =
        h8_3dConvertToPwm(*payload.offset(11 as libc::c_int as isize),
                          0x43 as libc::c_int as int16_t,
                          0xbb as libc::c_int as int16_t);
    *rcData.offset(RC_SPI_THROTTLE as libc::c_int as isize) =
        h8_3dConvertToPwm(*payload.offset(9 as libc::c_int as isize),
                          0 as libc::c_int as int16_t,
                          0xff as libc::c_int as int16_t);
    let yawByte: int8_t =
        *payload.offset(10 as libc::c_int as isize) as int8_t;
    *rcData.offset(RC_SPI_YAW as libc::c_int as isize) =
        if yawByte as libc::c_int >= 0 as libc::c_int {
            h8_3dConvertToPwm(yawByte as uint8_t,
                              -(0x3c as libc::c_int) as int16_t,
                              0x3c as libc::c_int as int16_t) as libc::c_int
        } else {
            h8_3dConvertToPwm(yawByte as uint8_t,
                              0xbc as libc::c_int as int16_t,
                              0x44 as libc::c_int as int16_t) as libc::c_int
        } as uint16_t;
    let flags: uint8_t = *payload.offset(17 as libc::c_int as isize);
    let flags2: uint8_t = *payload.offset(18 as libc::c_int as isize);
    if flags as libc::c_int & 0x4 as libc::c_int != 0 {
        *rcData.offset(RC_SPI_AUX1 as libc::c_int as isize) =
            2000 as libc::c_int as uint16_t
    } else if flags as libc::c_int & 0x2 as libc::c_int != 0 {
        *rcData.offset(RC_SPI_AUX1 as libc::c_int as isize) =
            (1000 as libc::c_int +
                 (2000 as libc::c_int - 1000 as libc::c_int) /
                     2 as libc::c_int) as uint16_t
    } else {
        *rcData.offset(RC_SPI_AUX1 as libc::c_int as isize) =
            1000 as libc::c_int as uint16_t
    }
    *rcData.offset(RC_SPI_AUX2 as libc::c_int as isize) =
        if flags as libc::c_int & 0x1 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    *rcData.offset(RC_SPI_AUX3 as libc::c_int as isize) =
        if flags2 as libc::c_int & 0x40 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    *rcData.offset(RC_SPI_AUX4 as libc::c_int as isize) =
        if flags2 as libc::c_int & 0x80 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    *rcData.offset(RC_SPI_AUX5 as libc::c_int as isize) =
        if flags as libc::c_int & 0x10 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    *rcData.offset(RC_SPI_AUX6 as libc::c_int as isize) =
        if flags as libc::c_int & 0x20 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    if flags2 as libc::c_int & 0x4 as libc::c_int != 0 {
        *rcData.offset(RC_SPI_AUX7 as libc::c_int as isize) =
            2000 as libc::c_int as uint16_t
    } else if flags2 as libc::c_int & 0x8 as libc::c_int != 0 {
        *rcData.offset(RC_SPI_AUX7 as libc::c_int as isize) =
            1000 as libc::c_int as uint16_t
    } else {
        *rcData.offset(RC_SPI_AUX7 as libc::c_int as isize) =
            (1000 as libc::c_int +
                 (2000 as libc::c_int - 1000 as libc::c_int) /
                     2 as libc::c_int) as uint16_t
    }
    *rcData.offset(RC_SPI_AUX8 as libc::c_int as isize) =
        h8_3dConvertToPwm(*payload.offset(14 as libc::c_int as isize),
                          0x10 as libc::c_int as int16_t,
                          0x30 as libc::c_int as int16_t);
    *rcData.offset(RC_SPI_AUX9 as libc::c_int as isize) =
        h8_3dConvertToPwm(*payload.offset(15 as libc::c_int as isize),
                          0x30 as libc::c_int as int16_t,
                          0x10 as libc::c_int as int16_t);
    *rcData.offset(RC_SPI_AUX10 as libc::c_int as isize) =
        h8_3dConvertToPwm(*payload.offset(16 as libc::c_int as isize),
                          0x10 as libc::c_int as int16_t,
                          0x30 as libc::c_int as int16_t);
}
unsafe extern "C" fn h8_3dHopToNextChannel() {
    h8_3dRfChannelIndex = h8_3dRfChannelIndex.wrapping_add(1);
    if protocolState as libc::c_uint ==
           STATE_BIND as libc::c_int as libc::c_uint {
        if h8_3dRfChannelIndex as libc::c_int > 0x26 as libc::c_int {
            h8_3dRfChannelIndex = 0x6 as libc::c_int as uint8_t
        }
        NRF24L01_SetChannel(h8_3dRfChannelIndex);
    } else {
        if h8_3dRfChannelIndex as libc::c_int >=
               h8_3dRfChannelCount as libc::c_int {
            h8_3dRfChannelIndex = 0 as libc::c_int as uint8_t
        }
        NRF24L01_SetChannel(h8_3dRfChannels[h8_3dRfChannelIndex as usize]);
    };
}
// The hopping channels are determined by the txId
unsafe extern "C" fn h8_3dSetHoppingChannels(mut txId_0: *const uint8_t) {
    let mut ii: libc::c_int = 0 as libc::c_int;
    while ii < 4 as libc::c_int {
        h8_3dRfChannels[ii as usize] =
            (0x6 as libc::c_int + 0xf as libc::c_int * ii +
                 ((*txId_0.offset(ii as isize) as libc::c_int >>
                       4 as libc::c_int) +
                      (*txId_0.offset(ii as isize) as libc::c_int &
                           0xf as libc::c_int)) % 0xf as libc::c_int) as
                uint8_t;
        ii += 1
    };
}
unsafe extern "C" fn h8_3dSetBound(mut txId_0: *const uint8_t) {
    protocolState = STATE_DATA;
    h8_3dSetHoppingChannels(txId_0);
    hopTimeout = 5000 as libc::c_int as uint32_t;
    timeOfLastHop = micros();
    h8_3dRfChannelIndex = 0 as libc::c_int as uint8_t;
    NRF24L01_SetChannel(h8_3dRfChannels[0 as libc::c_int as usize]);
}
unsafe extern "C" fn h8_3dCrcOK(mut crc: uint16_t,
                                mut payload: *const uint8_t) -> bool {
    if *payload.offset(payloadSize as isize) as libc::c_int !=
           crc as libc::c_int >> 8 as libc::c_int {
        return 0 as libc::c_int != 0
    }
    if *payload.offset((payloadSize as libc::c_int + 1 as libc::c_int) as
                           isize) as libc::c_int !=
           crc as libc::c_int & 0xff as libc::c_int {
        return 0 as libc::c_int != 0
    }
    return 1 as libc::c_int != 0;
}
/*
 * This is called periodically by the scheduler.
 * Returns NRF24L01_RECEIVED_DATA if a data packet was received.
 */
#[no_mangle]
pub unsafe extern "C" fn h8_3dNrf24DataReceived(mut payload: *mut uint8_t)
 -> rx_spi_received_e {
    let mut ret: rx_spi_received_e =
        RX_SPI_RECEIVED_NONE; // sets PWR_UP, no CRC - hardware CRC not used for XN297
    let mut payloadReceived: bool = 0 as libc::c_int != 0;
    if NRF24L01_ReadPayloadIfAvailable(payload,
                                       (payloadSize as libc::c_int +
                                            2 as libc::c_int) as uint8_t) {
        let crc: uint16_t =
            XN297_UnscramblePayload(payload, payloadSize as libc::c_int,
                                    rxTxAddrXN297.as_mut_ptr());
        if h8_3dCrcOK(crc, payload) {
            payloadReceived = 1 as libc::c_int != 0
        }
    }
    match protocolState as libc::c_uint {
        0 => {
            if payloadReceived {
                let bindPacket: bool = h8_3dCheckBindPacket(payload);
                if bindPacket {
                    ret = RX_SPI_RECEIVED_BIND;
                    h8_3dSetBound(txId.as_mut_ptr());
                }
            }
        }
        1 => { if payloadReceived { ret = RX_SPI_RECEIVED_DATA } }
        _ => { }
    }
    let timeNowUs: uint32_t = micros();
    if ret as libc::c_uint ==
           RX_SPI_RECEIVED_DATA as libc::c_int as libc::c_uint ||
           timeNowUs > timeOfLastHop.wrapping_add(hopTimeout) {
        h8_3dHopToNextChannel();
        timeOfLastHop = timeNowUs
    }
    return ret;
}
unsafe extern "C" fn h8_3dNrf24Setup(mut protocol: rx_spi_protocol_e,
                                     mut rxSpiId: *const uint32_t) {
    protocolState = STATE_BIND;
    NRF24L01_Initialize(0 as libc::c_int as uint8_t);
    NRF24L01_SetupBasic();
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP as libc::c_int as uint8_t,
                      (NRF24L01_06_RF_SETUP_RF_DR_1Mbps as libc::c_int |
                           NRF24L01_06_RF_SETUP_RF_PWR_n12dbm as libc::c_int)
                          as uint8_t);
    // RX_ADDR for pipes P1-P5 are left at default values
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0 as libc::c_int as
                                    uint8_t, rxTxAddrXN297.as_mut_ptr(),
                                5 as libc::c_int as
                                    uint8_t); // payload + 2 bytes CRC
    rxSpiIdPtr = rxSpiId as *mut uint32_t;
    if rxSpiId.is_null() || *rxSpiId == 0 as libc::c_int as libc::c_uint {
        h8_3dRfChannelIndex = 0x6 as libc::c_int as uint8_t;
        NRF24L01_SetChannel(0x6 as libc::c_int as uint8_t);
    } else { h8_3dSetBound(rxSpiId as *mut uint8_t); }
    payloadSize = 20 as libc::c_int as uint8_t;
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0 as libc::c_int as uint8_t,
                      (payloadSize as libc::c_int + 2 as libc::c_int) as
                          uint8_t);
    NRF24L01_SetRxMode();
    // enter receive mode to start listening for packets
}
#[no_mangle]
pub unsafe extern "C" fn h8_3dNrf24Init(mut rxSpiConfig: *const rxSpiConfig_t,
                                        mut rxRuntimeConfig:
                                            *mut rxRuntimeConfig_t) -> bool {
    (*rxRuntimeConfig).channelCount = 14 as libc::c_int as uint8_t;
    h8_3dNrf24Setup((*rxSpiConfig).rx_spi_protocol as rx_spi_protocol_e,
                    &(*rxSpiConfig).rx_spi_id);
    return 1 as libc::c_int != 0;
}
