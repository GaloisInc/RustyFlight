use ::libc;
extern "C" {
    #[no_mangle]
    fn NRF24L01_Initialize(baseConfig: uint8_t);
    #[no_mangle]
    fn NRF24L01_WriteReg(reg: uint8_t, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn NRF24L01_WriteRegisterMulti(reg: uint8_t, data: *const uint8_t,
                                   length: uint8_t) -> uint8_t;
    #[no_mangle]
    fn NRF24L01_ReadReg(reg: uint8_t) -> uint8_t;
    // Utility functions
    #[no_mangle]
    fn NRF24L01_FlushTx();
    #[no_mangle]
    fn NRF24L01_FlushRx();
    #[no_mangle]
    fn NRF24L01_SetupBasic();
    #[no_mangle]
    fn NRF24L01_SetRxMode();
    #[no_mangle]
    fn NRF24L01_SetTxMode();
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
                               rxAddr_0: *const uint8_t) -> uint16_t;
    #[no_mangle]
    fn XN297_WritePayload(data: *mut uint8_t, len: libc::c_int,
                          rxAddr_0: *const uint8_t) -> uint8_t;
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
    fn micros() -> timeUs_t;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// microsecond time
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
// Bit position mnemonics
pub type C2RustUnnamed_1 = libc::c_uint;
pub const NRF24L01_1D_FEATURE_EN_DYN_ACK: C2RustUnnamed_1 = 0;
pub const NRF24L01_1D_FEATURE_EN_ACK_PAY: C2RustUnnamed_1 = 1;
pub const NRF24L01_1D_FEATURE_EN_DPL: C2RustUnnamed_1 = 2;
pub const NRF24L01_1C_DYNPD_DPL_P0: C2RustUnnamed_1 = 0;
pub const NRF24L01_1C_DYNPD_DPL_P1: C2RustUnnamed_1 = 1;
pub const NRF24L01_1C_DYNPD_DPL_P2: C2RustUnnamed_1 = 2;
pub const NRF24L01_1C_DYNPD_DPL_P3: C2RustUnnamed_1 = 3;
pub const NRF24L01_1C_DYNPD_DPL_P4: C2RustUnnamed_1 = 4;
pub const NRF24L01_1C_DYNPD_DPL_P5: C2RustUnnamed_1 = 5;
pub const NRF24L01_17_FIFO_STATUS_RX_EMPTY: C2RustUnnamed_1 = 0;
pub const NRF24L01_17_FIFO_STATUS_RX_FULL: C2RustUnnamed_1 = 1;
pub const NRF24L01_17_FIFO_STATUS_TX_EMPTY: C2RustUnnamed_1 = 4;
pub const NRF24L01_17_FIFO_STATUS_TX_FULL: C2RustUnnamed_1 = 5;
pub const NRF24L01_07_STATUS_MAX_RT: C2RustUnnamed_1 = 4;
pub const NRF24L01_07_STATUS_TX_DS: C2RustUnnamed_1 = 5;
pub const NRF24L01_07_STATUS_RX_DR: C2RustUnnamed_1 = 6;
pub const NRF24L01_06_RF_SETUP_RF_PWR_LOW: C2RustUnnamed_1 = 1;
pub const NRF24L01_06_RF_SETUP_RF_PWR_HIGH: C2RustUnnamed_1 = 2;
pub const NRF24L01_06_RF_SETUP_RF_DR_HIGH: C2RustUnnamed_1 = 3;
pub const NRF24L01_06_RF_SETUP_RF_DR_LOW: C2RustUnnamed_1 = 5;
pub const NRF24L01_02_EN_RXADDR_ERX_P0: C2RustUnnamed_1 = 0;
pub const NRF24L01_02_EN_RXADDR_ERX_P1: C2RustUnnamed_1 = 1;
pub const NRF24L01_02_EN_RXADDR_ERX_P2: C2RustUnnamed_1 = 2;
pub const NRF24L01_02_EN_RXADDR_ERX_P3: C2RustUnnamed_1 = 3;
pub const NRF24L01_02_EN_RXADDR_ERX_P4: C2RustUnnamed_1 = 4;
pub const NRF24L01_02_EN_RXADDR_ERX_P5: C2RustUnnamed_1 = 5;
pub const NRF24L01_01_EN_AA_ENAA_P0: C2RustUnnamed_1 = 0;
pub const NRF24L01_01_EN_AA_ENAA_P1: C2RustUnnamed_1 = 1;
pub const NRF24L01_01_EN_AA_ENAA_P2: C2RustUnnamed_1 = 2;
pub const NRF24L01_01_EN_AA_ENAA_P3: C2RustUnnamed_1 = 3;
pub const NRF24L01_01_EN_AA_ENAA_P4: C2RustUnnamed_1 = 4;
pub const NRF24L01_01_EN_AA_ENAA_P5: C2RustUnnamed_1 = 5;
pub const NRF24L01_00_CONFIG_PRIM_RX: C2RustUnnamed_1 = 0;
pub const NRF24L01_00_CONFIG_PWR_UP: C2RustUnnamed_1 = 1;
pub const NRF24L01_00_CONFIG_CRCO: C2RustUnnamed_1 = 2;
pub const NRF24L01_00_CONFIG_EN_CRC: C2RustUnnamed_1 = 3;
pub const NRF24L01_00_CONFIG_MASK_MAX_RT: C2RustUnnamed_1 = 4;
pub const NRF24L01_00_CONFIG_MASK_TX_DS: C2RustUnnamed_1 = 5;
pub const NRF24L01_00_CONFIG_MASK_RX_DR: C2RustUnnamed_1 = 6;
// Pre-shifted and combined bits
pub type C2RustUnnamed_2 = libc::c_uint;
pub const NRF24L01_1C_DYNPD_ALL_PIPES: C2RustUnnamed_2 = 63;
pub const NRF24L01_06_RF_SETUP_RF_PWR_0dbm: C2RustUnnamed_2 = 6;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n6dbm: C2RustUnnamed_2 = 4;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n12dbm: C2RustUnnamed_2 = 2;
pub const NRF24L01_06_RF_SETUP_RF_PWR_n18dbm: C2RustUnnamed_2 = 1;
pub const NRF24L01_06_RF_SETUP_RF_DR_250Kbps: C2RustUnnamed_2 = 32;
pub const NRF24L01_06_RF_SETUP_RF_DR_1Mbps: C2RustUnnamed_2 = 0;
pub const NRF24L01_06_RF_SETUP_RF_DR_2Mbps: C2RustUnnamed_2 = 8;
pub const NRF24L01_04_SETUP_RETR_ARC_15: C2RustUnnamed_2 = 15;
pub const NRF24L01_04_SETUP_RETR_ARC_14: C2RustUnnamed_2 = 14;
pub const NRF24L01_04_SETUP_RETR_ARC_13: C2RustUnnamed_2 = 13;
pub const NRF24L01_04_SETUP_RETR_ARC_12: C2RustUnnamed_2 = 12;
pub const NRF24L01_04_SETUP_RETR_ARC_11: C2RustUnnamed_2 = 11;
pub const NRF24L01_04_SETUP_RETR_ARC_10: C2RustUnnamed_2 = 10;
pub const NRF24L01_04_SETUP_RETR_ARC_9: C2RustUnnamed_2 = 9;
pub const NRF24L01_04_SETUP_RETR_ARC_8: C2RustUnnamed_2 = 8;
pub const NRF24L01_04_SETUP_RETR_ARC_7: C2RustUnnamed_2 = 7;
pub const NRF24L01_04_SETUP_RETR_ARC_6: C2RustUnnamed_2 = 6;
pub const NRF24L01_04_SETUP_RETR_ARC_5: C2RustUnnamed_2 = 5;
pub const NRF24L01_04_SETUP_RETR_ARC_4: C2RustUnnamed_2 = 4;
pub const NRF24L01_04_SETUP_RETR_ARC_3: C2RustUnnamed_2 = 3;
pub const NRF24L01_04_SETUP_RETR_ARC_2: C2RustUnnamed_2 = 2;
pub const NRF24L01_04_SETUP_RETR_ARC_1: C2RustUnnamed_2 = 1;
pub const NRF24L01_04_SETUP_RETR_ARC_0: C2RustUnnamed_2 = 0;
pub const NRF24L01_04_SETUP_RETR_ARD_4000us: C2RustUnnamed_2 = 240;
pub const NRF24L01_04_SETUP_RETR_ARD_3750us: C2RustUnnamed_2 = 224;
pub const NRF24L01_04_SETUP_RETR_ARD_3500us: C2RustUnnamed_2 = 208;
pub const NRF24L01_04_SETUP_RETR_ARD_3250us: C2RustUnnamed_2 = 192;
pub const NRF24L01_04_SETUP_RETR_ARD_3000us: C2RustUnnamed_2 = 176;
pub const NRF24L01_04_SETUP_RETR_ARD_2750us: C2RustUnnamed_2 = 160;
pub const NRF24L01_04_SETUP_RETR_ARD_2500us: C2RustUnnamed_2 = 144;
pub const NRF24L01_04_SETUP_RETR_ARD_2250us: C2RustUnnamed_2 = 128;
pub const NRF24L01_04_SETUP_RETR_ARD_2000us: C2RustUnnamed_2 = 112;
pub const NRF24L01_04_SETUP_RETR_ARD_1750us: C2RustUnnamed_2 = 96;
pub const NRF24L01_04_SETUP_RETR_ARD_1500us: C2RustUnnamed_2 = 80;
pub const NRF24L01_04_SETUP_RETR_ARD_1250us: C2RustUnnamed_2 = 64;
pub const NRF24L01_04_SETUP_RETR_ARD_1000us: C2RustUnnamed_2 = 48;
pub const NRF24L01_04_SETUP_RETR_ARD_750us: C2RustUnnamed_2 = 32;
pub const NRF24L01_04_SETUP_RETR_ARD_500us: C2RustUnnamed_2 = 16;
pub const NRF24L01_04_SETUP_RETR_ARD_250us: C2RustUnnamed_2 = 0;
pub const NRF24L01_03_SETUP_AW_5BYTES: C2RustUnnamed_2 = 3;
pub const NRF24L01_03_SETUP_AW_4BYTES: C2RustUnnamed_2 = 2;
pub const NRF24L01_03_SETUP_AW_3BYTES: C2RustUnnamed_2 = 1;
pub const NRF24L01_02_EN_RXADDR_ERX_ALL_PIPES: C2RustUnnamed_2 = 63;
pub const NRF24L01_01_EN_AA_ALL_PIPES: C2RustUnnamed_2 = 63;
pub type protocol_state_t = libc::c_uint;
pub const STATE_DATA: protocol_state_t = 2;
pub const STATE_ACK: protocol_state_t = 1;
pub const STATE_BIND: protocol_state_t = 0;
pub const RATE_MID: C2RustUnnamed_3 = 1;
pub const RATE_LOW: C2RustUnnamed_3 = 0;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const RATE_HIGH: C2RustUnnamed_3 = 2;
static mut cx10Protocol: rx_spi_protocol_e = RX_SPI_NRF24_V202_250K;
static mut protocolState: protocol_state_t = STATE_BIND;
static mut payloadSize: uint8_t = 0;
static mut txAddr: [uint8_t; 5] =
    [0x55 as libc::c_int as uint8_t, 0xf as libc::c_int as uint8_t,
     0x71 as libc::c_int as uint8_t, 0xc as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t];
// converted XN297 address, 0xC710F55 (28 bit)
static mut rxAddr: [uint8_t; 5] =
    [0x49 as libc::c_int as uint8_t, 0x26 as libc::c_int as uint8_t,
     0x87 as libc::c_int as uint8_t, 0x7d as libc::c_int as uint8_t,
     0x2f as libc::c_int as uint8_t];
// converted XN297 address
static mut txId: [uint8_t; 4] = [0; 4];
static mut cx10RfChannelIndex: uint8_t = 0 as libc::c_int as uint8_t;
static mut cx10RfChannels: [uint8_t; 4] = [0; 4];
// 6.5ms
static mut hopTimeout: uint32_t = 0;
static mut timeOfLastHop: uint32_t = 0;
/*
 * Returns true if it is a bind packet.
 */
unsafe extern "C" fn cx10CheckBindPacket(mut packet: *const uint8_t) -> bool {
    let bindPacket: bool =
        *packet.offset(0 as libc::c_int as isize) as libc::c_int ==
            0xaa as libc::c_int; // 10101010
    if bindPacket {
        txId[0 as libc::c_int as usize] =
            *packet.offset(1 as libc::c_int as
                               isize); // mask out top bit which is used for a flag for the rudder
        txId[1 as libc::c_int as usize] =
            *packet.offset(2 as libc::c_int as isize); // aileron
        txId[2 as libc::c_int as usize] =
            *packet.offset(3 as libc::c_int as isize); // elevator
        txId[3 as libc::c_int as usize] =
            *packet.offset(4 as libc::c_int as isize); // throttle
        return 1 as libc::c_int != 0
    } // rudder
    return 0 as libc::c_int != 0; // takes values 0, 1, 2
}
unsafe extern "C" fn cx10ConvertToPwmUnsigned(mut pVal: *const uint8_t)
 -> uint16_t {
    let mut ret: uint16_t =
        (*pVal.offset(1 as libc::c_int as isize) as libc::c_int &
             0x7f as libc::c_int) as uint16_t;
    ret =
        ((ret as libc::c_int) << 8 as libc::c_int | *pVal as libc::c_int) as
            uint16_t;
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn cx10Nrf24SetRcDataFromPayload(mut rcData:
                                                           *mut uint16_t,
                                                       mut payload:
                                                           *const uint8_t) {
    let offset: uint8_t =
        if cx10Protocol as libc::c_uint ==
               RX_SPI_NRF24_CX10 as libc::c_int as libc::c_uint {
            0 as libc::c_int
        } else { 4 as libc::c_int } as uint8_t;
    *rcData.offset(RC_SPI_ROLL as libc::c_int as isize) =
        (2000 as libc::c_int + 1000 as libc::c_int -
             cx10ConvertToPwmUnsigned(&*payload.offset((5 as libc::c_int +
                                                            offset as
                                                                libc::c_int)
                                                           as isize)) as
                 libc::c_int) as uint16_t;
    *rcData.offset(RC_SPI_PITCH as libc::c_int as isize) =
        (2000 as libc::c_int + 1000 as libc::c_int -
             cx10ConvertToPwmUnsigned(&*payload.offset((7 as libc::c_int +
                                                            offset as
                                                                libc::c_int)
                                                           as isize)) as
                 libc::c_int) as uint16_t;
    *rcData.offset(RC_SPI_THROTTLE as libc::c_int as isize) =
        cx10ConvertToPwmUnsigned(&*payload.offset((9 as libc::c_int +
                                                       offset as libc::c_int)
                                                      as isize));
    *rcData.offset(RC_SPI_YAW as libc::c_int as isize) =
        cx10ConvertToPwmUnsigned(&*payload.offset((11 as libc::c_int +
                                                       offset as libc::c_int)
                                                      as isize));
    let flags1: uint8_t =
        *payload.offset((13 as libc::c_int + offset as libc::c_int) as isize);
    let rate: uint8_t =
        (flags1 as libc::c_int & 0x3 as libc::c_int) as uint8_t;
    if rate as libc::c_int == RATE_LOW as libc::c_int {
        *rcData.offset(RC_SPI_AUX1 as libc::c_int as isize) =
            1000 as libc::c_int as uint16_t
    } else if rate as libc::c_int == RATE_MID as libc::c_int {
        *rcData.offset(RC_SPI_AUX1 as libc::c_int as isize) =
            (1000 as libc::c_int +
                 (2000 as libc::c_int - 1000 as libc::c_int) /
                     2 as libc::c_int) as uint16_t
    } else {
        *rcData.offset(RC_SPI_AUX1 as libc::c_int as isize) =
            2000 as libc::c_int as uint16_t
    }
    // flip flag is in YAW byte
    *rcData.offset(RC_SPI_AUX2 as libc::c_int as isize) =
        if *payload.offset((12 as libc::c_int + offset as libc::c_int) as
                               isize) as libc::c_int & 0x10 as libc::c_int !=
               0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    let flags2: uint8_t =
        *payload.offset((14 as libc::c_int + offset as libc::c_int) as isize);
    *rcData.offset(RC_SPI_AUX3 as libc::c_int as isize) =
        if flags2 as libc::c_int & 0x4 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    *rcData.offset(RC_SPI_AUX4 as libc::c_int as isize) =
        if flags2 as libc::c_int & 0x2 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
    *rcData.offset(RC_SPI_AUX5 as libc::c_int as isize) =
        if flags1 as libc::c_int & 0x4 as libc::c_int != 0 {
            2000 as libc::c_int
        } else { 1000 as libc::c_int } as uint16_t;
}
unsafe extern "C" fn cx10HopToNextChannel() {
    cx10RfChannelIndex = cx10RfChannelIndex.wrapping_add(1);
    if cx10RfChannelIndex as libc::c_int >= 4 as libc::c_int {
        cx10RfChannelIndex = 0 as libc::c_int as uint8_t
    }
    NRF24L01_SetChannel(cx10RfChannels[cx10RfChannelIndex as usize]);
}
// The hopping channels are determined by the txId
unsafe extern "C" fn cx10SetHoppingChannels(mut txId_0: *const uint8_t) {
    cx10RfChannelIndex = 0 as libc::c_int as uint8_t;
    cx10RfChannels[0 as libc::c_int as usize] =
        (0x3 as libc::c_int +
             (*txId_0.offset(0 as libc::c_int as isize) as libc::c_int &
                  0xf as libc::c_int)) as uint8_t;
    cx10RfChannels[1 as libc::c_int as usize] =
        (0x16 as libc::c_int +
             (*txId_0.offset(0 as libc::c_int as isize) as libc::c_int >>
                  4 as libc::c_int)) as uint8_t;
    cx10RfChannels[2 as libc::c_int as usize] =
        (0x2d as libc::c_int +
             (*txId_0.offset(1 as libc::c_int as isize) as libc::c_int &
                  0xf as libc::c_int)) as uint8_t;
    cx10RfChannels[3 as libc::c_int as usize] =
        (0x40 as libc::c_int +
             (*txId_0.offset(1 as libc::c_int as isize) as libc::c_int >>
                  4 as libc::c_int)) as uint8_t;
}
unsafe extern "C" fn cx10CrcOK(mut crc: uint16_t, mut payload: *const uint8_t)
 -> bool {
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
unsafe extern "C" fn cx10ReadPayloadIfAvailable(mut payload: *mut uint8_t)
 -> bool {
    if NRF24L01_ReadPayloadIfAvailable(payload,
                                       (payloadSize as libc::c_int +
                                            2 as libc::c_int) as uint8_t) {
        let crc: uint16_t =
            XN297_UnscramblePayload(payload, payloadSize as libc::c_int,
                                    rxAddr.as_mut_ptr());
        if cx10CrcOK(crc, payload) { return 1 as libc::c_int != 0 }
    }
    return 0 as libc::c_int != 0;
}
/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
#[no_mangle]
pub unsafe extern "C" fn cx10Nrf24DataReceived(mut payload: *mut uint8_t)
 -> rx_spi_received_e {
    static mut ackCount: uint8_t = 0;
    let mut ret: rx_spi_received_e = RX_SPI_RECEIVED_NONE;
    let mut totalDelayUs: libc::c_int = 0;
    let mut timeNowUs: uint32_t = 0;
    static mut fifoDelayUs: libc::c_int = 100 as libc::c_int;
    static mut delayBetweenPacketsUs: libc::c_int = 1000 as libc::c_int;
    match protocolState as libc::c_uint {
        0 => {
            if cx10ReadPayloadIfAvailable(payload) {
                let bindPacket: bool = cx10CheckBindPacket(payload);
                if bindPacket {
                    // set the hopping channels as determined by the txId received in the bind packet
                    cx10SetHoppingChannels(txId.as_mut_ptr());
                    ret = RX_SPI_RECEIVED_BIND;
                    protocolState = STATE_ACK;
                    ackCount = 0 as libc::c_int as uint8_t
                }
            }
        }
        1 => {
            // transmit an ACK packet
            ackCount = ackCount.wrapping_add(1);
            totalDelayUs = 0 as libc::c_int;
            // send out an ACK on the bind channel, required by deviationTx
            *payload.offset(9 as libc::c_int as isize) =
                0x1 as libc::c_int as
                    uint8_t; // enter transmit mode to send the packet
            NRF24L01_SetChannel(0x2 as libc::c_int as uint8_t);
            NRF24L01_FlushTx();
            XN297_WritePayload(payload, payloadSize as libc::c_int,
                               rxAddr.as_mut_ptr());
            NRF24L01_SetTxMode();
            // wait for the ACK packet to send before changing channel
            while NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS as libc::c_int as
                                       uint8_t) as libc::c_int &
                      (1 as libc::c_int) <<
                          NRF24L01_17_FIFO_STATUS_TX_EMPTY as libc::c_int == 0
                  {
                delayMicroseconds(fifoDelayUs as timeUs_t);
                totalDelayUs += fifoDelayUs
            }
            // send out an ACK on each of the hopping channels, required by CX10 transmitter
            let mut ii: libc::c_int =
                0 as libc::c_int; // enter transmit mode to send the packet
            while ii < 4 as libc::c_int {
                NRF24L01_SetChannel(cx10RfChannels[ii as usize]);
                XN297_WritePayload(payload, payloadSize as libc::c_int,
                                   rxAddr.as_mut_ptr());
                NRF24L01_SetTxMode();
                // wait for the ACK packet to send before changing channel
                while NRF24L01_ReadReg(NRF24L01_17_FIFO_STATUS as libc::c_int
                                           as uint8_t) as libc::c_int &
                          (1 as libc::c_int) <<
                              NRF24L01_17_FIFO_STATUS_TX_EMPTY as libc::c_int
                          == 0 {
                    delayMicroseconds(fifoDelayUs as
                                          timeUs_t); //reenter receive mode after sending ACKs
                    totalDelayUs += fifoDelayUs
                }
                ii += 1
            }
            if totalDelayUs < delayBetweenPacketsUs {
                delayMicroseconds((delayBetweenPacketsUs - totalDelayUs) as
                                      timeUs_t);
            }
            NRF24L01_SetRxMode();
            if ackCount as libc::c_int > 8 as libc::c_int {
                NRF24L01_SetChannel(cx10RfChannels[0 as libc::c_int as
                                                       usize]);
                // and go into data state to wait for first data packet
                protocolState = STATE_DATA
            }
        }
        2 => {
            timeNowUs = micros();
            // read the payload, processing of payload is deferred
            if cx10ReadPayloadIfAvailable(payload) {
                cx10HopToNextChannel(); // sets PWR_UP, no CRC
                timeOfLastHop = timeNowUs;
                ret = RX_SPI_RECEIVED_DATA
            }
            if timeNowUs > timeOfLastHop.wrapping_add(hopTimeout) {
                cx10HopToNextChannel();
                timeOfLastHop = timeNowUs
            }
        }
        _ => { }
    }
    return ret;
}
unsafe extern "C" fn cx10Nrf24Setup(mut protocol: rx_spi_protocol_e) {
    cx10Protocol = protocol;
    protocolState = STATE_BIND;
    payloadSize =
        if protocol as libc::c_uint ==
               RX_SPI_NRF24_CX10 as libc::c_int as libc::c_uint {
            15 as libc::c_int
        } else { 19 as libc::c_int } as uint8_t;
    hopTimeout =
        if protocol as libc::c_uint ==
               RX_SPI_NRF24_CX10 as libc::c_int as libc::c_uint {
            1500 as libc::c_int
        } else { 6500 as libc::c_int } as uint32_t;
    NRF24L01_Initialize(0 as libc::c_int as uint8_t);
    NRF24L01_SetupBasic();
    NRF24L01_SetChannel(0x2 as libc::c_int as uint8_t);
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP as libc::c_int as uint8_t,
                      (NRF24L01_06_RF_SETUP_RF_DR_1Mbps as libc::c_int |
                           NRF24L01_06_RF_SETUP_RF_PWR_n12dbm as libc::c_int)
                          as uint8_t);
    // RX_ADDR for pipes P2 to P5 are left at default values
    NRF24L01_FlushRx(); // payload + 2 bytes CRC
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR as libc::c_int as uint8_t,
                                txAddr.as_mut_ptr(),
                                5 as libc::c_int as uint8_t);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0 as libc::c_int as
                                    uint8_t, rxAddr.as_mut_ptr(),
                                5 as libc::c_int as uint8_t);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0 as libc::c_int as uint8_t,
                      (payloadSize as libc::c_int + 2 as libc::c_int) as
                          uint8_t);
    NRF24L01_SetRxMode();
    // enter receive mode to start listening for packets
}
#[no_mangle]
pub unsafe extern "C" fn cx10Nrf24Init(mut rxSpiConfig: *const rxSpiConfig_t,
                                       mut rxRuntimeConfig:
                                           *mut rxRuntimeConfig_t) -> bool {
    (*rxRuntimeConfig).channelCount = 9 as libc::c_int as uint8_t;
    cx10Nrf24Setup((*rxSpiConfig).rx_spi_protocol as rx_spi_protocol_e);
    return 1 as libc::c_int != 0;
}
