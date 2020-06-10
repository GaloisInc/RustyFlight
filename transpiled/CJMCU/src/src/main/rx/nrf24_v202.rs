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
    // This file is copied with modifications from project Deviation,
// see http://deviationtx.com, file iface_nrf24l01.h
    // bit value
    // Register map of nRF24L01
    // Auto Acknowledge
    // Address Width
    // automatic RETRansmission
    // RF CHannel
    //Received Power Detector in the nRF23L01+, called CD (Carrier Detect) in the nRF24L01
    // Payload Width
    // DYNamic PayloaD
    // Bit position mnemonics
    // Pre-shifted and combined bits
    // Pipes
    #[no_mangle]
    fn NRF24L01_ReadPayload(data: *mut uint8_t, length: uint8_t) -> uint8_t;
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
    fn NRF24L01_SetRxMode();
    #[no_mangle]
    fn micros() -> timeUs_t;
    #[no_mangle]
    static mut rxSpiRcData: [uint16_t; 0];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
pub type C2RustUnnamed_0 = libc::c_uint;
pub const NRF24L01_1D_FEATURE: C2RustUnnamed_0 = 29;
pub const NRF24L01_1C_DYNPD: C2RustUnnamed_0 = 28;
pub const NRF24L01_17_FIFO_STATUS: C2RustUnnamed_0 = 23;
pub const NRF24L01_16_RX_PW_P5: C2RustUnnamed_0 = 22;
pub const NRF24L01_15_RX_PW_P4: C2RustUnnamed_0 = 21;
pub const NRF24L01_14_RX_PW_P3: C2RustUnnamed_0 = 20;
pub const NRF24L01_13_RX_PW_P2: C2RustUnnamed_0 = 19;
pub const NRF24L01_12_RX_PW_P1: C2RustUnnamed_0 = 18;
pub const NRF24L01_11_RX_PW_P0: C2RustUnnamed_0 = 17;
pub const NRF24L01_10_TX_ADDR: C2RustUnnamed_0 = 16;
pub const NRF24L01_0F_RX_ADDR_P5: C2RustUnnamed_0 = 15;
pub const NRF24L01_0E_RX_ADDR_P4: C2RustUnnamed_0 = 14;
pub const NRF24L01_0D_RX_ADDR_P3: C2RustUnnamed_0 = 13;
pub const NRF24L01_0C_RX_ADDR_P2: C2RustUnnamed_0 = 12;
pub const NRF24L01_0B_RX_ADDR_P1: C2RustUnnamed_0 = 11;
pub const NRF24L01_0A_RX_ADDR_P0: C2RustUnnamed_0 = 10;
pub const NRF24L01_09_RPD: C2RustUnnamed_0 = 9;
pub const NRF24L01_08_OBSERVE_TX: C2RustUnnamed_0 = 8;
pub const NRF24L01_07_STATUS: C2RustUnnamed_0 = 7;
pub const NRF24L01_06_RF_SETUP: C2RustUnnamed_0 = 6;
pub const NRF24L01_05_RF_CH: C2RustUnnamed_0 = 5;
pub const NRF24L01_04_SETUP_RETR: C2RustUnnamed_0 = 4;
pub const NRF24L01_03_SETUP_AW: C2RustUnnamed_0 = 3;
pub const NRF24L01_02_EN_RXADDR: C2RustUnnamed_0 = 2;
pub const NRF24L01_01_EN_AA: C2RustUnnamed_0 = 1;
pub const NRF24L01_00_CONFIG: C2RustUnnamed_0 = 0;
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
pub const PHASE_NOT_BOUND: C2RustUnnamed_4 = 0;
pub const V2X2_FLAG_MAG_CAL_Y: C2RustUnnamed_3 = 32;
pub const V2X2_FLAG_MAG_CAL_X: C2RustUnnamed_3 = 8;
// packet[10] flags
pub const V2X2_FLAG_HEADLESS: C2RustUnnamed_3 = 2;
// also automatic Missile Launcher and Hoist in one direction
pub const V2X2_FLAG_VIDEO: C2RustUnnamed_3 = 2;
// packet[14] flags
pub const V2X2_FLAG_CAMERA: C2RustUnnamed_3 = 1;
// also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
pub const V2X2_FLAG_FLIP: C2RustUnnamed_3 = 4;
pub const V2X2_FLAG_LED: C2RustUnnamed_3 = 16;
pub const V2X2_FLAG_BIND: C2RustUnnamed_3 = 192;
pub const PHASE_BOUND: C2RustUnnamed_4 = 1;
pub type C2RustUnnamed_3 = libc::c_uint;
pub const V2X2_FLAG_UNK10: C2RustUnnamed_3 = 32;
pub const V2X2_FLAG_UNK9: C2RustUnnamed_3 = 8;
pub type C2RustUnnamed_4 = libc::c_uint;
// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
static mut v2x2_freq_hopping: [[uint8_t; 16]; 4] =
    [[0x27 as libc::c_int as uint8_t, 0x1b as libc::c_int as uint8_t,
      0x39 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x24 as libc::c_int as uint8_t, 0x22 as libc::c_int as uint8_t,
      0x2e as libc::c_int as uint8_t, 0x36 as libc::c_int as uint8_t,
      0x19 as libc::c_int as uint8_t, 0x21 as libc::c_int as uint8_t,
      0x29 as libc::c_int as uint8_t, 0x14 as libc::c_int as uint8_t,
      0x1e as libc::c_int as uint8_t, 0x12 as libc::c_int as uint8_t,
      0x2d as libc::c_int as uint8_t, 0x18 as libc::c_int as uint8_t],
     [0x2e as libc::c_int as uint8_t, 0x33 as libc::c_int as uint8_t,
      0x25 as libc::c_int as uint8_t, 0x38 as libc::c_int as uint8_t,
      0x19 as libc::c_int as uint8_t, 0x12 as libc::c_int as uint8_t,
      0x18 as libc::c_int as uint8_t, 0x16 as libc::c_int as uint8_t,
      0x2a as libc::c_int as uint8_t, 0x1c as libc::c_int as uint8_t,
      0x1f as libc::c_int as uint8_t, 0x37 as libc::c_int as uint8_t,
      0x2f as libc::c_int as uint8_t, 0x23 as libc::c_int as uint8_t,
      0x34 as libc::c_int as uint8_t, 0x10 as libc::c_int as uint8_t],
     [0x11 as libc::c_int as uint8_t, 0x1a as libc::c_int as uint8_t,
      0x35 as libc::c_int as uint8_t, 0x24 as libc::c_int as uint8_t,
      0x28 as libc::c_int as uint8_t, 0x18 as libc::c_int as uint8_t,
      0x25 as libc::c_int as uint8_t, 0x2a as libc::c_int as uint8_t,
      0x32 as libc::c_int as uint8_t, 0x2c as libc::c_int as uint8_t,
      0x14 as libc::c_int as uint8_t, 0x27 as libc::c_int as uint8_t,
      0x36 as libc::c_int as uint8_t, 0x34 as libc::c_int as uint8_t,
      0x1c as libc::c_int as uint8_t, 0x17 as libc::c_int as uint8_t],
     [0x22 as libc::c_int as uint8_t, 0x27 as libc::c_int as uint8_t,
      0x17 as libc::c_int as uint8_t, 0x39 as libc::c_int as uint8_t,
      0x34 as libc::c_int as uint8_t, 0x28 as libc::c_int as uint8_t,
      0x2b as libc::c_int as uint8_t, 0x1d as libc::c_int as uint8_t,
      0x18 as libc::c_int as uint8_t, 0x2a as libc::c_int as uint8_t,
      0x21 as libc::c_int as uint8_t, 0x38 as libc::c_int as uint8_t,
      0x10 as libc::c_int as uint8_t, 0x26 as libc::c_int as uint8_t,
      0x20 as libc::c_int as uint8_t, 0x1f as libc::c_int as uint8_t]];
static mut rf_channels: [uint8_t; 16] = [0; 16];
static mut rf_ch_num: uint8_t = 0;
static mut bind_phase: uint8_t = 0;
static mut packet_timer: uint32_t = 0;
static mut txid: [uint8_t; 3] = [0; 3];
static mut rx_timeout: uint32_t = 0;
static mut v2x2_channelindex: [libc::c_uchar; 11] =
    [RC_SPI_THROTTLE as libc::c_int as libc::c_uchar,
     RC_SPI_YAW as libc::c_int as libc::c_uchar,
     RC_SPI_PITCH as libc::c_int as libc::c_uchar,
     RC_SPI_ROLL as libc::c_int as libc::c_uchar,
     RC_SPI_AUX1 as libc::c_int as libc::c_uchar,
     RC_SPI_AUX2 as libc::c_int as libc::c_uchar,
     RC_SPI_AUX3 as libc::c_int as libc::c_uchar,
     RC_SPI_AUX4 as libc::c_int as libc::c_uchar,
     RC_SPI_AUX5 as libc::c_int as libc::c_uchar,
     RC_SPI_AUX6 as libc::c_int as libc::c_uchar,
     RC_SPI_AUX7 as libc::c_int as libc::c_uchar];
unsafe extern "C" fn prepare_to_bind() {
    packet_timer = micros();
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 16 as libc::c_int {
        rf_channels[i as usize] =
            v2x2_freq_hopping[0 as libc::c_int as usize][i as usize];
        i += 1
    }
    rx_timeout = 1000 as libc::c_long as uint32_t;
}
unsafe extern "C" fn switch_channel() {
    NRF24L01_WriteReg(NRF24L01_05_RF_CH as libc::c_int as uint8_t,
                      rf_channels[rf_ch_num as usize]);
    rf_ch_num = rf_ch_num.wrapping_add(1);
    if rf_ch_num as libc::c_int >= 16 as libc::c_int {
        rf_ch_num = 0 as libc::c_int as uint8_t
    };
}
unsafe extern "C" fn v2x2_set_tx_id(mut id: *mut uint8_t) {
    let mut sum: uint8_t = 0;
    txid[0 as libc::c_int as usize] = *id.offset(0 as libc::c_int as isize);
    txid[1 as libc::c_int as usize] = *id.offset(1 as libc::c_int as isize);
    txid[2 as libc::c_int as usize] = *id.offset(2 as libc::c_int as isize);
    sum =
        (*id.offset(0 as libc::c_int as isize) as libc::c_int +
             *id.offset(1 as libc::c_int as isize) as libc::c_int +
             *id.offset(2 as libc::c_int as isize) as libc::c_int) as uint8_t;
    // Base row is defined by lowest 2 bits
    let mut fh_row: *const uint8_t =
        v2x2_freq_hopping[(sum as libc::c_int & 0x3 as libc::c_int) as
                              usize].as_ptr();
    // Higher 3 bits define increment to corresponding row
    let mut increment: uint8_t =
        ((sum as libc::c_int & 0x1e as libc::c_int) >> 2 as libc::c_int) as
            uint8_t;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 16 as libc::c_int {
        let mut val: uint8_t =
            (*fh_row.offset(i as isize) as libc::c_int +
                 increment as libc::c_int) as uint8_t;
        // Strange avoidance of channels divisible by 16
        rf_channels[i as usize] =
            if val as libc::c_int & 0xf as libc::c_int != 0 {
                val as libc::c_int
            } else { (val as libc::c_int) - 3 as libc::c_int } as uint8_t;
        i += 1
    };
}
unsafe extern "C" fn decode_bind_packet(mut packet: *mut uint8_t) {
    if *packet.offset(14 as libc::c_int as isize) as libc::c_int &
           V2X2_FLAG_BIND as libc::c_int == V2X2_FLAG_BIND as libc::c_int {
        // Fill out rf_channels with bound protocol parameters
        v2x2_set_tx_id(&mut *packet.offset(7 as libc::c_int as isize));
        bind_phase = PHASE_BOUND as libc::c_int as uint8_t;
        rx_timeout = 1000 as libc::c_long as uint32_t
        // find the channel as fast as possible
    };
}
// Returns whether the data was successfully decoded
unsafe extern "C" fn decode_packet(mut packet: *mut uint8_t)
 -> rx_spi_received_e {
    if bind_phase as libc::c_int != PHASE_BOUND as libc::c_int {
        decode_bind_packet(packet);
        return RX_SPI_RECEIVED_BIND
    }
    // Decode packet
    if *packet.offset(14 as libc::c_int as isize) as libc::c_int &
           V2X2_FLAG_BIND as libc::c_int == V2X2_FLAG_BIND as libc::c_int {
        return RX_SPI_RECEIVED_BIND
    }
    if *packet.offset(7 as libc::c_int as isize) as libc::c_int !=
           txid[0 as libc::c_int as usize] as libc::c_int ||
           *packet.offset(8 as libc::c_int as isize) as libc::c_int !=
               txid[1 as libc::c_int as usize] as libc::c_int ||
           *packet.offset(9 as libc::c_int as isize) as libc::c_int !=
               txid[2 as libc::c_int as usize] as libc::c_int {
        return RX_SPI_RECEIVED_NONE
    }
    // Restore regular interval
    rx_timeout =
        10000 as libc::c_long as
            uint32_t; // 4ms interval, duplicate packets, (8ms unique) + 25%
    // TREA order in packet to MultiWii order is handled by
    // correct assignment to channelindex
    // Throttle 0..255 to 1000..2000
    *rxSpiRcData.as_mut_ptr().offset(v2x2_channelindex[0 as libc::c_int as
                                                           usize] as isize) =
        (*packet.offset(0 as libc::c_int as isize) as uint16_t as libc::c_int
             * 1000 as libc::c_int / 255 as libc::c_int + 1000 as libc::c_int)
            as uint16_t; // two more unknown bits
    let mut i: libc::c_int = 1 as libc::c_int;
    while i < 4 as libc::c_int {
        let mut a: uint8_t = *packet.offset(i as isize);
        *rxSpiRcData.as_mut_ptr().offset(v2x2_channelindex[i as usize] as
                                             isize) =
            ((if (a as libc::c_int) < 0x80 as libc::c_int {
                  (0x7f as libc::c_int) - a as libc::c_int
              } else { a as libc::c_int }) as uint16_t as libc::c_int *
                 1000 as libc::c_int / 255 as libc::c_int +
                 1000 as libc::c_int) as uint16_t;
        i += 1
    }
    let flags: [uint8_t; 4] =
        [V2X2_FLAG_LED as libc::c_int as uint8_t,
         V2X2_FLAG_FLIP as libc::c_int as uint8_t,
         V2X2_FLAG_CAMERA as libc::c_int as uint8_t,
         V2X2_FLAG_VIDEO as libc::c_int as uint8_t];
    let mut i_0: libc::c_int = 4 as libc::c_int;
    while i_0 < 8 as libc::c_int {
        *rxSpiRcData.as_mut_ptr().offset(v2x2_channelindex[i_0 as usize] as
                                             isize) =
            if *packet.offset(14 as libc::c_int as isize) as libc::c_int &
                   flags[(i_0 - 4 as libc::c_int) as usize] as libc::c_int !=
                   0 {
                2000 as libc::c_int
            } else { 1000 as libc::c_int } as uint16_t;
        i_0 += 1
    }
    let flags10: [uint8_t; 3] =
        [V2X2_FLAG_HEADLESS as libc::c_int as uint8_t,
         V2X2_FLAG_MAG_CAL_X as libc::c_int as uint8_t,
         V2X2_FLAG_MAG_CAL_Y as libc::c_int as uint8_t];
    let mut i_1: libc::c_int = 8 as libc::c_int;
    while i_1 < 11 as libc::c_int {
        *rxSpiRcData.as_mut_ptr().offset(v2x2_channelindex[i_1 as usize] as
                                             isize) =
            if *packet.offset(10 as libc::c_int as isize) as libc::c_int &
                   flags10[(i_1 - 8 as libc::c_int) as usize] as libc::c_int
                   != 0 {
                2000 as libc::c_int
            } else { 1000 as libc::c_int } as uint16_t;
        i_1 += 1
    }
    packet_timer = micros();
    return RX_SPI_RECEIVED_DATA;
}
#[no_mangle]
pub unsafe extern "C" fn v202Nrf24SetRcDataFromPayload(mut rcData:
                                                           *mut uint16_t,
                                                       mut packet:
                                                           *const uint8_t) {
    // Ideally the decoding of the packet should be moved into here, to reduce the overhead of v202DataReceived function.
}
unsafe extern "C" fn readrx(mut packet: *mut uint8_t) -> rx_spi_received_e {
    if NRF24L01_ReadReg(NRF24L01_07_STATUS as libc::c_int as uint8_t) as
           libc::c_int &
           (1 as libc::c_int) << NRF24L01_07_STATUS_RX_DR as libc::c_int == 0
       {
        let mut t: uint32_t =
            micros().wrapping_sub(packet_timer); // clear the RX_DR flag
        if t > rx_timeout { switch_channel(); packet_timer = micros() }
        return RX_SPI_RECEIVED_NONE
    }
    packet_timer = micros();
    NRF24L01_WriteReg(NRF24L01_07_STATUS as libc::c_int as uint8_t,
                      ((1 as libc::c_int) <<
                           NRF24L01_07_STATUS_RX_DR as libc::c_int) as
                          uint8_t);
    NRF24L01_ReadPayload(packet, 16 as libc::c_int as uint8_t);
    NRF24L01_FlushRx();
    switch_channel();
    return decode_packet(packet);
}
/*
 * This is called periodically by the scheduler.
 * Returns RX_SPI_RECEIVED_DATA if a data packet was received.
 */
#[no_mangle]
pub unsafe extern "C" fn v202Nrf24DataReceived(mut packet: *mut uint8_t)
 -> rx_spi_received_e {
    return readrx(packet); // 2-bytes CRC
}
unsafe extern "C" fn v202Nrf24Setup(mut protocol: rx_spi_protocol_e) {
    NRF24L01_Initialize(((1 as libc::c_int) <<
                             NRF24L01_00_CONFIG_EN_CRC as libc::c_int |
                             (1 as libc::c_int) <<
                                 NRF24L01_00_CONFIG_CRCO as libc::c_int) as
                            uint8_t); // No Auto Acknowledgment
    NRF24L01_WriteReg(NRF24L01_01_EN_AA as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t); // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR as libc::c_int as uint8_t,
                      ((1 as libc::c_int) <<
                           NRF24L01_02_EN_RXADDR_ERX_P0 as libc::c_int) as
                          uint8_t); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW as libc::c_int as uint8_t,
                      NRF24L01_03_SETUP_AW_5BYTES as libc::c_int as
                          uint8_t); // 4ms retransmit t/o, 15 tries
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR as libc::c_int as uint8_t,
                      0xff as libc::c_int as
                          uint8_t); // Clear data ready, data sent, and retransmit
    if protocol as libc::c_uint ==
           RX_SPI_NRF24_V202_250K as libc::c_int as libc::c_uint {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP as libc::c_int as uint8_t,
                          (NRF24L01_06_RF_SETUP_RF_DR_250Kbps as libc::c_int |
                               NRF24L01_06_RF_SETUP_RF_PWR_n12dbm as
                                   libc::c_int) as
                              uint8_t); // bytes of data payload for pipe 0
    } else {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP as libc::c_int as uint8_t,
                          (NRF24L01_06_RF_SETUP_RF_DR_1Mbps as libc::c_int |
                               NRF24L01_06_RF_SETUP_RF_PWR_n12dbm as
                                   libc::c_int) as
                              uint8_t); // Just in case, no real bits to write here
    }
    NRF24L01_WriteReg(NRF24L01_07_STATUS as libc::c_int as uint8_t,
                      ((1 as libc::c_int) <<
                           NRF24L01_07_STATUS_RX_DR as libc::c_int |
                           (1 as libc::c_int) <<
                               NRF24L01_07_STATUS_TX_DS as libc::c_int |
                           (1 as libc::c_int) <<
                               NRF24L01_07_STATUS_MAX_RT as libc::c_int) as
                          uint8_t);
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0 as libc::c_int as uint8_t,
                      16 as libc::c_int as uint8_t);
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t);
    let rx_tx_addr: [uint8_t; 5] =
        [0x66 as libc::c_int as uint8_t, 0x88 as libc::c_int as uint8_t,
         0x68 as libc::c_int as uint8_t, 0x68 as libc::c_int as uint8_t,
         0x68 as libc::c_int as uint8_t];
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0 as libc::c_int as
                                    uint8_t, rx_tx_addr.as_ptr(),
                                5 as libc::c_int as uint8_t);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR as libc::c_int as uint8_t,
                                rx_tx_addr.as_ptr(),
                                5 as libc::c_int as uint8_t);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    rf_ch_num = 0 as libc::c_int as uint8_t;
    bind_phase = PHASE_NOT_BOUND as libc::c_int as uint8_t;
    prepare_to_bind();
    switch_channel();
    NRF24L01_SetRxMode();
    // enter receive mode to start listening for packets
}
#[no_mangle]
pub unsafe extern "C" fn v202Nrf24Init(mut rxSpiConfig: *const rxSpiConfig_t,
                                       mut rxRuntimeConfig:
                                           *mut rxRuntimeConfig_t) -> bool {
    (*rxRuntimeConfig).channelCount = 11 as libc::c_int as uint8_t;
    v202Nrf24Setup((*rxSpiConfig).rx_spi_protocol as rx_spi_protocol_e);
    return 1 as libc::c_int != 0;
}
