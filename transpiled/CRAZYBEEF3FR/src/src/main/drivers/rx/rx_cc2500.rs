use ::libc;
extern "C" {
    #[no_mangle]
    fn rxSpiWriteByte(data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteCommand(command: uint8_t, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiWriteCommandMulti(command: uint8_t, data: *const uint8_t,
                              length: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiReadCommand(command: uint8_t, commandData: uint8_t) -> uint8_t;
    #[no_mangle]
    fn rxSpiReadCommandMulti(command: uint8_t, commandData: uint8_t,
                             retData: *mut uint8_t, length: uint8_t)
     -> uint8_t;
    #[no_mangle]
    fn delayMicroseconds(us: timeUs_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
// microsecond time
pub type timeUs_t = uint32_t;
pub type C2RustUnnamed = libc::c_uint;
pub const CC2500_3F_RXFIFO: C2RustUnnamed = 63;
pub const CC2500_3F_TXFIFO: C2RustUnnamed = 63;
pub const CC2500_3E_PATABLE: C2RustUnnamed = 62;
pub const CC2500_3B_RXBYTES: C2RustUnnamed = 59;
pub const CC2500_3A_TXBYTES: C2RustUnnamed = 58;
pub const CC2500_39_VCO_VC_DAC: C2RustUnnamed = 57;
pub const CC2500_38_PKTSTATUS: C2RustUnnamed = 56;
pub const CC2500_37_WORTIME0: C2RustUnnamed = 55;
pub const CC2500_36_WORTIME1: C2RustUnnamed = 54;
pub const CC2500_35_MARCSTATE: C2RustUnnamed = 53;
pub const CC2500_34_RSSI: C2RustUnnamed = 52;
pub const CC2500_33_LQI: C2RustUnnamed = 51;
pub const CC2500_32_FREQEST: C2RustUnnamed = 50;
pub const CC2500_31_VERSION: C2RustUnnamed = 49;
pub const CC2500_30_PARTNUM: C2RustUnnamed = 48;
pub const CC2500_2E_TEST0: C2RustUnnamed = 46;
pub const CC2500_2D_TEST1: C2RustUnnamed = 45;
pub const CC2500_2C_TEST2: C2RustUnnamed = 44;
pub const CC2500_2B_AGCTEST: C2RustUnnamed = 43;
pub const CC2500_2A_PTEST: C2RustUnnamed = 42;
pub const CC2500_29_FSTEST: C2RustUnnamed = 41;
pub const CC2500_28_RCCTRL0: C2RustUnnamed = 40;
pub const CC2500_27_RCCTRL1: C2RustUnnamed = 39;
pub const CC2500_26_FSCAL0: C2RustUnnamed = 38;
pub const CC2500_25_FSCAL1: C2RustUnnamed = 37;
pub const CC2500_24_FSCAL2: C2RustUnnamed = 36;
pub const CC2500_23_FSCAL3: C2RustUnnamed = 35;
pub const CC2500_22_FREND0: C2RustUnnamed = 34;
pub const CC2500_21_FREND1: C2RustUnnamed = 33;
pub const CC2500_20_WORCTRL: C2RustUnnamed = 32;
pub const CC2500_1F_WOREVT0: C2RustUnnamed = 31;
pub const CC2500_1E_WOREVT1: C2RustUnnamed = 30;
pub const CC2500_1D_AGCCTRL0: C2RustUnnamed = 29;
pub const CC2500_1C_AGCCTRL1: C2RustUnnamed = 28;
pub const CC2500_1B_AGCCTRL2: C2RustUnnamed = 27;
pub const CC2500_1A_BSCFG: C2RustUnnamed = 26;
pub const CC2500_19_FOCCFG: C2RustUnnamed = 25;
pub const CC2500_18_MCSM0: C2RustUnnamed = 24;
pub const CC2500_17_MCSM1: C2RustUnnamed = 23;
pub const CC2500_16_MCSM2: C2RustUnnamed = 22;
pub const CC2500_15_DEVIATN: C2RustUnnamed = 21;
pub const CC2500_14_MDMCFG0: C2RustUnnamed = 20;
pub const CC2500_13_MDMCFG1: C2RustUnnamed = 19;
pub const CC2500_12_MDMCFG2: C2RustUnnamed = 18;
pub const CC2500_11_MDMCFG3: C2RustUnnamed = 17;
pub const CC2500_10_MDMCFG4: C2RustUnnamed = 16;
pub const CC2500_0F_FREQ0: C2RustUnnamed = 15;
pub const CC2500_0E_FREQ1: C2RustUnnamed = 14;
pub const CC2500_0D_FREQ2: C2RustUnnamed = 13;
pub const CC2500_0C_FSCTRL0: C2RustUnnamed = 12;
pub const CC2500_0B_FSCTRL1: C2RustUnnamed = 11;
pub const CC2500_0A_CHANNR: C2RustUnnamed = 10;
pub const CC2500_09_ADDR: C2RustUnnamed = 9;
pub const CC2500_08_PKTCTRL0: C2RustUnnamed = 8;
pub const CC2500_07_PKTCTRL1: C2RustUnnamed = 7;
pub const CC2500_06_PKTLEN: C2RustUnnamed = 6;
pub const CC2500_05_SYNC0: C2RustUnnamed = 5;
pub const CC2500_04_SYNC1: C2RustUnnamed = 4;
pub const CC2500_03_FIFOTHR: C2RustUnnamed = 3;
pub const CC2500_02_IOCFG0: C2RustUnnamed = 2;
pub const CC2500_01_IOCFG1: C2RustUnnamed = 1;
pub const CC2500_00_IOCFG2: C2RustUnnamed = 0;
#[no_mangle]
pub unsafe extern "C" fn cc2500ReadFifo(mut dpbuffer: *mut uint8_t,
                                        mut len: uint8_t) -> uint8_t {
    return rxSpiReadCommandMulti((CC2500_3F_RXFIFO as libc::c_int |
                                      0xc0 as libc::c_int) as uint8_t,
                                 0xff as libc::c_int as uint8_t, dpbuffer,
                                 len); // 0x3B SFTX
}
#[no_mangle]
pub unsafe extern "C" fn cc2500WriteFifo(mut dpbuffer: *mut uint8_t,
                                         mut len: uint8_t) -> uint8_t {
    let mut ret: uint8_t = 0; // 0x35
    cc2500Strobe(0x3b as libc::c_int as uint8_t);
    ret =
        rxSpiWriteCommandMulti((CC2500_3F_TXFIFO as libc::c_int |
                                    0x40 as libc::c_int) as uint8_t, dpbuffer,
                               len);
    cc2500Strobe(0x35 as libc::c_int as uint8_t);
    return ret;
}
#[no_mangle]
pub unsafe extern "C" fn cc2500ReadRegisterMulti(mut address: uint8_t,
                                                 mut data: *mut uint8_t,
                                                 mut length: uint8_t)
 -> uint8_t {
    return rxSpiReadCommandMulti(address, 0xff as libc::c_int as uint8_t,
                                 data, length);
}
#[no_mangle]
pub unsafe extern "C" fn cc2500WriteRegisterMulti(mut address: uint8_t,
                                                  mut data: *mut uint8_t,
                                                  mut length: uint8_t)
 -> uint8_t {
    return rxSpiWriteCommandMulti(address, data, length);
}
#[no_mangle]
pub unsafe extern "C" fn cc2500ReadReg(mut reg: uint8_t) -> uint8_t {
    return rxSpiReadCommand((reg as libc::c_int | 0x80 as libc::c_int) as
                                uint8_t, 0xff as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn cc2500Strobe(mut address: uint8_t) {
    rxSpiWriteByte(address);
}
#[no_mangle]
pub unsafe extern "C" fn cc2500WriteReg(mut address: uint8_t,
                                        mut data: uint8_t) -> uint8_t {
    return rxSpiWriteCommand(address, data);
}
#[no_mangle]
pub unsafe extern "C" fn cc2500SetPower(mut power: uint8_t) {
    let patable: [uint8_t; 8] =
        [0xc5 as libc::c_int as uint8_t, 0x97 as libc::c_int as uint8_t,
         0x6e as libc::c_int as uint8_t, 0x7f as libc::c_int as uint8_t,
         0xa9 as libc::c_int as uint8_t, 0xbb as libc::c_int as uint8_t,
         0xfe as libc::c_int as uint8_t, 0xff as libc::c_int as uint8_t];
    if power as libc::c_int > 7 as libc::c_int {
        power = 7 as libc::c_int as uint8_t
    }
    cc2500WriteReg(CC2500_3E_PATABLE as libc::c_int as uint8_t,
                   patable[power as usize]);
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
/*
 CC2500 SPI drivers
*/
// GDO2 output pin configuration
// GDO1 output pin configuration
// GDO0 output pin configuration
// RX FIFO and TX FIFO thresholds
// Sync word, high byte
// Sync word, low byte
// Packet length
// Packet automation control
// Packet automation control
// Device address
// Channel number
// Frequency synthesizer control
// Frequency synthesizer control
// Frequency control word, high byte
// Frequency control word, middle byte
// Frequency control word, low byte
// Modem configuration
// Modem configuration
// Modem configuration
// Modem configuration
// Modem configuration
// Modem deviation setting
// Main Radio Cntrl State Machine config
// Main Radio Cntrl State Machine config
// Main Radio Cntrl State Machine config
// Frequency Offset Compensation config
// Bit Synchronization configuration
// AGC control
// AGC control
// AGC control
// High byte Event 0 timeout
// Low byte Event 0 timeout
// Wake On Radio control
// Front end RX configuration
// Front end TX configuration
// Frequency synthesizer calibration
// Frequency synthesizer calibration
// Frequency synthesizer calibration
// Frequency synthesizer calibration
// RC oscillator configuration
// RC oscillator configuration
// Frequency synthesizer cal control
// Production test
// AGC test
// Various test settings
// Various test settings
// Various test settings
// Status registers
// Part number
// Current version number
// Frequency offset estimate
// Demodulator estimate for link quality
// Received signal strength indication
// Control state machine state
// High byte of WOR timer
// Low byte of WOR timer
// Current GDOx status and packet status
// Current setting from PLL cal module
// Underflow and # of bytes in TXFIFO
// Overflow and # of bytes in RXFIFO
// Multi byte memory locations
// Definitions for burst/single access to registers
// Strobe commands
// Reset chip.
// Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
         // If in RX/TX: Go to a wait state where only the synthesizer is
         // running (for quick RX / TX turnaround).
// Turn off crystal oscillator.
// Calibrate frequency synthesizer and turn it off
                          // (enables quick start).
// Enable RX. Perform calibration first if coming from IDLE and
         // MCSM0.FS_AUTOCAL=1.
// In IDLE state: Enable TX. Perform calibration first if
         // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
         // Only go to TX if channel is clear.
// Exit RX / TX, turn off frequency synthesizer and exit
         // Wake-On-Radio mode if applicable.
// Perform AFC adjustment of the frequency synthesizer
// Start automatic RX polling sequence (Wake-on-Radio)
// Enter power down mode when CSn goes high.
// Flush the RX FIFO buffer.
// Flush the TX FIFO buffer.
// Reset real time clock.
// No operation. May be used to pad strobe commands to two
         // bytes for simpler software.
//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------
// Bit fields in the chip status byte
// Chip states
//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#[no_mangle]
pub unsafe extern "C" fn cc2500Reset() -> uint8_t {
    cc2500Strobe(0x30 as libc::c_int as uint8_t); // 1000us
    delayMicroseconds(1000 as libc::c_int as timeUs_t);
    // CC2500_SetTxRxMode(TXRX_OFF);
    // RX_EN_off;//off tx
    // TX_EN_off;//off rx
    return (cc2500ReadReg(CC2500_0E_FREQ1 as libc::c_int as uint8_t) as
                libc::c_int == 0xc4 as libc::c_int) as libc::c_int as uint8_t;
    // check if reset
}
