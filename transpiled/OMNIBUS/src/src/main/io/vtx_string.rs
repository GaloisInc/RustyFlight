use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
#[no_mangle]
pub static mut vtx58frequencyTable: [[uint16_t; 8]; 5] =
    [[5865 as libc::c_int as uint16_t, 5845 as libc::c_int as uint16_t,
      5825 as libc::c_int as uint16_t, 5805 as libc::c_int as uint16_t,
      5785 as libc::c_int as uint16_t, 5765 as libc::c_int as uint16_t,
      5745 as libc::c_int as uint16_t, 5725 as libc::c_int as uint16_t],
     [5733 as libc::c_int as uint16_t, 5752 as libc::c_int as uint16_t,
      5771 as libc::c_int as uint16_t, 5790 as libc::c_int as uint16_t,
      5809 as libc::c_int as uint16_t, 5828 as libc::c_int as uint16_t,
      5847 as libc::c_int as uint16_t, 5866 as libc::c_int as uint16_t],
     [5705 as libc::c_int as uint16_t, 5685 as libc::c_int as uint16_t,
      5665 as libc::c_int as uint16_t, 5645 as libc::c_int as uint16_t,
      5885 as libc::c_int as uint16_t, 5905 as libc::c_int as uint16_t,
      5925 as libc::c_int as uint16_t, 5945 as libc::c_int as uint16_t],
     [5740 as libc::c_int as uint16_t, 5760 as libc::c_int as uint16_t,
      5780 as libc::c_int as uint16_t, 5800 as libc::c_int as uint16_t,
      5820 as libc::c_int as uint16_t, 5840 as libc::c_int as uint16_t,
      5860 as libc::c_int as uint16_t, 5880 as libc::c_int as uint16_t],
     [5658 as libc::c_int as uint16_t, 5695 as libc::c_int as uint16_t,
      5732 as libc::c_int as uint16_t, 5769 as libc::c_int as uint16_t,
      5806 as libc::c_int as uint16_t, 5843 as libc::c_int as uint16_t,
      5880 as libc::c_int as uint16_t, 5917 as libc::c_int as uint16_t]];
#[no_mangle]
pub static mut vtx58BandNames: [*const libc::c_char; 6] =
    [b"--------\x00" as *const u8 as *const libc::c_char,
     b"BOSCAM A\x00" as *const u8 as *const libc::c_char,
     b"BOSCAM B\x00" as *const u8 as *const libc::c_char,
     b"BOSCAM E\x00" as *const u8 as *const libc::c_char,
     b"FATSHARK\x00" as *const u8 as *const libc::c_char,
     b"RACEBAND\x00" as *const u8 as *const libc::c_char];
#[no_mangle]
pub static mut vtx58BandLetter: [libc::c_char; 7] =
    [45, 65, 66, 69, 70, 82, 0];
#[no_mangle]
pub static mut vtx58ChannelNames: [*const libc::c_char; 9] =
    [b"-\x00" as *const u8 as *const libc::c_char,
     b"1\x00" as *const u8 as *const libc::c_char,
     b"2\x00" as *const u8 as *const libc::c_char,
     b"3\x00" as *const u8 as *const libc::c_char,
     b"4\x00" as *const u8 as *const libc::c_char,
     b"5\x00" as *const u8 as *const libc::c_char,
     b"6\x00" as *const u8 as *const libc::c_char,
     b"7\x00" as *const u8 as *const libc::c_char,
     b"8\x00" as *const u8 as *const libc::c_char];
//Converts frequency (in MHz) to band and channel values.
#[no_mangle]
pub unsafe extern "C" fn vtx58_Freq2Bandchan(mut freq: uint16_t,
                                             mut pBand: *mut uint8_t,
                                             mut pChannel: *mut uint8_t)
 -> bool {
    // Use reverse lookup order so that 5880Mhz
    // get Raceband 7 instead of Fatshark 8.
    let mut band: libc::c_int = 5 as libc::c_int - 1 as libc::c_int;
    while band >= 0 as libc::c_int {
        let mut channel: libc::c_int = 0 as libc::c_int;
        while channel < 8 as libc::c_int {
            if vtx58frequencyTable[band as usize][channel as usize] as
                   libc::c_int == freq as libc::c_int {
                *pBand = (band + 1 as libc::c_int) as uint8_t;
                *pChannel = (channel + 1 as libc::c_int) as uint8_t;
                return 1 as libc::c_int != 0
            }
            channel += 1
        }
        band -= 1
    }
    *pBand = 0 as libc::c_int as uint8_t;
    *pChannel = 0 as libc::c_int as uint8_t;
    return 0 as libc::c_int != 0;
}
//Converts band and channel values to a frequency (in MHz) value.
// band:  Band value (1 to 5).
// channel:  Channel value (1 to 8).
// Returns frequency value (in MHz), or 0 if band/channel out of range.
#[no_mangle]
pub unsafe extern "C" fn vtx58_Bandchan2Freq(mut band: uint8_t,
                                             mut channel: uint8_t)
 -> uint16_t {
    if band as libc::c_int > 0 as libc::c_int &&
           band as libc::c_int <= 5 as libc::c_int &&
           channel as libc::c_int > 0 as libc::c_int &&
           channel as libc::c_int <= 8 as libc::c_int {
        return vtx58frequencyTable[(band as libc::c_int - 1 as libc::c_int) as
                                       usize][(channel as libc::c_int -
                                                   1 as libc::c_int) as usize]
    }
    return 0 as libc::c_int as uint16_t;
}
